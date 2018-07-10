#include "PSL_IterativeSelection.hpp"

#include "PSL_DiscreteGlobalOptimizer.hpp"
#include "PSL_DiscreteObjective.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_FreeStandardization.hpp"
#include "PSL_Random.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cmath>
#include <math.h>
#include <utility>
#include <cassert>
#include <algorithm>
#include <string>

namespace PlatoSubproblemLibrary
{

IterativeSelection::IterativeSelection(AbstractAuthority* authority) :
        DiscreteGlobalOptimizer(authority),
        m_num_stages(),
        m_initial_stage_num_evaluations_upscale(),
        m_final_stage_num_evaluations_upscale(),
        m_initial_stage_best_fraction_consider(),
        m_maximum_fraction_remove_per_dimension_per_stage(),
        m_verbose(),
        m_num_values(),
        m_considering_parameter_value()
{
    default_parameters();
}

IterativeSelection::~IterativeSelection()
{
}

void IterativeSelection::default_parameters()
{
    m_num_stages = 8;
    m_initial_stage_num_evaluations_upscale = 10;
    m_final_stage_num_evaluations_upscale = 1.5;
    m_initial_stage_best_fraction_consider = .01;
    m_maximum_fraction_remove_per_dimension_per_stage = 2. / double(m_num_stages);
    m_verbose = false;
}

double IterativeSelection::find_min(std::vector<double>& best_parameters)
{
    // get sizes
    m_num_values.clear();
    get_num_values(m_num_values);

    // dimensions
    const int dimension = m_num_values.size();
    const double initial_log_product = log_product(m_num_values);

    // slope, intercept
    const double evaluations_slope = (m_final_stage_num_evaluations_upscale - m_initial_stage_num_evaluations_upscale)
                                     / double(m_num_stages - 1.);
    const double evaluations_intercept = m_initial_stage_num_evaluations_upscale;
    const double best_fraction_slope = (1. - m_initial_stage_best_fraction_consider)
                                     / double(m_num_stages - 1.);
    const double best_fraction_intercept = m_initial_stage_best_fraction_consider;

    // begin by considering all parameter_values
    m_considering_parameter_value.resize(dimension);
    for(int d = 0; d < dimension; d++)
    {
        m_considering_parameter_value[d].assign(m_num_values[d], true);
    }

    // remember still feasible experiences
    std::vector<std::pair<std::vector<int>, double> > feA;
    std::vector<std::pair<std::vector<int>, double> > feB;
    std::vector<std::pair<std::vector<int>, double> >* current_feasible_experiences = &feA;
    std::vector<std::pair<std::vector<int>, double> >* next_feasible_experiences = &feB;

    // for each stage
    int prior_stage_evaluations = 0;
    for(int stage = 0; stage < m_num_stages; stage++)
    {
        // determine number of evaluations
        const int this_stage_total_dimension = count(m_considering_parameter_value);
        const double this_stage_evaluations_upscale = double(stage) * evaluations_slope + evaluations_intercept;
        const int num_evaluations = round(this_stage_evaluations_upscale * double(this_stage_total_dimension));
        std::vector<std::vector<int> > this_stage_parameters(num_evaluations, std::vector<int>(dimension));

        // if verbose, print
        if(m_verbose)
        {
            m_authority->utilities->print(std::string("stage: ") + std::to_string(stage) + std::string(" of ")
                                          + std::to_string(m_num_stages)
                                          + std::string(", eval: ")
                                          + std::to_string(num_evaluations)
                                          + std::string("\n"));
        }

        // compute random, feasible parameters
        build_parameters(this_stage_parameters);

        // assess these parameters
        assess_objective(prior_stage_evaluations, stage, current_feasible_experiences, this_stage_parameters);

        // compute lowest(best) fraction considered
        const double lowest_fraction_considered = double(stage) * best_fraction_slope + best_fraction_intercept;

        // compute sorted means
        std::vector<std::pair<std::pair<double, int>, int> > means_then_dimension_then_value;
        compute_feasible_means(lowest_fraction_considered, current_feasible_experiences, means_then_dimension_then_value);

        // do removals
        remove_from_considerings(stage, initial_log_product, means_then_dimension_then_value);

        // consolidate experiences
        build_next_experiences(current_feasible_experiences, next_feasible_experiences);

        // prepare for next iteration
        std::swap(next_feasible_experiences, current_feasible_experiences);
    }

    // compute and return best
    const double best_objective = compute_best(best_parameters, current_feasible_experiences);
    return best_objective;
}

void IterativeSelection::build_parameters(std::vector<std::vector<int> >& this_stage_parameters)
{
    const int num_evaluations = this_stage_parameters.size();
    const int dimension = m_num_values.size();
    for(int d = 0; d < dimension; d++)
    {
        // count considering
        const int num_considering_this_dimension = count(m_considering_parameter_value[d]);
        assert(num_considering_this_dimension > 0);

        // create mapping
        std::vector<int> consider_to_actual(num_considering_this_dimension);
        int mapping_counter = 0;
        for(int v = 0; v < m_num_values[d]; v++)
        {
            if(m_considering_parameter_value[d][v])
            {
                consider_to_actual[mapping_counter++] = v;
            }
        }

        // get permutations
        std::vector<int> parameter_values;
        random_permutations(num_considering_this_dimension, num_evaluations, parameter_values);

        // fill parameters for this dimension
        for(int e = 0; e < num_evaluations; e++)
        {
            this_stage_parameters[e][d] = consider_to_actual[parameter_values[e]];
        }
    }
}

void IterativeSelection::assess_objective(int& prior_stage_evaluations,
                                          const int& stage,
                                          std::vector<std::pair<std::vector<int>, double> >* experiences,
                                          const std::vector<std::vector<int> >& this_stage_parameters)
{
    const int num_evaluations = this_stage_parameters.size();
    const double expected_final_num_evaluations = prior_stage_evaluations + (m_num_stages - stage) * num_evaluations;

    // assess objective on these parameters
    for(int e = 0; e < num_evaluations; e++)
    {
        // compute parameters
        std::vector<double> parameters;
        fill_parameters(this_stage_parameters[e], parameters);

        // evaluate
        const double obj_value = m_obj->evaluate(parameters);
        count_evaluated_objective();

        // if verbose, print
        if(m_verbose)
        {
            const double expected_percent_complete = double(prior_stage_evaluations + e) * 100. / expected_final_num_evaluations;
            m_authority->utilities->print(std::string("eval: ") + std::to_string(e) + std::string(" of ")
                                          + std::to_string(num_evaluations)
                                          + std::string(" (")
                                          + std::to_string(expected_percent_complete)
                                          + std::string("%), objective: ")
                                          + std::to_string(obj_value)
                                          + std::string(", parameters: "));
            m_authority->utilities->print(this_stage_parameters[e], true);
        }

        // record experience
        experiences->push_back(std::make_pair(this_stage_parameters[e], obj_value));
    }

    prior_stage_evaluations += num_evaluations;
}

void IterativeSelection::compute_feasible_means(const double& lowest_fraction_considered,
                                                std::vector<std::pair<std::vector<int>, double> >* experiences,
                                                std::vector<std::pair<std::pair<double, int>, int> >& means_then_dimension_then_value)
{
    const int dimension = m_num_values.size();

    // allocate
    std::vector<std::vector<std::vector<double> > > dimension_then_value_then_instance(dimension);
    for(int d = 0; d < dimension; d++)
    {
        dimension_then_value_then_instance[d].resize(m_num_values[d], std::vector<double>());
    }

    // record experiences
    const int num_experiences = experiences->size();
    for(int x = 0; x < num_experiences; x++)
    {
        // for each dimension
        for(int d = 0; d < dimension; d++)
        {
            const int v = (*experiences)[x].first[d];
            const double obj_value = (*experiences)[x].second;
            dimension_then_value_then_instance[d][v].push_back(obj_value);
        }
    }

    // allocate
    double lowest_fraction_mean = -1.;
    double lowest_fraction_std = -1.;

    // compute lowest fractions
    for(int d = 0; d < dimension; d++)
    {
        for(int v = 0; v < m_num_values[d]; v++)
        {
            // if considered this dimension and value, then have instances
            if(dimension_then_value_then_instance[d][v].size() > 0u)
            {
                get_mean_and_std_of_lowest_fraction(lowest_fraction_considered,
                                                    dimension_then_value_then_instance[d][v],
                                                    lowest_fraction_mean,
                                                    lowest_fraction_std);

                means_then_dimension_then_value.push_back(std::make_pair(std::make_pair(lowest_fraction_mean, d), v));
            }
        }
    }

    // sort by largest means
    sort_descending(means_then_dimension_then_value);
}

void IterativeSelection::remove_from_considerings(const int& stage,
                                                  const double& initial_log_product,
                                                  const std::vector<std::pair<std::pair<double, int>, int> >& means_then_dimension_then_value)
{
    const int dimension = m_num_values.size();

    // pre-compute for later constraint enforcement
    std::vector<int> this_stage_num_considering(dimension);
    std::vector<double> this_stage_min_remaining(dimension);
    for(int d = 0; d < dimension; d++)
    {
        this_stage_num_considering[d] = count(m_considering_parameter_value[d]);
        this_stage_min_remaining[d] =
                std::max(1., round(double(m_num_values[d])
                             * (1. - double(stage + 1) * m_maximum_fraction_remove_per_dimension_per_stage)));
    }
    const double this_stage_min_log_product = initial_log_product * double(m_num_stages - (stage + 1)) / double(m_num_stages);

    // for each dimension
    const int candidate_removal_dimensions = means_then_dimension_then_value.size();
    for(int crd = 0; crd < candidate_removal_dimensions; crd++)
    {
        // do not go beneath minimum for that dimension
        const int d = means_then_dimension_then_value[crd].first.second;
        if(double(this_stage_num_considering[d] - 1) < this_stage_min_remaining[d])
        {
            continue;
        }

        // compute removal if proceed with removal
        std::vector<int> post_removal_num_considering = this_stage_num_considering;
        post_removal_num_considering[d]--;
        const double post_removal_log_product = log_product(post_removal_num_considering);

        // if removal maintains feasibility
        if(this_stage_min_log_product <= post_removal_log_product)
        {
            // do removal
            this_stage_num_considering[d]--;
            const int v = means_then_dimension_then_value[crd].second;
            m_considering_parameter_value[d][v] = false;

            if(m_verbose)
            {
                m_authority->utilities->print(std::string("no longer considering, d: ") + std::to_string(d) + std::string(", v: ")
                                              + std::to_string(v)
                                              + std::string("\n"));
            }
        }
    }
}

void IterativeSelection::build_next_experiences(std::vector<std::pair<std::vector<int>, double> >* current_feasible_experiences,
                                                          std::vector<std::pair<std::vector<int>, double> >* next_feasible_experiences)
{
    const int dimension = m_num_values.size();

    // prune from feasible experiences
    const int num_experiences = current_feasible_experiences->size();
    next_feasible_experiences->clear();
    next_feasible_experiences->reserve(num_experiences);

    // for each experience
    bool is_feasible = false;
    for(int x = 0; x < num_experiences; x++)
    {
        is_feasible = true;

        // for each dimension
        for(int d = 0; d < dimension; d++)
        {
            const int v = (*current_feasible_experiences)[x].first[d];

            // d,v should be feasible
            if(!m_considering_parameter_value[d][v])
            {
                is_feasible = false;
                break;
            }
        }

        // transfer to next
        if(is_feasible)
        {
            next_feasible_experiences->push_back((*current_feasible_experiences)[x]);
        }
    }
}

double IterativeSelection::compute_best(std::vector<double>& best_double_parameters,
                                        std::vector<std::pair<std::vector<int>, double> >* current_feasible_experiences)
{
    const int dimension = m_num_values.size();
    best_double_parameters.resize(dimension);
    std::vector<int> best_int_parameters(dimension, -1);

    // for each dimension
    for(int d = 0; d < dimension; d++)
    {
        assert(count(m_considering_parameter_value[d]) == 1);

        // determine final parameter
        for(int v = 0; v < m_num_values[d]; v++)
        {
            if(m_considering_parameter_value[d][v])
            {
                best_int_parameters[d] = v;
                break;
            }
        }
    }

    // compute best parameters
    fill_parameters(best_int_parameters, best_double_parameters);

    // look for experiences
    std::vector<double> objective_of_best;
    const int num_experiences = current_feasible_experiences->size();
    for(int x = 0; x < num_experiences; x++)
    {
        // if match with best
        if((*current_feasible_experiences)[x].first == best_int_parameters)
        {
            objective_of_best.push_back((*current_feasible_experiences)[x].second);
        }
    }

    // if have experience with best, return mean
    const int num_best_objectives = objective_of_best.size();
    if(num_best_objectives > 1) {
        double best_mean = -1.;
        double best_std = -1.;

        // get mean
        get_mean_and_std(objective_of_best, best_mean, best_std);
        return best_mean;
    }

    // if no experience with best, run it once
    const double obj_value = m_obj->evaluate(best_double_parameters);
    count_evaluated_objective();
    return obj_value;
}

}

