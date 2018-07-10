#include "PSL_RandomSearch.hpp"

#include "PSL_DiscreteGlobalOptimizer.hpp"
#include "PSL_DiscreteObjective.hpp"
#include "PSL_Random.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cassert>

namespace PlatoSubproblemLibrary
{

RandomSearch::RandomSearch(AbstractAuthority* authority) :
        DiscreteGlobalOptimizer(authority),
        m_num_trials()
{
    default_parameters();
}
RandomSearch::~RandomSearch()
{

}

void RandomSearch::default_parameters()
{
    m_num_trials = 100;
}
double RandomSearch::find_min(std::vector<double>& best_parameters)
{
    // store best found
    bool found_a_best = false;
    double best_objective = 0.;

    // get sizes
    std::vector<int> num_values;
    get_num_values(num_values);
    const int dimension = num_values.size();

    // for a random usage
    std::vector<int> values(dimension);
    std::vector<double> parameters(dimension);
    for(int trial = 0; trial < m_num_trials; trial++)
    {
        // generate values
        for(int d = 0; d < dimension; d++)
        {
            values[d] = rand_int(0, num_values[d]);
        }

        // compute parameters
        fill_parameters(values, parameters);

        // evaluate
        const double obj_value = m_obj->evaluate(parameters);
        count_evaluated_objective();

        // if new best, update
        if(!found_a_best || (obj_value < best_objective))
        {
            found_a_best = true;
            best_objective = obj_value;
            best_parameters = parameters;
        }
    }

    assert(found_a_best);
    return best_objective;
}

}

