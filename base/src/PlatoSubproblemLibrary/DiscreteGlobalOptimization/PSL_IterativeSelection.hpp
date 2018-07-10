#pragma once

#include "PSL_DiscreteGlobalOptimizer.hpp"
#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class DiscreteObjective;

#define PSL_IS_parameter(_type, _name) \
    protected: _type m_##_name; \
    public: void set_##_name(_type p){m_##_name = p;}

class IterativeSelection : public DiscreteGlobalOptimizer
{
public:
    IterativeSelection(AbstractAuthority* authority);
    virtual ~IterativeSelection();

    void default_parameters();
    PSL_IS_parameter(int,num_stages)
    PSL_IS_parameter(double,initial_stage_num_evaluations_upscale)
    PSL_IS_parameter(double,final_stage_num_evaluations_upscale)
    PSL_IS_parameter(double,initial_stage_best_fraction_consider)
    PSL_IS_parameter(double,maximum_fraction_remove_per_dimension_per_stage)
    PSL_IS_parameter(bool,verbose)

    virtual double find_min(std::vector<double>& best_parameters);

protected:

    void build_parameters(std::vector<std::vector<int> >& this_stage_parameters);
    void assess_objective(int& prior_stage_evaluations,
                          const int& stage,
                          std::vector<std::pair<std::vector<int>, double> >* experiences,
                          const std::vector<std::vector<int> >& this_stage_parameters);
    void compute_feasible_means(const double& lowest_fraction_considered,
                                std::vector<std::pair<std::vector<int>, double> >* experiences,
                                std::vector<std::pair<std::pair<double, int>, int> >& means_then_dimension_then_value);
    void remove_from_considerings(const int& stage,
                                  const double& initial_log_product,
                                  const std::vector<std::pair<std::pair<double, int>, int> >& means_then_dimension_then_value);
    void build_next_experiences(std::vector<std::pair<std::vector<int>, double> >* current_feasible_experiences,
                                std::vector<std::pair<std::vector<int>, double> >* next_feasible_experiences);
    double compute_best(std::vector<double>& best_parameters,
                        std::vector<std::pair<std::vector<int>, double> >* current_feasible_experiences);

    std::vector<int> m_num_values;
    std::vector<std::vector<bool> > m_considering_parameter_value;

};

}
