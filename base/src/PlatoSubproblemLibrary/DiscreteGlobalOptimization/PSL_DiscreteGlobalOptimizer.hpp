#pragma once

#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class DiscreteObjective;

class DiscreteGlobalOptimizer
{
public:
    DiscreteGlobalOptimizer(AbstractAuthority* authority);
    virtual ~DiscreteGlobalOptimizer();

    void set_objective(DiscreteObjective* obj);
    virtual double find_min(std::vector<double>& best_parameters) = 0;

    int get_number_of_evaluations();

protected:
    void fill_parameters(const std::vector<int>& all_values, std::vector<double>& parameters);
    void get_num_values(std::vector<int>& num_values);
    double get_parameter(const int& dimen, const int& value);
    void count_evaluated_objective();

    AbstractAuthority* m_authority;
    DiscreteObjective* m_obj;
private:
    int m_objective_evaluations;
    std::vector<int> m_num_values;
    std::vector<double> m_parameter_slope;
    std::vector<double> m_parameter_intercept;

};

}
