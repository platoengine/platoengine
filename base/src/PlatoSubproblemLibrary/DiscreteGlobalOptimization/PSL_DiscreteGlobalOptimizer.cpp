#include "PSL_DiscreteGlobalOptimizer.hpp"

#include "PSL_DiscreteObjective.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{

DiscreteGlobalOptimizer::DiscreteGlobalOptimizer(AbstractAuthority* authority) :
        m_authority(authority),
        m_obj(NULL),
        m_objective_evaluations(0),
        m_num_values(),
        m_parameter_slope(),
        m_parameter_intercept()
{
}
DiscreteGlobalOptimizer::~DiscreteGlobalOptimizer()
{
}

int DiscreteGlobalOptimizer::get_number_of_evaluations()
{
    return m_objective_evaluations;
}

void DiscreteGlobalOptimizer::set_objective(DiscreteObjective* obj)
{
    m_obj = obj;

    // get domain from objective
    std::vector<double> inclusive_lower;
    std::vector<double> inclusive_upper;
    m_obj->get_domain(inclusive_lower, inclusive_upper, m_num_values);
    const int dimension = m_num_values.size();

    // allocate
    m_parameter_intercept = inclusive_lower;
    m_parameter_slope.resize(dimension);

    // fill slope
    for(int d = 0; d < dimension; d++)
    {
        if(m_num_values[d] == 1)
        {
            m_parameter_slope[d] = 0.;
        }
        else
        {
            m_parameter_slope[d] = (inclusive_upper[d] - inclusive_lower[d]) / double(m_num_values[d] - 1);
        }
    }
}

void DiscreteGlobalOptimizer::fill_parameters(const std::vector<int>& all_values, std::vector<double>& parameters)
{
    // allocate
    const int dimension = all_values.size();
    parameters.resize(dimension);

    // fill
    for(int d = 0; d < dimension; d++)
    {
        parameters[d] = get_parameter(d, all_values[d]);
    }
}

void DiscreteGlobalOptimizer::get_num_values(std::vector<int>& num_values)
{
    num_values = m_num_values;
}

double DiscreteGlobalOptimizer::get_parameter(const int& dimen, const int& value)
{
    return m_parameter_intercept[dimen] + m_parameter_slope[dimen] * double(value);
}

void DiscreteGlobalOptimizer::count_evaluated_objective()
{
    m_objective_evaluations++;
}

}

