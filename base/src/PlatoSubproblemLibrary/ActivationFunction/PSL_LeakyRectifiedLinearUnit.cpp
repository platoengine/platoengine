#include "PSL_LeakyRectifiedLinearUnit.hpp"

#include "PSL_ActivationFunction.hpp"
#include "PSL_ParameterDataEnums.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

LeakyRectifiedLinearUnit::LeakyRectifiedLinearUnit(double leaky_constant) :
        ActivationFunction(activation_function_t::activation_function_t::leaky_rectified_linear_unit),
        m_leaky_constant(leaky_constant)
{
}
LeakyRectifiedLinearUnit::~LeakyRectifiedLinearUnit()
{
}

void LeakyRectifiedLinearUnit::activate(const double& input, double& output, double& output_gradient)
{
    const double slope = (input >= 0. ? 1. : m_leaky_constant);
    output = slope * input;
    output_gradient = slope;
}

}
