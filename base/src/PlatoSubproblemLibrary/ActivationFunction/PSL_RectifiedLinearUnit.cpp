#include "PSL_RectifiedLinearUnit.hpp"

#include "PSL_ActivationFunction.hpp"
#include "PSL_ParameterDataEnums.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

RectifiedLinearUnit::RectifiedLinearUnit() :
        ActivationFunction(activation_function_t::activation_function_t::rectified_linear_unit)
{
}
RectifiedLinearUnit::~RectifiedLinearUnit()
{
}

void RectifiedLinearUnit::activate(const double& input, double& output, double& output_gradient)
{
    const double slope = (input >= 0. ? 1. : 0.);
    output = slope * input;
    output_gradient = slope;
}

}
