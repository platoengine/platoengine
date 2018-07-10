#include "PSL_SoftPlus.hpp"

#include "PSL_ActivationFunction.hpp"
#include "PSL_ParameterDataEnums.hpp"

#include <vector>
#include <cstddef>
#include <cmath>
#include <math.h>

namespace PlatoSubproblemLibrary
{

SoftPlus::SoftPlus() :
        ActivationFunction(activation_function_t::activation_function_t::soft_plus_activation)
{
}
SoftPlus::~SoftPlus()
{
}

void SoftPlus::activate(const double& input, double& output, double& output_gradient)
{
    output = log(1. + exp(input));
    output_gradient = 1. / (1. + exp(-input));
}

}
