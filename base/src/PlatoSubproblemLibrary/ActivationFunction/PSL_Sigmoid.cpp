#include "PSL_Sigmoid.hpp"

#include "PSL_ActivationFunction.hpp"
#include "PSL_ParameterDataEnums.hpp"

#include <cmath>
#include <math.h>

namespace PlatoSubproblemLibrary
{

Sigmoid::Sigmoid() :
        ActivationFunction(activation_function_t::activation_function_t::sigmoid_activation)
{
}
Sigmoid::~Sigmoid()
{
}

void Sigmoid::activate(const double& input, double& output, double& output_gradient)
{
    output = 1. / (1. + exp(-input));
    output_gradient = output * (1. - output);
}

}
