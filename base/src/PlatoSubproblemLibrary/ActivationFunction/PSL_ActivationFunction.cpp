#include "PSL_ActivationFunction.hpp"

#include "PSL_ParameterDataEnums.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

ActivationFunction::ActivationFunction(activation_function_t::activation_function_t type) :
        m_type(type)
{
}

ActivationFunction::~ActivationFunction()
{
}

void ActivationFunction::activate(const std::vector<double>& input, std::vector<double>& output, std::vector<double>& output_gradient)
{
    // allocate
    const size_t length = input.size();
    output.resize(length);
    output_gradient.resize(length);

    // calculate
    for(size_t i = 0u; i < length; i++)
    {
        activate(input[i], output[i], output_gradient[i]);
    }
}

}
