#pragma once

#include "PSL_ParameterDataEnums.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class ActivationFunction
{
public:
    ActivationFunction(activation_function_t::activation_function_t type);
    virtual ~ActivationFunction();

    virtual void activate(const double& input, double& output, double& output_gradient) = 0;
    virtual void activate(const std::vector<double>& input, std::vector<double>& output, std::vector<double>& output_gradient);

protected:
    activation_function_t::activation_function_t m_type;

};

}
