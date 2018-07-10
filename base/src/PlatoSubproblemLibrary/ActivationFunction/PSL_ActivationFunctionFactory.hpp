#pragma once

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class GlobalUtilities;
}
class ActivationFunction;

ActivationFunction* build_activation_function(activation_function_t::activation_function_t type,
                                              double auxiliary_parameter,
                                              AbstractInterface::GlobalUtilities* utilities);

}
