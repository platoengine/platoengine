#pragma once

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class GlobalUtilities;
}
class DataLossFunction;

DataLossFunction* build_data_loss_function(data_loss_function_t::data_loss_function_t type,
                                           AbstractInterface::GlobalUtilities* utilities);

}
