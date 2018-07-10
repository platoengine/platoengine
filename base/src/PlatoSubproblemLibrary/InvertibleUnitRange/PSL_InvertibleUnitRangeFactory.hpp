#pragma once

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class GlobalUtilities;
}
class InvertibleUnitRange;

InvertibleUnitRange* build_invertible_unit_range(AbstractInterface::GlobalUtilities* util,
                                                 invertible_unit_range_t::invertible_unit_range_t type,
                                                 const double& negative_scale,
                                                 const double& positive_scale);

}
