#pragma once

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class GlobalUtilities;
}
class VectorKernel;

VectorKernel* build_vector_kernel(const vector_kernel_t::vector_kernel_t& type,
                                  const int& vector_kernel_parameter,
                                  AbstractInterface::GlobalUtilities* util);

}
