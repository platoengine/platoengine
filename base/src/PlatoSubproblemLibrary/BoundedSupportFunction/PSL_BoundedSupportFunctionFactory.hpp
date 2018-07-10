// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class Abstract_BoundedSupportFunction;

Abstract_BoundedSupportFunction* build_bounded_support_function(bounded_support_function_t::bounded_support_function_t function_type,
                                                                AbstractAuthority* authority);

}
