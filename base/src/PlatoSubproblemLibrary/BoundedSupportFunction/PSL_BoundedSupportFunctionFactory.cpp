// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#include "PSL_BoundedSupportFunctionFactory.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Abstract_BoundedSupportFunction.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_PolynomialTentFunction.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_OverhangInclusionFunction.hpp"

namespace PlatoSubproblemLibrary
{

Abstract_BoundedSupportFunction* build_bounded_support_function(bounded_support_function_t::bounded_support_function_t function_type,
                                                                AbstractAuthority* authority)
{
    Abstract_BoundedSupportFunction* result = NULL;

    switch(function_type)
    {
        case bounded_support_function_t::polynomial_tent_function:
        {
            result = new PolynomialTentFunction;
            break;
        }
        case bounded_support_function_t::overhang_inclusion_function:
        {
            result = new OverhangInclusionFunction;
            break;
        }
        case bounded_support_function_t::unset_bounded_support_function:
        default:
        {
            authority->utilities->fatal_error("PlatoSubproblemLibrary could match enum to build bounded support function. Aborting.\n\n");
            break;
        }
    }

    return result;
}

}

