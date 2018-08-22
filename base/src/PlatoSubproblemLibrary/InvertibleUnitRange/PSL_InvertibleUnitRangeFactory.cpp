#include "PSL_InvertibleUnitRangeFactory.hpp"

#include "PSL_InvertibleUnitRange.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_AlgebraicFunction.hpp"
#include "PSL_LogisticFunction.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{

InvertibleUnitRange* build_invertible_unit_range(AbstractInterface::GlobalUtilities* util,
                                                 invertible_unit_range_t::invertible_unit_range_t type,
                                                 const double& negative_scale,
                                                 const double& positive_scale)
{
    InvertibleUnitRange* result = NULL;

    switch(type)
    {
        case invertible_unit_range_t::invertible_unit_range_t::logistic_function:
        {
            result = new LogisticFunction(negative_scale, positive_scale);
            break;
        }
        case invertible_unit_range_t::invertible_unit_range_t::algebraic_function:
        {
            result = new AlgebraicFunction(negative_scale, positive_scale);
            break;
        }
        default:
        {
            util->fatal_error("PlatoSubproblemLibrary could not match enum to InvertibleUnitRange. Aborting.\n\n");
            break;
        }
    }

    return result;
}

}
