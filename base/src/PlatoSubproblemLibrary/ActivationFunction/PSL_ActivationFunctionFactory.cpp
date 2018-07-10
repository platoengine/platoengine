#include "PSL_ActivationFunctionFactory.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_ActivationFunction.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Sigmoid.hpp"
#include "PSL_RectifiedLinearUnit.hpp"
#include "PSL_LeakyRectifiedLinearUnit.hpp"
#include "PSL_SoftPlus.hpp"

namespace PlatoSubproblemLibrary
{

ActivationFunction* build_activation_function(activation_function_t::activation_function_t type,
                                              double auxiliary_parameter,
                                              AbstractInterface::GlobalUtilities* utilities)
{
    ActivationFunction* result = NULL;

    switch(type)
    {
        case activation_function_t::sigmoid_activation:
        {
            result = new Sigmoid();
            break;
        }
        case activation_function_t::rectified_linear_unit:
        {
            result = new RectifiedLinearUnit();
            break;
        }
        case activation_function_t::leaky_rectified_linear_unit:
        {
            result = new LeakyRectifiedLinearUnit(auxiliary_parameter);
            break;
        }
        case activation_function_t::soft_plus_activation:
        {
            result = new SoftPlus();
            break;
        }
        case activation_function_t::unset_activation_function:
        default:
        {
            utilities->fatal_error("PlatoSubproblemLibrary could not match enum to activation function. Aborting.\n\n");
            break;
        }
    }

    return result;
}

}
