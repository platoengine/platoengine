#include "PSL_DataLossFunctionFactory.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_DataLossFunction.hpp"
#include "PSL_CrossEntropyLoss.hpp"
#include "PSL_SquaredErrorLoss.hpp"

namespace PlatoSubproblemLibrary
{

DataLossFunction* build_data_loss_function(data_loss_function_t::data_loss_function_t type,
                                           AbstractInterface::GlobalUtilities* utilities)
{
    DataLossFunction* result = NULL;

    switch(type)
    {
        case data_loss_function_t::squared_error_loss:
        {
            result = new SquaredErrorLoss();
            break;
        }
        case data_loss_function_t::cross_entropy_loss:
        {
            result = new CrossEntropyLoss();
            break;
        }
        case data_loss_function_t::unset_data_loss_function:
        default:
        {
            utilities->fatal_error("PlatoSubproblemLibrary could not match enum to data loss function. Aborting.\n\n");
            break;
        }
    }

    return result;
}

}
