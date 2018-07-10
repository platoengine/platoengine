#include "PSL_VectorKernelFactory.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_VectorKernel.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_PureVectorKernel.hpp"
#include "PSL_MixedVectorKernel.hpp"
#include "PSL_CrossVectorKernel.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{

VectorKernel* build_vector_kernel(const vector_kernel_t::vector_kernel_t& type,
                                  const int& vector_kernel_parameter,
                                  AbstractInterface::GlobalUtilities* util)
{
    VectorKernel* result = NULL;

    switch(type)
    {
        case vector_kernel_t::vector_kernel_t::pure_kernel:
        {
            result = new PureVectorKernel(vector_kernel_parameter);
            break;
        }
        case vector_kernel_t::vector_kernel_t::mixed_kernel:
        {
            result = new MixedVectorKernel(vector_kernel_parameter);
            break;
        }
        case vector_kernel_t::vector_kernel_t::cross_kernel:
        {
            result = new CrossVectorKernel(vector_kernel_parameter);
            break;
        }
        default:
        {
            util->fatal_error("PlatoSubproblemLibrary could not match enum to VectorKernel. Aborting.\n\n");
            break;
        }
    }

    return result;
}

}


