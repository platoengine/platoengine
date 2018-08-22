#include "PSL_VectorKernel.hpp"

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{

VectorKernel::VectorKernel(vector_kernel_t::vector_kernel_t type) :
        m_type(type)
{
}
VectorKernel::~VectorKernel()
{
}
vector_kernel_t::vector_kernel_t VectorKernel::get_type()
{
    return m_type;
}

}


