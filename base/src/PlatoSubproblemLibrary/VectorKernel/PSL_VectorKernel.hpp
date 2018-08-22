#pragma once

#include "PSL_ParameterDataEnums.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class VectorKernel
{
public:
    VectorKernel(vector_kernel_t::vector_kernel_t type);
    virtual ~VectorKernel();
    vector_kernel_t::vector_kernel_t get_type();

    virtual int projection_length(const int& source_length) = 0;
    virtual void project(const std::vector<double>& source, std::vector<double>& destination) = 0;

protected:
    vector_kernel_t::vector_kernel_t m_type;

};

}
