#include "PSL_PureVectorKernel.hpp"

#include "PSL_VectorKernel.hpp"

#include <vector>
#include <cassert>
#include <cmath>
#include <math.h>

namespace PlatoSubproblemLibrary
{

PureVectorKernel::PureVectorKernel(int projection_power) :
        VectorKernel(vector_kernel_t::vector_kernel_t::pure_kernel),
        m_projection_power(projection_power)
{
    assert(0 < m_projection_power);
}
PureVectorKernel::~PureVectorKernel()
{
}

int PureVectorKernel::projection_length(const int& source_length)
{
    return m_projection_power * source_length;
}
void PureVectorKernel::project(const std::vector<double>& source, std::vector<double>& destination)
{
    // allocate
    const int source_length = source.size();
    const int destination_length = projection_length(source_length);
    destination.resize(destination_length);

    // fill
    for(int p = 1; p <= m_projection_power; p++)
    {
        for(int i = 0; i < source_length; i++)
        {
            const int source_index = i;
            const int destination_index = (p - 1) * source_length + i;
            destination[destination_index] = std::pow(source[source_index], p);
        }
    }
}

}


