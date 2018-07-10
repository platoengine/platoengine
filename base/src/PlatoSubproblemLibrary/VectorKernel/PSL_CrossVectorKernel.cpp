#include "PSL_CrossVectorKernel.hpp"

#include "PSL_VectorKernel.hpp"
#include "PSL_FreeHelpers.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

CrossVectorKernel::CrossVectorKernel(int projection_power) :
        VectorKernel(vector_kernel_t::vector_kernel_t::cross_kernel),
        m_projection_power(projection_power)
{
}
CrossVectorKernel::~CrossVectorKernel()
{
}

int CrossVectorKernel::projection_length(const int& source_length)
{
    int result = 0;
    for(int power = 1; power <= m_projection_power; power++)
    {
        result += choose(source_length, power);
    }

    return result;
}
void CrossVectorKernel::project(const std::vector<double>& source, std::vector<double>& destination)
{
    // allocate
    const int source_length = source.size();
    const int destination_length = projection_length(source_length);
    destination.assign(destination_length, 1.);

    // fill
    int counter = 0;
    for(int power = 1; power <= m_projection_power; power++)
    {
        recursive_for_loop(power, 0, source, 1., destination, counter);
    }
}
void CrossVectorKernel::recursive_for_loop(const int& num_for_loops,
                                           const int& begin_index,
                                           const std::vector<double>& source,
                                           const double& input_partial_product,
                                           std::vector<double>& destination,
                                           int& counter)
{
    // if at end of recursion
    if(num_for_loops == 0)
    {
        destination[counter++] = input_partial_product;
        return;
    }

    // otherwise recurse
    const int source_length = source.size();
    for(int index = begin_index; index < source_length; index++)
    {
        const double output_partial_product = input_partial_product * source[index];
        recursive_for_loop(num_for_loops - 1, index + 1, source, output_partial_product, destination, counter);
    }
}

}


