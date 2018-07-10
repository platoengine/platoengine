// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_BasicDenseVectorOperations.hpp"

#include "PSL_Abstract_DenseVectorOperations.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_BasicDenseVectorOperations::Interface_BasicDenseVectorOperations() :
        AbstractInterface::DenseVectorOperations()
{
}

Interface_BasicDenseVectorOperations::~Interface_BasicDenseVectorOperations()
{
}

double Interface_BasicDenseVectorOperations::dot(const std::vector<double>& x, const std::vector<double>& y)
{
    const size_t num_entries = x.size();

    double result = 0.;
    for(size_t index = 0; index < num_entries; index++)
    {
        result += x[index] * y[index];
    }
    return (result);
}

void Interface_BasicDenseVectorOperations::axpy(double alpha, const std::vector<double>& x, std::vector<double>& y)
{
    const size_t num_entries = x.size();

    for(size_t index = 0; index < num_entries; ++index)
    {
        y[index] = alpha * x[index] + y[index];
    }
}

void Interface_BasicDenseVectorOperations::scale(double alpha, std::vector<double>& x)
{
    const size_t num_entries = x.size();

    for(size_t index = 0; index < num_entries; ++index)
    {
        x[index] = alpha * x[index];
    }
}

void Interface_BasicDenseVectorOperations::multiply(const std::vector<double>& x,
                                                    const std::vector<double>& y,
                                                    std::vector<double>& z)
{
    const size_t num_entries = x.size();
    z.resize(num_entries);
    for(size_t i = 0; i < num_entries; ++i)
    {
        z[i] = x[i] * y[i];
    }
}

void Interface_BasicDenseVectorOperations::multiply(const std::vector<double>& x, std::vector<double>& y)
{
    const size_t num_entries = x.size();
    for(size_t i = 0; i < num_entries; ++i)
    {
        y[i] *= x[i];
    }
}

}
}
