// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_DenseVectorOperations.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{

DenseVectorOperations::DenseVectorOperations()
{
}

DenseVectorOperations::~DenseVectorOperations()
{
}

double DenseVectorOperations::delta_squared(const std::vector<double>& x, const std::vector<double>& y)
{
    std::vector<double> delta = y;
    axpy(-1., x, delta);
    return dot(delta, delta);
}

}
}
