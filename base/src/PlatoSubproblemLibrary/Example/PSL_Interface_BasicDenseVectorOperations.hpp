// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <vector>
#include "PSL_Abstract_DenseVectorOperations.hpp"

namespace PlatoSubproblemLibrary
{
namespace example
{

class Interface_BasicDenseVectorOperations : public AbstractInterface::DenseVectorOperations
{
public:
    Interface_BasicDenseVectorOperations();
    virtual ~Interface_BasicDenseVectorOperations();

    virtual double dot(const std::vector<double>& x, const std::vector<double>& y);
    virtual void axpy(double alpha, const std::vector<double>& x, std::vector<double>& y);
    virtual void scale(double alpha, std::vector<double>& x);
    virtual void multiply(const std::vector<double>& x, const std::vector<double>& y, std::vector<double>& z);
    virtual void multiply(const std::vector<double>& x, std::vector<double>& y);

protected:

};

}
}
