// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <vector>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{

class DenseVectorOperations
{
public:
    DenseVectorOperations();
    virtual ~DenseVectorOperations();

    virtual double dot(const std::vector<double>& x, const std::vector<double>& y) = 0;
    virtual void axpy(double alpha, const std::vector<double>& x, std::vector<double>& y) = 0;
    virtual void scale(double alpha, std::vector<double>& x) = 0;
    virtual void multiply(const std::vector<double>& x, const std::vector<double>& y, std::vector<double>& z) = 0;
    virtual void multiply(const std::vector<double>& x, std::vector<double>& y) = 0;
    virtual double delta_squared(const std::vector<double>& x, const std::vector<double>& y);

protected:

};

}

}
