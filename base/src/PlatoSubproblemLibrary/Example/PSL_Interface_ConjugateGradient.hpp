// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_PositiveDefiniteLinearSolver.hpp"

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class GlobalUtilities;
class DenseMatrix;
class DenseVectorOperations;
}

namespace example
{

class Interface_CojugateGradient : public AbstractInterface::PositiveDefiniteLinearSolver
{
public:
    Interface_CojugateGradient(AbstractInterface::GlobalUtilities* utilities,
                               AbstractInterface::DenseVectorOperations* operations);
    virtual ~Interface_CojugateGradient();

    void setTolerance(double tolerance_);
    void setVerbosity(bool verbose_);

    // true if success
    virtual bool solve(AbstractInterface::DenseMatrix* matrix, const std::vector<double>& rhs, std::vector<double>& sol);

protected:
    AbstractInterface::GlobalUtilities* m_utilities;
    AbstractInterface::DenseVectorOperations* m_operations;
    double m_tolerance;
    bool m_verbosity;

};

}
}
