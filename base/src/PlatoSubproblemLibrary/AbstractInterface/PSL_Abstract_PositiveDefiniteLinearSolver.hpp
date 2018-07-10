// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include <vector>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{
class DenseMatrix;

class PositiveDefiniteLinearSolver
{
public:

    PositiveDefiniteLinearSolver();
    virtual ~PositiveDefiniteLinearSolver();

    // true if success
    virtual bool solve(DenseMatrix* matrix, const std::vector<double>& rhs, std::vector<double>& sol) = 0;

protected:

};

}

}
