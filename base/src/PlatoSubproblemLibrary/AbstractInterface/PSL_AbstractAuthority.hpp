#pragma once

#include "PSL_FreeHelpers.hpp"
#include <mpi.h>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class GlobalUtilities;
class MpiWrapper;
class SparseMatrixBuilder;
class DenseMatrixBuilder;
class DenseVectorOperations;
class PositiveDefiniteLinearSolver;
}

class AbstractAuthority
{
public:
    AbstractAuthority(MPI_Comm* comm = NULL);
    virtual ~AbstractAuthority();

    AbstractInterface::GlobalUtilities* utilities;
    AbstractInterface::MpiWrapper* mpi_wrapper;
    AbstractInterface::SparseMatrixBuilder* sparse_builder;
    AbstractInterface::DenseMatrixBuilder* dense_builder;
    AbstractInterface::DenseVectorOperations* dense_vector_operations;
    AbstractInterface::PositiveDefiniteLinearSolver* dense_solver;

protected:
    void allocate(MPI_Comm* comm);
    MPI_Comm m_comm;

};

}
