#include "PSL_AbstractAuthority.hpp"

#include "PSL_FreeHelpers.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_SparseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_Abstract_PositiveDefiniteLinearSolver.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_Interface_SparseMatrixBuilder.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_Interface_BasicDenseVectorOperations.hpp"
#include "PSL_Interface_ConjugateGradient.hpp"

#include <mpi.h>

namespace PlatoSubproblemLibrary
{

AbstractAuthority::AbstractAuthority(MPI_Comm* comm) :
        utilities(NULL),
        mpi_wrapper(NULL),
        sparse_builder(NULL),
        dense_builder(NULL),
        dense_vector_operations(NULL),
        dense_solver(NULL),
        m_comm()
{
    if(comm == NULL)
    {
        m_comm = MPI_COMM_WORLD;
        comm = &m_comm;
    }
    allocate(comm);
}
AbstractAuthority::~AbstractAuthority()
{
    safe_free(utilities);
    safe_free(mpi_wrapper);
    safe_free(sparse_builder);
    safe_free(dense_builder);
    safe_free(dense_vector_operations);
    safe_free(dense_solver);
}
void AbstractAuthority::allocate(MPI_Comm* comm)
{
    utilities = new example::Interface_BasicGlobalUtilities;
    mpi_wrapper = new example::Interface_MpiWrapper(utilities, comm);
    sparse_builder = new example::SparseMatrixBuilder(mpi_wrapper);
    dense_builder = new example::Interface_DenseMatrixBuilder(utilities);
    dense_vector_operations = new example::Interface_BasicDenseVectorOperations;
    dense_solver = new example::Interface_CojugateGradient(utilities, dense_vector_operations);
}

}
