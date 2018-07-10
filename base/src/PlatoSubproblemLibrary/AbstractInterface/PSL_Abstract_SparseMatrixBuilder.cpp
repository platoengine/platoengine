// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_SparseMatrixBuilder.hpp"

#include "PSL_Abstract_SparseMatrix.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{

SparseMatrixBuilder::SparseMatrixBuilder(MpiWrapper* mpi_wrapper) :
        m_mpi_wrapper(mpi_wrapper),
        m_utilities(mpi_wrapper->get_utilities())
{
}

SparseMatrixBuilder::~SparseMatrixBuilder()
{
    m_mpi_wrapper = NULL;
    m_utilities = NULL;
}

MpiWrapper* SparseMatrixBuilder::get_mpi_wrapper()
{
    return m_mpi_wrapper;
}

GlobalUtilities* SparseMatrixBuilder::get_utilities()
{
    return m_utilities;
}

}

}
