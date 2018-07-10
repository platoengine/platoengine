// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_DenseMatrixBuilder.hpp"

#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Implementation_DenseMatrix.hpp"
#include "PSL_Interface_DenseMatrix.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_DenseMatrixBuilder::Interface_DenseMatrixBuilder(AbstractInterface::GlobalUtilities* utilities) :
        AbstractInterface::DenseMatrixBuilder(utilities)
{
}

Interface_DenseMatrixBuilder::~Interface_DenseMatrixBuilder()
{
}

AbstractInterface::DenseMatrix* Interface_DenseMatrixBuilder::build_by_row_major(size_t num_rows,
                                                                                 size_t num_columns,
                                                                                 const std::vector<double>& in)
{
    // build
    double** data = dense_matrix::create(num_rows, num_columns);
    dense_matrix::fill_by_row_major(in.data(), data, num_rows, num_columns);

    // transfer
    Interface_DenseMatrix* result = new Interface_DenseMatrix(m_utilities, this);
    result->receive(data, num_rows, num_columns);
    return result;
}

}
}
