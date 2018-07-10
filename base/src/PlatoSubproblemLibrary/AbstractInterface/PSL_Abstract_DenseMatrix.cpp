// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_DenseMatrix.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{

DenseMatrix::DenseMatrix(GlobalUtilities* utilities, DenseMatrixBuilder* builder) :
        m_utilities(utilities),
        m_builder(builder)
{

}

DenseMatrix::~DenseMatrix()
{
    m_utilities = NULL;
}

DenseMatrixBuilder* DenseMatrix::get_builder()
{
    return m_builder;
}

void DenseMatrix::get_row_major(std::vector<std::vector<double> >& matrix)
{
    // allocate
    const size_t num_rows = get_num_rows();
    const size_t num_cols = get_num_columns();
    matrix.resize(num_rows);

    // fill
    for(size_t r = 0u; r < num_rows; r++)
    {
        matrix[r].resize(num_cols);
        for(size_t c = 0u; c < num_cols; c++)
        {
            matrix[r][c] = get_value(r, c);
        }
    }
}

}

}
