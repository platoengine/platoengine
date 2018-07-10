/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Abstract_DenseMatrixBuilder.hpp"

#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_FreeHelpers.hpp"

#include <cassert>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{

DenseMatrixBuilder::DenseMatrixBuilder(GlobalUtilities* utilities) :
        m_utilities(utilities)
{

}

DenseMatrixBuilder::~DenseMatrixBuilder()
{

}

GlobalUtilities* DenseMatrixBuilder::get_utilities()
{
    return m_utilities;
}

DenseMatrix* DenseMatrixBuilder::build_by_fill(size_t num_rows, size_t num_columns, double fill_value)
{
    std::vector<double> vec(num_rows * num_columns, fill_value);
    return build_by_row_major(num_rows, num_columns, vec);
}

DenseMatrix* DenseMatrixBuilder::build_diagonal(const std::vector<double>& diagonal)
{
    // sizes
    const size_t diagonal_size = diagonal.size();
    const size_t matrix_num_rows = diagonal_size;
    const size_t matrix_num_columns = diagonal_size;

    // set matrix data
    std::vector<double> diagonal_matrix_data(matrix_num_rows * matrix_num_columns, 0.);
    for(size_t index = 0u; index < diagonal_size; index++)
    {
        diagonal_matrix_data[(diagonal_size+1u)*index] = diagonal[index];
    }

    return build_by_row_major(matrix_num_rows, matrix_num_columns, diagonal_matrix_data);
}

DenseMatrix* DenseMatrixBuilder::build_submatrix(size_t begin_row_inclusive,
                                                 size_t end_row_exclusive,
                                                 size_t begin_column_inclusive,
                                                 size_t end_column_exclusive,
                                                 DenseMatrix* matrix)
{
    // get sizes
    const size_t submatrix_num_rows = end_row_exclusive - begin_row_inclusive;
    const size_t submatrix_num_columns = end_column_exclusive - begin_column_inclusive;
    assert(end_row_exclusive <= matrix->get_num_rows());
    assert(end_column_exclusive <= matrix->get_num_columns());

    // set submatrix data
    size_t counter = 0u;
    std::vector<double> row_major_submatrix(submatrix_num_rows * submatrix_num_columns);
    for(size_t submatrix_row = 0u; submatrix_row < submatrix_num_rows; submatrix_row++)
    {
        const size_t matrix_row = begin_row_inclusive + submatrix_row;
        for(size_t submatrix_column = 0u; submatrix_column < submatrix_num_columns; submatrix_column++)
        {
            const size_t matrix_column = begin_column_inclusive + submatrix_column;
            row_major_submatrix[counter++] = matrix->get_value(matrix_row, matrix_column);
        }
    }

    return build_by_row_major(submatrix_num_rows, submatrix_num_columns, row_major_submatrix);
}

DenseMatrix* DenseMatrixBuilder::build_by_outerProduct(const std::vector<double>& x, const std::vector<double>& y)
{
    // allocate
    const size_t num_rows = x.size();
    const size_t num_columns = y.size();
    std::vector<double> row_major_result(num_rows * num_columns);

    // fill
    size_t counter = 0u;
    for(size_t row = 0u; row < num_rows; row++)
    {
        for(size_t column = 0u; column < num_columns; column++)
        {
            row_major_result[counter++] = x[row] * y[column];
        }
    }

    return build_by_row_major(num_rows, num_columns, row_major_result);
}

}

}
