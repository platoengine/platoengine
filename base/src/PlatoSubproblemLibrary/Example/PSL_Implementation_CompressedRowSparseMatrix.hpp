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
#pragma once

#include <vector>
#include <map>
#include <cstddef>
#include "PSL_Abstract_SparseMatrix.hpp"

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class MpiWrapper;
}

namespace example
{

class CompressedRowSparseMatrix : public AbstractInterface::SparseMatrix
{
public:
    CompressedRowSparseMatrix(const size_t num_rows,
                              const size_t num_columns,
                              const std::vector<size_t>& integer_row_bounds,
                              const std::vector<size_t>& columns,
                              const std::vector<double>& data);
    virtual ~CompressedRowSparseMatrix();

    virtual size_t getNumRows();
    virtual size_t getNumColumns();

    virtual void matVec(const std::vector<double>& input, std::vector<double>& output, bool transpose = false);
    virtual void matVecToReduced(const std::vector<double>& input, std::vector<double>& output, bool transpose = false);

    virtual void rowNormalize(const std::vector<double>& rowNormalizationFactors);
    virtual void columnNormalize(const std::vector<double>& columnNormalizationFactors);

    virtual void getNonZeroSortedRows(std::vector<size_t>& nonZeroRows);
    virtual size_t getNumNonZeroSortedRows();
    virtual void getNonZeroSortedColumns(std::vector<size_t>& nonZeroColumns);
    virtual size_t getNumNonZeroSortedColumns();

    virtual void getRow(size_t row, std::vector<double>& data, std::vector<size_t>& columns);
    virtual void setRow(size_t row, const std::vector<double>& data);

    std::vector<size_t> m_matrix_row_bounds;
    std::vector<size_t> m_matrix_columns;
    std::vector<double> m_matrix_data;

private:
    void internal_build_nonzero_sorted_rows_and_columns();

    size_t m_num_rows;
    size_t m_num_columns;

    bool m_built_nonzero_sorted_rows_and_columns;
    std::vector<size_t> m_nonzero_sorted_rows;
    std::vector<size_t> m_nonzero_sorted_columns;
    std::map<size_t, size_t> m_full_column_to_reduced_column;

    CompressedRowSparseMatrix(const CompressedRowSparseMatrix &);
    CompressedRowSparseMatrix operator=(const CompressedRowSparseMatrix &);
};

CompressedRowSparseMatrix* transposeCompressedRowSparseMatrix(CompressedRowSparseMatrix* input);
void sendCompressedRowSparseMatrix(AbstractInterface::MpiWrapper* mpi_wrapper, size_t rank, CompressedRowSparseMatrix* input);
CompressedRowSparseMatrix* receiveCompressedRowSparseMatrix(AbstractInterface::MpiWrapper* mpi_wrapper, size_t rank);

}

}
