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
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class GlobalUtilities;
class DenseMatrixBuilder;

class DenseMatrix
{
public:
    DenseMatrix(GlobalUtilities* utilities, DenseMatrixBuilder* builder);
    virtual ~DenseMatrix();

    virtual DenseMatrixBuilder* get_builder();

    virtual size_t get_num_rows() = 0;
    virtual size_t get_num_columns() = 0;
    virtual double get_value(size_t row, size_t column) = 0;
    virtual void set_value(size_t row, size_t column, double value) = 0;

    virtual void matvec(const std::vector<double>& in, std::vector<double>& out, bool transpose) = 0;
    virtual void fill(double alpha) = 0;
    virtual void fill_by_row_major(const std::vector<double>& in) = 0;
    virtual void set_to_identity() = 0;
    virtual void copy(DenseMatrix* source) = 0;
    virtual void aXpY(double alpha, DenseMatrix* X) = 0;
    virtual void matrix_matrix_product(double alpha, DenseMatrix* X, bool X_transpose, DenseMatrix* Y, bool Y_transpose) = 0;
    virtual void scale(double alpha) = 0;
    virtual double dot(DenseMatrix* other) = 0;
    virtual void get_row(const int& row_index, std::vector<double>& row) = 0;
    virtual void get_column(const int& column_index, std::vector<double>& column) = 0;
    virtual void get_diagonal(std::vector<double>& diagonal) = 0;
    virtual void permute_columns(const std::vector<int>& permutation) = 0;
    virtual void scale_column(const int& column_index, const double& scale) = 0;

    virtual void get_row_major(std::vector<std::vector<double> >& matrix);

protected:
    GlobalUtilities* m_utilities;
    DenseMatrixBuilder* m_builder;

};

}
}
