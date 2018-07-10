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

#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{
namespace dense_matrix
{

void matvec(size_t num_rows_,
            size_t num_columns,
            double** matrix_,
            const double* vec_in_,
            double* vec_out_,
            bool transpose = false);

double** create(size_t nrows_, size_t ncols_);
void fill(const double alpha_, double** matrix_, size_t nrows_, size_t ncols_);
void fill_by_row_major(const double* vec_, double** matrix_, size_t nrows_, size_t ncols_);
void identity(double** matrix_, size_t nrows_, size_t ncols_);
void copy(double** input_, double** output_, size_t nrows_, size_t ncols_);
void aXpY(const double alpha_, double** matrix_X, double** matrix_Y, size_t nrows_, size_t ncols_);
void matrix_matrix_product(const double alpha_,
                           double** matrix_X,
                           size_t X_nrows_,
                           size_t X_ncols_,
                           bool X_transpose,
                           double** matrix_Y,
                           size_t Y_nrows_,
                           size_t Y_ncols_,
                           bool Y_transpose,
                           double** result);
void scale(const double alpha_, double** matrix, size_t nrows_, size_t ncols_);
void print(double** matrix_, size_t nrows_, size_t ncols_);
void destroy(double** matrix_, size_t nrows_, size_t ncols_);

}
}
}
