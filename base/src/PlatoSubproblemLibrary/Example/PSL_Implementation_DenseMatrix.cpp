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
#include "PSL_Implementation_DenseMatrix.hpp"

#include <cstddef>
#include <cassert>
#include <algorithm>
#include <iostream>

namespace PlatoSubproblemLibrary
{
namespace example
{
namespace dense_matrix
{

void matvec(size_t num_rows_, size_t num_columns, double** matrix_, const double* vec_in_, double* vec_out_, bool transpose)
{
    if(!transpose)
    {
        std::fill(&vec_out_[0], &vec_out_[num_rows_], .0);
        for(size_t row = 0; row < num_rows_; ++row)
        {
            for(size_t column = 0; column < num_columns; ++column)
            {
                vec_out_[row] += matrix_[row][column] * vec_in_[column];
            }
        }
    }
    else
    {
        assert(transpose);

        std::fill(&vec_out_[0], &vec_out_[num_columns], .0);
        for(size_t row = 0; row < num_rows_; ++row)
        {
            for(size_t column = 0; column < num_columns; ++column)
            {
                vec_out_[column] += matrix_[row][column] * vec_in_[row];
            }
        }
    }
}

double** create(size_t nrows_, size_t ncols_)
{
    double** result = new double*[nrows_];
    for(size_t row = 0; row < nrows_; row++)
    {
        result[row] = new double[ncols_];
    }
    return result;
}

void fill(const double alpha_, double** matrix_, size_t nrows_, size_t ncols_)
{
    for(size_t row = 0; row < nrows_; row++)
    {
        std::fill(&matrix_[row][0], &matrix_[row][ncols_], alpha_);
    }
}

void fill_by_row_major(const double* vec_, double** matrix_, size_t nrows_, size_t ncols_)
{
    for(size_t row = 0; row < nrows_; row++)
    {
        for(size_t column = 0; column < ncols_; column++)
        {
            matrix_[row][column] = vec_[row * ncols_ + column];
        }
    }
}

void identity(double** matrix_, size_t nrows_, size_t ncols_)
{
    for(size_t row = 0; row < nrows_; row++)
    {
        std::fill(&matrix_[row][0], &matrix_[row][ncols_], 0.0);
        if (row < ncols_)
        {
            matrix_[row][row] = 1.0;
        }
    }
}

void copy(double** input_, double** output_, size_t nrows_, size_t ncols_)
{
    for(size_t row = 0; row < nrows_; row++)
    {
        for(size_t column = 0; column < ncols_; column++)
        {
            output_[row][column] = input_[row][column];
        }
    }
}

void aXpY(const double alpha_, double** matrix_X, double** matrix_Y, size_t nrows_, size_t ncols_)
{
    for(size_t row = 0; row < nrows_; row++)
    {
        for(size_t column = 0; column < ncols_; column++)
        {
            matrix_Y[row][column] += alpha_ * matrix_X[row][column];
        }
    }
}

void matrix_matrix_product(const double alpha_,
                           double** matrix_X, size_t X_nrows_, size_t X_ncols_, bool X_transpose,
                           double** matrix_Y, size_t Y_nrows_, size_t Y_ncols_, bool Y_transpose,
                           double** result)
{
    if(!X_transpose && !Y_transpose)
    {
        // result = a * X * Y
        assert(X_ncols_ == Y_nrows_);
        const size_t inner_dimension = X_ncols_;

        for(size_t row = 0; row < X_nrows_; row++)
        {
            for(size_t column = 0; column < Y_ncols_; column++)
            {
                result[row][column] = 0.0;
                for(size_t inner = 0; inner < inner_dimension; inner++)
                {
                    result[row][column] += alpha_ * matrix_X[row][inner] * matrix_Y[inner][column];
                }
            }
        }
    }
    else if(X_transpose && !Y_transpose)
    {
        // result = a * X * Y'
        assert(X_nrows_ == Y_nrows_);
        const size_t inner_dimension = X_nrows_;

        for(size_t row = 0; row < X_ncols_; row++)
        {
            for(size_t column = 0; column < Y_ncols_; column++)
            {
                result[row][column] = 0.0;
                for(size_t inner = 0; inner < inner_dimension; inner++)
                {
                    result[row][column] += alpha_ * matrix_X[inner][row] * matrix_Y[inner][column];
                }
            }
        }
    }
    else if(!X_transpose && Y_transpose)
    {
        // result = a * X' * Y
        assert(X_ncols_ == Y_ncols_);
        const size_t inner_dimension = X_ncols_;

        for(size_t row = 0; row < X_nrows_; row++)
        {
            for(size_t column = 0; column < Y_nrows_; column++)
            {
                result[row][column] = 0.0;
                for(size_t inner = 0; inner < inner_dimension; inner++)
                {
                    result[row][column] += alpha_ * matrix_X[row][inner] * matrix_Y[column][inner];
                }
            }
        }
    }
    else
    {
        // result = a * X' * Y'
        assert(X_transpose && Y_transpose);
        assert(X_nrows_ == Y_ncols_);
        const size_t inner_dimension = X_nrows_;

        for(size_t row = 0; row < X_ncols_; row++)
        {
            for(size_t column = 0; column < Y_nrows_; column++)
            {
                result[row][column] = 0.0;
                for(size_t inner = 0; inner < inner_dimension; inner++)
                {
                    result[row][column] += alpha_ * matrix_X[inner][row] * matrix_Y[column][inner];
                }
            }
        }
    }
}

void scale(const double alpha_, double** matrix, size_t nrows_, size_t ncols_)
{
    for(size_t row = 0; row < nrows_; row++)
    {
        for(size_t column = 0; column < ncols_; column++)
        {
            matrix[row][column] *= alpha_;
        }
    }
}

void print(double** matrix_, size_t nrows_, size_t ncols_)
{
    // useful for debugging

    std::cout << "{r:" << nrows_ << ",c:" << ncols_ << "}=" << std::endl;
    for(size_t row = 0u; row < nrows_; row++)
    {
        std::cout << "[" << matrix_[row][0];
        for(size_t col = 1u; col < ncols_; col++)
        {
            std::cout << "," << matrix_[row][col];
        }
        std::cout << "]" << std::endl;
    }
}

void destroy(double** matrix_, size_t nrows_, size_t ncols_)
{
    if(matrix_)
    {
        for(size_t row = 0; row < nrows_; row++)
        {
            delete[] matrix_[row];
            matrix_[row] = NULL;
        }
        delete[] matrix_;
        matrix_ = NULL;
    }
}

}
}
}
