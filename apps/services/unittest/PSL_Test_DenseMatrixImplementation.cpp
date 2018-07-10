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
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Implementation_DenseMatrix.hpp"
#include "PSL_Random.hpp"

#include <iostream>

namespace PlatoSubproblemLibrary
{

namespace TestingDenseMatrixImplementation
{

PSL_TEST(DenseMatrixImplementation,matvec_simple)
{
    set_rand_seed();
    size_t num_rows = 3;
    size_t num_columns = 2;
    double** matrix = example::dense_matrix::create(num_rows, num_columns);
    example::dense_matrix::fill(0.0, matrix, num_rows, num_columns);
    for(size_t row = 0; row < num_rows; row++)
    {
        for(size_t column = 0; column < num_columns; column++)
        {
            EXPECT_FLOAT_EQ(matrix[row][column], 0.0);
        }
    }

    matrix[0][0] = 1.0;
    matrix[0][1] = 2.0;
    matrix[1][0] = 0.0;
    matrix[1][1] = 4.0;
    matrix[2][0] = 0.5;
    matrix[2][1] = 0.5;

    std::vector<double> vec_in(num_columns, 1.0);
    std::vector<double> vec_out(num_rows, 0.0);

    example::dense_matrix::matvec(num_rows, num_columns, matrix, vec_in.data(), vec_out.data());

    std::vector<double> gold = {3.0, 4.0, 1.0};

    for(size_t row = 0; row < num_rows; row++)
    {
        EXPECT_FLOAT_EQ(gold[row], vec_out[row]);
    }

    example::dense_matrix::destroy(matrix, num_rows, num_columns);
}

PSL_TEST(DenseMatrixImplementation,matvec_simpleTranspose)
{
    set_rand_seed();
    size_t num_rows = 2;
    size_t num_columns = 3;
    double** matrix = example::dense_matrix::create(num_rows, num_columns);
    std::vector<double> matrix_values = {1., 0., .5, 2., 4., .5};
    example::dense_matrix::fill_by_row_major(matrix_values.data(), matrix, num_rows, num_columns);

    std::vector<double> vec_in(num_rows, 1.);
    std::vector<double> vec_out(num_columns, 0.);

    example::dense_matrix::matvec(num_rows, num_columns, matrix, vec_in.data(), vec_out.data(), true);

    std::vector<double> gold = {3., 4., 1.};

    for(size_t index = 0u; index < num_columns; index++)
    {
        EXPECT_FLOAT_EQ(gold[index], vec_out[index]);
    }

    example::dense_matrix::destroy(matrix, num_rows, num_columns);
}

PSL_TEST(DenseMatrixImplementation,matvec_anotherTranspose)
{
    set_rand_seed();
    size_t num_rows = 2;
    size_t num_columns = 3;
    double** matrix = example::dense_matrix::create(num_rows, num_columns);
    std::vector<double> matrix_values = {5., 4., 3., 2., 1., 0.};
    example::dense_matrix::fill_by_row_major(matrix_values.data(), matrix, num_rows, num_columns);

    std::vector<double> vec_in = {-1., 3.};
    std::vector<double> vec_out(num_columns, 0.);

    example::dense_matrix::matvec(num_rows, num_columns, matrix, vec_in.data(), vec_out.data(), true);

    std::vector<double> gold = {1., -1., -3.};

    for(size_t index = 0u; index < num_columns; index++)
    {
        EXPECT_FLOAT_EQ(gold[index], vec_out[index]);
    }

    example::dense_matrix::destroy(matrix, num_rows, num_columns);
}

PSL_TEST(DenseMatrixImplementation,fill)
{
    set_rand_seed();
    // create matrix
    const size_t nrows = 3;
    const size_t ncols = 2;
    double** matrix = example::dense_matrix::create(nrows, ncols);

    // use fill
    const double fill_value = 0.31415;
    example::dense_matrix::fill(fill_value, matrix, nrows, ncols);

    // test expectations
    for(size_t row_index = 0u; row_index < nrows; ++row_index)
    {
        for(size_t col_index = 0u; col_index < ncols; ++col_index)
        {
            EXPECT_FLOAT_EQ(matrix[row_index][col_index], fill_value);
        }
    }

    // clean up
    example::dense_matrix::destroy(matrix, nrows, ncols);
}

PSL_TEST(DenseMatrixImplementation,fillByRowMajor)
{
    set_rand_seed();
    // create matrix
    const size_t nrows = 4;
    const size_t ncols = 2;
    double** matrix = example::dense_matrix::create(nrows, ncols);

    // use fill_by_row_major
    std::vector<double> matrix_values = {0., 1., 5., 7., -1., -3., 4.5, 2.};
    example::dense_matrix::fill_by_row_major(matrix_values.data(), matrix, nrows, ncols);

    // test expectations
    EXPECT_FLOAT_EQ(matrix[0][0], 0.);
    EXPECT_FLOAT_EQ(matrix[0][1], 1.);
    EXPECT_FLOAT_EQ(matrix[1][0], 5.);
    EXPECT_FLOAT_EQ(matrix[1][1], 7.);
    EXPECT_FLOAT_EQ(matrix[2][0], -1.);
    EXPECT_FLOAT_EQ(matrix[2][1], -3.);
    EXPECT_FLOAT_EQ(matrix[3][0], 4.5);
    EXPECT_FLOAT_EQ(matrix[3][1], 2.);

    // clean up
    example::dense_matrix::destroy(matrix, nrows, ncols);
}

PSL_TEST(DenseMatrixImplementation,identity)
{
    set_rand_seed();
    // create matrix
    const size_t nrows = 3;
    const size_t ncols = 4;
    double** matrix = example::dense_matrix::create(nrows, ncols);

    // use identity
    example::dense_matrix::identity(matrix, nrows, ncols);

    // test expectations
    for(size_t row_index = 0u; row_index < nrows; ++row_index)
    {
        for(size_t col_index = 0u; col_index < ncols; ++col_index)
        {
            if(row_index == col_index)
            {
                EXPECT_FLOAT_EQ(matrix[row_index][col_index], 1.0);
            }
            else
            {
                EXPECT_FLOAT_EQ(matrix[row_index][col_index], 0.0);
            }
        }
    }

    // clean up
    example::dense_matrix::destroy(matrix, nrows, ncols);
}

PSL_TEST(DenseMatrixImplementation,aXpY)
{
    set_rand_seed();
    // create matrices
    const size_t nrows = 2;
    const size_t ncols = 4;
    double** matrix_one = example::dense_matrix::create(nrows, ncols);
    double** matrix_two = example::dense_matrix::create(nrows, ncols);

    // define matrices
    std::vector<double> matrix_one_values = {1., 2., 3., 4., 0., 5., 3., 7.};
    example::dense_matrix::fill_by_row_major(matrix_one_values.data(), matrix_one, nrows, ncols);
    std::vector<double> matrix_two_values = {2., 4., 7., 9., 3., 6., 2., 5.};
    example::dense_matrix::fill_by_row_major(matrix_two_values.data(), matrix_two, nrows, ncols);

    // use aXpY
    const double alpha = 2.0;
    example::dense_matrix::aXpY(alpha, matrix_one, matrix_two, nrows, ncols);

    // test expectations
    EXPECT_FLOAT_EQ(matrix_two[0][0], 4.);
    EXPECT_FLOAT_EQ(matrix_two[0][1], 8.);
    EXPECT_FLOAT_EQ(matrix_two[0][2], 13.);
    EXPECT_FLOAT_EQ(matrix_two[0][3], 17.);
    EXPECT_FLOAT_EQ(matrix_two[1][0], 3.);
    EXPECT_FLOAT_EQ(matrix_two[1][1], 16.);
    EXPECT_FLOAT_EQ(matrix_two[1][2], 8.);
    EXPECT_FLOAT_EQ(matrix_two[1][3], 19.);

    // clean up
    example::dense_matrix::destroy(matrix_one, nrows, ncols);
    example::dense_matrix::destroy(matrix_two, nrows, ncols);
}

PSL_TEST(DenseMatrixImplementation,product)
{
    set_rand_seed();
    // create matrices
    const size_t nrows = 4;
    const size_t inner = 3;
    const size_t ncols = 2;
    double** left_matrix = example::dense_matrix::create(nrows, inner);
    double** right_matrix = example::dense_matrix::create(inner, ncols);
    double** result_matrix = example::dense_matrix::create(nrows, ncols);

    // fill matrices
    std::vector<double> left_matrix_values = {1., 2., 3., 4., 0., 2., 5., -3., -1., -2., 7., 3.};
    example::dense_matrix::fill_by_row_major(left_matrix_values.data(), left_matrix, nrows, inner);
    std::vector<double> right_matrix_values = {1., 5., 2., -1., -7., -3.};
    example::dense_matrix::fill_by_row_major(right_matrix_values.data(), right_matrix, inner, ncols);
    example::dense_matrix::fill(0., result_matrix, nrows, ncols);

    // compute product
    const double alpha = 2.0;
    example::dense_matrix::matrix_matrix_product(alpha,
                                         left_matrix, nrows, inner, false,
                                         right_matrix, inner, ncols, false,
                                         result_matrix);

    // test expectations
    EXPECT_FLOAT_EQ(result_matrix[0][0], -32.);
    EXPECT_FLOAT_EQ(result_matrix[0][1], -12.);
    EXPECT_FLOAT_EQ(result_matrix[1][0], -20.);
    EXPECT_FLOAT_EQ(result_matrix[1][1],  28.);
    EXPECT_FLOAT_EQ(result_matrix[2][0],  12.);
    EXPECT_FLOAT_EQ(result_matrix[2][1],  62.);
    EXPECT_FLOAT_EQ(result_matrix[3][0], -18.);
    EXPECT_FLOAT_EQ(result_matrix[3][1], -52.);

    example::dense_matrix::destroy(left_matrix, nrows, inner);
    example::dense_matrix::destroy(right_matrix, inner, ncols);
    example::dense_matrix::destroy(result_matrix, nrows, ncols);
}

PSL_TEST(DenseMatrixImplementation,productLeftTranspose)
{
    set_rand_seed();
    // create matrices
    const size_t nrows = 4;
    const size_t inner = 3;
    const size_t ncols = 2;
    double** left_matrix = example::dense_matrix::create(inner, nrows);
    double** right_matrix = example::dense_matrix::create(inner, ncols);
    double** result_matrix = example::dense_matrix::create(nrows, ncols);

    // fill matrices
    std::vector<double> left_matrix_values = {1., 4., 5., -2., 2., 0., -3., 7., 3., 2., -1., 3.};
    example::dense_matrix::fill_by_row_major(left_matrix_values.data(), left_matrix, inner, nrows);
    std::vector<double> right_matrix_values = {4., 1., 7., 0., -3., -1.};
    example::dense_matrix::fill_by_row_major(right_matrix_values.data(), right_matrix, inner, ncols);
    example::dense_matrix::fill(0., result_matrix, nrows, ncols);

    // compute product
    const double alpha = 2.0;
    example::dense_matrix::matrix_matrix_product(alpha,
                                         left_matrix, inner, nrows, true,
                                         right_matrix, inner, ncols, false,
                                         result_matrix);

    // test expectations
    EXPECT_FLOAT_EQ(result_matrix[0][0],  18.);
    EXPECT_FLOAT_EQ(result_matrix[0][1],  -4.);
    EXPECT_FLOAT_EQ(result_matrix[1][0],  20.);
    EXPECT_FLOAT_EQ(result_matrix[1][1],   4.);
    EXPECT_FLOAT_EQ(result_matrix[2][0],   4.);
    EXPECT_FLOAT_EQ(result_matrix[2][1],  12.);
    EXPECT_FLOAT_EQ(result_matrix[3][0],  64.);
    EXPECT_FLOAT_EQ(result_matrix[3][1], -10.);

    example::dense_matrix::destroy(left_matrix, inner, nrows);
    example::dense_matrix::destroy(right_matrix, inner, ncols);
    example::dense_matrix::destroy(result_matrix, nrows, ncols);
}

PSL_TEST(DenseMatrixImplementation,productRightTranspose)
{
    set_rand_seed();
    // create matrices
    const size_t nrows = 4;
    const size_t inner = 2;
    const size_t ncols = 3;
    double** left_matrix = example::dense_matrix::create(nrows, inner);
    double** right_matrix = example::dense_matrix::create(ncols, inner);
    double** result_matrix = example::dense_matrix::create(nrows, ncols);

    // fill matrices
    std::vector<double> left_matrix_values = {1., 2., 3., 4., 0., 2., 5., -3.};
    example::dense_matrix::fill_by_row_major(left_matrix_values.data(), left_matrix, nrows, inner);
    std::vector<double> right_matrix_values = {1., 0., 2., -1., -7., -3.};
    example::dense_matrix::fill_by_row_major(right_matrix_values.data(), right_matrix, ncols, inner);
    example::dense_matrix::fill(0., result_matrix, nrows, ncols);

    // compute product
    const double alpha = -1.0;
    example::dense_matrix::matrix_matrix_product(alpha,
                                         left_matrix, nrows, inner, false,
                                         right_matrix, ncols, inner, true,
                                         result_matrix);

    // test expectations
    EXPECT_FLOAT_EQ(result_matrix[0][0], -1. );
    EXPECT_FLOAT_EQ(result_matrix[0][1],  0. );
    EXPECT_FLOAT_EQ(result_matrix[0][2],  13.);
    EXPECT_FLOAT_EQ(result_matrix[1][0], -3. );
    EXPECT_FLOAT_EQ(result_matrix[1][1], -2. );
    EXPECT_FLOAT_EQ(result_matrix[1][2],  33.);
    EXPECT_FLOAT_EQ(result_matrix[2][0],  0. );
    EXPECT_FLOAT_EQ(result_matrix[2][1],  2. );
    EXPECT_FLOAT_EQ(result_matrix[2][2],  6. );
    EXPECT_FLOAT_EQ(result_matrix[3][0], -5. );
    EXPECT_FLOAT_EQ(result_matrix[3][1], -13.);
    EXPECT_FLOAT_EQ(result_matrix[3][2],  26.);

    example::dense_matrix::destroy(left_matrix, nrows, inner);
    example::dense_matrix::destroy(right_matrix, ncols, inner);
    example::dense_matrix::destroy(result_matrix, nrows, ncols);
}

PSL_TEST(DenseMatrixImplementation,productBothTranspose)
{
    set_rand_seed();
    // create matrices
    const size_t nrows = 3;
    const size_t inner = 4;
    const size_t ncols = 2;
    double** left_matrix = example::dense_matrix::create(inner, nrows);
    double** right_matrix = example::dense_matrix::create(ncols, inner);
    double** result_matrix = example::dense_matrix::create(nrows, ncols);

    // fill matrices
    std::vector<double> left_matrix_values = {1., 2., 3., 4., 0., 2., 5., -3., -1., -2., 7., 3.};
    example::dense_matrix::fill_by_row_major(left_matrix_values.data(), left_matrix, inner, nrows);
    std::vector<double> right_matrix_values = {1., 5., 2., -1., 0., -7., -3., 5.};
    example::dense_matrix::fill_by_row_major(right_matrix_values.data(), right_matrix, ncols, inner);
    example::dense_matrix::fill(0., result_matrix, nrows, ncols);

    // compute product
    const double alpha = 3.0;
    example::dense_matrix::matrix_matrix_product(alpha,
                                         left_matrix, inner, nrows, true,
                                         right_matrix, ncols, inner, true,
                                         result_matrix);

    // test expectations
    EXPECT_FLOAT_EQ(result_matrix[0][0],  99. );
    EXPECT_FLOAT_EQ(result_matrix[0][1], -159.);
    EXPECT_FLOAT_EQ(result_matrix[1][0], -33. );
    EXPECT_FLOAT_EQ(result_matrix[1][1],  132.);
    EXPECT_FLOAT_EQ(result_matrix[2][0],  24. );
    EXPECT_FLOAT_EQ(result_matrix[2][1],  12. );

    example::dense_matrix::destroy(left_matrix, inner, nrows);
    example::dense_matrix::destroy(right_matrix, ncols, inner);
    example::dense_matrix::destroy(result_matrix, nrows, ncols);
}

PSL_TEST(DenseMatrixImplementation,scale)
{
    set_rand_seed();
    // create matrix
    const size_t nrows = 4;
    const size_t ncols = 2;
    double** matrix = example::dense_matrix::create(nrows, ncols);

    // use fill_by_row_major
    std::vector<double> matrix_values = {0., 1., 5., 7., -1., -3., 4.5, 2.};
    example::dense_matrix::fill_by_row_major(matrix_values.data(), matrix, nrows, ncols);
    const double alpha = 2.;
    example::dense_matrix::scale(alpha, matrix, nrows, ncols);

    // test expectations
    EXPECT_FLOAT_EQ(matrix[0][0], 0.);
    EXPECT_FLOAT_EQ(matrix[0][1], 2.);
    EXPECT_FLOAT_EQ(matrix[1][0], 10.);
    EXPECT_FLOAT_EQ(matrix[1][1], 14.);
    EXPECT_FLOAT_EQ(matrix[2][0], -2.);
    EXPECT_FLOAT_EQ(matrix[2][1], -6.);
    EXPECT_FLOAT_EQ(matrix[3][0], 9.);
    EXPECT_FLOAT_EQ(matrix[3][1], 4.);

    // clean up
    example::dense_matrix::destroy(matrix, nrows, ncols);
}

}

}
