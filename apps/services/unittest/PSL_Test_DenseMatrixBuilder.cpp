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

#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Random.hpp"

namespace PlatoSubproblemLibrary
{

#define DenseMatrixBuilderTest_AllocateUtilities \
    example::Interface_BasicGlobalUtilities utilities; \
    example::Interface_DenseMatrixBuilder e_builder(&utilities); \
    AbstractInterface::DenseMatrixBuilder* builder = &e_builder;

namespace TestingDenseMatrixBuilder
{

PSL_TEST(DenseMatrixBuilder,matvec_simple)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    size_t num_rows = 3;
    size_t num_columns = 2;
    AbstractInterface::DenseMatrix* matrix = builder->build_by_fill(num_rows, num_columns, 0.0);
    for(size_t row = 0; row < num_rows; row++)
    {
        for(size_t column = 0; column < num_columns; column++)
        {
            EXPECT_FLOAT_EQ(matrix->get_value(row,column), 0.0);
        }
    }

    matrix->set_value(0, 0, 1.0);
    matrix->set_value(0, 1, 2.0);
    matrix->set_value(1, 0, 0.0);
    matrix->set_value(1, 1, 4.0);
    matrix->set_value(2, 0, 0.5);
    matrix->set_value(2, 1, 0.5);

    std::vector<double> vec_in(num_columns, 1.0);
    std::vector<double> vec_out(num_rows, 0.0);

    matrix->matvec(vec_in,vec_out,false);

    std::vector<double> gold = {3.0, 4.0, 1.0};

    for(size_t row = 0; row < num_rows; row++)
    {
        EXPECT_FLOAT_EQ(gold[row], vec_out[row]);
    }

    safe_free(matrix);
}

PSL_TEST(DenseMatrixBuilder,matvec_simpleTranspose)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    size_t num_rows = 2;
    size_t num_columns = 3;
    std::vector<double> matrix_values = {1., 0., .5, 2., 4., .5};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(num_rows, num_columns, matrix_values);

    std::vector<double> vec_in(num_rows, 1.);
    std::vector<double> vec_out(num_columns, 0.);

    matrix->matvec(vec_in, vec_out, true);

    std::vector<double> gold = {3., 4., 1.};

    for(size_t index = 0u; index < num_columns; index++)
    {
        EXPECT_FLOAT_EQ(gold[index], vec_out[index]);
    }

    safe_free(matrix);
}

PSL_TEST(DenseMatrixBuilder,matvec_anotherTranspose)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    size_t num_rows = 2;
    size_t num_columns = 3;
    std::vector<double> matrix_values = {5., 4., 3., 2., 1., 0.};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(num_rows, num_columns, matrix_values);

    std::vector<double> vec_in = {-1., 3.};
    std::vector<double> vec_out(num_columns, 0.);

    matrix->matvec(vec_in, vec_out, true);

    std::vector<double> gold = {1., -1., -3.};

    for(size_t index = 0u; index < num_columns; index++)
    {
        EXPECT_FLOAT_EQ(gold[index], vec_out[index]);
    }

    safe_free(matrix);
}

PSL_TEST(DenseMatrixBuilder,fill)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrix
    const size_t nrows = 3;
    const size_t ncols = 2;
    const double fill_value = 0.31415;
    AbstractInterface::DenseMatrix* matrix = builder->build_by_fill(nrows, ncols, fill_value);

    // test expectations
    for(size_t row_index = 0u; row_index < nrows; ++row_index)
    {
        for(size_t col_index = 0u; col_index < ncols; ++col_index)
        {
            EXPECT_FLOAT_EQ(matrix->get_value(row_index,col_index), fill_value);
        }
    }

    // clean up
    safe_free(matrix);
}

PSL_TEST(DenseMatrixBuilder,fillByRowMajor)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrix
    const size_t nrows = 4;
    const size_t ncols = 2;
    std::vector<double> matrix_values = {0., 1., 5., 7., -1., -3., 4.5, 2.};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(nrows, ncols, matrix_values);

    // test expectations
    EXPECT_FLOAT_EQ(matrix->get_value(0,0),0.);
    EXPECT_FLOAT_EQ(matrix->get_value(0,1),1.);
    EXPECT_FLOAT_EQ(matrix->get_value(1,0),5.);
    EXPECT_FLOAT_EQ(matrix->get_value(1,1),7.);
    EXPECT_FLOAT_EQ(matrix->get_value(2,0),-1.);
    EXPECT_FLOAT_EQ(matrix->get_value(2,1),-3.);
    EXPECT_FLOAT_EQ(matrix->get_value(3,0),4.5);
    EXPECT_FLOAT_EQ(matrix->get_value(3,1),2.);

    // row major vectors
    std::vector<std::vector<double> > data;
    matrix->get_row_major(data);
    EXPECT_EQ(data.size(), nrows);
    for(size_t r = 0u; r < nrows; r++)
    {
        std::vector<double> row(&matrix_values[r * ncols], &matrix_values[(r + 1u) * ncols]);
        expect_equal_float_vectors(row, data[r]);
    }

    // clean up
    safe_free(matrix);
}

PSL_TEST(DenseMatrixBuilder,submatrix)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrix
    const size_t nrows = 4;
    const size_t ncols = 2;
    std::vector<double> matrix_values = {0., 1., 5., 7., -1., -3., 4.5, 2.};
    /* [ 0   1
     *   5   7
     *   -1 -3
     *   4.5 2 ]
     */
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(nrows, ncols, matrix_values);

    // test expectations
    EXPECT_EQ(matrix->get_num_rows(), 4u);
    EXPECT_EQ(matrix->get_num_columns(), 2u);
    EXPECT_FLOAT_EQ(matrix->get_value(0,0),0.);
    EXPECT_FLOAT_EQ(matrix->get_value(0,1),1.);
    EXPECT_FLOAT_EQ(matrix->get_value(1,0),5.);
    EXPECT_FLOAT_EQ(matrix->get_value(1,1),7.);
    EXPECT_FLOAT_EQ(matrix->get_value(2,0),-1.);
    EXPECT_FLOAT_EQ(matrix->get_value(2,1),-3.);
    EXPECT_FLOAT_EQ(matrix->get_value(3,0),4.5);
    EXPECT_FLOAT_EQ(matrix->get_value(3,1),2.);

    AbstractInterface::DenseMatrix* submatrix1 = builder->build_submatrix(1u, 4u, 0u, 1u, matrix);
    EXPECT_EQ(submatrix1->get_num_rows(), 3u);
    EXPECT_EQ(submatrix1->get_num_columns(), 1u);
    EXPECT_FLOAT_EQ(submatrix1->get_value(0,0), 5.);
    EXPECT_FLOAT_EQ(submatrix1->get_value(1,0), -1.);
    EXPECT_FLOAT_EQ(submatrix1->get_value(2,0),4.5);

    AbstractInterface::DenseMatrix* submatrix2 = builder->build_submatrix(0u, 2u, 0u, 2u, matrix);
    EXPECT_EQ(submatrix2->get_num_rows(), 2u);
    EXPECT_EQ(submatrix2->get_num_columns(), 2u);
    EXPECT_FLOAT_EQ(submatrix2->get_value(0,0), 0.);
    EXPECT_FLOAT_EQ(submatrix2->get_value(0,1), 1.);
    EXPECT_FLOAT_EQ(submatrix2->get_value(1,0), 5.);
    EXPECT_FLOAT_EQ(submatrix2->get_value(1,1), 7.);

    // clean up
    safe_free(matrix);
    safe_free(submatrix1);
    safe_free(submatrix2);
}

PSL_TEST(DenseMatrixBuilder,outerProduct)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrix
    std::vector<double> x = {1.2, -3.1, 4.5};
    std::vector<double> y = {-7.1, 0., 1.8, .5};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_outerProduct(x, y);

    // test expectations
    EXPECT_EQ(matrix->get_num_rows(), x.size());
    EXPECT_EQ(matrix->get_num_columns(), y.size());
    EXPECT_FLOAT_EQ(matrix->get_value(0,0), -8.52);
    EXPECT_FLOAT_EQ(matrix->get_value(0,1), 0.);
    EXPECT_FLOAT_EQ(matrix->get_value(0,2), 2.16);
    EXPECT_FLOAT_EQ(matrix->get_value(0,3), .6);
    EXPECT_FLOAT_EQ(matrix->get_value(1,0), 22.01);
    EXPECT_FLOAT_EQ(matrix->get_value(1,1), 0.);
    EXPECT_FLOAT_EQ(matrix->get_value(1,2), -5.58);
    EXPECT_FLOAT_EQ(matrix->get_value(1,3), -1.55);
    EXPECT_FLOAT_EQ(matrix->get_value(2,0), -31.95);
    EXPECT_FLOAT_EQ(matrix->get_value(2,1), 0.);
    EXPECT_FLOAT_EQ(matrix->get_value(2,2), 8.1);
    EXPECT_FLOAT_EQ(matrix->get_value(2,3), 2.25);

    // clean up
    safe_free(matrix);
}

PSL_TEST(DenseMatrixBuilder,getBuilder)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrix
    const size_t nrows = 4;
    const size_t ncols = 2;
    const double fill_value = 0.15;
    AbstractInterface::DenseMatrix* matrix = builder->build_by_fill(nrows, ncols, fill_value);

    // get builder
    AbstractInterface::DenseMatrixBuilder* another_builder = matrix->get_builder();

    // clean up
    safe_free(matrix);

    // create another matrix
    std::vector<double> matrix_values = {0., 1., 5., 7., -1., -3., 4.5, 2.};
    AbstractInterface::DenseMatrix* another_matrix = another_builder->build_by_row_major(nrows, ncols, matrix_values);

    // test expectations
    EXPECT_FLOAT_EQ(another_matrix->get_value(0,0),0.);
    EXPECT_FLOAT_EQ(another_matrix->get_value(0,1),1.);
    EXPECT_FLOAT_EQ(another_matrix->get_value(1,0),5.);
    EXPECT_FLOAT_EQ(another_matrix->get_value(1,1),7.);
    EXPECT_FLOAT_EQ(another_matrix->get_value(2,0),-1.);
    EXPECT_FLOAT_EQ(another_matrix->get_value(2,1),-3.);
    EXPECT_FLOAT_EQ(another_matrix->get_value(3,0),4.5);
    EXPECT_FLOAT_EQ(another_matrix->get_value(3,1),2.);

    // clean up
    safe_free(another_matrix);
}

PSL_TEST(DenseMatrixBuilder,identity)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrix
    const size_t nrows = 3;
    const size_t ncols = 4;
    AbstractInterface::DenseMatrix* matrix = builder->build_by_fill(nrows, ncols, 0.0);

    // use identity
    matrix->set_to_identity();

    // test expectations
    for(size_t row_index = 0u; row_index < nrows; ++row_index)
    {
        for(size_t col_index = 0u; col_index < ncols; ++col_index)
        {
            if(row_index == col_index)
            {
                EXPECT_FLOAT_EQ(matrix->get_value(row_index,col_index), 1.0);
            }
            else
            {
                EXPECT_FLOAT_EQ(matrix->get_value(row_index,col_index), 0.0);
            }
        }
    }

    // clean up
    safe_free(matrix);
}

PSL_TEST(DenseMatrixBuilder,aXpY)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrices
    const size_t nrows = 2;
    const size_t ncols = 4;
    std::vector<double> matrix_one_values = {1., 2., 3., 4., 0., 5., 3., 7.};
    AbstractInterface::DenseMatrix* matrix_one = builder->build_by_row_major(nrows, ncols, matrix_one_values);
    std::vector<double> matrix_two_values = {2., 4., 7., 9., 3., 6., 2., 5.};
    AbstractInterface::DenseMatrix* matrix_two = builder->build_by_row_major(nrows, ncols, matrix_two_values);

    // use aXpY
    const double alpha = 2.0;
    matrix_two->aXpY(alpha, matrix_one);

    // test expectations
    EXPECT_FLOAT_EQ(matrix_two->get_value(0,0), 4.);
    EXPECT_FLOAT_EQ(matrix_two->get_value(0,1), 8.);
    EXPECT_FLOAT_EQ(matrix_two->get_value(0,2), 13.);
    EXPECT_FLOAT_EQ(matrix_two->get_value(0,3), 17.);
    EXPECT_FLOAT_EQ(matrix_two->get_value(1,0), 3.);
    EXPECT_FLOAT_EQ(matrix_two->get_value(1,1), 16.);
    EXPECT_FLOAT_EQ(matrix_two->get_value(1,2), 8.);
    EXPECT_FLOAT_EQ(matrix_two->get_value(1,3), 19.);

    // clean up
    safe_free(matrix_one);
    safe_free(matrix_two);
}

PSL_TEST(DenseMatrixBuilder,product)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrices
    const size_t nrows = 4;
    const size_t inner = 3;
    const size_t ncols = 2;
    std::vector<double> left_matrix_values = {1., 2., 3., 4., 0., 2., 5., -3., -1., -2., 7., 3.};
    AbstractInterface::DenseMatrix* left_matrix = builder->build_by_row_major(nrows, inner, left_matrix_values);
    std::vector<double> right_matrix_values = {1., 5., 2., -1., -7., -3.};
    AbstractInterface::DenseMatrix* right_matrix = builder->build_by_row_major(inner, ncols, right_matrix_values);
    AbstractInterface::DenseMatrix* result_matrix = builder->build_by_fill(nrows, ncols, 0.0);

    // compute product
    const double alpha = 2.0;
    result_matrix->matrix_matrix_product(alpha,left_matrix,false,right_matrix,false);

    // test expectations
    EXPECT_FLOAT_EQ(result_matrix->get_value(0,0), -32.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(0,1), -12.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(1,0), -20.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(1,1),  28.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(2,0),  12.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(2,1),  62.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(3,0), -18.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(3,1), -52.);

    safe_free(left_matrix);
    safe_free(right_matrix);
    safe_free(result_matrix);
}

PSL_TEST(DenseMatrixBuilder,productLeftTranspose)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrices
    const size_t nrows = 4;
    const size_t inner = 3;
    const size_t ncols = 2;
    std::vector<double> left_matrix_values = {1., 4., 5., -2., 2., 0., -3., 7., 3., 2., -1., 3.};
    AbstractInterface::DenseMatrix* left_matrix = builder->build_by_row_major(inner, nrows, left_matrix_values);
    std::vector<double> right_matrix_values = {4., 1., 7., 0., -3., -1.};
    AbstractInterface::DenseMatrix* right_matrix = builder->build_by_row_major(inner, ncols, right_matrix_values);
    AbstractInterface::DenseMatrix* result_matrix = builder->build_by_fill(nrows, ncols, 0.0);

    // compute product
    const double alpha = 2.0;
    result_matrix->matrix_matrix_product(alpha, left_matrix, true, right_matrix, false);

    // test expectations
    EXPECT_FLOAT_EQ(result_matrix->get_value(0,0), 18.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(0,1), -4.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(1,0), 20.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(1,1),  4.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(2,0),  4.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(2,1), 12.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(3,0), 64.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(3,1), -10.);

    safe_free(left_matrix);
    safe_free(right_matrix);
    safe_free(result_matrix);
}

PSL_TEST(DenseMatrixBuilder,productRightTranspose)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrices
    const size_t nrows = 4;
    const size_t inner = 2;
    const size_t ncols = 3;
    std::vector<double> left_matrix_values = {1., 2., 3., 4., 0., 2., 5., -3.};
    AbstractInterface::DenseMatrix* left_matrix = builder->build_by_row_major(nrows, inner, left_matrix_values);
    std::vector<double> right_matrix_values = {1., 0., 2., -1., -7., -3.};
    AbstractInterface::DenseMatrix* right_matrix = builder->build_by_row_major(ncols, inner, right_matrix_values);
    AbstractInterface::DenseMatrix* result_matrix = builder->build_by_fill(nrows, ncols, 0.0);

    // compute product
    const double alpha = -1.0;
    result_matrix->matrix_matrix_product(alpha, left_matrix, false, right_matrix, true);

    // test expectations
    EXPECT_FLOAT_EQ(result_matrix->get_value(0,0), -1.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(0,1),  0.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(0,2),  13.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(1,0), -3.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(1,1), -2.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(1,2),  33.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(2,0),  0.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(2,1),  2.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(2,2),  6.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(3,0), -5.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(3,1), -13.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(3,2),  26.);

    safe_free(left_matrix);
    safe_free(right_matrix);
    safe_free(result_matrix);
}

PSL_TEST(DenseMatrixBuilder,productBothTranspose)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrices
    const size_t nrows = 3;
    const size_t inner = 4;
    const size_t ncols = 2;
    std::vector<double> left_matrix_values = {1., 2., 3., 4., 0., 2., 5., -3., -1., -2., 7., 3.};
    AbstractInterface::DenseMatrix* left_matrix = builder->build_by_row_major(inner, nrows, left_matrix_values);
    std::vector<double> right_matrix_values = {1., 5., 2., -1., 0., -7., -3., 5.};
    AbstractInterface::DenseMatrix* right_matrix = builder->build_by_row_major(ncols, inner, right_matrix_values);
    AbstractInterface::DenseMatrix* result_matrix = builder->build_by_fill(nrows, ncols, 0.0);

    // compute product
    const double alpha = 3.0;
    result_matrix->matrix_matrix_product(alpha, left_matrix, true, right_matrix, true);

    // test expectations
    EXPECT_FLOAT_EQ(result_matrix->get_value(0,0),  99.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(0,1), -159.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(1,0), -33.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(1,1),  132.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(2,0),  24.);
    EXPECT_FLOAT_EQ(result_matrix->get_value(2,1),  12.);

    safe_free(left_matrix);
    safe_free(right_matrix);
    safe_free(result_matrix);
}

PSL_TEST(DenseMatrixBuilder,scale)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrix
    const size_t nrows = 4;
    const size_t ncols = 2;
    std::vector<double> matrix_values = {0., 1., 5., 7., -1., -3., 4.5, 2.};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(nrows, ncols, matrix_values);

    // use scale
    const double alpha = 2.;
    matrix->scale(alpha);

    // test expectations
    EXPECT_FLOAT_EQ(matrix->get_value(0,0), 0.);
    EXPECT_FLOAT_EQ(matrix->get_value(0,1), 2.);
    EXPECT_FLOAT_EQ(matrix->get_value(1,0), 10.);
    EXPECT_FLOAT_EQ(matrix->get_value(1,1), 14.);
    EXPECT_FLOAT_EQ(matrix->get_value(2,0), -2.);
    EXPECT_FLOAT_EQ(matrix->get_value(2,1), -6.);
    EXPECT_FLOAT_EQ(matrix->get_value(3,0), 9.);
    EXPECT_FLOAT_EQ(matrix->get_value(3,1), 4.);

    // clean up
    safe_free(matrix);
}

PSL_TEST(DenseMatrixBuilder,dotSelf)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrix
    const size_t nrows = 4;
    const size_t ncols = 2;
    std::vector<double> matrix_values = {0., 1.1, 5., 7., -1., -3., 4.5, 2.};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(nrows, ncols, matrix_values);

    // compute
    const double computed_sum = matrix->dot(matrix);

    // compare to expectation
    const double expected_sum = 109.46;
    EXPECT_FLOAT_EQ(computed_sum, expected_sum);

    // clean up
    safe_free(matrix);
}

PSL_TEST(DenseMatrixBuilder,dotOther)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrix
    const size_t nrows = 4;
    const size_t ncols = 2;
    std::vector<double> matrix_values0 = {0., 1.1, 5., 7., -1., -3., 4.5, 2.};
    AbstractInterface::DenseMatrix* matrix0 = builder->build_by_row_major(nrows, ncols, matrix_values0);
    std::vector<double> matrix_values1 = {-1., -3., 0., 2., .3, 1.1, -5., 7.};
    AbstractInterface::DenseMatrix* matrix1 = builder->build_by_row_major(nrows, ncols, matrix_values1);

    // compute
    const double computed_sumA = matrix0->dot(matrix1);
    const double computed_sumB = matrix1->dot(matrix0);

    // compare to expectation
    const double expected_sum = -1.4;
    EXPECT_FLOAT_EQ(computed_sumA, expected_sum);
    EXPECT_FLOAT_EQ(computed_sumB, expected_sum);

    // clean up
    safe_free(matrix0);
    safe_free(matrix1);
}

PSL_TEST(DenseMatrixBuilder,getDiagonal)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrix
    const size_t nsize1 = 2;
    const size_t nsize2 = 3;
    std::vector<double> matrix_values0 = {0., 1.1, 5., 7.};
    std::vector<double> matrix_values1 = {-1., -3., 0., 2., .3, 1.1};
    std::vector<double> matrix_values3 = {0., 1.1, 5., 7., -1., -3., 4.5, 2., -6};
    AbstractInterface::DenseMatrix* matrix0 = builder->build_by_row_major(nsize1, nsize1, matrix_values0);
    AbstractInterface::DenseMatrix* matrix1 = builder->build_by_row_major(nsize1, nsize2, matrix_values1);
    AbstractInterface::DenseMatrix* matrix2 = builder->build_by_row_major(nsize2, nsize1, matrix_values1);
    AbstractInterface::DenseMatrix* matrix3 = builder->build_by_row_major(nsize2, nsize2, matrix_values3);

    // compute & compare
    std::vector<double> computed_diagonal;
    matrix0->get_diagonal(computed_diagonal);
    std::vector<double> expected_diagonal0 = {matrix_values0[0], matrix_values0[3]};
    expect_equal_float_vectors(computed_diagonal, expected_diagonal0);

    // compute & compare
    matrix1->get_diagonal(computed_diagonal);
    std::vector<double> expected_diagonal1 = {matrix_values1[0], matrix_values1[4]};
    expect_equal_float_vectors(computed_diagonal, expected_diagonal1);

    // compute & compare
    matrix2->get_diagonal(computed_diagonal);
    std::vector<double> expected_diagonal2 = {matrix_values1[0], matrix_values1[3]};
    expect_equal_float_vectors(computed_diagonal, expected_diagonal2);

    // compute & compare
    matrix3->get_diagonal(computed_diagonal);
    std::vector<double> expected_diagonal3 = {matrix_values3[0], matrix_values3[4], matrix_values3[8]};
    expect_equal_float_vectors(computed_diagonal, expected_diagonal3);

    // clean up
    safe_free(matrix0);
    safe_free(matrix1);
    safe_free(matrix2);
    safe_free(matrix3);
}

PSL_TEST(DenseMatrixBuilder,permuteColumns)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrix
    const size_t nrows = 2;
    const size_t ncols = 3;
    std::vector<double> matrix_values = {-1., -3., 0., 2., .3, 1.1};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(nrows, ncols, matrix_values);

    // do permute
    std::vector<int> permute = {0, 2, 1};
    matrix->permute_columns(permute);

    // compare to expectations
    std::vector<double> expected_matrix_values = {matrix_values[0], matrix_values[2], matrix_values[1],
                                                  matrix_values[3], matrix_values[5], matrix_values[4]};
    AbstractInterface::DenseMatrix* expected_matrix = builder->build_by_row_major(nrows, ncols, expected_matrix_values);
    expect_equal_matrix(matrix, expected_matrix);

    // clean up
    safe_free(matrix);
    safe_free(expected_matrix);
}

PSL_TEST(DenseMatrixBuilder,scaleColumns)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrix
    const size_t nrows = 2;
    const size_t ncols = 3;
    std::vector<double> matrix_values = {-1., -3., 0., 2., .3, 1.1};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(nrows, ncols, matrix_values);

    // do scale
    const double scale0 = 1.32;
    matrix->scale_column(1, scale0);

    // compare to expectations
    std::vector<double> expected_values = matrix_values;
    expected_values[1] *= scale0;
    expected_values[4] *= scale0;
    AbstractInterface::DenseMatrix* expected_matrix = builder->build_by_row_major(nrows, ncols, expected_values);
    expect_equal_matrix(matrix, expected_matrix);
    safe_free(expected_matrix);

    // do scale
    const double scale1 = -2.57;
    matrix->scale_column(2, scale1);

    // compare to expectations
    expected_values[2] *= scale1;
    expected_values[5] *= scale1;
    expected_matrix = builder->build_by_row_major(nrows, ncols, expected_values);
    expect_equal_matrix(matrix, expected_matrix);
    safe_free(expected_matrix);

    // do scale
    const double scale2 = 0.;
    matrix->scale_column(1, scale2);

    // compare to expectations
    expected_values[1] *= scale2;
    expected_values[4] *= scale2;
    expected_matrix = builder->build_by_row_major(nrows, ncols, expected_values);
    expect_equal_matrix(matrix, expected_matrix);
    safe_free(expected_matrix);

    // clean up
    safe_free(matrix);
}

PSL_TEST(DenseMatrixBuilder,getRowAndColumn)
{
    set_rand_seed();
    DenseMatrixBuilderTest_AllocateUtilities

    // create matrix
    const size_t nrows = 4;
    const size_t ncols = 3;
    std::vector<double> v = {0., 1.1, 5., 7., -1., -3., 4.5, 2., 5.1, -.15, -.27, 1.2};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(nrows, ncols, v);

    std::vector<std::vector<double> > expected_rows = {{v[0], v[1], v[2]},
                                                       {v[3], v[4], v[5]},
                                                       {v[6], v[7], v[8]},
                                                       {v[9], v[10], v[11]}};

    // for each row
    for(size_t r = 0u; r < nrows; r++)
    {
        std::vector<double> computed_row;
        matrix->get_row(r, computed_row);

        // compare to expectation
        expect_equal_float_vectors(computed_row, expected_rows[r]);
    }

    std::vector<std::vector<double> > expected_columns = {{v[0], v[3], v[6], v[9]},
                                                          {v[1], v[4], v[7], v[10]},
                                                          {v[2], v[5], v[8], v[11]}};

    // for each column
    for(size_t c = 0u; c < ncols; c++)
    {
        std::vector<double> computed_column;
        matrix->get_column(c, computed_column);

        // compare to expectation
        expect_equal_float_vectors(computed_column, expected_columns[c]);
    }

    // clean up
    safe_free(matrix);
}

}

}
