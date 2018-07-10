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

#include "PSL_UnitTestingHelper.hpp"

#include "PSL_MultiLayoutVector.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Interface_BasicDenseVectorOperations.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_DataOrganizerFactory.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_InputOutputManager.hpp"
#include "PSL_Random.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace TestingMultiLayoutVector
{

PSL_TEST(MultiLayoutVector, getAndSet)
{
    set_rand_seed();
    // allocate
    AbstractAuthority authority;
    MultiLayoutVector mlv(&authority);

    // fill integer scalars
    const std::vector<int> expected_ints = {0, -1, 2, 32154, 7, -7, 5};
    const int num_ints = expected_ints.size();
    mlv.resize_ints(num_ints);
    mlv.set_int(0, expected_ints[0] + 1); // intentionally superfluous
    for(int i = 0; i < num_ints; i++)
    {
        mlv.set_int(i, expected_ints[i]);
    }

    // fill double scalars
    const std::vector<double> expected_doubles = {1., 2., 0., 5., 3., 7., -13.1, -.2, -42., 58.9, 17.1};
    const int num_doubles = expected_doubles.size();
    mlv.resize_doubles(num_doubles);
    mlv.set_double(1, expected_doubles[1] + 1.); // intentionally superfluous
    for(int i = 0; i < num_doubles; i++)
    {
        mlv.set_double(i, expected_doubles[i]);
    }

    // fill integer vectors
    const int num_int_vectors = 5;
    mlv.resize_int_vectors(num_int_vectors);
    const std::vector<int> iv0 = {3, -1, 2};
    mlv.transfer_int_vector(0, iv0);
    mlv.transfer_int_vector(1, iv0); // intentionally superfluous
    std::vector<int> iv1 = {};
    mlv.set_int_vector(1, &iv1);
    std::vector<int> iv2 = {5, 2, 7};
    mlv.set_int_vector(2, &iv2);
    std::vector<int> iv3 = {4, 1, 9, 11};
    mlv.set_int_vector(3, &iv3);
    const std::vector<int> iv4 = {0, 7};
    mlv.transfer_int_vector(4, iv4);

    // fill double vectors
    const int num_double_vectors = 2;
    mlv.resize_double_vectors(num_double_vectors);
    std::vector<double> dv0 = {5., -3., 2.};
    mlv.set_double_vector(0, &dv0);
    mlv.set_double_vector(1, &dv0); // intentionally superfluous
    const std::vector<double> dv1 = {-2., 4.2};
    mlv.transfer_double_vector(1, dv1);

    // fill matrices
    const int num_matrices = 3;
    mlv.resize_matrices(num_matrices);
    const std::vector<double> m0v = {1., 2., 3., 4.1, -9., 8.2};
    AbstractInterface::DenseMatrix* m0 = authority.dense_builder->build_by_row_major(2u, 3u, m0v);
    mlv.set_matrix(0, m0);
    const std::vector<double> m1v = {-5.1, 2.7, 3., 4.2, 0., 7.2, 2.1, 5.7};
    AbstractInterface::DenseMatrix* m1 = authority.dense_builder->build_by_row_major(4u, 2u, m1v);
    mlv.transfer_matrix(1, m1);
    mlv.set_matrix(2, m1); // intentionally superfluous
    const std::vector<double> m2v = {4.1, -9., 8.2, 1., 2., 3.};
    AbstractInterface::DenseMatrix* m2 = authority.dense_builder->build_by_row_major(1u, 6u, m2v);
    mlv.set_matrix(2, m2);

    // ensure accuracy of integer scalars
    EXPECT_EQ(num_ints, mlv.get_ints_size());
    std::vector<int> computed_all_ints;
    mlv.get_all_ints(computed_all_ints);
    expect_equal_vectors(computed_all_ints, expected_ints);
    for(int i = 0; i < num_ints; i++)
    {
        EXPECT_EQ(expected_ints[i], *mlv.get_int_ptr(i));
    }

    // ensure accuracy of double scalars
    EXPECT_EQ(num_doubles, mlv.get_doubles_size());
    std::vector<double> computed_all_doubles;
    mlv.get_all_doubles(computed_all_doubles);
    expect_equal_float_vectors(computed_all_doubles, expected_doubles);
    for(int i = 0; i < num_doubles; i++)
    {
        EXPECT_EQ(expected_doubles[i], *mlv.get_double_ptr(i));
    }

    // ensure accuracy of integer vectors
    EXPECT_EQ(num_int_vectors, mlv.get_int_vectors_size());
    expect_equal_vectors(*mlv.get_int_vector(0), iv0);
    expect_equal_vectors(*mlv.get_int_vector(1), iv1);
    expect_equal_vectors(*mlv.get_int_vector(2), iv2);
    expect_equal_vectors(*mlv.get_int_vector(3), iv3);
    expect_equal_vectors(*mlv.get_int_vector(4), iv4);

    // ensure accuracy of double vectors
    EXPECT_EQ(num_double_vectors, mlv.get_double_vectors_size());
    expect_equal_float_vectors(*mlv.get_double_vector(0), dv0);
    expect_equal_float_vectors(*mlv.get_double_vector(1), dv1);

    // ensure accuracy of double vectors
    EXPECT_EQ(num_matrices, mlv.get_matrices_size());
    expect_equal_matrix(m0, mlv.get_matrix(0));
    expect_equal_matrix(m1, mlv.get_matrix(1));
    expect_equal_matrix(m2, mlv.get_matrix(2));

    // cleanup memory
    delete m0;
    delete m2;
}

PSL_TEST(MultiLayoutVector, axpy)
{
    set_rand_seed();
    // allocate
    AbstractAuthority authority;
    MultiLayoutVector x(&authority);
    MultiLayoutVector y(&authority);
    x.resize_ints(2);
    y.resize_ints(4);
    const int num_doubles = 5;
    x.resize_doubles(num_doubles);
    y.resize_doubles(num_doubles);
    const int num_double_vectors = 3;
    x.resize_double_vectors(num_double_vectors);
    y.resize_double_vectors(num_double_vectors);
    const int num_matrices = 2;
    x.resize_matrices(num_matrices);
    y.resize_matrices(num_matrices);

    // fill scalars
    x.set_double(0, 1.);
    x.set_double(1, 2.);
    x.set_double(2, 3.);
    x.set_double(3, 5.);
    x.set_double(4, 7.);
    y.set_double(0, 2.1);
    y.set_double(1, 4.2);
    y.set_double(2, 8.3);
    y.set_double(3, -16.4);
    y.set_double(4, 32.5);

    // fill vectors
    const std::vector<double> gold_x_dv0 = {5., -3., 2.};
    std::vector<double> dvx0 = gold_x_dv0;
    x.set_double_vector(0, &dvx0);
    const std::vector<double> gold_x_dv1 = {};
    const std::vector<double> gold_x_dv2 = {-2., 4.2};
    std::vector<double> dvx2 = gold_x_dv2;
    x.set_double_vector(2, &dvx2);
    std::vector<double> dvy0 = {-3., 7., 2.};
    y.set_double_vector(0, &dvy0);
    std::vector<double> dvy2 = {4., -5.1};
    y.set_double_vector(2, &dvy2);

    // fill matrices
    const std::vector<double> mx0_v = {1., 2., 3., 4.1, -9., 8.2};
    AbstractInterface::DenseMatrix* mx0 = authority.dense_builder->build_by_row_major(2u, 3u, mx0_v);
    x.set_matrix(0, mx0);
    const std::vector<double> mx1_v = {-5.1, 2.7, 3., 4.2, 0., 7.2, 2.1, 5.7};
    AbstractInterface::DenseMatrix* mx1 = authority.dense_builder->build_by_row_major(4u, 2u, mx1_v);
    x.set_matrix(1, mx1);
    const std::vector<double> my0_v = {4.1, -9., 8.2, 1., 2., 3.};
    AbstractInterface::DenseMatrix* my0 = authority.dense_builder->build_by_row_major(2u, 3u, my0_v);
    y.set_matrix(0, my0);
    const std::vector<double> my1_v = {0., 7.2, 2.1, 5.7, 4.2, -5.1, 2.7, 3.};
    AbstractInterface::DenseMatrix* my1 = authority.dense_builder->build_by_row_major(4u, 2u, my1_v);
    y.set_matrix(1, my1);

    // axpy
    const double scale = -1.2;
    y.axpy(scale, &x);

    // set scalar golds
    const std::vector<double> gold_x_doubles = {1., 2., 3., 5., 7.};
    const std::vector<double> gold_y_doubles = {.9, 1.8, 4.7, -22.4, 24.1};

    // set vector golds
    const std::vector<double> gold_y_dv0 = {-9., 10.6, -0.4};
    const std::vector<double> gold_y_dv1 = {};
    const std::vector<double> gold_y_dv2 = {6.4, -10.14};

    // set matrix golds
    AbstractInterface::DenseMatrix* gold_mx0 = authority.dense_builder->build_by_row_major(2u, 3u, mx0_v);
    AbstractInterface::DenseMatrix* gold_mx1 = authority.dense_builder->build_by_row_major(4u, 2u, mx1_v);
    const std::vector<double> gold_my0_v = {2.9, -11.4, 4.6, -3.92, 12.8, -6.84};
    AbstractInterface::DenseMatrix* gold_my0 = authority.dense_builder->build_by_row_major(2u, 3u, gold_my0_v);
    const std::vector<double> gold_my1_v = {6.12, 3.96, -1.5, 0.66, 4.2, -13.74, .18, -3.84};
    AbstractInterface::DenseMatrix* gold_my1 = authority.dense_builder->build_by_row_major(4u, 2u, gold_my1_v);

    // compare scalars to expectations
    ASSERT_EQ(num_doubles, x.get_doubles_size());
    ASSERT_EQ(num_doubles, y.get_doubles_size());
    std::vector<double> computed_x_doubles;
    x.get_all_doubles(computed_x_doubles);
    expect_equal_float_vectors(gold_x_doubles, computed_x_doubles);
    std::vector<double> computed_y_doubles;
    y.get_all_doubles(computed_y_doubles);
    expect_equal_float_vectors(gold_y_doubles, computed_y_doubles);

    // compare vectors to expectations
    ASSERT_EQ(num_double_vectors, x.get_double_vectors_size());
    ASSERT_EQ(num_double_vectors, y.get_double_vectors_size());
    expect_equal_float_vectors(gold_x_dv0, *x.get_double_vector(0));
    expect_equal_float_vectors(gold_x_dv1, *x.get_double_vector(1));
    expect_equal_float_vectors(gold_x_dv2, *x.get_double_vector(2));
    expect_equal_float_vectors(gold_y_dv0, *y.get_double_vector(0));
    expect_equal_float_vectors(gold_y_dv1, *y.get_double_vector(1));
    expect_equal_float_vectors(gold_y_dv2, *y.get_double_vector(2));

    // compare matrices to expectations
    ASSERT_EQ(num_matrices, x.get_matrices_size());
    ASSERT_EQ(num_matrices, y.get_matrices_size());
    expect_equal_matrix(gold_mx0, x.get_matrix(0));
    expect_equal_matrix(gold_mx1, x.get_matrix(1));
    expect_equal_matrix(gold_my0, y.get_matrix(0));
    expect_equal_matrix(gold_my1, y.get_matrix(1));

    // cleanup memory
    delete mx0;
    delete mx1;
    delete my0;
    delete my1;
    delete gold_mx0;
    delete gold_mx1;
    delete gold_my0;
    delete gold_my1;
}

PSL_TEST(MultiLayoutVector, dotSelf)
{
    set_rand_seed();
    // allocate
    AbstractAuthority authority;
    MultiLayoutVector mlv(&authority);
    mlv.resize_ints(3);
    const int num_doubles = 7;
    mlv.resize_doubles(num_doubles);
    const int num_double_vectors = 4;
    mlv.resize_double_vectors(num_double_vectors);
    const int num_matrices = 3;
    mlv.resize_matrices(num_matrices);

    // fill scalars
    mlv.set_double(0, 1.);
    mlv.set_double(1, 2.);
    mlv.set_double(2, 3.);
    mlv.set_double(3, 5.);
    mlv.set_double(4, 7.);
    mlv.set_double(5, -13.1);
    mlv.set_double(6, -.2);

    // fill vectors
    std::vector<double> v0 = {5., -3., 2.};
    mlv.set_double_vector(0, &v0);
    std::vector<double> v1 = {};
    mlv.set_double_vector(1, &v1);
    std::vector<double> v2 = {-2., 4.2};
    mlv.set_double_vector(2, &v2);
    std::vector<double> v3 = {-3., 7., 2.};
    mlv.set_double_vector(3, &v3);

    // fill matrices
    const std::vector<double> m0v = {1., 2., 3., 4.1, -9., 8.2};
    AbstractInterface::DenseMatrix* m0 = authority.dense_builder->build_by_row_major(2u, 3u, m0v);
    mlv.set_matrix(0, m0);
    const std::vector<double> m1v = {-5.1, 2.7, 3., 4.2, 0., 7.2, 2.1, 5.7};
    AbstractInterface::DenseMatrix* m1 = authority.dense_builder->build_by_row_major(4u, 2u, m1v);
    mlv.set_matrix(1, m1);
    const std::vector<double> m2v = {4.1, -9., 8.2, 1., 2., 3.};
    AbstractInterface::DenseMatrix* m2 = authority.dense_builder->build_by_row_major(1u, 6u, m2v);
    mlv.set_matrix(2, m2);

    // compute
    const double computed_sum_of_squares = mlv.dot(&mlv);

    // compare to expectation
    const double expected_sum_of_squares = 888.07;
    EXPECT_FLOAT_EQ(computed_sum_of_squares, expected_sum_of_squares);

    // cleanup memory
    delete m0;
    delete m1;
    delete m2;
}

PSL_TEST(MultiLayoutVector, dotOther)
{
    set_rand_seed();
    // allocate
    AbstractAuthority authority;
    MultiLayoutVector mlv_A(&authority);
    mlv_A.resize_ints(3);
    const int num_doubles = 5;
    mlv_A.resize_doubles(num_doubles);
    const int num_double_vectors = 3;
    mlv_A.resize_double_vectors(num_double_vectors);
    const int num_matrices = 2;
    mlv_A.resize_matrices(num_matrices);

    // fill scalars
    mlv_A.set_double(0, 1.);
    mlv_A.set_double(1, .2);
    mlv_A.set_double(2, -13.1);
    mlv_A.set_double(3, 5.);
    mlv_A.set_double(4, 7.);

    // fill vectors
    std::vector<double> v0_A = {5., -3., 2.};
    mlv_A.set_double_vector(0, &v0_A);
    std::vector<double> v1_A = {};
    mlv_A.set_double_vector(1, &v1_A);
    std::vector<double> v2_A = {-2., 4.2};
    mlv_A.set_double_vector(2, &v2_A);

    // fill matrices
    const std::vector<double> m0v_A = {1., 2., 3., 4.1, -9., 8.2};
    AbstractInterface::DenseMatrix* m0_A = authority.dense_builder->build_by_row_major(2u, 3u, m0v_A);
    mlv_A.set_matrix(0, m0_A);
    const std::vector<double> m1v_A = {4.1, -9., 8.2, 1., 2., 3.};
    AbstractInterface::DenseMatrix* m1_A = authority.dense_builder->build_by_row_major(1u, 6u, m1v_A);
    mlv_A.set_matrix(1, m1_A);

    // allocate
    MultiLayoutVector mlv_B(&authority);
    mlv_B.resize_doubles(num_doubles);
    mlv_B.resize_double_vectors(num_double_vectors);
    mlv_B.resize_matrices(num_matrices);

    // fill scalars
    mlv_B.set_double(0, 4.2);
    mlv_B.set_double(1, 0.);
    mlv_B.set_double(2, 1.2);
    mlv_B.set_double(3, -3.);
    mlv_B.set_double(4, 0.1);

    // fill vectors
    std::vector<double> v0_B = {4., 2.1, -3.};
    mlv_B.set_double_vector(0, &v0_B);
    std::vector<double> v1_B = {};
    mlv_B.set_double_vector(1, &v1_B);
    std::vector<double> v2_B = {-3.1, 2.};
    mlv_B.set_double_vector(2, &v2_B);

    // fill matrices
    const std::vector<double> m0v_B = {7.1, -9., 8.2, 2., -3., 4.1};
    AbstractInterface::DenseMatrix* m0_B = authority.dense_builder->build_by_row_major(2u, 3u, m0v_B);
    mlv_B.set_matrix(0, m0_B);
    const std::vector<double> m1v_B = {4.1, -3., -9., 8.2, 1., 1.97};
    AbstractInterface::DenseMatrix* m1_B = authority.dense_builder->build_by_row_major(1u, 6u, m1v_B);
    mlv_B.set_matrix(1, m1_B);

    // compute
    const double calc_0 = mlv_A.dot(&mlv_B);
    const double calc_1 = mlv_B.dot(&mlv_A);

    // compare to expectation
    const double expected_calc = 65.12;
    EXPECT_FLOAT_EQ(calc_0, expected_calc);
    EXPECT_FLOAT_EQ(calc_1, expected_calc);

    // cleanup memory
    delete m0_A;
    delete m1_A;
    delete m0_B;
    delete m1_B;
}

PSL_TEST(MultiLayoutVector, zeros)
{
    set_rand_seed();
    // allocate
    AbstractAuthority authority;
    MultiLayoutVector original_mlv(&authority);
    original_mlv.resize_ints(3);
    const int original_num_doubles = 7;
    original_mlv.resize_doubles(original_num_doubles);
    const int num_double_vectors = 4;
    original_mlv.resize_double_vectors(num_double_vectors);
    const int num_matrices = 3;
    original_mlv.resize_matrices(num_matrices);

    // fill scalars
    original_mlv.set_double(0, 1.);
    original_mlv.set_double(1, 2.);
    original_mlv.set_double(2, 3.);
    original_mlv.set_double(3, 5.);
    original_mlv.set_double(4, 7.);
    original_mlv.set_double(5, -13.1);
    original_mlv.set_double(6, -.2);

    // fill vectors
    std::vector<double> v0 = {5., -3., 2.};
    original_mlv.set_double_vector(0, &v0);
    std::vector<double> v1 = {};
    original_mlv.set_double_vector(1, &v1);
    std::vector<double> v2 = {-2., 4.2};
    original_mlv.set_double_vector(2, &v2);
    std::vector<double> v3 = {-3., 7., 2.};
    original_mlv.set_double_vector(3, &v3);

    // fill matrices
    const std::vector<double> m0v = {1., 2., 3., 4.1, -9., 8.2};
    AbstractInterface::DenseMatrix* m0 = authority.dense_builder->build_by_row_major(2u, 3u, m0v);
    original_mlv.transfer_matrix(0, m0);
    const std::vector<double> m1v = {-5.1, 2.7, 3., 4.2, 0., 7.2, 2.1, 5.7};
    AbstractInterface::DenseMatrix* m1 = authority.dense_builder->build_by_row_major(4u, 2u, m1v);
    original_mlv.set_matrix(1, m1);
    const std::vector<double> m2v = {4.1, -9., 8.2, 1., 2., 3.};
    AbstractInterface::DenseMatrix* m2 = authority.dense_builder->build_by_row_major(1u, 6u, m2v);
    original_mlv.set_matrix(2, m2);

    // build from zeros
    MultiLayoutVector other_mlv(&authority);
    const int initial_num_ints = 1;
    other_mlv.resize_ints(initial_num_ints);
    const int initial_num_int_vectors = 3;
    other_mlv.resize_int_vectors(initial_num_int_vectors);
    other_mlv.resize_double_vectors(num_double_vectors * 2 + 1); // intentionally superfluous
    other_mlv.zeros(&original_mlv);

    // ints same as initial
    const int other_ints_size = other_mlv.get_ints_size();
    EXPECT_EQ(other_ints_size, initial_num_ints);
    const int other_int_vectors_size = other_mlv.get_int_vectors_size();
    EXPECT_EQ(other_int_vectors_size, initial_num_int_vectors);

    // doubles get sizes
    const int other_doubles_size = other_mlv.get_doubles_size();
    EXPECT_EQ(other_doubles_size, original_num_doubles);
    const int other_double_vectors_size = other_mlv.get_double_vectors_size();
    ASSERT_EQ(other_double_vectors_size, num_double_vectors);
    const int other_matrices_size = other_mlv.get_matrices_size();
    ASSERT_EQ(other_matrices_size, num_matrices);

    // compare scalar doubles to expectations
    std::vector<double> computed_doubles_vector;
    other_mlv.get_all_doubles(computed_doubles_vector);
    const std::vector<double> gold_doubles_vector(original_num_doubles, 0.);
    expect_equal_float_vectors(computed_doubles_vector, gold_doubles_vector);

    // compare vector doubles to expectations
    for(int i = 0; i < other_double_vectors_size; i++)
    {
        std::vector<double>* computed = other_mlv.get_double_vector(i);
        std::vector<double> gold(original_mlv.get_double_vector(i)->size(), 0.);
        expect_equal_float_vectors(*computed, gold);
    }

    // compare matrix to expectations
    for(int i = 0; i < other_matrices_size; i++)
    {
        AbstractInterface::DenseMatrix* computed = other_mlv.get_matrix(i);
        AbstractInterface::DenseMatrix* original = original_mlv.get_matrix(i);
        AbstractInterface::DenseMatrix* gold = authority.dense_builder->build_by_fill(original->get_num_rows(), original->get_num_columns(), 0.);
        expect_equal_matrix(computed, gold);
        delete gold;
    }

    // cleanup memory
    delete m1;
    delete m2;
}

PSL_TEST(MultiLayoutVector, scale)
{
    set_rand_seed();
    // allocate
    AbstractAuthority authority;
    MultiLayoutVector mlv(&authority);
    mlv.resize_ints(3);
    const int num_doubles = 7;
    mlv.resize_doubles(num_doubles);
    const int num_double_vectors = 4;
    mlv.resize_double_vectors(num_double_vectors);
    const int num_matrices = 3;
    mlv.resize_matrices(num_matrices);

    // fill scalars
    std::vector<double> doubles = {1., 2., 3., 5., 7., -13.1, -.2};
    for(int d = 0; d < num_doubles; d++)
    {
        mlv.set_double(d, doubles[d]);
    }

    // fill vectors
    std::vector<std::vector<double> > vector_original = { {5., -3., 2.}, {}, {-2., 4.2}, {-3., 7., 2.}};
    std::vector<double> v0 = vector_original[0];
    mlv.set_double_vector(0, &v0);
    std::vector<double> v1 = vector_original[1];
    mlv.set_double_vector(1, &v1);
    std::vector<double> v2 = vector_original[2];
    mlv.transfer_double_vector(2, v2);
    std::vector<double> v3 = vector_original[3];
    mlv.set_double_vector(3, &v3);

    // fill matrices
    const std::vector<size_t> matrix_row_sizes = {2u, 4u, 1u};
    const std::vector<size_t> matrix_column_sizes = {3u, 2u, 6u};
    std::vector<std::vector<double> > matrix_original = { {1., 2., 3., 4.1, -9., 8.2}, {-5.1, 2.7, 3., 4.2, 0., 7.2, 2.1, 5.7}, {
            4.1, -9., 8.2, 1., 2., 3.}};
    AbstractInterface::DenseMatrix* m0 = authority.dense_builder->build_by_row_major(matrix_row_sizes[0],
                                                                    matrix_column_sizes[0],
                                                                    matrix_original[0]);
    mlv.transfer_matrix(0, m0);
    AbstractInterface::DenseMatrix* m1 = authority.dense_builder->build_by_row_major(matrix_row_sizes[1],
                                                                    matrix_column_sizes[1],
                                                                    matrix_original[1]);
    mlv.transfer_matrix(1, m1);
    AbstractInterface::DenseMatrix* m2 = authority.dense_builder->build_by_row_major(matrix_row_sizes[2],
                                                                    matrix_column_sizes[2],
                                                                    matrix_original[2]);
    mlv.set_matrix(2, m2);

    // scale
    const double scale_factor = -2.015;
    mlv.scale(scale_factor);

    // compare scalar doubles
    std::vector<double> computed_doubles;
    mlv.get_all_doubles(computed_doubles);
    authority.dense_vector_operations->scale(scale_factor, doubles);
    expect_equal_float_vectors(computed_doubles, doubles);

    // compare double vectors
    const int final_num_double_vectors = mlv.get_double_vectors_size();
    ASSERT_EQ(final_num_double_vectors, num_double_vectors);
    for(int d = 0; d < final_num_double_vectors; d++)
    {
        std::vector<double>* computed_vector = mlv.get_double_vector(d);
        authority.dense_vector_operations->scale(scale_factor, vector_original[d]);
        expect_equal_float_vectors(*computed_vector, vector_original[d]);
    }

    // compare matrices
    const int final_num_matrices = mlv.get_matrices_size();
    ASSERT_EQ(final_num_matrices, num_matrices);
    for(int m = 0; m < final_num_matrices; m++)
    {
        AbstractInterface::DenseMatrix* computed_matrix = mlv.get_matrix(m);
        AbstractInterface::DenseMatrix* gold_matrix = authority.dense_builder->build_by_row_major(matrix_row_sizes[m],
                                                                                 matrix_column_sizes[m],
                                                                                 matrix_original[m]);
        gold_matrix->scale(scale_factor);
        expect_equal_matrix(computed_matrix, gold_matrix);
        delete gold_matrix;
    }

    // cleanup memory
    delete m2;
}

PSL_TEST(MultiLayoutVector, randDoubles)
{
    set_rand_seed();
    // allocate
    AbstractAuthority authority;
    MultiLayoutVector mlv(&authority);
    mlv.resize_ints(3);
    const int num_doubles = 5;
    mlv.resize_doubles(num_doubles);
    const int num_double_vectors = 3;
    mlv.resize_double_vectors(num_double_vectors);
    const int num_matrices = 2;
    mlv.resize_matrices(num_matrices);

    // fill scalars
    mlv.set_double(0, 1.);
    mlv.set_double(1, .2);
    mlv.set_double(2, -13.1);
    mlv.set_double(3, 5.);
    mlv.set_double(4, 7.);

    // fill vectors
    std::vector<std::vector<double> > vector_original = { {5., -3., 2.}, {}, {-2., 4.2}};
    mlv.set_double_vector(0, &vector_original[0]);
    mlv.set_double_vector(1, &vector_original[1]);
    mlv.set_double_vector(2, &vector_original[2]);

    // fill matrices
    const std::vector<size_t> matrix_row_sizes = {2u, 1u};
    const std::vector<size_t> matrix_column_sizes = {3u, 5u};
    std::vector<std::vector<double> > matrix_original = { {1., 2., 3., 4.1, -9., 8.2}, {4.1, -9., 8.2, 1., 2.}};
    AbstractInterface::DenseMatrix* m0 = authority.dense_builder->build_by_row_major(matrix_row_sizes[0],
                                                                    matrix_column_sizes[0],
                                                                    matrix_original[0]);
    mlv.set_matrix(0, m0);
    AbstractInterface::DenseMatrix* m1 = authority.dense_builder->build_by_row_major(matrix_row_sizes[1],
                                                                    matrix_column_sizes[1],
                                                                    matrix_original[1]);
    mlv.set_matrix(1, m1);

    // fill
    const double lower = 151.12;
    const double upper = 153.415;
    mlv.random_double(lower, upper);

    // check scalars
    const int final_num_doubles = mlv.get_doubles_size();
    EXPECT_EQ(final_num_doubles, num_doubles);
    for(int d = 0; d < final_num_doubles; d++)
    {
        EXPECT_GT(mlv.get_double(d), lower);
        EXPECT_GT(upper, mlv.get_double(d));
    }

    // check vectors
    const int final_num_double_vectors = mlv.get_double_vectors_size();
    EXPECT_EQ(final_num_double_vectors, num_double_vectors);
    for(int vd = 0; vd < final_num_double_vectors; vd++)
    {
        std::vector<double>* this_double_vector = mlv.get_double_vector(vd);

        // consistent length
        const int length = this_double_vector->size();
        EXPECT_EQ(length, int(vector_original[vd].size()));

        // enforce expectations
        for(int i = 0; i < length; i++)
        {
            EXPECT_GT((*this_double_vector)[i], lower);
            EXPECT_GT(upper, (*this_double_vector)[i]);
        }
    }

    // check matrix
    const int final_num_matrices = mlv.get_matrices_size();
    EXPECT_EQ(final_num_matrices, num_matrices);
    for(int m = 0; m < final_num_matrices; m++)
    {
        AbstractInterface::DenseMatrix* this_matrix = mlv.get_matrix(m);

        // consistent sizes
        const int num_rows = this_matrix->get_num_rows();
        EXPECT_EQ(num_rows, int(matrix_row_sizes[m]));
        const int num_columns = this_matrix->get_num_columns();
        EXPECT_EQ(num_columns, int(matrix_column_sizes[m]));

        // enforce expectations
        for(int r = 0; r < num_rows; r++)
        {
            for(int c = 0; c < num_columns; c++)
            {
                EXPECT_GT(this_matrix->get_value(r,c), lower);
                EXPECT_GT(upper, this_matrix->get_value(r,c));
            }
        }
    }

    // cleanup memory
    delete m0;
    delete m1;
}


void test_multiLayoutVector_writeAndRead(const bool& prefer_binary)
{
    // allocate
    AbstractAuthority authority;
    MultiLayoutVector mlv(&authority);

    // fill integer scalars
    const std::vector<int> expected_ints = {0, -1, 2, 571258, 7, -7, 5};
    const int num_ints = expected_ints.size();
    mlv.resize_ints(num_ints);
    for(int i = 0; i < num_ints; i++)
    {
        mlv.set_int(i, expected_ints[i]);
    }

    // fill double scalars
    const std::vector<double> expected_doubles = {1., 2., 0., 5., 3., 7., -13.1, -.2, -42., 58.9, 17.1};
    const int num_doubles = expected_doubles.size();
    mlv.resize_doubles(num_doubles);
    for(int i = 0; i < num_doubles; i++)
    {
        mlv.set_double(i, expected_doubles[i]);
    }

    // fill integer vectors
    const int num_int_vectors = 5;
    mlv.resize_int_vectors(num_int_vectors);
    const std::vector<int> iv0 = {3, -1, 2};
    mlv.transfer_int_vector(0, iv0);
    std::vector<int> iv1 = {};
    mlv.set_int_vector(1, &iv1);
    std::vector<int> iv2 = {5, 2, 7};
    mlv.set_int_vector(2, &iv2);
    std::vector<int> iv3 = {4, 1, 9, 11};
    mlv.set_int_vector(3, &iv3);
    const std::vector<int> iv4 = {0, 7};
    mlv.transfer_int_vector(4, iv4);

    // fill double vectors
    const int num_double_vectors = 2;
    mlv.resize_double_vectors(num_double_vectors);
    std::vector<double> dv0 = {5., -3., 2.};
    mlv.set_double_vector(0, &dv0);
    const std::vector<double> dv1 = {-2., 4.2};
    mlv.transfer_double_vector(1, dv1);

    // fill matrices
    const int num_matrices = 3;
    mlv.resize_matrices(num_matrices);
    const std::vector<double> m0v = {1., 2., 3., 4.1, -9., 8.2};
    AbstractInterface::DenseMatrix* m0 = authority.dense_builder->build_by_row_major(2u, 3u, m0v);
    mlv.set_matrix(0, m0);
    const std::vector<double> m1v = {-5.1, 2.7, 3., 4.2, 0., 7.2, 2.1, 5.7};
    AbstractInterface::DenseMatrix* m1 = authority.dense_builder->build_by_row_major(4u, 2u, m1v);
    mlv.transfer_matrix(1, m1);
    const std::vector<double> m2v = {4.1, -9., 8.2, 1., 2., 3.};
    AbstractInterface::DenseMatrix* m2 = authority.dense_builder->build_by_row_major(1u, 6u, m2v);
    mlv.set_matrix(2, m2);

    // save mlv
    const std::string mlv_filename = "_mlv_example.psl";
    mlv.save_to_file(mlv_filename, prefer_binary);

    // load mlv
    DataSequence* loaded_mlv = load_data_sequence(mlv_filename, &authority);

    // for each mlv
    std::vector<DataSequence*> mlvs = {&mlv, loaded_mlv};
    const int num_mlvs = mlvs.size();
    for(int mlv_index = 0; mlv_index < num_mlvs; mlv_index++)
    {
        DataSequence* this_mlv = mlvs[mlv_index];

        // ensure accuracy of integer scalars
        EXPECT_EQ(num_ints, this_mlv->get_ints_size());
        std::vector<int> computed_all_ints;
        this_mlv->get_all_ints(computed_all_ints);
        expect_equal_vectors(computed_all_ints, expected_ints);
        for(int i = 0; i < num_ints; i++)
        {
            EXPECT_EQ(expected_ints[i], *this_mlv->get_int_ptr(i));
        }

        // ensure accuracy of double scalars
        EXPECT_EQ(num_doubles, this_mlv->get_doubles_size());
        std::vector<double> computed_all_doubles;
        this_mlv->get_all_doubles(computed_all_doubles);
        expect_equal_float_vectors(computed_all_doubles, expected_doubles);
        for(int i = 0; i < num_doubles; i++)
        {
            EXPECT_EQ(expected_doubles[i], *this_mlv->get_double_ptr(i));
        }

        // ensure accuracy of integer vectors
        EXPECT_EQ(num_int_vectors, this_mlv->get_int_vectors_size());
        expect_equal_vectors(*this_mlv->get_int_vector(0), iv0);
        expect_equal_vectors(*this_mlv->get_int_vector(1), iv1);
        expect_equal_vectors(*this_mlv->get_int_vector(2), iv2);
        expect_equal_vectors(*this_mlv->get_int_vector(3), iv3);
        expect_equal_vectors(*this_mlv->get_int_vector(4), iv4);

        // ensure accuracy of double vectors
        EXPECT_EQ(num_double_vectors, this_mlv->get_double_vectors_size());
        expect_equal_float_vectors(*this_mlv->get_double_vector(0), dv0);
        expect_equal_float_vectors(*this_mlv->get_double_vector(1), dv1);

        // ensure accuracy of double vectors
        EXPECT_EQ(num_matrices, this_mlv->get_matrices_size());
        expect_equal_matrix(m0, this_mlv->get_matrix(0));
        expect_equal_matrix(m1, this_mlv->get_matrix(1));
        expect_equal_matrix(m2, this_mlv->get_matrix(2));
    }

    // cleanup memory
    mlvs.clear();
    delete loaded_mlv;
    delete m0;
    delete m2;

    // revisit saved file
    InputOutputManager io_manager(&authority);
    io_manager.set_filename(mlv_filename);

    // expect non-empty
    EXPECT_EQ(io_manager.is_empty(), false);

    // expect got binary format if preferred
    EXPECT_EQ(io_manager.is_binary(), prefer_binary);

    // cleanup file
    io_manager.delete_file();
}

PSL_TEST(MultiLayoutVector, writeAndRead_binary)
{
    set_rand_seed();
    test_multiLayoutVector_writeAndRead(true);
}
PSL_TEST(MultiLayoutVector, writeAndRead_plainText)
{
    set_rand_seed();
    test_multiLayoutVector_writeAndRead(false);
}

}
}
