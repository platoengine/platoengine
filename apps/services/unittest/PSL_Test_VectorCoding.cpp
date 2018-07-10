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

#include "PSL_VectorCoding.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Random.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace TestingVectorCoding
{

void test_vectorCoding_sideBySide(const std::vector<double>& expected_input_scalars,
                                  const std::vector<int>& expected_input_enums,
                                  const int& expected_output_enum,
                                  const std::vector<double>& expected_encoded_row)
{
    // encode
    std::vector<double> computed_encoded_row;
    sidebyside_encode(expected_input_scalars, expected_input_enums, expected_output_enum, computed_encoded_row);

    // compare to expectations
    expect_equal_float_vectors(computed_encoded_row, expected_encoded_row);

    // decode
    std::vector<double> computed_input_scalars = {.1, -.2};
    std::vector<int> computed_input_enums;
    int computed_output_enum = -126;
    const int num_input_enums = expected_input_enums.size();
    sidebyside_decode(expected_encoded_row, num_input_enums, computed_input_scalars, computed_input_enums, computed_output_enum);

    // compare to expectations
    expect_equal_float_vectors(computed_input_scalars, expected_input_scalars);
    expect_equal_vectors(computed_input_enums, expected_input_enums);
    EXPECT_EQ(computed_output_enum, expected_output_enum);
}

PSL_TEST(VectorCoding, sideBySide_basic)
{
    set_rand_seed();
    const std::vector<double> expected_input_scalars = {.1, .5, -.4, 2.71};
    const std::vector<int> expected_input_enums = {1, 0, 3};
    const int expected_output_enum = 42;
    const std::vector<double> expected_encoded_row = {.1, .5, -.4, 2.71, 1., 0., 3., 42.};

    test_vectorCoding_sideBySide(expected_input_scalars, expected_input_enums, expected_output_enum, expected_encoded_row);
}

PSL_TEST(VectorCoding, sideBySide_noInputEnum)
{
    set_rand_seed();
    const std::vector<double> expected_input_scalars = {-2.71, .1, .0};
    const std::vector<int> expected_input_enums = {};
    const int expected_output_enum = 13;
    const std::vector<double> expected_encoded_row = {-2.71, .1, .0, 13.};

    test_vectorCoding_sideBySide(expected_input_scalars, expected_input_enums, expected_output_enum, expected_encoded_row);
}

PSL_TEST(VectorCoding, decodeDataset)
{
    set_rand_seed();
    example::Interface_BasicGlobalUtilities utilities;
    example::Interface_DenseMatrixBuilder e_builder(&utilities);
    AbstractInterface::DenseMatrixBuilder* builder = &e_builder;

    // define dataset
    size_t num_rows = 8;
    size_t num_columns = 5;
    std::vector<double> matrix_values = {0., 9., 1., 0., 0.,
                                         1., 4., 0., 1., 0.,
                                         2., 3., 0., 0., 1.,
                                         3., 2., 1., 0., 0.,
                                         4., 7., 0., 0., 1.,
                                         5., 1., 0., 1., 0.,
                                         6., 5., 0., 0., 1.,
                                         7., 6., 1., 0., 0.};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(num_rows, num_columns, matrix_values);

    // decode
    const int output_enum_size = 3;
    std::vector<std::vector<int> > computed_rows_of_each_output_enum;
    onehot_decode_all_rows(matrix, output_enum_size, computed_rows_of_each_output_enum);

    // compare to expectations
    std::vector<std::vector<int> > expected_rows_of_each_output_enum = { {0, 3, 7}, {1, 5}, {2, 4, 6}};
    expect_equal_vectors(computed_rows_of_each_output_enum, expected_rows_of_each_output_enum);

    // cleanup
    safe_free(matrix);
}

PSL_TEST(VectorCoding, simple)
{
    set_rand_seed();
    // pose problem
    std::vector<double> gold_input_scalar = {3., -1.2, 2.1};
    std::vector<int> gold_input_enum = {0, 2};
    std::vector<int> input_enum_sizes = {2, 3};
    int gold_output_enum = 1;
    int output_enum_size = 4;
    std::vector<double> computed_onehot_row = {3.14, 2.71}; // intentional garbage

    // encode
    onehot_encode(gold_input_scalar, gold_input_enum, input_enum_sizes, gold_output_enum, output_enum_size, computed_onehot_row);

    // compare to gold
    std::vector<double> gold_onehot_row = {3., -1.2, 2.1, 1., 0., 0., 0., 1., 0., 1., 0., 0.};
    const size_t gold_num_input = 8u;
    const size_t gold_num_output = output_enum_size;
    expect_equal_float_vectors(gold_onehot_row, computed_onehot_row);

    // encode
    std::vector<double> computed_onehot_input_row = {3.14}; // intentional garbage
    onehot_encode(gold_input_scalar, gold_input_enum, input_enum_sizes, computed_onehot_input_row);

    // compare to gold
    std::vector<double> gold_onehot_input_row(gold_onehot_row.begin(), gold_onehot_row.end() - output_enum_size);
    expect_equal_float_vectors(gold_onehot_input_row, computed_onehot_input_row);

    // decode
    std::vector<double> split_computed_all_input;
    std::vector<double> split_computed_all_output;
    onehot_split(gold_onehot_row, output_enum_size, split_computed_all_input, split_computed_all_output);

    // compare to gold
    expect_equal_float_vectors(gold_onehot_input_row, split_computed_all_input);
    EXPECT_EQ(split_computed_all_input.size(), gold_num_input);
    std::vector<double> gold_onehot_output_row(gold_onehot_row.end() - output_enum_size, gold_onehot_row.end());
    expect_equal_float_vectors(gold_onehot_output_row, split_computed_all_output);
    EXPECT_EQ(split_computed_all_output.size(), gold_num_output);

    // allocate for decode
    std::vector<double> computed_input_scalar = {};
    std::vector<int> computed_input_enum = {5}; // intentional garbage
    int computed_output_enum = -2; // intentional garbage

    // decode
    onehot_decode(gold_onehot_row,
                  input_enum_sizes,
                  output_enum_size,
                  computed_input_scalar,
                  computed_input_enum,
                  computed_output_enum);

    // compare to gold
    expect_equal_float_vectors(gold_input_scalar, computed_input_scalar);
    expect_equal_vectors(gold_input_enum, computed_input_enum);
    EXPECT_EQ(gold_output_enum, computed_output_enum);

    // allocate for decode
    std::vector<double> computed_all_input;
    computed_output_enum = 72; // intentional garbage

    // decode
    onehot_decode(gold_onehot_row, output_enum_size, computed_all_input);
    onehot_decode(gold_onehot_row, output_enum_size, computed_output_enum);

    // compare to gold
    std::vector<double> gold_all_input(gold_onehot_row.begin(), gold_onehot_row.end() - output_enum_size);
    expect_equal_float_vectors(gold_all_input, computed_all_input);
    EXPECT_EQ(gold_output_enum, computed_output_enum);
}

PSL_TEST(VectorCoding, noInputEnums)
{
    set_rand_seed();
    // pose problem
    std::vector<double> gold_input_scalar = {-1.2, 5.7};
    std::vector<int> gold_input_enum = {};
    std::vector<int> input_enum_sizes = {};
    int gold_output_enum = 0;
    int output_enum_size = 2;
    std::vector<double> computed_onehot_row = {3.14, 2.71}; // intentional garbage

    // encode
    onehot_encode(gold_input_scalar, gold_input_enum, input_enum_sizes, gold_output_enum, output_enum_size, computed_onehot_row);

    // compare to gold
    std::vector<double> gold_onehot_row = {-1.2, 5.7, 1., 0.};
    expect_equal_float_vectors(gold_onehot_row, computed_onehot_row);

    // allocate for decode
    std::vector<double> computed_input_scalar = {};
    std::vector<int> computed_input_enum = {5}; // intentional garbage
    int computed_output_enum = -2; // intentional garbage

    // decode
    onehot_decode(gold_onehot_row,
                  input_enum_sizes,
                  output_enum_size,
                  computed_input_scalar,
                  computed_input_enum,
                  computed_output_enum);

    // compare to gold
    expect_equal_float_vectors(gold_input_scalar, computed_input_scalar);
    expect_equal_vectors(gold_input_enum, computed_input_enum);
    EXPECT_EQ(gold_output_enum, computed_output_enum);

    // allocate for decode
    std::vector<double> computed_all_input;
    computed_output_enum = 72; // intentional garbage

    // decode
    onehot_decode(gold_onehot_row, output_enum_size, computed_all_input);
    onehot_decode(gold_onehot_row, output_enum_size, computed_output_enum);

    // compare to gold
    std::vector<double> gold_all_input(gold_onehot_row.begin(), gold_onehot_row.end() - output_enum_size);
    expect_equal_float_vectors(gold_all_input, computed_all_input);
    EXPECT_EQ(gold_output_enum, computed_output_enum);
}

}
}
