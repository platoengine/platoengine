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

#include "PSL_VectorKernelFactory.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_VectorKernel.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Random.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace TestingVectorKernel
{

void test_vector_kernel_lengths(ParameterData* parameter_data,
                                const std::vector<int>& input_lengths,
                                const std::vector<int>& expected_output_lengths)
{
    // allocate
    example::Interface_BasicGlobalUtilities utilities;

    // build kernel
    VectorKernel* vk = build_vector_kernel(parameter_data->get_vector_kernel(),
                                           parameter_data->get_vector_kernel_parameter(),
                                           &utilities);

    // for each expectation
    const size_t num_inputs = input_lengths.size();
    for(size_t i = 0u; i < num_inputs; i++)
    {
        // computed length
        const int computed_output_length = vk->projection_length(input_lengths[i]);

        // compare length to expectations
        EXPECT_EQ(expected_output_lengths[i], computed_output_length);
    }

    safe_free(vk);
}

PSL_TEST(VectorKernel, pureProjectionLengths)
{
    set_rand_seed();
    // allocate
    ParameterData parameter_data;
    parameter_data.set_vector_kernel_parameter(4);
    parameter_data.set_vector_kernel(vector_kernel_t::vector_kernel_t::pure_kernel);
    const std::vector<int> input_lengths = {1, 2, 11, 50, 100};
    const std::vector<int> expected_output_lengths = {4, 8, 44, 200, 400};

    // solve
    test_vector_kernel_lengths(&parameter_data, input_lengths, expected_output_lengths);
}
PSL_TEST(VectorKernel, mixedProjectionLengths)
{
    set_rand_seed();
    // allocate
    ParameterData parameter_data;
    parameter_data.set_vector_kernel_parameter(3);
    parameter_data.set_vector_kernel(vector_kernel_t::vector_kernel_t::mixed_kernel);
    const std::vector<int> input_lengths = {1, 2, 11, 50, 100};
    const std::vector<int> expected_output_lengths = {3, 9, 363, 23425, 176850};

    // solve
    test_vector_kernel_lengths(&parameter_data, input_lengths, expected_output_lengths);
}
PSL_TEST(VectorKernel, crossProjectionLengths)
{
    set_rand_seed();
    // allocate
    ParameterData parameter_data;
    parameter_data.set_vector_kernel_parameter(3);
    parameter_data.set_vector_kernel(vector_kernel_t::vector_kernel_t::cross_kernel);
    const std::vector<int> input_lengths = {1, 2, 11, 50, 100};
    const std::vector<int> expected_output_lengths = {1, 3, 231, 20875, 166750};

    // solve
    test_vector_kernel_lengths(&parameter_data, input_lengths, expected_output_lengths);
}

PSL_TEST(VectorKernel, buildFromFactory)
{
    set_rand_seed();
    // allocate
    example::Interface_BasicGlobalUtilities utilities;
    const int vector_kernel_param = 2; // arbitrary

    // list types
    std::vector<vector_kernel_t::vector_kernel_t> types = {vector_kernel_t::vector_kernel_t::pure_kernel,
                                                           vector_kernel_t::vector_kernel_t::mixed_kernel,
                                                           vector_kernel_t::vector_kernel_t::cross_kernel};

    // for each type to try
    VectorKernel* vk = NULL;
    const int num_types = types.size();
    for(int t = 0; t < num_types; t++)
    {
        vk = build_vector_kernel(types[t], vector_kernel_param, &utilities);
        EXPECT_EQ(vk->get_type(), types[t]);
        safe_free(vk);
    }
}

void test_vector_kernel(ParameterData* parameter_data,
                        const std::vector<std::vector<double> >& inputs,
                        const std::vector<std::vector<double> >& expected_outputs)
{
    // allocate
    example::Interface_BasicGlobalUtilities utilities;

    // build kernel
    VectorKernel* vk = build_vector_kernel(parameter_data->get_vector_kernel(),
                                           parameter_data->get_vector_kernel_parameter(),
                                           &utilities);

    // for each expectation
    const size_t num_inputs = inputs.size();
    ASSERT_EQ(num_inputs, expected_outputs.size());
    for(size_t i = 0u; i < num_inputs; i++)
    {
        // expected length
        const int expected_output_length = vk->projection_length(inputs[i].size());

        std::vector<double> actual_output;
        vk->project(inputs[i], actual_output);

        // compare length to expectations
        const int actual_output_length = actual_output.size();
        EXPECT_EQ(expected_output_length, actual_output_length);

        // compare vector to expectations
        expect_equal_float_vectors(expected_outputs[i], actual_output);
    }

    safe_free(vk);
}

PSL_TEST(VectorKernel, pureLinear)
{
    set_rand_seed();
    // allocate
    ParameterData parameter_data;
    parameter_data.set_vector_kernel_parameter(1);
    parameter_data.set_vector_kernel(vector_kernel_t::vector_kernel_t::pure_kernel);

    // define expectations
    const std::vector<std::vector<double> > inputs = { {1., -2., 3.}, {-.5, .25, 4., -7., 0.}};
    const std::vector<std::vector<double> > expected_outputs = inputs;

    test_vector_kernel(&parameter_data, inputs, expected_outputs);
}
PSL_TEST(VectorKernel, pureQuadratic)
{
    set_rand_seed();
    // allocate
    ParameterData parameter_data;
    parameter_data.set_vector_kernel_parameter(2);
    parameter_data.set_vector_kernel(vector_kernel_t::vector_kernel_t::pure_kernel);

    // define expectations
    const std::vector<std::vector<double> > inputs = { {1., -2., 3.}, {-.5, .25, 4., -7., 0.}};
    const std::vector<std::vector<double> > expected_outputs = { {1., -2., 3., 1., 4., 9.}, {-.5, .25, 4., -7., 0., .25, .0625,
                                                                                             16., 49., 0.}};

    test_vector_kernel(&parameter_data, inputs, expected_outputs);
}
PSL_TEST(VectorKernel, pureCubic)
{
    set_rand_seed();
    // allocate
    ParameterData parameter_data;
    parameter_data.set_vector_kernel_parameter(3);
    parameter_data.set_vector_kernel(vector_kernel_t::vector_kernel_t::pure_kernel);

    // define expectations
    const std::vector<std::vector<double> > inputs = { {1., -2., 3.}, {-.5, .25, 4., -7., 0.}};
    const std::vector<std::vector<double> > expected_outputs = { {1., -2., 3., 1., 4., 9., 1., -8., 27.}, {-.5, .25, 4., -7., 0.,
                                                                                                           .25, .0625, 16., 49.,
                                                                                                           0., -.125, .015625,
                                                                                                           64., -343., 0.}};

    test_vector_kernel(&parameter_data, inputs, expected_outputs);
}

PSL_TEST(VectorKernel, mixedLinear)
{
    set_rand_seed();
    // allocate
    ParameterData parameter_data;
    parameter_data.set_vector_kernel_parameter(1);
    parameter_data.set_vector_kernel(vector_kernel_t::vector_kernel_t::mixed_kernel);

    // define expectations
    const std::vector<std::vector<double> > inputs = { {1., -2., 3.}, {-.5, .25, 4., -7., 0.}};
    const std::vector<std::vector<double> > expected_outputs = inputs;

    test_vector_kernel(&parameter_data, inputs, expected_outputs);
}
PSL_TEST(VectorKernel, mixedQuadratic)
{
    set_rand_seed();
    // allocate
    ParameterData parameter_data;
    parameter_data.set_vector_kernel_parameter(2);
    parameter_data.set_vector_kernel(vector_kernel_t::vector_kernel_t::mixed_kernel);

    // define expectations
    const std::vector<std::vector<double> > inputs = { {1., -2., 3.}, {-.5, .25, 4., -7., 0.}};
    const std::vector<std::vector<double> > expected_outputs = { {1., -2., 3., 1., -2., 3., 4., -6., 9.}, {-.5, .25, 4., -7., 0.,
                                                                                                           .25, -.125, -2., 3.5,
                                                                                                           0., .0625, 1., -1.75,
                                                                                                           0., 16., -28., 0., 49.,
                                                                                                           0., 0.}};
    test_vector_kernel(&parameter_data, inputs, expected_outputs);
}
PSL_TEST(VectorKernel, mixedCubic)
{
    set_rand_seed();
    // allocate
    ParameterData parameter_data;
    parameter_data.set_vector_kernel_parameter(3);
    parameter_data.set_vector_kernel(vector_kernel_t::vector_kernel_t::mixed_kernel);

    // define expectations
    const std::vector<std::vector<double> > inputs = { {1., -2., 3.}, {-.5, .25, 4., -7., 0.}};
    const std::vector<std::vector<double> > expected_outputs = { {1., -2., 3., 1., -2., 3., 4., -6., 9., 1., -2., 3., 4., -6., 9.,
                                                                  -8., 12., -18., 27.},
                                                                {-.5, .25, 4., -7., 0., .25, -.125, -2., 3.5, 0., .0625, 1.,
                                                                 -1.75, 0., 16., -28., 0., 49., 0., 0., -.125, .0625, 1., -1.75,
                                                                 .0, -.03125, -.5, .875, .0, -8., 14., .0, -24.5, 0., .0, .015625,
                                                                 .25, -.4375, .0, 4., -7., 0., 12.25, 0., 0., 64., -112., 0.,
                                                                 196., 0., 0., -343., 0., 0., 0.}};

    test_vector_kernel(&parameter_data, inputs, expected_outputs);
}

PSL_TEST(VectorKernel, crossLinear)
{
    set_rand_seed();
    // allocate
    ParameterData parameter_data;
    parameter_data.set_vector_kernel_parameter(1);
    parameter_data.set_vector_kernel(vector_kernel_t::vector_kernel_t::cross_kernel);

    // define expectations
    const std::vector<std::vector<double> > inputs = { {1., -2., 3.}, {-.5, .25, 4., -7., 0.}};
    const std::vector<std::vector<double> > expected_outputs = inputs;

    test_vector_kernel(&parameter_data, inputs, expected_outputs);
}
PSL_TEST(VectorKernel, crossQuadratic)
{
    set_rand_seed();
    // allocate
    ParameterData parameter_data;
    parameter_data.set_vector_kernel_parameter(2);
    parameter_data.set_vector_kernel(vector_kernel_t::vector_kernel_t::cross_kernel);

    // define expectations
    const std::vector<std::vector<double> > inputs = { {1., -2., 3.}, {-.5, .25, 4., -7., 0.}};
    const std::vector<std::vector<double> > expected_outputs = { {1., -2., 3., -2., 3., -6.},
    {-.5, .25, 4., -7., 0., -.125, -2., 3.5, 0., 1., -1.75, 0., -28., 0., 0.}};
    test_vector_kernel(&parameter_data, inputs, expected_outputs);
}
PSL_TEST(VectorKernel, crossCubic)
{
    set_rand_seed();
    // allocate
    ParameterData parameter_data;
    parameter_data.set_vector_kernel_parameter(3);
    parameter_data.set_vector_kernel(vector_kernel_t::vector_kernel_t::cross_kernel);

    // define expectations
    const std::vector<std::vector<double> > inputs = { {1., -2., 3.}, {-.5, .25, 4., -7., 0.}};
    const std::vector<std::vector<double> > expected_outputs = { {1., -2., 3., -2., 3., -6., -6.},
    {-.5, .25, 4., -7., 0., -.125, -2., 3.5, 0., 1., -1.75, 0., -28., 0., 0., -.5, .875, 0., 14., 0., 0., -7., 0., 0., 0.}};
    test_vector_kernel(&parameter_data, inputs, expected_outputs);
}

}
}
