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

#include "PSL_ActivationFunctionFactory.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_ActivationFunction.hpp"
#include "PSL_Random.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

#define PSL_TestingActivationFunction_Allocate \
        example::Interface_BasicGlobalUtilities utilities; \
        const std::vector<double> inputs = {-1.5, -1., -.75, 0., .15, 1., 1.5};

namespace TestingActivationFunction
{

PSL_TEST(ActivationFunction, sigmoid)
{
    set_rand_seed();
    PSL_TestingActivationFunction_Allocate
    ActivationFunction* af = build_activation_function(activation_function_t::sigmoid_activation, 0., &utilities);

    // compute outputs
    std::vector<double> computed_output;
    std::vector<double> computed_output_gradients;
    af->activate(inputs, computed_output, computed_output_gradients);

    // compare to gold
    const std::vector<double> gold_outputs = {.182425523806356, .268941421369995, .320821300824607, .5, .537429845343750,
                                              .731058578630005, .817574476193644};
    expect_equal_float_vectors(gold_outputs, computed_output);
    const std::vector<double> gold_output_gradients = {.149146452070333, .196611933241482, .217894993761814, .25,
                                                       .248599006677543, .196611933241482, .149146452070333};
    expect_equal_float_vectors(gold_output_gradients, computed_output_gradients);

    delete af;
}

PSL_TEST(ActivationFunction, rectified_linear_unit)
{
    set_rand_seed();
    PSL_TestingActivationFunction_Allocate
    ActivationFunction* af = build_activation_function(activation_function_t::rectified_linear_unit, 0., &utilities);

    // compute outputs
    std::vector<double> computed_output;
    std::vector<double> computed_output_gradients;
    af->activate(inputs, computed_output, computed_output_gradients);

    // compare to gold
    const std::vector<double> gold_outputs = {0., 0., 0., 0., 0.15, 1., 1.5};
    expect_equal_float_vectors(gold_outputs, computed_output);
    const std::vector<double> gold_output_gradients = {0., 0., 0., 1., 1., 1., 1.};
    expect_equal_float_vectors(gold_output_gradients, computed_output_gradients);

    delete af;
}

PSL_TEST(ActivationFunction, leaky_rectified_linear_unit)
{
    set_rand_seed();
    PSL_TestingActivationFunction_Allocate
    ActivationFunction* af = build_activation_function(activation_function_t::leaky_rectified_linear_unit, .001, &utilities);

    // compute outputs
    std::vector<double> computed_output;
    std::vector<double> computed_output_gradients;
    af->activate(inputs, computed_output, computed_output_gradients);

    // compare to gold
    const std::vector<double> gold_outputs = {-.0015, -.001, -.00075, 0., .15, 1., 1.5};
    expect_equal_float_vectors(gold_outputs, computed_output);
    const std::vector<double> gold_output_gradients = {.001, .001, .001, 1., 1., 1., 1.};
    expect_equal_float_vectors(gold_output_gradients, computed_output_gradients);

    delete af;
}

PSL_TEST(ActivationFunction, soft_plus)
{
    set_rand_seed();
    PSL_TestingActivationFunction_Allocate
    ActivationFunction* af = build_activation_function(activation_function_t::soft_plus_activation, 0., &utilities);

    // compute outputs
    std::vector<double> computed_output;
    std::vector<double> computed_output_gradients;
    af->activate(inputs, computed_output, computed_output_gradients);

    // compare to gold
    const std::vector<double> gold_outputs = {0.201413277982752, 0.313261687518223, 0.386871006114900, 0.693147180559945,
                                              0.770957047789532, 1.313261687518223, 1.701413277982752};
    expect_equal_float_vectors(gold_outputs, computed_output);
    const std::vector<double> gold_output_gradients = {0.182425523806356, 0.268941421369995, 0.320821300824607, 0.5,
                                                       0.537429845343750, 0.731058578630005, 0.817574476193644};
    expect_equal_float_vectors(gold_output_gradients, computed_output_gradients);

    delete af;
}

}
}
