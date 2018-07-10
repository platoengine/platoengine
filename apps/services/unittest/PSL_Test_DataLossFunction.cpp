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

#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_DataLossFunction.hpp"
#include "PSL_DataLossFunctionFactory.hpp"
#include "PSL_Random.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace TestingDataLossFunction
{

PSL_TEST(DataLossFunction, squared_error_loss)
{
    set_rand_seed();
    example::Interface_BasicGlobalUtilities utilities;
    DataLossFunction* dlf = build_data_loss_function(data_loss_function_t::squared_error_loss, &utilities);

    // compute
    const std::vector<double> computed_input = {.1, .9, .8, -1., 1.2};
    const std::vector<double> true_input = {0., 0., 1., 0., 0.};
    std::vector<double> computed_gradient = {-3., -2.}; // intentionally superfluous
    const double computed_loss = dlf->compute_loss(computed_input, true_input, computed_gradient);

    // compare to expectations
    const double expected_loss = 1.65;
    EXPECT_FLOAT_EQ(expected_loss, computed_loss);
    const std::vector<double> expected_gradient = {.1, .9, -.2, -1., 1.2};
    expect_equal_float_vectors(computed_gradient, expected_gradient);

    delete dlf;
}

PSL_TEST(DataLossFunction, cross_entropy_loss)
{
    set_rand_seed();
    example::Interface_BasicGlobalUtilities utilities;
    DataLossFunction* dlf = build_data_loss_function(data_loss_function_t::cross_entropy_loss, &utilities);

    // compute
    const std::vector<double> computed_input = {.1, .9, .8, -1., 1.2};
    const std::vector<double> true_input = {0., 0., 1., 0., 0.};
    std::vector<double> computed_gradient = {-3., -2., -1., 0., 1., 2., 3.}; // intentionally superfluous
    const double computed_loss = dlf->compute_loss(computed_input, true_input, computed_gradient);

    // compare to expectations
    const double expected_loss = 1.449006169770361;
    EXPECT_FLOAT_EQ(expected_loss, computed_loss);
    const std::vector<double> expected_gradient = {.116599980795443, .259498029521695, -.765196472982170, .038812761966554,
                                                   .350285700698478};
    expect_equal_float_vectors(computed_gradient, expected_gradient);

    delete dlf;
}

}
}
