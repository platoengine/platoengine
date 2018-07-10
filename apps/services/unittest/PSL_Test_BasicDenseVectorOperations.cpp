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

#include <iostream> // for std::cout
#include <vector>
#include <cstddef> // for size_t

#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_Interface_BasicDenseVectorOperations.hpp"
#include "PSL_Random.hpp"

namespace PlatoSubproblemLibrary
{

#define TestingBasicDenseVectorOperations_AllocateUtilities \
    example::Interface_BasicDenseVectorOperations basic_operations; \
    AbstractInterface::DenseVectorOperations* operations = &basic_operations;

namespace TestingBasicDenseVectorOperations
{

PSL_TEST(BasicDenseVectorOperations,dot)
{
    set_rand_seed();
    TestingBasicDenseVectorOperations_AllocateUtilities

    std::vector<double> x = {-7.7, -2.3, 4.1, -7.9, 3.2};
    std::vector<double> y = {3.6, 9.7, 4.6, 5.4, 9.};

    const double dot_xy = operations->dot(x, y);
    EXPECT_FLOAT_EQ(dot_xy, -45.03);
    const double dot_yx = operations->dot(y, x);
    EXPECT_FLOAT_EQ(dot_yx, dot_xy);
    const double dot_yy = operations->dot(y, y);
    EXPECT_FLOAT_EQ(dot_yy, 238.37);
    const double dot_xx = operations->dot(x, x);
    EXPECT_FLOAT_EQ(dot_xx, 154.04);

    const double delta_xy = operations->delta_squared(x, y);
    EXPECT_FLOAT_EQ(delta_xy, 482.47);
    const double delta_yx = operations->delta_squared(y, x);
    EXPECT_FLOAT_EQ(delta_xy, delta_yx);
    const double delta_xx = operations->delta_squared(x, x);
    EXPECT_FLOAT_EQ(delta_xx, 0.);
    const double delta_yy = operations->delta_squared(y, y);
    EXPECT_FLOAT_EQ(delta_yy, 0.);
}

PSL_TEST(BasicDenseVectorOperations,axpy)
{
    set_rand_seed();
    TestingBasicDenseVectorOperations_AllocateUtilities

    const double alpha = -2.;
    std::vector<double> x = {1., 2., 3., 4.};
    std::vector<double> y = {-1., 0., 5., 2.};

    operations->axpy(alpha, x, y);

    std::vector<double> expected_y = {-3., -4., -1., -6.};
    expect_equal_float_vectors(expected_y, y);
}

PSL_TEST(BasicDenseVectorOperations,scale)
{
    set_rand_seed();
    TestingBasicDenseVectorOperations_AllocateUtilities

    const double alpha = 3.;
    std::vector<double> y = {-1., 0., 5., 2.};

    operations->scale(alpha, y);

    std::vector<double> expected_y = {-3., 0., 15., 6.};
    expect_equal_float_vectors(expected_y, y);
}

PSL_TEST(BasicDenseVectorOperations,multiply)
{
    set_rand_seed();
    TestingBasicDenseVectorOperations_AllocateUtilities

    const std::vector<double> x = {-1., 0., 5., 2., 3., -2.};
    const std::vector<double> y = {2., 5., -1., 0., 4., -3.};
    std::vector<double> z = {0., 2.}; // intentionally superfluous

    operations->multiply(x, y, z);
    std::vector<double> expected_z = {-2., 0., -5., 0., 12., 6.};
    expect_equal_float_vectors(expected_z, z);

    std::vector<double> y2 = y;
    operations->multiply(x, y2);
    expect_equal_float_vectors(expected_z, y2);
}

}

}
