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

#include "PSL_Point.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Random.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace TestingPoint
{

PSL_TEST(Point,allocation)
{
    set_rand_seed();
    // allocate
    Point p(0u, {0.0,uniform_rand_double(),1.0});

    // test
    EXPECT_EQ(p.dimension(), 3u);
    EXPECT_EQ(p(0), 0.0);
    EXPECT_EQ(p(2), 1.0);

    // mutate
    p.set(0u, {0.1,-0.1});

    // test
    EXPECT_EQ(p.dimension(), 2u);
    EXPECT_EQ(p(0), 0.1);
    EXPECT_EQ(p(1), -0.1);
}

PSL_TEST(Point,charSet)
{
    set_rand_seed();

    // allocate
    Point p;

    // define point
    std::vector<float> xyz_floats({0.251437, -42.1375, 0.00234571});

    // make char's
    char xyz[12];
    for (int offset=0;offset<3;offset++) {
        char* c = reinterpret_cast<char*>(&xyz_floats[offset]);
        for (int offset2=0;offset2<4;offset2++) {
            xyz[offset2+4*offset] = c[offset2];
        }
    }

    // set
    p.set(0u, xyz);

    // test
    EXPECT_EQ(p(0),xyz_floats[0]);
    EXPECT_EQ(p(1),xyz_floats[1]);
    EXPECT_EQ(p(2),xyz_floats[2]);
}

}
}
