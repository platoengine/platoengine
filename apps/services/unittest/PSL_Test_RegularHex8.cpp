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

#include "PSL_RegularHex8.hpp"
#include "PSL_Vector.hpp"

#include <vector>
#include <cmath>

namespace PlatoSubproblemLibrary
{
namespace TestingPoint
{

PSL_TEST(RegularHex8,construction)
{
    Vector tMinCoords({0.0,0.0,0.0});
    Vector tMaxCoords({1.0,1.0,1.0});

    EXPECT_NO_THROW(RegularHex8(tMinCoords,tMaxCoords));

    // min and max flipped
    EXPECT_THROW(RegularHex8(tMaxCoords,tMinCoords),std::domain_error);
}

PSL_TEST(RegularHex8,interpolateScalar)
{
    Vector tMinCoords({0.0,0.0,0.0});
    Vector tMaxCoords({1.0,1.0,1.0});

    RegularHex8 tHex(tMinCoords,tMaxCoords);

    Vector tPoint({0.0,0.0,0.0});

    // incorrect number of scalar values to interpolate
    std::vector<double> tBogusScalars({0.0,0.0});
    EXPECT_THROW(tHex.interpolateScalar(tPoint,tBogusScalars),std::domain_error);
    
    // point is not inside hex
    Vector tBogusPoint({-1.0,0,0});
    std::vector<double> tScalars({1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0});
    EXPECT_THROW(tHex.interpolateScalar(tBogusPoint,tScalars),std::domain_error);

    // check each corner
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),1.0);

    tPoint = Vector({1.0,0.0,0.0});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),2.0);

    tPoint = Vector({0.0,1.0,0.0});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),3.0);

    tPoint = Vector({1.0,1.0,0.0});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),4.0);

    tPoint = Vector({0.0,0.0,1.0});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),5.0);

    tPoint = Vector({1.0,0.0,1.0});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),6.0);

    tPoint = Vector({0.0,1.0,1.0});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),7.0);

    tPoint = Vector({1.0,1.0,1.0});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),8.0);

    // check midway on each edge
    tPoint = Vector({0.5,0,0});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),1.5);

    tPoint = Vector({0,0.5,0});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),2.0);

    tPoint = Vector({0,0,0.5});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),3.0);

    tPoint = Vector({1,0.5,0});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),3.0);

    tPoint = Vector({1,0,0.5});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),4.0);

    tPoint = Vector({0.5,1,0});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),3.5);

    tPoint = Vector({0,1,0.5});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),5.0);
    
    tPoint = Vector({0,0.5,1});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),6.0);

    tPoint = Vector({0.5,0,1});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),5.5);

    tPoint = Vector({0.5,1,1});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),7.5);

    tPoint = Vector({1,0.5,1});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),7.0);

    tPoint = Vector({1,1,0.5});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),6.0);

    // check midpoint of each face
    tPoint = Vector({0.5,0.5,0});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),2.5);

    tPoint = Vector({0.5,0,0.5});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),3.5);

    tPoint = Vector({0,0.5,0.5});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),4.0);

    tPoint = Vector({0.5,0.5,1});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),6.5);

    tPoint = Vector({0.5,1,0.5});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),5.5);

    tPoint = Vector({1,0.5,0.5});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),5.0);

    // check midpoint of hex
    tPoint = Vector({0.5,0.5,0.5});
    EXPECT_EQ(tHex.interpolateScalar(tPoint,tScalars),4.5);
}

}
}
