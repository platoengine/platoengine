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

#include "PSL_BoundarySupportPoint.hpp"

#include <vector>
#include <cmath>
#include <iostream>

namespace PlatoSubproblemLibrary
{
namespace TestingPoint
{

PSL_TEST(BoundarySupportPoint,constructorAndGetters)
{
    EXPECT_NO_THROW(BoundarySupportPoint(3, {0,1}));

    BoundarySupportPoint tBoundarySupportPoint(3, {0,1});

    std::set<int> tTempSet = {0,1};

    EXPECT_EQ(tBoundarySupportPoint.getSupportedNodeIndex(), 3);
    EXPECT_EQ(tBoundarySupportPoint.getSupportingNodeIndices(), tTempSet);
    EXPECT_EQ(tBoundarySupportPoint.getIndexOnEdge(), 0);

    tBoundarySupportPoint = BoundarySupportPoint(3, {0,1}, {1.0/3.0,1.0/3.0}, 1);

    std::vector<double> tTempVector = {1.0/3.0,1.0/3.0};

    EXPECT_EQ(tBoundarySupportPoint.getSupportedNodeIndex(), 3);
    EXPECT_EQ(tBoundarySupportPoint.getSupportingNodeIndices(), tTempSet);
    EXPECT_EQ(tBoundarySupportPoint.getCoefficients(), tTempVector);
    EXPECT_EQ(tBoundarySupportPoint.getIndexOnEdge(), 1);

    EXPECT_THROW(BoundarySupportPoint(3, {1,1}, {0.5,0.5}), std::domain_error);
    EXPECT_THROW(BoundarySupportPoint(3, std::vector<int>({}), std::vector<double>({})), std::domain_error);
    EXPECT_THROW(BoundarySupportPoint(3, {1,2}, std::vector<double>({1.0})), std::domain_error);
    EXPECT_THROW(BoundarySupportPoint(3, {1,2,3}, {1.0/3.0,1.0/3.0,1.0/3.0}), std::domain_error);
    EXPECT_THROW(BoundarySupportPoint(3, {1,2}, {0.5,0.5}, 2), std::domain_error);
    EXPECT_THROW(BoundarySupportPoint(3, {1}, {1.0}, 1), std::domain_error);
    EXPECT_THROW(BoundarySupportPoint(3, {3}, std::vector<double>({1.0})), std::domain_error);

    EXPECT_THROW(BoundarySupportPoint(3, {1,1}), std::domain_error);
    EXPECT_THROW(BoundarySupportPoint(3, std::vector<int>({})), std::domain_error);
    EXPECT_THROW(BoundarySupportPoint(3, {1,2,3}), std::domain_error);
    EXPECT_THROW(BoundarySupportPoint(3, {1}, 1), std::domain_error);
    EXPECT_THROW(BoundarySupportPoint(3, {3}), std::domain_error);
}

PSL_TEST(BoundarySupportPoint,setAndGetCoefficients)
{
    BoundarySupportPoint tBoundarySupportPoint(3, {0,1});

    EXPECT_THROW(tBoundarySupportPoint.getCoefficients(),std::runtime_error);

    tBoundarySupportPoint.setCoefficients({0.5,0.5});

    EXPECT_EQ(tBoundarySupportPoint.getCoefficients(), std::vector<double>({0.5,0.5}));
}

PSL_TEST(BoundarySupportPoint,getCorrespondingCoefficient)
{
    BoundarySupportPoint tBoundarySupportPoint(3, {6,2}, {0.21,0.79});

    EXPECT_EQ(tBoundarySupportPoint.getCorrespondingCoefficient(6), 0.21);
    EXPECT_EQ(tBoundarySupportPoint.getCorrespondingCoefficient(2), 0.79);
    EXPECT_THROW(tBoundarySupportPoint.getCorrespondingCoefficient(4), std::domain_error);

    tBoundarySupportPoint = BoundarySupportPoint(3, {6,2});
    EXPECT_THROW(tBoundarySupportPoint.getCorrespondingCoefficient(6), std::runtime_error);
}

PSL_TEST(BoundarySupportPoint,operatorLessThan)
{
    BoundarySupportPoint tBoundarySupportPoint1(3, {0,1}, {0.21,0.79});
    BoundarySupportPoint tBoundarySupportPoint2(3, {0,2}, {0.21,0.79});
    BoundarySupportPoint tBoundarySupportPoint3(2, {0,1}, {0.21,0.79});
    BoundarySupportPoint tBoundarySupportPoint4(2, {0,1}, {0.5,0.5}, 1);

    EXPECT_LT(tBoundarySupportPoint1,tBoundarySupportPoint2);
    EXPECT_LT(tBoundarySupportPoint3,tBoundarySupportPoint1);
    EXPECT_LT(tBoundarySupportPoint3,tBoundarySupportPoint4);
}

PSL_TEST(BoundarySupportPoint,operatorEqualTo)
{
    BoundarySupportPoint tBoundarySupportPoint1(3, {0,1}, {0.21,0.79});
    BoundarySupportPoint tBoundarySupportPoint2(3, {0,1}, {0.21,0.79});
    BoundarySupportPoint tBoundarySupportPoint3(3, {0,1}, {0.5,0.5});

    BoundarySupportPoint tBoundarySupportPoint4(3, {0,1}, {0.5,0.5}, 1);

    EXPECT_EQ(tBoundarySupportPoint1,tBoundarySupportPoint2);
    EXPECT_EQ(tBoundarySupportPoint1,tBoundarySupportPoint3);
    EXPECT_EQ(tBoundarySupportPoint1 == tBoundarySupportPoint4, false);
    EXPECT_NE(tBoundarySupportPoint1, tBoundarySupportPoint4);
}

PSL_TEST(BoundarySupportPoint,operatorOutput)
{
    BoundarySupportPoint tBoundarySupportPoint1(3, {0,1}, {0.21,0.79});
    std::stringstream tStringStream;
    tStringStream << tBoundarySupportPoint1;
    std::string tTestString = tStringStream.str();
    std::string tGoldString = "{3, {0,1}}";
    EXPECT_EQ(tTestString, tGoldString);
}

}
}
