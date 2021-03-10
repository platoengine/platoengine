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
#include "PSL_OrthogonalGridUtilities.hpp"
#include "PSL_Vector.hpp"

#include <vector>
#include <cmath>

namespace PlatoSubproblemLibrary
{
namespace TestingPoint
{

PSL_TEST(OrthogonalGridUtilities, constructor)
{
    Vector tUBasisVector({1,0,0});
    Vector tVBasisVector({0,1,0});
    Vector tWBasisVector({0,0,1});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    double tTargetEdgeLength = 0.1;

    EXPECT_NO_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength));

    // Basis not orthogonal
    EXPECT_THROW(OrthogonalGridUtilities(tVBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength),std::domain_error);

    // Basis not positively oriented
    EXPECT_THROW(OrthogonalGridUtilities(tVBasisVector,tUBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength),std::domain_error);

    // Basis not unit length
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,2*tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength),std::domain_error);

    // Max not strictly greater than min
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMinUVWCoords,tMaxUVWCoords,tTargetEdgeLength),std::domain_error);
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMaxUVWCoords,tTargetEdgeLength),std::domain_error);

    // Target edge length non-positive 
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,-tTargetEdgeLength),std::domain_error);
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,0),std::domain_error);

    std::vector<int> aNumElementsInEachDirection({5,5,5});
    EXPECT_NO_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,aNumElementsInEachDirection));

    // Wrong size vector
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,std::vector<int>({5,5})),std::domain_error);

    // Non-positive number of elements
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,std::vector<int>({5,5,0})),std::domain_error);
}

PSL_TEST(OrthogonalGridUtilities, getGridDimensions)
{
    Vector tUBasisVector({1,0,0});
    Vector tVBasisVector({0,1,0});
    Vector tWBasisVector({0,0,1});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    double tTargetEdgeLength = 0.1;
    OrthogonalGridUtilities tUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    auto tDimensions = tUtilities.getGridDimensions();
    EXPECT_EQ(tDimensions, std::vector<int>({11,21,31}));

    tTargetEdgeLength = 0.11;
    OrthogonalGridUtilities tUtilities2(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    tDimensions = tUtilities2.getGridDimensions();
    EXPECT_EQ(tDimensions, std::vector<int>({10,19,28}));

    tTargetEdgeLength = 0.105;
    OrthogonalGridUtilities tUtilities3(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    tDimensions = tUtilities3.getGridDimensions();
    EXPECT_EQ(tDimensions, std::vector<int>({10,20,29}));

    // // target length larger than bounding box
    tTargetEdgeLength = 1.1;
    OrthogonalGridUtilities tUtilities4(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    tDimensions = tUtilities4.getGridDimensions();
    EXPECT_EQ(tDimensions, std::vector<int>({2,2,3}));

    std::vector<int> tNumElements({5,6,7});
    OrthogonalGridUtilities tUtilities5(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);
    tDimensions = tUtilities5.getGridDimensions();
    EXPECT_EQ(tDimensions, std::vector<int>({6,7,8}));
}

PSL_TEST(OrthogonalGridUtilities, computeGridXYZCoordinates)
{
    Vector tUBasisVector(std::vector<double>({1.0,0.0,0.0}));
    Vector tVBasisVector(std::vector<double>({0.0,1.0,0.0}));
    Vector tWBasisVector(std::vector<double>({0.0,0.0,1.0}));

    std::vector<int> tNumElements({10,20,30});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    OrthogonalGridUtilities tUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);

    std::vector<Vector> tCoordinates;

    tUtilities.computeGridXYZCoordinates(tCoordinates);
    std::vector<int> tIndex = {1,1,1};
    Vector tGridPointCoordinate = tCoordinates[tUtilities.getSerializedIndex(tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),0.1);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),0.1);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),0.1);

    tIndex = {1, 5, 12};
    tGridPointCoordinate = tCoordinates[tUtilities.getSerializedIndex(tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),0.1);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),0.5);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),1.2);

    tNumElements = {5,8,15};
    OrthogonalGridUtilities tUtilities2(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);
    tUtilities2.computeGridXYZCoordinates(tCoordinates);
    tIndex = {1, 1, 1};
    tGridPointCoordinate = tCoordinates[tUtilities2.getSerializedIndex(tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),0.2);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),0.25);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),0.2);
    
    tNumElements = {10,20,30};
    tMaxUVWCoords = Vector({1.0,2.0,3.0});
    tMinUVWCoords = Vector({-1.0,-2.0,-3.0});
    OrthogonalGridUtilities tUtilities3(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);
    tUtilities3.computeGridXYZCoordinates(tCoordinates);
    tIndex = {1, 1, 1};
    tGridPointCoordinate = tCoordinates[tUtilities3.getSerializedIndex(tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),-0.8);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),-1.8);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),-2.8);

    tUBasisVector = Vector({0.0,1.0,0.0});
    tVBasisVector = Vector({0.0,0.0,1.0});
    tWBasisVector = Vector({1.0,0.0,0.0});
    OrthogonalGridUtilities tUtilities4(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);
    tUtilities4.computeGridXYZCoordinates(tCoordinates);
    tGridPointCoordinate = tCoordinates[tUtilities4.getSerializedIndex(tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),-2.8);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),-0.8);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),-1.8);

    tUBasisVector = Vector({1.0,1.0,0.0});
    tVBasisVector = Vector({-1.0,1.0,0.0});
    tUBasisVector.normalize();
    tVBasisVector.normalize();
    tWBasisVector = Vector({0.0,0.0,1.0});
    tMaxUVWCoords = Vector({1.0,1.0,3.0});
    tMinUVWCoords = Vector({0.0,0.0,0.0});
    tNumElements = {10,10,10};
    OrthogonalGridUtilities tUtilities5(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);
    tUtilities5.computeGridXYZCoordinates(tCoordinates);
    tIndex = {0, 0, 1};
    tGridPointCoordinate = tCoordinates[tUtilities5.getSerializedIndex(tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),0.0);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),0.0);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),0.3);

    tIndex = {1, 0, 0};
    tGridPointCoordinate = tCoordinates[tUtilities5.getSerializedIndex(tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),0.1/sqrt(2));
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),0.1/sqrt(2));
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),0.0);

    tIndex = {1, 3, 2};
    tGridPointCoordinate = tCoordinates[tUtilities5.getSerializedIndex(tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),-0.2/sqrt(2));
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),0.4/sqrt(2));
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),0.6);
}

PSL_TEST(OrthogonalGridUtilities, getSerializedIndex)
{
    Vector tUBasisVector({1,0,0});
    Vector tVBasisVector({0,1,0});
    Vector tWBasisVector({0,0,1});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    std::vector<int> tNumElements = {2,2,2};
    OrthogonalGridUtilities tUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);

    // tIndex wrong dimension
    std::vector<int> tIndex = {0,0};
    EXPECT_THROW(tUtilities.getSerializedIndex(tIndex), std::domain_error);

    // tIndex out of range
    tIndex = {-1,0,0};
    EXPECT_THROW(tUtilities.getSerializedIndex(tIndex), std::out_of_range);

    // tIndex out of range
    tIndex = {0,4,0};
    EXPECT_THROW(tUtilities.getSerializedIndex(tIndex), std::out_of_range);

    tIndex = {0,0,0};
    int tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,0);

    tIndex = {1,0,0};
    tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,1);

    tIndex = {2,0,0};
    tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,2);
    
    tIndex = {0,1,0};
    tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,3);

    tIndex = {0,1,1};
    tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,12);

    tIndex = {1,1,1};
    tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,13);

    tIndex = {1,1,2};
    tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,22);
}

PSL_TEST(OrthogonalGridUtilities, getSupportIndices)
{
    Vector tUBasisVector({1,0,0});
    Vector tVBasisVector({0,1,0});
    Vector tWBasisVector({0,0,1});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    std::vector<int> tNumElements = {2,2,2};
    OrthogonalGridUtilities tUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);

    auto tDimensions = tUtilities.getGridDimensions();

    EXPECT_EQ(tUtilities.getSupportIndices(0,0,0).size(),0u);
    EXPECT_EQ(tUtilities.getSupportIndices(1,0,0).size(),0u);
    EXPECT_EQ(tUtilities.getSupportIndices(2,0,0).size(),0u);
    EXPECT_EQ(tUtilities.getSupportIndices(0,1,0).size(),0u);
    EXPECT_EQ(tUtilities.getSupportIndices(1,1,0).size(),0u);
    EXPECT_EQ(tUtilities.getSupportIndices(2,1,0).size(),0u);
    EXPECT_EQ(tUtilities.getSupportIndices(0,2,0).size(),0u);
    EXPECT_EQ(tUtilities.getSupportIndices(1,2,0).size(),0u);
    EXPECT_EQ(tUtilities.getSupportIndices(2,2,0).size(),0u);

    // a couple example support sets
    EXPECT_EQ(tUtilities.getSupportIndices(0,0,1).size(),3u);
    auto tSupportIndices = tUtilities.getSupportIndices(0,0,1);
    EXPECT_EQ(tSupportIndices[0],std::vector<int>({0,0,0}));
    EXPECT_EQ(tSupportIndices[1],std::vector<int>({0,1,0}));
    EXPECT_EQ(tSupportIndices[2],std::vector<int>({1,0,0}));

    EXPECT_EQ(tUtilities.getSupportIndices(1,0,1).size(),4u);
    tSupportIndices = tUtilities.getSupportIndices(1,0,1);
    EXPECT_EQ(tSupportIndices[0],std::vector<int>({1,0,0}));
    EXPECT_EQ(tSupportIndices[1],std::vector<int>({1,1,0}));
    EXPECT_EQ(tSupportIndices[2],std::vector<int>({0,0,0}));
    EXPECT_EQ(tSupportIndices[3],std::vector<int>({2,0,0}));

    EXPECT_EQ(tUtilities.getSupportIndices(2,0,1).size(),3u);
    EXPECT_EQ(tUtilities.getSupportIndices(0,1,1).size(),4u);
    EXPECT_EQ(tUtilities.getSupportIndices(1,1,1).size(),5u);
    EXPECT_EQ(tUtilities.getSupportIndices(2,1,1).size(),4u);
    EXPECT_EQ(tUtilities.getSupportIndices(0,2,1).size(),3u);
    EXPECT_EQ(tUtilities.getSupportIndices(1,2,1).size(),4u);
    EXPECT_EQ(tUtilities.getSupportIndices(2,2,1).size(),3u);

    EXPECT_EQ(tUtilities.getSupportIndices(0,0,2).size(),3u);
    EXPECT_EQ(tUtilities.getSupportIndices(1,0,2).size(),4u);
    EXPECT_EQ(tUtilities.getSupportIndices(2,0,2).size(),3u);
    EXPECT_EQ(tUtilities.getSupportIndices(0,1,2).size(),4u);
    EXPECT_EQ(tUtilities.getSupportIndices(1,1,2).size(),5u);
    EXPECT_EQ(tUtilities.getSupportIndices(2,1,2).size(),4u);
    EXPECT_EQ(tUtilities.getSupportIndices(0,2,2).size(),3u);
    EXPECT_EQ(tUtilities.getSupportIndices(1,2,2).size(),4u);
    EXPECT_EQ(tUtilities.getSupportIndices(2,2,2).size(),3u);
}

}
}
