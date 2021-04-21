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
    Vector tBogusVector(tWBasisVector);
    tBogusVector.multiply(2);
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tBogusVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength),std::domain_error);

    // Max not strictly greater than min
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMinUVWCoords,tMaxUVWCoords,tTargetEdgeLength),std::domain_error);
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMaxUVWCoords,tTargetEdgeLength),std::domain_error);

    // Target edge length non-positive 
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,-tTargetEdgeLength),std::domain_error);
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,0),std::domain_error);

    std::vector<size_t> aNumElementsInEachDirection({5,5,5});
    EXPECT_NO_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,aNumElementsInEachDirection));

    // Wrong size vector
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,std::vector<size_t>({5,5})),std::domain_error);

    // Non-positive number of elements
    EXPECT_THROW(OrthogonalGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,std::vector<size_t>({5,5,0})),std::domain_error);
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
    EXPECT_EQ(tDimensions, std::vector<size_t>({11,21,31}));

    tTargetEdgeLength = 0.11;
    OrthogonalGridUtilities tUtilities2(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    tDimensions = tUtilities2.getGridDimensions();
    EXPECT_EQ(tDimensions, std::vector<size_t>({10,19,28}));

    tTargetEdgeLength = 0.105;
    OrthogonalGridUtilities tUtilities3(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    tDimensions = tUtilities3.getGridDimensions();
    EXPECT_EQ(tDimensions, std::vector<size_t>({10,20,29}));

    // // target length larger than bounding box
    tTargetEdgeLength = 1.1;
    OrthogonalGridUtilities tUtilities4(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    tDimensions = tUtilities4.getGridDimensions();
    EXPECT_EQ(tDimensions, std::vector<size_t>({2,2,3}));

    std::vector<size_t> tNumElements({5,6,7});
    OrthogonalGridUtilities tUtilities5(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);
    tDimensions = tUtilities5.getGridDimensions();
    EXPECT_EQ(tDimensions, std::vector<size_t>({6,7,8}));
}

PSL_TEST(OrthogonalGridUtilities, computeGridXYZCoordinates)
{
    Vector tUBasisVector(std::vector<double>({1.0,0.0,0.0}));
    Vector tVBasisVector(std::vector<double>({0.0,1.0,0.0}));
    Vector tWBasisVector(std::vector<double>({0.0,0.0,1.0}));

    std::vector<size_t> tNumElements({10,20,30});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    OrthogonalGridUtilities tUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);

    std::vector<Vector> tCoordinates;

    tUtilities.computeGridXYZCoordinates(tCoordinates);
    std::vector<size_t> tIndex = {1,1,1};
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

    std::vector<size_t> tNumElements = {2,2,2};
    OrthogonalGridUtilities tUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);

    // tIndex wrong dimension
    std::vector<size_t> tIndex = {0,0};
    EXPECT_THROW(tUtilities.getSerializedIndex(tIndex), std::domain_error);

    // tIndex out of range
    tIndex = {10000,0,0};
    EXPECT_THROW(tUtilities.getSerializedIndex(tIndex), std::out_of_range);

    // tIndex out of range
    tIndex = {0,4,0};
    EXPECT_THROW(tUtilities.getSerializedIndex(tIndex), std::out_of_range);

    tIndex = {0,0,0};
    size_t tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,0u);

    tIndex = {1,0,0};
    tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,1u);

    tIndex = {2,0,0};
    tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,2u);
    
    tIndex = {0,1,0};
    tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,3u);

    tIndex = {0,1,1};
    tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,12u);

    tIndex = {1,1,1};
    tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,13u);

    tIndex = {1,1,2};
    tSerializedIndex = tUtilities.getSerializedIndex(tIndex);
    EXPECT_EQ(tSerializedIndex,22u);
}

PSL_TEST(OrthogonalGridUtilities, getSupportIndices)
{
    Vector tUBasisVector({1,0,0});
    Vector tVBasisVector({0,1,0});
    Vector tWBasisVector({0,0,1});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    std::vector<size_t> tNumElements = {2,2,2};
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
    EXPECT_EQ(tSupportIndices[0],std::vector<size_t>({0,0,0}));
    EXPECT_EQ(tSupportIndices[1],std::vector<size_t>({0,1,0}));
    EXPECT_EQ(tSupportIndices[2],std::vector<size_t>({1,0,0}));

    EXPECT_EQ(tUtilities.getSupportIndices(1,0,1).size(),4u);
    tSupportIndices = tUtilities.getSupportIndices(1,0,1);
    EXPECT_EQ(tSupportIndices[0],std::vector<size_t>({1,0,0}));
    EXPECT_EQ(tSupportIndices[1],std::vector<size_t>({1,1,0}));
    EXPECT_EQ(tSupportIndices[2],std::vector<size_t>({0,0,0}));
    EXPECT_EQ(tSupportIndices[3],std::vector<size_t>({2,0,0}));

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

PSL_TEST(OrthogonalGridUtilities, getContainingGridElement)
{
    Vector tUBasisVector({1,0,0});
    Vector tVBasisVector({0,1,0});
    Vector tWBasisVector({0,0,1});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    std::vector<size_t> tNumElements = {2,2,2};
    OrthogonalGridUtilities tUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);

    Vector tPoint({0.25,0.5,0.75});

    std::vector<std::vector<size_t>> tGridIndicies = tUtilities.getContainingGridElement(tPoint);

    EXPECT_EQ(tGridIndicies.size(),8u);
    EXPECT_EQ(tGridIndicies[0],std::vector<size_t>({0,0,0}));
    EXPECT_EQ(tGridIndicies[1],std::vector<size_t>({1,0,0}));
    EXPECT_EQ(tGridIndicies[2],std::vector<size_t>({0,1,0}));
    EXPECT_EQ(tGridIndicies[3],std::vector<size_t>({1,1,0}));
    EXPECT_EQ(tGridIndicies[4],std::vector<size_t>({0,0,1}));
    EXPECT_EQ(tGridIndicies[5],std::vector<size_t>({1,0,1}));
    EXPECT_EQ(tGridIndicies[6],std::vector<size_t>({0,1,1}));
    EXPECT_EQ(tGridIndicies[7],std::vector<size_t>({1,1,1}));
}

PSL_TEST(OrthogonalGridUtilities, getSurroundingIndices)
{
    Vector tUBasisVector({1,0,0});
    tUBasisVector.normalize();
    Vector tVBasisVector({0,1,0});
    tVBasisVector.normalize();
    Vector tWBasisVector({0,0,1});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    std::vector<size_t> tNumElements = {2,2,2};
    OrthogonalGridUtilities tUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);

    Vector tPoint({0.25,0.5,0.75});

    std::vector<size_t> tSurroundingIndices = tUtilities.getSurroundingIndices(0,tPoint);
    EXPECT_EQ(tSurroundingIndices,std::vector<size_t>({0,1}));
    tSurroundingIndices = tUtilities.getSurroundingIndices(1,tPoint);
    EXPECT_EQ(tSurroundingIndices,std::vector<size_t>({0,1}));
    tSurroundingIndices = tUtilities.getSurroundingIndices(2,tPoint);
    EXPECT_EQ(tSurroundingIndices,std::vector<size_t>({0,1}));

    tPoint = Vector({0.75,0.5,1.7});

    tSurroundingIndices = tUtilities.getSurroundingIndices(0,tPoint);
    EXPECT_EQ(tSurroundingIndices,std::vector<size_t>({1,2}));
    tSurroundingIndices = tUtilities.getSurroundingIndices(1,tPoint);
    EXPECT_EQ(tSurroundingIndices,std::vector<size_t>({0,1}));
    tSurroundingIndices = tUtilities.getSurroundingIndices(2,tPoint);
    EXPECT_EQ(tSurroundingIndices,std::vector<size_t>({1,2}));

    tUBasisVector = Vector({1,1,0});
    tUBasisVector.normalize();
    tVBasisVector = Vector({-1,1,0});
    tVBasisVector.normalize();

    OrthogonalGridUtilities tUtilities2(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);

    tPoint = Vector({sqrt(2)/7 - sqrt(2)/3 ,sqrt(2)/7 + sqrt(2)/3 ,1});

    tSurroundingIndices = tUtilities2.getSurroundingIndices(0,tPoint);
    EXPECT_EQ(tSurroundingIndices,std::vector<size_t>({0,1}));
    tSurroundingIndices = tUtilities2.getSurroundingIndices(1,tPoint);
    EXPECT_EQ(tSurroundingIndices,std::vector<size_t>({0,1}));
    tSurroundingIndices = tUtilities2.getSurroundingIndices(2,tPoint);
    EXPECT_EQ(tSurroundingIndices,std::vector<size_t>({0,1}));

    tPoint.multiply(2);

    tSurroundingIndices = tUtilities2.getSurroundingIndices(0,tPoint);
    EXPECT_EQ(tSurroundingIndices,std::vector<size_t>({1,2}));
    tSurroundingIndices = tUtilities2.getSurroundingIndices(1,tPoint);
    EXPECT_EQ(tSurroundingIndices,std::vector<size_t>({1,2}));
    tSurroundingIndices = tUtilities2.getSurroundingIndices(2,tPoint);
    EXPECT_EQ(tSurroundingIndices,std::vector<size_t>({1,2}));
}

PSL_TEST(OrthogonalGridUtilities, interpolateScalar)
{
    Vector tUBasisVector({1,0,0});
    tUBasisVector.normalize();
    Vector tVBasisVector({0,1,0});
    tVBasisVector.normalize();
    Vector tWBasisVector({0,0,1});

    Vector tMaxUVWCoords({1.0,1.0,1.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    std::vector<size_t> tNumElements = {1,1,1};
    OrthogonalGridUtilities tUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);

    std::vector<std::vector<size_t>> tContainingElementIndicies;
    tContainingElementIndicies.push_back(std::vector<size_t>({0,0,0}));
    tContainingElementIndicies.push_back(std::vector<size_t>({1,0,0}));
    tContainingElementIndicies.push_back(std::vector<size_t>({0,1,0}));
    tContainingElementIndicies.push_back(std::vector<size_t>({1,1,0}));
    tContainingElementIndicies.push_back(std::vector<size_t>({0,0,1}));
    tContainingElementIndicies.push_back(std::vector<size_t>({1,0,1}));
    tContainingElementIndicies.push_back(std::vector<size_t>({0,1,1}));
    tContainingElementIndicies.push_back(std::vector<size_t>({1,1,1}));

    std::vector<double> tContainingElementDensities = {1, 2, 3, 4, 5, 6, 7, 8};

    Vector tPoint({0,0,0});

    // check each corner
    std::vector<double> tScalars({1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),1.0);

    tPoint = Vector({1.0,0.0,0.0});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),2.0);

    tPoint = Vector({0.0,1.0,0.0});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),3.0);

    tPoint = Vector({1.0,1.0,0.0});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),4.0);

    tPoint = Vector({0.0,0.0,1.0});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),5.0);

    tPoint = Vector({1.0,0.0,1.0});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),6.0);

    tPoint = Vector({0.0,1.0,1.0});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),7.0);

    tPoint = Vector({1.0,1.0,1.0});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),8.0);

    // check midway on each edge
    tPoint = Vector({0.5,0,0});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),1.5);

    tPoint = Vector({0,0.5,0});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),2.0);

    tPoint = Vector({0,0,0.5});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),3.0);

    tPoint = Vector({1,0.5,0});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),3.0);

    tPoint = Vector({1,0,0.5});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),4.0);

    tPoint = Vector({0.5,1,0});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),3.5);

    tPoint = Vector({0,1,0.5});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),5.0);
    
    tPoint = Vector({0,0.5,1});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),6.0);

    tPoint = Vector({0.5,0,1});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),5.5);

    tPoint = Vector({0.5,1,1});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),7.5);

    tPoint = Vector({1,0.5,1});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),7.0);

    tPoint = Vector({1,1,0.5});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),6.0);

    // check midpoint of each face
    tPoint = Vector({0.5,0.5,0});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),2.5);

    tPoint = Vector({0.5,0,0.5});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),3.5);

    tPoint = Vector({0,0.5,0.5});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),4.0);

    tPoint = Vector({0.5,0.5,1});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),6.5);

    tPoint = Vector({0.5,1,0.5});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),5.5);

    tPoint = Vector({1,0.5,0.5});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),5.0);

    // check midpoint of hex
    tPoint = Vector({0.5,0.5,0.5});
    EXPECT_EQ(tUtilities.interpolateScalar(tContainingElementIndicies,tContainingElementDensities,tPoint),4.5);

    // incorrect number of indices
    std::vector<std::vector<size_t>> tBogusContainingElementIndicies;
    EXPECT_THROW(tUtilities.interpolateScalar(tBogusContainingElementIndicies,tContainingElementDensities,tPoint),std::domain_error);

    // wrong number of dimensions
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({0,0}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({0,1}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({1,0}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({1,1}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({0,0}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({1,0}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({0,1}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({1,1}));
    EXPECT_THROW(tUtilities.interpolateScalar(tBogusContainingElementIndicies,tContainingElementDensities,tPoint),std::domain_error);


    // indices out of order
    tBogusContainingElementIndicies.clear();
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({0,0,0}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({0,1,0}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({1,0,0}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({1,1,0}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({0,0,1}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({1,0,1}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({0,1,1}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({1,1,1}));
    EXPECT_THROW(tUtilities.interpolateScalar(tBogusContainingElementIndicies,tContainingElementDensities,tPoint),std::domain_error);

    //out of bounds
    tBogusContainingElementIndicies.clear();
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({1,1,1}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({1,2,1}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({2,1,1}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({2,2,1}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({1,1,2}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({2,1,2}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({1,2,2}));
    tBogusContainingElementIndicies.push_back(std::vector<size_t>({2,2,2}));
    EXPECT_THROW(tUtilities.interpolateScalar(tBogusContainingElementIndicies,tContainingElementDensities,tPoint),std::out_of_range);

    // incorrect number of scalars
    std::vector<double> tBogusContainingElementDensities = {1,0,0};
    EXPECT_THROW(tUtilities.interpolateScalar(tContainingElementIndicies,tBogusContainingElementDensities,tPoint),std::domain_error);
}

PSL_TEST(OrthogonalGridUtilities, computePointUVWCoordinates)
{
    Vector tUBasisVector({1,1,0});
    tUBasisVector.normalize();
    Vector tVBasisVector({-1,1,0});
    tVBasisVector.normalize();
    Vector tWBasisVector({0,0,1});

    Vector tMinUVWCoords({0.0,0.0,0.0});
    Vector tMaxUVWCoords({1.0,1.0,1.0});

    std::vector<size_t> tNumElements = {1,1,1};
    OrthogonalGridUtilities tUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);

    Vector tXYZPoint({1.0/sqrt(2),1.0/sqrt(2),0});
    Vector tUVWCoords = tUtilities.computePointUVWCoordinates(tXYZPoint);
    EXPECT_DOUBLE_EQ(tUVWCoords(0),1.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(1),0.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(2),0.0);

    tXYZPoint = Vector({-1.0/sqrt(2),1.0/sqrt(2),0});
    tUVWCoords = tUtilities.computePointUVWCoordinates(tXYZPoint);
    EXPECT_DOUBLE_EQ(tUVWCoords(0),0.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(1),1.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(2),0.0);

    tXYZPoint = Vector({0.0,0.0,1.0});
    tUVWCoords = tUtilities.computePointUVWCoordinates(tXYZPoint);
    EXPECT_DOUBLE_EQ(tUVWCoords(0),0.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(1),0.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(2),1.0);
}

PSL_TEST(OrthogonalGridUtilities, computeGridPointUVWCoordinates)
{
    Vector tUBasisVector({1,1,0});
    tUBasisVector.normalize();
    Vector tVBasisVector({-1,1,0});
    tVBasisVector.normalize();
    Vector tWBasisVector({0,0,1});

    Vector tMinUVWCoords({0.0,0.0,0.0});
    Vector tMaxUVWCoords({1.0,2.0,3.0});

    std::vector<size_t> tNumElements = {1,1,1};
    OrthogonalGridUtilities tUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tNumElements);
    
    // out of range 
    std::vector<size_t> tIndex({2,0,0});
    EXPECT_THROW(tUtilities.computeGridPointUVWCoordinates(tIndex),std::out_of_range);

    tIndex = {10000,0,0};
    EXPECT_THROW(tUtilities.computeGridPointUVWCoordinates(tIndex),std::out_of_range);

    tIndex = {0,0,0};
    Vector tUVWCoords = tUtilities.computeGridPointUVWCoordinates(tIndex);
    EXPECT_DOUBLE_EQ(tUVWCoords(0),0.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(1),0.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(2),0.0);

    tIndex = {1,0,0};
    tUVWCoords = tUtilities.computeGridPointUVWCoordinates(tIndex);
    EXPECT_DOUBLE_EQ(tUVWCoords(0),1.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(1),0.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(2),0.0);

    tIndex = {0,1,0};
    tUVWCoords = tUtilities.computeGridPointUVWCoordinates(tIndex);
    EXPECT_DOUBLE_EQ(tUVWCoords(0),0.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(1),2.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(2),0.0);

    tIndex = {0,0,1};
    tUVWCoords = tUtilities.computeGridPointUVWCoordinates(tIndex);
    EXPECT_DOUBLE_EQ(tUVWCoords(0),0.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(1),0.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(2),3.0);

    tIndex = {1,1,0};
    tUVWCoords = tUtilities.computeGridPointUVWCoordinates(tIndex);
    EXPECT_DOUBLE_EQ(tUVWCoords(0),1.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(1),2.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(2),0.0);

    tIndex = {1,0,1};
    tUVWCoords = tUtilities.computeGridPointUVWCoordinates(tIndex);
    EXPECT_DOUBLE_EQ(tUVWCoords(0),1.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(1),0.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(2),3.0);

    tIndex = {0,1,1};
    tUVWCoords = tUtilities.computeGridPointUVWCoordinates(tIndex);
    EXPECT_DOUBLE_EQ(tUVWCoords(0),0.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(1),2.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(2),3.0);

    tIndex = {1,1,1};
    tUVWCoords = tUtilities.computeGridPointUVWCoordinates(tIndex);
    EXPECT_DOUBLE_EQ(tUVWCoords(0),1.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(1),2.0);
    EXPECT_DOUBLE_EQ(tUVWCoords(2),3.0);
}

}
}
