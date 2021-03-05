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

// PSL_TEST(TetMeshUtilities, computeBoundingBox)
// {
//     std::vector<std::vector<double>> tCoordinates;

//     tCoordinates.push_back({-1.0,-2.0,-3.0});
//     tCoordinates.push_back({1.0,0.0,0.0});
//     tCoordinates.push_back({0.0,2.0,0.0});
//     tCoordinates.push_back({0.0,0.0,3.0});

//     std::vector<std::vector<int>> tConnectivity;

//     tConnectivity.push_back({0,1,2,3});

//     Vector tUBasisVector(std::vector<double>({1.0,0.0,0.0}));
//     Vector tVBasisVector(std::vector<double>({0.0,1.0,0.0}));
//     Vector tBuildDirection(std::vector<double>({0.0,0.0,1.0}));

//     TetMeshUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection);

//     Vector tMaxUVWCoords, tMinUVWCoords;

//     tUtilities.computeBoundingBox(tMaxUVWCoords,tMinUVWCoords);

//     EXPECT_EQ(tMaxUVWCoords,Vector(std::vector<double>({1.0,2.0,3.0})));
//     EXPECT_EQ(tMinUVWCoords,Vector(std::vector<double>({-1.0,-2.0,-3.0})));

//     TetMeshUtilities tUtilities2(tCoordinates,tConnectivity,tBuildDirection,tUBasisVector,tVBasisVector);
    
//     // rotate space 90 degrees
//     tUtilities2.computeBoundingBox(tMaxUVWCoords,tMinUVWCoords);

//     EXPECT_EQ(tMaxUVWCoords,Vector(std::vector<double>({3.0,1.0,2.0})));
//     EXPECT_EQ(tMinUVWCoords,Vector(std::vector<double>({-3.0,-1.0,-2.0})));

//     // rotate space 45 degrees
//     std::vector<std::vector<double>> tCoordinates2;

//     tCoordinates2.push_back({0.0,0.0,0.0});
//     tCoordinates2.push_back({1.0,0.0,0.0});
//     tCoordinates2.push_back({0.0,1.0,0.0});
//     tCoordinates2.push_back({0.0,0.0,1.0});

//     Vector tBuildDirection2(std::vector<double>({0.0,0.0,1.0}));
//     Vector tUBasisVector2(std::vector<double>({1.0,1.0,0.0}));
//     Vector tVBasisVector2(std::vector<double>({-1.0,1.0,0.0}));

//     tUBasisVector2.normalize();
//     tVBasisVector2.normalize();

//     TetMeshUtilities tUtilities3(tCoordinates2,tConnectivity,tUBasisVector2,tVBasisVector2,tBuildDirection2);

//     tUtilities3.computeBoundingBox(tMaxUVWCoords,tMinUVWCoords);

//     EXPECT_DOUBLE_EQ(tMaxUVWCoords(0),sqrt(2)/2);
//     EXPECT_DOUBLE_EQ(tMaxUVWCoords(1),sqrt(2)/2);
//     EXPECT_EQ(tMaxUVWCoords(2),1.0);

//     EXPECT_EQ(tMinUVWCoords(0),0.0);
//     EXPECT_DOUBLE_EQ(tMinUVWCoords(1),-1*sqrt(2)/2);
//     EXPECT_EQ(tMinUVWCoords(2),0.0);
// }

PSL_TEST(TetMeshUtilities, computeNumElementsInEachDirection)
{
    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    double tTargetEdgeLength = 0.1;
    std::vector<int> tNumElements = computeNumElementsInEachDirection(tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    EXPECT_EQ(tNumElements, std::vector<int>({10,20,30}));

    tTargetEdgeLength = 0.11;
    tNumElements = computeNumElementsInEachDirection(tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    EXPECT_EQ(tNumElements, std::vector<int>({9,18,27}));

    tTargetEdgeLength = 0.11;
    tNumElements = computeNumElementsInEachDirection(tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    EXPECT_EQ(tNumElements, std::vector<int>({9,18,27}));

    tTargetEdgeLength = 0.105;
    tNumElements = computeNumElementsInEachDirection(tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    EXPECT_EQ(tNumElements, std::vector<int>({9,19,28}));

    // target length larger than bounding box
    tTargetEdgeLength = 1.1;
    EXPECT_THROW(computeNumElementsInEachDirection(tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength),std::domain_error);

    // negative target length
    tTargetEdgeLength = -1.0;
    EXPECT_THROW(computeNumElementsInEachDirection(tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength),std::domain_error);
}

PSL_TEST(TetMeshUtilities, computeGridXYZCoordinates)
{
    Vector tUBasisVector(std::vector<double>({1.0,0.0,0.0}));
    Vector tVBasisVector(std::vector<double>({0.0,1.0,0.0}));
    Vector tBuildDirection(std::vector<double>({0.0,0.0,1.0}));

    std::vector<int> tNumElements({10,20,30});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    std::vector<Vector> tCoordinates;

    // non-positive number of elements
    tNumElements = {-1,20,30};
    EXPECT_THROW(computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tCoordinates),std::domain_error);
    tNumElements = {0,20,30};
    EXPECT_THROW(computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tCoordinates),std::domain_error);

    // min and max flipped so max input is less than min
    tNumElements = {10,20,30};
    EXPECT_THROW(computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMinUVWCoords,tMaxUVWCoords,tNumElements,tCoordinates),std::domain_error);

    computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tCoordinates);
    std::vector<int> tIndex = {1,1,1};
    Vector tGridPointCoordinate = tCoordinates[getSerializedIndex(tNumElements,tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),0.1);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),0.1);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),0.1);

    tIndex = {1, 5, 12};
    tGridPointCoordinate = tCoordinates[getSerializedIndex(tNumElements,tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),0.1);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),0.5);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),1.2);

    tNumElements = {5,8,15};
    computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tCoordinates);
    tIndex = {1, 1, 1};
    tGridPointCoordinate = tCoordinates[getSerializedIndex(tNumElements,tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),0.2);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),0.25);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),0.2);
    
    tNumElements = {10,20,30};
    tMaxUVWCoords = Vector({1.0,2.0,3.0});
    tMinUVWCoords = Vector({-1.0,-2.0,-3.0});
    computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tCoordinates);
    tIndex = {1, 1, 1};
    tGridPointCoordinate = tCoordinates[getSerializedIndex(tNumElements,tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),-0.8);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),-1.8);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),-2.8);

    tUBasisVector = Vector({0.0,1.0,0.0});
    tVBasisVector = Vector({0.0,0.0,1.0});
    tBuildDirection = Vector({1.0,0.0,0.0});
    computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tCoordinates);
    tGridPointCoordinate = tCoordinates[getSerializedIndex(tNumElements,tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),-2.8);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),-0.8);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),-1.8);

    tUBasisVector = Vector({1.0,1.0,0.0});
    tVBasisVector = Vector({-1.0,1.0,0.0});
    tBuildDirection = Vector({0.0,0.0,1.0});
    tMaxUVWCoords = Vector({1.0,1.0,3.0});
    tMinUVWCoords = Vector({0.0,0.0,0.0});
    tNumElements = {10,10,10};
    computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tCoordinates);
    tIndex = {0, 0, 1};
    tGridPointCoordinate = tCoordinates[getSerializedIndex(tNumElements,tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),0.0);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),0.0);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),0.3);

    tIndex = {1, 0, 0};
    tGridPointCoordinate = tCoordinates[getSerializedIndex(tNumElements,tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),0.1);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),0.1);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),0.0);

    tIndex = {1, 3, 2};
    tGridPointCoordinate = tCoordinates[getSerializedIndex(tNumElements,tIndex)];
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(0),-0.2);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(1),0.4);
    EXPECT_DOUBLE_EQ(tGridPointCoordinate(2),0.6);
}

PSL_TEST(TetMeshUtilities, getSerializedIndex)
{
    // tIndex wrong dimension
    std::vector<int> tIndex = {0,0};
    std::vector<int> tNumElements = {2,2,2};
    EXPECT_THROW(getSerializedIndex(tNumElements,tIndex), std::domain_error);

    // Non-positive number of elements
    tIndex = {0,0,0};
    tNumElements = {2,0,2};
    EXPECT_THROW(getSerializedIndex(tNumElements,tIndex), std::domain_error);

    // tNumElements wrong dimension
    tNumElements = {2,2};
    EXPECT_THROW(getSerializedIndex(tNumElements,tIndex), std::domain_error);

    // tIndex out of range
    tIndex = {-1,0,0};
    tNumElements = {2,2,2};
    EXPECT_THROW(getSerializedIndex(tNumElements,tIndex), std::out_of_range);

    // tIndex out of range
    tIndex = {0,4,0};
    EXPECT_THROW(getSerializedIndex(tNumElements,tIndex), std::out_of_range);

    tIndex = {0,0,0};
    int tSerializedIndex = getSerializedIndex(tNumElements,tIndex);
    EXPECT_EQ(tSerializedIndex,0);

    tIndex = {1,0,0};
    tSerializedIndex = getSerializedIndex(tNumElements,tIndex);
    EXPECT_EQ(tSerializedIndex,1);

    tIndex = {2,0,0};
    tSerializedIndex = getSerializedIndex(tNumElements,tIndex);
    EXPECT_EQ(tSerializedIndex,2);
    
    tIndex = {0,1,0};
    tSerializedIndex = getSerializedIndex(tNumElements,tIndex);
    EXPECT_EQ(tSerializedIndex,3);

    tIndex = {0,1,1};
    tSerializedIndex = getSerializedIndex(tNumElements,tIndex);
    EXPECT_EQ(tSerializedIndex,12);

    tIndex = {1,1,1};
    tSerializedIndex = getSerializedIndex(tNumElements,tIndex);
    EXPECT_EQ(tSerializedIndex,13);

    tIndex = {1,1,2};
    tSerializedIndex = getSerializedIndex(tNumElements,tIndex);
    EXPECT_EQ(tSerializedIndex,22);
}

}
}
