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

#include "PSL_AMFilterUtilities.hpp"
#include "PSL_Vector.hpp"

#include <vector>
#include <cmath>

namespace PlatoSubproblemLibrary
{
namespace TestingPoint
{

PSL_TEST(AMFilterUtilities,construction)
{
    std::vector<std::vector<double>> tCoordinates;

    std::vector<std::vector<int>> tConnectivity;

    Vector tBuildDirection(std::vector<double>({0.0,0.0,1.0}));
    Vector tUBasisVector(std::vector<double>({1.0,0.0,0.0}));
    Vector tVBasisVector(std::vector<double>({0.0,1.0,0.0}));

    std::vector<int> tBaseLayer({});

    // tCoordinates empty
    EXPECT_THROW(AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer),std::domain_error);

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    // tConnectivity empty
    EXPECT_THROW(AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer),std::domain_error);

    tConnectivity.push_back({0,1,2,3});

    // tBaseLayer empty
    EXPECT_THROW(AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer),std::domain_error);

    tBaseLayer = std::vector<int>({0,1,2});

    // valid construction
    EXPECT_NO_THROW(AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer));

    // wrong size coordinate vector
    tCoordinates.push_back({0.0,1.0,1.0,0.0});
    EXPECT_THROW(AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer),std::domain_error);

    // wrong size connectivity vector
    tCoordinates.pop_back();
    tConnectivity.push_back({0,1,2});
    EXPECT_THROW(AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer),std::domain_error);

    // index in connectivity out of range of coordinate vector
    tConnectivity.pop_back();
    tConnectivity.push_back({1,2,3,4});
    EXPECT_THROW(AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer),std::out_of_range);

    // provided basis not orthogonal
    tConnectivity.pop_back();
    EXPECT_THROW(AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,tUBasisVector,tBuildDirection,tBaseLayer),std::domain_error);
    
    // provided basis is not positively oriented
    EXPECT_THROW(AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tVBasisVector,tUBasisVector,tBuildDirection,tBaseLayer),std::domain_error);

    // provided basis is not unit length 
    EXPECT_THROW(AMFilterUtilities tUtilities(tCoordinates,tConnectivity,2*tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer),std::domain_error);
    EXPECT_THROW(AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,2*tVBasisVector,tBuildDirection,tBaseLayer),std::domain_error);
    EXPECT_THROW(AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,2*tBuildDirection,tBaseLayer),std::domain_error);
}

PSL_TEST(AMFilterUtilities, computeBoundingBox)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back({-1.0,-2.0,-3.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,2.0,0.0});
    tCoordinates.push_back({0.0,0.0,3.0});

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0,1,2,3});

    Vector tUBasisVector(std::vector<double>({1.0,0.0,0.0}));
    Vector tVBasisVector(std::vector<double>({0.0,1.0,0.0}));
    Vector tBuildDirection(std::vector<double>({0.0,0.0,1.0}));

    std::vector<int> tBaseLayer({0,1,2});

    AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer);

    Vector tMaxUVWCoords, tMinUVWCoords;

    tUtilities.computeBoundingBox(tMaxUVWCoords,tMinUVWCoords);

    EXPECT_EQ(tMaxUVWCoords,Vector(std::vector<double>({1.0,2.0,3.0})));
    EXPECT_EQ(tMinUVWCoords,Vector(std::vector<double>({-1.0,-2.0,-3.0})));

    AMFilterUtilities tUtilities2(tCoordinates,tConnectivity,tBuildDirection,tUBasisVector,tVBasisVector,tBaseLayer);
    
    // rotate space 90 degrees
    tUtilities2.computeBoundingBox(tMaxUVWCoords,tMinUVWCoords);

    EXPECT_EQ(tMaxUVWCoords,Vector(std::vector<double>({3.0,1.0,2.0})));
    EXPECT_EQ(tMinUVWCoords,Vector(std::vector<double>({-3.0,-1.0,-2.0})));

    // rotate space 45 degrees
    std::vector<std::vector<double>> tCoordinates2;

    tCoordinates2.push_back({0.0,0.0,0.0});
    tCoordinates2.push_back({1.0,0.0,0.0});
    tCoordinates2.push_back({0.0,1.0,0.0});
    tCoordinates2.push_back({0.0,0.0,1.0});

    Vector tBuildDirection2(std::vector<double>({0.0,0.0,1.0}));
    Vector tUBasisVector2(std::vector<double>({1.0,1.0,0.0}));
    Vector tVBasisVector2(std::vector<double>({-1.0,1.0,0.0}));

    tUBasisVector2.normalize();
    tVBasisVector2.normalize();

    AMFilterUtilities tUtilities3(tCoordinates2,tConnectivity,tUBasisVector2,tVBasisVector2,tBuildDirection2,tBaseLayer);

    tUtilities3.computeBoundingBox(tMaxUVWCoords,tMinUVWCoords);

    EXPECT_DOUBLE_EQ(tMaxUVWCoords(0),sqrt(2)/2);
    EXPECT_DOUBLE_EQ(tMaxUVWCoords(1),sqrt(2)/2);
    EXPECT_EQ(tMaxUVWCoords(2),1.0);

    EXPECT_EQ(tMinUVWCoords(0),0.0);
    EXPECT_DOUBLE_EQ(tMinUVWCoords(1),-1*sqrt(2)/2);
    EXPECT_EQ(tMinUVWCoords(2),0.0);
}

PSL_TEST(AMFilterUtilities, computeMinEdgeLength)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0,1,2,3});

    Vector tUBasisVector(std::vector<double>({1.0,0.0,0.0}));
    Vector tVBasisVector(std::vector<double>({0.0,1.0,0.0}));
    Vector tBuildDirection(std::vector<double>({0.0,0.0,1.0}));

    std::vector<int> tBaseLayer({0,1,2});

    AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer);

    double tMinEdgeLength = tUtilities.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 1.0);

    tCoordinates.clear();

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({0.4,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    AMFilterUtilities tUtilities2(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer);

    tMinEdgeLength = tUtilities2.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 0.4);

    tCoordinates.clear();

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,0.4,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    AMFilterUtilities tUtilities3(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer);

    tMinEdgeLength = tUtilities3.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 0.4);

    tCoordinates.clear();

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,0.4});

    AMFilterUtilities tUtilities4(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer);

    tMinEdgeLength = tUtilities4.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 0.4);

    tCoordinates.clear();

    tCoordinates.push_back({0.1,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    AMFilterUtilities tUtilities5(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer);

    tMinEdgeLength = tUtilities5.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 0.9);
    
    tCoordinates.clear();

    tCoordinates.push_back({0.0,0.1,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    AMFilterUtilities tUtilities6(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer);

    tMinEdgeLength = tUtilities6.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 0.9);

    tCoordinates.clear();

    tCoordinates.push_back({0.0,0.0,0.1});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    AMFilterUtilities tUtilities7(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer);

    tMinEdgeLength = tUtilities7.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 0.9);
}

PSL_TEST(AMFilterUtilities, computeNumElementsInEachDirection)
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

PSL_TEST(AMFilterUtilities, computeGridXYZCoordinates)
{
    Vector tUBasisVector(std::vector<double>({1.0,0.0,0.0}));
    Vector tVBasisVector(std::vector<double>({0.0,1.0,0.0}));
    Vector tBuildDirection(std::vector<double>({0.0,0.0,1.0}));

    std::vector<int> tNumElements({10,20,30});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    std::vector<int> tIndex = {1,1,1};
    Vector tCoordinates = computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tIndex);
    EXPECT_DOUBLE_EQ(tCoordinates(0),0.1);
    EXPECT_DOUBLE_EQ(tCoordinates(1),0.1);
    EXPECT_DOUBLE_EQ(tCoordinates(2),0.1);

    tIndex = {1, 5, 12};
    tCoordinates = computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tIndex);
    EXPECT_DOUBLE_EQ(tCoordinates(0),0.1);
    EXPECT_DOUBLE_EQ(tCoordinates(1),0.5);
    EXPECT_DOUBLE_EQ(tCoordinates(2),1.2);

    tNumElements = {5,8,15};
    tIndex = {1, 1, 1};
    tCoordinates = computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tIndex);
    EXPECT_DOUBLE_EQ(tCoordinates(0),0.2);
    EXPECT_DOUBLE_EQ(tCoordinates(1),0.25);
    EXPECT_DOUBLE_EQ(tCoordinates(2),0.2);
    
    tNumElements = {10,20,30};
    tMaxUVWCoords = Vector({1.0,2.0,3.0});
    tMinUVWCoords = Vector({-1.0,-2.0,-3.0});
    tIndex = {1, 1, 1};
    tCoordinates = computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tIndex);
    EXPECT_DOUBLE_EQ(tCoordinates(0),-0.8);
    EXPECT_DOUBLE_EQ(tCoordinates(1),-1.8);
    EXPECT_DOUBLE_EQ(tCoordinates(2),-2.8);

    tUBasisVector = Vector({0.0,1.0,0.0});
    tVBasisVector = Vector({0.0,0.0,1.0});
    tBuildDirection = Vector({1.0,0.0,0.0});
    tCoordinates = computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tIndex);
    EXPECT_DOUBLE_EQ(tCoordinates(0),-2.8);
    EXPECT_DOUBLE_EQ(tCoordinates(1),-0.8);
    EXPECT_DOUBLE_EQ(tCoordinates(2),-1.8);

    tUBasisVector = Vector({1.0,1.0,0.0});
    tVBasisVector = Vector({-1.0,1.0,0.0});
    tBuildDirection = Vector({0.0,0.0,1.0});
    tMaxUVWCoords = Vector({1.0,1.0,3.0});
    tMinUVWCoords = Vector({0.0,0.0,0.0});
    tNumElements = {10,10,10};
    tIndex = {0, 0, 1};
    tCoordinates = computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tIndex);
    EXPECT_DOUBLE_EQ(tCoordinates(0),0.0);
    EXPECT_DOUBLE_EQ(tCoordinates(1),0.0);
    EXPECT_DOUBLE_EQ(tCoordinates(2),0.3);

    tIndex = {1, 0, 0};
    tCoordinates = computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tIndex);
    EXPECT_DOUBLE_EQ(tCoordinates(0),0.1);
    EXPECT_DOUBLE_EQ(tCoordinates(1),0.1);
    EXPECT_DOUBLE_EQ(tCoordinates(2),0.0);

    tIndex = {1, 3, 2};
    tCoordinates = computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tIndex);
    EXPECT_DOUBLE_EQ(tCoordinates(0),-0.2);
    EXPECT_DOUBLE_EQ(tCoordinates(1),0.4);
    EXPECT_DOUBLE_EQ(tCoordinates(2),0.6);

    // min and max flipped so max input is less than min
    EXPECT_THROW(computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMinUVWCoords,tMaxUVWCoords,tNumElements,tIndex),std::domain_error);

    // non-positive number of elements
    tNumElements = {-1,20,30};
    EXPECT_THROW(computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tIndex),std::domain_error);
    tNumElements = {0,20,30};
    EXPECT_THROW(computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tIndex),std::domain_error);
    tNumElements = {10,20,30};

    // index out of range
    tIndex = {-1,1,1};
    EXPECT_THROW(computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tIndex),std::out_of_range);
    tIndex = {20,1,1};
    EXPECT_THROW(computeGridXYZCoordinates(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords,tNumElements,tIndex),std::out_of_range);
}

}
}
