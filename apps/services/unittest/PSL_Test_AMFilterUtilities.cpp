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

PSL_TEST(AMFilterUtilities, getBoundingBox)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back({-1.0,-2.0,-3.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,2.0,0.0});
    tCoordinates.push_back({0.0,0.0,3.0});

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0,1,2,3});

    Vector tBuildDirection(std::vector<double>({0.0,0.0,1.0}));
    Vector tUBasisVector(std::vector<double>({1.0,0.0,0.0}));
    Vector tVBasisVector(std::vector<double>({0.0,1.0,0.0}));

    std::vector<int> tBaseLayer({0,1,2});

    AMFilterUtilities tUtilities(tCoordinates,tConnectivity,tUBasisVector,tVBasisVector,tBuildDirection,tBaseLayer);

    Vector tMaxUVWCoords, tMinUVWCoords;

    tUtilities.getBoundingBox(tMaxUVWCoords,tMinUVWCoords);

    EXPECT_EQ(tMaxUVWCoords,Vector(std::vector<double>({1.0,2.0,3.0})));
    EXPECT_EQ(tMinUVWCoords,Vector(std::vector<double>({-1.0,-2.0,-3.0})));

    AMFilterUtilities tUtilities2(tCoordinates,tConnectivity,tBuildDirection,tUBasisVector,tVBasisVector,tBaseLayer);
    
    // rotate space 90 degrees
    tUtilities2.getBoundingBox(tMaxUVWCoords,tMinUVWCoords);

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

    tUtilities3.getBoundingBox(tMaxUVWCoords,tMinUVWCoords);

    EXPECT_DOUBLE_EQ(tMaxUVWCoords(0),sqrt(2)/2);
    EXPECT_DOUBLE_EQ(tMaxUVWCoords(1),sqrt(2)/2);
    EXPECT_EQ(tMaxUVWCoords(2),1.0);

    EXPECT_EQ(tMinUVWCoords(0),0.0);
    EXPECT_DOUBLE_EQ(tMinUVWCoords(1),-1*sqrt(2)/2);
    EXPECT_EQ(tMinUVWCoords(2),0.0);
}

}
}
