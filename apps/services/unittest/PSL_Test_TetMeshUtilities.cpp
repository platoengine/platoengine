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
#include "PSL_TetMeshUtilities.hpp"
#include "PSL_Vector.hpp"

#include <vector>
#include <cmath>

namespace PlatoSubproblemLibrary
{
namespace TestingPoint
{

PSL_TEST(TetMeshUtilities,construction)
{
    std::vector<std::vector<double>> tCoordinates;

    std::vector<std::vector<int>> tConnectivity;

    // tCoordinates empty
    EXPECT_THROW(TetMeshUtilities tUtilities(tCoordinates,tConnectivity),std::domain_error);

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    // tConnectivity empty
    EXPECT_THROW(TetMeshUtilities tUtilities(tCoordinates,tConnectivity),std::domain_error);

    tConnectivity.push_back({0,1,2,3});

    // valid construction
    EXPECT_NO_THROW(TetMeshUtilities tUtilities(tCoordinates,tConnectivity));

    // wrong size coordinate vector
    tCoordinates.push_back({0.0,1.0,1.0,0.0});
    EXPECT_THROW(TetMeshUtilities tUtilities(tCoordinates,tConnectivity),std::domain_error);

    // wrong size connectivity vector
    tCoordinates.pop_back();
    tConnectivity.push_back({0,1,2});
    EXPECT_THROW(TetMeshUtilities tUtilities(tCoordinates,tConnectivity),std::domain_error);

    // index in connectivity out of range of coordinate vector
    tConnectivity.pop_back();
    tConnectivity.push_back({1,2,3,4});
    EXPECT_THROW(TetMeshUtilities tUtilities(tCoordinates,tConnectivity),std::out_of_range);
}

PSL_TEST(TetMeshUtilities, computeBoundingBox)
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

    TetMeshUtilities tUtilities(tCoordinates,tConnectivity);

    Vector tMaxUVWCoords, tMinUVWCoords;

    tUtilities.computeBoundingBox(tUBasisVector,tVBasisVector,tBuildDirection,tMaxUVWCoords,tMinUVWCoords);

    EXPECT_EQ(tMaxUVWCoords,Vector(std::vector<double>({1.0,2.0,3.0})));
    EXPECT_EQ(tMinUVWCoords,Vector(std::vector<double>({-1.0,-2.0,-3.0})));

    // rotate space 90 degrees
    tUtilities.computeBoundingBox(tBuildDirection,tUBasisVector,tVBasisVector,tMaxUVWCoords,tMinUVWCoords);

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

    TetMeshUtilities tUtilities2(tCoordinates2,tConnectivity);

    tUtilities2.computeBoundingBox(tUBasisVector2,tVBasisVector2,tBuildDirection2,tMaxUVWCoords,tMinUVWCoords);

    EXPECT_DOUBLE_EQ(tMaxUVWCoords(0),sqrt(2)/2);
    EXPECT_DOUBLE_EQ(tMaxUVWCoords(1),sqrt(2)/2);
    EXPECT_EQ(tMaxUVWCoords(2),1.0);

    EXPECT_EQ(tMinUVWCoords(0),0.0);
    EXPECT_DOUBLE_EQ(tMinUVWCoords(1),-1*sqrt(2)/2);
    EXPECT_EQ(tMinUVWCoords(2),0.0);
}

PSL_TEST(TetMeshUtilities, computeMinEdgeLength)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0,1,2,3});

    TetMeshUtilities tUtilities(tCoordinates,tConnectivity);

    double tMinEdgeLength = tUtilities.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 1.0);

    tCoordinates.clear();

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({0.4,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    TetMeshUtilities tUtilities2(tCoordinates,tConnectivity);

    tMinEdgeLength = tUtilities2.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 0.4);

    tCoordinates.clear();

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,0.4,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    TetMeshUtilities tUtilities3(tCoordinates,tConnectivity);

    tMinEdgeLength = tUtilities3.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 0.4);

    tCoordinates.clear();

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,0.4});

    TetMeshUtilities tUtilities4(tCoordinates,tConnectivity);

    tMinEdgeLength = tUtilities4.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 0.4);

    tCoordinates.clear();

    tCoordinates.push_back({0.1,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    TetMeshUtilities tUtilities5(tCoordinates,tConnectivity);

    tMinEdgeLength = tUtilities5.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 0.9);
    
    tCoordinates.clear();

    tCoordinates.push_back({0.0,0.1,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    TetMeshUtilities tUtilities6(tCoordinates,tConnectivity);

    tMinEdgeLength = tUtilities6.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 0.9);

    tCoordinates.clear();

    tCoordinates.push_back({0.0,0.0,0.1});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    TetMeshUtilities tUtilities7(tCoordinates,tConnectivity);

    tMinEdgeLength = tUtilities7.computeMinEdgeLength();

    EXPECT_DOUBLE_EQ(tMinEdgeLength, 0.9);
}

PSL_TEST(TetMeshUtilities, isPointInTetrahedron)
{
    std::vector<std::vector<double>> tCoordinates;
    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    std::vector<int> tTet = {0,1,2,3};

    Vector tPoint({0.1,0.1,0.1});

    // not enough indices
    std::vector<int> tBogusTet = {0,1,2};
    EXPECT_THROW(isPointInTetrahedron(tCoordinates,tBogusTet,tPoint),std::domain_error);

    // repeated index
    tBogusTet = {0,0,1,2};
    EXPECT_THROW(isPointInTetrahedron(tCoordinates,tBogusTet,tPoint),std::domain_error);

    // index out of range
    tBogusTet = {0,1,2,4};
    EXPECT_THROW(isPointInTetrahedron(tCoordinates,tBogusTet,tPoint),std::out_of_range);

    bool tIsPointInTet = isPointInTetrahedron(tCoordinates, tTet, tPoint);
    EXPECT_EQ(tIsPointInTet, true);

    tPoint = Vector({0.01,0.01,0.02});
    tIsPointInTet = isPointInTetrahedron(tCoordinates, tTet, tPoint);
    EXPECT_EQ(tIsPointInTet, true);

    tPoint = Vector({0.0001,0.0001,0.0001});
    tIsPointInTet = isPointInTetrahedron(tCoordinates, tTet, tPoint);
    EXPECT_EQ(tIsPointInTet, true);

    tPoint = Vector({0.0001,0.0001,0.9});
    tIsPointInTet = isPointInTetrahedron(tCoordinates, tTet, tPoint);
    EXPECT_EQ(tIsPointInTet, true);

    tPoint = Vector({1.0,1.0,1.0});
    tIsPointInTet = isPointInTetrahedron(tCoordinates, tTet, tPoint);
    EXPECT_EQ(tIsPointInTet, false);

    tPoint = Vector({-0.1,0.1,0.1});
    tIsPointInTet = isPointInTetrahedron(tCoordinates, tTet, tPoint);
    EXPECT_EQ(tIsPointInTet, false);

    tPoint = Vector({0.1,-0.1,0.1});
    tIsPointInTet = isPointInTetrahedron(tCoordinates, tTet, tPoint);
    EXPECT_EQ(tIsPointInTet, false);

    tPoint = Vector({0.1,0.1,-0.1});
    tIsPointInTet = isPointInTetrahedron(tCoordinates, tTet, tPoint);
    EXPECT_EQ(tIsPointInTet, false);
}

PSL_TEST(TetMeshUtilities, computeBarycentricCoordinates)
{
    std::vector<std::vector<double>> tCoordinates;
    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    std::vector<std::vector<int>> tConnectivity;
    tConnectivity.push_back({0,1,2,3});

    TetMeshUtilities tUtilities(tCoordinates,tConnectivity);

    Vector tPoint({0.0,0.0,0.0});

    // not enough indices
    std::vector<int> tBogusTet = {0,1,2};
    EXPECT_THROW(tUtilities.computeBarycentricCoordinates(tBogusTet,tPoint),std::domain_error);

    // repeated index
    tBogusTet = {0,0,1,2};
    EXPECT_THROW(tUtilities.computeBarycentricCoordinates(tBogusTet,tPoint),std::domain_error);

    // index out of range
    tBogusTet = {0,1,2,4};
    EXPECT_THROW(tUtilities.computeBarycentricCoordinates(tBogusTet,tPoint),std::out_of_range);

    // singular tet
    std::vector<std::vector<double>> tCoordinates2;
    tCoordinates2.push_back({0.0,0.0,0.0});
    tCoordinates2.push_back({0.0,0.0,0.0});
    tCoordinates2.push_back({0.0,1.0,0.0});
    tCoordinates2.push_back({0.0,0.0,1.0});
    TetMeshUtilities tUtilities2(tCoordinates2,tConnectivity);
    EXPECT_THROW(tUtilities2.computeBarycentricCoordinates(tConnectivity[0],tPoint),std::domain_error);

    std::vector<double> tBarycentricCoordinates = tUtilities.computeBarycentricCoordinates(tConnectivity[0], tPoint);
    EXPECT_EQ(tBarycentricCoordinates.size(), 4u);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[0], 1.0);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[1], 0.0);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[2], 0.0);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[3], 0.0);

    tPoint = Vector({1.0,0.0,0.0});
    tBarycentricCoordinates = tUtilities.computeBarycentricCoordinates(tConnectivity[0], tPoint);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[0], 0.0);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[1], 1.0);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[2], 0.0);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[3], 0.0);

    tPoint = Vector({0.0,1.0,0.0});
    tBarycentricCoordinates = tUtilities.computeBarycentricCoordinates(tConnectivity[0], tPoint);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[0], 0.0);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[1], 0.0);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[2], 1.0);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[3], 0.0);

    tPoint = Vector({0.0,0.0,1.0});
    tBarycentricCoordinates = tUtilities.computeBarycentricCoordinates(tConnectivity[0], tPoint);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[0], 0.0);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[1], 0.0);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[2], 0.0);
    EXPECT_DOUBLE_EQ(tBarycentricCoordinates[3], 1.0);
}

PSL_TEST(TetMeshUtilities, getTetIDForEachPoint)
{
    std::vector<std::vector<double>> tCoordinates;

     tCoordinates.push_back({1,0,1}); //0
     tCoordinates.push_back({1,1,1}); //1
     tCoordinates.push_back({0,1,1}); //2
     tCoordinates.push_back({0,0,1}); //3
     tCoordinates.push_back({1,1,0}); //4
     tCoordinates.push_back({1,0,0}); //5
     tCoordinates.push_back({0,0,0}); //6
     tCoordinates.push_back({0,1,0}); //7
    
    std::vector<std::vector<int>> tConnectivity;

     tConnectivity.push_back({0,2,7,6}); //0
     tConnectivity.push_back({0,2,6,3}); //1
     tConnectivity.push_back({0,4,5,6}); //2
     tConnectivity.push_back({0,4,6,7}); //3
     tConnectivity.push_back({0,1,4,7}); //4
     tConnectivity.push_back({0,1,7,2}); //5
    
    TetMeshUtilities tUtilities(tCoordinates,tConnectivity);

    std::vector<Vector> tGridCoordinates;
    tGridCoordinates.push_back(Vector({0.1,0.1,0.1}));
    tGridCoordinates.push_back(Vector({0.1,0.1,0.0}));
    tGridCoordinates.push_back(Vector({0.25,0.25,0.75}));
    tGridCoordinates.push_back(Vector({0.25,0.5,0.5}));
    tGridCoordinates.push_back(Vector({0.75,0.25,0.25}));
    tGridCoordinates.push_back(Vector({0.5,0.5,0.25}));
    tGridCoordinates.push_back(Vector({0.75,0.75,0.5}));
    tGridCoordinates.push_back(Vector({0.5,0.75,0.75}));

    std::vector<int> tContainingTetID;
    tUtilities.getTetIDForEachPoint(tGridCoordinates,tContainingTetID);

    EXPECT_EQ(tGridCoordinates.size(), tContainingTetID.size());

    EXPECT_EQ(tContainingTetID,std::vector<int>({ 0, 2, 1, 0, 2, 3, 4, 5 }));
}

}
}
