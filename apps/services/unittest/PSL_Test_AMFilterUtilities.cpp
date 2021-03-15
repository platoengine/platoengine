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
#include "PSL_OrthogonalGridUtilities.hpp"
#include "PSL_AMFilterUtilities.hpp"
#include "PSL_Interface_ParallelVector.hpp"

#include <vector>
#include <cmath>

namespace PlatoSubproblemLibrary
{
namespace TestingPoint
{

PSL_TEST(AMFilterUtilities,construction)
{
    // build TetMeshUtilities
    std::vector<std::vector<double>> tCoordinates;
    std::vector<std::vector<int>> tConnectivity;

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    tConnectivity.push_back({0,1,2,3});

    TetMeshUtilities tTetUtilities(tCoordinates,tConnectivity);

    // build OrthogonalGridUtilities
    Vector tUBasisVector({1.0,0.0,0.0});
    Vector tVBasisVector({0.0,1.0,0.0});
    Vector tWBasisVector({0.0,0.0,1.0});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    double tTargetEdgeLength = 0.1;

    OrthogonalGridUtilities tGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);

    double tPNorm = 200;

    AMFilterUtilities tAMFilterUtilities(tTetUtilities,tGridUtilities, tPNorm);

    EXPECT_THROW(AMFilterUtilities(tTetUtilities,tGridUtilities, 0.5),std::domain_error);
}

PSL_TEST(AMFilterUtilities,computeGridBlueprintDensity)
{
    // build TetMeshUtilities
    std::vector<std::vector<double>> tCoordinates;
    std::vector<std::vector<int>> tConnectivity;

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    tConnectivity.push_back({0,1,2,3});

    TetMeshUtilities tTetUtilities(tCoordinates,tConnectivity);

    // build OrthogonalGridUtilities
    Vector tUBasisVector({1.0,0.0,0.0});
    Vector tVBasisVector({0.0,1.0,0.0});
    Vector tWBasisVector({0.0,0.0,1.0});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    double tTargetEdgeLength = 0.1;

    OrthogonalGridUtilities tGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    std::vector<int> tGridDimensions = tGridUtilities.getGridDimensions();
    std::vector<Vector> tGridCoordinates;
    tGridUtilities.computeGridXYZCoordinates(tGridCoordinates);

    double tPNorm = 200;

    AMFilterUtilities tAMFilterUtilities(tTetUtilities,tGridUtilities, tPNorm);

    example::Interface_ParallelVector tVector({1,1,1,0});

    std::vector<double> tGridBlueprintDensity;
    tAMFilterUtilities.computeGridBlueprintDensity(&tVector,tGridBlueprintDensity);

    for(int i = 0; i < tGridDimensions[0]; ++i)
    {
        for(int j = 0; j < tGridDimensions[1]; ++j)
        {
            for(int k = 0; k < tGridDimensions[2]; ++k)
            {
                double tDensity = tGridBlueprintDensity[tGridUtilities.getSerializedIndex(i,j,k)];
                if(isPointInTetrahedron(tCoordinates,tConnectivity[0],tGridCoordinates[tGridUtilities.getSerializedIndex(i,j,k)]))
                {
                    double tGold = 1 - (k*0.1);
                    EXPECT_DOUBLE_EQ(tDensity,tGold);
                }
                else
                {
                    EXPECT_DOUBLE_EQ(tDensity,0);
                }
            }
        }
    }

    EXPECT_THROW(tAMFilterUtilities.computeGridPointBlueprintDensity({0,1},&tVector),std::domain_error);
}

PSL_TEST(AMFilterUtilities,computeGridPointBlueprintDensity)
{
    // build TetMeshUtilities
    std::vector<std::vector<double>> tCoordinates;
    std::vector<std::vector<int>> tConnectivity;

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    tConnectivity.push_back({0,1,2,3});

    TetMeshUtilities tTetUtilities(tCoordinates,tConnectivity);

    // build OrthogonalGridUtilities
    Vector tUBasisVector({1.0,0.0,0.0});
    Vector tVBasisVector({0.0,1.0,0.0});
    Vector tWBasisVector({0.0,0.0,1.0});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    double tTargetEdgeLength = 0.1;

    OrthogonalGridUtilities tGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    std::vector<int> tGridDimensions = tGridUtilities.getGridDimensions();
    std::vector<Vector> tGridCoordinates;
    tGridUtilities.computeGridXYZCoordinates(tGridCoordinates);

    double tPNorm = 200;

    AMFilterUtilities tAMFilterUtilities(tTetUtilities,tGridUtilities, tPNorm);

    example::Interface_ParallelVector tVector({1,1,1,0});

    for(int i = 0; i < tGridDimensions[0]; ++i)
    {
        for(int j = 0; j < tGridDimensions[1]; ++j)
        {
            for(int k = 0; k < tGridDimensions[2]; ++k)
            {
                double tDensity = tAMFilterUtilities.computeGridPointBlueprintDensity(i,j,k,&tVector);
                if(isPointInTetrahedron(tCoordinates,tConnectivity[0],tGridCoordinates[tGridUtilities.getSerializedIndex(i,j,k)]))
                {
                    double tGold = 1 - (k*0.1);
                    EXPECT_DOUBLE_EQ(tDensity,tGold);
                }
                else
                {
                    EXPECT_DOUBLE_EQ(tDensity,0);
                }
            }
        }
    }

    EXPECT_THROW(tAMFilterUtilities.computeGridPointBlueprintDensity({0,1},&tVector),std::domain_error);
}

PSL_TEST(AMFilterUtilities,computeGridSupportDensity)
{
    // build TetMeshUtilities
    std::vector<std::vector<double>> tCoordinates;
    std::vector<std::vector<int>> tConnectivity;

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    tConnectivity.push_back({0,1,2,3});

    TetMeshUtilities tTetUtilities(tCoordinates,tConnectivity);

    // build OrthogonalGridUtilities
    Vector tUBasisVector({1.0,0.0,0.0});
    Vector tVBasisVector({0.0,1.0,0.0});
    Vector tWBasisVector({0.0,0.0,1.0});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    double tTargetEdgeLength = 0.1;

    OrthogonalGridUtilities tGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    std::vector<int> tGridDimensions = tGridUtilities.getGridDimensions();

    double tPNorm = 200;

    AMFilterUtilities tAMFilterUtilities(tTetUtilities,tGridUtilities,tPNorm);

    example::Interface_ParallelVector tVector({1,1,0,1});

    std::vector<double> tGridBlueprintDensity;
    tAMFilterUtilities.computeGridBlueprintDensity(&tVector,tGridBlueprintDensity); 
    std::vector<double> tGridSupportDensity;
    tAMFilterUtilities.computeGridSupportDensity(tGridBlueprintDensity, tGridSupportDensity);

    EXPECT_EQ(tGridSupportDensity.size(),(unsigned int) tGridDimensions[0]*tGridDimensions[1]*tGridDimensions[2]);

    for(int i = 0; i < tGridDimensions[0]; ++i)
    {
        for(int j = 0; j < tGridDimensions[1]; ++j)
        {
            for(int k = 0; k < tGridDimensions[2]; ++k)
            {
                if(k == 0)
                {
                    EXPECT_EQ(tGridSupportDensity[tGridUtilities.getSerializedIndex(i,j,k)],1);
                }
                else
                {
                    auto tSupportIndices = tGridUtilities.getSupportIndices(i,j,k);
                    std::vector<double> tSupportDensityBelow;
                    for(auto tSupportIndex : tSupportIndices)
                    {
                        tSupportDensityBelow.push_back(tAMFilterUtilities.computeGridPointBlueprintDensity(tSupportIndex,&tVector));
                    }
                    double tSupportDensityAtIndex = smax(tSupportDensityBelow,tPNorm);
                    EXPECT_DOUBLE_EQ(tGridSupportDensity[tGridUtilities.getSerializedIndex(i,j,k)],tSupportDensityAtIndex);
                }
            }
        }
    }
}

PSL_TEST(AMFilterUtilities, computeGridPointPrintableDensity)
{
    // build TetMeshUtilities
    std::vector<std::vector<double>> tCoordinates;
    std::vector<std::vector<int>> tConnectivity;

    tCoordinates.push_back({0.0,0.0,0.0});
    tCoordinates.push_back({1.0,0.0,0.0});
    tCoordinates.push_back({0.0,1.0,0.0});
    tCoordinates.push_back({0.0,0.0,1.0});

    tConnectivity.push_back({0,1,2,3});

    TetMeshUtilities tTetUtilities(tCoordinates,tConnectivity);

    // build OrthogonalGridUtilities
    Vector tUBasisVector({1.0,0.0,0.0});
    Vector tVBasisVector({0.0,1.0,0.0});
    Vector tWBasisVector({0.0,0.0,1.0});

    Vector tMaxUVWCoords({1.0,2.0,3.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    double tTargetEdgeLength = 0.1;

    OrthogonalGridUtilities tGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    std::vector<int> tGridDimensions = tGridUtilities.getGridDimensions();

    double tPNorm = 200;

    AMFilterUtilities tAMFilterUtilities(tTetUtilities,tGridUtilities,tPNorm);

    example::Interface_ParallelVector tVector({1,1,0,1});

    std::vector<double> tGridBlueprintDensity;
    tAMFilterUtilities.computeGridBlueprintDensity(&tVector,tGridBlueprintDensity); 
    std::vector<double> tGridSupportDensity;
    tAMFilterUtilities.computeGridSupportDensity(tGridBlueprintDensity, tGridSupportDensity);

    // wrong dimensions of index
    EXPECT_THROW(tAMFilterUtilities.computeGridPointPrintableDensity({0,0},tGridBlueprintDensity,tGridSupportDensity),std::domain_error);
    // wrong size of support density vector
    EXPECT_THROW(tAMFilterUtilities.computeGridPointPrintableDensity({0,0,0},tGridBlueprintDensity,{0.0,0.0}),std::domain_error);
    // wrong size of blueprint density vector
    EXPECT_THROW(tAMFilterUtilities.computeGridPointPrintableDensity({0,0,0},{0.0,0.0},tGridSupportDensity),std::domain_error);

    for(int i = 0; i < tGridDimensions[0]; ++i)
    {
        for(int j = 1; j < tGridDimensions[1]; ++j)
        {
            for(int k = 1; k < tGridDimensions[2]; ++k)
            {
                double tPrintableDensity = tAMFilterUtilities.computeGridPointPrintableDensity(i,j,k,tGridBlueprintDensity,tGridSupportDensity);
                double tBlueprintDensity = tGridBlueprintDensity[tGridUtilities.getSerializedIndex(i,j,k)];
                double tGold = smin(tBlueprintDensity,tGridSupportDensity[tGridUtilities.getSerializedIndex(i,j,k)]);
                EXPECT_DOUBLE_EQ(tPrintableDensity,tGold);
            }
        }
    }

}

PSL_TEST(AMFilterUtilities, smoothMax)
{
    std::vector<double> tArgs;

    tArgs.push_back(0);
    tArgs.push_back(0.1);
    tArgs.push_back(0.2);
    tArgs.push_back(0.3);
    tArgs.push_back(0.4);
    tArgs.push_back(0.5);
    tArgs.push_back(0.6);
    tArgs.push_back(0.7);
    tArgs.push_back(0.8);
    tArgs.push_back(0.9);

    double tPNorm = 200;
    
    double tSmax = smax(tArgs,tPNorm);
    EXPECT_DOUBLE_EQ(tSmax,0.89839982193243095);

    tArgs.clear();

    tArgs.push_back(0);
    tArgs.push_back(0.1);
    tArgs.push_back(0.23);
    tArgs.push_back(0.48);
    tArgs.push_back(0.71);
    tArgs.push_back(0.32);
    tArgs.push_back(0.22);
    tArgs.push_back(0.17);
    tArgs.push_back(1.0);
    tArgs.push_back(0.336);

    tSmax = smax(tArgs,tPNorm);

    EXPECT_DOUBLE_EQ(tSmax,1.0);
}

PSL_TEST(AMFilterUtilities, smin)
{
    EXPECT_DOUBLE_EQ(smin(0,1),7.4505805969238281e-09);
    EXPECT_DOUBLE_EQ(smin(1,0),7.4505805969238281e-09);
    EXPECT_DOUBLE_EQ(smin(0.5,0.8),0.50000000745058037);
    EXPECT_DOUBLE_EQ(smin(0.5,0.5),0.5);
    EXPECT_DOUBLE_EQ(smin(-1,1),-0.9999999925494194);
    EXPECT_DOUBLE_EQ(smin(-2,-1),-1.9999999925494194);
}


}
}
