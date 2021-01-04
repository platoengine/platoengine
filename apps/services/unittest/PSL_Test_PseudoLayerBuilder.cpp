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

#include "PSL_PseudoLayerBuilder.hpp"
#include "PSL_Vector.hpp"

#include <vector>
#include <cmath>

namespace PlatoSubproblemLibrary
{
namespace TestingPoint
{

PSL_TEST(PseudoLayerBuilder,construction)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({1.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 1.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});

    double tCriticalPrintAngle = M_PI/4;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer({0, 1, 2});

    EXPECT_NO_THROW(PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer));

    tConnectivity[0] = {0, 1, 2, 4};
    EXPECT_THROW(PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer),std::out_of_range);

    tConnectivity[0] = {-1, 1, 2, 3};
    EXPECT_THROW(PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer),std::out_of_range);

    tConnectivity[0] = {0, 1, 2, 3};
    tBaseLayer = {-1, 1, 2};
    EXPECT_THROW(PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer),std::out_of_range);

    tBaseLayer = {0, 1, 4};
    EXPECT_THROW(PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer),std::out_of_range);

    tBaseLayer = {0, 1, 3};
    tCriticalPrintAngle = -1.0;
    EXPECT_THROW(PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer),std::out_of_range);

    tCriticalPrintAngle = M_PI/4;
    tBuildDirection = std::vector<double>({0.0,0.0,0.0});
    EXPECT_THROW(PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer),std::overflow_error);
}

PSL_TEST(PseudoLayerBuilder,orderNodesInBuildDirection)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.1, 0.1, 0.1}));
    tCoordinates.push_back(std::vector<double>({1.0, 0.2, 0.2}));
    tCoordinates.push_back(std::vector<double>({0.3, 1.0, 0.3}));
    tCoordinates.push_back(std::vector<double>({0.2, 0.3, 1.0}));

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});

    double tCriticalPrintAngle = M_PI/4;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer({0, 1, 2});

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tOrderedNodes = tBuilder.orderNodesInBuildDirection();

    EXPECT_EQ(tOrderedNodes[0], 0);
    EXPECT_EQ(tOrderedNodes[1], 1);
    EXPECT_EQ(tOrderedNodes[2], 2);
    EXPECT_EQ(tOrderedNodes[3], 3);

    tBuildDirection = std::vector<double>({0.0,1.0,0.0});
    PseudoLayerBuilder tBuilder2(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    tOrderedNodes = tBuilder2.orderNodesInBuildDirection();
    EXPECT_EQ(tOrderedNodes[0], 0);
    EXPECT_EQ(tOrderedNodes[1], 1);
    EXPECT_EQ(tOrderedNodes[2], 3);
    EXPECT_EQ(tOrderedNodes[3], 2);

    tBuildDirection = std::vector<double>({1.0,0.0,0.0});
    PseudoLayerBuilder tBuilder3(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    tOrderedNodes = tBuilder3.orderNodesInBuildDirection();
    EXPECT_EQ(tOrderedNodes[0], 0);
    EXPECT_EQ(tOrderedNodes[1], 3);
    EXPECT_EQ(tOrderedNodes[2], 2);
    EXPECT_EQ(tOrderedNodes[3], 1);

    tBuildDirection = std::vector<double>({1.0,1.0,0.0});
    PseudoLayerBuilder tBuilder4(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    tOrderedNodes = tBuilder4.orderNodesInBuildDirection();
    EXPECT_EQ(tOrderedNodes[0], 0);
    EXPECT_EQ(tOrderedNodes[1], 3);
    EXPECT_EQ(tOrderedNodes[2], 1);
    EXPECT_EQ(tOrderedNodes[3], 2);

    tBuildDirection = std::vector<double>({1.0,-1.0,1.0});
    PseudoLayerBuilder tBuilder5(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    tOrderedNodes = tBuilder5.orderNodesInBuildDirection();
    EXPECT_EQ(tOrderedNodes[0], 2);
    EXPECT_EQ(tOrderedNodes[1], 0);
    EXPECT_EQ(tOrderedNodes[2], 3);
    EXPECT_EQ(tOrderedNodes[3], 1);
}

PSL_TEST(PseudoLayerBuilder,setBaseLayerIDToZeroAndOthersToMinusOne)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({1.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 1.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});

    double tCriticalPrintAngle = M_PI/4;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer({0, 1, 2});

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();

    EXPECT_EQ(tPseudoLayers[0], 0);
    EXPECT_EQ(tPseudoLayers[1], 0);
    EXPECT_EQ(tPseudoLayers[2], 0);
    EXPECT_EQ(tPseudoLayers[3], -1);

    tCoordinates.push_back(std::vector<double>({-1.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, -1.0, 0.0}));

    tConnectivity.push_back(std::vector<int>({0,4,5,3}));

    tBaseLayer.push_back(4);
    tBaseLayer.push_back(5);

    PseudoLayerBuilder tBuilder2(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    tPseudoLayers = tBuilder2.setBaseLayerIDToZeroAndOthersToMinusOne();

    EXPECT_EQ(tPseudoLayers[0], 0);
    EXPECT_EQ(tPseudoLayers[1], 0);
    EXPECT_EQ(tPseudoLayers[2], 0);
    EXPECT_EQ(tPseudoLayers[3], -1);
    EXPECT_EQ(tPseudoLayers[4], 0);
    EXPECT_EQ(tPseudoLayers[5], 0);
}

void checkCoefficients(const PseudoLayerBuilder& tBuilder,
                       const std::vector<std::set<PlatoSubproblemLibrary::SupportPointData>>& tSupportSet,
                       const std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>>& tSupportCoefficients,
                       const std::vector<std::vector<double>>& tCoordinates,
                       const PlatoSubproblemLibrary::Vector& tBuildDirection,
                       const double& tCriticalPrintAngle)

{
    for(std::set<SupportPointData> tNodeSupportSet : tSupportSet)
    {
        for(SupportPointData tSupportPoint : tNodeSupportSet)
        {
            PlatoSubproblemLibrary::Vector tVec = getVectorToSupportPoint(tSupportPoint,tSupportCoefficients,tCoordinates);

            auto tSupportingNodes = tSupportPoint.second;

            if(tSupportingNodes.size() == 1u)
            {
                auto tNodeCoordinates = tCoordinates[*(tSupportingNodes.begin())];
                for(int i = 0; i < 3; ++i)
                {
                    EXPECT_DOUBLE_EQ(tVec(i), tNodeCoordinates[i]);
                }
            }
            else if(tSupportingNodes.size() == 2u)
            {
                PlatoSubproblemLibrary::Vector tVec0(tCoordinates[tSupportPoint.first]);
                PlatoSubproblemLibrary::Vector tSupportIncline = tVec - tVec0;

                EXPECT_NEAR(PlatoSubproblemLibrary::angle_between(tSupportIncline, -1*tBuildDirection),tCriticalPrintAngle,1e-8);
            }
            else
                throw(std::runtime_error("Support Point can be defined by only one or two nodes"));
        }
    }
}

PSL_TEST(PseudoLayerBuilder,computeSupportSetAndCoefficients_supportingNodesBelowSupportedNode_twoNodesInsideCriticalWindow)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.9, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 1.1, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});

    double tCriticalPrintAngle = M_PI/4;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer({0, 1, 2});

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tOrderedNodes = tBuilder.orderNodesInBuildDirection();
    std::vector<int> tPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();

    std::vector<std::set<PlatoSubproblemLibrary::SupportPointData>> tSupportSet;
    std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>> tSupportCoefficients;

    tSupportSet.resize(tCoordinates.size());

    tBuilder.computeSupportSetAndCoefficients(tSupportSet,tSupportCoefficients);

    std::set<SupportPointData> tGoldSupportSet;

    EXPECT_EQ(tSupportSet[0], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[1], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[2], tGoldSupportSet);

    tGoldSupportSet.insert(SupportPointData({3, {0}}));
    tGoldSupportSet.insert(SupportPointData({3, {1}}));
    tGoldSupportSet.insert(SupportPointData({3, {0,2}}));
    tGoldSupportSet.insert(SupportPointData({3, {1,2}}));

    EXPECT_EQ(tSupportSet[3], tGoldSupportSet);

    checkCoefficients(tBuilder, tSupportSet, tSupportCoefficients, tCoordinates, tBuildDirection, tCriticalPrintAngle);
}

PSL_TEST(PseudoLayerBuilder,computeSupportSetAndCoefficients_supportingNodesBelowSupportedNode_allNodesInsideCriticalWindow)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.9, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 1.1, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});

    double tCriticalPrintAngle = 5*M_PI/11;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer({0, 1, 2});

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tOrderedNodes = tBuilder.orderNodesInBuildDirection();
    std::vector<int> tPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();

    std::vector<std::set<PlatoSubproblemLibrary::SupportPointData>> tSupportSet;
    std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>> tSupportCoefficients;

    tSupportSet.resize(tCoordinates.size());

    tBuilder.computeSupportSetAndCoefficients(tSupportSet,tSupportCoefficients);

    std::set<SupportPointData> tGoldSupportSet;

    EXPECT_EQ(tSupportSet[0], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[1], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[2], tGoldSupportSet);

    tGoldSupportSet.insert(SupportPointData({3, {0}}));
    tGoldSupportSet.insert(SupportPointData({3, {1}}));
    tGoldSupportSet.insert(SupportPointData({3, {2}}));

    EXPECT_EQ(tSupportSet[3], tGoldSupportSet);

    checkCoefficients(tBuilder, tSupportSet, tSupportCoefficients, tCoordinates, tBuildDirection, tCriticalPrintAngle);
}

PSL_TEST(PseudoLayerBuilder,computeSupportSetAndCoefficients_supportingNodesBelowSupportedNode_oneNodeInsideCriticalWindow)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.9, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 1.1, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});

    double tCriticalPrintAngle = M_PI/8;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer({0, 1, 2});

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tOrderedNodes = tBuilder.orderNodesInBuildDirection();
    std::vector<int> tPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();

    std::vector<std::set<PlatoSubproblemLibrary::SupportPointData>> tSupportSet;
    std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>> tSupportCoefficients;

    tSupportSet.resize(tCoordinates.size());

    tBuilder.computeSupportSetAndCoefficients(tSupportSet,tSupportCoefficients);

    std::set<SupportPointData> tGoldSupportSet;

    EXPECT_EQ(tSupportSet[0], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[1], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[2], tGoldSupportSet);

    tGoldSupportSet.insert(SupportPointData({3, {0}}));
    tGoldSupportSet.insert(SupportPointData({3, {0,1}}));
    tGoldSupportSet.insert(SupportPointData({3, {0,2}}));

    EXPECT_EQ(tSupportSet[3], tGoldSupportSet);

    checkCoefficients(tBuilder, tSupportSet, tSupportCoefficients, tCoordinates, tBuildDirection, tCriticalPrintAngle);
}

PSL_TEST(PseudoLayerBuilder,computeSupportSetAndCoefficients_supportingNodesBelowSupportedNode_twoTets1)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.9, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 1.1, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0}));
    tCoordinates.push_back(std::vector<double>({0.0, -1.1, 0.0}));

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});
    tConnectivity.push_back({0, 4, 1, 3});

    double tCriticalPrintAngle = M_PI/4;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer({0, 1, 2, 4});

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tOrderedNodes = tBuilder.orderNodesInBuildDirection();
    std::vector<int> tPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();

    std::vector<std::set<PlatoSubproblemLibrary::SupportPointData>> tSupportSet;
    std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>> tSupportCoefficients;

    tSupportSet.resize(tCoordinates.size());

    tBuilder.computeSupportSetAndCoefficients(tSupportSet,tSupportCoefficients);

    std::set<SupportPointData> tGoldSupportSet;

    EXPECT_EQ(tSupportSet[0], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[1], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[2], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[4], tGoldSupportSet);

    tGoldSupportSet.insert(SupportPointData({3, {0}}));
    tGoldSupportSet.insert(SupportPointData({3, {1}}));
    tGoldSupportSet.insert(SupportPointData({3, {0,2}}));
    tGoldSupportSet.insert(SupportPointData({3, {1,2}}));
    tGoldSupportSet.insert(SupportPointData({3, {0,4}}));
    tGoldSupportSet.insert(SupportPointData({3, {1,4}}));

    EXPECT_EQ(tSupportSet[3], tGoldSupportSet);

    checkCoefficients(tBuilder, tSupportSet, tSupportCoefficients, tCoordinates, tBuildDirection, tCriticalPrintAngle);
}

PSL_TEST(PseudoLayerBuilder,computeSupportSetAndCoefficients_supportingNodesBelowSupportedNode_twoTets2)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.9, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 1.1, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0}));
    tCoordinates.push_back(std::vector<double>({0.0, -0.9, 0.0}));

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});
    tConnectivity.push_back({0, 4, 1, 3});

    double tCriticalPrintAngle = M_PI/4;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer({0, 1, 2, 4});

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tOrderedNodes = tBuilder.orderNodesInBuildDirection();
    std::vector<int> tPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();

    std::vector<std::set<PlatoSubproblemLibrary::SupportPointData>> tSupportSet;
    std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>> tSupportCoefficients;

    tSupportSet.resize(tCoordinates.size());

    tBuilder.computeSupportSetAndCoefficients(tSupportSet,tSupportCoefficients);

    std::set<SupportPointData> tGoldSupportSet;

    EXPECT_EQ(tSupportSet[0], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[1], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[2], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[4], tGoldSupportSet);

    tGoldSupportSet.insert(SupportPointData({3, {0}}));
    tGoldSupportSet.insert(SupportPointData({3, {1}}));
    tGoldSupportSet.insert(SupportPointData({3, {0,2}}));
    tGoldSupportSet.insert(SupportPointData({3, {1,2}}));
    tGoldSupportSet.insert(SupportPointData({3, {4}}));

    EXPECT_EQ(tSupportSet[3], tGoldSupportSet);

    checkCoefficients(tBuilder, tSupportSet, tSupportCoefficients, tCoordinates, tBuildDirection, tCriticalPrintAngle);
}

PSL_TEST(PseudoLayerBuilder,computeSupportSetAndCoefficients_supportingNodesBelowSupportedNode_twoTets3)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.9, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 1.1, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0}));
    tCoordinates.push_back(std::vector<double>({-1.1, 0.0, 0.0}));

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});
    tConnectivity.push_back({0, 2, 4, 3});

    double tCriticalPrintAngle = M_PI/4;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer({0, 1, 2, 4});

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tOrderedNodes = tBuilder.orderNodesInBuildDirection();
    std::vector<int> tPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();

    std::vector<std::set<PlatoSubproblemLibrary::SupportPointData>> tSupportSet;
    std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>> tSupportCoefficients;

    tSupportSet.resize(tCoordinates.size());

    tBuilder.computeSupportSetAndCoefficients(tSupportSet,tSupportCoefficients);

    std::set<SupportPointData> tGoldSupportSet;

    EXPECT_EQ(tSupportSet[0], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[1], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[2], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[4], tGoldSupportSet);

    tGoldSupportSet.insert(SupportPointData({3, {0}}));
    tGoldSupportSet.insert(SupportPointData({3, {1}}));
    tGoldSupportSet.insert(SupportPointData({3, {0,2}}));
    tGoldSupportSet.insert(SupportPointData({3, {1,2}}));
    tGoldSupportSet.insert(SupportPointData({3, {0,4}}));

    EXPECT_EQ(tSupportSet[3], tGoldSupportSet);

    checkCoefficients(tBuilder, tSupportSet, tSupportCoefficients, tCoordinates, tBuildDirection, tCriticalPrintAngle);
}

PSL_TEST(PseudoLayerBuilder,computeSupportSetAndCoefficients_supportingNodesBelowSupportedNode_twoTets4)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.9, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 1.1, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0}));
    tCoordinates.push_back(std::vector<double>({1.0, 1.0, 0.0}));

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});
    tConnectivity.push_back({2, 1, 4, 3});

    double tCriticalPrintAngle = M_PI/4;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer({0, 1, 2, 4});

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tOrderedNodes = tBuilder.orderNodesInBuildDirection();
    std::vector<int> tPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();

    std::vector<std::set<PlatoSubproblemLibrary::SupportPointData>> tSupportSet;
    std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>> tSupportCoefficients;

    tSupportSet.resize(tCoordinates.size());

    tBuilder.computeSupportSetAndCoefficients(tSupportSet,tSupportCoefficients);

    std::set<SupportPointData> tGoldSupportSet;

    EXPECT_EQ(tSupportSet[0], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[1], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[2], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[4], tGoldSupportSet);

    tGoldSupportSet.insert(SupportPointData({3, {0}}));
    tGoldSupportSet.insert(SupportPointData({3, {1}}));
    tGoldSupportSet.insert(SupportPointData({3, {0,2}}));
    tGoldSupportSet.insert(SupportPointData({3, {1,2}}));
    tGoldSupportSet.insert(SupportPointData({3, {1,4}}));

    EXPECT_EQ(tSupportSet[3], tGoldSupportSet);

    checkCoefficients(tBuilder, tSupportSet, tSupportCoefficients, tCoordinates, tBuildDirection, tCriticalPrintAngle);
}

PSL_TEST(PseudoLayerBuilder,computeSupportSetAndCoefficients_supportingNodesBelowSupportedNode_twoTets5)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.9, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 1.1, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0}));
    tCoordinates.push_back(std::vector<double>({0.6, 0.6, 1.1}));

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});
    tConnectivity.push_back({1, 2, 3, 4});

    double tCriticalPrintAngle = M_PI/4;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer({0, 1, 2});

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tOrderedNodes = tBuilder.orderNodesInBuildDirection();
    std::vector<int> tPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();

    std::vector<std::set<PlatoSubproblemLibrary::SupportPointData>> tSupportSet;
    std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>> tSupportCoefficients;

    tSupportSet.resize(tCoordinates.size());

    tBuilder.computeSupportSetAndCoefficients(tSupportSet,tSupportCoefficients);

    std::set<SupportPointData> tGoldSupportSet;

    EXPECT_EQ(tSupportSet[0], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[1], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[2], tGoldSupportSet);

    tGoldSupportSet.insert(SupportPointData({3, {0}}));
    tGoldSupportSet.insert(SupportPointData({3, {1}}));
    tGoldSupportSet.insert(SupportPointData({3, {0,2}}));
    tGoldSupportSet.insert(SupportPointData({3, {1,2}}));

    EXPECT_EQ(tSupportSet[3], tGoldSupportSet);

    tGoldSupportSet.clear();

    tGoldSupportSet.insert(SupportPointData({4, {1}}));
    tGoldSupportSet.insert(SupportPointData({4, {2}}));
    tGoldSupportSet.insert(SupportPointData({4, {1,3}}));
    tGoldSupportSet.insert(SupportPointData({4, {2,3}}));

    EXPECT_EQ(tSupportSet[4], tGoldSupportSet);

    checkCoefficients(tBuilder, tSupportSet, tSupportCoefficients, tCoordinates, tBuildDirection, tCriticalPrintAngle);
}

PSL_TEST(PseudoLayerBuilder,computeSupportSetAndCoefficients_supportingNodesBelowSupportedNode_threeTets)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.9, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 1.1, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0}));
    tCoordinates.push_back(std::vector<double>({1.0, 1.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({-0.9, 0.0, 0.0}));

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});
    tConnectivity.push_back({0, 2, 5, 3});
    tConnectivity.push_back({2, 1, 4, 3});

    double tCriticalPrintAngle = M_PI/4;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer({0, 1, 2, 4, 5});

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tOrderedNodes = tBuilder.orderNodesInBuildDirection();
    std::vector<int> tPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();

    std::vector<std::set<PlatoSubproblemLibrary::SupportPointData>> tSupportSet;
    std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>> tSupportCoefficients;

    tSupportSet.resize(tCoordinates.size());

    tBuilder.computeSupportSetAndCoefficients(tSupportSet,tSupportCoefficients);

    std::set<SupportPointData> tGoldSupportSet;

    EXPECT_EQ(tSupportSet[0], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[1], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[2], tGoldSupportSet);
    EXPECT_EQ(tSupportSet[4], tGoldSupportSet);

    tGoldSupportSet.insert(SupportPointData({3, {0}}));
    tGoldSupportSet.insert(SupportPointData({3, {1}}));
    tGoldSupportSet.insert(SupportPointData({3, {5}}));
    tGoldSupportSet.insert(SupportPointData({3, {0,2}}));
    tGoldSupportSet.insert(SupportPointData({3, {1,2}}));
    tGoldSupportSet.insert(SupportPointData({3, {1,4}}));
    tGoldSupportSet.insert(SupportPointData({3, {2,5}}));

    EXPECT_EQ(tSupportSet[3], tGoldSupportSet);

    checkCoefficients(tBuilder, tSupportSet, tSupportCoefficients, tCoordinates, tBuildDirection, tCriticalPrintAngle);
}

PSL_TEST(PseudoLayerBuilder, getVectorToSupportPoint)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({1.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 1.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<SupportPointData> tSupportPoints;
    tSupportPoints.push_back(SupportPointData({3, {0,1}}));
    tSupportPoints.push_back(SupportPointData({3, {1,2}}));
    tSupportPoints.push_back(SupportPointData({3, {0,5}}));
    tSupportPoints.push_back(SupportPointData({3, {0,2}}));

    std::map<SupportPointData, std::vector<double>> tSupportCoefficients;

    tSupportCoefficients[tSupportPoints[0]] = {0.73, 0.27};
    tSupportCoefficients[tSupportPoints[1]] = {0.53, 0.47};

    PlatoSubproblemLibrary::Vector tVec = getVectorToSupportPoint(tSupportPoints[0], tSupportCoefficients, tCoordinates);

    EXPECT_DOUBLE_EQ(tVec(0), 0.27);
    EXPECT_DOUBLE_EQ(tVec(1), 0.0);
    EXPECT_DOUBLE_EQ(tVec(2), 0.0);

    tVec = getVectorToSupportPoint(tSupportPoints[1], tSupportCoefficients, tCoordinates);

    EXPECT_DOUBLE_EQ(tVec(0), 0.53);
    EXPECT_DOUBLE_EQ(tVec(1), 0.47);
    EXPECT_DOUBLE_EQ(tVec(2), 0.0);


    EXPECT_THROW(getVectorToSupportPoint(tSupportPoints[2], tSupportCoefficients, tCoordinates),std::out_of_range);

    tSupportCoefficients[tSupportPoints[2]] = {0.5, 0.5};

    EXPECT_THROW(getVectorToSupportPoint(tSupportPoints[2], tSupportCoefficients, tCoordinates),std::out_of_range);

    tSupportCoefficients[tSupportPoints[3]] = {0.5};

    EXPECT_THROW(getVectorToSupportPoint(tSupportPoints[3], tSupportCoefficients, tCoordinates),std::domain_error);
}

PSL_TEST(PseudoLayerBuilder,assignNodeToPseudoLayer)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.9, 0.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 1.1, 0.0}));
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0}));
    tCoordinates.push_back(std::vector<double>({1.0, 1.0, 0.0}));
    tCoordinates.push_back(std::vector<double>({-0.9, 0.0, 0.0}));

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});
    tConnectivity.push_back({0, 2, 5, 3});
    tConnectivity.push_back({2, 1, 4, 3});

    double tCriticalPrintAngle = M_PI/4;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer;

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tOrderedNodes = tBuilder.orderNodesInBuildDirection();
    std::vector<int> tPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();

    std::vector<std::set<PlatoSubproblemLibrary::SupportPointData>> tSupportSet;
    std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>> tSupportCoefficients;

    tSupportSet.resize(tCoordinates.size());

    tBuilder.computeSupportSetAndCoefficients(tSupportSet,tSupportCoefficients);

    tPseudoLayers[0] = 0;
    tPseudoLayers[1] = 0;
    tPseudoLayers[2] = 0;
    tPseudoLayers[3] = -1;
    tPseudoLayers[4] = 1;
    tPseudoLayers[5] = 1;

    tPseudoLayers[3] = tBuilder.assignNodeToPseudoLayer(3, tPseudoLayers, tSupportSet[3]);

    EXPECT_EQ(tPseudoLayers[3],1);

    tPseudoLayers[1] = 1;
    tPseudoLayers[2] = 1;
    tPseudoLayers[3] = -1;
    
    tPseudoLayers[3] = tBuilder.assignNodeToPseudoLayer(3, tPseudoLayers, tSupportSet[3]);

    EXPECT_EQ(tPseudoLayers[3],2);
}

PSL_TEST(PseudoLayerBuilder,pruneSupportSet)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0})); // in critical window
    tCoordinates.push_back(std::vector<double>({0.9, 0.0, 0.0})); // in critical window
    tCoordinates.push_back(std::vector<double>({0.0, 1.1, 0.0})); // not in critical window
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0})); // node being supported
    tCoordinates.push_back(std::vector<double>({1.0, 1.0, 0.0})); // not in critical window
    tCoordinates.push_back(std::vector<double>({-0.9, 0.0, 0.0}));// in critical window

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});
    tConnectivity.push_back({0, 2, 5, 3});
    tConnectivity.push_back({2, 1, 4, 3});

    double tCriticalPrintAngle = M_PI/4;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer;

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tOrderedNodes = tBuilder.orderNodesInBuildDirection();
    std::vector<int> tPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();

    std::vector<std::set<PlatoSubproblemLibrary::SupportPointData>> tSupportSet;
    std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>> tSupportCoefficients;

    tSupportSet.resize(tCoordinates.size());

    tBuilder.computeSupportSetAndCoefficients(tSupportSet,tSupportCoefficients);

    tPseudoLayers[0] = 0;
    tPseudoLayers[1] = 0;
    tPseudoLayers[2] = 0;
    tPseudoLayers[3] = 1;
    tPseudoLayers[4] = 1;
    tPseudoLayers[5] = 1;

    std::set<SupportPointData> tGoldSupportSet;

    tGoldSupportSet.insert({3, {0}});
    tGoldSupportSet.insert({3, {1}});
    tGoldSupportSet.insert({3, {5}});
    tGoldSupportSet.insert({3, {0,2}});
    tGoldSupportSet.insert({3, {1,2}});
    tGoldSupportSet.insert({3, {1,4}});
    tGoldSupportSet.insert({3, {2,5}});

    EXPECT_EQ(tSupportSet[3], tGoldSupportSet);

    
    // nodes 4 and 5 are not in the support layer so any support points
    // depending on them will be removed from the support set when pruned
    // a support point depending on node 2 gets added back into the support set however 
    // because 5 is in the critical window but not supporting and 2 is not in the critical
    // window but is supporting
    
    tSupportSet[3] = tBuilder.pruneSupportSet(3, tPseudoLayers, tSupportSet[3], tSupportCoefficients);
    tGoldSupportSet.erase({3, {5}});
    tGoldSupportSet.erase({3, {1,4}});
    tGoldSupportSet.erase({3, {2,5}});
    tGoldSupportSet.insert({3, {2}});

    EXPECT_EQ(tSupportSet[3], tGoldSupportSet);
}

PSL_TEST(PseudoLayerBuilder,pruneSupportSet2)
{
    std::vector<std::vector<double>> tCoordinates;

    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 0.0})); // in critical window
    tCoordinates.push_back(std::vector<double>({0.9, 0.0, 0.0})); // in critical window
    tCoordinates.push_back(std::vector<double>({0.0, 1.1, 0.0})); // not in critical window
    tCoordinates.push_back(std::vector<double>({0.0, 0.0, 1.0})); // node being supported
    tCoordinates.push_back(std::vector<double>({1.0, 1.0, 0.0})); // not in critical window
    tCoordinates.push_back(std::vector<double>({-0.9, 0.0, 0.0}));// in critical window

    std::vector<std::vector<int>> tConnectivity;

    tConnectivity.push_back({0, 1, 2, 3});
    tConnectivity.push_back({0, 2, 5, 3});
    tConnectivity.push_back({2, 1, 4, 3});

    double tCriticalPrintAngle = M_PI/4;

    PlatoSubproblemLibrary::Vector tBuildDirection(std::vector<double>({0.0, 0.0, 1.0}));

    std::vector<int> tBaseLayer;

    PseudoLayerBuilder tBuilder(tCoordinates,tConnectivity,tCriticalPrintAngle,tBuildDirection,tBaseLayer);

    std::vector<int> tOrderedNodes = tBuilder.orderNodesInBuildDirection();
    std::vector<int> tPseudoLayers = tBuilder.setBaseLayerIDToZeroAndOthersToMinusOne();

    std::vector<std::set<PlatoSubproblemLibrary::SupportPointData>> tSupportSet;
    std::map<PlatoSubproblemLibrary::SupportPointData,std::vector<double>> tSupportCoefficients;

    tSupportSet.resize(tCoordinates.size());

    tBuilder.computeSupportSetAndCoefficients(tSupportSet,tSupportCoefficients);

    tPseudoLayers[0] = 0;
    tPseudoLayers[1] = 1;
    tPseudoLayers[2] = 1;
    tPseudoLayers[3] = 2;
    tPseudoLayers[4] = 1;
    tPseudoLayers[5] = 1;

    std::set<SupportPointData> tGoldSupportSet;

    tGoldSupportSet.insert({3, {0}});
    tGoldSupportSet.insert({3, {1}});
    tGoldSupportSet.insert({3, {5}});
    tGoldSupportSet.insert({3, {0,2}});
    tGoldSupportSet.insert({3, {1,2}});
    tGoldSupportSet.insert({3, {1,4}});
    tGoldSupportSet.insert({3, {2,5}});

    EXPECT_EQ(tSupportSet[3], tGoldSupportSet);

    
    // node 0 is not in the support layer so any support points
    // depending on it will be removed from the support set when pruned
    // a support point depending on node 2 gets added back into the support set however 
    // because 0 is in the critical window but not supporting and 2 is not in the critical
    // window but is supporting
    
    auto tGoldSupportCoefficients = tSupportCoefficients;
    
    tSupportSet[3] = tBuilder.pruneSupportSet(3, tPseudoLayers, tSupportSet[3], tSupportCoefficients);
    tGoldSupportSet.erase({3, {0}});
    tGoldSupportSet.erase({3, {0,2}});
    tGoldSupportCoefficients.erase({3, {0}});

    double tCoefficient = tGoldSupportCoefficients.at({3, {0,2}})[1];
    tGoldSupportCoefficients.erase({3, {0,2}});

    SupportPointData tNewSupportPoint = {3, {2}};
    tGoldSupportSet.insert(tNewSupportPoint);
    tGoldSupportCoefficients[tNewSupportPoint] = {tCoefficient};

    EXPECT_EQ(tSupportSet[3], tGoldSupportSet);
    EXPECT_EQ(tSupportCoefficients, tGoldSupportCoefficients);
}

}
}
