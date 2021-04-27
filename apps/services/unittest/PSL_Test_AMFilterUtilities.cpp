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

PSL_TEST(AMFilterUtilities,computeGridBlueprintDensity_oneElement)
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
    std::vector<size_t> tGridDimensions = tGridUtilities.getGridDimensions();
    std::vector<Vector> tGridCoordinates;
    tGridUtilities.computeGridXYZCoordinates(tGridCoordinates);

    double tPNorm = 200;

    AMFilterUtilities tAMFilterUtilities(tTetUtilities,tGridUtilities, tPNorm);

    example::Interface_ParallelVector tVector({1,1,1,0});

    std::vector<double> tGridBlueprintDensity;
    tAMFilterUtilities.computeGridBlueprintDensity(&tVector,tGridBlueprintDensity);

    for(size_t i = 0; i < tGridDimensions[0]; ++i)
    {
        for(size_t j = 0; j < tGridDimensions[1]; ++j)
        {
            for(size_t k = 0; k < tGridDimensions[2]; ++k)
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
}

PSL_TEST(AMFilterUtilities, computeGridBlueprintDensity_manyElements)
{
    // build OrthogonalGridUtilities
    Vector tUBasisVector({1.0,0.0,0.0});
    Vector tVBasisVector({0.0,1.0,0.0});
    Vector tWBasisVector({0.0,0.0,1.0});

    std::vector<std::vector<double>> tCoordinates;
    std::vector<std::vector<int>> tConnectivity;

    tCoordinates.push_back({1.0000000E+00,  -2.0000000E+00,   5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,  -2.0000000E+00,   5.0000000E-01});
    tCoordinates.push_back({0.0000000E+00,  -2.0000000E+00,   5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,  -2.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,  -2.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({0.0000000E+00,  -2.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,   2.0000000E+00 ,  5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,   2.0000000E+00,  -5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,   2.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({0.0000000E+00,   2.0000000E+00,  -5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,   2.0000000E+00 ,  5.0000000E-01});
    tCoordinates.push_back({0.0000000E+00,   2.0000000E+00 ,  5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,  -1.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,   0.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,   1.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,  -1.0000000E+00 ,  5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,   0.0000000E+00 ,  5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,   1.0000000E+00,   5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,   1.0000000E+00,  -5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,   0.0000000E+00,  -5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,  -1.0000000E+00,  -5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,   1.0000000E+00,   5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,   0.0000000E+00,   5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,  -1.0000000E+00 ,  5.0000000E-01});
    tCoordinates.push_back({3.4649723E-01,  -2.0000000E+00,   6.6227364E-04});
    tCoordinates.push_back({-4.5755927E-01,  -2.0000000E+00 , -1.3018114E-03});
    tCoordinates.push_back({3.7057984E-01,   2.0000000E+00,  -1.0030927E-03});
    tCoordinates.push_back({-4.8425606E-01,   2.0000000E+00 , -1.2747575E-01});
    tCoordinates.push_back({3.7704415E-01,  -1.2851638E+00,   5.0000000E-01});
    tCoordinates.push_back({-3.8986427E-01,  -1.3331544E+00 ,  5.0000000E-01});
    tCoordinates.push_back({4.7721431E-01,  -5.6466477E-01,   5.0000000E-01});
    tCoordinates.push_back({-4.5127513E-01,   4.1228361E-01 ,  5.0000000E-01});
    tCoordinates.push_back({3.3426469E-01,   1.4277973E+00 ,  5.0000000E-01});
    tCoordinates.push_back({4.9931655E-01,   3.5993220E-01,   5.0000000E-01});
    tCoordinates.push_back({-4.1495173E-01,   1.2439895E+00,   5.0000000E-01});
    tCoordinates.push_back({-7.1110536E-01,  -5.0176541E-01 ,  5.0000000E-01});
    tCoordinates.push_back({1.2718026E-01,   6.9585179E-01 ,  5.0000000E-01});
    tCoordinates.push_back({1.3835560E-02,  -8.1731689E-01,   5.0000000E-01});
    tCoordinates.push_back({-4.3732245E-01,  -8.4620111E-01,   5.0000000E-01});
    tCoordinates.push_back({-4.2138274E-01,  -1.2413886E-01 ,  5.0000000E-01});
    tCoordinates.push_back({3.7674078E-01,  -1.0905474E-01,   5.0000000E-01});
    tCoordinates.push_back({-3.0176796E-01,  -5.0405007E-01,   5.0000000E-01});
    tCoordinates.push_back({-1.1463497E-02,   1.3159500E-01,   5.0000000E-01});
    tCoordinates.push_back({-7.2702803E-03,  -3.1737371E-01 ,  5.0000000E-01});
    tCoordinates.push_back({3.0368259E-01,   1.3801634E+00 , -5.0000000E-01});
    tCoordinates.push_back({3.2007693E-01,   4.9573262E-01,  -5.0000000E-01});
    tCoordinates.push_back({-3.2297714E-01,  -5.9135782E-01 , -5.0000000E-01});
    tCoordinates.push_back({3.8761751E-01,  -1.4761064E+00 , -5.0000000E-01});
    tCoordinates.push_back({3.6181753E-01,  -3.8191212E-01,  -5.0000000E-01});
    tCoordinates.push_back({-3.4925889E-01,  -1.3889216E+00,  -5.0000000E-01});
    tCoordinates.push_back({-4.5864357E-01,   1.4575972E+00,  -5.0000000E-01});
    tCoordinates.push_back({-4.3492309E-01,   5.0528988E-01 , -5.0000000E-01});
    tCoordinates.push_back({2.1925244E-01,  -9.6959312E-01,  -5.0000000E-01});
    tCoordinates.push_back({-1.7417699E-01,   4.9920719E-02,  -5.0000000E-01});
    tCoordinates.push_back({-1.3999204E-01,   9.2784471E-01,  -5.0000000E-01});
    tCoordinates.push_back({-1.1241963E-01,  -4.7770185E-01 ,  2.4559946E-02});
    tCoordinates.push_back({1.3403264E-01,  -4.8723804E-01 ,  3.1963932E-01});
    tCoordinates.push_back({1.6589050E-01,   4.5969729E-02 ,  1.1225805E-02});
    tCoordinates.push_back({6.0420996E-01,  -1.5343626E-01 ,  9.1492117E-02});
    tCoordinates.push_back({2.5935486E-01,  -6.8884358E-01,   9.2958968E-02});
    tCoordinates.push_back({-3.6511753E-01,  -2.8768576E-01,  -1.1661862E-01});
    tCoordinates.push_back({-3.0765422E-01,   1.2593259E-01,   1.8706483E-01});
    tCoordinates.push_back({-2.8055589E-02,  -1.1333659E+00,  -6.1686190E-02});
    tCoordinates.push_back({-5.4833900E-01,   4.1417642E-01,  -8.4496843E-03});
    tCoordinates.push_back({-1.5004674E-01,   6.3182813E-01 ,  4.8866220E-02});
    tCoordinates.push_back({4.5369823E-01,  -7.3889238E-01,  -1.7314432E-01});
    tCoordinates.push_back({-1.6154908E-02,   1.5802358E+00 , -9.3357940E-02});
    tCoordinates.push_back({4.4728267E-01,  -1.3645467E+00,   3.6961955E-02});
    tCoordinates.push_back({-3.0335394E-02,  -1.5347573E+00,  -2.1365023E-02});
    tCoordinates.push_back({-5.7201740E-01,  -1.0118763E+00,   9.6961621E-02});
    tCoordinates.push_back({-5.0626935E-01,  -1.4464236E+00,  -4.0889135E-02});
    tCoordinates.push_back({-5.1374269E-01,   1.2379537E+00 , -7.3816009E-02});
    tCoordinates.push_back({3.8616245E-01,   1.1675581E+00 ,  2.3536345E-02});
    tCoordinates.push_back({4.8247555E-01,   5.3747499E-01 ,  7.3740368E-02});

    tConnectivity.push_back({38,        56,        42,        57});
    tConnectivity.push_back({44,        42,        56,        57});
    tConnectivity.push_back({8,        27,        10,        45});
    tConnectivity.push_back({44,        57,        56,        58});
    tConnectivity.push_back({41,        57,        58,        59});
    tConnectivity.push_back({57,        58,        59,        60});
    tConnectivity.push_back({31,        38,        57,        60});
    tConnectivity.push_back({44,        41,        57,        58});
    tConnectivity.push_back({35,        11,        12,        28});
    tConnectivity.push_back({40,        56,        61,        62});
    tConnectivity.push_back({30,        39,        38,        63});
    tConnectivity.push_back({23,        62,        61,        64});
    tConnectivity.push_back({23,        32,        62,        64});
    tConnectivity.push_back({32,        62,        64,        65});
    tConnectivity.push_back({44,        43,        41,        58});
    tConnectivity.push_back({44,        43,        58,        62});
    tConnectivity.push_back({16,        60,        59,        66});
    tConnectivity.push_back({16,        59,        14,        66});
    tConnectivity.push_back({33,        12,        27,        67});
    tConnectivity.push_back({49,        60,        53,        66});
    tConnectivity.push_back({53,        66,        60,        68});
    tConnectivity.push_back({14,        59,        49,        66});
    tConnectivity.push_back({48,        68,        63,        69});
    tConnectivity.push_back({25,        29,        68,        69});
    tConnectivity.push_back({25,        68,        48,        69});
    tConnectivity.push_back({30,        24,        70,        71});
    tConnectivity.push_back({21,        50,        70,        71});
    tConnectivity.push_back({50,        69,        63,        71});
    tConnectivity.push_back({30,        70,        63,        71});
    tConnectivity.push_back({1,        29,        16,        68});
    tConnectivity.push_back({5,        13,        48,        68});
    tConnectivity.push_back({26,        30,        69,        71});
    tConnectivity.push_back({29,        30,        63,        69});
    tConnectivity.push_back({29,        63,        68,        69});
    tConnectivity.push_back({53,        60,        63,        68});
    tConnectivity.push_back({6,        48,        50,        69});
    tConnectivity.push_back({25,         5,        48,        68});
    tConnectivity.push_back({10,        51,        45,        67});
    tConnectivity.push_back({42,        39,        36,        56});
    tConnectivity.push_back({50,        63,        47,        70});
    tConnectivity.push_back({47,        63,        56,        70});
    tConnectivity.push_back({30,        39,        63,        70});
    tConnectivity.push_back({36,        24,        21,        70});
    tConnectivity.push_back({30,        63,        69,        71});
    tConnectivity.push_back({29,        60,        16,        68});
    tConnectivity.push_back({53,        56,        47,        63});
    tConnectivity.push_back({2,        24,        30,        71});
    tConnectivity.push_back({30,        24,        39,        70});
    tConnectivity.push_back({29,        38,        60,        63});
    tConnectivity.push_back({28,        51,        10,        67});
    tConnectivity.push_back({27,        28,        10,        67});
    tConnectivity.push_back({35,        28,        67,        72});
    tConnectivity.push_back({33,        67,        27,        73});
    tConnectivity.push_back({33,        35,        67,        73});
    tConnectivity.push_back({14,        46,        59,        74});
    tConnectivity.push_back({40,        36,        23,        61});
    tConnectivity.push_back({46,        58,        59,        74});
    tConnectivity.push_back({34,        17,        59,        74});
    tConnectivity.push_back({37,        73,        18,        74});
    tConnectivity.push_back({35,        32,        22,        65});
    tConnectivity.push_back({48,        53,        63,        68});
    tConnectivity.push_back({6,        50,        26,        69});
    tConnectivity.push_back({16,        60,        66,        68});
    tConnectivity.push_back({53,        60,        56,        63});
    tConnectivity.push_back({38,        56,        60,        63});
    tConnectivity.push_back({56,        58,        57,        60});
    tConnectivity.push_back({15,         8,        45,        73});
    tConnectivity.push_back({31,        57,        59,        60});
    tConnectivity.push_back({37,        65,        35,        73});
    tConnectivity.push_back({45,        67,        55,        73});
    tConnectivity.push_back({35,        22,        11,        72});
    tConnectivity.push_back({28,         9,        51,        72});
    tConnectivity.push_back({35,        67,        65,        72});
    tConnectivity.push_back({35,        65,        22,        72});
    tConnectivity.push_back({49,        58,        56,        60});
    tConnectivity.push_back({34,        41,        58,        59});
    tConnectivity.push_back({54,        61,        62,        64});
    tConnectivity.push_back({49,        46,        58,        59});
    tConnectivity.push_back({35,        65,        67,        73});
    tConnectivity.push_back({51,        55,        67,        72});
    tConnectivity.push_back({19,        64,        52,        72});
    tConnectivity.push_back({37,        35,        33,        73});
    tConnectivity.push_back({32,        43,        62,        65});
    tConnectivity.push_back({22,        65,        64,        72});
    tConnectivity.push_back({22,        64,        19,        72});
    tConnectivity.push_back({54,        58,        46,        65});
    tConnectivity.push_back({52,        64,        54,        65});
    tConnectivity.push_back({32,        64,        22,        65});
    tConnectivity.push_back({12,        28,        27,        67});
    tConnectivity.push_back({54,        64,        62,        65});
    tConnectivity.push_back({40,        61,        23,        62});
    tConnectivity.push_back({52,        54,        46,        65});
    tConnectivity.push_back({20,        61,        54,        64});
    tConnectivity.push_back({36,        21,        23,        61});
    tConnectivity.push_back({22,        20,        19,        64});
    tConnectivity.push_back({36,        61,        56,        70});
    tConnectivity.push_back({54,        47,        56,        61});
    tConnectivity.push_back({23,        61,        20,        64});
    tConnectivity.push_back({42,        36,        40,        56});
    tConnectivity.push_back({44,        42,        40,        56});
    tConnectivity.push_back({49,        56,        53,        60});
    tConnectivity.push_back({38,        56,        57,        60});
    tConnectivity.push_back({3,        30,        29,        69});
    tConnectivity.push_back({39,        56,        63,        70});
    tConnectivity.push_back({23,        22,        32,        64});
    tConnectivity.push_back({20,        47,        54,        61});
    tConnectivity.push_back({44,        40,        43,        62});
    tConnectivity.push_back({11,        19,         9,        72});
    tConnectivity.push_back({31,        17,        16,        59});
    tConnectivity.push_back({49,        59,        60,        66});
    tConnectivity.push_back({31,        44,        41,        57});
    tConnectivity.push_back({34,        59,        58,        74});
    tConnectivity.push_back({17,        14,        59,        74});
    tConnectivity.push_back({33,        12,         7,        27});
    tConnectivity.push_back({37,        65,        73,        74});
    tConnectivity.push_back({18,        33,         7,        73});
    tConnectivity.push_back({52,        65,        55,        72});
    tConnectivity.push_back({33,        27,         7,        73});
    tConnectivity.push_back({23,        20,        22,        64});
    tConnectivity.push_back({9,        19,        51,        72});
    tConnectivity.push_back({29,        30,        38,        63});
    tConnectivity.push_back({29,        31,        16,        60});
    tConnectivity.push_back({13,        66,        53,        68});
    tConnectivity.push_back({1,        16,        13,        68});
    tConnectivity.push_back({16,        17,        14,        59});
    tConnectivity.push_back({45,        51,        55,        67});
    tConnectivity.push_back({15,        45,        46,        73});
    tConnectivity.push_back({55,        65,        67,        72});
    tConnectivity.push_back({50,        63,        70,        71});
    tConnectivity.push_back({24,        21,        70,        71});
    tConnectivity.push_back({25,         3,        29,        69});
    tConnectivity.push_back({45,        55,        46,        73});
    tConnectivity.push_back({37,        32,        35,        65});
    tConnectivity.push_back({14,        46,        49,        59});
    tConnectivity.push_back({37,        58,        43,        65});
    tConnectivity.push_back({37,        43,        32,        65});
    tConnectivity.push_back({31,        59,        16,        60});
    tConnectivity.push_back({34,        18,        17,        74});
    tConnectivity.push_back({13,        49,        53,        66});
    tConnectivity.push_back({47,        56,        61,        70});
    tConnectivity.push_back({48,        53,        50,        63});
    tConnectivity.push_back({26,        69,        50,        71});
    tConnectivity.push_back({18,         7,         8,        73});
    tConnectivity.push_back({52,        64,        65,        72});
    tConnectivity.push_back({11,        22,        19,        72});
    tConnectivity.push_back({28,         9,        10,        51});
    tConnectivity.push_back({38,        39,        56,        63});
    tConnectivity.push_back({26,        30,         3,        69});
    tConnectivity.push_back({35,        11,        28,        72});
    tConnectivity.push_back({43,        58,        62,        65});
    tConnectivity.push_back({29,        38,        31,        60});
    tConnectivity.push_back({31,        38,        44,        57});
    tConnectivity.push_back({4,        24,         2,        71});
    tConnectivity.push_back({36,        56,        39,        70});
    tConnectivity.push_back({4,        50,        21,        71});
    tConnectivity.push_back({25,        48,         6,        69});
    tConnectivity.push_back({7,        27,         8,        73});
    tConnectivity.push_back({23,        21,        20,        61});
    tConnectivity.push_back({49,        47,        53,        56});
    tConnectivity.push_back({4,        21,        24,        71});
    tConnectivity.push_back({18,        15,        17,        74});
    tConnectivity.push_back({31,        41,        17,        59});
    tConnectivity.push_back({16,        14,        13,        66});
    tConnectivity.push_back({54,        47,        49,        56});
    tConnectivity.push_back({49,        59,        58,        60});
    tConnectivity.push_back({40,        56,        36,        61});
    tConnectivity.push_back({54,        62,        58,        65});
    tConnectivity.push_back({37,        34,        58,        74});
    tConnectivity.push_back({17,        15,        14,        74});
    tConnectivity.push_back({54,        49,        46,        58});
    tConnectivity.push_back({54,        56,        58,        62});
    tConnectivity.push_back({26,         2,         3,        30});
    tConnectivity.push_back({1,        13,         5,        68});
    tConnectivity.push_back({6,         4,        26,        50});
    tConnectivity.push_back({50,        53,        47,        63});
    tConnectivity.push_back({48,        63,        50,        69});
    tConnectivity.push_back({21,        47,        61,        70});
    tConnectivity.push_back({1,        25,        29,        68});
    tConnectivity.push_back({34,        17,        41,        59});
    tConnectivity.push_back({44,        58,        56,        62});
    tConnectivity.push_back({46,        65,        58,        74});
    tConnectivity.push_back({20,        52,        19,        64});
    tConnectivity.push_back({35,        28,        12,        67});
    tConnectivity.push_back({14,        15,        46,        74});
    tConnectivity.push_back({37,        33,        18,        73});
    tConnectivity.push_back({14,        49,        13,        66});
    tConnectivity.push_back({44,        56,        40,        62});
    tConnectivity.push_back({37,        58,        65,        74});
    tConnectivity.push_back({20,        54,        52,        64});
    tConnectivity.push_back({40,        23,        32,        62});
    tConnectivity.push_back({18,         8,        15,        73});
    tConnectivity.push_back({38,        39,        42,        56});
    tConnectivity.push_back({36,        21,        61,        70});
    tConnectivity.push_back({52,        46,        55,        65});
    tConnectivity.push_back({16,        66,        13,        68});
    tConnectivity.push_back({46,        55,        65,        73});
    tConnectivity.push_back({34,        41,        43,        58});
    tConnectivity.push_back({15,        73,        46,        74});
    tConnectivity.push_back({44,        38,        42,        57});
    tConnectivity.push_back({25,         5,         6,        48});
    tConnectivity.push_back({1,        25,         3,        29});
    tConnectivity.push_back({26,         4,         2,        71});
    tConnectivity.push_back({36,        24,        23,        21});
    tConnectivity.push_back({27,        10,        45,        67});
    tConnectivity.push_back({46,        73,        65,        74});
    tConnectivity.push_back({54,        56,        49,        58});
    tConnectivity.push_back({29,        63,        60,        68});
    tConnectivity.push_back({25,        26,         3,        69});
    tConnectivity.push_back({36,        39,        24,        70});
    tConnectivity.push_back({20,        21,        47,        61});
    tConnectivity.push_back({54,        61,        56,        62});
    tConnectivity.push_back({40,        32,        43,        62});
    tConnectivity.push_back({8,        27,        45,        73});
    tConnectivity.push_back({25,         6,        26,        69});
    tConnectivity.push_back({26,         2,        30,        71});
    tConnectivity.push_back({1,         5,        25,        68});
    tConnectivity.push_back({21,        50,        47,        70});
    tConnectivity.push_back({27,        67,        45,        73});
    tConnectivity.push_back({28,        51,        67,        72});
    tConnectivity.push_back({37,        18,        34,        74});
    tConnectivity.push_back({19,        55,        51,        72});
    tConnectivity.push_back({11,         9,        28,        72});
    tConnectivity.push_back({18,        73,        15,        74});
    tConnectivity.push_back({13,        53,        48,        68});
    tConnectivity.push_back({33,        35,        12,        67});
    tConnectivity.push_back({19,        52,        55,        72});
    tConnectivity.push_back({31,        57,        41,        59});
    tConnectivity.push_back({26,        50,         4,        71});
    tConnectivity.push_back({37,        34,        43,        58});
    tConnectivity.push_back({55,        67,        65,        73});

    // shift indices down one
    for(int j = 0; j < (int) tConnectivity.size(); ++j)
    {
        auto tet = tConnectivity[j];
        for(int i = 0; i < 4; ++i)
        {
            tet[i] -= 1;
        }
        tConnectivity[j] = tet;
    }

    std::vector<double> tData;

    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(1.0000000E+00);
    tData.push_back(1.0000000E+00);
    tData.push_back(1.0000000E+00);
    tData.push_back(1.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(1.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(1.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(6.2942016E-01);
    tData.push_back(1.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(1.8605334E-01);
    tData.push_back(2.8479974E-01);
    tData.push_back(0.0000000E+00);
    tData.push_back(5.5769735E-01);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(1.5085389E-03);
    tData.push_back(0.0000000E+00);
    tData.push_back(2.6471438E-01);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(7.0636678E-01);
    tData.push_back(2.1976224E-01);
    tData.push_back(0.0000000E+00);
    tData.push_back(8.6950405E-03);
    tData.push_back(1.2989087E-01);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(3.8743692E-02);
    tData.push_back(0.0000000E+00);
    tData.push_back(2.2710908E-01);
    tData.push_back(9.4803749E-02);
    tData.push_back(0.0000000E+00);
    tData.push_back(5.8701701E-01);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(6.2944940E-01);
    tData.push_back(1.0285348E-01);
    tData.push_back(0.0000000E+00);

    example::Interface_ParallelVector tDensityVector(tData);
    TetMeshUtilities tTetUtilities2(tCoordinates,tConnectivity);

    double tTargetEdgeLength2 = 1;

    Vector tMaxUVWCoords2, tMinUVWCoords2;
    tTetUtilities2.computeBoundingBox(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords2,tMinUVWCoords2);

    OrthogonalGridUtilities tGridUtilities2(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords2,tMinUVWCoords2,tTargetEdgeLength2);

    double tPNorm = 200;

    AMFilterUtilities tAMFilterUtilities2(tTetUtilities2,tGridUtilities2, tPNorm);

    std::vector<double> tGridBlueprintDensity;
    tAMFilterUtilities2.computeGridBlueprintDensity(&tDensityVector,tGridBlueprintDensity);

    std::vector<double> tGoldDensity;

    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(5.79696540572970619e-03);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(1.00000000000000000e+00);
    tGoldDensity.push_back(1.51312955027044477e-01);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(1.00000000000000000e+00);
    tGoldDensity.push_back(1.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(1.04781779238341099e-03);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(1.00000000000000000e+00);
    tGoldDensity.push_back(2.23678171091486522e-01);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(1.00000000000000000e+00);
    tGoldDensity.push_back(1.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);

    EXPECT_EQ(tGridBlueprintDensity.size(), tGoldDensity.size());

    for(int i = 0; i < (int) tGoldDensity.size(); ++i)
    {
        EXPECT_DOUBLE_EQ(tGridBlueprintDensity[i], tGoldDensity[i]);
    }
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
    std::vector<size_t> tGridDimensions = tGridUtilities.getGridDimensions();
    std::vector<Vector> tGridCoordinates;
    tGridUtilities.computeGridXYZCoordinates(tGridCoordinates);

    double tPNorm = 200;

    AMFilterUtilities tAMFilterUtilities(tTetUtilities,tGridUtilities, tPNorm);

    example::Interface_ParallelVector tVector({1,1,1,0});

    for(size_t i = 0; i < tGridDimensions[0]; ++i)
    {
        for(size_t j = 0; j < tGridDimensions[1]; ++j)
        {
            for(size_t k = 0; k < tGridDimensions[2]; ++k)
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

PSL_TEST(AMFilterUtilities, computeGridPrintableDensity)
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
    std::vector<size_t> tGridDimensions = tGridUtilities.getGridDimensions();

    double tPNorm = 200;

    AMFilterUtilities tAMFilterUtilities(tTetUtilities,tGridUtilities,tPNorm);

    example::Interface_ParallelVector tVector({1,1,0,1});

    std::vector<double> tGridBlueprintDensity;
    tAMFilterUtilities.computeGridBlueprintDensity(&tVector,tGridBlueprintDensity); 

    std::vector<double> tGridSupportDensity(tGridBlueprintDensity.size());
    std::vector<double> tGridPrintableDensity(tGridBlueprintDensity.size());

    // wrong size vectors
    std::vector<double> tBogusVector(3);
    EXPECT_THROW(tAMFilterUtilities.computeGridLayerSupportDensity(0,tBogusVector,tGridSupportDensity),std::domain_error);
    EXPECT_THROW(tAMFilterUtilities.computeGridLayerSupportDensity(0,tGridPrintableDensity,tBogusVector),std::domain_error);

    EXPECT_THROW(tAMFilterUtilities.computeGridLayerPrintableDensity(0,tBogusVector,tGridSupportDensity,tGridPrintableDensity),std::domain_error);
    EXPECT_THROW(tAMFilterUtilities.computeGridLayerPrintableDensity(0,tGridBlueprintDensity,tBogusVector,tGridPrintableDensity),std::domain_error);
    EXPECT_THROW(tAMFilterUtilities.computeGridLayerPrintableDensity(0,tGridBlueprintDensity,tGridSupportDensity,tBogusVector),std::domain_error);

    for(size_t k = 0; k < tGridDimensions[2]; ++k)
    {
        tAMFilterUtilities.computeGridLayerSupportDensity(k,tGridPrintableDensity,tGridSupportDensity);
        tAMFilterUtilities.computeGridLayerPrintableDensity(k,tGridBlueprintDensity,tGridSupportDensity,tGridPrintableDensity);

        for(size_t i = 0; i < tGridDimensions[0]; ++i)
        {
            for(size_t j = 0; j < tGridDimensions[1]; ++j)
            {
                size_t tSerializedIndex = tGridUtilities.getSerializedIndex(i,j,k);
                if(k == 0)
                {
                    EXPECT_DOUBLE_EQ(tGridSupportDensity[tSerializedIndex],1.0);
                }
                else
                {
                    std::vector<std::vector<size_t>> tSupportIndices = tGridUtilities.getSupportIndices(i,j,k);
                    std::vector<double> tSupportDensities;
                    for(auto tSupportIndex : tSupportIndices)
                    {
                        size_t tSupportSerializedIndex = tGridUtilities.getSerializedIndex(tSupportIndex);
                        tSupportDensities.push_back(tGridPrintableDensity[tSupportSerializedIndex]);
                    }
                    EXPECT_DOUBLE_EQ(tGridSupportDensity[tSerializedIndex],smax(tSupportDensities,tPNorm));
                }
                EXPECT_DOUBLE_EQ(tGridPrintableDensity[tSerializedIndex],smin(tGridBlueprintDensity[tSerializedIndex],tGridSupportDensity[tSerializedIndex]));
            }
        }
    }
}

PSL_TEST(AMFilterUtilities, computeTetNodePrintableDensity)
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

    Vector tMaxUVWCoords({2.0,2.0,2.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    double tTargetEdgeLength = 2.0;

    OrthogonalGridUtilities tGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    std::vector<size_t> tGridDimensions = tGridUtilities.getGridDimensions();

    double tPNorm = 200;

    AMFilterUtilities tAMFilterUtilities(tTetUtilities,tGridUtilities,tPNorm);

    std::vector<double> tGridPrintableDensity({1,2,3,4,5,6,7,8});

    EXPECT_THROW(tAMFilterUtilities.computeTetNodePrintableDensity(0,{0.0,0.0}),std::domain_error);
    EXPECT_THROW(tAMFilterUtilities.computeTetNodePrintableDensity(-1,tGridPrintableDensity),std::out_of_range);
    EXPECT_THROW(tAMFilterUtilities.computeTetNodePrintableDensity(4,tGridPrintableDensity),std::out_of_range);
    EXPECT_NO_THROW(tAMFilterUtilities.computeTetNodePrintableDensity(0,tGridPrintableDensity));

    EXPECT_EQ(tAMFilterUtilities.computeTetNodePrintableDensity(0,tGridPrintableDensity),1);
    EXPECT_EQ(tAMFilterUtilities.computeTetNodePrintableDensity(1,tGridPrintableDensity),1.5);
    EXPECT_EQ(tAMFilterUtilities.computeTetNodePrintableDensity(2,tGridPrintableDensity),2);
    EXPECT_EQ(tAMFilterUtilities.computeTetNodePrintableDensity(3,tGridPrintableDensity),3);
}

PSL_TEST(AMFilterUtilities, computeTetMeshPrintableDensity_oneElement)
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

    Vector tMaxUVWCoords({2.0,2.0,2.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    double tTargetEdgeLength = 2.0;

    OrthogonalGridUtilities tGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    std::vector<size_t> tGridDimensions = tGridUtilities.getGridDimensions();

    double tPNorm = 200;

    AMFilterUtilities tAMFilterUtilities(tTetUtilities,tGridUtilities,tPNorm);

    std::vector<double> tGridPrintableDensity({1,2,3,4,5,6,7,8});
    example::Interface_ParallelVector tVector({0,0,0,0});

    // wrong size of output density vector
    example::Interface_ParallelVector tBogusVector({1,1,1});
    EXPECT_THROW(tAMFilterUtilities.computeTetMeshPrintableDensity(tGridPrintableDensity,&tBogusVector),std::domain_error);

    // wrong size of grid printable density vector
    EXPECT_THROW(tAMFilterUtilities.computeTetMeshPrintableDensity({0.0,0.0},&tVector),std::domain_error);

    EXPECT_NO_THROW(tAMFilterUtilities.computeTetMeshPrintableDensity(tGridPrintableDensity,&tVector));

    EXPECT_EQ(tVector.get_value(0),1);
    EXPECT_EQ(tVector.get_value(1),1.5);
    EXPECT_EQ(tVector.get_value(2),2);
    EXPECT_EQ(tVector.get_value(3),3);
}

PSL_TEST(AMFilterUtilities, computeTetMeshPrintableDensity_oneElement_differentBasis)
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
    Vector tUBasisVector({0.0,0.0,1.0});
    Vector tVBasisVector({1.0,0.0,0.0});
    Vector tWBasisVector({0.0,1.0,0.0});

    Vector tMaxUVWCoords({2.0,2.0,2.0});
    Vector tMinUVWCoords({0.0,0.0,0.0});

    double tTargetEdgeLength = 2.0;

    OrthogonalGridUtilities tGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);
    std::vector<size_t> tGridDimensions = tGridUtilities.getGridDimensions();

    double tPNorm = 200;

    AMFilterUtilities tAMFilterUtilities(tTetUtilities,tGridUtilities,tPNorm);

    std::vector<double> tGridPrintableDensity({1,5,2,6,3,7,4,8});
    example::Interface_ParallelVector tVector({0,0,0,0});

    // wrong size of output density vector
    example::Interface_ParallelVector tBogusVector({1,1,1});
    EXPECT_THROW(tAMFilterUtilities.computeTetMeshPrintableDensity(tGridPrintableDensity,&tBogusVector),std::domain_error);

    // wrong size of grid printable density vector
    EXPECT_THROW(tAMFilterUtilities.computeTetMeshPrintableDensity({0.0,0.0},&tVector),std::domain_error);

    EXPECT_NO_THROW(tAMFilterUtilities.computeTetMeshPrintableDensity(tGridPrintableDensity,&tVector));

    EXPECT_EQ(tVector.get_value(0),1);
    EXPECT_EQ(tVector.get_value(1),1.5);
    EXPECT_EQ(tVector.get_value(2),2);
    EXPECT_EQ(tVector.get_value(3),3);
}

PSL_TEST(AMFilterUtilities, computeTetMeshPrintableDensity_manyElements)
{
    // build OrthogonalGridUtilities
    Vector tUBasisVector({1.0,0.0,0.0});
    Vector tVBasisVector({0.0,1.0,0.0});
    Vector tWBasisVector({0.0,0.0,1.0});

    std::vector<std::vector<double>> tCoordinates;
    std::vector<std::vector<int>> tConnectivity;

    tCoordinates.push_back({1.0000000E+00,  -2.0000000E+00,   5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,  -2.0000000E+00,   5.0000000E-01});
    tCoordinates.push_back({0.0000000E+00,  -2.0000000E+00,   5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,  -2.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,  -2.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({0.0000000E+00,  -2.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,   2.0000000E+00 ,  5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,   2.0000000E+00,  -5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,   2.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({0.0000000E+00,   2.0000000E+00,  -5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,   2.0000000E+00 ,  5.0000000E-01});
    tCoordinates.push_back({0.0000000E+00,   2.0000000E+00 ,  5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,  -1.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,   0.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,   1.0000000E+00 , -5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,  -1.0000000E+00 ,  5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,   0.0000000E+00 ,  5.0000000E-01});
    tCoordinates.push_back({1.0000000E+00,   1.0000000E+00,   5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,   1.0000000E+00,  -5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,   0.0000000E+00,  -5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,  -1.0000000E+00,  -5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,   1.0000000E+00,   5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,   0.0000000E+00,   5.0000000E-01});
    tCoordinates.push_back({-1.0000000E+00,  -1.0000000E+00 ,  5.0000000E-01});
    tCoordinates.push_back({3.4649723E-01,  -2.0000000E+00,   6.6227364E-04});
    tCoordinates.push_back({-4.5755927E-01,  -2.0000000E+00 , -1.3018114E-03});
    tCoordinates.push_back({3.7057984E-01,   2.0000000E+00,  -1.0030927E-03});
    tCoordinates.push_back({-4.8425606E-01,   2.0000000E+00 , -1.2747575E-01});
    tCoordinates.push_back({3.7704415E-01,  -1.2851638E+00,   5.0000000E-01});
    tCoordinates.push_back({-3.8986427E-01,  -1.3331544E+00 ,  5.0000000E-01});
    tCoordinates.push_back({4.7721431E-01,  -5.6466477E-01,   5.0000000E-01});
    tCoordinates.push_back({-4.5127513E-01,   4.1228361E-01 ,  5.0000000E-01});
    tCoordinates.push_back({3.3426469E-01,   1.4277973E+00 ,  5.0000000E-01});
    tCoordinates.push_back({4.9931655E-01,   3.5993220E-01,   5.0000000E-01});
    tCoordinates.push_back({-4.1495173E-01,   1.2439895E+00,   5.0000000E-01});
    tCoordinates.push_back({-7.1110536E-01,  -5.0176541E-01 ,  5.0000000E-01});
    tCoordinates.push_back({1.2718026E-01,   6.9585179E-01 ,  5.0000000E-01});
    tCoordinates.push_back({1.3835560E-02,  -8.1731689E-01,   5.0000000E-01});
    tCoordinates.push_back({-4.3732245E-01,  -8.4620111E-01,   5.0000000E-01});
    tCoordinates.push_back({-4.2138274E-01,  -1.2413886E-01 ,  5.0000000E-01});
    tCoordinates.push_back({3.7674078E-01,  -1.0905474E-01,   5.0000000E-01});
    tCoordinates.push_back({-3.0176796E-01,  -5.0405007E-01,   5.0000000E-01});
    tCoordinates.push_back({-1.1463497E-02,   1.3159500E-01,   5.0000000E-01});
    tCoordinates.push_back({-7.2702803E-03,  -3.1737371E-01 ,  5.0000000E-01});
    tCoordinates.push_back({3.0368259E-01,   1.3801634E+00 , -5.0000000E-01});
    tCoordinates.push_back({3.2007693E-01,   4.9573262E-01,  -5.0000000E-01});
    tCoordinates.push_back({-3.2297714E-01,  -5.9135782E-01 , -5.0000000E-01});
    tCoordinates.push_back({3.8761751E-01,  -1.4761064E+00 , -5.0000000E-01});
    tCoordinates.push_back({3.6181753E-01,  -3.8191212E-01,  -5.0000000E-01});
    tCoordinates.push_back({-3.4925889E-01,  -1.3889216E+00,  -5.0000000E-01});
    tCoordinates.push_back({-4.5864357E-01,   1.4575972E+00,  -5.0000000E-01});
    tCoordinates.push_back({-4.3492309E-01,   5.0528988E-01 , -5.0000000E-01});
    tCoordinates.push_back({2.1925244E-01,  -9.6959312E-01,  -5.0000000E-01});
    tCoordinates.push_back({-1.7417699E-01,   4.9920719E-02,  -5.0000000E-01});
    tCoordinates.push_back({-1.3999204E-01,   9.2784471E-01,  -5.0000000E-01});
    tCoordinates.push_back({-1.1241963E-01,  -4.7770185E-01 ,  2.4559946E-02});
    tCoordinates.push_back({1.3403264E-01,  -4.8723804E-01 ,  3.1963932E-01});
    tCoordinates.push_back({1.6589050E-01,   4.5969729E-02 ,  1.1225805E-02});
    tCoordinates.push_back({6.0420996E-01,  -1.5343626E-01 ,  9.1492117E-02});
    tCoordinates.push_back({2.5935486E-01,  -6.8884358E-01,   9.2958968E-02});
    tCoordinates.push_back({-3.6511753E-01,  -2.8768576E-01,  -1.1661862E-01});
    tCoordinates.push_back({-3.0765422E-01,   1.2593259E-01,   1.8706483E-01});
    tCoordinates.push_back({-2.8055589E-02,  -1.1333659E+00,  -6.1686190E-02});
    tCoordinates.push_back({-5.4833900E-01,   4.1417642E-01,  -8.4496843E-03});
    tCoordinates.push_back({-1.5004674E-01,   6.3182813E-01 ,  4.8866220E-02});
    tCoordinates.push_back({4.5369823E-01,  -7.3889238E-01,  -1.7314432E-01});
    tCoordinates.push_back({-1.6154908E-02,   1.5802358E+00 , -9.3357940E-02});
    tCoordinates.push_back({4.4728267E-01,  -1.3645467E+00,   3.6961955E-02});
    tCoordinates.push_back({-3.0335394E-02,  -1.5347573E+00,  -2.1365023E-02});
    tCoordinates.push_back({-5.7201740E-01,  -1.0118763E+00,   9.6961621E-02});
    tCoordinates.push_back({-5.0626935E-01,  -1.4464236E+00,  -4.0889135E-02});
    tCoordinates.push_back({-5.1374269E-01,   1.2379537E+00 , -7.3816009E-02});
    tCoordinates.push_back({3.8616245E-01,   1.1675581E+00 ,  2.3536345E-02});
    tCoordinates.push_back({4.8247555E-01,   5.3747499E-01 ,  7.3740368E-02});

    tConnectivity.push_back({38,        56,        42,        57});
    tConnectivity.push_back({44,        42,        56,        57});
    tConnectivity.push_back({8,        27,        10,        45});
    tConnectivity.push_back({44,        57,        56,        58});
    tConnectivity.push_back({41,        57,        58,        59});
    tConnectivity.push_back({57,        58,        59,        60});
    tConnectivity.push_back({31,        38,        57,        60});
    tConnectivity.push_back({44,        41,        57,        58});
    tConnectivity.push_back({35,        11,        12,        28});
    tConnectivity.push_back({40,        56,        61,        62});
    tConnectivity.push_back({30,        39,        38,        63});
    tConnectivity.push_back({23,        62,        61,        64});
    tConnectivity.push_back({23,        32,        62,        64});
    tConnectivity.push_back({32,        62,        64,        65});
    tConnectivity.push_back({44,        43,        41,        58});
    tConnectivity.push_back({44,        43,        58,        62});
    tConnectivity.push_back({16,        60,        59,        66});
    tConnectivity.push_back({16,        59,        14,        66});
    tConnectivity.push_back({33,        12,        27,        67});
    tConnectivity.push_back({49,        60,        53,        66});
    tConnectivity.push_back({53,        66,        60,        68});
    tConnectivity.push_back({14,        59,        49,        66});
    tConnectivity.push_back({48,        68,        63,        69});
    tConnectivity.push_back({25,        29,        68,        69});
    tConnectivity.push_back({25,        68,        48,        69});
    tConnectivity.push_back({30,        24,        70,        71});
    tConnectivity.push_back({21,        50,        70,        71});
    tConnectivity.push_back({50,        69,        63,        71});
    tConnectivity.push_back({30,        70,        63,        71});
    tConnectivity.push_back({1,        29,        16,        68});
    tConnectivity.push_back({5,        13,        48,        68});
    tConnectivity.push_back({26,        30,        69,        71});
    tConnectivity.push_back({29,        30,        63,        69});
    tConnectivity.push_back({29,        63,        68,        69});
    tConnectivity.push_back({53,        60,        63,        68});
    tConnectivity.push_back({6,        48,        50,        69});
    tConnectivity.push_back({25,         5,        48,        68});
    tConnectivity.push_back({10,        51,        45,        67});
    tConnectivity.push_back({42,        39,        36,        56});
    tConnectivity.push_back({50,        63,        47,        70});
    tConnectivity.push_back({47,        63,        56,        70});
    tConnectivity.push_back({30,        39,        63,        70});
    tConnectivity.push_back({36,        24,        21,        70});
    tConnectivity.push_back({30,        63,        69,        71});
    tConnectivity.push_back({29,        60,        16,        68});
    tConnectivity.push_back({53,        56,        47,        63});
    tConnectivity.push_back({2,        24,        30,        71});
    tConnectivity.push_back({30,        24,        39,        70});
    tConnectivity.push_back({29,        38,        60,        63});
    tConnectivity.push_back({28,        51,        10,        67});
    tConnectivity.push_back({27,        28,        10,        67});
    tConnectivity.push_back({35,        28,        67,        72});
    tConnectivity.push_back({33,        67,        27,        73});
    tConnectivity.push_back({33,        35,        67,        73});
    tConnectivity.push_back({14,        46,        59,        74});
    tConnectivity.push_back({40,        36,        23,        61});
    tConnectivity.push_back({46,        58,        59,        74});
    tConnectivity.push_back({34,        17,        59,        74});
    tConnectivity.push_back({37,        73,        18,        74});
    tConnectivity.push_back({35,        32,        22,        65});
    tConnectivity.push_back({48,        53,        63,        68});
    tConnectivity.push_back({6,        50,        26,        69});
    tConnectivity.push_back({16,        60,        66,        68});
    tConnectivity.push_back({53,        60,        56,        63});
    tConnectivity.push_back({38,        56,        60,        63});
    tConnectivity.push_back({56,        58,        57,        60});
    tConnectivity.push_back({15,         8,        45,        73});
    tConnectivity.push_back({31,        57,        59,        60});
    tConnectivity.push_back({37,        65,        35,        73});
    tConnectivity.push_back({45,        67,        55,        73});
    tConnectivity.push_back({35,        22,        11,        72});
    tConnectivity.push_back({28,         9,        51,        72});
    tConnectivity.push_back({35,        67,        65,        72});
    tConnectivity.push_back({35,        65,        22,        72});
    tConnectivity.push_back({49,        58,        56,        60});
    tConnectivity.push_back({34,        41,        58,        59});
    tConnectivity.push_back({54,        61,        62,        64});
    tConnectivity.push_back({49,        46,        58,        59});
    tConnectivity.push_back({35,        65,        67,        73});
    tConnectivity.push_back({51,        55,        67,        72});
    tConnectivity.push_back({19,        64,        52,        72});
    tConnectivity.push_back({37,        35,        33,        73});
    tConnectivity.push_back({32,        43,        62,        65});
    tConnectivity.push_back({22,        65,        64,        72});
    tConnectivity.push_back({22,        64,        19,        72});
    tConnectivity.push_back({54,        58,        46,        65});
    tConnectivity.push_back({52,        64,        54,        65});
    tConnectivity.push_back({32,        64,        22,        65});
    tConnectivity.push_back({12,        28,        27,        67});
    tConnectivity.push_back({54,        64,        62,        65});
    tConnectivity.push_back({40,        61,        23,        62});
    tConnectivity.push_back({52,        54,        46,        65});
    tConnectivity.push_back({20,        61,        54,        64});
    tConnectivity.push_back({36,        21,        23,        61});
    tConnectivity.push_back({22,        20,        19,        64});
    tConnectivity.push_back({36,        61,        56,        70});
    tConnectivity.push_back({54,        47,        56,        61});
    tConnectivity.push_back({23,        61,        20,        64});
    tConnectivity.push_back({42,        36,        40,        56});
    tConnectivity.push_back({44,        42,        40,        56});
    tConnectivity.push_back({49,        56,        53,        60});
    tConnectivity.push_back({38,        56,        57,        60});
    tConnectivity.push_back({3,        30,        29,        69});
    tConnectivity.push_back({39,        56,        63,        70});
    tConnectivity.push_back({23,        22,        32,        64});
    tConnectivity.push_back({20,        47,        54,        61});
    tConnectivity.push_back({44,        40,        43,        62});
    tConnectivity.push_back({11,        19,         9,        72});
    tConnectivity.push_back({31,        17,        16,        59});
    tConnectivity.push_back({49,        59,        60,        66});
    tConnectivity.push_back({31,        44,        41,        57});
    tConnectivity.push_back({34,        59,        58,        74});
    tConnectivity.push_back({17,        14,        59,        74});
    tConnectivity.push_back({33,        12,         7,        27});
    tConnectivity.push_back({37,        65,        73,        74});
    tConnectivity.push_back({18,        33,         7,        73});
    tConnectivity.push_back({52,        65,        55,        72});
    tConnectivity.push_back({33,        27,         7,        73});
    tConnectivity.push_back({23,        20,        22,        64});
    tConnectivity.push_back({9,        19,        51,        72});
    tConnectivity.push_back({29,        30,        38,        63});
    tConnectivity.push_back({29,        31,        16,        60});
    tConnectivity.push_back({13,        66,        53,        68});
    tConnectivity.push_back({1,        16,        13,        68});
    tConnectivity.push_back({16,        17,        14,        59});
    tConnectivity.push_back({45,        51,        55,        67});
    tConnectivity.push_back({15,        45,        46,        73});
    tConnectivity.push_back({55,        65,        67,        72});
    tConnectivity.push_back({50,        63,        70,        71});
    tConnectivity.push_back({24,        21,        70,        71});
    tConnectivity.push_back({25,         3,        29,        69});
    tConnectivity.push_back({45,        55,        46,        73});
    tConnectivity.push_back({37,        32,        35,        65});
    tConnectivity.push_back({14,        46,        49,        59});
    tConnectivity.push_back({37,        58,        43,        65});
    tConnectivity.push_back({37,        43,        32,        65});
    tConnectivity.push_back({31,        59,        16,        60});
    tConnectivity.push_back({34,        18,        17,        74});
    tConnectivity.push_back({13,        49,        53,        66});
    tConnectivity.push_back({47,        56,        61,        70});
    tConnectivity.push_back({48,        53,        50,        63});
    tConnectivity.push_back({26,        69,        50,        71});
    tConnectivity.push_back({18,         7,         8,        73});
    tConnectivity.push_back({52,        64,        65,        72});
    tConnectivity.push_back({11,        22,        19,        72});
    tConnectivity.push_back({28,         9,        10,        51});
    tConnectivity.push_back({38,        39,        56,        63});
    tConnectivity.push_back({26,        30,         3,        69});
    tConnectivity.push_back({35,        11,        28,        72});
    tConnectivity.push_back({43,        58,        62,        65});
    tConnectivity.push_back({29,        38,        31,        60});
    tConnectivity.push_back({31,        38,        44,        57});
    tConnectivity.push_back({4,        24,         2,        71});
    tConnectivity.push_back({36,        56,        39,        70});
    tConnectivity.push_back({4,        50,        21,        71});
    tConnectivity.push_back({25,        48,         6,        69});
    tConnectivity.push_back({7,        27,         8,        73});
    tConnectivity.push_back({23,        21,        20,        61});
    tConnectivity.push_back({49,        47,        53,        56});
    tConnectivity.push_back({4,        21,        24,        71});
    tConnectivity.push_back({18,        15,        17,        74});
    tConnectivity.push_back({31,        41,        17,        59});
    tConnectivity.push_back({16,        14,        13,        66});
    tConnectivity.push_back({54,        47,        49,        56});
    tConnectivity.push_back({49,        59,        58,        60});
    tConnectivity.push_back({40,        56,        36,        61});
    tConnectivity.push_back({54,        62,        58,        65});
    tConnectivity.push_back({37,        34,        58,        74});
    tConnectivity.push_back({17,        15,        14,        74});
    tConnectivity.push_back({54,        49,        46,        58});
    tConnectivity.push_back({54,        56,        58,        62});
    tConnectivity.push_back({26,         2,         3,        30});
    tConnectivity.push_back({1,        13,         5,        68});
    tConnectivity.push_back({6,         4,        26,        50});
    tConnectivity.push_back({50,        53,        47,        63});
    tConnectivity.push_back({48,        63,        50,        69});
    tConnectivity.push_back({21,        47,        61,        70});
    tConnectivity.push_back({1,        25,        29,        68});
    tConnectivity.push_back({34,        17,        41,        59});
    tConnectivity.push_back({44,        58,        56,        62});
    tConnectivity.push_back({46,        65,        58,        74});
    tConnectivity.push_back({20,        52,        19,        64});
    tConnectivity.push_back({35,        28,        12,        67});
    tConnectivity.push_back({14,        15,        46,        74});
    tConnectivity.push_back({37,        33,        18,        73});
    tConnectivity.push_back({14,        49,        13,        66});
    tConnectivity.push_back({44,        56,        40,        62});
    tConnectivity.push_back({37,        58,        65,        74});
    tConnectivity.push_back({20,        54,        52,        64});
    tConnectivity.push_back({40,        23,        32,        62});
    tConnectivity.push_back({18,         8,        15,        73});
    tConnectivity.push_back({38,        39,        42,        56});
    tConnectivity.push_back({36,        21,        61,        70});
    tConnectivity.push_back({52,        46,        55,        65});
    tConnectivity.push_back({16,        66,        13,        68});
    tConnectivity.push_back({46,        55,        65,        73});
    tConnectivity.push_back({34,        41,        43,        58});
    tConnectivity.push_back({15,        73,        46,        74});
    tConnectivity.push_back({44,        38,        42,        57});
    tConnectivity.push_back({25,         5,         6,        48});
    tConnectivity.push_back({1,        25,         3,        29});
    tConnectivity.push_back({26,         4,         2,        71});
    tConnectivity.push_back({36,        24,        23,        21});
    tConnectivity.push_back({27,        10,        45,        67});
    tConnectivity.push_back({46,        73,        65,        74});
    tConnectivity.push_back({54,        56,        49,        58});
    tConnectivity.push_back({29,        63,        60,        68});
    tConnectivity.push_back({25,        26,         3,        69});
    tConnectivity.push_back({36,        39,        24,        70});
    tConnectivity.push_back({20,        21,        47,        61});
    tConnectivity.push_back({54,        61,        56,        62});
    tConnectivity.push_back({40,        32,        43,        62});
    tConnectivity.push_back({8,        27,        45,        73});
    tConnectivity.push_back({25,         6,        26,        69});
    tConnectivity.push_back({26,         2,        30,        71});
    tConnectivity.push_back({1,         5,        25,        68});
    tConnectivity.push_back({21,        50,        47,        70});
    tConnectivity.push_back({27,        67,        45,        73});
    tConnectivity.push_back({28,        51,        67,        72});
    tConnectivity.push_back({37,        18,        34,        74});
    tConnectivity.push_back({19,        55,        51,        72});
    tConnectivity.push_back({11,         9,        28,        72});
    tConnectivity.push_back({18,        73,        15,        74});
    tConnectivity.push_back({13,        53,        48,        68});
    tConnectivity.push_back({33,        35,        12,        67});
    tConnectivity.push_back({19,        52,        55,        72});
    tConnectivity.push_back({31,        57,        41,        59});
    tConnectivity.push_back({26,        50,         4,        71});
    tConnectivity.push_back({37,        34,        43,        58});
    tConnectivity.push_back({55,        67,        65,        73});

    // shift indices down one
    for(int j = 0; j < (int) tConnectivity.size(); ++j)
    {
        auto tet = tConnectivity[j];
        for(int i = 0; i < 4; ++i)
        {
            tet[i] -= 1;
        }
        tConnectivity[j] = tet;
    }

    std::vector<double> tData;

    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(1.0000000E+00);
    tData.push_back(1.0000000E+00);
    tData.push_back(1.0000000E+00);
    tData.push_back(1.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(1.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(1.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(6.2942016E-01);
    tData.push_back(1.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(1.8605334E-01);
    tData.push_back(2.8479974E-01);
    tData.push_back(0.0000000E+00);
    tData.push_back(5.5769735E-01);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(1.5085389E-03);
    tData.push_back(0.0000000E+00);
    tData.push_back(2.6471438E-01);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(7.0636678E-01);
    tData.push_back(2.1976224E-01);
    tData.push_back(0.0000000E+00);
    tData.push_back(8.6950405E-03);
    tData.push_back(1.2989087E-01);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(3.8743692E-02);
    tData.push_back(0.0000000E+00);
    tData.push_back(2.2710908E-01);
    tData.push_back(9.4803749E-02);
    tData.push_back(0.0000000E+00);
    tData.push_back(5.8701701E-01);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(0.0000000E+00);
    tData.push_back(6.2944940E-01);
    tData.push_back(1.0285348E-01);
    tData.push_back(0.0000000E+00);

    example::Interface_ParallelVector tDensityVector(tData);
    TetMeshUtilities tTetUtilities(tCoordinates,tConnectivity);

    double tTargetEdgeLength = 1;

    Vector tMaxUVWCoords, tMinUVWCoords;
    tTetUtilities.computeBoundingBox(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords);

    OrthogonalGridUtilities tGridUtilities(tUBasisVector,tVBasisVector,tWBasisVector,tMaxUVWCoords,tMinUVWCoords,tTargetEdgeLength);

    double tPNorm = 200;

    AMFilterUtilities tAMFilterUtilities(tTetUtilities,tGridUtilities, tPNorm);

    std::vector<double> tGridDensity;

    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(5.79696540572970619e-03);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(1.00000000000000000e+00);
    tGridDensity.push_back(1.51312955027044477e-01);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(1.00000000000000000e+00);
    tGridDensity.push_back(1.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(1.04781779238341099e-03);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(1.00000000000000000e+00);
    tGridDensity.push_back(2.23678171091486522e-01);
    tGridDensity.push_back(0.00000000000000000e+00);
    tGridDensity.push_back(1.00000000000000000e+00);
    tGridDensity.push_back(1.00000000000000000e+00);
    tGridDensity.push_back(0.00000000000000000e+00);

    tAMFilterUtilities.computeTetMeshPrintableDensity(tGridDensity,&tDensityVector);

    std::vector<double> tGoldDensity;

    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(1.11022302462515654e-16);
    tGoldDensity.push_back(-1.11022302462515654e-16);
    tGoldDensity.push_back(9.99999999999999778e-01);
    tGoldDensity.push_back(1.00000000000000000e+00);
    tGoldDensity.push_back(9.99999999999999667e-01);
    tGoldDensity.push_back(9.99999999999999778e-01);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(1.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(1.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(6.29420159999999895e-01);
    tGoldDensity.push_back(1.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(2.38469737879460600e-04);
    tGoldDensity.push_back(2.36994028385453942e-01);
    tGoldDensity.push_back(3.70006733441293201e-01);
    tGoldDensity.push_back(4.06453075532847707e-02);
    tGoldDensity.push_back(6.56630809367280577e-01);
    tGoldDensity.push_back(1.50820066561421166e-04);
    tGoldDensity.push_back(1.36129808743503555e-01);
    tGoldDensity.push_back(1.88770229320298939e-04);
    tGoldDensity.push_back(9.06772952853735718e-05);
    tGoldDensity.push_back(5.31021874168152589e-04);
    tGoldDensity.push_back(5.81842582465505898e-04);
    tGoldDensity.push_back(3.62846865332052922e-04);
    tGoldDensity.push_back(3.15055398208369183e-02);
    tGoldDensity.push_back(7.10067773563093671e-04);
    tGoldDensity.push_back(3.30021521804534756e-01);
    tGoldDensity.push_back(5.29891165092542318e-02);
    tGoldDensity.push_back(1.60378901389091099e-03);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(2.28662952526737396e-03);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(7.50797270327565380e-01);
    tGoldDensity.push_back(2.64586804941578602e-01);
    tGoldDensity.push_back(1.37620523166387355e-04);
    tGoldDensity.push_back(1.94813033882427419e-02);
    tGoldDensity.push_back(2.50991352302035009e-01);
    tGoldDensity.push_back(1.53248611881657767e-03);
    tGoldDensity.push_back(8.45610397966129238e-04);
    tGoldDensity.push_back(9.90143616169406916e-03);
    tGoldDensity.push_back(1.00112422437710949e-03);
    tGoldDensity.push_back(6.86971378728080994e-04);
    tGoldDensity.push_back(1.79819394027674905e-03);
    tGoldDensity.push_back(5.78049624566033399e-02);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(2.62985155626895839e-01);
    tGoldDensity.push_back(1.98390815732072484e-01);
    tGoldDensity.push_back(6.05476178301867169e-04);
    tGoldDensity.push_back(6.61659464319707569e-01);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(0.00000000000000000e+00);
    tGoldDensity.push_back(6.96946632149682110e-01);
    tGoldDensity.push_back(1.99531019056064823e-01);
    tGoldDensity.push_back(5.43727922950622625e-02);


    EXPECT_EQ(tDensityVector.get_length(), tGoldDensity.size());

    for(int i = 0; i < (int) tGoldDensity.size(); ++i)
    {
        // printf("%.17e\n", tDensityVector.get_value(i));
        EXPECT_DOUBLE_EQ(tDensityVector.get_value(i), tGoldDensity[i]);
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

    tArgs.clear();

    tArgs.push_back(0);
    tArgs.push_back(0.1);
    tArgs.push_back(0.23);
    tArgs.push_back(0.48);
    tArgs.push_back(-0.71);
    tArgs.push_back(0.32);
    tArgs.push_back(0.22);
    tArgs.push_back(0.17);
    tArgs.push_back(1.0);
    tArgs.push_back(0.336);

    // negative argument
    EXPECT_THROW(smax(tArgs,tPNorm),std::domain_error);
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
