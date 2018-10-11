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

/*
 * Plato_Test_Geometry_MLS.cpp
 *
 *  Created on: Feb 27, 2018
 */

#include <gtest/gtest.h>
#include "XMLGenerator_UnitTester.hpp"
#include "Plato_Vector3DVariations.hpp"
#include <cmath>

const int MAX_CHARS_PER_LINE = 512;

namespace PlatoTestXMLGenerator
{
TEST(PlatoTestXMLGenerator, parseSingleValue)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tTokens;
    std::vector<std::string> tInputStringList;
    std::string tStringValue;
    bool tReturnValue;

    tReturnValue = tester.publicParseSingleValue(tTokens, tInputStringList = {"car", "truck", "bus"}, tStringValue);
    EXPECT_EQ(tReturnValue, false);
    tTokens.push_back("car");
    tTokens.push_back("truck");
    tTokens.push_back("bus");
    tReturnValue = tester.publicParseSingleValue(tTokens, tInputStringList = {}, tStringValue);
    EXPECT_EQ(tReturnValue, false);
    tStringValue = "";
    tReturnValue = tester.publicParseSingleValue(tTokens, tInputStringList = {"car"}, tStringValue);
    EXPECT_EQ(tReturnValue, true);
    EXPECT_EQ(tStringValue, "");
    tStringValue = "";
    tReturnValue = tester.publicParseSingleValue(tTokens, tInputStringList = {"car", "truck", "bus"}, tStringValue);
    EXPECT_EQ(tReturnValue, true);
    EXPECT_EQ(tStringValue, "");
    tReturnValue = tester.publicParseSingleValue(tTokens, tInputStringList = {"car", "bus"}, tStringValue);
    EXPECT_EQ(tReturnValue, false);
    tStringValue = "";
    tReturnValue = tester.publicParseSingleValue(tTokens, tInputStringList = {"car","truck"}, tStringValue);
    EXPECT_EQ(tReturnValue, true);
    EXPECT_EQ(tStringValue, "bus");
}
TEST(PlatoTestXMLGenerator, parseSingleUnLoweredValue)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tTokens;
    std::vector<std::string> tUnLoweredTokens;
    std::vector<std::string> tInputStringList;
    std::string tStringValue;
    bool tReturnValue;

    tReturnValue = tester.publicParseSingleUnLoweredValue(tTokens, tUnLoweredTokens, tInputStringList = {"car", "truck", "bus"}, tStringValue);
    EXPECT_EQ(tReturnValue, false);
    tTokens.push_back("car");
    tTokens.push_back("truck");
    tTokens.push_back("bus");
    tUnLoweredTokens.push_back("Car");
    tUnLoweredTokens.push_back("Truck");
    tUnLoweredTokens.push_back("Bus");
    tReturnValue = tester.publicParseSingleUnLoweredValue(tTokens, tUnLoweredTokens, tInputStringList = {}, tStringValue);
    EXPECT_EQ(tReturnValue, false);
    tStringValue = "";
    tReturnValue = tester.publicParseSingleUnLoweredValue(tTokens, tUnLoweredTokens, tInputStringList = {"car"}, tStringValue);
    EXPECT_EQ(tReturnValue, true);
    EXPECT_EQ(tStringValue, "");
    tStringValue = "";
    tReturnValue = tester.publicParseSingleUnLoweredValue(tTokens, tUnLoweredTokens, tInputStringList = {"car", "truck", "bus"}, tStringValue);
    EXPECT_EQ(tReturnValue, true);
    EXPECT_EQ(tStringValue, "");
    tStringValue = "";
    tReturnValue = tester.publicParseSingleUnLoweredValue(tTokens, tUnLoweredTokens, tInputStringList = {"car", "bus"}, tStringValue);
    EXPECT_EQ(tReturnValue, false);
    tStringValue = "";
    tReturnValue = tester.publicParseSingleUnLoweredValue(tTokens, tUnLoweredTokens, tInputStringList = {"car","truck"}, tStringValue);
    EXPECT_EQ(tReturnValue, true);
    EXPECT_EQ(tStringValue, "Bus");
}
TEST(PlatoTestXMLGenerator, parseTokens)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    char buffer[MAX_CHARS_PER_LINE];

    buffer[0] = '\0';
    tester.publicParseTokens(buffer, tokens);
    EXPECT_EQ((int)tokens.size(), 0);
    buffer[0] = '\t';
    buffer[1] = '\0';
    tester.publicParseTokens(buffer, tokens);
    EXPECT_EQ((int)tokens.size(), 0);
    strcpy(buffer, "   car");
    tester.publicParseTokens(buffer, tokens);
    EXPECT_EQ((int)tokens.size(), 1);
    tokens.clear();
    buffer[0] = '\t';
    strcpy(&(buffer[1]), "   car");
    tester.publicParseTokens(buffer, tokens);
    EXPECT_EQ((int)tokens.size(), 1);
    tokens.clear();
    strcpy(buffer, "   car\tbus   trike\t");
    tester.publicParseTokens(buffer, tokens);
    EXPECT_EQ((int)tokens.size(), 3);
    EXPECT_EQ(tokens[0], "car");
    EXPECT_EQ(tokens[1], "bus");
    EXPECT_EQ(tokens[2], "trike");
}

TEST(PlatoTestXMLGenerator, parseUncertainties)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    stringInput =
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 10\n"
            "    axis X\n"
            "    distribution beta\n"
            "    mean 0\n"
            "    upper bound 5\n"
            "    lower bound -5\n"
            "    standard deviation 1\n"
            "    num samples 5\n"
            "end uncertainty\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseUncertainties(iss), true);
    stringInput =
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 10\n"
            "    axis X\n"
            "    distribution beta\n"
            "    mean 0\n"
            "    upper bound 5\n"
            "    lower -5\n"
            "    standard deviation 1\n"
            "    num samples 5\n"
            "end uncertainty\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseUncertainties(iss), false);
    stringInput =
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 10\n"
            "    axis bad\n"
            "    distribution uniform\n"
            "    upper bound 5\n"
            "    lower bound -5\n"
            "    num samples 5\n"
            "end uncertainty\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseUncertainties(iss), false);
    stringInput =
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 10\n"
            "    axis Y\n"
            "    distribution beta\n"
            "    mean 0\n"
            "    upper bound 5\n"
            "    lower bound -5\n"
            "    std deviation 1\n"
            "    num samples 5\n"
            "end uncertainty\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseUncertainties(iss), false);
    stringInput =
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 10\n"
            "    axis X\n"
            "    distribution beta\n"
            "    mean 0\n"
            "    upper bound 5\n"
            "    lower bound -5\n"
            "    standard deviation 1\n"
            "    num samples 5\n"
            "end uncertainty\n"
            "\n"
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 12\n"
            "    axis Y\n"
            "    distribution normal\n"
            "    mean 0\n"
            "    standard deviation 4\n"
            "    num samples 11\n"
            "end uncertainty\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseUncertainties(iss), true);
    stringInput =
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 10\n"
            "    axis X\n"
            "    distribution beta\n"
            "    upper bound 5\n"
            "    lower bound -5\n"
            "    standard deviation 1\n"
            "    num samples 5\n"
            "end uncertainty\n"
            "\n"
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 12\n"
            "    axis Y\n"
            "    distribution normal\n"
            "    mean 0\n"
            "    standard deviation 4\n"
            "    num samples 11\n"
            "end uncertainty\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseUncertainties(iss), false);
    stringInput =
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 10\n"
            "    axis X\n"
            "    distribution normal\n"
            "    mean 0\n"
            "    standard deviation 1\n"
            "    num samples 5\n"
            "end uncertainty\n"
            "\n"
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 12\n"
            "    axis Z\n"
            "    distribution beta\n"
            "    mean 0\n"
            "    upper bound 8\n"
            "    lower bound -2\n"
            "    standard deviation 4\n"
            "    num samples 11\n"
            "end uncertainty\n"
            "\n"
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 15\n"
            "    axis Y\n"
            "    distribution uniform\n"
            "    upper bound 3\n"
            "    lower bound -1\n"
            "    num samples 8\n"
            "end uncertainty\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseUncertainties(iss), true);
}

TEST(PlatoTestXMLGenerator, parseOptimizationParameters)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    // material_box
    stringInput = "begin optimization parameters\n"
            "begin material_box\n"
            "min coords 0 0 0\n"
            "end material_box\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "begin material_box\n"
            "max coords 0 0 0\n"
            "end material_box\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "begin material_box\n"
            "min coords 0 0 1 \n"
            "max coords 0 0 \n"
            "end material_box\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "begin material_box\n"
            "min coords 0 0  \n"
            "max coords 0 0 1\n"
            "end material_box\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "begin material_box\n"
            "min coords 1 2 3 \n"
            "max coords 4 5 6\n"
            "end material_box\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getMatBoxMinCoords(), "1 2 3");
    EXPECT_EQ(tester.getMatBoxMaxCoords(), "4 5 6");

    // Invalid keywords
    stringInput = "begin optimization parameters\n"
            "hippo\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);

    // initial density value
    stringInput = "begin optimization parameters\n"
            "initial density value\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "initial density value .1\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getInitDensityValue(), ".1");

    // create levelset spheres
    stringInput = "begin optimization parameters\n"
            "create levelset spheres\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "create levelset spheres true\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getCreateLevelsetSpheres(), "true");

    // levelset initialization method
    stringInput = "begin optimization parameters\n"
            "levelset initialization method\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "levelset initialization method primitives\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getLevelsetInitMethod(), "primitives");

    // max iterations
    stringInput = "begin optimization parameters\n"
            "max iterations\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "max iterations 45\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getMaxIterations(), "45");

    // restart iteration
    stringInput = "begin optimization parameters\n"
            "restart iteration\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "restart iteration 22\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getRestartIteration(), "1");
    stringInput = "begin optimization parameters\n"
            "initial guess filename file.gen\n"
            "initial guess field name topology\n"
            "restart iteration 22\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getRestartIteration(), "22");
    EXPECT_EQ(tester.getRestartFieldName(), "topology");
    EXPECT_EQ(tester.getRestartMeshFilename(), "file.gen");
    stringInput = "begin optimization parameters\n"
            "initial guess filename fIle.gen\n"
            "initial guess field name TopologY\n"
            "restart iteration 22\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getRestartIteration(), "22");
    EXPECT_EQ(tester.getRestartFieldName(), "TopologY");
    EXPECT_EQ(tester.getRestartMeshFilename(), "fIle.gen");

    // ks max trust region iterations
    stringInput = "begin optimization parameters\n"
            "ks max trust region iterations\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks max trust region iterations 10\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSMaxTrustIterations(), "10");

    // ks trust region expansion factor
    stringInput = "begin optimization parameters\n"
            "ks trust region expansion factor\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks trust region expansion factor .5\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSExpansionFactor(), ".5");

    // ks trust region contraction factor
    stringInput = "begin optimization parameters\n"
            "ks trust region contraction factor\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks trust region contraction factor .2\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSContractionFactor(), ".2");

    // ks outer gradient tolerance
    stringInput = "begin optimization parameters\n"
            "ks outer gradient tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks outer gradient tolerance .001\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterGradientTolerance(), ".001");

    // ks outer stationarity tolerance
    stringInput = "begin optimization parameters\n"
            "ks outer stationarity tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks outer stationarity tolerance .00001\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterStationarityTolerance(), ".00001");

    // ks outer stagnation tolerance
    stringInput = "begin optimization parameters\n"
            "ks outer stagnation tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks outer stagnation tolerance 2.3\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterStagnationTolerance(), "2.3");

    // ks outer control stagnation tolerance
    stringInput = "begin optimization parameters\n"
            "ks outer control stagnation tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks outer control stagnation tolerance 1e-8\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterControlStagnationTolerance(), "1e-8");

    // ks outer actual reduction tolerance
    stringInput = "begin optimization parameters\n"
            "ks outer actual reduction tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks outer actual reduction tolerance 1e-7\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterActualReductionTolerance(), "1e-7");

    // gcmma max inner iterations
    stringInput = "begin optimization parameters\n"
            "gcmma max inner iterations\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma max inner iterations 100\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAMaxInnerIterations(), "100");

    // gcmma inner kkt tolerance
    stringInput = "begin optimization parameters\n"
            "gcmma inner kkt tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma inner kkt tolerance 1e-12\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAInnerKKTTolerance(), "1e-12");

    // gcmma inner control stagnation tolerance
    stringInput = "begin optimization parameters\n"
            "gcmma inner control stagnation tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma inner control stagnation tolerance 5e-10\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAInnerControlStagnationTolerance(), "5e-10");

    // gcmma outer kkt tolerance
    stringInput = "begin optimization parameters\n"
            "gcmma outer kkt tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma outer kkt tolerance 1e-8\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAOuterKKTTolerance(), "1e-8");

    // gcmma outer control stagnation tolerance
    stringInput = "begin optimization parameters\n"
            "gcmma outer control stagnation tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma outer control stagnation tolerance 1e-11\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAOuterControlStagnationTolerance(), "1e-11");

    // gcmma outer objective stagnation tolerance
    stringInput = "begin optimization parameters\n"
            "gcmma outer objective stagnation tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma outer objective stagnation tolerance 4e-8\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAOuterObjectiveStagnationTolerance(), "4e-8");

    // gcmma outer stationarity tolerance
    stringInput = "begin optimization parameters\n"
            "gcmma outer stationarity tolerance\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma outer stationarity tolerance 2e-10\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAOuterStationarityTolerance(), "2e-10");

    // gcmma initial moving asymptotes scale factor
    stringInput = "begin optimization parameters\n"
            "gcmma initial moving asymptotes scale factor\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "gcmma initial moving asymptotes scale factor 0.4\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAInitialMovingAsymptotesScaleFactor(), "0.4");

    // levelset sphere packing factor
    stringInput = "begin optimization parameters\n"
            "levelset sphere packing factor\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "levelset sphere packing factor .2\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getLevelsetSpherePackingFactor(), ".2");

    // levelset sphere radius
    stringInput = "begin optimization parameters\n"
            "levelset sphere radius\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "levelset sphere radius .33\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getLevelsetSphereRadius(), ".33");

    // levelset nodesets
    stringInput = "begin optimization parameters\n"
            "levelset nodesets\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "levelset nodesets 4 10 22\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getLevelsetNodeset(0), "4");
    EXPECT_EQ(tester.getLevelsetNodeset(1), "10");
    EXPECT_EQ(tester.getLevelsetNodeset(2), "22");

    // output frequency
    stringInput = "begin optimization parameters\n"
            "output frequency\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "output frequency 5\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getOutputFrequency(), "5");

    // output method
    stringInput = "begin optimization parameters\n"
            "output method\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "output method parallel write\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getOutputMethod(), "parallel write");

    // fixed blocks
    stringInput = "begin optimization parameters\n"
            "fixed blocks\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "fixed blocks 1 3 5\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFixedBlock(0), "1");
    EXPECT_EQ(tester.getFixedBlock(1), "3");
    EXPECT_EQ(tester.getFixedBlock(2), "5");

    // fixed sidesets
    stringInput = "begin optimization parameters\n"
            "fixed sidesets\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "fixed sidesets 33 44 55\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFixedSideset(0), "33");
    EXPECT_EQ(tester.getFixedSideset(1), "44");
    EXPECT_EQ(tester.getFixedSideset(2), "55");

    // fixed nodesets
    stringInput = "begin optimization parameters\n"
            "fixed nodesets\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "fixed nodesets 5 7 33\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFixedNodeset(0), "5");
    EXPECT_EQ(tester.getFixedNodeset(1), "7");
    EXPECT_EQ(tester.getFixedNodeset(2), "33");

    // number processors
    stringInput = "begin optimization parameters\n"
            "number processors\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "number processors 16\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getNumberProcessors(), "16");

    // filter scale
    stringInput = "begin optimization parameters\n"
            "filter radius scale\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter radius scale 1.6\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFilterScale(), "1.6");

    // filter absolute
    stringInput = "begin optimization parameters\n"
            "filter radius absolute\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter radius absolute 1.6\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFilterAbsolute(), "1.6");

    // filter power
    stringInput = "begin optimization parameters\n"
            "filter power\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter power 2.5\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFilterPower(), "2.5");

    // algorithm
    stringInput = "begin optimization parameters\n"
            "algorithm\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "algorithm ksbc\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getAlgorithm(), "ksbc");

    // discretization
    stringInput = "begin optimization parameters\n"
            "discretization\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "discretization levelset\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getDiscretization(), "levelset");

    // check gradient
    stringInput = "begin optimization parameters\n"
            "check gradient\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "check gradient false\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getCheckGradient(), "false");

    // check hessian
    stringInput = "begin optimization parameters\n"
            "check hessian\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "check hessian true\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getCheckHessian(), "true");
}

TEST(PlatoTestXMLGenerator, parseConstraints)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    stringInput = "begin constraint\n"
            "type\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), false);
    stringInput = "begin constraint\n"
            "name\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), false);
    stringInput = "begin constraint\n"
            "volume fraction\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), false);
    stringInput = "begin constraint\n"
            "surface area\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), false);
    stringInput = "begin constraint\n"
            "surface area sideset id\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), false);
    stringInput = "begin constraint\n"
            "bad_keyword\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), false);
    stringInput = "begin constraint\n"
            "type volume\n"
            "name vol1\n"
            "volume fraction .5\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), true);
    EXPECT_EQ(tester.getConstraintName(0), "vol1");
    EXPECT_EQ(tester.getConstraintType(0), "volume");
    EXPECT_EQ(tester.getConstraintVolFrac(0), ".5");
    stringInput = "begin constraint\n"
            "type surface area\n"
            "name surf1\n"
            "surface area 23\n"
            "surface area sideset id 20\n"
            "end constraint\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseConstraints(iss), true);
    EXPECT_EQ(tester.getConstraintName(0), "surf1");
    EXPECT_EQ(tester.getConstraintType(0), "surface area");
    EXPECT_EQ(tester.getConstraintSurfArea(0), "23");
    EXPECT_EQ(tester.getConstraintSurfAreaSidesetID(0), "20");
}
TEST(PlatoTestXMLGenerator, parseMesh)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    stringInput = "begin mesh\n"
            "name\n"
            "end mesh\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMesh(iss), false);
    stringInput = "begin mesh\n"
            "bad_keyword\n"
            "end mesh\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMesh(iss), false);
    stringInput = "begin mesh\n"
            "name file.gen\n"
            "end mesh\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMesh(iss), true);
    EXPECT_EQ(tester.getMeshName(), "file.gen");
}
TEST(PlatoTestXMLGenerator, parseCodePaths)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    stringInput = "begin paths\n"
            "code sierra_sd\n"
            "end paths\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseCodePaths(iss), false);
    stringInput = "begin paths\n"
            "code lightmp\n"
            "end paths\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseCodePaths(iss), false);
    stringInput = "begin paths\n"
            "code albany\n"
            "end paths\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseCodePaths(iss), false);
    stringInput = "begin paths\n"
            "code platomain\n"
            "end paths\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseCodePaths(iss), false);
    stringInput = "begin paths\n"
            "bad_keyword\n"
            "end paths\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseCodePaths(iss), false);
    stringInput = "begin paths\n"
            "code sierra_sd /Users/bwclark/salinas\n"
            "code albany /Users/bwclark/albany\n"
            "code lightmp /Users/bwclark/lightmp\n"
            "code platomain /Users/bwclark/platomain\n"
            "end paths\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseCodePaths(iss), true);
    EXPECT_EQ(tester.getSalinasPath(), "/Users/bwclark/salinas");
    EXPECT_EQ(tester.getAlbanyPath(), "/Users/bwclark/albany");
    EXPECT_EQ(tester.getLightMPPath(), "/Users/bwclark/lightmp");
    EXPECT_EQ(tester.getPlatoMainPath(), "/Users/bwclark/platomain");
}
TEST(PlatoTestXMLGenerator, parseBlocks)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    stringInput = "begin block\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), false);
    stringInput = "begin block 1\n"
            "material\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), false);
    stringInput = "begin block 1\n"
            "bad_keywordl\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), false);
    stringInput = "begin block 1\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), true);
    stringInput = "begin block\n"
            "material 1\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), false);
    stringInput = "begin block 44\n"
            "material 89\n"
            "end block\n"
            "begin block 33\n"
            "material 34\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), true);
    EXPECT_EQ(tester.getBlockID(0), "44");
    EXPECT_EQ(tester.getBlockMaterialID(0), "89");
    EXPECT_EQ(tester.getBlockID(1), "33");
    EXPECT_EQ(tester.getBlockMaterialID(1), "34");
}
TEST(PlatoTestXMLGenerator, parseMaterials)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    stringInput = "begin material 1\n"
            "penalty exponent\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material 1\n"
            "youngs modulus\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material 1\n"
            "poissons ratio\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material 1\n"
            "density\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material 1\n"
            "thermal conductivity\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material 1\n"
            "bad_keyword\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material 34\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
            "end material\n"
            "begin material 388\n"
            "penalty exponent 5\n"
            "youngs modulus 1e7\n"
            "poissons ratio 0.34\n"
            "thermal conductivity .03\n"
            "density .009\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), true);
    EXPECT_EQ(tester.getMaterialID(0), "34");
    EXPECT_EQ(tester.getMaterialPenaltyExponent(0), "3");
    EXPECT_EQ(tester.getMaterialYoungsModulus(0), "1e6");
    EXPECT_EQ(tester.getMaterialPoissonsRatio(0), "0.33");
    EXPECT_EQ(tester.getMaterialThermalConductivity(0), ".02");
    EXPECT_EQ(tester.getMaterialDensity(0), ".001");
    EXPECT_EQ(tester.getMaterialID(1), "388");
    EXPECT_EQ(tester.getMaterialPenaltyExponent(1), "5");
    EXPECT_EQ(tester.getMaterialYoungsModulus(1), "1e7");
    EXPECT_EQ(tester.getMaterialPoissonsRatio(1), "0.34");
    EXPECT_EQ(tester.getMaterialThermalConductivity(1), ".03");
    EXPECT_EQ(tester.getMaterialDensity(1), ".009");
}
TEST(PlatoTestXMLGenerator, parseObjectives)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    // Test some examples of valid objectives.
    stringInput = "begin objective\n"
            "type match frf data\n"
            "code sierra_sd\n"
            "end objective\n"
            "begin objective\n"
            "type match frf data\n"
            "name frf_1\n"
            "end objective\n";
    iss.str(stringInput);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.getObjectiveName(0), "1");
    EXPECT_EQ(tester.getObjectiveName(1), "frf_1");

    // Test the unrecognized keyword
    stringInput = "begin objective\n"
            "hippo \n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    stringInput = "begin objective\n"
            "fire fly\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    stringInput = "begin objective\n"
            "begin\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "type" keyword
    stringInput = "begin objective\n"
            "type \n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "distribute objective" keyword
    stringInput = "begin objective\n"
            "distribute\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    stringInput = "begin objective\n"
            "distribute objective\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    stringInput = "begin objective\n"
            "distribute objective kangaroo\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    stringInput = "begin objective\n"
            "distribute objective none\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    stringInput = "begin objective\n"
            "multi load case true\n"
            "distribute objective at most 256 processors\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);

    // Test the "name" keyword
    stringInput = "begin objective\n"
            "name \n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "code" keyword
    stringInput = "begin objective\n"
            "code \n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "complex error measure" keywords
    stringInput = "begin objective\n"
            "complex error measure \n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "number processors" keywords
    stringInput = "begin objective\n"
            "number processors\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "weight" keyword
    stringInput = "begin objective\n"
            "weight \n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "frf match nodesets" keywords
    stringInput = "begin objective\n"
            "frf match nodesets\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "sierra_sd weightmass scale factor" keywords
    stringInput = "begin objective\n"
            "weightmass scale factor\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "sierra_sd analysis solver tolerance" keywords
    stringInput = "begin objective\n"
            "analysis solver tolerance\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "reference frf file" keywords
    stringInput = "begin objective\n"
            "reference frf file\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "raleigh damping alpha" keywords
    stringInput = "begin objective\n"
            "raleigh damping alpha\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "raleigh damping beta" keywords
    stringInput = "begin objective\n"
            "raleigh damping beta\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    // Test the "stress limit" keywords
    stringInput = "begin objective\n"
            "type limit stress\n"
            "stress limit 42.5\n"
            "stress ramp factor 0.24\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.getObjectiveType(0), "limit stress");
    EXPECT_EQ(tester.getObjStressLimit(0), "42.5");
    EXPECT_EQ(tester.getObjStressRampFactor(0), "0.24");
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type limit stress\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.getObjectiveType(0), "limit stress");
    EXPECT_EQ(tester.getObjStressLimit(0), "");
    EXPECT_EQ(tester.getObjStressRampFactor(0), "");
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type limit stress\n"
            "stress limit\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type limit stress\n"
            "stress ramp factor\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    // Test the "bcs" keywords
    stringInput =
            "begin boundary conditions\n"
            "fixed displacement nodeset 1 bc id 33\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    EXPECT_EQ(tester.getBCApplicationType("33"), "nodeset");
    EXPECT_EQ(tester.getBCApplicationID("33"), "1");
    EXPECT_EQ(tester.getBCApplicationDOF("33"), "");
    stringInput =
            "begin boundary conditions\n"
            "fixed displacement nodeset 2 x bc id 33\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    EXPECT_EQ(tester.getBCApplicationType("33"), "nodeset");
    EXPECT_EQ(tester.getBCApplicationID("33"), "2");
    EXPECT_EQ(tester.getBCApplicationDOF("33"), "x");
    stringInput =
            "begin boundary conditions\n"
            "fixed displacement\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), false);
    stringInput =
            "begin boundary conditions\n"
            "displacement nodeset 1 x\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), false);
    stringInput =
            "begin boundary conditions\n"
            "fixed flux nodeset 1 x\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), false);
    stringInput =
            "begin boundary conditions\n"
            "fixed displacement sideset 1 x bc id 33\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    stringInput =
            "begin boundary conditions\n"
            "fixed displacement nodeset 1 truck bc id 33\n"
            "end boundary conditions\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBCs(iss), false);

    // Test loads.
    // Check for known load types.
    stringInput =
            "begin loads\n"
            "pressure blah blah blah\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);

    // Tractions
    // check number of parameters
    stringInput =
            "begin loads\n"
            "traction\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // check applying to correct mesh type
    stringInput =
            "begin loads\n"
            "traction nodeset 1 value 0 0 1 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // check for direction keyword
    stringInput =
            "begin loads\n"
            "traction sideset 1 bad_keyword 0 0 1 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // Do a correct example
    stringInput =
            "begin loads\n"
            "traction sideset 3 value 990 0 1000 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    EXPECT_EQ(tester.getLoadType("34",0), "traction");
    EXPECT_EQ(tester.getLoadApplicationType("34",0), "sideset");
    EXPECT_EQ(tester.getLoadApplicationID("34",0), "3");
    EXPECT_EQ(tester.getLoadDirectionX("34",0), "990");
    EXPECT_EQ(tester.getLoadDirectionY("34",0), "0");
    EXPECT_EQ(tester.getLoadDirectionZ("34",0), "1000");

    // Pressure
    // check number of parameters
    stringInput =
            "begin loads\n"
            "pressure\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // check applying to correct mesh type
    stringInput =
            "begin loads\n"
            "pressure nodeset 1 value 1000 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // check for value keyword
    stringInput =
            "begin loads\n"
            "pressure sideset 1 bad_keyword 1000 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // Do a correct example
    stringInput =
            "begin loads\n"
            "pressure sideset 3 value 1000 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    EXPECT_EQ(tester.getLoadType("34",0), "pressure");
    EXPECT_EQ(tester.getLoadApplicationType("34",0), "sideset");
    EXPECT_EQ(tester.getLoadApplicationID("34",0), "3");

    // Heat Flux
    // check number of parameters
    stringInput =
            "begin loads\n"
            "heat flux\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // check applying to correct mesh type
    stringInput =
            "begin loads\n"
            "heat flux nodeset 1 value 1000 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // Do a correct example
    stringInput =
            "begin loads\n"
            "heat flux sideset 4 value 90 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    EXPECT_EQ(tester.getLoadType("34",0), "heat");
    EXPECT_EQ(tester.getLoadApplicationType("34",0), "sideset");
    EXPECT_EQ(tester.getLoadApplicationID("34",0), "4");

    // Force
    // check number of parameters
    stringInput =
            "begin loads\n"
            "force\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // check applying to correct mesh type
    stringInput =
            "begin loads\n"
            "force block 1 value 0 0 1000 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // check for direction keyword
    stringInput =
            "begin loads\n"
            "force nodeset 1 bad_keyword 0 0 1 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), false);
    // Do a good example
    stringInput =
            "begin loads\n"
            "force nodeset 2 value 222 0 0 load id 34\n"
            "end loads\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    EXPECT_EQ(tester.getLoadType("34",0), "force");
    EXPECT_EQ(tester.getLoadApplicationType("34",0), "nodeset");
    EXPECT_EQ(tester.getLoadApplicationID("34",0), "2");
    EXPECT_EQ(tester.getLoadDirectionX("34",0), "222");
    EXPECT_EQ(tester.getLoadDirectionY("34",0), "0");
    EXPECT_EQ(tester.getLoadDirectionZ("34",0), "0");

    // Test frequency block
    // check for scale keyword
    stringInput = "begin objective\n"
            "begin frequency\n"
            "bad_keyword\n"
            "end frequency\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    // check for missing info
    stringInput = "begin objective\n"
            "begin frequency\n"
            "min 100\n"
            "step 4\n"
            "end frequency\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    // Do good example
    stringInput = "begin objective\n"
            "begin frequency\n"
            "min 100\n"
            "max 900\n"
            "step 4\n"
            "end frequency\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.getObjFreqMin(0), "100");
    EXPECT_EQ(tester.getObjFreqMax(0), "900");
    EXPECT_EQ(tester.getObjFreqStep(0), "4");

    // Some checks for objective names
    stringInput = "begin objective\n"
            "code sierra_sd\n"
            "name obj1\n"
            "end objective\n"
            "begin objective\n"
            "code sierra_sd\n"
            "end objective\n"
            "begin objective\n"
            "code albany\n"
            "end objective\n"
            "begin objective\n"
            "code sierra_sd\n"
            "end objective\n"
            "begin objective\n"
            "code lightmp\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.getObjectiveName(0), "obj1");
    EXPECT_EQ(tester.getObjectiveName(1), "2");
    EXPECT_EQ(tester.getObjectiveName(2), "1");
    EXPECT_EQ(tester.getObjectiveName(3), "3");
    EXPECT_EQ(tester.getObjectiveName(4), "1");
    EXPECT_EQ(tester.getObjCodeName(0), "sierra_sd");
    EXPECT_EQ(tester.getObjCodeName(1), "sierra_sd");
    EXPECT_EQ(tester.getObjCodeName(2), "albany");
    EXPECT_EQ(tester.getObjCodeName(3), "sierra_sd");
    EXPECT_EQ(tester.getObjCodeName(4), "lightmp");
    EXPECT_EQ(tester.getObjPerfName(0), "sierra_sd_obj1");
    EXPECT_EQ(tester.getObjPerfName(1), "sierra_sd_2");
    EXPECT_EQ(tester.getObjPerfName(2), "albany_1");
    EXPECT_EQ(tester.getObjPerfName(3), "sierra_sd_3");
    EXPECT_EQ(tester.getObjPerfName(4), "lightmp_1");
}

TEST(PlatoTestXMLGenerator,uncertainLoad_single)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 34\n"
            "    boundary condition ids 256\n"
            "    multi load case true\n"
            "    code sierra_sd\n"
            "    number processors 9\n"
            "    weight 1 \n"
            "end objective\n"
            "begin loads\n"
            "    force nodeset 2 value 7 0 0 load id 34\n"
            "    force nodeset 72 value 1 1 1 load id 1\n"
            "end loads\n"
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 34\n"
            "    axis Z\n"
            "    distribution beta\n"
            "    mean 2\n"
            "    upper bound 40\n"
            "    lower bound -35\n"
            "    standard deviation 20\n"
            "    num samples 3\n"
            "end uncertainty\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseUncertainties(iss), true);

    // BEFORE modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);
    // check load
    EXPECT_EQ(tester.getLoadType("34",0), "force");
    EXPECT_EQ(tester.getLoadApplicationType("34",0), "nodeset");
    EXPECT_EQ(tester.getLoadApplicationID("34",0), "2");
    EXPECT_EQ(tester.getLoadDirectionX("34",0), "7");
    EXPECT_EQ(tester.getLoadDirectionY("34",0), "0");
    EXPECT_EQ(tester.getLoadDirectionZ("34",0), "0");

    // do expand
    EXPECT_EQ(tester.publicExpandUncertaintiesForGenerate(), true);

    // AFTER modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);
    // check loads
    EXPECT_EQ(tester.getLoadType("34",0), "force");
    EXPECT_EQ(tester.getLoadApplicationType("34",0), "nodeset");
    EXPECT_EQ(tester.getLoadApplicationID("34",0), "2");
    const double load34x = std::atof(tester.getLoadDirectionX("34", 0).c_str());
    const double load34y = std::atof(tester.getLoadDirectionY("34", 0).c_str());
    const double load34z = std::atof(tester.getLoadDirectionZ("34", 0).c_str());
    std::cout << "34:" << load34x << "," << load34y << "," << load34z << std::endl;
    EXPECT_NEAR(load34x, 6.6, 0.5);
    EXPECT_NEAR(load34y, -2.3, 0.5);
    EXPECT_NEAR(load34z, 0, 1e-5);
    EXPECT_EQ(tester.getLoadType("0",0), "force");
    EXPECT_EQ(tester.getLoadApplicationType("0",0), "nodeset");
    EXPECT_EQ(tester.getLoadApplicationID("0",0), "2");
    const double load0x = std::atof(tester.getLoadDirectionX("0", 0).c_str());
    const double load0y = std::atof(tester.getLoadDirectionY("0", 0).c_str());
    const double load0z = std::atof(tester.getLoadDirectionZ("0", 0).c_str());
    std::cout << "0:" << load0x << "," << load0y << "," << load0z << std::endl;
    EXPECT_NEAR(load0x, 7.0, 0.5);
    EXPECT_NEAR(load0y, 0.7, 0.5);
    EXPECT_NEAR(load0z, 0, 1e-5);
    EXPECT_EQ(tester.getLoadType("2",0), "force");
    EXPECT_EQ(tester.getLoadApplicationType("2",0), "nodeset");
    EXPECT_EQ(tester.getLoadApplicationID("2",0), "2");
    const double load2x = std::atof(tester.getLoadDirectionX("2", 0).c_str());
    const double load2y = std::atof(tester.getLoadDirectionY("2", 0).c_str());
    const double load2z = std::atof(tester.getLoadDirectionZ("2", 0).c_str());
    std::cout << "2:" << load2x << "," << load2y << "," << load2z << std::endl;
    EXPECT_NEAR(load2x, 6.1, 0.5);
    EXPECT_NEAR(load2y, 3.4, 0.5);
    EXPECT_NEAR(load2z, 0, 1e-5);

    // check load ids
    std::vector<std::string> modified_load_ids = tester.getObjLoadIds(0);
    ASSERT_EQ(modified_load_ids.size(), 3u);
    EXPECT_EQ(modified_load_ids[0], "34");
    EXPECT_EQ(modified_load_ids[1], "0");
    EXPECT_EQ(modified_load_ids[2], "2");

    // check weights
    std::vector<std::string> modified_load_weights = tester.getObjLoadWeights(0);
    ASSERT_EQ(modified_load_weights.size(), 3u);
    const double lcw0 = std::atof(modified_load_weights[0].c_str());
    const double lcw1 = std::atof(modified_load_weights[1].c_str());
    const double lcw2 = std::atof(modified_load_weights[2].c_str());
    EXPECT_NEAR(lcw0, 0.33, 0.2);
    EXPECT_NEAR(lcw1, 0.33, 0.2);
    EXPECT_NEAR(lcw2, 0.33, 0.2);
    EXPECT_NEAR(lcw0+lcw1+lcw2, 1.0, 0.05);
    std::cout << "lcw:" << lcw0 << "," << lcw1 << "," << lcw2 << std::endl;
}

TEST(PlatoTestXMLGenerator,uncertainLoad_two)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 8\n"
            "    boundary condition ids 256\n"
            "    multi load case true\n"
            "    code sierra_sd\n"
            "    number processors 3\n"
            "    weight 1 \n"
            "end objective\n"
            "begin loads\n"
            "    force nodeset 2 value 5 0 0 load id 8\n"
            "    force nodeset 4 value 1 1 1 load id 1\n"
            "    force nodeset 3 value 0 -1 -2 load id 1\n"
            "    force nodeset 1 value 3 -4 1 load id 2\n"
            "end loads\n"
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 8\n"
            "    axis Y\n"
            "    distribution beta\n"
            "    mean 2\n"
            "    upper bound 30\n"
            "    lower bound -25\n"
            "    standard deviation 15\n"
            "    num samples 4\n"
            "end uncertainty\n"
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 8\n"
            "    axis Z\n"
            "    distribution beta\n"
            "    mean 5\n"
            "    upper bound 30\n"
            "    lower bound -25\n"
            "    standard deviation 15\n"
            "    num samples 3\n"
            "end uncertainty\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseUncertainties(iss), true);

    // BEFORE modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);
    // check load
    EXPECT_EQ(tester.getLoadType("8",0), "force");
    EXPECT_EQ(tester.getLoadApplicationType("8",0), "nodeset");
    EXPECT_EQ(tester.getLoadApplicationID("8",0), "2");
    EXPECT_EQ(tester.getLoadDirectionX("8",0), "5");
    EXPECT_EQ(tester.getLoadDirectionY("8",0), "0");
    EXPECT_EQ(tester.getLoadDirectionZ("8",0), "0");

    // do expand
    EXPECT_EQ(tester.publicExpandUncertaintiesForGenerate(), true);

    // AFTER modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);

    const size_t num_expected_samples = 12u;

    // check loads that will be generated
    std::vector<std::string> loads_to_check = {"8", "5", "6", "7", "9", "10", "11", "12", "13", "14", "15", "16"};
    ASSERT_EQ(loads_to_check.size(), num_expected_samples);
    for(size_t i = 0u; i < num_expected_samples; i++)
    {
        EXPECT_EQ(tester.getLoadType(loads_to_check[i],0), "force");
        EXPECT_EQ(tester.getLoadApplicationType(loads_to_check[i],0), "nodeset");
        EXPECT_EQ(tester.getLoadApplicationID(loads_to_check[i],0), "2");
        const double tX = std::atof(tester.getLoadDirectionX(loads_to_check[i], 0).c_str());
        const double tY = std::atof(tester.getLoadDirectionY(loads_to_check[i], 0).c_str());
        const double tZ = std::atof(tester.getLoadDirectionZ(loads_to_check[i], 0).c_str());
        EXPECT_NEAR(tX, 5., 2.);
        EXPECT_NEAR(tY, 0., 2.);
        EXPECT_NEAR(tZ, 0., 2.);
//        std::cout << i << "," << tX << "," << tY << "," << tZ << std::endl;    // announce
    }

    // check load ids
    std::vector<std::string> modified_load_ids = tester.getObjLoadIds(0);
    ASSERT_EQ(modified_load_ids.size(), num_expected_samples);
    for(size_t i = 0u; i < num_expected_samples; i++)
    {
        EXPECT_EQ(modified_load_ids[i], loads_to_check[i]);
    }

    // check weights
    std::vector<std::string> modified_load_weights = tester.getObjLoadWeights(0);
    ASSERT_EQ(modified_load_weights.size(), num_expected_samples);
    double totalW = 0;
    for(size_t i = 0u; i < num_expected_samples; i++)
    {
        const double tW = std::atof(modified_load_weights[i].c_str());
        EXPECT_NEAR(tW, 0.083, 0.082);
        totalW += tW;
//        std::cout << i << "," << tW << std::endl; // announce
    }
    EXPECT_NEAR(totalW, 1.0, 0.05);
}

TEST(PlatoTestXMLGenerator,uncertainLoad_certainWithUncertain)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 1 8\n"
            "    boundary condition ids 256\n"
            "    code sierra_sd\n"
            "    multi load case true\n"
            "    number processors 3\n"
            "    weight 1 \n"
            "end objective\n"
            "begin loads\n"
            "    force nodeset 2 value 0 -6 0 load id 8\n"
            "    force nodeset 4 value 1 1 1 load id 1\n"
            "    force nodeset 3 value 0 -1 -2 load id 1\n"
            "    force nodeset 1 value 3 -4 1 load id 2\n"
            "end loads\n"
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 8\n"
            "    axis X\n"
            "    distribution beta\n"
            "    mean 2\n"
            "    upper bound 25\n"
            "    lower bound -30\n"
            "    standard deviation 15\n"
            "    num samples 5\n"
            "end uncertainty\n"
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 8\n"
            "    axis Z\n"
            "    distribution beta\n"
            "    mean 5\n"
            "    upper bound 30\n"
            "    lower bound -25\n"
            "    standard deviation 15\n"
            "    num samples 3\n"
            "end uncertainty\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseUncertainties(iss), true);

    // BEFORE modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);
    // check load
    EXPECT_EQ(tester.getLoadType("8",0), "force");
    EXPECT_EQ(tester.getLoadApplicationType("8",0), "nodeset");
    EXPECT_EQ(tester.getLoadApplicationID("8",0), "2");
    EXPECT_EQ(tester.getLoadDirectionX("8",0), "0");
    EXPECT_EQ(tester.getLoadDirectionY("8",0), "-6");
    EXPECT_EQ(tester.getLoadDirectionZ("8",0), "0");

    // do expand
    EXPECT_EQ(tester.publicExpandUncertaintiesForGenerate(), true);

    // AFTER modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);

    const size_t num_expected_samples = 15u;

    // check loads that will be generated
    std::vector<std::string> loads_to_check = {"8", "6", "7", "9", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20"};
    ASSERT_EQ(loads_to_check.size(), num_expected_samples);
    for(size_t i = 0u; i < num_expected_samples; i++)
    {
        EXPECT_EQ(tester.getLoadType(loads_to_check[i],0), "force");
        EXPECT_EQ(tester.getLoadApplicationType(loads_to_check[i],0), "nodeset");
        EXPECT_EQ(tester.getLoadApplicationID(loads_to_check[i],0), "2");
        const double tX = std::atof(tester.getLoadDirectionX(loads_to_check[i], 0).c_str());
        const double tY = std::atof(tester.getLoadDirectionY(loads_to_check[i], 0).c_str());
        const double tZ = std::atof(tester.getLoadDirectionZ(loads_to_check[i], 0).c_str());
        EXPECT_NEAR(tX, 0., 2.5);
        EXPECT_NEAR(tY, -6., 1.);
        EXPECT_NEAR(tZ, 0., 2.5);
        //        std::cout << i << "," << tX << "," << tY << "," << tZ << std::endl; // announce
    }

    // check load ids
    std::vector<std::string> modified_load_ids = tester.getObjLoadIds(0);
    ASSERT_EQ(modified_load_ids.size(), 1u+num_expected_samples);
    for(size_t i = 0u; i < num_expected_samples; i++)
    {
        EXPECT_EQ(modified_load_ids[1u+i], loads_to_check[i]);
    }

    // check weights
    std::vector<std::string> modified_load_weights = tester.getObjLoadWeights(0);
    ASSERT_EQ(modified_load_weights.size(), 1u+num_expected_samples);
    double totalW = 0;
    for(size_t i = 0u; i < num_expected_samples; i++)
    {
        const double tW = std::atof(modified_load_weights[1u + i].c_str());
        EXPECT_NEAR(tW, 1.0/15.0, 0.05);
        totalW += tW;
        //        std::cout << i << "," << tW << std::endl; // announce
    }
    EXPECT_NEAR(totalW, 1.0, 0.05);
}

TEST(PlatoTestXMLGenerator,uncertainLoad_singleBug)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 34\n"
            "    boundary condition ids 256\n"
            "    code sierra_sd\n"
            "    number processors 9\n"
            "    multi load case true\n"
            "    weight 1 \n"
            "end objective\n"
            "begin loads\n"
            "    force nodeset 2 value 7 0 0 load id 34\n"
            "end loads\n"
            "begin uncertainty\n"
            "    type angle variation\n"
            "    load 34\n"
            "    axis Z\n"
            "    distribution beta\n"
            "    mean 45.0\n"
            "    upper bound 90\n"
            "    lower bound 0\n"
            "    standard deviation 1\n"
            "    num samples 6\n"
            "end uncertainty\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseUncertainties(iss), true);

    // BEFORE modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);
    // check load
    EXPECT_EQ(tester.getLoadType("34",0), "force");
    EXPECT_EQ(tester.getLoadApplicationType("34",0), "nodeset");
    EXPECT_EQ(tester.getLoadApplicationID("34",0), "2");
    EXPECT_EQ(tester.getLoadDirectionX("34",0), "7");
    EXPECT_EQ(tester.getLoadDirectionY("34",0), "0");
    EXPECT_EQ(tester.getLoadDirectionZ("34",0), "0");

    // do expand
    EXPECT_EQ(tester.publicExpandUncertaintiesForGenerate(), true);

    // AFTER modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);
    const size_t num_expected_samples = 6u;

    // check loads that will be generated
    std::vector<std::string> loads_to_check = {"34", "0", "1", "2", "3", "4"};
    ASSERT_EQ(loads_to_check.size(), num_expected_samples);
    for(size_t i = 0u; i < num_expected_samples; i++)
    {
        EXPECT_EQ(tester.getLoadType(loads_to_check[i],0), "force");
        EXPECT_EQ(tester.getLoadApplicationType(loads_to_check[i],0), "nodeset");
        EXPECT_EQ(tester.getLoadApplicationID(loads_to_check[i],0), "2");
        const double tX = std::atof(tester.getLoadDirectionX(loads_to_check[i], 0).c_str());
        const double tY = std::atof(tester.getLoadDirectionY(loads_to_check[i], 0).c_str());
        const double tZ = std::atof(tester.getLoadDirectionZ(loads_to_check[i], 0).c_str());
        EXPECT_EQ(std::isnan(tX), true);
        EXPECT_EQ(std::isnan(tY), true);
        EXPECT_NEAR(tZ, 0., 0.1);
//        std::cout << i << "," << tX << "," << tY << "," << tZ << std::endl;    // announce
    }

    // check load ids
    std::vector<std::string> modified_load_ids = tester.getObjLoadIds(0);
    ASSERT_EQ(modified_load_ids.size(), num_expected_samples);
    for(size_t i = 0u; i < num_expected_samples; i++)
    {
        EXPECT_EQ(modified_load_ids[i], loads_to_check[i]);
    }

    // check weights
    std::vector<std::string> modified_load_weights = tester.getObjLoadWeights(0);
    ASSERT_EQ(modified_load_weights.size(), num_expected_samples);
    for(size_t i = 0u; i < num_expected_samples; i++)
    {
        const double tW = std::atof(modified_load_weights[i].c_str());
        EXPECT_EQ(std::isnan(tW), true);
//        std::cout << i << "," << tW << std::endl; // announce
    }
}

TEST(PlatoTestXMLGenerator,distributeObjective_notEnoughProcessors)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 2 5 7 11\n"
            "    boundary condition ids 256\n"
            "    code sierra_sd\n"
            "    number processors 2\n"
            "    multi load case true\n"
            "    weight 1 \n"
            "    distribute objective at most 7 processors\n"
            "end objective\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);

    // BEFORE modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);

    // do expand
    EXPECT_EQ(tester.publicDistributeObjectivesForGenerate(), true);

    // AFTER modification

    // three objectives
    const size_t expected_num_objectives = 3u;
    ASSERT_EQ(tester.getNumObjectives(), expected_num_objectives);

    // check names
    EXPECT_EQ(tester.getObjectiveName(0u),"1");
    EXPECT_EQ(tester.getObjectiveName(1u),"2");
    EXPECT_EQ(tester.getObjectiveName(2u),"3");
    EXPECT_EQ(tester.getObjPerfName(0u),"sierra_sd_1");
    EXPECT_EQ(tester.getObjPerfName(1u),"sierra_sd_2");
    EXPECT_EQ(tester.getObjPerfName(2u),"sierra_sd_3");

    // get loadIds for distributed objective
    std::vector<std::string> distributed_loadIds;
    for(size_t objI = 0u; objI < expected_num_objectives; objI++)
    {
        const std::vector<std::string> thisObj_loadIds = tester.getObjLoadIds(objI);

        // expected number of loads is either 1 or 2
        const size_t num_load_ids = thisObj_loadIds.size();
        EXPECT_EQ((num_load_ids==1u) || (num_load_ids==2u), true);

        // concatenate
        distributed_loadIds.insert(distributed_loadIds.end(), thisObj_loadIds.begin(), thisObj_loadIds.end());
    }

    // expect maintenance of num of loads in distributed loads
    const size_t original_num_loads = 4u;
    ASSERT_EQ(distributed_loadIds.size(), original_num_loads);
    std::vector<bool> wasMaintained(12u, false);
    for(size_t di = 0u; di < original_num_loads; di++)
    {
        wasMaintained[std::atoi(distributed_loadIds[di].c_str())] = true;
    }

    // expect each load id was maintained
    EXPECT_EQ(wasMaintained[2], true);
    EXPECT_EQ(wasMaintained[5], true);
    EXPECT_EQ(wasMaintained[7], true);
    EXPECT_EQ(wasMaintained[11], true);
}

TEST(PlatoTestXMLGenerator,distributeObjective_exactlyEnoughProcessors)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 2 6 4 8\n"
            "    boundary condition ids 256\n"
            "    code sierra_sd\n"
            "    number processors 2\n"
            "    multi load case true\n"
            "    weight 1 \n"
            "    distribute objective at most 8 processors\n"
            "end objective\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);

    // BEFORE modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);

    // do expand
    EXPECT_EQ(tester.publicDistributeObjectivesForGenerate(), true);

    // AFTER modification

    // four objectives
    const size_t expected_num_objectives = 4u;
    ASSERT_EQ(tester.getNumObjectives(), expected_num_objectives);

    // check names
    EXPECT_EQ(tester.getObjectiveName(0u), "1");
    EXPECT_EQ(tester.getObjectiveName(1u), "2");
    EXPECT_EQ(tester.getObjectiveName(2u), "3");
    EXPECT_EQ(tester.getObjectiveName(3u), "4");
    EXPECT_EQ(tester.getObjPerfName(0u), "sierra_sd_1");
    EXPECT_EQ(tester.getObjPerfName(1u), "sierra_sd_2");
    EXPECT_EQ(tester.getObjPerfName(2u), "sierra_sd_3");
    EXPECT_EQ(tester.getObjPerfName(3u), "sierra_sd_4");

    // get loadIds for distributed objective
    std::vector<std::string> distributed_loadIds;
    for(size_t objI = 0u; objI < expected_num_objectives; objI++)
    {
        const std::vector<std::string> thisObj_loadIds = tester.getObjLoadIds(objI);

        // expected number of loads is 1
        const size_t num_load_ids = thisObj_loadIds.size();
        EXPECT_EQ(num_load_ids,1u);

        // concatenate
        distributed_loadIds.insert(distributed_loadIds.end(), thisObj_loadIds.begin(), thisObj_loadIds.end());
    }

    // expect maintenance of num of loads in distributed loads
    const size_t original_num_loads = 4u;
    ASSERT_EQ(distributed_loadIds.size(), original_num_loads);
    std::vector<bool> wasMaintained(12u, false);
    for(size_t di = 0u; di < original_num_loads; di++)
    {
        wasMaintained[std::atoi(distributed_loadIds[di].c_str())] = true;
    }

    // expect each load id was maintained
    EXPECT_EQ(wasMaintained[2], true);
    EXPECT_EQ(wasMaintained[6], true);
    EXPECT_EQ(wasMaintained[4], true);
    EXPECT_EQ(wasMaintained[8], true);
}

TEST(PlatoTestXMLGenerator,distributeObjective_moreThanEnoughProcessors)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 1 7 3 4\n"
            "    boundary condition ids 256\n"
            "    code sierra_sd\n"
            "    number processors 2\n"
            "    multi load case true\n"
            "    weight 1 \n"
            "    distribute objective at most 9 processors\n"
            "end objective\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);

    // BEFORE modification

    // one objective
    ASSERT_EQ(tester.getNumObjectives(), 1u);

    // do expand
    EXPECT_EQ(tester.publicDistributeObjectivesForGenerate(), true);

    // AFTER modification

    // four objectives
    const size_t expected_num_objectives = 4u;
    ASSERT_EQ(tester.getNumObjectives(), expected_num_objectives);

    // check names
    EXPECT_EQ(tester.getObjectiveName(0u), "1");
    EXPECT_EQ(tester.getObjectiveName(1u), "2");
    EXPECT_EQ(tester.getObjectiveName(2u), "3");
    EXPECT_EQ(tester.getObjectiveName(3u), "4");
    EXPECT_EQ(tester.getObjPerfName(0u), "sierra_sd_1");
    EXPECT_EQ(tester.getObjPerfName(1u), "sierra_sd_2");
    EXPECT_EQ(tester.getObjPerfName(2u), "sierra_sd_3");
    EXPECT_EQ(tester.getObjPerfName(3u), "sierra_sd_4");

    // get loadIds for distributed objective
    std::vector<std::string> distributed_loadIds;
    for(size_t objI = 0u; objI < expected_num_objectives; objI++)
    {
        const std::vector<std::string> thisObj_loadIds = tester.getObjLoadIds(objI);

        // expected number of loads is 1
        const size_t num_load_ids = thisObj_loadIds.size();
        EXPECT_EQ(num_load_ids,1u);

        // concatenate
        distributed_loadIds.insert(distributed_loadIds.end(), thisObj_loadIds.begin(), thisObj_loadIds.end());
    }

    // expect maintenance of num of loads in distributed loads
    const size_t original_num_loads = 4u;
    ASSERT_EQ(distributed_loadIds.size(), original_num_loads);
    std::vector<bool> wasMaintained(12u, false);
    for(size_t di = 0u; di < original_num_loads; di++)
    {
        wasMaintained[std::atoi(distributed_loadIds[di].c_str())] = true;
    }

    // expect each load id was maintained
    EXPECT_EQ(wasMaintained[1], true);
    EXPECT_EQ(wasMaintained[7], true);
    EXPECT_EQ(wasMaintained[3], true);
    EXPECT_EQ(wasMaintained[4], true);
}

} // end PlatoTestXMLGenerator namespace
