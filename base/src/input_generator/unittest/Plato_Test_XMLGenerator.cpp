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
#include "DefaultInputGenerator_UnitTester.hpp"
#include "ComplianceMinTOPlatoAnalyzeInputGenerator_UnitTester.hpp"
#include "Plato_Vector3DVariations.hpp"
#include "XML_GoldValues.hpp"
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

    // ks trust region ratio low/mid/upper
    stringInput = "begin optimization parameters\n"
            "ks trust region ratio low\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks trust region ratio mid\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks trust region ratio high\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "ks trust region ratio low 1.1\n"
            "ks trust region ratio mid 2.1\n"
            "ks trust region ratio high 3.1\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->mTrustRegionRatioLowKS, "1.1");
    EXPECT_EQ(tester.exposeInputData()->mTrustRegionRatioMidKS, "2.1");
    EXPECT_EQ(tester.exposeInputData()->mTrustRegionRatioUpperKS, "3.1");

    // ks disable post smoothing
    stringInput = "begin optimization parameters\n"
            "ks disable post smoothing\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "end optimization parameters\n";
    stringInput = "begin optimization parameters\n"
            "ks disable post smoothing fALse\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->mDisablePostSmoothingKS, "false");
    stringInput = "begin optimization parameters\n"
            "ks disable post smoothing tRuE\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->mDisablePostSmoothingKS, "true");


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

    // filter type
    stringInput = "begin optimization parameters\n"
            "filter type kernel\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_type,"kernel");
    stringInput = "begin optimization parameters\n"
            "filter type kernel then heaviside\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_type,"kernel then heaviside");
    stringInput = "begin optimization parameters\n"
            "filter type\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter type lions and tigers\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);

    // filter heaviside min
    stringInput = "begin optimization parameters\n"
            "filter heaviside min\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter heaviside min 1.526\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_min,"1.526");

    // filter heaviside update
    stringInput = "begin optimization parameters\n"
            "filter heaviside update\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter heaviside update 1.526\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_update,"1.526");

    // filter heaviside max
    stringInput = "begin optimization parameters\n"
            "filter heaviside max\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter heaviside max 1.526\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_max,"1.526");

    // filter heaviside scale
    stringInput = "begin optimization parameters\n"
            "filter heaviside scale\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter heaviside scale 1.526\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_min,"1.526");
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_max,"1.526");

    // filter heaviside various
    stringInput = "begin optimization parameters\n"
            "filter heaviside update 4.4\n"
            "filter heaviside scale 1.2\n"
            "filter heaviside min 0.5\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter heaviside update 4.4\n"
            "filter heaviside scale 1.2\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_min,"1.2");
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_update,"4.4");
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_max,"1.2");
    stringInput = "begin optimization parameters\n"
            "filter heaviside update 4.4\n"
            "filter heaviside min 0.5\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_min,"0.5");
    EXPECT_EQ(tester.exposeInputData()->filter_heaviside_update,"4.4");

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

    // filter radial power
    stringInput = "begin optimization parameters\n"
            "filter radial power\n"
            "end optimization parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), false);
    stringInput = "begin optimization parameters\n"
            "filter radial power 2.5\n"
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
            "thermal conductivity coefficient\n"
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
            "thermal conductivity coefficient .02\n"
            "density .001\n"
            "end material\n"
            "begin material 388\n"
            "penalty exponent 5\n"
            "youngs modulus 1e7\n"
            "poissons ratio 0.34\n"
            "thermal conductivity coefficient .03\n"
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

    // Test the "volume misfit target"
    stringInput = "begin objective\n"
            "volume misfit target 0.5125\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.exposeInputData()->objectives[0].volume_misfit_target, "0.5125");
    tester.clearInputData();

    stringInput = "begin objective\n"
            "volume misfit target\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    // Test the "stress p norm"
    stringInput = "begin objective\n"
            "type stress p norm\n"
            "stress limit 39.5\n"
            "stress ramp factor 0.22\n"
            "stress p norm power 7\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.exposeInputData()->objectives[0].type, "stress p norm");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].stress_limit, "39.5");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].stress_ramp_factor, "0.22");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].stress_p_norm_power, "7");
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type stress p norm\n"
            "stress p norm power\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type stress p norm\n"
            "stress ramp factor\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
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
            "limit power min 2.1\n"
            "limit power max 42.3\n"
            "limit power feasible bias 1.1\n"
            "limit power feasible slope 1.9\n"
            "limit power infeasible bias -0.51\n"
            "limit power infeasible slope 0.92\n"
            "limit reset subfrequency 11\n"
            "limit reset count 3\n"
            "inequality allowable feasibility upper 0.045\n"
            "stress inequality power 1.51\n"
            "volume penalty power 1.254\n"
            "volume penalty divisor 5.124\n"
            "volume penalty bias 0.4242\n"
            "inequality feasibility scale 2.2\n"
            "inequality infeasibility scale 0.48\n"
            "stress favor final 10.12\n"
            "stress favor updates 39\n"
            "relative stress limit 1.57\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    EXPECT_EQ(tester.getObjectiveType(0), "limit stress");
    EXPECT_EQ(tester.getObjStressLimit(0), "");
    EXPECT_EQ(tester.getObjStressRampFactor(0), "");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_power_min, "2.1");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_power_max, "42.3");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_power_feasible_bias, "1.1");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_power_feasible_slope, "1.9");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_power_infeasible_bias, "-0.51");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_power_infeasible_slope, "0.92");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].inequality_allowable_feasiblity_upper, "0.045");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].stress_inequality_power, "1.51");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].volume_penalty_power, "1.254");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].volume_penalty_divisor, "5.124");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].volume_penalty_bias, "0.4242");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_reset_subfrequency, "11");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].limit_reset_count, "3");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].inequality_feasibility_scale, "2.2");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].inequality_infeasibility_scale, "0.48");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].stress_favor_final, "10.12");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].stress_favor_updates, "39");
    EXPECT_EQ(tester.exposeInputData()->objectives[0].relative_stress_limit, "1.57");
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
            "stress limit 1.5\n"
            "relative stress limit 2.5\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);
    tester.clearInputData();

    stringInput = "begin objective\n"
            "type limit stress\n"
            "relative stress limit 1.3\n"
            "stress limit 2.1\n"
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

TEST(PlatoTestXMLGenerator,parseTractionLoad_valid_input)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "traction sideset 2 value 0 -3e3 0 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseTractionLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"traction");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    std::vector<std::string> values = {"0","-3e3","0"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseTractionLoad_valid_input_name_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "traction sideset name ss_2 value 0 -3e3 0 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseTractionLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"traction");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"");
    EXPECT_EQ(new_load.app_name,"ss_2");
    std::vector<std::string> values = {"0","-3e3","0"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseTractionLoad_valid_input_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "traction sideset id 2 value 0 -3e3 0 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseTractionLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"traction");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"");
    std::vector<std::string> values = {"0","-3e3","0"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseTractionLoad_valid_input_name_and_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "traction sideset id 2 name ss_2 value 0 -3e3 0 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseTractionLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"traction");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"ss_2");
    std::vector<std::string> values = {"0","-3e3","0"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parsePressureLoad_valid_input)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "pressure sideset 2 value 5 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParsePressureLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"pressure");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    std::vector<std::string> values = {"5"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parsePressureLoad_valid_input_name_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "pressure sideset name ss_2 value 5 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParsePressureLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"pressure");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"");
    EXPECT_EQ(new_load.app_name,"ss_2");
    std::vector<std::string> values = {"5"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parsePressureLoad_valid_input_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "pressure sideset id 2 value -3e3 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParsePressureLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"pressure");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"");
    std::vector<std::string> values = {"-3e3"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parsePressureLoad_valid_input_name_and_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "pressure sideset id 2 name ss_2 value -3e3 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParsePressureLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"pressure");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"ss_2");
    std::vector<std::string> values = {"-3e3"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseHeatFluxLoad_valid_input_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "heat flux sideset id 2 value -3e3 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseHeatFluxLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"heat");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"");
    std::vector<std::string> values = {"-3e3"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseHeatFluxLoad_valid_input_name_and_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "heat flux sideset id 2 name ss_2 value -3e3 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseHeatFluxLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"heat");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"ss_2");
    std::vector<std::string> values = {"-3e3"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseForceLoad_valid_input_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "force sideset id 2 value 0 -3e3 0 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseForceLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"force");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"");
    std::vector<std::string> values = {"0","-3e3","0"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
}

TEST(PlatoTestXMLGenerator,parseForceLoad_valid_input_name_and_id_specified)
{
    XMLGenerator_UnitTester tester;
    std::vector<std::string> tokens;
    XMLGen::Load new_load;
    std::istringstream iss;

    std::string stringInput =
    "force sideset id 2 name ss_2 value 0 -3e3 0 load id 1\n";

    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);

    char buf[MAX_CHARS_PER_LINE];
    
    tokens.clear();
    iss.getline(buf, MAX_CHARS_PER_LINE);
    tester.publicParseTokens(buf, tokens);

    new_load.type = tokens[0];

    EXPECT_EQ(tester.publicParseForceLoad(tokens,new_load),true);
    EXPECT_EQ(new_load.type,"force");
    EXPECT_EQ(new_load.app_type,"sideset");
    EXPECT_EQ(new_load.app_id,"2");
    EXPECT_EQ(new_load.app_name,"ss_2");
    std::vector<std::string> values = {"0","-3e3","0"};
    EXPECT_EQ(new_load.values,values);
    EXPECT_EQ(new_load.load_id,"1");
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

TEST(PlatoTestXMLGenerator,uncertainty_analyzeNewWorkflow)
{
  XMLGenerator_UnitTester tester;
  std::istringstream iss;
  std::string stringInput =
  "begin objective\n"
  "   type maximize stiffness\n"
  "   load ids 10\n"
  "   boundary condition ids 11\n"
  "   code plato_analyze\n"
  "   number processors 1\n"
  "   weight 1\n"
  "   analyze new workflow true\n"
  "   number ranks 1\n"
  "end objective\n"
  "begin boundary conditions\n"
  "   fixed displacement nodeset name 1 bc id 11\n"
  "end boundary conditions\n"
  "begin loads\n"
  "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
  "end loads\n"
  "begin uncertainty\n"
  "    type angle variation\n"
  "    load 10\n"
  "    axis X\n"
  "    distribution beta\n"
  "    mean 0.0\n"
  "    upper bound 45.0\n"
  "    lower bound -45.0\n"
  "    standard deviation 22.5\n"
  "    num samples 2\n"
  "end uncertainty\n";
  // do parse
  iss.str(stringInput);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseObjectives(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseLoads(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseBCs(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseUncertainties(iss), true);
  EXPECT_EQ(tester.publicRunSROMForUncertainVariables(), true);
  EXPECT_EQ(tester.publicDistributeObjectivesForGenerate(), true);

  size_t numSamples = tester.getNumSamples();
  size_t numVariables = tester.getNumVariables();
  size_t numPeformers = tester.getNumPerformers();
  EXPECT_EQ(numSamples,2u);
  EXPECT_EQ(numVariables,1u);
  EXPECT_EQ(numPeformers,1u);

  std::vector<size_t> randomVariableIndices = {0u};
  std::vector<size_t> deterministicVariableIndices = {};

  EXPECT_EQ(tester.getRandomVariableIndices(), randomVariableIndices);
  EXPECT_EQ(tester.getDeterministicVariableIndices(), deterministicVariableIndices);

  std::vector<double> gold_loadCaseProbabilities = { 0.36112468067266207, 0.63887286897558715 };
  std::vector<double> loadCaseProbabilities = tester.getLoadCaseProbabilities();
  for(size_t i = 0; i < numSamples; ++i)
    EXPECT_DOUBLE_EQ(loadCaseProbabilities[i],gold_loadCaseProbabilities[i]);

  std::vector<XMLGen::LoadCase> loadCases = tester.getLoadCases();
  EXPECT_EQ(loadCases.size(), numSamples);


  std::vector<std::vector<std::string>> gold_values;
  gold_values.resize(numSamples);
  gold_values[0] = { "0.000000000000000000000e+00", "-4.751921387767659325618e+04", "1.555391630579348566243e+04" };
  gold_values[1] = { "0.000000000000000000000e+00", "-4.813588076578034088016e+04", "-1.352541987897522631101e+04"};


  for(size_t i = 0; i < numSamples; ++i)
  {
    std::vector<XMLGen::Load> loads = loadCases[i].loads;
    EXPECT_EQ(loads.size(), 1u);
    for(size_t j = 0; j < loads.size(); ++j)
    {
      XMLGen::Load load = loads[j];
      EXPECT_EQ(load.type, "traction");
      std::vector<std::string> values = load.values;
      EXPECT_EQ(load.values, gold_values[i+j]);
    }
  }

  size_t numObjectives = tester.getNumObjectives();
  EXPECT_EQ(numObjectives, 1u);
}

TEST(PlatoTestXMLGenerator,uncertainty_analyzeNewWorkflow_randomPlusDeterministic)
{
  XMLGenerator_UnitTester tester;
  std::istringstream iss;
  std::string stringInput =
  "begin objective\n"
  "   type maximize stiffness\n"
  "   load ids 10 1\n"
  "   boundary condition ids 11\n"
  "   code plato_analyze\n"
  "   number processors 1\n"
  "   weight 1\n"
  "   analyze new workflow true\n"
  "   number ranks 5\n"
  "end objective\n"
  "begin boundary conditions\n"
  "   fixed displacement nodeset name 1 bc id 11\n"
  "end boundary conditions\n"
  "begin loads\n"
  "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
  "    traction sideset name 3 value 0 -5e4 0 load id 1\n"
  "end loads\n"
  "begin uncertainty\n"
  "    type angle variation\n"
  "    load 10\n"
  "    axis X\n"
  "    distribution beta\n"
  "    mean 0.0\n"
  "    upper bound 45.0\n"
  "    lower bound -45.0\n"
  "    standard deviation 22.5\n"
  "    num samples 10\n"
  "end uncertainty\n";
  // do parse
  iss.str(stringInput);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseObjectives(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseLoads(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseBCs(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseUncertainties(iss), true);
  EXPECT_EQ(tester.publicRunSROMForUncertainVariables(), true);
  EXPECT_EQ(tester.publicDistributeObjectivesForGenerate(), true);

  size_t numSamples = tester.getNumSamples();
  size_t numVariables = tester.getNumVariables();
  size_t numPeformers = tester.getNumPerformers();
  EXPECT_EQ(numSamples,10u);
  EXPECT_EQ(numVariables,2u);
  EXPECT_EQ(numPeformers,5u);

  std::vector<size_t> randomVariableIndices = {0u};
  std::vector<size_t> deterministicVariableIndices = {1u};

  EXPECT_EQ(tester.getRandomVariableIndices(), randomVariableIndices);
  EXPECT_EQ(tester.getDeterministicVariableIndices(), deterministicVariableIndices);

  std::vector<double> gold_loadCaseProbabilities = { 0.094172629104440519, 0.096118338919238849, 0.099663940442524482,
                                                     0.10447870334065364, 0.10564855564584232, 0.10576322174283935,
                                                     0.10486290959769146, 0.099755080570144233, 0.095660857410530639,
                                                     0.093878478812968374};
  std::vector<double> loadCaseProbabilities = tester.getLoadCaseProbabilities();
  for(size_t i = 0; i < numSamples; ++i)
    EXPECT_DOUBLE_EQ(loadCaseProbabilities[i],gold_loadCaseProbabilities[i]);

  std::vector<XMLGen::LoadCase> loadCases = tester.getLoadCases();
  EXPECT_EQ(loadCases.size(), numSamples);

  size_t numLoadsExpectedPerSample = 2u;

  std::vector<std::vector<std::string>> gold_values;
  gold_values.resize(numSamples*numLoadsExpectedPerSample);
  gold_values[0] = { "0.000000000000000000000e+00", "-4.008042184600126347505e+04", "2.989247036707714141812e+04" };
  gold_values[1] = { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" };
  gold_values[2] = { "0.000000000000000000000e+00", "-4.410827789159363601357e+04", "2.354697053631213930203e+04" };
  gold_values[3] = { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" };
  gold_values[4] = { "0.000000000000000000000e+00", "-4.704161308330694009783e+04", "1.694363120822820928879e+04" };
  gold_values[5] = { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" };
  gold_values[6] = { "0.000000000000000000000e+00", "-4.894831379618879145710e+04", "1.020110663162747732713e+04" };
  gold_values[7] = { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" };
  gold_values[8] = { "0.000000000000000000000e+00", "-4.988485492991151841125e+04", "3.391349085789085165743e+03" };
  gold_values[9] = { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" };
  gold_values[10] = { "0.000000000000000000000e+00", "-4.988284696095977415098e+04", "-3.420757090099503329839e+03"};
  gold_values[11] = { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" };
  gold_values[12] = { "0.000000000000000000000e+00", "-4.894769754769306018716e+04", "-1.020406315050838020397e+04"};
  gold_values[13] = { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" };
  gold_values[14] = { "0.000000000000000000000e+00", "-4.706038546730067173485e+04", "-1.689142148160054057371e+04"};
  gold_values[15] = { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" };
  gold_values[16] = { "0.000000000000000000000e+00", "-4.418230226249388215365e+04", "-2.340778004821533613722e+04"};
  gold_values[17] = { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" };
  gold_values[18] = { "0.000000000000000000000e+00", "-4.025755085941775905667e+04", "-2.965349218559918881510e+04"};
  gold_values[19] = { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" };


  for(size_t i = 0; i < numSamples; ++i)
  {
    std::vector<XMLGen::Load> loads = loadCases[i].loads;
    EXPECT_EQ(loads.size(), numLoadsExpectedPerSample);
    for(size_t j = 0; j < loads.size(); ++j)
    {
      XMLGen::Load load = loads[j];
      EXPECT_EQ(load.type, "traction");
      std::vector<std::string> values = load.values;
      EXPECT_EQ(load.values, gold_values[loads.size()*i+j]);
    }
  }

  size_t numObjectives = tester.getNumObjectives();
  EXPECT_EQ(numObjectives, 1u);
}

TEST(PlatoTestXMLGenerator,uncertainty_analyzeOldWorkflow)
{
  XMLGenerator_UnitTester tester;
  std::istringstream iss;
  std::string stringInput =
  "begin objective\n"
  "   type maximize stiffness\n"
  "   load ids 10\n"
  "   boundary condition ids 11\n"
  "   code plato_analyze\n"
  "   number processors 1\n"
  "   weight 1\n"
  "   multi load case true\n"
  "   distribute objective at most 3 processors\n"
  "end objective\n"
  "begin boundary conditions\n"
  "   fixed displacement nodeset name 1 bc id 11\n"
  "end boundary conditions\n"
  "begin loads\n"
  "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
  "end loads\n"
  "begin uncertainty\n"
  "    type angle variation\n"
  "    load 10\n"
  "    axis X\n"
  "    distribution beta\n"
  "    mean 0.0\n"
  "    upper bound 45.0\n"
  "    lower bound -45.0\n"
  "    standard deviation 22.5\n"
  "    num samples 3\n"
  "end uncertainty\n";
  // do parse
  iss.str(stringInput);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseObjectives(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseLoads(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseBCs(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseUncertainties(iss), true);
  EXPECT_EQ(tester.publicRunSROMForUncertainVariables(), true);
  EXPECT_EQ(tester.publicDistributeObjectivesForGenerate(), true);

  size_t numSamples = tester.getNumSamples();
  size_t numVariables = tester.getNumVariables();
  size_t numPeformers = tester.getNumPerformers();
  EXPECT_EQ(numSamples,3u);
  EXPECT_EQ(numVariables,1u);
  EXPECT_EQ(numPeformers,3u);

  std::vector<size_t> randomVariableIndices = {0u};
  std::vector<size_t> deterministicVariableIndices = {};

  EXPECT_EQ(tester.getRandomVariableIndices(), randomVariableIndices);
  EXPECT_EQ(tester.getDeterministicVariableIndices(), deterministicVariableIndices);

  std::vector<double> gold_loadCaseProbabilities = { 0.36406606886994869, 0.27079380819551258, 0.36514142000522448};

  std::vector<double> loadCaseProbabilities = tester.getLoadCaseProbabilities();
  for(size_t i = 0; i < numSamples; ++i)
    EXPECT_DOUBLE_EQ(loadCaseProbabilities[i],gold_loadCaseProbabilities[i]);

  std::vector<XMLGen::LoadCase> loadCases = tester.getLoadCases();
  EXPECT_EQ(loadCases.size(), numSamples);

  size_t numLoadsExpectedPerSample = 1u;

  std::vector<std::vector<std::string>> gold_values;
  gold_values.resize(numSamples*numLoadsExpectedPerSample);
  gold_values[0] = { "0.000000000000000000000e+00", "-4.534825437206416972913e+04", "2.106029024981761904201e+04" };
  gold_values[1] = { "0.000000000000000000000e+00", "-4.999912137119322869694e+04", "-2.964154326084487820481e+02" };
  gold_values[2] = { "0.000000000000000000000e+00", "-4.486609665898913226556e+04", "-2.206883256056478785467e+04" };


  for(size_t i = 0; i < numSamples; ++i)
  {
    std::vector<XMLGen::Load> loads = loadCases[i].loads;
    EXPECT_EQ(loads.size(), numLoadsExpectedPerSample);
    for(size_t j = 0; j < loads.size(); ++j)
    {
      XMLGen::Load load = loads[j];
      EXPECT_EQ(load.type, "traction");
      std::vector<std::string> values = load.values;
      EXPECT_EQ(load.values, gold_values[loads.size()*i+j]);
    }
  }

  size_t numObjectives = tester.getNumObjectives();
  EXPECT_EQ(numObjectives, 3u);

  EXPECT_EQ(numSamples % numObjectives, 0u);
  size_t numSamplesPerObjective = numSamples/numObjectives;

  for(size_t i = 0; i < numObjectives; ++i)
    EXPECT_EQ(tester.getObjLoadIds(i).size(), numSamplesPerObjective);
}

TEST(PlatoTestXMLGenerator,uncertainty_sierra)
{
  XMLGenerator_UnitTester tester;
  std::istringstream iss;
  std::string stringInput =
  "begin objective\n"
  "   type maximize stiffness\n"
  "   load ids 10\n"
  "   boundary condition ids 11\n"
  "   code sierra_sd\n"
  "   number processors 1\n"
  "   weight 1\n"
  "   multi load case true\n"
  "   distribute objective at most 3 processors\n"
  "end objective\n"
  "begin boundary conditions\n"
  "   fixed displacement nodeset name 1 bc id 11\n"
  "end boundary conditions\n"
  "begin loads\n"
  "    traction sideset name 2 value 0 -5e4 0 load id 10\n"
  "end loads\n"
  "begin uncertainty\n"
  "    type angle variation\n"
  "    load 10\n"
  "    axis X\n"
  "    distribution beta\n"
  "    mean 0.0\n"
  "    upper bound 45.0\n"
  "    lower bound -45.0\n"
  "    standard deviation 22.5\n"
  "    num samples 3\n"
  "end uncertainty\n";
  // do parse
  iss.str(stringInput);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseObjectives(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseLoads(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseBCs(iss), true);
  iss.clear();
  iss.seekg(0);
  EXPECT_EQ(tester.publicParseUncertainties(iss), true);
  EXPECT_EQ(tester.publicRunSROMForUncertainVariables(), true);
  EXPECT_EQ(tester.publicDistributeObjectivesForGenerate(), true);

  size_t numSamples = tester.getNumSamples();
  size_t numVariables = tester.getNumVariables();
  size_t numPeformers = tester.getNumPerformers();
  EXPECT_EQ(numSamples,3u);
  EXPECT_EQ(numVariables,1u);
  EXPECT_EQ(numPeformers,3u);

  std::vector<size_t> randomVariableIndices = {0u};
  std::vector<size_t> deterministicVariableIndices = {};

  EXPECT_EQ(tester.getRandomVariableIndices(), randomVariableIndices);
  EXPECT_EQ(tester.getDeterministicVariableIndices(), deterministicVariableIndices);

  std::vector<double> gold_loadCaseProbabilities = { 0.36406606886994869, 0.27079380819551258, 0.36514142000522448};

  std::vector<double> loadCaseProbabilities = tester.getLoadCaseProbabilities();
  for(size_t i = 0; i < numSamples; ++i)
    EXPECT_DOUBLE_EQ(loadCaseProbabilities[i],gold_loadCaseProbabilities[i]);

  std::vector<XMLGen::LoadCase> loadCases = tester.getLoadCases();
  EXPECT_EQ(loadCases.size(), numSamples);

  size_t numLoadsExpectedPerSample = 1u;

  std::vector<std::vector<std::string>> gold_values;
  gold_values.resize(numSamples*numLoadsExpectedPerSample);
  gold_values[0] = { "0.000000000000000000000e+00", "-4.534825437206416972913e+04", "2.106029024981761904201e+04" };
  gold_values[1] = { "0.000000000000000000000e+00", "-4.999912137119322869694e+04", "-2.964154326084487820481e+02" };
  gold_values[2] = { "0.000000000000000000000e+00", "-4.486609665898913226556e+04", "-2.206883256056478785467e+04" };


  for(size_t i = 0; i < numSamples; ++i)
  {
    std::vector<XMLGen::Load> loads = loadCases[i].loads;
    EXPECT_EQ(loads.size(), numLoadsExpectedPerSample);
    for(size_t j = 0; j < loads.size(); ++j)
    {
      XMLGen::Load load = loads[j];
      EXPECT_EQ(load.type, "traction");
      std::vector<std::string> values = load.values;
      EXPECT_EQ(load.values, gold_values[loads.size()*i+j]);
    }
  }

  size_t numObjectives = tester.getNumObjectives();
  EXPECT_EQ(numObjectives, 3u);

  EXPECT_EQ(numSamples % numObjectives, 0u);
  size_t numSamplesPerObjective = numSamples/numObjectives;

  for(size_t i = 0; i < numObjectives; ++i)
    EXPECT_EQ(tester.getObjLoadIds(i).size(), numSamplesPerObjective);
}

TEST(PlatoTestXMLGenerator,generatePlatoAnalyzeInputDeck_mechanical_valid)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 1\n"
            "    boundary condition ids 1 2 3\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "end objective\n"
            "begin loads\n"
            "    traction sideset name 2 value 0 -3e3 0 load id 1\n"
            "end loads\n"
            "begin material 1\n"
            "    poissons ratio 0.3\n"
            "    youngs modulus 1e8\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed displacement nodeset name 1 bc id 1\n"
            "    fixed displacement nodeset name 1 x bc id 2\n"
            "    fixed displacement nodeset name 1 y 3.0 bc id 3\n"
            "end boundary conditions\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);
    const XMLGen::InputData& tInputData = tester.getInputData();
    DefaultInputGenerator_UnitTester tGenerator(tInputData);;
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGeneratePlatoAnalyzeInputDecks(&tOStringStream), true);
    EXPECT_EQ(tOStringStream.str(), gMechanicalGoldString);
}

TEST(PlatoTestXMLGenerator,generatePlatoAnalyzeInputDeckNewWriter_mechanical_valid)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 1\n"
            "    boundary condition ids 1 2 3\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "end objective\n"
            "begin loads\n"
            "    traction sideset name 2 value 0 -3e3 0 load id 1\n"
            "end loads\n"
            "begin material 1\n"
            "    poissons ratio 0.3\n"
            "    youngs modulus 1e8\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed displacement nodeset name 1 bc id 1\n"
            "    fixed displacement nodeset name 1 x bc id 2\n"
            "    fixed displacement nodeset name 1 y 3.0 bc id 3\n"
            "end boundary conditions\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);
    const XMLGen::InputData& tInputData = tester.getInputData();
    ComplianceMinTOPlatoAnalyzeInputGenerator_UnitTester tGenerator(tInputData);;
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGeneratePlatoAnalyzeInputDecks(&tOStringStream), true);
    EXPECT_EQ(tOStringStream.str(), gMechanicalGoldString);
}

TEST(PlatoTestXMLGenerator,generateInterfaceXMLWithCompMinTOPlatoAnalyzeWriter)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 1\n"
            "    boundary condition ids 1 2 3\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "end objective\n"
            "begin loads\n"
            "    traction sideset name 2 value 0 -3e3 0 load id 1\n"
            "end loads\n"
            "begin material 1\n"
            "    poissons ratio 0.3\n"
            "    youngs modulus 1e8\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed displacement nodeset name 1 bc id 1\n"
            "    fixed displacement nodeset name 1 x bc id 2\n"
            "    fixed displacement nodeset name 1 y 3.0 bc id 3\n"
            "end boundary conditions\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);
    tester.publicLookForPlatoAnalyzePerformers();
    const XMLGen::InputData& tInputData = tester.getInputData();
    ComplianceMinTOPlatoAnalyzeInputGenerator_UnitTester tGenerator(tInputData);;
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGenerateInterfaceXML(&tOStringStream), true);
    EXPECT_EQ(tOStringStream.str(), gInterfaceXMLCompMinTOPAGoldString);
}

TEST(PlatoTestXMLGenerator,generatePlatoAnalyzeInputDeck_mechanical_duplicate_names)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 1\n"
            "    boundary condition ids 1 2 3\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "end objective\n"
            "begin loads\n"
            "    traction sideset name 1 value 0 -3e3 0 load id 1\n"
            "end loads\n"
            "begin material 1\n"
            "    poissons ratio 0.3\n"
            "    youngs modulus 1e8\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed displacement nodeset name 1 bc id 1\n"
            "    fixed displacement nodeset name 1 x bc id 2\n"
            "    fixed displacement nodeset name 1 y 3.0 bc id 3\n"
            "end boundary conditions\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);

    const XMLGen::InputData& tInputData = tester.getInputData();
    DefaultInputGenerator_UnitTester tGenerator(tInputData);
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGeneratePlatoAnalyzeInputDecks(&tOStringStream), false);
    EXPECT_EQ(tOStringStream.str(), "");

}
TEST(PlatoTestXMLGenerator,generatePlatoAnalyzeInputDeckNewWriter_mechanical_duplicate_names)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize stiffness\n"
            "    load ids 1\n"
            "    boundary condition ids 1 2 3\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "end objective\n"
            "begin loads\n"
            "    traction sideset name 1 value 0 -3e3 0 load id 1\n"
            "end loads\n"
            "begin material 1\n"
            "    poissons ratio 0.3\n"
            "    youngs modulus 1e8\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed displacement nodeset name 1 bc id 1\n"
            "    fixed displacement nodeset name 1 x bc id 2\n"
            "    fixed displacement nodeset name 1 y 3.0 bc id 3\n"
            "end boundary conditions\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);

    const XMLGen::InputData& tInputData = tester.getInputData();
    ComplianceMinTOPlatoAnalyzeInputGenerator_UnitTester tGenerator(tInputData);
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGeneratePlatoAnalyzeInputDecks(&tOStringStream), false);
    EXPECT_EQ(tOStringStream.str(), "");

}

TEST(PlatoTestXMLGenerator,generatePlatoAnalyzeInputDeck_thermal)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type maximize heat conduction\n"
            "    load ids 1\n"
            "    boundary condition ids 1 2 3\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "end objective\n"
            "begin loads\n"
            "    heat flux sideset name ss_1 value -1e2 load id 1\n"
            "end loads\n"
            "begin material 1\n"
            "    density 2703\n"
            "    specific heat 900\n"
            "    thermal conductivity coefficient 210.0\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed temperature nodeset name 1 bc id 1\n"
            "    fixed temperature nodeset name 2 bc id 2\n"
            "    fixed temperature nodeset name 3 value 25.0 bc id 3\n"
            "end boundary conditions\n";

    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);

    const XMLGen::InputData& tInputData = tester.getInputData();
    DefaultInputGenerator_UnitTester tGenerator(tInputData);
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGeneratePlatoAnalyzeInputDecks(&tOStringStream), true);
    EXPECT_EQ(tOStringStream.str(), gThermalGoldString);

}

TEST(PlatoTestXMLGenerator,generatePlatoAnalyzeInputDeck_thermoelastic)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput =
            "begin objective\n"
            "    type minimize thermoelastic energy\n"
            "    load ids 1 2\n"
            "    boundary condition ids 1 2 3 4 5 6 7 8\n"
            "    code plato_analyze\n"
            "    number processors 2\n"
            "end objective\n"
            "begin loads\n"
            "    traction sideset name ss_1 value 0.0 1.0e5 0.0 load id 1\n"
            "    heat flux sideset name ss_1 value 0.0 load id 2\n"
            "end loads\n"
            "begin material 1\n"
            "    poissons ratio 0.3\n"
            "    youngs modulus 1e11\n"
            "    thermal expansion coefficient 1e-5\n"
            "    thermal conductivity coefficient 910.0\n"
            "    reference temperature 1e-2\n"
            "end material\n"
            "begin boundary conditions\n"
            "    fixed displacement nodeset name 1 y bc id 1\n"
            "    fixed displacement nodeset name 1 z bc id 2\n"
            "    fixed temperature nodeset name 1 bc id 3\n"
            "    fixed displacement nodeset name 11 x bc id 4\n"
            "    fixed displacement nodeset name 2 y bc id 5\n"
            "    fixed displacement nodeset name 2 z bc id 6\n"
            "    fixed temperature nodeset name 2 bc id 7\n"
            "    fixed displacement nodeset name 21 x bc id 8\n"
            "end boundary conditions\n";



    // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseObjectives(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseLoads(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseBCs(iss), true);
    iss.clear();
    iss.seekg(0);
    EXPECT_EQ(tester.publicParseMaterials(iss), true);

    const XMLGen::InputData& tInputData = tester.getInputData();
    DefaultInputGenerator_UnitTester tGenerator(tInputData);
    std::ostringstream tOStringStream;
    EXPECT_EQ(tGenerator.publicGeneratePlatoAnalyzeInputDecks(&tOStringStream), true);
    EXPECT_EQ(tOStringStream.str(), gThermomechanicalGoldString);

}

} // end PlatoTestXMLGenerator namespace
