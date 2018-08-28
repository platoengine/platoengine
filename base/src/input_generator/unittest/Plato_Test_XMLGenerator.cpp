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
    EXPECT_EQ(tester.getMeshName(), "file");
}
TEST(PlatoTestXMLGenerator, parseCodePaths)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    stringInput = "begin paths\n"
            "code salinas\n"
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
            "code salinas /Users/bwclark/salinas\n"
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
            "id\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), false);
    stringInput = "begin block\n"
            "material\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), false);
    stringInput = "begin block\n"
            "bad_keywordl\n"
            "end block\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseBlocks(iss), false);
    stringInput = "begin block\n"
            "id 1\n"
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
    stringInput = "begin block\n"
            "material 89\n"
            "id 44\n"
            "end block\n"
            "begin block\n"
            "id 33\n"
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

    stringInput = "begin material\n"
            "penalty exponent\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material\n"
            "id\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material\n"
            "youngs modulus\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material\n"
            "poissons ratio\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material\n"
            "density\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material\n"
            "thermal conductivity\n"
            "end material\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseMaterials(iss), false);
    stringInput = "begin material\n"
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
    stringInput = "begin material\n"
            "id 34\n"
            "penalty exponent 3\n"
            "youngs modulus 1e6\n"
            "poissons ratio 0.33\n"
            "thermal conductivity .02\n"
            "density .001\n"
            "end material\n"
            "begin material\n"
            "id 388\n"
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
            "code salinas\n"
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

    // Test the "salinas weight mass scale factor" keywords
    stringInput = "begin objective\n"
            "salinas weight mass scale factor\n"
            "end objective\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    EXPECT_EQ(tester.publicParseObjectives(iss), false);

    // Test the "salinas gdsw tolerance" keywords
    stringInput = "begin objective\n"
            "salinas gdsw tolerance\n"
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
            "code salinas\n"
            "name obj1\n"
            "end objective\n"
            "begin objective\n"
            "code salinas\n"
            "end objective\n"
            "begin objective\n"
            "code albany\n"
            "end objective\n"
            "begin objective\n"
            "code salinas\n"
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
    EXPECT_EQ(tester.getObjCodeName(0), "salinas");
    EXPECT_EQ(tester.getObjCodeName(1), "salinas");
    EXPECT_EQ(tester.getObjCodeName(2), "albany");
    EXPECT_EQ(tester.getObjCodeName(3), "salinas");
    EXPECT_EQ(tester.getObjCodeName(4), "lightmp");
    EXPECT_EQ(tester.getObjPerfName(0), "salinas_obj1");
    EXPECT_EQ(tester.getObjPerfName(1), "salinas_2");
    EXPECT_EQ(tester.getObjPerfName(2), "albany_1");
    EXPECT_EQ(tester.getObjPerfName(3), "salinas_3");
    EXPECT_EQ(tester.getObjPerfName(4), "lightmp_1");
}
} // end PlatoTestXMLGenerator namespace
