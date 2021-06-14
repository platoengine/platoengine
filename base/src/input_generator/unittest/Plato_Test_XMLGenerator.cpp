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

#include <map>
#include <cmath>
#include <numeric>

#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorParseUncertainty.hpp"
#include "XMLGenerator_UnitTester.hpp"
#include "XML_GoldValues.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"
#include "XMLGenerator_UnitTester_Tools.hpp"

#include "Plato_Utils.hpp"
#include "XMLG_Macros.hpp"

const int MAX_CHARS_PER_LINE = 512;

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, ComputeGreatestDivisor)
{
    size_t tNumSamples = 10;
    size_t tNumPerformers = 2;
    auto tOutput = XMLGen::compute_greatest_divisor(tNumSamples, tNumPerformers);
    ASSERT_EQ(2u, tOutput);

    tNumPerformers = 3;
    tOutput = XMLGen::compute_greatest_divisor(tNumSamples, tNumPerformers);
    ASSERT_EQ(2u, tOutput);

    tNumPerformers = 4;
    tOutput = XMLGen::compute_greatest_divisor(tNumSamples, tNumPerformers);
    ASSERT_EQ(2u, tOutput);

    tNumPerformers = 6;
    tOutput = XMLGen::compute_greatest_divisor(tNumSamples, tNumPerformers);
    ASSERT_EQ(5u, tOutput);

    tNumPerformers = 0;
    ASSERT_THROW(XMLGen::compute_greatest_divisor(tNumSamples, tNumPerformers), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, TransformTokens)
{
    // TEST 1
    std::vector<std::string> tTokens;
    auto tOutput = XMLGen::transform_tokens(tTokens);
    ASSERT_STREQ("", tOutput.c_str());

    // TEST 2
    tTokens = {"1", "2", "3", "4", "5", "6"};
    tOutput = XMLGen::transform_tokens(tTokens);
    ASSERT_STREQ("1, 2, 3, 4, 5, 6", tOutput.c_str());
}

TEST(PlatoTestXMLGenerator, ParseTagValues)
{
    XMLGen::MetaDataTags tTags;
    tTags.insert({ "tag", { { {"tag"}, ""}, "" } });
    tTags.insert({ "attribute", { { {"attribute"}, ""}, "" } });
    tTags.insert({ "load", { { {"load"}, ""}, "" } });
    tTags.insert({ "mean", { { {"mean"}, ""}, "" } });
    tTags.insert({ "distribution", { { {"distribution"}, ""}, "" } });
    tTags.insert({ "num samples", { { {"num", "samples"}, ""}, "" } });
    tTags.insert({ "lower bound", { { {"lower", "bound"}, ""}, "" } });
    tTags.insert({ "upper bound", { { {"upper", "bound"}, ""}, "" } });
    tTags.insert({ "category", { { {"category"}, ""}, "" } });
    tTags.insert({ "standard deviation", { { {"standard", "deviation"}, ""}, "" } });

    XMLGen::parse_tag_values(std::vector<std::string>{"tag", "angle", "variation"}, tTags);
    ASSERT_STREQ("angle variation", tTags.find("tag")->second.first.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"attribute", "x"}, tTags);
    ASSERT_STREQ("x", tTags.find("attribute")->second.first.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"load", "10"}, tTags);
    ASSERT_STREQ("10", tTags.find("load")->second.first.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"mean", "1"}, tTags);
    ASSERT_STREQ("1", tTags.find("mean")->second.first.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"distribution", "beta"}, tTags);
    ASSERT_STREQ("beta", tTags.find("distribution")->second.first.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"num", "samples", "4"}, tTags);
    ASSERT_STREQ("4", tTags.find("num samples")->second.first.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"lower", "bound", "0.5"}, tTags);
    ASSERT_STREQ("0.5", tTags.find("lower bound")->second.first.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"upper", "bound", "2"}, tTags);
    ASSERT_STREQ("2", tTags.find("upper bound")->second.first.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"category", "load"}, tTags);
    ASSERT_STREQ("load", tTags.find("category")->second.first.second.c_str());
    XMLGen::parse_tag_values(std::vector<std::string>{"standard", "deviation", "0.2"}, tTags);
    ASSERT_STREQ("0.2", tTags.find("standard deviation")->second.first.second.c_str());
}

TEST(PlatoTestXMLGenerator, ParseUncertainty_ErrorInvalidMean)
{
    std::string tStringInput =
        "begin uncertainty\n"
        "    category load\n"
        "    tag angle_variation\n"
        "    load id 10\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean -50.0\n"
        "    upper bound 45.0\n"
        "    lower bound -45.0\n"
        "    standard deviation 22.5\n"
        "    num samples 2\n"
        "end uncertainty\n";

    std::istringstream tInputs;
    tInputs.str(tStringInput);
    XMLGen::ParseUncertainty tUncertainty;
    ASSERT_THROW(tUncertainty.parse(tInputs), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseUncertainty_ErrorMeanMinusStdLesserThanLowerBound)
{
    std::string tStringInput =
        "begin uncertainty\n"
        "    category load\n"
        "    tag angle_variation\n"
        "    load id 10\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean -20.0\n"
        "    upper bound 45.0\n"
        "    lower bound -45.0\n"
        "    standard deviation 30\n"
        "    num samples 2\n"
        "end uncertainty\n";

    std::istringstream tInputs;
    tInputs.str(tStringInput);
    XMLGen::ParseUncertainty tUncertainty;
    ASSERT_THROW(tUncertainty.parse(tInputs), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseUncertainty_ErrorMeanPlusStdGreaterThanUpperBound)
{
    std::string tStringInput =
        "begin uncertainty\n"
        "    category load\n"
        "    tag angle_variation\n"
        "    load id 10\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean 20.0\n"
        "    upper bound 45.0\n"
        "    lower bound -45.0\n"
        "    standard deviation 30\n"
        "    num samples 2\n"
        "end uncertainty\n";

    std::istringstream tInputs;
    tInputs.str(tStringInput);
    XMLGen::ParseUncertainty tUncertainty;
    ASSERT_THROW(tUncertainty.parse(tInputs), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseUncertainty_OneRandomVar)
{
    std::string tStringInput =
        "begin load 10\n"
        "  type traction\n"
        "  location_type sideset\n"
        "  location_name ss_2\n"
        "  value 0 -5e4 0\n"
        "end load\n"
        "begin uncertainty\n"
        "    category load\n"
        "    tag angle_variation\n"
        "    load_id 10\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean 0.0\n"
        "    upper_bound 45.0\n"
        "    lower_bound -45.0\n"
        "    standard_deviation 22.5\n"
        "    number_samples 2\n"
        "end uncertainty\n";

    std::istringstream tInputs;
    tInputs.str(tStringInput);
    XMLGen::ParseUncertainty tUncertainty;
    tUncertainty.parse(tInputs);

    auto tMetadata = tUncertainty.data();
    ASSERT_STREQ("10", tMetadata[0].id().c_str());
    ASSERT_STREQ("x", tMetadata[0].attribute().c_str());
    ASSERT_STREQ("0.0", tMetadata[0].mean().c_str());
    ASSERT_STREQ("45.0", tMetadata[0].upper().c_str());
    ASSERT_STREQ("-45.0", tMetadata[0].lower().c_str());
    ASSERT_STREQ("2", tMetadata[0].samples().c_str());
    ASSERT_STREQ("beta", tMetadata[0].distribution().c_str());
    ASSERT_STREQ("load", tMetadata[0].category().c_str());
    ASSERT_STREQ("angle_variation", tMetadata[0].tag().c_str());
    ASSERT_STREQ("22.5", tMetadata[0].std().c_str());
}

TEST(PlatoTestXMLGenerator, ParseUncertainty_TwoRandomVar)
{
    std::string tStringInput =
        "begin load 10\n"
        "  type traction\n"
        "  location_type sideset\n"
        "  location_name ss_2\n"
        "  value 0 -5e4 0\n"
        "end load\n"
        "begin material 1\n"
            "material_model isotropic linear elastic\n"
            "youngs_modulus 1e6\n"
            "poissons_ratio 0.33\n"
        "end material\n"
        "begin uncertainty\n"
        "    category load\n"
        "    load_id 10\n"
        "    tag angle_variation\n"
        "    attribute X\n"
        "    distribution beta\n"
        "    mean 0.0\n"
        "    upper_bound 45.0\n"
        "    lower_bound -45.0\n"
        "    standard_deviation 22.5\n"
        "    number_samples 2\n"
        "end uncertainty\n"
        "begin uncertainty\n"
        "    category material\n"
        "    material_id 1\n"
        "    tag poissons_ratio\n"
        "    attribute homogeneous\n"
        "    distribution beta\n"
        "    mean 0.28\n"
        "    upper_bound 0.4\n"
        "    lower_bound 0.2\n"
        "    standard_deviation 0.05\n"
        "    number_samples 3\n"
        "end uncertainty\n";

    std::istringstream tInputs;
    tInputs.str(tStringInput);
    XMLGen::ParseUncertainty tUncertainty;
    tUncertainty.parse(tInputs);

    std::vector<std::string> tGoldID = {"10", "1"};
    std::vector<std::string> tGoldMean = {"0.0", "0.28"};
    std::vector<std::string> tGoldUpper = {"45.0", "0.4"};
    std::vector<std::string> tGoldNumSamples = {"2", "3"};
    std::vector<std::string> tGoldLower = {"-45.0", "0.2"};
    std::vector<std::string> tGoldStdDev = {"22.5", "0.05"};
    std::vector<std::string> tGoldCategory = {"load", "material"};
    std::vector<std::string> tGoldDistribution = {"beta", "beta"};
    std::vector<std::string> tGoldAttribute = {"x", "homogeneous"};
    std::vector<std::string> tGoldTag = {"angle_variation", "poissons_ratio"};
    auto tMetadata = tUncertainty.data();
    for (auto& tVar : tMetadata)
    {
        auto tIndex = &tVar - &tMetadata[0];
        ASSERT_STREQ(tGoldID[tIndex].c_str(), tVar.id().c_str());
        ASSERT_STREQ(tGoldMean[tIndex].c_str(), tVar.mean().c_str());
        ASSERT_STREQ(tGoldUpper[tIndex].c_str(), tVar.upper().c_str());
        ASSERT_STREQ(tGoldLower[tIndex].c_str(), tVar.lower().c_str());
        ASSERT_STREQ(tGoldAttribute[tIndex].c_str(), tVar.attribute().c_str());
        ASSERT_STREQ(tGoldCategory[tIndex].c_str(), tVar.category().c_str());
        ASSERT_STREQ(tGoldNumSamples[tIndex].c_str(), tVar.samples().c_str());
        ASSERT_STREQ(tGoldStdDev[tIndex].c_str(), tVar.std().c_str());
        ASSERT_STREQ(tGoldDistribution[tIndex].c_str(), tVar.distribution().c_str());
    }
}

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
    stringInput = "begin optimization_parameters\n"
            "levelset_material_box_min 0 0 0\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "levelset_material_box_max 0 0 0\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "levelset_material_box_min 0 0 1\n"
            "levelset_material_box_max 0 0\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "levelset_material_box_min 0 0\n"
            "levelset_material_box_max 0 0 1\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "levelset_material_box_min 1 2 3\n"
            "levelset_material_box_max 4 5 6\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_NO_THROW(tester.publicParseOptimizationParameters(iss));
    EXPECT_EQ(tester.getMatBoxMinCoords(), "1 2 3");
    EXPECT_EQ(tester.getMatBoxMaxCoords(), "4 5 6");

    // initial_density_value
    stringInput = "begin optimization_parameters\n"
            "initial_density_value\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "initial_density_value .1\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getInitDensityValue(), ".1");

    // create levelset spheres
    stringInput = "begin optimization_parameters\n"
            "create_levelset_spheres\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "create_levelset_spheres true\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getCreateLevelsetSpheres(), "true");

    // levelset initialization method
    stringInput = "begin optimization_parameters\n"
            "levelset_initialization_method\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "levelset_initialization_method primitives\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getLevelsetInitMethod(), "primitives");

    // max iterations
    stringInput = "begin optimization_parameters\n"
            "max_iterations\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "max_iterations 45\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getMaxIterations(), "45");

    // restart iteration
    stringInput = "begin optimization_parameters\n"
            "restart_iteration\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "restart_iteration 22\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getRestartIteration(), "1");
    stringInput = "begin optimization_parameters\n"
            "initial_guess_file_name file.gen\n"
            "initial_guess_field_name topology\n"
            "restart_iteration 22\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getRestartIteration(), "22");
    EXPECT_EQ(tester.getRestartFieldName(), "topology");
    EXPECT_EQ(tester.getRestartMeshFilename(), "file.gen");
    stringInput = "begin optimization_parameters\n"
            "initial_guess_file_name fIle.gen\n"
            "initial_guess_field_name TopologY\n"
            "restart_iteration 22\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getRestartIteration(), "22");
    EXPECT_EQ(tester.getRestartFieldName(), "topology");
    EXPECT_EQ(tester.getRestartMeshFilename(), "file.gen");

    // ks max trust region iterations
    stringInput = "begin optimization_parameters\n"
            "ks_max_trust_region_iterations\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "ks_max_trust_region_iterations 10\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSMaxTrustIterations(), "10");

    // ks trust region ratio low/mid/upper
    stringInput = "begin optimization_parameters\n"
            "ks_trust_region_ratio_low\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "ks_trust_region_ratio_mid\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "ks_trust_region_ratio_high\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "ks_trust_region_ratio_low 1.1\n"
            "ks_trust_region_ratio_mid 2.1\n"
            "ks_trust_region_ratio_high 3.1\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().ks_trust_region_ratio_low(), "1.1");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().ks_trust_region_ratio_mid(), "2.1");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().ks_trust_region_ratio_high(), "3.1");

    // ks disable post smoothing
    stringInput = "begin optimization_parameters\n"
            "ks_disable_post_smoothing\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "end optimization_parameters\n";
    stringInput = "begin optimization_parameters\n"
            "ks_disable_post_smoothing fALse\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().ks_disable_post_smoothing(), "false");
    stringInput = "begin optimization_parameters\n"
            "ks_disable_post_smoothing tRuE\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().ks_disable_post_smoothing(), "true");


    // ks trust region expansion factor
    stringInput = "begin optimization_parameters\n"
            "ks_trust_region_expansion_factor\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "ks_trust_region_expansion_factor .5\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSExpansionFactor(), ".5");

    // ks trust region contraction factor
    stringInput = "begin optimization_parameters\n"
            "ks_trust_region_contraction_factor\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "ks_trust_region_contraction_factor .2\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSContractionFactor(), ".2");

    // ks outer gradient tolerance
    stringInput = "begin optimization_parameters\n"
            "ks_outer_gradient_tolerance\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "ks_outer_gradient_tolerance .001\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterGradientTolerance(), ".001");

    // ks outer stationarity tolerance
    stringInput = "begin optimization_parameters\n"
            "ks_outer_stationarity_tolerance\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "ks_outer_stationarity_tolerance .00001\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterStationarityTolerance(), ".00001");

    // ks outer stagnation tolerance
    stringInput = "begin optimization_parameters\n"
            "ks_outer_stagnation_tolerance\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "ks_outer_stagnation_tolerance 2.3\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterStagnationTolerance(), "2.3");

    // ks outer control stagnation tolerance
    stringInput = "begin optimization_parameters\n"
            "ks_outer_control_stagnation_tolerance\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "ks_outer_control_stagnation_tolerance 1e-8\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterControlStagnationTolerance(), "1e-8");

    // ks outer actual reduction tolerance
    stringInput = "begin optimization_parameters\n"
            "ks_outer_actual_reduction_tolerance\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "ks_outer_actual_reduction_tolerance 1e-7\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getKSOuterActualReductionTolerance(), "1e-7");

    // gcmma max inner iterations
    stringInput = "begin optimization_parameters\n"
            "gcmma_max_inner_iterations\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "gcmma_max_inner_iterations 100\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAMaxInnerIterations(), "100");

    // gcmma inner kkt tolerance
    stringInput = "begin optimization_parameters\n"
            "gcmma_inner_kkt_tolerance\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "gcmma_inner_kkt_tolerance 1e-12\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAInnerKKTTolerance(), "1e-12");

    // gcmma inner control stagnation tolerance
    stringInput = "begin optimization_parameters\n"
            "gcmma_inner_control_stagnation_tolerance\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "gcmma_inner_control_stagnation_tolerance 5e-10\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAInnerControlStagnationTolerance(), "5e-10");

    // gcmma outer kkt tolerance
    stringInput = "begin optimization_parameters\n"
            "gcmma_outer_kkt_tolerance\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "gcmma_outer_kkt_tolerance 1e-8\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAOuterKKTTolerance(), "1e-8");

    // gcmma outer control stagnation tolerance
    stringInput = "begin optimization_parameters\n"
            "gcmma_outer_control_stagnation_tolerance\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "gcmma_outer_control_stagnation_tolerance 1e-11\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAOuterControlStagnationTolerance(), "1e-11");

    // gcmma outer objective stagnation tolerance
    stringInput = "begin optimization_parameters\n"
            "gcmma_outer_objective_stagnation_tolerance\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "gcmma_outer_objective_stagnation_tolerance 4e-8\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAOuterObjectiveStagnationTolerance(), "4e-8");

    // gcmma outer stationarity tolerance
    stringInput = "begin optimization_parameters\n"
            "gcmma_outer_stationarity_tolerance\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "gcmma_outer_stationarity_tolerance 2e-10\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAOuterStationarityTolerance(), "2e-10");

    // gcmma initial moving asymptotes scale factor
    stringInput = "begin optimization_parameters\n"
            "gcmma_initial_moving_asymptotes_scale_factor\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "gcmma_initial_moving_asymptotes_scale_factor 0.4\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getGCMMAInitialMovingAsymptotesScaleFactor(), "0.4");

    // levelset sphere packing factor
    stringInput = "begin optimization_parameters\n"
            "levelset_sphere_packing_factor\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "levelset_sphere_packing_factor .2\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getLevelsetSpherePackingFactor(), ".2");

    // levelset sphere radius
    stringInput = "begin optimization_parameters\n"
            "levelset_sphere_radius\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "levelset_sphere_radius .33\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getLevelsetSphereRadius(), ".33");

    // levelset nodesets
    stringInput = "begin optimization_parameters\n"
            "levelset_nodesets\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "levelset_nodesets 4 10 22\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getLevelsetNodeset(0), "4");
    EXPECT_EQ(tester.getLevelsetNodeset(1), "10");
    EXPECT_EQ(tester.getLevelsetNodeset(2), "22");

    // fixed blocks
    stringInput = "begin optimization_parameters\n"
            "fixed_block_ids\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "fixed_block_ids 1 3 5\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFixedBlock(0), "1");
    EXPECT_EQ(tester.getFixedBlock(1), "3");
    EXPECT_EQ(tester.getFixedBlock(2), "5");

    // fixed sidesets
    stringInput = "begin optimization_parameters\n"
            "fixed_sideset_ids\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "fixed_sideset_ids 33 44 55\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFixedSideset(0), "33");
    EXPECT_EQ(tester.getFixedSideset(1), "44");
    EXPECT_EQ(tester.getFixedSideset(2), "55");

    // fixed nodesets
    stringInput = "begin optimization_parameters\n"
            "fixed_nodeset_ids\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "fixed_nodeset_ids 5 7 33\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFixedNodeset(0), "5");
    EXPECT_EQ(tester.getFixedNodeset(1), "7");
    EXPECT_EQ(tester.getFixedNodeset(2), "33");

    // filter scale
    stringInput = "begin optimization_parameters\n"
            "filter_radius_scale\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "filter_radius_scale 1.6\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFilterScale(), "1.6");

    // filter type
    stringInput = "begin optimization_parameters\n"
            "filter_type kernel\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().filter_type(),"kernel");
    stringInput = "begin optimization_parameters\n"
            "filter_type kernel_then_heaviside\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().filter_type(),"kernel_then_heaviside");
    stringInput = "begin optimization_parameters\n"
            "filter_type\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "filter_type lions_and_tigers\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);

    // filter heaviside min
    stringInput = "begin optimization_parameters\n"
            "filter_heaviside_min\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "filter_heaviside_min 1.526\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().filter_heaviside_min(),"1.526");

    // filter heaviside update
    stringInput = "begin optimization_parameters\n"
            "filter_heaviside_update\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "filter_heaviside_update 1.526\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().filter_heaviside_update(),"1.526");

    // filter heaviside max
    stringInput = "begin optimization_parameters\n"
            "filter_heaviside_max\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "filter_heaviside_max 1.526\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().filter_heaviside_max(),"1.526");

    // filter heaviside scale
    stringInput = "begin optimization_parameters\n"
            "filter_heaviside_scale\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "filter_type kernel_then_heaviside\n"
            "filter_heaviside_scale 1.526\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().filter_heaviside_min(),"1.526");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().filter_heaviside_max(),"1.526");

    // filter heaviside various
    stringInput = "begin optimization_parameters\n"
            "filter_type kernel_then_heaviside\n"
            "filter_heaviside_update 4.4\n"
            "filter_heaviside_scale 1.2\n"
            "filter_heaviside_min 0.5\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "filter_type kernel_then_heaviside\n"
            "filter_heaviside_update 4.4\n"
            "filter_heaviside_scale 1.2\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().filter_heaviside_min(),"1.2");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().filter_heaviside_update(),"4.4");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().filter_heaviside_max(),"1.2");
    stringInput = "begin optimization_parameters\n"
            "filter_type kernel_then_heaviside\n"
            "filter_heaviside_update 4.4\n"
            "filter_heaviside_min 0.5\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().filter_heaviside_min(),"0.5");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().filter_heaviside_update(),"4.4");

    // filter absolute
    stringInput = "begin optimization_parameters\n"
            "filter_radius_absolute\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "filter_radius_absolute 1.6\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFilterAbsolute(), "1.6");

    // filter absolute
    stringInput = "begin optimization_parameters\n"
            "filter_radius_absolute\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "filter_radius_absolute 1.6\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFilterAbsolute(), "1.6");

    // filter radial power
    stringInput = "begin optimization_parameters\n"
            "filter_power\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "filter_power 2.5\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getFilterPower(), "2.5");

    // algorithm
    stringInput = "begin optimization_parameters\n"
            "optimization_algorithm\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "optimization_algorithm ksbc\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getAlgorithm(), "ksbc");

    // discretization
    stringInput = "begin optimization_parameters\n"
            "discretization\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "discretization levelset\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getDiscretization(), "levelset");

    // check gradient
    stringInput = "begin optimization_parameters\n"
            "check_gradient\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "check_gradient false\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getCheckGradient(), "false");

    // check hessian
    stringInput = "begin optimization_parameters\n"
            "check_hessian\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_THROW(tester.publicParseOptimizationParameters(iss), std::runtime_error);
    stringInput = "begin optimization_parameters\n"
            "check_hessian true\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    EXPECT_EQ(tester.publicParseOptimizationParameters(iss), true);
    EXPECT_EQ(tester.getCheckHessian(), "true");
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

TEST(PlatoTestXMLGenerator, SROM_SolveSromProblem_ReadSampleProbPairsFromFile)
{
    // POSE PROBLEM
    XMLGenerator_UnitTester tTester;
    std::istringstream iss;
    std::string stringInput =
      "begin service 1\n"
      "  code platomain\n"
      "  number_processors 1\n"
      "end service\n"
      "begin service 2\n"
      "  code plato_analyze\n"
      "  number_processors 1\n"
      "end service\n"
      "begin criterion 1\n"
      "  type mechanical_compliance\n"
      "end criterion\n"
      "begin scenario 1\n"
      "  physics steady_state_mechanics\n"
      "  dimensions 3\n"
      "  loads 10\n"
      "  boundary_conditions 1 2 3\n"
      "  material 1\n"
      "end scenario\n"
      "begin objective\n"
      "  scenarios 1\n"
      "  criteria 1\n"
      "  services 2\n"
      "  type weighted_sum\n"
      "  weights 1\n"
      "end objective\n"
      "begin boundary_condition 1\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispx\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 2\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispy\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 3\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispz\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin load 10\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_2\n"
      "  value 0 -5e4 0\n"
      "end load\n"
      "begin block 1\n"
      "  material 1\n"
      "end block\n"
      "begin material 1\n"
      "  material_model isotropic_linear_elastic\n"
      "  poissons_ratio 0.33\n"
      "  youngs_modulus 1e6\n"
      "end material\n"
      "begin uncertainty\n"
      "  category load\n"
      "  tag angle_variation\n"
      "  load_id 10\n"
      "  attribute X\n"
      "  filename test.csv\n"
      "end uncertainty\n"
      "begin optimization_parameters\n"
      "end optimization_parameters\n";

    // WRITE SAMPLE-PROBABILITY PAIRS TO FILE
    int tPrecision = 64;
    std::string tFilename("test.csv");
    std::vector<Plato::srom::DataPairs> tGoldDataSet;
    tGoldDataSet.push_back( { "Samples", std::vector<double>{} } );
    tGoldDataSet[0].second =
        {-18.124227441680492489695097901858389377593994140625, 15.69452170045176586654633865691721439361572265625};
    tGoldDataSet.push_back( { "Probabilities", std::vector<double>{} } );
    tGoldDataSet[1].second =
        {0.361124680672662068392497758395620621740818023681640625, 0.638872868975587149265038533485494554042816162109375};
    Plato::srom::write_data(tFilename, tGoldDataSet, tPrecision);

    // PARSE INPUTS AND RUN THE PROBLEM
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(iss));
    iss.clear();
    iss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(iss));
    iss.clear();
    iss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(iss));
    iss.clear();
    iss.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(iss));
    iss.clear();
    iss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(iss));
    iss.clear();
    iss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseUncertainties(iss));
    iss.clear();
    iss.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(iss));
    iss.clear();
    iss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(iss));

    auto tXMLGenMetadata = tTester.getInputData();
    EXPECT_EQ(tTester.publicRunSROMForUncertainVariables(tXMLGenMetadata), true);

    auto tNumSamples = tXMLGenMetadata.mRandomMetaData.numSamples();
    auto tNumPerformers = tXMLGenMetadata.m_UncertaintyMetaData.numPerformers;
    EXPECT_EQ(tNumSamples,2u);
    EXPECT_EQ(tNumPerformers,1u);

    // TEST SAMPLES
    std::vector<std::string> tGoldLoadCaseProbabilities = { "0.36112468067266207", "0.63887286897558715" };
    std::vector<std::vector<std::string>> tGoldValues =
      {
        { "0.000000000000000000000e+00", "-4.751921387767659325618e+04", "1.555391630579348566243e+04" },
        { "0.000000000000000000000e+00", "-4.813588076578034088016e+04", "-1.352541987897522631101e+04"}
      };

    const double tTolerance = 1e-10;
    auto tSamples = tXMLGenMetadata.mRandomMetaData.samples();
    for(auto& tSample : tSamples)
    {
        auto tSampleIndex = &tSample - &tSamples[0];
        ASSERT_NEAR(std::stod(tGoldLoadCaseProbabilities[tSampleIndex]), std::stod(tSample.probability()), tTolerance);

        for(auto& tLoad : tSample.loadcase().loads)
        {
            ASSERT_STREQ("traction", tLoad.type().c_str());
            for(auto& tValue : tLoad.load_values())
            {
                auto tComponent = &tValue - &tLoad.load_values()[0];
                ASSERT_NEAR(std::stod(tValue), std::stod(tGoldValues[tSampleIndex][tComponent]), tTolerance);
            }
        }
    }

    Plato::system("rm -f test.csv");
}

TEST(PlatoTestXMLGenerator, uncertainty_analyzeNewWorkflow)
{
    // POSE PROBLEM
  XMLGenerator_UnitTester tTester;
  std::istringstream iss;
  std::string stringInput =
      "begin service 1\n"
      "  code platomain\n"
      "  number_processors 1\n"
      "end service\n"
      "begin service 2\n"
      "  code plato_analyze\n"
      "  number_processors 1\n"
      "end service\n"
      "begin criterion 1\n"
      "  type mechanical_compliance\n"
      "end criterion\n"
      "begin scenario 1\n"
      "  physics steady_state_mechanics\n"
      "  dimensions 3\n"
      "  loads 10\n"
      "  boundary_conditions 1 2 3\n"
      "  material 1\n"
      "end scenario\n"
      "begin objective\n"
      "  scenarios 1\n"
      "  criteria 1\n"
      "  services 2\n"
      "  type weighted_sum\n"
      "  weights 1\n"
      "end objective\n"
      "begin boundary_condition 1\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispx\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 2\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispy\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 3\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispz\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin load 10\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_2\n"
      "  value 0 -5e4 0\n"
      "end load\n"
      "begin block 1\n"
      "  material 1\n"
      "end block\n"
      "begin material 1\n"
      "  material_model isotropic_linear_elastic\n"
      "  poissons_ratio 0.33\n"
      "  youngs_modulus 1e6\n"
      "end material\n"
      "begin uncertainty\n"
      "  category load\n"
      "  tag angle_variation\n"
      "  load_id 10\n"
      "  attribute X\n"
      "  distribution beta\n"
      "  mean 0.0\n"
      "  upper_bound 45.0\n"
      "  lower_bound -45.0\n"
      "  standard_deviation 22.5\n"
      "  number_samples 2\n"
      "  initial_guess uniform\n"
      "end uncertainty\n"
      "begin optimization_parameters\n"
      "end optimization_parameters\n";
  // do parse
    iss.str(stringInput);
    iss.clear();
    iss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(iss));
    iss.clear();
    iss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(iss));
    iss.clear();
    iss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(iss));
    iss.clear();
    iss.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(iss));
    iss.clear();
    iss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(iss));
    iss.clear();
    iss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseUncertainties(iss));
    iss.clear();
    iss.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(iss));
    iss.clear();
    iss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(iss));

  auto tXMLGenMetadata = tTester.getInputData();
  EXPECT_EQ(tTester.publicRunSROMForUncertainVariables(tXMLGenMetadata), true);

  auto tNumSamples = tXMLGenMetadata.mRandomMetaData.numSamples();
  size_t tNumPerformers = tXMLGenMetadata.m_UncertaintyMetaData.numPerformers;
  EXPECT_EQ(tNumSamples,2u);
  EXPECT_EQ(tNumPerformers,1u);

  // TEST SAMPLES
  std::vector<std::string> tGoldLoadCaseProbabilities = { "0.36112074847743636", "0.6388796023915212" };
  std::vector<std::vector<std::string>> tGoldValues =
      {
        { "0.000000000000000000000e+00", "-47518.476388930852", "15556.169248070713" },
        { "0.000000000000000000000e+00", "-4.813588076578034088016e+04", "-1.352541987897522631101e+04"}
      };

  const double tTolerance = 1e-6;
  auto tSamples = tXMLGenMetadata.mRandomMetaData.samples();
  for(auto& tSample : tSamples)
  {
      auto tSampleIndex = &tSample - &tSamples[0];
      ASSERT_NEAR(std::stod(tGoldLoadCaseProbabilities[tSampleIndex]), std::stod(tSample.probability()), tTolerance);

      for(auto& tLoad : tSample.loadcase().loads)
      {
          ASSERT_STREQ("traction", tLoad.type().c_str());
          for(auto& tValue : tLoad.load_values())
          {
              auto tComponent = &tValue - &tLoad.load_values()[0];
              ASSERT_NEAR(std::stod(tValue), std::stod(tGoldValues[tSampleIndex][tComponent]), tTolerance);
          }
      }
  }

  Plato::system("rm -f plato_cdf_output.txt");
  Plato::system("rm -f plato_srom_diagnostics.txt");
  Plato::system("rm -f plato_ksal_algorithm_diagnostics.txt");
}

TEST(PlatoTestXMLGenerator,uncertainty_analyzeNewWorkflow_randomPlusDeterministic)
{
    // POSE INPUT DATA
    XMLGenerator_UnitTester tTester;
    std::istringstream tIss;
    std::string tStringInput =
      "begin service 1\n"
      "  code platomain\n"
      "  number_processors 1\n"
      "end service\n"
      "begin service 2\n"
      "  code plato_analyze\n"
      "  number_processors 5\n"
      "end service\n"
      "begin criterion 1\n"
      "  type mechanical_compliance\n"
      "end criterion\n"
      "begin scenario 1\n"
      "  physics steady_state_mechanics\n"
      "  dimensions 3\n"
      "  loads 10 1\n"
      "  boundary_conditions 1 2 3\n"
      "  material 1\n"
      "end scenario\n"
      "begin objective\n"
      "  scenarios 1\n"
      "  criteria 1\n"
      "  services 2\n"
      "  type weighted_sum\n"
      "  weights 1\n"
      "end objective\n"
      "begin boundary_condition 1\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispx\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 2\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispy\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 3\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispz\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin load 10\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_2\n"
      "  value 0 -5e4 0\n"
      "end load\n"
      "begin load 1\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_3\n"
      "  value 0 -5e4 0\n"
      "end load\n"
      "begin block 1\n"
      "  material 1\n"
      "end block\n"
      "begin material 1\n"
      "  material_model isotropic_linear_elastic\n"
      "  poissons_ratio 0.33\n"
      "  youngs_modulus 1e6\n"
      "end material\n"
      "begin uncertainty\n"
      "  category load\n"
      "  tag angle_variation\n"
      "  load_id 10\n"
      "  attribute X\n"
      "  distribution beta\n"
      "  mean 0.0\n"
      "  upper_bound 45.0\n"
      "  lower_bound -45.0\n"
      "  standard_deviation 22.5\n"
      "  number_samples 10\n"
      "  initial_guess uniform\n"
      "end uncertainty\n"
      "begin optimization_parameters\n"
      "end optimization_parameters\n";
  // do parse
    tIss.str(tStringInput);
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tIss));
    tIss.clear();
    tIss.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseUncertainties(tIss));
    tIss.clear();
    tIss.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tIss));

    auto tXMLGenMetadata = tTester.getInputData();
    EXPECT_EQ(tTester.publicRunSROMForUncertainVariables(tXMLGenMetadata), true);

    // TEST DATA
    auto tNumSamples = tXMLGenMetadata.mRandomMetaData.numSamples();
    EXPECT_EQ(tNumSamples,10u);
    size_t tNumPerformers = tXMLGenMetadata.m_UncertaintyMetaData.numPerformers;
    EXPECT_EQ(tNumPerformers,5u);

    // POSE GOLD VALUES
    std::vector<std::string> tGoldLoadCaseProbabilities =
        {"0.094129618760905517", "0.096090610891147593", "0.099662653884575278", "0.10449800813241415" , "0.10567878404136007",
         "0.10579431139496034" , "0.10488572142307051" , "0.099762429835208791", "0.095650141042446082", "0.093863777827277631"};

    std::vector<std::vector<std::vector<std::string>>> tGoldLoadValues =
      {
        { { "0.000000000000000000000e+00", "-40080.529655427614", "29892.325813499148" },
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-44109.29605382467", "23545.063211553443" },
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-47042.380137430053", "16941.501438937652" },
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-48948.627509715574", "10199.601213533577" },
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-49884.894530613543", "3390.7665312673744" },
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-49882.875644480926", "-3420.338789835318"},
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-48947.942158115213", "-10202.889712518099"},
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-47060.949467672071", "-16889.850064497783"},
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-44182.941372529094", "-23406.57368500689"},
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } },

        { { "0.000000000000000000000e+00", "-40257.154550975094", "-29654.030206009185"},
          { "0.000000000000000000000e+00", "-5.000000000000000000000e+04", "0.000000000000000000000e+00" } }
      };

    // TEST SAMPLES
    constexpr double tTolerance = 1e-6;
    auto tSamples = tXMLGenMetadata.mRandomMetaData.samples();
    for (auto &tSample : tSamples)
    {
        auto tSampleIndex = &tSample - &tSamples[0];
        ASSERT_NEAR(std::stod(tGoldLoadCaseProbabilities[tSampleIndex]), std::stod(tSample.probability()), tTolerance);

        auto tLoadCase = tSample.loadcase();
        for (auto &tLoad : tLoadCase.loads)
        {
            auto tLoadIndex = &tLoad - &tLoadCase.loads[0];
            ASSERT_STREQ("traction", tLoad.type().c_str());
            for (auto &tValue : tLoad.load_values())
            {
                auto tComponent = &tValue - &tLoad.load_values()[0];
                ASSERT_NEAR(std::stod(tValue), std::stod(tGoldLoadValues[tSampleIndex][tLoadIndex][tComponent]), tTolerance);
            }
        }
    }

    Plato::system("rm -f plato_cdf_output.txt");
    Plato::system("rm -f plato_srom_diagnostics.txt");
    Plato::system("rm -f plato_ksal_algorithm_diagnostics.txt");
}

TEST(PlatoTestXMLGenerator, IncompressibleFluidsWorkFlow)
{
    // POSE INPUT DATA
    std::istringstream tIss;
    std::string tStringInput =
        "begin service 1\n"
        "  code platomain\n"
        "  number_processors 1\n"
        "end service\n"
        "begin service 2\n"
        "  code plato_analyze\n"
        "  number_processors 1\n"
        "end service\n"
        "begin criterion 1\n"
        "  type composite\n"
        "  criterion_ids 2 3\n"
        "  criterion_weights 1.0 -1.0\n"
        "end criterion\n"
        "begin criterion 2\n"
        "  type surface_pressure\n"
        "  location_name inlet\n"
        "end criterion\n"
        "begin criterion 3\n"
        "  type surface_pressure\n"
        "  location_name outlet\n"
        "end criterion\n"
        "begin criterion 4\n"
        "   type volume\n"
        "end criterion\n"
        "begin scenario 1\n"
        "  physics steady_state_incompressible_fluids\n"
        "  dimensions 2\n"
        "  loads 10 1\n"
        "  boundary_conditions 1 2 3 4 5\n"
        "  material 1\n"
        "end scenario\n"
        "begin objective\n"
        "  scenarios 1\n"
        "  criteria 1\n"
        "  services 2\n"
        "  type weighted_sum\n"
        "  weights 1\n"
        "end objective\n"
        "begin constraint 1\n"
        "  criterion 4\n"
        "  relative_target 0.25\n"
        "  type less_than\n"
        "  service 1\n"
        "  scenario 1\n"
        "end constraint\n"
        "begin boundary_condition 1\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name no_slip\n"
        "  degree_of_freedom velx\n"
        "end boundary_condition\n"
        "begin boundary_condition 2\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name no_slip\n"
        "  degree_of_freedom vely\n"
        "end boundary_condition\n"
        "begin boundary_condition 3\n"
        "  type fixed_value\n"
        "  location_type nodeset\n"
        "  location_name inlet\n"
        "  degree_of_freedom velx\n"
        "  value 1.5\n"
        "end boundary_condition\n"
        "begin boundary_condition 4\n"
        "  type fixed_value\n"
        "  location_type nodeset\n"
        "  location_name inlet\n"
        "  degree_of_freedom vely\n"
        "  value 0\n"
        "end boundary_condition\n"
        "begin boundary_condition 5\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name outlet\n"
        "  degree_of_freedom press\n"
        "end boundary_condition\n"
        "begin block 1\n"
        "  material 1\n"
        "end block\n"
        "begin material 1\n"
        "  material_model incompressible_flow\n"
        "  reynolds_number 100\n"
        "  impermeability_number 100\n"
        "end material\n"
        "begin optimization_parameters\n"
        "  optimization_algorithm mma\n"
        "  discretization density\n"
        "  max_iterations 50\n"
        "  filter_radius_scale 1.75\n"
        "end optimization_parameters\n"
        "begin mesh\n"
        "  name bolted_bracket.exo\n"
        "end mesh\n"
        "begin paths\n"
        "  code PlatoMain PlatoMain\n"
        "  code plato_analyze analyze_MPMD\n"
        "end paths\n";

    // do parse
    XMLGenerator_UnitTester tTester;
    tIss.str(tStringInput);
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseConstraints(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tIss));
    tIss.clear();
    tIss.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMaterials(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseOptimizationParameters(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCodePaths(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMesh(tIss));

    // CALL GENERATE
    auto tXMLGenMetadata = tTester.getInputData();
    tTester.generate(tXMLGenMetadata);

    auto tReadData = XMLGen::read_data_from_file("plato_analyze_2_input_deck.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><ParameterListname=\"Problem\"><Parametername=\"Physics\"type=\"string\"value=\"PlatoDriver\"/><Parametername=\"SpatialDimension\"type=\"int\"value=\"2\"/><Parametername=\"InputMesh\"type=\"string\"value=\"bolted_bracket.exo\"/><ParameterListname=\"PlatoProblem\"><Parametername=\"Physics\"type=\"string\"value=\"IncompressibleFluids\"/><Parametername=\"PDEConstraint\"type=\"string\"value=\"Hyperbolic\"/><Parametername=\"Self-Adjoint\"type=\"bool\"value=\"false\"/><ParameterListname=\"Criteria\"><ParameterListname=\"MyObjective\"><Parametername=\"Type\"type=\"string\"value=\"WeightedSum\"/><Parametername=\"Functions\"type=\"Array(string)\"value=\"{my_surface_pressure_criterion_id_2,my_surface_pressure_criterion_id_3}\"/><Parametername=\"Weights\"type=\"Array(double)\"value=\"{1.0,-1.0}\"/></ParameterList><ParameterListname=\"my_surface_pressure_criterion_id_2\"><Parametername=\"Type\"type=\"string\"value=\"ScalarFunction\"/><Parametername=\"ScalarFunctionType\"type=\"string\"value=\"AverageSurfacePressure\"/><Parametername=\"Sides\"type=\"Array(string)\"value=\"{inlet}\"/></ParameterList><ParameterListname=\"my_surface_pressure_criterion_id_3\"><Parametername=\"Type\"type=\"string\"value=\"ScalarFunction\"/><Parametername=\"ScalarFunctionType\"type=\"string\"value=\"AverageSurfacePressure\"/><Parametername=\"Sides\"type=\"Array(string)\"value=\"{outlet}\"/></ParameterList></ParameterList><ParameterListname=\"Hyperbolic\"><Parametername=\"Scenario\"type=\"string\"value=\"Density-BasedTopologyOptimization\"/><Parametername=\"HeatTransfer\"type=\"string\"value=\"none\"/></ParameterList><ParameterListname=\"TimeIntegration\"><Parametername=\"SafetyFactor\"type=\"double\"value=\"0.7\"/></ParameterList><ParameterListname=\"Convergence\"><Parametername=\"OutputFrequency\"type=\"int\"value=\"1\"/><Parametername=\"MaximumSteadyStateIterations\"type=\"int\"value=\"500\"/><Parametername=\"SteadyStateTolerance\"type=\"double\"value=\"1e-4\"/></ParameterList><ParameterListname=\"LinearSolver\"><Parametername=\"Iterations\"type=\"int\"value=\"1000\"/><Parametername=\"Tolerance\"type=\"double\"value=\"1e-12\"/><Parametername=\"SolverStack\"type=\"string\"value=\"Amgx\"/></ParameterList><ParameterListname=\"SpatialModel\"><ParameterListname=\"Domains\"><ParameterListname=\"Block1\"><Parametername=\"ElementBlock\"type=\"string\"value=\"block_1\"/><Parametername=\"MaterialModel\"type=\"string\"value=\"material_1\"/></ParameterList></ParameterList></ParameterList><ParameterListname=\"MaterialModels\"><ParameterListname=\"material_1\"><Parametername=\"ReynoldsNumber\"type=\"double\"value=\"100\"/><Parametername=\"ImpermeabilityNumber\"type=\"double\"value=\"100\"/></ParameterList></ParameterList><ParameterListname=\"MomentumNaturalBoundaryConditions\"/><ParameterListname=\"ThermalNaturalBoundaryConditions\"/><ParameterListname=\"PressureEssentialBoundaryConditions\"><ParameterListname=\"PressureBoundaryConditionwithID5appliedtoDofwithtagPRESS\"><Parametername=\"Type\"type=\"string\"value=\"ZeroValue\"/><Parametername=\"Index\"type=\"int\"value=\"0\"/><Parametername=\"Sides\"type=\"string\"value=\"outlet\"/></ParameterList></ParameterList><ParameterListname=\"VelocityEssentialBoundaryConditions\"><ParameterListname=\"VelocityBoundaryConditionwithID1appliedtoDofwithtagVELX\"><Parametername=\"Type\"type=\"string\"value=\"ZeroValue\"/><Parametername=\"Index\"type=\"int\"value=\"0\"/><Parametername=\"Sides\"type=\"string\"value=\"no_slip\"/></ParameterList><ParameterListname=\"VelocityBoundaryConditionwithID2appliedtoDofwithtagVELY\"><Parametername=\"Type\"type=\"string\"value=\"ZeroValue\"/><Parametername=\"Index\"type=\"int\"value=\"1\"/><Parametername=\"Sides\"type=\"string\"value=\"no_slip\"/></ParameterList><ParameterListname=\"VelocityBoundaryConditionwithID3appliedtoDofwithtagVELX\"><Parametername=\"Type\"type=\"string\"value=\"FixedValue\"/><Parametername=\"Index\"type=\"int\"value=\"0\"/><Parametername=\"Sides\"type=\"string\"value=\"inlet\"/><Parametername=\"Value\"type=\"double\"value=\"1.5\"/></ParameterList><ParameterListname=\"VelocityBoundaryConditionwithID4appliedtoDofwithtagVELY\"><Parametername=\"Type\"type=\"string\"value=\"FixedValue\"/><Parametername=\"Index\"type=\"int\"value=\"1\"/><Parametername=\"Sides\"type=\"string\"value=\"inlet\"/><Parametername=\"Value\"type=\"double\"value=\"0\"/></ParameterList></ParameterList></ParameterList></ParameterList>");
    EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());

    auto tTrash = std::system("rm -rf *.xml amgx.json mpirun.source");
    Plato::Utils::ignore_unused(tTrash);
}

TEST(PlatoTestXMLGenerator, appendMMAOptions_topology)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    // material_box
    stringInput = "begin optimization_parameters\n"
            "optimization_algorithm mma\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_NO_THROW(tester.publicParseOptimizationParameters(iss));

    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_move_limit(), "0.5");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_asymptote_expansion(), "1.2");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_asymptote_contraction(), "0.7");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_max_sub_problem_iterations(), "50");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_control_stagnation_tolerance(), "1e-6");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_objective_stagnation_tolerance(), "1e-8");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_sub_problem_feasibility_tolerance(), "1e-8");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_output_subproblem_diagnostics(), "false");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_sub_problem_penalty_multiplier(), "1.025");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_sub_problem_initial_penalty(), "0.0015");

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_method_moving_asymptotes_options(*(tester.exposeInputData()), tDocument));
    auto tOptionsNode = tDocument.child("Options");
    std::vector<std::string> tKeys = {"MoveLimit", "AsymptoteExpansion", "AsymptoteContraction", 
                   "MaxNumSubProblemIter", "ControlStagnationTolerance", "ObjectiveStagnationTolerance",
                   "OutputSubProblemDiagnostics", "SubProblemInitialPenalty", "SubProblemPenaltyMultiplier", 
                   "SubProblemFeasibilityTolerance"};
    std::vector<std::string> tValues = {"0.5", "1.2", "0.7", "50", "1e-6", "1e-8", "false", "0.0015",
                                        "1.025", "1e-8"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOptionsNode);
}

TEST(PlatoTestXMLGenerator, appendMMAOptions_shape)
{
    XMLGenerator_UnitTester tester;
    std::istringstream iss;
    std::string stringInput;

    // material_box
    stringInput = "begin optimization_parameters\n"
            "optimization_algorithm mma\n"
            "optimization_type shape\n"
            "end optimization_parameters\n";
    iss.str(stringInput);
    iss.clear();
    iss.seekg (0);
    tester.clearInputData();
    ASSERT_NO_THROW(tester.publicParseOptimizationParameters(iss));

    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_move_limit(), "0.5");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_asymptote_expansion(), "1.2");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_asymptote_contraction(), "0.7");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_max_sub_problem_iterations(), "50");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_control_stagnation_tolerance(), "-1");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_objective_stagnation_tolerance(), "-1");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_sub_problem_feasibility_tolerance(), "1e-8");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_output_subproblem_diagnostics(), "false");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_sub_problem_penalty_multiplier(), "1.025");
    EXPECT_EQ(tester.exposeInputData()->optimization_parameters().mma_sub_problem_initial_penalty(), "0.0015");

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_method_moving_asymptotes_options(*(tester.exposeInputData()), tDocument));
    auto tOptionsNode = tDocument.child("Options");
    std::vector<std::string> tKeys = {"MoveLimit", "AsymptoteExpansion", "AsymptoteContraction", 
                   "MaxNumSubProblemIter", "ControlStagnationTolerance", "ObjectiveStagnationTolerance",
                   "OutputSubProblemDiagnostics", "SubProblemInitialPenalty", "SubProblemPenaltyMultiplier", 
                   "SubProblemFeasibilityTolerance"};
    std::vector<std::string> tValues = {"0.5", "1.2", "0.7", "50", "-1", "-1", "false", "0.0015",
                                        "1.025", "1e-8"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOptionsNode);
}

TEST(PlatoTestXMLGenerator, ShapeOptimization_num_shape_design_varibles_good_1)
{
    // POSE INPUT DATA
    std::istringstream tIss;
    std::string tStringInput =
        "begin service 1\n"
        "  code platomain\n"
        "  number_processors 1\n"
        "end service\n"
        "begin service 2\n"
        "  code plato_esp\n"
        "  number_processors 4\n"
        "end service\n"
        "begin service 3\n"
        "  code plato_analyze\n"
        "  number_processors 1\n"
        "end service\n"
        "begin criterion 1\n"
        "  type composite\n"
        "  criterion_ids 2 3\n"
        "  criterion_weights 1.0 -1.0\n"
        "end criterion\n"
        "begin criterion 2\n"
        "  type surface_pressure\n"
        "  location_name inlet\n"
        "end criterion\n"
        "begin criterion 3\n"
        "  type surface_pressure\n"
        "  location_name outlet\n"
        "end criterion\n"
        "begin criterion 4\n"
        "   type volume\n"
        "end criterion\n"
        "begin scenario 1\n"
        "  physics steady_state_incompressible_fluids\n"
        "  dimensions 2\n"
        "  loads 10 1\n"
        "  boundary_conditions 1 2 3 4 5\n"
        "  material 1\n"
        "end scenario\n"
        "begin objective\n"
        "  scenarios 1\n"
        "  criteria 1\n"
        "  services 3\n"
        "  shape_services 2\n"
        "  type weighted_sum\n"
        "  weights 1\n"
        "end objective\n"
        "begin constraint 1\n"
        "  criterion 4\n"
        "  relative_target 0.25\n"
        "  type less_than\n"
        "  service 1\n"
        "  scenario 1\n"
        "end constraint\n"
        "begin boundary_condition 1\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name no_slip\n"
        "  degree_of_freedom velx\n"
        "end boundary_condition\n"
        "begin boundary_condition 2\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name no_slip\n"
        "  degree_of_freedom vely\n"
        "end boundary_condition\n"
        "begin boundary_condition 3\n"
        "  type fixed_value\n"
        "  location_type nodeset\n"
        "  location_name inlet\n"
        "  degree_of_freedom velx\n"
        "  value 1.5\n"
        "end boundary_condition\n"
        "begin boundary_condition 4\n"
        "  type fixed_value\n"
        "  location_type nodeset\n"
        "  location_name inlet\n"
        "  degree_of_freedom vely\n"
        "  value 0\n"
        "end boundary_condition\n"
        "begin boundary_condition 5\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name outlet\n"
        "  degree_of_freedom press\n"
        "end boundary_condition\n"
        "begin block 1\n"
        "  material 1\n"
        "end block\n"
        "begin material 1\n"
        "  material_model incompressible_flow\n"
        "  reynolds_number 100\n"
        "  impermeability_number 100\n"
        "end material\n"
        "begin optimization_parameters\n"
        "  optimization_algorithm mma\n"
        "  discretization density\n"
        "  max_iterations 50\n"
        "  filter_radius_scale 1.75\n"
        "  optimization_type shape\n"
        "  num_shape_design_variables 4\n"
        "end optimization_parameters\n"
        "begin mesh\n"
        "  name bolted_bracket.exo\n"
        "end mesh\n"
        "begin paths\n"
        "  code PlatoMain PlatoMain\n"
        "  code plato_analyze analyze_MPMD\n"
        "end paths\n";

    // do parse
    XMLGenerator_UnitTester tTester;
    tIss.str(tStringInput);
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseConstraints(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tIss));
    tIss.clear();
    tIss.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMaterials(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseOptimizationParameters(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCodePaths(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMesh(tIss));

    // CALL GENERATE
    auto tXMLGenMetadata = tTester.getInputData();
    ASSERT_NO_THROW(tTester.generate(tXMLGenMetadata));
}

TEST(PlatoTestXMLGenerator, ShapeOptimization_num_shape_design_varibles_good_2)
{
    // POSE INPUT DATA
    std::istringstream tIss;
    std::string tStringInput =
        "begin service 1\n"
        "  code platomain\n"
        "  number_processors 1\n"
        "end service\n"
        "begin service 2\n"
        "  code plato_esp\n"
        "  number_processors 1\n"
        "end service\n"
        "begin service 3\n"
        "  code plato_analyze\n"
        "  number_processors 1\n"
        "end service\n"
        "begin criterion 1\n"
        "  type composite\n"
        "  criterion_ids 2 3\n"
        "  criterion_weights 1.0 -1.0\n"
        "end criterion\n"
        "begin criterion 2\n"
        "  type surface_pressure\n"
        "  location_name inlet\n"
        "end criterion\n"
        "begin criterion 3\n"
        "  type surface_pressure\n"
        "  location_name outlet\n"
        "end criterion\n"
        "begin criterion 4\n"
        "   type volume\n"
        "end criterion\n"
        "begin scenario 1\n"
        "  physics steady_state_incompressible_fluids\n"
        "  dimensions 2\n"
        "  loads 10 1\n"
        "  boundary_conditions 1 2 3 4 5\n"
        "  material 1\n"
        "end scenario\n"
        "begin objective\n"
        "  scenarios 1\n"
        "  criteria 1\n"
        "  services 3\n"
        "  shape_services 2\n"
        "  type weighted_sum\n"
        "  weights 1\n"
        "end objective\n"
        "begin constraint 1\n"
        "  criterion 4\n"
        "  relative_target 0.25\n"
        "  type less_than\n"
        "  service 1\n"
        "  scenario 1\n"
        "end constraint\n"
        "begin boundary_condition 1\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name no_slip\n"
        "  degree_of_freedom velx\n"
        "end boundary_condition\n"
        "begin boundary_condition 2\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name no_slip\n"
        "  degree_of_freedom vely\n"
        "end boundary_condition\n"
        "begin boundary_condition 3\n"
        "  type fixed_value\n"
        "  location_type nodeset\n"
        "  location_name inlet\n"
        "  degree_of_freedom velx\n"
        "  value 1.5\n"
        "end boundary_condition\n"
        "begin boundary_condition 4\n"
        "  type fixed_value\n"
        "  location_type nodeset\n"
        "  location_name inlet\n"
        "  degree_of_freedom vely\n"
        "  value 0\n"
        "end boundary_condition\n"
        "begin boundary_condition 5\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name outlet\n"
        "  degree_of_freedom press\n"
        "end boundary_condition\n"
        "begin block 1\n"
        "  material 1\n"
        "end block\n"
        "begin material 1\n"
        "  material_model incompressible_flow\n"
        "  reynolds_number 100\n"
        "  impermeability_number 100\n"
        "end material\n"
        "begin optimization_parameters\n"
        "  optimization_algorithm mma\n"
        "  discretization density\n"
        "  max_iterations 50\n"
        "  filter_radius_scale 1.75\n"
        "  optimization_type shape\n"
        "  num_shape_design_variables 4\n"
        "end optimization_parameters\n"
        "begin mesh\n"
        "  name bolted_bracket.exo\n"
        "end mesh\n"
        "begin paths\n"
        "  code PlatoMain PlatoMain\n"
        "  code plato_analyze analyze_MPMD\n"
        "end paths\n";

    // do parse
    XMLGenerator_UnitTester tTester;
    tIss.str(tStringInput);
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseConstraints(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tIss));
    tIss.clear();
    tIss.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMaterials(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseOptimizationParameters(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCodePaths(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMesh(tIss));

    // CALL GENERATE
    auto tXMLGenMetadata = tTester.getInputData();
    ASSERT_NO_THROW(tTester.generate(tXMLGenMetadata));
    auto tTrash = std::system("rm -rf *.xml amgx.json mpirun.source");
    Plato::Utils::ignore_unused(tTrash);
}

TEST(PlatoTestXMLGenerator, ShapeOptimization_num_shape_design_varibles_bad)
{
    // POSE INPUT DATA
    std::istringstream tIss;
    std::string tStringInput =
        "begin service 1\n"
        "  code platomain\n"
        "  number_processors 1\n"
        "end service\n"
        "begin service 2\n"
        "  code plato_esp\n"
        "  number_processors 10\n"
        "end service\n"
        "begin service 3\n"
        "  code plato_analyze\n"
        "  number_processors 1\n"
        "end service\n"
        "begin criterion 1\n"
        "  type composite\n"
        "  criterion_ids 2 3\n"
        "  criterion_weights 1.0 -1.0\n"
        "end criterion\n"
        "begin criterion 2\n"
        "  type surface_pressure\n"
        "  location_name inlet\n"
        "end criterion\n"
        "begin criterion 3\n"
        "  type surface_pressure\n"
        "  location_name outlet\n"
        "end criterion\n"
        "begin criterion 4\n"
        "   type volume\n"
        "end criterion\n"
        "begin scenario 1\n"
        "  physics steady_state_incompressible_fluids\n"
        "  dimensions 2\n"
        "  loads 10 1\n"
        "  boundary_conditions 1 2 3 4 5\n"
        "  material 1\n"
        "end scenario\n"
        "begin objective\n"
        "  scenarios 1\n"
        "  criteria 1\n"
        "  services 3\n"
        "  shape_services 2\n"
        "  type weighted_sum\n"
        "  weights 1\n"
        "end objective\n"
        "begin constraint 1\n"
        "  criterion 4\n"
        "  relative_target 0.25\n"
        "  type less_than\n"
        "  service 1\n"
        "  scenario 1\n"
        "end constraint\n"
        "begin boundary_condition 1\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name no_slip\n"
        "  degree_of_freedom velx\n"
        "end boundary_condition\n"
        "begin boundary_condition 2\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name no_slip\n"
        "  degree_of_freedom vely\n"
        "end boundary_condition\n"
        "begin boundary_condition 3\n"
        "  type fixed_value\n"
        "  location_type nodeset\n"
        "  location_name inlet\n"
        "  degree_of_freedom velx\n"
        "  value 1.5\n"
        "end boundary_condition\n"
        "begin boundary_condition 4\n"
        "  type fixed_value\n"
        "  location_type nodeset\n"
        "  location_name inlet\n"
        "  degree_of_freedom vely\n"
        "  value 0\n"
        "end boundary_condition\n"
        "begin boundary_condition 5\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name outlet\n"
        "  degree_of_freedom press\n"
        "end boundary_condition\n"
        "begin block 1\n"
        "  material 1\n"
        "end block\n"
        "begin material 1\n"
        "  material_model incompressible_flow\n"
        "  reynolds_number 100\n"
        "  impermeability_number 100\n"
        "end material\n"
        "begin optimization_parameters\n"
        "  optimization_algorithm mma\n"
        "  discretization density\n"
        "  max_iterations 50\n"
        "  filter_radius_scale 1.75\n"
        "  optimization_type shape\n"
        "  num_shape_design_variables 4\n"
        "end optimization_parameters\n"
        "begin mesh\n"
        "  name bolted_bracket.exo\n"
        "end mesh\n"
        "begin paths\n"
        "  code PlatoMain PlatoMain\n"
        "  code plato_analyze analyze_MPMD\n"
        "end paths\n";

    // do parse
    XMLGenerator_UnitTester tTester;
    tIss.str(tStringInput);
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseConstraints(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tIss));
    tIss.clear();
    tIss.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMaterials(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseOptimizationParameters(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCodePaths(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMesh(tIss));

    // CALL GENERATE
    auto tXMLGenMetadata = tTester.getInputData();
    ASSERT_THROW(tTester.generate(tXMLGenMetadata), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, NaturalConvectionWorkFlow)
{
    // POSE INPUT DATA
    std::istringstream tIss;
    std::string tStringInput =
        "begin service 1\n"
        "  code platomain\n"
        "  number_processors 1\n"
        "end service\n"
        "begin service 2\n"
        "  code plato_analyze\n"
        "  number_processors 1\n"
        "end service\n"
        "begin criterion 1\n"
        "  type composite\n"
        "  criterion_ids 2 3\n"
        "  criterion_weights 1.0 -1.0\n"
        "end criterion\n"
        "begin criterion 2\n"
        "  type surface_temperature\n"
        "  location_name inlet\n"
        "end criterion\n"
        "begin criterion 3\n"
        "  type surface_temperature\n"
        "  location_name outlet\n"
        "end criterion\n"
        "begin criterion 4\n"
        "   type volume\n"
        "end criterion\n"
        "begin scenario 1\n"
        "  physics steady_state_incompressible_fluids\n"
        "  heat_transfer natural\n"
        "  dimensions 2\n"
        "  boundary_conditions 1 2 3 4\n"
        "  material 1\n"
        "end scenario\n"
        "begin objective\n"
        "  scenarios 1\n"
        "  criteria 1\n"
        "  services 2\n"
        "  type weighted_sum\n"
        "  weights 1\n"
        "end objective\n"
        "begin constraint 1\n"
        "  criterion 4\n"
        "  relative_target 0.25\n"
        "  type less_than\n"
        "  service 1\n"
        "  scenario 1\n"
        "end constraint\n"
        "begin boundary_condition 1\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name no_slip\n"
        "  degree_of_freedom velx\n"
        "end boundary_condition\n"
        "begin boundary_condition 2\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name no_slip\n"
        "  degree_of_freedom vely\n"
        "end boundary_condition\n"
        "begin boundary_condition 3\n"
        "  type fixed_value\n"
        "  location_type nodeset\n"
        "  location_name cold\n"
        "  degree_of_freedom temp\n"
        "  value 0\n"
        "end boundary_condition\n"
        "begin boundary_condition 4\n"
        "  type fixed_value\n"
        "  location_type nodeset\n"
        "  location_name hot\n"
        "  degree_of_freedom temp\n"
        "  value 1\n"
        "end boundary_condition\n"
        "begin block 1\n"
        "  material 1\n"
        "end block\n"
        "begin material 1\n"
        "  material_model natural_buoyancy\n"
        "  prandtl_number 0.7\n"
        "  thermal_diffusivity_ratio 0.5\n"
        "  rayleigh_number 0 1e3\n"
        "end material\n"
        "begin optimization_parameters\n"
        "  optimization_algorithm mma\n"
        "  discretization density\n"
        "  max_iterations 50\n"
        "  filter_radius_scale 1.75\n"
        "end optimization_parameters\n"
        "begin mesh\n"
        "  name bolted_bracket.exo\n"
        "end mesh\n"
        "begin paths\n"
        "  code PlatoMain PlatoMain\n"
        "  code plato_analyze analyze_MPMD\n"
        "end paths\n";

    // do parse
    XMLGenerator_UnitTester tTester;
    tIss.str(tStringInput);
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseConstraints(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tIss));
    tIss.clear();
    tIss.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tIss));
    tIss.clear();
    tIss.seekg(0);
    tTester.publicParseScenarios(tIss);
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMaterials(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseOptimizationParameters(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCodePaths(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMesh(tIss));

    // CALL GENERATE
    auto tXMLGenMetadata = tTester.getInputData();
    ASSERT_NO_THROW(tTester.generate(tXMLGenMetadata));
    
    auto tReadData = XMLGen::read_data_from_file("plato_analyze_2_input_deck.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><ParameterListname=\"Problem\"><Parametername=\"Physics\"type=\"string\"value=\"PlatoDriver\"/><Parametername=\"SpatialDimension\"type=\"int\"value=\"2\"/><Parametername=\"InputMesh\"type=\"string\"value=\"bolted_bracket.exo\"/><ParameterListname=\"PlatoProblem\"><Parametername=\"Physics\"type=\"string\"value=\"IncompressibleFluids\"/><Parametername=\"PDEConstraint\"type=\"string\"value=\"Hyperbolic\"/><Parametername=\"Self-Adjoint\"type=\"bool\"value=\"false\"/><ParameterListname=\"Criteria\"><ParameterListname=\"MyObjective\"><Parametername=\"Type\"type=\"string\"value=\"WeightedSum\"/><Parametername=\"Functions\"type=\"Array(string)\"value=\"{my_surface_temperature_criterion_id_2,my_surface_temperature_criterion_id_3}\"/><Parametername=\"Weights\"type=\"Array(double)\"value=\"{1.0,-1.0}\"/></ParameterList><ParameterListname=\"my_surface_temperature_criterion_id_2\"><Parametername=\"Type\"type=\"string\"value=\"ScalarFunction\"/><Parametername=\"ScalarFunctionType\"type=\"string\"value=\"AverageSurfaceTemperature\"/><Parametername=\"Sides\"type=\"Array(string)\"value=\"{inlet}\"/></ParameterList><ParameterListname=\"my_surface_temperature_criterion_id_3\"><Parametername=\"Type\"type=\"string\"value=\"ScalarFunction\"/><Parametername=\"ScalarFunctionType\"type=\"string\"value=\"AverageSurfaceTemperature\"/><Parametername=\"Sides\"type=\"Array(string)\"value=\"{outlet}\"/></ParameterList></ParameterList><ParameterListname=\"Hyperbolic\"><Parametername=\"Scenario\"type=\"string\"value=\"Density-BasedTopologyOptimization\"/><Parametername=\"HeatTransfer\"type=\"string\"value=\"natural\"/><ParameterListname=\"EnergyConservation\"><ParameterListname=\"PenaltyFunction\"><Parametername=\"HeatSourcePenaltyExponent\"type=\"double\"value=\"3\"/><Parametername=\"ThermalDiffusionPenaltyExponent\"type=\"double\"value=\"3\"/></ParameterList></ParameterList></ParameterList><ParameterListname=\"TimeIntegration\"><Parametername=\"SafetyFactor\"type=\"double\"value=\"0.7\"/></ParameterList><ParameterListname=\"Convergence\"><Parametername=\"OutputFrequency\"type=\"int\"value=\"1\"/><Parametername=\"MaximumSteadyStateIterations\"type=\"int\"value=\"500\"/><Parametername=\"SteadyStateTolerance\"type=\"double\"value=\"1e-4\"/></ParameterList><ParameterListname=\"LinearSolver\"><Parametername=\"Iterations\"type=\"int\"value=\"1000\"/><Parametername=\"Tolerance\"type=\"double\"value=\"1e-12\"/><Parametername=\"SolverStack\"type=\"string\"value=\"Amgx\"/></ParameterList><ParameterListname=\"SpatialModel\"><ParameterListname=\"Domains\"><ParameterListname=\"Block1\"><Parametername=\"ElementBlock\"type=\"string\"value=\"block_1\"/><Parametername=\"MaterialModel\"type=\"string\"value=\"material_1\"/></ParameterList></ParameterList></ParameterList><ParameterListname=\"MaterialModels\"><ParameterListname=\"material_1\"><Parametername=\"ImpermeabilityNumber\"type=\"double\"value=\"100\"/><Parametername=\"PrandtlNumber\"type=\"double\"value=\"0.7\"/><Parametername=\"ThermalDiffusivity\"type=\"double\"value=\"2.1117e-5\"/><Parametername=\"KinematicViscocity\"type=\"double\"value=\"1.5111e-5\"/><Parametername=\"ThermalDiffusivityRatio\"type=\"double\"value=\"0.5\"/><Parametername=\"RayleighNumber\"type=\"Array(double)\"value=\"{01e3}\"/></ParameterList></ParameterList><ParameterListname=\"MomentumNaturalBoundaryConditions\"/><ParameterListname=\"ThermalNaturalBoundaryConditions\"/><ParameterListname=\"TemperatureEssentialBoundaryConditions\"><ParameterListname=\"TemperatureBoundaryConditionwithID3appliedtoDofwithtagTEMP\"><Parametername=\"Type\"type=\"string\"value=\"FixedValue\"/><Parametername=\"Index\"type=\"int\"value=\"0\"/><Parametername=\"Sides\"type=\"string\"value=\"cold\"/><Parametername=\"Value\"type=\"double\"value=\"0\"/></ParameterList><ParameterListname=\"TemperatureBoundaryConditionwithID4appliedtoDofwithtagTEMP\"><Parametername=\"Type\"type=\"string\"value=\"FixedValue\"/><Parametername=\"Index\"type=\"int\"value=\"0\"/><Parametername=\"Sides\"type=\"string\"value=\"hot\"/><Parametername=\"Value\"type=\"double\"value=\"1\"/></ParameterList></ParameterList><ParameterListname=\"VelocityEssentialBoundaryConditions\"><ParameterListname=\"VelocityBoundaryConditionwithID1appliedtoDofwithtagVELX\"><Parametername=\"Type\"type=\"string\"value=\"ZeroValue\"/><Parametername=\"Index\"type=\"int\"value=\"0\"/><Parametername=\"Sides\"type=\"string\"value=\"no_slip\"/></ParameterList><ParameterListname=\"VelocityBoundaryConditionwithID2appliedtoDofwithtagVELY\"><Parametername=\"Type\"type=\"string\"value=\"ZeroValue\"/><Parametername=\"Index\"type=\"int\"value=\"1\"/><Parametername=\"Sides\"type=\"string\"value=\"no_slip\"/></ParameterList></ParameterList></ParameterList></ParameterList>");
    EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());

    auto tTrash = std::system("rm -rf *.xml amgx.json mpirun.source");
    Plato::Utils::ignore_unused(tTrash);
}

TEST(PlatoTestXMLGenerator, NaturalConvectionWorkFlow_WithThermalFlux)
{
    // POSE INPUT DATA
    std::istringstream tIss;
    std::string tStringInput =
        "begin service 1\n"
        "  code platomain\n"
        "  number_processors 1\n"
        "end service\n"
        "begin service 2\n"
        "  code plato_analyze\n"
        "  number_processors 1\n"
        "end service\n"
        "begin criterion 1\n"
        "  type composite\n"
        "  criterion_ids 2 3\n"
        "  criterion_weights 1.0 -1.0\n"
        "end criterion\n"
        "begin criterion 2\n"
        "  type surface_temperature\n"
        "  location_name inlet\n"
        "end criterion\n"
        "begin criterion 3\n"
        "  type surface_temperature\n"
        "  location_name outlet\n"
        "end criterion\n"
        "begin criterion 4\n"
        "   type volume\n"
        "end criterion\n"
        "begin scenario 1\n"
        "  physics steady_state_incompressible_fluids\n"
        "  heat_transfer natural\n"
        "  dimensions 2\n"
        "  loads 1\n"
        "  boundary_conditions 1 2 3 4\n"
        "  material 1\n"
        "end scenario\n"
        "begin objective\n"
        "  scenarios 1\n"
        "  criteria 1\n"
        "  services 2\n"
        "  type weighted_sum\n"
        "  weights 1\n"
        "end objective\n"
        "begin constraint 1\n"
        "  criterion 4\n"
        "  relative_target 0.25\n"
        "  type less_than\n"
        "  service 1\n"
        "  scenario 1\n"
        "end constraint\n"
        "begin load 1\n"
        "  type uniform_surface_flux\n"
        "  location_type sideset\n"
        "  location_name thermal_flux\n"
        "  value -1\n"
        "end load\n"
        "begin boundary_condition 1\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name no_slip\n"
        "  degree_of_freedom velx\n"
        "end boundary_condition\n"
        "begin boundary_condition 2\n"
        "  type zero_value\n"
        "  location_type nodeset\n"
        "  location_name no_slip\n"
        "  degree_of_freedom vely\n"
        "end boundary_condition\n"
        "begin boundary_condition 3\n"
        "  type fixed_value\n"
        "  location_type nodeset\n"
        "  location_name cold\n"
        "  degree_of_freedom temp\n"
        "  value 0\n"
        "end boundary_condition\n"
        "begin boundary_condition 4\n"
        "  type fixed_value\n"
        "  location_type nodeset\n"
        "  location_name hot\n"
        "  degree_of_freedom temp\n"
        "  value 1\n"
        "end boundary_condition\n"
        "begin block 1\n"
        "  material 1\n"
        "end block\n"
        "begin material 1\n"
        "  material_model natural_buoyancy\n"
        "  prandtl_number 0.7\n"
        "  thermal_diffusivity_ratio 0.5\n"
        "  rayleigh_number 0 1e3\n"
        "end material\n"
        "begin optimization_parameters\n"
        "  optimization_algorithm mma\n"
        "  discretization density\n"
        "  max_iterations 50\n"
        "  filter_radius_scale 1.75\n"
        "end optimization_parameters\n"
        "begin mesh\n"
        "  name bolted_bracket.exo\n"
        "end mesh\n"
        "begin paths\n"
        "  code PlatoMain PlatoMain\n"
        "  code plato_analyze analyze_MPMD\n"
        "end paths\n";

    // do parse
    XMLGenerator_UnitTester tTester;
    tIss.str(tStringInput);
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseConstraints(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tIss));
    tIss.clear();
    tIss.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tIss));
    tIss.clear();
    tIss.seekg(0);
    tTester.publicParseScenarios(tIss);
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMaterials(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseOptimizationParameters(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCodePaths(tIss));
    tIss.clear();
    tIss.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMesh(tIss));
    tIss.clear();
    tIss.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tIss));

    // CALL GENERATE
    auto tXMLGenMetadata = tTester.getInputData();
    ASSERT_NO_THROW(tTester.generate(tXMLGenMetadata));
    
    auto tReadData = XMLGen::read_data_from_file("plato_analyze_2_input_deck.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><ParameterListname=\"Problem\"><Parametername=\"Physics\"type=\"string\"value=\"PlatoDriver\"/><Parametername=\"SpatialDimension\"type=\"int\"value=\"2\"/><Parametername=\"InputMesh\"type=\"string\"value=\"bolted_bracket.exo\"/><ParameterListname=\"PlatoProblem\"><Parametername=\"Physics\"type=\"string\"value=\"IncompressibleFluids\"/><Parametername=\"PDEConstraint\"type=\"string\"value=\"Hyperbolic\"/><Parametername=\"Self-Adjoint\"type=\"bool\"value=\"false\"/><ParameterListname=\"Criteria\"><ParameterListname=\"MyObjective\"><Parametername=\"Type\"type=\"string\"value=\"WeightedSum\"/><Parametername=\"Functions\"type=\"Array(string)\"value=\"{my_surface_temperature_criterion_id_2,my_surface_temperature_criterion_id_3}\"/><Parametername=\"Weights\"type=\"Array(double)\"value=\"{1.0,-1.0}\"/></ParameterList><ParameterListname=\"my_surface_temperature_criterion_id_2\"><Parametername=\"Type\"type=\"string\"value=\"ScalarFunction\"/><Parametername=\"ScalarFunctionType\"type=\"string\"value=\"AverageSurfaceTemperature\"/><Parametername=\"Sides\"type=\"Array(string)\"value=\"{inlet}\"/></ParameterList><ParameterListname=\"my_surface_temperature_criterion_id_3\"><Parametername=\"Type\"type=\"string\"value=\"ScalarFunction\"/><Parametername=\"ScalarFunctionType\"type=\"string\"value=\"AverageSurfaceTemperature\"/><Parametername=\"Sides\"type=\"Array(string)\"value=\"{outlet}\"/></ParameterList></ParameterList><ParameterListname=\"Hyperbolic\"><Parametername=\"Scenario\"type=\"string\"value=\"Density-BasedTopologyOptimization\"/><Parametername=\"HeatTransfer\"type=\"string\"value=\"natural\"/><ParameterListname=\"EnergyConservation\"><ParameterListname=\"PenaltyFunction\"><Parametername=\"HeatSourcePenaltyExponent\"type=\"double\"value=\"3\"/><Parametername=\"ThermalDiffusionPenaltyExponent\"type=\"double\"value=\"3\"/></ParameterList></ParameterList></ParameterList><ParameterListname=\"TimeIntegration\"><Parametername=\"SafetyFactor\"type=\"double\"value=\"0.7\"/></ParameterList><ParameterListname=\"Convergence\"><Parametername=\"OutputFrequency\"type=\"int\"value=\"1\"/><Parametername=\"MaximumSteadyStateIterations\"type=\"int\"value=\"500\"/><Parametername=\"SteadyStateTolerance\"type=\"double\"value=\"1e-4\"/></ParameterList><ParameterListname=\"LinearSolver\"><Parametername=\"Iterations\"type=\"int\"value=\"1000\"/><Parametername=\"Tolerance\"type=\"double\"value=\"1e-12\"/><Parametername=\"SolverStack\"type=\"string\"value=\"Amgx\"/></ParameterList><ParameterListname=\"SpatialModel\"><ParameterListname=\"Domains\"><ParameterListname=\"Block1\"><Parametername=\"ElementBlock\"type=\"string\"value=\"block_1\"/><Parametername=\"MaterialModel\"type=\"string\"value=\"material_1\"/></ParameterList></ParameterList></ParameterList><ParameterListname=\"MaterialModels\"><ParameterListname=\"material_1\"><Parametername=\"ImpermeabilityNumber\"type=\"double\"value=\"100\"/><Parametername=\"PrandtlNumber\"type=\"double\"value=\"0.7\"/><Parametername=\"ThermalDiffusivity\"type=\"double\"value=\"2.1117e-5\"/><Parametername=\"KinematicViscocity\"type=\"double\"value=\"1.5111e-5\"/><Parametername=\"ThermalDiffusivityRatio\"type=\"double\"value=\"0.5\"/><Parametername=\"RayleighNumber\"type=\"Array(double)\"value=\"{01e3}\"/></ParameterList></ParameterList><ParameterListname=\"MomentumNaturalBoundaryConditions\"/><ParameterListname=\"ThermalNaturalBoundaryConditions\"><ParameterListname=\"UniformSurfaceFluxBoundaryConditionwithID1\"><Parametername=\"Type\"type=\"string\"value=\"Uniform\"/><Parametername=\"Value\"type=\"string\"value=\"-1\"/><Parametername=\"Sides\"type=\"string\"value=\"thermal_flux\"/></ParameterList></ParameterList><ParameterListname=\"TemperatureEssentialBoundaryConditions\"><ParameterListname=\"TemperatureBoundaryConditionwithID3appliedtoDofwithtagTEMP\"><Parametername=\"Type\"type=\"string\"value=\"FixedValue\"/><Parametername=\"Index\"type=\"int\"value=\"0\"/><Parametername=\"Sides\"type=\"string\"value=\"cold\"/><Parametername=\"Value\"type=\"double\"value=\"0\"/></ParameterList><ParameterListname=\"TemperatureBoundaryConditionwithID4appliedtoDofwithtagTEMP\"><Parametername=\"Type\"type=\"string\"value=\"FixedValue\"/><Parametername=\"Index\"type=\"int\"value=\"0\"/><Parametername=\"Sides\"type=\"string\"value=\"hot\"/><Parametername=\"Value\"type=\"double\"value=\"1\"/></ParameterList></ParameterList><ParameterListname=\"VelocityEssentialBoundaryConditions\"><ParameterListname=\"VelocityBoundaryConditionwithID1appliedtoDofwithtagVELX\"><Parametername=\"Type\"type=\"string\"value=\"ZeroValue\"/><Parametername=\"Index\"type=\"int\"value=\"0\"/><Parametername=\"Sides\"type=\"string\"value=\"no_slip\"/></ParameterList><ParameterListname=\"VelocityBoundaryConditionwithID2appliedtoDofwithtagVELY\"><Parametername=\"Type\"type=\"string\"value=\"ZeroValue\"/><Parametername=\"Index\"type=\"int\"value=\"1\"/><Parametername=\"Sides\"type=\"string\"value=\"no_slip\"/></ParameterList></ParameterList></ParameterList></ParameterList>");
    EXPECT_STREQ(tReadData.str().c_str(),tGold.c_str());

    auto tTrash = std::system("rm -rf *.xml amgx.json mpirun.source");
    Plato::Utils::ignore_unused(tTrash);
}

TEST(PlatoTestXMLGenerator, Clean_XML_Files)
{
    auto tTrash = std::system("rm -rf *.xml amgx.json mpirun.source");
    Plato::Utils::ignore_unused(tTrash);
}

} // end PlatoTestXMLGenerator namespace
