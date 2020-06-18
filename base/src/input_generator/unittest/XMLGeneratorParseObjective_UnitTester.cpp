/*
 * XMLGeneratorParseObjective_UnitTester.cpp
 *
 *  Created on: Jun 16, 2020
 */

#include <gtest/gtest.h>

#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorParseObjective.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorInvalidCriterion)
{
    std::string tStringInput =
        "begin objective\n"
        "   type hippo\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorDimMistmachNaturalBCArrays)
{
    std::string tStringInput =
        "begin objective\n"
        "   type stress p-norm\n"
        "   load ids 10\n"
        "   load case weights 1 2\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorInvalidCode)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code dog\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorInvalidOutputKey)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz bats\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorEmptyNaturalBCIDs)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorEmptyEssentialBCIDs)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   load ids 10\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_ErrorNumProcessorIsNotNumber)
{
    std::string tStringInput =
        "begin objective\n"
        "   type compliance\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "   distribute objective at most P3 processors\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    ASSERT_THROW(tObjectiveParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseObjective_OneObjective)
{
    std::string tStringInput =
        "begin objective\n"
        "   type maximize stiffness\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   code plato_analyze\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    tObjectiveParser.parse(tInputSS);
    auto tObjectiveMetadata = tObjectiveParser.data();
    ASSERT_EQ(1u, tObjectiveMetadata.size());

    ASSERT_STREQ("maximize stiffness", tObjectiveMetadata[0].type.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[0].num_procs.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[0].num_ranks.c_str());
    ASSERT_STREQ("1.0", tObjectiveMetadata[0].weight.c_str());
    ASSERT_STREQ("3.0", tObjectiveMetadata[0].mPenaltyParam.c_str());
    ASSERT_STREQ("6.0", tObjectiveMetadata[0].mPnormExponent.c_str());
    ASSERT_STREQ("1e-9", tObjectiveMetadata[0].mMinimumErsatzValue.c_str());
    ASSERT_STREQ("plato_analyze", tObjectiveMetadata[0].code_name.c_str());
    ASSERT_STREQ("plato_analyze_1", tObjectiveMetadata[0].mPerformerName.c_str());
    ASSERT_STREQ("true", tObjectiveMetadata[0].normalize_objective.c_str());
    ASSERT_STREQ("1e-7", tObjectiveMetadata[0].analysis_solver_tolerance.c_str());
    ASSERT_STREQ("false", tObjectiveMetadata[0].multi_load_case.c_str());
    ASSERT_STREQ("none", tObjectiveMetadata[0].distribute_objective_type.c_str());

    ASSERT_EQ(1u, tObjectiveMetadata[0].bc_ids.size());
    ASSERT_STREQ("11", tObjectiveMetadata[0].bc_ids[0].c_str());
    ASSERT_EQ(1u, tObjectiveMetadata[0].load_case_ids.size());
    ASSERT_STREQ("10", tObjectiveMetadata[0].load_case_ids[0].c_str());

    ASSERT_EQ(3u, tObjectiveMetadata[0].output_for_plotting.size());
    std::vector<std::string> tGoldOutputs = {"dispx", "dispy", "dispz"};
    for(auto& tValue : tObjectiveMetadata[0].output_for_plotting)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[0].output_for_plotting[0];
        ASSERT_STREQ(tGoldOutputs[tIndex].c_str(), tValue.c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseObjective_TwoObjective)
{
    std::string tStringInput =
        "begin objective\n"
        "   load ids 10\n"
        "   boundary condition ids 11\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dispy dispz\n"
        "end objective\n"
        "begin objective\n"
        "   type stress p-norm\n"
        "   normalize objective false\n"
        "   load ids 10 20\n"
        "   boundary condition ids 11 21\n"
        "   number processors 1\n"
        "   weight 1.0\n"
        "   number ranks 1\n"
        "   output for plotting DISpx dIspy dispz vonmises\n"
        "   distribute objective at most 3 processors\n"
        "end objective\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseObjective tObjectiveParser;
    tObjectiveParser.parse(tInputSS);
    auto tObjectiveMetadata = tObjectiveParser.data();
    ASSERT_EQ(2u, tObjectiveMetadata.size());

    // TEST OBJECTIVE 1
    ASSERT_STREQ("compliance", tObjectiveMetadata[0].type.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[0].num_procs.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[0].num_ranks.c_str());
    ASSERT_STREQ("1.0", tObjectiveMetadata[0].weight.c_str());
    ASSERT_STREQ("3.0", tObjectiveMetadata[0].mPenaltyParam.c_str());
    ASSERT_STREQ("6.0", tObjectiveMetadata[0].mPnormExponent.c_str());
    ASSERT_STREQ("1e-9", tObjectiveMetadata[0].mMinimumErsatzValue.c_str());
    ASSERT_STREQ("plato_analyze", tObjectiveMetadata[0].code_name.c_str());
    ASSERT_STREQ("plato_analyze_1", tObjectiveMetadata[0].mPerformerName.c_str());
    ASSERT_STREQ("true", tObjectiveMetadata[0].normalize_objective.c_str());
    ASSERT_STREQ("1e-7", tObjectiveMetadata[0].analysis_solver_tolerance.c_str());
    ASSERT_STREQ("false", tObjectiveMetadata[0].multi_load_case.c_str());
    ASSERT_STREQ("none", tObjectiveMetadata[0].distribute_objective_type.c_str());

    ASSERT_EQ(1u, tObjectiveMetadata[0].bc_ids.size());
    ASSERT_STREQ("11", tObjectiveMetadata[0].bc_ids[0].c_str());
    ASSERT_EQ(1u, tObjectiveMetadata[0].load_case_ids.size());
    ASSERT_STREQ("10", tObjectiveMetadata[0].load_case_ids[0].c_str());

    ASSERT_EQ(3u, tObjectiveMetadata[0].output_for_plotting.size());
    std::vector<std::string> tGoldOutputs = {"dispx", "dispy", "dispz"};
    for(auto& tValue : tObjectiveMetadata[0].output_for_plotting)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[0].output_for_plotting[0];
        ASSERT_STREQ(tGoldOutputs[tIndex].c_str(), tValue.c_str());
    }

    // TEST OBJECTIVE 2
    ASSERT_STREQ("stress p-norm", tObjectiveMetadata[1].type.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[1].num_procs.c_str());
    ASSERT_STREQ("1", tObjectiveMetadata[1].num_ranks.c_str());
    ASSERT_STREQ("1.0", tObjectiveMetadata[1].weight.c_str());
    ASSERT_STREQ("3.0", tObjectiveMetadata[1].mPenaltyParam.c_str());
    ASSERT_STREQ("6.0", tObjectiveMetadata[1].mPnormExponent.c_str());
    ASSERT_STREQ("1e-9", tObjectiveMetadata[1].mMinimumErsatzValue.c_str());
    ASSERT_STREQ("plato_analyze", tObjectiveMetadata[1].code_name.c_str());
    ASSERT_STREQ("plato_analyze_2", tObjectiveMetadata[1].mPerformerName.c_str());
    ASSERT_STREQ("false", tObjectiveMetadata[1].normalize_objective.c_str());
    ASSERT_STREQ("1e-7", tObjectiveMetadata[1].analysis_solver_tolerance.c_str());
    ASSERT_STREQ("true", tObjectiveMetadata[1].multi_load_case.c_str());
    ASSERT_STREQ("3", tObjectiveMetadata[1].atmost_total_num_processors.c_str());
    ASSERT_STREQ("atmost", tObjectiveMetadata[1].distribute_objective_type.c_str());

    std::vector<std::string> tGoldEssentialBCIDs = {"11", "21"};
    ASSERT_EQ(2u, tObjectiveMetadata[1].bc_ids.size());
    for(auto& tValue : tObjectiveMetadata[1].bc_ids)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[1].bc_ids[0];
        ASSERT_STREQ(tGoldEssentialBCIDs[tIndex].c_str(), tValue.c_str());
    }

    std::vector<std::string> tGoldNaturalBCIDs = {"10", "20"};
    ASSERT_EQ(2u, tObjectiveMetadata[1].load_case_ids.size());
    for(auto& tValue : tObjectiveMetadata[1].load_case_ids)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[1].load_case_ids[0];
        ASSERT_STREQ(tGoldNaturalBCIDs[tIndex].c_str(), tValue.c_str());
    }

    ASSERT_EQ(4u, tObjectiveMetadata[1].output_for_plotting.size());
    tGoldOutputs = {"dispx", "dispy", "dispz", "vonmises"};
    for(auto& tValue : tObjectiveMetadata[0].output_for_plotting)
    {
        auto tIndex = &tValue - &tObjectiveMetadata[0].output_for_plotting[0];
        ASSERT_STREQ(tGoldOutputs[tIndex].c_str(), tValue.c_str());
    }
}

TEST(PlatoTestXMLGenerator, Split)
{
    std::string tInput("1 2 3 4 5 6 7 8");
    std::vector<std::string> tOutput;
    XMLGen::split(tInput, tOutput);
    ASSERT_EQ(8u, tOutput.size());

    std::vector<std::string> tGold = {"1", "2", "3", "4", "5", "6", "7", "8"};
    for(auto& tValue : tOutput)
    {
        auto tIndex = &tValue - &tOutput[0];
        ASSERT_STREQ(tGold[tIndex].c_str(), tValue.c_str());
    }
}

}
// namespace PlatoTestXMLGenerator
