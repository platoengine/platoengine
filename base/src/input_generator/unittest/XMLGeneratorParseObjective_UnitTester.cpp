/*
 * XMLGeneratorParseObjective_UnitTester.cpp
 *
 *  Created on: Jun 16, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorParseOutput.hpp"
#include "XMLGeneratorParseObjective.hpp"
#include "XMLGeneratorParserUtilities.hpp"

#include "XMLGeneratorValidInputKeys.hpp"

namespace XMLGen
{

std::string check_code_keyword(const std::string& aInput)
{
    auto tLowerInput = Plato::tolower(aInput);
    XMLGen::ValidPhysicsPerformerKeys tValidKeys;
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerInput);
    if(tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Check Code Keyword: keyword 'code' with tag '") + tLowerInput + "' is not supported.")
    }
    return (tItr.operator*());
}

std::string check_physics_keyword(const std::string& aInput)
{
    auto tLowerInput = Plato::tolower(aInput);
    XMLGen::ValidPhysicsKeys tValidKeys;
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerInput);
    if(tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Check Physics Keyword: keyword 'physics' with tag '") + tLowerInput + "' is not supported.")
    }
    return (tItr.operator*());
}

std::string check_spatial_dimensions_keyword(const std::string& aInput)
{
    auto tLowerInput = Plato::tolower(aInput);
    XMLGen::ValidSpatialDimsKeys tValidKeys;
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerInput);
    if(tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Check Dimensions Keyword: keyword 'dimensions' with tag '") + tLowerInput + "' is not supported.")
    }
    return (tItr.operator*());
}

void check_number(const std::string& aInput)
{
    if(!XMLGen::is_number(aInput))
    {
        THROWERR(std::string("Parse Scenario: keyword 'minimum_ersatz_material_value' with tag '") + aInput + "'is not a finite number.")
    }
}

/******************************************************************************//**
 * \class ParseScenario
 * \brief Parse inputs in scenario block and store values in XMLGen::Scenario.
**********************************************************************************/
class ParseScenario : public XMLGen::ParseMetadata<XMLGen::Scenario>
{
private:
    XMLGen::Scenario mData; /*!< scenario metadata */
    XMLGen::UseCaseTags mTags; /*!< map from plato input file tags to valid tokens-value pairs, i.e. map<tag, pair<tokens,value> > */

private:
    void allocate()
    {
        mTags.clear();
        mTags.insert({ "code", { {"code"}, "" } });
        mTags.insert({ "physics", { {"physics"}, "" } });
        mTags.insert({ "dimensions", { {"dimensions"}, "" } });
        mTags.insert({ "scenario_id", { {"scenario_id"}, "" } });
        mTags.insert({ "material_penalty_exponent", { {"material_penalty_exponent"}, "" } });
        mTags.insert({ "minimum_ersatz_material_value", { {"minimum_ersatz_material_value"}, "" } });
    }

private:
    void setCode()
    {
        auto tItr = mTags.find("code");
        if (tItr != mTags.end() && !tItr->second.second.empty())
        {
            auto tValidTag = XMLGen::check_code_keyword(tItr->second.second);
            mData.code(tValidTag);
        }
        else
        {
            auto tValidTag = XMLGen::check_code_keyword("plato_analyze");
            mData.code(tValidTag);
        }
    }

    void setPhysics()
    {
        auto tItr = mTags.find("physics");
        if (tItr != mTags.end() && !tItr->second.second.empty())
        {
            auto tValidTag = XMLGen::check_physics_keyword(tItr->second.second);
            mData.physics(tValidTag);
        }
        else
        {
            THROWERR("Parse Scenario: keyword 'physics' is not defined.")
        }
    }

    void setDimensions()
    {
        auto tItr = mTags.find("dimensions");
        if (tItr != mTags.end() && !tItr->second.second.empty())
        {
            auto tValidTag = XMLGen::check_spatial_dimensions_keyword(tItr->second.second);
            mData.dimensions(tValidTag);
        }
        else
        {
            auto tValidTag = XMLGen::check_spatial_dimensions_keyword("3");
            mData.dimensions(tValidTag);
        }
    }

    void setScenarioID()
    {
        auto tItr = mTags.find("scenario_id");
        if (tItr != mTags.end() && !tItr->second.second.empty())
        {
            mData.id(tItr->second.second);
        }
        else
        {
            auto tID = mData.code() + "_" + mData.physics() + "_0";
            mData.id(tID);
        }
    }

    void setMateriaPenaltyExponent()
    {
        auto tItr = mTags.find("material_penalty_exponent");
        if (tItr != mTags.end() && !tItr->second.second.empty())
        {
            XMLGen::check_number(tItr->second.second);
            mData.materialPenaltyExponent(tItr->second.second);
        }
        else
        {
            mData.materialPenaltyExponent("3");
        }
    }

    void setMinimumErsatzMaterialValue()
    {
        auto tItr = mTags.find("minimum_ersatz_material_value");
        if (tItr != mTags.end() && !tItr->second.second.empty())
        {
            XMLGen::check_number(tItr->second.second);
            mData.minErsatzMaterialConstant(tItr->second.second);
        }
        else
        {
            mData.minErsatzMaterialConstant("3");
        }
    }

    void setMetaData()
    {
        this->setCode();
        this->setPhysics();
        this->setDimensions();
        this->setMateriaPenaltyExponent();
        this->setMinimumErsatzMaterialValue();
    }

public:
    /******************************************************************************//**
     * \fn data
     * \brief Return output metadata.
     * \return metadata
    **********************************************************************************/
    XMLGen::Scenario data() const
    {
        return mData;
    }

    /******************************************************************************//**
     * \fn parse
     * \brief Parse output metadata.
     * \param [in] aInputFile input file metadata.
    **********************************************************************************/
    void parse(std::istream &aInputFile)
    {
        this->allocate();
        constexpr int MAX_CHARS_PER_LINE = 10000;
        std::vector<char> tBuffer(MAX_CHARS_PER_LINE);
        while (!aInputFile.eof())
        {
            // read an entire line into memory
            std::vector<std::string> tTokens;
            aInputFile.getline(tBuffer.data(), MAX_CHARS_PER_LINE);
            XMLGen::parse_tokens(tBuffer.data(), tTokens);
            XMLGen::to_lower(tTokens);

            std::string tTag;
            if (XMLGen::parse_single_value(tTokens, { "begin", "scenario" }, tTag))
            {
                XMLGen::parse_input_metadata( { "end", "scenario" }, aInputFile, mTags);
                this->setMetaData();
            }
        }
        this->setScenarioID();
    }
};

}

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, ParseOutput_EmptyScenarioMetadata)
{
    std::string tStringInput =
        "begin scenario\n"
        "end scenario\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseScenario tScenarioParser;
    ASSERT_THROW(tScenarioParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseOutput_EmptyOutputMetadata)
{
    std::string tStringInput =
        "begin output\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));

    auto tOutputMetadata = tOutputParser.data();
    ASSERT_TRUE(tOutputMetadata.getRandomQoI().empty());
    ASSERT_TRUE(tOutputMetadata.getDeterminsiticQoI().empty());
}

TEST(PlatoTestXMLGenerator, ParseOutput_ErrorInvalidQoI)
{
    std::string tStringInput =
        "begin output\n"
        "   quantities_of_interest dispx dispy dispz hippo\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_THROW(tOutputParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseOutput_ErrorInvalidRandomQoI)
{
    std::string tStringInput =
        "begin output\n"
        "   quantities_of_interest dispx dispy dispz\n"
        "   random_quantities_of_interest dispx dispy hippo dispz\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_THROW(tOutputParser.parse(tInputSS), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ParseOutput_DeterministicOnly)
{
    std::string tStringInput =
        "begin output\n"
        "   quantities_of_interest dispx dispy dispz temperature\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));
    auto tOutputMetadata = tOutputParser.data();
    ASSERT_TRUE(tOutputMetadata.getRandomQoI().empty());
    ASSERT_FALSE(tOutputMetadata.getDeterminsiticQoI().empty());

    std::vector<std::string> tGoldDeterministic = {"dispx", "dispy", "dispz", "temperature"};
    for(auto& tValue : tOutputMetadata.getDeterminsiticQoI())
    {
        auto tItr = std::find(tGoldDeterministic.begin(), tGoldDeterministic.end(), tValue.first);
        ASSERT_TRUE(tItr != tGoldDeterministic.end());
        ASSERT_STREQ(tItr->c_str(), tValue.first.c_str());
        ASSERT_STREQ("nodal field", tValue.second.c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseOutput_RandomOnly)
{
    std::string tStringInput =
        "begin output\n"
        "   random_quantities_of_interest accumulated_plastic_strain temperature\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));
    auto tOutputMetadata = tOutputParser.data();
    ASSERT_FALSE(tOutputMetadata.getRandomQoI().empty());
    ASSERT_TRUE(tOutputMetadata.getDeterminsiticQoI().empty());

    std::vector<std::string> tGoldRandom = {"accumulated_plastic_strain", "temperature"};
    std::vector<std::string> tGoldRandomLayout = {"element field", "nodal field"};
    for(auto& tValue : tOutputMetadata.getRandomQoI())
    {
        auto tItr = std::find(tGoldRandom.begin(), tGoldRandom.end(), tValue.first);
        ASSERT_TRUE(tItr != tGoldRandom.end());
        ASSERT_STREQ(tItr->c_str(), tValue.first.c_str());

        tItr = std::find(tGoldRandomLayout.begin(), tGoldRandomLayout.end(), tValue.second);
        ASSERT_TRUE(tItr != tGoldRandomLayout.end());
        ASSERT_STREQ(tItr->c_str(), tValue.second.c_str());
    }
}

TEST(PlatoTestXMLGenerator, ParseOutput_DeterministicPlusRandom)
{
    std::string tStringInput =
        "begin output\n"
        "   quantities_of_interest dispx dispy dispz temperature\n"
        "   random_quantities_of_interest accumulated_plastic_strain temperature\n"
        "end output\n";
    std::istringstream tInputSS;
    tInputSS.str(tStringInput);

    XMLGen::ParseOutput tOutputParser;
    ASSERT_NO_THROW(tOutputParser.parse(tInputSS));
    auto tOutputMetadata = tOutputParser.data();
    ASSERT_FALSE(tOutputMetadata.getRandomQoI().empty());
    ASSERT_FALSE(tOutputMetadata.getDeterminsiticQoI().empty());

    std::vector<std::string> tGoldRandom = {"accumulated_plastic_strain", "temperature"};
    std::vector<std::string> tGoldRandomLayout = {"element field", "nodal field"};
    for(auto& tValue : tOutputMetadata.getRandomQoI())
    {
        auto tItr = std::find(tGoldRandom.begin(), tGoldRandom.end(), tValue.first);
        ASSERT_TRUE(tItr != tGoldRandom.end());
        ASSERT_STREQ(tItr->c_str(), tValue.first.c_str());

        tItr = std::find(tGoldRandomLayout.begin(), tGoldRandomLayout.end(), tValue.second);
        ASSERT_TRUE(tItr != tGoldRandomLayout.end());
        ASSERT_STREQ(tItr->c_str(), tValue.second.c_str());
    }
    std::vector<std::string> tGoldDeterministic = {"dispx", "dispy", "dispz", "temperature"};
    for(auto& tValue : tOutputMetadata.getDeterminsiticQoI())
    {
        auto tItr = std::find(tGoldDeterministic.begin(), tGoldDeterministic.end(), tValue.first);
        ASSERT_TRUE(tItr != tGoldDeterministic.end());
        ASSERT_STREQ(tItr->c_str(), tValue.first.c_str());
        ASSERT_STREQ("nodal field", tValue.second.c_str());
    }
}

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
