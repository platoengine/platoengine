/*
 * XMLGeneratorParseScenario.cpp
 *
 *  Created on: Jun 18, 2020
 */

#include <algorithm>

#include "XMLGeneratorParseScenario.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

void ParseScenario::setMainTags()
{
    for(auto& tTag : mMainTags)
    {
        if(tTag.second.first.second.empty())
        {
            auto tDefaultValue = tTag.second.second;
            mData.append(tTag.first, tDefaultValue);
        }
        else
        {
            auto tInputValue = tTag.second.first.second;
            mData.append(tTag.first, tInputValue);
        }
    }
}

void ParseScenario::check()
{
    this->checkCode();
    this->checkPhysics();
    this->checkPerformer();
    this->checkScenarioID();
    this->checkSpatialDimensions();
}

void ParseScenario::allocate()
{
    mMainTags.clear();
    mMainTags.insert({ "id", { { {"id"}, ""}, "" } });
    mMainTags.insert({ "code", { { {"code"}, ""}, "plato_analyze" } });
    mMainTags.insert({ "physics", { { {"physics"}, ""}, "" } });
    mMainTags.insert({ "performer", { { {"performer"}, ""}, "" } });
    mMainTags.insert({ "dimensions", { { {"dimensions"}, ""}, "3" } });
    mMainTags.insert({ "enable_cache_state", { { {"enable_cache_state"}, ""}, "false" } });
    mMainTags.insert({ "analyze_new_workflow", { { {"analyze_new_workflow"}, ""}, "false" } });
    mMainTags.insert({ "enable_update_problem", { { {"enable_update_problem"}, ""}, "false" } });
    mMainTags.insert({ "additive_continuation", { { {"additive_continuation"}, ""}, "" } });
    mMainTags.insert({ "material_penalty_exponent", { { {"material_penalty_exponent"}, ""}, "3.0" } });
    mMainTags.insert({ "minimum_ersatz_material_value", { { {"minimum_ersatz_material_value"}, ""}, "1e-9" } });
    mMainTags.insert({ "use_new_analyze_uq_workflow", { { {"use_new_analyze_uq_workflow"}, ""}, "false" } });

    mMainTags.insert({ "number_time_steps", { { {"number_time_steps"}, ""}, "40" } });
    mMainTags.insert({ "max_number_time_steps", { { {"max_number_time_steps"}, ""}, "160" } });
    mMainTags.insert({ "time_step_expansion_multiplier", { { {"time_step_expansion_multiplier"}, ""}, "1.25" } });
}

void ParseScenario::checkCode()
{
    auto tValidCode = XMLGen::check_code_keyword(mData.value("code"));
    mData.code(tValidCode);
}

void ParseScenario::checkPerformer()
{
    auto tPerformer = mData.value("performer");
    if (tPerformer.empty())
    {
        tPerformer = mData.value("code") + "_1";
        mData.performer(tPerformer);
    }
}

void ParseScenario::checkSpatialDimensions()
{
    XMLGen::ValidSpatialDimsKeys tValidKeys;
    auto tDim = mData.value("dimensions");
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tDim);
    if (tItr == tValidKeys.mKeys.end())
    {
        THROWERR("Parse Scenario: Problems with " + tDim + "-D spatial dimensions are not supported.")
    }
}

void ParseScenario::checkPhysics()
{
    auto tPhysics = mData.value("physics");
    auto tItr = mMainTags.find("physics");
    if (tPhysics.empty())
    {
        THROWERR("Parse Scenario: keyword 'physics' is empty.")
    }
    auto tValidPhysics = XMLGen::check_physics_keyword(tPhysics);
    mData.physics(tValidPhysics);
}

void ParseScenario::checkScenarioID()
{
    auto tID = mData.value("id");
    if (tID.empty())
    {
        tID = mData.value("code") + "_" + mData.value("physics") + "_1";
        mData.id(tID);
    }
}

void ParseScenario::setMainTagsMetaData(std::istream &aInputFile)
{
    constexpr int MAX_CHARS_PER_LINE = 10000;
    std::vector<char> tBuffer(MAX_CHARS_PER_LINE);
    while (!aInputFile.eof())
    {
        // read an entire line into memory
        std::vector<std::string> tTokens;
        aInputFile.getline(tBuffer.data(), MAX_CHARS_PER_LINE);
        XMLGen::parse_tokens(tBuffer.data(), tTokens);
        XMLGen::to_lower(tTokens);

        std::string tScenarioID;
        if (XMLGen::parse_single_value(tTokens, { "begin", "scenario" }, tScenarioID))
        {
            XMLGen::is_metadata_block_id_valid(tTokens);
            XMLGen::parse_input_metadata( { "end", "scenario" }, aInputFile, mMainTags);
            this->setMainTags();
            mData.id(tScenarioID);
            this->check();
        }
    }
}

XMLGen::Scenario ParseScenario::data() const
{
    return mData;
}

void ParseScenario::parse(std::istream &aInputFile)
{
    this->allocate();
    this->setMainTagsMetaData(aInputFile);
}

}
// namespace XMLGen
