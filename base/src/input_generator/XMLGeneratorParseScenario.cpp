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

void ParseScenario::set()
{
    for(auto& tTag : mTags)
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
    mTags.clear();
    mTags.insert({ "id", { { {"id"}, ""}, "" } });
    mTags.insert({ "code", { { {"code"}, ""}, "plato_analyze" } });
    mTags.insert({ "physics", { { {"physics"}, ""}, "" } });
    mTags.insert({ "performer", { { {"performer"}, ""}, "" } });
    mTags.insert({ "dimensions", { { {"dimensions"}, ""}, "3" } });
    mTags.insert({ "enable_cache_state", { { {"enable_cache_state"}, ""}, "false" } });
    mTags.insert({ "analyze_new_workflow", { { {"analyze_new_workflow"}, ""}, "false" } });
    mTags.insert({ "enable_update_problem", { { {"enable_update_problem"}, ""}, "false" } });
    mTags.insert({ "additive_continuation", { { {"additive_continuation"}, ""}, "" } });
    mTags.insert({ "material_penalty_exponent", { { {"material_penalty_exponent"}, ""}, "3.0" } });
    mTags.insert({ "minimum_ersatz_material_value", { { {"minimum_ersatz_material_value"}, ""}, "1e-9" } });
    mTags.insert({ "use_new_analyze_uq_workflow", { { {"use_new_analyze_uq_workflow"}, ""}, "false" } });
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
    auto tItr = mTags.find("physics");
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

XMLGen::Scenario ParseScenario::data() const
{
    return mData;
}

void ParseScenario::parse(std::istream &aInputFile)
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

        std::string tScenarioID;
        if (XMLGen::parse_single_value(tTokens, { "begin", "scenario" }, tScenarioID))
        {
            XMLGen::is_metadata_block_id_valid(tTokens);
            XMLGen::parse_input_metadata( { "end", "scenario" }, aInputFile, mTags);
            this->set();
            mData.id(tScenarioID);
            this->check();
        }
    }
}

}
// namespace XMLGen
