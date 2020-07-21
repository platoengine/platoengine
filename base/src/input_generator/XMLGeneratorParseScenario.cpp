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

void ParseScenario::allocate()
{
    mTags.clear();
    mTags.insert({ "id", { {"id"}, "" } });
    mTags.insert({ "code", { {"code"}, "" } });
    mTags.insert({ "physics", { {"physics"}, "" } });
    mTags.insert({ "performer", { {"performer"}, "" } });
    mTags.insert({ "dimensions", { {"dimensions"}, "" } });
    mTags.insert({ "enable_cache_state", { {"enable_cache_state"}, "" } });
    mTags.insert({ "analyze_new_workflow", { {"analyze_new_workflow"}, "" } });
    mTags.insert({ "enable_update_problem", { {"enable_update_problem"}, "" } });
    mTags.insert({ "additive_continuation", { {"additive_continuation"}, "" } });
    mTags.insert({ "material_penalty_exponent", { {"material_penalty_exponent"}, "" } });
    mTags.insert({ "minimum_ersatz_material_value", { {"minimum_ersatz_material_value"}, "" } });
    mTags.insert({ "use_new_analyze_uq_workflow", { {"use_new_analyze_uq_workflow"}, "" } });
}

void ParseScenario::setCode()
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

void ParseScenario::setPerformer()
{
    auto tItr = mTags.find("performer");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        mData.performer(tItr->second.second);
    }
    else
    {
        auto tPerformer = mData.code() + "_1";
        mData.performer(tPerformer);
    }
}

void ParseScenario::setPhysics()
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

void ParseScenario::setDimensions()
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

void ParseScenario::checkScenarioID()
{
    if (mData.id().empty())
    {
        auto tID = mData.code() + "_" + mData.physics() + "_1";
        mData.id(tID);
    }
}

void ParseScenario::setAdditiveContinuation()
{
    auto tItr = mTags.find("additive_continuation");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        mData.additiveContinuation(tItr->second.second);
    }
}

void ParseScenario::setMateriaPenaltyExponent()
{
    auto tItr = mTags.find("material_penalty_exponent");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        mData.materialPenaltyExponent(tItr->second.second);
    }
    else
    {
        mData.materialPenaltyExponent("3.0");
    }
}

void ParseScenario::setMinimumErsatzMaterialValue()
{
    auto tItr = mTags.find("minimum_ersatz_material_value");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        mData.minErsatzMaterialConstant(tItr->second.second);
    }
    else
    {
        mData.minErsatzMaterialConstant("1e-9");
    }
}

void ParseScenario::setCacheState()
{
    auto tItr = mTags.find("enable_cache_state");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        auto tFlag = XMLGen::check_boolean_key(tItr->second.second);
        mData.cacheState(tFlag);
    }
    else
    {
        mData.cacheState(false);
    }
}

void ParseScenario::setUpdateProblem()
{
    auto tItr = mTags.find("enable_update_problem");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        auto tFlag = XMLGen::check_boolean_key(tItr->second.second);
        mData.updateProblem(tFlag);
    }
    else
    {
        mData.updateProblem(false);
    }
}

void ParseScenario::setUseAnalyzeNewUQWorkflow()
{
    auto tItr = mTags.find("use_new_analyze_uq_workflow");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        auto tFlag = XMLGen::check_boolean_key(tItr->second.second);
        mData.useNewAnalyzeUQWorkflow(tFlag);
    }
    else
    {
        mData.useNewAnalyzeUQWorkflow(false);
    }
}

void ParseScenario::setMetaData()
{
    this->setCode();
    this->setPhysics();
    this->setDimensions();
    this->setCacheState();
    this->setUpdateProblem();
    this->setAdditiveContinuation();
    this->setMateriaPenaltyExponent();
    this->setUseAnalyzeNewUQWorkflow();
    this->setMinimumErsatzMaterialValue();
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
            mData.id(tScenarioID);
            XMLGen::parse_input_metadata( { "end", "scenario" }, aInputFile, mTags);
            this->setMetaData();
        }
    }
    this->setPerformer();
    this->checkScenarioID();
}

}
// namespace XMLGen
