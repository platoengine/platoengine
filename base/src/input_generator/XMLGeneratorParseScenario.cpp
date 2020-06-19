/*
 * XMLGeneratorParseScenario.cpp
 *
 *  Created on: Jun 18, 2020
 */

#include "XMLGeneratorParseScenario.hpp"
#include "XMLGeneratorValidInputKeys.hpp"

namespace XMLGen
{

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
    if(!XMLGen::is_integer(aInput))
    {
        THROWERR(std::string("Parse Scenario: keyword 'minimum_ersatz_material_value' with tag '") + aInput + "'is not a finite number.")
    }
}

void ParseScenario::allocate()
{
    mTags.clear();
    mTags.insert({ "code", { {"code"}, "" } });
    mTags.insert({ "physics", { {"physics"}, "" } });
    mTags.insert({ "performer", { {"performer"}, "" } });
    mTags.insert({ "dimensions", { {"dimensions"}, "" } });
    mTags.insert({ "scenario_id", { {"scenario_id"}, "" } });
    mTags.insert({ "material_penalty_exponent", { {"material_penalty_exponent"}, "" } });
    mTags.insert({ "minimum_ersatz_material_value", { {"minimum_ersatz_material_value"}, "" } });
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
        auto tPerformer = mData.code() + "_0";
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

void ParseScenario::setScenarioID()
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

void ParseScenario::setMetaData()
{
    this->setCode();
    this->setPhysics();
    this->setDimensions();
    this->setMateriaPenaltyExponent();
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

        std::string tTag;
        if (XMLGen::parse_single_value(tTokens, { "begin", "scenario" }, tTag))
        {
            XMLGen::parse_input_metadata( { "end", "scenario" }, aInputFile, mTags);
            this->setMetaData();
        }
    }
    this->setPerformer();
    this->setScenarioID();
}

}
// namespace XMLGen
