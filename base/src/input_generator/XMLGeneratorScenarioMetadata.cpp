/*
 * XMLGeneratorScenarioMetadata.cpp
 *
 *  Created on: Jul 22, 2020
 */

#include "XMLG_Macros.hpp"
#include "Plato_FreeFunctions.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorScenarioMetadata.hpp"

namespace XMLGen
{

std::string Scenario::getValue(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        return "";
    }
    return tItr->second;
}

bool Scenario::getBool(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        THROWERR(std::string("XML Generator Scenario Metadata: '") + aTag + "' keyword is not defined.")
    }
    return (XMLGen::transform_boolean_key(tItr->second));
}

std::string Scenario::value(const std::string& aTag) const
{
    auto tTag = Plato::tolower(aTag);
    auto tItr = mMetaData.find(tTag);
    if(tItr == mMetaData.end())
    {
        THROWERR(std::string("XML Generator Scenario Metadata: Parameter with tag '") + aTag + "' is not defined in metadata.")
    }
    return (tItr->second);
}

std::vector<std::string> Scenario::tags() const
{
    std::vector<std::string> tTags;
    for(auto& tProperty : mMetaData)
    {
        tTags.push_back(tProperty.first);
    }
    return tTags;
}

void Scenario::append(const std::string& aTag, const std::string& aValue)
{
    if (aTag.empty())
    {
        THROWERR(std::string("XML Generator Scenario Metadata: Parameter with tag '") + aTag + "' is empty.")
    }
    auto tTag = Plato::tolower(aTag);
    mMetaData[aTag] = aValue;
}

void Scenario::id(const std::string& aInput)
{
    mMetaData["id"] = aInput;
}

std::string Scenario::id() const
{
    return (this->getValue("id"));
}

void Scenario::physics(const std::string& aInput)
{
    mMetaData["physics"] = aInput;
}

std::string Scenario::physics() const
{
    return (this->getValue("physics"));
}

void Scenario::dimensions(const std::string& aInput)
{
    mMetaData["dimensions"] = aInput;
}

std::string Scenario::dimensions() const
{
    return (this->getValue("dimensions"));
}

void Scenario::materialPenaltyExponent(const std::string& aInput)
{
    mMetaData["material_penalty_exponent"] = aInput;
}

std::string Scenario::materialPenaltyExponent() const
{
    return (this->getValue("material_penalty_exponent"));
}

void Scenario::minErsatzMaterialConstant(const std::string& aInput)
{
    mMetaData["minimum_ersatz_material_value"] = aInput;
}

std::string Scenario::minErsatzMaterialConstant() const
{
    return (this->getValue("minimum_ersatz_material_value"));
}

}
// namespace XMLGen
