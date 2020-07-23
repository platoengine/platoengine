/*
 * XMLGeneratorConstraintMetadata.cpp
 *
 *  Created on: Jul 22, 2020
 */

#include "XMLG_Macros.hpp"
#include "Plato_FreeFunctions.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorConstraintMetadata.hpp"

namespace XMLGen
{

std::string Constraint::getValue(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        return "";
    }
    return tItr->second;
}

std::string Constraint::value(const std::string& aTag) const
{
    auto tTag = Plato::tolower(aTag);
    auto tItr = mMetaData.find(tTag);
    if(tItr == mMetaData.end())
    {
        THROWERR(std::string("XML Generator Constraint Metadata: Parameter with tag '") + aTag + "' is not defined in metadata.")
    }
    return (tItr->second);
}

std::vector<std::string> Constraint::tags() const
{
    std::vector<std::string> tTags;
    for(auto& tProperty : mMetaData)
    {
        tTags.push_back(tProperty.first);
    }
    return tTags;
}

void Constraint::append(const std::string& aTag, const std::string& aValue)
{
    if (aTag.empty())
    {
        THROWERR(std::string("XML Generator Constraint Metadata: Parameter with tag '") + aTag + "' is empty.")
    }
    auto tTag = Plato::tolower(aTag);
    mMetaData[aTag] = aValue;
}

void Constraint::id(const std::string& aInput)
{
    mMetaData["id"] = aInput;
}

std::string Constraint::id() const
{
    return (this->getValue("id"));
}

void Constraint::name(const std::string& aInput)
{
    mMetaData["name"] = aInput;
}

std::string Constraint::name() const
{
    return (this->getValue("name"));
}

void Constraint::code(const std::string& aInput)
{
    mMetaData["code"] = aInput;
}

std::string Constraint::code() const
{
    return (this->getValue("code"));
}

void Constraint::weight(const std::string& aInput)
{
    mMetaData["weight"] = aInput;
}

std::string Constraint::weight() const
{
    return (this->getValue("weight"));
}

void Constraint::category(const std::string& aInput)
{
    mMetaData["type"] = aInput;
}

std::string Constraint::category() const
{
    return (this->getValue("type"));
}

void Constraint::performer(const std::string& aInput)
{
    mMetaData["performer"] = aInput;
}

std::string Constraint::performer() const
{
    return (this->getValue("performer"));
}

void Constraint::pnormExponent(const std::string& aInput)
{
    mMetaData["pnorm exponent"] = aInput;
}

std::string Constraint::pnormExponent() const
{
    return (this->getValue("pnorm exponent"));
}

void Constraint::normalizedTarget(const std::string& aInput)
{
    mMetaData["target normalized"] = aInput;
}

std::string Constraint::normalizedTarget() const
{
    return (this->getValue("target normalized"));
}

void Constraint::absoluteTarget(const std::string& aInput)
{
    mMetaData["target absolute"] = aInput;
}

std::string Constraint::absoluteTarget() const
{
    return (this->getValue("target absolute"));
}

void Constraint::materialPenaltyExponent(const std::string& aInput)
{
    mMetaData["penalty power"] = aInput;
}

std::string Constraint::materialPenaltyExponent() const
{
    return (this->getValue("penalty power"));
}

void Constraint::minErsatzMaterialConstant(const std::string& aInput)
{
    mMetaData["minimum ersatz material value"] = aInput;
}

std::string Constraint::minErsatzMaterialConstant() const
{
    return (this->getValue("minimum ersatz material value"));
}

}
// namespace XMLGen
