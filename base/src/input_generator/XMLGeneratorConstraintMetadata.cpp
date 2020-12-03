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

void Constraint::criterion(const std::string& aInput)
{
    mMetaData["criterion"] = aInput;
}

std::string Constraint::criterion() const
{
    return (this->getValue("criterion"));
}

void Constraint::service(const std::string& aInput)
{
    mMetaData["service"] = aInput;
}

std::string Constraint::service() const
{
    return (this->getValue("service"));
}

void Constraint::weight(const std::string& aInput)
{
    mMetaData["weight"] = aInput;
}

std::string Constraint::weight() const
{
    return (this->getValue("weight"));
}

void Constraint::scenario(const std::string& aInput)
{
    mMetaData["scenario"] = aInput;
}

std::string Constraint::scenario() const
{
    return (this->getValue("scenario"));
}
void Constraint::absoluteTarget(const std::string& aInput)
{
    mMetaData["absolute_target"] = aInput;
}

std::string Constraint::absoluteTarget() const
{
    return (this->getValue("absolute_target"));
}
void Constraint::relativeTarget(const std::string& aInput)
{
    mMetaData["relative_target"] = aInput;
}

std::string Constraint::relativeTarget() const
{
    return (this->getValue("relative_target"));
}

}
// namespace XMLGen
