/*
 * XMLGeneratorServiceMetadata.cpp
 *
 *  Created on: Jul 22, 2020
 */

#include "XMLG_Macros.hpp"
#include "Plato_FreeFunctions.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorServiceMetadata.hpp"

namespace XMLGen
{

std::string Service::getValue(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        return "";
    }
    return tItr->second;
}

bool Service::getBool(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        THROWERR(std::string("XML Generator Service Metadata: '") + aTag + "' keyword is not defined.")
    }
    return (XMLGen::transform_boolean_key(tItr->second));
}

std::string Service::value(const std::string& aTag) const
{
    auto tTag = Plato::tolower(aTag);
    auto tItr = mMetaData.find(tTag);
    if(tItr == mMetaData.end())
    {
        THROWERR(std::string("XML Generator Service Metadata: Parameter with tag '") + aTag + "' is not defined in metadata.")
    }
    return (tItr->second);
}

std::vector<std::string> Service::tags() const
{
    std::vector<std::string> tTags;
    for(auto& tProperty : mMetaData)
    {
        tTags.push_back(tProperty.first);
    }
    return tTags;
}

void Service::append(const std::string& aTag, const std::string& aValue)
{
    if (aTag.empty())
    {
        THROWERR(std::string("XML Generator Service Metadata: Input tag '") + aTag + "' is empty.")
    }
    auto tTag = Plato::tolower(aTag);
    mMetaData[aTag] = aValue;
}

void Service::id(const std::string& aInput)
{
    mMetaData["id"] = aInput;
}

std::string Service::id() const
{
    return (this->getValue("id"));
}

void Service::code(const std::string& aInput)
{
    mMetaData["code"] = aInput;
}

std::string Service::code() const
{
    return (this->getValue("code"));
}

void Service::numberRanks(const std::string& aInput)
{
    mMetaData["number_ranks"] = aInput;
}

std::string Service::numberRanks() const
{
    return (this->getValue("number_ranks"));
}

void Service::numberProcessors(const std::string& aInput)
{
    mMetaData["number_processors"] = aInput;
}

std::string Service::numberProcessors() const
{
    return (this->getValue("number_processors"));
}

void Service::performer(const std::string& aInput)
{
    mMetaData["performer"] = aInput;
}

std::string Service::performer() const
{
    return (this->getValue("performer"));
}

void Service::cacheState(const std::string& aInput)
{
    mMetaData["cache_state"] = aInput;
}

bool Service::cacheState() const
{
    return (this->getBool("cache_state"));
}

void Service::updateProblem(const std::string& aInput)
{
    mMetaData["update_problem"] = aInput;
}

bool Service::updateProblem() const
{
    return (this->getBool("update_problem"));
}

}
// namespace XMLGen
