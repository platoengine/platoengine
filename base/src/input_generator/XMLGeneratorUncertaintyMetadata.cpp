/*
 * XMLGeneratorUncertaintyMetadata.cpp
 *
 *  Created on: Jul 26, 2020
 */

#include <iostream>
#include "XMLGeneratorUncertaintyMetadata.hpp"

namespace XMLGen
{

std::string Uncertainty::getValue(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        return "";
    }
    return tItr->second;
}

std::string Uncertainty::value(const std::string& aTag) const
{
    auto tTag = XMLGen::to_lower(aTag);
    auto tItr = mMetaData.find(tTag);
    if(tItr == mMetaData.end())
    {
        THROWERR(std::string("XML Generator Service Metadata: Parameter with tag '") + aTag + "' is not defined in metadata.")
    }
    return (tItr->second);
}

std::vector<std::string> Uncertainty::tags() const
{
    std::vector<std::string> tTags;
    for(auto& tProperty : mMetaData)
    {
        tTags.push_back(tProperty.first);
    }
    return tTags;
}

void Uncertainty::append(const std::string& aTag, const std::string& aValue)
{
    if (aTag.empty())
    {
        THROWERR(std::string("XML Generator Service Metadata: Input tag '") + aTag + "' is empty.")
    }
    auto tTag = XMLGen::to_lower(aTag);
    mMetaData[aTag] = aValue;
}

std::string Uncertainty::seed() const
{
    return (this->getValue("random_seed"));
}

void Uncertainty::seed(const std::string& aInput)
{
    mMetaData["random_seed"] = aInput;
}

std::string Uncertainty::dimensions() const
{
    return (this->getValue("dimensions"));
}

void Uncertainty::dimensions(const std::string& aInput)
{
    mMetaData["dimensions"] = aInput;
}

std::string Uncertainty::category() const
{
    return (this->getValue("category"));
}

void Uncertainty::category(const std::string& aInput)
{
    mMetaData["category"] = aInput;
}

std::string Uncertainty::tag() const
{
    return (this->getValue("tag"));
}

void Uncertainty::tag(const std::string& aInput)
{
    mMetaData["tag"] = aInput;
}

std::string Uncertainty::id() const
{
    return (this->getValue("id"));
}

void Uncertainty::id(const std::string& aInput)
{
    mMetaData["id"] = aInput;
}

std::string Uncertainty::attribute() const
{
    return (this->getValue("attribute"));
}

void Uncertainty::attribute(const std::string& aInput)
{
    mMetaData["attribute"] = aInput;
}

std::string Uncertainty::distribution() const
{
    return (this->getValue("distribution"));
}

void Uncertainty::distribution(const std::string& aInput)
{
    mMetaData["distribution"] = aInput;
}

std::string Uncertainty::samples() const
{
    return (this->getValue("number_samples"));
}

void Uncertainty::samples(const std::string& aInput)
{
    mMetaData["number_samples"] = aInput;
}

std::string Uncertainty::filename() const
{
    return (this->getValue("filename"));
}

void Uncertainty::filename(const std::string& aInput)
{
    mMetaData["filename"] = aInput;
}

std::string Uncertainty::std() const
{
    return (this->getValue("standard_deviation"));
}

void Uncertainty::std(const std::string& aInput)
{
    mMetaData["standard_deviation"] = aInput;
}

std::string Uncertainty::mean() const
{
    return (this->getValue("mean"));
}

void Uncertainty::mean(const std::string& aInput)
{
    mMetaData["mean"] = aInput;
}

std::string Uncertainty::guess() const
{
    return (this->getValue("initial_guess"));
}

void Uncertainty::guess(const std::string& aInput)
{
    mMetaData["initial_guess"] = aInput;
}

std::string Uncertainty::lower() const
{
    return (this->getValue("lower_bound"));
}

void Uncertainty::lower(const std::string& aInput)
{
    mMetaData["lower_bound"] = aInput;
}

std::string Uncertainty::upper() const
{
    return (this->getValue("upper_bound"));
}

void Uncertainty::upper(const std::string& aInput)
{
    mMetaData["upper_bound"] = aInput;
}

std::string Uncertainty::correlationFilename() const
{
    return (this->getValue("correlation_filename"));
}

void Uncertainty::correlationFilename(const std::string& aInput)
{
    mMetaData["correlation_filename"] = aInput;
}

}
// namespace XMLGen
