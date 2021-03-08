/*
 * XMLGeneratorOutputMetadata.cpp
 *
 *  Created on: Jul 4, 2020
 */

#include "XMLG_Macros.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorOutputMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

std::string Output::getValue(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        return "";
    }
    return tItr->second;
}

bool Output::getBool(const std::string& aTag) const
{
    auto tItr = mMetaData.find(aTag);
    if(tItr == mMetaData.end())
    {
        return false;
    }
    return (XMLGen::transform_boolean_key(tItr->second));
}

std::string Output::value(const std::string& aTag) const
{
    auto tTag = XMLGen::to_lower(aTag);
    auto tItr = mMetaData.find(tTag);
    if(tItr == mMetaData.end())
    {
        THROWERR(std::string("XML Generator Output Metadata: Parameter with tag '") + aTag + "' is not defined in metadata.")
    }
    return (tItr->second);
}

std::vector<std::string> Output::tags() const
{
    std::vector<std::string> tTags;
    for(auto& tProperty : mMetaData)
    {
        tTags.push_back(tProperty.first);
    }
    return tTags;
}

void Output::appendParam(const std::string& aTag, const std::string& aValue)
{
    if (aTag.empty())
    {
        THROWERR(std::string("XML Generator Output Metadata: Parameter with tag '") + aTag + "' is empty.")
    }
    auto tTag = XMLGen::to_lower(aTag);
    mMetaData[aTag] = aValue;
}

bool Output::isOutputDisabled() const
{
    return (this->getBool("disable"));
}
void Output::disableOutput()
{
    mMetaData["disable"] = "true";
}

bool Output::outputSamples() const
{
    return (this->getBool("output_samples"));
}
void Output::outputSamples(const std::string& aInput)
{
    mMetaData["output_samples"] = aInput;
}

void Output::serviceID(const std::string& aInput)
{
    mMetaData["service"] = aInput;
}

std::string Output::serviceID() const
{
    return (this->getValue("service"));
}

void Output::appendRandomQoI(const std::string& aID, const std::string& aDataLayout)
{
    auto tLowerOutputID = XMLGen::check_output_keyword(aID);
    auto tLowerLayout = XMLGen::check_data_layout_keyword(aDataLayout);
    auto tArgumentName = tLowerOutputID + " {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    auto tSharedDataName = tLowerOutputID + " {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    mRandomQoIs[tLowerOutputID] =
        { {"ArgumentName", tArgumentName}, {"SharedDataName", tSharedDataName}, {"DataLayout", tLowerLayout} };
}

void Output::appendDeterminsiticQoI(const std::string& aID, const std::string& aDataLayout)
{
    auto tValidOutputID = XMLGen::check_output_keyword(aID);
    auto tValidLayout = XMLGen::check_data_layout_keyword(aDataLayout);
    mDeterministicQoIs[tValidOutputID] =
        { {"ArgumentName", tValidOutputID}, {"SharedDataName", tValidOutputID}, {"DataLayout", tValidLayout} };
}

std::string Output::randomLayout(const std::string& aID) const
{
    auto tItr = mRandomQoIs.find(aID);
    if(tItr == mRandomQoIs.end())
    {
        THROWERR(std::string("Output Metadata: Did not find QoI with identifier '") + aID + "'.");
    }
    return (tItr->second.find("DataLayout")->second);
}

std::string Output::deterministicLayout(const std::string& aID) const
{
    auto tItr = mDeterministicQoIs.find(aID);
    if(tItr == mDeterministicQoIs.end())
    {
        THROWERR(std::string("Output Metadata: Did not find QoI with identifier '") + aID + "'.")
    }
    return (tItr->second.find("DataLayout")->second);
}

std::string Output::argumentName(const std::string& aID) const
{
    auto tRandomItr = mRandomQoIs.find(aID);
    auto tFoundRandomMatch = tRandomItr != mRandomQoIs.end();
    auto tDetrmItr = mDeterministicQoIs.find(aID);
    auto tFoundDetrmMatch = tDetrmItr != mDeterministicQoIs.end();

    std::string tOutput;
    if(tFoundRandomMatch && !tFoundDetrmMatch)
    {
        tOutput = tRandomItr->second.find("ArgumentName")->second;
    }
    else if(!tFoundRandomMatch && tFoundDetrmMatch)
    {
        tOutput = tDetrmItr->second.find("ArgumentName")->second;
    }
    else
    {
        THROWERR(std::string("Output Metadata: Did not find QoI with identifier '") + aID + "'.")
    }
    return tOutput;
}

std::string Output::randomArgumentName(const std::string& aID) const
{
    auto tItr = mRandomQoIs.find(aID);
    if(tItr == mRandomQoIs.end())
    {
        THROWERR(std::string("Output Metadata: Did not find QoI with identifier '") + aID + "'.")
    }
    return (tItr->second.find("ArgumentName")->second);
}

std::string Output::deterministicArgumentName(const std::string& aID) const
{
    auto tItr = mDeterministicQoIs.find(aID);
    if(tItr == mDeterministicQoIs.end())
    {
        THROWERR(std::string("Output Metadata: Did not find QoI with identifier '") + aID + "'.")
    }
    return (tItr->second.find("ArgumentName")->second);
}

std::string Output::sharedDataName(const std::string& aID) const
{
    auto tRandomItr = mRandomQoIs.find(aID);
    auto tFoundRandomMatch = tRandomItr != mRandomQoIs.end();
    auto tDetrmItr = mDeterministicQoIs.find(aID);
    auto tFoundDetrmMatch = tDetrmItr != mDeterministicQoIs.end();

    std::string tOutput;
    if(tFoundRandomMatch && !tFoundDetrmMatch)
    {
        tOutput = tRandomItr->second.find("SharedDataName")->second;
    }
    else if(!tFoundRandomMatch && tFoundDetrmMatch)
    {
        tOutput = tDetrmItr->second.find("SharedDataName")->second;
    }
    else
    {
        THROWERR(std::string("Output Metadata: Did not find QoI with identifier '") + aID + "'.")
    }
    return tOutput;
}

std::string Output::randomSharedDataName(const std::string& aID) const
{
    auto tItr = mRandomQoIs.find(aID);
    if(tItr == mRandomQoIs.end())
    {
        THROWERR(std::string("Output Metadata: Did not find QoI with identifier '") + aID + "'.")
    }
    return (tItr->second.find("SharedDataName")->second);
}

std::string Output::deterministicSharedDataName(const std::string& aID) const
{
    auto tItr = mDeterministicQoIs.find(aID);
    if(tItr == mDeterministicQoIs.end())
    {
        THROWERR(std::string("Output Metadata: Did not find QoI with identifier '") + aID + "'.")
    }
    return (tItr->second.find("SharedDataName")->second);
}

std::vector<std::string> Output::outputIDs() const
{
    std::vector<std::string> tOutputIDs;
    for(auto& tQoI : mRandomQoIs)
    {
        tOutputIDs.push_back(tQoI.first);
    }

    for(auto& tQoI : mDeterministicQoIs)
    {
        tOutputIDs.push_back(tQoI.first);
    }

    return tOutputIDs;
}

std::vector<std::string> Output::randomIDs() const
{
    std::vector<std::string> tOutput;
    for(auto& tPair : mRandomQoIs)
    {
        tOutput.push_back(tPair.first);
    }
    return tOutput;
}

std::vector<std::string> Output::deterministicIDs() const
{
    std::vector<std::string> tOutput;
    for(auto& tPair : mDeterministicQoIs)
    {
        tOutput.push_back(tPair.first);
    }
    return tOutput;
}

bool Output::isRandomMapEmpty() const
{
    return (mRandomQoIs.empty());
}

bool Output::isDeterministicMapEmpty() const
{
    return (mDeterministicQoIs.empty());
}

}
// namespace XMLGen

