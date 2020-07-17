/*
 * XMLGeneratorOutputMetadata.cpp
 *
 *  Created on: Jul 4, 2020
 */

#include "XMLG_Macros.hpp"
#include "XMLGeneratorOutputMetadata.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

bool Output::outputData() const
{
    return mOutputDataToFile;
}
void Output::outputData(const bool& aOutputData)
{
    mOutputDataToFile = aOutputData;
}

void Output::scenarioID(const std::string& aID)
{
    mScenarioID = aID;
}

std::string Output::scenarioID() const
{
    return mScenarioID;
}

void Output::appendRandomQoI(const std::string& aID, const std::string& aDataLayout)
{
    auto tLowerID = XMLGen::to_lower(aID);
    auto tLowerLayout = XMLGen::to_lower(aDataLayout);
    auto tArgumentName = tLowerID + " {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    auto tSharedDataName = tLowerID + " {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    mRandomQoIs[tLowerID] =
        { {"ArgumentName", tArgumentName}, {"SharedDataName", tSharedDataName}, {"DataLayout", tLowerLayout} };
}

void Output::appendDeterminsiticQoI(const std::string& aID, const std::string& aDataLayout)
{
    auto tLowerID = XMLGen::to_lower(aID);
    auto tLowerLayout = XMLGen::to_lower(aDataLayout);
    mDeterministicQoIs[tLowerID] =
        { {"ArgumentName", tLowerID}, {"SharedDataName", tLowerID}, {"DataLayout", tLowerLayout} };
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

