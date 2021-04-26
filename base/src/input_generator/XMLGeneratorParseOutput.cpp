/*
 * XMLGeneratorParseOutput.cpp
 *
 *  Created on: Jun 18, 2020
 */

#include <iostream>

#include "XMLGeneratorParseOutput.hpp"
#include "XMLGeneratorOutputMetadata.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

void ParseOutput::allocate()
{
    mTags.clear();
    mTags.insert({ "data", { { {"data"}, ""}, "" } });
    mTags.insert({ "service", { { {"service"}, ""}, "" } });
    mTags.insert({ "disable", { { {"disable"}, ""}, "false" } });
    mTags.insert({ "statistics", { { {"statistics"}, ""}, "" } });
    mTags.insert({ "output_samples", { { {"output_samples"}, ""}, "false" } });
}

void ParseOutput::setParameters(XMLGen::Output &aOutput)
{
    for(auto& tTag : mTags)
    {
        if(tTag.second.first.second.empty())
        {
            auto tDefaultValue = tTag.second.second;
            aOutput.appendParam(tTag.first, tDefaultValue);
        }
        else
        {
            auto tInputValue = tTag.second.first.second;
            aOutput.appendParam(tTag.first, tInputValue);
        }
    }
}

void ParseOutput::checkOutputData(XMLGen::Output &aOutput)
{
    if(!aOutput.isDeterministicMapEmpty() && !aOutput.isRandomMapEmpty())
    {
        THROWERR("Parse Output: Invalid use case - 'statistics' and 'data' keywords cannot be used simultaneously.")
    }
}

void ParseOutput::checkService()
{
    auto tItr = mTags.find("service");
    if(tItr->second.first.second.empty())
    {
        THROWERR(std::string("Parse Output: Service identifier (id) is not defined. ")
           + "Output quantities of interest must be associated with a Plato 'service'.")
    }
}

void ParseOutput::checkMetaData(XMLGen::Output &aOutput)
{
    this->checkService();
    this->checkOutputData(aOutput);
}

void ParseOutput::setRandomQoI(XMLGen::Output &aOutput)
{
    auto tItr = mTags.find("statistics");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        XMLGen::ValidOutputToLayoutKeys tValidKeys;
        std::vector<std::string> tOutputQoIs;
        XMLGen::split(tItr->second.first.second, tOutputQoIs);
        for (auto& tOutputQoI : tOutputQoIs)
        {
            auto tLayout = tValidKeys.value(tOutputQoI);
            if(tLayout.empty())
            {
                THROWERR(std::string("Parse Output: Output random quantity of interest with tag '") + tOutputQoI + "' is not supported.")
            }
            auto tLowertOutputQoI = Plato::tolower(tOutputQoI);
            aOutput.appendRandomQoI(tLowertOutputQoI, tLayout);
        }
    }
}

void ParseOutput::setDeterministicQoI(XMLGen::Output &aOutput)
{
    auto tItr = mTags.find("data");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        XMLGen::ValidOutputToLayoutKeys tValidKeys;
        std::vector<std::string> tOutputQoIs;
        XMLGen::split(tItr->second.first.second, tOutputQoIs);
        for (auto& tOutputQoI : tOutputQoIs)
        {
            auto tLayout = tValidKeys.value(tOutputQoI);
            if(tLayout.empty())
            {
                THROWERR(std::string("Parse Output: Output quantity of interest with tag '") + tOutputQoI + "' is not supported.")
            }
            auto tLowerToken = Plato::tolower(tOutputQoI);
            aOutput.appendDeterminsiticQoI(tLowerToken, tLayout);
        }
    }
}

void ParseOutput::setMetaData(XMLGen::Output &aOutput)
{
    this->setParameters(aOutput);
    this->setRandomQoI(aOutput);
    this->setDeterministicQoI(aOutput);
}

std::vector<XMLGen::Output> ParseOutput::data() const
{
    return mData;
}

void ParseOutput::parse(std::istream &aInputFile)
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
        if (XMLGen::parse_single_value(tTokens, { "begin", "output" }, tTag))
        {
            XMLGen::Output tOutput;
            XMLGen::parse_input_metadata( { "end", "output" }, aInputFile, mTags);
            this->setMetaData(tOutput);
            this->checkMetaData(tOutput);
            mData.push_back(tOutput);
        }
    }
}

}
// namespace XMLGen
