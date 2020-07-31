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
    mTags.insert({ "scenario", { { {"scenario"}, ""}, "" } });
    mTags.insert({ "disable", { { {"disable"}, ""}, "false" } });
    mTags.insert({ "statistics", { { {"statistics"}, ""}, "" } });
    mTags.insert({ "output_samples", { { {"output_samples"}, ""}, "false" } });
}

void ParseOutput::setParameters()
{
    for(auto& tTag : mTags)
    {
        if(tTag.second.first.second.empty())
        {
            auto tDefaultValue = tTag.second.second;
            mData.appendParam(tTag.first, tDefaultValue);
        }
        else
        {
            auto tInputValue = tTag.second.first.second;
            mData.appendParam(tTag.first, tInputValue);
        }
    }
}

void ParseOutput::checkOutputData()
{
    if(!mData.isDeterministicMapEmpty() && !mData.isRandomMapEmpty())
    {
        THROWERR("Parse Output: Invalid use case - 'statistics' and 'data' keywords cannot be used simultaneously.")
    }
}

void ParseOutput::checkScenario()
{
    auto tItr = mTags.find("scenario");
    if(tItr->second.first.second.empty())
    {
        THROWERR(std::string("Parse Output: Scenario identifier (id) is not defined. ")
           + "Output quantities of interest must be associated with a Plato 'scenario'.")
    }
}

void ParseOutput::checkMetaData()
{
    this->checkScenario();
    this->checkOutputData();
}

void ParseOutput::setRandomQoI()
{
    auto tItr = mTags.find("statistics");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        XMLGen::ValidOutputToLayoutKeys tValidKeys;
        std::vector<std::string> tTokens;
        XMLGen::split(tItr->second.first.second, tTokens);
        for (auto &tToken : tTokens)
        {
            auto tLowerToken = Plato::tolower(tToken);
            auto tItr = tValidKeys.mKeys.find(tLowerToken);
            if(tItr == tValidKeys.mKeys.end())
            {
                THROWERR(std::string("Parse Output: Output random quantity of interest with tag '")
                    + tLowerToken + "' is not supported.")
            }
            mData.appendRandomQoI(tItr->first, tItr->second);
        }
    }
}

void ParseOutput::setDeterministicQoI()
{
    auto tItr = mTags.find("data");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        XMLGen::ValidOutputToLayoutKeys tValidKeys;
        std::vector<std::string> tTokens;
        XMLGen::split(tItr->second.first.second, tTokens);
        for (auto &tToken : tTokens)
        {
            auto tLowerToken = Plato::tolower(tToken);
            auto tItr = tValidKeys.mKeys.find(tLowerToken);
            if(tItr == tValidKeys.mKeys.end())
            {
                THROWERR(std::string("Parse Output: Output quantity of interest with tag '")
                    + tLowerToken + "' is not supported.")
            }
            mData.appendDeterminsiticQoI(tItr->first, tItr->second);
        }
    }
}

void ParseOutput::setMetaData()
{
    this->setParameters();
    this->setRandomQoI();
    this->setDeterministicQoI();
}

XMLGen::Output ParseOutput::data() const
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
            XMLGen::parse_input_metadata( { "end", "output" }, aInputFile, mTags);
            this->setMetaData();
            this->checkMetaData();
        }
    }
}

}
// namespace XMLGen
