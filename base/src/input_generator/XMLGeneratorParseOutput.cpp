/*
 * XMLGeneratorParseOutput.cpp
 *
 *  Created on: Jun 18, 2020
 */

#include <iostream>

#include "XMLGeneratorParseOutput.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

void ParseOutput::allocate()
{
    mTags.clear();
    mTags.insert({ "quantities_of_interest", { {"quantities_of_interest"}, "" } });
    mTags.insert({ "random_quantities_of_interest", { {"random_quantities_of_interest"}, "" } });
}

void ParseOutput::setRandomQoI()
{
    auto tItr = mTags.find("random_quantities_of_interest");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        XMLGen::ValidOutputKeys tValidKeys;

        std::vector<std::string> tTokens;
        XMLGen::split(tItr->second.second, tTokens);
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
    auto tItr = mTags.find("quantities_of_interest");
    if (tItr != mTags.end() && !tItr->second.second.empty())
    {
        XMLGen::ValidOutputKeys tValidKeys;

        std::vector<std::string> tTokens;
        XMLGen::split(tItr->second.second, tTokens);
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
            this->setDeterministicQoI();
            this->setRandomQoI();
        }
    }
}

}
// namespace XMLGen
