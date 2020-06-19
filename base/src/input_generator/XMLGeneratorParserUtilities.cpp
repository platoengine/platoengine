/*
 * XMLGeneratorParserUtilities.cpp
 *
 *  Created on: Jun 16, 2020
 */

#include <string>
#include <sstream>
#include <numeric>

#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

void erase_tags(XMLGen::UseCaseTags& aTags)
{
    for (auto &tTag : aTags)
    {
        tTag.second.second.clear();
    }
}

void parse_input_metadata
(const std::vector<std::string>& aStopKeys,
 std::istream& aInputFile,
 XMLGen::UseCaseTags& aTags)
{
    constexpr int tMAX_CHARS_PER_LINE = 10000;
    std::vector<char> tBuffer(tMAX_CHARS_PER_LINE);
    while (!aInputFile.eof())
    {
        std::vector<std::string> tTokens;
        aInputFile.getline(tBuffer.data(), tMAX_CHARS_PER_LINE);
        XMLGen::parse_tokens(tBuffer.data(), tTokens);
        XMLGen::to_lower(tTokens);

        std::string tTag;
        if (XMLGen::parse_single_value(tTokens, aStopKeys, tTag))
        {
            break;
        }
        XMLGen::parse_tag_values(tTokens, aTags);
    }
}

bool parse_single_value
(const std::vector<std::string> &aTokens,
 const std::vector<std::string> &aTarget,
 std::string &aKeyword)
{
    aKeyword = "";
    if(aTarget.size() < 1 || aTokens.size() < 1 || aTokens.size() < aTarget.size())
    {
        return false;
    }

    for(auto& tString : aTarget)
    {
        auto tIndex = &tString - &aTarget[0];
        if(aTokens[tIndex].compare(tString))
        {
            return false;
        }
    }

    if(aTokens.size() == (aTarget.size() + 1))
    {
        auto tIndex = aTokens.size() + 1;
        aKeyword = aTokens[tIndex];
    }

    return true;
}
// function parse_single_value

std::string to_lower(const std::string &aInput)
{
    char tBuffer[500];
    std::string tOutput;
    if (aInput.size() > 500)
    {
        tOutput = "";
    }
    else
    {
        for (size_t tIndex = 0; tIndex < aInput.size(); ++tIndex)
        {
            tBuffer[tIndex] = tolower(aInput[tIndex]);
        }
        tBuffer[aInput.size()] = '\0';
        tOutput = tBuffer;
    }
    return tOutput;
}
// function to_lower

void to_lower(std::vector<std::string>& aInput)
{
    for(auto& tToken : aInput)
    {
        auto tIndex = &tToken - &aInput[0];
        aInput[tIndex] = XMLGen::to_lower(tToken);
    }
}
// function to_lower

bool parse_tokens(char *aBuffer, std::vector<std::string> &aTokens)
{
    const std::string tDELIMITER = " \t";
    constexpr int tMAX_TOKENS_PER_LINE = 5000;
    const char* tToken[tMAX_TOKENS_PER_LINE] = {}; // initialize to 0

    // parse the line
    tToken[0] = std::strtok(aBuffer, tDELIMITER.c_str()); // first token

    // If there is a comment...
    if(tToken[0] && std::strlen(tToken[0]) > 1 && tToken[0][0] == '/' && tToken[0][1] == '/')
    {
        aTokens.clear();
        return true;
    }

    int tN = 0;
    if (tToken[0]) // zero if line is blank
    {
        for (tN = 1; tN < tMAX_TOKENS_PER_LINE; tN++)
        {
            tToken[tN] = std::strtok(0, tDELIMITER.c_str()); // subsequent tokens
            if (!tToken[tN])
            {
                break; // no more tokens
            }
        }
    }
    for(int tIndex=0; tIndex<tN; ++tIndex)
    {
        aTokens.push_back(tToken[tIndex]);
    }

    return true;
}
// function parse_tokens

std::string transform_tag_values(const std::vector<std::string>& aTokens)
{
    std::ostringstream tOutput;
    for(auto& tToken : aTokens)
    {
        tOutput << tToken;
        auto tIndex = &tToken - &aTokens[0];
        if(tIndex != aTokens.size() - 1)
        {
            tOutput << " ";
        }
    }
    return (tOutput.str());
}
// function transform_tag_values

void parse_tag_values(const std::vector<std::string>& aTokens, XMLGen::UseCaseTags& aTags)
{
    for (auto& tTag : aTags)
    {
        std::vector<size_t> tMatch(tTag.second.first.size());
        for (auto& tToken : tTag.second.first)
        {
            auto tIndex = &tToken - &tTag.second.first[0];
            tMatch[tIndex] = tToken == aTokens[tIndex] ? 1 : 0;
        }
        auto tSum = std::accumulate(tMatch.begin(), tMatch.end(), 0);
        bool tFoundMatch = tSum == tMatch.size() ? true : false;
        if (tFoundMatch)
        {
            auto tBeginItr = aTokens.begin();
            auto tBeginIndex = tMatch.size();
            std::vector<std::string> tTokenList;
            for (auto tItr = std::next(tBeginItr, tBeginIndex); tItr != aTokens.end(); tItr++)
            {
                tTokenList.push_back(tItr.operator*());
            }
            tTag.second.second = XMLGen::transform_tag_values(tTokenList);
            break;
        }
    }
}
// function parse_tag_values

bool is_number(const std::string& aInput)
{
    auto tIsDigit = true;
    for(auto tValue : aInput)
    {
        if(std::isdigit(tValue) == false)
        {
            tIsDigit = false;
            break;
        }
    }
    return tIsDigit;
}
// function is_number

void split(const std::string& aInput, std::vector<std::string>& aOutput, bool aToLower)
{
    std::istringstream tInputSS(aInput);
    std::copy(std::istream_iterator<std::string>(tInputSS),
              std::istream_iterator<std::string>(),
              std::back_inserter(aOutput));

    if(aToLower) { XMLGen::to_lower(aOutput); }
}
// function split

}
