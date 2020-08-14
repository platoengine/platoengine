/*
 * XMLGeneratorParseObjective.cpp
 *
 *  Created on: Jun 17, 2020
 */

#include <iostream>
#include <algorithm>

#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParseObjective.hpp"

namespace XMLGen
{

void ParseObjective::allocate()
{
    mTags.clear();

    mTags.insert({ "type", { { {"type"}, ""}, "" } });
    mTags.insert({ "weights", { { {"weights"}, ""}, "" } });
    mTags.insert({ "criteria", { { {"criteria"}, ""}, "" } });
    mTags.insert({ "services", { { {"services"}, ""}, "" } });
}

void ParseObjective::setType(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("type");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.type = tItr->second.first.second;
    }
    else
    {
        THROWERR(std::string("Parse Objective: objective 'type' keyword is empty. ")
            + "At least one objective must be defined for an optimization problem.")
    }
}

void ParseObjective::setWeights(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("weights");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tWeights;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tWeights);
        aMetadata.weights = tWeights;
    }
    else
    {
        THROWERR("Weights for criteria are not defined");
    }
}

void ParseObjective::setCriteriaIDs(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("criteria");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tCriteriaIDs;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tCriteriaIDs);
        aMetadata.criteriaIDs = tCriteriaIDs;
    }
    else
    {
        THROWERR("Objective criteria are not defined");
    }
}

void ParseObjective::setServiceIDs(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("services");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tServiceIDs;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tServiceIDs);
        aMetadata.serviceIDs = tServiceIDs;
    }
    else
    {
        THROWERR("Objective services are not defined");
    }
}

void ParseObjective::setMetaData(XMLGen::Objective &aMetadata)
{
    this->setType(aMetadata);
    this->setWeights(aMetadata);
    this->setServiceIDs(aMetadata);
    this->setCriteriaIDs(aMetadata);
}

void ParseObjective::checkType(const XMLGen::Objective &aMetadata)
{
    XMLGen::ValidObjectiveTypeKeys tValidKeys;
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), aMetadata.type);
    if (tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Parse Objective: 'type' keyword '") + aMetadata.type + "' is not supported. ")
    }
}

void ParseObjective::checkMetaData(XMLGen::Objective &aMetadata)
{
    this->checkType(aMetadata);
}

void ParseObjective::finalize()
{

}

XMLGen::Objective ParseObjective::data() const
{
    return mObjective;
}

void ParseObjective::parse(std::istream &aInputFile)
{
    this->allocate();
    constexpr int MAX_CHARS_PER_LINE = 10000;
    std::vector<char> tBuffer(MAX_CHARS_PER_LINE);
    bool tObjectiveRead = false;
    while (!aInputFile.eof())
    {
        // read an entire line into memory
        std::vector<std::string> tTokens;
        aInputFile.getline(tBuffer.data(), MAX_CHARS_PER_LINE);
        XMLGen::parse_tokens(tBuffer.data(), tTokens);
        XMLGen::to_lower(tTokens);

        std::string tTag;
        if (XMLGen::parse_single_value(tTokens, { "begin", "objective" }, tTag))
        {
            if(tObjectiveRead)
                THROWERR("Parse Objective Error: More than one Objective block found")
            tObjectiveRead = true;
            XMLGen::Objective tMetadata;
            XMLGen::erase_tag_values(mTags);
            XMLGen::parse_input_metadata( { "end", "objective" }, aInputFile, mTags);
            this->setMetaData(tMetadata);
            this->checkMetaData(tMetadata);
            mObjective = tMetadata;
        }
    }
    this->finalize();
}

}
// namespace XMLGen
