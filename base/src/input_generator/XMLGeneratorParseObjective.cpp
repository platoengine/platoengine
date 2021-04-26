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
    mTags.insert({ "shape_services", { { {"shape_services"}, ""}, "" } });
    mTags.insert({ "scenarios", { { {"scenarios"}, ""}, "" } });
    mTags.insert({ "multi_load_case", { { {"multi_load_case"}, ""}, "" } });
    //mTags.insert({ "standard_deviation_multiplier", { { {"standard_deviation_multiplier"}, ""}, "0" } });
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

void ParseObjective::setMultiLoadCase(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("multi_load_case");
    if (tItr != mTags.end() && !tItr->second.first.second.empty())
    {
        aMetadata.multi_load_case = tItr->second.first.second;
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
        mNumberComponents = tCriteriaIDs.size();
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

void ParseObjective::setShapeServiceIDs(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("shape_services");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tServiceIDs;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tServiceIDs);
        aMetadata.shapeServiceIDs = tServiceIDs;
    }
}

void ParseObjective::setScenarioIDs(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("scenarios");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tScenarioIDs;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tScenarioIDs);
        aMetadata.scenarioIDs = tScenarioIDs;
    }
    else
    {
        THROWERR("Objective scenarios are not defined");
    }
}

void ParseObjective::setWeights(XMLGen::Objective &aMetadata)
{
    auto tItr = mTags.find("weights");
    std::string tValues = tItr->second.first.second;
    if(aMetadata.type == "single_criterion" && tItr !=mTags.end() && !tValues.empty())
        THROWERR("Weights defined for single_criterion objective type")
    if(aMetadata.type == "weighted_sum")
    {
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
            THROWERR("Objective Weights are not defined for weighted_sum objective type");
        }
    }
}

void ParseObjective::setMetaData(XMLGen::Objective &aMetadata)
{
    this->setType(aMetadata);
    this->setCriteriaIDs(aMetadata);
    this->setServiceIDs(aMetadata);
    this->setShapeServiceIDs(aMetadata);
    this->setScenarioIDs(aMetadata);
    this->setWeights(aMetadata);
    this->setMultiLoadCase(aMetadata);
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

void ParseObjective::checkMetaData(const XMLGen::Objective &aMetadata)
{
    this->checkType(aMetadata);
    this->checkCriteriaIDs(aMetadata);
    this->checkServiceIDs(aMetadata);
    this->checkScenarioIDs(aMetadata);
    this->checkWeights(aMetadata);
}

void ParseObjective::checkCriteriaIDs(const XMLGen::Objective &aMetadata)
{
    if(aMetadata.type == "single_criterion")
    {
        if(aMetadata.criteriaIDs.size() != 1)
            THROWERR("Exactly 1 criterion is expected to be defined for single_criterion objective type")
    }
    else if(aMetadata.criteriaIDs.size() != mNumberComponents)
        THROWERR("All objective parameters must have the same number of entries")
}

void ParseObjective::checkServiceIDs(const XMLGen::Objective &aMetadata)
{
    if(aMetadata.type == "single_criterion")
    {
        if(aMetadata.serviceIDs.size() != 1)
            THROWERR("Exactly 1 service is expected to be defined for single_criterion objective type")
    }
    else if(aMetadata.serviceIDs.size() != mNumberComponents)
        THROWERR("All objective parameters must have the same number of entries")
}

void ParseObjective::checkScenarioIDs(const XMLGen::Objective &aMetadata)
{
    if(aMetadata.type == "single_criterion")
    {
        if(aMetadata.scenarioIDs.size() != 1)
            THROWERR("Exactly 1 scenario is expected to be defined for single_criterion objective type")
    }
    else if(aMetadata.scenarioIDs.size() != mNumberComponents)
        THROWERR("All objective parameters must have the same number of entries")
}

void ParseObjective::checkWeights(const XMLGen::Objective &aMetadata)
{
    if(aMetadata.type == "single_criterion")
    {
        if(aMetadata.weights.size() > 0)
            THROWERR("Weights should not be specified for single_criterion objective type")
    }
    else if(aMetadata.weights.size() != mNumberComponents)
        THROWERR("All objective parameters must have the same number of entries")
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
}

}
// namespace XMLGen
