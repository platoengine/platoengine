/*
 * XMLGeneratorParseCriteria.cpp
 *
 *  Created on: Jun 23, 2020
 */

#include <algorithm>

#include "XMLGeneratorParseCriteria.hpp"
#include "XMLGeneratorValidInputKeys.hpp"

namespace XMLGen
{

void ParseCriteria::setTags(XMLGen::Criterion& aCriterion)
{
    for(auto& tTag : mTags)
    {
        if(tTag.second.first.second.empty())
        {
            auto tDefaultValue = tTag.second.second;
            aCriterion.append(tTag.first, tDefaultValue);
        }
        else
        {
            auto tInputValue = tTag.second.first.second;
            aCriterion.append(tTag.first, tInputValue);
        }
    }
}

std::string check_criterion_type_key
(const std::string& aKeyword)
{
    XMLGen::ValidCriterionTypeKeys tValidKeys;
    auto tLowerKey = XMLGen::to_lower(aKeyword);
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerKey);
    if(tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Check Criterion type Key: Criterion type '") + tLowerKey + "' is not supported.")
    }
    return tLowerKey;
}

void ParseCriteria::allocate()
{
    mTags.clear();

    // core properties
    mTags.insert({ "type", { { {"type"}, ""}, "" } });
    mTags.insert({ "normalize", { { {"normalize"}, ""}, "false" } });
    mTags.insert({ "normalization_value", { { {"normalization_value"}, ""}, "1.0" } });

    // stress/flux p-norm parameter
    mTags.insert({ "p", { { {"p"}, ""}, "" } });


    // // These keywords came from the constraint block but should eventually belong to the criterion block
    // mTags.insert({ "surface_area_sideset_id", { { {"surface_area_sideset_id"}, ""}, "" } });
    // mTags.insert({ "surface_area", { { {"surface_area"}, ""}, "" } });
    // mTags.insert({ "minimum ersatz material value", { { {"minimum", "ersatz", "material", "value"}, ""}, "1e-9" } });
    
    // // Not sure where these ones should go..
    // mTags.insert({ "penalty power", { { {"penalty", "power"}, ""}, "3.0" } });
    // mTags.insert({ "standard_deviation_multiplier", { { {"standard_deviation_multiplier"}, ""}, "0" } });
}

void ParseCriteria::setCriterionType(XMLGen::Criterion& aMetadata)
{
    auto tItr = mTags.find("type");
    if (tItr->second.first.second.empty())
    {
        THROWERR("Parse Criteria: Criterion type is not defined.")
    }
    else
    {
        auto tValidCriterionType = XMLGen::check_criterion_type_key(tItr->second.first.second);
        aMetadata.type(tValidCriterionType);
    }
}

void ParseCriteria::setCriterionParameters(XMLGen::Criterion& aMetadata)
{
    XMLGen::ValidCriterionParameterKeyMap tValidMap;
    XMLGen::ValidKeys tValidKeys;
    std::string tCriterionType = mTags.find("type")->second.first.second;
    tValidKeys.mKeys = tValidMap.getValidKeysForCriterion(tCriterionType);

    for(auto& tKeyword : tValidKeys.mKeys)
    {
        auto tItr = mTags.find(tKeyword);
        if(tItr == mTags.end())
        {
            THROWERR(std::string("Parse Criteria: Criterion parameter '") + tKeyword + "' is not a valid keyword.")
        }

        if(!tItr->second.first.second.empty())
        {
            aMetadata.parameter(tKeyword, tItr->second.first.second);
        }
        else
            THROWERR("Parse Criteria: " + tKeyword + " parameter required for " + tCriterionType + " criterion type" )
    }

    for(auto& tTag : mTags)
    {
        if(!tTag.second.first.second.empty() && tTag.first != "type" && tTag.first != "normalize" && tTag.first != "normalization_value")
        {
            bool found_invalid_parameter = true;
            for(auto& tKeyword : tValidKeys.mKeys)
            {
                if(tTag.first == tKeyword)
                    found_invalid_parameter = false;
            }
            if(found_invalid_parameter)
                THROWERR("Parse Criterion: " + tTag.first + " is an invalid parameter for " + tCriterionType + " criterion type")
        }
    }
}

void ParseCriteria::setMetadata(XMLGen::Criterion& aMetadata)
{
    this->setCriterionType(aMetadata);
    this->setTags(aMetadata);
    this->setCriterionParameters(aMetadata);
}

void ParseCriteria::checkUniqueIDs()
{
    std::vector<std::string> tIDs;
    for(auto& tCriterion : mData)
    {
        tIDs.push_back(tCriterion.id());
    }

    if(!XMLGen::unique(tIDs))
    {
        THROWERR("Parse Criteria: Criterion block identification numbers, i.e. IDs, are not unique.  Criterion block IDs must be unique.")
    }
}

std::vector<XMLGen::Criterion> ParseCriteria::data() const
{
    return mData;
}

void ParseCriteria::parse(std::istream &aInputFile)
{
    mData.clear();
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

        std::string tCriterionBlockID;
        if (XMLGen::parse_single_value(tTokens, { "begin", "criterion" }, tCriterionBlockID))
        {
            XMLGen::Criterion tMetadata;
            if(tCriterionBlockID.empty())
            {
                THROWERR(std::string("Parse Criteria: criterion identification number is empty. ")
                    + "A unique criterion identification number must be assigned to each criterion block.")
            }
            tMetadata.id(tCriterionBlockID);
            XMLGen::erase_tag_values(mTags);
            XMLGen::parse_input_metadata( { "end", "criterion" }, aInputFile, mTags);
            this->setMetadata(tMetadata);
            mData.push_back(tMetadata);
        }
    }
    this->checkUniqueIDs();
}

}
// namespace XMLGen
