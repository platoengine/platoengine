/*
 * XMLGeneratorParseConstraint.cpp
 *
 *  Created on: Jun 19, 2020
 */

#include <iostream>

#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParseConstraint.hpp"
#include "XMLGeneratorConstraintMetadata.hpp"

namespace XMLGen
{

void ParseConstraint::set(XMLGen::Constraint& aMetaData)
{
    for(auto& tTag : mTags)
    {
        if(tTag.second.first.second.empty())
        {
            auto tDefaultValue = tTag.second.second;
            aMetaData.append(tTag.first, tDefaultValue);
        }
        else
        {
            auto tInputValue = tTag.second.first.second;
            aMetaData.append(tTag.first, tInputValue);
        }
    }
}

void ParseConstraint::check(XMLGen::Constraint& aMetaData)
{
    this->checkCode(aMetaData);
    this->checkCategory(aMetaData);
    this->checkPerformer(aMetaData);
    this->checkTargeValue(aMetaData);
}

void ParseConstraint::allocate()
{
    mTags.clear();

    mTags.insert({ "id", { { {"id"}, ""}, "" } });
    mTags.insert({ "type", { { {"type"}, ""}, "" } });
    mTags.insert({ "name", { { {"name"}, ""}, "" } });
    mTags.insert({ "code", { { {"code"}, ""}, "" } });
    mTags.insert({ "target absolute", { { {"target", "absolute"}, ""}, "" } });
    mTags.insert({ "target normalized", { { {"target", "normalized"}, ""}, "" } });
    mTags.insert({ "weight", { { {"weight"}, ""}, "1.0" } });
    mTags.insert({ "performer", { { {"performer"}, ""}, "" } });
    mTags.insert({ "penalty power", { { {"penalty", "power"}, ""}, "3.0" } });
    mTags.insert({ "pnorm exponent", { { {"pnorm", "exponent"}, ""}, "6.0" } });
    mTags.insert({ "surface_area_sideset_id", { { {"surface_area_sideset_id"}, ""}, "" } });
    mTags.insert({ "minimum ersatz material value", { { {"minimum", "ersatz", "material", "value"}, ""}, "1e-9" } });
    mTags.insert({ "surface_area", { { {"surface_area"}, ""}, "" } });
    mTags.insert({ "volume absolute", { { {"volume", "absolute"}, ""}, "" } });
    mTags.insert({ "volume fraction", { { {"volume", "fraction"}, ""}, "" } });
}

std::string ParseConstraint::returnConstraintTargetAbsoluteKeywordSet(XMLGen::Constraint& aMetaData) const
{
    std::string tOutput;
    XMLGen::ValidConstraintTargetAbsoluteKeys tValidKeys;
    for(auto& tKeyword : tValidKeys.mKeys)
    {
        if(!aMetaData.value(tKeyword).empty())
        {
            tOutput = tKeyword;
            break;
        }
    }
    return tOutput;
}

std::string ParseConstraint::returnConstraintTargetNormalizedKeywordSet(XMLGen::Constraint& aMetaData) const
{
    std::string tOutput;
    XMLGen::ValidConstraintTargetNormalizedKeys tValidKeys;
    for(auto& tKeyword : tValidKeys.mKeys)
    {
        if(!aMetaData.value(tKeyword).empty())
        {
            tOutput = tKeyword;
            break;
        }
    }
    return tOutput;
}

void ParseConstraint::checkTargeValue(XMLGen::Constraint& aMetaData)
{
    auto tAbsoluteKeyword = this->returnConstraintTargetAbsoluteKeywordSet(aMetaData);
    auto tNormalizedKeyword = this->returnConstraintTargetNormalizedKeywordSet(aMetaData);
    if (tNormalizedKeyword.empty() && tAbsoluteKeyword.empty())
    {
        THROWERR(std::string("Parse Constraint: 'target normalized' and 'target absolute' keywords are empty in constraint with category '")
          + aMetaData.category() + "' and computed by '" + aMetaData.code() + "'.\nOne of the two keywords: 'target normalized' or 'target absolute' "
          + "must be defined in a constrained optimization problem.")
    }

    // set value
    if(tNormalizedKeyword.empty())
    {
        auto tValue = mTags.find(tAbsoluteKeyword)->second.first.second;
        aMetaData.absoluteTarget(tValue);
    }
    else
    {
        auto tValue = mTags.find(tNormalizedKeyword)->second.first.second;
        aMetaData.normalizedTarget(tValue);
    }
}

void ParseConstraint::checkIDs()
{
    for (auto &tOuterCriterion : mData)
    {
        // For each code name we will make sure there are names set
        auto tMyCodeName = tOuterCriterion.code();
        size_t tObjectiveIdentificationNumber = 0;
        for (auto &tInnerCriterion : mData)
        {
            if (!tInnerCriterion.code().compare(tMyCodeName))
            {
                tObjectiveIdentificationNumber++;
                if (tInnerCriterion.name().empty())
                {
                    tInnerCriterion.name(std::to_string(tObjectiveIdentificationNumber));
                }
            }
        }
    }
}

void ParseConstraint::checkCode(XMLGen::Constraint &aMetadata)
{
    auto tCode = aMetadata.value("code");
    if(tCode.empty())
    {
        THROWERR(std::string("Parse Constraint: 'code' keyword is empty in constraint with tag '")
            + aMetadata.category()  + "' and name/id '" + aMetadata.name() + "' is empty.")
    }
    auto tValidCode = XMLGen::check_code_keyword(aMetadata.value("code"));
    aMetadata.code(tValidCode);
}

void ParseConstraint::checkPerformer(XMLGen::Constraint &aMetadata)
{
    if(aMetadata.performer().empty())
    {
        auto tCode = aMetadata.value("code");
        aMetadata.performer(tCode);
    }
}

void ParseConstraint::checkCategory(XMLGen::Constraint &aMetadata)
{
    auto tValidCategory = XMLGen::check_criterion_category_keyword(aMetadata.value("type"));
    aMetadata.category(tValidCategory);
}

std::vector<XMLGen::Constraint> ParseConstraint::data() const
{
    return mData;
}

void ParseConstraint::parse(std::istream &aInputFile)
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

        std::string tID;
        if (XMLGen::parse_single_value(tTokens, { "begin", "constraint" }, tID))
        {
            XMLGen::is_metadata_block_id_valid(tTokens);
            XMLGen::Constraint tMetadata;
            XMLGen::erase_tag_values(mTags);
            XMLGen::parse_input_metadata( { "end", "constraint" }, aInputFile, mTags);
            this->set(tMetadata);
            tMetadata.id(tID);
            this->check(tMetadata);
            mData.push_back(tMetadata);
        }
    }

    this->checkIDs();
}

}
// namespace XMLGen
