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
    if(aMetaData.criterion().empty())
        THROWERR("Criterion not defined for constraint " + aMetaData.id())
    if(aMetaData.service().empty())
        THROWERR("Service not defined for constraint " + aMetaData.id())
//    if(aMetaData.scenario().empty())
//        THROWERR("Scenario not defined for constraint " + aMetaData.id())
}

void ParseConstraint::allocate()
{
    mTags.clear();

    mTags.insert({ "id", { { {"id"}, ""}, "" } });
    mTags.insert({ "criterion", { { {"criterion"}, ""}, "" } });
    mTags.insert({ "service", { { {"service"}, ""}, "" } });
    mTags.insert({ "scenario", { { {"scenario"}, ""}, "" } });
    mTags.insert({ "relative_target", { { {"relative_target"}, ""}, "" } });
    mTags.insert({ "absolute_target", { { {"absolute_target"}, ""}, "" } });
    mTags.insert({ "weight", { { {"weight"}, ""}, "1.0" } });

//    mTags.insert({ "standard_deviation_multiplier", { { {"standard_deviation_multiplier"}, ""}, "0" } });

}

// std::string ParseConstraint::returnConstraintTargetAbsoluteKeywordSet(XMLGen::Constraint& aMetaData) const
// {
//     std::string tOutput;
//     XMLGen::ValidConstraintTargetAbsoluteKeys tValidKeys;
//     for(auto& tKeyword : tValidKeys.mKeys)
//     {
//         if(!aMetaData.value(tKeyword).empty())
//         {
//             tOutput = tKeyword;
//             break;
//         }
//     }
//     return tOutput;
// }

// std::string ParseConstraint::returnConstraintTargetNormalizedKeywordSet(XMLGen::Constraint& aMetaData) const
// {
//     std::string tOutput;
//     XMLGen::ValidConstraintTargetNormalizedKeys tValidKeys;
//     for(auto& tKeyword : tValidKeys.mKeys)
//     {
//         if(!aMetaData.value(tKeyword).empty())
//         {
//             tOutput = tKeyword;
//             break;
//         }
//     }
//     return tOutput;
// }

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
}

}
// namespace XMLGen
