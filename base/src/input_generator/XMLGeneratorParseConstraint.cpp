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
    if(aMetaData.scenario().empty())
        THROWERR("Scenario not defined for constraint " + aMetaData.id())
}

void ParseConstraint::allocate()
{
    mTags.clear();

    mTags.insert({ "id", { { {"id"}, ""}, "" } });
    mTags.insert({ "criterion", { { {"criterion"}, ""}, "" } });
    mTags.insert({ "service", { { {"service"}, ""}, "" } });
    mTags.insert({ "scenario", { { {"scenario"}, ""}, "" } });
    mTags.insert({ "target", { { {"target"}, ""}, "0.0" } });

    // // I'm not sure if it makes sense to keep these keywords with constraint
    // mTags.insert({ "penalty power", { { {"penalty", "power"}, ""}, "3.0" } });
    // mTags.insert({ "surface_area_sideset_id", { { {"surface_area_sideset_id"}, ""}, "" } });
    // mTags.insert({ "minimum ersatz material value", { { {"minimum", "ersatz", "material", "value"}, ""}, "1e-9" } });
    // mTags.insert({ "surface_area", { { {"surface_area"}, ""}, "" } });
    // mTags.insert({ "standard_deviation_multiplier", { { {"standard_deviation_multiplier"}, ""}, "0" } });
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
}

}
// namespace XMLGen
