/*
 * XMLGeneratorParseNaturalBoundaryCondition.cpp
 *
 *  Created on: Jan 5, 2021
 */

#include <algorithm>

#include "XMLGeneratorParseNaturalBoundaryCondition.hpp"
#include "XMLGeneratorValidInputKeys.hpp"

namespace XMLGen
{

void ParseNaturalBoundaryCondition::insertCoreProperties()
{
    mTags.insert({ "id", { { {"id"}, ""}, "" } });
    mTags.insert({ "type", { { {"type"}, ""}, "" } });
    mTags.insert({ "location_type", { { {"location_type"}, ""}, "" } });
    mTags.insert({ "location_name", { { {"location_name"}, ""}, "" } });
    mTags.insert({ "location_id", { { {"location_id"}, ""}, "" } });
    mTags.insert({ "value", { { {"value"}, ""}, "" } });
}

void ParseNaturalBoundaryCondition::allocate()
{
    mTags.clear();
    this->insertCoreProperties();
}

void ParseNaturalBoundaryCondition::setNaturalBoundaryConditionIdentification(XMLGen::NaturalBoundaryCondition& aMetadata)
{
    if(aMetadata.id().empty())
    {
        auto tItr = mTags.find("id");
        if(tItr->second.first.second.empty())
        {
            THROWERR(std::string("Parse NaturalBoundaryCondition: natural boundary condition identification number is empty. ")
                + std::string("A unique natural boundary condition identification number must be assigned to an natural boundary condition block."))
        }
        aMetadata.id(tItr->second.first.second);
    }
}

void ParseNaturalBoundaryCondition::setType(XMLGen::NaturalBoundaryCondition& aMetadata)
{
    if(aMetadata.value("type").empty())
    {
        auto tItr = mTags.find("type");
        if(tItr->second.first.second.empty())
        {
            THROWERR("Parse NaturalBoundaryCondition: natural boundary condition type is empty.")
        }
        aMetadata.property("type", tItr->second.first.second);
    }
}

void ParseNaturalBoundaryCondition::setLocationType(XMLGen::NaturalBoundaryCondition& aMetadata)
{
    if(aMetadata.value("location_type").empty())
    {
        auto tItr = mTags.find("location_type");
        if(!tItr->second.first.second.empty())
        {
            aMetadata.property("location_type", tItr->second.first.second);
        }
    }
}

void ParseNaturalBoundaryCondition::setLocationName(XMLGen::NaturalBoundaryCondition& aMetadata)
{
    if(aMetadata.value("location_name").empty())
    {
        auto tItr = mTags.find("location_name");
        if(!tItr->second.first.second.empty())
        {
            aMetadata.property("location_name", tItr->second.first.second);
        }
    }
}

void ParseNaturalBoundaryCondition::setLocationID(XMLGen::NaturalBoundaryCondition& aMetadata)
{
    if(aMetadata.value("location_id").empty())
    {
        auto tItr = mTags.find("location_id");
        if(!tItr->second.first.second.empty())
        {
            aMetadata.property("location_id", tItr->second.first.second);
        }
    }
}

void ParseNaturalBoundaryCondition::setValueMetadata(XMLGen::NaturalBoundaryCondition& aMetadata)
{
    if(aMetadata.load_values().size() == 0)
    {
        auto tItr = mTags.find("value");
        std::string tValues = tItr->second.first.second;
        if (tItr != mTags.end() && !tValues.empty())
        {
            std::vector<std::string> tParsedValues;
            char tValuesBuffer[10000];
            strcpy(tValuesBuffer, tValues.c_str());
            XMLGen::parse_tokens(tValuesBuffer, tParsedValues);
            aMetadata.load_values(tParsedValues);
        }
        else
        {
            THROWERR("Load values are not defined");
        }
    }
}

void ParseNaturalBoundaryCondition::setMetadata(XMLGen::NaturalBoundaryCondition& aMetadata)
{
    this->setNaturalBoundaryConditionIdentification(aMetadata);
    this->setType(aMetadata);
    this->setLocationType(aMetadata);
    this->setLocationName(aMetadata);
    this->setLocationID(aMetadata);
    this->setValueMetadata(aMetadata);
}

void ParseNaturalBoundaryCondition::checkUniqueIDs()
{
    std::vector<std::string> tIDs;
    for(auto& tEBC : mData)
    {
        tIDs.push_back(tEBC.id());
    }

    if(!XMLGen::unique(tIDs))
    {
        THROWERR("Parse NaturalBoundaryCondition: BoundaryCondition block identification numbers, i.e. IDs, are not unique.  BoundaryCondition block IDs must be unique.")
    }
}

std::vector<XMLGen::NaturalBoundaryCondition> ParseNaturalBoundaryCondition::data() const
{
    return mData;
}

void ParseNaturalBoundaryCondition::parse(std::istream &aInputFile)
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

        std::string tBCBlockID;
        if (XMLGen::parse_single_value(tTokens, { "begin", "load" }, tBCBlockID))
        {
            XMLGen::NaturalBoundaryCondition tMetadata;
            tMetadata.id(tBCBlockID);
            XMLGen::erase_tag_values(mTags);
            XMLGen::parse_input_metadata( { "end", "load" }, aInputFile, mTags);
            this->setMetadata(tMetadata);
            mData.push_back(tMetadata);
        }
    }
    this->checkUniqueIDs();
}

}
// namespace XMLGen
