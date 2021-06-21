/*
 * XMLGeneratorParseLoads.cpp
 *
 *  Created on: Jan 5, 2021
 */

#include <algorithm>

#include "XMLGeneratorParseLoads.hpp"
#include "XMLGeneratorValidInputKeys.hpp"

namespace XMLGen
{

void ParseLoad::insertCoreProperties()
{
    mTags.insert({ "id", { { {"id"}, ""}, "" } });
    mTags.insert({ "type", { { {"type"}, ""}, "" } });
    mTags.insert({ "location_type", { { {"location_type"}, ""}, "" } });
    mTags.insert({ "location_name", { { {"location_name"}, ""}, "" } });
    mTags.insert({ "location_id", { { {"location_id"}, ""}, "" } });
    mTags.insert({ "value", { { {"value"}, ""}, "" } });
}

void ParseLoad::allocate()
{
    mTags.clear();
    this->insertCoreProperties();
}

void ParseLoad::setLoadIdentification(XMLGen::Load& aMetadata)
{
    if(aMetadata.id().empty())
    {
        auto tItr = mTags.find("id");
        if(tItr->second.first.second.empty())
        {
            THROWERR(std::string("Parse Load: load identification number is empty. ")
                + std::string("A unique load identification number must be assigned to a load block."))
        }
        aMetadata.id(tItr->second.first.second);
    }
}

void ParseLoad::setType(XMLGen::Load& aMetadata)
{
    if(aMetadata.value("type").empty())
    {
        auto tItr = mTags.find("type");
        if(tItr->second.first.second.empty())
        {
            THROWERR("Parse Load: load type is empty.")
        }
        aMetadata.property("type", tItr->second.first.second);
    }
}

void ParseLoad::setLocationType(XMLGen::Load& aMetadata)
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

void ParseLoad::setLocationName(XMLGen::Load& aMetadata)
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

void ParseLoad::setLocationID(XMLGen::Load& aMetadata)
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

void ParseLoad::setValueMetadata(XMLGen::Load& aMetadata)
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

void ParseLoad::setMetadata(XMLGen::Load& aMetadata)
{
    this->setLoadIdentification(aMetadata);
    this->setType(aMetadata);
    this->setLocationType(aMetadata);
    this->setLocationName(aMetadata);
    this->setLocationID(aMetadata);
    this->setValueMetadata(aMetadata);
}

void ParseLoad::checkUniqueIDs()
{
    std::vector<std::string> tIDs;
    for(auto& tEBC : mData)
    {
        tIDs.push_back(tEBC.id());
    }

    if(!XMLGen::unique(tIDs))
    {
        THROWERR("Parse Load: BoundaryCondition block identification numbers, i.e. IDs, are not unique. BoundaryCondition block IDs must be unique.")
    }
}

std::vector<XMLGen::Load> ParseLoad::data() const
{
    return mData;
}

void ParseLoad::parse(std::istream &aInputFile)
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
            XMLGen::Load tMetadata;
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
