/*
 * XMLGeneratorParseModel.cpp
 *
 *  Created on: Jun 18, 2020
 */

#include <algorithm>

#include "XMLGeneratorParseModels.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

void ParseModel::setTags(XMLGen::Model& aModel)
{
    for(auto& tTag : mTags)
    {
        if(tTag.second.first.second.empty())
        {
            auto tDefaultValue = tTag.second.second;
            aModel.append(tTag.first, tDefaultValue);
        }
        else
        {
            auto tInputValue = tTag.second.first.second;
            aModel.append(tTag.first, tInputValue);
        }
    }
}

void ParseModel::setBlockIDs(XMLGen::Model &aMetadata)
{
    auto tItr = mTags.find("blocks");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tBlockIDS;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tBlockIDS);
        aMetadata.block_ids = tBlockIDS;
    }
    else
    {
        THROWERR("Block IDs are not defined for model");
    }
}

void ParseModel::setMaterialIDs(XMLGen::Model &aMetadata)
{
    auto tItr = mTags.find("materials");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tMaterialIDs;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tMaterialIDs);
        aMetadata.material_ids = tMaterialIDs;
    }
    else
    {
        THROWERR("Block IDs are not defined for model");
    }
}

void ParseModel::checkTags(XMLGen::Model& aModel)
{
}

void ParseModel::allocate()
{
    mTags.clear();

    mTags.insert({ "blocks", { { {"blocks"}, ""}, "" } });
    mTags.insert({ "materials", { { {"materials"}, ""}, "" } });
}

void ParseModel::finalize()
{

}

std::vector<XMLGen::Model> ParseModel::data() const
{
    return mData;
}

void ParseModel::parse(std::istream &aInputFile)
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

        std::string tModelBlockID;
        if (XMLGen::parse_single_value(tTokens, { "begin", "model" }, tModelBlockID))
        {
            XMLGen::Model tModel;
            XMLGen::is_metadata_block_id_valid(tTokens);
            XMLGen::erase_tag_values(mTags);
            XMLGen::parse_input_metadata( { "end", "model" }, aInputFile, mTags);
            this->setTags(tModel);
            this->setBlockIDs(tModel);
            this->setMaterialIDs(tModel);
            tModel.id(tModelBlockID);
            this->checkTags(tModel);
            mData.push_back(tModel);
        }
    }
    this->finalize();
}

}
// namespace XMLGen
