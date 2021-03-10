/*
 * XMLGeneratorParseService.cpp
 *
 *  Created on: Jun 18, 2020
 */

#include <algorithm>

#include "XMLGeneratorParseServices.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

void ParseService::setTags(XMLGen::Service& aService)
{
    for(auto& tTag : mTags)
    {
        if(tTag.second.first.second.empty())
        {
            auto tDefaultValue = tTag.second.second;
            aService.append(tTag.first, tDefaultValue);
        }
        else
        {
            auto tInputValue = tTag.second.first.second;
            aService.append(tTag.first, tInputValue);
        }
    }
}

void ParseService::checkTags(XMLGen::Service& aService)
{
    this->checkCode(aService);
    this->checkCacheState(aService);
}

void ParseService::allocate()
{
    mTags.clear();
    mTags.insert({ "id", { { {"id"}, ""}, "" } });
    mTags.insert({ "code", { { {"code"}, ""}, "" } });
    mTags.insert({ "cache_state", { { {"cache_state"}, ""}, "false" } });
    mTags.insert({ "update_problem", { { {"update_problem"}, ""}, "false" } });
    mTags.insert({ "additive_continuation", { { {"additive_continuation"}, ""}, "false" } }); //this should be in the optimizer block

    mTags.insert({ "number_processors", { { {"number_processors"}, ""}, "1" } });
    mTags.insert({ "number_ranks", { { {"number_ranks"}, ""}, "1" } });
    mTags.insert({ "device_ids", { { {"device_ids"}, ""}, "" } });
}

void ParseService::checkCacheState(XMLGen::Service& aService)
{
    // Sierra SD has to have cache state set to true and we don't want to make 
    // the user have to set this.
    if(aService.value("code") == "sierra_sd")
    {
        aService.cacheState("true");
    }
}

void ParseService::checkCode(XMLGen::Service& aService)
{
    auto tValidCode = XMLGen::check_code_keyword(aService.value("code"));
    aService.code(tValidCode);
}

void ParseService::checkServiceID()
{
    for (auto &tService : mData)
    {
        if (tService.value("id").empty())
        {
            auto tIndex = &tService - &mData[0] + 1u;
            auto tID = tService.value("code") + "_" + std::to_string(tIndex);
            tService.id(tID);
        }
    }
}

void ParseService::finalize()
{
    this->checkServiceID();
}

std::vector<XMLGen::Service> ParseService::data() const
{
    return mData;
}

void ParseService::setDeviceIDs(XMLGen::Service &aMetadata)
{
    auto tItr = mTags.find("device_ids");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tDeviceIDs;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tDeviceIDs);
        aMetadata.deviceIDs(tDeviceIDs);
    }
}

void ParseService::parse(std::istream &aInputFile)
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

        std::string tServiceBlockID;
        if (XMLGen::parse_single_value(tTokens, { "begin", "service" }, tServiceBlockID))
        {
            XMLGen::Service tService;
            XMLGen::is_metadata_block_id_valid(tTokens);
            XMLGen::erase_tag_values(mTags);
            XMLGen::parse_input_metadata( { "end", "service" }, aInputFile, mTags);
            this->setTags(tService);
            this->setDeviceIDs(tService);
            tService.id(tServiceBlockID);
            this->checkTags(tService);
            mData.push_back(tService);
        }
    }
    this->finalize();
}

}
// namespace XMLGen
