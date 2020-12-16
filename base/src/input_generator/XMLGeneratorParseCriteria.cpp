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
    XMLGen::ValidCriterionParameterKeys tValidKeys;
    for(auto& tTag : mTags)
    {
        auto tLowerKey = XMLGen::to_lower(tTag.first);
        auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerKey);
        if(tItr == tValidKeys.mKeys.end())
        {
            THROWERR(std::string("Check Keyword: keyword '") + tTag.first + std::string("' is not supported"))
        }

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
    auto tValue = tValidKeys.value(aKeyword);
    if(tValue.empty())
    {
        THROWERR(std::string("Check Criterion type Key: Criterion type '") + tValue + "' is not supported.")
    }
    return tValue;
}

void ParseCriteria::allocate()
{
    mTags.clear();

    // core properties
    mTags.insert({ "type", { { {"type"}, ""}, "" } });
    mTags.insert({ "normalize", { { {"normalize"}, ""}, "false" } });
    mTags.insert({ "normalization_value", { { {"normalization_value"}, ""}, "1.0" } });
    mTags.insert({ "stress_limit", { { {"stress_limit"}, ""}, "" } });
    mTags.insert({ "scmm_initial_penalty", { { {"sccmm_initial_penalty"}, ""}, "" } });
    mTags.insert({ "scmm_penalty_expansion_multiplier", { { {"scmm_penalty_expansion_multiplier"}, ""}, "" } });
    mTags.insert({ "scmm_penalty_upper_bound", { { {"scmm_penalty_upper_bound"}, ""}, "" } });
    mTags.insert({ "scmm_stress_weight", { { {"scmm_stress_weight"}, ""}, "1.0" } });
    mTags.insert({ "scmm_mass_weight", { { {"scmm_mass_weight"}, ""}, "1.0" } });

    mTags.insert({ "stress_p_norm_exponent", { { {"stress_p_norm_exponent"}, ""}, "6.0" } });
    mTags.insert({ "material_penalty_model", { { {"material_penalty_model"}, ""}, "simp" } });
    mTags.insert({ "material_penalty_exponent", { { {"material_penalty_exponent"}, ""}, "3.0" } });
    mTags.insert({ "minimum_ersatz_material_value", { { {"minimum_ersatz_material_value"}, ""}, "1e-9" } });
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

void ParseCriteria::setMetadata(XMLGen::Criterion& aMetadata)
{
    this->setCriterionType(aMetadata);
    this->setTags(aMetadata);
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
