/*
 * XMLGeneratorParseScenario.cpp
 *
 *  Created on: Jun 18, 2020
 */

#include <algorithm>

#include "XMLGeneratorParseScenario.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

void ParseScenario::setTags(XMLGen::Scenario& aScenario)
{
    for(auto& tTag : mTags)
    {
        if(tTag.second.first.second.empty())
        {
            auto tDefaultValue = tTag.second.second;
            aScenario.append(tTag.first, tDefaultValue);
        }
        else
        {
            auto tInputValue = tTag.second.first.second;
            aScenario.append(tTag.first, tInputValue);
        }
    }
}

void ParseScenario::setLoadIDs(XMLGen::Scenario &aMetadata)
{
    auto tItr = mTags.find("loads");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tLoadIDs;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tLoadIDs);
        aMetadata.LoadIDs = tLoadIDs;
    }
    else
    {
        THROWERR("Objective criteria are not defined");
    }
}

void ParseScenario::setBCIDs(XMLGen::Scenario &aMetadata)
{
    auto tItr = mTags.find("boundary_conditions");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tBCIDs;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tBCIDs);
        aMetadata.LoadIDs = tBCIDs;
    }
    else
    {
        THROWERR("Objective criteria are not defined");
    }
}

void ParseScenario::checkTags(XMLGen::Scenario& aScenario)
{
    this->checkPhysics(aScenario);
    this->checkSpatialDimensions(aScenario);
}

void ParseScenario::allocate()
{
    mTags.clear();
    mTags.insert({ "id", { { {"id"}, ""}, "" } });
    mTags.insert({ "physics", { { {"physics"}, ""}, "" } });
    mTags.insert({ "dimensions", { { {"dimensions"}, ""}, "" } });
    mTags.insert({ "material_penalty_model", { { {"material_penalty_model"}, ""}, "simp" } });
    mTags.insert({ "material_penalty_exponent", { { {"material_penalty_exponent"}, ""}, "3.0" } });
    mTags.insert({ "minimum_ersatz_material_value", { { {"minimum_ersatz_material_value"}, ""}, "1e-9" } });

    mTags.insert({ "loads", { { {"loads"}, ""}, "" } });
    mTags.insert({ "boundary_conditions", { { {"boundary_conditions"}, ""}, "" } });

    //frf matching, some of these probably should belong as criterion parameters instead
    //
    // mTags.insert({ "complex_error_measure", { { {"complex_error_measure"}, ""}, "" } });
    // mTags.insert({ "convert_to_tet10", { { {"convert_to_tet10"}, ""}, "" } });
    // mTags.insert({ "frf_match_nodesets", { { {"frf_match_nodesets"}, ""}, "" } });
    // mTags.insert({ "freq_min", { { {"freq_min"}, ""}, "" } });
    // mTags.insert({ "freq_max", { { {"freq_max"}, ""}, "" } });
    // mTags.insert({ "freq_step", { { {"freq_step"}, ""}, "" } });
    // mTags.insert({ "ref_frf_file", { { {"ref_frf_file"}, ""}, "" } });
    // mTags.insert({ "raleigh_damping_alpha", { { {"raleigh_damping_alpha"}, ""}, "" } });
    // mTags.insert({ "raleigh_damping_beta", { { {"raleigh_damping_beta"}, ""}, "" } });
    // mTags.insert({ "wtmass_scale_factor", { { {"wtmass_scale_factor"}, ""}, "" } });
    // mTags.insert({ "normalize_objective", { { {"normalize_objective"}, ""}, "" } });
}

void ParseScenario::checkSpatialDimensions(XMLGen::Scenario& aScenario)
{
    auto tDim = aScenario.value("dimensions");
    if (tDim.empty())
    {
        THROWERR("Parse Scenario: 'dimensions' keyword is empty.")
    }
    XMLGen::ValidSpatialDimsKeys tValidKeys;
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tDim);
    if (tItr == tValidKeys.mKeys.end())
    {
        THROWERR("Parse Scenario: Problems with " + tDim + "-D spatial dimensions are not supported.")
    }
}

void ParseScenario::checkPhysics(XMLGen::Scenario& aScenario)
{
    auto tPhysics = aScenario.value("physics");
    if (tPhysics.empty())
    {
        THROWERR("Parse Scenario: 'physics' keyword is empty.")
    }
    auto tValidPhysics = XMLGen::check_physics_keyword(tPhysics);
    aScenario.physics(tValidPhysics);
}

void ParseScenario::checkScenarioID()
{
    for (auto &tScenario : mData)
    {
        if (tScenario.value("id").empty())
        {
            auto tIndex = &tScenario - &mData[0] + 1u;
            auto tID = tScenario.value("physics") + "_" + std::to_string(tIndex);
            tScenario.id(tID);
        }
    }
}

void ParseScenario::finalize()
{
    this->checkScenarioID();
}

std::vector<XMLGen::Scenario> ParseScenario::data() const
{
    return mData;
}

void ParseScenario::parse(std::istream &aInputFile)
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

        std::string tScenarioBlockID;
        if (XMLGen::parse_single_value(tTokens, { "begin", "scenario" }, tScenarioBlockID))
        {
            XMLGen::Scenario tScenario;
            XMLGen::is_metadata_block_id_valid(tTokens);
            XMLGen::erase_tag_values(mTags);
            XMLGen::parse_input_metadata( { "end", "scenario" }, aInputFile, mTags);
            this->setTags(tScenario);
            tScenario.id(tScenarioBlockID);
            this->checkTags(tScenario);
            mData.push_back(tScenario);
        }
    }
    this->finalize();
}

}
// namespace XMLGen
