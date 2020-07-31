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

void ParseScenario::checkTags(XMLGen::Scenario& aScenario)
{
    this->checkCode(aScenario);
    this->checkPhysics(aScenario);
    this->checkSpatialDimensions(aScenario);
}

void ParseScenario::allocate()
{
    mTags.clear();
    mTags.insert({ "id", { { {"id"}, ""}, "" } });
    mTags.insert({ "code", { { {"code"}, ""}, "plato_analyze" } });
    mTags.insert({ "physics", { { {"physics"}, ""}, "" } });
    mTags.insert({ "performer", { { {"performer"}, ""}, "" } });
    mTags.insert({ "dimensions", { { {"dimensions"}, ""}, "" } });
    mTags.insert({ "cache_state", { { {"cache_state"}, ""}, "false" } });
    mTags.insert({ "analyze_new_workflow", { { {"analyze_new_workflow"}, ""}, "false" } });
    mTags.insert({ "update_problem", { { {"update_problem"}, ""}, "false" } });
    mTags.insert({ "additive_continuation", { { {"additive_continuation"}, ""}, "" } });
    mTags.insert({ "material_penalty_model", { { {"material_penalty_model"}, ""}, "simp" } });
    mTags.insert({ "material_penalty_exponent", { { {"material_penalty_exponent"}, ""}, "3.0" } });
    mTags.insert({ "minimum_ersatz_material_value", { { {"minimum_ersatz_material_value"}, ""}, "1e-9" } });
    mTags.insert({ "use_new_analyze_uq_workflow", { { {"use_new_analyze_uq_workflow"}, ""}, "false" } });

    mTags.insert({ "time_step", { { {"time_step"}, ""}, "1.0" } });
    mTags.insert({ "newmark_beta", { { {"newmark_beta"}, ""}, "0.25" } });
    mTags.insert({ "newmark_gamma", { { {"newmark_gamma"}, ""}, "0.5" } });
    mTags.insert({ "number_time_steps", { { {"number_time_steps"}, ""}, "40" } });
    mTags.insert({ "max_number_time_steps", { { {"max_number_time_steps"}, ""}, "160" } });
    mTags.insert({ "time_step_expansion_multiplier", { { {"time_step_expansion_multiplier"}, ""}, "1.25" } });

    mTags.insert({ "tolerance", { { {"tolerance"}, ""}, "1e-8" } });
    mTags.insert({ "max_number_iterations", { { {"max_number_iterations"}, ""}, "25" } });
    mTags.insert({ "convergence_criterion", { { {"convergence_criterion"}, ""}, "residual" } });
}

void ParseScenario::checkCode(XMLGen::Scenario& aScenario)
{
    auto tValidCode = XMLGen::check_code_keyword(aScenario.value("code"));
    aScenario.code(tValidCode);
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

void ParseScenario::checkPerformer()
{
    for(auto& tScenario : mData)
    {
        if (tScenario.value("performer").empty())
        {
            auto tIndex = &tScenario - &mData[0] + 1u;
            auto tPerformer = tScenario.value("code") + "_" + std::to_string(tIndex);
            tScenario.performer(tPerformer);
        }
    }
}

void ParseScenario::checkScenarioID()
{
    for (auto &tScenario : mData)
    {
        if (tScenario.value("id").empty())
        {
            auto tIndex = &tScenario - &mData[0] + 1u;
            auto tID = tScenario.value("code") + "_" + tScenario.value("physics") + "_" + std::to_string(tIndex);
            tScenario.id(tID);
        }
    }
}

void ParseScenario::finalize()
{
    this->checkPerformer();
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
