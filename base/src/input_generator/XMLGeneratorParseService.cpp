/*
 * XMLGeneratorParseService.cpp
 *
 *  Created on: Jun 18, 2020
 */

#include <algorithm>

#include "XMLGeneratorParseService.hpp"
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
    this->checkPhysics(aService);
    this->checkSpatialDimensions(aService);
}

void ParseService::allocate()
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

    mTags.insert({ "newton_solver_tolerance", { { {"newton_solver_tolerance"}, ""}, "1e-8" } });
    mTags.insert({ "linear_solver_tolerance", { { {"linear_solver_tolerance"}, ""}, "1e-8" } });
    mTags.insert({ "max_number_iterations", { { {"max_number_iterations"}, ""}, "25" } });
    mTags.insert({ "convergence_criterion", { { {"convergence_criterion"}, ""}, "residual" } });
}

void ParseService::checkCode(XMLGen::Service& aService)
{
    auto tValidCode = XMLGen::check_code_keyword(aService.value("code"));
    aService.code(tValidCode);
}

void ParseService::checkSpatialDimensions(XMLGen::Service& aService)
{
    auto tDim = aService.value("dimensions");
    if (tDim.empty())
    {
        THROWERR("Parse Service: 'dimensions' keyword is empty.")
    }
    XMLGen::ValidSpatialDimsKeys tValidKeys;
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tDim);
    if (tItr == tValidKeys.mKeys.end())
    {
        THROWERR("Parse Service: Problems with " + tDim + "-D spatial dimensions are not supported.")
    }
}

void ParseService::checkPhysics(XMLGen::Service& aService)
{
    auto tPhysics = aService.value("physics");
    if (tPhysics.empty())
    {
        THROWERR("Parse Service: 'physics' keyword is empty.")
    }
    auto tValidPhysics = XMLGen::check_physics_keyword(tPhysics);
    aService.physics(tValidPhysics);
}

void ParseService::checkPerformer()
{
    for(auto& tService : mData)
    {
        if (tService.value("performer").empty())
        {
            auto tIndex = &tService - &mData[0] + 1u;
            auto tPerformer = tService.value("code") + "_" + std::to_string(tIndex);
            tService.performer(tPerformer);
        }
    }
}

void ParseService::checkServiceID()
{
    for (auto &tService : mData)
    {
        if (tService.value("id").empty())
        {
            auto tIndex = &tService - &mData[0] + 1u;
            auto tID = tService.value("code") + "_" + tService.value("physics") + "_" + std::to_string(tIndex);
            tService.id(tID);
        }
    }
}

void ParseService::finalize()
{
    this->checkPerformer();
    this->checkServiceID();
}

std::vector<XMLGen::Service> ParseService::data() const
{
    return mData;
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
            tService.id(tServiceBlockID);
            this->checkTags(tService);
            mData.push_back(tService);
        }
    }
    this->finalize();
}

}
// namespace XMLGen
