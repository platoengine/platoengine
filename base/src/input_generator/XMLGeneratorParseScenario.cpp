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
        aMetadata.setLoadIDs(tLoadIDs);
    }
    else
    {
        REPORT("Parse Scenario: loads are not defined.");
    }
}

void ParseScenario::setMatchNodesetIDs(XMLGen::Scenario &aMetadata)
{
    auto tItr = mTags.find("match_nodesets");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tNodesetIDs;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tNodesetIDs);
        aMetadata.setMatchNodesetIDs(tNodesetIDs);
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
        aMetadata.setBCIDs(tBCIDs);
    }
    else if (aMetadata.physics() != "modal_response")
    {
        THROWERR("Parse Scenario: boundary_conditions are not defined");
    }
}

void ParseScenario::checkTags(XMLGen::Scenario& aScenario)
{
    this->checkPhysics(aScenario);
    this->checkSpatialDimensions(aScenario);
    this->checkIDs(aScenario);
}

void ParseScenario::allocate()
{
    mTags.clear();
    mTags.insert({ "id", { { {"id"}, ""}, "" } });
    mTags.insert({ "physics", { { {"physics"}, ""}, "" } });
    mTags.insert({ "dimensions", { { {"dimensions"}, ""}, "" } });

    mTags.insert({ "heat_transfer", { { {"heat_transfer"}, ""}, "none" } });
    mTags.insert({ "momentum_damping", { { {"momentum_damping"}, ""}, "" } });
    mTags.insert({ "output_frequency", { { {"output_frequency"}, ""}, "1" } });
    mTags.insert({ "steady_state_tolerance", { { {"steady_state_tolerance"}, ""}, "1e-3" } });
    mTags.insert({ "max_steady_state_iterations", { { {"max_steady_state_iterations"}, ""}, "500" } });
    mTags.insert({ "thermal_source_penalty_exponent", { { {"thermal_source_penalty_exponent"}, ""}, "3" } });
    mTags.insert({ "thermal_diffusion_penalty_exponent", { { {"thermal_diffusion_penalty_exponent"}, ""}, "3" } });

    mTags.insert({ "material", { { {"material"}, ""}, "" } });
    mTags.insert({ "material_penalty_model", { { {"material_penalty_model"}, ""}, "simp" } });
    mTags.insert({ "material_penalty_exponent", { { {"material_penalty_exponent"}, ""}, "3.0" } });
    mTags.insert({ "minimum_ersatz_material_value", { { {"minimum_ersatz_material_value"}, ""}, "" } });
    mTags.insert({ "pressure_scaling", { { {"pressure_scaling"}, ""}, "1.0" } });
    mTags.insert({ "temperature_scaling", { { {"temperature_scaling"}, ""}, "1.0" } });

    mTags.insert({ "time_step", { { {"time_step"}, ""}, "1.0" } });
    mTags.insert({ "newmark_beta", { { {"newmark_beta"}, ""}, "0.25" } });
    mTags.insert({ "newmark_gamma", { { {"newmark_gamma"}, ""}, "0.5" } });
    mTags.insert({ "number_time_steps", { { {"number_time_steps"}, ""}, "40" } });
    mTags.insert({ "max_number_time_steps", { { {"max_number_time_steps"}, ""}, "160" } });
    mTags.insert({ "time_step_safety_factor", { { {"time_step_safety_factor"}, ""}, "0.7" } });
    mTags.insert({ "critical_time_step_damping", { { {"critical_time_step_damping"}, ""}, "" } });
    mTags.insert({ "time_step_expansion_multiplier", { { {"time_step_expansion_multiplier"}, ""}, "1.25" } });

    mTags.insert({ "tolerance", { { {"tolerance"}, ""}, "1e-8" } });
    mTags.insert({ "newton_solver_tolerance", { { {"newton_solver_tolerance"}, ""}, "1e-8" } });
    mTags.insert({ "max_number_iterations", { { {"max_number_iterations"}, ""}, "25" } });
    mTags.insert({ "convergence_criterion", { { {"convergence_criterion"}, ""}, "residual" } });

    mTags.insert({ "loads", { { {"loads"}, ""}, "" } });
    mTags.insert({ "boundary_conditions", { { {"boundary_conditions"}, ""}, "" } });
    mTags.insert({ "ref_data_file", { { {"ref_data_file"}, ""}, "" } });
    mTags.insert({ "weight_mass_scale_factor", { { {"weight_mass_scale_factor"}, ""}, "" } });

    mTags.insert({ "frequency_min", { { {"frequency_min"}, ""}, "" } });
    mTags.insert({ "frequency_max", { { {"frequency_max"}, ""}, "" } });
    mTags.insert({ "frequency_step", { { {"frequency_step"}, ""}, "" } });
    mTags.insert({ "raleigh_damping_alpha", { { {"raleigh_damping_alpha"}, ""}, "" } });
    mTags.insert({ "raleigh_damping_beta", { { {"raleigh_damping_beta"}, ""}, "" } });
    mTags.insert({ "match_nodesets", { { {"match_nodesets"}, ""}, "" } });
    mTags.insert({ "complex_error_measure", { { {"complex_error_measure"}, ""}, "" } });
    mTags.insert({ "convert_to_tet10", { { {"convert_to_tet10"}, ""}, "" } });

    mTags.insert({ "shape_sideset", { { {"shape_sideset"}, ""}, "" } });
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

void ParseScenario::checkIDs(XMLGen::Scenario& aScenario)
{
    auto tLoadIDs = aScenario.loadIDs();
    if (tLoadIDs.empty())
    {
        REPORT("Parse Scenario: No load IDs are defined.\n")
    }
/*
 for frf matching case there are no boundary conditions
    auto tBCIDs = aScenario.bcIDs();
    if (tBCIDs.empty())
    {
        THROWERR("Parse Scenario: No boundary condition IDs are defined")
    }
*/
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
            this->setLoadIDs(tScenario);
            this->setBCIDs(tScenario);
            this->setMatchNodesetIDs(tScenario);
            tScenario.id(tScenarioBlockID);
            this->checkTags(tScenario);
            mData.push_back(tScenario);
        }
    }
    this->finalize();
}

}
// namespace XMLGen
