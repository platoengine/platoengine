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

void ParseService::setLoadIDs(XMLGen::Service &aMetadata)
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

void ParseService::setBCIDs(XMLGen::Service &aMetadata)
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
// /******************************************************************************/
// bool XMLGenerator::fillObjectiveAndPerfomerNames()
// /******************************************************************************/
// {
//     // assigns objective names to yet un-named objectives
//     // assigns performer name to each objective

//     char buf2[200];
//     size_t num_objs = m_InputData.objectives.size();
//     // If there were objectives without names add a default name
//     for(size_t i=0; i<num_objs; ++i)
//     {
//         // For each code name we will make sure there are names set
//         std::string cur_code_name = m_InputData.objectives[i].code_name;
//         int num_cur_code_objs=0;
//         for(size_t j=i; j<num_objs; ++j)
//         {
//             if(!m_InputData.objectives[j].code_name.compare(cur_code_name))
//             {
//                 num_cur_code_objs++;
//                 if(m_InputData.objectives[j].name.empty())
//                 {
//                     sprintf(buf2, "%d", num_cur_code_objs);
//                     m_InputData.objectives[j].name = buf2;
//                 }
//             }
//         }
//     }
//     // Set the performer names
//     for(size_t i=0; i<num_objs; ++i)
//     {
//         m_InputData.objectives[i].mPerformerName =
//                 m_InputData.objectives[i].code_name +
//                 "_" + m_InputData.objectives[i].name;
//     }

//     return true;
// }


// void ParseObjective::setIdentification()
// {
//     for (auto &tOuterObjective : mData)
//     {
//         // For each code name we will make sure there are names set
//         auto tMyCodeName = tOuterObjective.code_name;
//         size_t tObjectiveIdentificationNumber = 0;
//         for (auto &tInnerObjective : mData)
//         {
//             if (!tInnerObjective.code_name.compare(tMyCodeName))
//             {
//                 tObjectiveIdentificationNumber++;
//                 if (tInnerObjective.name.empty())
//                 {
//                     tInnerObjective.name = std::to_string(tObjectiveIdentificationNumber);
//                 }
//             }
//         }
//     }
// }

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

    mTags.insert({ "tolerance", { { {"tolerance"}, ""}, "1e-8" } });
    mTags.insert({ "max_number_iterations", { { {"max_number_iterations"}, ""}, "25" } });
    mTags.insert({ "convergence_criterion", { { {"convergence_criterion"}, ""}, "residual" } });

    mTags.insert({ "number_processors", { { {"number_processors"}, ""}, "" } });
    mTags.insert({ "number_ranks", { { {"number_ranks"}, ""}, "" } });

    mTags.insert({ "loads", { { {"loads"}, ""}, "" } });
    mTags.insert({ "boundary_conditions", { { {"boundary_conditions"}, ""}, "" } });

    //frf matching
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
