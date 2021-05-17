/*
 * XMLGeneratorParseOptimizationParameters.cpp
 *
 *  Created on: Jan 11, 2021
 */

#include <algorithm>

#include "XMLGeneratorParseOptimizationParameters.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParserUtilities.hpp"

namespace XMLGen
{

void ParseOptimizationParameters::setTags(XMLGen::OptimizationParameters &aOptimizationParameters)
{
    ValidOptimizationParameterKeys tValidKeys;
    for(auto& tTag : mTags)
    {
        if(tValidKeys.mKeys.find(tTag.first) == tValidKeys.mKeys.end())
        {
            THROWERR("Parse Optimization Parameters: Invalid keyword: " + tTag.first);
        }
        if(tTag.second.first.second.empty())
        {
            auto tDefaultValue = tTag.second.second;
            aOptimizationParameters.append(tTag.first, tDefaultValue);
        }
        else
        {
            auto tInputValue = tTag.second.first.second;
            aOptimizationParameters.append(tTag.first, tInputValue);
        }
    }
}

void ParseOptimizationParameters::allocate()
{
    mTags.clear();
    mTags.insert({ "discretization", { { {"discretization"}, ""}, "density" } });
    mTags.insert({ "verbose", { { {"verbose"}, ""}, "false" } });
    mTags.insert({ "enforce_bounds", { { {"enforce_bounds"}, ""}, "" } });
    mTags.insert({ "number_refines", { { {"number_refines"}, ""}, "" } });
    mTags.insert({ "csm_file", { { {"csm_file"}, ""}, "" } });
    mTags.insert({ "num_shape_design_variables", { { {"num_shape_design_variables"}, ""}, "" } });
    mTags.insert({ "mma_move_limit", { { {"mma_move_limit"}, ""}, "" } });
    mTags.insert({ "max_iterations", { { {"max_iterations"}, ""}, "" } });
    mTags.insert({ "filter_in_engine", { { {"filter_in_engine"}, ""}, "true" } });
    mTags.insert({ "symmetry_plane_normal", { { {"symmetry_plane_normal"}, ""}, "" } });
    mTags.insert({ "symmetry_plane_origin", { { {"symmetry_plane_origin"}, ""}, "" } });
    mTags.insert({ "mesh_map_filter_radius", { { {"mesh_map_filter_radius"}, ""}, "" } });
    mTags.insert({ "filter_before_symmetry_enforcement", { { {"filter_before_symmetry_enforcement"}, ""}, "" } });
    mTags.insert({ "mma_asymptote_expansion", { { {"mma_asymptote_expansion"}, ""}, "" } });
    mTags.insert({ "mma_asymptote_contraction", { { {"mma_asymptote_contraction"}, ""}, "" } });
    mTags.insert({ "mma_max_sub_problem_iterations", { { {"mma_max_sub_problem_iterations"}, ""}, "" } });
    mTags.insert({ "mma_control_stagnation_tolerance", { { {"mma_control_stagnation_tolerance"}, ""}, "" } });
    mTags.insert({ "mma_objective_stagnation_tolerance", { { {"mma_objective_stagnation_tolerance"}, ""}, "" } });
    mTags.insert({ "oc_control_stagnation_tolerance", { { {"oc_control_stagnation_tolerance"}, ""}, "" } });
    mTags.insert({ "oc_objective_stagnation_tolerance", { { {"oc_objective_stagnation_tolerance"}, ""}, "" } });
    mTags.insert({ "oc_gradient_tolerance", { { {"oc_gradient_tolerance"}, ""}, "" } });
    mTags.insert({ "initial_guess_file_name", { { {"initial_guess_file_name"}, ""}, "" } });
    mTags.insert({ "initial_guess_field_name", { { {"initial_guess_field_name"}, ""}, "" } });
    mTags.insert({ "normalize_in_aggregator", { { {"normalize_in_aggregator"}, ""}, "" } });
    mTags.insert({ "derivative_checker_final_superscript", { { {"derivative_checker_final_superscript"}, ""}, "8" } });
    mTags.insert({ "derivative_checker_initial_superscript", { { {"derivative_checker_initial_superscript"}, ""}, "1" } });
    mTags.insert({ "output_method", { { {"output_method"}, ""}, "epu" } });
    mTags.insert({ "output_frequency", { { {"output_frequency"}, ""}, "5" } });
    mTags.insert({ "initial_density_value", { { {"initial_density_value"}, ""}, "0.5" } });
    mTags.insert({ "restart_iteration", { { {"restart_iteration"}, ""}, "" } });
    mTags.insert({ "create_levelset_spheres", { { {"create_levelset_spheres"}, ""}, "" } });
    mTags.insert({ "levelset_material_box_min", { { {"levelset_material_box_min"}, ""}, "" } });
    mTags.insert({ "levelset_material_box_max", { { {"levelset_material_box_max"}, ""}, "" } });
    mTags.insert({ "levelset_sphere_radius", { { {"levelset_sphere_radius"}, ""}, "" } });
    mTags.insert({ "levelset_sphere_packing_factor", { { {"levelset_sphere_packing_factor"}, ""}, "" } });
    mTags.insert({ "levelset_initialization_method", { { {"levelset_initialization_method"}, ""}, "" } });
    mTags.insert({ "fixed_block_ids", { { {"fixed_block_ids"}, ""}, "" } });
    mTags.insert({ "fixed_sideset_ids", { { {"fixed_sideset_ids"}, ""}, "" } });
    mTags.insert({ "fixed_nodeset_ids", { { {"fixed_nodeset_ids"}, ""}, "" } });
    mTags.insert({ "levelset_nodesets", { { {"levelset_nodesets"}, ""}, "" } });
    mTags.insert({ "number_prune_and_refine_processors", { { {"number_prune_and_refine_processors"}, ""}, "" } });
    mTags.insert({ "number_buffer_layers", { { {"number_buffer_layers"}, ""}, "" } });
    mTags.insert({ "prune_mesh", { { {"prune_mesh"}, ""}, "" } });
    mTags.insert({ "optimization_algorithm", { { {"optimization_algorithm"}, ""}, "oc" } });
    mTags.insert({ "check_gradient", { { {"check_gradient"}, ""}, "false" } });
    mTags.insert({ "check_hessian", { { {"check_hessian"}, ""}, "false" } });
    mTags.insert({ "filter_type", { { {"filter_type"}, ""}, "kernel" } });
    mTags.insert({ "filter_power", { { {"filter_power"}, ""}, "1" } });
    mTags.insert({ "gcmma_inner_kkt_tolerance", { { {"gcmma_inner_kkt_tolerance"}, ""}, "" } });
    mTags.insert({ "gcmma_outer_kkt_tolerance", { { {"gcmma_outer_kkt_tolerance"}, ""}, "" } });
    mTags.insert({ "gcmma_inner_control_stagnation_tolerance", { { {"gcmma_inner_control_stagnation_tolerance"}, ""}, "" } });
    mTags.insert({ "gcmma_outer_control_stagnation_tolerance", { { {"gcmma_outer_control_stagnation_tolerance"}, ""}, "" } });
    mTags.insert({ "gcmma_outer_objective_stagnation_tolerance", { { {"gcmma_outer_objective_stagnation_tolerance"}, ""}, "" } });
    mTags.insert({ "gcmma_max_inner_iterations", { { {"gcmma_max_inner_iterations"}, ""}, "" } });
    mTags.insert({ "gcmma_outer_stationarity_tolerance", { { {"gcmma_outer_stationarity_tolerance"}, ""}, "" } });
    mTags.insert({ "gcmma_initial_moving_asymptotes_scale_factor", { { {"gcmma_initial_moving_asymptotes_scale_factor"}, ""}, "" } });
    mTags.insert({ "ks_max_radius_scale", { { {"ks_max_radius_scale"}, ""}, "" } });
    mTags.insert({ "ks_initial_radius_scale", { { {"ks_initial_radius_scale"}, ""}, "" } });
    mTags.insert({ "max_trust_region_radius", { { {"max_trust_region_radius"}, ""}, "" } });
    mTags.insert({ "ks_min_trust_region_radius", { { {"ks_min_trust_region_radius"}, ""}, "" } });
    mTags.insert({ "ks_max_trust_region_iterations", { { {"ks_max_trust_region_iterations"}, ""}, "5" } });
    mTags.insert({ "ks_trust_region_expansion_factor", { { {"ks_trust_region_expansion_factor"}, ""}, "" } });
    mTags.insert({ "ks_trust_region_contraction_factor", { { {"ks_trust_region_contraction_factor"}, ""}, "" } });
    mTags.insert({ "ks_trust_region_ratio_low", { { {"ks_trust_region_ratio_low"}, ""}, "" } });
    mTags.insert({ "ks_trust_region_ratio_mid", { { {"ks_trust_region_ratio_mid"}, ""}, "" } });
    mTags.insert({ "ks_trust_region_ratio_high", { { {"ks_trust_region_ratio_high"}, ""}, "" } });
    mTags.insert({ "ks_disable_post_smoothing", { { {"ks_disable_post_smoothing"}, ""}, "true" } });
    mTags.insert({ "use_mean_norm", { { {"use_mean_norm"}, ""}, "" } });
    mTags.insert({ "objective_number_standard_deviations", { { {"objective_number_standard_deviations"}, ""}, "" } });
    mTags.insert({ "filter_radius_scale", { { {"filter_radius_scale"}, ""}, "2.0" } });
    mTags.insert({ "filter_radius_absolute", { { {"filter_radius_absolute"}, ""}, "" } });
    mTags.insert({ "al_penalty_parameter", { { {"al_penalty_parameter"}, ""}, "" } });
    mTags.insert({ "feasibility_tolerance", { { {"feasibility_tolerance"}, ""}, "" } });
    mTags.insert({ "al_penalty_scale_factor", { { {"al_penalty_scale_factor"}, ""}, "" } });
    mTags.insert({ "al_max_subproblem_iterations", { { {"al_max_subproblem_iterations"}, ""}, "" } });
    mTags.insert({ "hessian_type", { { {"hessian_type"}, ""}, "" } });
    mTags.insert({ "limited_memory_storage", { { {"limited_memory_storage"}, ""}, "8" } });
    mTags.insert({ "problem_update_frequency", { { {"problem_update_frequency"}, ""}, "5" } });
    mTags.insert({ "ks_outer_gradient_tolerance", { { {"ks_outer_gradient_tolerance"}, ""}, "" } });
    mTags.insert({ "ks_outer_stationarity_tolerance", { { {"ks_outer_stationarity_tolerance"}, ""}, "" } });
    mTags.insert({ "ks_outer_stagnation_tolerance", { { {"ks_outer_stagnation_tolerance"}, ""}, "" } });
    mTags.insert({ "ks_outer_control_stagnation_tolerance", { { {"ks_outer_control_stagnation_tolerance"}, ""}, "" } });
    mTags.insert({ "ks_outer_actual_reduction_tolerance", { { {"ks_outer_actual_reduction_tolerance"}, ""}, "" } });
    mTags.insert({ "filter_heaviside_min", { { {"filter_heaviside_min"}, ""}, "" } });
    mTags.insert({ "filter_heaviside_scale", { { {"filter_heaviside_scale"}, ""}, "" } });
    mTags.insert({ "filter_heaviside_update", { { {"filter_heaviside_update"}, ""}, "" } });
    mTags.insert({ "filter_heaviside_max", { { {"filter_heaviside_max"}, ""}, "" } });
    mTags.insert({ "filter_projection_start_iteration", { { {"filter_projection_start_iteration"}, ""}, "" } });
    mTags.insert({ "filter_projection_update_interval", { { {"filter_projection_update_interval"}, ""}, "" } });
    mTags.insert({ "filter_use_additive_continuation", { { {"filter_use_additive_continuation"}, ""}, "" } });
    mTags.insert({ "write_restart_file", { { {"write_restart_file"}, ""}, "False" } });
    mTags.insert({ "optimization_type", { { {"optimization_type"}, ""}, "topology" } });
    mTags.insert({ "filter_type_identity_generator_name", { { {"filter_type_identity_generator_name"}, ""}, "identity" } });
    mTags.insert({ "filter_type_kernel_generator_name", { { {"filter_type_kernel_generator_name"}, ""}, "kernel" } });
    mTags.insert({ "filter_type_kernel_then_heaviside_generator_name", { { {"filter_type_kernel_then_heaviside_generator_name"}, ""}, "kernel_then_heaviside" } });
    mTags.insert({ "filter_type_kernel_then_tanh_generator_name", { { {"filter_type_kernel_then_tanh_generator_name"}, ""}, "kernel_then_tanh" } });
}

std::vector<XMLGen::OptimizationParameters> ParseOptimizationParameters::data() const 
{
    return mData;
}

void ParseOptimizationParameters::setMeshMapData(XMLGen::OptimizationParameters &aMetadata)
{
    auto tItr = mTags.find("symmetry_plane_origin");
    if(tItr != mTags.end())
    {
        std::string tValues = tItr->second.first.second;
        if (!tValues.empty())
        {
            std::vector<std::string> tOrigin;
            char tValuesBuffer[10000];
            strcpy(tValuesBuffer, tValues.c_str());
            XMLGen::parse_tokens(tValuesBuffer, tOrigin);
            aMetadata.symmetryOrigin(tOrigin);
        }
    }
    tItr = mTags.find("symmetry_plane_normal");
    if(tItr != mTags.end())
    {
        std::string tValues = tItr->second.first.second;
        if (!tValues.empty())
        {
            std::vector<std::string> tNormal;
            char tValuesBuffer[10000];
            strcpy(tValuesBuffer, tValues.c_str());
            XMLGen::parse_tokens(tValuesBuffer, tNormal);
            aMetadata.symmetryNormal(tNormal);
        }
    }
}

void ParseOptimizationParameters::parse(std::istream &aInputFile)
{
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

        std::string tDummy;
        if (XMLGen::parse_single_value(tTokens, { "begin", "optimization_parameters" }, tDummy))
        {
            XMLGen::OptimizationParameters tOptimizationParameters;
            XMLGen::is_metadata_block_id_valid(tTokens);
            XMLGen::erase_tag_values(mTags);
            XMLGen::parse_input_metadata( { "end", "optimization_parameters" }, aInputFile, mTags);
            this->setTags(tOptimizationParameters);
            this->setMetaData(tOptimizationParameters);
            mData.push_back(tOptimizationParameters);
        }
    }
}

void ParseOptimizationParameters::autoFillRestartParameters(XMLGen::OptimizationParameters &aMetadata)
{
    // If there is a restart iteration but no filename specified then we will
    // assume we are coming from the gui and we will use the default "restart_XXX.exo"
    // filename, iteration 1 from the file, and field name "optimizationdofs".
    if(aMetadata.restart_iteration() != "" &&
       aMetadata.restart_iteration() != "0")
    {
        if(aMetadata.initial_guess_file_name() == "" &&
           aMetadata.initial_guess_field_name() == "")
        {
            // This block indicates that we are coming from the gui so only the
            // restart iteration was specified.  We will fill in the other values
            // based on what we know the gui will be providing for the run.
            aMetadata.append("initial_guess_file_name",  "restart_" + aMetadata.restart_iteration() + ".exo");
            aMetadata.append("restart_iteration", "1");
            aMetadata.append("initial_guess_field_name", "control");
        }
        else
        {
            // This block indicates that the user is manually setting up the
            // restart file and so we depend on him having specified a filename
            // and field name.  If either of these is empty we need to error out.
            if(aMetadata.initial_guess_field_name() == "" ||
               aMetadata.initial_guess_file_name() == "")
            {
                THROWERR("ERROR:XMLGenerator:parseOptimizationParameters: You must specify a valid initial guess mesh filename and a valid field name on that mesh from which initial values will be obtained.\n");
            }
        }
    }
}

void ParseOptimizationParameters::setMetaData(XMLGen::OptimizationParameters &aMetadata)
{
    this->setFixedBlockIDs(aMetadata);
    this->setFixedSidesetIDs(aMetadata);
    this->setFixedNodesetIDs(aMetadata);
    this->setLevelsetNodesetIDs(aMetadata);
    this->setMaterialBoxExtents(aMetadata);
    this->checkHeavisideFilterParams(aMetadata);
    this->setMeshMapData(aMetadata);
    this->setCSMParameters(aMetadata);
    this->autoFillRestartParameters(aMetadata);
}

void ParseOptimizationParameters::checkHeavisideFilterParams(XMLGen::OptimizationParameters &aMetadata)
{
    auto tMinString = aMetadata.filter_heaviside_min();
    auto tMaxString = aMetadata.filter_heaviside_max();
    auto tScaleString = aMetadata.filter_heaviside_scale();
    if(!tScaleString.empty() && (!tMinString.empty() || !tMaxString.empty()))
    {
        THROWERR("Parse Optimization Parameters: You can't specify filter heaviside scale along with filter heaviside min or max.");
    }
    if(!tScaleString.empty() && tMinString.empty() && tMaxString.empty())
    {
        aMetadata.append("filter_heaviside_min", tScaleString);
        aMetadata.append("filter_heaviside_max", tScaleString);
    }
}

void ParseOptimizationParameters::setMaterialBoxExtents(XMLGen::OptimizationParameters &aMetadata)
{
    mTags.insert({ "levelset_material_box_min", { { {"levelset_material_box_min"}, ""}, "" } });
    auto tMinItr = mTags.find("levelset_material_box_min");
    auto tMaxItr = mTags.find("levelset_material_box_max");
    if(tMinItr != mTags.end() && tMaxItr != mTags.end())
    {
        auto tMinValString = tMinItr->second.first.second.empty() ? tMinItr->second.second : tMinItr->second.first.second;
        auto tMaxValString = tMaxItr->second.first.second.empty() ? tMaxItr->second.second : tMaxItr->second.first.second;
        if(tMinValString.empty() && tMaxValString.empty()) 
        {
            return;
        }
        if(tMinValString.empty() || tMaxValString.empty())
        {
            THROWERR("Parse Optimization Parameters: Levelset material box min or max is not defined.");
        }
        std::vector<std::string> tMinValues;
        std::vector<std::string> tMaxValues;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tMinValString.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tMinValues);
        strcpy(tValuesBuffer, tMaxValString.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tMaxValues);
        if(tMinValues.size() != 3 || tMaxValues.size() != 3)
        {
            THROWERR("Parse Optimization Parameters: Levelset material box min or max is not fully defined.");
        }
    }
}

void ParseOptimizationParameters::setFixedNodesetIDs(XMLGen::OptimizationParameters &aMetadata)
{
    auto tItr = mTags.find("fixed_nodeset_ids");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tFixedIDs;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tFixedIDs);
        aMetadata.setFixedNodesetIDs(tFixedIDs);
    }
}

void ParseOptimizationParameters::setLevelsetNodesetIDs(XMLGen::OptimizationParameters &aMetadata)
{
    auto tItr = mTags.find("levelset_nodesets");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tIDs;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tIDs);
        aMetadata.setLevelsetNodesets(tIDs);
    }
}

void ParseOptimizationParameters::setFixedSidesetIDs(XMLGen::OptimizationParameters &aMetadata)
{
    auto tItr = mTags.find("fixed_sideset_ids");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tFixedIDs;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tFixedIDs);
        aMetadata.setFixedSidesetIDs(tFixedIDs);
    }
}

void ParseOptimizationParameters::setFixedBlockIDs(XMLGen::OptimizationParameters &aMetadata)
{
    auto tItr = mTags.find("fixed_block_ids");
    std::string tValues = tItr->second.first.second;
    if (tItr != mTags.end() && !tValues.empty())
    {
        std::vector<std::string> tFixedBlockIDs;
        char tValuesBuffer[10000];
        strcpy(tValuesBuffer, tValues.c_str());
        XMLGen::parse_tokens(tValuesBuffer, tFixedBlockIDs);
        aMetadata.setFixedBlockIDs(tFixedBlockIDs);
    }
}

void ParseOptimizationParameters::setCSMParameters(XMLGen::OptimizationParameters &aMetadata)
{
    auto tItr = mTags.find("csm_file");
    std::string tValue = tItr->second.first.second;
    if (tItr != mTags.end() && !tValue.empty())
    {
        size_t tPos = tValue.rfind(".csm");
        if(tPos != std::string::npos)
        {
            std::string tBaseName = tValue.substr(0, tPos);
            std::string tTessName = tBaseName + ".eto";
            std::string tExoName = tBaseName + ".exo";
            std::string tOptName = tBaseName + "_opt.csm";
            aMetadata.append("csm_opt_file", tOptName);
            aMetadata.append("csm_tesselation_file", tTessName);
            aMetadata.append("csm_exodus_file", tExoName);
        }
    }
}


}
// namespace XMLGen
