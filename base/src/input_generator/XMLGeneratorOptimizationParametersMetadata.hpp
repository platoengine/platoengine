/*
 * XMLGeneratorOptimizationParametersMetadata.hpp
 *
 *  Created on: Jan 11, 2021
 */

#pragma once

#include <vector>
#include <string>
#include <unordered_map>

namespace XMLGen
{
    enum OptimizationType
    {
        OT_TOPOLOGY=0,
        OT_SHAPE,
        OT_UNKNOWN
    };

/******************************************************************************//**
 * \struct OptimizationParameters
 * \brief OptimizationParameters metadata for Plato problems.
**********************************************************************************/
struct OptimizationParameters
{
    struct ValueData
    {
        bool mIsDefault;
        std::string mValue;
    };

// private member data
private:
    bool mNormalizeInAggregator = false;
    bool mEnforceBounds = false;
    bool mFilterInEngine = true;
    OptimizationType mOptimizationType = OT_TOPOLOGY;
    bool mIsARestartRun = false;
    std::unordered_map<std::string, ValueData> mMetaData; /*!< OptimizationParameters metadata, map< tag, value > */
    //std::unordered_map<std::string, std::string> mMetaData; /*!< OptimizationParameters metadata, map< tag, value > */
    std::vector<std::string> mLevelsetNodesets;
    std::vector<std::string> mFixedBlockIDs;
    std::vector<std::string> mFixedSidesetIDs;
    std::vector<std::string> mFixedNodesetIDs;
    std::vector<std::string> mFixedBlockDomainValues;
    std::vector<std::string> mFixedBlockBoundaryValues;
    std::vector<std::string> mFixedBlockMaterialStates;
    std::vector<std::string> mSymmetryPlaneOrigin;
    std::vector<std::string> mSymmetryPlaneNormal;

// private member functions
private:
    /******************************************************************************//**
     * \fn getValue
     * \brief Return string value for property with input tag; else, throw an error if \n
     * property is not defined in the metadata.
     * \param [in] aTag property tag
     * \return property string value
    **********************************************************************************/
    std::string getValue(const std::string& aTag) const;

    /******************************************************************************//**
     * \fn getBool
     * \brief Return bool value for property with input tag; else, throw an error if \n
     * property is not defined in the metadata.
     * \param [in] aTag property tag
     * \return property bool value
    **********************************************************************************/
    bool getBool(const std::string& aTag) const;

public:
    /******************************************************************************//**
     * \fn normalizeInAggregator
     * \brief Return whether to normalize in the aggregator
     * \return whether to normalize in the aggregator
    **********************************************************************************/
    bool normalizeInAggregator() const { return mNormalizeInAggregator; }

    /******************************************************************************//**
     * \fn normalizeInAggregator
     * \brief Set whether to normalize in the aggregator
    **********************************************************************************/
    void normalizeInAggregator(const bool& aVal) { mNormalizeInAggregator = aVal; }

    /******************************************************************************//**
     * \fn enforceBounds
     * \brief Return whether to enforce bounds in the engine
     * \return whether to enforce bounds in the engine
    **********************************************************************************/
    bool enforceBounds() const { return mEnforceBounds; }

    /******************************************************************************//**
     * \fn enforceBounds
     * \brief Set whether to enforce bounds in engine type
    **********************************************************************************/
    void enforceBounds(const bool& aVal) { mEnforceBounds = aVal; }

    /******************************************************************************//**
     * \fn filterInEngine
     * \brief Return whether to filter in the engine
     * \return whether to filter in engine
    **********************************************************************************/
    bool filterInEngine() const { return mFilterInEngine; }

    /******************************************************************************//**
     * \fn filterInEngine
     * \brief Set whether to filter in engine type
    **********************************************************************************/
    void filterInEngine(const bool& aVal) { mFilterInEngine = aVal; }

    /******************************************************************************//**
     * \fn optimizationType
     * \brief Return the optimization type
     * \return optimization type
    **********************************************************************************/
    OptimizationType optimizationType() const { return mOptimizationType; }

    /******************************************************************************//**
     * \fn optimizationType
     * \brief Set optimization type
    **********************************************************************************/
    void optimizationType(const OptimizationType& aVal) { mOptimizationType = aVal; }

    /******************************************************************************//**
     * \fn isARestartRun
     * \brief Return whether this is a restart run or not
     * \return whether this is a restart run or not
    **********************************************************************************/
    bool isARestartRun() const { return mIsARestartRun; }

    /******************************************************************************//**
     * \fn isARestartRun
     * \brief Set whether this is a restart run or not
    **********************************************************************************/
    void isARestartRun(const bool& aVal) { mIsARestartRun = aVal; }

    /******************************************************************************//**
     * \fn symmetryNormal
     * \brief Return scenario symmetry plane normal
     * \return scenario symmetry plane normal
    **********************************************************************************/
    std::vector<std::string> symmetryNormal() const {return mSymmetryPlaneNormal;}

    /******************************************************************************//**
     * \fn symmetryOrigin
     * \brief Return scenario symmetry plane origin
     * \return scenario symmetry plane origin
    **********************************************************************************/
    std::vector<std::string> symmetryOrigin() const {return mSymmetryPlaneOrigin;}

    /******************************************************************************//**
     * \fn symmetryOrigin
     * \brief Set scenario symmetry plane origin
     * \param [in] input symmetry plane origin
    **********************************************************************************/
    void symmetryOrigin(std::vector<std::string>& aOrigin) {mSymmetryPlaneOrigin = aOrigin;}

    /******************************************************************************//**
     * \fn symmetryNormal
     * \brief Set scenario symmetry plane normal
     * \param [in] input symmetry plane normal
    **********************************************************************************/
    void symmetryNormal(std::vector<std::string>& aNormal) {mSymmetryPlaneNormal = aNormal;}

    /******************************************************************************//**
     * \fn value
     * \brief Return value for property with input tag; else, throw an error if \n
     * property is not defined in the metadata.
     * \param [in] aTag property tag
     * \return property string value
    **********************************************************************************/
    std::string value(const std::string& aTag) const;

    /******************************************************************************//**
     * \fn tags
     * \brief Return list of parameter tags.
     * \return parameter tags
    **********************************************************************************/
    std::vector<std::string> tags() const;

    /******************************************************************************//**
     * \fn append
     * \brief Append parameter to metadata.
     * \param [in] aTag   parameter tag
     * \param [in] aValue parameter value
     * \param [in] aIsDefault parameter specifying whether this is a default or user set value
    **********************************************************************************/
    void append(const std::string& aTag, const std::string& aValue, const bool& aIsDefault=false);

    /******************************************************************************//**
     * \fn mesh_map_filter_radius 
     * \brief Return string value for keyword 'mesh_map_filter_radius'.
     * \return value
    **********************************************************************************/
    std::string mesh_map_filter_radius() const;

    /******************************************************************************//**
     * \fn filter_before_symmetry_enforcement 
     * \brief Return string value for keyword 'filter_before_symmetry_enforcement'.
     * \return value
    **********************************************************************************/
    std::string filter_before_symmetry_enforcement() const;

    /******************************************************************************//**
     * \fn setFixedBlockIDs
     * \brief Set the fixed block ids
     * \param [in] aFixedBlockIDs vector of strings with fixed block ids
    **********************************************************************************/
    void setFixedBlockIDs(const std::vector<std::string> &aFixedBlockIDs) { mFixedBlockIDs = aFixedBlockIDs; }

    /******************************************************************************//**
     * \fn setFixedSidesetIDs
     * \brief Set the fixed sideset ids
     * \param [in] aFixedSidesetIDs vector of strings with fixed sideset ids
    **********************************************************************************/
    void setFixedSidesetIDs(const std::vector<std::string> &aFixedSidesetIDs) { mFixedSidesetIDs = aFixedSidesetIDs; }

    /******************************************************************************//**
     * \fn setFixedNodesetIDs
     * \brief Set the fixed nodeset ids
     * \param [in] aFixedNodesetIDs vector of strings with fixed nodeset ids
    **********************************************************************************/
    void setFixedNodesetIDs(const std::vector<std::string> &aFixedNodesetIDs) { mFixedNodesetIDs = aFixedNodesetIDs; }

    /******************************************************************************//**
     * \fn setFixedBlockDomainValues
     * \brief Set density values for the nodes associated with the fixed block domains. 
     *        assign to the domain.
     * \param [in] aFixedBlockDomainValues list of homogeneous density values 
    **********************************************************************************/
    void setFixedBlockDomainValues(const std::vector<std::string> &aFixedBlockDomainValues) { mFixedBlockDomainValues = aFixedBlockDomainValues; }

    /******************************************************************************//**
     * \fn setFixedBlockBoundaryValues
     * \brief Set density values for the nodes associated with the fixed block boundaries.
     * \param [in] aFixedBlockBoundaryValues list of homogeneous density values 
    **********************************************************************************/
    void setFixedBlockBoundaryValues(const std::vector<std::string> &aFixedBlockBoundaryValues) { mFixedBlockBoundaryValues = aFixedBlockBoundaryValues; }

    /******************************************************************************//**
     * \fn setFixedBlockMaterialStates
     * \brief Set material state (options: solid and fluid) for each fixed block. 
     * \param [in] aFixedBlockMaterialStates list of material states 
    **********************************************************************************/
    void setFixedBlockMaterialStates(const std::vector<std::string> &aFixedBlockMaterialStates) { mFixedBlockMaterialStates = aFixedBlockMaterialStates; }

    /******************************************************************************//**
     * \fn setLevelsetNodesets
     * \brief Set the levelset nodeset ids
     * \param [in] aLevelsetNodesets vector of strings with levelset nodeset ids
    **********************************************************************************/
    void setLevelsetNodesets(const std::vector<std::string> &aLevelsetNodesets) { mLevelsetNodesets = aLevelsetNodesets; }

    /******************************************************************************//**
     * \fn addFixedBlockID
     * \brief Add a fixed block id to the fixed block id list
     * \param [in] aFixedBlockID string with the fixed block id
    **********************************************************************************/
    void addFixedBlockID(const std::string &aFixedBlockID) { mFixedBlockIDs.push_back(aFixedBlockID); }

    /******************************************************************************//**
     * \fn addFixedNodesetID
     * \brief Add a fixed nodeset id to the fixed nodeset id list
     * \param [in] aFixedNodesetID string with the fixed nodeset id
    **********************************************************************************/
    void addFixedNodesetID(const std::string &aFixedNodesetID) {  mFixedNodesetIDs.push_back(aFixedNodesetID); }

    /******************************************************************************//**
     * \fn addFixedSidesetID
     * \brief Add a fixed sideset id to the fixed sideset id list
     * \param [in] aFixedSidesetID string with the fixed sideset id
    **********************************************************************************/
    void addFixedSidesetID(const std::string &aFixedSidesetID) { mFixedSidesetIDs.push_back(aFixedSidesetID); }

    /******************************************************************************//**
     * \fn addLevelsetNodeset
     * \brief Add a levelset nodeset id to the levelset nodeset id list
     * \param [in] aLevelsetNodesetID string with the levelset nodeset id
    **********************************************************************************/
    void addLevelsetNodeset(const std::string &aLevelsetNodesetID) { mLevelsetNodesets.push_back(aLevelsetNodesetID); }

    /******************************************************************************//**
     * \fn needsMeshMap 
     * \brief Return bool stating whether a mesh map is needed
     * \return value
    **********************************************************************************/
    bool needsMeshMap() const;

    // Functions to get member values
    std::string optimization_algorithm() const {return value("optimization_algorithm");} 
    std::string normalize_in_aggregator() const {return value("normalize_in_aggregator");} 
    std::string number_refines() const {return value("number_refines");}
    std::string number_prune_and_refine_processors() const {return value("number_prune_and_refine_processors");}
    std::string number_buffer_layers() const {return value("number_buffer_layers");}
    std::string prune_mesh() const {return value("prune_mesh");}
    std::string num_shape_design_variables() const {return value("num_shape_design_variables");}
    std::string initial_guess_file_name() const {return value("initial_guess_file_name");}
    std::string initial_guess_field_name() const {return value("initial_guess_field_name");}
    std::string verbose() const {return value("verbose");}
    std::string optimization_type() const {return value("optimization_type");}
    std::string check_gradient() const {return value("check_gradient");}
    std::string check_hessian() const {return value("check_hessian");}
    std::string derivative_checker_initial_superscript() const {return value("derivative_checker_initial_superscript");}
    std::string derivative_checker_final_superscript() const {return value("derivative_checker_final_superscript");}
    std::string max_iterations() const {return value("max_iterations");}
    std::string filter_in_engine() const {return value("filter_in_engine");}
    void filter_in_engine(const std::string& aValue) { append("filter_in_engine", aValue, true); }
    std::string output_method() const {return value("output_method");}
    std::string discretization() const {return value("discretization");}
    std::string initial_density_value() const {return value("initial_density_value");}
    std::string restart_iteration() const {return value("restart_iteration");}
    std::string create_levelset_spheres() const {return value("create_levelset_spheres");}
    std::string levelset_initialization_method() const {return value("levelset_initialization_method");}
    std::string levelset_material_box_min() const {return value("levelset_material_box_min");}
    std::string levelset_material_box_max() const {return value("levelset_material_box_max");}
    std::string levelset_sphere_radius() const {return value("levelset_sphere_radius");}
    std::string levelset_sphere_packing_factor() const {return value("levelset_sphere_packing_factor");}
    std::vector<std::string> levelset_nodesets() const {return mLevelsetNodesets;}
    std::vector<std::string> fixed_block_ids() const {return mFixedBlockIDs;}
    std::vector<std::string> fixed_nodeset_ids() const {return mFixedNodesetIDs;}
    std::vector<std::string> fixed_sideset_ids() const {return mFixedSidesetIDs;}
    std::vector<std::string> fixed_block_domain_values() const {return mFixedBlockDomainValues;}
    std::vector<std::string> fixed_block_boundary_values() const {return mFixedBlockBoundaryValues;}
    std::vector<std::string> fixed_block_material_states() const {return mFixedBlockMaterialStates;}
    std::string mma_move_limit() const {return value("mma_move_limit");}
    std::string mma_asymptote_expansion() const {return value("mma_asymptote_expansion");}
    std::string mma_asymptote_contraction() const {return value("mma_asymptote_contraction");}
    std::string mma_max_sub_problem_iterations() const {return value("mma_max_sub_problem_iterations");}
    std::string mma_sub_problem_initial_penalty() const {return value("mma_sub_problem_initial_penalty");}
    std::string mma_sub_problem_penalty_multiplier() const {return value("mma_sub_problem_penalty_multiplier");}
    std::string mma_sub_problem_feasibility_tolerance() const {return value("mma_sub_problem_feasibility_tolerance");}
    std::string mma_control_stagnation_tolerance() const {return value("mma_control_stagnation_tolerance");}
    std::string mma_objective_stagnation_tolerance() const {return value("mma_objective_stagnation_tolerance");}
    std::string mma_output_subproblem_diagnostics() const {return value("mma_output_subproblem_diagnostics");}
    std::string ks_trust_region_expansion_factor() const {return value("ks_trust_region_expansion_factor");}
    std::string ks_trust_region_contraction_factor() const {return value("ks_trust_region_contraction_factor");}
    std::string ks_max_trust_region_iterations() const {return value("ks_max_trust_region_iterations");}
    std::string ks_initial_radius_scale() const {return value("ks_initial_radius_scale");}
    std::string ks_max_radius_scale() const {return value("ks_max_radius_scale");}
    std::string hessian_type() const {return value("hessian_type");}
    std::string ks_min_trust_region_radius() const {return value("ks_min_trust_region_radius");}
    std::string limited_memory_storage() const {return value("limited_memory_storage");}
    std::string ks_outer_gradient_tolerance() const {return value("ks_outer_gradient_tolerance");}
    std::string ks_outer_stationarity_tolerance() const {return value("ks_outer_stationarity_tolerance");}
    std::string ks_outer_stagnation_tolerance() const {return value("ks_outer_stagnation_tolerance");}
    std::string ks_outer_control_stagnation_tolerance() const {return value("ks_outer_control_stagnation_tolerance");}
    std::string ks_outer_actual_reduction_tolerance() const {return value("ks_outer_actual_reduction_tolerance");}
    std::string problem_update_frequency() const {return value("problem_update_frequency");}
    std::string ks_disable_post_smoothing() const {return value("ks_disable_post_smoothing");}
    std::string ks_trust_region_ratio_low() const {return value("ks_trust_region_ratio_low");}
    std::string ks_trust_region_ratio_mid() const {return value("ks_trust_region_ratio_mid");}
    std::string ks_trust_region_ratio_high() const {return value("ks_trust_region_ratio_high");}
    std::string al_penalty_parameter() const {return value("al_penalty_parameter");}
    std::string al_penalty_scale_factor() const {return value("al_penalty_scale_factor");}
    std::string objective_number_standard_deviations() const {return value("objective_number_standard_deviations");}
    std::string filter_radius_scale() const {return value("filter_radius_scale");}
    std::string filter_radius_absolute() const {return value("filter_radius_absolute");}
    std::string filter_projection_start_iteration() const {return value("filter_projection_start_iteration");}
    std::string filter_projection_update_interval() const {return value("filter_projection_update_interval");}
    std::string filter_use_additive_continuation() const {return value("filter_use_additive_continuation");}
    std::string filter_power() const {return value("filter_power");}
    std::string filter_heaviside_scale() const {return value("filter_heaviside_scale");}
    std::string filter_heaviside_min() const {return value("filter_heaviside_min");}
    std::string filter_heaviside_update() const {return value("filter_heaviside_update");}
    std::string filter_heaviside_max() const {return value("filter_heaviside_max");}
    std::string filter_type() const {return value("filter_type");}
    std::string write_restart_file() const {return value("write_restart_file");}
    std::string output_frequency() const {return value("output_frequency");}
    std::string gcmma_max_inner_iterations() const {return value("gcmma_max_inner_iterations");}
    std::string gcmma_inner_kkt_tolerance() const {return value("gcmma_inner_kkt_tolerance");}
    std::string gcmma_outer_kkt_tolerance() const {return value("gcmma_outer_kkt_tolerance");}
    std::string gcmma_inner_control_stagnation_tolerance() const {return value("gcmma_inner_control_stagnation_tolerance");}
    std::string gcmma_outer_control_stagnation_tolerance() const {return value("gcmma_outer_control_stagnation_tolerance");}
    std::string gcmma_outer_objective_stagnation_tolerance() const {return value("gcmma_outer_objective_stagnation_tolerance");}
    std::string gcmma_outer_stationarity_tolerance() const {return value("gcmma_outer_stationarity_tolerance");}
    std::string gcmma_initial_moving_asymptotes_scale_factor() const {return value("gcmma_initial_moving_asymptotes_scale_factor");}
    std::string csm_file() const {return value("csm_file");}
    std::string csm_exodus_file() const {return value("csm_exodus_file");}
    std::string csm_tesselation_file() const {return value("csm_tesselation_file");}
    std::string csm_opt_file() const {return value("csm_opt_file");}
    std::string enforce_bounds() const {return value("enforce_bounds");}
    std::string oc_control_stagnation_tolerance() const {return value("oc_control_stagnation_tolerance");}
    std::string oc_objective_stagnation_tolerance() const {return value("oc_objective_stagnation_tolerance");}
    std::string oc_gradient_tolerance() const {return value("oc_gradient_tolerance");}


};
// struct OptimizationParameters

}
