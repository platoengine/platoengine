/*
 * XMLGeneratorDataStruct.hpp
 *
 *  Created on: Jun 1, 2019
 */

#pragma once

#include <string>
#include <vector>

namespace XMLGen
{

struct Uncertainty
{
    std::string variable_type; // Load or material
    std::string type; // currently always "angle variation"
    std::string id; // which random variable to vary
    std::string axis; // x, y, z
    std::string distribution; // normal, uniform, beta
    std::string mean; // scalar value
    std::string upper; // scalar value
    std::string lower; // scalar value
    std::string standard_deviation; // scalar value
    std::string num_samples; // integer value
};

struct Load
{
    std::string type; // traction, heat flux, force, pressure ...
    std::string app_type; // nodeset or sideset
    std::string app_id; // nodeset/sideset id
    std::string app_name; // nodeset/sideset name
    std::vector<std::string> values;
    std::string dof;  // x, y, or z
    std::string load_id;
};

struct LoadCase
{
    std::vector<XMLGen::Load> loads;
    std::string id; // note LoadCase id is not necessarily shared with its Loads
};

struct BC
{
    std::string type;     // temperature, displacement
    std::string app_type; // nodeset or sideset
    std::string app_id; // nodeset/sideset id
    std::string app_name; // nodeset/sideset name
    std::string dof;
    std::string bc_id;
    std::string value;
};

struct Objective
{
    std::string name;
    std::string type;
    std::string code_name;
    std::string performer_name;
    std::string weight;
    std::string num_procs;
    std::string analysis_solver_tolerance;
    std::string multi_load_case;
    std::vector<std::string> output_for_plotting;
    std::vector<std::string> load_case_ids;
    std::vector<std::string> bc_ids;
    std::vector<std::string> load_case_weights;
    std::string distribute_objective_type;
    std::string atmost_total_num_processors;
    std::string num_ranks;

    // type: "match frf data"
    std::string complex_error_measure;
    std::string convert_to_tet10;
    std::vector<std::string> frf_match_nodesets;
    std::string freq_min;
    std::string freq_max;
    std::string freq_step;
    std::string ref_frf_file;
    std::string raleigh_damping_alpha;
    std::string raleigh_damping_beta;
    std::string wtmass_scale_factor;
    std::string normalize_objective;

    // type: "compliance and volume min"
    std::string volume_misfit_target;

    // type: "stress p norm"
    std::string stress_p_norm_power;

    // type: "stress limit"
    std::string stress_limit;
    std::string relative_stress_limit;
    std::string stress_ramp_factor;
    std::string limit_power_min;
    std::string limit_power_max;
    std::string limit_power_feasible_bias;
    std::string limit_power_feasible_slope;
    std::string limit_power_infeasible_bias;
    std::string limit_power_infeasible_slope;
    std::string limit_reset_subfrequency;
    std::string limit_reset_count;
    std::string inequality_allowable_feasiblity_lower;
    std::string inequality_allowable_feasiblity_upper;
    std::string inequality_feasibility_scale;
    std::string inequality_infeasibility_scale;
    std::string stress_favor_final;
    std::string stress_favor_updates;
    std::string stress_inequality_power;
    std::string volume_penalty_power;
    std::string volume_penalty_divisor;
    std::string volume_penalty_bias;
    std::string scmm_mass_to_stress_constraint_ratio;
    std::string scmm_initial_penalty;
    std::string scmm_penalty_expansion_factor;
    std::string scmm_constraint_exponent;
    std::string scmm_initial_lagrange_multiplier;
    std::string scmm_initial_mass_weight_factor;
    std::string scmm_control_stagnation_tolerance;
    std::string scmm_write_debug_output_files;
};

struct Constraint
{
    std::string name;
    std::string type;
    std::string volume_fraction;
    std::string volume_absolute;
    std::string surface_area;
    std::string surface_area_ssid;
    std::string weight;
};

struct Block
{
    std::string block_id;
    std::string material_id;
    std::string element_type;
};

struct Material
{
    std::string material_id;
    std::string poissons_ratio;
    std::string youngs_modulus;
    std::string thermal_conductivity;
    std::string thermal_expansion;
    std::string reference_temperature;
    std::string density;
    std::string penalty_exponent;
    std::string specific_heat;
};

struct InputData
{
    std::vector<XMLGen::Objective> objectives;
    std::vector<XMLGen::Constraint> constraints;
    std::vector<XMLGen::Material> materials;
    std::vector<XMLGen::Block> blocks;

    std::string filter_type;
    std::string filter_radius_scale;
    std::string filter_radius_absolute;
    std::string filter_power;
    std::string filter_heaviside_min;
    std::string filter_heaviside_update;
    std::string filter_heaviside_max;

    int num_shape_design_variables;
    std::string optimization_type;
    std::string csm_filename;
    std::string csm_tesselation_filename;
    std::string csm_exodus_filename;
    std::string num_opt_processors;
    std::string output_frequency;
    std::string output_method;
    std::string max_iterations;
    std::string discretization;
    std::string volume_fraction;
    std::string mesh_name;
    std::string run_mesh_name;
    std::string mesh_name_without_extension;
    std::string run_mesh_name_without_extension;
    std::string mesh_extension;
    std::string plato_main_path;
    std::string lightmp_path;
    std::string sierra_sd_path;
    std::string albany_path;
    std::string plato_analyze_path;
    std::string prune_and_refine_path;
    std::string number_prune_and_refine_processors;
    std::string optimization_algorithm;
    std::string check_gradient;
    std::string check_hessian;
    std::string objective_number_standard_deviations;

    std::string restart_iteration;
    std::string initial_guess_filename;
    std::string initial_guess_field_name;
    std::string prune_mesh;
    std::string number_refines;
    std::string number_buffer_layers;
    std::string initial_density_value;
    std::string write_restart_file;

    std::string create_levelset_spheres;
    std::string levelset_sphere_radius;
    std::string levelset_sphere_packing_factor;
    std::string levelset_initialization_method;
    std::string levelset_material_box_min;
    std::string levelset_material_box_max;

    std::string mInnerKKTtoleranceGCMMA;
    std::string mOuterKKTtoleranceGCMMA;
    std::string mMaxInnerIterationsGCMMA;
    std::string mOuterStationarityToleranceGCMMA;
    std::string mInnerControlStagnationToleranceGCMMA;
    std::string mOuterControlStagnationToleranceGCMMA;
    std::string mOuterObjectiveStagnationToleranceGCMMA;
    std::string mInitialMovingAsymptotesScaleFactorGCMMA;

    std::string mHessianType;
    std::string mLimitedMemoryStorage;
    std::string mProblemUpdateFrequency;
    std::string mDisablePostSmoothingKS;
    std::string mOuterGradientToleranceKS;
    std::string mOuterStationarityToleranceKS;
    std::string mOuterStagnationToleranceKS;
    std::string mOuterControlStagnationToleranceKS;
    std::string mOuterActualReductionToleranceKS;
    std::string mTrustRegionRatioLowKS;
    std::string mTrustRegionRatioMidKS;
    std::string mTrustRegionRatioUpperKS;

    std::string mMaxRadiusScale;
    std::string mInitialRadiusScale;
    std::string mMaxTrustRegionRadius;
    std::string mMinTrustRegionRadius;
    std::string mMaxTrustRegionIterations;
    std::string mTrustRegionExpansionFactor;
    std::string mTrustRegionContractionFactor;
    std::string mMMAMoveLimit;
    std::string mMMAAsymptoteExpansion;
    std::string mMMAAsymptoteContraction;
    std::string mMMAMaxNumSubProblemIterations;
    std::string mMMAMaxTrustRegionIterations;
    std::string mMMAControlStagnationTolerance;
    std::string mMMAObjectiveStagnationTolerance;

    std::string mUseMeanNorm;
    std::string mAugLagPenaltyParam;
    std::string mFeasibilityTolerance;
    std::string mAugLagPenaltyParamScale;
    std::string mMaxNumAugLagSubProbIter;

    std::vector<std::string> levelset_nodesets;
    std::vector<std::string> fixed_block_ids;
    std::vector<std::string> fixed_sideset_ids;
    std::vector<std::string> fixed_nodeset_ids;
    std::vector<std::string> mStandardDeviations;
    std::vector<std::string> mShapeDesignVariableValues;

    std::vector<XMLGen::LoadCase> load_cases;
    std::vector<double> load_case_probabilities;
    std::vector<XMLGen::BC> bcs;
    std::vector<XMLGen::Uncertainty> uncertainties;

    std::string filter_projection_start_iteration;
    std::string filter_projection_update_interval;
    std::string filter_use_additive_continuation;
    std::string mUseNormalizationInAggregator;

    bool mPlatoAnalyzePerformerExists;
};

struct UncertaintyMetaData
{
  size_t numSamples;
  size_t numVariables;
  size_t numPeformers;
  std::vector<size_t> randomVariableIndices;
  std::vector<size_t> deterministicVariableIndices;
};

enum struct Arch
{
  CEE,
  SUMMIT
};

}
// namespace XMLGen
