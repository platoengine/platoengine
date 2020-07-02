/*
 * XMLGeneratorDataStruct.hpp
 *
 *  Created on: Jun 1, 2019
 */

#pragma once

#include <unordered_map>
#include <string>
#include <vector>
#include <map>

#include "XMLGeneratorRandomMetadata.hpp"
#include "Plato_SromHelpers.hpp"

namespace XMLGen
{

struct Scenario
{
private:
    bool mCacheState = false;
    bool mUpdateProblem = false;
    bool mUseNewAnalyzeUQWorkflow = false;

    std::string mID = "0";
    std::string mCode = "plato_analyze";
    std::string mPhysics = "mechanical";
    std::string mPerformer = "plato_analyze";
    std::string mSpatialDims = "3";
    std::string mPenaltyParam = "3.0";
    std::string mMinimumErsatzValue = "1e-9";

public:
    void id(const std::string& aInput)
    {
        mID = aInput;
    }
    std::string id() const
    {
        return mID;
    }

    void code(const std::string& aInput)
    {
        mCode = aInput;
    }
    std::string code() const
    {
        return mCode;
    }

    void physics(const std::string& aInput)
    {
        mPhysics = aInput;
    }
    std::string physics() const
    {
        return mPhysics;
    }

    void performer(const std::string& aInput)
    {
        mPerformer = aInput;
    }
    std::string performer() const
    {
        return mPerformer;
    }

    void dimensions(const std::string& aInput)
    {
        mSpatialDims = aInput;
    }
    std::string dimensions() const
    {
        return mSpatialDims;
    }

    void materialPenaltyExponent(const std::string& aInput)
    {
        mPenaltyParam = aInput;
    }
    std::string materialPenaltyExponent() const
    {
        return mPenaltyParam;
    }

    void minErsatzMaterialConstant(const std::string& aInput)
    {
        mMinimumErsatzValue = aInput;
    }
    std::string minErsatzMaterialConstant() const
    {
        return mMinimumErsatzValue;
    }

    void cacheState(const bool& aInput)
    {
        mCacheState = aInput;
    }
    bool cacheState() const
    {
        return mCacheState;
    }

    void updateProblem(const bool& aInput)
    {
        mUpdateProblem = aInput;
    }
    bool updateProblem() const
    {
        return mUpdateProblem;
    }

    void useNewAnalyzeUQWorkflow(const bool& aInput)
    {
        mUseNewAnalyzeUQWorkflow = aInput;
    }
    bool useNewAnalyzeUQWorkflow() const
    {
        return mUseNewAnalyzeUQWorkflow;
    }
};

/******************************************************************************//**
 * \struct Output
 * \brief The Output metadata structure owns the random and deterministic output \n
 * Quantities of Interests (QoIs) defined by the user in the plato input file.
**********************************************************************************/
struct Output
{
private:

    /******************************************************************************//**
     * \var QoI maps
     * \brief Maps from QoIs argument names to shared_data_name-data_layout pairs, i.e. \n
     * map<qoi_argument_name, pair<qoi_shared_data_name, qoi_layout>>, where qoi_layout \n
     * denotes the shared data layout and qoi_argument_name denotes the keyword used by \n
     * the plato input file to denote an output QoI. The argument name keyword is used \n
     * to define the shared data name used for the interface.xml file.
    **********************************************************************************/
    std::map<std::string, std::pair<std::string, std::string>> mRandomQoIs;
    std::map<std::string, std::pair<std::string, std::string>> mDeterministicQoIs;

    bool mEnableOutputStage = false;

public:
    bool outputData() const
    {
        return mEnableOutputStage;
    }
    void outputData(const bool& aOutputData)
    {
        mEnableOutputStage = aOutputData;
    }

    void appendRandomQoI(const std::string& aArgumentName, const std::string& aDataLayout)
    {
        auto tLowerLayout = Plato::tolower(aDataLayout);
        auto tLowerArgumentName = Plato::tolower(aArgumentName);
        auto tSharedDataName = tLowerArgumentName + " {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
        mRandomQoIs[tLowerArgumentName] = std::make_pair(tSharedDataName, tLowerLayout);
    }

    void appendDeterminsiticQoI(const std::string& aArgumentName, const std::string& aDataLayout)
    {
        auto tLowerLayout = Plato::tolower(aDataLayout);
        auto tLowerArgumentName = Plato::tolower(aArgumentName);
        auto tSharedDataName = tLowerArgumentName;
        mDeterministicQoIs[tLowerArgumentName] = std::make_pair(tSharedDataName, tLowerLayout);
    }

    const std::map<std::string, std::pair<std::string, std::string>>& getRandomQoIs() const
    {
        return mRandomQoIs;
    }

    const std::map<std::string, std::pair<std::string, std::string>>& getDeterminsiticQoIs() const
    {
        return mDeterministicQoIs;
    }

    std::string getSharedDataNameRandomQoI(const std::string& aArgumentName) const
    {
        auto tLowerArgumentName = Plato::tolower(aArgumentName);
        auto tItr = mRandomQoIs.find(tLowerArgumentName);
        if(tItr == mRandomQoIs.end())
        {
            THROWERR("XML Generator Output Metadata: Did not find argument name '" + tLowerArgumentName
                + "' in map from random QoI argument name to shared_data_name-data_layout pair.")
        }
        return tItr->second.first;
    }

    std::string getSharedDataNameDeterministicQoI(const std::string& aArgumentName) const
    {
        auto tLowerArgumentName = Plato::tolower(aArgumentName);
        auto tItr = mDeterministicQoIs.find(tLowerArgumentName);
        if(tItr == mDeterministicQoIs.end())
        {
            THROWERR("XML Generator Output Metadata: Did not find argument name '" + tLowerArgumentName
                + "' in map from deterministic QoI argument name to shared_data_name-data_layout pair.")
        }
        return tItr->second.first;
    }
};

struct Objective
{
    std::string code_name;
    std::string code() const {return code_name;}
    std::string type;
    std::string category() const {return type;}
    std::string mPerformerName = "plato_analyze";
    std::string performer() const { return mPerformerName; }
    std::string mMinimumErsatzValue = "1e-9";
    std::string minErsatzMaterialConstant() const { return mMinimumErsatzValue; }
    std::string mPenaltyParam = "3.0";
    std::string materialPenaltyExponent() const { return mPenaltyParam; }
    std::string mPnormExponent = "6";
    std::string pnormExponent() const { return mPnormExponent; }

    std::string name;
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
private:
    std::string mName;
    std::string mCode;
    std::string mCategory;
    std::string mPerformerName = "PlatoMain";
    std::string mNormalizedTargetValue;
    std::string mAbsoluteTargetValue;
    std::string mPenaltyParam = "3.0";
    std::string mPnormExponent = "6";
    std::string mMinimumErsatzValue = "1e-9";
    std::string mWeight;

public:
    std::string surface_area; // TO BE DEPRECATED SOON!
    std::string surface_area_ssid; // TO BE DEPRECATED SOON!

    void name(const std::string& aInput)
    {
        mName = aInput;
    }
    std::string name() const
    {
        return mName;
    }

    void code(const std::string& aInput)
    {
        mCode = aInput;
    }
    std::string code() const
    {
        return mCode;
    }

    void weight(const std::string& aInput)
    {
        mWeight = aInput;
    }
    std::string weight() const
    {
        return mWeight;
    }

    void category(const std::string& aInput)
    {
        mCategory = aInput;
    }
    std::string category() const
    {
        return mCategory;
    }

    void performer(const std::string& aInput)
    {
        mPerformerName = aInput;
    }
    std::string performer() const
    {
        return mPerformerName;
    }

    void pnormExponent(const std::string& aInput)
    {
        mPnormExponent = aInput;
    }
    std::string pnormExponent() const
    {
        return mPnormExponent;
    }

    void normalizedTarget(const std::string& aInput)
    {
        mNormalizedTargetValue = aInput;
    }
    std::string normalizedTarget() const
    {
        return mNormalizedTargetValue;
    }

    void absoluteTarget(const std::string& aInput)
    {
        mAbsoluteTargetValue = aInput;
    }
    std::string absoluteTarget() const
    {
        return mAbsoluteTargetValue;
    }

    void materialPenaltyExponent(const std::string& aInput)
    {
        mPenaltyParam = aInput;
    }
    std::string materialPenaltyExponent() const
    {
        return mPenaltyParam;
    }

    void minErsatzMaterialConstant(const std::string& aInput)
    {
        mMinimumErsatzValue = aInput;
    }
    std::string minErsatzMaterialConstant() const
    {
        return mMinimumErsatzValue;
    }
};

struct Block
{
    std::string block_id;
    std::string material_id;
    std::string element_type;
};

struct UncertaintyMetaData
{
  size_t numPerformers = 0;
  std::vector<size_t> randomVariableIndices;
  std::vector<size_t> deterministicVariableIndices;
};
// struct UncertaintyMetaData

struct Uncertainty
{
    std::string variable_type = "load"; // Load or material
    std::string type; // currently always "angle variation"
    std::string id; // which random category identification number, i.e. material or load id
    std::string axis; // x, y, z
    std::string distribution; // normal, uniform, beta
    std::string mean; // scalar value
    std::string upper; // scalar value
    std::string lower; // scalar value
    std::string standard_deviation; // scalar value
    std::string num_samples; // integer value
    std::string file = ""; // filename with sample-probability pairs
};
// struct Uncertainty

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

    std::string mDerivativeCheckerFinalSuperscript = "8";
    std::string mDerivativeCheckerInitialSuperscript = "1";

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
    bool mAllPerformersArePlatoAnalyze;
    bool mAllObjectivesAreComplianceMinimization;

    bool m_UseLaunch;
    bool m_HasUncertainties;
    bool m_RequestedVonMisesOutput;
    bool m_UseNewPlatoAnalyzeUncertaintyWorkflow;
    Arch m_Arch;
    std::string m_filterType_identity_generatorName;
    std::string m_filterType_identity_XMLName;
    std::string m_filterType_kernel_generatorName;
    std::string m_filterType_kernel_XMLName;
    std::string m_filterType_kernelThenHeaviside_generatorName;
    std::string m_filterType_kernelThenHeaviside_XMLName;
    std::string m_filterType_kernelThenTANH_generatorName;
    std::string m_filterType_kernelThenTANH_XMLName;

    XMLGen::Output mOutputMetaData;
    XMLGen::Scenario mScenarioMetaData;
    XMLGen::RandomMetaData mRandomMetaData;
    XMLGen::UncertaintyMetaData m_UncertaintyMetaData;
    std::string input_generator_version;
};



}
// namespace XMLGen
