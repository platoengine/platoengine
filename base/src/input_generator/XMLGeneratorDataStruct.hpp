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

// #include "Plato_SromHelpers.hpp"
#include "XMLGeneratorOutputMetadata.hpp"
// #include "XMLGeneratorRandomMetadata.hpp"
#include "XMLGeneratorServiceMetadata.hpp"
#include "XMLGeneratorScenarioMetadata.hpp"
#include "XMLGeneratorConstraintMetadata.hpp"
#include "XMLGeneratorMaterialMetadata.hpp"
#include "XMLGeneratorUncertaintyMetadata.hpp"
#include "XMLGeneratorCriterionMetadata.hpp"

namespace XMLGen
{

struct Objective
{
    std::string type;
    std::vector<std::string> criteriaIDs;
    std::vector<std::string> serviceIDs;
    std::vector<std::string> scenarioIDs;
    std::vector<std::string> weights;
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

struct BC
{
    std::string mCategory = "rigid";
    std::string type;     // temperature, displacement
    std::string app_type; // nodeset or sideset
    std::string app_id; // nodeset/sideset id
    std::string app_name; // nodeset/sideset name
    std::string dof;
    std::string bc_id;
    std::string value;
};

struct Block
{
    std::string block_id;
    std::string material_id;
    std::string element_type;
};

struct CodePaths
{
    std::string plato_main_path;
    std::string lightmp_path;
    std::string sierra_sd_path;
    std::string albany_path;
    std::string plato_analyze_path;
    std::string prune_and_refine_path;
};

struct Optimizer
{
    std::string levelset_material_box_min;
    std::string levelset_material_box_max;
    std::string initial_density_value;
    std::string optimization_type;
    std::string csm_filename;
    std::string create_levelset_spheres;
    std::string write_restart_file;
    std::string levelset_initialization_method;
    std::string max_iterations;
    std::string restart_iteration;
    std::string initial_guess_filename;
    std::string initial_guess_field_name;
    std::string prune_mesh;
    std::string number_buffer_layers;
    std::string number_prune_and_refine_processors;
    std::string number_refines;
    std::string mVerbose = "false";
    std::string mMMAMoveLimit;
    std::string mMMAControlStagnationTolerance;
    std::string mMMAObjectiveStagnationTolerance;
    std::string mMMAAsymptoteExpansion;
    std::string mMMAAsymptoteContraction;
    std::string mMMAMaxNumSubProblemIterations;
    std::string mMMAMaxTrustRegionIterations;
    std::string mMaxTrustRegionIterations;
    std::string mTrustRegionExpansionFactor;
    std::string mTrustRegionContractionFactor;
    std::string mOuterGradientToleranceKS;
    std::string mOuterStationarityToleranceKS;
    std::string mOuterStagnationToleranceKS;
    std::string mOuterControlStagnationToleranceKS;
    std::string mDisablePostSmoothingKS;
    std::string mOuterActualReductionToleranceKS;
    std::string mTrustRegionRatioLowKS;
    std::string mTrustRegionRatioMidKS;
    std::string mTrustRegionRatioUpperKS;
    std::string mInitialRadiusScale;
    std::string mMaxRadiusScale;
    std::string mProblemUpdateFrequency;
    std::string mMaxInnerIterationsGCMMA;
    std::string mInnerKKTtoleranceGCMMA;
    std::string mInnerControlStagnationToleranceGCMMA;
    std::string mOuterKKTtoleranceGCMMA;
    std::string mOuterControlStagnationToleranceGCMMA;
    std::string mOuterObjectiveStagnationToleranceGCMMA;
    std::string mOuterStationarityToleranceGCMMA;
    std::string mInitialMovingAsymptotesScaleFactorGCMMA;
    std::string levelset_sphere_packing_factor;
    std::string levelset_sphere_radius;
    std::vector<std::string> levelset_nodesets;
    std::vector<std::string> fixed_block_ids;
    std::vector<std::string> fixed_sideset_ids;
    std::vector<std::string> fixed_nodeset_ids;
    std::string num_opt_processors;
    std::string filter_type;
    std::string m_filterType_identity_generatorName;
    std::string m_filterType_kernel_generatorName;
    std::string m_filterType_kernelThenHeaviside_generatorName;
    std::string m_filterType_kernelThenTANH_generatorName;
    std::string filter_heaviside_min;
    std::string filter_heaviside_update;
    std::string filter_heaviside_max;
    std::string filter_power;
    std::string filter_radius_scale;
    std::string filter_radius_absolute;
    std::string filter_projection_start_iteration;
    std::string filter_projection_update_interval;
    std::string filter_use_additive_continuation;
    std::string optimization_algorithm;
    std::string discretization;
    std::string check_gradient;
    std::string check_hessian;
    std::string mHessianType;
    std::string mLimitedMemoryStorage;
    std::string mUseMeanNorm;
    std::string mAugLagPenaltyParam;
    std::string mFeasibilityTolerance;
    std::string mAugLagPenaltyParamScale;
    std::string mMaxNumAugLagSubProbIter;
    std::string mMaxTrustRegionRadius;
    std::string mMinTrustRegionRadius;
};

struct Mesh
{
    std::string name;
    std::string name_without_extension;
    std::string file_extension;
};

struct InputData
{
private:
    std::vector<XMLGen::Scenario> mScenarios;

public:
    const XMLGen::Scenario& scenario(const size_t& aIndex) const
    {
        if(mScenarios.empty())
        {
            THROWERR("XML Generator Input Metadata: 'scenario' list is empty.")
        }
        else if(aIndex >= mScenarios.size())
        {
            THROWERR("XML Generator Input Metadata: index is out-of-bounds, 'scenario' size is '"
                + std::to_string(mScenarios.size()) + "'.")
        }
        return mScenarios[aIndex];
    }
    const XMLGen::Scenario& scenario(const std::string& aID) const
    {
        if(mScenarios.empty())
        {
            THROWERR("XML Generator Input Metadata: 'scenario' list member data is empty.")
        }
        else if(aID.empty())
        {
            THROWERR("XML Generator Input Metadata: 'scenario' identification (id) input argument is empty.")
        }
        size_t tIndex = 0u;
        auto tFoundMatch = false;
        for(auto& tScenario : mScenarios)
        {
            tIndex = &tScenario - &mScenarios[0];
            if(tScenario.id().compare(aID) == 0)
            {
                tFoundMatch = true;
                break;
            }
        }

        if(!tFoundMatch)
        {
            THROWERR("XML Generator Input Metadata: Did not find 'scenario' with identification (id) '" + aID + "' in scenario list.")
        }

        return mScenarios[tIndex];
    }
    const std::vector<XMLGen::Scenario>& scenarios() const
    {
        return mScenarios;
    }
    void set(const std::vector<XMLGen::Scenario>& aScenarios)
    {
        mScenarios = aScenarios;
    }
    void append(const XMLGen::Scenario& aScenario)
    {
        mScenarios.push_back(aScenario);
    }

    XMLGen::Objective objective;
    std::vector<XMLGen::Constraint> constraints;
    std::vector<XMLGen::Service> services;
    std::vector<XMLGen::Criterion> criteria;
    std::vector<XMLGen::Material> materials;
    std::vector<XMLGen::Block> blocks;
    std::vector<XMLGen::Load> loads;
    std::vector<XMLGen::BC> bcs;
    XMLGen::Mesh mesh;
    XMLGen::CodePaths codepaths;
    std::vector<XMLGen::Uncertainty> uncertainties;
    XMLGen::Output mOutputMetaData;
    XMLGen::Optimizer optimizer;

    // // miscelaneous data that may be unused
    // int num_shape_design_variables;
    // std::string csm_tesselation_filename;
    // std::string csm_exodus_filename;
    // std::string volume_fraction;
    // std::string mDerivativeCheckerFinalSuperscript = "8";
    // std::string mDerivativeCheckerInitialSuperscript = "1";
    // std::vector<std::string> mShapeDesignVariableValues;
    // bool m_UseLaunch;
    // bool m_HasUncertainties;
    // bool m_RequestedVonMisesOutput;
    // Arch m_Arch;
    // std::string m_filterType_identity_XMLName;
    // std::string m_filterType_kernel_XMLName;
    // std::string m_filterType_kernelThenHeaviside_XMLName;
    // std::string m_filterType_kernelThenTANH_XMLName;
    // XMLGen::RandomMetaData mRandomMetaData;
    // XMLGen::UncertaintyMetaData m_UncertaintyMetaData;
    
    // // I think these are going to be handled somewhere other than the optimizer
    // std::string objective_number_standard_deviations;
    // std::vector<std::string> mStandardDeviations;
    // std::string mUseNormalizationInAggregator;
    // std::string output_frequency;
    // std::string output_method;
};



}
// namespace XMLGen
