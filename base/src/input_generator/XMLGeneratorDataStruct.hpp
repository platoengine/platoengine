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
// #include "XMLGeneratorOutputMetadata.hpp"
// #include "XMLGeneratorRandomMetadata.hpp"
// #include "XMLGeneratorServiceMetadata.hpp"
// #include "XMLGeneratorServiceMetadata.hpp"
// #include "XMLGeneratorConstraintMetadata.hpp"
// #include "XMLGeneratorUncertaintyMetadata.hpp"
// #include "XMLGeneratorCriterionMetadata.hpp"

namespace XMLGen
{

// struct Objective
// {
//     std::string type;
//     std::vector<std::string> criteriaIDs;
//     std::vector<std::string> serviceIDs;
//     std::vector<std::string> scenarioIDs;
//     std::vector<std::string> weights;
// };

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

// struct UncertaintyMetaData
// {
//   size_t numPerformers = 0;
//   std::vector<size_t> randomVariableIndices;
//   std::vector<size_t> deterministicVariableIndices;
// };

struct Mesh
{
    std::string name;
    std::string name_without_extension;
    std::string file_extension;
};

struct InputData
{
private:
    // std::vector<XMLGen::Service> mServices;

public:
    // const XMLGen::Service& service(const size_t& aIndex) const
    // {
    //     if(mServices.empty())
    //     {
    //         THROWERR("XML Generator Input Metadata: 'service' list is empty.")
    //     }
    //     else if(aIndex >= mServices.size())
    //     {
    //         THROWERR("XML Generator Input Metadata: index is out-of-bounds, 'service' size is '"
    //             + std::to_string(mServices.size()) + "'.")
    //     }
    //     return mServices[aIndex];
    // }
    // const XMLGen::Service& service(const std::string& aID) const
    // {
    //     if(mServices.empty())
    //     {
    //         THROWERR("XML Generator Input Metadata: 'service' list member data is empty.")
    //     }
    //     else if(aID.empty())
    //     {
    //         THROWERR("XML Generator Input Metadata: 'service' identification (id) input argument is empty.")
    //     }
    //     size_t tIndex = 0u;
    //     auto tFoundMatch = false;
    //     for(auto& tService : mServices)
    //     {
    //         tIndex = &tService - &mServices[0];
    //         if(tService.id().compare(aID) == 0)
    //         {
    //             tFoundMatch = true;
    //             break;
    //         }
    //     }

    //     if(!tFoundMatch)
    //     {
    //         THROWERR("XML Generator Input Metadata: Did not find 'service' with identification (id) '" + aID + "' in service list.")
    //     }

    //     return mServices[tIndex];
    // }
    // const std::vector<XMLGen::Service>& services() const
    // {
    //     return mServices;
    // }
    // void set(const std::vector<XMLGen::Service>& aServices)
    // {
    //     mServices = aServices;
    // }
    // void append(const XMLGen::Service& aService)
    // {
    //     mServices.push_back(aService);
    // }

    // XMLGen::Objective objective;
    // std::vector<XMLGen::Constraint> constraints;
    // std::vector<XMLGen::Criterion> criteria;
    // std::vector<XMLGen::Material> materials;
    std::vector<XMLGen::Block> blocks;
    std::vector<XMLGen::Load> loads;
    std::vector<XMLGen::BC> bcs;
    XMLGen::Mesh mesh;
    // std::vector<XMLGen::Uncertainty> uncertainties;

    // std::string mVerbose = "false";
    // std::string filter_type;
    // std::string filter_radius_scale;
    // std::string filter_radius_absolute;
    // std::string filter_power;
    // std::string filter_heaviside_min;
    // std::string filter_heaviside_update;
    // std::string filter_heaviside_max;

    // int num_shape_design_variables;
    // std::string optimization_type;
    // std::string csm_filename;
    // std::string csm_tesselation_filename;
    // std::string csm_exodus_filename;
    // std::string num_opt_processors;
    // std::string output_frequency;
    // std::string output_method;
    // std::string max_iterations;
    // std::string discretization;
    // std::string volume_fraction;
    // std::string plato_main_path;
    // std::string lightmp_path;
    // std::string sierra_sd_path;
    // std::string albany_path;
    // std::string plato_analyze_path;
    // std::string prune_and_refine_path;
    // std::string number_prune_and_refine_processors;
    // std::string optimization_algorithm;
    // std::string check_gradient;
    // std::string check_hessian;
    // std::string objective_number_standard_deviations;

    // std::string restart_iteration;
    // std::string initial_guess_filename;
    // std::string initial_guess_field_name;
    // std::string prune_mesh;
    // std::string number_refines;
    // std::string number_buffer_layers;
    // std::string initial_density_value;
    // std::string write_restart_file;

    // std::string create_levelset_spheres;
    // std::string levelset_sphere_radius;
    // std::string levelset_sphere_packing_factor;
    // std::string levelset_initialization_method;
    // std::string levelset_material_box_min;
    // std::string levelset_material_box_max;

    // std::string mDerivativeCheckerFinalSuperscript = "8";
    // std::string mDerivativeCheckerInitialSuperscript = "1";

    // std::string mInnerKKTtoleranceGCMMA;
    // std::string mOuterKKTtoleranceGCMMA;
    // std::string mMaxInnerIterationsGCMMA;
    // std::string mOuterStationarityToleranceGCMMA;
    // std::string mInnerControlStagnationToleranceGCMMA;
    // std::string mOuterControlStagnationToleranceGCMMA;
    // std::string mOuterObjectiveStagnationToleranceGCMMA;
    // std::string mInitialMovingAsymptotesScaleFactorGCMMA;

    // std::string mHessianType;
    // std::string mLimitedMemoryStorage;
    // std::string mProblemUpdateFrequency;
    // std::string mDisablePostSmoothingKS;
    // std::string mOuterGradientToleranceKS;
    // std::string mOuterStationarityToleranceKS;
    // std::string mOuterStagnationToleranceKS;
    // std::string mOuterControlStagnationToleranceKS;
    // std::string mOuterActualReductionToleranceKS;
    // std::string mTrustRegionRatioLowKS;
    // std::string mTrustRegionRatioMidKS;
    // std::string mTrustRegionRatioUpperKS;

    // std::string mMaxRadiusScale;
    // std::string mInitialRadiusScale;
    // std::string mMaxTrustRegionRadius;
    // std::string mMinTrustRegionRadius;
    // std::string mMaxTrustRegionIterations;
    // std::string mTrustRegionExpansionFactor;
    // std::string mTrustRegionContractionFactor;
    // std::string mMMAMoveLimit;
    // std::string mMMAAsymptoteExpansion;
    // std::string mMMAAsymptoteContraction;
    // std::string mMMAMaxNumSubProblemIterations;
    // std::string mMMAMaxTrustRegionIterations;
    // std::string mMMAControlStagnationTolerance;
    // std::string mMMAObjectiveStagnationTolerance;

    // std::string mUseMeanNorm;
    // std::string mAugLagPenaltyParam;
    // std::string mFeasibilityTolerance;
    // std::string mAugLagPenaltyParamScale;
    // std::string mMaxNumAugLagSubProbIter;

    // std::vector<std::string> levelset_nodesets;
    // std::vector<std::string> fixed_block_ids;
    // std::vector<std::string> fixed_sideset_ids;
    // std::vector<std::string> fixed_nodeset_ids;
    // std::vector<std::string> mStandardDeviations;
    // std::vector<std::string> mShapeDesignVariableValues;

    // std::vector<double> load_case_probabilities;

    // std::string filter_projection_start_iteration;
    // std::string filter_projection_update_interval;
    // std::string filter_use_additive_continuation;
    // std::string mUseNormalizationInAggregator;

    // bool m_UseLaunch;
    // bool m_HasUncertainties;
    // bool m_RequestedVonMisesOutput;
    // bool m_UseNewPlatoAnalyzeUncertaintyWorkflow;
    // Arch m_Arch;
    // std::string m_filterType_identity_generatorName;
    // std::string m_filterType_identity_XMLName;
    // std::string m_filterType_kernel_generatorName;
    // std::string m_filterType_kernel_XMLName;
    // std::string m_filterType_kernelThenHeaviside_generatorName;
    // std::string m_filterType_kernelThenHeaviside_XMLName;
    // std::string m_filterType_kernelThenTANH_generatorName;
    // std::string m_filterType_kernelThenTANH_XMLName;

    // XMLGen::Output mOutputMetaData;
    // XMLGen::RandomMetaData mRandomMetaData;
    // XMLGen::UncertaintyMetaData m_UncertaintyMetaData;
    // std::string input_generator_version;
};



}
// namespace XMLGen
