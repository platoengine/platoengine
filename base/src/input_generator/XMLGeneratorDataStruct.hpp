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
#include <set>

#include "Plato_SromHelpers.hpp"
#include "XMLGeneratorOutputMetadata.hpp"
#include "XMLGeneratorRandomMetadata.hpp"
#include "XMLGeneratorBoundaryMetadata.hpp"
#include "XMLGeneratorServiceMetadata.hpp"
#include "XMLGeneratorScenarioMetadata.hpp"
#include "XMLGeneratorConstraintMetadata.hpp"
#include "XMLGeneratorMaterialMetadata.hpp"
#include "XMLGeneratorEssentialBoundaryConditionMetadata.hpp"
#include "XMLGeneratorNaturalBoundaryConditionMetadata.hpp"
#include "XMLGeneratorUncertaintyMetadata.hpp"
#include "XMLGeneratorCriterionMetadata.hpp"
#include "XMLGeneratorOptimizationParametersMetadata.hpp"

namespace XMLGen
{

/******************************************************************************//**
* \enum \struct High-Performance Platform
* \brief Denotes the High-Performance Platform use to run stochastic use cases.
**********************************************************************************/
enum struct Arch
{
    CEE,
    SUMMIT
};
// enum struct Arch
    
struct Objective
{
    std::string type;
    std::vector<std::string> criteriaIDs;
    std::vector<std::string> serviceIDs;
    std::vector<std::string> shapeServiceIDs;
    std::vector<std::string> scenarioIDs;
    std::vector<std::string> weights;
    std::string multi_load_case;
};

struct Block
{
    std::string block_id;
    std::string name;
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

/*
struct OptimizationParameters
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
    std::string mNormalizeInAggregator = "";
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
    std::string mDerivativeCheckerFinalSuperscript = "8";
    std::string mDerivativeCheckerInitialSuperscript = "1";
    std::string mMinTrustRegionRadius;
    std::string output_frequency;
    std::string output_method;
    std::string objective_number_standard_deviations;
};
*/

struct Mesh
{
    std::string name;
    std::string name_without_extension;
    std::string run_name;
    std::string run_name_without_extension;
    std::string file_extension;
};

struct UncertaintyMetaData
{
  size_t numPerformers = 0;
  std::vector<size_t> randomVariableIndices;
  std::vector<size_t> deterministicVariableIndices;
};

struct InputData
{
private:
    std::vector<XMLGen::Scenario> mScenarios;
    std::vector<XMLGen::Service> mServices;
    std::vector<XMLGen::Criterion> mCriteria;
    XMLGen::OptimizationParameters mOptimizationParameters;

public:
    
    std::vector<XMLGen::NaturalBoundaryCondition> scenarioLoads(const std::string& aID) const
    {
        std::vector<XMLGen::NaturalBoundaryCondition> tScenarioLoads;
        auto &tScenario = scenario(aID);
        for(auto &tLoadID : tScenario.loadIDs())
        {
            for(auto &tLoad : loads)
            {
                if(tLoad.id() == tLoadID)
                {
                    tScenarioLoads.push_back(tLoad);
                    break;
                }
            }
        }
        return tScenarioLoads;
    }

    std::set<ConcretizedCriterion> getConcretizedCriteria() const
    {
        std::set<ConcretizedCriterion> tConcretizedCriteria;

        for(auto& tConstraint: constraints)
        {
            std::string tCriterionID = tConstraint.criterion();
            ConcretizedCriterion tConcretizedCriterion(tCriterionID,tConstraint.service(),tConstraint.scenario());
            tConcretizedCriteria.insert(tConcretizedCriterion);
        }

        if(objective.multi_load_case == "true")
        {
            if(objective.criteriaIDs.size() > 0)
            {
                std::string tCriterionID = objective.criteriaIDs[0];
                std::string tServiceID = objective.serviceIDs[0];
                std::string tScenarioID = "";
                for(auto tCurScenarioID : objective.scenarioIDs)
                {
                    tScenarioID += tCurScenarioID;
                }
                ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
                tConcretizedCriteria.insert(tConcretizedCriterion);
            }
        }
        else
        {
            for(size_t i=0; i<objective.criteriaIDs.size(); ++i)
            {
                std::string tCriterionID = objective.criteriaIDs[i];
                std::string tServiceID = objective.serviceIDs[i];
                std::string tScenarioID = objective.scenarioIDs[i];
                ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
                tConcretizedCriteria.insert(tConcretizedCriterion);
            }
        }


        return tConcretizedCriteria;
    }

    bool normalizeInAggregator() const
    {
        bool tReturnValue = false;

        // First try to determine based off of algorithm
        if(mOptimizationParameters.optimization_algorithm() == "ksal" ||
           mOptimizationParameters.optimization_algorithm() == "ksbc" ||
           mOptimizationParameters.optimization_algorithm() == "oc")
        {
            tReturnValue = true;
        }   

        // User-set flag trumps everything else
        if(mOptimizationParameters.normalize_in_aggregator() != "")
        {
            tReturnValue = (XMLGen::to_lower(mOptimizationParameters.normalize_in_aggregator()) == "true");
        }

        return tReturnValue;
    }

    bool needToDoWeightingInAggregator() const
    {
        return (objective.type == "weighted_sum") || (objective.criteriaIDs.size() > 1);
        //return !allSubObjectivesOnOnePlatoAnalyzePerformer();
    }

    bool allSubObjectivesOnOnePlatoAnalyzePerformer() const
    {
        bool tReturn = true;
        if(objective.criteriaIDs.size() == 0)
        {
            THROWERR("XML Generator Input Metadata: There is no objective information.")
        }
        std::string tFirstServiceID = objective.serviceIDs[0];
        for(auto &tServiceID : objective.serviceIDs)
        {
            const XMLGen::Service &tService = service(tServiceID);
            if(tService.code() != "plato_analyze" ||
               tServiceID != tFirstServiceID)
            {
                tReturn = false;
                break;
            }
        }
        return tReturn; 
    }

    bool needToAggregate() const
    {
        bool tReturnValue = false;
        if(normalizeInAggregator())
        {
            tReturnValue = true;
        }
        else
        {
            if(objective.multi_load_case != "true" && 
               needToDoWeightingInAggregator())
            {
                tReturnValue = true;
            }
        }
        
        return tReturnValue;
    }

    std::string getFirstPlatoAnalyzePerformer() const
    {
        std::string tReturnValue = "";
        for(auto& tService : mServices)
        {
            if(tService.code() == "plato_analyze")
            {
                tReturnValue = tService.performer();
                break;
            }
        }
        return tReturnValue;
    }

    std::string getFirstPlatoMainPerformer() const
    {
        std::string tReturnValue = "";
        for(auto& tService : mServices)
        {
            if(tService.code() == "platomain")
            {
                tReturnValue = tService.performer();
                break;
            }
        }
        return tReturnValue;
    }

    const XMLGen::OptimizationParameters& optimization_parameters() const 
    {
        return mOptimizationParameters;
    }

    // Scenario access functions
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
    void append_unique(const XMLGen::Scenario& aScenario)
    {
        for(auto &tScenario : mScenarios)
        {
            if(tScenario.id() == aScenario.id())
            {
                return;
            }
        }
        mScenarios.push_back(aScenario);
    }

    // Service access functions
    const XMLGen::Service& service(const size_t& aIndex) const
    {
        if(mServices.empty())
        {
            THROWERR("XML Generator Input Metadata: 'service' list is empty.")
        }
        else if(aIndex >= mServices.size())
        {
            THROWERR("XML Generator Input Metadata: index is out-of-bounds, 'service' size is '"
                + std::to_string(mServices.size()) + "'.")
        }
        return mServices[aIndex];
    }
    const XMLGen::Service& service(const std::string& aID) const
    {
        if(mServices.empty())
        {
            THROWERR("XML Generator Input Metadata: 'service' list member data is empty.")
        }
        else if(aID.empty())
        {
            THROWERR("XML Generator Input Metadata: 'service' identification (id) input argument is empty.")
        }
        size_t tIndex = 0u;
        auto tFoundMatch = false;
        for(auto& tService : mServices)
        {
            tIndex = &tService - &mServices[0];
            if(tService.id().compare(aID) == 0)
            {
                tFoundMatch = true;
                break;
            }
        }

        if(!tFoundMatch)
        {
            THROWERR("XML Generator Input Metadata: Did not find 'service' with identification (id) '" + aID + "' in service list.")
        }

        return mServices[tIndex];
    }
    const std::vector<XMLGen::Service>& services() const
    {
        return mServices;
    }
    void set(const std::vector<XMLGen::Service>& aServices)
    {
        mServices = aServices;
    }
    void append(const XMLGen::Service& aService)
    {
        mServices.push_back(aService);
    }
    void append_unique(const XMLGen::Service& aService)
    {
        for(auto &tService : mServices)
        {
            if(tService.id() == aService.id())
            {
                return;
            }
        }
        mServices.push_back(aService);
    }
    // Criteria access
    const XMLGen::Criterion& criterion(const std::string& aID) const
    {
        if(mCriteria.empty())
        {
            THROWERR("XML Generator Input Metadata: 'criterion' list member data is empty.")
        }
        else if(aID.empty())
        {
            THROWERR("XML Generator Input Metadata: 'criterion' identification (id) input argument is empty.")
        }
        size_t tIndex = 0u;
        auto tFoundMatch = false;
        for(auto& tCriterion : mCriteria)
        {
            tIndex = &tCriterion - &mCriteria[0];
            if(tCriterion.id().compare(aID) == 0)
            {
                tFoundMatch = true;
                break;
            }
        }

        if(!tFoundMatch)
        {
            THROWERR("XML Generator Input Metadata: Did not find 'criterion' with identification (id) '" + aID + "' in criteria list.")
        }

        return mCriteria[tIndex];
    }
    void set(const std::vector<XMLGen::Criterion>& aCriteria)
    {
        mCriteria = aCriteria;
    }
    void append(const XMLGen::Criterion& aCriterion)
    {
        mCriteria.push_back(aCriterion);
    }
    void append_unique(const XMLGen::Criterion& aCriterion)
    {
        for(auto &tCriterion : mCriteria)
        {
            if(tCriterion.id() == aCriterion.id())
            {
                return;
            }
        }
        mCriteria.push_back(aCriterion);
    }

    // OptimizationParameters
    void set(XMLGen::OptimizationParameters aOptimizationParameters)
    {
        mOptimizationParameters = aOptimizationParameters;
    }

    XMLGen::Objective objective;
    std::vector<XMLGen::Constraint> constraints;
    std::vector<XMLGen::Material> materials;
    std::vector<XMLGen::EssentialBoundaryCondition> ebcs;
    std::vector<XMLGen::Block> blocks;
    std::vector<XMLGen::NaturalBoundaryCondition> loads;
    XMLGen::Mesh mesh;
    XMLGen::CodePaths codepaths;
    std::vector<XMLGen::Uncertainty> uncertainties;
    std::vector<XMLGen::Output> mOutputMetaData;
    XMLGen::Arch m_Arch;
    bool m_UseLaunch;
    XMLGen::RandomMetaData mRandomMetaData;
    XMLGen::UncertaintyMetaData m_UncertaintyMetaData;
    std::vector<XMLGen::Service> mPerformerServices;

    // // miscelaneous data that may be unused
    // int num_shape_design_variables;
    // std::string csm_tesselation_filename;
    // std::string csm_exodus_filename;
    // std::string volume_fraction;
    // std::vector<std::string> mShapeDesignVariableValues;
    // bool m_HasUncertainties;
    // bool m_RequestedVonMisesOutput;
    // std::string m_filterType_identity_XMLName;
    // std::string m_filterType_kernel_XMLName;
    // std::string m_filterType_kernelThenHeaviside_XMLName;
    // std::string m_filterType_kernelThenTANH_XMLName;
    
    // // I think these are going to be handled somewhere other than the optimizer
    // std::vector<std::string> mStandardDeviations;
    // std::string mUseNormalizationInAggregator;
};





}
// namespace XMLGen
