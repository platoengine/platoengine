/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

/*
 * XMLGenerator.hpp
 *
 *  Created on: Sep 28, 2017
 *
 */

#ifndef SRC_XMLGENERATOR_HPP_
#define SRC_XMLGENERATOR_HPP_

#include <string>
#include <map>
#include <vector>
#include <fstream>
#include "Plato_Parser.hpp"

struct Uncertainty
{
    std::string type; // currently always "angle variation"
    std::string load_id; // which load to vary
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
  std::string x, y, z; // direction vector
  std::string dof;  // x, y, or z
  std::string scale; // magnitude
  std::string load_id;
};
struct LoadCase
{
    std::vector<Load> loads;
    std::string id; // note LoadCase id is not necessarily shared with its Loads
};
struct BC
{
  std::string app_type; // nodeset or sideset
  std::string app_id; // nodeset/sideset id
  std::string dof;
  std::string bc_id;
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
  std::string scmm_initial_penalty;
  std::string scmm_penalty_expansion_factor;
  std::string scmm_constraint_exponent;
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
    std::string density;
    std::string penalty_exponent;
};
struct InputData 
{
  std::vector<Objective> objectives;
  std::vector<Constraint> constraints;
  std::vector<Material> materials;
  std::vector<Block> blocks;

  std::string filter_type;
  std::string filter_radius_scale;
  std::string filter_radius_absolute;
  std::string filter_power;
  std::string filter_heaviside_min;
  std::string filter_heaviside_update;
  std::string filter_heaviside_max;
  std::string filter_projection_start_iteration;
  std::string filter_projection_update_interval;
  std::string filter_use_additive_continuation;

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

  std::string mUseMeanNorm;
  std::string mAugLagPenaltyParam;
  std::string mFeasibilityTolerance;
  std::string mAugLagPenaltyParamScale;
  std::string mMaxNumAugLagSubProbIter;

  std::vector<std::string> levelset_nodesets;
  std::vector<std::string> fixed_block_ids;
  std::vector<std::string> fixed_sideset_ids;
  std::vector<std::string> fixed_nodeset_ids;

  std::vector<LoadCase> load_cases;
  std::vector<BC> bcs;
  std::vector<Uncertainty> uncertainties;
};

class XMLGenerator {

public:
  XMLGenerator(const std::string &input_filename = "", bool use_launch = false);
  ~XMLGenerator();
  bool generate();

protected:

  bool parseLoads(std::istream &fin);
  bool parseBCs(std::istream &fin);
  bool generateInterfaceXML();
  bool generateLaunchScript();
  bool generatePlatoOperationsXML();
  bool generatePlatoMainInputDeckXML();
  bool generatePerformerOperationsXML();
  bool generateSalinasOperationsXML();
  bool generateAlbanyOperationsXML();
  bool generatePlatoAnalyzeOperationsXML();
  bool generateLightMPOperationsXML();
  bool generatePhysicsInputDecks();
  bool generateSalinasInputDecks();
  bool generatePlatoAnalyzeInputDecks();
  bool generateLightMPInputDecks();
  bool generateAlbanyInputDecks();
  bool expandUncertaintiesForGenerate();
  bool distributeObjectivesForGenerate();
  bool parseFile();
  bool parseMesh(std::istream &fin);
  bool parseCodePaths(std::istream &fin);
  bool parseMaterials(std::istream &fin);
  bool parseBlocks(std::istream &fin);
  bool parseObjectives(std::istream &fin);
  bool fillObjectiveAndPerfomerNames();
  bool parseConstraints(std::istream &fin);
  bool parseOptimizationParameters(std::istream &fin);
  bool parseUncertainties(std::istream &fin);
  bool parseTokens(char *buffer, std::vector<std::string> &tokens);
  bool addNTVParameter(pugi::xml_node parent_node,
                       const std::string &name,
                       const std::string &type,
                       const std::string &value);
  bool addChild(pugi::xml_node parent_node,
                const std::string &name,
                const std::string &value);
  bool outputVolumeStage(pugi::xml_document &doc);
  bool outputVolumeGradientStage(pugi::xml_document &doc);
  bool outputSurfaceAreaStage(pugi::xml_document &doc);
  bool outputSurfaceAreaGradientStage(pugi::xml_document &doc);
  bool outputComputeStateStage(pugi::xml_document &doc);
  bool outputInternalEnergyStage(pugi::xml_document &doc);
  bool outputInternalEnergyGradientStage(pugi::xml_document &doc);
  bool outputInternalEnergyHessianStage(pugi::xml_document &doc);
  std::string toLower(const std::string &s);
  std::string toUpper(const std::string &s);
  bool find_tokens(std::vector<std::string> &tokens,
                   const int &start_index,
                   const char *str1,
                   const char *str2);
  bool parseSingleValue(const std::vector<std::string> &aTokens,
                        const std::vector<std::string> &aInputStrings,
                        std::string &aReturnStringValue);

  bool parseSingleUnLoweredValue(const std::vector<std::string> &aTokens,
                                               const std::vector<std::string> &aUnLoweredTokens,
                                      const std::vector<std::string> &aInputStrings,
                                      std::string &aReturnStringValue);

  std::string m_InputFilename;
  bool m_UseLaunch;
  InputData m_InputData;
  std::string m_filterType_identity_generatorName;
  std::string m_filterType_identity_XMLName;
  std::string m_filterType_kernel_generatorName;
  std::string m_filterType_kernel_XMLName;
  std::string m_filterType_kernelThenHeaviside_generatorName;
  std::string m_filterType_kernelThenHeaviside_XMLName;
  std::string m_filterType_kernelThenTANH_generatorName;
  std::string m_filterType_kernelThenTANH_XMLName;

private:
  /******************************************************************************//**
   * @brief Initialize Plato problem options
  **********************************************************************************/
  void initializePlatoProblemOptions();

  /******************************************************************************//**
   * @brief Set algorithm used to solve optimization problem
   * @param [in] aXMLnode data structure with information parsed from XML input file.
  **********************************************************************************/
  bool setOptimizerMethod(pugi::xml_node & aXMLnode);

  /******************************************************************************//**
   * @brief Set parameters associated with the Optimality Criteria algorithm
   * @param [in] aXMLnode data structure with information parsed from XML input file.
  **********************************************************************************/
  bool setOptimalityCriteriaOptions(pugi::xml_node & aXMLnode);

  /******************************************************************************//**
   * @brief Set parameters associated with the augmented Lagrangian algorithm
   * @param [in] aXMLnode data structure with information parsed from XML input file.
  **********************************************************************************/
  bool setAugmentedLagrangianOptions(const pugi::xml_node & aXMLnode);

  /******************************************************************************//**
   * @brief Set parameters associated with the trust region algorithm
   * @param [in] aXMLnode data structure with information parsed from XML input file.
  **********************************************************************************/
  bool setTrustRegionAlgorithmOptions(const pugi::xml_node & aXMLnode);

  /******************************************************************************//**
   * @brief Set parameters associated with the trust region Kelley-Sachs algorithm
   * @param [in] aXMLnode data structure with information parsed from XML input file.
  **********************************************************************************/
  bool setKelleySachsAlgorithmOptions(const pugi::xml_node & aXMLnode);

  /******************************************************************************//**
   * @brief Set parameters associated with the Globally Convergent Method of Moving Asymptotes algorithm
   * @param [in] aXMLnode data structure with information parsed from XML input file.
  **********************************************************************************/
  bool setGCMMAoptions(const pugi::xml_node & aXMLnode);
};


#endif /* SRC_XMLGENERATOR_HPP_ */
