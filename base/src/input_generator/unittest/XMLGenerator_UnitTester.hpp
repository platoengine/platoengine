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
 * XMLGenerator_UnitTester.hpp
 *
 *  Created on: Sep 28, 2017
 *
 */

#ifndef SRC_XMLGENERATOR_UNITTESTER_HPP_
#define SRC_XMLGENERATOR_UNITTESTER_HPP_

#include "XMLGenerator.hpp"


class XMLGenerator_UnitTester : public XMLGenerator
{

public:
    XMLGenerator_UnitTester();
    ~XMLGenerator_UnitTester();

    bool publicParseSingleValue(const std::vector<std::string> &aTokens,
                                const std::vector<std::string> &aInputStrings,
                                std::string &aReturnStringValue);
    bool publicParseTokens(char *buffer, std::vector<std::string> &tokens);
    bool publicParseSingleUnLoweredValue(const std::vector<std::string> &aTokens,
                                         const std::vector<std::string> &aUnLoweredTokens,
                                         const std::vector<std::string> &aInputStrings,
                                         std::string &aReturnStringValue);
    bool publicParseObjectives(std::istream &sin);
    bool publicParseLoads(std::istream &sin);
    bool publicParseBCs(std::istream &sin);
    bool publicParseOptimizationParameters(std::istream &sin);
    bool publicParseConstraints(std::istream &sin);
    bool publicParseMesh(std::istream &sin);
    bool publicParseCodePaths(std::istream &sin);
    bool publicParseBlocks(std::istream &sin);
    bool publicParseMaterials(std::istream &sin);
    bool publicParseUncertainties(std::istream &sin);
    bool publicExpandUncertaintiesForGenerate();
    bool publicDistributeObjectivesForGenerate();
    std::string getConstraintName(const int &aIndex) {return m_InputData.constraints[aIndex].name;}
    std::string getConstraintType(const int &aIndex) {return m_InputData.constraints[aIndex].type;}
    std::string getConstraintVolFrac(const int &aIndex) {return m_InputData.constraints[aIndex].volume_fraction;}
    std::string getConstraintSurfArea(const int &aIndex) {return m_InputData.constraints[aIndex].surface_area;}
    std::string getConstraintSurfAreaSidesetID(const int &aIndex) {return m_InputData.constraints[aIndex].surface_area_ssid;}
    std::string getMaterialID(const int &aIndex) {return m_InputData.materials[aIndex].material_id;}
    std::string getMaterialPenaltyExponent(const int &aIndex) {return m_InputData.materials[aIndex].penalty_exponent;}
    std::string getMaterialYoungsModulus(const int &aIndex) {return m_InputData.materials[aIndex].youngs_modulus;}
    std::string getMaterialPoissonsRatio(const int &aIndex) {return m_InputData.materials[aIndex].poissons_ratio;}
    std::string getMaterialThermalConductivity(const int &aIndex) {return m_InputData.materials[aIndex].thermal_conductivity;}
    std::string getMaterialDensity(const int &aIndex) {return m_InputData.materials[aIndex].density;}
    std::string getBlockID(const int &aIndex) {return m_InputData.blocks[aIndex].block_id;}
    std::string getBlockMaterialID(const int &aIndex) {return m_InputData.blocks[aIndex].material_id;}
    size_t getNumObjectives() {return m_InputData.objectives.size();}
    std::string getObjectiveName(const int &aIndex) {return m_InputData.objectives[aIndex].name;}
    std::string getObjectiveType(const int &aIndex) {return m_InputData.objectives[aIndex].type;}
    std::string getObjCodeName(const int &aIndex) {return m_InputData.objectives[aIndex].code_name;}
    std::string getObjPerfName(const int &aIndex) {return m_InputData.objectives[aIndex].performer_name;}
    std::string getObjFreqMin(const int &aIndex) {return m_InputData.objectives[aIndex].freq_min;}
    std::string getObjFreqMax(const int &aIndex) {return m_InputData.objectives[aIndex].freq_max;}
    std::string getObjFreqStep(const int &aIndex) {return m_InputData.objectives[aIndex].freq_step;}
    std::vector<std::string> getObjLoadIds(const int &aIndex) {return m_InputData.objectives[aIndex].load_case_ids;}
    std::vector<std::string> getObjLoadWeights(const int &aIndex) {return m_InputData.objectives[aIndex].load_case_weights;}
    std::string getObjStressLimit(const int &aIndex) {return m_InputData.objectives[aIndex].stress_limit;}
    std::string getObjStressRampFactor(const int &aIndex) {return m_InputData.objectives[aIndex].stress_ramp_factor;}
    std::string getBCApplicationType(const std::string &aBCID);
    std::string getBCApplicationID(const std::string &aBCID);
    std::string getBCApplicationDOF(const std::string &aBCID);
    std::string getLoadType(const std::string &aLoadID, const int &aLoadIndex);
    std::string getLoadApplicationType(const std::string &aLoadID, const int &aLoadIndex);
    std::string getLoadApplicationID(const std::string &aLoadID, const int &aLoadIndex);
    std::string getLoadDirectionX(const std::string &aLoadID, const int &aLoadIndex);
    std::string getLoadDirectionY(const std::string &aLoadID, const int &aLoadIndex);
    std::string getLoadDirectionZ(const std::string &aLoadID, const int &aLoadIndex);
    std::string getMatBoxMinCoords() {return m_InputData.levelset_material_box_min;}
    std::string getMatBoxMaxCoords() {return m_InputData.levelset_material_box_max;}
    std::string getInitDensityValue() {return m_InputData.initial_density_value;}
    std::string getCreateLevelsetSpheres() {return m_InputData.create_levelset_spheres;}
    std::string getLevelsetInitMethod() {return m_InputData.levelset_initialization_method;}
    std::string getMaxIterations() {return m_InputData.max_iterations;}
    std::string getRestartIteration() {return m_InputData.restart_iteration;}
    std::string getRestartFieldName() {return m_InputData.initial_guess_field_name;}
    std::string getRestartMeshFilename() {return m_InputData.initial_guess_filename;}
    std::string getKSMaxTrustIterations() {return m_InputData.KS_max_trust_region_iterations;}
    std::string getKSExpansionFactor() {return m_InputData.KS_trust_region_expansion_factor;}
    std::string getKSContractionFactor() {return m_InputData.KS_trust_region_contraction_factor;}
    std::string getKSOuterGradientTolerance() {return m_InputData.KS_outer_gradient_tolerance;}
    std::string getKSOuterStationarityTolerance() {return m_InputData.KS_outer_stationarity_tolerance;}
    std::string getKSOuterStagnationTolerance() {return m_InputData.KS_outer_stagnation_tolerance;}
    std::string getKSOuterControlStagnationTolerance() {return m_InputData.KS_outer_control_stagnation_tolerance;}
    std::string getKSOuterActualReductionTolerance() {return m_InputData.KS_outer_actual_reduction_tolerance;}
    std::string getGCMMAMaxInnerIterations() {return m_InputData.GCMMA_max_inner_iterations;}
    std::string getGCMMAInnerKKTTolerance() {return m_InputData.GCMMA_inner_kkt_tolerance;}
    std::string getGCMMAInnerControlStagnationTolerance() {return m_InputData.GCMMA_inner_control_stagnation_tolerance;}
    std::string getGCMMAOuterKKTTolerance() {return m_InputData.GCMMA_outer_kkt_tolerance;}
    std::string getGCMMAOuterControlStagnationTolerance() {return m_InputData.GCMMA_outer_control_stagnation_tolerance;}
    std::string getGCMMAOuterObjectiveStagnationTolerance() {return m_InputData.GCMMA_outer_objective_stagnation_tolerance;}
    std::string getGCMMAOuterStationarityTolerance() {return m_InputData.GCMMA_outer_stationarity_tolerance;}
    std::string getGCMMAInitialMovingAsymptotesScaleFactor() {return m_InputData.GCMMA_initial_moving_asymptotes_scale_factor;}
    std::string getLevelsetSpherePackingFactor() {return m_InputData.levelset_sphere_packing_factor;}
    std::string getLevelsetSphereRadius() {return m_InputData.levelset_sphere_radius;}
    std::string getLevelsetNodeset(const int &aIndex) {return m_InputData.levelset_nodesets[aIndex];}
    std::string getFixedBlock(const int &aIndex) {return m_InputData.fixed_block_ids[aIndex];}
    std::string getOutputFrequency() {return m_InputData.output_frequency;}
    std::string getOutputMethod() {return m_InputData.output_method;}
    std::string getFixedSideset(const int &aIndex) {return m_InputData.fixed_sideset_ids[aIndex];}
    std::string getFixedNodeset(const int &aIndex) {return m_InputData.fixed_nodeset_ids[aIndex];}
    std::string getNumberProcessors() {return m_InputData.num_opt_processors;}
    std::string getFilterPower() {return m_InputData.filter_power;}
    std::string getFilterScale() {return m_InputData.filter_radius_scale;}
    std::string getFilterAbsolute() {return m_InputData.filter_radius_absolute;}
    std::string getAlgorithm() {return m_InputData.optimization_algorithm;}
    std::string getDiscretization() {return m_InputData.discretization;}
    std::string getCheckGradient() {return m_InputData.check_gradient;}
    std::string getCheckHessian() {return m_InputData.check_hessian;}
    std::string getMeshName() {return m_InputData.mesh_name;}
    std::string getSalinasPath() {return m_InputData.sierra_sd_path;}
    std::string getAlbanyPath() {return m_InputData.albany_path;}
    std::string getLightMPPath() {return m_InputData.lightmp_path;}
    std::string getPlatoMainPath() {return m_InputData.plato_main_path;}
    void clearInputData();



};


#endif /* SRC_XMLGENERATOR_UNITTESTER_HPP_ */
