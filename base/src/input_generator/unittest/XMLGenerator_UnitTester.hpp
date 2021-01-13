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

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGenerator.hpp"


class XMLGenerator_UnitTester : public XMLGen::XMLGenerator
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
    bool publicParseLoads(std::istream &sin);
    void publicParseObjective(std::istream &sin);
    void publicParseMaterials(std::istream &sin);
    void publicParseServices(std::istream &sin);
    void publicParseScenarios(std::istream &sin);
    void publicParseCriteria(std::istream &sin);
    void publicParseUncertainties(std::istream &sin);
    void publicParseBCs(std::istream &sin);
    bool publicParseOptimizationParameters(std::istream &sin);
    bool publicParseMesh(std::istream &sin);
    bool publicParseCodePaths(std::istream &sin);
    bool publicParseBlocks(std::istream &sin);
    bool publicRunSROMForUncertainVariables();
    std::string getBlockID(const int &aIndex) {return m_InputData.blocks[aIndex].block_id;}
    std::string getBlockMaterialID(const int &aIndex) {return m_InputData.blocks[aIndex].material_id;}
    std::string getBCApplicationType(const std::string &aBCID);
    std::string getBCApplicationID(const std::string &aBCID);
    std::string getBCApplicationDOF(const std::string &aBCID);
    std::string getMatBoxMinCoords() {return m_InputData.optimization_parameters().levelset_material_box_min();}
    std::string getMatBoxMaxCoords() {return m_InputData.optimization_parameters().levelset_material_box_max();}
    std::string getInitDensityValue() {return m_InputData.optimization_parameters().initial_density_value();}
    std::string getCreateLevelsetSpheres() {return m_InputData.optimization_parameters().create_levelset_spheres();}
    std::string getLevelsetInitMethod() {return m_InputData.optimization_parameters().levelset_initialization_method();}
    std::string getMaxIterations() {return m_InputData.optimization_parameters().max_iterations();}
    std::string getRestartIteration() {return m_InputData.optimization_parameters().restart_iteration();}
    std::string getRestartFieldName() {return m_InputData.optimization_parameters().initial_guess_field_name();}
    std::string getRestartMeshFilename() {return m_InputData.optimization_parameters().initial_guess_file_name();}
    std::string getKSMaxTrustIterations() {return m_InputData.optimization_parameters().ks_max_trust_region_iterations();}
    std::string getKSExpansionFactor() {return m_InputData.optimization_parameters().ks_trust_region_expansion_factor();}
    std::string getKSContractionFactor() {return m_InputData.optimization_parameters().ks_trust_region_contraction_factor();}
    std::string getKSOuterGradientTolerance() {return m_InputData.optimization_parameters().ks_outer_gradient_tolerance();}
    std::string getKSOuterStationarityTolerance() {return m_InputData.optimization_parameters().ks_outer_stationarity_tolerance();}
    std::string getKSOuterStagnationTolerance() {return m_InputData.optimization_parameters().ks_outer_stagnation_tolerance();}
    std::string getKSOuterControlStagnationTolerance() {return m_InputData.optimization_parameters().ks_outer_control_stagnation_tolerance();}
    std::string getKSOuterActualReductionTolerance() {return m_InputData.optimization_parameters().ks_outer_actual_reduction_tolerance();}
    std::string getGCMMAMaxInnerIterations() {return m_InputData.optimization_parameters().gcmma_max_inner_iterations();}
    std::string getGCMMAInnerKKTTolerance() {return m_InputData.optimization_parameters().gcmma_inner_kkt_tolerance();}
    std::string getGCMMAInnerControlStagnationTolerance() {return m_InputData.optimization_parameters().gcmma_inner_control_stagnation_tolerance();}
    std::string getGCMMAOuterKKTTolerance() {return m_InputData.optimization_parameters().gcmma_outer_kkt_tolerance();}
    std::string getGCMMAOuterControlStagnationTolerance() {return m_InputData.optimization_parameters().gcmma_outer_control_stagnation_tolerance();}
    std::string getGCMMAOuterObjectiveStagnationTolerance() {return m_InputData.optimization_parameters().gcmma_outer_objective_stagnation_tolerance();}
    std::string getGCMMAOuterStationarityTolerance() {return m_InputData.optimization_parameters().gcmma_outer_stationarity_tolerance();}
    std::string getGCMMAInitialMovingAsymptotesScaleFactor() {return m_InputData.optimization_parameters().gcmma_initial_moving_asymptotes_scale_factor();}
    std::string getLevelsetSpherePackingFactor() {return m_InputData.optimization_parameters().levelset_sphere_packing_factor();}
    std::string getLevelsetSphereRadius() {return m_InputData.optimization_parameters().levelset_sphere_radius();}
    std::string getLevelsetNodeset(const int &aIndex) {return m_InputData.optimization_parameters().levelset_nodesets()[aIndex];}
    std::string getFixedBlock(const int &aIndex) {return m_InputData.optimization_parameters().fixed_block_ids()[aIndex];}
    std::string getFixedSideset(const int &aIndex) {return m_InputData.optimization_parameters().fixed_sideset_ids()[aIndex];}
    std::string getFixedNodeset(const int &aIndex) {return m_InputData.optimization_parameters().fixed_nodeset_ids()[aIndex];}
    std::string getFilterPower() {return m_InputData.optimization_parameters().filter_power();}
    std::string getNumberProcessors();
    std::string getFilterScale() {return m_InputData.optimization_parameters().filter_radius_scale();}
    std::string getFilterAbsolute() {return m_InputData.optimization_parameters().filter_radius_absolute();}
    std::string getAlgorithm() {return m_InputData.optimization_parameters().optimization_algorithm();}
    std::string getDiscretization() {return m_InputData.optimization_parameters().discretization();}
    std::string getCheckGradient() {return m_InputData.optimization_parameters().check_gradient();}
    std::string getCheckHessian() {return m_InputData.optimization_parameters().check_hessian();}
    std::string getMeshName() {return m_InputData.mesh.name;}
    std::string getSalinasPath() {return m_InputData.codepaths.sierra_sd_path;}
    std::string getAlbanyPath() {return m_InputData.codepaths.albany_path;}
    std::string getLightMPPath() {return m_InputData.codepaths.lightmp_path;}
    std::string getPlatoMainPath() {return m_InputData.codepaths.plato_main_path;}
    void clearInputData();
    XMLGen::InputData* exposeInputData() {return &m_InputData;}
    size_t      getNumPerformers() {return m_InputData.m_UncertaintyMetaData.numPerformers;}


};


#endif /* SRC_XMLGENERATOR_UNITTESTER_HPP_ */
