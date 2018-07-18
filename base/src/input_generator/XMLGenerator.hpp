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

struct Load
{
  std::string type; // traction, heat flux, force, ...
  std::string app_type; // nodeset or sideset
  std::string app_id; // nodeset/sideset id
  std::string x, y, z; // direction vector
  std::string dof;  // x, y, or z
  std::string scale; // magnitude
};
struct BC
{
  std::string app_type; // nodeset or sideset
  std::string app_id; // nodeset/sideset id
  std::string dof;
};
struct Objective
{
  std::string name;
  std::string type;
  std::string code_name;
  std::string performer_name;
  std::vector<Load> loads;
  std::vector<BC> bcs;
  std::string weight;
  std::string num_procs;
  std::string freq_min;
  std::string freq_max;
  std::string freq_step;
  std::string ref_frf_file;
  std::string raleigh_damping_alpha;
  std::string raleigh_damping_beta;
  std::string salinas_wtmass_scale_factor;
  std::string salinas_gdsw_tolerance;
  std::string complex_error_measure;
  std::string convert_to_tet10;
  std::vector<std::string> frf_match_nodesets;
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
  std::string filter_radius_scale;
  std::string filter_radius_absolute;
  std::string num_opt_processors;
  std::string output_frequency;
  std::string output_method;
  std::string max_iterations;
  std::string discretization;
  std::string volume_fraction;
  std::string mesh_name;
  std::string plato_main_path;
  std::string lightmp_path;
  std::string salinas_path;
  std::string albany_path;
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
  std::string create_levelset_spheres;
  std::string levelset_sphere_radius;
  std::string levelset_sphere_packing_factor;
  std::string levelset_initialization_method;
  std::string levelset_material_box_min;
  std::string levelset_material_box_max;
  std::string GCMMA_inner_kkt_tolerance;
  std::string GCMMA_outer_kkt_tolerance;
  std::string GCMMA_inner_control_stagnation_tolerance;
  std::string GCMMA_outer_control_stagnation_tolerance;
  std::string GCMMA_outer_objective_stagnation_tolerance;
  std::string GCMMA_max_inner_iterations;
  std::string GCMMA_outer_stationarity_tolerance;
  std::string GCMMA_initial_moving_asymptotes_scale_factor;
  std::string KS_max_trust_region_iterations;
  std::string KS_trust_region_expansion_factor;
  std::string KS_trust_region_contraction_factor;
  std::string KS_outer_gradient_tolerance;
  std::string KS_outer_stationarity_tolerance;
  std::string KS_outer_stagnation_tolerance;
  std::string KS_outer_control_stagnation_tolerance;
  std::string KS_outer_actual_reduction_tolerance;
  std::vector<std::string> levelset_nodesets;
  std::vector<std::string> fixed_block_ids;
  std::vector<std::string> fixed_sideset_ids;
  std::vector<std::string> fixed_nodeset_ids;
};

class XMLGenerator {

public:
  XMLGenerator(const std::string &input_filename);
  ~XMLGenerator();
  bool generate();

protected:
  XMLGenerator(){};
  std::string m_InputFilename;
  InputData m_InputData;

  bool generateInterfaceXML();
  bool generateLaunchScript();
  bool generatePlatoOperationsXML();
  bool generatePlatoMainInputDeckXML();
  bool generatePerformerOperationsXML();
  bool generateSalinasOperationsXML();
  bool generateAlbanyOperationsXML();
  bool generateLightMPOperationsXML();
  bool generatePhysicsInputDecks();
  bool generateSalinasInputDecks();
  bool generateLightMPInputDecks();
  bool generateAlbanyInputDecks();
  bool parseFile();
  bool parseMesh(std::istream &fin);
  bool parseCodePaths(std::istream &fin);
  bool parseMaterials(std::istream &fin);
  bool parseBlocks(std::istream &fin);
  bool parseObjectives(std::istream &fin);
  bool parseConstraints(std::istream &fin);
  bool parseOptimizationParameters(std::istream &fin);
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



};


#endif /* SRC_XMLGENERATOR_HPP_ */
