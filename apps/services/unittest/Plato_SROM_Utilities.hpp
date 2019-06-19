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
 * Plato_SROM_Metadata.hpp
 *
 *  Created on: June 18, 2019
 */

#ifndef PLATO_SROM_UTILITIES_HPP_
#define PLATO_SROM_UTILITIES_HPP_

#include <set>

#include "../../base/src/optimize/Plato_SromProbDataStruct.hpp"
#include "../../base/src/tools/Plato_UniqueCounter.hpp"
#include "../../base/src/input_generator/XMLGeneratorDataStruct.hpp"
#include "Plato_SROM_Metadata.hpp"

using namespace XMLGen;

namespace Plato
{

struct VariableType
{

    enum type_t
    {
        LOAD, MATERIAL, UNDEFINED
    };

};

bool variable_type_string_to_enum(const std::string& aStringVarType, Plato::VariableType::type_t& aEnumVarType);
bool check_vector3d_values(const Plato::Vector3D & aMyOriginalLoad);
bool initialize_load_id_counter(const std::vector<XMLGen::LoadCase> &aLoadCases,
                                       Plato::UniqueCounter &aUniqueCounter);
bool expand_single_load_case(const XMLGen::LoadCase &aOldLoadCase,
                                    std::vector<XMLGen::LoadCase> &aNewLoadCaseList,
                                    Plato::UniqueCounter &aUniqueLoadIDCounter,
                                    std::map<int, std::vector<int> > &tOriginalToNewLoadCaseMap);
bool expand_load_cases(const std::vector<XMLGen::LoadCase> &aInputLoadCases,
                              std::vector<XMLGen::LoadCase> &aNewLoadCaseList,
                              std::map<int, std::vector<int> > &aOriginalToNewLoadCaseMap);
bool set_random_variable_statistics(const XMLGen::Uncertainty &aRandomVariable, Plato::srom::Statistics& aStatistics);
bool create_deterministic_load_variable(const XMLGen::LoadCase &aLoadCase, Plato::srom::Load& aLoad);
int get_or_create_random_load_variable(const XMLGen::LoadCase &aLoadCase,
                                              std::vector<Plato::srom::Load> &aRandomLoads);
void add_random_variable_to_random_load(Plato::srom::Load &aRandomLoad,
                                               const XMLGen::Uncertainty &aRandomVariable);
void create_random_loads_from_uncertainty(const XMLGen::Uncertainty& aRandomVariable,
                                                 const std::vector<XMLGen::LoadCase> &aNewLoadCases,
                                                 std::map<int, std::vector<int> > &aOriginalToNewLoadCaseMap,
                                                 std::set<int> &aRandomLoadIDs,
                                                 std::vector<Plato::srom::Load> &aRandomLoads);
void create_random_load_variables(const std::vector<XMLGen::Uncertainty> &aRandomVariables,
                                         const std::vector<XMLGen::LoadCase> &aNewLoadCases,
                                         std::map<int, std::vector<int> > &aOriginalToNewLoadCaseMap,
                                         std::set<int> &aRandomLoadIDs,
                                         std::vector<Plato::srom::Load> &aLoad);
void create_deterministic_load_variables(const std::vector<XMLGen::LoadCase> &aNewLoadCases,
                                                const std::set<int> & aRandomLoadIDs,
                                                std::vector<Plato::srom::Load> &aLoad);
bool generate_srom_load_inputs(const std::vector<XMLGen::LoadCase> &aInputLoadCases,
                                      const std::vector<XMLGen::Uncertainty> &aUncertainties,
                                      std::vector<Plato::srom::Load> &aLoads);
bool apply_rotation_matrix(const Plato::Vector3D& aRotatioAnglesInDegrees, Plato::Vector3D& aVectorToRotate);
bool define_distribution(const Plato::srom::Variable & aMyRandomVar, Plato::SromInputs<double> & aInput);
bool check_input_mean(const Plato::srom::Variable & aMyRandomVar);
bool check_input_lower_bound(const Plato::srom::Variable & aMyRandomVar);
bool check_input_upper_bound(const Plato::srom::Variable & aMyRandomVar);
bool check_input_standard_deviation(const Plato::srom::Variable & aMyRandomVar);
bool check_input_number_samples(const Plato::srom::Variable & aMyRandomVar);
bool check_input_statistics(const Plato::srom::Variable & aMyRandomVar);
bool define_input_statistics(const Plato::srom::Variable & aMyRandomVar, Plato::SromInputs<double> & aInput);
bool post_process_sample_probability_pairs(const std::vector<Plato::SromOutputs<double>> aMySromSolution,
                                                  const Plato::srom::Variable & aMyRandomVariable,
                                                  Plato::srom::RandomVariable & aMySromRandomVariable);
bool compute_uniform_random_variable_statistics(const Plato::SromInputs<double> & aSromInputs,
                                                       std::vector<Plato::SromOutputs<double>> & aSromOutputSet);
bool compute_random_variable_statistics(const Plato::SromInputs<double> & aSromInputs,
                                               std::vector<Plato::SromOutputs<double>> & aSromOutputs);
bool compute_sample_probability_pairs(const std::vector<Plato::srom::Variable> & aSetRandomVariables,
                                             std::vector<Plato::srom::RandomVariable> & aMySampleProbPairs);
bool expand_load_sample_probability_pair(const std::vector<Plato::srom::RandomVariable> & aMySampleProbPairs,
                                                Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                                Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                                Plato::srom::SampleProbabilityPairs& aMyZaxisSampleProbPairs);
bool compute_random_rotations_about_xyz(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                               const Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                               const Plato::srom::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                               std::vector<Plato::srom::RandomRotations> & aMyRandomRotations);
bool compute_random_rotations_about_xy(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                              const Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                              std::vector<Plato::srom::RandomRotations> & aMyRandomRotations);
bool compute_random_rotations_about_xz(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                              const Plato::srom::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                              std::vector<Plato::srom::RandomRotations> & aMyRandomRotations);
bool compute_random_rotations_about_yz(const Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                              const Plato::srom::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                              std::vector<Plato::srom::RandomRotations> & aMyRandomRotations);
bool compute_random_rotations_about_x(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                             std::vector<Plato::srom::RandomRotations> & aMyRandomRotations);
bool compute_random_rotations_about_y(const Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                             std::vector<Plato::srom::RandomRotations> & aMyRandomRotations);
bool compute_random_rotations_about_z(const Plato::srom::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                             std::vector<Plato::srom::RandomRotations> & aMyRandomRotations);
bool expand_random_rotations(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                    const Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                    const Plato::srom::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                    std::vector<Plato::srom::RandomRotations> & aMyRandomRotations);
bool check_expand_random_loads_inputs(const Plato::Vector3D & aMyOriginalLoad,
                                             const std::vector<Plato::srom::RandomRotations> & aMyRandomRotations);
bool expand_random_loads(const Plato::Vector3D & aMyOriginalLoad,
                                const std::vector<Plato::srom::RandomRotations> & aMyRandomRotations,
                                std::vector<Plato::srom::RandomLoad> & aMyRandomLoads);
bool update_initial_random_load_case(const std::vector<Plato::srom::RandomLoad> & aNewSetRandomLoads,
                                            std::vector<Plato::srom::RandomLoadCase> & aOldRandomLoadCases);
bool update_random_load_cases(const std::vector<Plato::srom::RandomLoad> & aNewSetRandomLoads,
                                     std::vector<Plato::srom::RandomLoadCase> & aOldRandomLoadCases);
bool expand_random_load_cases(const std::vector<Plato::srom::RandomLoad> & aNewSetRandomLoads,
                                     std::vector<Plato::srom::RandomLoadCase> & aOldRandomLoadCases);
bool expand_random_and_deterministic_loads(const std::vector<Plato::srom::Load>& aLoads,
                                                  std::vector<Plato::srom::Load>& aRandomLoads,
                                                  std::vector<Plato::srom::Load>& aDeterministicLoads);
bool check_load_parameters(const Plato::srom::Load& aLoad);
bool set_load_components(const std::vector<std::string> & aInput, Plato::Vector3D & aOutput);
bool set_random_load_parameters(const Plato::srom::Load & aOriginalLoad, std::vector<Plato::srom::RandomLoad> & aSetRandomLoads);
bool generate_set_random_rotations(const std::vector<Plato::srom::RandomVariable> & aMySampleProbPairs,
                                          std::vector<Plato::srom::RandomRotations> & aMySetRandomRotation);
bool generate_set_random_loads(const Plato::srom::Load & aOriginalLoad,
                                      const std::vector<Plato::srom::RandomRotations> & aSetRandomRotations,
                                      std::vector<Plato::srom::RandomLoad> & aSetRandomLoads);
void generate_load_case_identifiers(std::vector<Plato::srom::RandomLoadCase> & aSetLoadCases);
bool check_deterministic_loads(const std::vector<Plato::srom::Load>& aDeterministicLoads);
void append_deterministic_loads(const std::vector<Plato::srom::Load>& aDeterministicLoads,
                                       std::vector<Plato::srom::RandomLoadCase> & aSetLoadCases);
bool generate_output_random_load_cases(const std::vector<Plato::srom::Load>& aDeterministicLoads,
                                              std::vector<Plato::srom::RandomLoadCase> & aSetRandomLoadCases);
bool generate_load_sroms(const Plato::srom::InputMetaData & aInput, Plato::srom::OutputMetaData & aOutput);

} // namespace Plato

#endif /* PLATO_SROM_UTILITIES_HPP_ */
