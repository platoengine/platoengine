/*
 * XMLGeneratorLaunchScriptUtilities.hpp
 *
 *  Created on: June 2, 2020
 */

#pragma once

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{
  size_t compute_number_of_nodes_needed(const size_t& aNumProcessorsNeeded, const size_t& aNumProcessorsPerNode);
  void generate_batch_script
    (const size_t& aNumPerformers,
     const size_t& aNumProcessorsPerNode = 6/*num gpus per node on summit*/);
  void generate_jsrun_script(const size_t& aNumPerformers);
  void generate_engine_bash_script();
  void generate_analyze_bash_script();
  void generate_summit_launch_scripts(const XMLGen::InputData& aInputData);
  std::string get_num_procs(const XMLGen::Objective& aObjective);
  std::string get_num_opt_processors(const XMLGen::InputData& aInputData);
  void determine_mpi_env_and_separation_strings(std::string& envString, std::string& separationString);
  void determine_mpi_launch_strings(const XMLGen::InputData& aInputData, std::string& aLaunchString, std::string& aNumProcsString);
  void append_decomp_line(FILE*& fp, const std::string& num_processors, const std::string& mesh_file_name);
  void append_decomp_lines_for_optimizer(const XMLGen::InputData& aInputData,
                                              FILE*& fp,
                                              std::map<std::string,int>& hasBeenDecompedForThisNumberOfProcessors);
  void append_decomp_lines_for_performers(const XMLGen::InputData& aInputData, FILE*& fp,
                                          std::map<std::string,int>& hasBeenDecompedForThisNumberOfProcessors);
  void append_decomp_lines_to_mpirun_launch_script(const XMLGen::InputData& aInputData, FILE*& fp);

  void generate_mpirun_launch_script(const XMLGen::InputData& aInputData);
  void append_prune_and_refine_lines_to_mpirun_launch_script(const XMLGen::InputData& aInputData, FILE*& fp);
  void append_mpirun_call(const XMLGen::InputData& aInputData, FILE*& fp,
                          const std::string& envString, const std::string& separationString,
                          const std::string& tLaunchString, const std::string& tNumProcsString);
}
// namespace XMLGen
