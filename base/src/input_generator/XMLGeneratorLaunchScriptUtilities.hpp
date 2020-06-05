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
  void generate_mpirun_launch_script(const XMLGen::InputData& aInputData);
  void append_prune_and_refine_lines_to_mpirun_launch_script(const XMLGen::InputData& aInputData, FILE*& fp);
  void append_decomp_lines_to_mpirun_launch_script(const XMLGen::InputData& aInputData, FILE*& fp, std::string& num_opt_procs);
}
// namespace XMLGen
