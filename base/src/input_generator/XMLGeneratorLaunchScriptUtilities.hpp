#pragma once

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{
  void determine_plato_engine_name(const XMLGen::InputData& aInputData, std::string& aPlatoEngineName);
  void generate_batch_script(const size_t& aNumPerformers,const size_t& aNumProcessorsPerNode = 6/*num gpus per node on summit*/);
  void generate_jsrun_script(const size_t& aNumPerformers, const std::string& performerName);
  void generate_engine_bash_script();
  void append_decomp_line(FILE*& fp, const std::string& num_processors, const std::string& mesh_file_name);
  void append_decomp_line(FILE*& fp, const int& num_processors, const std::string& mesh_file_name);
  void append_decomp_lines_for_optimizer(const XMLGen::InputData& aInputData,
      FILE*& fp,
      std::map<std::string,int>& hasBeenDecompedForThisNumberOfProcessors);
  void append_decomp_lines_for_performers(const XMLGen::InputData& aInputData, FILE*& fp,
      std::map<std::string,int>& hasBeenDecompedForThisNumberOfProcessors);
  void append_decomp_lines_for_prune_and_refine(const XMLGen::InputData& aInputData, FILE*& fp);
  void append_decomp_lines_to_mpirun_launch_script(const XMLGen::InputData& aInputData, FILE*& fp);
  void append_engine_mpirun_lines(const XMLGen::InputData& aInputData, int &aNextPerformerID, FILE*& fp);
  void append_concatenate_mesh_file_lines(const XMLGen::InputData& aInputData, FILE*& fp);
  void append_prune_and_refine_command(const XMLGen::InputData& aInputData, FILE*& fp);
  void append_prune_and_refine_lines_to_mpirun_launch_script(const XMLGen::InputData& aInputData, FILE*& fp);
  void determine_mpi_env_and_separation_strings(std::string& envString, std::string& separationString);
  void determine_mpi_launch_strings(const XMLGen::InputData& aInputData, std::string& aLaunchString, std::string& aNumProcsString);
  
  namespace Internal
  {
    int get_number_of_refines(const XMLGen::InputData& aInputData);
    int get_max_number_of_objective_procs(const XMLGen::InputData& aInputData);
    int get_number_of_prune_and_refine_procs(const XMLGen::InputData& aInputData);
    std::string get_num_opt_processors(const XMLGen::InputData& aInputData);
    std::string get_num_buffer_layers(const XMLGen::InputData& aInputData);
    std::string get_prune_string(const XMLGen::InputData& aInputData);
    std::string get_extension_string(const std::string& tNumberPruneAndRefineProcsString);
    std::string get_prune_and_refine_executable_path(const XMLGen::InputData& aInputData);
    size_t compute_number_of_nodes_needed(const size_t& aNumProcessorsNeeded, const size_t& aNumProcessorsPerNode);
  }
}
// namespace XMLGen
