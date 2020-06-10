#include "pugixml.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorLaunchScriptUtilities.hpp"
#include <iostream>

namespace XMLGen
{
  void generate_batch_script
    (const size_t& aNumPerformers,
     const size_t& aNumProcessorsPerNode)
  {
    std::ofstream batchFile;
    batchFile.open ("plato.batch");
    batchFile << "#!/bin/bash\n";
    batchFile << "# LSF Directives\n";
    batchFile << "#BSUB -P <PROJECT>\n";
    batchFile << "#BSUB -W 0:00\n";

    size_t tNumNodesNeeded = XMLGen::Internal::compute_number_of_nodes_needed(aNumPerformers,aNumProcessorsPerNode);

    batchFile << "#BSUB -nnodes " << tNumNodesNeeded << "\n";
    batchFile << "#BSUB -J plato\n";
    batchFile << "#BSUB -o plato.%J\n";
    batchFile << "\n";
    batchFile << "cd <path/to/working/directory>\n";
    batchFile << "date\n";
    batchFile << "jsrun -A eng -n1 -a1 -c1 -g0\n";
    batchFile << "jsrun -A per -n" << aNumPerformers << " -a1 -c1 -g1\n";
    batchFile << "jsrun -f jsrun.source\n";
    
    batchFile.close();
  }

  void generate_jsrun_script
    (const size_t& aNumPerformers, const std::string& performerName)
  {
    std::ofstream jsrun;
    jsrun.open("jsrun.source");
    jsrun << "1 : eng : bash engine.sh\n";
    jsrun << aNumPerformers << " : per : bash " << performerName << ".sh\n";
    jsrun.close();
  }

  void generate_engine_bash_script()
  {
    std::ofstream engineBash;
    engineBash.open("engine.sh");
    engineBash << "export PLATO_PERFORMER_ID=0\n";
    engineBash << "export PLATO_INTERFACE_FILE=interface.xml\n";
    engineBash << "export PLATO_APP_FILE=plato_main_operations.xml\n";
    engineBash << "\n";
    engineBash << "PlatoMain plato_main_input_deck.xml";
    engineBash.close();
  }

  void append_prune_and_refine_lines_to_mpirun_launch_script(const XMLGen::InputData& aInputData, FILE*& fp)
  {
    int tNumRefines = XMLGen::Internal::get_number_of_refines(aInputData);

    bool need_to_transfer_prune_or_refine = tNumRefines > 0 || aInputData.initial_guess_filename != ""
                                                          && aInputData.initial_guess_field_name != "";
    if(need_to_transfer_prune_or_refine)
    {
      XMLGen::append_prune_and_refine_command(aInputData, fp);
      XMLGen::append_concatenate_mesh_file_lines(aInputData,fp);
    }
  }

  void append_prune_and_refine_command(const XMLGen::InputData& aInputData, FILE*& fp)
  {
    if(aInputData.mesh_name.empty())
      THROWERR("No mesh name provided");
    if(aInputData.run_mesh_name.empty())
      THROWERR("No output mesh name provided");

    std::string tPruneString = XMLGen::Internal::get_prune_string(aInputData);
    std::string tNumRefinesString = Plato::to_string(XMLGen::Internal::get_number_of_refines(aInputData));
    std::string tNumBufferLayersString = XMLGen::Internal::get_num_buffer_layers(aInputData);
    std::string tNumberPruneAndRefineProcsString = Plato::to_string(XMLGen::Internal::get_number_of_prune_and_refine_procs(aInputData));
    std::string tPruneAndRefineExe = XMLGen::Internal::get_prune_and_refine_executable_path(aInputData);

    std::string tCommand;
    if(aInputData.m_UseLaunch)
      tCommand = "launch -n " + tNumberPruneAndRefineProcsString + " " + tPruneAndRefineExe;
    else
      tCommand = "mpiexec -np " + tNumberPruneAndRefineProcsString + " " + tPruneAndRefineExe;
    if(aInputData.initial_guess_filename != "")
      tCommand += (" --mesh_with_variable=" + aInputData.initial_guess_filename);
    tCommand += (" --mesh_to_be_pruned=" + aInputData.mesh_name);
    tCommand += (" --result_mesh=" + aInputData.run_mesh_name);
    if(aInputData.initial_guess_field_name != "")
      tCommand += (" --field_name=" + aInputData.initial_guess_field_name);
    tCommand += (" --number_of_refines=" + tNumRefinesString);
    tCommand += (" --number_of_buffer_layers=" + tNumBufferLayersString);
    tCommand += (" --prune_mesh=" + tPruneString);

    fprintf(fp, "%s\n", tCommand.c_str());
  }

  void append_concatenate_mesh_file_lines(const XMLGen::InputData& aInputData, FILE*& fp)
  {
    int tNumberPruneAndRefineProcs = XMLGen::Internal::get_number_of_prune_and_refine_procs(aInputData);
    std::string tNumberPruneAndRefineProcsString = Plato::to_string(tNumberPruneAndRefineProcs);

    if(tNumberPruneAndRefineProcs > 1)
    {
      if(aInputData.run_mesh_name == "")
        THROWERR("run_mesh_name not set")

          std::string tExtensionString = XMLGen::Internal::get_extension_string(tNumberPruneAndRefineProcsString);
      fprintf(fp, "epu -auto %s%s\n", aInputData.run_mesh_name.c_str(), tExtensionString.c_str());
    }
  }

  void append_decomp_lines_for_prune_and_refine(const XMLGen::InputData& aInputData, FILE*& fp)
  {
    int tNumRefines = XMLGen::Internal::get_number_of_refines(aInputData);
    bool need_to_transfer_prune_or_refine = tNumRefines > 0 || aInputData.initial_guess_filename != "" 
                                                          && aInputData.initial_guess_field_name != "";
    if(need_to_transfer_prune_or_refine)
    {
      int tNumberPruneAndRefineProcs = XMLGen::Internal::get_number_of_prune_and_refine_procs(aInputData);
      if(tNumberPruneAndRefineProcs > 1)
      {
        if(aInputData.mesh_name.empty())
          THROWERR("Missing input mesh name\n")
        XMLGen::append_decomp_line(fp,tNumberPruneAndRefineProcs,aInputData.mesh_name);
        if(aInputData.initial_guess_filename != "")
          XMLGen::append_decomp_line(fp,tNumberPruneAndRefineProcs,aInputData.initial_guess_filename);
      }
    }
  }

  void append_decomp_lines_to_mpirun_launch_script(const XMLGen::InputData& aInputData, FILE*& fp)
  {
    std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;

    XMLGen::append_decomp_lines_for_optimizer(aInputData, fp, hasBeenDecompedForThisNumberOfProcessors);
    XMLGen::append_decomp_lines_for_performers(aInputData, fp, hasBeenDecompedForThisNumberOfProcessors);
    XMLGen::append_decomp_lines_for_prune_and_refine(aInputData, fp);
  }

  void append_decomp_lines_for_optimizer(const XMLGen::InputData& aInputData,
                                              FILE*& fp,
                                              std::map<std::string,int>& hasBeenDecompedForThisNumberOfProcessors)
  {
    if(aInputData.run_mesh_name.empty())
      THROWERR("Cannot add decomp line: Mesh name not provided\n")

    std::string num_opt_procs = XMLGen::Internal::get_num_opt_processors(aInputData);
    XMLGen::assert_is_positive_integer(num_opt_procs);
    
    bool need_to_decompose = num_opt_procs.compare("1") != 0;
    if(need_to_decompose)
    {
      if(hasBeenDecompedForThisNumberOfProcessors[num_opt_procs]++ == 0)
        XMLGen::append_decomp_line(fp, num_opt_procs, aInputData.run_mesh_name);
      if(aInputData.initial_guess_filename != "")
        XMLGen::append_decomp_line(fp, num_opt_procs, aInputData.initial_guess_filename);
    }
  }

  void append_decomp_lines_for_performers(const XMLGen::InputData& aInputData, FILE*& fp,
                                          std::map<std::string,int>& hasBeenDecompedForThisNumberOfProcessors)
  {
    if(aInputData.run_mesh_name.empty())
      THROWERR("Cannot add decomp line: Mesh name not provided\n")

    for(size_t i=0; i<aInputData.objectives.size(); ++i)
    {
        std::string num_procs = XMLGen::Internal::get_num_procs(aInputData.objectives[i]);

        XMLGen::assert_is_positive_integer(num_procs);

        bool need_to_decompose = num_procs.compare("1") != 0;
        if(need_to_decompose)
        {
            if(hasBeenDecompedForThisNumberOfProcessors[num_procs]++ == 0)
              XMLGen::append_decomp_line(fp, num_procs, aInputData.run_mesh_name);
            if(aInputData.objectives[i].ref_frf_file.length() > 0)
              XMLGen::append_decomp_line(fp, num_procs, aInputData.objectives[i].ref_frf_file);
        }
    }
  }

  void append_decomp_line(FILE*& fp, const std::string& num_processors, const std::string& mesh_file_name)
  {
    fprintf(fp, "decomp -p %s %s\n", num_processors.c_str(), mesh_file_name.c_str());
  }

  void append_decomp_line(FILE*& fp, const int& num_processors, const std::string& mesh_file_name)
  {
    fprintf(fp, "decomp -p %d %s\n", num_processors, mesh_file_name.c_str());
  }

  void append_engine_mpirun_lines(const XMLGen::InputData& aInputData, FILE*& fp)
  {
    std::string envString, separationString, tLaunchString, tNumProcsString;

    XMLGen::determine_mpi_env_and_separation_strings(envString,separationString);
    XMLGen::determine_mpi_launch_strings(aInputData,tLaunchString,tNumProcsString);

    std::string num_opt_procs = XMLGen::Internal::get_num_opt_processors(aInputData);
    XMLGen::assert_is_positive_integer(num_opt_procs);

    // Now add the main mpirun call.
    fprintf(fp, "%s %s %s %s PLATO_PERFORMER_ID%s0 \\\n", tLaunchString.c_str(), 
                                tNumProcsString.c_str(), num_opt_procs.c_str(),
                                envString.c_str(),separationString.c_str());
    fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
    fprintf(fp, "%s PLATO_APP_FILE%splato_main_operations.xml \\\n", envString.c_str(),separationString.c_str());
    if(aInputData.plato_main_path.length() != 0)
      fprintf(fp, "%s plato_main_input_deck.xml \\\n", aInputData.plato_main_path.c_str());
    else
      fprintf(fp, "plato_main plato_main_input_deck.xml \\\n");
  }

  void determine_mpi_env_and_separation_strings(std::string& envString, std::string& separationString)
  {
#ifndef USING_OPEN_MPI
    envString = "-env";
    separationString = " ";
#else
    envString = "-x";
    separationString = "=";
#endif
  }

  void determine_mpi_launch_strings(const XMLGen::InputData& aInputData, std::string& aLaunchString, std::string& aNumProcsString)
  {
    if(aInputData.m_UseLaunch)
    {
      aLaunchString = "launch";
      aNumProcsString = "-n";
    }
    else
    {
      aLaunchString = "mpiexec";
      aNumProcsString = "-np";
    }
  }

  namespace Internal
  {
    int get_number_of_refines(const XMLGen::InputData& aInputData)
    {
      int tNumRefines = 0;
      if(aInputData.number_refines != "" && aInputData.number_refines != "0")
      {
        XMLGen::assert_is_positive_integer(aInputData.number_refines);
        tNumRefines = std::atoi(aInputData.number_refines.c_str());
      }
      return tNumRefines;
    }

    int get_max_number_of_objective_procs(const XMLGen::InputData& aInputData)
    {
      int tNumberPruneAndRefineProcs = 1;
      for(size_t i=0; i<aInputData.objectives.size(); ++i)
      {
          if(!aInputData.objectives[i].num_procs.empty())
          {
              XMLGen::assert_is_positive_integer(aInputData.objectives[i].num_procs);
              int tNumProcs = std::atoi(aInputData.objectives[i].num_procs.c_str());
              if(tNumProcs > tNumberPruneAndRefineProcs)
                tNumberPruneAndRefineProcs = tNumProcs;
          }
      }

      return tNumberPruneAndRefineProcs;
    }

    int get_number_of_prune_and_refine_procs(const XMLGen::InputData& aInputData)
    {
      int tNumberPruneAndRefineProcs = 1;
      if(aInputData.number_prune_and_refine_processors != "" &&
              aInputData.number_prune_and_refine_processors != "0")
      {
          std::string tNumberPruneAndRefineProcsString = aInputData.number_prune_and_refine_processors;
          XMLGen::assert_is_positive_integer(tNumberPruneAndRefineProcsString);
          tNumberPruneAndRefineProcs = std::atoi(tNumberPruneAndRefineProcsString.c_str());
      }
      else
        tNumberPruneAndRefineProcs = XMLGen::Internal::get_max_number_of_objective_procs(aInputData);

      return tNumberPruneAndRefineProcs;
    }

    std::string get_num_procs(const XMLGen::Objective& aObjective)
    {
      std::string num_procs = "4";
      if(!aObjective.num_procs.empty())
          num_procs = aObjective.num_procs;
      return num_procs;
    }

    std::string get_num_opt_processors(const XMLGen::InputData& aInputData)
    {
      std::string num_opt_procs = "1";
      if(!aInputData.num_opt_processors.empty())
          num_opt_procs = aInputData.num_opt_processors;
      return num_opt_procs;
    }

    std::string get_num_buffer_layers(const XMLGen::InputData& aInputData)
    {
      std::string tNumBufferLayersString = "2";
      if(aInputData.number_buffer_layers != "")
        tNumBufferLayersString = aInputData.number_buffer_layers;
      if(tNumBufferLayersString != "0")
        XMLGen::assert_is_positive_integer(tNumBufferLayersString);
      return tNumBufferLayersString;
    }

    std::string get_prune_string(const XMLGen::InputData& aInputData)
    {
      std::string tPruneString = "0";
      if(aInputData.prune_mesh == "true")
        tPruneString = "1";
      return tPruneString;
    }

    std::string get_extension_string(const std::string& tNumberPruneAndRefineProcsString)
    {
      std::string tExtensionString = "." + tNumberPruneAndRefineProcsString + ".";
      for(size_t g=0; g<tNumberPruneAndRefineProcsString.length(); ++g)
        tExtensionString += "0";
      return tExtensionString;
    }

    std::string get_prune_and_refine_executable_path(const XMLGen::InputData& aInputData)
    {
      std::string tPruneAndRefineExe = "prune_and_refine";
      if(aInputData.prune_and_refine_path.length() > 0)
        tPruneAndRefineExe = aInputData.prune_and_refine_path;
      return tPruneAndRefineExe;
    }

    size_t compute_number_of_nodes_needed(const size_t& aNumProcessorsNeeded, const size_t& aNumProcessorsPerNode)
    {
      if(aNumProcessorsPerNode == 0)
        THROWERR("Error: zero processors per node specified")
      size_t tNumNodesNeeded = aNumProcessorsNeeded/aNumProcessorsPerNode;
      if(aNumProcessorsNeeded % aNumProcessorsPerNode != 0)
        ++tNumNodesNeeded;
      return tNumNodesNeeded;
    }
  }
}
// namespace XMLGen