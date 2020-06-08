/*
 * XMLGeneratorLaunchScriptUtilities.cpp
 *
 *  Created on: June 2, 2020
 */

#include "pugixml.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorLaunchScriptUtilities.hpp"
#include <iostream>

namespace XMLGen
{
  size_t compute_number_of_nodes_needed(const size_t& aNumProcessorsNeeded, const size_t& aNumProcessorsPerNode)
  {
    if(aNumProcessorsPerNode == 0)
      THROWERR("Error: zero processors per node specified")
    size_t tNumNodesNeeded = aNumProcessorsNeeded/aNumProcessorsPerNode;
    if(aNumProcessorsNeeded % aNumProcessorsPerNode != 0)
      ++tNumNodesNeeded;
    return tNumNodesNeeded;
  }

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

    size_t tNumNodesNeeded = XMLGen::compute_number_of_nodes_needed(aNumPerformers,aNumProcessorsPerNode);

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
    (const size_t& aNumPerformers)
  {
    std::ofstream jsrun;
    jsrun.open("jsrun.source");
    jsrun << "1 : eng : bash engine.sh\n";
    jsrun << aNumPerformers << " : per : bash analyze.sh\n";
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

  void generate_analyze_bash_script()
  {
    std::ofstream analyzeBash;
    std::string filename = "analyze.sh";
    analyzeBash.open(filename);
    analyzeBash << "export PLATO_PERFORMER_ID=1\n";
    analyzeBash << "export PLATO_INTERFACE_FILE=interface.xml\n";
    analyzeBash << "export PLATO_APP_FILE=plato_analyze_operations.xml\n";
    analyzeBash << "\n";
    analyzeBash << "analyze_MPMD --input-config=plato_analyze_input_deck.xml\n";

    analyzeBash.close();
  }

  void generate_summit_launch_scripts(const XMLGen::InputData& aInputData)
  {
    for(auto objective:aInputData.objectives)
    {
      if(objective.code_name != "plato_analyze")
        THROWERR("ERROR: Summit output is only supported for Plato Analyze performers")
    }

    size_t tNumPerformers = aInputData.m_UncertaintyMetaData.numPerformers;

    XMLGen::generate_engine_bash_script();
    XMLGen::generate_analyze_bash_script();
    XMLGen::generate_jsrun_script(tNumPerformers);
    XMLGen::generate_batch_script(tNumPerformers);
  }

  void generate_mpirun_launch_script(const XMLGen::InputData& aInputData)
  {
      FILE *fp=fopen("mpirun.source", "w");

      std::string tEnvString;
      std::string tSeparationString;
      XMLGen::determine_mpi_env_and_separation_strings(tEnvString, tSeparationString);

      std::string tLaunchString;
      std::string tNumProcsString;
      XMLGen::determine_mpi_launch_strings(aInputData, tLaunchString, tNumProcsString);

      XMLGen::append_prune_and_refine_lines_to_mpirun_launch_script(aInputData, fp);
      XMLGen::append_decomp_lines_to_mpirun_launch_script(aInputData, fp);
      XMLGen::append_mpirun_call(aInputData, fp, tEnvString, tSeparationString, tLaunchString, tNumProcsString);

      fclose(fp);
  }

  void append_prune_and_refine_lines_to_mpirun_launch_script(const XMLGen::InputData& aInputData, FILE*& fp)
  {
    // For restarts where we need to call prune_and_refine to do a variable transfer, prune, refine, or
    // any combination of the above we need to add an mpirun call for this first so it will run as
    // a pre-processor on the input mesh.
    int tNumRefines = 0;
    if(aInputData.number_refines != "")
        tNumRefines = std::atoi(aInputData.number_refines.c_str());
    if(tNumRefines > 0 ||
            (aInputData.initial_guess_filename != "" && aInputData.initial_guess_field_name != ""))
    {
        // Determine how many processors to use for the prune_and_refine run.
        std::string tNumberPruneAndRefineProcsString = "1";
        int tNumberPruneAndRefineProcs = 1;
        if(aInputData.number_prune_and_refine_processors != "" &&
                aInputData.number_prune_and_refine_processors != "0")
        {
            tNumberPruneAndRefineProcsString = aInputData.number_prune_and_refine_processors;
            tNumberPruneAndRefineProcs = std::atoi(tNumberPruneAndRefineProcsString.c_str());
        }
        else
        {
            // Find the max number of objective procs.
            for(size_t i=0; i<aInputData.objectives.size(); ++i)
            {
                if(!aInputData.objectives[i].num_procs.empty())
                {
                    int tNumProcs = std::atoi(aInputData.objectives[i].num_procs.c_str());
                    if(tNumProcs > tNumberPruneAndRefineProcs)
                    {
                        tNumberPruneAndRefineProcsString = aInputData.objectives[i].num_procs;
                        tNumberPruneAndRefineProcs = tNumProcs;
                    }
                }
            }
        }

        // First decompose the files that will be involved below
        if(tNumberPruneAndRefineProcs > 1)
        {
            fprintf(fp, "decomp -p %d %s\n", tNumberPruneAndRefineProcs, aInputData.mesh_name.c_str());
            if(aInputData.initial_guess_filename != "")
                fprintf(fp, "decomp -p %d %s\n", tNumberPruneAndRefineProcs, aInputData.initial_guess_filename.c_str());
        }

        std::string tPruneString = "0";
        std::string tNumRefinesString = "0";
        std::string tNumBufferLayersString = "2";
        if(aInputData.prune_mesh == "true")
            tPruneString = "1";
        if(aInputData.number_refines != "")
            tNumRefinesString = aInputData.number_refines;
        if(aInputData.number_buffer_layers != "")
            tNumBufferLayersString = aInputData.number_buffer_layers;

        std::string tCommand;
        std::string tPruneAndRefineExe = "prune_and_refine";
        if(aInputData.prune_and_refine_path.length() > 0)
            tPruneAndRefineExe = aInputData.prune_and_refine_path;
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

        // Now concatenate the input mesh file again since we don't know what other decompositions will be needed.
        if(tNumberPruneAndRefineProcs > 1)
        {
            // Build the extension string we will need.
            std::string tExtensionString = "." + tNumberPruneAndRefineProcsString + ".";
            for(size_t g=0; g<tNumberPruneAndRefineProcsString.length(); ++g)
                tExtensionString += "0";
            fprintf(fp, "epu -auto %s%s\n", aInputData.run_mesh_name.c_str(), tExtensionString.c_str());
        }
    }
  }

  void append_decomp_lines_to_mpirun_launch_script(const XMLGen::InputData& aInputData, FILE*& fp)
  {
    std::map<std::string,int> hasBeenDecompedForThisNumberOfProcessors;

    XMLGen::append_decomp_lines_for_optimizer(aInputData, fp, hasBeenDecompedForThisNumberOfProcessors);
    XMLGen::append_decomp_lines_for_performers(aInputData, fp, hasBeenDecompedForThisNumberOfProcessors);
  }

  void append_decomp_lines_for_optimizer(const XMLGen::InputData& aInputData,
                                              FILE*& fp,
                                              std::map<std::string,int>& hasBeenDecompedForThisNumberOfProcessors)
  {
    if(aInputData.run_mesh_name.empty())
      THROWERR("Cannot add decomp line: Mesh name not provided\n")

    std::string num_opt_procs = get_num_opt_processors(aInputData);
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
        std::string num_procs = XMLGen::get_num_procs(aInputData.objectives[i]);

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

  void append_mpirun_call(const XMLGen::InputData& aInputData, FILE*& fp,
                          const std::string& envString, const std::string& separationString,
                          const std::string& tLaunchString, const std::string& tNumProcsString)
  {
      std::string num_opt_procs = XMLGen::get_num_opt_processors(aInputData);

      XMLGen::assert_is_positive_integer(num_opt_procs);

    // Now add the main mpirun call.
    fprintf(fp, "%s %s %s %s PLATO_PERFORMER_ID%s0 \\\n", tLaunchString.c_str(), tNumProcsString.c_str(), num_opt_procs.c_str(), envString.c_str(),separationString.c_str());
    fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
    fprintf(fp, "%s PLATO_APP_FILE%splato_main_operations.xml \\\n", envString.c_str(),separationString.c_str());
    if(aInputData.plato_main_path.length() != 0)
        fprintf(fp, "%s plato_main_input_deck.xml \\\n", aInputData.plato_main_path.c_str());
    else
        fprintf(fp, "plato_main plato_main_input_deck.xml \\\n");

    fprintf(fp, ": %s %s %s PLATO_PERFORMER_ID%s1 \\\n", tNumProcsString.c_str(), Plato::to_string(aInputData.m_UncertaintyMetaData.numPerformers).c_str(), envString.c_str(),separationString.c_str());
    fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
    fprintf(fp, "%s PLATO_APP_FILE%splato_analyze_operations.xml \\\n", envString.c_str(),separationString.c_str());
    if(aInputData.plato_analyze_path.length() != 0)
        fprintf(fp, "%s --input-config=plato_analyze_input_deck.xml \\\n", aInputData.plato_analyze_path.c_str());
    else
        fprintf(fp, "analyze_MPMD --input-config=plato_analyze_input_deck.xml \\\n");
  }

}
// namespace XMLGen
