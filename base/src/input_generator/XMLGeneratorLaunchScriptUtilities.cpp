/*
 * XMLGeneratorLaunchScriptUtilities.cpp
 *
 *  Created on: June 2, 2020
 */

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorLaunchScriptUtilities.hpp"

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

      XMLGen::append_prune_and_refine_lines_to_mpirun_launch_script(aInputData, fp);

      std::string num_opt_procs = "1";
      if(!aInputData.num_opt_processors.empty())
          num_opt_procs = aInputData.num_opt_processors;

      XMLGen::append_decomp_lines_to_mpirun_launch_script(aInputData, fp, num_opt_procs);

#ifndef USING_OPEN_MPI
      std::string envString = "-env";
      std::string separationString = " ";
#else
      std::string envString = "-x";
      std::string separationString = "=";
#endif

      std::string tLaunchString = "";
      std::string tNumProcsString = "";
      if(aInputData.m_UseLaunch)
      {
          tLaunchString = "launch";
          tNumProcsString = "-n";
      }
      else
      {
          tLaunchString = "mpiexec";
          tNumProcsString = "-np";
      }

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

  void append_decomp_lines_to_mpirun_launch_script(const XMLGen::InputData& aInputData, FILE*& fp, std::string& num_opt_procs)
  {
    // remember if the run_mesh has been decomposed to this processor count
    std::map<std::string,int> hasBeenDecompedToThisCount;

    // Now do the decomps for the TO run.

    if(num_opt_procs.compare("1") != 0) {
        if(++hasBeenDecompedToThisCount[num_opt_procs] == 1)
        {
            fprintf(fp, "decomp -p %s %s\n", num_opt_procs.c_str(), aInputData.run_mesh_name.c_str());
        }
    }
    if(aInputData.initial_guess_filename != "" && num_opt_procs.compare("1") != 0)
        fprintf(fp, "decomp -p %s %s\n", num_opt_procs.c_str(), aInputData.initial_guess_filename.c_str());
    for(size_t i=0; i<aInputData.objectives.size(); ++i)
    {
        std::string num_procs = "4";
        if(!aInputData.objectives[i].num_procs.empty())
            num_procs = aInputData.objectives[i].num_procs;
        if(num_procs.compare("1") != 0)
        {
            if(++hasBeenDecompedToThisCount[num_procs] == 1)
            {
                fprintf(fp, "decomp -p %s %s\n", num_procs.c_str(), aInputData.run_mesh_name.c_str());
            }
            if(aInputData.objectives[i].ref_frf_file.length() > 0)
                fprintf(fp, "decomp -p %s %s\n", num_procs.c_str(), aInputData.objectives[i].ref_frf_file.c_str());
        }
    }
  }
}
// namespace XMLGen
