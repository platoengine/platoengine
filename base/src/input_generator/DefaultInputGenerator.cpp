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
 * DefaultInputGenerator.cpp
 *
 *  Created on: Apr 16, 2020
 *
 */

#include <set>
#include <string>
#include <iostream>
#include <iostream>
#include <fstream>
#include <cstring>
#include <cassert>
#include <utility>
#include <string>
#include <map>
#include "DefaultInputGenerator.hpp"
#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorLaunchScriptUtilities.hpp"
#include "Plato_SolveUncertaintyProblem.hpp"
#include "Plato_UniqueCounter.hpp"
#include "Plato_Vector3DVariations.hpp"
#include "PlatoAnalyzeInputDeckWriter.hpp"
#include "SalinasInputDeckWriter.hpp"
#include "Plato_FreeFunctions.hpp"
#include "XMLG_Macros.hpp"

namespace XMLGen
{

/******************************************************************************/
DefaultInputGenerator::DefaultInputGenerator(const InputData& aInputData) :
m_InputData(aInputData)
/******************************************************************************/
{
}

/******************************************************************************/
DefaultInputGenerator::~DefaultInputGenerator() 
/******************************************************************************/
{
}

/******************************************************************************/
bool DefaultInputGenerator::generateInputFiles()
/******************************************************************************/
{
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow)
    {

      if(!generateDefinesXML())
      {
        std::cout << "Failed to generate defines.xml" << std::endl;
        return false;
      }
    }

    if(m_InputData.optimization_type == "shape" && m_InputData.mPlatoAnalyzePerformerExists)
    {
        if(!generatePlatoAnalyzeShapeDefinesXML())
        {
            std::cout << "Failed to generate defines.xml" << std::endl;
            return false;
        }

        if(!generatePlatoAnalyzeShapeInterfaceXML())
        {
            std::cout << "Failed to generate interface.xml" << std::endl;
            return false;
        }
    }
    else
    {
        if(!generateInterfaceXML())
        {
            std::cout << "Failed to generate interface.xml" << std::endl;
            return false;
        }
    }

    if(m_InputData.optimization_type == "shape" && m_InputData.mPlatoAnalyzePerformerExists)
    {
        if(!generatePlatoMainOperationsXMLForShape())
        {
            std::cout << "Failed to generate plato_main_operations.xml" << std::endl;
            return false;
        }
    }
    else
    {
        if(!generatePlatoMainOperationsXML())
        {
            std::cout << "Failed to generate plato_main_operations.xml" << std::endl;
            return false;
        }
    }

    if(!generatePlatoMainInputDeckXML())
    {
        std::cout << "Failed to generate plato_main_input_deck.xml" << std::endl;
        return false;
    }

    if(!generatePerformerOperationsXML())
    {
        std::cout << "Failed to generate performer_operations.xml" << std::endl;
        return false;
    }

    if(!generatePhysicsInputDecks())
    {
        std::cout << "Failed to generate physics input decks" << std::endl;
        return false;
    }

    if(m_InputData.optimization_type == "shape" && m_InputData.mPlatoAnalyzePerformerExists)
    {
        if(!generatePlatoESPInputDeckXML())
        {
            std::cout << "Failed to generate plato_esp_input_deck.xml" << std::endl;
            return false;
        }
    }

    if(m_InputData.optimization_type == "shape" && m_InputData.mPlatoAnalyzePerformerExists)
    {
        if(!generatePlatoAnalyzeShapeLaunchScript())
        {
            std::cout << "Failed to generate mpirun.source" << std::endl;
            return false;
        }
    }
    else
    {
        if(!generateLaunchScript())
        {
            std::cout << "Failed to generate mpirun.source" << std::endl;
            return false;
        }
    }

    std::cout << "Successfully wrote XML files." << std::endl;
    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generateLaunchScript()
/******************************************************************************/
{
    if(m_InputData.m_Arch == XMLGen::Arch::SUMMIT)
      generateSummitLaunchScripts();
    else
    {
      FILE *fp=fopen("mpirun.source", "w");
      if(!fp)
      {
          return true;
      }

      std::string num_opt_procs = "1";
      if(!m_InputData.num_opt_processors.empty())
          num_opt_procs = m_InputData.num_opt_processors;

      // For restarts where we need to call prune_and_refine to do a variable transfer, prune, refine, or
      // any combination of the above we need to add an mpirun call for this first so it will run as
      // a pre-processor on the input mesh.
      int tNumRefines = 0;
      if(m_InputData.number_refines != "")
          tNumRefines = std::atoi(m_InputData.number_refines.c_str());
      if(tNumRefines > 0 ||
              (m_InputData.initial_guess_filename != "" && m_InputData.initial_guess_field_name != ""))
      {
          // Determine how many processors to use for the prune_and_refine run.
          std::string tNumberPruneAndRefineProcsString = "1";
          int tNumberPruneAndRefineProcs = 1;
          if(m_InputData.number_prune_and_refine_processors != "" &&
                  m_InputData.number_prune_and_refine_processors != "0")
          {
              tNumberPruneAndRefineProcsString = m_InputData.number_prune_and_refine_processors;
              tNumberPruneAndRefineProcs = std::atoi(tNumberPruneAndRefineProcsString.c_str());
          }
          else
          {
              // Find the max number of objective procs.
              for(size_t i=0; i<m_InputData.objectives.size(); ++i)
              {
                  if(!m_InputData.objectives[i].num_procs.empty())
                  {
                      int tNumProcs = std::atoi(m_InputData.objectives[i].num_procs.c_str());
                      if(tNumProcs > tNumberPruneAndRefineProcs)
                      {
                          tNumberPruneAndRefineProcsString = m_InputData.objectives[i].num_procs;
                          tNumberPruneAndRefineProcs = tNumProcs;
                      }
                  }
              }
          }

          // First decompose the files that will be involved below
          if(tNumberPruneAndRefineProcs > 1)
          {
              fprintf(fp, "decomp -p %d %s\n", tNumberPruneAndRefineProcs, m_InputData.mesh_name.c_str());
              if(m_InputData.initial_guess_filename != "")
                  fprintf(fp, "decomp -p %d %s\n", tNumberPruneAndRefineProcs, m_InputData.initial_guess_filename.c_str());
          }

          std::string tPruneString = "0";
          std::string tNumRefinesString = "0";
          std::string tNumBufferLayersString = "2";
          if(m_InputData.prune_mesh == "true")
              tPruneString = "1";
          if(m_InputData.number_refines != "")
              tNumRefinesString = m_InputData.number_refines;
          if(m_InputData.number_buffer_layers != "")
              tNumBufferLayersString = m_InputData.number_buffer_layers;

          std::string tCommand;
          std::string tPruneAndRefineExe = "prune_and_refine";
          if(m_InputData.prune_and_refine_path.length() > 0)
              tPruneAndRefineExe = m_InputData.prune_and_refine_path;
          if(m_InputData.m_UseLaunch)
              tCommand = "launch -n " + tNumberPruneAndRefineProcsString + " " + tPruneAndRefineExe;
          else
              tCommand = "mpiexec -np " + tNumberPruneAndRefineProcsString + " " + tPruneAndRefineExe;
          if(m_InputData.initial_guess_filename != "")
              tCommand += (" --mesh_with_variable=" + m_InputData.initial_guess_filename);
          tCommand += (" --mesh_to_be_pruned=" + m_InputData.mesh_name);
          tCommand += (" --result_mesh=" + m_InputData.run_mesh_name);
          if(m_InputData.initial_guess_field_name != "")
              tCommand += (" --field_name=" + m_InputData.initial_guess_field_name);
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
              fprintf(fp, "epu -auto %s%s\n", m_InputData.run_mesh_name.c_str(), tExtensionString.c_str());
          }
      }

      // remember if the run_mesh has been decomposed to this processor count
      std::map<std::string,int> hasBeenDecompedToThisCount;

      // Now do the decomps for the TO run.
      if(num_opt_procs.compare("1") != 0) {
          if(++hasBeenDecompedToThisCount[num_opt_procs] == 1)
          {
              fprintf(fp, "decomp -p %s %s\n", num_opt_procs.c_str(), m_InputData.run_mesh_name.c_str());
          }
      }
      if(m_InputData.initial_guess_filename != "" && num_opt_procs.compare("1") != 0)
          fprintf(fp, "decomp -p %s %s\n", num_opt_procs.c_str(), m_InputData.initial_guess_filename.c_str());
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
          std::string num_procs = "4";
          if(!m_InputData.objectives[i].num_procs.empty())
              num_procs = m_InputData.objectives[i].num_procs;
          if(num_procs.compare("1") != 0)
          {
              if(++hasBeenDecompedToThisCount[num_procs] == 1)
              {
                  fprintf(fp, "decomp -p %s %s\n", num_procs.c_str(), m_InputData.run_mesh_name.c_str());
              }
              if(m_InputData.objectives[i].ref_frf_file.length() > 0)
                  fprintf(fp, "decomp -p %s %s\n", num_procs.c_str(), m_InputData.objectives[i].ref_frf_file.c_str());
          }
      }

#ifndef USING_OPEN_MPI
      std::string envString = "-env";
      std::string separationString = " ";
#else
      std::string envString = "-x";
      std::string separationString = "=";
#endif

      std::string tLaunchString = "";
      std::string tNumProcsString = "";
      if(m_InputData.m_UseLaunch)
      {
          tLaunchString = "launch";
          tNumProcsString = "-n";
      }
      else
      {
          tLaunchString = "mpiexec";
          tNumProcsString = "-np";
      }

      if(m_InputData.optimization_type == "shape")
      {
          fprintf(fp, "python aflr.py %s %s %s ", m_InputData.csm_filename.c_str(), m_InputData.csm_exodus_filename.c_str(), m_InputData.csm_tesselation_filename.c_str());
          for(size_t i=0; i<m_InputData.mShapeDesignVariableValues.size(); ++i)
              fprintf(fp, "%s ", m_InputData.mShapeDesignVariableValues[i].c_str());
          fprintf(fp, "\n");
      }

      // Now add the main mpirun call.
      fprintf(fp, "%s %s %s %s PLATO_PERFORMER_ID%s0 \\\n", tLaunchString.c_str(), tNumProcsString.c_str(), num_opt_procs.c_str(), envString.c_str(),separationString.c_str());
      fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
      fprintf(fp, "%s PLATO_APP_FILE%splato_main_operations.xml \\\n", envString.c_str(),separationString.c_str());
      if(m_InputData.plato_main_path.length() != 0)
          fprintf(fp, "%s plato_main_input_deck.xml \\\n", m_InputData.plato_main_path.c_str());
      else
          fprintf(fp, "plato_main plato_main_input_deck.xml \\\n");
      if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
      {

          fprintf(fp, ": %s %s %s PLATO_PERFORMER_ID%s1 \\\n", tNumProcsString.c_str(), Plato::to_string(m_InputData.m_UncertaintyMetaData.numPerformers).c_str(), envString.c_str(),separationString.c_str());
          fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
          fprintf(fp, "%s PLATO_APP_FILE%splato_analyze_operations.xml \\\n", envString.c_str(),separationString.c_str());
        if(m_InputData.plato_analyze_path.length() != 0)
          fprintf(fp, "%s --input-config=plato_analyze_input_deck.xml \\\n", m_InputData.plato_analyze_path.c_str());
        else
          fprintf(fp, "analyze_MPMD --input-config=plato_analyze_input_deck.xml \\\n");
      }
      else
      {
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
          const XMLGen::Objective& cur_obj = m_InputData.objectives[i];
          if(!cur_obj.num_procs.empty())
          {
            fprintf(fp, ": %s %s %s PLATO_PERFORMER_ID%s%d \\\n", tNumProcsString.c_str(), cur_obj.num_procs.c_str(), envString.c_str(),separationString.c_str(), (int)(i+1));
          }
          else
            fprintf(fp, ": %s 4 %s PLATO_PERFORMER_ID%s%d \\\n",  tNumProcsString.c_str(), envString.c_str(),separationString.c_str(),(int)(i+1));
          fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
          fprintf(fp, "%s PLATO_APP_FILE%s%s_operations_%s.xml \\\n", envString.c_str(),separationString.c_str(),cur_obj.code_name.c_str(),
              cur_obj.name.c_str());
          if(!cur_obj.code_name.compare("sierra_sd"))
          {
            if(m_InputData.sierra_sd_path.length() != 0)
              fprintf(fp, "%s sierra_sd_input_deck_%s.i \\\n", m_InputData.sierra_sd_path.c_str(), cur_obj.name.c_str());
            else
              fprintf(fp, "plato_sd_main sierra_sd_input_deck_%s.i \\\n", cur_obj.name.c_str());
          }
          else if(!cur_obj.code_name.compare("lightmp"))
          {
            if(m_InputData.lightmp_path.length() != 0)
              fprintf(fp, "%s lightmp_input_deck_%s.i \\\n", m_InputData.lightmp_path.c_str(), cur_obj.name.c_str());
          }
          else if(!cur_obj.code_name.compare("albany"))
          {
            if(m_InputData.albany_path.length() != 0)
              fprintf(fp, "%s albany_input_deck_%s.i \\\n", m_InputData.albany_path.c_str(), cur_obj.name.c_str());
            else
              fprintf(fp, "albany albany_input_deck_%s.i \\\n", cur_obj.name.c_str());
          }
          else if(!cur_obj.code_name.compare("plato_analyze"))
          {
            if(m_InputData.plato_analyze_path.length() != 0)
              fprintf(fp, "%s --input-config=plato_analyze_input_deck_%s.xml \\\n", m_InputData.plato_analyze_path.c_str(), cur_obj.name.c_str());
            else
              fprintf(fp, "analyze_MPMD --input-config=plato_analyze_input_deck_%s.xml \\\n", cur_obj.name.c_str());
          }
        }
      }

      fclose(fp);
    }
    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generatePlatoAnalyzeShapeLaunchScript()
/******************************************************************************/
{
    FILE *fp=fopen("mpirun.source", "w");
    if(!fp)
    {
        return true;
    }

    std::string num_opt_procs = "1";
    if(!m_InputData.num_opt_processors.empty())
        num_opt_procs = m_InputData.num_opt_processors;

    // remember if the run_mesh has been decomposed to this processor count
    std::map<std::string,int> hasBeenDecompedToThisCount;

    // Now do the decomps for the TO run.
    if(num_opt_procs.compare("1") != 0) 
    {
        if(++hasBeenDecompedToThisCount[num_opt_procs] == 1)
        {
            fprintf(fp, "decomp -p %s %s\n", num_opt_procs.c_str(), m_InputData.run_mesh_name.c_str());
        }
    }
    if(m_InputData.initial_guess_filename != "" && num_opt_procs.compare("1") != 0)
        fprintf(fp, "decomp -p %s %s\n", num_opt_procs.c_str(), m_InputData.initial_guess_filename.c_str());
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        std::string num_procs = "4";
        if(!m_InputData.objectives[i].num_procs.empty())
            num_procs = m_InputData.objectives[i].num_procs;
        if(num_procs.compare("1") != 0)
        {
            if(++hasBeenDecompedToThisCount[num_procs] == 1)
            {
                fprintf(fp, "decomp -p %s %s\n", num_procs.c_str(), m_InputData.run_mesh_name.c_str());
            }
            if(m_InputData.objectives[i].ref_frf_file.length() > 0)
                fprintf(fp, "decomp -p %s %s\n", num_procs.c_str(), m_InputData.objectives[i].ref_frf_file.c_str());
        }
    }

#ifndef USING_OPEN_MPI
    std::string envString = "-env";
    std::string separationString = " ";
#else
    std::string envString = "-x";
    std::string separationString = "=";
#endif

    std::string tLaunchString = "";
    std::string tNumProcsString = "";
    if(m_InputData.m_UseLaunch)
    {
        tLaunchString = "launch";
        tNumProcsString = "-n";
    }
    else
    {
        tLaunchString = "mpiexec";
        tNumProcsString = "-np";
    }

    fprintf(fp, "python aflr.py %s %s %s ", m_InputData.csm_filename.c_str(), m_InputData.csm_exodus_filename.c_str(), m_InputData.csm_tesselation_filename.c_str());
    for(size_t i=0; i<m_InputData.mShapeDesignVariableValues.size(); ++i)
        fprintf(fp, "%s ", m_InputData.mShapeDesignVariableValues[i].c_str());
    fprintf(fp, "\n");

    // Now add the main mpirun call.
    
    // PlatoMain
    fprintf(fp, "%s %s %s %s PLATO_PERFORMER_ID%s0 \\\n", tLaunchString.c_str(), tNumProcsString.c_str(), num_opt_procs.c_str(), envString.c_str(),separationString.c_str());
    fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
    fprintf(fp, "%s PLATO_APP_FILE%splato_main_operations.xml \\\n", envString.c_str(),separationString.c_str());
    if(m_InputData.plato_main_path.length() != 0)
        fprintf(fp, "%s plato_main_input_deck.xml \\\n", m_InputData.plato_main_path.c_str());
    else
        fprintf(fp, "plato_main plato_main_input_deck.xml \\\n");

    // PlatoESP performers
    fprintf(fp, ": %s %d %s PLATO_PERFORMER_ID%s1 \\\n", tNumProcsString.c_str(), m_InputData.num_shape_design_variables, envString.c_str(),separationString.c_str());
    fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
    fprintf(fp, "%s PLATO_APP_FILE%splato_esp_operations.xml \\\n", envString.c_str(),separationString.c_str());
    fprintf(fp, "PlatoESP plato_esp_input_deck.xml \\\n");

    // Physics performers
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        const XMLGen::Objective& cur_obj = m_InputData.objectives[i];
        if(!cur_obj.num_procs.empty())
            fprintf(fp, ": %s %s %s PLATO_PERFORMER_ID%s%d \\\n", tNumProcsString.c_str(), cur_obj.num_procs.c_str(), envString.c_str(),separationString.c_str(), (int)(i+2));
        else
            fprintf(fp, ": %s 4 %s PLATO_PERFORMER_ID%s%d \\\n",  tNumProcsString.c_str(), envString.c_str(),separationString.c_str(),(int)(i+2));
        fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
        fprintf(fp, "%s PLATO_APP_FILE%s%s_operations_%s.xml \\\n", envString.c_str(),separationString.c_str(),cur_obj.code_name.c_str(),
              cur_obj.name.c_str());
        if(m_InputData.plato_analyze_path.length() != 0)
            fprintf(fp, "%s --input-config=plato_analyze_input_deck_%s.xml \\\n", m_InputData.plato_analyze_path.c_str(), cur_obj.name.c_str());
        else
            fprintf(fp, "analyze_MPMD --input-config=plato_analyze_input_deck_%s.xml \\\n", cur_obj.name.c_str());
    }

    fclose(fp);
    return true;
}
/******************************************************************************/
bool DefaultInputGenerator::generateSummitLaunchScripts()
/******************************************************************************/
{
  if(m_InputData.objectives.size() > 99)
  {
    std::cout << "ERROR: Number of Samples must be less than 100" << std::endl;
    return false;
  }
  for(auto objective:m_InputData.objectives)
  {
    if(objective.code_name != "plato_analyze")
      std::cout << "ERROR: Summit output is only supported for Plato Analyze performers" << std::endl;
  }
  generatePerformerBashScripts();
  generateJSRunScript();
  generateBatchScript();

  return true;
}

/******************************************************************************/
void DefaultInputGenerator::generatePerformerBashScripts()
/******************************************************************************/
{
  generateEngineBashScript();
  generateAnalyzeBashScripts();
}

/******************************************************************************/
void DefaultInputGenerator::generateAnalyzeBashScripts()
/******************************************************************************/
{
  if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
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
  else
  {
    for(size_t i = 1; i <= m_InputData.objectives.size(); ++i)
    {
      std::ofstream analyzeBash;
      std::string filename = "analyze" + Plato::to_string(i) + ".sh";
      analyzeBash.open(filename);
      analyzeBash << "export PLATO_PERFORMER_ID=" << i << "\n";
      analyzeBash << "export PLATO_INTERFACE_FILE=interface.xml\n";
      analyzeBash << "export PLATO_APP_FILE=plato_analyze_operations_" << i << ".xml\n";
      analyzeBash << "\n";
      analyzeBash << "analyze_MPMD --input-config=plato_analyze_input_deck_" << i << ".xml\n";

      analyzeBash.close();
    }
  }
}

/******************************************************************************/
void DefaultInputGenerator::generateEngineBashScript()
/******************************************************************************/
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

/******************************************************************************/
void DefaultInputGenerator::generateJSRunScript()
/******************************************************************************/
{
  std::ofstream jsrun;
  jsrun.open("jsrun.source");
  jsrun << "1 : eng : bash engine.sh\n";
  if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
  {
    jsrun << Plato::to_string(m_InputData.m_UncertaintyMetaData.numPerformers) << " : per : bash analyze.sh\n";
  }
  else
  {
    for(size_t i = 1; i <= m_InputData.objectives.size(); ++i)
    {
      jsrun << "1 : per" << i << " : bash analyze" << i << ".sh\n";
    }
  }
  jsrun.close();
}

/******************************************************************************/
void DefaultInputGenerator::generateBatchScript()
/******************************************************************************/
{
  std::ofstream batchFile;
  batchFile.open ("plato.batch");
  batchFile << "#!/bin/bash\n";
  batchFile << "# LSF Directives\n";
  batchFile << "#BSUB -P <PROJECT>\n";
  batchFile << "#BSUB -W 0:00\n";

  size_t tNumGPUsNeeded;
  if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    tNumGPUsNeeded = m_InputData.m_UncertaintyMetaData.numPerformers;
  else
    tNumGPUsNeeded = m_InputData.objectives.size();
  size_t tNumGPUsPerNode = 6;
  size_t tNumNodesNeeded = tNumGPUsNeeded/tNumGPUsPerNode;
  if(tNumGPUsNeeded % tNumGPUsPerNode != 0)
    ++tNumNodesNeeded;

  batchFile << "#BSUB -nnodes " << tNumNodesNeeded << "\n";
  batchFile << "#BSUB -J plato\n";
  batchFile << "#BSUB -o plato.%J\n";
  batchFile << "\n";
  batchFile << "cd <path/to/working/directory>\n";
  batchFile << "date\n";
  batchFile << "jsrun -A eng -n1 -a1 -c1 -g0\n";

  if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
  {
    batchFile << "jsrun -A per -n" << Plato::to_string(m_InputData.m_UncertaintyMetaData.numPerformers) << " -a1 -c1 -g1\n";
  }
  else
  {
    for(size_t i = 1; i <= m_InputData.objectives.size(); ++i)
    {
      batchFile << "jsrun -A per" << i << " -n1 -a1 -c1 -g1\n";
    }
  }

  batchFile << "jsrun -f jsrun.source\n";
  

  batchFile.close();
}

/******************************************************************************/
size_t DefaultInputGenerator::computeNumberOfNodesNeeded()
/******************************************************************************/
{
  size_t tNumGPUsNeeded;
  if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    tNumGPUsNeeded = m_InputData.m_UncertaintyMetaData.numPerformers;
  else
    tNumGPUsNeeded = m_InputData.objectives.size();
  size_t tNumGPUsPerNode = 6;
  size_t tNumNodesNeeded = tNumGPUsNeeded/tNumGPUsPerNode;
  if(tNumGPUsNeeded % tNumGPUsPerNode != 0)
    ++tNumNodesNeeded;
  return tNumNodesNeeded;
}

/******************************************************************************/
bool DefaultInputGenerator::generateSalinasInputDecks(std::ostringstream *aStringStream)
/******************************************************************************/
{
    SalinasInputDeckWriter tWriter(m_InputData);
    tWriter.generate(m_InputData.m_HasUncertainties,m_InputData.m_RequestedVonMisesOutput,aStringStream);
    return true;
}
/******************************************************************************/
bool DefaultInputGenerator::generateAlbanyInputDecks()
/******************************************************************************/
{
    char string_var[200];
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        const XMLGen::Objective& cur_obj = m_InputData.objectives[i];
        if(!cur_obj.code_name.compare("albany"))
        {
            char buf[200];
            sprintf(buf, "albany_input_deck_%s.i", cur_obj.name.c_str());
            pugi::xml_document doc;

            // Version entry
            pugi::xml_node tmp_node = doc.append_child(pugi::node_declaration);
            tmp_node.set_name("xml");
            pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
            tmp_att.set_value("1.0");

            pugi::xml_node n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12;
            pugi::xml_attribute a1, a2, a3;
            n1 = doc.append_child("ParameterList");
            n2 = n1.append_child("ParameterList");
            n2.append_attribute("name") = "Problem";
            if(!cur_obj.type.compare("maximize stiffness"))
                addNTVParameter(n2, "Name", "string", "LinearElasticity 3D");
            else if(!cur_obj.type.compare("maximize heat conduction"))
                addNTVParameter(n2, "Name", "string", "Poissons Equation 3D");

            // Topologies block
            n3 = n2.append_child("ParameterList");
            n3.append_attribute("name") = "Topologies";
            size_t num_topologies = 1;
            sprintf(string_var, "%d", (int)num_topologies);
            addNTVParameter(n3, "Number of Topologies", "int", string_var);
            for(size_t j=0; j<num_topologies; ++j)
            {
                n4 = n3.append_child("ParameterList");
                sprintf(string_var, "Topology %d", (int)j);
                n4.append_attribute("name") = string_var;
                addNTVParameter(n4, "Topology Name", "string", "Rho");
                addNTVParameter(n4, "Entity Type", "string", "State Variable");
                addNTVParameter(n4, "Bounds", "Array(double)", "{0.0,1.0}");
                addNTVParameter(n4, "Initial Value", "double", m_InputData.initial_density_value);
                n5 = n4.append_child("ParameterList");
                n5.append_attribute("name") = "Functions";
                size_t num_functions = 1;
                sprintf(string_var, "%d", (int)num_functions);
                addNTVParameter(n5, "Number of Functions", "int", string_var);
                for(size_t k=0; k<num_functions; ++k)
                {
                    n6 = n5.append_child("ParameterList");
                    sprintf(string_var, "Function %d", (int)k);
                    n6.append_attribute("name") = string_var;
                    addNTVParameter(n6, "Function Type", "string", "SIMP");
                    addNTVParameter(n6, "Minimum", "double", "0.001");
                    addNTVParameter(n6, "Penalization Parameter", "double", m_InputData.materials[0].property("penalty exponent"));
                }
                addNTVParameter(n4, "Spatial Filter", "int", "0");
            }

            // Configuration block
            n3 = n2.append_child("ParameterList");
            n3.append_attribute("name") = "Configuration";
            n4 = n3.append_child("ParameterList");
            n4.append_attribute("name") = "Element Blocks";
            size_t num_blocks = m_InputData.blocks.size();
            sprintf(string_var, "%d", (int)num_blocks);
            addNTVParameter(n4, "Number of Element Blocks", "int", string_var);
            for(size_t j=0; j<num_blocks; ++j)
            {
                n5 = n4.append_child("ParameterList");
                sprintf(string_var, "Element Block %d", (int)j);
                n5.append_attribute("name") = string_var;
                sprintf(string_var, "block_%s", m_InputData.blocks[j].block_id.c_str());
                addNTVParameter(n5, "Name", "string", string_var);
                n6 = n5.append_child("ParameterList");
                n6.append_attribute("name") = "Material";
                for(size_t n=0; n<m_InputData.materials.size(); n++)
                {
                    if(m_InputData.materials[n].id().compare(m_InputData.blocks[j].material_id) == 0)
                    {
                        if(!m_InputData.materials[n].property("youngs modulus").empty())
                            addNTVParameter(n6, "Elastic Modulus", "double", m_InputData.materials[n].property("youngs modulus"));
                        if(!m_InputData.materials[n].property("poissons ratio").empty())
                            addNTVParameter(n6, "Poissons Ratio", "double", m_InputData.materials[n].property("poissons ratio"));
                        if(!m_InputData.materials[n].property("thermal conductivity").empty())
                            addNTVParameter(n6, "Isotropic Modulus", "double", m_InputData.materials[n].property("thermal conductivity"));
                        break;
                    }
                }
            }

            // Dirichlet BCs
            n3 = n2.append_child("ParameterList");
            n3.append_attribute("name") = "Dirichlet BCs";
            for(size_t j=0; j<cur_obj.bc_ids.size(); j++)
            {
                bool found = false;
                XMLGen::BC cur_bc;
                std::string cur_bc_id = cur_obj.bc_ids[j];
                for(size_t qq=0; qq<m_InputData.bcs.size(); ++qq)
                {
                    if(cur_bc_id == m_InputData.bcs[qq].bc_id)
                    {
                        found = true;
                        cur_bc = m_InputData.bcs[qq];
                    }
                }
                if(found)
                {
                    if(!cur_obj.type.compare("maximize stiffness"))
                    {
                        if(cur_bc.dof.empty())
                        {
                            // applying fixed boundary conditions to "sideset"s is currently supported
                            if(cur_bc.app_type != "nodeset")
                            {
                                std::cout << "ERROR:XMLGenerator:generateAlbanyInputDecks: "
                                          << "Albany boundary conditions can only be applied to \"nodeset\" types.\n";
                            }
                            else
                            {
                                sprintf(string_var, "DBC on NS nodelist_%s for DOF %s", cur_bc.app_id.c_str(), "X");
                                addNTVParameter(n3, string_var, "double", "0.0");
                                sprintf(string_var, "DBC on NS nodelist_%s for DOF %s", cur_bc.app_id.c_str(), "Y");
                                addNTVParameter(n3, string_var, "double", "0.0");
                                sprintf(string_var, "DBC on NS nodelist_%s for DOF %s", cur_bc.app_id.c_str(), "Z");
                                addNTVParameter(n3, string_var, "double", "0.0");
                            }
                        }
                        else
                        {
                            if(cur_bc.value.empty())
                            {
                                sprintf(string_var, "DBC on NS nodelist_%s for DOF %s",
                                        cur_bc.app_id.c_str(), cur_bc.dof.c_str());
                                addNTVParameter(n3, string_var, "double", "0.0");
                            }
                            else
                            {
                                sprintf(string_var, "DBC on NS nodelist_%s for DOF %s",
                                        cur_bc.app_id.c_str(), cur_bc.dof.c_str());
                                addNTVParameter(n3, string_var, "double", cur_bc.value);

                            }
                        }
                    }
                    else if(!cur_obj.type.compare("maximize heat conduction"))
                    {
                        if(cur_bc.value.empty())
                        {
                            sprintf(string_var, "DBC on NS nodelist_%s for DOF P",
                                    cur_bc.app_id.c_str());
                            addNTVParameter(n3, string_var, "double", "0.0");
                        }
                        else
                        {
                            sprintf(string_var, "DBC on NS nodelist_%s for DOF P",
                                    cur_bc.app_id.c_str());
                            addNTVParameter(n3, string_var, "double", cur_bc.value);
                        }
                    }
                }
            }

            // Neumann BCs
            n3 = n2.append_child("ParameterList");
            n3.append_attribute("name") = "Neumann BCs";
            for(size_t j=0; j<cur_obj.load_case_ids.size(); j++)
            {
                bool found = false;
                XMLGen::LoadCase cur_load_case;
                std::string cur_load_id = cur_obj.load_case_ids[j];
                for(size_t qq=0; qq<m_InputData.load_cases.size(); ++qq)
                {
                    if(cur_load_id == m_InputData.load_cases[qq].id)
                    {
                        found = true;
                        cur_load_case = m_InputData.load_cases[qq];
                    }
                }
                if(found)
                {
                    for(size_t e=0; e<cur_load_case.loads.size(); e++)
                    {
                        XMLGen::Load cur_load = cur_load_case.loads[e];
                        if(!cur_obj.type.compare("maximize stiffness"))
                        {
                            if(cur_load.type == "traction")
                            {
                                sprintf(string_var, "NBC on SS surface_%s for DOF all set (t_x, t_y, t_z)",
                                        cur_load.app_id.c_str());
                                char tmp_buf[200];
                                double x = std::atof(cur_load.values[0].c_str());
                                double y = std::atof(cur_load.values[1].c_str());
                                double z = std::atof(cur_load.values[2].c_str());
                                sprintf(tmp_buf, "{%lf,%lf,%lf}", x, y, z);
                                addNTVParameter(n3, string_var, "Array(double)", tmp_buf);
                            }
                        }
                        else if(!cur_obj.type.compare("maximize heat conduction"))
                        {
                            if(cur_load.type == "heat") // for "heat flux"
                            {
                                sprintf(string_var, "NBC on SS surface_%s for DOF P set (dudx, dudy, dudz)",
                                        cur_load.app_id.c_str());
                                char tmp_buf[200];
                                sprintf(tmp_buf, "{%s,0.0,0.0}", cur_load.values[0].c_str());
                                addNTVParameter(n3, string_var, "Array(double)", tmp_buf);
                            }
                        }
                    }
                }
            }

            // Topology Weights
            n3 = n2.append_child("ParameterList");
            n3.append_attribute("name") = "Apply Topology Weight Functions";
            size_t num_fields = 1;
            sprintf(string_var, "%d", (int)num_fields);
            addNTVParameter(n3, "Number of Fields", "int", string_var);
            for(size_t j=0; j<num_fields; ++j)
            {
                sprintf(string_var, "Field %d", (int)j);
                n4 = n3.append_child("ParameterList");
                n4.append_attribute("name") = string_var;
                if(!cur_obj.type.compare("maximize stiffness"))
                {
                    addNTVParameter(n4, "Name", "string", "Stress");
                    addNTVParameter(n4, "Layout", "string", "QP Tensor");
                }
                else if(!cur_obj.type.compare("maximize heat conduction"))
                {
                    addNTVParameter(n4, "Name", "string", "kinVar");
                    addNTVParameter(n4, "Layout", "string", "QP Vector");
                }
                addNTVParameter(n4, "Topology Index", "int", "0");
                addNTVParameter(n4, "Function Index", "int", "0");

            }

            // Response Functions
            n3 = n2.append_child("ParameterList");
            n3.append_attribute("name") = "Response Functions";
            size_t num_response_vectors = 1;
            sprintf(string_var, "%d", (int)num_response_vectors);
            addNTVParameter(n3, "Number of Response Vectors", "int", string_var);
            for(size_t j=0; j<num_fields; ++j)
            {
                sprintf(string_var, "Response Vector %d", (int)j);
                n4 = n3.append_child("ParameterList");
                n4.append_attribute("name") = string_var;
                addNTVParameter(n4, "Name", "string", "Stiffness Objective");
                if(!cur_obj.type.compare("maximize stiffness"))
                {
                    addNTVParameter(n4, "Gradient Field Name", "string", "Strain");
                    addNTVParameter(n4, "Gradient Field Layout", "string", "QP Tensor");
                    addNTVParameter(n4, "Work Conjugate Name", "string", "Stress");
                    addNTVParameter(n4, "Work Conjugate Layout", "string", "QP Tensor");
                }
                else if(!cur_obj.type.compare("maximize heat conduction"))
                {
                    addNTVParameter(n4, "Gradient Field Name", "string", "Phi Gradient");
                    addNTVParameter(n4, "Gradient Field Layout", "string", "QP Vector");
                    addNTVParameter(n4, "Work Conjugate Name", "string", "kinVar");
                    addNTVParameter(n4, "Work Conjugate Layout", "string", "QP Vector");
                }
                addNTVParameter(n4, "Topology Index", "int", "0");
                addNTVParameter(n4, "Function Index", "int", "0");
                sprintf(string_var, "R%d", (int)j);
                addNTVParameter(n4, "Response Name", "string", string_var);
                sprintf(string_var, "dR%ddRho", (int)j);
                addNTVParameter(n4, "Response Derivative Name", "string", string_var);
            }

            // Discretization
            n2 = n1.append_child("ParameterList");
            n2.append_attribute("name") = "Discretization";
            addNTVParameter(n2, "Method", "string", "Ioss");
            sprintf(string_var, "%s", m_InputData.run_mesh_name.c_str());
            addNTVParameter(n2, "Exodus Input File Name", "string", string_var);
            sprintf(string_var, "%s_alb_%s.exo", m_InputData.run_mesh_name_without_extension.c_str(), cur_obj.name.c_str());
            addNTVParameter(n2, "Exodus Output File Name", "string", string_var);
            addNTVParameter(n2, "Separate Evaluators by Element Block", "bool", "true");

            // Piro
            n2 = n1.append_child("ParameterList");
            n2.append_attribute("name") = "Piro";
            n3 = n2.append_child("ParameterList");
            n3.append_attribute("name") = "LOCA";
            n4 = n3.append_child("ParameterList");
            n4.append_attribute("name") = "Bifurcation";
            n4 = n3.append_child("ParameterList");
            n4.append_attribute("name") = "Constraints";
            n4 = n3.append_child("ParameterList");
            n4.append_attribute("name") = "Predictor";
            n5 = n4.append_child("ParameterList");
            n5.append_attribute("name") = "First Step Predictor";
            n5 = n4.append_child("ParameterList");
            n5.append_attribute("name") = "Last Step Predictor";
            n4 = n3.append_child("ParameterList");
            n4.append_attribute("name") = "Step Size";
            n4 = n3.append_child("ParameterList");
            n4.append_attribute("name") = "Stepper";
            n5 = n4.append_child("ParameterList");
            n5.append_attribute("name") = "Eigensolver";
            n3 = n2.append_child("ParameterList");
            n3.append_attribute("name") = "NOX";
            n4 = n3.append_child("ParameterList");
            n4.append_attribute("name") = "Status Tests";
            addNTVParameter(n4, "Test Type", "string", "Combo");
            addNTVParameter(n4, "Combo Type", "string", "OR");
            addNTVParameter(n4, "Number of Tests", "int", "2");
            n5 = n4.append_child("ParameterList");
            n5.append_attribute("name") = "Test 0";
            addNTVParameter(n5, "Test Type", "string", "NormF");
            addNTVParameter(n5, "Norm Type", "string", "Two Norm");
            addNTVParameter(n5, "Scale Type", "string", "Scaled");
            addNTVParameter(n5, "Tolerance", "double", "1e-8");
            n5 = n4.append_child("ParameterList");
            n5.append_attribute("name") = "Test 1";
            addNTVParameter(n5, "Test Type", "string", "MaxIters");
            addNTVParameter(n5, "Maximum Iterations", "int", m_InputData.max_iterations.c_str());
            n5 = n4.append_child("ParameterList");
            n5.append_attribute("name") = "Test 2";
            addNTVParameter(n5, "Test Type", "string", "NormUpdate");
            addNTVParameter(n5, "Norm Type", "string", "Two Norm");
            addNTVParameter(n5, "Scale Type", "string", "Scaled");
            addNTVParameter(n5, "Tolerance", "double", "1e-8");
            n4 = n3.append_child("ParameterList");
            n4.append_attribute("name") = "Direction";
            addNTVParameter(n4, "Method", "string", "Newton");
            n5 = n4.append_child("ParameterList");
            n5.append_attribute("name") = "Newton";
            addNTVParameter(n5, "Forcing Term Method", "string", "Constant");
            addNTVParameter(n5, "Rescue Bad Newton Solve", "bool", "1");
            n6 = n5.append_child("ParameterList");
            n6.append_attribute("name") = "Stratimikos Linear Solver";
            n7 = n6.append_child("ParameterList");
            n7.append_attribute("name") = "NOX Stratimikos Options";
            n7 = n6.append_child("ParameterList");
            n7.append_attribute("name") = "Stratimikos";
            addNTVParameter(n7, "Linear Solver Type", "string", "Belos");
            n8 = n7.append_child("ParameterList");
            n8.append_attribute("name") = "Linear Solver Types";
            n9 = n8.append_child("ParameterList");
            n9.append_attribute("name") = "AztecOO";
            n10 = n9.append_child("ParameterList");
            n10.append_attribute("name") = "Forward Solve";
            n11 = n10.append_child("ParameterList");
            n11.append_attribute("name") = "AztecOO Settings";
            addNTVParameter(n11, "Aztec Solver", "string", "GMRES");
            addNTVParameter(n11, "Convergence Test", "string", "r0");
            addNTVParameter(n11, "Size of Krylov Subspace", "int", "200");
            addNTVParameter(n11, "Output Frequency", "int", "10");
            addNTVParameter(n10, "Max Iterations", "int", "200");
            addNTVParameter(n10, "Tolerance", "double", "1e-5");
            n9 = n8.append_child("ParameterList");
            n9.append_attribute("name") = "Belos";
            addNTVParameter(n9, "Solver Type", "string", "Block GMRES");
            n10 = n9.append_child("ParameterList");
            n10.append_attribute("name") = "Solver Types";
            n11 = n10.append_child("ParameterList");
            n11.append_attribute("name") = "Block GMRES";
            addNTVParameter(n11, "Convergence Tolerance", "double", "1e-12");
            addNTVParameter(n11, "Output Frequency", "int", "2");
            addNTVParameter(n11, "Output Style", "int", "1");
            addNTVParameter(n11, "Verbosity", "int", "0");
            addNTVParameter(n11, "Maximum Iterations", "int", "200");
            addNTVParameter(n11, "Block Size", "int", "1");
            addNTVParameter(n11, "Num Blocks", "int", "200");
            addNTVParameter(n11, "Flexible Gmres", "bool", "0");
            addNTVParameter(n7, "Preconditioner Type", "string", "Ifpack2");
            n8 = n7.append_child("ParameterList");
            n8.append_attribute("name") = "Preconditioner Types";
            n9 = n8.append_child("ParameterList");
            n9.append_attribute("name") = "Ifpack2";
            addNTVParameter(n9, "Overlap", "int", "2");
            addNTVParameter(n9, "Prec Type", "string", "ILUT");
            n10 = n9.append_child("ParameterList");
            n10.append_attribute("name") = "Ifpack2 Settings";
            addNTVParameter(n10, "fact: drop tolerance", "double", "0");
            addNTVParameter(n10, "fact: ilut level-of-fill", "double", "1");
            n10 = n9.append_child("ParameterList");
            n10.append_attribute("name") = "VerboseObject";
            addNTVParameter(n10, "Verbosity Level", "string", "medium");
            n4 = n3.append_child("ParameterList");
            n4.append_attribute("name") = "Line Search";
            n5 = n4.append_child("ParameterList");
            n5.append_attribute("name") = "Full Step";
            addNTVParameter(n5, "Full Step", "double", "1");
            addNTVParameter(n4, "Method", "string", "Full Step");
            addNTVParameter(n3, "Nonlinear Solver", "string", "Line Search Based");
            n4 = n3.append_child("ParameterList");
            n4.append_attribute("name") = "Printing";
            addNTVParameter(n4, "Output Information", "int", "103");
            addNTVParameter(n4, "Output Precision", "int", "3");
            addNTVParameter(n4, "Output Processor", "int", "0");
            n4 = n3.append_child("ParameterList");
            n4.append_attribute("name") = "Solver Options";
            addNTVParameter(n4, "Status Test Check Type", "string", "Minimal");



            // Write the file to disk
            doc.save_file(buf, "  ");
        }
    }
    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generatePlatoAnalyzeInputDecks(std::ostringstream *aStringStream)
/******************************************************************************/
{
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
        if(!generatePlatoAnalyzeInputDeckForNewUncertaintyWorkflow())
            return false;
    }
    else
    {
        PlatoAnalyzeInputDeckWriter tInputDeckWriter(m_InputData);
        if(m_InputData.optimization_type == "shape" && m_InputData.mPlatoAnalyzePerformerExists)
        {
            return tInputDeckWriter.generateForShape(aStringStream);
        }
        else
        {
            return tInputDeckWriter.generate(aStringStream);
        }
    }
    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generatePlatoAnalyzeInputDeckForNewUncertaintyWorkflow()
/******************************************************************************/
{
  pugi::xml_document doc;
  pugi::xml_node tmp_node = doc.append_child(pugi::node_declaration);
  tmp_node.set_name("xml");
  pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
  tmp_att.set_value("1.0");

  pugi::xml_node n1, n2, n3, n4;
  n1 = doc.append_child("ParameterList");
  n1.append_attribute("name") = "Problem";
  addNTVParameter(n1, "Physics", "string", "Plato Driver");
  addNTVParameter(n1, "Spatial Dimension", "int", "3");
  addNTVParameter(n1, "Input Mesh", "string", m_InputData.run_mesh_name);

  n2 = n1.append_child("ParameterList");
  n2.append_attribute("name") = "Plato Problem";
  addNTVParameter(n2, "Physics", "string", "Mechanical");
  addNTVParameter(n2, "PDE Constraint", "string", "Elliptic");
  addNTVParameter(n2, "Constraint", "string", "My Volume");
  addNTVParameter(n2, "Objective", "string", "My Internal Elastic Energy");
  addNTVParameter(n2, "Self-Adjoint", "bool", "true");

  n3 = n2.append_child("ParameterList");
  n3.append_attribute("name") = "My Volume";
  addNTVParameter(n3, "Type", "string", "Scalar Function");
  addNTVParameter(n3, "Scalar Function Type", "string", "Volume");

  n4 = n3.append_child("ParameterList");
  n4.append_attribute("name") = "Penalty Function";
  addNTVParameter(n4, "Type", "string", "SIMP");
  addNTVParameter(n4, "Exponent", "double", "1.0");
  addNTVParameter(n4, "Minimum Value", "double", "0.0");

  n3 = n2.append_child("ParameterList");
  n3.append_attribute("name") = "My Internal Elastic Energy";
  addNTVParameter(n3, "Type", "string", "Scalar Function");
  addNTVParameter(n3, "Scalar Function Type", "string", "Internal Elastic Energy");

  n4 = n3.append_child("ParameterList");
  n4.append_attribute("name") = "Penalty Function";
  addNTVParameter(n4, "Type", "string", "SIMP");
  addNTVParameter(n4, "Exponent", "double", "3.0");
  addNTVParameter(n4, "Minimum Value", "double", "1.0e-3");

  n3 = n2.append_child("ParameterList");
  n3.append_attribute("name") = "Elliptic";

  n4 = n3.append_child("ParameterList");
  n4.append_attribute("name") = "Penalty Function";
  addNTVParameter(n4, "Type", "string", "SIMP");
  addNTVParameter(n4, "Exponent", "double", "3.0");
  addNTVParameter(n4, "Minimum Value", "double", "1.0e-3");

  n3 = n2.append_child("ParameterList");
  n3.append_attribute("name") = "Material Model";

  n4 = n3.append_child("ParameterList");
  n4.append_attribute("name") = "Isotropic Linear Elastic";
  addNTVParameter(n4, "Poissons Ratio", "double", m_InputData.materials[0].property("poissons ratio").c_str());
  addNTVParameter(n4, "Youngs Modulus", "double", m_InputData.materials[0].property("youngs modulus").c_str());


  n3 = n2.append_child("ParameterList");
  n3.append_attribute("name") = "Natural Boundary Conditions";

  for(size_t tRandomLoadIndex = 0; tRandomLoadIndex < m_InputData.m_UncertaintyMetaData.randomVariableIndices.size(); ++tRandomLoadIndex)
  {
    n4 = n3.append_child("ParameterList");
    std::string tRandomLoadName = "Random Traction Vector Boundary Condition " + Plato::to_string(tRandomLoadIndex);
    n4.append_attribute("name") = tRandomLoadName.c_str();
    addNTVParameter(n4, "Type", "string", "Uniform");
    addNTVParameter(n4, "Values", "Array(double)", "{0.0, 0.0, 0.0}");
    std::string tMeshSetName = m_InputData.load_cases[0].loads[tRandomLoadIndex].app_name;
    addNTVParameter(n4, "Sides", "string", tMeshSetName);
  }

  for(size_t tDeterministicLoadIndex = 0; tDeterministicLoadIndex < m_InputData.m_UncertaintyMetaData.deterministicVariableIndices.size(); ++tDeterministicLoadIndex)
  {
    n4 = n3.append_child("ParameterList");
    std::string tDeterministicLoadName = "Deterministic Traction Vector Boundary Condition " + Plato::to_string(tDeterministicLoadIndex);
    n4.append_attribute("name") = tDeterministicLoadName.c_str();
    addNTVParameter(n4, "Type", "string", "Uniform");
    std::vector<std::string> tDeterministicLoadValues = m_InputData.load_cases[0].loads[tDeterministicLoadIndex].values;

    std::string tValuesInputString = "{";
    for(size_t i = 0; i < tDeterministicLoadValues.size(); ++i)
    {
      tValuesInputString += tDeterministicLoadValues[i];
      if(i != tDeterministicLoadValues.size() - 1)
        tValuesInputString += ", ";
    }
  
    addNTVParameter(n4, "Values", "Array(double)", "{0.0, 0.0, 0.0}");
    std::string tMeshSetName = m_InputData.load_cases[0].loads[tDeterministicLoadIndex].app_name;
    addNTVParameter(n4, "Sides", "string", tMeshSetName);
  }



  n3 = n2.append_child("ParameterList");
  n3.append_attribute("name") = "Essential Boundary Conditions";

  for(size_t i = 0; i < m_InputData.bcs.size(); ++i)
  {
    XMLGen::BC bc = m_InputData.bcs[i];
    if(bc.type != "displacement")
    {
      std::cout << "Error: Only fixed displacement boundary conditions are currently supported for new uncertainty workflow" << std::endl;
      return false;
    }
    if(bc.dof == "x")
    {
      n4 = n3.append_child("ParameterList");
      std::string tBCName = "X Fixed Displacement Boundary Condition " + Plato::to_string(i);
      n4.append_attribute("name") = tBCName.c_str();
      if(bc.value.empty())
        addNTVParameter(n4, "Type", "string", "Zero Value");
      else
      {
        addNTVParameter(n4, "Type", "string", "Fixed Value");
        addNTVParameter(n4, "Value", "double", bc.value);
      }
      addNTVParameter(n4, "Index", "int", "0");
      addNTVParameter(n4, "Sides", "string", bc.app_name);
    }
    else if(bc.dof == "y")
    {
      n4 = n3.append_child("ParameterList");
      std::string tBCName = "Y Fixed Displacement Boundary Condition " + Plato::to_string(i);
      n4.append_attribute("name") = tBCName.c_str();
      if(bc.value.empty())
        addNTVParameter(n4, "Type", "string", "Zero Value");
      else
      {
        addNTVParameter(n4, "Type", "string", "Fixed Value");
        addNTVParameter(n4, "Value", "double", bc.value);
      }
      addNTVParameter(n4, "Index", "int", "1");
      addNTVParameter(n4, "Sides", "string", bc.app_name);
    }
    else if(bc.dof == "z")
    {
      n4 = n3.append_child("ParameterList");
      std::string tBCName = "Z Fixed Displacement Boundary Condition " + Plato::to_string(i);
      n4.append_attribute("name") = tBCName.c_str();
      if(bc.value.empty())
        addNTVParameter(n4, "Type", "string", "Zero Value");
      else
      {
        addNTVParameter(n4, "Type", "string", "Fixed Value");
        addNTVParameter(n4, "Value", "double", bc.value);
      }
      addNTVParameter(n4, "Index", "int", "2");
      addNTVParameter(n4, "Sides", "string", bc.app_name);
    }
    else if(bc.dof.empty())
    {
      n4 = n3.append_child("ParameterList");
      std::string tBCName = "X Fixed Displacement Boundary Condition " + Plato::to_string(i);
      n4.append_attribute("name") = tBCName.c_str();
      addNTVParameter(n4, "Type", "string", "Zero Value");
      addNTVParameter(n4, "Index", "int", "0");
      addNTVParameter(n4, "Sides", "string", bc.app_name);

      n4 = n3.append_child("ParameterList");
      tBCName = "Y Fixed Displacement Boundary Condition " + Plato::to_string(i);
      n4.append_attribute("name") = tBCName.c_str();
      addNTVParameter(n4, "Type", "string", "Zero Value");
      addNTVParameter(n4, "Index", "int", "1");
      addNTVParameter(n4, "Sides", "string", bc.app_name);

      n4 = n3.append_child("ParameterList");
      tBCName = "Z Fixed Displacement Boundary Condition " + Plato::to_string(i);
      n4.append_attribute("name") = tBCName.c_str();
      addNTVParameter(n4, "Type", "string", "Zero Value");
      addNTVParameter(n4, "Index", "int", "2");
      addNTVParameter(n4, "Sides", "string", bc.app_name);
    }
  }

  char buf[200];
  sprintf(buf, "plato_analyze_input_deck.xml");
  doc.save_file(buf, "  ");

  return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generateLightMPInputDecks()
/******************************************************************************/
{
    pugi::xml_node node1, node2, node3;
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        const XMLGen::Objective& cur_obj = m_InputData.objectives[i];
        if(!cur_obj.code_name.compare("lightmp"))
        {
            char buf[200];
            pugi::xml_document doc;

            // Version entry
            pugi::xml_node tmp_node = doc.append_child(pugi::node_declaration);
            tmp_node.set_name("xml");
            pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
            tmp_att.set_value("1.0");

            for(size_t n=0; n<m_InputData.materials.size(); ++n)
            {
                node1 = doc.append_child("material");
                node2 = node1.append_child("id");
                node3 = node2.append_child(pugi::node_pcdata);
                node3.set_value(m_InputData.materials[n].id().c_str());

                node1 = node1.append_child("linear_elastic");
                node2 = node1.append_child("poissons_ratio");
                node3 = node2.append_child(pugi::node_pcdata);
                node3.set_value(m_InputData.materials[n].property("poissons ratio").c_str());
                node2 = node1.append_child("youngs_modulus");
                node3 = node2.append_child(pugi::node_pcdata);
                node3.set_value(m_InputData.materials[n].property("youngs modulus").c_str());
            }

            node1 = doc.append_child("mesh");
            node2 = node1.append_child("type");
            node3 = node2.append_child(pugi::node_pcdata);
            node3.set_value("unstructured");
            node2 = node1.append_child("format");
            node3 = node2.append_child(pugi::node_pcdata);
            node3.set_value("exodus");
            node2 = node1.append_child("mesh");
            node3 = node2.append_child(pugi::node_pcdata);
            node3.set_value(m_InputData.run_mesh_name.c_str());
            // lightmp just needs one block specified with an integration rule--don't need all of the blocks listed here
            node2 = node1.append_child("block");
            node3 = node2.append_child("index");
            pugi::xml_node node4 = node3.append_child(pugi::node_pcdata);
            node4.set_value(m_InputData.blocks[0].block_id.c_str());
            node3 = node2.append_child("integration");
            node4 = node3.append_child("type");
            pugi::xml_node node5 = node4.append_child(pugi::node_pcdata);
            node5.set_value("gauss");
            node4 = node3.append_child("order");
            node5 = node4.append_child(pugi::node_pcdata);
            node5.set_value("2");
            node3 = node2.append_child("material");
            node4 = node3.append_child(pugi::node_pcdata);
            node4.set_value(m_InputData.blocks[0].material_id.c_str());

            node1 = doc.append_child("output");
            node2 = node1.append_child("file");
            node3 = node2.append_child(pugi::node_pcdata);
            sprintf(buf, "lightmp_output_%s", cur_obj.name.c_str());
            node3.set_value(buf);
            node2 = node1.append_child("format");
            node3 = node2.append_child(pugi::node_pcdata);
            node3.set_value("exodus");

            node1 = doc.append_child("control");
            node2 = node1.append_child("time");
            node3 = node2.append_child("start");
            node4 = node3.append_child(pugi::node_pcdata);
            node4.set_value("0.0");
            node3 = node2.append_child("end");
            node4 = node3.append_child(pugi::node_pcdata);
            node4.set_value("0.01");
            node3 = node2.append_child("timestep");
            node4 = node3.append_child(pugi::node_pcdata);
            node4.set_value("0.001");

            node1 = doc.append_child("physics");
            node2 = node1.append_child("solid_statics");
            node3 = node2.append_child("solver");
            node4 = node3.append_child("output_interval");
            node5 = node4.append_child(pugi::node_pcdata);
            node5.set_value("100");
            node4 = node3.append_child("tolerance");
            node5 = node4.append_child(pugi::node_pcdata);
            node5.set_value("1e-9");
            node4 = node3.append_child("iterations");
            node5 = node4.append_child(pugi::node_pcdata);
            node5.set_value("1000");
            node3 = node2.append_child("boundary_conditions");
            pugi::xml_node node6;
            for(size_t j=0; j<cur_obj.bc_ids.size(); ++j)
            {
                bool found = false;
                XMLGen::BC cur_bc;
                std::string cur_bc_id = cur_obj.bc_ids[j];
                for(size_t qq=0; qq<m_InputData.bcs.size(); ++qq)
                {
                    if(cur_bc_id == m_InputData.bcs[qq].bc_id)
                    {
                        found = true;
                        cur_bc = m_InputData.bcs[qq];
                    }
                }
                if(found)
                {
                    if(cur_bc.dof.empty())
                    {
                        node4 = node3.append_child("displacement");
                        node5 = node4.append_child("nodeset");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value(cur_bc.app_id.c_str());
                        node5 = node4.append_child("direction");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value("x");
                        node5 = node4.append_child("value");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value("0.0");
                        node5 = node4.append_child("scale");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value("1.0");
                        node4 = node3.append_child("displacement");
                        node5 = node4.append_child("nodeset");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value(cur_bc.app_id.c_str());
                        node5 = node4.append_child("direction");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value("y");
                        node5 = node4.append_child("value");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value("0.0");
                        node5 = node4.append_child("scale");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value("1.0");
                        node4 = node3.append_child("displacement");
                        node5 = node4.append_child("nodeset");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value(cur_bc.app_id.c_str());
                        node5 = node4.append_child("direction");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value("z");
                        node5 = node4.append_child("value");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value("0.0");
                        node5 = node4.append_child("scale");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value("1.0");
                    }
                    else
                    {
                        std::string tValue;
                        if(cur_bc.value.empty())
                        {
                            tValue = "0.0";
                        }
                        else
                        {
                            tValue = cur_bc.value;
                        }
                        node4 = node3.append_child("displacement");
                        node5 = node4.append_child("nodeset");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value(cur_bc.app_id.c_str());
                        node5 = node4.append_child("direction");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value(cur_bc.dof.c_str());
                        node5 = node4.append_child("value");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value(tValue.c_str());
                        node5 = node4.append_child("scale");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value("1.0");
                    }
                }
            }
            for(size_t j=0; j<cur_obj.load_case_ids.size(); ++j)
            {
                bool found = false;
                XMLGen::LoadCase cur_load_case;
                std::string cur_load_id = cur_obj.load_case_ids[j];
                for(size_t qq=0; found == false && qq<m_InputData.load_cases.size(); ++qq)
                {
                    if(cur_load_id == m_InputData.load_cases[qq].id)
                    {
                        found = true;
                        cur_load_case = m_InputData.load_cases[qq];
                    }
                }
                if(found)
                {
                    for(size_t e=0; e<cur_load_case.loads.size(); ++e)
                    {
                        XMLGen::Load cur_load = cur_load_case.loads[e];
                        node4 = node3.append_child("traction");
                        node5 = node4.append_child("nodeset");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value(cur_load.app_id.c_str());
                        node5 = node4.append_child("direction");
                        node6 = node5.append_child(pugi::node_pcdata);
                        double x = std::atof(cur_load.values[0].c_str());
                        double y = std::atof(cur_load.values[1].c_str());
                        double z = std::atof(cur_load.values[2].c_str());
                        double magx = fabs(x);
                        double magy = fabs(y);
                        double magz = fabs(z);
                        std::string value = "0.0";
                        if(magx > magy && magx > magz)
                        {
                            node6.set_value("x");
                            value = Plato::to_string(x);
                        }
                        else if(magy > magx && magy > magz)
                        {
                            node6.set_value("y");
                            value = Plato::to_string(y);
                        }
                        else if(magz > magx && magz > magy)
                        {
                            node6.set_value("z");
                            value = Plato::to_string(z);
                        }
                        node5 = node4.append_child("value");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value(value.c_str());
                        node5 = node4.append_child("scale");
                        node6 = node5.append_child(pugi::node_pcdata);
                        node6.set_value("1.0");
                    }
                }
            }

            // Write the file to disk
            sprintf(buf, "lightmp_input_deck_%s.i", cur_obj.name.c_str());
            doc.save_file(buf, "  ");
        }
    }
    return true;
}
/******************************************************************************/
bool DefaultInputGenerator::generatePhysicsInputDecks()
/******************************************************************************/
{
    if(!generateSalinasInputDecks())
      return false;
    if(!generateAlbanyInputDecks())
      return false;
    if(!generateLightMPInputDecks())
      return false;
    if(!generatePlatoAnalyzeInputDecks())
      return false;
    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generatePlatoMainInputDeckXML()
/******************************************************************************/
{
    pugi::xml_document doc;
    pugi::xml_node tmp_node1, tmp_node2, tmp_node3;

    // Version entry
    pugi::xml_node tmp_node = doc.append_child(pugi::node_declaration);
    tmp_node.set_name("xml");
    pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
    tmp_att.set_value("1.0");

    // mesh
    pugi::xml_node mesh_node = doc.append_child("mesh");
    addChild(mesh_node, "type", "unstructured");
    addChild(mesh_node, "format", "exodus");

    if(m_InputData.mPlatoAnalyzePerformerExists && !m_InputData.mAllPerformersArePlatoAnalyze)
    {
        std::cout << "*******************************************************************************\n";
        std::cout << "WARNING: Adding ignore_node_map for case where some performers may use it!!!!!\n";
        std::cout << "*******************************************************************************\n";
    }
    
    // If we have runs with mixed performers will this cause problems?
    if(m_InputData.mPlatoAnalyzePerformerExists)
        addChild(mesh_node, "ignore_node_map", "true");

    addChild(mesh_node, "mesh", m_InputData.run_mesh_name.c_str());
    // just need one block specified here
    if(m_InputData.blocks.size() > 0)
    {
        tmp_node = mesh_node.append_child("block");
        addChild(tmp_node, "index", m_InputData.blocks[0].block_id.c_str());
        tmp_node1 = tmp_node.append_child("integration");
        addChild(tmp_node1, "type", "gauss");
        addChild(tmp_node1, "order", "2");
        addChild(tmp_node, "material", m_InputData.blocks[0].material_id.c_str());
    }

    // output
    tmp_node = doc.append_child("output");
    addChild(tmp_node, "file", "platomain");
    addChild(tmp_node, "format", "exodus");

    // Write the file to disk
    doc.save_file("plato_main_input_deck.xml", "  ");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generatePlatoESPInputDeckXML()
/******************************************************************************/
{
    pugi::xml_document doc;

    // Version entry
    pugi::xml_node tmp_node1 = doc.append_child(pugi::node_declaration);
    tmp_node1.set_name("xml");
    pugi::xml_attribute tmp_att = tmp_node1.append_attribute("version");
    tmp_att.set_value("1.0");

    // ESP
    tmp_node1 = doc.append_child("ESP");
    addChild(tmp_node1, "ModelFileName", m_InputData.csm_filename);
    addChild(tmp_node1, "TessFileName", m_InputData.csm_tesselation_filename);
    addChild(tmp_node1, "ParameterIndex", "0");

    // Write the file to disk
    doc.save_file("plato_esp_input_deck.xml", "  ");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generatePerformerOperationsXML()
/******************************************************************************/
{
    generateSalinasOperationsXML();
    generateAlbanyOperationsXML();
    generateLightMPOperationsXML();
    if(m_InputData.optimization_type == "shape" && m_InputData.mPlatoAnalyzePerformerExists)
    {
        generatePlatoAnalyzeOperationsXMLForShape();
        generatePlatoESPOperationsXMLForShape();
    }
    else
    {
        generatePlatoAnalyzeOperationsXML();
    }
    generateAMGXInput();
    generateROLInput();
    return true;
}

/******************************************************************************/
void DefaultInputGenerator::generateROLInput()
/******************************************************************************/
{
  if(m_InputData.optimization_algorithm == "rol ksal" ||
      m_InputData.optimization_algorithm == "rol ksbc")
  {
    pugi::xml_document doc;
    pugi::xml_node n1, n2, n3;
    pugi::xml_attribute a1;

    // Version entry
    n1 = doc.append_child(pugi::node_declaration);
    n1.set_name("xml");
    a1 = n1.append_attribute("version");
    a1.set_value("1.0");

    n1 = doc.append_child("ParameterList");
    n1.append_attribute("name") = "Inputs";

    n2 = n1.append_child("ParameterList");
    n2.append_attribute("name") = "Problem";

    n2 = n1.append_child("ParameterList");
    n2.append_attribute("name") = "General";
    n3 = n2.append_child("ParameterList");
    n3.append_attribute("name") = "Secant";
    addNTVParameter(n3, "Type", "string", "Limited-Memory BFGS");
    addNTVParameter(n3, "Use as Preconditioner", "bool", "false");
    addNTVParameter(n3, "Use as Hessian", "bool", "true");
    addNTVParameter(n3, "Maximum Storage", "int", "20");
    addNTVParameter(n3, "Barzilai-Borwein", "int", "1");

    n2 = n1.append_child("ParameterList");
    n2.append_attribute("name") = "Step";

    // Trust region
    n3 = n2.append_child("ParameterList");
    n3.append_attribute("name") = "Trust Region";
    addNTVParameter(n3, "Subproblem Solver", "string", "Truncated CG");
    addNTVParameter(n3, "Subproblem Model", "string", "Kelley-Sachs");
    addNTVParameter(n3, "Initial Radius", "double", "-1.0");
    addNTVParameter(n3, "Maximum Radius", "double", "1.0e8");
    addNTVParameter(n3, "Step Acceptance Threshold", "double", "0.05");
    addNTVParameter(n3, "Radius Shrinking Threshold", "double", "0.05");
    addNTVParameter(n3, "Radius Growing Threshold", "double", "0.9");
    addNTVParameter(n3, "Radius Shrinking Rate (Negative rho)", "double", "0.0625");
    addNTVParameter(n3, "Radius Shrinking Rate (Positive rho)", "double", "0.25");
    addNTVParameter(n3, "Radius Growing Rate", "double", "2.5");
    addNTVParameter(n3, "Safeguard Size", "double", "100.0");

    // Augmented Lagrangian
    n3 = n2.append_child("ParameterList");
    n3.append_attribute("name") = "Augmented Lagrangian";
    addNTVParameter(n3, "Use Scaled Augmented Lagrangian", "bool", "false");
    addNTVParameter(n3, "Level of Hessian Approximation", "int", "0");
    addNTVParameter(n3, "Use Default Problem Scaling", "bool", "true");
    addNTVParameter(n3, "Objective Scaling", "double", "1.0");
    addNTVParameter(n3, "Constraint Scaling", "double", "1.0");
    addNTVParameter(n3, "Use Default Initial Penalty Parameter", "bool", "true");
    addNTVParameter(n3, "Initial Penalty Parameter", "double", "1.0e1");
    addNTVParameter(n3, "Penalty Parameter Growth Factor", "double", "1.0e1");
    addNTVParameter(n3, "Maximum Penalty Parameter", "double", "1.0e8");
    addNTVParameter(n3, "Initial Optimality Tolerance", "double", "1.0");
    addNTVParameter(n3, "Optimality Tolerance Update Exponent", "double", "1.0");
    addNTVParameter(n3, "Optimality Tolerance Decrease Exponent", "double", "1.0");
    addNTVParameter(n3, "Initial Feasibility Tolerance", "double", "1.0");
    addNTVParameter(n3, "Feasibility Tolerance Update Exponent", "double", "0.1");
    addNTVParameter(n3, "Feasibility Tolerance Decrease Exponent", "double", "0.9");
    addNTVParameter(n3, "Print Intermediate Optimization History", "bool", "false");
    addNTVParameter(n3, "Subproblem Step Type", "string", "Trust Region");
    addNTVParameter(n3, "Subproblem Iteration Limit", "int", "20");

    n2 = n1.append_child("ParameterList");
    n2.append_attribute("name") = "Status Test";
    addNTVParameter(n2, "Iteration Limit", "int", m_InputData.max_iterations);

    // Write the file to disk
    doc.save_file("rol_inputs.xml", "  ");
  }
}

/******************************************************************************/
void DefaultInputGenerator::generateAMGXInput()
/******************************************************************************/
{
    if(m_InputData.mPlatoAnalyzePerformerExists)
    {
        FILE *fp = fopen("amgx.json", "w");
        if(fp)
        {
            fprintf(fp, "{\n");
            fprintf(fp, "\"config_version\": 2,\n");
            fprintf(fp, "\"solver\": {\n");
            fprintf(fp, "\"preconditioner\": {\n");
            fprintf(fp, "\"print_grid_stats\": 1,\n");
            fprintf(fp, "\"algorithm\": \"AGGREGATION\",\n");
            fprintf(fp, "\"print_vis_data\": 0,\n");
            fprintf(fp, "\"max_matching_iterations\": 50,\n");
            fprintf(fp, "\"max_unassigned_percentage\": 0.01,\n");
            fprintf(fp, "\"solver\": \"AMG\",\n");
            fprintf(fp, "\"smoother\": {\n");
            fprintf(fp, "\"relaxation_factor\": 0.78,\n");
            fprintf(fp, "\"scope\": \"jacobi\",\n");
            fprintf(fp, "\"solver\": \"BLOCK_JACOBI\",\n");
            fprintf(fp, "\"monitor_residual\": 0,\n");
            fprintf(fp, "\"print_solve_stats\": 0\n");
            fprintf(fp, "},\n");
            fprintf(fp, "\"print_solve_stats\": 0,\n");
            fprintf(fp, "\"dense_lu_num_rows\": 64,\n");
            fprintf(fp, "\"presweeps\": 1,\n");
            fprintf(fp, "\"selector\": \"SIZE_8\",\n");
            fprintf(fp, "\"coarse_solver\": \"DENSE_LU_SOLVER\",\n");
            fprintf(fp, "\"coarsest_sweeps\": 2,\n");
            fprintf(fp, "\"max_iters\": 1,\n");
            fprintf(fp, "\"monitor_residual\": 0,\n");
            fprintf(fp, "\"store_res_history\": 0,\n");
            fprintf(fp, "\"scope\": \"amg\",\n");
            fprintf(fp, "\"max_levels\": 100,\n");
            fprintf(fp, "\"postsweeps\": 1,\n");
            fprintf(fp, "\"cycle\": \"W\"\n");
            fprintf(fp, "},\n");
            fprintf(fp, "\"solver\": \"PBICGSTAB\",\n");
            fprintf(fp, "\"print_solve_stats\": 0,\n");
            fprintf(fp, "\"obtain_timings\": 0,\n");
            fprintf(fp, "\"max_iters\": 1000,\n");
            fprintf(fp, "\"monitor_residual\": 1,\n");
            fprintf(fp, "\"convergence\": \"ABSOLUTE\",\n");
            fprintf(fp, "\"scope\": \"main\",\n");
            fprintf(fp, "\"tolerance\": 1e-12,\n");
            fprintf(fp, "\"norm\": \"L2\"\n");
            fprintf(fp, "}\n");
            fprintf(fp, "}\n");
            fclose(fp);
        }
    }
}

/******************************************************************************/
bool DefaultInputGenerator::generateLightMPOperationsXML()
/******************************************************************************/
{
  int num_lightmp_objs = 0;
  for(size_t i=0; i<m_InputData.objectives.size(); ++i)
  {
    if(!m_InputData.objectives[i].code_name.compare("lightmp"))
    {
      num_lightmp_objs++;

      pugi::xml_document doc;
      pugi::xml_node tmp_node, tmp_node1;

      // Version entry
      tmp_node = doc.append_child(pugi::node_declaration);
      tmp_node.set_name("xml");
      pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
      tmp_att.set_value("1.0");

      /* light mp doesn't currently support this
      // Cache State
      tmp_node = doc.append_child("Operation");
      addChild(tmp_node, "Function", "Cache State");
      addChild(tmp_node, "Name", "Cache State");
      */

      // InternalEnergy
      tmp_node = doc.append_child("Operation");
      addChild(tmp_node, "Function", "InternalEnergy");
      addChild(tmp_node, "Name", "Compute Internal Energy");
      tmp_node1 = tmp_node.append_child("Input");
      addChild(tmp_node1, "ArgumentName", "Topology");
      tmp_node1 = tmp_node.append_child("Output");
      addChild(tmp_node1, "ArgumentName", "Internal Energy");
      tmp_node1 = tmp_node.append_child("Output");
      addChild(tmp_node1, "ArgumentName", "Internal Energy Gradient");
      addChild(tmp_node, "PenaltyModel", "SIMP");
      tmp_node1 = tmp_node.append_child("SIMP");
      if(m_InputData.materials.size() > 0)
      {
        addChild(tmp_node1, "PenaltyExponent", m_InputData.materials[0].property("penalty exponent").c_str());
      }
      addChild(tmp_node1, "MinimumValue", "0.001");

      if(m_InputData.discretization == "levelset")
      {
        // Compute Surface Area
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "Compute Surface Area");
        addChild(tmp_node, "Name", "Compute Surface Area");
        tmp_node1 = tmp_node.append_child("Input");
        addChild(tmp_node1, "ArgumentName", "Topology");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Surface Area");

        // Compute Surface Area Gradient
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "Compute Surface Area Gradient");
        addChild(tmp_node, "Name", "Compute Surface Area Gradient");
        tmp_node1 = tmp_node.append_child("Input");
        addChild(tmp_node1, "ArgumentName", "Topology");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Surface Area Gradient");
      }

      char buf[200];
      sprintf(buf, "lightmp_operations_%s.xml", m_InputData.objectives[i].name.c_str());
      // Write the file to disk
      doc.save_file(buf, "  ");
    }
  }

  return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generateAlbanyOperationsXML()
/******************************************************************************/
{
  int num_albany_objs = 0;
  for(size_t i=0; i<m_InputData.objectives.size(); ++i)
  {
    if(!m_InputData.objectives[i].code_name.compare("albany"))
    {
      num_albany_objs++;

      pugi::xml_document doc;
      pugi::xml_node tmp_node, tmp_node1;

      // Version entry
      tmp_node = doc.append_child(pugi::node_declaration);
      tmp_node.set_name("xml");
      pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
      tmp_att.set_value("1.0");

      // Cache State
      /* For now Albany isn't handling this correctly.
         tmp_node = doc.append_child("Operation");
         addChild(tmp_node, "Function", "Cache State");
         addChild(tmp_node, "Name", "Cache State");
         */

      // InternalEnergy
      tmp_node = doc.append_child("Operation");
      addChild(tmp_node, "Function", "AlbanyResponse");
      addChild(tmp_node, "Name", "Compute Internal Energy");
      tmp_node1 = tmp_node.append_child("Input");
      addChild(tmp_node1, "LocalName", "Rho");
      addChild(tmp_node1, "Type", "Field");
      addChild(tmp_node1, "ArgumentName", "Topology");
      tmp_node1 = tmp_node.append_child("Output");
      addChild(tmp_node1, "LocalName", "R0");
      addChild(tmp_node1, "Type", "Value");
      addChild(tmp_node1, "ArgumentName", "Internal Energy");
      tmp_node1 = tmp_node.append_child("Output");
      addChild(tmp_node1, "LocalName", "dR0dRho 0");
      addChild(tmp_node1, "Type", "Field");
      addChild(tmp_node1, "ArgumentName", "Internal Energy Gradient");

      if(m_InputData.discretization == "levelset")
      {
        // Compute Surface Area
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "Compute Surface Area");
        addChild(tmp_node, "Name", "Compute Surface Area");
        tmp_node1 = tmp_node.append_child("Input");
        addChild(tmp_node1, "ArgumentName", "Topology");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Surface Area");

        // Compute Surface Area Gradient
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "Compute Surface Area Gradient");
        addChild(tmp_node, "Name", "Compute Surface Area Gradient");
        tmp_node1 = tmp_node.append_child("Input");
        addChild(tmp_node1, "ArgumentName", "Topology");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Surface Area Gradient");
      }

      char buf[200];
      sprintf(buf, "albany_operations_%s.xml", m_InputData.objectives[i].name.c_str());
      // Write the file to disk
      doc.save_file(buf, "  ");
    }
  }

  return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generatePlatoAnalyzeOperationsXML()
/******************************************************************************/
{
  int num_plato_analyze_objs = 0;
  if(m_InputData.optimization_type == "topology")
  {
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_document doc;
      pugi::xml_node tmp_node, tmp_node1;

      // Version entry
      tmp_node = doc.append_child(pugi::node_declaration);
      tmp_node.set_name("xml");
      pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
      tmp_att.set_value("1.0");

      // ComputeSolution
      tmp_node = doc.append_child("Operation");
      addChild(tmp_node, "Function", "ComputeSolution");
      addChild(tmp_node, "Name", "Compute Displacement Solution");
      tmp_node1 = tmp_node.append_child("Input");
      addChild(tmp_node1, "ArgumentName", "Topology");

      // ComputeObjectiveValue
      tmp_node = doc.append_child("Operation");
      addChild(tmp_node, "Function", "ComputeObjectiveValue");
      addChild(tmp_node, "Name", "Compute Objective Value");
      tmp_node1 = tmp_node.append_child("Input");
      addChild(tmp_node1, "ArgumentName", "Topology");
      tmp_node1 = tmp_node.append_child("Output");
      addChild(tmp_node1, "ArgumentName", "Objective Value");

      for(size_t tRandomLoadIndex = 0; tRandomLoadIndex < m_InputData.m_UncertaintyMetaData.randomVariableIndices.size(); ++tRandomLoadIndex)
      {
        tmp_node1 = tmp_node.append_child("Parameter");
        std::string tLoadName = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "X";
        addChild(tmp_node1, "ArgumentName", tLoadName);
        std::string tPathToLoadInInputFile = "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition " + Plato::to_string(tRandomLoadIndex) + "]:Values(0)";
        addChild(tmp_node1, "Target", tPathToLoadInInputFile);
        addChild(tmp_node1, "InitialValue", "0.0");
        tmp_node1 = tmp_node.append_child("Parameter");
        tLoadName = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "Y";
        addChild(tmp_node1, "ArgumentName", tLoadName);
        tPathToLoadInInputFile = "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition " + Plato::to_string(tRandomLoadIndex) + "]:Values(1)";
        addChild(tmp_node1, "Target", tPathToLoadInInputFile);
        addChild(tmp_node1, "InitialValue", "0.0");
        tmp_node1 = tmp_node.append_child("Parameter");
        tLoadName = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "Z";
        addChild(tmp_node1, "ArgumentName", tLoadName);
        tPathToLoadInInputFile = "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition " + Plato::to_string(tRandomLoadIndex) + "]:Values(2)";
        addChild(tmp_node1, "Target", tPathToLoadInInputFile);
        addChild(tmp_node1, "InitialValue", "0.0");
      }

      // ComputeObjectiveGradient
      tmp_node = doc.append_child("Operation");
      addChild(tmp_node, "Function", "ComputeObjective");
      addChild(tmp_node, "Name", "Compute Objective Gradient");
      tmp_node1 = tmp_node.append_child("Input");
      addChild(tmp_node1, "ArgumentName", "Topology");
      tmp_node1 = tmp_node.append_child("Output");
      addChild(tmp_node1, "ArgumentName", "Objective Gradient");

      for(size_t tRandomLoadIndex = 0; tRandomLoadIndex < m_InputData.m_UncertaintyMetaData.randomVariableIndices.size(); ++tRandomLoadIndex)
      {
        tmp_node1 = tmp_node.append_child("Parameter");
        std::string tLoadName = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "X";
        addChild(tmp_node1, "ArgumentName", tLoadName);
        std::string tPathToLoadInInputFile = "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition " + Plato::to_string(tRandomLoadIndex) + "]:Values(0)";
        addChild(tmp_node1, "Target", tPathToLoadInInputFile);
        addChild(tmp_node1, "InitialValue", "0.0");
        tmp_node1 = tmp_node.append_child("Parameter");
        tLoadName = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "Y";
        addChild(tmp_node1, "ArgumentName", tLoadName);
        tPathToLoadInInputFile = "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition " + Plato::to_string(tRandomLoadIndex) + "]:Values(1)";
        addChild(tmp_node1, "Target", tPathToLoadInInputFile);
        addChild(tmp_node1, "InitialValue", "0.0");
        tmp_node1 = tmp_node.append_child("Parameter");
        tLoadName = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "Z";
        addChild(tmp_node1, "ArgumentName", tLoadName);
        tPathToLoadInInputFile = "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition " + Plato::to_string(tRandomLoadIndex) + "]:Values(2)";
        addChild(tmp_node1, "Target", tPathToLoadInInputFile);
        addChild(tmp_node1, "InitialValue", "0.0");
      }


      // WriteOutput
      tmp_node = doc.append_child("Operation");
      addChild(tmp_node, "Function", "WriteOutput");
      addChild(tmp_node, "Name", "Write Output");

      char buf[200];
      sprintf(buf, "plato_analyze_operations.xml");
      // Write the file to disk
      doc.save_file(buf, "  ");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        if(m_InputData.objectives[i].code_name == "plato_analyze")
        {
          Objective tCurObjective = m_InputData.objectives[i];
          num_plato_analyze_objs++;

          pugi::xml_document doc;
          pugi::xml_node tmp_node, tmp_node1;

          // Version entry
          tmp_node = doc.append_child(pugi::node_declaration);
          tmp_node.set_name("xml");
          pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
          tmp_att.set_value("1.0");

          // ComputeSolution
          tmp_node = doc.append_child("Operation");
          addChild(tmp_node, "Function", "ComputeSolution");
          addChild(tmp_node, "Name", "Compute Displacement Solution");
          tmp_node1 = tmp_node.append_child("Input");
          addChild(tmp_node1, "ArgumentName", "Topology");

          // ComputeObjectiveValue
          tmp_node = doc.append_child("Operation");
          addChild(tmp_node, "Function", "ComputeObjectiveValue");
          addChild(tmp_node, "Name", "Compute Objective Value");
          tmp_node1 = tmp_node.append_child("Input");
          addChild(tmp_node1, "ArgumentName", "Topology");
          tmp_node1 = tmp_node.append_child("Output");
          addChild(tmp_node1, "ArgumentName", "Objective Value");

          // ComputeObjectiveGradient
          tmp_node = doc.append_child("Operation");
          addChild(tmp_node, "Function", "ComputeObjectiveGradient");
          addChild(tmp_node, "Name", "Compute Objective Gradient");
          tmp_node1 = tmp_node.append_child("Input");
          addChild(tmp_node1, "ArgumentName", "Topology");
          tmp_node1 = tmp_node.append_child("Output");
          addChild(tmp_node1, "ArgumentName", "Objective Gradient");

          // WriteOutput
          tmp_node = doc.append_child("Operation");
          addChild(tmp_node, "Function", "WriteOutput");
          addChild(tmp_node, "Name", "Write Output");
          for(size_t j=0; j<tCurObjective.output_for_plotting.size(); ++j)
          {
            tmp_node1 = tmp_node.append_child("Output");
            if(tCurObjective.output_for_plotting[j] == "dispx")
              addChild(tmp_node1, "ArgumentName", "Solution X");
            else if(tCurObjective.output_for_plotting[j] == "dispy")
              addChild(tmp_node1, "ArgumentName", "Solution Y");
            else if(tCurObjective.output_for_plotting[j] == "dispz")
              addChild(tmp_node1, "ArgumentName", "Solution Z");
            else if(tCurObjective.output_for_plotting[j] == "temperature")
              addChild(tmp_node1, "ArgumentName", "Solution");
          }

          char buf[200];
          sprintf(buf, "plato_analyze_operations_%s.xml", m_InputData.objectives[i].name.c_str());
          // Write the file to disk
          doc.save_file(buf, "  ");
        }
      }
    }
  }
  else if(m_InputData.optimization_type == "shape")
  {
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
      if(m_InputData.objectives[i].code_name == "plato_analyze")
      {
        Objective tCurObjective = m_InputData.objectives[i];
        num_plato_analyze_objs++;

        pugi::xml_document doc;
        pugi::xml_node tmp_node, tmp_node1;

        // Version entry
        tmp_node = doc.append_child(pugi::node_declaration);
        tmp_node.set_name("xml");
        pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
        tmp_att.set_value("1.0");

        // ComputeSolution
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "ComputeSolution");
        addChild(tmp_node, "Name", "Compute Displacement Solution");

        // ComputeObjectiveValue
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "ComputeObjectiveValue");
        addChild(tmp_node, "Name", "Compute Objective Value");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Objective Value");

        // ComputeObjectiveGradient
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "ComputeObjectiveP");
        addChild(tmp_node, "Name", "Compute Objective Gradient");
        addChild(tmp_node, "ESPName", "Design Geometry");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Objective Gradient");

        // ComputeObjective
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "ComputeObjectiveP");
        addChild(tmp_node, "Name", "Compute Objective");
        addChild(tmp_node, "ESPName", "Design Geometry");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Objective Value");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Objective Gradient");

        // ComputeConstraintValue
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "ComputeConstraintValue");
        addChild(tmp_node, "Name", "Compute Constraint Value");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Constraint Value");

        // ComputeConstraintGradient
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "ComputeConstraintGradientP");
        addChild(tmp_node, "Name", "Compute Constraint Gradient");
        addChild(tmp_node, "ESPName", "Design Geometry");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Constraint Gradient");

        // ComputeConstraint
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "ComputeConstraintP");
        addChild(tmp_node, "Name", "Compute Constraint");
        addChild(tmp_node, "ESPName", "Design Geometry");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Constraint Value");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Constraint Gradient");

        // Reinitialize on Change Operation
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "ReinitializeESP");
        addChild(tmp_node, "Name", "Reinitialize on Change");
        addChild(tmp_node, "OnChange", "true");
        tmp_node1 = tmp_node.append_child("Input");
        addChild(tmp_node1, "ArgumentName", "Parameters");
        addChild(tmp_node, "ESPName", "Design Geometry");
        tmp_node1 = tmp_node.append_child("Parameter");
        addChild(tmp_node1, "ArgumentName", "Input Mesh");
        addChild(tmp_node1, "Target", "Input Mesh");
        addChild(tmp_node1, "InitialValue", m_InputData.csm_exodus_filename);

        tmp_node = doc.append_child("ESP");
        addChild(tmp_node, "Name", "Design Geometry");
        addChild(tmp_node, "ModelFileName", m_InputData.csm_filename);
        addChild(tmp_node, "TessFileName", m_InputData.csm_tesselation_filename);

        char buf[200];
        sprintf(buf, "plato_analyze_operations_%s.xml", m_InputData.objectives[i].name.c_str());
        // Write the file to disk
        doc.save_file(buf, "  ");
      }
    }
  }

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generatePlatoAnalyzeOperationsXMLForShape()
/******************************************************************************/
{
    int num_plato_analyze_objs = 0;
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        if(m_InputData.objectives[i].code_name == "plato_analyze")
        {
            Objective tCurObjective = m_InputData.objectives[i];
            num_plato_analyze_objs++;

            pugi::xml_document doc;
            pugi::xml_node tmp_node, tmp_node1;
            pugi::xml_node tForNode;

            // Version entry
            tmp_node = doc.append_child(pugi::node_declaration);
            tmp_node.set_name("xml");
            pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
            tmp_att.set_value("1.0");

            // include defines.xml
            pugi::xml_node def_node = doc.append_child("include");
            def_node.append_attribute("filename") = "defines.xml";

            // Compute Constraint Sensitivity
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Name", "Compute Constraint Sensitivity");
            addChild(tmp_node, "Function", "MapConstraintGradientX");
            tForNode = tmp_node.append_child("For");
            tForNode.append_attribute("var") = "I";
            tForNode.append_attribute("in") = "Parameters";
            tmp_node1 = tForNode.append_child("Input");
            addChild(tmp_node1, "ArgumentName", "Parameter Sensitivity {I}");
            tmp_node1 = tmp_node.append_child("Output");
            addChild(tmp_node1, "ArgumentName", "Constraint Sensitivity");

            // Compute Objective Sensitivity
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Name", "Compute Objective Sensitivity");
            addChild(tmp_node, "Function", "MapObjectiveGradientX");
            tForNode = tmp_node.append_child("For");
            tForNode.append_attribute("var") = "I";
            tForNode.append_attribute("in") = "Parameters";
            tmp_node1 = tForNode.append_child("Input");
            addChild(tmp_node1, "ArgumentName", "Parameter Sensitivity {I}");
            tmp_node1 = tmp_node.append_child("Output");
            addChild(tmp_node1, "ArgumentName", "Objective Sensitivity");

            // ComputeObjectiveValue
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Name", "Compute Objective Value");
            addChild(tmp_node, "Function", "ComputeObjectiveValue");
            tmp_node1 = tmp_node.append_child("Output");
            addChild(tmp_node1, "ArgumentName", "Objective Value");

            // ComputeObjectiveGradient
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Name", "Compute Objective Gradient");
            addChild(tmp_node, "Function", "ComputeObjectiveX");

            // ComputeConstraintValue
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Name", "Compute Constraint Value");
            addChild(tmp_node, "Function", "ComputeConstraintValue");
            tmp_node1 = tmp_node.append_child("Output");
            addChild(tmp_node1, "ArgumentName", "Constraint Value");

            // ComputeConstraintGradient
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Name", "Compute Constraint Gradient");
            addChild(tmp_node, "Function", "ComputeConstraintX");

            // Reinitialize on Change Operation
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Name", "Reinitialize on Change");
            addChild(tmp_node, "Function", "Reinitialize");
            addChild(tmp_node, "OnChange", "true");
            tmp_node1 = tmp_node.append_child("Input");
            addChild(tmp_node1, "ArgumentName", "Parameters");
            addChild(tmp_node1, "SharedDataName", "Design Parameters");

            char buf[200];
            sprintf(buf, "plato_analyze_operations_%s.xml", m_InputData.objectives[i].name.c_str());
            // Write the file to disk
            doc.save_file(buf, "  ");
        }
    }

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generatePlatoESPOperationsXMLForShape()
/******************************************************************************/
{
    pugi::xml_document doc;
    pugi::xml_node tmp_node, tmp_node1;

    // Version entry
    tmp_node = doc.append_child(pugi::node_declaration);
    tmp_node.set_name("xml");
    pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
    tmp_att.set_value("1.0");

    // Timers
    tmp_node = doc.append_child("Timers");
    addChild(tmp_node, "time", "true");

    // Compute Constraint Sensitivity
    tmp_node = doc.append_child("Operation");
    addChild(tmp_node, "Name", "Compute Parameter Sensitivity on Change");
    addChild(tmp_node, "Function", "ComputeParameterSensitivity");
    addChild(tmp_node, "OnChange", "true");
    tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Parameters");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Parameter Sensitivity");
    tmp_node1 = tmp_node.append_child("Parameter");
    addChild(tmp_node1, "ArgumentName", "Parameter Index");
    addChild(tmp_node1, "InitialValue", "0");

    // Write the file to disk
    doc.save_file("plato_esp_operations.xml", "  ");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generateSalinasOperationsXML()
/******************************************************************************/
{
    int num_sierra_sd_objs = 0;
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        if(!m_InputData.objectives[i].code_name.compare("sierra_sd"))
        {
            XMLGen::Objective cur_obj = m_InputData.objectives[i];
            num_sierra_sd_objs++;

            pugi::xml_document doc;
            pugi::xml_node tmp_node1;

            // Version entry
            pugi::xml_node tmp_node = doc.append_child(pugi::node_declaration);
            tmp_node.set_name("xml");
            pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
            tmp_att.set_value("1.0");

            // Update Problem
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Function", "Update Problem");
            addChild(tmp_node, "Name", "Update Problem");

            // Cache State
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Function", "Cache State");
            addChild(tmp_node, "Name", "Cache State");
            if(cur_obj.multi_load_case == "true")
            {
                for(size_t k=0; k<cur_obj.load_case_ids.size(); k++)
                {
                    char buffer[100];
                    sprintf(buffer, "%lu", k);
                    for(size_t j=0; j<cur_obj.output_for_plotting.size(); j++)
                    {
                        tmp_node1 = tmp_node.append_child("Output");
                        addChild(tmp_node1, "ArgumentName", cur_obj.output_for_plotting[j] + buffer);
                    }
                }
            }
            else
            {
                for(size_t j=0; j<cur_obj.output_for_plotting.size(); j++)
                {
                    tmp_node1 = tmp_node.append_child("Output");
                    addChild(tmp_node1, "ArgumentName", cur_obj.output_for_plotting[j] + "0");
                }
            }

            // Displacement
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Function", "Displacement");
            addChild(tmp_node, "Name", "Compute Displacement");
            tmp_node1 = tmp_node.append_child("Topology");
            addChild(tmp_node1, "Name", "Topology");
            addChild(tmp_node, "PenaltyModel", "SIMP");
            tmp_node1 = tmp_node.append_child("SIMP");
            if(m_InputData.materials.size() > 0)
            {
                addChild(tmp_node1, "PenaltyExponent", m_InputData.materials[0].property("penalty exponent").c_str());
            }
            addChild(tmp_node1, "MinimumValue", "0.001");

            // InternalEnergy
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Function", "InternalEnergy");
            addChild(tmp_node, "Name", "Compute Objective");
            tmp_node1 = tmp_node.append_child("Topology");
            addChild(tmp_node1, "Name", "Topology");
            addChild(tmp_node, "PenaltyModel", "SIMP");
            tmp_node1 = tmp_node.append_child("SIMP");
            if(m_InputData.materials.size() > 0)
            {
                addChild(tmp_node1, "PenaltyExponent", m_InputData.materials[0].property("penalty exponent").c_str());
            }
            addChild(tmp_node1, "MinimumValue", "0.001");
            tmp_node1 = tmp_node.append_child("OutputValue");
            addChild(tmp_node1, "Name", "Internal Energy");

            // InternalEnergyGradient
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Function", "InternalEnergyGradient");
            addChild(tmp_node, "Name", "Compute Gradient");
            tmp_node1 = tmp_node.append_child("Topology");
            addChild(tmp_node1, "Name", "Topology");
            addChild(tmp_node, "PenaltyModel", "SIMP");
            tmp_node1 = tmp_node.append_child("SIMP");
            if(m_InputData.materials.size() > 0)
            {
                addChild(tmp_node1, "PenaltyExponent", m_InputData.materials[0].property("penalty exponent").c_str());
            }
            addChild(tmp_node1, "MinimumValue", "0.001");
            tmp_node1 = tmp_node.append_child("OutputGradient");
            addChild(tmp_node1, "Name", "Internal Energy Gradient");

            if(m_InputData.optimization_algorithm =="ksbc" ||
               m_InputData.optimization_algorithm == "ksal" ||
               m_InputData.optimization_algorithm == "rol ksal" ||
               m_InputData.optimization_algorithm == "rol ksbc")
            {
                // InternalEnergyHessian
                tmp_node = doc.append_child("Operation");
                addChild(tmp_node, "Function", "InternalEnergyHessian");
                addChild(tmp_node, "Name", "Compute HessianTimesVector");
                tmp_node1 = tmp_node.append_child("Topology");
                addChild(tmp_node1, "Name", "Topology");
                tmp_node1 = tmp_node.append_child("DescentDirection");
                addChild(tmp_node1, "Name", "Descent Direction");
                addChild(tmp_node, "PenaltyModel", "SIMP");
                tmp_node1 = tmp_node.append_child("SIMP");
                if(m_InputData.materials.size() > 0)
                {
                    addChild(tmp_node1, "PenaltyExponent", m_InputData.materials[0].property("penalty exponent").c_str());
                }
                addChild(tmp_node1, "MinimumValue", "0.001");
                tmp_node1 = tmp_node.append_child("OutputHessian");
                addChild(tmp_node1, "Name", "HessianTimesVector");
            }

            if(m_InputData.discretization == "levelset")
            {
                // Compute Surface Area
                tmp_node = doc.append_child("Operation");
                addChild(tmp_node, "Function", "Compute Surface Area");
                addChild(tmp_node, "Name", "Compute Surface Area");
                tmp_node1 = tmp_node.append_child("Topology");
                addChild(tmp_node1, "Name", "Topology");
                tmp_node1 = tmp_node.append_child("OutputValue");
                addChild(tmp_node1, "Name", "SurfaceArea");

                // Compute Surface Area Gradient
                tmp_node = doc.append_child("Operation");
                addChild(tmp_node, "Function", "Compute Surface Area Gradient");
                addChild(tmp_node, "Name", "Compute Surface Area Gradient");
                tmp_node1 = tmp_node.append_child("Topology");
                addChild(tmp_node1, "ArgumentName", "Topology");
                tmp_node1 = tmp_node.append_child("OutputValue");
                addChild(tmp_node1, "Name", "SurfaceAreaGradient");

            }

            char buf[200];
            sprintf(buf, "sierra_sd_operations_%s.xml", m_InputData.objectives[i].name.c_str());
            // Write the file to disk
            doc.save_file(buf, "  ");
        }
    }

    return true;
}

/******************************************************************************/
void DefaultInputGenerator::addStochasticObjectiveValueOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node1;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "MeanPlusStdDev");
    addChild(tmp_node, "Name", "Stochastic Objective Value");
    addChild(tmp_node, "Layout", "Scalar");
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node.append_child("For");
      for_node.append_attribute("var") = "performerIndex";
      for_node.append_attribute("in") = "Performers";
      for_node = for_node.append_child("For");
      for_node.append_attribute("var") = "PerformerSampleIndex";
      for_node.append_attribute("in") = "PerformerSamples";
      pugi::xml_node input_node = for_node.append_child("Input");
      addChild(input_node, "ArgumentName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
      addChild(input_node, "Probability", "{Probabilities[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        XMLGen::Objective cur_obj = m_InputData.objectives[i];
        tmp_node1 = tmp_node.append_child("Input");
        std::string tTmpString = "";
        if(cur_obj.code_name == "plato_analyze")
          tTmpString += "Objective Value ";
        else
          tTmpString += "Internal Energy ";
        tTmpString += Plato::to_string(i+1);
        addChild(tmp_node1, "ArgumentName", tTmpString);
        addChild(tmp_node1, "Probability", Plato::to_string(m_InputData.load_case_probabilities[i]).c_str());
      }
    }
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "Statistic", "mean");
    addChild(tmp_node1, "ArgumentName", "objective_mean");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "Statistic", "std_dev");
    addChild(tmp_node1, "ArgumentName", "objective_std_dev");

    tmp_node1 = tmp_node.append_child("Output");
    std::string tFieldName = "mean_plus_";
    tFieldName += m_InputData.objective_number_standard_deviations;
    tFieldName += "_std_dev";
    addChild(tmp_node1, "Statistic", tFieldName);
    tFieldName = "Objective Mean Plus ";
    tFieldName += m_InputData.objective_number_standard_deviations;
    tFieldName += " StdDev";
    addChild(tmp_node1, "ArgumentName", tFieldName);
}

/******************************************************************************/
void DefaultInputGenerator::addVonMisesStatisticsOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node1;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "MeanPlusStdDev");
    addChild(tmp_node, "Name", "VonMises Statistics");
    addChild(tmp_node, "Layout", "Element Field");
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node.append_child("For");
      for_node.append_attribute("var") = "performerIndex";
      for_node.append_attribute("in") = "Performers";
      for_node = for_node.append_child("For");
      for_node.append_attribute("var") = "PerformerSampleIndex";
      for_node.append_attribute("in") = "PerformerSamples";
      pugi::xml_node input_node = for_node.append_child("Input");
      addChild(input_node, "ArgumentName", "plato_analyze_{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}_vonmises");
      addChild(input_node, "Probability", "{Probabilities[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        XMLGen::Objective cur_obj = m_InputData.objectives[i];
        tmp_node1 = tmp_node.append_child("Input");
        addChild(tmp_node1, "ArgumentName", cur_obj.mPerformerName + "_vonmises");
        addChild(tmp_node1, "Probability", Plato::to_string(m_InputData.load_case_probabilities[i]).c_str());
      }
    }
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "Statistic", "mean");
    addChild(tmp_node1, "ArgumentName", "von_mises_mean");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "Statistic", "std_dev");
    addChild(tmp_node1, "ArgumentName", "von_mises_std_dev");
    for(size_t j=0; j<m_InputData.mStandardDeviations.size(); ++j)
    {
        tmp_node1 = tmp_node.append_child("Output");
        std::string tTmpString = "mean_plus_";
        tTmpString += m_InputData.mStandardDeviations[j];
        tTmpString += "_std_dev";
        addChild(tmp_node1, "Statistic", tTmpString);
        tTmpString = "VonMises Mean Plus ";
        tTmpString += m_InputData.mStandardDeviations[j];
        tTmpString += " StdDev";
        addChild(tmp_node1, "ArgumentName", tTmpString);
    }
}

/******************************************************************************/
void DefaultInputGenerator::addFilterControlOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "Filter");
    addChild(tmp_node, "Name", "FilterControl");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Field");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Filtered Field");
    addChild(tmp_node, "Gradient", "False");
}

/******************************************************************************/
bool DefaultInputGenerator::addAggregateGradientOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    char tBuffer[100];
    pugi::xml_node tmp_node2;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "Aggregator");
    addChild(tmp_node, "Name", "AggregateGradient");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Nodal Field");
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node1.append_child("For");
      for_node.append_attribute("var") = "sampleIndex";
      for_node.append_attribute("in") = "Samples";
      tmp_node2 = for_node.append_child("Input"); 
      addChild(tmp_node2, "ArgumentName", "Field {sampleIndex}");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
          tmp_node2 = tmp_node1.append_child("Input");
          std::string tTmpString = "Field ";
          tTmpString += Plato::to_string(i+1);
          addChild(tmp_node2, "ArgumentName", tTmpString);
      }
    }
    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Field");

    tmp_node1 = tmp_node.append_child("Weighting");
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node1.append_child("For");
      for_node.append_attribute("var") = "sampleIndex";
      for_node.append_attribute("in") = "Samples";
      tmp_node2 = for_node.append_child("Weight"); 
      addChild(tmp_node2, "Value", "1");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
          tmp_node2 = tmp_node1.append_child("Weight");
          addChild(tmp_node2, "Value", m_InputData.objectives[i].weight.c_str());
      }
    }
    if(m_InputData.mUseNormalizationInAggregator == "true")
    {
      if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
      {
        std::cerr << "Error: Normalization not supported for uncertain loads" << std::endl;
        return false;
      }
        tmp_node2 = tmp_node1.append_child("Normals");
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
            pugi::xml_node tmp_node3 = tmp_node2.append_child("Input");
            sprintf(tBuffer, "Normalization Factor %d", (int)(i+1));
            addChild(tmp_node3, "ArgumentName", tBuffer);
        }
    }

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::addAggregateEnergyOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    char tBuffer[100];
    pugi::xml_node tmp_node2;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "Aggregator");
    addChild(tmp_node, "Name", "AggregateEnergy");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Value");

    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node1.append_child("For");
      for_node.append_attribute("var") = "sampleIndex";
      for_node.append_attribute("in") = "Samples";
      tmp_node2 = for_node.append_child("Input"); 
      addChild(tmp_node2, "ArgumentName", "Value {sampleIndex}");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        tmp_node2 = tmp_node1.append_child("Input");
        std::string tTmpString = "Value ";
        tTmpString += Plato::to_string(i+1);
        addChild(tmp_node2, "ArgumentName", tTmpString);
      }
    }

    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Value");

    tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Nodal Field");

    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node1.append_child("For");
      for_node.append_attribute("var") = "sampleIndex";
      for_node.append_attribute("in") = "Samples";
      tmp_node2 = for_node.append_child("Input"); 
      addChild(tmp_node2, "ArgumentName", "Field {sampleIndex}");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        tmp_node2 = tmp_node1.append_child("Input");
        std::string tTmpString = "Field ";
        tTmpString += Plato::to_string(i+1);
        addChild(tmp_node2, "ArgumentName", tTmpString);
      }
    }


    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Field");

    tmp_node1 = tmp_node.append_child("Weighting");

    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node1.append_child("For");
      for_node.append_attribute("var") = "sampleIndex";
      for_node.append_attribute("in") = "Samples";
      tmp_node2 = for_node.append_child("Weight"); 
      addChild(tmp_node2, "Value", "1");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
          tmp_node2 = tmp_node1.append_child("Weight");
          addChild(tmp_node2, "Value", m_InputData.objectives[i].weight.c_str());
      }
    }

    if(m_InputData.mUseNormalizationInAggregator == "true")
    {
        if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
        {
          std::cerr << "Error: Normalization is not supported with uncertain loads" << std::endl;
          return false;
        }
        tmp_node2 = tmp_node1.append_child("Normals");
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
            pugi::xml_node tmp_node3 = tmp_node2.append_child("Input");
            sprintf(tBuffer, "Normalization Factor %d", (int)(i+1));
            addChild(tmp_node3, "ArgumentName", tBuffer);
        }
    }

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::addAggregateValuesOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    char tBuffer[100];
    pugi::xml_node tmp_node2;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "Aggregator");
    addChild(tmp_node, "Name", "AggregateValues");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Value");
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node1.append_child("For");
      for_node.append_attribute("var") = "sampleIndex";
      for_node.append_attribute("in") = "Samples";
      tmp_node2 = for_node.append_child("Input"); 
      addChild(tmp_node2, "ArgumentName", "Value {sampleIndex}");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
          tmp_node2 = tmp_node1.append_child("Input");
          std::string tTmpString = "Value ";
          tTmpString += Plato::to_string(i+1);
          addChild(tmp_node2, "ArgumentName", tTmpString);
      }
    }
    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Values");

    tmp_node1 = tmp_node.append_child("Weighting");
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node1.append_child("For");
      for_node.append_attribute("var") = "sampleIndex";
      for_node.append_attribute("in") = "Samples";
      tmp_node2 = for_node.append_child("Weight"); 
      addChild(tmp_node2, "Value", "1");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
          tmp_node2 = tmp_node1.append_child("Weight");
          addChild(tmp_node2, "Value", m_InputData.objectives[i].weight.c_str());
      }
    }
    if(m_InputData.mUseNormalizationInAggregator == "true")
    {
        if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
        {
          std::cerr << "Error: Normalization is not supported with uncertain loads" << std::endl;
          return false;
        }
        tmp_node2 = tmp_node1.append_child("Normals");
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
            pugi::xml_node tmp_node3 = tmp_node2.append_child("Input");
            sprintf(tBuffer, "Normalization Factor %d", (int)(i+1));
            addChild(tmp_node3, "ArgumentName", tBuffer);
        }
    }

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::addAggregateHessianOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    char tBuffer[100];
    pugi::xml_node tmp_node2;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "Aggregator");
    addChild(tmp_node, "Name", "AggregateHessian");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Nodal Field");
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node1.append_child("For");
      for_node.append_attribute("var") = "sampleIndex";
      for_node.append_attribute("in") = "Samples";
      tmp_node2 = for_node.append_child("Input"); 
      addChild(tmp_node2, "ArgumentName", "Field {sampleIndex}");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
          tmp_node2 = tmp_node1.append_child("Input");
          std::string tTmpString = "Field ";
          tTmpString += Plato::to_string(i+1);
          addChild(tmp_node2, "ArgumentName", tTmpString);
      }
    }
    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Field");

    tmp_node1 = tmp_node.append_child("Weighting");
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node1.append_child("For");
      for_node.append_attribute("var") = "sampleIndex";
      for_node.append_attribute("in") = "Samples";
      tmp_node2 = for_node.append_child("Weight"); 
      addChild(tmp_node2, "Value", "1");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
          tmp_node2 = tmp_node1.append_child("Weight");
          addChild(tmp_node2, "Value", m_InputData.objectives[i].weight.c_str());
      }
    }
    if(m_InputData.mUseNormalizationInAggregator == "true")
    {
        if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
        {
          std::cerr << "Error: Normalization is not supported with uncertain loads" << std::endl;
          return false;
        }
        tmp_node2 = tmp_node1.append_child("Normals");
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
            pugi::xml_node tmp_node3 = tmp_node2.append_child("Input");
            sprintf(tBuffer, "Normalization Factor %d", (int)(i+1));
            addChild(tmp_node3, "ArgumentName", tBuffer);
        }
    }

    return true;
}

/******************************************************************************/
void DefaultInputGenerator::addSetLowerBoundsOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "SetLowerBounds");
    addChild(tmp_node, "Discretization", m_InputData.discretization);
    addChild(tmp_node, "Name", "Calculate Lower Bounds");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Lower Bound Value");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Lower Bound Vector");
    if(m_InputData.fixed_block_ids.size() > 0)
    {
        tmp_node1 = tmp_node.append_child("FixedBlocks");
        for(size_t i=0; i<m_InputData.fixed_block_ids.size(); ++i)
        {
            addChild(tmp_node1, "Index", m_InputData.fixed_block_ids[i].c_str());
        }
    }
    if(m_InputData.fixed_sideset_ids.size() > 0)
    {
        tmp_node1 = tmp_node.append_child("FixedSidesets");
        for(size_t i=0; i<m_InputData.fixed_sideset_ids.size(); ++i)
        {
            addChild(tmp_node1, "Index", m_InputData.fixed_sideset_ids[i].c_str());
        }
    }
    if(m_InputData.fixed_nodeset_ids.size() > 0)
    {
        tmp_node1 = tmp_node.append_child("FixedNodesets");
        for(size_t i=0; i<m_InputData.fixed_nodeset_ids.size(); ++i)
        {
            addChild(tmp_node1, "Index", m_InputData.fixed_nodeset_ids[i].c_str());
        }
    }
}

/******************************************************************************/
void DefaultInputGenerator::addSetUpperBoundsOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "SetUpperBounds");
    addChild(tmp_node, "Discretization", m_InputData.discretization);
    addChild(tmp_node, "Name", "Calculate Upper Bounds");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Upper Bound Value");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Upper Bound Vector");
    if(m_InputData.fixed_block_ids.size() > 0)
    {
        tmp_node1 = tmp_node.append_child("FixedBlocks");
        for(size_t i=0; i<m_InputData.fixed_block_ids.size(); ++i)
        {
            addChild(tmp_node1, "Index", m_InputData.fixed_block_ids[i].c_str());
        }
    }
    if(m_InputData.fixed_sideset_ids.size() > 0)
    {
        tmp_node1 = tmp_node.append_child("FixedSidesets");
        for(size_t i=0; i<m_InputData.fixed_sideset_ids.size(); ++i)
        {
            addChild(tmp_node1, "Index", m_InputData.fixed_sideset_ids[i].c_str());
        }
    }
    if(m_InputData.fixed_nodeset_ids.size() > 0)
    {
        tmp_node1 = tmp_node.append_child("FixedNodesets");
        for(size_t i=0; i<m_InputData.fixed_nodeset_ids.size(); ++i)
        {
            addChild(tmp_node1, "Index", m_InputData.fixed_nodeset_ids[i].c_str());
        }
    }
}

/******************************************************************************/
void DefaultInputGenerator::addEnforceBoundsOperationToFile(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "EnforceBounds");
    addChild(tmp_node, "Name", "EnforceBounds");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Upper Bound Vector");
    tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Lower Bound Vector");
    tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Topology");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Topology");
}

/******************************************************************************/
void DefaultInputGenerator::addEnforceBoundsOperationToStage(pugi::xml_node &aStageNode)
/******************************************************************************/
{
    pugi::xml_node tOperationNode = aStageNode.append_child("Operation");
    addChild(tOperationNode, "Name", "EnforceBounds");
    addChild(tOperationNode, "PerformerName", "PlatoMain");
    pugi::xml_node tInputNode = tOperationNode.append_child("Input");
    addChild(tInputNode, "ArgumentName", "Lower Bound Vector");
    addChild(tInputNode, "SharedDataName", "Lower Bound Vector");
    tInputNode = tOperationNode.append_child("Input");
    addChild(tInputNode, "ArgumentName", "Upper Bound Vector");
    addChild(tInputNode, "SharedDataName", "Upper Bound Vector");
    tInputNode = tOperationNode.append_child("Input");
    addChild(tInputNode, "ArgumentName", "Topology");
    addChild(tInputNode, "SharedDataName", "Topology");
    pugi::xml_node tOutputNode = tOperationNode.append_child("Output");
    addChild(tOutputNode, "ArgumentName", "Topology");
    addChild(tOutputNode, "SharedDataName", "Topology");
}

/******************************************************************************/
void DefaultInputGenerator::addDesignVolumeOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "DesignVolume");
    addChild(tmp_node, "Name", "Design Volume");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Design Volume");
}

/******************************************************************************/
void DefaultInputGenerator::addComputeVolumeOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "ComputeVolume");
    addChild(tmp_node, "Name", "Compute Current Volume");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Topology");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Volume");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Volume Gradient");
    addChild(tmp_node, "PenaltyModel", "SIMP");
    tmp_node1 = tmp_node.append_child("SIMP");
    addChild(tmp_node1, "PenaltyExponent", "1.0");
    addChild(tmp_node1, "MinimumValue", "0.0");
}

/******************************************************************************/
void DefaultInputGenerator::addInitializeFieldOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    // InitializeField
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "InitializeField");
    addChild(tmp_node, "Name", "Initialize Field");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Initialized Field");
    if(m_InputData.initial_guess_filename != "")
    {
        // This handles both restarting from an iteration from a previous
        // run or by just specifying an initial guess filename.
        addChild(tmp_node, "Method", "FromFieldOnInputMesh");
        tmp_node1 = tmp_node.append_child("FromFieldOnInputMesh");
        addChild(tmp_node1, "Name", m_InputData.run_mesh_name);
        addChild(tmp_node1, "VariableName", m_InputData.initial_guess_field_name);
        if(m_InputData.restart_iteration != "")
            addChild(tmp_node1, "Iteration", m_InputData.restart_iteration);
    }
    else
    {
        if(m_InputData.discretization == "density")
        {
            addChild(tmp_node, "Method", "Uniform");
            tmp_node1 = tmp_node.append_child("Uniform");
            addChild(tmp_node1, "Value", m_InputData.initial_density_value);
        }
        else if(m_InputData.discretization == "levelset")
        {
            if(m_InputData.levelset_initialization_method == "primitives")
            {
                addChild(tmp_node, "Method", "PrimitivesLevelSet");
                tmp_node1 = tmp_node.append_child("PrimitivesLevelSet");
                addChild(tmp_node1, "BackgroundMeshName", m_InputData.run_mesh_name);
                if(m_InputData.levelset_material_box_min != "" &&
                   m_InputData.levelset_material_box_max != "")
                {
                    pugi::xml_node tmp_node2 = tmp_node1.append_child("MaterialBox");
                    addChild(tmp_node2, "MinCoords", m_InputData.levelset_material_box_min);
                    addChild(tmp_node2, "MaxCoords", m_InputData.levelset_material_box_max);
                }
            }
            else
            {
                addChild(tmp_node, "Method", "SwissCheeseLevelSet");
                tmp_node1 = tmp_node.append_child("SwissCheeseLevelSet");
                std::string tCreateLevelsetSpheres = m_InputData.create_levelset_spheres;
                if(tCreateLevelsetSpheres == "")
                    tCreateLevelsetSpheres = "false";
                addChild(tmp_node1, "CreateSpheres", tCreateLevelsetSpheres);
                if(tCreateLevelsetSpheres == "true" && m_InputData.levelset_sphere_radius != "")
                    addChild(tmp_node1, "SphereRadius", m_InputData.levelset_sphere_radius);
                if(tCreateLevelsetSpheres == "true" && m_InputData.levelset_sphere_packing_factor != "")
                    addChild(tmp_node1, "SpherePackingFactor", m_InputData.levelset_sphere_packing_factor);
                for(size_t r=0; r<m_InputData.levelset_nodesets.size(); ++r)
                {
                    addChild(tmp_node1, "NodeSet", m_InputData.levelset_nodesets[r]);
                }
                addChild(tmp_node1, "BackgroundMeshName", m_InputData.run_mesh_name);
            }
        }
    }
}

/******************************************************************************/
void DefaultInputGenerator::addUpdateGeometryOnChangeOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    // Update Geometry on Change
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "SystemCall");
    addChild(tmp_node, "Name", "Update Geometry on Change");
    addChild(tmp_node, "Command", "python aflr.py");
    addChild(tmp_node, "OnChange", "true");
    addChild(tmp_node, "Argument", m_InputData.csm_filename);
    addChild(tmp_node, "Argument", m_InputData.csm_exodus_filename);
    addChild(tmp_node, "Argument", m_InputData.csm_tesselation_filename);
    addChild(tmp_node, "AppendInput", "true");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Parameters");
}

/******************************************************************************/
void DefaultInputGenerator::addInitializeValuesOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    // InitializeField
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "InitializeValues");
    addChild(tmp_node, "Name", "Initialize Values");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Values");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Lower Bounds");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Upper Bounds");
    if(m_InputData.csm_filename.length() > 0)
    {
        addChild(tmp_node, "Method", "ReadFromCSMFile");
        addChild(tmp_node, "CSMFileName", m_InputData.csm_filename);
    }
    else
    {
        addChild(tmp_node, "Method", "UniformValue");
        addChild(tmp_node, "InitialValue", m_InputData.initial_density_value);
    }
}

/******************************************************************************/
void DefaultInputGenerator::addFilterInfo(pugi::xml_document &aDoc)
/******************************************************************************/
{
    // Filter
    pugi::xml_node tmp_node = aDoc.append_child("Filter");
    if(m_InputData.filter_type == m_InputData.m_filterType_kernelThenHeaviside_generatorName)
    {
        // kernel then heaviside
        addChild(tmp_node, "Name", m_InputData.m_filterType_kernelThenHeaviside_XMLName);
    }
    else if(m_InputData.filter_type == m_InputData.m_filterType_identity_generatorName)
    {
        // identity
        addChild(tmp_node, "Name", m_InputData.m_filterType_identity_XMLName);
    }
    else if(m_InputData.filter_type == m_InputData.m_filterType_kernelThenTANH_generatorName)
    {
        // kernel then tanh
        addChild(tmp_node, "Name", m_InputData.m_filterType_kernelThenTANH_XMLName);
    }
    else
    {
        // kernel is default
        addChild(tmp_node, "Name", m_InputData.m_filterType_kernel_XMLName);
    }

    if(m_InputData.filter_radius_scale != "")
    {
        addChild(tmp_node, "Scale", m_InputData.filter_radius_scale);
    }
    if(m_InputData.filter_radius_absolute != "")
    {
        addChild(tmp_node, "Absolute", m_InputData.filter_radius_absolute);
    }
    if(m_InputData.filter_projection_start_iteration != "")
    {
        addChild(tmp_node, "StartIteration", m_InputData.filter_projection_start_iteration);
    }
    if(m_InputData.filter_projection_update_interval != "")
    {
        addChild(tmp_node, "UpdateInterval", m_InputData.filter_projection_update_interval);
    }
    if(m_InputData.filter_use_additive_continuation != "")
    {
        addChild(tmp_node, "UseAdditiveContinuation", m_InputData.filter_use_additive_continuation);
    }
    if(m_InputData.filter_power != "")
    {
        addChild(tmp_node, "Power", m_InputData.filter_power);
    }
    if(m_InputData.filter_heaviside_min != "")
    {
        addChild(tmp_node, "HeavisideMin", m_InputData.filter_heaviside_min);
    }
    if(m_InputData.filter_heaviside_update != "")
    {
        addChild(tmp_node, "HeavisideUpdate", m_InputData.filter_heaviside_update);
    }
    if(m_InputData.filter_heaviside_max != "")
    {
        addChild(tmp_node, "HeavisideMax", m_InputData.filter_heaviside_max);
    }
}

/******************************************************************************/
void DefaultInputGenerator::addUpdateProblemOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "Update Problem");
    addChild(tmp_node, "Name", "Update Problem");
}

/******************************************************************************/
void DefaultInputGenerator::addPlatoMainOutputOperation(pugi::xml_document &aDoc,
                                               const bool &aHasUncertainties,
                                               const bool &aRequestedVonMises)
/******************************************************************************/
{
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "PlatoMainOutput");
    addChild(tmp_node, "Name", "PlatoMainOutput");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Topology");
    tmp_node1 = tmp_node.append_child("Input");
    if(m_InputData.mPlatoAnalyzePerformerExists)
      addChild(tmp_node1, "ArgumentName", "Objective Gradient");
    else
      addChild(tmp_node1, "ArgumentName", "Internal Energy Gradient");
    tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Optimization DOFs");
    if(m_InputData.constraints.size() > 0)
    {
        if(m_InputData.constraints[0].type == "volume")
        {
            tmp_node1 = tmp_node.append_child("Input");
            addChild(tmp_node1, "ArgumentName", "Volume Gradient");
        }
        else if(m_InputData.constraints[0].type == "surface area")
        {
            tmp_node1 = tmp_node.append_child("Input");
            addChild(tmp_node1, "ArgumentName", "Surface Area Gradient");
        }
    }

    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node.append_child("For");
      for_node.append_attribute("var") = "sampleIndex";
      for_node.append_attribute("in") = "Samples";

      tmp_node1 = for_node.append_child("Input");
      addChild(tmp_node1, "ArgumentName", "plato_analyze_{sampleIndex}_vonmises");
      addChild(tmp_node1, "Layout", "Element Field");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        XMLGen::Objective cur_obj = m_InputData.objectives[i];
        if(aHasUncertainties)
        {
          // Only handling vonmises for UQ runs and only one load case
          // per objective/performer.
          tmp_node1 = tmp_node.append_child("Input");
          addChild(tmp_node1, "ArgumentName", cur_obj.mPerformerName + "_vonmises");
          addChild(tmp_node1, "Layout", "Element Field");
        }
        else if(cur_obj.multi_load_case == "true")
        {
          for(size_t k=0; k<cur_obj.load_case_ids.size(); k++)
          {
            std::string cur_load_string = cur_obj.load_case_ids[k];
            for(size_t j=0; j<cur_obj.output_for_plotting.size(); j++)
            {
              tmp_node1 = tmp_node.append_child("Input");
              addChild(tmp_node1, "ArgumentName", cur_obj.mPerformerName + "_load" + cur_load_string + "_" + cur_obj.output_for_plotting[j]);
              if(cur_obj.output_for_plotting[j] == "vonmises")
                addChild(tmp_node1, "Layout", "Element Field");
            }
          }
        }
        else
        {
          for(size_t j=0; j<cur_obj.output_for_plotting.size(); j++)
          {
            tmp_node1 = tmp_node.append_child("Input");
            addChild(tmp_node1, "ArgumentName", cur_obj.mPerformerName + "_" + cur_obj.output_for_plotting[j]);
            if(cur_obj.output_for_plotting[j] == "vonmises" ||
                cur_obj.output_for_plotting[j] == "Vonmises")
              addChild(tmp_node1, "Layout", "Element Field");
          }
        }
      }
    }

    if(aHasUncertainties && aRequestedVonMises)
    {
        tmp_node1 = tmp_node.append_child("Input");
        addChild(tmp_node1, "ArgumentName", "VonMises Mean");
        addChild(tmp_node1, "Layout", "Element Field");
        tmp_node1 = tmp_node.append_child("Input");
        addChild(tmp_node1, "ArgumentName", "VonMises StdDev");
        addChild(tmp_node1, "Layout", "Element Field");
        for(size_t j=0; j<m_InputData.mStandardDeviations.size(); ++j)
        {
            tmp_node1 = tmp_node.append_child("Input");
            std::string tTempString = "VonMises Mean Plus ";
            tTempString += m_InputData.mStandardDeviations[j];
            tTempString += " StdDev";
            addChild(tmp_node1, "ArgumentName", tTempString);
            addChild(tmp_node1, "Layout", "Element Field");
        }
    }

    addChild(tmp_node, "WriteRestart", m_InputData.write_restart_file);
    addChild(tmp_node, "OutputFrequency", m_InputData.output_frequency);
    addChild(tmp_node, "MaxIterations", m_InputData.max_iterations);
    tmp_node1 = tmp_node.append_child("SurfaceExtraction");
    addChild(tmp_node1, "OutputMethod", m_InputData.output_method);
    addChild(tmp_node1, "Discretization", m_InputData.discretization);
    pugi::xml_node tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "Format", "Exodus");

    if(aHasUncertainties)
    {
        addStochasticObjectiveValueOperation(aDoc);
        addStochasticObjectiveGradientOperation(aDoc);
        if(aRequestedVonMises)
            addVonMisesStatisticsOperation(aDoc);
    }
}

/******************************************************************************/
void DefaultInputGenerator::addCSMMeshOutputOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "CSMMeshOutput");
    addChild(tmp_node, "Name", "CSMMeshOutput");
    addChild(tmp_node, "BaseMeshName", m_InputData.csm_exodus_filename);
    addChild(tmp_node, "OutputFrequency", m_InputData.output_frequency);
    addChild(tmp_node, "MaxIterations", m_InputData.max_iterations);
}

/******************************************************************************/
void DefaultInputGenerator::addFilterGradientOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "Filter");
    addChild(tmp_node, "Name", "FilterGradient");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Field");
    tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Gradient");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Filtered Gradient");
    addChild(tmp_node, "Gradient", "True");
}

/******************************************************************************/
void DefaultInputGenerator::addFilterHessianOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "Filter");
    addChild(tmp_node, "Name", "FilterHessian");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Field");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Filtered Field");
    addChild(tmp_node, "Gradient", "False");
}

/******************************************************************************/
void DefaultInputGenerator::addStochasticObjectiveGradientOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node1, tmp_node2;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "MeanPlusStdDevGradient");
    addChild(tmp_node, "Name", "Stochastic Objective Gradient");

    tmp_node1 = tmp_node.append_child("CriterionValue");
    addChild(tmp_node1, "Layout", "Global");
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node1.append_child("For");
      for_node.append_attribute("var") = "performerIndex";
      for_node.append_attribute("in") = "Performers";
      for_node = for_node.append_child("For");
      for_node.append_attribute("var") = "PerformerSampleIndex";
      for_node.append_attribute("in") = "PerformerSamples";
      pugi::xml_node input_node = for_node.append_child("Input");
      addChild(input_node, "ArgumentName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
      addChild(input_node, "Probability", "{Probabilities[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        XMLGen::Objective cur_obj = m_InputData.objectives[i];
        pugi::xml_node input_node = tmp_node1.append_child("Input");
        std::string tTmpString = "";
        if(cur_obj.code_name == "plato_analyze")
          tTmpString += "Objective Value ";
        else
          tTmpString += "Internal Energy ";
        tTmpString += Plato::to_string(i+1);
        addChild(input_node, "ArgumentName", tTmpString);
        addChild(input_node, "Probability", Plato::to_string(m_InputData.load_case_probabilities[i]).c_str());
      }
    }
    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "Statistic", "mean");
    addChild(tmp_node2, "ArgumentName", "objective_mean");
    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "Statistic", "std_dev");
    addChild(tmp_node2, "ArgumentName", "objective_std_dev");

    tmp_node1 = tmp_node.append_child("CriterionGradient");
    addChild(tmp_node1, "Layout", "Nodal Field");
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = tmp_node1.append_child("For");
      for_node.append_attribute("var") = "performerIndex";
      for_node.append_attribute("in") = "Performers";
      for_node = for_node.append_child("For");
      for_node.append_attribute("var") = "PerformerSampleIndex";
      for_node.append_attribute("in") = "PerformerSamples";
      pugi::xml_node input_node = for_node.append_child("Input");
      addChild(input_node, "ArgumentName", "Objective {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex} Gradient");
      addChild(input_node, "Probability", "{Probabilities[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        XMLGen::Objective cur_obj = m_InputData.objectives[i];
        pugi::xml_node input_node = tmp_node1.append_child("Input");
        std::string tTmpString = "";
        if(cur_obj.code_name == "plato_analyze")
          tTmpString += "Objective ";
        else
          tTmpString += "Internal Energy ";
        tTmpString += Plato::to_string(i+1);
        tTmpString += " Gradient";
        addChild(input_node, "ArgumentName", tTmpString);
        addChild(input_node, "Probability", Plato::to_string(m_InputData.load_case_probabilities[i]).c_str());
      }
    }
    tmp_node2 = tmp_node1.append_child("Output");
    std::string tTmpString = "mean_plus_";
    tTmpString += m_InputData.objective_number_standard_deviations;
    tTmpString += "_std_dev";
    addChild(tmp_node2, "Statistic", tTmpString);
    tTmpString = "Objective Mean Plus ";
    tTmpString += m_InputData.objective_number_standard_deviations;
    tTmpString += " StdDev Gradient";
    addChild(tmp_node2, "ArgumentName", tTmpString);
}

/******************************************************************************/
bool DefaultInputGenerator::generatePlatoMainOperationsXML()
/******************************************************************************/
{
    pugi::xml_document doc;
    pugi::xml_node tmp_node, tmp_node1, tmp_node2;

    // Version entry
    tmp_node = doc.append_child(pugi::node_declaration);
    tmp_node.set_name("xml");
    pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
    tmp_att.set_value("1.0");

    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node def_node = doc.append_child("include");
      def_node.append_attribute("filename") = "defines.xml";
    }

    //////////////////////////////////////////////////
    // Operations
    /////////////////////////////////////////////////

    if(m_InputData.optimization_type == "topology")
    {
        addFilterInfo(doc);
        addPlatoMainOutputOperation(doc, m_InputData.m_HasUncertainties, m_InputData.m_RequestedVonMisesOutput);
        addUpdateProblemOperation(doc);
        addFilterControlOperation(doc);
        addFilterGradientOperation(doc);
        if(m_InputData.optimization_algorithm =="ksbc" ||
                m_InputData.optimization_algorithm == "ksal" ||
                m_InputData.optimization_algorithm == "rol ksal" ||
                m_InputData.optimization_algorithm == "rol ksbc")
        {
            addFilterHessianOperation(doc);
        }
        addInitializeFieldOperation(doc);
        if(m_InputData.discretization == "density")
        {
            addDesignVolumeOperation(doc);
            addComputeVolumeOperation(doc);
        }
        else if(m_InputData.discretization == "levelset")
        {
        }
        if(!addAggregateEnergyOperation(doc))
          return false;
        if(!addAggregateGradientOperation(doc))
          return false;
        if(m_InputData.optimization_algorithm =="ksbc" ||
                m_InputData.optimization_algorithm == "ksal" ||
                m_InputData.optimization_algorithm == "rol ksal" ||
                m_InputData.optimization_algorithm == "rol ksbc")
        {
            addAggregateHessianOperation(doc);
        }
        addSetLowerBoundsOperation(doc);
        addSetUpperBoundsOperation(doc);
        if(!m_InputData.mPlatoAnalyzePerformerExists)
            addEnforceBoundsOperationToFile(doc);
    }
    else if(m_InputData.optimization_type == "shape")
    {
        addCSMMeshOutputOperation(doc);
        addInitializeValuesOperation(doc);
        addAggregateValuesOperation(doc);
        addUpdateGeometryOnChangeOperation(doc);
    }

    // Write the file to disk
    doc.save_file("plato_main_operations.xml", "  ");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generatePlatoMainOperationsXMLForShape()
/******************************************************************************/
{
    pugi::xml_document doc;
    pugi::xml_node tmp_node;

    // Version entry
    tmp_node = doc.append_child(pugi::node_declaration);
    tmp_node.set_name("xml");
    pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
    tmp_att.set_value("1.0");

    // Timers
    tmp_node = doc.append_child("Timers");
    addChild(tmp_node, "time", "true");

    addCSMMeshOutputOperation(doc);
    addInitializeValuesOperation(doc);
    addAggregateValuesOperation(doc);
    addUpdateGeometryOnChangeOperation(doc);

    // Write the file to disk
    doc.save_file("plato_main_operations.xml", "  ");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::outputVolumeGradientStage(pugi::xml_document &doc)
/******************************************************************************/
{
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Volume Gradient");

    pugi::xml_node input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Optimization DOFs");

    // Filter control
    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterControl");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Field");
    addChild(output_node, "SharedDataName", "Topology");

    // Enforce Bounds operation
    if(!m_InputData.mPlatoAnalyzePerformerExists)
        addEnforceBoundsOperationToStage(stage_node);

    // Compute current volume
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Compute Current Volume");
    addChild(op_node, "PerformerName", "PlatoMain");

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Volume");
    addChild(output_node, "SharedDataName", "Volume");
    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Volume Gradient");
    addChild(output_node, "SharedDataName", "Volume Gradient");

    // Filter gradient
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterGradient");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Gradient");
    addChild(input_node, "SharedDataName", "Volume Gradient");
    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Gradient");
    addChild(output_node, "SharedDataName", "Volume Gradient");

    // stage output
    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Volume Gradient");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::outputSurfaceAreaGradientStage(pugi::xml_document &doc)
/******************************************************************************/
{
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Surface Area Gradient");

    pugi::xml_node input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Optimization DOFs");

    // Filter control
    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterControl");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Field");
    addChild(output_node, "SharedDataName", "Topology");

    // Enforce Bounds operation
    if(!m_InputData.mPlatoAnalyzePerformerExists)
        addEnforceBoundsOperationToStage(stage_node);

    // Compute current surface area gradient
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Compute Surface Area Gradient");
    addChild(op_node, "PerformerName", m_InputData.objectives[0].mPerformerName.c_str());

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "SurfaceAreaGradient");
    addChild(output_node, "SharedDataName", "Surface Area Gradient");

    // Filter gradient
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterGradient");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Gradient");
    addChild(input_node, "SharedDataName", "Surface Area Gradient");
    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Gradient");
    addChild(output_node, "SharedDataName", "Surface Area Gradient");

    // stage output
    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Surface Area Gradient");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::outputVolumeStage(pugi::xml_document &doc)
/******************************************************************************/
{
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Volume");

    pugi::xml_node input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Optimization DOFs");

    // Filter control operation
    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterControl");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Field");
    addChild(output_node, "SharedDataName", "Topology");

    // Enforce Bounds operation
    if(!m_InputData.mPlatoAnalyzePerformerExists)
        addEnforceBoundsOperationToStage(stage_node);

    // Compute current volume operation
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Compute Current Volume");
    addChild(op_node, "PerformerName", "PlatoMain");

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Volume");
    addChild(output_node, "SharedDataName", "Volume");
    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Volume Gradient");
    addChild(output_node, "SharedDataName", "Volume Gradient");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Volume");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::outputConstraintStage(pugi::xml_document &doc)
/******************************************************************************/
{
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Constraint");

    pugi::xml_node input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Design Parameters");

    // Update Geometry on Change operation
    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Update Geometry on Change");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Parameters");
    addChild(input_node, "SharedDataName", "Design Parameters");

    // Reinitialize on Change operation
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Reinitialize on Change");
    addChild(op_node, "PerformerName", m_InputData.objectives[0].mPerformerName);
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Parameters");
    addChild(input_node, "SharedDataName", "Design Parameters");

    // Compute constraint value operation
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Compute Constraint Value");
    addChild(op_node, "PerformerName", m_InputData.objectives[0].mPerformerName);

    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Constraint Value");
    addChild(output_node, "SharedDataName", "Constraint");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Constraint");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::outputConstraintGradientStage(pugi::xml_document &doc)
/******************************************************************************/
{
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Constraint Gradient");

    pugi::xml_node input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Design Parameters");

    // Update Geometry on Change operation
    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Update Geometry on Change");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Parameters");
    addChild(input_node, "SharedDataName", "Design Parameters");

    // Reinitialize on Change operation
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Reinitialize on Change");
    addChild(op_node, "PerformerName", m_InputData.objectives[0].mPerformerName);
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Parameters");
    addChild(input_node, "SharedDataName", "Design Parameters");

    // Compute constraint gradient value operation
    op_node = stage_node.append_child("Operation");
    pugi::xml_node op_node2 = op_node.append_child("Operation");
    addChild(op_node2, "Name", "Compute Constraint Gradient");
    addChild(op_node2, "PerformerName", m_InputData.objectives[0].mPerformerName);

    pugi::xml_node tForNode = op_node.append_child("For");
    tForNode.append_attribute("var") = "I";
    tForNode.append_attribute("in") = "Parameters";
    op_node2 = tForNode.append_child("Operation");
    addChild(op_node2, "Name", "Compute Shape Sensitivity On Change");
    addChild(op_node2, "PerformerName", "PlatoESP_{I}");
    pugi::xml_node tmp_node = op_node2.append_child("Parameter");
    addChild(tmp_node, "ArgumentName", "Parameter Index");
    addChild(tmp_node, "ArgumentValue", "{I-1}");
    tmp_node = op_node2.append_child("Input");
    addChild(tmp_node, "ArgumentName", "Parameters");
    addChild(tmp_node, "SharedDataName", "Design Parameters");
    tmp_node = op_node2.append_child("Output");
    addChild(tmp_node, "ArgumentName", "Parameter Sensitivity");
    addChild(tmp_node, "SharedDataName", "Parameter Sensitivity {I}");

    op_node2 = stage_node.append_child("Operation");
    addChild(op_node2, "Name", "Compute Constraint Sensitivity");
    addChild(op_node2, "PerformerName", m_InputData.objectives[0].mPerformerName);
    tForNode = op_node2.append_child("For");
    tForNode.append_attribute("var") = "I";
    tForNode.append_attribute("in") = "Parameters";
    tmp_node = tForNode.append_child("Input");
    addChild(tmp_node, "ArgumentName", "Parameter Sensitivity {I}");
    addChild(tmp_node, "SharedDataName", "Parameter Sensitivity {I}");
    tmp_node = op_node2.append_child("Output");
    addChild(tmp_node, "ArgumentName", "Constraint Sensitivity");
    addChild(tmp_node, "SharedDataName", "Constraint Gradient");

    pugi::xml_node output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Constraint Gradient");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::outputSurfaceAreaStage(pugi::xml_document &doc)
/******************************************************************************/
{
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Surface Area");

    pugi::xml_node input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Optimization DOFs");

    // Filter control operation
    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterControl");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Field");
    addChild(output_node, "SharedDataName", "Topology");

    // Enforce Bounds operation
    if(!m_InputData.mPlatoAnalyzePerformerExists)
        addEnforceBoundsOperationToStage(stage_node);

    // Compute current surface area operation
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Compute Surface Area");
    addChild(op_node, "PerformerName", m_InputData.objectives[0].mPerformerName.c_str());

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "SurfaceArea");
    addChild(output_node, "SharedDataName", "Surface Area");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Surface Area");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::outputComputeStateStage(pugi::xml_document &doc)
/******************************************************************************/
{
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Compute State");

    pugi::xml_node input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Optimization DOFs");

    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterControl");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Field");
    addChild(output_node, "SharedDataName", "Topology");

    // Enforce Bounds operation
    if(!m_InputData.mPlatoAnalyzePerformerExists)
        addEnforceBoundsOperationToStage(stage_node);

    pugi::xml_node cur_parent = stage_node;
    if(m_InputData.objectives.size() > 1)
    {
        op_node = stage_node.append_child("Operation");
        cur_parent = op_node;
    }

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        op_node = cur_parent.append_child("Operation");
        addChild(op_node, "Name", "Compute Displacement");
        addChild(op_node, "PerformerName", m_InputData.objectives[i].mPerformerName.c_str());

        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Topology");
        addChild(input_node, "SharedDataName", "Topology");
    }

    return true;
}


/******************************************************************************/
bool DefaultInputGenerator::outputObjectiveStage(pugi::xml_document &doc,
                                             const bool &aHasUncertainties)
/******************************************************************************/
{
    char tmp_buf[200];
    // Internal Energy
    pugi::xml_node stage_node = doc.append_child("Stage");
    pugi::xml_node cur_parent = stage_node;
    if(m_InputData.mPlatoAnalyzePerformerExists)
      addChild(stage_node, "Name", "Objective Value");
    else
      addChild(stage_node, "Name", "Internal Energy");

    pugi::xml_node input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Optimization DOFs");

    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterControl");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Field");
    addChild(output_node, "SharedDataName", "Topology");

    // Enforce Bounds operation
    if(!m_InputData.mPlatoAnalyzePerformerExists)
        addEnforceBoundsOperationToStage(stage_node);

    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      addComputeObjectiveValueOperationForNewUncertaintyWorkflow(stage_node);
    }
    else
    {
      if(m_InputData.objectives.size() > 1)
      {
          op_node = stage_node.append_child("Operation");
          cur_parent = op_node;
      }

      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
          XMLGen::Objective cur_obj = m_InputData.objectives[i];
          op_node = cur_parent.append_child("Operation");
          if(cur_obj.code_name == "plato_analyze")
          {
              addChild(op_node, "Name", "Compute Objective Value");
              addChild(op_node, "PerformerName", cur_obj.mPerformerName.c_str());
          }
          else
          {
              addChild(op_node, "Name", "Compute Objective");
              addChild(op_node, "PerformerName", cur_obj.mPerformerName.c_str());
          }

          input_node = op_node.append_child("Input");
          addChild(input_node, "ArgumentName", "Topology");
          addChild(input_node, "SharedDataName", "Topology");

          output_node = op_node.append_child("Output");
          if(m_InputData.mPlatoAnalyzePerformerExists)
            addChild(output_node, "ArgumentName", "Objective Value");
          else
            addChild(output_node, "ArgumentName", "Internal Energy");
          if(cur_obj.code_name == "plato_analyze")
            sprintf(tmp_buf, "Objective Value %d", (int)(i+1));
          else
            sprintf(tmp_buf, "Internal Energy %d", (int)(i+1));

          addChild(output_node, "SharedDataName", tmp_buf);
      }
    }

    // AggregateEnergy
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "AggregateEnergy");
    addChild(op_node, "PerformerName", "PlatoMain");

    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
        pugi::xml_node for_node = op_node.append_child("For");
        for_node.append_attribute("var") = "performerIndex";
        for_node.append_attribute("in") = "Performers";
        for_node = for_node.append_child("For");
        for_node.append_attribute("var") = "PerformerSampleIndex";
        for_node.append_attribute("in") = "PerformerSamples";
        input_node = for_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
        addChild(input_node, "SharedDataName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");

        if(m_InputData.mUseNormalizationInAggregator == "true")
        {
          input_node = for_node.append_child("Input");
          addChild(input_node, "ArgumentName", "Normalization Factor {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
          addChild(input_node, "SharedDataName", "Initial Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
        }
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        input_node = op_node.append_child("Input");
        sprintf(tmp_buf, "Value %d", (int)(i+1));
        addChild(input_node, "ArgumentName", tmp_buf);
        if(m_InputData.objectives[i].code_name == "plato_analyze")
          sprintf(tmp_buf, "Objective Value %d", (int)(i+1));
        else
          sprintf(tmp_buf, "Internal Energy %d", (int)(i+1));
        addChild(input_node, "SharedDataName", tmp_buf);
        if(m_InputData.mUseNormalizationInAggregator == "true")
        {
          input_node = op_node.append_child("Input");
          sprintf(tmp_buf, "Normalization Factor %d", (int)(i+1));
          addChild(input_node, "ArgumentName", tmp_buf);
          if(m_InputData.objectives[i].code_name == "plato_analyze")
            sprintf(tmp_buf, "Initial Objective Value %d", (int)(i+1));
          else
            sprintf(tmp_buf, "Initial Internal Energy %d", (int)(i+1));
          addChild(input_node, "SharedDataName", tmp_buf);
        }
      }
    }


    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Value");
    if(m_InputData.mPlatoAnalyzePerformerExists)
      addChild(output_node, "SharedDataName", "Objective Value");
    else
      addChild(output_node, "SharedDataName", "Internal Energy");

    // If there are uncertainties add an operation for
    // the objective mean and std deviation.
    if(aHasUncertainties)
    {
      op_node = stage_node.append_child("Operation");
      addChild(op_node, "Name", "Stochastic Objective Value");
      addChild(op_node, "PerformerName", "PlatoMain");

      if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
      {
        pugi::xml_node for_node = op_node.append_child("For");
        for_node.append_attribute("var") = "performerIndex";
        for_node.append_attribute("in") = "Performers";
        for_node = for_node.append_child("For");
        for_node.append_attribute("var") = "PerformerSampleIndex";
        for_node.append_attribute("in") = "PerformerSamples";
        input_node = for_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
        addChild(input_node, "SharedDataName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
      }
      else
      {
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
          XMLGen::Objective cur_obj = m_InputData.objectives[i];
          input_node = op_node.append_child("Input");
          std::string tTmpString = "";
          if(cur_obj.code_name == "plato_analyze")
            tTmpString += "Objective Value ";
          else
            tTmpString += "Internal Energy ";
          tTmpString += Plato::to_string(i+1);
          addChild(input_node, "ArgumentName", tTmpString);
          addChild(input_node, "SharedDataName", tTmpString);
        }
      }

      output_node = op_node.append_child("Output");
      std::string tFieldName = "Objective Mean Plus ";
      tFieldName += m_InputData.objective_number_standard_deviations;
      tFieldName += " StdDev";
      addChild(output_node, "ArgumentName", tFieldName);
      addChild(output_node, "SharedDataName", "Objective Mean Plus StdDev Value");
    }

    output_node = stage_node.append_child("Output");
    if(m_InputData.mPlatoAnalyzePerformerExists)
      addChild(output_node, "SharedDataName", "Objective Value");
    else
      addChild(output_node, "SharedDataName", "Internal Energy");

    return true;
}

/******************************************************************************/
void DefaultInputGenerator::addComputeObjectiveValueOperationForNewUncertaintyWorkflow(pugi::xml_node &aNode)
/******************************************************************************/
{
  pugi::xml_node for_node = aNode.append_child("For");
  for_node.append_attribute("var") = "PerformerSampleIndex";
  for_node.append_attribute("in") = "PerformerSamples";
  pugi::xml_node op_node = for_node.append_child("Operation");
  for_node = op_node.append_child("For");
  for_node.append_attribute("var") = "performerIndex";
  for_node.append_attribute("in") = "Performers";
  op_node = for_node.append_child("Operation");
  addChild(op_node, "PerformerName", "plato_analyze_{performerIndex}");
  addChild(op_node, "Name", "Compute Objective Value");

  for(size_t tRandomLoadIndex = 0; tRandomLoadIndex < m_InputData.m_UncertaintyMetaData.randomVariableIndices.size(); ++tRandomLoadIndex)
  {
    pugi::xml_node tmp_node1 = op_node.append_child("Parameter");
    std::string tLoadName = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "X";
    addChild(tmp_node1, "ArgumentName", tLoadName);
    std::string tValueName = "{" + tLoadName + "[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}";
    addChild(tmp_node1, "ArgumentValue", tValueName);

    tmp_node1 = op_node.append_child("Parameter");
    tLoadName = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "Y";
    addChild(tmp_node1, "ArgumentName", tLoadName);
    tValueName = "{" + tLoadName + "[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}";
    addChild(tmp_node1, "ArgumentValue", tValueName);

    tmp_node1 = op_node.append_child("Parameter");
    tLoadName = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "Z";
    addChild(tmp_node1, "ArgumentName", tLoadName);
    tValueName = "{" + tLoadName + "[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}";
    addChild(tmp_node1, "ArgumentValue", tValueName);
  }

  pugi::xml_node input_node = op_node.append_child("Input");
  addChild(input_node, "ArgumentName", "Topology");
  addChild(input_node, "SharedDataName", "Topology");

  pugi::xml_node output_node = op_node.append_child("Output");
  addChild(output_node, "ArgumentName", "Objective Value");
  addChild(output_node, "SharedDataName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
}

/******************************************************************************/
void DefaultInputGenerator::addComputeObjectiveGradientOperationForNewUncertaintyWorkflow(pugi::xml_node &aNode)
/******************************************************************************/
{
  pugi::xml_node for_node = aNode.append_child("For");
  for_node.append_attribute("var") = "PerformerSampleIndex";
  for_node.append_attribute("in") = "PerformerSamples";
  pugi::xml_node op_node = for_node.append_child("Operation");
  for_node = op_node.append_child("For");
  for_node.append_attribute("var") = "performerIndex";
  for_node.append_attribute("in") = "Performers";
  op_node = for_node.append_child("Operation");
  addChild(op_node, "PerformerName", "plato_analyze_{performerIndex}");
  addChild(op_node, "Name", "Compute Objective Gradient");

  for(size_t tRandomLoadIndex = 0; tRandomLoadIndex < m_InputData.m_UncertaintyMetaData.randomVariableIndices.size(); ++tRandomLoadIndex)
  {
    pugi::xml_node tmp_node1 = op_node.append_child("Parameter");
    std::string tLoadName = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "X";
    addChild(tmp_node1, "ArgumentName", tLoadName);
    std::string tValueName = "{" + tLoadName + "[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}";
    addChild(tmp_node1, "ArgumentValue", tValueName);

    tmp_node1 = op_node.append_child("Parameter");
    tLoadName = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "Y";
    addChild(tmp_node1, "ArgumentName", tLoadName);
    tValueName = "{" + tLoadName + "[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}";
    addChild(tmp_node1, "ArgumentValue", tValueName);

    tmp_node1 = op_node.append_child("Parameter");
    tLoadName = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "Z";
    addChild(tmp_node1, "ArgumentName", tLoadName);
    tValueName = "{" + tLoadName + "[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}";
    addChild(tmp_node1, "ArgumentValue", tValueName);
  }

  pugi::xml_node input_node = op_node.append_child("Input");
  addChild(input_node, "ArgumentName", "Topology");
  addChild(input_node, "SharedDataName", "Topology");

  pugi::xml_node output_node = op_node.append_child("Output");
  addChild(output_node, "ArgumentName", "Objective Gradient");
  addChild(output_node, "SharedDataName", "Objective {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex} Gradient");
}

/******************************************************************************/
bool DefaultInputGenerator::outputObjectiveStageForShape(pugi::xml_document &doc)
/******************************************************************************/
{
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Objective");

    pugi::xml_node input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Design Parameters");

    // Update Geometry on Change operation
    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Update Geometry on Change");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Parameters");
    addChild(input_node, "SharedDataName", "Design Parameters");

    pugi::xml_node cur_parent = stage_node;
    if(m_InputData.objectives.size() > 1)
    {
        op_node = stage_node.append_child("Operation");
        cur_parent = op_node;
    }

    // Reinitialize on Change operation
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        op_node = cur_parent.append_child("Operation");
        addChild(op_node, "Name", "Reinitialize on Change");
        addChild(op_node, "PerformerName", m_InputData.objectives[i].mPerformerName);
        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Parameters");
        addChild(input_node, "SharedDataName", "Design Parameters");
    }

    cur_parent = stage_node;
    if(m_InputData.objectives.size() > 1)
    {
        op_node = stage_node.append_child("Operation");
        cur_parent = op_node;
    }

    char tmp_buf[200];
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        XMLGen::Objective cur_obj = m_InputData.objectives[i];
        op_node = cur_parent.append_child("Operation");
        addChild(op_node, "Name", "Compute Objective Value");
        addChild(op_node, "PerformerName", cur_obj.mPerformerName.c_str());


        pugi::xml_node output_node = op_node.append_child("Output");
        addChild(output_node, "ArgumentName", "Objective Value");
        sprintf(tmp_buf, "Objective %d", (int)(i+1));
        addChild(output_node, "SharedDataName", tmp_buf);
    }

    // Aggregate
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "AggregateValues");
    addChild(op_node, "PerformerName", "PlatoMain");

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        input_node = op_node.append_child("Input");
        sprintf(tmp_buf, "Value %d", (int)(i+1));
        addChild(input_node, "ArgumentName", tmp_buf);
        sprintf(tmp_buf, "Objective %d", (int)(i+1));
        addChild(input_node, "SharedDataName", tmp_buf);
    }

    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Values");
    addChild(output_node, "SharedDataName", "Objective");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Objective");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::outputObjectiveGradientStageForShape(pugi::xml_document &doc)
/******************************************************************************/
{
    // Internal Energy
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Objective Gradient");

    pugi::xml_node input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Design Parameters");

    // Update Geometry on Change operation
    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Update Geometry on Change");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Parameters");
    addChild(input_node, "SharedDataName", "Design Parameters");

    pugi::xml_node cur_parent = stage_node;
    if(m_InputData.objectives.size() > 1)
    {
        op_node = stage_node.append_child("Operation");
        cur_parent = op_node;
    }

    // Reinitialize on Change operation
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        op_node = cur_parent.append_child("Operation");
        addChild(op_node, "Name", "Reinitialize on Change");
        addChild(op_node, "PerformerName", m_InputData.objectives[i].mPerformerName);
        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Parameters");
        addChild(input_node, "SharedDataName", "Design Parameters");
    }

    op_node = stage_node.append_child("Operation");

    pugi::xml_node op_node2;
    cur_parent = op_node;
    if(m_InputData.objectives.size() > 1)
    {
        op_node2 = op_node.append_child("Operation");
        cur_parent = op_node2;
    }

    char tmp_buf[200];
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        XMLGen::Objective cur_obj = m_InputData.objectives[i];
        op_node2 = cur_parent.append_child("Operation");
        addChild(op_node2, "Name", "Compute Objective Gradient");
        addChild(op_node2, "PerformerName", cur_obj.mPerformerName.c_str());
    }

    pugi::xml_node tForNode = op_node.append_child("For");
    tForNode.append_attribute("var") = "I";
    tForNode.append_attribute("in") = "Parameters";
    op_node2 = tForNode.append_child("Operation");
    addChild(op_node2, "Name", "Compute Parameter Sensitivity On Change");
    addChild(op_node2, "PerformerName", "PlatoESP_{I}");
    pugi::xml_node tmp_node = op_node2.append_child("Parameter");
    addChild(tmp_node, "ArgumentName", "Parameter Index");
    addChild(tmp_node, "ArgumentValue", "{I-1}");
    tmp_node = op_node2.append_child("Input");
    addChild(tmp_node, "ArgumentName", "Parameters");
    addChild(tmp_node, "SharedDataName", "Design Parameters");
    tmp_node = op_node2.append_child("Output");
    addChild(tmp_node, "ArgumentName", "Parameter Sensitivity");
    addChild(tmp_node, "SharedDataName", "Parameter Sensitivity {I}");

    cur_parent = stage_node;
    if(m_InputData.objectives.size() > 1)
    {
        op_node2 = stage_node.append_child("Operation");
        cur_parent = op_node2;
    }
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        op_node2 = cur_parent.append_child("Operation");
        addChild(op_node2, "Name", "Compute Objective Sensitivity");
        addChild(op_node2, "PerformerName", m_InputData.objectives[i].mPerformerName);
        tForNode = op_node2.append_child("For");
        tForNode.append_attribute("var") = "I";
        tForNode.append_attribute("in") = "Parameters";
        tmp_node = tForNode.append_child("Input");
        addChild(tmp_node, "ArgumentName", "Parameter Sensitivity {I}");
        addChild(tmp_node, "SharedDataName", "Parameter Sensitivity {I}");
        tmp_node = op_node2.append_child("Output");
        addChild(tmp_node, "ArgumentName", "Objective Sensitivity");
        sprintf(tmp_buf, "Objective %d Gradient", int(i+1));
        addChild(tmp_node, "SharedDataName", tmp_buf);
    }

    // Aggregate
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "AggregateValues");
    addChild(op_node, "PerformerName", "PlatoMain");

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        input_node = op_node.append_child("Input");
        sprintf(tmp_buf, "Value %d", (int)(i+1));
        addChild(input_node, "ArgumentName", tmp_buf);
        sprintf(tmp_buf, "Objective %d Gradient", (int)(i+1));
        addChild(input_node, "SharedDataName", tmp_buf);
    }

    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Values");
    addChild(output_node, "SharedDataName", "Objective Gradient");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Objective Gradient");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::outputObjectiveGradientStage(pugi::xml_document &doc,
                                                     const bool &aHasUncertainties)
/******************************************************************************/
{
    char tmp_buf[200];
    // Internal Energy
    pugi::xml_node stage_node = doc.append_child("Stage");
    pugi::xml_node cur_parent = stage_node;
    if(m_InputData.mPlatoAnalyzePerformerExists)
      addChild(stage_node, "Name", "Objective Gradient");
    else
      addChild(stage_node, "Name", "Internal Energy Gradient");

    // fitler control
    pugi::xml_node input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterControl");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Field");
    addChild(output_node, "SharedDataName", "Topology");

    // Enforce Bounds operation
    if(!m_InputData.mPlatoAnalyzePerformerExists)
        addEnforceBoundsOperationToStage(stage_node);

    
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      addComputeObjectiveGradientOperationForNewUncertaintyWorkflow(stage_node);
    }
    else
    {
      if(m_InputData.objectives.size() > 1)
      {
        op_node = stage_node.append_child("Operation");
        cur_parent = op_node;
      }

      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        XMLGen::Objective cur_obj = m_InputData.objectives[i];
        op_node = cur_parent.append_child("Operation");
        if(cur_obj.code_name == "plato_analyze")
        {
          addChild(op_node, "Name", "Compute Objective Gradient");
          addChild(op_node, "PerformerName", cur_obj.mPerformerName.c_str());
        }
        else
        {
          addChild(op_node, "Name", "Compute Gradient");
          addChild(op_node, "PerformerName", cur_obj.mPerformerName.c_str());
        }

        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Topology");
        addChild(input_node, "SharedDataName", "Topology");

        output_node = op_node.append_child("Output");
        if(cur_obj.code_name == "plato_analyze")
          addChild(output_node, "ArgumentName", "Objective Gradient");
        else
          addChild(output_node, "ArgumentName", "Internal Energy Gradient");
        if(cur_obj.code_name == "plato_analyze")
          sprintf(tmp_buf, "Objective %d Gradient", (int)(i+1));
        else
          sprintf(tmp_buf, "Internal Energy %d Gradient", (int)(i+1));
        addChild(output_node, "SharedDataName", tmp_buf);
      }
    }
    

    // Aggregate
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "AggregateGradient");
    addChild(op_node, "PerformerName", "PlatoMain");

    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = op_node.append_child("For");
      for_node.append_attribute("var") = "performerIndex";
      for_node.append_attribute("in") = "Performers";
      for_node = for_node.append_child("For");
      for_node.append_attribute("var") = "PerformerSampleIndex";
      for_node.append_attribute("in") = "PerformerSamples";

      pugi::xml_node input_node = for_node.append_child("Input");
      addChild(input_node, "ArgumentName", "Field {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
      addChild(input_node, "SharedDataName", "Objective {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex} Gradient");
      
      if(m_InputData.mUseNormalizationInAggregator == "true")
      {
          input_node = for_node.append_child("Input");
          sprintf(tmp_buf, "Normalization Factor {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
          addChild(input_node, "ArgumentName", "Normalization Factor {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
          addChild(input_node, "SharedDataName", "Initial Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
      }
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        input_node = op_node.append_child("Input");
        sprintf(tmp_buf, "Field %d", (int)(i+1));
        addChild(input_node, "ArgumentName", tmp_buf);
        if(m_InputData.objectives[i].code_name == "plato_analyze")
          sprintf(tmp_buf, "Objective %d Gradient", (int)(i+1));
        else
          sprintf(tmp_buf, "Internal Energy %d Gradient", (int)(i+1));
        addChild(input_node, "SharedDataName", tmp_buf);
        if(m_InputData.mUseNormalizationInAggregator == "true")
        {
          input_node = op_node.append_child("Input");
          sprintf(tmp_buf, "Normalization Factor %d", (int)(i+1));
          addChild(input_node, "ArgumentName", tmp_buf);
          if(m_InputData.objectives[i].code_name == "plato_analyze")
            sprintf(tmp_buf, "Initial Objective Value %d", (int)(i+1));
          else
            sprintf(tmp_buf, "Initial Internal Energy %d", (int)(i+1));
          addChild(input_node, "SharedDataName", tmp_buf);
        }
      }
    }

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Field");
    if(m_InputData.mPlatoAnalyzePerformerExists)
      addChild(output_node, "SharedDataName", "Objective Gradient");
    else
      addChild(output_node, "SharedDataName", "Internal Energy Gradient");

    // filter gradient
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterGradient");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Gradient");
    if(m_InputData.mPlatoAnalyzePerformerExists)
      addChild(input_node, "SharedDataName", "Objective Gradient");
    else
      addChild(input_node, "SharedDataName", "Internal Energy Gradient");
    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Gradient");
    if(m_InputData.mPlatoAnalyzePerformerExists)
      addChild(output_node, "SharedDataName", "Objective Gradient");
    else
      addChild(output_node, "SharedDataName", "Internal Energy Gradient");

    if(aHasUncertainties)
    {
        op_node = stage_node.append_child("Operation");
        addChild(op_node, "Name", "Stochastic Objective Gradient");
        addChild(op_node, "PerformerName", "PlatoMain");

        if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
        {
          pugi::xml_node for_node = op_node.append_child("For");
          for_node.append_attribute("var") = "performerIndex";
          for_node.append_attribute("in") = "Performers";
          for_node = for_node.append_child("For");
          for_node.append_attribute("var") = "PerformerSampleIndex";
          for_node.append_attribute("in") = "PerformerSamples";

          pugi::xml_node input_node = for_node.append_child("Input");
          addChild(input_node, "ArgumentName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
          addChild(input_node, "SharedDataName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
          input_node = for_node.append_child("Input");
          addChild(input_node, "ArgumentName", "Objective {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex} Gradient");
          addChild(input_node, "SharedDataName", "Objective {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex} Gradient");
        }
        else
        {
          for(size_t i=0; i<m_InputData.objectives.size(); ++i)
          {
            XMLGen::Objective cur_obj = m_InputData.objectives[i];
            input_node = op_node.append_child("Input");
            std::string tTmpString = "";
            if(cur_obj.code_name == "plato_analyze")
              tTmpString += "Objective Value ";
            else
              tTmpString += "Internal Energy ";
            tTmpString += Plato::to_string(i+1);
            addChild(input_node, "ArgumentName", tTmpString);
            addChild(input_node, "SharedDataName", tTmpString);

            input_node = op_node.append_child("Input");
            tTmpString = "";
            if(cur_obj.code_name == "plato_analyze")
              tTmpString += "Objective ";
            else
              tTmpString += "Internal Energy ";
            tTmpString += Plato::to_string(i+1);
            tTmpString += " Gradient";
            addChild(input_node, "ArgumentName", tTmpString);
            addChild(input_node, "SharedDataName", tTmpString);
          }
        }

        output_node = op_node.append_child("Output");
        std::string tTmpString = "Objective Mean Plus ";
        tTmpString += m_InputData.objective_number_standard_deviations;
        tTmpString += " StdDev Gradient";
        addChild(output_node, "ArgumentName", tTmpString);
        addChild(output_node, "SharedDataName", "Objective Mean Plus StdDev Gradient");
    }

    // stage output
    output_node = stage_node.append_child("Output");
    if(m_InputData.mPlatoAnalyzePerformerExists)
      addChild(output_node, "SharedDataName", "Objective Gradient");
    else
      addChild(output_node, "SharedDataName", "Internal Energy Gradient");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::outputObjectiveHessianStage(pugi::xml_document &doc)
/******************************************************************************/
{
    char tmp_buf[200];
    // Internal Energy
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Internal Energy Hessian");
    
    pugi::xml_node input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Descent Direction");
    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterControl");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Field");
    addChild(output_node, "SharedDataName", "Topology");
    
    // Enforce Bounds operation
    if(!m_InputData.mPlatoAnalyzePerformerExists)
        addEnforceBoundsOperationToStage(stage_node);

    pugi::xml_node cur_parent = stage_node;
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
        pugi::xml_node for_node = stage_node.append_child("For");
        for_node.append_attribute("var") = "performerIndex";
        for_node.append_attribute("in") = "Performers";
        for_node = for_node.append_child("For");
        for_node.append_attribute("var") = "PerformerSampleIndex";
        for_node.append_attribute("in") = "PerformerSamples";

        op_node = for_node.append_child("Operation");
        addChild(op_node, "Name", "Compute HessianTimesVector");
        addChild(op_node, "PerformerName", "plato_analyze_{performerIndex}");

        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Topology");
        addChild(input_node, "SharedDataName", "Topology");
        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Descent Direction");
        addChild(input_node, "SharedDataName", "Descent Direction");

        output_node = op_node.append_child("Output");
        addChild(output_node, "ArgumentName", "HessianTimesVector");
        addChild(output_node, "SharedDataName", "HessianTimesVector {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    }
    else
    {
      if(m_InputData.objectives.size() > 1)
      {
        op_node = stage_node.append_child("Operation");
        cur_parent = op_node;
      }

      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        op_node = cur_parent.append_child("Operation");
        addChild(op_node, "Name", "Compute HessianTimesVector");
        addChild(op_node, "PerformerName", m_InputData.objectives[i].mPerformerName.c_str());

        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Topology");
        addChild(input_node, "SharedDataName", "Topology");
        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Descent Direction");
        addChild(input_node, "SharedDataName", "Descent Direction");

        output_node = op_node.append_child("Output");
        addChild(output_node, "ArgumentName", "HessianTimesVector");
        sprintf(tmp_buf, "HessianTimesVector %d", (int)(i+1));
        addChild(output_node, "SharedDataName", tmp_buf);
      }

    }
    
    // Aggregate
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "AggregateHessian");
    addChild(op_node, "PerformerName", "PlatoMain");
    
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
        pugi::xml_node for_node = op_node.append_child("For");
        for_node.append_attribute("var") = "performerIndex";
        for_node.append_attribute("in") = "Performers";
        for_node = for_node.append_child("For");
        for_node.append_attribute("var") = "PerformerSampleIndex";
        for_node.append_attribute("in") = "PerformerSamples";

        input_node = for_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Field {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
        addChild(input_node, "SharedDataName", "HessianTimesVector {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        input_node = op_node.append_child("Input");
        sprintf(tmp_buf, "Field %d", (int)(i+1));
        addChild(input_node, "ArgumentName", tmp_buf);
        sprintf(tmp_buf, "HessianTimesVector %d", (int)(i+1));
        addChild(input_node, "SharedDataName", tmp_buf);
      }
    }
    
    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Field");
    addChild(output_node, "SharedDataName", "Internal Energy Hessian");
    
    // filter Hessian
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterHessian");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Internal Energy Hessian");
    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Field");
    addChild(output_node, "SharedDataName", "Internal Energy Hessian");
    
    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Internal Energy Hessian");
    
    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::outputObjectiveHessianStageForShape(pugi::xml_document &doc)
/******************************************************************************/
{
    char tmp_buf[200];
    // Internal Energy
    pugi::xml_node op_node, output_node;
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Objective Hessian");

    pugi::xml_node input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Design Parameters");
    input_node = stage_node.append_child("Input");
    addChild(input_node, "SharedDataName", "Descent Direction");

    pugi::xml_node cur_parent = stage_node;
    if(m_InputData.objectives.size() > 1)
    {
        op_node = stage_node.append_child("Operation");
        cur_parent = op_node;
    }

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        op_node = cur_parent.append_child("Operation");
        addChild(op_node, "Name", "Compute HessianTimesVector");
        addChild(op_node, "PerformerName", m_InputData.objectives[i].mPerformerName.c_str());

        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Design Parameters");
        addChild(input_node, "SharedDataName", "Design Parameters");
        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Descent Direction");
        addChild(input_node, "SharedDataName", "Descent Direction");

        output_node = op_node.append_child("Output");
        addChild(output_node, "ArgumentName", "HessianTimesVector");
        sprintf(tmp_buf, "HessianTimesVector %d", (int)(i+1));
        addChild(output_node, "SharedDataName", tmp_buf);
    }

    // Aggregate
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "AggregateHessian");
    addChild(op_node, "PerformerName", "PlatoMain");

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        input_node = op_node.append_child("Input");
        sprintf(tmp_buf, "Value %d", (int)(i+1));
        addChild(input_node, "ArgumentName", tmp_buf);
        sprintf(tmp_buf, "HessianTimesVector %d", (int)(i+1));
        addChild(input_node, "SharedDataName", tmp_buf);
    }

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Value");
    addChild(output_node, "SharedDataName", "Objective Hessian");


    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Objective Hessian");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generatePlatoAnalyzeShapeDefinesXML()
/******************************************************************************/
{
    pugi::xml_document doc;

    pugi::xml_node tTmpNode = doc.append_child("Define");
    tTmpNode.append_attribute("name") = "NumParameters";
    std::string tString = Plato::to_string(m_InputData.num_shape_design_variables);
    tTmpNode.append_attribute("value") = tString.c_str();

    tTmpNode = doc.append_child("Array");
    tTmpNode.append_attribute("name") = "Parameters";
    tTmpNode.append_attribute("type") = "int";
    tTmpNode.append_attribute("from") = "1";
    tTmpNode.append_attribute("to") = "{NumParameters}";

    doc.save_file("defines.xml", "  ");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generateDefinesXML(std::ostringstream *aStringStream)
/******************************************************************************/
{
  if(m_InputData.mPlatoAnalyzePerformerExists && m_InputData.uncertainties.size() > 0)
  {
    pugi::xml_document doc;

    if(!addDefinesToDoc(doc))
      return false;
    
    if(aStringStream)
    {
      std::cout << "stringstream" << std::endl;
      doc.save(*aStringStream, "\t", pugi::format_default & ~pugi::format_indent);
    }
    else
      doc.save_file("defines.xml", "  ");
  }

  return true;
}

/******************************************************************************/
void DefaultInputGenerator::addVersionEntryToDoc(pugi::xml_document& doc)
/******************************************************************************/
{
    // Version entry
    pugi::xml_node tTmpNode = doc.append_child(pugi::node_declaration);
    tTmpNode.set_name("xml");
    pugi::xml_attribute tmp_att = tTmpNode.append_attribute("version");
    tmp_att.set_value("1.0");
}

/******************************************************************************/
bool DefaultInputGenerator::addDefinesToDoc(pugi::xml_document& doc)
/******************************************************************************/
{
  pugi::xml_node tTmpNode = doc.append_child("Define");
  tTmpNode.append_attribute("name") = "NumSamples";
  tTmpNode.append_attribute("type") = "int";
  std::string tNumSamplesString = Plato::to_string(m_InputData.mRandomMetaData.numSamples());
  tTmpNode.append_attribute("value") = tNumSamplesString.c_str();

  tTmpNode = doc.append_child("Define");
  tTmpNode.append_attribute("name") = "NumPerformers";
  tTmpNode.append_attribute("type") = "int";
  size_t tNumPerformers = m_InputData.m_UncertaintyMetaData.numPerformers;

  if(tNumPerformers == 0)
  {
    std::cout << "ERROR: Cannot assign zero MPI processes" << std::endl;
    return false;
  }

  size_t tNumSamples = stringToSizeT(tNumSamplesString);
  tNumPerformers = XMLGen::compute_greatest_divisor(tNumSamples,tNumPerformers);

  std::string tNumPerformersString = Plato::to_string(tNumPerformers);
  tTmpNode.append_attribute("value") = tNumPerformersString.c_str();

  tTmpNode = doc.append_child("Define");
  tTmpNode.append_attribute("name") = "NumSamplesPerPerformer";
  tTmpNode.append_attribute("type") = "int";
  tTmpNode.append_attribute("value") = "{NumSamples/NumPerformers}";

  tTmpNode = doc.append_child("Array");
  tTmpNode.append_attribute("name") = "Samples";
  tTmpNode.append_attribute("type") = "int";
  tTmpNode.append_attribute("from") = "0";
  tTmpNode.append_attribute("to") = "{NumSamples-1}";

  tTmpNode = doc.append_child("Array");
  tTmpNode.append_attribute("name") = "Performers";
  tTmpNode.append_attribute("type") = "int";
  tTmpNode.append_attribute("from") = "0";
  tTmpNode.append_attribute("to") = "{NumPerformers-1}";

  tTmpNode = doc.append_child("Array");
  tTmpNode.append_attribute("name") = "PerformerSamples";
  tTmpNode.append_attribute("type") = "int";
  tTmpNode.append_attribute("from") = "0";
  tTmpNode.append_attribute("to") = "{NumSamplesPerPerformer-1}";

  std::vector<XMLGen::LoadCase> tLoadCases = m_InputData.load_cases;

  std::vector<std::string> tProbabilitiesForLoadCaseIndex;

  for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tLoadCases.size(); ++tLoadCaseIndex)
  {
    XMLGen::LoadCase load_case = tLoadCases[tLoadCaseIndex];
    char buf[200];
    sprintf(buf, "%.17e", m_InputData.load_case_probabilities[tLoadCaseIndex]);
    tProbabilitiesForLoadCaseIndex.push_back(buf);
  }

  std::string tProbabilitiesString = makeValuesString(tProbabilitiesForLoadCaseIndex);

  for(size_t tRandomLoadIndex = 0; tRandomLoadIndex < m_InputData.m_UncertaintyMetaData.randomVariableIndices.size(); ++tRandomLoadIndex)
  {
    std::vector<std::string> tXValuesForRandomLoadIndex;
    std::vector<std::string> tYValuesForRandomLoadIndex;
    std::vector<std::string> tZValuesForRandomLoadIndex;

    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tLoadCases.size(); ++tLoadCaseIndex)
    {
      XMLGen::LoadCase load_case = tLoadCases[tLoadCaseIndex];
      XMLGen::Load tRandomLoad = load_case.loads[tRandomLoadIndex];
      tXValuesForRandomLoadIndex.push_back(tRandomLoad.values[0]);
      tYValuesForRandomLoadIndex.push_back(tRandomLoad.values[1]);
      tZValuesForRandomLoadIndex.push_back(tRandomLoad.values[2]);
      tProbabilitiesForLoadCaseIndex.push_back(Plato::to_string(m_InputData.load_case_probabilities[tLoadCaseIndex]));
    }

    std::string tXValuesString = makeValuesString(tXValuesForRandomLoadIndex);
    std::string tYValuesString = makeValuesString(tYValuesForRandomLoadIndex);
    std::string tZValuesString = makeValuesString(tZValuesForRandomLoadIndex);

    tTmpNode = doc.append_child("Array");
    std::string tRandomLoadString = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "X";
    tTmpNode.append_attribute("name") = tRandomLoadString.c_str();
    tTmpNode.append_attribute("type") = "real";
    tTmpNode.append_attribute("values") = tXValuesString.c_str();

    tTmpNode = doc.append_child("Array");
    tRandomLoadString = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "Y";
    tTmpNode.append_attribute("name") = tRandomLoadString.c_str();
    tTmpNode.append_attribute("type") = "real";
    tTmpNode.append_attribute("values") = tYValuesString.c_str();

    tTmpNode = doc.append_child("Array");
    tRandomLoadString = "RandomLoad" + Plato::to_string(tRandomLoadIndex) + "Z";
    tTmpNode.append_attribute("name") = tRandomLoadString.c_str();
    tTmpNode.append_attribute("type") = "real";
    tTmpNode.append_attribute("values") = tZValuesString.c_str();

  }

  tTmpNode = doc.append_child("Array");
  tTmpNode.append_attribute("name") = "Probabilities";
  tTmpNode.append_attribute("type") = "real";
  tTmpNode.append_attribute("values") = tProbabilitiesString.c_str();

  return true;
}

/******************************************************************************/
std::string DefaultInputGenerator::makeValuesString(const std::vector<std::string>& aValues)
/******************************************************************************/
{
  std::string tValuesString;
  for(size_t i = 0; i < aValues.size(); ++i)
  {
    tValuesString += aValues[i];
    if(i < aValues.size()-1)
      tValuesString += ", ";
  }
  return tValuesString;
}

/******************************************************************************/
size_t DefaultInputGenerator::stringToSizeT(const std::string& aString)
/******************************************************************************/
{
  std::stringstream sstream(aString);
  size_t result;
  sstream >> result;
  return result;
}

/******************************************************************************/
bool DefaultInputGenerator::generateInterfaceXML(std::ostringstream *aStringStream)
/******************************************************************************/
{
    pugi::xml_document doc;

    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node def_node = doc.append_child("include");
      def_node.append_attribute("filename") = "defines.xml";
    }

    // // Version entry
    pugi::xml_node tTmpNode;
    // pugi::xml_node tTmpNode = doc.append_child(pugi::node_declaration);
    // tTmpNode.set_name("xml");
    // pugi::xml_attribute tmp_att = tTmpNode.append_attribute("version");
    // tmp_att.set_value("1.0");
    

    // Console output control
    tTmpNode = doc.append_child("Console");
    addChild(tTmpNode, "Verbose", "true");

    //////////////////////////////////////////////////
    // Performers
    /////////////////////////////////////////////////

    // PlatoMain performer entry.
    pugi::xml_node performer_node = doc.append_child("Performer");
    addChild(performer_node, "Name", "PlatoMain");
    addChild(performer_node, "Code", "PlatoMain");
    addChild(performer_node, "PerformerID", "0");

    pugi::xml_node sd_node;
    char tmp_buf[200];

    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
        performer_node = doc.append_child("Performer");
        pugi::xml_node for_node = performer_node.append_child("For");
        for_node.append_attribute("var") = "performerIndex";
        for_node.append_attribute("in") = "Performers";
        addChild(for_node, "Name", "plato_analyze_{performerIndex}");
        addChild(for_node, "Code", "plato_analyze");
        addChild(performer_node, "PerformerID", "1");
    }
    else
    {
      // Other performers (one for each objective)
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        performer_node = doc.append_child("Performer");
        addChild(performer_node, "Name", m_InputData.objectives[i].mPerformerName);
        addChild(performer_node, "Code", m_InputData.objectives[i].code_name);
        sprintf(tmp_buf, "%d", (int)(i+1));
        addChild(performer_node, "PerformerID", tmp_buf);
      }
    }


    //////////////////////////////////////////////////
    // Shared Data
    /////////////////////////////////////////////////

    // Internal Energy XXX shared data
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = doc.append_child("For");
      for_node.append_attribute("var") = "performerIndex";
      for_node.append_attribute("in") = "Performers";
      for_node = for_node.append_child("For");
      for_node.append_attribute("var") = "PerformerSampleIndex";
      for_node.append_attribute("in") = "PerformerSamples";
      std::string tSharedDataName = "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
      createSingleUserGlobalSharedData(for_node, tSharedDataName.c_str(), "Scalar", "1", "plato_analyze_{performerIndex}", "PlatoMain");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        if(m_InputData.optimization_type == "topology")
        {
          // create shared data for objectives
          if(m_InputData.objectives[i].code_name == "plato_analyze")
            sprintf(tmp_buf, "Objective Value %d", (int)(i+1));
          else
            sprintf(tmp_buf, "Internal Energy %d", (int)(i+1));
          createSingleUserGlobalSharedData(doc, tmp_buf, "Scalar", "1", m_InputData.objectives[i].mPerformerName, "PlatoMain");
        }
        else if(m_InputData.optimization_type == "shape")
        {
          // create shared data for objectives
          sprintf(tmp_buf, "Objective %d", (int)(i+1));
          createSingleUserGlobalSharedData(doc, tmp_buf, "Scalar", "1", m_InputData.objectives[i].mPerformerName, "PlatoMain");
        }

        if(m_InputData.mUseNormalizationInAggregator == "true")
        {
          if(m_InputData.optimization_type == "topology")
          {
            if(m_InputData.objectives[i].code_name == "plato_analyze")
              sprintf(tmp_buf, "Initial Objective Value %d", (int)(i+1));
            else
              sprintf(tmp_buf, "Initial Internal Energy %d", (int)(i+1));
            createSingleUserGlobalSharedData(doc, tmp_buf, "Scalar", "1", m_InputData.objectives[i].mPerformerName, "PlatoMain");
          }
        }
      }
    }
    // Internal Energy XXX Gradient shared data
    if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
    {
      pugi::xml_node for_node = doc.append_child("For");
      for_node.append_attribute("var") = "performerIndex";
      for_node.append_attribute("in") = "Performers";
      for_node = for_node.append_child("For");
      for_node.append_attribute("var") = "PerformerSampleIndex";
      for_node.append_attribute("in") = "PerformerSamples";
      std::string tSharedDataName = "Objective {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex} Gradient";
      createSingleUserNodalSharedData(for_node, tSharedDataName.c_str(), "Scalar", "plato_analyze_{performerIndex}", "PlatoMain");
    }
    else
    {
      for(size_t i=0; i<m_InputData.objectives.size(); ++i)
      {
        if(m_InputData.optimization_type == "topology")
        {
          if(m_InputData.objectives[i].code_name == "plato_analyze")
            sprintf(tmp_buf, "Objective %d Gradient", (int)(i+1));
          else
            sprintf(tmp_buf, "Internal Energy %d Gradient", (int)(i+1));
          createSingleUserNodalSharedData(doc, tmp_buf, "Scalar", m_InputData.objectives[i].mPerformerName, "PlatoMain");
        }
        else if(m_InputData.optimization_type == "shape" && m_InputData.num_shape_design_variables > 0)
        {
          char tTempBuffer[100];
          sprintf(tTempBuffer, "%d", m_InputData.num_shape_design_variables);
          sprintf(tmp_buf, "Objective %d Gradient", (int)(i+1));
          createSingleUserGlobalSharedData(doc, tmp_buf, "Scalar", tTempBuffer, m_InputData.objectives[i].mPerformerName, "PlatoMain");
        }
      }
    }

    if(m_InputData.optimization_type == "topology")
    {
      if(m_InputData.m_HasUncertainties)
      {
        // Objective statistics
        createSingleUserGlobalSharedData(doc, "Objective Mean Plus StdDev Value", "Scalar", "1", "PlatoMain", "PlatoMain");
        createSingleUserNodalSharedData(doc, "Objective Mean Plus StdDev Gradient", "Scalar", "PlatoMain", "PlatoMain");

        if(m_InputData.m_RequestedVonMisesOutput)
        {
          // VonMises statistics
          createSingleUserElementSharedData(doc, "VonMises Mean", "Scalar", "PlatoMain", "PlatoMain");
          createSingleUserElementSharedData(doc, "VonMises StdDev", "Scalar", "PlatoMain", "PlatoMain");
          for(size_t i=0; i<m_InputData.mStandardDeviations.size(); ++i)
          {
            std::string tFieldName = "VonMises Mean Plus ";
            tFieldName += m_InputData.mStandardDeviations[i];
            tFieldName += " StdDev";
            createSingleUserElementSharedData(doc, tFieldName, "Scalar", "PlatoMain", "PlatoMain");
          }
        }
      }


      // QOI shared data
      if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
      {
        pugi::xml_node for_node = doc.append_child("For");
        for_node.append_attribute("var") = "performerIndex";
        for_node.append_attribute("in") = "Performers";
        for_node = for_node.append_child("For");
        for_node.append_attribute("var") = "PerformerSampleIndex";
        for_node.append_attribute("in") = "PerformerSamples";
        createSingleUserElementSharedData(for_node, "plato_analyze_{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}_vonmises", "Scalar", "plato_analyze_{performerIndex}", "PlatoMain");
      }
      else
      {
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
          XMLGen::Objective cur_obj = m_InputData.objectives[i];
          // If this is a UQ run we are assuming only one load case per objective
          // and we are only supporting vonmises stress right now.
          if(m_InputData.m_HasUncertainties)
          {
            // create shared data for vonmises
            sprintf(tmp_buf, "%s_%s", cur_obj.mPerformerName.c_str(), "vonmises");
            createSingleUserElementSharedData(doc, tmp_buf, "Scalar", cur_obj.mPerformerName, "PlatoMain");
          }
          else if(cur_obj.multi_load_case == "true")
          {
            for(size_t k=0; k<cur_obj.load_case_ids.size(); ++k)
            {
              std::string cur_load_string = cur_obj.load_case_ids[k];
              for(size_t j=0; j<cur_obj.output_for_plotting.size(); ++j)
              {
                // create shared data for objectives
                sprintf(tmp_buf, "%s_load%s_%s", cur_obj.mPerformerName.c_str(), cur_load_string.c_str(), cur_obj.output_for_plotting[j].c_str());
                if(cur_obj.output_for_plotting[j] == "vonmises")
                  createSingleUserElementSharedData(doc, tmp_buf, "Scalar", cur_obj.mPerformerName, "PlatoMain");
                else
                  createSingleUserNodalSharedData(doc, tmp_buf, "Scalar", cur_obj.mPerformerName, "PlatoMain");
              }
            }
          }
          else
          {
            for(size_t j=0; j<cur_obj.output_for_plotting.size(); ++j)
            {
              // create shared data for objectives
              sprintf(tmp_buf, "%s_%s", cur_obj.mPerformerName.c_str(), cur_obj.output_for_plotting[j].c_str());
              if(cur_obj.output_for_plotting[j] == "vonmises")
                createSingleUserElementSharedData(doc, tmp_buf, "Scalar", cur_obj.mPerformerName, "PlatoMain");
              else
                createSingleUserNodalSharedData(doc, tmp_buf, "Scalar", cur_obj.mPerformerName, "PlatoMain");
            }
          }
        }
      }
    }

    // Hessian shared data
    if(m_InputData.optimization_algorithm =="ksbc" ||
            m_InputData.optimization_algorithm == "ksal" ||
            m_InputData.optimization_algorithm == "rol ksal" ||
            m_InputData.optimization_algorithm == "rol ksbc")
    {
      if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
      {
        pugi::xml_node for_node = doc.append_child("For");
        for_node.append_attribute("var") = "performerIndex";
        for_node.append_attribute("in") = "Performers";
        for_node = for_node.append_child("For");
        for_node.append_attribute("var") = "PerformerSampleIndex";
        for_node.append_attribute("in") = "PerformerSamples";
        std::string tSharedDataName = "HessianTimesVector {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
        createSingleUserNodalSharedData(doc, tSharedDataName.c_str(), "Scalar", "plato_analyze_{performerIndex}", "PlatoMain");
      }
      else
      {
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
          sprintf(tmp_buf, "HessianTimesVector %d", (int)(i+1));
          createSingleUserNodalSharedData(doc, tmp_buf, "Scalar", m_InputData.objectives[i].mPerformerName, "PlatoMain");
        }
      }
    }

    if(m_InputData.optimization_type == "topology")
    {
        // Internal Energy
        if(m_InputData.mPlatoAnalyzePerformerExists)
          createSingleUserGlobalSharedData(doc, "Objective Value", "Scalar", "1", "PlatoMain", "PlatoMain");
        else
          createSingleUserGlobalSharedData(doc, "Internal Energy", "Scalar", "1", "PlatoMain", "PlatoMain");

        // Internal Energy Gradient
        if(m_InputData.mPlatoAnalyzePerformerExists)
          createSingleUserNodalSharedData(doc, "Objective Gradient", "Scalar", "PlatoMain", "PlatoMain");
        else
          createSingleUserNodalSharedData(doc, "Internal Energy Gradient", "Scalar", "PlatoMain", "PlatoMain");
    }
    else if(m_InputData.optimization_type == "shape" && m_InputData.num_shape_design_variables > 0)
    {
        createSingleUserGlobalSharedData(doc, "Objective", "Scalar", "1", "PlatoMain", "PlatoMain");

        char tTempBuffer[100];
        sprintf(tTempBuffer, "%d", m_InputData.num_shape_design_variables);
        createSingleUserGlobalSharedData(doc, "Objective Gradient", "Scalar", tTempBuffer, "PlatoMain", "PlatoMain");
    }


    // Internal Energy Hessian and Descent Direction
    if(m_InputData.optimization_algorithm =="ksbc" ||
       m_InputData.optimization_algorithm == "ksal" ||
       m_InputData.optimization_algorithm == "rol ksal" ||
       m_InputData.optimization_algorithm == "rol ksbc")
    {
      if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
      {
        createSingleUserNodalSharedData(doc, "Internal Energy Hessian", "Scalar", "PlatoMain", "PlatoMain");
        sd_node = createSingleUserNodalSharedData(doc, "Descent Direction", "Scalar", "PlatoMain", "PlatoMain");
        pugi::xml_node for_node = sd_node.append_child("For");
        for_node.append_attribute("var") = "performerIndex";
        for_node.append_attribute("in") = "Performers";
        addChild(for_node, "UserName", "plato_analyze_{performerIndex}");
      }
      else
      {
        if(m_InputData.optimization_type == "topology")
        {
          createSingleUserNodalSharedData(doc, "Internal Energy Hessian", "Scalar", "PlatoMain", "PlatoMain");
          sd_node = createSingleUserNodalSharedData(doc, "Descent Direction", "Scalar", "PlatoMain", "PlatoMain");
          for(size_t i=0; i<m_InputData.objectives.size(); ++i)
            addChild(sd_node, "UserName", m_InputData.objectives[i].mPerformerName);
        }
        else if(m_InputData.optimization_type == "shape")
        {
          createSingleUserNodalSharedData(doc, "Objective Hessian", "Scalar", "PlatoMain", "PlatoMain");
          sd_node = createSingleUserNodalSharedData(doc, "Descent Direction", "Scalar", "PlatoMain", "PlatoMain");
          for(size_t i=0; i<m_InputData.objectives.size(); ++i)
            addChild(sd_node, "UserName", m_InputData.objectives[i].mPerformerName);
        }
      }
    }
    
    if(m_InputData.optimization_type == "shape" && m_InputData.num_shape_design_variables > 0)
    {
        char tTempBuffer[100];
        sprintf(tTempBuffer, "%d", m_InputData.num_shape_design_variables);
        std::vector<std::string> tUserNames;
        tUserNames.push_back("PlatoMain");
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
            tUserNames.push_back(m_InputData.objectives[i].mPerformerName);
        createMultiUserGlobalSharedData(doc, "Design Parameters", "Scalar", tTempBuffer, "PlatoMain", tUserNames);
    }
    else if(m_InputData.optimization_type == "topology")
    {
      if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
      {
        sd_node = createSingleUserNodalSharedData(doc, "Optimization DOFs", "Scalar", "PlatoMain", "PlatoMain");
        pugi::xml_node for_node = sd_node.append_child("For");
        for_node.append_attribute("var") = "performerIndex";
        for_node.append_attribute("in") = "Performers";
        addChild(for_node, "UserName", "plato_analyze_{performerIndex}");

        sd_node = createSingleUserNodalSharedData(doc, "Topology", "Scalar", "PlatoMain", "PlatoMain");
        for_node = sd_node.append_child("For");
        for_node.append_attribute("var") = "performerIndex";
        for_node.append_attribute("in") = "Performers";
        addChild(for_node, "UserName", "plato_analyze_{performerIndex}");
      }
      else
      {
        // Optimization DOFs
        sd_node = createSingleUserNodalSharedData(doc, "Optimization DOFs", "Scalar", "PlatoMain", "PlatoMain");
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
          addChild(sd_node, "UserName", m_InputData.objectives[i].mPerformerName);

        // Topology
        sd_node = createSingleUserNodalSharedData(doc, "Topology", "Scalar", "PlatoMain", "PlatoMain");
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
          addChild(sd_node, "UserName", m_InputData.objectives[i].mPerformerName);
      }
    }

    if(m_InputData.optimization_type == "topology")
    {
        if(m_InputData.discretization == "density")
        {
            // Volume
            createSingleUserGlobalSharedData(doc, "Volume", "Scalar", "1", "PlatoMain", "PlatoMain");

            // Volume Gradient
            createSingleUserNodalSharedData(doc, "Volume Gradient", "Scalar", "PlatoMain", "PlatoMain");

            // Design Volume
            createSingleUserGlobalSharedData(doc, "Design Volume", "Scalar", "1", "PlatoMain", "PlatoMain");
        }
        else if(m_InputData.discretization == "levelset")
        {
            // For now we will assume there is only one type of objective
            // if doing levelsets and we will just grab the first performer
            // as the one that will calculate the surface area.
            // Surface Area
            createSingleUserGlobalSharedData(doc, "Surface Area", "Scalar", "1", m_InputData.objectives[0].mPerformerName, "PlatoMain");

            // Surface Area Gradient
            createSingleUserNodalSharedData(doc, "Surface Area Gradient", "Scalar", m_InputData.objectives[0].mPerformerName, "PlatoMain");
        }
    }
    else if(m_InputData.optimization_type == "shape")
    {
        createSingleUserGlobalSharedData(doc, "Constraint", "Scalar", "1", m_InputData.objectives[0].mPerformerName, "PlatoMain");
        char tTempBuffer[100];
        sprintf(tTempBuffer, "%d", m_InputData.num_shape_design_variables);
        createSingleUserGlobalSharedData(doc, "Constraint Gradient", "Scalar", tTempBuffer, m_InputData.objectives[0].mPerformerName, "PlatoMain");
    }

    // Lower Bound Value
    createSingleUserGlobalSharedData(doc, "Lower Bound Value", "Scalar", "1", "PlatoMain", "PlatoMain");

    // Upper Bound Value
    createSingleUserGlobalSharedData(doc, "Upper Bound Value", "Scalar", "1", "PlatoMain", "PlatoMain");

    if(m_InputData.optimization_type == "topology")
    {
        // Lower Bound Vector
        createSingleUserNodalSharedData(doc, "Lower Bound Vector", "Scalar", "PlatoMain", "PlatoMain");

        // Upper Bound Vector
        createSingleUserNodalSharedData(doc, "Upper Bound Vector", "Scalar", "PlatoMain", "PlatoMain");
    }
    else if(m_InputData.optimization_type == "shape" && m_InputData.num_shape_design_variables > 0)
    {
        char tTempBuffer[100];
        sprintf(tTempBuffer, "%d", m_InputData.num_shape_design_variables);
        createSingleUserGlobalSharedData(doc, "Lower Bound Vector", "Scalar", tTempBuffer, "PlatoMain", "PlatoMain");
        createSingleUserGlobalSharedData(doc, "Upper Bound Vector", "Scalar", tTempBuffer, "PlatoMain", "PlatoMain");
    }

    
    //////////////////////////////////////////////////
    // Stages
    /////////////////////////////////////////////////

    // Output To File
    outputOutputToFileStage(doc, m_InputData.m_HasUncertainties, m_InputData.m_RequestedVonMisesOutput);

    // Initialize Optimization
    outputInitializeOptimizationStage(doc);

    if(m_InputData.optimization_type == "topology")
    {
        // Update Problem
        outputUpdateProblemStage(doc);
    }

    // Cache State Stage
    outputCacheStateStage(doc, m_InputData.m_HasUncertainties);

    // Set Lower Bounds Stage
    outputSetLowerBoundsStage(doc);

    // Set Upper Bounds
    outputSetUpperBoundsStage(doc);

    if(m_InputData.optimization_type == "topology")
    {
        if(m_InputData.discretization == "density")
        {
            // Design Volume
            outputDesignVolumeStage(doc);

            // Volume
            outputVolumeStage(doc);

            // Volume Gradient
            outputVolumeGradientStage(doc);
        }
        else if(m_InputData.discretization == "levelset")
        {
            // Surface Area
            outputSurfaceAreaStage(doc);

            // Surface Area Gradient
            outputSurfaceAreaGradientStage(doc);
        }
    }
    else if(m_InputData.optimization_type == "shape")
    {
        outputConstraintStage(doc);
        outputConstraintGradientStage(doc);
    }

    // Compute State
//    outputComputeStateStage(doc);

    if(m_InputData.optimization_type == "topology")
    {
        // Internal Energy
        outputObjectiveStage(doc, m_InputData.m_HasUncertainties);

        // Internal Energy Gradient
        outputObjectiveGradientStage(doc, m_InputData.m_HasUncertainties);
    }
    else if(m_InputData.optimization_type == "shape")
    {
        // Internal Energy
        outputObjectiveStageForShape(doc);

        // Internal Energy Gradient
        outputObjectiveGradientStageForShape(doc);
    }

    // Internal Energy Hessian
    if(m_InputData.optimization_algorithm =="ksbc" ||
       m_InputData.optimization_algorithm == "ksal" ||
       m_InputData.optimization_algorithm == "rol ksal" ||
       m_InputData.optimization_algorithm == "rol ksbc")
    {
        if(m_InputData.optimization_type == "topology")
        {
            outputObjectiveHessianStage(doc);
        }
        else if(m_InputData.optimization_type == "shape")
        {
            outputObjectiveHessianStageForShape(doc);
        }
    }

    /////////////////////////////////////////////////
    // Misc.
    ////////////////////////////////////////////////

    pugi::xml_node tMiscNode = doc.append_child("Optimizer");
    this->setOptimizerMethod(tMiscNode);
    this->setOptimalityCriteriaOptions(tMiscNode);
    tTmpNode = tMiscNode.append_child("Options");
    this->setMMAoptions(tTmpNode);
    this->setAugmentedLagrangianOptions(tTmpNode);
    this->setTrustRegionAlgorithmOptions(tTmpNode);
    this->setKelleySachsAlgorithmOptions(tTmpNode);

    if(m_InputData.optimization_algorithm == "rol ksal" ||
       m_InputData.optimization_algorithm == "rol ksbc")
    {
        addChild(tTmpNode, "InputFileName", "rol_inputs.xml");
        addChild(tTmpNode, "OutputDiagnosticsToFile", "true");
    }

    tTmpNode = tMiscNode.append_child("Output");
    addChild(tTmpNode, "OutputStage", "Output To File");

    tTmpNode = tMiscNode.append_child("CacheStage");
    addChild(tTmpNode, "Name", "Cache State");

    if(m_InputData.optimization_type == "topology")
    {
        tTmpNode = tMiscNode.append_child("UpdateProblemStage");
        addChild(tTmpNode, "Name", "Update Problem");
    }

    tTmpNode = tMiscNode.append_child("OptimizationVariables");
    if(m_InputData.optimization_type == "topology")
    {
        addChild(tTmpNode, "ValueName", "Optimization DOFs");
        addChild(tTmpNode, "InitializationStage", "Initialize Optimization");
        addChild(tTmpNode, "FilteredName", "Topology");
        addChild(tTmpNode, "LowerBoundValueName", "Lower Bound Value");
        addChild(tTmpNode, "LowerBoundVectorName", "Lower Bound Vector");
        addChild(tTmpNode, "UpperBoundValueName", "Upper Bound Value");
        addChild(tTmpNode, "UpperBoundVectorName", "Upper Bound Vector");
        addChild(tTmpNode, "SetLowerBoundsStage", "Set Lower Bounds");
        addChild(tTmpNode, "SetUpperBoundsStage", "Set Upper Bounds");
        if(m_InputData.optimization_algorithm =="ksbc" ||
                m_InputData.optimization_algorithm == "ksal" ||
                m_InputData.optimization_algorithm == "rol ksal" ||
                m_InputData.optimization_algorithm == "rol ksbc")
        {
            addChild(tTmpNode, "DescentDirectionName", "Descent Direction");
        }
    }
    else if(m_InputData.optimization_type == "shape")
    {
        addChild(tTmpNode, "ValueName", "Design Parameters");
        addChild(tTmpNode, "InitializationStage", "Initialize Design Parameters");
        addChild(tTmpNode, "FilteredName", "Topology");
        addChild(tTmpNode, "LowerBoundValueName", "Lower Bound Value");
        addChild(tTmpNode, "LowerBoundVectorName", "Lower Bound Vector");
        addChild(tTmpNode, "UpperBoundValueName", "Upper Bound Value");
        addChild(tTmpNode, "UpperBoundVectorName", "Upper Bound Vector");
        addChild(tTmpNode, "SetLowerBoundsStage", "Set Lower Bounds");
        addChild(tTmpNode, "SetUpperBoundsStage", "Set Upper Bounds");
        if(m_InputData.optimization_algorithm =="ksbc" ||
                m_InputData.optimization_algorithm == "ksal" ||
                m_InputData.optimization_algorithm == "rol ksal" ||
                m_InputData.optimization_algorithm == "rol ksbc")
        {
            addChild(tTmpNode, "DescentDirectionName", "Descent Direction");
        }
    }

    if(m_InputData.optimization_type == "topology")
    {
        tTmpNode = tMiscNode.append_child("Objective");
        if(m_InputData.mPlatoAnalyzePerformerExists)
          addChild(tTmpNode, "ValueName", "Objective Value");
        else
          addChild(tTmpNode, "ValueName", "Internal Energy");
        if(m_InputData.mPlatoAnalyzePerformerExists)
          addChild(tTmpNode, "ValueStageName", "Objective Value");
        else
          addChild(tTmpNode, "ValueStageName", "Internal Energy");
        if(m_InputData.mPlatoAnalyzePerformerExists)
          addChild(tTmpNode, "GradientName", "Objective Gradient");
        else
          addChild(tTmpNode, "GradientName", "Internal Energy Gradient");
        if(m_InputData.mPlatoAnalyzePerformerExists)
          addChild(tTmpNode, "GradientStageName", "Objective Gradient");
        else
          addChild(tTmpNode, "GradientStageName", "Internal Energy Gradient");
        if(m_InputData.optimization_algorithm =="ksbc" ||
                m_InputData.optimization_algorithm == "ksal" ||
                m_InputData.optimization_algorithm == "rol ksal" ||
                m_InputData.optimization_algorithm == "rol ksbc")
        {
            addChild(tTmpNode, "HessianName", "Internal Energy Hessian");
        }
    }
    else if(m_InputData.optimization_type == "shape")
    {
        tTmpNode = tMiscNode.append_child("Objective");
        addChild(tTmpNode, "ValueName", "Objective");
        addChild(tTmpNode, "ValueStageName", "Objective");
        addChild(tTmpNode, "GradientName", "Objective Gradient");
        addChild(tTmpNode, "GradientStageName", "Objective Gradient");
        if(m_InputData.optimization_algorithm =="ksbc" ||
                m_InputData.optimization_algorithm == "ksal" ||
                m_InputData.optimization_algorithm == "rol ksal" ||
                m_InputData.optimization_algorithm == "rol ksbc")
        {
            addChild(tTmpNode, "HessianName", "Internal Energy Hessian");
        }
    }

    tTmpNode = tMiscNode.append_child("BoundConstraint");
    if(m_InputData.discretization == "density")
    {
        addChild(tTmpNode, "Upper", "1.0");
        addChild(tTmpNode, "Lower", "0.0");
    }
    else
    {
        addChild(tTmpNode, "Upper", "10.0");
        addChild(tTmpNode, "Lower", "-10.0");
    }

    if(m_InputData.optimization_type == "topology")
    {
        for(size_t b=0; b<m_InputData.constraints.size(); ++b)
        {
            tTmpNode = tMiscNode.append_child("Constraint");
            if(m_InputData.constraints[b].type == "volume")
            {
                addChild(tTmpNode, "Equality", "True");
                addChild(tTmpNode, "Linear", "True");
                addChild(tTmpNode, "ValueName", "Volume");
                addChild(tTmpNode, "ReferenceValueName", "Design Volume");
                addChild(tTmpNode, "GradientName", "Volume Gradient");
                addChild(tTmpNode, "ValueStageName", "Volume");
                addChild(tTmpNode, "GradientStageName", "Volume Gradient");
                if(m_InputData.constraints[b].mNormalizedTargetValue != "")
                    addChild(tTmpNode, "NormalizedTargetValue", m_InputData.constraints[b].mNormalizedTargetValue);
                if(m_InputData.constraints[b].mAbsoluteTargetValue != "")
                    addChild(tTmpNode, "AbsoluteTargetValue", m_InputData.constraints[b].mAbsoluteTargetValue);
                addChild(tTmpNode, "Tolerance", "1e-3");
            }
            else if(m_InputData.constraints[b].type == "surface area")
            {
                addChild(tTmpNode, "Equality", "True");
                addChild(tTmpNode, "Linear", "True");
                addChild(tTmpNode, "ValueName", "Surface Area");
                addChild(tTmpNode, "ReferenceValue", m_InputData.constraints[b].surface_area);
                addChild(tTmpNode, "GradientName", "Surface Area Gradient");
                addChild(tTmpNode, "NormalizedTargetValue", "1.0");
                addChild(tTmpNode, "Tolerance", "1e-3");
                addChild(tTmpNode, "ValueStageName", "Surface Area");
                addChild(tTmpNode, "GradientStageName", "Surface Area Gradient");
            }
        }
    }
    else if(m_InputData.optimization_type == "shape")
    {
        tTmpNode = tMiscNode.append_child("Constraint");
        addChild(tTmpNode, "ValueName", "Constraint");
        addChild(tTmpNode, "ValueStageName", "Constraint");
        addChild(tTmpNode, "GradientName", "Constraint Gradient");
        addChild(tTmpNode, "GradientStageName", "Constraint Gradient");
        if(m_InputData.constraints[0].mAbsoluteTargetValue != "")
            addChild(tTmpNode, "AbsoluteTargetValue", m_InputData.constraints[0].mAbsoluteTargetValue);
    }

    if(m_InputData.optimization_algorithm != "mma")
    {
        tTmpNode = tMiscNode.append_child("Convergence");
        addChild(tTmpNode, "MaxIterations", m_InputData.max_iterations);
    }

    if(m_InputData.optimization_type == "topology")
    {
        // mesh
        pugi::xml_node mesh_node = doc.append_child("mesh");
        addChild(mesh_node, "type", "unstructured");
        addChild(mesh_node, "format", "exodus");
        addChild(mesh_node, "mesh", m_InputData.run_mesh_name);
        for(size_t n=0; n<m_InputData.blocks.size(); ++n)
        {
            tTmpNode = mesh_node.append_child("block");
            addChild(tTmpNode, "index", m_InputData.blocks[n].block_id);
            pugi::xml_node tmp_node1 = tTmpNode.append_child("integration");
            addChild(tmp_node1, "type", "gauss");
            addChild(tmp_node1, "order", "2");
            addChild(tTmpNode, "material", m_InputData.blocks[n].material_id);
        }

        // output
        tTmpNode = doc.append_child("output");
        addChild(tTmpNode, "file", "plato");
        addChild(tTmpNode, "format", "exodus");
    }

    // Write to file or string
    if(aStringStream)
        doc.save(*aStringStream, "\t", pugi::format_default & ~pugi::format_indent);
    else    
        doc.save_file("interface.xml", "  ");

    return true;
}

/******************************************************************************/
bool DefaultInputGenerator::generatePlatoAnalyzeShapeInterfaceXML()
/******************************************************************************/
{
    pugi::xml_document tDoc;
    pugi::xml_node tTmpNode, tForNode;
    char tmp_buf[200];

    // Version entry
    tTmpNode = tDoc.append_child(pugi::node_declaration);
    tTmpNode.set_name("xml");
    pugi::xml_attribute tTmpAtt = tTmpNode.append_attribute("version");
    tTmpAtt.set_value("1.0");
    
    // Console output control
    tTmpNode = tDoc.append_child("Console");
    addChild(tTmpNode, "Verbose", "true");
    addChild(tTmpNode, "Enabled", "true");

    // defines.xml
    tTmpNode = tDoc.append_child("include");
    tTmpNode.append_attribute("filename") = "defines.xml";

    //////////////////////////////////////////////////
    // Performers
    /////////////////////////////////////////////////

    // PlatoMain performer entry.
    tTmpNode = tDoc.append_child("Performer");
    addChild(tTmpNode, "Name", "PlatoMain");
    addChild(tTmpNode, "Code", "PlatoMain");
    addChild(tTmpNode, "PerformerID", "0");

    // ESP performers
    tTmpNode = tDoc.append_child("Performer");
    tForNode = tTmpNode.append_child("For");
    tForNode.append_attribute("var") = "I";
    tForNode.append_attribute("in") = "Parameters";
    addChild(tForNode, "Name", "PlatoESP_{I}");
    addChild(tTmpNode, "PerformerID", "1");

    // Other PA perfromers
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        tTmpNode = tDoc.append_child("Performer");
        addChild(tTmpNode, "Name", m_InputData.objectives[i].mPerformerName);
        sprintf(tmp_buf, "%d", (int)(i+2));
        addChild(tTmpNode, "PerformerID", tmp_buf);
    }

    //////////////////////////////////////////////////
    // Shared Data
    /////////////////////////////////////////////////

    // Performer Objectives
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        sprintf(tmp_buf, "Objective %d", (int)(i+1));
        createSingleUserGlobalSharedData(tDoc, tmp_buf, "Scalar", "1", m_InputData.objectives[i].mPerformerName, "PlatoMain");
    }

    // Aggregated Objective
    createSingleUserGlobalSharedData(tDoc, "Objective", "Scalar", "1", "PlatoMain", "PlatoMain");

    // Performer Objective Gradients
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        char tTempBuffer[100];
        sprintf(tTempBuffer, "%d", m_InputData.num_shape_design_variables);
        sprintf(tmp_buf, "Objective %d Gradient", (int)(i+1));
        createSingleUserGlobalSharedData(tDoc, tmp_buf, "Scalar", tTempBuffer, m_InputData.objectives[i].mPerformerName, "PlatoMain");
    }

    // Aggregated Objective Gradient
    sprintf(tmp_buf, "%d", m_InputData.num_shape_design_variables);
    createSingleUserGlobalSharedData(tDoc, "Objective Gradient", "Scalar", tmp_buf, "PlatoMain", "PlatoMain");

    // Design Parameters
    sprintf(tmp_buf, "%d", m_InputData.num_shape_design_variables);
    std::vector<std::string> tUserNames;
    tUserNames.push_back("PlatoMain");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        tUserNames.push_back(m_InputData.objectives[i].mPerformerName);
    createMultiUserGlobalSharedData(tDoc, "Design Parameters", "Scalar", tmp_buf, "PlatoMain", tUserNames);

    // Design Parameter Sensitivities
    tForNode = tDoc.append_child("For");
    tForNode.append_attribute("var") = "I";
    tForNode.append_attribute("in") = "Parameters";
    tTmpNode = tForNode.append_child("SharedData");
    addChild(tTmpNode, "Name", "Parameter Sensitivity {I}");
    addChild(tTmpNode, "Type", "Scalar");
    addChild(tTmpNode, "Layout", "Global");
    addChild(tTmpNode, "Dynamic", "true");
    addChild(tTmpNode, "OwnerName", "PlatoESP_{I}");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        addChild(tTmpNode, "UserName", m_InputData.objectives[i].mPerformerName);

    // Constraint
    createSingleUserGlobalSharedData(tDoc, "Constraint", "Scalar", "1", m_InputData.objectives[0].mPerformerName, "PlatoMain");

    // Constraint Gradient
    sprintf(tmp_buf, "%d", m_InputData.num_shape_design_variables);
    createSingleUserGlobalSharedData(tDoc, "Constraint Gradient", "Scalar", tmp_buf, m_InputData.objectives[0].mPerformerName, "PlatoMain");

    // Lower/Upper Bound Vectors
    sprintf(tmp_buf, "%d", m_InputData.num_shape_design_variables);
    createSingleUserGlobalSharedData(tDoc, "Lower Bound Vector", "Scalar", tmp_buf, "PlatoMain", "PlatoMain");
    createSingleUserGlobalSharedData(tDoc, "Upper Bound Vector", "Scalar", tmp_buf, "PlatoMain", "PlatoMain");

    //////////////////////////////////////////////////
    // Stages
    /////////////////////////////////////////////////

    // Output To File
    outputOutputToFileStage(tDoc, m_InputData.m_HasUncertainties, m_InputData.m_RequestedVonMisesOutput);

    // Initialize Optimization
    outputInitializeOptimizationStage(tDoc);

    // Cache State Stage
    outputCacheStateStage(tDoc, m_InputData.m_HasUncertainties);

    // Set Lower Bounds Stage
    outputSetLowerBoundsStage(tDoc);

    // Set Upper Bounds
    outputSetUpperBoundsStage(tDoc);

    // Constraint
    outputConstraintStage(tDoc);
  
    // Constraint Gradient
    outputConstraintGradientStage(tDoc);

    // Objective
    outputObjectiveStageForShape(tDoc);

    // Objective Gradient
    outputObjectiveGradientStageForShape(tDoc);

    /////////////////////////////////////////////////
    // Misc.
    ////////////////////////////////////////////////

    pugi::xml_node tMiscNode = tDoc.append_child("Optimizer");
    this->setOptimizerMethod(tMiscNode);
    this->setOptimalityCriteriaOptions(tMiscNode);
    tTmpNode = tMiscNode.append_child("Options");
    this->setMMAoptions(tTmpNode);
    this->setAugmentedLagrangianOptions(tTmpNode);
    this->setTrustRegionAlgorithmOptions(tTmpNode);
    this->setKelleySachsAlgorithmOptions(tTmpNode);

    if(m_InputData.optimization_algorithm == "rol ksal" ||
       m_InputData.optimization_algorithm == "rol ksbc")
    {
        addChild(tTmpNode, "InputFileName", "rol_inputs.xml");
        addChild(tTmpNode, "OutputDiagnosticsToFile", "true");
    }

    tTmpNode = tMiscNode.append_child("Output");
    addChild(tTmpNode, "OutputStage", "Output To File");

    tTmpNode = tMiscNode.append_child("CacheStage");
    addChild(tTmpNode, "Name", "Cache State");

    if(m_InputData.optimization_type == "topology")
    {
        tTmpNode = tMiscNode.append_child("UpdateProblemStage");
        addChild(tTmpNode, "Name", "Update Problem");
    }

    tTmpNode = tMiscNode.append_child("OptimizationVariables");
    addChild(tTmpNode, "ValueName", "Design Parameters");
    addChild(tTmpNode, "InitializationStage", "Initialize Design Parameters");
    addChild(tTmpNode, "FilteredName", "Topology");
    addChild(tTmpNode, "LowerBoundValueName", "Lower Bound Value");
    addChild(tTmpNode, "LowerBoundVectorName", "Lower Bound Vector");
    addChild(tTmpNode, "UpperBoundValueName", "Upper Bound Value");
    addChild(tTmpNode, "UpperBoundVectorName", "Upper Bound Vector");
    addChild(tTmpNode, "SetLowerBoundsStage", "Set Lower Bounds");
    addChild(tTmpNode, "SetUpperBoundsStage", "Set Upper Bounds");
    if(m_InputData.optimization_algorithm =="ksbc" ||
            m_InputData.optimization_algorithm == "ksal" ||
            m_InputData.optimization_algorithm == "rol ksal" ||
            m_InputData.optimization_algorithm == "rol ksbc")
    {
        addChild(tTmpNode, "DescentDirectionName", "Descent Direction");
    }

    tTmpNode = tMiscNode.append_child("Objective");
    addChild(tTmpNode, "ValueName", "Objective");
    addChild(tTmpNode, "ValueStageName", "Objective");
    addChild(tTmpNode, "GradientName", "Objective Gradient");
    addChild(tTmpNode, "GradientStageName", "Objective Gradient");
    if(m_InputData.optimization_algorithm =="ksbc" ||
            m_InputData.optimization_algorithm == "ksal" ||
            m_InputData.optimization_algorithm == "rol ksal" ||
            m_InputData.optimization_algorithm == "rol ksbc")
    {
        addChild(tTmpNode, "HessianName", "Internal Energy Hessian");
    }

    tTmpNode = tMiscNode.append_child("BoundConstraint");
    if(m_InputData.discretization == "density")
    {
        addChild(tTmpNode, "Upper", "1.0");
        addChild(tTmpNode, "Lower", "0.0");
    }
    else
    {
        addChild(tTmpNode, "Upper", "10.0");
        addChild(tTmpNode, "Lower", "-10.0");
    }

    tTmpNode = tMiscNode.append_child("Constraint");
    addChild(tTmpNode, "ValueName", "Constraint");
    addChild(tTmpNode, "ValueStageName", "Constraint");
    addChild(tTmpNode, "GradientName", "Constraint Gradient");
    addChild(tTmpNode, "GradientStageName", "Constraint Gradient");
    if(m_InputData.constraints[0].mAbsoluteTargetValue != "")
        addChild(tTmpNode, "AbsoluteTargetValue", m_InputData.constraints[0].mAbsoluteTargetValue);

    if(m_InputData.optimization_algorithm != "mma")
    {
        tTmpNode = tMiscNode.append_child("Convergence");
        addChild(tTmpNode, "MaxIterations", m_InputData.max_iterations);
    }

    // Write the file to disk
    tDoc.save_file("interface.xml", "  ");

    return true;
}

/**********************************************************************************/
void DefaultInputGenerator::outputCacheStateStage(pugi::xml_document &doc,
                                         const bool &aHasUncertainties)
/**********************************************************************************/
{
    pugi::xml_node op_node, output_node;
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Cache State");

    size_t tNum_objectives = m_InputData.objectives.size();
    size_t tNum_to_write = 0;
    bool tHandles_cache_state;

    for(size_t i = 0; i < tNum_objectives; i++)
    {
      XMLGen::Objective cur_obj = m_InputData.objectives[i];
      tHandles_cache_state = cur_obj.code_name.compare("albany") && cur_obj.code_name.compare("plato_analyze") &&
              cur_obj.code_name.compare("lightmp"); // Albany, analyze, and lightmp don't handle Cache State correctly yet

      if(tHandles_cache_state)
        ++tNum_to_write;
    }

    pugi::xml_node cur_parent = stage_node;
    if(tNum_to_write > 1)
    {
        op_node = stage_node.append_child("Operation");
        cur_parent = op_node;
    }
    for(size_t i=0; i<tNum_objectives; ++i)
    {

        XMLGen::Objective cur_obj = m_InputData.objectives[i];

        tHandles_cache_state = cur_obj.code_name.compare("albany") && cur_obj.code_name.compare("plato_analyze") &&
                  cur_obj.code_name.compare("lightmp"); // Albany, analyze, and lightmp don't handle Cache State correctly yet

        if(tHandles_cache_state)
        {
            op_node = cur_parent.append_child("Operation");
            addChild(op_node, "Name", "Cache State");
            addChild(op_node, "PerformerName", cur_obj.mPerformerName);
            if(aHasUncertainties)
            {
                output_node = op_node.append_child("Output");
                addChild(output_node, "ArgumentName", "vonmises0");
                addChild(output_node, "SharedDataName", cur_obj.mPerformerName + "_" + "vonmises");
            }
            else if(cur_obj.multi_load_case == "true")
            {
                for(size_t k=0; k<cur_obj.load_case_ids.size(); k++)
                {
                    char buffer[100];
                    sprintf(buffer, "%lu", k);
                    std::string cur_load_string = cur_obj.load_case_ids[k];
                    for(size_t j=0; j<cur_obj.output_for_plotting.size(); j++)
                    {
                        output_node = op_node.append_child("Output");
                        addChild(output_node, "ArgumentName", cur_obj.output_for_plotting[j] + buffer);
                        addChild(output_node, "SharedDataName", cur_obj.mPerformerName + "_" + "load" + cur_load_string + "_" + cur_obj.output_for_plotting[j]);
                    }
                }
            }
            else
            {
                for(size_t j=0; j<cur_obj.output_for_plotting.size(); j++)
                {
                    output_node = op_node.append_child("Output");
                    addChild(output_node, "ArgumentName", cur_obj.output_for_plotting[j] + "0");
                    addChild(output_node, "SharedDataName", cur_obj.mPerformerName + "_" + cur_obj.output_for_plotting[j]);
                }
            }
        }
    }
}

/**********************************************************************************/
void DefaultInputGenerator::outputSetLowerBoundsStage(pugi::xml_document &doc)
/**********************************************************************************/
{
    pugi::xml_node stage_node, input_node, op_node, output_node;
    stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Set Lower Bounds");
    if(m_InputData.optimization_type == "topology")
    {
        input_node = stage_node.append_child("Input");
        addChild(input_node, "SharedDataName", "Lower Bound Value");

        op_node = stage_node.append_child("Operation");
        addChild(op_node, "Name", "Calculate Lower Bounds");
        addChild(op_node, "PerformerName", "PlatoMain");

        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Lower Bound Value");
        addChild(input_node, "SharedDataName", "Lower Bound Value");

        output_node = op_node.append_child("Output");
        addChild(output_node, "ArgumentName", "Lower Bound Vector");
        addChild(output_node, "SharedDataName", "Lower Bound Vector");
    }

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Lower Bound Vector");
}

/**********************************************************************************/
void DefaultInputGenerator::outputSetUpperBoundsStage(pugi::xml_document &doc)
/**********************************************************************************/
{
    pugi::xml_node stage_node, input_node, op_node, output_node;
    stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Set Upper Bounds");
    if(m_InputData.optimization_type == "topology")
    {
        input_node = stage_node.append_child("Input");
        addChild(input_node, "SharedDataName", "Upper Bound Value");

        op_node = stage_node.append_child("Operation");
        addChild(op_node, "Name", "Calculate Upper Bounds");
        addChild(op_node, "PerformerName", "PlatoMain");

        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Upper Bound Value");
        addChild(input_node, "SharedDataName", "Upper Bound Value");

        output_node = op_node.append_child("Output");
        addChild(output_node, "ArgumentName", "Upper Bound Vector");
        addChild(output_node, "SharedDataName", "Upper Bound Vector");
    }

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Upper Bound Vector");
}

/**********************************************************************************/
void DefaultInputGenerator::outputUpdateProblemStage(pugi::xml_document &doc)
/**********************************************************************************/
{
    pugi::xml_node op_node;
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Update Problem");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        // Hack
        if(m_InputData.objectives[i].code_name != "plato_analyze")
        {
            op_node = stage_node.append_child("Operation");
            addChild(op_node, "Name", "Update Problem");
            XMLGen::Objective cur_obj = m_InputData.objectives[i];
            addChild(op_node, "PerformerName", cur_obj.mPerformerName);
        }
    }
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Update Problem");
    addChild(op_node, "PerformerName", "PlatoMain");
}

/**********************************************************************************/
void DefaultInputGenerator::outputInitializeOptimizationStage(pugi::xml_document &doc)
/**********************************************************************************/
{
    if(m_InputData.optimization_type == "topology")
        outputInitializeOptimizationStageForTO(doc);
    else if(m_InputData.optimization_type == "shape")
        outputInitializeOptimizationStageForSO(doc);

}

/**********************************************************************************/
void DefaultInputGenerator::outputInitializeOptimizationStageForSO(pugi::xml_document &doc)
/**********************************************************************************/
{
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Initialize Design Parameters");

    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Initialize Values");
    addChild(op_node, "PerformerName", "PlatoMain");

    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Values");
    addChild(output_node, "SharedDataName", "Design Parameters");
    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Lower Bounds");
    addChild(output_node, "SharedDataName", "Lower Bound Vector");
    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Upper Bounds");
    addChild(output_node, "SharedDataName", "Upper Bound Vector");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Design Parameters");
    /*
    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Lower Bound Vector");
    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Upper Bound Vector");
    */
}

/**********************************************************************************/
void DefaultInputGenerator::outputInitializeOptimizationStageForTO(pugi::xml_document &doc)
/**********************************************************************************/
{
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Initialize Optimization");

    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Initialize Field");
    addChild(op_node, "PerformerName", "PlatoMain");

    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Initialized Field");
    addChild(output_node, "SharedDataName", "Optimization DOFs");

    if(m_InputData.mUseNormalizationInAggregator == "true")
    {
        // Filter control
        op_node = stage_node.append_child("Operation");
        addChild(op_node, "Name", "FilterControl");
        addChild(op_node, "PerformerName", "PlatoMain");
        pugi::xml_node input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Field");
        addChild(input_node, "SharedDataName", "Optimization DOFs");
        output_node = op_node.append_child("Output");
        addChild(output_node, "ArgumentName", "Filtered Field");
        addChild(output_node, "SharedDataName", "Topology");

        if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
        {
            pugi::xml_node for_node = stage_node.append_child("For");
            for_node.append_attribute("var") = "performerIndex";
            for_node.append_attribute("in") = "Performers";
            for_node = for_node.append_child("For");
            for_node.append_attribute("var") = "PerformerSampleIndex";
            for_node.append_attribute("in") = "PerformerSamples";
            op_node = for_node.append_child("Operation");
            addChild(op_node, "Name", "Compute Objective Value");
            addChild(op_node, "PerformerName", "plato_analyze_{performerIndex}");
            input_node = op_node.append_child("Input");
            addChild(input_node, "ArgumentName", "Topology");
            addChild(input_node, "SharedDataName", "Topology");
            output_node = op_node.append_child("Output");
            addChild(output_node, "ArgumentName", "Objective Value");
            addChild(output_node, "SharedDataName", "Initial Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
        }
        else
        {
          for(size_t i=0; i<m_InputData.objectives.size(); ++i)
          {
            XMLGen::Objective cur_obj = m_InputData.objectives[i];
            op_node = stage_node.append_child("Operation");
            if(cur_obj.code_name == "plato_analyze")
            {
              addChild(op_node, "Name", "Compute Objective Value");
              addChild(op_node, "PerformerName", cur_obj.mPerformerName.c_str());
            }
            else
            {
              addChild(op_node, "Name", "Compute Objective");
              addChild(op_node, "PerformerName", cur_obj.mPerformerName.c_str());
            }

            input_node = op_node.append_child("Input");
            addChild(input_node, "ArgumentName", "Topology");
            addChild(input_node, "SharedDataName", "Topology");

            output_node = op_node.append_child("Output");
            if(m_InputData.objectives[i].code_name == "plato_analyze")
              addChild(output_node, "ArgumentName", "Objective Value");
            else
              addChild(output_node, "ArgumentName", "Internal Energy");
            char tBuffer[800];
            if(m_InputData.objectives[i].code_name == "plato_analyze")
              sprintf(tBuffer, "Initial Objective Value %d", (int)(i+1));
            else
              sprintf(tBuffer, "Initial Internal Energy %d", (int)(i+1));
            addChild(output_node, "SharedDataName", tBuffer);
          }
        }
    }

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Optimization DOFs");
}

/**********************************************************************************/
void DefaultInputGenerator::outputOutputToFileStage(pugi::xml_document &doc,
                                           const bool &aHasUncertainties,
                                           const bool &aRequestedVonMises)
/**********************************************************************************/
{
    char tmp_buf[1000];
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Output To File");
    pugi::xml_node op_node, output_node, input_node;

    if(m_InputData.optimization_type == "topology")
    {
        // *********************************************************
        // This code is a hack and needs to be fixed!!!!!!!!!!!!!
        // This Alexa specific output information should probably
        // go in the Alexa input deck rather than in interface.xml.
        // We can't have it in hear because other performers won't
        // know how to execute this Alexa-specific operation.
        // *********************************************************
        bool tFirstTime = true;
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
            XMLGen::Objective cur_obj = m_InputData.objectives[i];
            if(cur_obj.code_name == "plato_analyze")
            {
                for(size_t j=0; j<cur_obj.output_for_plotting.size(); ++j)
                {
                    if(tFirstTime)
                    {
                        op_node = stage_node.append_child("Operation");
                        addChild(op_node, "Name", "Write Output");
                        addChild(op_node, "PerformerName", cur_obj.mPerformerName);
                        tFirstTime = false;
                    }
                    sprintf(tmp_buf, "%s_%s", cur_obj.mPerformerName.c_str(), cur_obj.output_for_plotting[j].c_str());
                    output_node = op_node.append_child("Output");
                    addChild(output_node, "SharedDataName", tmp_buf);
                    if(cur_obj.output_for_plotting[j] == "dispx")
                        addChild(output_node, "ArgumentName", "Solution X");
                    else if(cur_obj.output_for_plotting[j] == "dispy")
                        addChild(output_node, "ArgumentName", "Solution Y");
                    else if(cur_obj.output_for_plotting[j] == "dispz")
                        addChild(output_node, "ArgumentName", "Solution Z");
                    else if(cur_obj.output_for_plotting[j] == "temperature")
                        addChild(output_node, "ArgumentName", "Solution");
                }
            }
        }

        // VonMises Statistics
        if(aHasUncertainties && aRequestedVonMises)
        {
            op_node = stage_node.append_child("Operation");
            addChild(op_node, "Name", "VonMises Statistics");
            addChild(op_node, "PerformerName", "PlatoMain");
            // We are assuming only one load case per objective/performer which
            // means we will only have one vonmises per performer.
            if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow)
            {
              pugi::xml_node for_node = op_node.append_child("For");
              for_node.append_attribute("var") = "performerIndex";
              for_node.append_attribute("in") = "Performers";
              for_node = for_node.append_child("For");
              for_node.append_attribute("var") = "PerformerSampleIndex";
              for_node.append_attribute("in") = "PerformerSamples";
              input_node = for_node.append_child("Input");
              addChild(input_node, "ArgumentName", "plato_analyze_{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}_vonmises");
              addChild(input_node, "SharedDataName", "plato_analyze_{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}_vonmises");
            }
            else
            {
              for(size_t i=0; i<m_InputData.objectives.size(); ++i)
              {
                XMLGen::Objective cur_obj = m_InputData.objectives[i];
                input_node = op_node.append_child("Input");
                addChild(input_node, "ArgumentName", cur_obj.mPerformerName + "_vonmises");
                addChild(input_node, "SharedDataName", cur_obj.mPerformerName + "_vonmises");
              }
            }
            output_node = op_node.append_child("Output");
            addChild(output_node, "ArgumentName", "von_mises_mean");
            addChild(output_node, "SharedDataName", "VonMises Mean");
            output_node = op_node.append_child("Output");
            addChild(output_node, "ArgumentName", "von_mises_std_dev");
            addChild(output_node, "SharedDataName", "VonMises StdDev");
            for(size_t j=0; j<m_InputData.mStandardDeviations.size(); ++j)
            {
                output_node = op_node.append_child("Output");
                std::string tTmpString = "VonMises Mean Plus ";
                tTmpString += m_InputData.mStandardDeviations[j];
                tTmpString += " StdDev";
                addChild(output_node, "ArgumentName", tTmpString);
                addChild(output_node, "SharedDataName", tTmpString);
            }
        }

        op_node = stage_node.append_child("Operation");
        addChild(op_node, "Name", "PlatoMainOutput");
        addChild(op_node, "PerformerName", "PlatoMain");

        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Topology");
        addChild(input_node, "SharedDataName", "Topology");
        input_node = op_node.append_child("Input");
        if(m_InputData.mPlatoAnalyzePerformerExists)
          addChild(input_node, "ArgumentName", "Objective Gradient");
        else
          addChild(input_node, "ArgumentName", "Internal Energy Gradient");
        if(m_InputData.mPlatoAnalyzePerformerExists)
          addChild(input_node, "SharedDataName", "Objective Gradient");
        else
          addChild(input_node, "SharedDataName", "Internal Energy Gradient");
        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Optimization DOFs");
        addChild(input_node, "SharedDataName", "Optimization DOFs");

        if(m_InputData.constraints.size() > 0)
        {
            if(m_InputData.constraints[0].type == "volume")
            {
                input_node = op_node.append_child("Input");
                addChild(input_node, "ArgumentName", "Volume Gradient");
                addChild(input_node, "SharedDataName", "Volume Gradient");
            }
            else if(m_InputData.constraints[0].type == "surface area")
            {
                input_node = op_node.append_child("Input");
                addChild(input_node, "ArgumentName", "Surface Area Gradient");
                addChild(input_node, "SharedDataName", "Surface Area Gradient");
            }
        }
        if(m_InputData.m_UseNewPlatoAnalyzeUncertaintyWorkflow && m_InputData.m_HasUncertainties)
        {
          pugi::xml_node for_node = op_node.append_child("For");
          for_node.append_attribute("var") = "performerIndex";
          for_node.append_attribute("in") = "Performers";
          for_node = for_node.append_child("For");
          for_node.append_attribute("var") = "PerformerSampleIndex";
          for_node.append_attribute("in") = "PerformerSamples";
          input_node = for_node.append_child("Input");
          addChild(input_node, "ArgumentName", "plato_analyze_{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}_vonmises");
          addChild(input_node, "SharedDataName", "plato_analyze_{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}_vonmises");
        }
        else
        {
          for(size_t i=0; i<m_InputData.objectives.size(); ++i)
          {
            XMLGen::Objective cur_obj = m_InputData.objectives[i];
            if(aHasUncertainties)
            {
              input_node = op_node.append_child("Input");
              sprintf(tmp_buf, "%s_%s", cur_obj.mPerformerName.c_str(), "vonmises");
              addChild(input_node, "ArgumentName", tmp_buf);
              addChild(input_node, "SharedDataName", tmp_buf);
            }
            else if(cur_obj.multi_load_case == "true")
            {
              for(size_t k=0; k<cur_obj.load_case_ids.size(); k++)
              {
                std::string cur_load_string = cur_obj.load_case_ids[k];
                for(size_t j=0; j<cur_obj.output_for_plotting.size(); j++)
                {
                  input_node = op_node.append_child("Input");
                  sprintf(tmp_buf, "%s_load%s_%s", cur_obj.mPerformerName.c_str(), cur_load_string.c_str(), cur_obj.output_for_plotting[j].c_str());
                  addChild(input_node, "ArgumentName", tmp_buf);
                  addChild(input_node, "SharedDataName", tmp_buf);
                }
              }
            }
            else
            {
              for(size_t j=0; j<cur_obj.output_for_plotting.size(); j++)
              {
                input_node = op_node.append_child("Input");
                sprintf(tmp_buf, "%s_%s", cur_obj.mPerformerName.c_str(), cur_obj.output_for_plotting[j].c_str());
                addChild(input_node, "ArgumentName", tmp_buf);
                addChild(input_node, "SharedDataName", tmp_buf);
              }
            }
          }
        }

        if(aHasUncertainties && aRequestedVonMises)
        {
            input_node = op_node.append_child("Input");
            addChild(input_node, "ArgumentName", "VonMises Mean");
            addChild(input_node, "SharedDataName", "VonMises Mean");
            input_node = op_node.append_child("Input");
            addChild(input_node, "ArgumentName", "VonMises StdDev");
            addChild(input_node, "SharedDataName", "VonMises StdDev");
            for(size_t j=0; j<m_InputData.mStandardDeviations.size(); ++j)
            {
                input_node = op_node.append_child("Input");
                std::string tTmpString = "VonMises Mean Plus ";
                tTmpString += m_InputData.mStandardDeviations[j];
                tTmpString += " StdDev";
                addChild(input_node, "ArgumentName", tTmpString);
                addChild(input_node, "SharedDataName", tTmpString);
            }
        }
    }
    else if(m_InputData.optimization_type == "shape")
    {
        op_node = stage_node.append_child("Operation");
        addChild(op_node, "Name", "CSMMeshOutput");
        addChild(op_node, "PerformerName", "PlatoMain");
    }
}

/**********************************************************************************/
void DefaultInputGenerator::outputDesignVolumeStage(pugi::xml_document &doc)
/**********************************************************************************/
{
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Design Volume");

    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Design Volume");
    addChild(op_node, "PerformerName", "PlatoMain");

    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Design Volume");
    addChild(output_node, "SharedDataName", "Design Volume");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Design Volume");
}

/**********************************************************************************/
bool DefaultInputGenerator::setOptimizerMethod(pugi::xml_node & aXMLnode)
/**********************************************************************************/
{
    if(m_InputData.optimization_algorithm.compare("oc") == 0)
    {
        addChild(aXMLnode, "Package", "OC");
    }
    else if(m_InputData.optimization_algorithm.compare("gcmma") == 0)
    {
        addChild(aXMLnode, "Package", "GCMMA");
    }
    else if(m_InputData.optimization_algorithm.compare("mma") == 0)
    {
        addChild(aXMLnode, "Package", "MMA");
    }
    else if(m_InputData.optimization_algorithm.compare("ksbc") == 0)
    {
        addChild(aXMLnode, "Package", "KSBC");
    }
    else if(m_InputData.optimization_algorithm.compare("ksal") == 0)
    {
        addChild(aXMLnode, "Package", "KSAL");
    }
    else if(m_InputData.optimization_algorithm.compare("rol ksbc") == 0)
    {
        addChild(aXMLnode, "Package", "ROL KSBC");
    }
    else if(m_InputData.optimization_algorithm.compare("rol ksal") == 0)
    {
        addChild(aXMLnode, "Package", "ROL KSAL");
    }
    else if(m_InputData.optimization_algorithm.compare("derivativechecker") == 0)
    {
        addChild(aXMLnode, "Package", "DerivativeChecker");
        addChild(aXMLnode, "CheckGradient", m_InputData.check_gradient);
        addChild(aXMLnode, "CheckHessian", m_InputData.check_hessian);
        addChild(aXMLnode, "UseUserInitialGuess", "True");
        pugi::xml_node tTmpNode = aXMLnode.append_child("Options");
        addChild(tTmpNode, "DerivativeCheckerInitialSuperscript", "1");
        addChild(tTmpNode, "DerivativeCheckerFinalSuperscript", "8");
    }

    return true;
}

/**********************************************************************************/
bool DefaultInputGenerator::setOptimalityCriteriaOptions(pugi::xml_node & aXMLnode)
/**********************************************************************************/
{
    if(m_InputData.optimization_algorithm.compare("oc") == 0)
    {
//        addChild(aXMLnode, "Package", "OC");
        pugi::xml_node tTmpNode = aXMLnode.append_child("OC");
        addChild(tTmpNode, "MoveLimiter", "1.0");
        addChild(tTmpNode, "StabilizationParameter", "0.5");
        addChild(tTmpNode, "UseNewtonSearch", "True");
        addChild(tTmpNode, "ConstraintMaxIterations", "25");
    }

    return true;
}

/**********************************************************************************/
bool DefaultInputGenerator::setTrustRegionAlgorithmOptions(const pugi::xml_node & aXMLnode)
/**********************************************************************************/
{
    if(m_InputData.mMaxTrustRegionRadius.size() > 0)
    {
        addChild(aXMLnode, "MaxTrustRegionRadius", m_InputData.mMaxTrustRegionRadius);
    }
    if(m_InputData.mMinTrustRegionRadius.size() > 0)
    {
        addChild(aXMLnode, "MinTrustRegionRadius", m_InputData.mMinTrustRegionRadius);
    }
    if(m_InputData.mTrustRegionExpansionFactor.size() > 0)
    {
        addChild(aXMLnode, "KSTrustRegionExpansionFactor", m_InputData.mTrustRegionExpansionFactor);
    }
    if(m_InputData.mTrustRegionContractionFactor.size() > 0)
    {
        addChild(aXMLnode, "KSTrustRegionContractionFactor", m_InputData.mTrustRegionContractionFactor);
    }
    if(m_InputData.mMaxTrustRegionIterations.size() > 0)
    {
        addChild(aXMLnode, "KSMaxTrustRegionIterations", m_InputData.mMaxTrustRegionIterations);
    }
    if(m_InputData.mInitialRadiusScale.size() > 0)
    {
        addChild(aXMLnode, "KSInitialRadiusScale", m_InputData.mInitialRadiusScale);
    }
    if(m_InputData.mMaxRadiusScale.size() > 0)
    {
        addChild(aXMLnode, "KSMaxRadiusScale", m_InputData.mMaxRadiusScale);
    }

    return (true);
}

/**********************************************************************************/
bool DefaultInputGenerator::setAugmentedLagrangianOptions(const pugi::xml_node & aXMLnode)
/**********************************************************************************/
{
    if(m_InputData.mUseMeanNorm.size() > 0)
    {
        addChild(aXMLnode, "UseMeanNorm", m_InputData.mUseMeanNorm);
    }
    if(m_InputData.mAugLagPenaltyParam.size() > 0)
    {
        addChild(aXMLnode, "AugLagPenaltyParam", m_InputData.mAugLagPenaltyParam);
    }
    if(m_InputData.mAugLagPenaltyParamScale.size() > 0)
    {
        addChild(aXMLnode, "AugLagPenaltyParamScaleFactor", m_InputData.mAugLagPenaltyParamScale);
    }
    if(m_InputData.mMaxNumAugLagSubProbIter.size() > 0)
    {
        addChild(aXMLnode, "MaxNumAugLagSubProbIter", m_InputData.mMaxNumAugLagSubProbIter);
    }
    if(m_InputData.mFeasibilityTolerance.size() > 0)
    {
        addChild(aXMLnode, "FeasibilityTolerance", m_InputData.mFeasibilityTolerance);
    }

    return (true);
}

/**********************************************************************************/
bool DefaultInputGenerator::setKelleySachsAlgorithmOptions(const pugi::xml_node & aXMLnode)
/**********************************************************************************/
{
    if(m_InputData.mHessianType.size() > 0)
    {
        addChild(aXMLnode, "HessianType", m_InputData.mHessianType);
    }
    if(m_InputData.mLimitedMemoryStorage.size() > 0)
    {
        addChild(aXMLnode, "LimitedMemoryStorage", m_InputData.mLimitedMemoryStorage);
    }
    if(m_InputData.mOuterGradientToleranceKS.size() > 0)
    {
        addChild(aXMLnode, "KSOuterGradientTolerance", m_InputData.mOuterGradientToleranceKS);
    }
    if(m_InputData.mOuterStationarityToleranceKS.size() > 0)
    {
        addChild(aXMLnode, "KSOuterStationarityTolerance", m_InputData.mOuterStationarityToleranceKS);
    }
    if(m_InputData.mOuterStagnationToleranceKS.size() > 0)
    {
        addChild(aXMLnode, "KSOuterStagnationTolerance", m_InputData.mOuterStagnationToleranceKS);
    }
    if(m_InputData.mOuterControlStagnationToleranceKS.size() > 0)
    {
        addChild(aXMLnode, "KSOuterControlStagnationTolerance", m_InputData.mOuterControlStagnationToleranceKS);
    }
    if(m_InputData.mOuterActualReductionToleranceKS.size() > 0)
    {
        addChild(aXMLnode, "KSOuterActualReductionTolerance", m_InputData.mOuterActualReductionToleranceKS);
    }
    if(m_InputData.mProblemUpdateFrequency.size() > 0)
    {
        addChild(aXMLnode, "ProblemUpdateFrequency", m_InputData.mProblemUpdateFrequency);
    }
    if(m_InputData.mDisablePostSmoothingKS.size() > 0)
    {
        addChild(aXMLnode, "DisablePostSmoothing", m_InputData.mDisablePostSmoothingKS);
    }
    if(m_InputData.mTrustRegionRatioLowKS.size() > 0)
    {
        addChild(aXMLnode, "KSTrustRegionRatioLow", m_InputData.mTrustRegionRatioLowKS);
    }
    if(m_InputData.mTrustRegionRatioMidKS.size() > 0)
    {
        addChild(aXMLnode, "KSTrustRegionRatioMid", m_InputData.mTrustRegionRatioMidKS);
    }
    if(m_InputData.mTrustRegionRatioUpperKS.size() > 0)
    {
        addChild(aXMLnode, "KSTrustRegionRatioUpper", m_InputData.mTrustRegionRatioUpperKS);
    }

    return (true);
}

/**********************************************************************************/
bool DefaultInputGenerator::setMMAoptions(const pugi::xml_node & aXMLnode)
/**********************************************************************************/
{
    if(m_InputData.max_iterations.size() > 0)
    {
        addChild(aXMLnode, "MaxNumOuterIterations", m_InputData.max_iterations);
    }
    if(m_InputData.mMMAMoveLimit.size() > 0)
    {
        addChild(aXMLnode, "MoveLimit", m_InputData.mMMAMoveLimit);
    }
    if(m_InputData.mMMAAsymptoteExpansion.size() > 0)
    {
        addChild(aXMLnode, "AsymptoteExpansion", m_InputData.mMMAAsymptoteExpansion);
    }
    if(m_InputData.mMMAAsymptoteContraction.size() > 0)
    {
        addChild(aXMLnode, "AsymptoteContraction", m_InputData.mMMAAsymptoteContraction);
    }
    if(m_InputData.mMMAMaxNumSubProblemIterations.size() > 0)
    {
        addChild(aXMLnode, "MaxNumSubProblemIter", m_InputData.mMMAMaxNumSubProblemIterations);
    }
    if(m_InputData.mMMAControlStagnationTolerance.size() > 0)
    {
        addChild(aXMLnode, "ControlStagnationTolerance", m_InputData.mMMAControlStagnationTolerance);
    }
    if(m_InputData.mMMAObjectiveStagnationTolerance.size() > 0)
    {
        addChild(aXMLnode, "ObjectiveStagnationTolerance", m_InputData.mMMAObjectiveStagnationTolerance);
    }

    return (true);
}

}


