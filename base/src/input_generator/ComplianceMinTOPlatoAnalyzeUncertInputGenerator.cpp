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
 * ComplianceMinTOPlatoAnalyzeUncertInputGenerator.cpp
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

#include "XMLGeneratorUtilities.hpp"

#include "ComplianceMinTOPlatoAnalyzeUncertInputGenerator.hpp"
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
ComplianceMinTOPlatoAnalyzeUncertInputGenerator::ComplianceMinTOPlatoAnalyzeUncertInputGenerator(const InputData& aInputData) :
DefaultInputGenerator(aInputData)
/******************************************************************************/
{
}

/******************************************************************************/
ComplianceMinTOPlatoAnalyzeUncertInputGenerator::~ComplianceMinTOPlatoAnalyzeUncertInputGenerator() 
/******************************************************************************/
{
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeUncertInputGenerator::generateInputFiles()
/******************************************************************************/
{
    XMLGen::write_define_xml_file(m_InputData.mRandomMetaData, m_InputData.m_UncertaintyMetaData);
    XMLGen::write_interface_xml_file_for_nondeterministic_usecase(m_InputData);
    XMLGen::write_plato_main_operations_xml_file_for_nondeterministic_usecase(m_InputData);
    XMLGen::write_plato_main_input_deck_file(m_InputData);

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

    if(!generateLaunchScript())
    {
        std::cout << "Failed to generate mpirun.source" << std::endl;
        return false;
    }

    std::cout << "Successfully wrote XML files." << std::endl;
    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeUncertInputGenerator::generateLaunchScript()
/******************************************************************************/
{
    if(m_InputData.m_Arch == XMLGen::Arch::SUMMIT)
      XMLGen::generate_summit_launch_scripts(m_InputData);
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

      // Now add the main mpirun call.
      fprintf(fp, "%s %s %s %s PLATO_PERFORMER_ID%s0 \\\n", tLaunchString.c_str(), tNumProcsString.c_str(), num_opt_procs.c_str(), envString.c_str(),separationString.c_str());
      fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
      fprintf(fp, "%s PLATO_APP_FILE%splato_main_operations.xml \\\n", envString.c_str(),separationString.c_str());
      if(m_InputData.plato_main_path.length() != 0)
          fprintf(fp, "%s plato_main_input_deck.xml \\\n", m_InputData.plato_main_path.c_str());
      else
          fprintf(fp, "plato_main plato_main_input_deck.xml \\\n");

      fprintf(fp, ": %s %s %s PLATO_PERFORMER_ID%s1 \\\n", tNumProcsString.c_str(), Plato::to_string(m_InputData.m_UncertaintyMetaData.numPerformers).c_str(), envString.c_str(),separationString.c_str());
      fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
      fprintf(fp, "%s PLATO_APP_FILE%splato_analyze_operations.xml \\\n", envString.c_str(),separationString.c_str());
      if(m_InputData.plato_analyze_path.length() != 0)
          fprintf(fp, "%s --input-config=plato_analyze_input_deck.xml \\\n", m_InputData.plato_analyze_path.c_str());
      else
          fprintf(fp, "analyze_MPMD --input-config=plato_analyze_input_deck.xml \\\n");

      fclose(fp);
    }
    return true;
}

/******************************************************************************/
void ComplianceMinTOPlatoAnalyzeUncertInputGenerator::generateAnalyzeBashScripts()
/******************************************************************************/
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

/******************************************************************************/
void ComplianceMinTOPlatoAnalyzeUncertInputGenerator::generateJSRunScript()
/******************************************************************************/
{
  std::ofstream jsrun;
  jsrun.open("jsrun.source");
  jsrun << "1 : eng : bash engine.sh\n";
  jsrun << Plato::to_string(m_InputData.m_UncertaintyMetaData.numPerformers) << " : per : bash analyze.sh\n";
  jsrun.close();
}

/******************************************************************************/
void ComplianceMinTOPlatoAnalyzeUncertInputGenerator::generateBatchScript()
/******************************************************************************/
{
  std::ofstream batchFile;
  batchFile.open ("plato.batch");
  batchFile << "#!/bin/bash\n";
  batchFile << "# LSF Directives\n";
  batchFile << "#BSUB -P <PROJECT>\n";
  batchFile << "#BSUB -W 0:00\n";

  size_t tNumNodesNeeded = computeNumberOfNodesNeeded();

  batchFile << "#BSUB -nnodes " << tNumNodesNeeded << "\n";
  batchFile << "#BSUB -J plato\n";
  batchFile << "#BSUB -o plato.%J\n";
  batchFile << "\n";
  batchFile << "cd <path/to/working/directory>\n";
  batchFile << "date\n";
  batchFile << "jsrun -A eng -n1 -a1 -c1 -g0\n";

  batchFile << "jsrun -A per -n" << Plato::to_string(m_InputData.m_UncertaintyMetaData.numPerformers) << " -a1 -c1 -g1\n";

  batchFile << "jsrun -f jsrun.source\n";
  

  batchFile.close();
}

/******************************************************************************/
size_t ComplianceMinTOPlatoAnalyzeUncertInputGenerator::computeNumberOfNodesNeeded()
/******************************************************************************/
{
  size_t tNumGPUsNeeded;
  tNumGPUsNeeded = m_InputData.m_UncertaintyMetaData.numPerformers;
  size_t tNumGPUsPerNode = 6;
  size_t tNumNodesNeeded = tNumGPUsNeeded/tNumGPUsPerNode;
  if(tNumGPUsNeeded % tNumGPUsPerNode != 0)
    ++tNumNodesNeeded;
  return tNumNodesNeeded;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeUncertInputGenerator::generatePlatoAnalyzeInputDecks(std::ostringstream *aStringStream)
/******************************************************************************/
{
    if(!generatePlatoAnalyzeInputDeckForNewUncertaintyWorkflow())
        return false;
    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeUncertInputGenerator::generatePhysicsInputDecks()
/******************************************************************************/
{
    if(!generatePlatoAnalyzeInputDecks())
      return false;
    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeUncertInputGenerator::generatePerformerOperationsXML()
/******************************************************************************/
{
    generatePlatoAnalyzeOperationsXML();
    generateAMGXInput();
    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeUncertInputGenerator::generatePlatoAnalyzeOperationsXML()
/******************************************************************************/
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

    return true;
}

/******************************************************************************/
void ComplianceMinTOPlatoAnalyzeUncertInputGenerator::addStochasticObjectiveValueOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node1;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "MeanPlusStdDev");
    addChild(tmp_node, "Name", "Stochastic Objective Value");
    addChild(tmp_node, "Layout", "Scalar");
    pugi::xml_node for_node = tmp_node.append_child("For");
    for_node.append_attribute("var") = "performerIndex";
    for_node.append_attribute("in") = "Performers";
    for_node = for_node.append_child("For");
    for_node.append_attribute("var") = "PerformerSampleIndex";
    for_node.append_attribute("in") = "PerformerSamples";
    pugi::xml_node input_node = for_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    addChild(input_node, "Probability", "{Probabilities[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}");
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
void ComplianceMinTOPlatoAnalyzeUncertInputGenerator::addVonMisesStatisticsOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node1;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "MeanPlusStdDev");
    addChild(tmp_node, "Name", "VonMises Statistics");
    addChild(tmp_node, "Layout", "Element Field");
    pugi::xml_node for_node = tmp_node.append_child("For");
    for_node.append_attribute("var") = "performerIndex";
    for_node.append_attribute("in") = "Performers";
    for_node = for_node.append_child("For");
    for_node.append_attribute("var") = "PerformerSampleIndex";
    for_node.append_attribute("in") = "PerformerSamples";
    pugi::xml_node input_node = for_node.append_child("Input");
    addChild(input_node, "ArgumentName", "plato_analyze_{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}_vonmises");
    addChild(input_node, "Probability", "{Probabilities[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}");
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
bool ComplianceMinTOPlatoAnalyzeUncertInputGenerator::addAggregateGradientOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node2;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "Aggregator");
    addChild(tmp_node, "Name", "AggregateGradient");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Nodal Field");
    pugi::xml_node for_node = tmp_node1.append_child("For");
    for_node.append_attribute("var") = "sampleIndex";
    for_node.append_attribute("in") = "Samples";
    tmp_node2 = for_node.append_child("Input"); 
    addChild(tmp_node2, "ArgumentName", "Field {sampleIndex}");
    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Field");

    tmp_node1 = tmp_node.append_child("Weighting");
    for_node = tmp_node1.append_child("For");
    for_node.append_attribute("var") = "sampleIndex";
    for_node.append_attribute("in") = "Samples";
    tmp_node2 = for_node.append_child("Weight"); 
    addChild(tmp_node2, "Value", "1");

    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeUncertInputGenerator::addAggregateEnergyOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node2;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "Aggregator");
    addChild(tmp_node, "Name", "AggregateEnergy");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Value");

    pugi::xml_node for_node = tmp_node1.append_child("For");
    for_node.append_attribute("var") = "sampleIndex";
    for_node.append_attribute("in") = "Samples";
    tmp_node2 = for_node.append_child("Input"); 
    addChild(tmp_node2, "ArgumentName", "Value {sampleIndex}");

    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Value");

    tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Nodal Field");

    for_node = tmp_node1.append_child("For");
    for_node.append_attribute("var") = "sampleIndex";
    for_node.append_attribute("in") = "Samples";
    tmp_node2 = for_node.append_child("Input"); 
    addChild(tmp_node2, "ArgumentName", "Field {sampleIndex}");

    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Field");

    tmp_node1 = tmp_node.append_child("Weighting");

    for_node = tmp_node1.append_child("For");
    for_node.append_attribute("var") = "sampleIndex";
    for_node.append_attribute("in") = "Samples";
    tmp_node2 = for_node.append_child("Weight"); 
    addChild(tmp_node2, "Value", "1");

    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeUncertInputGenerator::addAggregateHessianOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node2;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "Aggregator");
    addChild(tmp_node, "Name", "AggregateHessian");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Nodal Field");
    pugi::xml_node for_node = tmp_node1.append_child("For");
    for_node.append_attribute("var") = "sampleIndex";
    for_node.append_attribute("in") = "Samples";
    tmp_node2 = for_node.append_child("Input"); 
    addChild(tmp_node2, "ArgumentName", "Field {sampleIndex}");
    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Field");

    tmp_node1 = tmp_node.append_child("Weighting");
    for_node = tmp_node1.append_child("For");
    for_node.append_attribute("var") = "sampleIndex";
    for_node.append_attribute("in") = "Samples";
    tmp_node2 = for_node.append_child("Weight"); 
    addChild(tmp_node2, "Value", "1");

    return true;
}

/******************************************************************************/
void ComplianceMinTOPlatoAnalyzeUncertInputGenerator::addPlatoMainOutputOperation(pugi::xml_document &aDoc,
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
    addChild(tmp_node1, "ArgumentName", "Objective Gradient");
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

    pugi::xml_node for_node = tmp_node.append_child("For");
    for_node.append_attribute("var") = "sampleIndex";
    for_node.append_attribute("in") = "Samples";

    tmp_node1 = for_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "plato_analyze_{sampleIndex}_vonmises");
    addChild(tmp_node1, "Layout", "Element Field");

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

    addStochasticObjectiveValueOperation(aDoc);
    addStochasticObjectiveGradientOperation(aDoc);
    if(aRequestedVonMises)
        addVonMisesStatisticsOperation(aDoc);
}

/******************************************************************************/
void ComplianceMinTOPlatoAnalyzeUncertInputGenerator::addStochasticObjectiveGradientOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    pugi::xml_node tmp_node1, tmp_node2;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "MeanPlusStdDevGradient");
    addChild(tmp_node, "Name", "Stochastic Objective Gradient");

    tmp_node1 = tmp_node.append_child("CriterionValue");
    addChild(tmp_node1, "Layout", "Global");
    pugi::xml_node for_node = tmp_node1.append_child("For");
    for_node.append_attribute("var") = "performerIndex";
    for_node.append_attribute("in") = "Performers";
    for_node = for_node.append_child("For");
    for_node.append_attribute("var") = "PerformerSampleIndex";
    for_node.append_attribute("in") = "PerformerSamples";
    pugi::xml_node input_node = for_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    addChild(input_node, "Probability", "{Probabilities[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}");
    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "Statistic", "mean");
    addChild(tmp_node2, "ArgumentName", "objective_mean");
    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "Statistic", "std_dev");
    addChild(tmp_node2, "ArgumentName", "objective_std_dev");

    tmp_node1 = tmp_node.append_child("CriterionGradient");
    addChild(tmp_node1, "Layout", "Nodal Field");
    for_node = tmp_node1.append_child("For");
    for_node.append_attribute("var") = "performerIndex";
    for_node.append_attribute("in") = "Performers";
    for_node = for_node.append_child("For");
    for_node.append_attribute("var") = "PerformerSampleIndex";
    for_node.append_attribute("in") = "PerformerSamples";
    input_node = for_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Objective {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex} Gradient");
    addChild(input_node, "Probability", "{Probabilities[{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}");
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
bool ComplianceMinTOPlatoAnalyzeUncertInputGenerator::outputObjectiveStage(pugi::xml_document &doc,
                                             const bool &aHasUncertainties)
/******************************************************************************/
{
    // Internal Energy
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Objective Value");

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

    addComputeObjectiveValueOperationForNewUncertaintyWorkflow(stage_node);

    // AggregateEnergy
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "AggregateEnergy");
    addChild(op_node, "PerformerName", "PlatoMain");

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

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Value");
    addChild(output_node, "SharedDataName", "Objective Value");

    // If there are uncertainties add an operation for
    // the objective mean and std deviation.
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Stochastic Objective Value");
    addChild(op_node, "PerformerName", "PlatoMain");

    for_node = op_node.append_child("For");
    for_node.append_attribute("var") = "performerIndex";
    for_node.append_attribute("in") = "Performers";
    for_node = for_node.append_child("For");
    for_node.append_attribute("var") = "PerformerSampleIndex";
    for_node.append_attribute("in") = "PerformerSamples";
    input_node = for_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    addChild(input_node, "SharedDataName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");

    output_node = op_node.append_child("Output");
    std::string tFieldName = "Objective Mean Plus ";
    tFieldName += m_InputData.objective_number_standard_deviations;
    tFieldName += " StdDev";
    addChild(output_node, "ArgumentName", tFieldName);
    addChild(output_node, "SharedDataName", "Objective Mean Plus StdDev Value");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Objective Value");

    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeUncertInputGenerator::outputObjectiveGradientStage(pugi::xml_document &doc,
                                                     const bool &aHasUncertainties)
/******************************************************************************/
{
    char tmp_buf[200];
    // Internal Energy
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Objective Gradient");

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

    addComputeObjectiveGradientOperationForNewUncertaintyWorkflow(stage_node);

    // Aggregate
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "AggregateGradient");
    addChild(op_node, "PerformerName", "PlatoMain");

    pugi::xml_node for_node = op_node.append_child("For");
    for_node.append_attribute("var") = "performerIndex";
    for_node.append_attribute("in") = "Performers";
    for_node = for_node.append_child("For");
    for_node.append_attribute("var") = "PerformerSampleIndex";
    for_node.append_attribute("in") = "PerformerSamples";

    input_node = for_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    addChild(input_node, "SharedDataName", "Objective {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex} Gradient");
      
    if(m_InputData.mUseNormalizationInAggregator == "true")
    {
        input_node = for_node.append_child("Input");
        sprintf(tmp_buf, "Normalization Factor {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
        addChild(input_node, "ArgumentName", "Normalization Factor {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
        addChild(input_node, "SharedDataName", "Initial Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    }

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Field");
    addChild(output_node, "SharedDataName", "Objective Gradient");

    // filter gradient
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterGradient");
    addChild(op_node, "PerformerName", "PlatoMain");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Optimization DOFs");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Gradient");
    addChild(input_node, "SharedDataName", "Objective Gradient");
    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Gradient");
    addChild(output_node, "SharedDataName", "Objective Gradient");

    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Stochastic Objective Gradient");
    addChild(op_node, "PerformerName", "PlatoMain");

    for_node = op_node.append_child("For");
    for_node.append_attribute("var") = "performerIndex";
    for_node.append_attribute("in") = "Performers";
    for_node = for_node.append_child("For");
    for_node.append_attribute("var") = "PerformerSampleIndex";
    for_node.append_attribute("in") = "PerformerSamples";

    input_node = for_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    addChild(input_node, "SharedDataName", "Objective Value {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    input_node = for_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Objective {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex} Gradient");
    addChild(input_node, "SharedDataName", "Objective {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex} Gradient");

    output_node = op_node.append_child("Output");
    std::string tTmpString = "Objective Mean Plus ";
    tTmpString += m_InputData.objective_number_standard_deviations;
    tTmpString += " StdDev Gradient";
    addChild(output_node, "ArgumentName", tTmpString);
    addChild(output_node, "SharedDataName", "Objective Mean Plus StdDev Gradient");

    // stage output
    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Objective Gradient");

    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeUncertInputGenerator::outputObjectiveHessianStage(pugi::xml_document &doc)
/******************************************************************************/
{
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
    
    // Aggregate
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "AggregateHessian");
    addChild(op_node, "PerformerName", "PlatoMain");
    
    for_node = op_node.append_child("For");
    for_node.append_attribute("var") = "performerIndex";
    for_node.append_attribute("in") = "Performers";
    for_node = for_node.append_child("For");
    for_node.append_attribute("var") = "PerformerSampleIndex";
    for_node.append_attribute("in") = "PerformerSamples";

    input_node = for_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    addChild(input_node, "SharedDataName", "HessianTimesVector {performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    
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

/**********************************************************************************/
void ComplianceMinTOPlatoAnalyzeUncertInputGenerator::outputCacheStateStage(pugi::xml_document &doc,
                                         const bool &aHasUncertainties)
/**********************************************************************************/
{
    pugi::xml_node op_node, output_node;
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Cache State");
}

/**********************************************************************************/
void ComplianceMinTOPlatoAnalyzeUncertInputGenerator::outputUpdateProblemStage(pugi::xml_document &doc)
/**********************************************************************************/
{
    pugi::xml_node op_node;
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Update Problem");
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Update Problem");
    addChild(op_node, "PerformerName", "PlatoMain");
}

/**********************************************************************************/
void ComplianceMinTOPlatoAnalyzeUncertInputGenerator::outputInitializeOptimizationStage(pugi::xml_document &doc)
/**********************************************************************************/
{
    outputInitializeOptimizationStageForTO(doc);
}

/**********************************************************************************/
void ComplianceMinTOPlatoAnalyzeUncertInputGenerator::outputInitializeOptimizationStageForTO(pugi::xml_document &doc)
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

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Optimization DOFs");
}

/**********************************************************************************/
void ComplianceMinTOPlatoAnalyzeUncertInputGenerator::outputOutputToFileStage(pugi::xml_document &doc,
                                           const bool &aHasUncertainties,
                                           const bool &aRequestedVonMises)
/**********************************************************************************/
{
    char tmp_buf[1000];
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Output To File");
    pugi::xml_node op_node, output_node, input_node;

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
        for(size_t j=0; j<cur_obj.output_for_plotting.size(); ++j)
        {
            if(tFirstTime)
            {
                op_node = stage_node.append_child("Operation");
                addChild(op_node, "Name", "Write Output");
                addChild(op_node, "PerformerName", cur_obj.performer_name);
                tFirstTime = false;
            }
            sprintf(tmp_buf, "%s_%s", cur_obj.performer_name.c_str(), cur_obj.output_for_plotting[j].c_str());
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

    // VonMises Statistics
    if(aHasUncertainties && aRequestedVonMises)
    {
        op_node = stage_node.append_child("Operation");
        addChild(op_node, "Name", "VonMises Statistics");
        addChild(op_node, "PerformerName", "PlatoMain");
        // We are assuming only one load case per objective/performer which
        // means we will only have one vonmises per performer.
        pugi::xml_node for_node = op_node.append_child("For");
        for_node.append_attribute("var") = "performerIndex";
        for_node.append_attribute("in") = "Performers";
        for_node = for_node.append_child("For");
        for_node.append_attribute("var") = "PerformerSampleIndex";
        for_node.append_attribute("in") = "PerformerSamples";
        input_node = for_node.append_child("Input");
        addChild(input_node, "ArgumentName", "plato_analyze_{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}_vonmises");
        addChild(input_node, "SharedDataName", "plato_analyze_{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}_vonmises");
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
    pugi::xml_node for_node = op_node.append_child("For");
    for_node.append_attribute("var") = "performerIndex";
    for_node.append_attribute("in") = "Performers";
    for_node = for_node.append_child("For");
    for_node.append_attribute("var") = "PerformerSampleIndex";
    for_node.append_attribute("in") = "PerformerSamples";
    input_node = for_node.append_child("Input");
    addChild(input_node, "ArgumentName", "plato_analyze_{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}_vonmises");
    addChild(input_node, "SharedDataName", "plato_analyze_{performerIndex*NumSamplesPerPerformer+PerformerSampleIndex}_vonmises");

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


}


