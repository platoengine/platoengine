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
 * ComplianceMinTOPlatoAnalyzeInputGenerator.cpp
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
#include "ComplianceMinTOPlatoAnalyzeInputGenerator.hpp"
#include "XMLGeneratorUtilities.hpp"
#include "Plato_SolveUncertaintyProblem.hpp"
#include "Plato_UniqueCounter.hpp"
#include "Plato_Vector3DVariations.hpp"
#include "PlatoAnalyzeInputDeckWriter.hpp"
#include "Plato_FreeFunctions.hpp"
#include "Plato_SromXMLUtils.hpp"
#include "Plato_SromXML.hpp"
#include "XMLG_Macros.hpp"

namespace XMLGen
{

/******************************************************************************/
ComplianceMinTOPlatoAnalyzeInputGenerator::ComplianceMinTOPlatoAnalyzeInputGenerator(const InputData& aInputData) :
DefaultInputGenerator(aInputData)
/******************************************************************************/
{
}

/******************************************************************************/
ComplianceMinTOPlatoAnalyzeInputGenerator::~ComplianceMinTOPlatoAnalyzeInputGenerator() 
/******************************************************************************/
{
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeInputGenerator::generateInputFiles()
/******************************************************************************/
{
    if(!generateInterfaceXML())
    {
        std::cout << "Failed to generate interface.xml" << std::endl;
        return false;
    }

    if(!generatePlatoMainOperationsXML())
    {
        std::cout << "Failed to generate plato_main_operations.xml" << std::endl;
        return false;
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

    if(!generateLaunchScript())
    {
            std::cout << "Failed to generate mpirun.source" << std::endl;
            return false;
    }

    std::cout << "Successfully wrote XML files." << std::endl;
    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeInputGenerator::generateLaunchScript()
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

        // Now add the main mpirun call.
        fprintf(fp, "%s %s %s %s PLATO_PERFORMER_ID%s0 \\\n", tLaunchString.c_str(), tNumProcsString.c_str(), num_opt_procs.c_str(), envString.c_str(),separationString.c_str());
        fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
        fprintf(fp, "%s PLATO_APP_FILE%splato_main_operations.xml \\\n", envString.c_str(),separationString.c_str());
        if(m_InputData.plato_main_path.length() != 0)
            fprintf(fp, "%s plato_main_input_deck.xml \\\n", m_InputData.plato_main_path.c_str());
        else
            fprintf(fp, "plato_main plato_main_input_deck.xml \\\n");
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
            if(m_InputData.plato_analyze_path.length() != 0)
                fprintf(fp, "%s --input-config=plato_analyze_input_deck_%s.xml \\\n", m_InputData.plato_analyze_path.c_str(), cur_obj.name.c_str());
            else
                fprintf(fp, "analyze_MPMD --input-config=plato_analyze_input_deck_%s.xml \\\n", cur_obj.name.c_str());
        }

        fclose(fp);
    }
    return true;
}

/******************************************************************************/
void ComplianceMinTOPlatoAnalyzeInputGenerator::generateAnalyzeBashScripts()
/******************************************************************************/
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

/******************************************************************************/
void ComplianceMinTOPlatoAnalyzeInputGenerator::generateJSRunScript()
/******************************************************************************/
{
    std::ofstream jsrun;
    jsrun.open("jsrun.source");
    jsrun << "1 : eng : bash engine.sh\n";
    for(size_t i = 1; i <= m_InputData.objectives.size(); ++i)
    {
        jsrun << "1 : per" << i << " : bash analyze" << i << ".sh\n";
    }
    jsrun.close();
}

/******************************************************************************/
void ComplianceMinTOPlatoAnalyzeInputGenerator::generateBatchScript()
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

    for(size_t i = 1; i <= m_InputData.objectives.size(); ++i)
    {
        batchFile << "jsrun -A per" << i << " -n1 -a1 -c1 -g1\n";
    }

    batchFile << "jsrun -f jsrun.source\n";
  
    batchFile.close();
}

/******************************************************************************/
size_t ComplianceMinTOPlatoAnalyzeInputGenerator::computeNumberOfNodesNeeded()
/******************************************************************************/
{
    size_t tNumGPUsNeeded;
    tNumGPUsNeeded = m_InputData.objectives.size();
    size_t tNumGPUsPerNode = 6;
    size_t tNumNodesNeeded = tNumGPUsNeeded/tNumGPUsPerNode;
    if(tNumGPUsNeeded % tNumGPUsPerNode != 0)
        ++tNumNodesNeeded;
    return tNumNodesNeeded;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeInputGenerator::generatePlatoAnalyzeInputDecks(std::ostringstream *aStringStream)
/******************************************************************************/
{
    PlatoAnalyzeInputDeckWriter tInputDeckWriter(m_InputData);
    return tInputDeckWriter.generate(aStringStream);
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeInputGenerator::generatePhysicsInputDecks()
/******************************************************************************/
{
    if(!generatePlatoAnalyzeInputDecks())
      return false;
    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeInputGenerator::generatePerformerOperationsXML()
/******************************************************************************/
{
    generatePlatoAnalyzeOperationsXML();
    generateAMGXInput();
    generateROLInput();
    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeInputGenerator::generatePlatoAnalyzeOperationsXML()
/******************************************************************************/
{
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        Objective tCurObjective = m_InputData.objectives[i];

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
            else if(tCurObjective.output_for_plotting[j] == "vonmises")
                addChild(tmp_node1, "ArgumentName", "Vonmises");
        }
  
        char buf[200];
        sprintf(buf, "plato_analyze_operations_%s.xml", m_InputData.objectives[i].name.c_str());

        // Write the file to disk
        doc.save_file(buf, "  ");
    }
    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeInputGenerator::addAggregateGradientOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    char tBuffer[100];
    pugi::xml_node tmp_node2;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "Aggregator");
    addChild(tmp_node, "Name", "AggregateGradient");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Nodal Field");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        tmp_node2 = tmp_node1.append_child("Input");
        std::string tTmpString = "Field ";
        tTmpString += Plato::to_string(i+1);
        addChild(tmp_node2, "ArgumentName", tTmpString);
    }
    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Field");

    tmp_node1 = tmp_node.append_child("Weighting");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        tmp_node2 = tmp_node1.append_child("Weight");
        addChild(tmp_node2, "Value", m_InputData.objectives[i].weight.c_str());
    }
    if(m_InputData.mUseNormalizationInAggregator == "true")
    {
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
bool ComplianceMinTOPlatoAnalyzeInputGenerator::addAggregateEnergyOperation(pugi::xml_document &aDoc)
/******************************************************************************/
{
    char tBuffer[100];
    pugi::xml_node tmp_node2;
    pugi::xml_node tmp_node = aDoc.append_child("Operation");
    addChild(tmp_node, "Function", "Aggregator");
    addChild(tmp_node, "Name", "AggregateEnergy");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Value");

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        tmp_node2 = tmp_node1.append_child("Input");
        std::string tTmpString = "Value ";
        tTmpString += Plato::to_string(i+1);
        addChild(tmp_node2, "ArgumentName", tTmpString);
    }

    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Value");

    tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Nodal Field");

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        tmp_node2 = tmp_node1.append_child("Input");
        std::string tTmpString = "Field ";
        tTmpString += Plato::to_string(i+1);
        addChild(tmp_node2, "ArgumentName", tTmpString);
    }

    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Field");

    tmp_node1 = tmp_node.append_child("Weighting");

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
          tmp_node2 = tmp_node1.append_child("Weight");
          addChild(tmp_node2, "Value", m_InputData.objectives[i].weight.c_str());
    }

    if(m_InputData.mUseNormalizationInAggregator == "true")
    {
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
void ComplianceMinTOPlatoAnalyzeInputGenerator::addInitializeFieldOperation(pugi::xml_document &aDoc)
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
        addChild(tmp_node, "Method", "Uniform");
        tmp_node1 = tmp_node.append_child("Uniform");
        addChild(tmp_node1, "Value", m_InputData.initial_density_value);
    }
}

/******************************************************************************/
void ComplianceMinTOPlatoAnalyzeInputGenerator::addPlatoMainOutputOperation(pugi::xml_document &aDoc,
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
    }

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        XMLGen::Objective cur_obj = m_InputData.objectives[i];
        for(size_t j=0; j<cur_obj.output_for_plotting.size(); j++)
        {
            tmp_node1 = tmp_node.append_child("Input");
            addChild(tmp_node1, "ArgumentName", cur_obj.performer_name + "_" + cur_obj.output_for_plotting[j]);
            if(cur_obj.output_for_plotting[j] == "vonmises" ||
                cur_obj.output_for_plotting[j] == "Vonmises")
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
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeInputGenerator::generatePlatoMainOperationsXML()
/******************************************************************************/
{
    pugi::xml_document doc;
    pugi::xml_node tmp_node, tmp_node1, tmp_node2;

    // Version entry
    tmp_node = doc.append_child(pugi::node_declaration);
    tmp_node.set_name("xml");
    pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
    tmp_att.set_value("1.0");

    //////////////////////////////////////////////////
    // Operations
    /////////////////////////////////////////////////

    addFilterInfo(doc);
    addPlatoMainOutputOperation(doc, m_InputData.m_HasUncertainties, m_InputData.m_RequestedVonMisesOutput);
    addUpdateProblemOperation(doc);
    addFilterControlOperation(doc);
    addFilterGradientOperation(doc);
    addInitializeFieldOperation(doc);
    addDesignVolumeOperation(doc);
    addComputeVolumeOperation(doc);
    if(!addAggregateEnergyOperation(doc))
        return false;
    if(!addAggregateGradientOperation(doc))
        return false;
    addSetLowerBoundsOperation(doc);
    addSetUpperBoundsOperation(doc);

    // Write the file to disk
    doc.save_file("plato_main_operations.xml", "  ");

    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeInputGenerator::outputObjectiveStage(pugi::xml_document &doc,
                                             const bool &aHasUncertainties)
/******************************************************************************/
{
    char tmp_buf[200];
    // Internal Energy
    pugi::xml_node stage_node = doc.append_child("Stage");
    pugi::xml_node cur_parent = stage_node;
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

    if(m_InputData.objectives.size() > 1)
    {
        op_node = stage_node.append_child("Operation");
        cur_parent = op_node;
    }

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        XMLGen::Objective cur_obj = m_InputData.objectives[i];
        op_node = cur_parent.append_child("Operation");
        addChild(op_node, "Name", "Compute Objective Value");
        addChild(op_node, "PerformerName", cur_obj.performer_name.c_str());
        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Topology");
        addChild(input_node, "SharedDataName", "Topology");
        output_node = op_node.append_child("Output");
        addChild(output_node, "ArgumentName", "Objective Value");
        sprintf(tmp_buf, "Objective Value %d", (int)(i+1));
        addChild(output_node, "SharedDataName", tmp_buf);
    }

    // AggregateEnergy
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "AggregateEnergy");
    addChild(op_node, "PerformerName", "PlatoMain");

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        input_node = op_node.append_child("Input");
        sprintf(tmp_buf, "Value %d", (int)(i+1));
        addChild(input_node, "ArgumentName", tmp_buf);
        sprintf(tmp_buf, "Objective Value %d", (int)(i+1));
        addChild(input_node, "SharedDataName", tmp_buf);
        if(m_InputData.mUseNormalizationInAggregator == "true")
        {
            input_node = op_node.append_child("Input");
            sprintf(tmp_buf, "Normalization Factor %d", (int)(i+1));
            addChild(input_node, "ArgumentName", tmp_buf);
            sprintf(tmp_buf, "Initial Objective Value %d", (int)(i+1));
            addChild(input_node, "SharedDataName", tmp_buf);
        }
    }

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Value");
    addChild(output_node, "SharedDataName", "Objective Value");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Objective Value");

    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeInputGenerator::outputObjectiveGradientStage(pugi::xml_document &doc,
                                                     const bool &aHasUncertainties)
/******************************************************************************/
{
    char tmp_buf[200];
    // Internal Energy
    pugi::xml_node stage_node = doc.append_child("Stage");
    pugi::xml_node cur_parent = stage_node;
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

    if(m_InputData.objectives.size() > 1)
    {
        op_node = stage_node.append_child("Operation");
        cur_parent = op_node;
    }

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        XMLGen::Objective cur_obj = m_InputData.objectives[i];
        op_node = cur_parent.append_child("Operation");
        addChild(op_node, "Name", "Compute Objective Gradient");
        addChild(op_node, "PerformerName", cur_obj.performer_name.c_str());
        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Topology");
        addChild(input_node, "SharedDataName", "Topology");
        output_node = op_node.append_child("Output");
        addChild(output_node, "ArgumentName", "Objective Gradient");
        sprintf(tmp_buf, "Objective %d Gradient", (int)(i+1));
        addChild(output_node, "SharedDataName", tmp_buf);
    }
    

    // Aggregate
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "AggregateGradient");
    addChild(op_node, "PerformerName", "PlatoMain");

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        input_node = op_node.append_child("Input");
        sprintf(tmp_buf, "Field %d", (int)(i+1));
        addChild(input_node, "ArgumentName", tmp_buf);
        sprintf(tmp_buf, "Objective %d Gradient", (int)(i+1));
        addChild(input_node, "SharedDataName", tmp_buf);
        if(m_InputData.mUseNormalizationInAggregator == "true")
        {
          input_node = op_node.append_child("Input");
          sprintf(tmp_buf, "Normalization Factor %d", (int)(i+1));
          addChild(input_node, "ArgumentName", tmp_buf);
          sprintf(tmp_buf, "Initial Objective Value %d", (int)(i+1));
          addChild(input_node, "SharedDataName", tmp_buf);
        }
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

    // stage output
    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Objective Gradient");

    return true;
}

/******************************************************************************/
bool ComplianceMinTOPlatoAnalyzeInputGenerator::generateInterfaceXML(std::ostringstream *aStringStream)
/******************************************************************************/
{
    pugi::xml_document doc;
    pugi::xml_node tTmpNode;

    // Console output control
    tTmpNode = doc.append_child("Console");
    addChild(tTmpNode, "Verbose", "true");
    addChild(tTmpNode, "Enabled", "true");

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

    // Other performers (one for each objective)
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        performer_node = doc.append_child("Performer");
        addChild(performer_node, "Name", m_InputData.objectives[i].performer_name);
        addChild(performer_node, "Code", m_InputData.objectives[i].code_name);
        sprintf(tmp_buf, "%d", (int)(i+1));
        addChild(performer_node, "PerformerID", tmp_buf);
    }

    //////////////////////////////////////////////////
    // Shared Data
    /////////////////////////////////////////////////

    // Performer Objectives 
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        // create shared data for objectives
        sprintf(tmp_buf, "Objective Value %d", (int)(i+1));
        createSingleUserGlobalSharedData(doc, tmp_buf, "Scalar", "1", m_InputData.objectives[i].performer_name, "PlatoMain");

        if(m_InputData.mUseNormalizationInAggregator == "true")
        {
            sprintf(tmp_buf, "Initial Objective Value %d", (int)(i+1));
            createSingleUserGlobalSharedData(doc, tmp_buf, "Scalar", "1", m_InputData.objectives[i].performer_name, "PlatoMain");
        }
    }

    // Performer Objective Gradients
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        sprintf(tmp_buf, "Objective %d Gradient", (int)(i+1));
        createSingleUserNodalSharedData(doc, tmp_buf, "Scalar", m_InputData.objectives[i].performer_name, "PlatoMain");
    }

    // Aggregated Objective
    createSingleUserGlobalSharedData(doc, "Objective Value", "Scalar", "1", "PlatoMain", "PlatoMain");

    // Aggregated Objective Gradient
    createSingleUserNodalSharedData(doc, "Objective Gradient", "Scalar", "PlatoMain", "PlatoMain");

    // Quantities of Interest
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        XMLGen::Objective cur_obj = m_InputData.objectives[i];
        for(size_t j=0; j<cur_obj.output_for_plotting.size(); ++j)
        {
            // create shared data for objectives
            sprintf(tmp_buf, "%s_%s", cur_obj.performer_name.c_str(), cur_obj.output_for_plotting[j].c_str());
            if(cur_obj.output_for_plotting[j] == "vonmises")
                createSingleUserElementSharedData(doc, tmp_buf, "Scalar", cur_obj.performer_name, "PlatoMain");
            else
                createSingleUserNodalSharedData(doc, tmp_buf, "Scalar", cur_obj.performer_name, "PlatoMain");
        }
    }

    // Optimization DOFs
    sd_node = createSingleUserNodalSharedData(doc, "Optimization DOFs", "Scalar", "PlatoMain", "PlatoMain");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        addChild(sd_node, "UserName", m_InputData.objectives[i].performer_name);

    // Topology
    sd_node = createSingleUserNodalSharedData(doc, "Topology", "Scalar", "PlatoMain", "PlatoMain");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        addChild(sd_node, "UserName", m_InputData.objectives[i].performer_name);

    // Volume
    createSingleUserGlobalSharedData(doc, "Volume", "Scalar", "1", "PlatoMain", "PlatoMain");

    // Volume Gradient
    createSingleUserNodalSharedData(doc, "Volume Gradient", "Scalar", "PlatoMain", "PlatoMain");

    // Design Volume
    createSingleUserGlobalSharedData(doc, "Design Volume", "Scalar", "1", "PlatoMain", "PlatoMain");

    // Lower Bound Value
    createSingleUserGlobalSharedData(doc, "Lower Bound Value", "Scalar", "1", "PlatoMain", "PlatoMain");

    // Upper Bound Value
    createSingleUserGlobalSharedData(doc, "Upper Bound Value", "Scalar", "1", "PlatoMain", "PlatoMain");

    // Lower Bound Vector
    createSingleUserNodalSharedData(doc, "Lower Bound Vector", "Scalar", "PlatoMain", "PlatoMain");

    // Upper Bound Vector
    createSingleUserNodalSharedData(doc, "Upper Bound Vector", "Scalar", "PlatoMain", "PlatoMain");
    
    //////////////////////////////////////////////////
    // Stages
    /////////////////////////////////////////////////

    // Output To File
    outputOutputToFileStage(doc, m_InputData.m_HasUncertainties, m_InputData.m_RequestedVonMisesOutput);

    // Initialize Optimization
    outputInitializeOptimizationStage(doc);

    // Update Problem
    outputUpdateProblemStage(doc);

    // Cache State Stage
    outputCacheStateStage(doc, m_InputData.m_HasUncertainties);

    // Set Lower Bounds Stage
    outputSetLowerBoundsStage(doc);

    // Set Upper Bounds
    outputSetUpperBoundsStage(doc);

    // Design Volume
    outputDesignVolumeStage(doc);

    // Volume
    outputVolumeStage(doc);

    // Volume Gradient
    outputVolumeGradientStage(doc);

    // Objective
    outputObjectiveStage(doc, m_InputData.m_HasUncertainties);

    // Objective Gradient
    outputObjectiveGradientStage(doc, m_InputData.m_HasUncertainties);

    /////////////////////////////////////////////////
    // Misc.
    ////////////////////////////////////////////////

    pugi::xml_node tMiscNode = doc.append_child("Optimizer");
    this->setOptimizerMethod(tMiscNode);
    this->setOptimalityCriteriaOptions(tMiscNode);
    tTmpNode = tMiscNode.append_child("Options");
    this->setGCMMAoptions(tTmpNode);
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

    tTmpNode = tMiscNode.append_child("UpdateProblemStage");
    addChild(tTmpNode, "Name", "Update Problem");

    tTmpNode = tMiscNode.append_child("OptimizationVariables");
    addChild(tTmpNode, "ValueName", "Optimization DOFs");
    addChild(tTmpNode, "InitializationStage", "Initialize Optimization");
    addChild(tTmpNode, "FilteredName", "Topology");
    addChild(tTmpNode, "LowerBoundValueName", "Lower Bound Value");
    addChild(tTmpNode, "LowerBoundVectorName", "Lower Bound Vector");
    addChild(tTmpNode, "UpperBoundValueName", "Upper Bound Value");
    addChild(tTmpNode, "UpperBoundVectorName", "Upper Bound Vector");
    addChild(tTmpNode, "SetLowerBoundsStage", "Set Lower Bounds");
    addChild(tTmpNode, "SetUpperBoundsStage", "Set Upper Bounds");

    tTmpNode = tMiscNode.append_child("Objective");
    addChild(tTmpNode, "ValueName", "Objective Value");
    addChild(tTmpNode, "ValueStageName", "Objective Value");
    addChild(tTmpNode, "GradientName", "Objective Gradient");
    addChild(tTmpNode, "GradientStageName", "Objective Gradient");

    tTmpNode = tMiscNode.append_child("BoundConstraint");
    addChild(tTmpNode, "Upper", "1.0");
    addChild(tTmpNode, "Lower", "0.0");

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
            if(m_InputData.constraints[b].volume_fraction != "")
                addChild(tTmpNode, "NormalizedTargetValue", m_InputData.constraints[b].volume_fraction);
            if(m_InputData.constraints[b].volume_absolute != "")
                addChild(tTmpNode, "AbsoluteTargetValue", m_InputData.constraints[b].volume_absolute);
            addChild(tTmpNode, "Tolerance", "1e-3");
        }
    }

    if(m_InputData.optimization_algorithm != "mma")
    {
        tTmpNode = tMiscNode.append_child("Convergence");
        addChild(tTmpNode, "MaxIterations", m_InputData.max_iterations);
    }

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

    // Write the file or string
    if(aStringStream)
        doc.save(*aStringStream, "\t", pugi::format_default & ~pugi::format_indent);
    else
        doc.save_file("interface.xml", "  ");

    return true;
}

/**********************************************************************************/
void ComplianceMinTOPlatoAnalyzeInputGenerator::outputCacheStateStage(pugi::xml_document &doc,
                                         const bool &aHasUncertainties)
/**********************************************************************************/
{
    pugi::xml_node op_node, output_node;
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Cache State");
}

/**********************************************************************************/
void ComplianceMinTOPlatoAnalyzeInputGenerator::outputUpdateProblemStage(pugi::xml_document &doc)
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
void ComplianceMinTOPlatoAnalyzeInputGenerator::outputInitializeOptimizationStageForTO(pugi::xml_document &doc)
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

          for(size_t i=0; i<m_InputData.objectives.size(); ++i)
          {
            XMLGen::Objective cur_obj = m_InputData.objectives[i];
            op_node = stage_node.append_child("Operation");
            addChild(op_node, "Name", "Compute Objective Value");
            addChild(op_node, "PerformerName", cur_obj.performer_name.c_str());

            input_node = op_node.append_child("Input");
            addChild(input_node, "ArgumentName", "Topology");
            addChild(input_node, "SharedDataName", "Topology");

            output_node = op_node.append_child("Output");
            addChild(output_node, "ArgumentName", "Objective Value");
            char tBuffer[800];
            sprintf(tBuffer, "Initial Objective Value %d", (int)(i+1));
            addChild(output_node, "SharedDataName", tBuffer);
          }
    }

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Optimization DOFs");
}

/**********************************************************************************/
void ComplianceMinTOPlatoAnalyzeInputGenerator::outputOutputToFileStage(pugi::xml_document &doc,
                                           const bool &aHasUncertainties,
                                           const bool &aRequestedVonMises)
/**********************************************************************************/
{
    char tmp_buf[1000];
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Output To File");
    pugi::xml_node op_node, output_node, input_node;

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
            else if(cur_obj.output_for_plotting[j] == "vonmises")
                addChild(output_node, "ArgumentName", "Vonmises");
        }
    }

    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "PlatoMainOutput");
    addChild(op_node, "PerformerName", "PlatoMain");

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Objective Gradient");
    addChild(input_node, "SharedDataName", "Objective Gradient");
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
    }
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        XMLGen::Objective cur_obj = m_InputData.objectives[i];
        for(size_t j=0; j<cur_obj.output_for_plotting.size(); j++)
        {
            input_node = op_node.append_child("Input");
            sprintf(tmp_buf, "%s_%s", cur_obj.performer_name.c_str(), cur_obj.output_for_plotting[j].c_str());
            addChild(input_node, "ArgumentName", tmp_buf);
            addChild(input_node, "SharedDataName", tmp_buf);
        }
    }
}


}


