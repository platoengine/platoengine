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
 * XMLGenerator.cpp
 *
 *  Created on: Sep 28, 2017
 *
 */

#include <set>
#include <string>
#include <iostream>
#include <iostream>
#include <fstream>
#include <cstring>
#include<cassert>
#include "XMLGenerator.hpp"

const int MAX_CHARS_PER_LINE = 10000;
const int MAX_TOKENS_PER_LINE = 5000;
const char* const DELIMITER = " \t";


/******************************************************************************/
XMLGenerator::XMLGenerator(const std::string &input_filename) 
/******************************************************************************/
{
    m_InputFilename = input_filename;
}

/******************************************************************************/
XMLGenerator::~XMLGenerator() 
/******************************************************************************/
{
}

/******************************************************************************/
bool XMLGenerator::generate()
/******************************************************************************/
{
    if(!parseFile())
    {
        std::cout << "Failed to parse input file." << std::endl;
        return false;
    }

    if(!generateInterfaceXML())
    {
        std::cout << "Failed to generate interface.xml" << std::endl;
        return false;
    }

    if(!generatePlatoOperationsXML())
    {
        std::cout << "Failed to generate plato_operations.xml" << std::endl;
        return false;
    }

    if(!generatePlatoMainInputDeckXML())
    {
        std::cout << "Failed to generate platomain.xml" << std::endl;
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
bool XMLGenerator::generateLaunchScript()
/******************************************************************************/
{
    FILE *fp=fopen("mpirun.source", "w");
    if(fp)
    {
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
                fprintf(fp, "decomp -p %d %s.gen\n", tNumberPruneAndRefineProcs, m_InputData.mesh_name.c_str());
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

            std::string tCommand = "mpiexec -np " + tNumberPruneAndRefineProcsString + " " + m_InputData.prune_and_refine_path;
            if(m_InputData.initial_guess_filename != "")
                tCommand += (" --mesh_with_variable=" + m_InputData.initial_guess_filename);
            tCommand += (" --mesh_to_be_pruned=" + m_InputData.mesh_name + ".gen");
            tCommand += (" --result_mesh=" + m_InputData.mesh_name + ".gen");
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
                fprintf(fp, "epu -auto %s.gen%s\n", m_InputData.mesh_name.c_str(), tExtensionString.c_str());
            }
        }

        // Now do the decomps for the TO run.
        if(num_opt_procs.compare("1") != 0)
            fprintf(fp, "decomp -p %s %s.gen\n", num_opt_procs.c_str(), m_InputData.mesh_name.c_str());
        if(m_InputData.initial_guess_filename != "" && num_opt_procs.compare("1") != 0)
            fprintf(fp, "decomp -p %s %s\n", num_opt_procs.c_str(), m_InputData.initial_guess_filename.c_str());
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
            std::string num_procs = "4";
            if(!m_InputData.objectives[i].num_procs.empty())
                num_procs = m_InputData.objectives[i].num_procs;
            if(num_procs.compare("1") != 0)
            {
                fprintf(fp, "decomp -p %s %s.gen\n", num_procs.c_str(), m_InputData.mesh_name.c_str());
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

        // Now add the main mpirun call.
        fprintf(fp, "mpiexec -np %s %s PLATO_COMM_ID%s0 \\\n", num_opt_procs.c_str(), envString.c_str(),separationString.c_str());
        fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
        fprintf(fp, "%s PLATO_APP_FILE%splato_operations.xml \\\n", envString.c_str(),separationString.c_str());
        if(m_InputData.plato_main_path.length() != 0)
            fprintf(fp, "%s platomain.xml \\\n", m_InputData.plato_main_path.c_str());
        else
            fprintf(fp, "/home/bwclark/dev/plato_engine/DEBUG/apps/services/PlatoMain platomain.xml \\\n");
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
            const Objective& cur_obj = m_InputData.objectives[i];
            if(!cur_obj.num_procs.empty())
                fprintf(fp, ": -np %s %s PLATO_COMM_ID%s%d \\\n", cur_obj.num_procs.c_str(), envString.c_str(),separationString.c_str(), (int)(i+1));
            else
                fprintf(fp, ": -np 4 %s PLATO_COMM_ID%s%d \\\n",  envString.c_str(),separationString.c_str(),(int)(i+1));
            fprintf(fp, "%s PLATO_INTERFACE_FILE%sinterface.xml \\\n", envString.c_str(),separationString.c_str());
            fprintf(fp, "%s PLATO_APP_FILE%s%s_operations_%s.xml \\\n", envString.c_str(),separationString.c_str(),cur_obj.code_name.c_str(),
                    cur_obj.name.c_str());
            if(!cur_obj.code_name.compare("salinas"))
            {
                if(m_InputData.salinas_path.length() != 0)
                    fprintf(fp, "%s salinas_input_deck_%s.i \\\n", m_InputData.salinas_path.c_str(), cur_obj.name.c_str());
                else
                    fprintf(fp, "/projects/plato/dev/plato_sd/plato_sd_main salinas_input_deck_%s.i \\\n", cur_obj.name.c_str());
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
            }
        }
        fclose(fp);
    }
    return true;
}

/******************************************************************************/
bool XMLGenerator::generateSalinasInputDecks()
/******************************************************************************/
{
    bool levelset = false;
    if(m_InputData.discretization.compare("levelset") == 0)
        levelset = true;
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        const Objective& cur_obj = m_InputData.objectives[i];
        if(!cur_obj.code_name.compare("salinas"))
        {
            bool frf = false;
            if(cur_obj.type.compare("match frf data") == 0)
                frf = true;
            char buf[200];
            sprintf(buf, "salinas_input_deck_%s.i", cur_obj.name.c_str());
            FILE *fp=fopen(buf, "w");
            if(fp)
            {
                fprintf(fp, "SOLUTION\n");
                fprintf(fp, "  case '%s'\n", cur_obj.name.c_str());
                fprintf(fp, "  topology_optimization\n");
                if(cur_obj.salinas_gdsw_tolerance.length() > 0)
                {
                    fprintf(fp, "  solver gdsw\n");
                }
                fprintf(fp, "END\n");
                if(cur_obj.salinas_wtmass_scale_factor.length() > 0)
                {
                    fprintf(fp, "PARAMETERS\n");
                    fprintf(fp, "  WTMASS = %s\n", cur_obj.salinas_wtmass_scale_factor.c_str());
                    fprintf(fp, "END\n");
                }
                if(frf)
                {
                    fprintf(fp, "INVERSE-PROBLEM\n");
                    if(levelset)
                    {
                        std::string tTruthTableFile = "dummy_ttable_";
                        tTruthTableFile += cur_obj.name;
                        tTruthTableFile += ".txt";
                        std::string tRealDataFile = "dummy_data_";
                        tRealDataFile += cur_obj.name;
                        tRealDataFile += ".txt";
                        std::string tImagDataFile = "dummy_data_im_";
                        tImagDataFile += cur_obj.name;
                        tImagDataFile += ".txt";
                        // For levelset frf we will be generating the experimental data
                        // files at each iteration corresponding to the new computational
                        // mesh node ids.  Therefore, we will start things off with 3
                        // dummy files that just have generic data in them.
                        fprintf(fp, "  data_truth_table %s\n", tTruthTableFile.c_str());
                        fprintf(fp, "  real_data_file %s\n", tRealDataFile.c_str());
                        fprintf(fp, "  imaginary_data_file %s\n", tImagDataFile.c_str());
                        // Create the 3 generic files.
                        double tFreqMin, tFreqMax, tFreqStep;
                        sscanf(cur_obj.freq_min.c_str(), "%lf", &tFreqMin);
                        sscanf(cur_obj.freq_max.c_str(), "%lf", &tFreqMax);
                        sscanf(cur_obj.freq_step.c_str(), "%lf", &tFreqStep);
                        // This is the formula salinas uses to get the number of frequencies
                        int tNumFreqs = (int)(((tFreqMax-tFreqMin)/tFreqStep)+0.5) + 1;
                        int tNumMatchNodes = cur_obj.frf_match_nodesets.size();
                        FILE *tTmpFP = fopen(tTruthTableFile.c_str(), "w");
                        if(tTmpFP)
                        {
                            fprintf(tTmpFP, "%d\n", tNumMatchNodes);
                            for(int tIndex=0; tIndex<tNumMatchNodes; ++tIndex)
                            {
                                fprintf(tTmpFP, "%d 1 1 1\n", tIndex+1);
                            }
                            fclose(tTmpFP);
                        }
                        tTmpFP = fopen(tRealDataFile.c_str(), "w");
                        if(tTmpFP)
                        {
                            fprintf(tTmpFP, "%d %d\n", 3*tNumMatchNodes, tNumFreqs);
                            for(int tIndex=0; tIndex<3*tNumMatchNodes; ++tIndex)
                            {
                                for(int tIndex2=0; tIndex2<tNumFreqs; ++tIndex2)
                                {
                                    fprintf(tTmpFP, "0 ");
                                }
                                fprintf(tTmpFP, "\n");
                            }
                            fclose(tTmpFP);
                        }
                        tTmpFP = fopen(tImagDataFile.c_str(), "w");
                        if(tTmpFP)
                        {
                            fprintf(tTmpFP, "%d %d\n", 3*tNumMatchNodes, tNumFreqs);
                            for(int tIndex=0; tIndex<3*tNumMatchNodes; ++tIndex)
                            {
                                for(int tIndex2=0; tIndex2<tNumFreqs; ++tIndex2)
                                {
                                    fprintf(tTmpFP, "0 ");
                                }
                                fprintf(tTmpFP, "\n");
                            }
                            fclose(tTmpFP);
                        }
                    }
                    else
                    {
                        fprintf(fp, "  data_truth_table %s\n", "ttable.txt");
                        fprintf(fp, "  real_data_file %s\n", "data.txt");
                        fprintf(fp, "  imaginary_data_file %s\n", "data_im.txt");
                    }
                    fprintf(fp, "END\n");
                    fprintf(fp, "OPTIMIZATION\n");
                    fprintf(fp, "  optimization_package ROL_lib\n");
                    fprintf(fp, "  ROLmethod linesearch\n");
                    fprintf(fp, "  LSstep Newton-Krylov\n");
                    fprintf(fp, "  LS_curvature_condition null\n");
                    fprintf(fp, "  Max_iter_Krylov 50\n");
                    fprintf(fp, "  Use_FD_hessvec false\n");
                    fprintf(fp, "  Use_inexact_hessvec false\n");
                    fprintf(fp, "END\n");
                    if(cur_obj.raleigh_damping_alpha.length() > 0 &&
                       cur_obj.raleigh_damping_beta.length() > 0)
                    {
                        fprintf(fp, "DAMPING\n");
                        fprintf(fp, "  alpha %s\n", cur_obj.raleigh_damping_alpha.c_str());
                        fprintf(fp, "  beta %s\n", cur_obj.raleigh_damping_beta.c_str());
                        fprintf(fp, "END\n");
                    }
                    fprintf(fp, "FREQUENCY\n");
                    fprintf(fp, "  freq_min %s\n", cur_obj.freq_min.c_str());
                    fprintf(fp, "  freq_max %s\n", cur_obj.freq_max.c_str());
                    fprintf(fp, "  freq_step %s\n", cur_obj.freq_step.c_str());
//                    fprintf(fp, "  disp\n");
//                    fprintf(fp, "  block ");
                    /*
                    for(size_t n=0; n<m_InputData.blocks.size(); ++n)
                    {
                        fprintf(fp, "%s", m_InputData.blocks[n].block_id.c_str());
                        if(n < (m_InputData.blocks.size()-1))
                        {
                            fprintf(fp, ",");
                        }
                    }
                    fprintf(fp, "\n");
                    */
                    fprintf(fp, "END\n");
                    fprintf(fp, "FUNCTION 1\n");
                    fprintf(fp, "  type linear\n");
                    fprintf(fp, "  data 0 1\n");
                    fprintf(fp, "  data 1e6 1\n");
                    fprintf(fp, "END\n");
                }
                if(cur_obj.salinas_gdsw_tolerance.length() > 0)
                {
                    fprintf(fp, "GDSW\n");
                 //   fprintf(fp, "  diag_scaling diagonal\n");
                    fprintf(fp, "  solver_tol = %s\n", cur_obj.salinas_gdsw_tolerance.c_str());
                 //   fprintf(fp, "  krylov_method = GMRESClassic\n");
                 //   fprintf(fp, "  orthog=0\n");
                    fprintf(fp, "END\n");
                }
                fprintf(fp, "OUTPUTS\n");
                if(!frf)
                    fprintf(fp, "  topology\n");
                fprintf(fp, "END\n");
                fprintf(fp, "ECHO\n");
                if(!frf)
                    fprintf(fp, "  topology\n");
                fprintf(fp, "END\n");
                for(size_t n=0; n<m_InputData.materials.size(); n++)
                {
                    fprintf(fp, "MATERIAL %s\n", m_InputData.materials[n].material_id.c_str());
                    fprintf(fp, "  isotropic\n");
                    fprintf(fp, "  E = %s\n", m_InputData.materials[n].youngs_modulus.c_str());
                    fprintf(fp, "  nu = %s\n", m_InputData.materials[n].poissons_ratio.c_str());
                    if(m_InputData.materials[n].density != "")
                        fprintf(fp, "  density = %s\n", m_InputData.materials[n].density.c_str());
                    fprintf(fp, "  material_penalty_model = simp\n");
                    if(m_InputData.discretization.compare("density") == 0)
                    {
                        if(m_InputData.materials[n].penalty_exponent.length() > 0)
                            fprintf(fp, "  penalty_coefficient = %s\n", m_InputData.materials[n].penalty_exponent.c_str());
                    }
                    if(frf)
                    {
                        fprintf(fp, "  minimum_stiffness_penalty_value=1e-3\n");
                    }
                    fprintf(fp, "END\n");
                }
                for(size_t n=0; n<m_InputData.blocks.size(); ++n)
                {
                    if(m_InputData.blocks[n].block_id.empty() == false)
                    {
                        fprintf(fp, "BLOCK %s\n", m_InputData.blocks[n].block_id.c_str());
                    }
                    if(m_InputData.blocks[n].material_id.empty() == false)
                    {
                        fprintf(fp, "  material %s\n", m_InputData.blocks[n].material_id.c_str());
                    }
                    if(m_InputData.blocks[n].element_type.empty() == false)
                    {
                        // For now we will just let salinas determine the element type based
                        // on what is in the exodus file (unless it is rbar).
                        if(m_InputData.blocks[n].element_type == "rbar")
                            fprintf(fp, "  rbar\n");
                    }
                    if(frf)
                    {
                        fprintf(fp, "  inverse_material_type homogeneous\n");
                    }
                    fprintf(fp, "END\n");
                }
                fprintf(fp, "TOPOLOGY-OPTIMIZATION\n");
                fprintf(fp, "  algorithm = plato_engine\n");
                if(frf)
                {
                    fprintf(fp, "  case = inverse_methods\n");
                    if(m_InputData.discretization == "density")
                        fprintf(fp, "  inverse_method_objective = directfrf-plato-density-method\n");
                    else if(m_InputData.discretization == "levelset")
                        fprintf(fp, "  inverse_method_objective = directfrf-plato-levelset-method\n");
                    fprintf(fp, "  ref_frf_file %s\n", cur_obj.ref_frf_file.c_str());
                    if(cur_obj.frf_match_nodesets.size() > 0)
                    {
                        fprintf(fp, "  frf_nodesets");
                        for(size_t b=0; b<cur_obj.frf_match_nodesets.size(); ++b)
                        {
                            fprintf(fp, " %s", cur_obj.frf_match_nodesets[b].c_str());
                        }
                        fprintf(fp, "\n");
                    }
                    if(cur_obj.complex_error_measure.length() > 0)
                        fprintf(fp, "  complex_error_measure %s\n", cur_obj.complex_error_measure.c_str());

                    if (cur_obj.convert_to_tet10.length()>0)
                        fprintf(fp, " ls_tet_mesh_type %s\n", cur_obj.convert_to_tet10.c_str());
                }
                else
                {
                    fprintf(fp, "  case = compliance_min\n");
                }
                if(m_InputData.constraints[0].type == "volume")
                    fprintf(fp, "  volume_fraction = %s\n", m_InputData.constraints[0].volume_fraction.c_str());
                else if(m_InputData.constraints[0].type == "surface area")
                {
                    fprintf(fp, "  surface_area_constraint_value = %s\n", m_InputData.constraints[0].surface_area.c_str());
                    fprintf(fp, "  surface_area_ssid = %s\n", m_InputData.constraints[0].surface_area_ssid.c_str());
                }
                fprintf(fp, "END\n");
                fprintf(fp, "FILE\n");
                fprintf(fp, "  geometry_file '%s.gen'\n", m_InputData.mesh_name.c_str());
                fprintf(fp, "END\n");
                fprintf(fp, "LOADS\n");
                for(size_t k=0; k<cur_obj.loads.size(); ++k)
                {
                    if(cur_obj.loads[k].type == "acceleration")
                    {
                        fprintf(fp, "  body gravity %s %s %s scale %s\n",
                                cur_obj.loads[k].x.c_str(),
                                cur_obj.loads[k].y.c_str(),
                                cur_obj.loads[k].z.c_str(),
                                cur_obj.loads[k].scale.c_str());
                    }
                    else if(cur_obj.loads[k].type == "pressure")
                    {
                        fprintf(fp, "  %s %s %s %s\n",
                                cur_obj.loads[k].app_type.c_str(),
                                cur_obj.loads[k].app_id.c_str(),
                                cur_obj.loads[k].type.c_str(),
                                cur_obj.loads[k].scale.c_str());
                    }
                    else
                    {
                        fprintf(fp, "  %s %s %s %s %s %s scale %s\n",
                                cur_obj.loads[k].app_type.c_str(),
                                cur_obj.loads[k].app_id.c_str(),
                                cur_obj.loads[k].type.c_str(),
                                cur_obj.loads[k].x.c_str(),
                                cur_obj.loads[k].y.c_str(),
                                cur_obj.loads[k].z.c_str(),
                                cur_obj.loads[k].scale.c_str());
                    }
                }
                if(frf)
                {
                    fprintf(fp, "  function=1\n");
                }
                fprintf(fp, "END\n");
                fprintf(fp, "BOUNDARY\n");
                for(size_t k=0; k<cur_obj.bcs.size(); ++k)
                {
                    if(cur_obj.bcs[k].dof.empty())
                    {
                        fprintf(fp, "  %s %s fixed\n",
                                cur_obj.bcs[k].app_type.c_str(),
                                cur_obj.bcs[k].app_id.c_str());
                    }
                    else
                    {
                        fprintf(fp, "  %s %s %s 0\n",
                                cur_obj.bcs[k].app_type.c_str(),
                                cur_obj.bcs[k].app_id.c_str(),
                                cur_obj.bcs[k].dof.c_str());
                    }
                }
                fprintf(fp, "END\n");
                fclose(fp);
            }
        }
    }
    return true;
}
/******************************************************************************/
bool XMLGenerator::generateAlbanyInputDecks()
/******************************************************************************/
{
    char string_var[200];
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        const Objective& cur_obj = m_InputData.objectives[i];
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
                addNTVParameter(n4, "Initial Value", "double", m_InputData.constraints[0].volume_fraction);
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
                    addNTVParameter(n6, "Penalization Parameter", "double", m_InputData.materials[0].penalty_exponent);
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
                    if(m_InputData.materials[n].material_id.compare(m_InputData.blocks[j].material_id) == 0)
                    {
                        if(!m_InputData.materials[n].youngs_modulus.empty())
                            addNTVParameter(n6, "Elastic Modulus", "double", m_InputData.materials[n].youngs_modulus);
                        if(!m_InputData.materials[n].poissons_ratio.empty())
                            addNTVParameter(n6, "Poissons Ratio", "double", m_InputData.materials[n].poissons_ratio);
                        if(!m_InputData.materials[n].thermal_conductivity.empty())
                            addNTVParameter(n6, "Isotropic Modulus", "double", m_InputData.materials[n].thermal_conductivity);
                        break;
                    }
                }
            }

            // Dirichlet BCs
            n3 = n2.append_child("ParameterList");
            n3.append_attribute("name") = "Dirichlet BCs";
            for(size_t j=0; j<cur_obj.bcs.size(); j++)
            {
                if(!cur_obj.type.compare("maximize stiffness"))
                {
                    if(cur_obj.bcs[j].dof.empty())
                    {
                        sprintf(string_var, "DBC on NS nodelist_%s for DOF %s",
                                cur_obj.bcs[j].app_id.c_str(), "X");
                        addNTVParameter(n3, string_var, "double", "0.0");
                        sprintf(string_var, "DBC on NS nodelist_%s for DOF %s",
                                cur_obj.bcs[j].app_id.c_str(), "Y");
                        addNTVParameter(n3, string_var, "double", "0.0");
                        sprintf(string_var, "DBC on NS nodelist_%s for DOF %s",
                                cur_obj.bcs[j].app_id.c_str(), "Z");
                        addNTVParameter(n3, string_var, "double", "0.0");
                    }
                    else
                    {
                        sprintf(string_var, "DBC on NS nodelist_%s for DOF %s",
                                cur_obj.bcs[j].app_id.c_str(), cur_obj.bcs[j].dof.c_str());
                        addNTVParameter(n3, string_var, "double", "0.0");
                    }
                }
                else if(!cur_obj.type.compare("maximize heat conduction"))
                {
                    sprintf(string_var, "DBC on NS nodelist_%s for DOF P",
                            cur_obj.bcs[j].app_id.c_str());
                    addNTVParameter(n3, string_var, "double", "0.0");
                }
            }

            // Neumann BCs
            n3 = n2.append_child("ParameterList");
            n3.append_attribute("name") = "Neumann BCs";
            for(size_t j=0; j<cur_obj.loads.size(); j++)
            {
                if(!cur_obj.type.compare("maximize stiffness"))
                {
                    sprintf(string_var, "NBC on SS surface_%s for DOF all set (t_x, t_y, t_z)",
                            cur_obj.loads[j].app_id.c_str());
                    char tmp_buf[200];
                    double x = std::atof(cur_obj.loads[j].x.c_str());
                    double y = std::atof(cur_obj.loads[j].y.c_str());
                    double z = std::atof(cur_obj.loads[j].z.c_str());
                    double scale = std::atof(cur_obj.loads[j].scale.c_str());
                    sprintf(tmp_buf, "{%lf,%lf,%lf}", x*scale, y*scale, z*scale);
                    addNTVParameter(n3, string_var, "Array(double)", tmp_buf);
                }
                else if(!cur_obj.type.compare("maximize heat conduction"))
                {
                    sprintf(string_var, "NBC on SS surface_%s for DOF P set (dudx, dudy, dudz)",
                            cur_obj.loads[j].app_id.c_str());
                    char tmp_buf[200];
                    sprintf(tmp_buf, "{%s,0.0,0.0}", cur_obj.loads[j].scale.c_str());
                    addNTVParameter(n3, string_var, "Array(double)", tmp_buf);
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
            sprintf(string_var, "%s.gen", m_InputData.mesh_name.c_str());
            addNTVParameter(n2, "Exodus Input File Name", "string", string_var);
            sprintf(string_var, "%s_alb_%s.exo", m_InputData.mesh_name.c_str(), cur_obj.name.c_str());
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
bool XMLGenerator::addNTVParameter(pugi::xml_node parent_node,
                                const std::string &name,
                                const std::string &type,
                                const std::string &value)
/******************************************************************************/
{
    pugi::xml_node node = parent_node.append_child("Parameter");
    node.append_attribute("name") = name.c_str();
    node.append_attribute("type") = type.c_str();
    node.append_attribute("value") = value.c_str();
    return true;
}

/******************************************************************************/
bool XMLGenerator::addChild(pugi::xml_node parent_node,
              const std::string &name,
              const std::string &value)
/******************************************************************************/
{
    pugi::xml_node tmp_node = parent_node.append_child(name.c_str());
    tmp_node = tmp_node.append_child(pugi::node_pcdata);
    tmp_node.set_value(value.c_str());
    return true;
}

/******************************************************************************/
bool XMLGenerator::generateLightMPInputDecks()
/******************************************************************************/
{
    pugi::xml_node node1, node2, node3;
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        const Objective& cur_obj = m_InputData.objectives[i];
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
                node3.set_value(m_InputData.materials[n].material_id.c_str());

                node1 = node1.append_child("linear_elastic");
                node2 = node1.append_child("poissons_ratio");
                node3 = node2.append_child(pugi::node_pcdata);
                node3.set_value(m_InputData.materials[n].poissons_ratio.c_str());
                node2 = node1.append_child("youngs_modulus");
                node3 = node2.append_child(pugi::node_pcdata);
                node3.set_value(m_InputData.materials[n].youngs_modulus.c_str());
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
            node3.set_value(m_InputData.mesh_name.c_str());
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
            for(size_t j=0; j<cur_obj.bcs.size(); ++j)
            {
                if(cur_obj.bcs[j].dof.empty())
                {
                    node4 = node3.append_child("displacement");
                    node5 = node4.append_child("nodeset");
                    node6 = node5.append_child(pugi::node_pcdata);
                    node6.set_value(cur_obj.bcs[j].app_id.c_str());
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
                    node6.set_value(cur_obj.bcs[j].app_id.c_str());
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
                    node6.set_value(cur_obj.bcs[j].app_id.c_str());
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
                    node4 = node3.append_child("displacement");
                    node5 = node4.append_child("nodeset");
                    node6 = node5.append_child(pugi::node_pcdata);
                    node6.set_value(cur_obj.bcs[j].app_id.c_str());
                    node5 = node4.append_child("direction");
                    node6 = node5.append_child(pugi::node_pcdata);
                    node6.set_value(cur_obj.bcs[j].dof.c_str());
                    node5 = node4.append_child("value");
                    node6 = node5.append_child(pugi::node_pcdata);
                    node6.set_value("0.0");
                    node5 = node4.append_child("scale");
                    node6 = node5.append_child(pugi::node_pcdata);
                    node6.set_value("1.0");
                }
            }
            for(size_t j=0; j<cur_obj.loads.size(); ++j)
            {
                node4 = node3.append_child("traction");
                node5 = node4.append_child("nodeset");
                node6 = node5.append_child(pugi::node_pcdata);
                node6.set_value(cur_obj.loads[j].app_id.c_str());
                node5 = node4.append_child("direction");
                node6 = node5.append_child(pugi::node_pcdata);
                double x = std::atof(cur_obj.loads[j].x.c_str());
                double y = std::atof(cur_obj.loads[j].y.c_str());
                double z = std::atof(cur_obj.loads[j].z.c_str());
                if(x > y && x > z)
                {
                    node6.set_value("x");
                }
                else if(y > x && y > z)
                {
                    node6.set_value("y");
                }
                else if(z > x && z > y)
                {
                    node6.set_value("z");
                }
                node5 = node4.append_child("value");
                node6 = node5.append_child(pugi::node_pcdata);
                node6.set_value(cur_obj.loads[j].scale.c_str());
                node5 = node4.append_child("scale");
                node6 = node5.append_child(pugi::node_pcdata);
                node6.set_value("1.0");
            }

            // Write the file to disk
            sprintf(buf, "lightmp_input_deck_%s.i", cur_obj.name.c_str());
            doc.save_file(buf, "  ");
        }
    }
    return true;
}
/******************************************************************************/
bool XMLGenerator::generatePhysicsInputDecks()
/******************************************************************************/
{
    generateSalinasInputDecks();
    generateAlbanyInputDecks();
    generateLightMPInputDecks();
    return true;
}

/******************************************************************************/
bool XMLGenerator::parseTokens(char *buffer, std::vector<std::string> &tokens)
/******************************************************************************/
{
    const char* token[MAX_TOKENS_PER_LINE] = {}; // initialize to 0
    int n = 0;

    // parse the line
    token[0] = strtok(buffer, DELIMITER); // first token

    // If there is a comment...
    if(token[0] && strlen(token[0]) > 1 && token[0][0] == '/' && token[0][1] == '/')
    {
        tokens.clear();
        return true;
    }

    if (token[0]) // zero if line is blank
    {
        for (n = 1; n < MAX_TOKENS_PER_LINE; n++)
        {
            token[n] = strtok(0, DELIMITER); // subsequent tokens
            if (!token[n])
                break; // no more tokens
        }
    }
    for(int i=0; i<n; ++i)
        tokens.push_back(token[i]);

    return true;
}

/******************************************************************************/
bool XMLGenerator::parseObjectives(std::istream &fin)
/******************************************************************************/
{
    std::vector<std::string> tInputStringList;
    char buf[MAX_CHARS_PER_LINE];
    std::vector<std::string> tokens;
    std::string tStringValue;

    // read each line of the file
    while (!fin.eof())
    {
        // read an entire line into memory
        fin.getline(buf, MAX_CHARS_PER_LINE);
        tokens.clear();
        parseTokens(buf, tokens);

        // process the tokens
        if(tokens.size() > 0)
        {
            for(size_t j=0; j<tokens.size(); ++j)
                tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"begin","objective"}, tStringValue))
            {
                Objective new_objective;
                // found an objective. parse it.
                // parse the rest of the objective
                while (!fin.eof())
                {
                    tokens.clear();
                    fin.getline(buf, MAX_CHARS_PER_LINE);
                    parseTokens(buf, tokens);
                    // process the tokens
                    if(tokens.size() > 0)
                    {
                        std::vector<std::string> unlowered_tokens = tokens;

                        for(size_t j=0; j<tokens.size(); ++j)
                            tokens[j] = toLower(tokens[j]);

                        if(parseSingleValue(tokens, tInputStringList = {"end","objective"}, tStringValue))
                        {
                            break;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"type"}, tStringValue))
                        {
                            if(tokens.size() < 2)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No type specified after \"type\" keyword.\n";
                                return false;
                            }
                            new_objective.type = tokens[1];
                            for(size_t j=2; j<tokens.size(); ++j)
                            {
                                new_objective.type += " ";
                                new_objective.type += tokens[j];
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"name"}, tStringValue))
                        {
                            if(tokens.size() < 2)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No name specified after \"name\" keyword.\n";
                                return false;
                            }
                            new_objective.name = tokens[1];
                            for(size_t j=2; j<tokens.size(); ++j)
                            {
                                new_objective.name += " ";
                                new_objective.name += tokens[j];
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"code"}, tStringValue))
                        {
                            if(tokens.size() < 2)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No code specified after \"code\" keyword.\n";
                                return false;
                            }
                            new_objective.code_name = tokens[1];
                            for(size_t j=2; j<tokens.size(); ++j)
                            {
                                new_objective.code_name += " ";
                                new_objective.code_name += tokens[j];
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"complex","error","measure"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No complex error measure value specified after \"complex error measure\" keywords.\n";
                                return false;
                            }
                            new_objective.complex_error_measure = tokens[3];
                            for(size_t j=4; j<tokens.size(); ++j)
                            {
                                new_objective.complex_error_measure += " ";
                                new_objective.complex_error_measure += tokens[j];
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"output","for","plotting"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No outputs specified after \"output for plotting\" keywords.\n";
                                return false;
                            }
                            for(size_t j=3; j<tokens.size(); ++j)
                            {
                                new_objective.output_for_plotting.push_back(tokens[j]);
                            }
                        }
                        else if (parseSingleValue(tokens, tInputStringList = {"ls","tet","type"}, tStringValue))
                        {
                            new_objective.convert_to_tet10 = tokens[3];
                          }
                        else if(parseSingleValue(tokens, tInputStringList = {"number","processors"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"number processors\" keywords.\n";
                                return false;
                            }
                            new_objective.num_procs = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"weight"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"weight\" keyword.\n";
                                return false;
                            }
                            new_objective.weight = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"frf","match","nodesets"}, tStringValue))
                        {
                            if(tokens.size() < 4)
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"frf match nodesets\" keywords.\n";
                                return false;
                            }
                            for(size_t j=3; j<tokens.size(); ++j)
                                new_objective.frf_match_nodesets.push_back(tokens[j]);
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"salinas","weight","mass","scale","factor"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"salinas weight mass scale factor\" keywords.\n";
                                return false;
                            }
                            new_objective.salinas_wtmass_scale_factor = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"salinas","gdsw","tolerance"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"salinas gdsw tolerance\" keywords.\n";
                                return false;
                            }
                            new_objective.salinas_gdsw_tolerance = tStringValue;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"reference","frf","file"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"reference frf file\" keywords.\n";
                                return false;
                            }
                            new_objective.ref_frf_file = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"raleigh","damping","alpha"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"raleigh damping alpha\" keywords.\n";
                                return false;
                            }
                            new_objective.raleigh_damping_alpha = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"raleigh","damping","beta"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseObjectives: No value specified after \"raleigh damping beta\" keywords.\n";
                                return false;
                            }
                            new_objective.raleigh_damping_beta = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"begin","bcs"}, tStringValue))
                        {
                            while (!fin.eof())
                            {
                                tokens.clear();
                                fin.getline(buf, MAX_CHARS_PER_LINE);
                                parseTokens(buf, tokens);
                                // process the tokens
                                if(tokens.size() > 0)
                                {
                                    for(size_t j=0; j<tokens.size(); ++j)
                                        tokens[j] = toLower(tokens[j]);

                                    BC new_bc;
                                    if(parseSingleValue(tokens, tInputStringList = {"end","bcs"}, tStringValue))
                                    {
                                        break;
                                    }
                                    else
                                    {
                                        if(tokens.size() < 4)
                                        {
                                            std::cout << "ERROR:XMLGenerator:parseObjectives: Valid number of BC parameters were not specified in \"bcs\" block.\n";
                                            return false;
                                        }
                                        if(tokens[0] != "fixed")
                                        {
                                            std::cout << "ERROR:XMLGenerator:parseObjectives: First BC token must be \"fixed\".\n";
                                            return false;
                                        }
                                        // token 0 is "fixed"
                                        size_t j = 1;  // "displacement or temperature"
                                        std::string cur_token1 = tokens[j];
                                        if(cur_token1 != "displacement" && cur_token1 != "temperature")
                                        {
                                            std::cout << "ERROR:XMLGenerator:parseObjectives: Only \"displacement\" and \"temperature\" BCs are currently allowed.\n";
                                            return false;
                                        }
                                        j++;  // "nodeset"
                                        std::string cur_token2 = tokens[j];
                                        if(cur_token2 != "nodeset")
                                        {
                                            std::cout << "ERROR:XMLGenerator:parseObjectives: BCs can only be applied to \"nodeset\" types at the moment.\n";
                                            return false;
                                        }
                                        new_bc.app_type = cur_token2;
                                        j++;  // nodeset id
                                        std::string cur_token3 = tokens[j];
                                        new_bc.app_id = cur_token3;
                                        j++;
                                        new_bc.dof = "";
                                        if(j<tokens.size())
                                        {
                                            // x, y, or z
                                            std::string cur_token4 = tokens[j];
                                            if(cur_token4 != "x" &&
                                               cur_token4 != "y" &&
                                               cur_token4 != "z")
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: BC degree of freedom must be either \"x\", \"y\", or \"z\".\n";
                                                return false;
                                            }
                                            new_bc.dof = cur_token4;
                                        }
                                    }
                                    new_objective.bcs.push_back(new_bc);
                                }
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"begin","loads"}, tStringValue))
                        {
                            while (!fin.eof())
                            {
                                tokens.clear();
                                fin.getline(buf, MAX_CHARS_PER_LINE);
                                parseTokens(buf, tokens);
                                // process the tokens
                                if(tokens.size() > 0)
                                {
                                    for(size_t j=0; j<tokens.size(); ++j)
                                        tokens[j] = toLower(tokens[j]);

                                    Load new_load;
                                    if(parseSingleValue(tokens, tInputStringList = {"end","loads"}, tStringValue))
                                    {
                                        break;
                                    }
                                    else
                                    {
                                        int j=0;
                                        new_load.type = tokens[j];  // traction or heat [flux] or force
                                        if(!new_load.type.compare("traction"))
                                        {
                                            if(tokens.size() != 9)
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: Wrong number of parameters specified for \"traction\" load.\n";
                                                return false;
                                            }
                                            new_load.app_type = tokens[++j];
                                            if(new_load.app_type != "sideset")
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: Tractions can only be specified on sidesets currently.\n";
                                                return false;
                                            }
                                            new_load.app_id = tokens[++j];
                                            ++j;  // "direction"
                                            if(tokens[3] != "direction")
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: \"direction\" keyword not specified after sideset id.\n";
                                                return false;
                                            }
                                            new_load.x = tokens[++j];
                                            new_load.y = tokens[++j];
                                            new_load.z = tokens[++j];
                                            if(tokens[7] != "scale")
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: \"scale\" keyword not specified after direction components.\n";
                                                return false;
                                            }
                                            ++j; // "scale"
                                            new_load.scale = tokens[++j];
                                        }
                                        else if(!new_load.type.compare("pressure"))
                                        {
                                            if(tokens.size() != 5)
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: Wrong number of parameters specified for \"pressure\" load.\n";
                                                return false;
                                            }
                                            new_load.app_type = tokens[++j];
                                            if(new_load.app_type != "sideset")
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: Pressures can currently only be specified on sidesets.\n";
                                                return false;
                                            }
                                            new_load.app_id = tokens[++j];
                                            ++j;  // "value"
                                            if(tokens[3] != "value")
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: \"value\" keyword not specified after sideset id.\n";
                                                return false;
                                            }
                                            new_load.scale = tokens[++j];
                                        }
                                        else if(!new_load.type.compare("acceleration"))
                                        {
                                            if(tokens.size() != 7)
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: Wrong number of parameters specified for \"acceleration\" load.\n";
                                                return false;
                                            }
                                            new_load.app_type = "body";
                                            ++j;  // "direction"
                                            if(tokens[j] != "direction")
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: \"direction\" keyword not specified after \"acceleration\".\n";
                                                return false;
                                            }
                                            new_load.x = tokens[++j];
                                            new_load.y = tokens[++j];
                                            new_load.z = tokens[++j];
                                            ++j;
                                            if(tokens[j] != "scale")
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: \"scale\" keyword not specified after direction components.\n";
                                                return false;
                                            }
                                            new_load.scale = tokens[++j];
                                        }
                                        else if(!new_load.type.compare("heat"))
                                        {
                                            if(tokens.size() != 5)
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: Wrong number of parameters specified for \"heat flux\" load.\n";
                                                return false;
                                            }
                                            ++j;
                                            if(!tokens[j].compare("flux"))
                                            {
                                                new_load.app_type = tokens[++j];
                                                if(new_load.app_type != "sideset")
                                                {
                                                    std::cout << "ERROR:XMLGenerator:parseObjectives: Heat flux can only be specified on sidesets currently.\n";
                                                    return false;
                                                }
                                                new_load.app_id = tokens[++j];
                                                new_load.scale = tokens[++j];
                                            }
                                            else
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: \"flux\" keyword must follow \"heat\" keyword.\n";
                                                return false;
                                            }
                                        }
                                        else if(!new_load.type.compare("force"))
                                        {
                                            if(tokens.size() != 9)
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: Wrong number of parameters specified for \"force\" load.\n";
                                                return false;
                                            }
                                            new_load.app_type = tokens[++j];
                                            if(new_load.app_type != "sideset" && new_load.app_type != "nodeset")
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: Forces can only be applied to nodesets or sidesets currently.\n";
                                                return false;
                                            }
                                            new_load.app_id = tokens[++j];
                                            ++j;  // "direction"
                                            if(tokens[3] != "direction")
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: \"direction\" keyword not specified after nodeset or sideset id.\n";
                                                return false;
                                            }
                                            new_load.x = tokens[++j];
                                            new_load.y = tokens[++j];
                                            new_load.z = tokens[++j];
                                            if(tokens[7] != "scale")
                                            {
                                                std::cout << "ERROR:XMLGenerator:parseObjectives: \"scale\" keyword not specified after direction components.\n";
                                                return false;
                                            }
                                            ++j; // "scale"
                                            new_load.scale = tokens[++j];
                                        }
                                        else
                                        {
                                            std::cout << "ERROR:XMLGenerator:parseObjectives: Unrecognized load type.  Must be traction, heat flux, or force.\n";
                                            return false;
                                        }
                                    }
                                    new_objective.loads.push_back(new_load);
                                }
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"begin","frequency"}, tStringValue))
                        {
                            while (!fin.eof())
                            {
                                tokens.clear();
                                fin.getline(buf, MAX_CHARS_PER_LINE);
                                parseTokens(buf, tokens);
                                // process the tokens
                                if(tokens.size() > 0)
                                {
                                    for(size_t j=0; j<tokens.size(); ++j)
                                        tokens[j] = toLower(tokens[j]);

                                    if(parseSingleValue(tokens, tInputStringList = {"end","frequency"}, tStringValue))
                                    {
                                        if(new_objective.freq_min == "" ||
                                                new_objective.freq_max == "" ||
                                                new_objective.freq_step == "")
                                        {
                                            std::cout << "ERROR:XMLGenerator:parseObjectives: Insufficient info in \"frequency block\".\n";
                                            return false;
                                        }
                                        break;
                                    }
                                    else
                                    {
                                        if(parseSingleValue(tokens, tInputStringList = {"min"}, tStringValue))
                                        {
                                            new_objective.freq_min = tStringValue;
                                        }
                                        else if(parseSingleValue(tokens, tInputStringList = {"max"}, tStringValue))
                                        {
                                            new_objective.freq_max = tStringValue;
                                        }
                                        else if(parseSingleValue(tokens, tInputStringList = {"step"}, tStringValue))
                                        {
                                            new_objective.freq_step = tStringValue;
                                        }
                                        else
                                        {
                                            std::cout << "ERROR:XMLGenerator:parseObjectives: Unrecognized keyword in \"frequency block\".\n";
                                            return false;
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            std::cout << "ERROR: Unhandled keyword(s) \"";
                            for(size_t ii=0; ii<tokens.size(); ++ii)
                            {
                                std::cout << tokens[ii];
                                if(ii != (tokens.size()-1))
                                    std::cout << " ";
                            }
                            std::cout << "\"\n";
                            return false;
                        }
                    }
                }
                m_InputData.objectives.push_back(new_objective);
            }
        }
    }
    char buf2[200];
    size_t num_objs = m_InputData.objectives.size();
    // If there were objectives without names add a default name
    for(size_t i=0; i<num_objs; ++i)
    {
        // For each code name we will make sure there are names set
        std::string cur_code_name = m_InputData.objectives[i].code_name;
        int num_cur_code_objs=0;
        for(size_t j=i; j<num_objs; ++j)
        {
            if(!m_InputData.objectives[j].code_name.compare(cur_code_name))
            {
                num_cur_code_objs++;
                if(m_InputData.objectives[j].name.empty())
                {
                    sprintf(buf2, "%d", num_cur_code_objs);
                    m_InputData.objectives[j].name = buf2;
                }
            }
        }
    }
    // Set the performer names
    for(size_t i=0; i<num_objs; ++i)
    {
        m_InputData.objectives[i].performer_name =
                m_InputData.objectives[i].code_name +
                "_" + m_InputData.objectives[i].name;
    }
    return true;
}

/******************************************************************************/
bool XMLGenerator::parseSingleValue(const std::vector<std::string> &aTokens,
                                    const std::vector<std::string> &aInputStrings,
                                    std::string &aReturnStringValue)
/******************************************************************************/
{
    size_t i;

    if(aInputStrings.size() < 1 || aTokens.size() < 1 || aTokens.size() < aInputStrings.size())
        return false;

    for(i=0; i<aInputStrings.size(); ++i)
    {
        if(aTokens[i].compare(aInputStrings[i]))
        {
            return false;
        }
    }

    if(aTokens.size() == (aInputStrings.size() + 1))
        aReturnStringValue = aTokens[i];

    return true;
}

/******************************************************************************/
bool XMLGenerator::parseSingleUnLoweredValue(const std::vector<std::string> &aTokens,
                                             const std::vector<std::string> &aUnLoweredTokens,
                                    const std::vector<std::string> &aInputStrings,
                                    std::string &aReturnStringValue)
/******************************************************************************/
{
    size_t i;

    assert(aTokens.size() == aUnLoweredTokens.size());

    if(aInputStrings.size() < 1 || aTokens.size() < 1 || aTokens.size() < aInputStrings.size())
        return false;

    for(i=0; i<aInputStrings.size(); ++i)
    {
        if(aTokens[i].compare(aInputStrings[i]))
        {
            return false;
        }
    }

    if(aTokens.size() == (aInputStrings.size() + 1))
        aReturnStringValue = aUnLoweredTokens[i];

    return true;
}

/******************************************************************************/
bool XMLGenerator::parseOptimizationParameters(std::istream &fin)
/******************************************************************************/
{
    // Initialize variables
    m_InputData.output_frequency="5";
    m_InputData.output_method="epu";
    m_InputData.check_gradient = "false";
    m_InputData.check_hessian = "false";
    m_InputData.GCMMA_inner_kkt_tolerance = "";
    m_InputData.GCMMA_outer_kkt_tolerance = "";
    m_InputData.GCMMA_inner_control_stagnation_tolerance = "";
    m_InputData.GCMMA_outer_control_stagnation_tolerance = "";
    m_InputData.GCMMA_outer_objective_stagnation_tolerance = "";
    m_InputData.GCMMA_max_inner_iterations = "";
    m_InputData.GCMMA_outer_stationarity_tolerance = "";
    m_InputData.GCMMA_initial_moving_asymptotes_scale_factor = "";
    m_InputData.KS_max_trust_region_iterations = "";
    m_InputData.KS_trust_region_expansion_factor = "";
    m_InputData.KS_trust_region_contraction_factor = "";
    m_InputData.KS_outer_gradient_tolerance = "";
    m_InputData.KS_outer_stationarity_tolerance = "";
    m_InputData.KS_outer_stagnation_tolerance = "";
    m_InputData.KS_outer_control_stagnation_tolerance = "";
    m_InputData.KS_outer_actual_reduction_tolerance = "";


    std::string tStringValue;
    std::vector<std::string> tInputStringList;
    // read each line of the file
    while (!fin.eof())
    {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        fin.getline(buf, MAX_CHARS_PER_LINE);
        std::vector<std::string> tokens;
        parseTokens(buf, tokens);

        // process the tokens
        if(tokens.size() > 0)
        {
            for(size_t j=0; j<tokens.size(); ++j)
                tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"begin","optimization","parameters"}, tStringValue))
            {
                // found optimization parameters
                while (!fin.eof())
                {
                    fin.getline(buf, MAX_CHARS_PER_LINE);
                    tokens.clear();
                    parseTokens(buf, tokens);
                    // process the tokens
                    if(tokens.size() > 0)
                    {
                        std::vector<std::string> unlowered_tokens = tokens;

                        for(size_t j=0; j<tokens.size(); ++j)
                            tokens[j] = toLower(tokens[j]);

                        if(parseSingleValue(tokens, tInputStringList = {"end","optimization","parameters"}, tStringValue))
                        {
                            break;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"begin","material_box"}, tStringValue))
                        {
                            while (!fin.eof())
                            {
                                fin.getline(buf, MAX_CHARS_PER_LINE);
                                tokens.clear();
                                parseTokens(buf, tokens);
                                // process the tokens
                                if(tokens.size() > 0)
                                {
                                    for(size_t j=0; j<tokens.size(); ++j)
                                        tokens[j] = toLower(tokens[j]);

                                    if(parseSingleValue(tokens, tInputStringList = {"end","material_box"}, tStringValue))
                                    {
                                        if(m_InputData.levelset_material_box_min == "" ||
                                           m_InputData.levelset_material_box_max == "")
                                        {
                                            std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: Not enough info in \"material_box\" block.\n";
                                            return false;
                                        }
                                        break;
                                    }
                                    else if(parseSingleValue(tokens, tInputStringList = {"min", "coords"}, tStringValue))
                                    {
                                        if(tokens.size() != 5)
                                        {
                                            std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: Wrong number of params in \"min coords\" line.\n";
                                            return false;
                                        }
                                        for(size_t j=2; j<5; ++j)
                                        {
                                            m_InputData.levelset_material_box_min += tokens[j];
                                            if(j<4)
                                                m_InputData.levelset_material_box_min += " ";
                                        }
                                    }
                                    else if(parseSingleValue(tokens, tInputStringList = {"max", "coords"}, tStringValue))
                                    {
                                        if(tokens.size() != 5)
                                        {
                                            std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: Wrong number of params in \"max coords\" line.\n";
                                            return false;
                                        }
                                        for(size_t j=2; j<5; ++j)
                                        {
                                            m_InputData.levelset_material_box_max += tokens[j];
                                            if(j<4)
                                                m_InputData.levelset_material_box_max += " ";
                                        }
                                    }
                                }
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"initial","density","value"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"initial density value\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.initial_density_value = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"create","levelset","spheres"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"create levelset spheres\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.create_levelset_spheres = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"levelset","initialization","method"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset initialization method\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.levelset_initialization_method = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"max","iterations"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"max iterations\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.max_iterations = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"restart","iteration"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"restart iteration\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.restart_iteration = tStringValue;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"initial","guess","filename"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"initial guess filename\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.initial_guess_filename = tStringValue;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"initial","guess","field","name"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"initial guess field name\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.initial_guess_field_name = tStringValue;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"prune","mesh"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"prune mesh\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.prune_mesh = tStringValue;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"number","buffer","layers"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number buffer layers\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.number_buffer_layers = tStringValue;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"number","prune","and","refine","processors"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number prune and refine processors\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.number_prune_and_refine_processors = tStringValue;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"number","refines"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number refines\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.number_refines = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"ks","max","trust","region","iterations"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks max trust region iterations\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.KS_max_trust_region_iterations = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","expansion","factor"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region expansion factor\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.KS_trust_region_expansion_factor = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"ks","trust","region","contraction","factor"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks trust region contraction factor\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.KS_trust_region_contraction_factor = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","gradient","tolerance"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer gradient tolerance\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.KS_outer_gradient_tolerance = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","stationarity","tolerance"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer stationarity tolerance\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.KS_outer_stationarity_tolerance = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","stagnation","tolerance"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer stagnation tolerance\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.KS_outer_stagnation_tolerance = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","control","stagnation","tolerance"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer control stagnation tolerance\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.KS_outer_control_stagnation_tolerance = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"ks","outer","actual","reduction","tolerance"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"ks outer actual reduction tolerance\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.KS_outer_actual_reduction_tolerance = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"gcmma","max","inner","iterations"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma max inner iterations\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.GCMMA_max_inner_iterations = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"gcmma","inner","kkt","tolerance"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma inner kkt tolerance\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.GCMMA_inner_kkt_tolerance = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"gcmma","inner","control","stagnation","tolerance"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma inner control stagnation tolerance\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.GCMMA_inner_control_stagnation_tolerance = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","kkt","tolerance"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma outer kkt tolerance\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.GCMMA_outer_kkt_tolerance = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","control","stagnation","tolerance"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma control stagnation tolerance\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.GCMMA_outer_control_stagnation_tolerance = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","objective","stagnation","tolerance"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma outer objective stagnation tolerance\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.GCMMA_outer_objective_stagnation_tolerance = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"gcmma","outer","stationarity","tolerance"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma outer stationarity tolerance\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.GCMMA_outer_stationarity_tolerance = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"gcmma","initial","moving","asymptotes","scale","factor"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"gcmma initial moving asymptotes scale factor\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.GCMMA_initial_moving_asymptotes_scale_factor = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"levelset","sphere","packing", "factor"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset sphere packing factor\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.levelset_sphere_packing_factor = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"levelset","sphere","radius"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset sphere radius\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.levelset_sphere_radius = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"levelset","nodesets"}, tStringValue))
                        {
                            if(tokens.size() < 3)
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"levelset nodesets\" keyword(s).\n";
                                return false;
                            }
                            for(size_t j=2; j<tokens.size(); ++j)
                            {
                                m_InputData.levelset_nodesets.push_back(tokens[j]);
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"output","frequency"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"output frequency\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.output_frequency = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"output","method"}, tStringValue))
                        {
                            if(tokens.size() < 3)
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"output method\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.output_method = tokens[2];
                            for(size_t j=3; j<tokens.size(); ++j)
                            {
                                m_InputData.output_method += " ";
                                m_InputData.output_method += tokens[j];
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"fixed","blocks"}, tStringValue))
                        {
                            if(tokens.size() < 3)
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"fixed blocks\" keyword(s).\n";
                                return false;
                            }
                            for(size_t j=2; j<tokens.size(); ++j)
                            {
                                m_InputData.fixed_block_ids.push_back(tokens[j]);
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"fixed","sidesets"}, tStringValue))
                        {
                            if(tokens.size() < 3)
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"fixed sidesets\" keyword(s).\n";
                                return false;
                            }
                            for(size_t j=2; j<tokens.size(); ++j)
                            {
                                m_InputData.fixed_sideset_ids.push_back(tokens[j]);
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"fixed","nodesets"}, tStringValue))
                        {
                            if(tokens.size() < 3)
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"fixed nodesets\" keyword(s).\n";
                                return false;
                            }
                            for(size_t j=2; j<tokens.size(); ++j)
                            {
                                m_InputData.fixed_nodeset_ids.push_back(tokens[j]);
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"number","processors"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"number processors\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.num_opt_processors = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"filter","radius","scale"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter radius scale\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.filter_radius_scale = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"filter","radius","absolute"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"filter radius absolute\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.filter_radius_absolute = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"algorithm"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"algorithm\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.optimization_algorithm = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"discretization"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"discretization\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.discretization = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"check","gradient"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"check gradient\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.check_gradient = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"check","hessian"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: No value specified after \"check hessian\" keyword(s).\n";
                                return false;
                            }
                            m_InputData.check_hessian = tStringValue;
                        }
                        else
                        {
                            std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: Unrecognized keyword.\n";
                            return false;
                        }
                    }
                }
            }
        }
    }

    // If there is a restart iteration but no filename specified then we will
    // assume we are coming from the gui and we will use the default "restart_XXX.exo"
    // filename, iteration 1 from the file, and field name "optimizationdofs".
    if(m_InputData.restart_iteration != "" &&
       m_InputData.restart_iteration != "0")
    {
        if(m_InputData.initial_guess_filename == "" &&
           m_InputData.initial_guess_field_name == "")
        {
            // This block indicates that we are coming from the gui so only the
            // restart iteration was specified.  We will fill in the other values
            // based on what we know the gui will be providing for the run.
            m_InputData.initial_guess_filename = "restart_" + m_InputData.restart_iteration + ".exo";
            m_InputData.restart_iteration = "1";
            m_InputData.initial_guess_field_name = "optimizationdofs";
        }
        else
        {
            // This block indicates that the user is manually setting up the
            // restart file and so we depend on him having specified a filename
            // and field name.  If either of these is empty we need to error out.
            if(m_InputData.initial_guess_field_name == "" ||
               m_InputData.initial_guess_filename == "")
            {
                std::cout << "ERROR:XMLGenerator:parseOptimizationParameters: You must specify a valid initial guess mesh filename and a valid field name on that mesh from which initial values will be obtained.\n";
                return false;
            }
        }
    }

    return true;
}

/******************************************************************************/
bool XMLGenerator::parseMesh(std::istream &fin)
/******************************************************************************/
{
    std::string tStringValue;
    std::vector<std::string> tInputStringList;

    // read each line of the file
    while (!fin.eof())
    {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        fin.getline(buf, MAX_CHARS_PER_LINE);
        std::vector<std::string> tokens;
        parseTokens(buf, tokens);

        // process the tokens
        if(tokens.size() > 0)
        {
            for(size_t j=0; j<tokens.size(); ++j)
                tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"begin","mesh"}, tStringValue))
            {
                // found mesh block
                while (!fin.eof())
                {
                    fin.getline(buf, MAX_CHARS_PER_LINE);
                    tokens.clear();
                    parseTokens(buf, tokens);
                    // process the tokens
                    if(tokens.size() > 0)
                    {
                        std::vector<std::string> unlowered_tokens = tokens;

                        for(size_t j=0; j<tokens.size(); ++j)
                            tokens[j] = toLower(tokens[j]);

                        if(parseSingleValue(tokens, tInputStringList = {"end","mesh"}, tStringValue))
                        {
                            break;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"name"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseMesh: No value specified after \"name\" keyword.\n";
                                return false;
                            }
                            size_t index = tStringValue.find(".gen");
                            if(index != std::string::npos)
                            {
                                tStringValue = tStringValue.substr(0, index);
                            }
                            m_InputData.mesh_name = tStringValue;
                        }
                        else
                        {
                            std::cout << "ERROR:XMLGenerator:parseMesh: Unrecognized keyword.\n";
                            return false;
                        }
                    }
                }
            }
        }
    }
    return true;
}
/******************************************************************************/
bool XMLGenerator::parseCodePaths(std::istream &fin)
/******************************************************************************/
{
    std::string tStringValue;
    std::vector<std::string> tInputStringList;

    // read each line of the file
    while (!fin.eof())
    {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        fin.getline(buf, MAX_CHARS_PER_LINE);
        std::vector<std::string> tokens;
        parseTokens(buf, tokens);

        // process the tokens
        if(tokens.size() > 0)
        {
            for(size_t j=0; j<tokens.size(); ++j)
                tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"begin","paths"}, tStringValue))
            {
                // found mesh block
                while (!fin.eof())
                {
                    fin.getline(buf, MAX_CHARS_PER_LINE);
                    tokens.clear();
                    parseTokens(buf, tokens);
                    // process the tokens
                    if(tokens.size() > 0)
                    {
                        std::vector<std::string> unlowered_tokens = tokens;

                        for(size_t j=0; j<tokens.size(); ++j)
                            tokens[j] = toLower(tokens[j]);

                        if(parseSingleValue(tokens, tInputStringList = {"end","paths"}, tStringValue))
                        {
                            break;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","salinas"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code salinas\" keywords.\n";
                                return false;
                            }
                            m_InputData.salinas_path = tStringValue;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","platomain"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code platomain\" keywords.\n";
                                return false;
                            }
                            m_InputData.plato_main_path = tStringValue;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","lightmp"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code lightmp\" keywords.\n";
                                return false;
                            }
                            m_InputData.lightmp_path = tStringValue;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","prune_and_refine"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code prune_and_refine\" keywords.\n";
                                return false;
                            }
                            m_InputData.prune_and_refine_path = tStringValue;
                        }
                        else if(parseSingleUnLoweredValue(tokens, unlowered_tokens, tInputStringList = {"code","albany"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseCodePaths: No value specified after \"code albany\" keywords.\n";
                                return false;
                            }
                            m_InputData.albany_path = tStringValue;
                        }
                        else
                        {
                            std::cout << "ERROR:XMLGenerator:parseCodePaths: Unrecognized keyword.\n";
                            return false;
                        }
                    }
                }
            }
        }
    }
    return true;
}
/******************************************************************************/
bool XMLGenerator::parseBlocks(std::istream &fin)
/******************************************************************************/
{
    std::string tStringValue;
    std::vector<std::string> tInputStringList;

    // read each line of the file
    while (!fin.eof())
    {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        fin.getline(buf, MAX_CHARS_PER_LINE);
        std::vector<std::string> tokens;
        parseTokens(buf, tokens);

        // process the tokens
        if(tokens.size() > 0)
        {
            for(size_t j=0; j<tokens.size(); ++j)
                tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"begin","block"}, tStringValue))
            {
                Block new_block;
                // found mesh block
                while (!fin.eof())
                {
                    fin.getline(buf, MAX_CHARS_PER_LINE);
                    tokens.clear();
                    parseTokens(buf, tokens);
                    // process the tokens
                    if(tokens.size() > 0)
                    {
                        for(size_t j=0; j<tokens.size(); ++j)
                            tokens[j] = toLower(tokens[j]);

                        if(parseSingleValue(tokens, tInputStringList = {"end","block"}, tStringValue))
                        {
                            if(new_block.block_id == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseBlocks: Block id was not specified for block.\n";
                                return false;
                            }
                            break;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"id"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseBlocks: No value specified after \"id\" keyword.\n";
                                return false;
                            }
                            new_block.block_id = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"material"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseBlocks: No value specified after \"material\" keyword.\n";
                                return false;
                            }
                            new_block.material_id = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"element","type"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseBlocks: No value specified after \"element type\" keywords.\n";
                                return false;
                            }
                            new_block.element_type = tStringValue;
                        }
                        else
                        {
                            std::cout << "ERROR:XMLGenerator:parseBlocks: Unrecognized keyword.\n";
                            return false;
                        }
                    }
                }
                m_InputData.blocks.push_back(new_block);
            }
        }
    }
    return true;
}
/******************************************************************************/
bool XMLGenerator::parseMaterials(std::istream &fin)
/******************************************************************************/
{
    std::string tStringValue;
    std::vector<std::string> tInputStringList;

    // read each line of the file
    while (!fin.eof())
    {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        fin.getline(buf, MAX_CHARS_PER_LINE);
        std::vector<std::string> tokens;
        parseTokens(buf, tokens);

        // process the tokens
        if(tokens.size() > 0)
        {
            for(size_t j=0; j<tokens.size(); ++j)
                tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"begin","material"}, tStringValue))
            {
                Material new_material;
                // found mesh block
                while (!fin.eof())
                {
                    fin.getline(buf, MAX_CHARS_PER_LINE);
                    tokens.clear();
                    parseTokens(buf, tokens);
                    // process the tokens
                    if(tokens.size() > 0)
                    {
                        for(size_t j=0; j<tokens.size(); ++j)
                            tokens[j] = toLower(tokens[j]);

                        if(parseSingleValue(tokens, tInputStringList = {"end","material"}, tStringValue))
                        {
                            if(new_material.material_id == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseMaterials: Material ids was not specified for material.\n";
                                return false;
                            }
                            break;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"id"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"id\" keyword.\n";
                                return false;
                            }
                            new_material.material_id = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"penalty","exponent"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"penalty exponent\" keywords.\n";
                                return false;
                            }
                            new_material.penalty_exponent = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"youngs","modulus"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"youngs modulus\" keywords.\n";
                                return false;
                            }
                            new_material.youngs_modulus = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"poissons","ratio"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"poissons ratio\" keywords.\n";
                                return false;
                            }
                            new_material.poissons_ratio = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"thermal","conductivity"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"thermal conductivity\" keywords.\n";
                                return false;
                            }
                            new_material.thermal_conductivity = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"density"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseMaterials: No value specified after \"density\" keywords.\n";
                                return false;
                            }
                            new_material.density = tStringValue;
                        }
                        else
                        {
                            std::cout << "ERROR:XMLGenerator:parseMaterials: Unrecognized keyword.\n";
                            return false;
                        }
                    }
                }
                m_InputData.materials.push_back(new_material);
            }
        }
    }
    return true;
}
/******************************************************************************/
bool XMLGenerator::parseConstraints(std::istream &fin)
/******************************************************************************/
{
    std::string tStringValue;
    std::vector<std::string> tInputStringList;

    // read each line of the file
    while (!fin.eof())
    {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        fin.getline(buf, MAX_CHARS_PER_LINE);
        std::vector<std::string> tokens;
        parseTokens(buf, tokens);

        // process the tokens
        if(tokens.size() > 0)
        {
            for(size_t j=0; j<tokens.size(); ++j)
                tokens[j] = toLower(tokens[j]);

            if(parseSingleValue(tokens, tInputStringList = {"begin","constraint"}, tStringValue))
            {
                Constraint new_constraint;
                // found constraint
                while (!fin.eof())
                {
                    fin.getline(buf, MAX_CHARS_PER_LINE);
                    tokens.clear();
                    parseTokens(buf, tokens);
                    // process the tokens
                    if(tokens.size() > 0)
                    {
                        for(size_t j=0; j<tokens.size(); ++j)
                            tokens[j] = toLower(tokens[j]);

                        if(parseSingleValue(tokens, tInputStringList = {"end","constraint"}, tStringValue))
                        {
                            break;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"type"}, tStringValue))
                        {
                            if(tokens.size() < 2)
                            {
                                std::cout << "ERROR:XMLGenerator:parseConstraints: Not enough params after \"type\" keyword.\n";
                                return false;
                            }
                            new_constraint.type = tokens[1];
                            for(size_t j=2; j<tokens.size(); ++j)
                            {
                                new_constraint.type += " ";
                                new_constraint.type += tokens[j];
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"name"}, tStringValue))
                        {
                            if(tokens.size() < 2)
                            {
                                std::cout << "ERROR:XMLGenerator:parseConstraints: Not enough params after \"name\" keyword.\n";
                                return false;
                            }
                            new_constraint.name = tokens[1];
                            for(size_t j=2; j<tokens.size(); ++j)
                            {
                                new_constraint.name += " ";
                                new_constraint.name += tokens[j];
                            }
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"volume","fraction"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseConstraints: Not value specified for \"volume fraction\".\n";
                                return false;
                            }
                            new_constraint.volume_fraction = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"volume","absolute"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseConstraints: Not value specified for \"volume absolute\".\n";
                                return false;
                            }
                            new_constraint.volume_absolute = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"surface","area","sideset","id"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseConstraints: Not value specified for \"surface area sideset id\".\n";
                                return false;
                            }
                            new_constraint.surface_area_ssid = tStringValue;
                        }
                        else if(parseSingleValue(tokens, tInputStringList = {"surface","area"}, tStringValue))
                        {
                            if(tStringValue == "")
                            {
                                std::cout << "ERROR:XMLGenerator:parseConstraints: Not value specified for \"surface area\".\n";
                                return false;
                            }
                            new_constraint.surface_area = tStringValue;
                        }
                        else
                        {
                            std::cout << "ERROR:XMLGenerator:parseConstraints: Invalid keyword.\n";
                            return false;
                        }
                    }
                }
                m_InputData.constraints.push_back(new_constraint);
            }
        }
    }

    return true;
}

bool XMLGenerator::find_tokens(std::vector<std::string> &tokens,
                               const int &start_index,
                               const char *str1,
                               const char *str2)
{
    return true;
}

/******************************************************************************/
bool XMLGenerator::parseFile()
/******************************************************************************/
{
    std::ifstream fin;
    fin.open(m_InputFilename.c_str()); // open a file
    if (!fin.good())
    {
        std::cout << "Failed to open " << m_InputFilename << "." << std::endl;
        return false; // exit if file not found
    }

    parseObjectives(fin);
    fin.close();
    fin.open(m_InputFilename.c_str()); // open a file
    parseOptimizationParameters(fin);
    fin.close();
    fin.open(m_InputFilename.c_str()); // open a file
    parseConstraints(fin);
    fin.close();
    fin.open(m_InputFilename.c_str()); // open a file
    parseMesh(fin);
    fin.close();
    fin.open(m_InputFilename.c_str()); // open a file
    parseMaterials(fin);
    fin.close();
    fin.open(m_InputFilename.c_str()); // open a file
    parseBlocks(fin);
    fin.close();
    fin.open(m_InputFilename.c_str()); // open a file
    parseCodePaths(fin);
    fin.close();


    return true;
}

/******************************************************************************/
std::string XMLGenerator::toLower(const std::string &s)
/******************************************************************************/
{
    char buffer[500];
    std::string ret;
    if(s.size() > 500)
        ret = "";
    else
    {
        size_t i;
        for(i=0; i<s.size(); ++i)
        {
            buffer[i] = tolower(s[i]);
        }
        buffer[i] = '\0';
        ret = buffer;
    }
    return ret;
}

/******************************************************************************/
std::string XMLGenerator::toUpper(const std::string &s)
/******************************************************************************/
{
    char buffer[500];
    std::string ret;
    if(s.size() > 500)
        ret = "";
    else
    {
        size_t i;
        for(i=0; i<s.size(); ++i)
        {
            buffer[i] = toupper(s[i]);
        }
        buffer[i] = '\0';
        ret = buffer;
    }
    return ret;
}

/******************************************************************************/
bool XMLGenerator::generatePlatoMainInputDeckXML()
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
    addChild(mesh_node, "mesh", m_InputData.mesh_name.c_str());
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
    doc.save_file("platomain.xml", "  ");

    return true;
}
/******************************************************************************/
bool XMLGenerator::generatePerformerOperationsXML()
/******************************************************************************/
{
    generateSalinasOperationsXML();
    generateAlbanyOperationsXML();
    generateLightMPOperationsXML();
    return true;
}
/******************************************************************************/
bool XMLGenerator::generateLightMPOperationsXML()
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

            // Cache State
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Function", "Cache State");
            addChild(tmp_node, "Name", "Cache State");

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
                addChild(tmp_node1, "PenaltyExponent", m_InputData.materials[0].penalty_exponent.c_str());
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
bool XMLGenerator::generateAlbanyOperationsXML()
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
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Function", "Cache State");
            addChild(tmp_node, "Name", "Cache State");

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
bool XMLGenerator::generateSalinasOperationsXML()
/******************************************************************************/
{
    int num_salinas_objs = 0;
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        if(!m_InputData.objectives[i].code_name.compare("salinas"))
        {
            num_salinas_objs++;

            pugi::xml_document doc;
            pugi::xml_node tmp_node1;

            // Version entry
            pugi::xml_node tmp_node = doc.append_child(pugi::node_declaration);
            tmp_node.set_name("xml");
            pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
            tmp_att.set_value("1.0");

            // Cache State
            tmp_node = doc.append_child("Operation");
            addChild(tmp_node, "Function", "Cache State");
            addChild(tmp_node, "Name", "Cache State");

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
                addChild(tmp_node1, "PenaltyExponent", m_InputData.materials[0].penalty_exponent.c_str());
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
                addChild(tmp_node1, "PenaltyExponent", m_InputData.materials[0].penalty_exponent.c_str());
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
                addChild(tmp_node1, "PenaltyExponent", m_InputData.materials[0].penalty_exponent.c_str());
            }
            addChild(tmp_node1, "MinimumValue", "0.001");
            tmp_node1 = tmp_node.append_child("OutputGradient");
            addChild(tmp_node1, "Name", "Internal Energy Gradient");

            if(m_InputData.optimization_algorithm.compare("ksbc") == 0 ||
               m_InputData.optimization_algorithm.compare("ksal") == 0)
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
                    addChild(tmp_node1, "PenaltyExponent", m_InputData.materials[0].penalty_exponent.c_str());
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
            sprintf(buf, "salinas_operations_%s.xml", m_InputData.objectives[i].name.c_str());
            // Write the file to disk
            doc.save_file(buf, "  ");
        }
    }

    return true;
}
/******************************************************************************/
bool XMLGenerator::generatePlatoOperationsXML()
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

    // Filter
    tmp_node = doc.append_child("Filter");
    addChild(tmp_node, "Name", "Kernel");
    if(m_InputData.filter_radius_scale != "")
        addChild(tmp_node, "Scale", m_InputData.filter_radius_scale);
    if(m_InputData.filter_radius_absolute != "")
        addChild(tmp_node, "Absolute", m_InputData.filter_radius_absolute);

    // PlatoMainOutput
    tmp_node = doc.append_child("Operation");
    addChild(tmp_node, "Function", "PlatoMainOutput");
    addChild(tmp_node, "Name", "PlatoMainOutput");
    tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Topology");
    tmp_node1 = tmp_node.append_child("Input");
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
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        for(size_t j=0; j<m_InputData.objectives[i].output_for_plotting.size(); j++)
        {
            tmp_node1 = tmp_node.append_child("Input");
            addChild(tmp_node1, "ArgumentName", m_InputData.objectives[i].performer_name + "_" + m_InputData.objectives[i].output_for_plotting[j]);
            if(m_InputData.objectives[i].output_for_plotting[j] == "vonmises")
            {
                addChild(tmp_node1, "Layout", "Element Field");
            }
        }
    }
    addChild(tmp_node, "OutputFrequency", m_InputData.output_frequency.c_str());
    addChild(tmp_node, "OutputMethod", m_InputData.output_method.c_str());
    addChild(tmp_node, "Discretization", m_InputData.discretization);

    // FilterControl
    tmp_node = doc.append_child("Operation");
    addChild(tmp_node, "Function", "Filter");
    addChild(tmp_node, "Name", "FilterControl");
    tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Field");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Filtered Field");

    // FilterGradient
    tmp_node = doc.append_child("Operation");
    addChild(tmp_node, "Function", "Filter");
    addChild(tmp_node, "Name", "FilterGradient");
    tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Field");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Filtered Field");
    addChild(tmp_node, "Transpose", "True");

    if(m_InputData.optimization_algorithm.compare("ksbc") == 0 ||
       m_InputData.optimization_algorithm.compare("ksal") == 0)
    {
        // FilterHessian
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "Filter");
        addChild(tmp_node, "Name", "FilterHessian");
        tmp_node1 = tmp_node.append_child("Input");
        addChild(tmp_node1, "ArgumentName", "Field");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Filtered Field");
        addChild(tmp_node, "Transpose", "True");
    }

    // InitializeField
    tmp_node = doc.append_child("Operation");
    addChild(tmp_node, "Function", "InitializeField");
    addChild(tmp_node, "Name", "Initialize Field");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Initialized Field");
    if(m_InputData.initial_guess_filename != "")
    {
        // This handles both restarting from an iteration from a previous
        // run or by just specifying an initial guess filename.
        addChild(tmp_node, "Method", "FromFieldOnInputMesh");
        tmp_node1 = tmp_node.append_child("FromFieldOnInputMesh");
        addChild(tmp_node1, "Name", m_InputData.mesh_name + ".gen");
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
            addChild(tmp_node1, "Value", m_InputData.initial_density_value.c_str());
        }
        else if(m_InputData.discretization == "levelset")
        {
            if(m_InputData.levelset_initialization_method == "primitives")
            {
                addChild(tmp_node, "Method", "PrimitivesLevelSet");
                tmp_node1 = tmp_node.append_child("PrimitivesLevelSet");
                addChild(tmp_node1, "BackgroundMeshName", m_InputData.mesh_name + ".gen");
                if(m_InputData.levelset_material_box_min != "" &&
                   m_InputData.levelset_material_box_max != "")
                {
                    tmp_node2 = tmp_node1.append_child("MaterialBox");
                    addChild(tmp_node2, "MinCoords", m_InputData.levelset_material_box_min);
                    addChild(tmp_node2, "MaxCoords", m_InputData.levelset_material_box_max);
                }
            }
            else
            {
                addChild(tmp_node, "Method", "SwissCheeseLevelSet");
                tmp_node1 = tmp_node.append_child("SwissCheeseLevelSet");
                if(m_InputData.create_levelset_spheres == "")
                    m_InputData.create_levelset_spheres = "false";
                addChild(tmp_node1, "CreateSpheres", m_InputData.create_levelset_spheres);
                if(m_InputData.create_levelset_spheres == "true" && m_InputData.levelset_sphere_radius != "")
                    addChild(tmp_node1, "SphereRadius", m_InputData.levelset_sphere_radius);
                if(m_InputData.create_levelset_spheres == "true" && m_InputData.levelset_sphere_packing_factor != "")
                    addChild(tmp_node1, "SpherePackingFactor", m_InputData.levelset_sphere_packing_factor);
                for(size_t r=0; r<m_InputData.levelset_nodesets.size(); ++r)
                {
                    addChild(tmp_node1, "NodeSet", m_InputData.levelset_nodesets[r]);
                }
                addChild(tmp_node1, "BackgroundMeshName", m_InputData.mesh_name + ".gen");
            }
        }
    }

    /* Original InitializeField code
    tmp_node = doc.append_child("Operation");
    addChild(tmp_node, "Function", "InitializeField");
    addChild(tmp_node, "Name", "Initialize Field");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Initialized Field");
    if(m_InputData.restart_iteration.size() > 0)
    {
        addChild(tmp_node, "Method", "FromFile");
        tmp_node1 = tmp_node.append_child("FromFile");
        addChild(tmp_node1, "Name", m_InputData.restart_mesh_filename);
        addChild(tmp_node1, "VariableName", m_InputData.restart_field_name);
        addChild(tmp_node1, "Iteration", m_InputData.restart_iteration);
    }
    else
    {
        if(m_InputData.discretization == "density")
        {
            addChild(tmp_node, "Method", "Uniform");
            tmp_node1 = tmp_node.append_child("Uniform");
            addChild(tmp_node1, "Value", m_InputData.initial_density_value.c_str());
        }
        else if(m_InputData.discretization == "levelset")
        {
            if(m_InputData.levelset_initialization_method == "primitives")
            {
                addChild(tmp_node, "Method", "PrimitivesLevelSet");
                tmp_node1 = tmp_node.append_child("PrimitivesLevelSet");
                addChild(tmp_node1, "BackgroundMeshName", m_InputData.mesh_name + ".gen");
                if(m_InputData.levelset_material_box_min != "" &&
                   m_InputData.levelset_material_box_max != "")
                {
                    tmp_node2 = tmp_node1.append_child("MaterialBox");
                    addChild(tmp_node2, "MinCoords", m_InputData.levelset_material_box_min);
                    addChild(tmp_node2, "MaxCoords", m_InputData.levelset_material_box_max);
                }
            }
            else
            {
                addChild(tmp_node, "Method", "SwissCheeseLevelSet");
                tmp_node1 = tmp_node.append_child("SwissCheeseLevelSet");
                if(m_InputData.create_levelset_spheres == "")
                    m_InputData.create_levelset_spheres = "false";
                addChild(tmp_node1, "CreateSpheres", m_InputData.create_levelset_spheres);
                if(m_InputData.create_levelset_spheres == "true" && m_InputData.levelset_sphere_radius != "")
                    addChild(tmp_node1, "SphereRadius", m_InputData.levelset_sphere_radius);
                if(m_InputData.create_levelset_spheres == "true" && m_InputData.levelset_sphere_packing_factor != "")
                    addChild(tmp_node1, "SpherePackingFactor", m_InputData.levelset_sphere_packing_factor);
                for(size_t r=0; r<m_InputData.levelset_nodesets.size(); ++r)
                {
                    addChild(tmp_node1, "NodeSet", m_InputData.levelset_nodesets[r]);
                }
                addChild(tmp_node1, "BackgroundMeshName", m_InputData.mesh_name + ".gen");
            }
        }
    }
    */

    if(m_InputData.discretization == "density")
    {
        // DesignVolume
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "DesignVolume");
        addChild(tmp_node, "Name", "Design Volume");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Design Volume");

        // ComputeVolume
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "ComputeVolume");
        addChild(tmp_node, "Name", "Compute Current Volume");
        tmp_node1 = tmp_node.append_child("Input");
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
    else if(m_InputData.discretization == "levelset")
    {
    }

    char buf[200];
    // AggregateEnergy
    tmp_node = doc.append_child("Operation");
    addChild(tmp_node, "Function", "Aggregator");
    addChild(tmp_node, "Name", "AggregateEnergy");
    tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Value");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        tmp_node2 = tmp_node1.append_child("Input");
        sprintf(buf, "Value %d", (int)(i+1));
        addChild(tmp_node2, "ArgumentName", buf);
    }
    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Value");

    tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Nodal Field");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        tmp_node2 = tmp_node1.append_child("Input");
        sprintf(buf, "Field %d", (int)(i+1));
        addChild(tmp_node2, "ArgumentName", buf);
    }
    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Field");

    tmp_node1 = tmp_node.append_child("Weighting");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        tmp_node2 = tmp_node1.append_child("Weight");
        addChild(tmp_node2, "Value", m_InputData.objectives[i].weight.c_str());
    }

    // AggregateGradient
    tmp_node = doc.append_child("Operation");
    addChild(tmp_node, "Function", "Aggregator");
    addChild(tmp_node, "Name", "AggregateGradient");
    tmp_node1 = tmp_node.append_child("Aggregate");
    addChild(tmp_node1, "Layout", "Nodal Field");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        tmp_node2 = tmp_node1.append_child("Input");
        sprintf(buf, "Field %d", (int)(i+1));
        addChild(tmp_node2, "ArgumentName", buf);
    }
    tmp_node2 = tmp_node1.append_child("Output");
    addChild(tmp_node2, "ArgumentName", "Field");

    tmp_node1 = tmp_node.append_child("Weighting");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        tmp_node2 = tmp_node1.append_child("Weight");
        addChild(tmp_node2, "Value", m_InputData.objectives[i].weight.c_str());
    }

    if(m_InputData.optimization_algorithm.compare("ksbc") == 0 ||
       m_InputData.optimization_algorithm.compare("ksal") == 0)
    {
        // AggregateHessian
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Function", "Aggregator");
        addChild(tmp_node, "Name", "AggregateHessian");
        tmp_node1 = tmp_node.append_child("Aggregate");
        addChild(tmp_node1, "Layout", "Nodal Field");
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
            tmp_node2 = tmp_node1.append_child("Input");
            sprintf(buf, "Field %d", (int)(i+1));
            addChild(tmp_node2, "ArgumentName", buf);
        }
        tmp_node2 = tmp_node1.append_child("Output");
        addChild(tmp_node2, "ArgumentName", "Field");
        
        tmp_node1 = tmp_node.append_child("Weighting");
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
            tmp_node2 = tmp_node1.append_child("Weight");
            addChild(tmp_node2, "Value", m_InputData.objectives[i].weight.c_str());
        }
    }

    // Set Lower Bounds
    tmp_node = doc.append_child("Operation");
    addChild(tmp_node, "Function", "SetLowerBounds");
    addChild(tmp_node, "Discretization", m_InputData.discretization);
    addChild(tmp_node, "Name", "Calculate Lower Bounds");
    tmp_node1 = tmp_node.append_child("Input");
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

    // Set Upper Bounds
    tmp_node = doc.append_child("Operation");
    addChild(tmp_node, "Function", "SetUpperBounds");
    addChild(tmp_node, "Discretization", m_InputData.discretization);
    addChild(tmp_node, "Name", "Calculate Upper Bounds");
    tmp_node1 = tmp_node.append_child("Input");
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

    // Enforce Bounds
    tmp_node = doc.append_child("Operation");
    addChild(tmp_node, "Function", "EnforceBounds");
    addChild(tmp_node, "Name", "EnforceBounds");
    tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Upper Bound Vector");
    tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Lower Bound Vector");
    tmp_node1 = tmp_node.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Topology");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Topology");


    // Write the file to disk
    doc.save_file("plato_operations.xml", "  ");

    return true;
}

/******************************************************************************/
bool XMLGenerator::outputVolumeGradientStage(pugi::xml_document &doc)
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
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "EnforceBounds");
    addChild(op_node, "PerformerName", "PlatoMain");

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Lower Bound Vector");
    addChild(input_node, "SharedDataName", "Lower Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Upper Bound Vector");
    addChild(input_node, "SharedDataName", "Upper Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Topology");
    addChild(output_node, "SharedDataName", "Topology");

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
    addChild(input_node, "SharedDataName", "Volume Gradient");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Field");
    addChild(output_node, "SharedDataName", "Volume Gradient");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Volume Gradient");

    return true;
}

/******************************************************************************/
bool XMLGenerator::outputSurfaceAreaGradientStage(pugi::xml_document &doc)
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
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "EnforceBounds");
    addChild(op_node, "PerformerName", "PlatoMain");

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Lower Bound Vector");
    addChild(input_node, "SharedDataName", "Lower Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Upper Bound Vector");
    addChild(input_node, "SharedDataName", "Upper Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Topology");
    addChild(output_node, "SharedDataName", "Topology");

    // Compute current surface area gradient
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Compute Surface Area Gradient");
    addChild(op_node, "PerformerName", m_InputData.objectives[0].performer_name.c_str());

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
    addChild(input_node, "SharedDataName", "Surface Area Gradient");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Field");
    addChild(output_node, "SharedDataName", "Surface Area Gradient");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Surface Area Gradient");

    return true;
}

/******************************************************************************/
bool XMLGenerator::outputVolumeStage(pugi::xml_document &doc)
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
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "EnforceBounds");
    addChild(op_node, "PerformerName", "PlatoMain");

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Lower Bound Vector");
    addChild(input_node, "SharedDataName", "Lower Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Upper Bound Vector");
    addChild(input_node, "SharedDataName", "Upper Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Topology");
    addChild(output_node, "SharedDataName", "Topology");

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
bool XMLGenerator::outputSurfaceAreaStage(pugi::xml_document &doc)
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
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "EnforceBounds");
    addChild(op_node, "PerformerName", "PlatoMain");

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Lower Bound Vector");
    addChild(input_node, "SharedDataName", "Lower Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Upper Bound Vector");
    addChild(input_node, "SharedDataName", "Upper Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Topology");
    addChild(output_node, "SharedDataName", "Topology");

    // Compute current surface area operation
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Compute Surface Area");
    addChild(op_node, "PerformerName", m_InputData.objectives[0].performer_name.c_str());

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
bool XMLGenerator::outputComputeStateStage(pugi::xml_document &doc)
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
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "EnforceBounds");
    addChild(op_node, "PerformerName", "PlatoMain");

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Lower Bound Vector");
    addChild(input_node, "SharedDataName", "Lower Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Upper Bound Vector");
    addChild(input_node, "SharedDataName", "Upper Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Topology");
    addChild(output_node, "SharedDataName", "Topology");

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
        addChild(op_node, "PerformerName", m_InputData.objectives[i].performer_name.c_str());

        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Topology");
        addChild(input_node, "SharedDataName", "Topology");
    }

    return true;
}


/******************************************************************************/
bool XMLGenerator::outputInternalEnergyStage(pugi::xml_document &doc)
/******************************************************************************/
{
    char tmp_buf[200];
    // Internal Energy
    pugi::xml_node stage_node = doc.append_child("Stage");
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
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "EnforceBounds");
    addChild(op_node, "PerformerName", "PlatoMain");

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Lower Bound Vector");
    addChild(input_node, "SharedDataName", "Lower Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Upper Bound Vector");
    addChild(input_node, "SharedDataName", "Upper Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Topology");
    addChild(output_node, "SharedDataName", "Topology");

    pugi::xml_node cur_parent = stage_node;
    if(m_InputData.objectives.size() > 1)
    {
        op_node = stage_node.append_child("Operation");
        cur_parent = op_node;
    }

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        op_node = cur_parent.append_child("Operation");
        addChild(op_node, "Name", "Compute Objective");
        addChild(op_node, "PerformerName", m_InputData.objectives[i].performer_name.c_str());

        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Topology");
        addChild(input_node, "SharedDataName", "Topology");

        output_node = op_node.append_child("Output");
        addChild(output_node, "ArgumentName", "Internal Energy");
        sprintf(tmp_buf, "Internal Energy %d", (int)(i+1));
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
        sprintf(tmp_buf, "Internal Energy %d", (int)(i+1));
        addChild(input_node, "SharedDataName", tmp_buf);
    }

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Value");
    addChild(output_node, "SharedDataName", "Internal Energy");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Internal Energy");

    return true;
}

/******************************************************************************/
bool XMLGenerator::outputInternalEnergyGradientStage(pugi::xml_document &doc)
/******************************************************************************/
{
    char tmp_buf[200];
    // Internal Energy
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Internal Energy Gradient");

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
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "EnforceBounds");
    addChild(op_node, "PerformerName", "PlatoMain");

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Lower Bound Vector");
    addChild(input_node, "SharedDataName", "Lower Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Upper Bound Vector");
    addChild(input_node, "SharedDataName", "Upper Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Topology");
    addChild(output_node, "SharedDataName", "Topology");

    pugi::xml_node cur_parent = stage_node;
    if(m_InputData.objectives.size() > 1)
    {
        op_node = stage_node.append_child("Operation");
        cur_parent = op_node;
    }

    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        op_node = cur_parent.append_child("Operation");
        addChild(op_node, "Name", "Compute Gradient");
        addChild(op_node, "PerformerName", m_InputData.objectives[i].performer_name.c_str());

        input_node = op_node.append_child("Input");
        addChild(input_node, "ArgumentName", "Topology");
        addChild(input_node, "SharedDataName", "Topology");

        output_node = op_node.append_child("Output");
        addChild(output_node, "ArgumentName", "Internal Energy Gradient");
        sprintf(tmp_buf, "Internal Energy %d Gradient", (int)(i+1));
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
        sprintf(tmp_buf, "Internal Energy %d Gradient", (int)(i+1));
        addChild(input_node, "SharedDataName", tmp_buf);
    }

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Field");
    addChild(output_node, "SharedDataName", "Internal Energy Gradient");

    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "FilterGradient");
    addChild(op_node, "PerformerName", "PlatoMain");

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Field");
    addChild(input_node, "SharedDataName", "Internal Energy Gradient");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Filtered Field");
    addChild(output_node, "SharedDataName", "Internal Energy Gradient");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Internal Energy Gradient");

    return true;
}

/******************************************************************************/
bool XMLGenerator::outputInternalEnergyHessianStage(pugi::xml_document &doc)
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
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "EnforceBounds");
    addChild(op_node, "PerformerName", "PlatoMain");

    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Lower Bound Vector");
    addChild(input_node, "SharedDataName", "Lower Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Upper Bound Vector");
    addChild(input_node, "SharedDataName", "Upper Bound Vector");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");

    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Topology");
    addChild(output_node, "SharedDataName", "Topology");

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
        addChild(op_node, "PerformerName", m_InputData.objectives[i].performer_name.c_str());
        
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
    
    // Aggregate
    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "AggregateHessian");
    addChild(op_node, "PerformerName", "PlatoMain");
    
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        input_node = op_node.append_child("Input");
        sprintf(tmp_buf, "Field %d", (int)(i+1));
        addChild(input_node, "ArgumentName", tmp_buf);
        sprintf(tmp_buf, "HessianTimesVector %d", (int)(i+1));
        addChild(input_node, "SharedDataName", tmp_buf);
    }
    
    output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Field");
    addChild(output_node, "SharedDataName", "Internal Energy Hessian");
    
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
bool XMLGenerator::generateInterfaceXML()
/******************************************************************************/
{
    pugi::xml_document doc;

    // Version entry
    pugi::xml_node tmp_node = doc.append_child(pugi::node_declaration);
    tmp_node.set_name("xml");
    pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
    tmp_att.set_value("1.0");

    //////////////////////////////////////////////////
    // Performers
    /////////////////////////////////////////////////

    // PlatoMain performer entry.
    pugi::xml_node performer_node = doc.append_child("Performer");
    addChild(performer_node, "Name", "PlatoMain");
    addChild(performer_node, "Code", "PlatoMain");
    addChild(performer_node, "CommID", "0");

    pugi::xml_node sd_node;
    char tmp_buf[200];

    // Other performers (one for each objective)
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        performer_node = doc.append_child("Performer");
        addChild(performer_node, "Name", m_InputData.objectives[i].performer_name);
        addChild(performer_node, "Code", m_InputData.objectives[i].code_name);
        sprintf(tmp_buf, "%d", (int)(i+1));
        addChild(performer_node, "CommID", tmp_buf);
    }

    //////////////////////////////////////////////////
    // Shared Data
    /////////////////////////////////////////////////

    // Internal Energy XXX shared data
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        // create shared data for objectives
        sd_node = doc.append_child("SharedData");
        sprintf(tmp_buf, "Internal Energy %d", (int)(i+1));
        addChild(sd_node, "Name", tmp_buf);
        addChild(sd_node, "Type", "Scalar");
        addChild(sd_node, "Layout", "Global");
        addChild(sd_node, "Size", "1");
        addChild(sd_node, "OwnerName", m_InputData.objectives[i].performer_name);
        addChild(sd_node, "UserName", "PlatoMain");
    }

    // Internal Energy XXX Gradient shared data
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        // create shared data for objectives
        sd_node = doc.append_child("SharedData");
        sprintf(tmp_buf, "Internal Energy %d Gradient", (int)(i+1));
        addChild(sd_node, "Name", tmp_buf);
        addChild(sd_node, "Type", "Scalar");
        addChild(sd_node, "Layout", "Nodal Field");
        addChild(sd_node, "OwnerName", m_InputData.objectives[i].performer_name);
        addChild(sd_node, "UserName", "PlatoMain");
    }
    
    // Output shared data
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        for(size_t j=0; j<m_InputData.objectives[i].output_for_plotting.size(); ++j)
        {
            // create shared data for objectives
            sd_node = doc.append_child("SharedData");
            sprintf(tmp_buf, "%s_%s", m_InputData.objectives[i].performer_name.c_str(), m_InputData.objectives[i].output_for_plotting[j].c_str());
            addChild(sd_node, "Name", tmp_buf);
            addChild(sd_node, "Type", "Scalar");
            if(m_InputData.objectives[i].output_for_plotting[j] == "vonmises")
                addChild(sd_node, "Layout", "Element Field");
            else
                addChild(sd_node, "Layout", "Nodal Field");
            addChild(sd_node, "OwnerName", m_InputData.objectives[i].performer_name);
            addChild(sd_node, "UserName", "PlatoMain");
        }
    }

    // Hessian shared data
    if(m_InputData.optimization_algorithm.compare("ksbc") == 0 ||
       m_InputData.optimization_algorithm.compare("ksal") == 0)
    {
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        {
            sd_node = doc.append_child("SharedData");
            sprintf(tmp_buf, "HessianTimesVector %d", (int)(i+1));
            addChild(sd_node, "Name", tmp_buf);
            addChild(sd_node, "Type", "Scalar");
            addChild(sd_node, "Layout", "Nodal Field");
            addChild(sd_node, "OwnerName", m_InputData.objectives[i].performer_name);
            addChild(sd_node, "UserName", "PlatoMain");
        }
    }

    // Internal Energy
    sd_node = doc.append_child("SharedData");
    addChild(sd_node, "Name", "Internal Energy");
    addChild(sd_node, "Type", "Scalar");
    addChild(sd_node, "Layout", "Global");
    addChild(sd_node, "Size", "1");
    addChild(sd_node, "OwnerName", "PlatoMain");
    addChild(sd_node, "UserName", "PlatoMain");

    // Internal Energy Gradient
    sd_node = doc.append_child("SharedData");
    addChild(sd_node, "Name", "Internal Energy Gradient");
    addChild(sd_node, "Type", "Scalar");
    addChild(sd_node, "Layout", "Nodal Field");
    addChild(sd_node, "OwnerName", "PlatoMain");
    addChild(sd_node, "UserName", "PlatoMain");

    // Internal Energy Hessian and Descent Direction
    if(m_InputData.optimization_algorithm.compare("ksbc") == 0 ||
       m_InputData.optimization_algorithm.compare("ksal") == 0)
    {
        sd_node = doc.append_child("SharedData");
        addChild(sd_node, "Name", "Internal Energy Hessian");
        addChild(sd_node, "Type", "Scalar");
        addChild(sd_node, "Layout", "Nodal Field");
        addChild(sd_node, "OwnerName", "PlatoMain");
        addChild(sd_node, "UserName", "PlatoMain");
        
        sd_node = doc.append_child("SharedData");
        addChild(sd_node, "Name", "Descent Direction");
        addChild(sd_node, "Type", "Scalar");
        addChild(sd_node, "Layout", "Nodal Field");
        addChild(sd_node, "OwnerName", "PlatoMain");
        addChild(sd_node, "UserName", "PlatoMain");
        for(size_t i=0; i<m_InputData.objectives.size(); ++i)
            addChild(sd_node, "UserName", m_InputData.objectives[i].performer_name);
    }
    
    // Optimization DOFs
    sd_node = doc.append_child("SharedData");
    addChild(sd_node, "Name", "Optimization DOFs");
    addChild(sd_node, "Type", "Scalar");
    addChild(sd_node, "Layout", "Nodal Field");
    addChild(sd_node, "OwnerName", "PlatoMain");
    addChild(sd_node, "UserName", "PlatoMain");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        addChild(sd_node, "UserName", m_InputData.objectives[i].performer_name);

    // Topology
    sd_node = doc.append_child("SharedData");
    addChild(sd_node, "Name", "Topology");
    addChild(sd_node, "Type", "Scalar");
    addChild(sd_node, "Layout", "Nodal Field");
    addChild(sd_node, "OwnerName", "PlatoMain");
    addChild(sd_node, "UserName", "PlatoMain");
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
        addChild(sd_node, "UserName", m_InputData.objectives[i].performer_name);

    if(m_InputData.discretization == "density")
    {
        // Volume
        sd_node = doc.append_child("SharedData");
        addChild(sd_node, "Name", "Volume");
        addChild(sd_node, "Type", "Scalar");
        addChild(sd_node, "Layout", "Global");
        addChild(sd_node, "Size", "1");
        addChild(sd_node, "OwnerName", "PlatoMain");
        addChild(sd_node, "UserName", "PlatoMain");

        // Volume Gradient
        sd_node = doc.append_child("SharedData");
        addChild(sd_node, "Name", "Volume Gradient");
        addChild(sd_node, "Type", "Scalar");
        addChild(sd_node, "Layout", "Nodal Field");
        addChild(sd_node, "OwnerName", "PlatoMain");
        addChild(sd_node, "UserName", "PlatoMain");

        // Design Volume
        sd_node = doc.append_child("SharedData");
        addChild(sd_node, "Name", "Design Volume");
        addChild(sd_node, "Type", "Scalar");
        addChild(sd_node, "Layout", "Global");
        addChild(sd_node, "Size", "1");
        addChild(sd_node, "OwnerName", "PlatoMain");
        addChild(sd_node, "UserName", "PlatoMain");

    }
    else if(m_InputData.discretization == "levelset")
    {
        // For now we will assume there is only one type of objective
        // if doing levelsets and we will just grab the first performer
        // as the one that will calculate the surface area.
        // Surface Area
        sd_node = doc.append_child("SharedData");
        addChild(sd_node, "Name", "Surface Area");
        addChild(sd_node, "Type", "Scalar");
        addChild(sd_node, "Layout", "Global");
        addChild(sd_node, "Size", "1");
        addChild(sd_node, "OwnerName", m_InputData.objectives[0].performer_name);
        addChild(sd_node, "UserName", "PlatoMain");

        // Surface Area Gradient
        sd_node = doc.append_child("SharedData");
        addChild(sd_node, "Name", "Surface Area Gradient");
        addChild(sd_node, "Type", "Scalar");
        addChild(sd_node, "Layout", "Nodal Field");
        addChild(sd_node, "OwnerName", m_InputData.objectives[0].performer_name);
        addChild(sd_node, "UserName", "PlatoMain");

    }

    // Lower Bound Value
    sd_node = doc.append_child("SharedData");
    addChild(sd_node, "Name", "Lower Bound Value");
    addChild(sd_node, "Type", "Scalar");
    addChild(sd_node, "Layout", "Global");
    addChild(sd_node, "Size", "1");
    addChild(sd_node, "OwnerName", "PlatoMain");
    addChild(sd_node, "UserName", "PlatoMain");

    // Upper Bound Value
    sd_node = doc.append_child("SharedData");
    addChild(sd_node, "Name", "Upper Bound Value");
    addChild(sd_node, "Type", "Scalar");
    addChild(sd_node, "Layout", "Global");
    addChild(sd_node, "Size", "1");
    addChild(sd_node, "OwnerName", "PlatoMain");
    addChild(sd_node, "UserName", "PlatoMain");

    // Lower Bound Vector
    sd_node = doc.append_child("SharedData");
    addChild(sd_node, "Name", "Lower Bound Vector");
    addChild(sd_node, "Type", "Scalar");
    addChild(sd_node, "Layout", "Nodal Field");
    addChild(sd_node, "OwnerName", "PlatoMain");
    addChild(sd_node, "UserName", "PlatoMain");

    // Upper Bound Vector
    sd_node = doc.append_child("SharedData");
    addChild(sd_node, "Name", "Upper Bound Vector");
    addChild(sd_node, "Type", "Scalar");
    addChild(sd_node, "Layout", "Nodal Field");
    addChild(sd_node, "OwnerName", "PlatoMain");
    addChild(sd_node, "UserName", "PlatoMain");

    
    //////////////////////////////////////////////////
    // Stages
    /////////////////////////////////////////////////

    // Output To File
    pugi::xml_node stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Output To File");

    pugi::xml_node op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "PlatoMainOutput");
    addChild(op_node, "PerformerName", "PlatoMain");

    pugi::xml_node input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Topology");
    addChild(input_node, "SharedDataName", "Topology");
    input_node = op_node.append_child("Input");
    addChild(input_node, "ArgumentName", "Internal Energy Gradient");
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
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        for(size_t j=0; j<m_InputData.objectives[i].output_for_plotting.size(); j++)
        {
            input_node = op_node.append_child("Input");
            sprintf(tmp_buf, "%s_%s", m_InputData.objectives[i].performer_name.c_str(), m_InputData.objectives[i].output_for_plotting[j].c_str());
            addChild(input_node, "ArgumentName", tmp_buf);
            addChild(input_node, "SharedDataName", tmp_buf);
        }
    }

    // Initialize Optimization DOFs
    stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Initialize Optimization DOFs");

    op_node = stage_node.append_child("Operation");
    addChild(op_node, "Name", "Initialize Field");
    addChild(op_node, "PerformerName", "PlatoMain");

    pugi::xml_node output_node = op_node.append_child("Output");
    addChild(output_node, "ArgumentName", "Initialized Field");
    addChild(output_node, "SharedDataName", "Optimization DOFs");

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Optimization DOFs");

    // Cache State
    stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Cache State");
    
    pugi::xml_node cur_parent = stage_node;
    if(m_InputData.objectives.size() > 1)
    {
        op_node = stage_node.append_child("Operation");
        cur_parent = op_node;
    }
    
    for(size_t i=0; i<m_InputData.objectives.size(); ++i)
    {
        op_node = cur_parent.append_child("Operation");
        addChild(op_node, "Name", "Cache State");
        addChild(op_node, "PerformerName", m_InputData.objectives[i].performer_name);
        for(size_t j=0; j<m_InputData.objectives[i].output_for_plotting.size(); j++)
        {
            output_node = op_node.append_child("Output");
            addChild(output_node, "ArgumentName", m_InputData.objectives[i].output_for_plotting[j]);
            addChild(output_node, "SharedDataName", m_InputData.objectives[i].performer_name + "_" + m_InputData.objectives[i].output_for_plotting[j]);
        }
    }

    // Set Lower Bounds
    stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Set Lower Bounds");
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

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Lower Bound Vector");

    // Set Upper Bounds
    stage_node = doc.append_child("Stage");
    addChild(stage_node, "Name", "Set Upper Bounds");
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

    output_node = stage_node.append_child("Output");
    addChild(output_node, "SharedDataName", "Upper Bound Vector");


    if(m_InputData.discretization == "density")
    {
        // Design Volume
        stage_node = doc.append_child("Stage");
        addChild(stage_node, "Name", "Design Volume");

        op_node = stage_node.append_child("Operation");
        addChild(op_node, "Name", "Design Volume");
        addChild(op_node, "PerformerName", "PlatoMain");

        output_node = op_node.append_child("Output");
        addChild(output_node, "ArgumentName", "Design Volume");
        addChild(output_node, "SharedDataName", "Design Volume");

        output_node = stage_node.append_child("Output");
        addChild(output_node, "SharedDataName", "Design Volume");

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

    // Compute State
    outputComputeStateStage(doc);

    // Internal Energy
    outputInternalEnergyStage(doc);

    // Internal Energy Gradient
    outputInternalEnergyGradientStage(doc);

    // Internal Energy Hessian
    if(m_InputData.optimization_algorithm.compare("ksbc") == 0 ||
            m_InputData.optimization_algorithm.compare("ksal") == 0)
    {
        outputInternalEnergyHessianStage(doc);
    }

    /////////////////////////////////////////////////
    // Misc.
    ////////////////////////////////////////////////


    // Optimizer
    pugi::xml_node misc_node = doc.append_child("Optimizer");
    if(m_InputData.optimization_algorithm.compare("oc") == 0)
        addChild(misc_node, "Package", "OC");
    else if(m_InputData.optimization_algorithm.compare("gcmma") == 0)
        addChild(misc_node, "Package", "GCMMA");
    else if(m_InputData.optimization_algorithm.compare("mma") == 0)
        addChild(misc_node, "Package", "MMA");
    else if(m_InputData.optimization_algorithm.compare("ksbc") == 0)
        addChild(misc_node, "Package", "KSBC");
    else if(m_InputData.optimization_algorithm.compare("ksal") == 0)
        addChild(misc_node, "Package", "KSAL");
    else if(m_InputData.optimization_algorithm.compare("derivativechecker") == 0)
    {
        addChild(misc_node, "Package", "DerivativeChecker");
        addChild(misc_node, "CheckGradient", m_InputData.check_gradient);
        addChild(misc_node, "CheckHessian", m_InputData.check_hessian);
    }


    tmp_node = misc_node.append_child("NLopt");
    addChild(tmp_node, "Method", "MMA");

    tmp_node = misc_node.append_child("Options");
    addChild(tmp_node, "DerivativeCheckerInitialSuperscript", "1");
    addChild(tmp_node, "DerivativeCheckerFinalSuperscript", "8");
    if(m_InputData.GCMMA_inner_kkt_tolerance.size() > 0)
        addChild(tmp_node, "GCMMAInnerKKTTolerance", m_InputData.GCMMA_inner_kkt_tolerance);
    if(m_InputData.GCMMA_outer_kkt_tolerance.size() > 0)
        addChild(tmp_node, "GCMMAOuterKKTTolerance", m_InputData.GCMMA_outer_kkt_tolerance);
    if(m_InputData.GCMMA_inner_control_stagnation_tolerance.size() > 0)
        addChild(tmp_node, "GCMMAInnerControlStagnationTolerance", m_InputData.GCMMA_inner_control_stagnation_tolerance);
    if(m_InputData.GCMMA_outer_control_stagnation_tolerance.size() > 0)
        addChild(tmp_node, "GCMMAOuterControlStagnationTolerance", m_InputData.GCMMA_outer_control_stagnation_tolerance);
    if(m_InputData.GCMMA_outer_objective_stagnation_tolerance.size() > 0)
        addChild(tmp_node, "GCMMAOuterObjectiveStagnationTolerance", m_InputData.GCMMA_outer_objective_stagnation_tolerance);
    if(m_InputData.GCMMA_max_inner_iterations.size() > 0)
        addChild(tmp_node, "GCMMAMaxInnerIterations", m_InputData.GCMMA_max_inner_iterations);
    if(m_InputData.GCMMA_outer_stationarity_tolerance.size() > 0)
        addChild(tmp_node, "GCMMAOuterStationarityTolerance", m_InputData.GCMMA_outer_stationarity_tolerance);
    if(m_InputData.GCMMA_initial_moving_asymptotes_scale_factor.size() > 0)
        addChild(tmp_node, "GCMMAInitialMovingAsymptoteScaleFactor", m_InputData.GCMMA_initial_moving_asymptotes_scale_factor);
    if(m_InputData.KS_trust_region_expansion_factor.size() > 0)
        addChild(tmp_node, "KSTrustRegionExpansionFactor", m_InputData.KS_trust_region_expansion_factor);
    if(m_InputData.KS_trust_region_contraction_factor.size() > 0)
        addChild(tmp_node, "KSTrustRegionContractionFactor", m_InputData.KS_trust_region_contraction_factor);
    if(m_InputData.KS_max_trust_region_iterations.size() > 0)
        addChild(tmp_node, "KSMaxTrustRegionIterations", m_InputData.KS_max_trust_region_iterations);
    if(m_InputData.KS_outer_gradient_tolerance.size() > 0)
        addChild(tmp_node, "KSOuterGradientTolerance", m_InputData.KS_outer_gradient_tolerance);
    if(m_InputData.KS_outer_stationarity_tolerance.size() > 0)
        addChild(tmp_node, "KSOuterStationarityTolerance", m_InputData.KS_outer_stationarity_tolerance);
    if(m_InputData.KS_outer_stagnation_tolerance.size() > 0)
        addChild(tmp_node, "KSOuterStagnationTolerance", m_InputData.KS_outer_stagnation_tolerance);
    if(m_InputData.KS_outer_control_stagnation_tolerance.size() > 0)
        addChild(tmp_node, "KSOuterControlStagnationTolerance", m_InputData.KS_outer_control_stagnation_tolerance);
    if(m_InputData.KS_outer_actual_reduction_tolerance.size() > 0)
        addChild(tmp_node, "KSOuterActualReductionTolerance", m_InputData.KS_outer_actual_reduction_tolerance);

    tmp_node = misc_node.append_child("Output");
    addChild(tmp_node, "OutputStage", "Output To File");

    tmp_node = misc_node.append_child("CacheStage");
    addChild(tmp_node, "Name", "Cache State");

    tmp_node = misc_node.append_child("OC");
    addChild(tmp_node, "MoveLimiter", "1.0");
    addChild(tmp_node, "StabilizationParameter", "0.5");
    addChild(tmp_node, "UseNewtonSearch", "True");
    addChild(tmp_node, "ConstraintMaxIterations", "25");

    tmp_node = misc_node.append_child("OptimizationVariables");
    addChild(tmp_node, "ValueName", "Optimization DOFs");
    addChild(tmp_node, "InitializationStage", "Initialize Optimization DOFs");
    addChild(tmp_node, "FilteredName", "Topology");
    addChild(tmp_node, "FilterControlStage", "FilterControl");
    addChild(tmp_node, "LowerBoundValueName", "Lower Bound Value");
    addChild(tmp_node, "LowerBoundVectorName", "Lower Bound Vector");
    addChild(tmp_node, "UpperBoundValueName", "Upper Bound Value");
    addChild(tmp_node, "UpperBoundVectorName", "Upper Bound Vector");
    addChild(tmp_node, "SetLowerBoundsStage", "Set Lower Bounds");
    addChild(tmp_node, "SetUpperBoundsStage", "Set Upper Bounds");
    if(m_InputData.optimization_algorithm.compare("ksbc") == 0 ||
       m_InputData.optimization_algorithm.compare("ksal") == 0)
    {
        addChild(tmp_node, "DescentDirectionName", "Descent Direction");
    }
    
    tmp_node = misc_node.append_child("Objective");
    addChild(tmp_node, "ValueName", "Internal Energy");
    addChild(tmp_node, "GradientName", "Internal Energy Gradient");
    addChild(tmp_node, "ValueStageName", "Internal Energy");
    addChild(tmp_node, "GradientStageName", "Internal Energy Gradient");
    if(m_InputData.optimization_algorithm.compare("ksbc") == 0 ||
       m_InputData.optimization_algorithm.compare("ksal") == 0)
    {
        addChild(tmp_node, "HessianName", "Internal Energy Hessian");
    }

    tmp_node = misc_node.append_child("BoundConstraint");
    if(m_InputData.discretization == "density")
    {
        addChild(tmp_node, "Upper", "1.0");
        addChild(tmp_node, "Lower", "0.0");
    }
    else
    {
        addChild(tmp_node, "Upper", "10.0");
        addChild(tmp_node, "Lower", "-10.0");
    }

    for(size_t b=0; b<m_InputData.constraints.size(); ++b)
    {
        tmp_node = misc_node.append_child("Constraint");
        if(m_InputData.constraints[b].type == "volume")
        {
            addChild(tmp_node, "Equality", "True");
            addChild(tmp_node, "Linear", "True");
            addChild(tmp_node, "ValueName", "Volume");
            addChild(tmp_node, "ReferenceValueName", "Design Volume");
            addChild(tmp_node, "GradientName", "Volume Gradient");
            if(m_InputData.constraints[b].volume_fraction != "")
                addChild(tmp_node, "NormalizedTargetValue", m_InputData.constraints[b].volume_fraction);
            if(m_InputData.constraints[b].volume_absolute != "")
                addChild(tmp_node, "AbsoluteTargetValue", m_InputData.constraints[b].volume_absolute);
            addChild(tmp_node, "Tolerance", "1e-3");
        }
        else if(m_InputData.constraints[b].type == "surface area")
        {
            addChild(tmp_node, "Equality", "True");
            addChild(tmp_node, "Linear", "True");
            addChild(tmp_node, "ValueName", "Surface Area");
            addChild(tmp_node, "ReferenceValue", m_InputData.constraints[b].surface_area);
            addChild(tmp_node, "GradientName", "Surface Area Gradient");
            addChild(tmp_node, "NormalizedTargetValue", "1.0");
            addChild(tmp_node, "Tolerance", "1e-3");
        }
    }

    tmp_node = misc_node.append_child("Convergence");
    addChild(tmp_node, "MaxIterations", m_InputData.max_iterations);

    // mesh
    pugi::xml_node mesh_node = doc.append_child("mesh");
    addChild(mesh_node, "type", "unstructured");
    addChild(mesh_node, "format", "exodus");
    addChild(mesh_node, "mesh", m_InputData.mesh_name);
    for(size_t n=0; n<m_InputData.blocks.size(); ++n)
    {
        tmp_node = mesh_node.append_child("block");
        addChild(tmp_node, "index", m_InputData.blocks[n].block_id);
        pugi::xml_node tmp_node1 = tmp_node.append_child("integration");
        addChild(tmp_node1, "type", "gauss");
        addChild(tmp_node1, "order", "2");
        addChild(tmp_node, "material", m_InputData.blocks[n].material_id);
    }

    // output
    tmp_node = doc.append_child("output");
    addChild(tmp_node, "file", "plato");
    addChild(tmp_node, "format", "exodus");

    // Write the file to disk
    doc.save_file("interface.xml", "  ");

    return true;
}



