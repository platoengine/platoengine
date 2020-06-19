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
 * SalinasInputDeckWriter.cpp
 *
 *  Created on: Nov 19, 2019
 *
 */


#include "XMLGeneratorDataStruct.hpp"
#include "SalinasInputDeckWriter.hpp"
#include "XMLGeneratorUtilities.hpp"

namespace XMLGen
{


/******************************************************************************/
SalinasInputDeckWriter::SalinasInputDeckWriter(const InputData &aInputData) :
        mInputData(aInputData)
/******************************************************************************/
{
}

/******************************************************************************/
SalinasInputDeckWriter::~SalinasInputDeckWriter()
/******************************************************************************/
{
}

void SalinasInputDeckWriter::generate(bool tHasUncertainties, bool tRequestedVonMisesOutput, std::ostringstream *aStringStream)
{
    for(size_t i=0; i<mInputData.objectives.size(); ++i)
    {
        const XMLGen::Objective& cur_obj = mInputData.objectives[i];
        if(!cur_obj.code_name.compare("sierra_sd"))
        {
            bool tNormalizeObjective = true;

            // Check for reasons for turning local objective normalization off.
            if(tHasUncertainties)
                tNormalizeObjective = false;
            if(cur_obj.type == "stress constrained mass minimization")
                tNormalizeObjective = false;

            bool frf = false;
            if(cur_obj.type.compare("match frf data") == 0)
                frf = true;
            char buf[200];
            sprintf(buf, "sierra_sd_input_deck_%s.i", cur_obj.name.c_str());
            FILE *fp=fopen(buf, "w");
            if(fp)
            {
                writeSolutionBlock(fp, cur_obj);
                writeParametersBlock(fp, cur_obj);
                writeFRFRelatedBlocks(fp, cur_obj, frf);
                writeGDSWBlock(fp, cur_obj);
                writeOutputsBlock(fp, frf);
                writeEchoBlock(fp, frf);
                writeMaterialBlocks(fp, frf);
                writeBlockBlocks(fp, frf);
                writeTOBlock(fp, cur_obj, frf, tNormalizeObjective);
                writeFileBlock(fp);
                writeLoadsBlock(fp, cur_obj, frf);
                writeBoundaryBlock(fp, cur_obj);
                fclose(fp);
            }
        }
    }
}

void SalinasInputDeckWriter::writeSolutionBlock(FILE *aFilePtr, const Objective &aObjective)
{
    if(aObjective.multi_load_case == "true")
    {
        fprintf(aFilePtr, "SOLUTION\n");
        for(size_t k=0; k<aObjective.load_case_ids.size(); ++k)
        {
            bool found = false;
            XMLGen::LoadCase cur_load_case;
            std::string cur_load_id = aObjective.load_case_ids[k];
            for(size_t qq=0; qq<mInputData.load_cases.size(); ++qq)
            {
                if(cur_load_id == mInputData.load_cases[qq].id)
                {
                    found = true;
                    cur_load_case = mInputData.load_cases[qq];
                }
            }
            if(found)
            {
                fprintf(aFilePtr, "  case '%s'\n", cur_load_case.id.c_str());
                fprintf(aFilePtr, "    topology_optimization\n");
                fprintf(aFilePtr, "    load=%s\n", cur_load_case.id.c_str());
            }
        }
        fprintf(aFilePtr, "  solver gdsw\n");
        fprintf(aFilePtr, "END\n");
    }
    else
    {
        fprintf(aFilePtr, "SOLUTION\n");
        fprintf(aFilePtr, "  case '%s'\n", aObjective.name.c_str());
        fprintf(aFilePtr, "  topology_optimization\n");
        fprintf(aFilePtr, "  solver gdsw\n");
        fprintf(aFilePtr, "END\n");
    }

}

void SalinasInputDeckWriter::writeBoundaryBlock(FILE *aFilePtr, const Objective &aObjective)
{
    fprintf(aFilePtr, "BOUNDARY\n");
    for(size_t k=0; k<aObjective.bc_ids.size(); ++k)
    {
        bool found = false;
        XMLGen::BC cur_bc;
        std::string cur_bc_id = aObjective.bc_ids[k];
        for(size_t qq=0; qq<mInputData.bcs.size(); ++qq)
        {
            if(cur_bc_id == mInputData.bcs[qq].bc_id)
            {
                found = true;
                cur_bc = mInputData.bcs[qq];
            }
        }
        if(found)
        {
            if(cur_bc.dof.empty())
            {
                fprintf(aFilePtr, "  %s %s fixed\n",
                        cur_bc.app_type.c_str(),
                        cur_bc.app_id.c_str());
            }
            else
            {
                if(cur_bc.value.empty())
                {
                    fprintf(aFilePtr, "  %s %s %s 0\n",
                            cur_bc.app_type.c_str(),
                            cur_bc.app_id.c_str(),
                            cur_bc.dof.c_str());
                }
                else
                {
                    fprintf(aFilePtr, "  %s %s %s %s\n",
                            cur_bc.app_type.c_str(),
                            cur_bc.app_id.c_str(),
                            cur_bc.dof.c_str(),
                            cur_bc.value.c_str());
                }
            }
        }
    }
    fprintf(aFilePtr, "END\n");
}

void SalinasInputDeckWriter::writeLoadsBlock(FILE *aFilePtr, const Objective &aObjective,
                                             const bool &aFRF)
{
    // Do the load/loads block(s)
    if(aObjective.multi_load_case == "true")
    {
        for(size_t k=0; k<aObjective.load_case_ids.size(); ++k)
        {
            bool found = false;
            XMLGen::LoadCase cur_load_case;
            std::string cur_load_id = aObjective.load_case_ids[k];
            for(size_t qq=0; qq<mInputData.load_cases.size(); ++qq)
            {
                if(cur_load_id == mInputData.load_cases[qq].id)
                {
                    found = true;
                    cur_load_case = mInputData.load_cases[qq];
                }
            }
            if(found)
            {
                fprintf(aFilePtr, "LOAD=%s\n", cur_load_case.id.c_str());
                for(size_t d=0; d<cur_load_case.loads.size(); d++)
                {
                    XMLGen::Load cur_load = cur_load_case.loads[d];
                    if(cur_load.type == "acceleration")
                    {
                        fprintf(aFilePtr, "  body gravity %s %s %s scale 1.0\n",
                                cur_load.values[0].c_str(),
                                cur_load.values[1].c_str(),
                                cur_load.values[2].c_str());
                    }
                    else if(cur_load.type == "pressure")
                    {
                        fprintf(aFilePtr, "  %s %s %s %s\n",
                                cur_load.app_type.c_str(),
                                cur_load.app_id.c_str(),
                                cur_load.type.c_str(),
                                cur_load.values[0].c_str());
                    }
                    else
                    {
                        fprintf(aFilePtr, "  %s %s %s %s %s %s scale 1.0\n",
                                cur_load.app_type.c_str(),
                                cur_load.app_id.c_str(),
                                cur_load.type.c_str(),
                                cur_load.values[0].c_str(),
                                cur_load.values[1].c_str(),
                                cur_load.values[2].c_str());
                    }
                }
                fprintf(aFilePtr, "END\n");
            }
        }
    }
    else
    {
        fprintf(aFilePtr, "LOADS\n");
        for(size_t k=0; k<aObjective.load_case_ids.size(); ++k)
        {
            bool found = false;
            XMLGen::LoadCase cur_load_case;
            std::string cur_load_id = aObjective.load_case_ids[k];
            for(size_t qq=0; qq<mInputData.load_cases.size(); ++qq)
            {
                if(cur_load_id == mInputData.load_cases[qq].id)
                {
                    found = true;
                    cur_load_case = mInputData.load_cases[qq];
                }
            }
            if(found)
            {
                for(size_t d=0; d<cur_load_case.loads.size(); d++)
                {
                    XMLGen::Load cur_load = cur_load_case.loads[d];
                    if(cur_load.type == "acceleration")
                    {
                        fprintf(aFilePtr, "  body gravity %s %s %s scale 1.0\n",
                                cur_load.values[0].c_str(),
                                cur_load.values[1].c_str(),
                                cur_load.values[2].c_str());
                    }
                    else if(cur_load.type == "pressure")
                    {
                        fprintf(aFilePtr, "  %s %s %s %s\n",
                                cur_load.app_type.c_str(),
                                cur_load.app_id.c_str(),
                                cur_load.type.c_str(),
                                cur_load.values[0].c_str());
                    }
                    else
                    {
                        fprintf(aFilePtr, "  %s %s %s %s %s %s scale 1.0\n",
                                cur_load.app_type.c_str(),
                                cur_load.app_id.c_str(),
                                cur_load.type.c_str(),
                                cur_load.values[0].c_str(),
                                cur_load.values[1].c_str(),
                                cur_load.values[2].c_str());
                    }
                }
            }
        }
        if(aFRF)
        {
            fprintf(aFilePtr, "  function=1\n");
        }
        fprintf(aFilePtr, "END\n");
    }
}

void SalinasInputDeckWriter::writeFileBlock(FILE *aFilePtr)
{
    fprintf(aFilePtr, "FILE\n");
    fprintf(aFilePtr, "  geometry_file '%s'\n", mInputData.run_mesh_name.c_str());
    fprintf(aFilePtr, "END\n");
}

void SalinasInputDeckWriter::writeParametersBlock(FILE *aFilePtr, const Objective &aObjective)
{
    if(aObjective.wtmass_scale_factor.length() > 0)
    {
        fprintf(aFilePtr, "PARAMETERS\n");
        fprintf(aFilePtr, "  WTMASS = %s\n", aObjective.wtmass_scale_factor.c_str());
        fprintf(aFilePtr, "END\n");
    }

}

void SalinasInputDeckWriter::writeEchoBlock(FILE *aFilePtr, const bool &aFRF)
{
    fprintf(aFilePtr, "ECHO\n");
    if(!aFRF)
        fprintf(aFilePtr, "  topology\n");
    fprintf(aFilePtr, "END\n");
}

void SalinasInputDeckWriter::writeBlockBlocks(FILE *aFilePtr, const bool &aFRF)
{
    for(size_t n=0; n<mInputData.blocks.size(); ++n)
    {
        if(mInputData.blocks[n].block_id.empty() == false)
        {
            fprintf(aFilePtr, "BLOCK %s\n", mInputData.blocks[n].block_id.c_str());
        }
        if(mInputData.blocks[n].material_id.empty() == false)
        {
            fprintf(aFilePtr, "  material %s\n", mInputData.blocks[n].material_id.c_str());
        }
        if(mInputData.blocks[n].element_type == "hex8")
        {
            // pass through element type
            fprintf(aFilePtr, "  hex8u\n"); //user under integrated hexes.  Others seem to fail the derivative checker
        }
        else if(mInputData.blocks[n].element_type == "tet4")
        {
            // pass through element type
            fprintf(aFilePtr, "  tet4\n");
        }
        else if(mInputData.blocks[n].element_type == "tet10")
        {
            // pass through element type
            fprintf(aFilePtr, "  tet10\n");
        }
        else if(mInputData.blocks[n].element_type == "rbar")
        {
            // pass through element type
            fprintf(aFilePtr, "  rbar\n");
        }
        else if(mInputData.blocks[n].element_type == "rbe3")
        {
            // pass through element type
            fprintf(aFilePtr, "  rbe3\n");
        }
        if(aFRF)
        {
            fprintf(aFilePtr, "  inverse_material_type homogeneous\n");
        }
        fprintf(aFilePtr, "END\n");
    }
}

void SalinasInputDeckWriter::writeMaterialBlocks(FILE *aFilePtr, const bool &aFRF)
{
    for(size_t n=0; n<mInputData.materials.size(); n++)
    {
        fprintf(aFilePtr, "MATERIAL %s\n", mInputData.materials[n].id().c_str());
        fprintf(aFilePtr, "  isotropic\n");
        fprintf(aFilePtr, "  E = %s\n", mInputData.materials[n].property("youngs modulus").c_str());
        fprintf(aFilePtr, "  nu = %s\n", mInputData.materials[n].property("poissons ratio").c_str());
        if(mInputData.materials[n].property("density").empty() == false)
            fprintf(aFilePtr, "  density = %s\n", mInputData.materials[n].property("density").c_str());
        fprintf(aFilePtr, "  material_penalty_model = simp\n");
        if(mInputData.discretization.compare("density") == 0)
        {
            if(mInputData.materials[n].property("penalty exponent").length() > 0)
                fprintf(aFilePtr, "  penalty_coefficient = %s\n", mInputData.materials[n].property("penalty exponent").c_str());
        }
        if(aFRF)
        {
            fprintf(aFilePtr, "  minimum_stiffness_penalty_value=1e-3\n");
        }
        fprintf(aFilePtr, "END\n");
    }
}

void SalinasInputDeckWriter::writeTOBlock(FILE *aFilePtr, const Objective &aObjective,
                                          const bool &aFRF, const bool &aNormalizeObjective)
{
    fprintf(aFilePtr, "TOPOLOGY-OPTIMIZATION\n");
    fprintf(aFilePtr, "  algorithm = plato_engine\n");
    if(aFRF)
    {
        fprintf(aFilePtr, "  case = inverse_methods\n");
        if(mInputData.discretization == "density")
            fprintf(aFilePtr, "  inverse_method_objective = directfrf-plato-density-method\n");
        else if(mInputData.discretization == "levelset")
            fprintf(aFilePtr, "  inverse_method_objective = directfrf-plato-levelset-method\n");
        fprintf(aFilePtr, "  ref_frf_file %s\n", aObjective.ref_frf_file.c_str());
        if(aObjective.frf_match_nodesets.size() > 0)
        {
            fprintf(aFilePtr, "  frf_nodesets");
            for(size_t b=0; b<aObjective.frf_match_nodesets.size(); ++b)
            {
                fprintf(aFilePtr, " %s", aObjective.frf_match_nodesets[b].c_str());
            }
            fprintf(aFilePtr, "\n");
        }
        if(aObjective.complex_error_measure.length() > 0)
            fprintf(aFilePtr, "  complex_error_measure %s\n", aObjective.complex_error_measure.c_str());

        if (aObjective.convert_to_tet10.length()>0)
            fprintf(aFilePtr, " ls_tet_mesh_type %s\n", aObjective.convert_to_tet10.c_str());
    }
    else
    {
        if(aObjective.type == "maximize stiffness")
        {
            fprintf(aFilePtr, "  case = compliance_min\n");
        }
        else if(aObjective.type == "limit stress")
        {
            fprintf(aFilePtr, "  case = stress_limit\n");
        }
        else if(aObjective.type == "stress constrained mass minimization")
        {
            fprintf(aFilePtr, "  case = stress_constrained_mass_minimization\n");
        }
        else if(aObjective.type == "stress p norm")
        {
            fprintf(aFilePtr, "  case = stress_min\n");
        }
        else if(aObjective.type == "compliance and volume min")
        {
            fprintf(aFilePtr, "  case = primary_compliance_secondary_volume\n");
        }
    }
    if(aObjective.volume_misfit_target != "")
    {
        fprintf(aFilePtr, "  volume_misfit_target = %s\n", aObjective.volume_misfit_target.c_str());
    }
    if(aObjective.scmm_initial_penalty != "")
    {
        fprintf(aFilePtr, "  aug_lag_penalty_initial_value = %s\n", aObjective.scmm_initial_penalty.c_str());
    }
    if(aObjective.scmm_penalty_expansion_factor != "")
    {
        fprintf(aFilePtr, "  aug_lag_penalty_expansion_factor = %s\n", aObjective.scmm_penalty_expansion_factor.c_str());
    }
    if(aObjective.scmm_constraint_exponent != "")
    {
        fprintf(aFilePtr, "  aug_lag_constraint_exponent = %s\n", aObjective.scmm_constraint_exponent.c_str());
    }
    if(aObjective.stress_limit != "")
    {
        fprintf(aFilePtr, "  stress_limit_value = %s\n", aObjective.stress_limit.c_str());
    }
    if(aObjective.relative_stress_limit != "")
    {
        fprintf(aFilePtr, "  relative_stress_limit = %s\n", aObjective.relative_stress_limit.c_str());
    }
    if(aObjective.stress_p_norm_power != "")
    {
        fprintf(aFilePtr, "  stress_p_norm_power = %s\n", aObjective.stress_p_norm_power.c_str());
    }
    if(aObjective.stress_ramp_factor != "")
    {
        fprintf(aFilePtr, "  relaxed_stress_ramp_factor = %s\n", aObjective.stress_ramp_factor.c_str());
    }
    if(aObjective.limit_power_min != "")
    {
        fprintf(aFilePtr, "  limit_min = %s\n", aObjective.limit_power_min.c_str());
    }
    if(aObjective.limit_power_max != "")
    {
        fprintf(aFilePtr, "  limit_max = %s\n", aObjective.limit_power_max.c_str());
    }
    if(aObjective.limit_power_feasible_bias != "")
    {
        fprintf(aFilePtr, "  limit_feasible_bias = %s\n", aObjective.limit_power_feasible_bias.c_str());
    }
    if(aObjective.limit_power_feasible_slope != "")
    {
        fprintf(aFilePtr, "  limit_feasible_slope = %s\n", aObjective.limit_power_feasible_slope.c_str());
    }
    if(aObjective.limit_power_infeasible_bias != "")
    {
        fprintf(aFilePtr, "  limit_infeasible_bias = %s\n", aObjective.limit_power_infeasible_bias.c_str());
    }
    if(aObjective.limit_power_infeasible_slope != "")
    {
        fprintf(aFilePtr, "  limit_infeasible_slope = %s\n", aObjective.limit_power_infeasible_slope.c_str());
    }
    if(aObjective.limit_reset_subfrequency != "")
    {
        fprintf(aFilePtr, "  limit_reset_subfrequency = %s\n", aObjective.limit_reset_subfrequency.c_str());
    }
    if(aObjective.limit_reset_count != "")
    {
        fprintf(aFilePtr, "  limit_reset_count = %s\n", aObjective.limit_reset_count.c_str());
    }
    if(aObjective.inequality_allowable_feasiblity_upper != "")
    {
        fprintf(aFilePtr, "  stress_ineq_upper = %s\n", aObjective.inequality_allowable_feasiblity_upper.c_str());
    }
    if(aObjective.inequality_feasibility_scale != "")
    {
        fprintf(aFilePtr, "  inequality_feasiblity_scale = %s\n", aObjective.inequality_feasibility_scale.c_str());
    }
    if(aObjective.inequality_infeasibility_scale != "")
    {
        fprintf(aFilePtr, "  inequality_infeasiblity_scale = %s\n", aObjective.inequality_infeasibility_scale.c_str());
    }
    if(aObjective.stress_inequality_power != "")
    {
        fprintf(aFilePtr, "  stress_inequality_power = %s\n", aObjective.stress_inequality_power.c_str());
    }
    if(aObjective.stress_favor_final != "")
    {
        fprintf(aFilePtr, "  stress_favor_final = %s\n", aObjective.stress_favor_final.c_str());
    }
    if(aObjective.stress_favor_updates != "")
    {
        fprintf(aFilePtr, "  stress_favor_updates = %s\n", aObjective.stress_favor_updates.c_str());
    }
    if(aObjective.volume_penalty_power != "")
    {
        fprintf(aFilePtr, "  volume_penalty_power = %s\n", aObjective.volume_penalty_power.c_str());
    }
    if(aObjective.volume_penalty_divisor != "")
    {
        fprintf(aFilePtr, "  volume_penalty_divisor = %s\n", aObjective.volume_penalty_divisor.c_str());
    }
    if(aObjective.volume_penalty_bias != "")
    {
        fprintf(aFilePtr, "  volume_penalty_bias = %s\n", aObjective.volume_penalty_bias.c_str());
    }
    if(aFRF)
    {
        if(mInputData.constraints.size() > 0)
        {
            if(mInputData.constraints[0].category() == "surface area")
            {
                fprintf(aFilePtr, "  surface_area_constraint_value = %s\n", mInputData.constraints[0].surface_area.c_str());
                fprintf(aFilePtr, "  surface_area_ssid = %s\n", mInputData.constraints[0].surface_area_ssid.c_str());
            }
        }
    }
    else
    {
        // this volume fraction communicates single material optimization to Sierra-SD.
        // this volume fraction does suggest a constraint value, and hence, is arbitrary.
        fprintf(aFilePtr, "  volume_fraction = %f\n", 0.314);
    }
    if(aObjective.multi_load_case == "true")
    {
        fprintf(aFilePtr, "  load_case_weights = ");
        for(size_t gg=0; gg<aObjective.load_case_weights.size(); ++gg)
        {
            fprintf(aFilePtr, "%s ", aObjective.load_case_weights[gg].c_str());
        }
        fprintf(aFilePtr, "\n");
    }
    if(aNormalizeObjective == false)
        fprintf(aFilePtr, "  objective_normalization false\n");
    fprintf(aFilePtr, "END\n");
}

void SalinasInputDeckWriter::writeFRFRelatedBlocks(FILE *aFilePtr, const Objective &aObjective,
                                                   const bool &aFRF)
{
    if(aFRF)
    {
        fprintf(aFilePtr, "INVERSE-PROBLEM\n");
        if(mInputData.discretization.compare("levelset") == 0)
        {
            std::string tTruthTableFile = "dummy_ttable_";
            tTruthTableFile += aObjective.name;
            tTruthTableFile += ".txt";
            std::string tRealDataFile = "dummy_data_";
            tRealDataFile += aObjective.name;
            tRealDataFile += ".txt";
            std::string tImagDataFile = "dummy_data_im_";
            tImagDataFile += aObjective.name;
            tImagDataFile += ".txt";
            // For levelset frf we will be generating the experimental data
            // files at each iteration corresponding to the new computational
            // mesh node ids.  Therefore, we will start things off with 3
            // dummy files that just have generic data in them.
            fprintf(aFilePtr, "  data_truth_table %s\n", tTruthTableFile.c_str());
            fprintf(aFilePtr, "  real_data_file %s\n", tRealDataFile.c_str());
            fprintf(aFilePtr, "  imaginary_data_file %s\n", tImagDataFile.c_str());
            // Create the 3 generic files.
            double tFreqMin, tFreqMax, tFreqStep;
            sscanf(aObjective.freq_min.c_str(), "%lf", &tFreqMin);
            sscanf(aObjective.freq_max.c_str(), "%lf", &tFreqMax);
            sscanf(aObjective.freq_step.c_str(), "%lf", &tFreqStep);
            // This is the formula sierra_sd uses to get the number of frequencies
            int tNumFreqs = (int)(((tFreqMax-tFreqMin)/tFreqStep)+0.5) + 1;
            int tNumMatchNodes = aObjective.frf_match_nodesets.size();
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
            fprintf(aFilePtr, "  data_truth_table %s\n", "ttable.txt");
            fprintf(aFilePtr, "  real_data_file %s\n", "data.txt");
            fprintf(aFilePtr, "  imaginary_data_file %s\n", "data_im.txt");
        }
        fprintf(aFilePtr, "END\n");
        fprintf(aFilePtr, "OPTIMIZATION\n");
        fprintf(aFilePtr, "  optimization_package ROL_lib\n");
        fprintf(aFilePtr, "  ROLmethod linesearch\n");
        fprintf(aFilePtr, "  LSstep Newton-Krylov\n");
        fprintf(aFilePtr, "  LS_curvature_condition null\n");
        fprintf(aFilePtr, "  Max_iter_Krylov 50\n");
        fprintf(aFilePtr, "  Use_FD_hessvec false\n");
        fprintf(aFilePtr, "  Use_inexact_hessvec false\n");
        fprintf(aFilePtr, "END\n");
        if(aObjective.raleigh_damping_alpha.length() > 0 &&
                aObjective.raleigh_damping_beta.length() > 0)
        {
            fprintf(aFilePtr, "DAMPING\n");
            fprintf(aFilePtr, "  alpha %s\n", aObjective.raleigh_damping_alpha.c_str());
            fprintf(aFilePtr, "  beta %s\n", aObjective.raleigh_damping_beta.c_str());
            fprintf(aFilePtr, "END\n");
        }
        fprintf(aFilePtr, "FREQUENCY\n");
        fprintf(aFilePtr, "  freq_min %s\n", aObjective.freq_min.c_str());
        fprintf(aFilePtr, "  freq_max %s\n", aObjective.freq_max.c_str());
        fprintf(aFilePtr, "  freq_step %s\n", aObjective.freq_step.c_str());
        fprintf(aFilePtr, "END\n");
        fprintf(aFilePtr, "FUNCTION 1\n");
        fprintf(aFilePtr, "  type linear\n");
        fprintf(aFilePtr, "  data 0 1\n");
        fprintf(aFilePtr, "  data 1e6 1\n");
        fprintf(aFilePtr, "END\n");
    }
}

void SalinasInputDeckWriter::writeGDSWBlock(FILE *aFilePtr, const Objective &aObjective)
{
    bool isDifficultForSolver = false;
    if(aObjective.type == "limit stress")
        isDifficultForSolver = true;
    else if(aObjective.type == "stress p norm")
        isDifficultForSolver = true;
    else if(aObjective.type == "stress constrained mass minimization")
        isDifficultForSolver = true;
    fprintf(aFilePtr, "GDSW\n");
    const bool haveSolverTolerance = (aObjective.analysis_solver_tolerance.length() > 0);
    if(haveSolverTolerance)
    {
        fprintf(aFilePtr, "  solver_tol = %s\n", aObjective.analysis_solver_tolerance.c_str());
    }
    if(isDifficultForSolver)
    {
        fprintf(aFilePtr, "  diag_scaling diagonal\n");
        fprintf(aFilePtr, "  krylov_method = GMRESClassic\n");
        fprintf(aFilePtr, "  stag_tol = 0.001\n");
        fprintf(aFilePtr, "  orthog = 0\n");
        fprintf(aFilePtr, "  num_GS_steps = 2\n");
        fprintf(aFilePtr, "  overlap = 4\n");
        fprintf(aFilePtr, "  bailout true\n");
        if(!haveSolverTolerance)
        {
            fprintf(aFilePtr, "  solver_tol = 1e-4\n");
        }
    }
    fprintf(aFilePtr, "END\n");

}

void SalinasInputDeckWriter::writeOutputsBlock(FILE *aFilePtr, const bool &aFRF)
{
    fprintf(aFilePtr, "OUTPUTS\n");
    if(!aFRF)
        fprintf(aFilePtr, "  topology\n");
    fprintf(aFilePtr, "END\n");
}


}


