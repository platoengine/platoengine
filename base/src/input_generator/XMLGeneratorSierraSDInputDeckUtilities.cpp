/*
 * XMLGeneratorSierraSDOperationsFileUtilities.cpp
 *
 *  Created on: Feb 6, 2021
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorSierraSDOperationsFileUtilities.hpp"

namespace XMLGen
{

/**************************************************************************/
void append_solution_block
(const XMLGen::InputData& aMetaData,
 FILE *aFilePtr)
{
    fprintf(aFilePtr, "SOLUTION\n");
    if(aMetaData.objective.multi_load_case == "true")
    {
        for(auto tScenarioID : aMetaData.objective.scenarioIDs)
        {
            fprintf(aFilePtr, "  case '%s'\n", tScenarioID.c_str());
            fprintf(aFilePtr, "    topology_optimization\n");
            fprintf(aFilePtr, "    load=%s\n", tScenarioID.c_str());
        }
    }
    else
    {
        fprintf(aFilePtr, "  case '%s'\n", aMetaData.objective.scenarioIDs[0].c_str());
        fprintf(aFilePtr, "  topology_optimization\n");
    }
    fprintf(aFilePtr, "  solver gdsw\n");
    fprintf(aFilePtr, "END\n");
}
/**************************************************************************/
void append_parameters_block
(const XMLGen::Scenario &aScenario,
 FILE *aFilePtr)
{
    auto tWeightMassScaleFactor = aScenario.weightMassScaleFactor();
    if(tWeightMassScaleFactor != "")
    {
        fprintf(aFilePtr, "PARAMETERS\n");
        fprintf(aFilePtr, "  WTMASS = %s\n", tWeightMassScaleFactor.c_str());
        fprintf(aFilePtr, "END\n");
    }
}
/**************************************************************************/
void append_frf_related_blocks
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 FILE *aFilePtr)
{
    if(aCriterion.type() == "frf_mismatch")
    {
        fprintf(aFilePtr, "INVERSE-PROBLEM\n");
        if(aMetaData.optimization_parameters().discretization().compare("levelset") == 0)
        {
            std::string tTruthTableFile = "dummy_ttable_";
            tTruthTableFile += aScenario.id();
            tTruthTableFile += ".txt";
            std::string tRealDataFile = "dummy_data_";
            tRealDataFile += aScenario.id();
            tRealDataFile += ".txt";
            std::string tImagDataFile = "dummy_data_im_";
            tImagDataFile += aScenario.id();
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
            sscanf(aScenario.frequency_min().c_str(), "%lf", &tFreqMin);
            sscanf(aScenario.frequency_max().c_str(), "%lf", &tFreqMax);
            sscanf(aScenario.frequency_step().c_str(), "%lf", &tFreqStep);
            // This is the formula sierra_sd uses to get the number of frequencies
            int tNumFreqs = (int)(((tFreqMax-tFreqMin)/tFreqStep)+0.5) + 1;
            int tNumMatchNodes = aScenario.frfMatchNodesetIDs().size();
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
        if(aScenario.raleigh_damping_alpha().length() > 0 &&
           aScenario.raleigh_damping_beta().length() > 0)
        {
            fprintf(aFilePtr, "DAMPING\n");
            fprintf(aFilePtr, "  alpha %s\n", aScenario.raleigh_damping_alpha().c_str());
            fprintf(aFilePtr, "  beta %s\n", aScenario.raleigh_damping_beta().c_str());
            fprintf(aFilePtr, "END\n");
        }
        fprintf(aFilePtr, "FREQUENCY\n");
        fprintf(aFilePtr, "  freq_min %s\n", aScenario.frequency_min().c_str());
        fprintf(aFilePtr, "  freq_max %s\n", aScenario.frequency_max().c_str());
        fprintf(aFilePtr, "  freq_step %s\n", aScenario.frequency_step().c_str());
        fprintf(aFilePtr, "END\n");
        fprintf(aFilePtr, "FUNCTION 1\n");
        fprintf(aFilePtr, "  type linear\n");
        fprintf(aFilePtr, "  data 0 1\n");
        fprintf(aFilePtr, "  data 1e6 1\n");
        fprintf(aFilePtr, "END\n");
    }
}
/**************************************************************************/
void append_gdsw_block
(const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 FILE *aFilePtr)
{
    bool isDifficultForSolver = false;
    if(aCriterion.type() == "limit_stress")
        isDifficultForSolver = true;
    else if(aCriterion.type() == "stress_p-norm")
        isDifficultForSolver = true;
    else if(aCriterion.type() == "stress_and_mass")
        isDifficultForSolver = true;
    fprintf(aFilePtr, "GDSW\n");
    const bool haveSolverTolerance = (aScenario.solverTolerance() != "");
    if(haveSolverTolerance)
    {
        fprintf(aFilePtr, "  solver_tol = %s\n", aScenario.solverTolerance().c_str());
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
/**************************************************************************/
void append_outputs_block
(const XMLGen::Criterion &aCriterion,
 FILE *aFilePtr)
{
    fprintf(aFilePtr, "OUTPUTS\n");
    if(aCriterion.type() != "frf_mismatch")
    {
        fprintf(aFilePtr, "  topology\n");
    }
    fprintf(aFilePtr, "END\n");
}
/**************************************************************************/
void append_echo_block
(const XMLGen::Criterion &aCriterion,
 FILE *aFilePtr)
{
    fprintf(aFilePtr, "ECHO\n");
    if(aCriterion.type() != "frf_mismatch")
    {
        fprintf(aFilePtr, "  topology\n");
    }
    fprintf(aFilePtr, "END\n");
}
/**************************************************************************/
void append_material_blocks
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 FILE *aFilePtr)
{
    for(auto &tMaterial : aMetaData.materials)
    {
        fprintf(aFilePtr, "MATERIAL %s\n", tMaterial.id().c_str());
        fprintf(aFilePtr, "  isotropic\n");
        fprintf(aFilePtr, "  E = %s\n", tMaterial.value("youngs_modulus").c_str());
        fprintf(aFilePtr, "  nu = %s\n", tMaterial.value("poissons_ratio").c_str());
        if(tMaterial.value("mass_density").empty() == false)
            fprintf(aFilePtr, "  density = %s\n", tMaterial.value("mass_density").c_str());
        fprintf(aFilePtr, "  material_penalty_model = simp\n");
        if(aMetaData.optimization_parameters().discretization().compare("density") == 0)
        {
            if(aScenario.materialPenaltyExponent().length() > 0)
                fprintf(aFilePtr, "  penalty_coefficient = %s\n", aScenario.materialPenaltyExponent().c_str());
        }
        if(aCriterion.type() == "frf_mismatch")
        {
            fprintf(aFilePtr, "  minimum_stiffness_penalty_value=1e-3\n");
        }
        fprintf(aFilePtr, "END\n");
    }
}
/**************************************************************************/
void append_block_blocks
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 FILE *aFilePtr)
{
    for(auto &tBlock : aMetaData.blocks)
    {
        if(tBlock.block_id.empty() == false)
        {
            fprintf(aFilePtr, "BLOCK %s\n", tBlock.block_id.c_str());
        }
        if(tBlock.material_id.empty() == false)
        {
            fprintf(aFilePtr, "  material %s\n", tBlock.material_id.c_str());
        }
        if(tBlock.element_type == "hex8")
        {
            fprintf(aFilePtr, "  hex8u\n"); 
        }
        else if(tBlock.element_type == "tet4")
        {
            fprintf(aFilePtr, "  tet4\n");
        }
        else if(tBlock.element_type == "tet10")
        {
            fprintf(aFilePtr, "  tet10\n");
        }
        else if(tBlock.element_type == "rbar")
        {
            fprintf(aFilePtr, "  rbar\n");
        }
        else if(tBlock.element_type == "rbe3")
        {
            fprintf(aFilePtr, "  rbe3\n");
        }
        if(aCriterion.type() == "frf_mismatch")
        {
            fprintf(aFilePtr, "  inverse_material_type homogeneous\n");
        }
        fprintf(aFilePtr, "END\n");
    }
}
/**************************************************************************/
void append_stress_parameters
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 FILE *aFilePtr)
{
    if(aCriterion.type() == "stress_and_mass" ||
       aCriterion.type() == "limit_stress" ||
       aCriterion.type() == "stress_p-norm")
    {
        if(aCriterion.volume_misfit_target() != "")
        {
            fprintf(aFilePtr, "  volume_misfit_target = %s\n", aCriterion.volume_misfit_target().c_str());
        }
        if(aCriterion.scmmInitialPenalty() != "")
        {
            fprintf(aFilePtr, "  aug_lag_penalty_initial_value = %s\n", aCriterion.scmmInitialPenalty().c_str());
        }
        if(aCriterion.scmmPenaltyExpansionMultiplier() != "")
        {
            fprintf(aFilePtr, "  aug_lag_penalty_expansion_factor = %s\n", aCriterion.scmmPenaltyExpansionMultiplier().c_str());
        }
        if(aCriterion.scmm_constraint_exponent() != "")
        {
            fprintf(aFilePtr, "  aug_lag_constraint_exponent = %s\n", aCriterion.scmm_constraint_exponent().c_str());
        }
        if(aCriterion.stressLimit() != "")
        {
            fprintf(aFilePtr, "  stress_limit_value = %s\n", aCriterion.stressLimit().c_str());
        }
        if(aCriterion.relative_stress_limit() != "")
        {
            fprintf(aFilePtr, "  relative_stress_limit = %s\n", aCriterion.relative_stress_limit().c_str());
        }
        if(aCriterion.pnormExponent() != "")
        {
            fprintf(aFilePtr, "  stress_p_norm_power = %s\n", aCriterion.pnormExponent().c_str());
        }
        if(aCriterion.relaxed_stress_ramp_factor() != "")
        {
            fprintf(aFilePtr, "  relaxed_stress_ramp_factor = %s\n", aCriterion.relaxed_stress_ramp_factor().c_str());
        }
        if(aCriterion.limit_power_min() != "")
        {
            fprintf(aFilePtr, "  limit_min = %s\n", aCriterion.limit_power_min().c_str());
        }
        if(aCriterion.limit_power_max() != "")
        {
            fprintf(aFilePtr, "  limit_max = %s\n", aCriterion.limit_power_max().c_str());
        }
        if(aCriterion.limit_power_feasible_bias() != "")
        {
            fprintf(aFilePtr, "  limit_feasible_bias = %s\n", aCriterion.limit_power_feasible_bias().c_str());
        }
        if(aCriterion.limit_power_feasible_slope() != "")
        {
            fprintf(aFilePtr, "  limit_feasible_slope = %s\n", aCriterion.limit_power_feasible_slope().c_str());
        }
        if(aCriterion.limit_power_infeasible_bias() != "")
        {
            fprintf(aFilePtr, "  limit_infeasible_bias = %s\n", aCriterion.limit_power_infeasible_bias().c_str());
        }
        if(aCriterion.limit_power_infeasible_slope() != "")
        {
            fprintf(aFilePtr, "  limit_infeasible_slope = %s\n", aCriterion.limit_power_infeasible_slope().c_str());
        }
        if(aCriterion.limit_reset_subfrequency() != "")
        {
            fprintf(aFilePtr, "  limit_reset_subfrequency = %s\n", aCriterion.limit_reset_subfrequency().c_str());
        }
        if(aCriterion.limit_reset_count() != "")
        {
            fprintf(aFilePtr, "  limit_reset_count = %s\n", aCriterion.limit_reset_count().c_str());
        }
        if(aCriterion.inequality_allowable_feasibility_upper() != "")
        {
            fprintf(aFilePtr, "  stress_ineq_upper = %s\n", aCriterion.inequality_allowable_feasibility_upper().c_str());
        }
        if(aCriterion.inequality_feasibility_scale() != "")
        {
            fprintf(aFilePtr, "  inequality_feasibility_scale = %s\n", aCriterion.inequality_feasibility_scale().c_str());
        }
        if(aCriterion.inequality_infeasibility_scale() != "")
        {
            fprintf(aFilePtr, "  inequality_infeasibility_scale = %s\n", aCriterion.inequality_infeasibility_scale().c_str());
        }
        if(aCriterion.stress_inequality_power() != "")
        {
            fprintf(aFilePtr, "  stress_inequality_power = %s\n", aCriterion.stress_inequality_power().c_str());
        }
        if(aCriterion.stress_favor_final() != "")
        {
            fprintf(aFilePtr, "  stress_favor_final = %s\n", aCriterion.stress_favor_final().c_str());
        }
        if(aCriterion.stress_favor_updates() != "")
        {
            fprintf(aFilePtr, "  stress_favor_updates = %s\n", aCriterion.stress_favor_updates().c_str());
        }
        if(aCriterion.volume_penalty_power() != "")
        {
            fprintf(aFilePtr, "  volume_penalty_power = %s\n", aCriterion.volume_penalty_power().c_str());
        }
        if(aCriterion.volume_penalty_divisor() != "")
        {
            fprintf(aFilePtr, "  volume_penalty_divisor = %s\n", aCriterion.volume_penalty_divisor().c_str());
        }
        if(aCriterion.volume_penalty_bias() != "")
        {
            fprintf(aFilePtr, "  volume_penalty_bias = %s\n", aCriterion.volume_penalty_bias().c_str());
        }
    }
}
/**************************************************************************/
void append_case
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 FILE *aFilePtr)
{
    if(aCriterion.type() == "mechanical_compliance")
    {
        fprintf(aFilePtr, "  case = compliance_min\n");
    }
    else if(aCriterion.type() == "limit_stress")
    {
        fprintf(aFilePtr, "  case = stress_limit\n");
    }
    else if(aCriterion.type() == "stress_and_mass")
    {
        fprintf(aFilePtr, "  case = stress_constrained_mass_minimization\n");
    }
    else if(aCriterion.type() == "stress_p-norm")
    {
        fprintf(aFilePtr, "  case = stress_min\n");
    }
    else if(aCriterion.type() == "compliance_and_volume_min")
    {
        fprintf(aFilePtr, "  case = primary_compliance_secondary_volume\n");
    }
    else if(aCriterion.type() == "frf_mismatch")
    {
        fprintf(aFilePtr, "  case = inverse_methods\n");
        if(aMetaData.optimization_parameters().discretization() == "density")
            fprintf(aFilePtr, "  inverse_method_objective = directfrf-plato-density-method\n");
        else if(aMetaData.optimization_parameters().discretization() == "levelset")
            fprintf(aFilePtr, "  inverse_method_objective = directfrf-plato-levelset-method\n");
        fprintf(aFilePtr, "  ref_frf_file %s\n", aScenario.ref_frf_file().c_str());
        if(aScenario.frfMatchNodesetIDs().size() > 0)
        {
            fprintf(aFilePtr, "  frf_nodesets");
            for(auto tNodesetID : aScenario.frfMatchNodesetIDs())
            {
                fprintf(aFilePtr, " %s", tNodesetID.c_str());
            }
            fprintf(aFilePtr, "\n");
        }
        if(aScenario.complex_error_measure().length() > 0)
            fprintf(aFilePtr, "  complex_error_measure %s\n", aScenario.complex_error_measure().c_str());
        if (aScenario.convert_to_tet10().length()>0)
            fprintf(aFilePtr, " ls_tet_mesh_type %s\n", aScenario.convert_to_tet10().c_str());
    }
}
/**************************************************************************/
void append_normalization_parameter
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 FILE *aFilePtr)
{
    bool tNormalizeObjective = true;
    if(XMLGen::is_robust_optimization_problem(aMetaData))
    {
        tNormalizeObjective = false;
    }
    if(aCriterion.type() == "stress_and_mass")
    {
        tNormalizeObjective = false;
    }
    if(tNormalizeObjective == false)
    {
        fprintf(aFilePtr, "  objective_normalization false\n");
    }
}
/**************************************************************************/
void append_multi_load_case_data
(const XMLGen::InputData& aMetaData,
 FILE *aFilePtr)
{
    if(aMetaData.objective.multi_load_case == "true")
    {
        fprintf(aFilePtr, "  load_case_weights = ");
        for(auto tWeight : aMetaData.objective.weights)
        {
            fprintf(aFilePtr, "%s ", tWeight.c_str());
        }
        fprintf(aFilePtr, "\n");
    }
}
/**************************************************************************/
void append_topology_optimization_block
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 FILE *aFilePtr)
{
    fprintf(aFilePtr, "TOPOLOGY-OPTIMIZATION\n");
    fprintf(aFilePtr, "  algorithm = plato_engine\n");
    append_case(aMetaData, aCriterion, aScenario, aFilePtr);
    append_stress_parameters(aMetaData, aCriterion, aScenario, aFilePtr);
    if(aCriterion.type() == "frf_mismatch")
    {
        if(aMetaData.constraints.size() > 0)
        {
            auto tConstraintCriterionID = aMetaData.constraints[0].criterion();
            auto &tConstraintCriterion = aMetaData.criterion(tConstraintCriterionID);
            if(tConstraintCriterion.type() == "surface_area")
            {
                fprintf(aFilePtr, "  surface_area_constraint_value = %s\n", aMetaData.constraints[0].absoluteTarget().c_str());
                fprintf(aFilePtr, "  surface_area_ssid = %s\n", tConstraintCriterion.surface_area_sideset_id().c_str());
            }
        }

    }
    else
    {
        // This is a dummy place holder value and is simply here to tell Salinas that 
        // this is a single material optimization.
        fprintf(aFilePtr, "  volume_fraction = %f\n", .314);
    }
    append_multi_load_case_data(aMetaData, aFilePtr);
    append_normalization_parameter(aMetaData, aCriterion, aFilePtr);
    fprintf(aFilePtr, "END\n");
}
/**************************************************************************/
void append_file_block
(const XMLGen::InputData& aMetaData,
 FILE *aFilePtr)
{
    fprintf(aFilePtr, "FILE\n");
    fprintf(aFilePtr, "  geometry_file '%s'\n", aMetaData.mesh.run_name.c_str());
    fprintf(aFilePtr, "END\n");
}
/**************************************************************************/
void append_load
(const XMLGen::NaturalBoundaryCondition& aLoad,
 FILE *aFilePtr)
{
    if(aLoad.type() == "acceleration")
    {
        auto &tLoadValues = aLoad.load_values(); 
        fprintf(aFilePtr, "  body gravity %s %s %s scale 1.0\n",
                tLoadValues[0].c_str(),
                tLoadValues[1].c_str(),
                tLoadValues[2].c_str());
    }
    else if(aLoad.type() == "pressure")
    {
        auto &tLoadValues = aLoad.load_values(); 
        fprintf(aFilePtr, "  %s %s %s %s\n",
                aLoad.location_type().c_str(),
                aLoad.location_id().c_str(),
                aLoad.type().c_str(),
                tLoadValues[0].c_str());
    }
    else
    {
        auto &tLoadValues = aLoad.load_values(); 
        fprintf(aFilePtr, "  %s %s %s %s %s %s scale 1.0\n",
                aLoad.location_type().c_str(),
                aLoad.location_id().c_str(),
                aLoad.type().c_str(),
                tLoadValues[0].c_str(),
                tLoadValues[1].c_str(),
                tLoadValues[2].c_str());
    }
}
/**************************************************************************/
void append_loads_block
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion& aCriterion,
 const XMLGen::Scenario& aScenario,
 FILE *aFilePtr)
{
    if(aMetaData.objective.multi_load_case == "true")
    {
        for(auto tScenarioID : aMetaData.objective.scenarioIDs)
        {
            fprintf(aFilePtr, "LOAD=%s\n", tScenarioID.c_str());
            for(auto &tLoad : aMetaData.scenarioLoads(tScenarioID))
            {
                append_load(tLoad, aFilePtr);
            }
            fprintf(aFilePtr, "END\n");
        }
    }
    else
    {
        fprintf(aFilePtr, "LOADS\n");
        for(auto &tLoad : aMetaData.scenarioLoads(aScenario.id()))
        {
            append_load(tLoad, aFilePtr);
        }
        if(aCriterion.type() == "frf_mismatch")
        {
            fprintf(aFilePtr, "  function=1\n");
        }
        fprintf(aFilePtr, "END\n");
    }
}
/**************************************************************************/
std::string convertDOF
(const std::string &aDOFIn)
{
    std::string tReturn = aDOFIn;
    if(aDOFIn == "dispx")
    {
        tReturn = "x";
    }
    else if(aDOFIn == "dispy")
    {
        tReturn = "y";
    }
    else if(aDOFIn == "dispz")
    {
        tReturn = "z";
    }
    return tReturn;
}
/**************************************************************************/
void append_boundary_block
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion& aCriterion,
 const XMLGen::Scenario& aScenario,
 FILE *aFilePtr)
{
    fprintf(aFilePtr, "BOUNDARY\n");
    for(auto tBCID : aScenario.bcIDs())
    {
        bool found = false;
        XMLGen::EssentialBoundaryCondition tCurEBC;
        for(auto &tBC : aMetaData.ebcs)
        {
            if(tBC.id() == tBCID)
            {
                tCurEBC = tBC;
                found = true;
            }
        }
        if(found)
        {
            if(tCurEBC.degree_of_freedom().empty())
            {
                fprintf(aFilePtr, "  %s %s fixed\n",
                        tCurEBC.location_type().c_str(),
                        tCurEBC.location_id().c_str());
            }
            else
            {
                std::string tDOF = convertDOF(tCurEBC.degree_of_freedom());
                if(tCurEBC.dof_value().empty())
                {
                    fprintf(aFilePtr, "  %s %s %s 0\n",
                            tCurEBC.location_type().c_str(),
                            tCurEBC.location_id().c_str(),
                            tDOF.c_str());
                }
                else
                {
                    fprintf(aFilePtr, "  %s %s %s %s\n",
                            tCurEBC.location_type().c_str(),
                            tCurEBC.location_id().c_str(),
                            tDOF.c_str(),
                            tCurEBC.dof_value().c_str());
                }
            }
        }
    }
    fprintf(aFilePtr, "END\n");
}
/**************************************************************************/
void add_input_deck_blocks
(const XMLGen::InputData& aMetaData,
 FILE *aFilePtr)
{
    if(aMetaData.objective.serviceIDs.size() > 0)
    {
        auto tServiceID = aMetaData.objective.serviceIDs[0];
        auto &tService = aMetaData.service(tServiceID);
        if(tService.code() == "sierra_sd")
        {
            auto tScenarioID = aMetaData.objective.scenarioIDs[0];
            auto &tScenario = aMetaData.scenario(tScenarioID);
            auto tCriterionID = aMetaData.objective.criteriaIDs[0];
            auto &tCriterion = aMetaData.criterion(tCriterionID);
            append_solution_block(aMetaData, aFilePtr);
            append_parameters_block(tScenario, aFilePtr);
            append_frf_related_blocks(aMetaData, tCriterion, tScenario, aFilePtr);
            append_gdsw_block(tCriterion, tScenario, aFilePtr);
            append_outputs_block(tCriterion, aFilePtr);
            append_echo_block(tCriterion, aFilePtr);
            append_material_blocks(aMetaData, tCriterion, tScenario, aFilePtr);
            append_block_blocks(aMetaData, tCriterion, tScenario, aFilePtr);
            append_topology_optimization_block(aMetaData, tCriterion, tScenario, aFilePtr);
            append_file_block(aMetaData, aFilePtr);
            append_loads_block(aMetaData, tCriterion, tScenario, aFilePtr);
            append_boundary_block(aMetaData, tCriterion, tScenario, aFilePtr);
        }
    }
}
/******************************************************************************/
void write_sierra_sd_input_deck
(const XMLGen::InputData& aXMLMetaData)
{
    std::string tServiceID = get_salinas_service_id(aXMLMetaData);
    std::string tFilename = std::string("sierra_sd_") + tServiceID + "_input_deck.i";
    FILE *tFilePtr=fopen(tFilename.c_str(), "w");
    if(tFilePtr)
    {
        add_input_deck_blocks(aXMLMetaData, tFilePtr);
        fclose(tFilePtr);
    }
}
/**************************************************************************/


}
// namespace XMLGen
