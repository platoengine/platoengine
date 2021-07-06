/*
 * XMLGeneratorSierraSDOperationsFileUtilities.cpp
 *
 *  Created on: Feb 6, 2021
 */

#include <algorithm>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorSierraSDOperationsFileUtilities.hpp"

namespace XMLGen
{

bool isModalCriterion(const XMLGen::Criterion &aCriterion) {
    return aCriterion.type() == "modal_matching" ||
           aCriterion.type() == "modal_projection_error";
}

bool isInverseMethodCase(const XMLGen::Criterion &aCriterion) {
    return aCriterion.type() == "frf_mismatch" || isModalCriterion(aCriterion);
}

/**************************************************************************/
void append_solution_block(const XMLGen::InputData &aMetaData,
                           const XMLGen::Criterion &aCriterion,
                           std::ostream &outfile) {
    outfile << "SOLUTION" << std::endl;
    if(aMetaData.objective.multi_load_case == "true")
    {
        for(auto tScenarioID : aMetaData.objective.scenarioIDs)
        {
            outfile << "  case '" << tScenarioID << "'" << std::endl;
            outfile << "    topology_optimization" << std::endl;
            outfile << "    load=" << tScenarioID << std::endl;
        }
    }
    else
    {
        outfile << "  case '" << aMetaData.objective.scenarioIDs[0] << "'" << std::endl;
        outfile << "  topology_optimization" << std::endl;
    }
    if (isModalCriterion(aCriterion)) {
        outfile << "  nmodes " << aCriterion.num_modes_compute() << std::endl;
        outfile << "  modalAdjointSolver = camp" << std::endl;
        outfile << "  shift " << aCriterion.eigen_solver_shift() << std::endl;
    }
    outfile << "  solver gdsw" << std::endl;
    outfile << "END" << std::endl;
}
/**************************************************************************/
void append_parameters_block(const XMLGen::Scenario &aScenario,
                             const XMLGen::Criterion &aCriterion,
                             std::ostream &outfile) {
    auto tWeightMassScaleFactor = aScenario.weightMassScaleFactor();
    const bool haveWtMass = (tWeightMassScaleFactor != "");
    const bool writeSwapModes = isModalCriterion(aCriterion);

    if(haveWtMass || writeSwapModes)
    {
        outfile << "PARAMETERS" << std::endl;
        if (haveWtMass) {
            outfile << "  WTMASS = " << tWeightMassScaleFactor << std::endl;
        }
        if (writeSwapModes) {
            outfile << "  swapModes no" << std::endl;
        }
        outfile << "END" << std::endl;
    }
}

void append_camp_block(const XMLGen::Scenario &aScenario, const XMLGen::Criterion &aCriterion, std::ostream &outfile) {
    if (isModalCriterion(aCriterion)) {
        outfile << "CAMP" << std::endl;
        outfile << "  solver_tol " << aCriterion.camp_solver_tol() << std::endl;
        outfile << "  preconditioner gdsw" << std::endl;
        outfile << "  max_iter " << aCriterion.camp_max_iter() << std::endl;
        outfile << "END" << std::endl;
    }
}

void writeOptimizationBlock(std::ostream &outfile)
{
    outfile << "OPTIMIZATION" << std::endl;
    outfile << "  optimization_package ROL_lib" << std::endl;
    outfile << "  ROLmethod linesearch" << std::endl;
    outfile << "  LSstep Newton-Krylov" << std::endl;
    outfile << "  LS_curvature_condition null" << std::endl;
    outfile << "  Max_iter_Krylov 50" << std::endl;
    outfile << "  Use_FD_hessvec false" << std::endl;
    outfile << "  Use_inexact_hessvec false" << std::endl;
    outfile << "END" << std::endl;
}

void writeDummyFRFFiles(const XMLGen::Scenario &aScenario, std::ostream &outfile) {
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
    outfile << "  data_truth_table " << tTruthTableFile << std::endl;
    outfile << "  real_data_file " << tRealDataFile << std::endl;
    outfile << "  imaginary_data_file " << tImagDataFile << std::endl;
    // Create the 3 generic files.
    double tFreqMin, tFreqMax, tFreqStep;
    sscanf(aScenario.frequency_min().c_str(), "%lf", &tFreqMin);
    sscanf(aScenario.frequency_max().c_str(), "%lf", &tFreqMax);
    sscanf(aScenario.frequency_step().c_str(), "%lf", &tFreqStep);
    // This is the formula sierra_sd uses to get the number of frequencies
    int tNumFreqs = (int)(((tFreqMax - tFreqMin) / tFreqStep) + 0.5) + 1;
    int tNumMatchNodes = aScenario.matchNodesetIDs().size();
    FILE *tTmpFP = fopen(tTruthTableFile.c_str(), "w");
    if (tTmpFP) {
        fprintf(tTmpFP, "%d\n", tNumMatchNodes);
        for (int tIndex = 0; tIndex < tNumMatchNodes; ++tIndex) {
            fprintf(tTmpFP, "%d 1 1 1\n", tIndex + 1);
        }
        fclose(tTmpFP);
    }
    tTmpFP = fopen(tRealDataFile.c_str(), "w");
    if (tTmpFP) {
        fprintf(tTmpFP, "%d %d\n", 3 * tNumMatchNodes, tNumFreqs);
        for (int tIndex = 0; tIndex < 3 * tNumMatchNodes; ++tIndex) {
            for (int tIndex2 = 0; tIndex2 < tNumFreqs; ++tIndex2) {
                fprintf(tTmpFP, "0 ");
            }
            fprintf(tTmpFP, "\n");
        }
        fclose(tTmpFP);
    }
    tTmpFP = fopen(tImagDataFile.c_str(), "w");
    if (tTmpFP) {
        fprintf(tTmpFP, "%d %d\n", 3 * tNumMatchNodes, tNumFreqs);
        for (int tIndex = 0; tIndex < 3 * tNumMatchNodes; ++tIndex) {
            for (int tIndex2 = 0; tIndex2 < tNumFreqs; ++tIndex2) {
                fprintf(tTmpFP, "0 ");
            }
            fprintf(tTmpFP, "\n");
        }
        fclose(tTmpFP);
    }
}

void writeFRFInverseBlocks(const XMLGen::InputData &aMetaData,
                           const XMLGen::Criterion &aCriterion,
                           const XMLGen::Scenario &aScenario,
                           std::ostream &outfile)
{
    outfile << "INVERSE-PROBLEM" << std::endl;
    if (aMetaData.optimization_parameters().discretization().compare("levelset") == 0) {
        writeDummyFRFFiles(aScenario, outfile);
    } else {
        outfile << "  data_truth_table ttable.txt" << std::endl;
        outfile << "  real_data_file data.txt" << std::endl;
        outfile << "  imaginary_data_file data_im.txt" << std::endl;
    }
    outfile << "END" << std::endl;

    writeOptimizationBlock(outfile);

    if (aScenario.raleigh_damping_alpha().length() > 0 && aScenario.raleigh_damping_beta().length() > 0) {
        outfile << "DAMPING" << std::endl;
        outfile << "  alpha " << aScenario.raleigh_damping_alpha() << std::endl;
        outfile << "  beta " << aScenario.raleigh_damping_beta() << std::endl;
        outfile << "END" << std::endl;
    }
    outfile << "FREQUENCY" << std::endl;
    outfile << "  freq_min " << aScenario.frequency_min() << std::endl;
    outfile << "  freq_max " << aScenario.frequency_max() << std::endl;
    outfile << "  freq_step " << aScenario.frequency_step() << std::endl;
    outfile << "END" << std::endl;

    outfile << "FUNCTION 1" << std::endl;
    outfile << "  type linear" << std::endl;
    outfile << "  data 0 1" << std::endl;
    outfile << "  data 1e6 1" << std::endl;
    outfile << "END" << std::endl;
}

void writeDummyModalFiles(const XMLGen::Scenario &aScenario, const XMLGen::Criterion &aCriterion, std::ostream &outfile) {
    const std::string id = aScenario.id();

    std::string data_truth_table = "dummy_eigen_ttable_";
    data_truth_table += aScenario.id();
    data_truth_table += ".txt";
    std::string data_file = "dummy_eigen_data_";
    data_file += aScenario.id();
    data_file += ".txt";
    std::string modal_data_file = "dummy_modal_data_";
    modal_data_file += aScenario.id();
    modal_data_file += ".txt";
    std::string modal_weight_table = "dummy_modal_weight_";
    modal_weight_table += aScenario.id();
    modal_weight_table += ".txt";

    outfile << "  data_truth_table " << data_truth_table << std::endl;
    outfile << "  data_file " << data_file << std::endl;
    outfile << "  modal_data_file " << modal_data_file << std::endl;
    outfile << "  modal_weight_table " << modal_weight_table << std::endl;

    const auto &matchNodes = aScenario.matchNodesetIDs();
    const int numNodes = matchNodes.size();
    int numModes;
    sscanf(aCriterion.num_modes_compute().c_str(), "%d", &numModes);

    // Create the 4 generic files.

    std::ofstream dummy;

    dummy.open(data_truth_table);
    dummy << numNodes << " 4" << std::endl;
    for(int i=0;i<numNodes;i++) {
        dummy << matchNodes[i] << " 1 1 1" << std::endl;
    }
    dummy.close();

    dummy.open(data_file);
    dummy << 3*numNodes << " " << numModes << std::endl;
    for(int i=0;i<3*numNodes;i++) {
        for(int j=0;j<numModes;j++) {
            dummy << "0 ";
        }
        dummy << std::endl;
    }
    dummy.close();

    dummy.open(modal_data_file);
    dummy << numModes << std::endl;
    for(int i=0;i<numNodes;i++) {
        dummy << "0" << std::endl;
    }
    dummy.close();

    dummy.open(modal_weight_table);
    dummy << numModes << std::endl;
    for(int i=0;i<numNodes;i++) {
        dummy << "1" << std::endl;
    }
    dummy.close();
}

void writeModalInverseBlocks(const XMLGen::InputData &aMetaData,
                             const XMLGen::Criterion &aCriterion,
                             const XMLGen::Scenario &aScenario,
                             std::ostream &outfile)
{
    outfile << "INVERSE-PROBLEM" << std::endl;

    writeDummyModalFiles(aScenario, aCriterion, outfile);

    outfile << "  shape_sideset " << aScenario.shapeSideset() << std::endl;

    // dummy value doesn't matter because SD just computes the gradient
    outfile << "  shape_bounds 1.0" << std::endl;
 
    if (aCriterion.type() == "modal_projection_error") {
        outfile << "  eigen_objective mpe" << std::endl;
    }
    outfile << "  design_variable shape" << std::endl;
    outfile << "END" << std::endl;

    writeOptimizationBlock(outfile);
}

/**************************************************************************/
void append_inverse_problem_blocks
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 std::ostream &outfile)
{
    if(aCriterion.type() == "frf_mismatch")
    {
        writeFRFInverseBlocks(aMetaData, aCriterion, aScenario, outfile);
    }
    else if (isModalCriterion(aCriterion))
    {
        writeModalInverseBlocks(aMetaData, aCriterion, aScenario, outfile);
    }
}
/**************************************************************************/
void append_gdsw_block
(const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 std::ostream &outfile)
{
    bool isDifficultForSolver = false;
    if(aCriterion.type() == "limit_stress")
        isDifficultForSolver = true;
    else if(aCriterion.type() == "stress_p-norm")
        isDifficultForSolver = true;
    else if(aCriterion.type() == "stress_and_mass")
        isDifficultForSolver = true;
    outfile << "GDSW" << std::endl;
    const bool haveSolverTolerance = (aScenario.solverTolerance() != "");
    if(haveSolverTolerance)
    {
        outfile << "  solver_tol = " << aScenario.solverTolerance() << std::endl;
    }
    if(isDifficultForSolver)
    {
        outfile << "  diag_scaling diagonal" << std::endl;
        outfile << "  krylov_method = GMRESClassic" << std::endl;
        outfile << "  stag_tol = 0.001" << std::endl;
        outfile << "  orthog = 0" << std::endl;
        outfile << "  num_GS_steps = 2" << std::endl;
        outfile << "  overlap = 4" << std::endl;
        outfile << "  bailout true" << std::endl;
        if(!haveSolverTolerance)
        {
            outfile << "  solver_tol = 1e-4" << std::endl;
        }
    }
    if (isModalCriterion(aCriterion)) {
        outfile << "  SC_option 0" << std::endl;
    }
    outfile << "END" << std::endl;
}
/**************************************************************************/
void append_outputs_block
(const XMLGen::Criterion &aCriterion,
 std::ostream &outfile)
{
    outfile << "OUTPUTS" << std::endl;
    if(!isInverseMethodCase(aCriterion))
    {
        outfile << "  topology" << std::endl;
    }
    outfile << "END" << std::endl;
}
/**************************************************************************/
void append_echo_block
(const XMLGen::Criterion &aCriterion,
 std::ostream &outfile)
{
    outfile << "ECHO" << std::endl;
    if(!isInverseMethodCase(aCriterion))
    {
        outfile << "  topology" << std::endl;
    }
    outfile << "END" << std::endl;
}

void append_penalty_terms_to_material_block(const XMLGen::InputData &aMetaData,
                                            const XMLGen::Criterion &aCriterion,
                                            const XMLGen::Scenario &aScenario,
                                            std::ostream &outfile) {
    outfile << "  material_penalty_model = simp" << std::endl;
    if (aMetaData.optimization_parameters().discretization().compare("density") == 0) {
        if (aScenario.materialPenaltyExponent().length() > 0)
            outfile << "  penalty_coefficient = " << aScenario.materialPenaltyExponent() << std::endl;
    }
    if (aCriterion.type() == "frf_mismatch") {
        outfile << "  minimum_stiffness_penalty_value=1e-3" << std::endl;
    }
}

/**************************************************************************/
void append_material_blocks
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 std::ostream &outfile)
{
    for(auto &tMaterial : aMetaData.materials)
    {
        outfile << "MATERIAL " << tMaterial.id() << std::endl;
        outfile << "  isotropic" << std::endl;
        outfile << "  E = " << tMaterial.property("youngs_modulus") << std::endl;
        outfile << "  nu = " << tMaterial.property("poissons_ratio") << std::endl;
        if(tMaterial.property("mass_density").empty() == false)
            outfile << "  density = " << tMaterial.property("mass_density") << std::endl;
        if (!isModalCriterion(aCriterion)) {
            append_penalty_terms_to_material_block(aMetaData, aCriterion, aScenario, outfile);
        }
        outfile << "END" << std::endl;
    }
}
/**************************************************************************/
void append_block_blocks
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 std::ostream &outfile)
{
    for(auto &tBlock : aMetaData.blocks)
    {
        if(tBlock.block_id.empty() == false)
        {
            outfile << "BLOCK " << tBlock.block_id << std::endl;

        }
        if(tBlock.material_id.empty() == false)
        {
            outfile << "  material " << tBlock.material_id << std::endl;

        }
        if(tBlock.element_type == "hex8")
        {
            outfile << "  hex8u" << std::endl; 
        }
        else if(tBlock.element_type == "tet4")
        {
            outfile << "  tet4" << std::endl;
        }
        else if(tBlock.element_type == "tet10")
        {
            if (isInverseMethodCase(aCriterion)) {
                outfile << "  cutet10" << std::endl;
            }
            else {
                outfile << "  tet10" << std::endl;
            }
        }
        else if(tBlock.element_type == "rbar")
        {
            outfile << "  rbar" << std::endl;
        }
        else if(tBlock.element_type == "rbe3")
        {
            outfile << "  rbe3" << std::endl;
        }
        if(aCriterion.type() == "frf_mismatch")
        {
            outfile << "  inverse_material_type homogeneous" << std::endl;
        }
        outfile << "END" << std::endl;
    }
}
/**************************************************************************/
void append_stress_parameters
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 std::ostream &outfile)
{
    if(aCriterion.type() == "stress_and_mass" ||
       aCriterion.type() == "limit_stress" ||
       aCriterion.type() == "stress_p-norm")
    {
        if(aCriterion.volume_misfit_target() != "")
        {
            outfile << "  volume_misfit_target = " << aCriterion.volume_misfit_target() << std::endl;

        }
        if(aCriterion.scmmInitialPenalty() != "")
        {
            outfile << "  aug_lag_penalty_initial_value = " << aCriterion.scmmInitialPenalty() << std::endl;

        }
        if(aCriterion.scmmPenaltyExpansionMultiplier() != "")
        {
            outfile << "  aug_lag_penalty_expansion_factor = " << aCriterion.scmmPenaltyExpansionMultiplier() << std::endl;

        }
        if(aCriterion.scmm_constraint_exponent() != "")
        {
            outfile << "  aug_lag_constraint_exponent = " << aCriterion.scmm_constraint_exponent() << std::endl;

        }
        if(aCriterion.stressLimit() != "")
        {
            outfile << "  stress_limit_value = " << aCriterion.stressLimit() << std::endl;

        }
        if(aCriterion.relative_stress_limit() != "")
        {
            outfile << "  relative_stress_limit = " << aCriterion.relative_stress_limit() << std::endl;

        }
        if(aCriterion.pnormExponent() != "")
        {
            outfile << "  stress_p_norm_power = " << aCriterion.pnormExponent() << std::endl;

        }
        if(aCriterion.relaxed_stress_ramp_factor() != "")
        {
            outfile << "  relaxed_stress_ramp_factor = " << aCriterion.relaxed_stress_ramp_factor() << std::endl;

        }
        if(aCriterion.limit_power_min() != "")
        {
            outfile << "  limit_min = " << aCriterion.limit_power_min() << std::endl;

        }
        if(aCriterion.limit_power_max() != "")
        {
            outfile << "  limit_max = " << aCriterion.limit_power_max() << std::endl;

        }
        if(aCriterion.limit_power_feasible_bias() != "")
        {
            outfile << "  limit_feasible_bias = " << aCriterion.limit_power_feasible_bias() << std::endl;

        }
        if(aCriterion.limit_power_feasible_slope() != "")
        {
            outfile << "  limit_feasible_slope = " << aCriterion.limit_power_feasible_slope() << std::endl;

        }
        if(aCriterion.limit_power_infeasible_bias() != "")
        {
            outfile << "  limit_infeasible_bias = " << aCriterion.limit_power_infeasible_bias() << std::endl;

        }
        if(aCriterion.limit_power_infeasible_slope() != "")
        {
            outfile << "  limit_infeasible_slope = " << aCriterion.limit_power_infeasible_slope() << std::endl;

        }
        if(aCriterion.limit_reset_subfrequency() != "")
        {
            outfile << "  limit_reset_subfrequency = " << aCriterion.limit_reset_subfrequency() << std::endl;

        }
        if(aCriterion.limit_reset_count() != "")
        {
            outfile << "  limit_reset_count = " << aCriterion.limit_reset_count() << std::endl;

        }
        if(aCriterion.inequality_allowable_feasibility_upper() != "")
        {
            outfile << "  stress_ineq_upper = " << aCriterion.inequality_allowable_feasibility_upper() << std::endl;

        }
        if(aCriterion.inequality_feasibility_scale() != "")
        {
            outfile << "  inequality_feasibility_scale = " << aCriterion.inequality_feasibility_scale() << std::endl;

        }
        if(aCriterion.inequality_infeasibility_scale() != "")
        {
            outfile << "  inequality_infeasibility_scale = " << aCriterion.inequality_infeasibility_scale() << std::endl;

        }
        if(aCriterion.stress_inequality_power() != "")
        {
            outfile << "  stress_inequality_power = " << aCriterion.stress_inequality_power() << std::endl;

        }
        if(aCriterion.stress_favor_final() != "")
        {
            outfile << "  stress_favor_final = " << aCriterion.stress_favor_final() << std::endl;

        }
        if(aCriterion.stress_favor_updates() != "")
        {
            outfile << "  stress_favor_updates = " << aCriterion.stress_favor_updates() << std::endl;

        }
        if(aCriterion.volume_penalty_power() != "")
        {
            outfile << "  volume_penalty_power = " << aCriterion.volume_penalty_power() << std::endl;

        }
        if(aCriterion.volume_penalty_divisor() != "")
        {
            outfile << "  volume_penalty_divisor = " << aCriterion.volume_penalty_divisor() << std::endl;

        }
        if(aCriterion.volume_penalty_bias() != "")
        {
            outfile << "  volume_penalty_bias = " << aCriterion.volume_penalty_bias() << std::endl;

        }
    }
}

void writeInverseMethodObjective(const std::string &tDiscretization, const XMLGen::Criterion &aCriterion, std::ostream &outfile) {
    if (aCriterion.type() == "frf_mismatch") {
        if(tDiscretization == "density")
            outfile << "  inverse_method_objective = directfrf-plato-density-method" << std::endl;
        else if(tDiscretization == "levelset")
            outfile << "  inverse_method_objective = directfrf-plato-levelset-method" << std::endl;
    }
    else if (isModalCriterion(aCriterion)) {
        outfile << "  inverse_method_objective = eigen-inverse" << std::endl;
    }
}

/**************************************************************************/
void append_case
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 std::ostream &outfile)
{
    if(aCriterion.type() == "mechanical_compliance")
    {
        outfile << "  case = compliance_min" << std::endl;
    }
    else if(aCriterion.type() == "limit_stress")
    {
        outfile << "  case = stress_limit" << std::endl;
    }
    else if(aCriterion.type() == "stress_and_mass")
    {
        outfile << "  case = stress_constrained_mass_minimization" << std::endl;
    }
    else if(aCriterion.type() == "stress_p-norm")
    {
        outfile << "  case = stress_min" << std::endl;
    }
    else if(aCriterion.type() == "compliance_and_volume_min")
    {
        outfile << "  case = primary_compliance_secondary_volume" << std::endl;
    }
    else if(isInverseMethodCase(aCriterion))
    {
        outfile << "  case = inverse_methods" << std::endl;

        std::string tDiscretization = aMetaData.optimization_parameters().discretization();
        writeInverseMethodObjective(tDiscretization, aCriterion, outfile);

        outfile << "  ref_data_file " << aScenario.ref_data_file() << std::endl;
        if(aScenario.matchNodesetIDs().size() > 0)
        {
            outfile << "  match_nodesets";
            for(auto tNodesetID : aScenario.matchNodesetIDs())
            {
                outfile << " " << tNodesetID;
            }
            outfile << std::endl;
        }
        if (aCriterion.modesToExclude().size() > 0) {
            outfile << "  modes_to_exclude";
            for(auto tModeID : aCriterion.modesToExclude()) {
                outfile << " " << tModeID;
            }
            outfile << std::endl;
        }

        if (aCriterion.type() == "frf_mismatch") {
            if (aScenario.complex_error_measure().length() > 0)
                outfile << "  complex_error_measure " << aScenario.complex_error_measure() << std::endl;
        }

        if (aScenario.convert_to_tet10().length()>0)
            outfile << " ls_tet_mesh_type " << aScenario.convert_to_tet10() << std::endl;
    }
}
/**************************************************************************/
void append_normalization_parameter
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 std::ostream &outfile)
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
    if (isModalCriterion(aCriterion)) {
        tNormalizeObjective = false;
    }
    if(tNormalizeObjective == false)
    {
        outfile << "  objective_normalization false" << std::endl;
    }
}
/**************************************************************************/
void append_multi_load_case_data
(const XMLGen::InputData& aMetaData,
 std::ostream &outfile)
{
    if(aMetaData.objective.multi_load_case == "true")
    {
        outfile << "  load_case_weights = ";
        for(auto tWeight : aMetaData.objective.weights)
        {
            outfile << tWeight << " " << std::endl;

        }
        outfile << "" << std::endl;
    }
}
/**************************************************************************/
void append_topology_optimization_block
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion &aCriterion,
 const XMLGen::Scenario &aScenario,
 std::ostream &outfile)
{
    outfile << "TOPOLOGY-OPTIMIZATION" << std::endl;
    outfile << "  algorithm = plato_engine" << std::endl;
    append_case(aMetaData, aCriterion, aScenario, outfile);
    append_stress_parameters(aMetaData, aCriterion, aScenario, outfile);
    if(aCriterion.type() == "frf_mismatch")
    {
        if(aMetaData.constraints.size() > 0)
        {
            auto tConstraintCriterionID = aMetaData.constraints[0].criterion();
            auto &tConstraintCriterion = aMetaData.criterion(tConstraintCriterionID);
            if(tConstraintCriterion.type() == "surface_area")
            {
                outfile << "  surface_area_constraint_value = " << aMetaData.constraints[0].absoluteTarget() << std::endl;
    
                outfile << "  surface_area_ssid = " << tConstraintCriterion.surface_area_sideset_id() << std::endl;
    
            }
        }

    }
    else
    {
        // This is a dummy place holder value and is simply here to tell Salinas that 
        // this is a single material optimization.
        outfile << "  volume_fraction = .314" << std::endl;
    }
    append_multi_load_case_data(aMetaData, outfile);
    append_normalization_parameter(aMetaData, aCriterion, outfile);
    outfile << "END" << std::endl;
}
/**************************************************************************/
void append_file_block
(const XMLGen::InputData& aMetaData,
 std::ostream &outfile)
{
    outfile << "FILE" << std::endl;
    outfile << "  geometry_file '" << aMetaData.mesh.run_name << "'"  << std::endl;
    outfile << "END" << std::endl;
}
/**************************************************************************/
void append_load
(const XMLGen::Load& aLoad,
 std::ostream &outfile)
{
    if(aLoad.type() == "acceleration")
    {
        auto &tLoadValues = aLoad.load_values(); 
        outfile << "  body gravity " << tLoadValues[0] << " " << tLoadValues[1] << " " << tLoadValues[2] << " scale 1.0" << std::endl;
    }
    else if(aLoad.type() == "pressure")
    {
        auto &tLoadValues = aLoad.load_values();     
        outfile << "  " << aLoad.location_type() << " " << aLoad.location_id() << " " << aLoad.type() << " " << tLoadValues[0] << std::endl;
    }
    else
    {
        auto &tLoadValues = aLoad.load_values(); 
        outfile << "  " << aLoad.location_type() << " " << aLoad.location_id() << " " << aLoad.type() << " " << tLoadValues[0] << " " << tLoadValues[1] << " " << tLoadValues[2] << " scale 1.0" << std::endl;
    }
}
/**************************************************************************/
void append_loads_block
(const XMLGen::InputData& aMetaData,
 const XMLGen::Criterion& aCriterion,
 const XMLGen::Scenario& aScenario,
 std::ostream &outfile)
{
    if(aMetaData.objective.multi_load_case == "true")
    {
        for(auto tScenarioID : aMetaData.objective.scenarioIDs)
        {
            outfile << "LOAD=" << tScenarioID << std::endl;

            for(auto &tLoad : aMetaData.scenarioLoads(tScenarioID))
            {
                append_load(tLoad, outfile);
            }
            outfile << "END" << std::endl;
        }
    }
    else
    {
        outfile << "LOADS" << std::endl;
        for(auto &tLoad : aMetaData.scenarioLoads(aScenario.id()))
        {
            append_load(tLoad, outfile);
        }
        if(aCriterion.type() == "frf_mismatch")
        {
            outfile << "  function=1" << std::endl;
        }
        outfile << "END" << std::endl;
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
 std::ostream &outfile)
{
    outfile << "BOUNDARY" << std::endl;
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
                outfile << "  " << tCurEBC.location_type() << " " << tCurEBC.location_id() << " fixed" << std::endl;
            }
            else
            {
                std::string tDOF = convertDOF(tCurEBC.degree_of_freedom());
                if(tCurEBC.dof_value().empty())
                {
                    outfile << "  " << tCurEBC.location_type() << " " << tCurEBC.location_id() << " " << tDOF << " 0" << std::endl;
                }
                else
                {
                    outfile << "  " << tCurEBC.location_type() << " " << tCurEBC.location_id() << " " << tDOF << " " << tCurEBC.dof_value() << std::endl;
                }
            }
        }
    }
    outfile << "END" << std::endl;
}
/**************************************************************************/
bool extractMetaDataForWritingInputDeck(const XMLGen::InputData &aMetaData,
                                        XMLGen::Service &tService,
                                        XMLGen::Scenario &tScenario,
                                        XMLGen::Criterion &tCriterion)
{
    if(aMetaData.objective.serviceIDs.size() > 0)
    {
        auto tServiceID = aMetaData.objective.serviceIDs[0];
        tService = aMetaData.service(tServiceID);
        if(tService.code() == "sierra_sd")
        {
            auto tScenarioID = aMetaData.objective.scenarioIDs[0];
            tScenario = aMetaData.scenario(tScenarioID);
            auto tCriterionID = aMetaData.objective.criteriaIDs[0];
            tCriterion = aMetaData.criterion(tCriterionID);
            return true;
        }
    }
    return false;
}

void add_input_deck_blocks
(const XMLGen::InputData& aMetaData,
 std::ostream &outfile)
{
    XMLGen::Service tService;
    XMLGen::Scenario tScenario;
    XMLGen::Criterion tCriterion;

    if (!extractMetaDataForWritingInputDeck(aMetaData, tService, tScenario, tCriterion)) {
        return;
    }

    append_solution_block(aMetaData, tCriterion, outfile);
    append_parameters_block(tScenario, tCriterion, outfile);
    append_camp_block(tScenario, tCriterion, outfile);
    append_inverse_problem_blocks(aMetaData, tCriterion, tScenario, outfile);
    append_gdsw_block(tCriterion, tScenario, outfile);
    append_outputs_block(tCriterion, outfile);
    append_echo_block(tCriterion, outfile);
    append_material_blocks(aMetaData, tCriterion, tScenario, outfile);
    append_block_blocks(aMetaData, tCriterion, tScenario, outfile);
    append_topology_optimization_block(aMetaData, tCriterion, tScenario, outfile);
    append_file_block(aMetaData, outfile);
    append_loads_block(aMetaData, tCriterion, tScenario, outfile);
    append_boundary_block(aMetaData, tCriterion, tScenario, outfile);
}
/******************************************************************************/

// sometimes, people generate Salinas input decks with DOS end-of-line characters - yikes!
bool getlineSafeForDOS(std::istream &input, std::string& line)
{
    bool ret = false;
    if (std::getline(input, line)) {
        ret = true;
        if (line.back() == '\r') {
            line.pop_back();
        }
    }
    return ret;
}

bool isEmptyLine(std::string line) {
    return std::count_if(line.begin(), line.end(),
    [](unsigned char c){
        return !std::isspace(c);}) == 0;
}

void makeLowerCase(std::string &str) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);    
}

std::string getFirstTokenInLine(std::string line) {
    std::istringstream stream(line);
    std::string firstToken;
    stream >> firstToken;
    return firstToken;
}

std::string lowerCaseFirstTokenInLine(std::string line) {
    std::string firstToken = getFirstTokenInLine(line);
    makeLowerCase(firstToken);
    return firstToken;
}

bool tokenMatches(std::string token, std::string str) {
    return (token.find(str) != std::string::npos);
}

bool loadBlockFromSDInputDeck(std::istream &inputDeck, std::vector<std::string> &blockLines) {
    blockLines.clear();
    std::string line;
    while (getlineSafeForDOS(inputDeck, line)) {
        if (isEmptyLine(line)) {
            continue;
        }

        std::string token = lowerCaseFirstTokenInLine(line);
        blockLines.push_back(line);
        if (tokenMatches(token, "end")) {
            return true;
        }
    }
    return false;
}

void printBlockExceptEndToken(const std::vector<std::string> &blockLines, std::ostream &outfile) {
    for(size_t i=0;i<blockLines.size()-1;i++) {
        outfile << blockLines[i] << std::endl;
    }
}

void printBlockEndToken(const std::vector<std::string> &blockLines, std::ostream &outfile) {
    const std::string &endToken = *(blockLines.end()-1);
    outfile << endToken << std::endl;
}

void replaceSolutionCases(const std::vector<std::string> &blockLines, std::ostream &outfile) {
    for(auto it = blockLines.begin(); it < blockLines.end(); it++) {
        const std::string &line = *it;
        outfile << line << std::endl;
        if (lowerCaseFirstTokenInLine(line) == "case") {
            outfile << "  topology_optimization" << std::endl;
            it++; 
        }
    }
}

void augment_sierra_sd_input_deck_with_plato_problem_description(const XMLGen::InputData &aXMLMetaData, std::istream &inputDeck, std::ostream &outfile) {
    XMLGen::Service tService;
    XMLGen::Scenario tScenario;
    XMLGen::Criterion tCriterion;

    if (!extractMetaDataForWritingInputDeck(aXMLMetaData, tService, tScenario, tCriterion)) {
        return;
    }

    std::vector<std::string> blockLines;
    while (loadBlockFromSDInputDeck(inputDeck, blockLines)) {
        std::string token = lowerCaseFirstTokenInLine(*blockLines.begin());

        if (tokenMatches(token, "solution")) {
            replaceSolutionCases(blockLines, outfile);
        }
        else if (tokenMatches(token, "material")) {
            printBlockExceptEndToken(blockLines, outfile);
            append_penalty_terms_to_material_block(aXMLMetaData, tCriterion, tScenario, outfile);
            printBlockEndToken(blockLines, outfile);
        }
        else if (tokenMatches(token, "outputs") || tokenMatches(token, "echo")) {
            printBlockExceptEndToken(blockLines, outfile);
            outfile << "  topology" << std::endl;
            printBlockEndToken(blockLines, outfile);
        }
        else {
            printBlockExceptEndToken(blockLines, outfile);
            printBlockEndToken(blockLines, outfile);
        }
    }

    append_topology_optimization_block(aXMLMetaData, tCriterion, tScenario, outfile);
}

void write_sierra_sd_input_deck
(const XMLGen::InputData& aXMLMetaData)
{
    std::string tServiceID = get_salinas_service_id(aXMLMetaData);
    std::string tFilename = std::string("sierra_sd_") + tServiceID + "_input_deck.i";
    std::ofstream outfile(tFilename, std::ofstream::out);

    std::string existingDeck = aXMLMetaData.service(tServiceID).existingInputDeck();

    struct stat buffer;
    const bool haveExistingDeck = !existingDeck.empty() && (stat(existingDeck.c_str(), &buffer) == 0);

    if (!haveExistingDeck) {
        add_input_deck_blocks(aXMLMetaData, outfile);
    } else {
        std::ifstream iDeck(existingDeck);
        augment_sierra_sd_input_deck_with_plato_problem_description(aXMLMetaData, iDeck, outfile);
    }
}
/**************************************************************************/


}
// namespace XMLGen
