/*
 * XMLGeneratorPlatoAnalyzeOperationsFile_UnitTester.cpp
 *
 *  Created on: Jun 2, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \fn is_plato_analyze_performer
 * \brief Returns true if Plato Analyze is used to compute physical quantities of \n
 * interest during optimization, i.e. Plato Analyze is an active performer.
 * \param [in] aPerformer physics app name, i.e. performer name
 * \return Flag
**********************************************************************************/
bool is_plato_analyze_performer(const std::string& aPerformer)
{
    auto tLowerKey = Plato::tolower(aPerformer);
    auto tIsPlatoAnalyze = tLowerKey.compare("plato_analyze") == 0;
    return (tIsPlatoAnalyze);
}

/******************************************************************************//**
 * \fn is_plato_analyze_performer
 * \brief Returns true if solving a topology optimization problem.
 * \param [in] aProblemType optimization problem type, i.e. shape, topology, inverse, others.
 * \return Flag
**********************************************************************************/
bool is_topology_optimization_problem(const std::string& aProblemType)
{
    auto tLowerKey = Plato::tolower(aProblemType);
    auto tIsTopologyOptimization = tLowerKey.compare("topology") == 0;
    return (tIsTopologyOptimization);
}

/******************************************************************************//**
 * \fn is_any_objective_computed_by_plato_analyze
 * \brief Returns true if any objective is computed by a Plato Analyze performer.
 * \param [in] aXMLMetaData Plato problem input data
 * \return Flag
**********************************************************************************/
bool is_any_objective_computed_by_plato_analyze
(const XMLGen::InputData& aXMLMetaData)
{
    auto tAtLeastOnePerformerIsPlatoAnalyze = false;
    for(auto& tObjective : aXMLMetaData.objectives)
    {
        if(XMLGen::is_plato_analyze_performer(tObjective.performer_name))
        {
            tAtLeastOnePerformerIsPlatoAnalyze = true;
            break;
        }
    }
    return (tAtLeastOnePerformerIsPlatoAnalyze);
}

/******************************************************************************//**
 * \fn is_any_constraint_computed_by_plato_analyze
 * \brief Returns true if any constraint is computed by a Plato Analyze performer.
 * \param [in] aXMLMetaData Plato problem input data
 * \return Flag
**********************************************************************************/
bool is_any_constraint_computed_by_plato_analyze
(const XMLGen::InputData& aXMLMetaData)
{
    auto tAtLeastOnePerformerIsPlatoAnalyze = false;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        if(XMLGen::is_plato_analyze_performer(tConstraint.mPerformerName))
        {
            tAtLeastOnePerformerIsPlatoAnalyze = true;
            break;
        }
    }
    return (tAtLeastOnePerformerIsPlatoAnalyze);
}

/******************************************************************************//**
 * \fn append_update_problem_to_plato_analyze_operation
 * \brief Append update problem operation to PUGI XML document. The update problem \n
 * operation is used to enable safe non-Plato parameter, e.g. physics-only parameters \n
 * updates during optimization.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_update_problem_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsUpdateFrequencyGreaterThanZero = std::stoi(aXMLMetaData.mProblemUpdateFrequency) > 0;
    if (!aXMLMetaData.mProblemUpdateFrequency.empty() && tIsUpdateFrequencyGreaterThanZero)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "UpdateProblem", "Update Problem" }, tOperation);
    }
}

/******************************************************************************//**
 * \fn append_compute_solution_to_plato_analyze_operation
 * \brief Append compute solution operation to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_compute_solution_to_plato_analyze_operation
(pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children({"Function", "Name"}, {"Compute Solution", "Compute Displacement Solution"}, tOperation);
    auto tInput = tOperation.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"Topology"}, tInput);
}

/******************************************************************************//**
 * \fn append_compute_objective_value_to_plato_analyze_operation
 * \brief Append compute objective value operation to PUGI XML document. The \n
 * operation is only appended if Plato Analyze is responsible for its evaluation.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_compute_objective_value_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsTopologyOptimization = XMLGen::is_topology_optimization_problem(aXMLMetaData.optimization_type);
    auto tIsPlatoAnalyzePerformer = XMLGen::is_any_objective_computed_by_plato_analyze(aXMLMetaData);
    auto tAppendComputeObjectiveValueOperation = tIsTopologyOptimization && tIsPlatoAnalyzePerformer;

    if(tAppendComputeObjectiveValueOperation)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "ComputeObjectiveValue", "Compute Objective Value" }, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children( { "ArgumentName" }, { "Objective Value" }, tOutput);
    }
}

/******************************************************************************//**
 * \fn append_compute_objective_gradient_to_plato_analyze_operation
 * \brief Append compute objective gradient operation to PUGI XML document. The \n
 * operation is only appended if Plato Analyze is responsible for its evaluation.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_compute_objective_gradient_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsTopologyOptimization = XMLGen::is_topology_optimization_problem(aXMLMetaData.optimization_type);
    auto tIsPlatoAnalyzePerformer = XMLGen::is_any_objective_computed_by_plato_analyze(aXMLMetaData);
    auto tAppendComputeObjectiveGradientOperation = tIsTopologyOptimization && tIsPlatoAnalyzePerformer;

    if(tAppendComputeObjectiveGradientOperation)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "ComputeObjectiveGradient", "Compute Objective Gradient" }, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children( { "ArgumentName" }, { "Objective Gradient" }, tOutput);
    }
}

/******************************************************************************//**
 * \fn append_compute_constraint_value_to_plato_analyze_operation
 * \brief Append compute constraint value operation to PUGI XML document. The \n
 * operation is only appended if Plato Analyze is responsible for its evaluation.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_compute_constraint_value_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsTopologyOptimization = XMLGen::is_topology_optimization_problem(aXMLMetaData.optimization_type);
    auto tIsPlatoAnalyzePerformer = XMLGen::is_any_constraint_computed_by_plato_analyze(aXMLMetaData);
    auto tAppendComputeConstraintValueOperation = tIsTopologyOptimization && tIsPlatoAnalyzePerformer;

    if(tAppendComputeConstraintValueOperation)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "ComputeConstraintValue", "Compute Constraint Value" }, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children( { "ArgumentName" }, { "Constraint Value" }, tOutput);
    }
}

/******************************************************************************//**
 * \fn append_compute_constraint_gradient_to_plato_analyze_operation
 * \brief Append compute constraint gradient operation to PUGI XML document. The \n
 * operation is only appended if Plato Analyze is responsible for its evaluation.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_compute_constraint_gradient_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsTopologyOptimization = XMLGen::is_topology_optimization_problem(aXMLMetaData.optimization_type);
    auto tIsPlatoAnalyzePerformer = XMLGen::is_any_constraint_computed_by_plato_analyze(aXMLMetaData);
    auto tAppendComputeConstraintGradientOperation = tIsTopologyOptimization && tIsPlatoAnalyzePerformer;

    if(tAppendComputeConstraintGradientOperation)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "ComputeConstraintGradient", "Compute Constraint Gradient" }, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children( { "ArgumentName" }, { "Constraint Gradient" }, tOutput);
    }
}

struct ValidAnalyzeOutputKeys
{
    /*!<
     * valid plato analyze output keys \n
     * \brief light-input file output key to plato analyze key map, i.e. map<light_input_file_output key, plato_analyze_output_key>. \n
     * Basically, this maps connects the key use for any quantity of interest inside Plato's light input file to the key used in \n
     * Plato Analyze to identify the quantity of interest.
     **/
    std::unordered_map<std::string, std::string> mKeys = { {"von_mises", "vonmises"}, {"plastic_multiplier_increment", "plastic multiplier increment"},
      {"accumulated_plastic_strain", "accumulated plastic strain"}, {"deviatoric_stress", "deviatoric stress"}, {"elastic_strain", "elastic_strain"},
      {"plastic_strain", "plastic strain"}, {"cauchy_stress", "cauchy stress"}, {"backstress", "backstress"}, {"dispx", "Solution X"},
      {"dispy", "Solution Y"}, {"dispz", "Solution Z"} };
};
// struct ValidLevelSetInitKeys

/******************************************************************************//**
 * \fn append_write_output_to_plato_analyze_operation
 * \brief Append write output operation to PUGI XML document. The operation is only \n
 * appended if the user specified output quantities of interest for visualization.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_write_output_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.mOutputMetaData.mDeterministicQuantitiesOfInterest.empty())
    {
        return;
    }

    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children({"Function", "Name"}, {"WriteOutput", "Write Output"}, tOperation);

    XMLGen::ValidAnalyzeOutputKeys tValidKeys;
    for(auto& tTag : aXMLMetaData.mOutputMetaData.mDeterministicQuantitiesOfInterest)
    {
        auto tLowerTag = Plato::tolower(tTag.first);
        auto tKeyItr = tValidKeys.mKeys.find(tLowerTag);
        if(tKeyItr == tValidKeys.mKeys.end())
        {
            THROWERR(std::string("Append Write Output To Plato Analyze Operation: ") + "Output tag '" + tTag.first + "' is not a valid output key.")
        }

        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children({"ArgumentName"}, {tTag.second}, tOutput);
    }
}

/******************************************************************************//**
 * \fn write_amgx_input_file
 * \brief Write AMGX input .json file. This file is used to assign values for the \n
 * linear solver parameters. Interested readers can find more information on AMGX \n
 * in \see{https://github.com/NVIDIA/AMGX}.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void write_amgx_input_file()
{
    FILE *tFilePointer = fopen("amgx.json", "w");
    if(tFilePointer)
    {
        fprintf(tFilePointer, "{\n");
        fprintf(tFilePointer, "\"config_version\": 2,\n");
        fprintf(tFilePointer, "\"solver\": {\n");
        fprintf(tFilePointer, "\"preconditioner\": {\n");
        fprintf(tFilePointer, "\"print_grid_stats\": 1,\n");
        fprintf(tFilePointer, "\"algorithm\": \"AGGREGATION\",\n");
        fprintf(tFilePointer, "\"print_vis_data\": 0,\n");
        fprintf(tFilePointer, "\"max_matching_iterations\": 50,\n");
        fprintf(tFilePointer, "\"max_unassigned_percentage\": 0.01,\n");
        fprintf(tFilePointer, "\"solver\": \"AMG\",\n");
        fprintf(tFilePointer, "\"smoother\": {\n");
        fprintf(tFilePointer, "\"relaxation_factor\": 0.78,\n");
        fprintf(tFilePointer, "\"scope\": \"jacobi\",\n");
        fprintf(tFilePointer, "\"solver\": \"BLOCK_JACOBI\",\n");
        fprintf(tFilePointer, "\"monitor_residual\": 0,\n");
        fprintf(tFilePointer, "\"print_solve_stats\": 0\n");
        fprintf(tFilePointer, "},\n");
        fprintf(tFilePointer, "\"print_solve_stats\": 0,\n");
        fprintf(tFilePointer, "\"dense_lu_num_rows\": 64,\n");
        fprintf(tFilePointer, "\"presweeps\": 1,\n");
        fprintf(tFilePointer, "\"selector\": \"SIZE_8\",\n");
        fprintf(tFilePointer, "\"coarse_solver\": \"DENSE_LU_SOLVER\",\n");
        fprintf(tFilePointer, "\"coarsest_sweeps\": 2,\n");
        fprintf(tFilePointer, "\"max_iters\": 1,\n");
        fprintf(tFilePointer, "\"monitor_residual\": 0,\n");
        fprintf(tFilePointer, "\"store_res_history\": 0,\n");
        fprintf(tFilePointer, "\"scope\": \"amg\",\n");
        fprintf(tFilePointer, "\"max_levels\": 100,\n");
        fprintf(tFilePointer, "\"postsweeps\": 1,\n");
        fprintf(tFilePointer, "\"cycle\": \"W\"\n");
        fprintf(tFilePointer, "},\n");
        fprintf(tFilePointer, "\"solver\": \"PBICGSTAB\",\n");
        fprintf(tFilePointer, "\"print_solve_stats\": 0,\n");
        fprintf(tFilePointer, "\"obtain_timings\": 0,\n");
        fprintf(tFilePointer, "\"max_iters\": 1000,\n");
        fprintf(tFilePointer, "\"monitor_residual\": 1,\n");
        fprintf(tFilePointer, "\"convergence\": \"ABSOLUTE\",\n");
        fprintf(tFilePointer, "\"scope\": \"main\",\n");
        fprintf(tFilePointer, "\"tolerance\": 1e-12,\n");
        fprintf(tFilePointer, "\"norm\": \"L2\"\n");
        fprintf(tFilePointer, "}\n");
        fprintf(tFilePointer, "}\n");
        fclose(tFilePointer);
    }
}

void write_plato_analyze_operation_xml_file
(const XMLGen::InputData& aXMLMetaData)
{
    pugi::xml_document tDocument;

    XMLGen::append_write_output_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_update_problem_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_objective_value_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_objective_gradient_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_constraint_value_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_constraint_gradient_to_plato_analyze_operation(aXMLMetaData, tDocument);

    tDocument.save_file("plato_analyze_operations.xml", "  ");
}

}

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, WriteAmgxInputFile)
{
    XMLGen::write_amgx_input_file();
    auto tData = XMLGen::read_data_from_file("amgx.json");
    auto tGold = std::string("{\"config_version\":2,\"solver\":{\"preconditioner\":{\"print_grid_stats\":1,\"algorithm\":\"AGGREGATION\",\"print_vis_data\":0,\"max_matching_iterations\":50,")
        +"\"max_unassigned_percentage\":0.01,\"solver\":\"AMG\",\"smoother\":{\"relaxation_factor\":0.78,\"scope\":\"jacobi\",\"solver\":\"BLOCK_JACOBI\",\"monitor_residual\":0,\"print_solve_stats\":0}"
        +",\"print_solve_stats\":0,\"dense_lu_num_rows\":64,\"presweeps\":1,\"selector\":\"SIZE_8\",\"coarse_solver\":\"DENSE_LU_SOLVER\",\"coarsest_sweeps\":2,\"max_iters\":1,\"monitor_residual\":0,"
        +"\"store_res_history\":0,\"scope\":\"amg\",\"max_levels\":100,\"postsweeps\":1,\"cycle\":\"W\"},\"solver\":\"PBICGSTAB\",\"print_solve_stats\":0,\"obtain_timings\":0,\"max_iters\":1000,"
        +"\"monitor_residual\":1,\"convergence\":\"ABSOLUTE\",\"scope\":\"main\",\"tolerance\":1e-12,\"norm\":\"L2\"}}";
    ASSERT_STREQ(tGold.c_str(), tData.str().c_str());
}

TEST(PlatoTestXMLGenerator, AppendWriteOutputToPlatoAnalyzeOperation_NoWriteOutputOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_NO_THROW(XMLGen::append_write_output_to_plato_analyze_operation(tInputData, tDocument));
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendWriteOutputToPlatoAnalyzeOperation_InvalidKey)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.mOutputMetaData.mDeterministicQuantitiesOfInterest.push_back({"fluid pressure", "fpressure"});
    ASSERT_THROW(XMLGen::append_write_output_to_plato_analyze_operation(tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendWriteOutputToPlatoAnalyzeOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.mOutputMetaData.mDeterministicQuantitiesOfInterest.push_back({"dispx", "Solution X"});
    tInputData.mOutputMetaData.mDeterministicQuantitiesOfInterest.push_back({"dispy", "Solution Y"});
    tInputData.mOutputMetaData.mDeterministicQuantitiesOfInterest.push_back({"dispz", "Solution Z"});
    ASSERT_NO_THROW(XMLGen::append_write_output_to_plato_analyze_operation(tInputData, tDocument));

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Output", "Output", "Output"};
    std::vector<std::string> tValues = {"WriteOutput", "Write Output", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Solution X"}, tOutput);

    tOutput = tOutput.next_sibling("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Solution Y"}, tOutput);

    tOutput = tOutput.next_sibling("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Solution Z"}, tOutput);

    tDocument.save_file("dummy.xml", "  ");
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemToPlatoAnalyzeOperation_DoNotWriteUpdateProblemOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.mProblemUpdateFrequency = "0";
    XMLGen::append_update_problem_to_plato_analyze_operation(tInputData, tDocument);
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemToPlatoAnalyzeOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.mProblemUpdateFrequency = "5";
    XMLGen::append_update_problem_to_plato_analyze_operation(tInputData, tDocument);
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());

    PlatoTestXMLGenerator::test_children({"Function", "Name"}, {"UpdateProblem", "Update Problem"}, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendComputeSolutionToPlatoAnalyzeOperation)
{
    pugi::xml_document tDocument;
    XMLGen::append_compute_solution_to_plato_analyze_operation(tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    PlatoTestXMLGenerator::test_children({"Function", "Name", "Input"}, {"Compute Solution", "Compute Displacement Solution", ""}, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
}

TEST(PlatoTestXMLGenerator, AppendComputeConstraintValueToPlatoAnalyzeOperation_Empty)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::append_compute_constraint_value_to_plato_analyze_operation(tInputData, tDocument);
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeConstraintValueToPlatoAnalyzeOperation_NotPlatoAnalyzePerformer)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::Constraint tConstraint;
    tConstraint.mPerformerName = "sierra_sd";
    tInputData.constraints.push_back(tConstraint);
    XMLGen::append_compute_constraint_value_to_plato_analyze_operation(tInputData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeConstraintValueToPlatoAnalyzeOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::Constraint tConstraint;
    tConstraint.mPerformerName = "plato_analyze";
    tInputData.constraints.push_back(tConstraint);
    XMLGen::append_compute_constraint_value_to_plato_analyze_operation(tInputData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = { "Function", "Name", "Input", "Output" };
    std::vector<std::string> tValues = { "ComputeConstraintValue", "Compute Constraint Value", "", "" };
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({ "ArgumentName" }, { "Topology" }, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({ "ArgumentName" }, { "Constraint Value" }, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendComputeConstraintGradientToPlatoAnalyzeOperation_Empty)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::append_compute_constraint_gradient_to_plato_analyze_operation(tInputData, tDocument);
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeConstraintGradientToPlatoAnalyzeOperation_NotPlatoAnalyzePerformer)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::Constraint tConstraint;
    tConstraint.mPerformerName = "sierra_sd";
    tInputData.constraints.push_back(tConstraint);
    XMLGen::append_compute_constraint_gradient_to_plato_analyze_operation(tInputData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeConstraintGradientToPlatoAnalyzeOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::Constraint tConstraint;
    tConstraint.mPerformerName = "plato_analyze";
    tInputData.constraints.push_back(tConstraint);
    XMLGen::append_compute_constraint_gradient_to_plato_analyze_operation(tInputData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = { "Function", "Name", "Input", "Output" };
    std::vector<std::string> tValues = { "ComputeConstraintGradient", "Compute Constraint Gradient", "", "" };
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({ "ArgumentName" }, { "Topology" }, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({ "ArgumentName" }, { "Constraint Gradient" }, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendComputeObjectiveValueToPlatoAnalyzeOperation_Empty)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::append_compute_objective_value_to_plato_analyze_operation(tInputData, tDocument);
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeObjectiveValueToPlatoAnalyzeOperation_NotPlatoAnalyzePerformer)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::Objective tObjective;
    tObjective.performer_name = "sierra_sd";
    tInputData.objectives.push_back(tObjective);
    XMLGen::append_compute_objective_value_to_plato_analyze_operation(tInputData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeObjectiveValueToPlatoAnalyzeOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato_analyze";
    tInputData.objectives.push_back(tObjective);
    XMLGen::append_compute_objective_value_to_plato_analyze_operation(tInputData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = { "Function", "Name", "Input", "Output" };
    std::vector<std::string> tValues = { "ComputeObjectiveValue", "Compute Objective Value", "", "" };
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({ "ArgumentName" }, { "Topology" }, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({ "ArgumentName" }, { "Objective Value" }, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendComputeObjectiveGradientToPlatoAnalyzeOperation_Empty)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::append_compute_objective_gradient_to_plato_analyze_operation(tInputData, tDocument);
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeObjectiveGradientToPlatoAnalyzeOperation_NotPlatoAnalyzePerformer)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::Objective tObjective;
    tObjective.performer_name = "sierra_sd";
    tInputData.objectives.push_back(tObjective);
    XMLGen::append_compute_objective_gradient_to_plato_analyze_operation(tInputData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeObjectiveGradientToPlatoAnalyzeOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::Objective tObjective;
    tObjective.performer_name = "plato_analyze";
    tInputData.objectives.push_back(tObjective);
    XMLGen::append_compute_objective_gradient_to_plato_analyze_operation(tInputData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = { "Function", "Name", "Input", "Output" };
    std::vector<std::string> tValues = { "ComputeObjectiveGradient", "Compute Objective Gradient", "", "" };
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({ "ArgumentName" }, { "Topology" }, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({ "ArgumentName" }, { "Objective Gradient" }, tOutput);
}

TEST(PlatoTestXMLGenerator, IsAnyObjectiveComputedByPlatoAnalyze)
{
    // NO OBJECTIVE DEFINED
    XMLGen::InputData tInputData;
    ASSERT_FALSE(XMLGen::is_any_objective_computed_by_plato_analyze(tInputData));

    XMLGen::Objective tObjective1;
    tObjective1.performer_name = "sierra_sd";
    tInputData.objectives.push_back(tObjective1);
    ASSERT_FALSE(XMLGen::is_any_objective_computed_by_plato_analyze(tInputData));

    XMLGen::Objective tObjective2;
    tObjective2.performer_name = "plato_analyze";
    tInputData.objectives.push_back(tObjective2);
    ASSERT_TRUE(XMLGen::is_any_objective_computed_by_plato_analyze(tInputData));

    tObjective2.performer_name = "sierra_sd";
    tInputData.objectives.pop_back();
    tInputData.objectives.push_back(tObjective2);
    ASSERT_FALSE(XMLGen::is_any_objective_computed_by_plato_analyze(tInputData));

    tObjective2.performer_name = "PLATO_Analyze";
    tInputData.objectives.pop_back();
    tInputData.objectives.push_back(tObjective2);
    ASSERT_TRUE(XMLGen::is_any_objective_computed_by_plato_analyze(tInputData));
}

TEST(PlatoTestXMLGenerator, IsPlatoAnalyzePerformer)
{
    ASSERT_FALSE(XMLGen::is_plato_analyze_performer("plato_main"));
    ASSERT_TRUE(XMLGen::is_plato_analyze_performer("plato_analyze"));
    ASSERT_TRUE(XMLGen::is_plato_analyze_performer("plAto_anAlyZe"));
    ASSERT_FALSE(XMLGen::is_plato_analyze_performer("plato analyze"));
    ASSERT_FALSE(XMLGen::is_plato_analyze_performer("plAto anAlyZe"));
}

TEST(PlatoTestXMLGenerator, IsTopologyOptimizationProblem)
{
    ASSERT_FALSE(XMLGen::is_topology_optimization_problem("shape"));
    ASSERT_TRUE(XMLGen::is_topology_optimization_problem("topology"));
    ASSERT_TRUE(XMLGen::is_topology_optimization_problem("ToPoLogy"));
    ASSERT_FALSE(XMLGen::is_topology_optimization_problem("_topology"));
    ASSERT_FALSE(XMLGen::is_topology_optimization_problem("_topology_"));
}

}
// namespace PlatoTestXMLGenerator
