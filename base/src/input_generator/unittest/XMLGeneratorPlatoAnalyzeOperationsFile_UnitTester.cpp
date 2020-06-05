/*
 * XMLGeneratorPlatoAnalyzeOperationsFile_UnitTester.cpp
 *
 *  Created on: Jun 2, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeOperationsFileUtilities.hpp"

namespace XMLGen
{

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
