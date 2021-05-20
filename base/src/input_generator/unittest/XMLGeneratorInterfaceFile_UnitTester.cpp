/*
 * XMLGeneratorInterfaceFile_UnitTester.cpp
 *
 *  Created on: Jul 24, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorPlatoAnalyzeUtilities.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"
#include "XMLGeneratorPlatoMainInputFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeOperationsFileUtilities.hpp"
#include "XMLGeneratorAnalyzeUncertaintyLaunchScriptUtilities.hpp"

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, AppendWriteOutputPlatoAnalyzeOperation)
{
    XMLGen::Output tOutput;
    tOutput.appendRandomQoI("dispx", "nodal field");
    tOutput.appendDeterminsiticQoI("vonmises", "element field");
}

TEST(PlatoTestXMLGenerator, AppendComputeQoiStatisticsOperation)
{
    XMLGen::InputData tMetaData;
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.appendRandomQoI("vonmises", "element field");
    tMetaData.mOutputMetaData.push_back(tOutputMetadata);

    pugi::xml_document tDocument;
    XMLGen::append_compute_qoi_statistics_operation(tMetaData, tDocument);

    // TEST OPERATION ARGUMENTS
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Name", "PerformerName", "For", "Output", "Output"};
    std::vector<std::string> tValues = {"compute vonmises statistics", "platomain", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    // TEST OUTPUTS
    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"vonmises mean", "vonmises mean"}, tOutput);
    tOutput = tOutput.next_sibling("Output");
    ASSERT_FALSE(tOutput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"vonmises standard deviation", "vonmises standard deviation"}, tOutput);
    tOutput = tOutput.next_sibling("Output");
    ASSERT_TRUE(tOutput.empty());

    // TEST RANDOM QoIs
    auto tOuterFor = tOperation.child("For");
    ASSERT_FALSE(tOuterFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
    auto tInnerFor = tOuterFor.child("For");
    ASSERT_FALSE(tInnerFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
    auto tInput = tInnerFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    tKeys = {"ArgumentName", "SharedDataName"};
    tValues = {"vonmises {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
               "vonmises {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
}

TEST(PlatoTestXMLGenerator, WritePlatoMainOperationsXmlFile)
{
    XMLGen::InputData tMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("max_iterations", "10");
    tOptimizationParameters.append("discretization", "density");
    tOptimizationParameters.append("optimization_algorithm", "oc");
    tOptimizationParameters.append("optimization_type", "topology");
    tOptimizationParameters.append("filter_in_engine", "true");
    tMetaData.set(tOptimizationParameters);
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tService.cacheState("false");
    tService.updateProblem("true");
    tMetaData.append(tService);
    XMLGen::Objective tObjective;
    tObjective.serviceIDs.push_back("1");
    tObjective.criteriaIDs.push_back("1");
    tMetaData.objective = tObjective;

    XMLGen::Output tOutputMetaData;
    tOutputMetaData.disableOutput();
    tMetaData.mOutputMetaData.push_back(tOutputMetaData);
    ASSERT_NO_THROW(XMLGen::write_plato_main_operations_xml_file(tMetaData));

    auto tReadData = XMLGen::read_data_from_file("plato_main_operations.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><Filter><Name>Kernel</Name><Scale>2.0</Scale></Filter><Operation><Function>InitializeField</Function><Name>InitializeField</Name><Method>Uniform</Method><Uniform><Value>0.5</Value></Uniform>")
    +"<Output><ArgumentName>InitializedField</ArgumentName></Output></Operation><Operation><Function>SetLowerBounds</Function><Name>ComputeLowerBounds</Name><Discretization>density</Discretization><Input><ArgumentName>LowerBoundValue</ArgumentName>"
    +"</Input><Output><ArgumentName>LowerBoundVector</ArgumentName></Output></Operation><Operation><Function>SetUpperBounds</Function><Name>ComputeUpperBounds</Name><Discretization>density</Discretization><Input><ArgumentName>UpperBoundValue</ArgumentName>"
    +"</Input><Output><ArgumentName>UpperBoundVector</ArgumentName></Output></Operation><Operation><Function>CopyField</Function><Name>CopyField</Name><Input><ArgumentName>InputField</ArgumentName></Input><Output><ArgumentName>OutputField</ArgumentName></Output></Operation><Operation><Function>CopyValue</Function><Name>CopyValue</Name><Input><ArgumentName>InputValue</ArgumentName></Input><Output><ArgumentName>OutputValue</ArgumentName></Output></Operation><Operation><Function>UpdateProblem</Function><Name>UpdateProblem</Name></Operation><Operation><Function>Filter</Function><Name>FilterControl</Name>"
    +"<Gradient>False</Gradient><Input><ArgumentName>Field</ArgumentName></Input><Output><ArgumentName>FilteredField</ArgumentName></Output></Operation><Operation><Function>Filter</Function><Name>FilterGradient</Name><Gradient>True</Gradient>"
    +"<Input><ArgumentName>Field</ArgumentName></Input><Input><ArgumentName>Gradient</ArgumentName></Input><Output><ArgumentName>FilteredGradient</ArgumentName></Output></Operation><Operation><Name>AggregateData</Name><Function>Aggregator</Function><Aggregate><Layout>Value</Layout><Input><ArgumentName>Value1</ArgumentName></Input><Output><ArgumentName>Value</ArgumentName></Output></Aggregate><Aggregate><Layout>NodalField</Layout><Input><ArgumentName>Field1</ArgumentName></Input><Output><ArgumentName>Field</ArgumentName></Output></Aggregate><Weighting><Weight><Value>1.0</Value></Weight><Normals><Input><ArgumentName>Normal1</ArgumentName></Input></Normals></Weighting></Operation>";
    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());
    Plato::system("rm -f plato_main_operations.xml");
}

TEST(PlatoTestXMLGenerator, AppendObjectiveGradientStage)
{
    XMLGen::InputData tMetaData;

    // Create services
    XMLGen::Service tService;
    tService.id("2");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    tService.id("1");
    tService.code("platomain");
    tMetaData.append(tService);

    // Create a criterion
    XMLGen::Criterion tCriterion;
    tCriterion.id("3");
    tCriterion.type("mass");
    tMetaData.append(tCriterion);

    // Create a scenario
    XMLGen::Scenario tScenario;
    tScenario.id("14");
    tScenario.physics("steady_state_mechanics");
    tMetaData.append(tScenario);
    
    // Create an objective
    XMLGen::Objective tObjective;
    tObjective.type = "single_criterion";
    tObjective.serviceIDs.push_back("2");
    tObjective.criteriaIDs.push_back("3");
    tObjective.scenarioIDs.push_back("14");
    tObjective.weights.push_back("1");
    tMetaData.objective = tObjective;

    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tMetaData.set(tOptimizationParameters);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_objective_gradient_stage(tMetaData, tDocument));

    // STAGE INPUTS
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    ASSERT_STREQ("Stage", tStage.name());
    auto tName = tStage.child("Name");
    ASSERT_STREQ("Compute Objective Gradient", tName.child_value());
    auto tInput = tStage.child("Input");
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({"SharedDataName"}, {"Control"}, tInput);

    // FILTER CONTROL OPERATION
    auto tOperation = tStage.child("Operation");
    std::vector<std::string> tKeys = {"Name", "PerformerName", "Input", "Output"};
    std::vector<std::string> tValues = {"Filter Control", "platomain_1", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Field", "Control"}, tOpInputs);
    auto tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Filtered Field", "Topology"}, tOpOutputs);

    // OBJECTIVE VALUE OPERATION
    tOperation = tOperation.next_sibling("Operation");
    tValues = {"Compute Objective Gradient", "plato_analyze_2", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tOpInputs);
    tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Objective Gradient", "Criterion Gradient - criterion_3_service_2_scenario_14"}, tOpOutputs);

    // FILTER GRADIENT OPERATION
    tOperation = tOperation.next_sibling("Operation");
    tKeys = {"Name", "PerformerName", "Input", "Input", "Output"};
    tValues = {"Filter Gradient", "platomain_1", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Field", "Control"}, tOpInputs);
    tOpInputs = tOpInputs.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Gradient", "Criterion Gradient - criterion_3_service_2_scenario_14"}, tOpInputs);
    tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Filtered Gradient", "Objective Gradient"}, tOpOutputs);

    // STAGE OUTPUT
    auto tOutput = tStage.child("Output");
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"SharedDataName"}, {"Objective Gradient"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveValueStage)
{
    XMLGen::InputData tMetaData;

    // Create a service
    XMLGen::Service tService;
    tService.id("2");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    tService.id("1");
    tService.code("platomain");
    tMetaData.append(tService);

    // Create a criterion
    XMLGen::Criterion tCriterion;
    tCriterion.id("3");
    tCriterion.type("mass");
    tMetaData.append(tCriterion);

    // Create a scenario
    XMLGen::Scenario tScenario;
    tScenario.id("14");
    tScenario.physics("steady_state_mechanics");
    tMetaData.append(tScenario);
    
    // Create an objective
    XMLGen::Objective tObjective;
    tObjective.type = "single_criterion";
    tObjective.serviceIDs.push_back("2");
    tObjective.criteriaIDs.push_back("3");
    tObjective.scenarioIDs.push_back("14");
    tObjective.weights.push_back("1");
    tMetaData.objective = tObjective;

    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("filter_in_engine", "true");
    tOptimizationParameters.append("optimization_type", "topology");
    tMetaData.set(tOptimizationParameters);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_objective_value_stage(tMetaData, tDocument));

    // STAGE INPUTS
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    ASSERT_STREQ("Stage", tStage.name());
    auto tName = tStage.child("Name");
    ASSERT_STREQ("Compute Objective Value", tName.child_value());
    auto tInput = tStage.child("Input");
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({"SharedDataName"}, {"Control"}, tInput);

    // FILTER CONTROL OPERATION
    auto tOperation = tStage.child("Operation");
    std::vector<std::string> tKeys = {"Name", "PerformerName", "Input", "Output"};
    std::vector<std::string> tValues = {"Filter Control", "platomain_1", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Field", "Control"}, tOpInputs);
    auto tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Filtered Field", "Topology"}, tOpOutputs);

    // OBJECTIVE VALUE OPERATION
    tOperation = tOperation.next_sibling("Operation");
    tValues = {"Compute Objective Value", "plato_analyze_2", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tOpInputs);
    tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Objective Value", "Criterion Value - criterion_3_service_2_scenario_14"}, tOpOutputs);

    // STAGE OUTPUT
    auto tOutput = tStage.child("Output");
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"SharedDataName"}, {"Objective Value"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveValueStage_MultiObjective)
{
    XMLGen::InputData tMetaData;

    // Create 2 services
    XMLGen::Service tService;
    tService.id("2");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    tService.id("5");
    tService.code("plato_analyze");
    tMetaData.append(tService);

    // Create 2 criteria
    XMLGen::Criterion tCriterion;
    tCriterion.id("3");
    tCriterion.type("mass");
    tMetaData.append(tCriterion);
    tCriterion.id("7");
    tCriterion.type("heat_conduction");
    tMetaData.append(tCriterion);

    // Create 2 scenarios
    XMLGen::Scenario tScenario;
    tScenario.id("14");
    tScenario.physics("mechanical");
    tMetaData.append(tScenario);
    tScenario.id("15");
    tScenario.physics("thermal");
    tMetaData.append(tScenario);
    
    // Create an objective
    XMLGen::Objective tObjective;
    tObjective.type = "weighted_sum";
    tObjective.serviceIDs.push_back("2");
    tObjective.serviceIDs.push_back("5");
    tObjective.criteriaIDs.push_back("3");
    tObjective.criteriaIDs.push_back("7");
    tObjective.scenarioIDs.push_back("14");
    tObjective.scenarioIDs.push_back("15");
    tObjective.weights.push_back("1");
    tObjective.weights.push_back("1");
    tMetaData.objective = tObjective;

    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tMetaData.set(tOptimizationParameters);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_objective_value_stage(tMetaData, tDocument));

    // STAGE INPUTS
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    ASSERT_STREQ("Stage", tStage.name());
    auto tName = tStage.child("Name");
    ASSERT_STREQ("Compute Objective Value", tName.child_value());
    auto tInput = tStage.child("Input");
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({"SharedDataName"}, {"Control"}, tInput);

    // FILTER CONTROL OPERATION
    auto tOperation = tStage.child("Operation");
    std::vector<std::string> tKeys = {"Name", "PerformerName", "Input", "Output"};
    std::vector<std::string> tValues = {"Filter Control", "platomain", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Field", "Control"}, tOpInputs);
    auto tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Filtered Field", "Topology"}, tOpOutputs);

    // OBJECTIVE VALUE OPERATION
    tOperation = tOperation.next_sibling("Operation");
    auto tSubOperation = tOperation.child("Operation");
    tValues = {"Compute Objective Value", "plato_analyze_2", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSubOperation);
    tOpInputs = tSubOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tOpInputs);
    tOpOutputs = tSubOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Objective Value", "Criterion Value - criterion_3_service_2_scenario_14"}, tOpOutputs);

    tSubOperation = tSubOperation.next_sibling("Operation");
    tValues = {"Compute Objective Value", "plato_analyze_5", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSubOperation);
    tOpInputs = tSubOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tOpInputs);
    tOpOutputs = tSubOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Objective Value", "Criterion Value - criterion_7_service_5_scenario_15"}, tOpOutputs);

    // STAGE OUTPUT
    auto tOutput = tStage.child("Output");
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"SharedDataName"}, {"Objective Value"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveValueOperation_non_multi_load_case)
{
    XMLGen::InputData tMetaData;

    // Create a service
    XMLGen::Service tService;
    tService.id("2");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    tService.id("1");
    tService.code("platomain");
    tMetaData.append(tService);

    // Create a criterion
    XMLGen::Criterion tCriterion;
    tCriterion.id("3");
    tCriterion.type("mass");
    tMetaData.append(tCriterion);

    // Create a scenario
    XMLGen::Scenario tScenario;
    tScenario.id("14");
    tScenario.physics("steady_state_mechanics");
    tMetaData.append(tScenario);
    
    // Create an objective
    XMLGen::Objective tObjective;
    tObjective.type = "single_criterion";
    tObjective.serviceIDs.push_back("2");
    tObjective.criteriaIDs.push_back("3");
    tObjective.scenarioIDs.push_back("14");
    tObjective.weights.push_back("1");
    tMetaData.objective = tObjective;

    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("filter_in_engine", "true");
    tOptimizationParameters.append("optimization_type", "topology");
    tMetaData.set(tOptimizationParameters);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_objective_value_operation_for_non_multi_load_case(tMetaData, tDocument, false));
    //tDocument.save_file("xml.txt", " ");

    auto tOperation = tDocument.child("Operation");
    std::vector<std::string> tKeys = {"Name", "PerformerName", "Input", "Output"};
    std::vector<std::string> tValues = {"Compute Objective Value", "plato_analyze_2", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tOpInputs);
    auto tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Objective Value", "Criterion Value - criterion_3_service_2_scenario_14"}, tOpOutputs);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveValueOperation_multi_load_case)
{
    XMLGen::InputData tMetaData;

    // Create a service
    XMLGen::Service tService;
    tService.id("2");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    tService.id("1");
    tService.code("platomain");
    tMetaData.append(tService);

    // Create a criterion
    XMLGen::Criterion tCriterion;
    tCriterion.id("3");
    tCriterion.type("mass");
    tMetaData.append(tCriterion);

    // Create a scenario
    XMLGen::Scenario tScenario;
    tScenario.id("14");
    tScenario.physics("steady_state_mechanics");
    tMetaData.append(tScenario);
    
    // Create a scenario
    tScenario.id("15");
    tScenario.physics("steady_state_mechanics");
    tMetaData.append(tScenario);

    // Create an objective
    XMLGen::Objective tObjective;
    tObjective.type = "single_criterion";
    tObjective.serviceIDs.push_back("2");
    tObjective.criteriaIDs.push_back("3");
    tObjective.scenarioIDs.push_back("14");
    tObjective.scenarioIDs.push_back("15");
    tObjective.weights.push_back("1");
    tObjective.multi_load_case = "true";
    tMetaData.objective = tObjective;

    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("filter_in_engine", "true");
    tOptimizationParameters.append("optimization_type", "topology");
    tMetaData.set(tOptimizationParameters);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_objective_value_operation_for_multi_load_case(tMetaData, tDocument, false));
    //tDocument.save_file("xml.txt", " ");

    auto tOperation = tDocument.child("Operation");
    std::vector<std::string> tKeys = {"Name", "PerformerName", "Input", "Output"};
    std::vector<std::string> tValues = {"Compute Objective Value", "plato_analyze_2", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tOpInputs);
    auto tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Objective Value", "Criterion Value - criterion_3_service_2_scenario_1415"}, tOpOutputs);
}

TEST(PlatoTestXMLGenerator, AppendAggregateObjectiveValueOperation_non_multi_load_case)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.serviceIDs.push_back("2");
    tObjective.serviceIDs.push_back("3");
    tObjective.criteriaIDs.push_back("3");
    tObjective.criteriaIDs.push_back("4");
    tObjective.scenarioIDs.push_back("14");
    tObjective.scenarioIDs.push_back("15");
    tObjective.multi_load_case = "false";
    tMetaData.objective = tObjective;

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_aggregate_objective_value_operation_for_non_multi_load_case(tMetaData, tDocument));
    //tDocument.save_file("xml.txt", " ");

    auto tInput = tDocument.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    std::vector<std::string> tKeys = {"ArgumentName", "SharedDataName"};
    std::vector<std::string> tValues = {"Value 1", "Criterion Value - criterion_3_service_2_scenario_14"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tValues = {"Value 2", "Criterion Value - criterion_4_service_3_scenario_15"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
}

TEST(PlatoTestXMLGenerator, AppendAggregateObjectiveValueOperation_multi_load_case)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.serviceIDs.push_back("2");
    tObjective.serviceIDs.push_back("3");
    tObjective.criteriaIDs.push_back("3");
    tObjective.criteriaIDs.push_back("4");
    tObjective.scenarioIDs.push_back("14");
    tObjective.scenarioIDs.push_back("15");
    tObjective.multi_load_case = "true";
    tMetaData.objective = tObjective;

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_aggregate_objective_value_operation_for_multi_load_case(tMetaData, tDocument));
    //tDocument.save_file("xml.txt", " ");

    auto tInput = tDocument.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    std::vector<std::string> tKeys = {"ArgumentName", "SharedDataName"};
    std::vector<std::string> tValues = {"Value 1", "Criterion Value - criterion_3_service_2_scenario_1415"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_TRUE(tInput.empty());
}

TEST(PlatoTestXMLGenerator, AppendAggregateObjectiveGradientOperation_non_multi_load_case)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.serviceIDs.push_back("2");
    tObjective.serviceIDs.push_back("3");
    tObjective.criteriaIDs.push_back("3");
    tObjective.criteriaIDs.push_back("4");
    tObjective.scenarioIDs.push_back("14");
    tObjective.scenarioIDs.push_back("15");
    tObjective.multi_load_case = "false";
    tMetaData.objective = tObjective;

    pugi::xml_document tDocument;
    std::string tType = "Field";
    ASSERT_NO_THROW(XMLGen::append_aggregate_objective_gradient_operation_for_non_multi_load_case(tMetaData, tDocument, tType));
    //tDocument.save_file("xml.txt", " ");

    auto tInput = tDocument.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    std::vector<std::string> tKeys = {"ArgumentName", "SharedDataName"};
    std::vector<std::string> tValues = {"Field 1", "Criterion Gradient - criterion_3_service_2_scenario_14"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tValues = {"Field 2", "Criterion Gradient - criterion_4_service_3_scenario_15"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
    auto tOutput = tInput.next_sibling("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    tValues = {"Field", "Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendAggregateObjectiveGradientOperation_multi_load_case)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.serviceIDs.push_back("2");
    tObjective.serviceIDs.push_back("3");
    tObjective.criteriaIDs.push_back("3");
    tObjective.criteriaIDs.push_back("4");
    tObjective.scenarioIDs.push_back("14");
    tObjective.scenarioIDs.push_back("15");
    tObjective.multi_load_case = "true";
    tMetaData.objective = tObjective;

    pugi::xml_document tDocument;
    std::string tType = "Field";
    ASSERT_NO_THROW(XMLGen::append_aggregate_objective_gradient_operation_for_multi_load_case(tMetaData, tDocument, tType));
    //tDocument.save_file("xml.txt", " ");

    auto tInput = tDocument.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    std::vector<std::string> tKeys = {"ArgumentName", "SharedDataName"};
    std::vector<std::string> tValues = {"Field 1", "Criterion Gradient - criterion_3_service_2_scenario_1415"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
    auto tOutput = tInput.next_sibling("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    tValues = {"Field", "Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendCacheStateStage)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.id("1");
    tService.cacheState("true");
    tMetaData.append(tService);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_cache_state_stage(tMetaData, tDocument));

    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    ASSERT_STREQ("Stage", tStage.name());
    PlatoTestXMLGenerator::test_children({"Name", "Operation"}, {"Cache State", ""}, tStage);
    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    PlatoTestXMLGenerator::test_children({"Name", "PerformerName"}, {"Cache State", "plato_analyze_1"}, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendCacheStateStage_multi_load_case)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.id("1");
    tService.cacheState("true");
    tMetaData.append(tService);
    tMetaData.objective.multi_load_case = "true";
    tMetaData.objective.scenarioIDs.push_back("33");
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.serviceID("1");
    tOutputMetadata.appendDeterminsiticQoI("dispx", "nodal field");
    tMetaData.mOutputMetaData.push_back(tOutputMetadata);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_cache_state_stage(tMetaData, tDocument));

    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    ASSERT_STREQ("Stage", tStage.name());
    PlatoTestXMLGenerator::test_children({"Name", "Operation"}, {"Cache State", ""}, tStage);
    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    PlatoTestXMLGenerator::test_children({"Name", "PerformerName", "Output"}, {"Cache State", "plato_analyze_1", ""}, tOperation);
    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"dispx0", "dispx_plato_analyze_1_scenario_33"}, tOutput);
    //tDocument.save_file("xml.txt", " ");
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemStage)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.id("1");
    tService.updateProblem("true");
    tMetaData.append(tService);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_update_problem_stage(tMetaData, tDocument));

    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    ASSERT_STREQ("Stage", tStage.name());
    PlatoTestXMLGenerator::test_children({"Name", "Operation"}, {"Update Problem", ""}, tStage);
    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    PlatoTestXMLGenerator::test_children({"Name", "PerformerName"}, {"Update Problem", "plato_analyze_1"}, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendSharedData)
{
    XMLGen::InputData tMetaData;

    // Create a service
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.serviceID("1");
    tOutputMetadata.appendDeterminsiticQoI("dispx", "nodal field");
    tMetaData.mOutputMetaData.push_back(tOutputMetadata);
    tService.id("2");
    tService.code("platomain");
    tMetaData.append(tService);

    // Create a criterion
    XMLGen::Criterion tCriterion;
    tCriterion.id("3");
    tCriterion.type("mass");
    tMetaData.append(tCriterion);

    // Create a scenario
    XMLGen::Scenario tScenario;
    tScenario.id("14");
    tScenario.physics("mechanical");
    tMetaData.append(tScenario);
    
    // Create an objective
    XMLGen::Objective tObjective;
    tObjective.type = "single_criterion";
    tObjective.serviceIDs.push_back("1");
    tObjective.criteriaIDs.push_back("3");
    tObjective.scenarioIDs.push_back("14");
    tObjective.weights.push_back("1");
    tMetaData.objective = tObjective;

    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tMetaData.set(tOptimizationParameters);

    pugi::xml_document tDocument;
    XMLGen::append_shared_data(tMetaData, tDocument);

    // TEST
    auto tSharedData = tDocument.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Control", "Scalar", "Nodal Field", "platomain_2", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    tValues = {"Lower Bound Value", "Scalar", "Global", "1", "platomain_2", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"Lower Bound Vector", "Scalar", "Nodal Field", "platomain_2", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    tValues = {"Upper Bound Value", "Scalar", "Global", "1", "platomain_2", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"Upper Bound Vector", "Scalar", "Nodal Field", "platomain_2", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    tValues = {"Design Volume", "Scalar", "Global", "1", "platomain_2", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    tValues = {"Criterion Value - criterion_3_service_1_scenario_14", "Scalar", "Global", "1", "plato_analyze_1", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"Criterion Gradient - criterion_3_service_1_scenario_14", "Scalar", "Nodal Field", "plato_analyze_1", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    tValues = {"Objective Value", "Scalar", "Global", "1", "platomain_2", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"Objective Gradient", "Scalar", "Nodal Field", "platomain_2", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"dispx_plato_analyze_1", "Scalar", "Nodal Field", "plato_analyze_1", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName", "UserName"};
    tValues = {"Topology", "Scalar", "Nodal Field", "platomain_2", "platomain_2", "plato_analyze_1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
}

TEST(PlatoTestXMLGenerator, AppendTopologySharedData_ErrorEmptyService)
{
    XMLGen::InputData tMetaData;
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_topology_shared_data(tMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendTopologySharedData)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tMetaData.append(tService);
    tService.id("2");
    tService.code("plato_analyze");
    tMetaData.append(tService);

    pugi::xml_document tDocument;

    ASSERT_NO_THROW(XMLGen::append_topology_shared_data(tMetaData, tDocument));

    auto tSharedData = tDocument.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName", "UserName"};
    std::vector<std::string> tValues = {"Topology", "Scalar", "Nodal Field", "platomain_1", "platomain_1", "plato_analyze_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
    tSharedData = tSharedData.next_sibling("Performer");
    ASSERT_TRUE(tSharedData.empty());
}

TEST(PlatoTestXMLGenerator, AppendQoISharedData_ErrorEmptyService)
{
    XMLGen::InputData tMetaData;
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_qoi_shared_data(tMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendQoISharedData_EmptySharedData)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.id("1");
    tMetaData.append(tService);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_qoi_shared_data(tMetaData, tDocument));

    auto tSharedData = tDocument.child("SharedData");
    ASSERT_TRUE(tSharedData.empty());
}

TEST(PlatoTestXMLGenerator, AppendQoISharedData_ErrorDidNotMatchOuputServiceID)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.code("plato_analyze");
    tService.id("1");
    tMetaData.append(tService);
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.serviceID("2");
    tOutputMetadata.appendDeterminsiticQoI("dispx", "nodal field");
    tMetaData.mOutputMetaData.push_back(tOutputMetadata);

    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_qoi_shared_data(tMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendDeterministicQoIToPlatoMainOutput_non_multi_load_case)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    tService.id("2");
    tService.code("platomain");
    tMetaData.append(tService);
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.serviceID("1");
    tOutputMetadata.appendDeterminsiticQoI("dispx", "nodal field");
    tMetaData.mOutputMetaData.push_back(tOutputMetadata);

    pugi::xml_document tDocument;
    XMLGen::append_deterministic_qoi_to_plato_main_output_stage_for_non_multi_load_case(tMetaData, tDocument);
    //tDocument.save_file("xml.txt", " ");

    auto tInput = tDocument.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    std::vector<std::string> tKeys = {"ArgumentName", "SharedDataName"};
    std::vector<std::string> tValues = {"dispx_plato_analyze_1", "dispx_plato_analyze_1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
}

TEST(PlatoTestXMLGenerator, AppendDeterministicQoIToPlatoMainOutput_multi_load_case)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    tService.id("2");
    tService.code("platomain");
    tMetaData.append(tService);
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.serviceID("1");
    tOutputMetadata.appendDeterminsiticQoI("dispx", "nodal field");
    tMetaData.mOutputMetaData.push_back(tOutputMetadata);
    tMetaData.objective.multi_load_case = "true";
    tMetaData.objective.scenarioIDs.push_back("1");
    tMetaData.objective.scenarioIDs.push_back("2");

    pugi::xml_document tDocument;
    XMLGen::append_deterministic_qoi_to_plato_main_output_stage_for_multi_load_case(tMetaData, tDocument);
    //tDocument.save_file("xml.txt", " ");

    auto tInput = tDocument.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    std::vector<std::string> tKeys = {"ArgumentName", "SharedDataName"};
    std::vector<std::string> tValues = {"dispx_plato_analyze_1_scenario_1", "dispx_plato_analyze_1_scenario_1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tValues = {"dispx_plato_analyze_1_scenario_2", "dispx_plato_analyze_1_scenario_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
}

TEST(PlatoTestXMLGenerator, AppendNormalizationSharedData_non_multi_load_case)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    tService.id("2");
    tService.code("platomain");
    tMetaData.append(tService);
    tMetaData.objective.multi_load_case = "false";
    tMetaData.objective.scenarioIDs.push_back("5");
    tMetaData.objective.scenarioIDs.push_back("6");
    tMetaData.objective.serviceIDs.push_back("1");
    tMetaData.objective.serviceIDs.push_back("2");
    tMetaData.objective.criteriaIDs.push_back("8");
    tMetaData.objective.criteriaIDs.push_back("9");
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("normalize_in_aggregator", "true");
    tMetaData.set(tOptimizationParameters);

    pugi::xml_document tDocument;
    XMLGen::append_normalization_shared_data(tMetaData, tDocument);
    //tDocument.save_file("xml.txt", " ");

    auto tSharedData = tDocument.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Initial Criterion Value - criterion_8_service_1_scenario_5", 
               "Scalar", "Global", "1", "plato_analyze_1", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tValues = {"Initial Criterion Value - criterion_9_service_2_scenario_6", 
               "Scalar", "Global", "1", "platomain_2", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
}

TEST(PlatoTestXMLGenerator, AppendNormalizationSharedData_multi_load_case)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    tService.id("2");
    tService.code("platomain");
    tMetaData.append(tService);
    tMetaData.objective.multi_load_case = "true";
    tMetaData.objective.scenarioIDs.push_back("5");
    tMetaData.objective.scenarioIDs.push_back("6");
    tMetaData.objective.serviceIDs.push_back("1");
    tMetaData.objective.serviceIDs.push_back("2");
    tMetaData.objective.criteriaIDs.push_back("8");
    tMetaData.objective.criteriaIDs.push_back("9");
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("normalize_in_aggregator", "true");
    tMetaData.set(tOptimizationParameters);

    pugi::xml_document tDocument;
    XMLGen::append_normalization_shared_data(tMetaData, tDocument);
    //tDocument.save_file("xml.txt", " ");

    auto tSharedData = tDocument.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Initial Criterion Value - criterion_8_service_1_scenario_56", 
               "Scalar", "Global", "1", "plato_analyze_1", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_TRUE(tSharedData.empty());
}

TEST(PlatoTestXMLGenerator, AppendQoISharedData)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    tService.id("2");
    tService.code("platomain");
    tMetaData.append(tService);
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.serviceID("1");
    tOutputMetadata.appendDeterminsiticQoI("dispx", "nodal field");
    tMetaData.mOutputMetaData.push_back(tOutputMetadata);

    pugi::xml_document tDocument;
    XMLGen::append_qoi_shared_data(tMetaData, tDocument);

    auto tSharedData = tDocument.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName", "UserName"};
    std::vector<std::string> tValues = {"dispx_plato_analyze_1", "Scalar", "Nodal Field", "plato_analyze_1", "plato_analyze_1", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
    tSharedData = tSharedData.next_sibling("Performer");
    ASSERT_TRUE(tSharedData.empty());
}

TEST(PlatoTestXMLGenerator, AppendQoISharedData_multi_load_case)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    tService.id("2");
    tService.code("platomain");
    tMetaData.append(tService);
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.serviceID("1");
    tOutputMetadata.appendDeterminsiticQoI("dispx", "nodal field");
    tMetaData.mOutputMetaData.push_back(tOutputMetadata);
    tMetaData.objective.multi_load_case = "true";
    tMetaData.objective.scenarioIDs.push_back("22");

    pugi::xml_document tDocument;
    XMLGen::append_qoi_shared_data(tMetaData, tDocument);

    auto tSharedData = tDocument.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName", "UserName"};
    std::vector<std::string> tValues = {"dispx_plato_analyze_1_scenario_22", "Scalar", "Nodal Field", "plato_analyze_1", "plato_analyze_1", "platomain_2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
    tSharedData = tSharedData.next_sibling("Performer");
    ASSERT_TRUE(tSharedData.empty());
}

TEST(PlatoTestXMLGenerator, AppendPhysicsPerformers_EmptyService)
{
    XMLGen::InputData tMetaData;
    pugi::xml_document tDocument;
    int tID = 0;
    ASSERT_THROW(XMLGen::append_physics_performers(tMetaData, tID, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendPhysicsPerformers)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tMetaData.append(tService);
    tService.id("2");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    tMetaData.mPerformerServices.push_back(tService);

    pugi::xml_document tDocument;
    int tID = 0;
    ASSERT_NO_THROW(XMLGen::append_plato_main_performer(tMetaData, tID, tDocument));
    ASSERT_NO_THROW(XMLGen::append_physics_performers(tMetaData, tID, tDocument));

    auto tPerformer = tDocument.child("Performer");
    ASSERT_FALSE(tPerformer.empty());
    tPerformer = tPerformer.next_sibling("Performer");
    ASSERT_FALSE(tPerformer.empty());
    ASSERT_STREQ("Performer", tPerformer.name());
    std::vector<std::string> tKeys = {"Name", "Code", "PerformerID"};
    std::vector<std::string> tValues = {"plato_analyze_2", "plato_analyze", "1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tPerformer);
    tPerformer = tPerformer.next_sibling("Performer");
    ASSERT_TRUE(tPerformer.empty());
}

}
// namespace PlatoTestXMLGenerator
