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

TEST(PlatoTestXMLGenerator, WritePlatoAnalyzeOperationsXmlFile)
{
    XMLGen::InputData tMetaData;
    tMetaData.max_iterations = "10";
    tMetaData.discretization = "density";
    tMetaData.optimization_algorithm = "oc";
    tMetaData.mProblemUpdateFrequency = "5";
    XMLGen::Objective tObjective;
    tObjective.name = "1";
    tObjective.type = "compliance";
    tObjective.code_name = "plato_analyze";
    tObjective.mPerformerName = "plato_analyze_1";
    tMetaData.objectives.push_back(tObjective);
    XMLGen::Constraint tConstraint;
    tConstraint.code("plato_analyze");
    tMetaData.constraints.push_back(tConstraint);
    XMLGen::Scenario tScenario;
    tScenario.performer("plato_analyze_1");
    tScenario.cacheState("false");
    tScenario.updateProblem("true");
    tMetaData.append(tScenario);

    XMLGen::write_plato_analyze_operation_xml_file(tMetaData);

    auto tReadData = XMLGen::read_data_from_file("plato_analyze_operations.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><Operation><Function>UpdateProblem</Function><Name>UpdateProblem</Name></Operation><Operation><Function>ComputeObjectiveValue</Function><Name>ComputeObjectiveValue</Name>")
        +"<Input><ArgumentName>Topology</ArgumentName></Input><Output><ArgumentName>ObjectiveValue</ArgumentName></Output></Operation><Operation><Function>ComputeObjectiveGradient</Function><Name>ComputeObjectiveGradient</Name>"
        +"<Input><ArgumentName>Topology</ArgumentName></Input><Output><ArgumentName>ObjectiveGradient</ArgumentName></Output></Operation><Operation><Function>ComputeConstraintValue</Function><Name>ComputeConstraintValue</Name>"
        +"<Input><ArgumentName>Topology</ArgumentName></Input><Output><ArgumentName>ConstraintValue</ArgumentName></Output></Operation><Operation><Function>ComputeConstraintGradient</Function><Name>ComputeConstraintGradient</Name>"
        +"<Input><ArgumentName>Topology</ArgumentName></Input><Output><ArgumentName>ConstraintGradient</ArgumentName></Output></Operation>";
    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());
    Plato::system("rm -f plato_analyze_operations.xml");
}

TEST(PlatoTestXMLGenerator, AppendComputeQoiStatisticsOperation)
{
    XMLGen::InputData tMetaData;
    tMetaData.mOutputMetaData.appendRandomQoI("vonmises", "element field");

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

TEST(PlatoTestXMLGenerator, AppendComputeConstraintGradientToPlatoAnalyzeOperation)
{
    XMLGen::InputData tMetaData;
    XMLGen::Constraint tConstraint;
    tConstraint.code("plato_analyze");
    tMetaData.constraints.push_back(tConstraint);

    pugi::xml_document tDocument;
    XMLGen::append_compute_constraint_gradient_to_plato_analyze_operation(tMetaData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Input", "Output"};
    std::vector<std::string> tValues = {"ComputeConstraintGradient", "Compute Constraint Gradient", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    auto tOutput = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Constraint Gradient"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendComputeConstraintValueToPlatoAnalyzeOperation)
{
    XMLGen::InputData tMetaData;
    XMLGen::Constraint tConstraint;
    tConstraint.code("plato_analyze");
    tMetaData.constraints.push_back(tConstraint);

    pugi::xml_document tDocument;
    XMLGen::append_compute_constraint_value_to_plato_analyze_operation(tMetaData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Input", "Output"};
    std::vector<std::string> tValues = {"ComputeConstraintValue", "Compute Constraint Value", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    auto tOutput = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Constraint Value"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendComputeObjectiveGradientToPlatoAnalyzeOperation)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.code_name = "plato_analyze";
    tMetaData.objectives.push_back(tObjective);

    pugi::xml_document tDocument;
    XMLGen::append_compute_objective_gradient_to_plato_analyze_operation(tMetaData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Input", "Output"};
    std::vector<std::string> tValues = {"ComputeObjectiveGradient", "Compute Objective Gradient", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    auto tOutput = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Objective Gradient"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendComputeObjectiveValueToPlatoAnalyzeOperation)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.code_name = "plato_analyze";
    tMetaData.objectives.push_back(tObjective);

    pugi::xml_document tDocument;
    XMLGen::append_compute_objective_value_to_plato_analyze_operation(tMetaData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Input", "Output"};
    std::vector<std::string> tValues = {"ComputeObjectiveValue", "Compute Objective Value", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    auto tOutput = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Objective Value"}, tOutput);
}

TEST(PlatoTestXMLGenerator, WritePlatoMainOperationsXmlFile)
{
    XMLGen::InputData tMetaData;
    tMetaData.max_iterations = "10";
    tMetaData.discretization = "density";
    tMetaData.optimization_algorithm = "oc";
    XMLGen::Objective tObjective;
    tObjective.name = "1";
    tObjective.type = "compliance";
    tObjective.mPerformerName = "plato_analyze_1";
    tMetaData.objectives.push_back(tObjective);
    XMLGen::Scenario tScenario;
    tScenario.performer("plato_analyze_1");
    tScenario.cacheState("false");
    tScenario.updateProblem("true");
    tMetaData.append(tScenario);
    tMetaData.mOutputMetaData.outputData("false");

    ASSERT_NO_THROW(XMLGen::write_plato_main_operations_xml_file(tMetaData));

    auto tReadData = XMLGen::read_data_from_file("plato_main_operations.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><Filter><Name>Kernel</Name><Scale>2.0</Scale></Filter><Operation><Function>InitializeField</Function><Name>InitializeField</Name><Method>Uniform</Method><Uniform><Value>0.5</Value></Uniform>")
    +"<Output><ArgumentName>InitializedField</ArgumentName></Output></Operation><Operation><Function>SetLowerBounds</Function><Name>ComputeLowerBounds</Name><Discretization>density</Discretization><Input><ArgumentName>LowerBoundValue</ArgumentName>"
    +"</Input><Output><ArgumentName>LowerBoundVector</ArgumentName></Output></Operation><Operation><Function>SetUpperBounds</Function><Name>ComputeUpperBounds</Name><Discretization>density</Discretization><Input><ArgumentName>UpperBoundValue</ArgumentName>"
    +"</Input><Output><ArgumentName>UpperBoundVector</ArgumentName></Output></Operation><Operation><Function>UpdateProblem</Function><Name>UpdateProblem</Name></Operation><Operation><Function>Filter</Function><Name>FilterControl</Name>"
    +"<Gradient>False</Gradient><Input><ArgumentName>Field</ArgumentName></Input><Output><ArgumentName>FilteredField</ArgumentName></Output></Operation><Operation><Function>Filter</Function><Name>FilterGradient</Name><Gradient>True</Gradient>"
    +"<Input><ArgumentName>Field</ArgumentName></Input><Input><ArgumentName>Gradient</ArgumentName></Input><Output><ArgumentName>FilteredGradient</ArgumentName></Output></Operation>";
    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());
    Plato::system("rm -f plato_main_operations.xml");
}

TEST(PlatoTestXMLGenerator, AppendObjectiveGradientStage)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.name = "1";
    tObjective.type = "compliance";
    tObjective.mPerformerName = "plato_analyze_1";
    tMetaData.objectives.push_back(tObjective);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_objective_gradient_stage(tMetaData, tDocument));

    // STAGE INPUTS
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    ASSERT_STREQ("Stage", tStage.name());
    auto tName = tStage.child("Name");
    ASSERT_STREQ("Compute Objective Gradient ID-1", tName.child_value());
    auto tType = tStage.child("Type");
    ASSERT_STREQ("compliance", tType.child_value());
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
    tValues = {"Compute Objective Gradient", "plato_analyze_1", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tOpInputs);
    tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Objective Gradient", "Objective Gradient ID-1"}, tOpOutputs);

    // FILTER GRADIENT OPERATION
    tOperation = tOperation.next_sibling("Operation");
    tKeys = {"Name", "PerformerName", "Input", "Input", "Output"};
    tValues = {"Filter Gradient", "platomain", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Field", "Control"}, tOpInputs);
    tOpInputs = tOpInputs.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Gradient", "Objective Gradient ID-1"}, tOpInputs);
    tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Filtered Gradient", "Objective Gradient ID-1"}, tOpOutputs);

    // STAGE OUTPUT
    auto tOutput = tStage.child("Output");
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"SharedDataName"}, {"Objective Gradient ID-1"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveValueStage)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.name = "1";
    tObjective.type = "compliance";
    tObjective.mPerformerName = "plato_analyze_1";
    tMetaData.objectives.push_back(tObjective);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_objective_value_stage(tMetaData, tDocument));

    // STAGE INPUTS
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    ASSERT_STREQ("Stage", tStage.name());
    auto tName = tStage.child("Name");
    ASSERT_STREQ("Compute Objective Value ID-1", tName.child_value());
    auto tType = tStage.child("Type");
    ASSERT_STREQ("compliance", tType.child_value());
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
    tValues = {"Compute Objective Value", "plato_analyze_1", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tOpInputs);
    tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Objective Value", "Objective Value ID-1"}, tOpOutputs);

    // STAGE OUTPUT
    auto tOutput = tStage.child("Output");
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"SharedDataName"}, {"Objective Value ID-1"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendCacheStateStage)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.performer("plato_analyze_1");
    tScenario.cacheState("true");
    tMetaData.append(tScenario);

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

TEST(PlatoTestXMLGenerator, AppendUpdateProblemStage)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.performer("plato_analyze_1");
    tScenario.updateProblem("true");
    tMetaData.append(tScenario);

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
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tScenario.performer("plato_analyze_1");
    tMetaData.append(tScenario);
    tMetaData.mOutputMetaData.scenarioID("1");
    tMetaData.mOutputMetaData.appendDeterminsiticQoI("dispx", "nodal field");
    XMLGen::Objective tObjective;
    tObjective.name = "1";
    tObjective.mPerformerName = "plato_analyze_1";
    tMetaData.objectives.push_back(tObjective);

    pugi::xml_document tDocument;
    XMLGen::append_shared_data(tMetaData, tDocument);

    // TEST
    auto tSharedData = tDocument.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Control", "Scalar", "Nodal Field", "platomain", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    tValues = {"Lower Bound Value", "Scalar", "Global", "1", "platomain", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"Lower Bound Vector", "Scalar", "Nodal Field", "platomain", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    tValues = {"Upper Bound Value", "Scalar", "Global", "1", "platomain", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"Upper Bound Vector", "Scalar", "Nodal Field", "platomain", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    tValues = {"Design Volume", "Scalar", "Global", "1", "platomain", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"dispx", "Scalar", "Nodal Field", "plato_analyze_1", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName", "UserName"};
    tValues = {"Topology", "Scalar", "Nodal Field", "platomain", "platomain", "plato_analyze_1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    tValues = {"Objective Value ID-1", "Scalar", "Global", "1", "plato_analyze_1", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"Objective Gradient ID-1", "Scalar", "Nodal Field", "plato_analyze_1", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
}

TEST(PlatoTestXMLGenerator, AppendTopologySharedData_ErrorEmptyScenario)
{
    XMLGen::InputData tMetaData;
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_topology_shared_data(tMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendTopologySharedData)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tScenario.performer("plato_analyze_1");
    tMetaData.append(tScenario);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_topology_shared_data(tMetaData, tDocument));

    auto tSharedData = tDocument.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName", "UserName"};
    std::vector<std::string> tValues = {"Topology", "Scalar", "Nodal Field", "platomain", "platomain", "plato_analyze_1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
    tSharedData = tSharedData.next_sibling("Performer");
    ASSERT_TRUE(tSharedData.empty());
}

TEST(PlatoTestXMLGenerator, AppendQoISharedData_ErrorEmptyScenario)
{
    XMLGen::InputData tMetaData;
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_qoi_shared_data(tMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendQoISharedData_EmptySharedData)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tScenario.performer("plato_analyze_1");
    tMetaData.append(tScenario);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_qoi_shared_data(tMetaData, tDocument));

    auto tSharedData = tDocument.child("SharedData");
    ASSERT_TRUE(tSharedData.empty());
}

TEST(PlatoTestXMLGenerator, AppendQoISharedData_ErrorDidNotMatchOuputScenarioID)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tScenario.performer("plato_analyze_1");
    tMetaData.append(tScenario);
    tMetaData.mOutputMetaData.scenarioID("2");
    tMetaData.mOutputMetaData.appendDeterminsiticQoI("dispx", "nodal field");

    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_qoi_shared_data(tMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendQoISharedData)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tScenario.performer("plato_analyze_1");
    tMetaData.append(tScenario);
    tMetaData.mOutputMetaData.scenarioID("1");
    tMetaData.mOutputMetaData.appendDeterminsiticQoI("dispx", "nodal field");

    pugi::xml_document tDocument;
    XMLGen::append_qoi_shared_data(tMetaData, tDocument);

    auto tSharedData = tDocument.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"dispx", "Scalar", "Nodal Field", "plato_analyze_1", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
    tSharedData = tSharedData.next_sibling("Performer");
    ASSERT_TRUE(tSharedData.empty());
}

TEST(PlatoTestXMLGenerator, AppendPhysicsPerformers_EmptyScenario)
{
    XMLGen::InputData tMetaData;
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_physics_performers(tMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendPhysicsPerformers)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tScenario.code("plato_analyze");
    tScenario.performer("plato_analyze_1");
    tMetaData.append(tScenario);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_physics_performers(tMetaData, tDocument));

    auto tPerformer = tDocument.child("Performer");
    ASSERT_FALSE(tPerformer.empty());
    ASSERT_STREQ("Performer", tPerformer.name());
    std::vector<std::string> tKeys = {"Name", "Code", "PerformerID"};
    std::vector<std::string> tValues = {"plato_analyze_1", "plato_analyze", "1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tPerformer);
    tPerformer = tPerformer.next_sibling("Performer");
    ASSERT_TRUE(tPerformer.empty());
}

}
// namespace PlatoTestXMLGenerator
