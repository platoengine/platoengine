/*
 * XMLGeneratorRandomInterfaceFile_UnitTester.cpp
 *
 *  Created on: May 28, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"
#include "XMLGeneratorRandomInterfaceFileUtilities.hpp"
#include "XMLGeneratorPlatoMainConstraintValueOperationInterface.hpp"
#include "XMLGeneratorPlatoMainConstraintGradientOperationInterface.hpp"

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, AppendWriteOuputOperation_OutputDataSetFalse)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.serviceID("1");
    tOutputMetadata.disableOutput();
    tOutputMetadata.appendRandomQoI("dispx", "nodal field");
    tMetaData.mOutputMetaData.push_back(tOutputMetadata);

    pugi::xml_document tDocument;
    XMLGen::append_write_ouput_operation(tMetaData, tDocument);
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendWriteOuputOperation_OutputDataSetTrueButEmptyOutputList)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.serviceID("1");
    tMetaData.mOutputMetaData.push_back(tOutputMetadata);

    pugi::xml_document tDocument;
    XMLGen::append_write_ouput_operation(tMetaData, tDocument);
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendWriteOuputOperation_Random)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.serviceID("1");
    tOutputMetadata.appendRandomQoI("dispx", "nodal field");
    tMetaData.mOutputMetaData.push_back(tOutputMetadata);

    pugi::xml_document tDocument;
    XMLGen::append_write_ouput_operation(tMetaData, tDocument);

    auto tOuterFor = tDocument.child("For");
    ASSERT_FALSE(tOuterFor.empty());
    ASSERT_STREQ("For", tOuterFor.name());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tOuterFor);
    auto tOperation = tOuterFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    auto tInnerFor = tOperation.child("For");
    ASSERT_FALSE(tInnerFor.empty());
    ASSERT_STREQ("For", tInnerFor.name());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tInnerFor);

    tOperation = tInnerFor.child("Operation");
    std::vector<std::string> tGoldKeys = {"Name", "PerformerName", "Output"};
    std::vector<std::string> tGoldValues = {"Write Output", "plato_analyze_1_{PerformerIndex}", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Solution X", "dispx {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
    tOutput = tOutput.next_sibling("Output");
    ASSERT_TRUE(tOutput.empty());
}

TEST(PlatoTestXMLGenerator, AppendWriteOuputOperation_Deterministic)
{
    XMLGen::InputData tMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tMetaData.append(tService);
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.serviceID("1");
    tOutputMetadata.appendDeterminsiticQoI("vonmises", "element field");
    tMetaData.mOutputMetaData.push_back(tOutputMetadata);
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("filter_in_engine", "true");
    tMetaData.set(tOptimizationParameters);

    pugi::xml_document tDocument;
    XMLGen::append_write_ouput_operation(tMetaData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tGoldKeys = {"Name", "PerformerName", "Output"};
    std::vector<std::string> tGoldValues = {"Write Output", "plato_analyze_1", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Vonmises", "vonmises_plato_analyze_1"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendTrustRegionKelleySachsOptions)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("problem_update_frequency", "5");
    tOptimizationParameters.append("ks_max_trust_region_iterations", "10");
    tOptimizationParameters.append("ks_disable_post_smoothing", "false");
    tXMLMetaData.set(tOptimizationParameters);
    ASSERT_NO_THROW(XMLGen::append_trust_region_kelley_sachs_options(tXMLMetaData, tDocument));
    auto tOptions = tDocument.child("Options");
    ASSERT_FALSE(tOptions.empty());
    std::vector<std::string> tGoldKeys = {"KSMaxTrustRegionIterations", "ProblemUpdateFrequency", "DisablePostSmoothing"};
    std::vector<std::string> tGoldValues = {"10", "5", "false"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptions);
}

TEST(PlatoTestXMLGenerator, AppendPlatoMainOutputStage_EmptyStage_OutputDisabled)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.disableOutput();
    tXMLMetaData.mOutputMetaData.push_back(tOutputMetadata);
    ASSERT_NO_THROW(XMLGen::append_plato_main_output_stage(tXMLMetaData, tDocument));
    auto tStage = tDocument.child("Stage");
    ASSERT_TRUE(tStage.empty());
}

TEST(PlatoTestXMLGenerator, AppendPlatoMainOutputStageDeterministic)
{
    // POSE PARAMETERS
    XMLGen::InputData tXMLMetaData;
    XMLGen::Objective tObjective;
    tXMLMetaData.objective = tObjective;

    XMLGen::Constraint tConstraint;
    tConstraint.id("1");
    tXMLMetaData.constraints.push_back(tConstraint);

    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tXMLMetaData.append(tService);
    tService.id("2");
    tService.code("plato_analyze");
    tXMLMetaData.append(tService);

    XMLGen::Output tOutputMetadata;
    tOutputMetadata.serviceID("2");
    tOutputMetadata.appendDeterminsiticQoI("vonmises", "element field");
    tXMLMetaData.mOutputMetaData.push_back(tOutputMetadata);

    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("filter_in_engine", "true");
    tOptimizationParameters.append("optimization_type", "topology");
    tXMLMetaData.set(tOptimizationParameters);

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_plato_main_output_stage(tXMLMetaData, tDocument);

    // TEST RESULTS
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Operation", "Operation"};
    std::vector<std::string> tGoldValues = {"Output To File", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    tGoldKeys = {"Name", "PerformerName", "Output"};
    tGoldValues = {"Write Output", "plato_analyze_2", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Vonmises", "vonmises_plato_analyze_2"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
    tOutput = tOutput.next_sibling("Output");
    ASSERT_TRUE(tOutput.empty());

    tOperation = tOperation.next_sibling("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Input", "Input", "Input", "Input"};
    tGoldValues = {"PlatoMainOutput", "platomain_1", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"topology", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"control", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"objective gradient", "Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"constraint gradient 1", "Constraint Gradient 1"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"vonmises", "vonmises_plato_analyze_2"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_TRUE(tInput.empty());
}

TEST(PlatoTestXMLGenerator, AppendPlatoMainOutputStageRandom)
{
    // POSE PARAMETERS
    XMLGen::InputData tXMLMetaData;

    XMLGen::Constraint tConstraint;
    tConstraint.id("1");
    tXMLMetaData.constraints.push_back(tConstraint);

    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tXMLMetaData.append(tService);
    tService.id("2");
    tService.code("plato_analyze");
    tXMLMetaData.append(tService);

    XMLGen::Output tOutputMetadata;
    tOutputMetadata.serviceID("2");
    tOutputMetadata.outputSamples("true");
    tOutputMetadata.appendRandomQoI("vonmises", "element field");
    tXMLMetaData.mOutputMetaData.push_back(tOutputMetadata);

    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tXMLMetaData.set(tOptimizationParameters);

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_plato_main_output_stage(tXMLMetaData, tDocument);

    // TEST STAGE ARGUMENTS
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "For", "Operation", "Operation"};
    std::vector<std::string> tGoldValues = {"Output To File", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    // TEST WRITE OUPUT OPERATION
    auto tOuterFor = tStage.child("For");
    ASSERT_FALSE(tOuterFor.empty());
    ASSERT_STREQ("For", tOuterFor.name());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tOuterFor);
    auto tOperation = tOuterFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    auto tInnerFor = tOperation.child("For");
    ASSERT_FALSE(tInnerFor.empty());
    ASSERT_STREQ("For", tInnerFor.name());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tInnerFor);

    tOperation = tInnerFor.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "Output"};
    tGoldValues = {"Write Output", "plato_analyze_2_{PerformerIndex}", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Vonmises", "vonmises {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
    tOutput = tOutput.next_sibling("Output");
    ASSERT_TRUE(tOutput.empty());

    // TEST COMPUTE STATISTICS OPERATION
    tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Name", "PerformerName", "For", "Output", "Output"};
    std::vector<std::string> tValues = {"compute vonmises statistics", "platomain_1", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"vonmises mean", "vonmises mean"}, tOutput);
    tOutput = tOutput.next_sibling("Output");
    ASSERT_FALSE(tOutput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"vonmises standard deviation", "vonmises standard deviation"}, tOutput);
    tOutput = tOutput.next_sibling("Output");
    ASSERT_TRUE(tOutput.empty());

    tOuterFor = tOperation.child("For");
    ASSERT_FALSE(tOuterFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
    tInnerFor = tOuterFor.child("For");
    ASSERT_FALSE(tInnerFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
    auto tInput = tInnerFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    tKeys = {"ArgumentName", "SharedDataName"};
    tValues = {"vonmises {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
               "vonmises {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);

    // TEST PLATOMAIN OUTPUT OPERATION
    tOperation = tOperation.next_sibling("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Input", "Input", "Input", "For", "Input", "Input"};
    tGoldValues = {"PlatoMainOutput", "platomain_1", "", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    // DEFAULT QoIs
    tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"topology", "Topology"}, tInput);
    tInput = tInput.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"control", "Control"}, tInput);
    tInput = tInput.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"objective gradient", "Objective Gradient"}, tInput);
    tInput = tInput.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"constraint gradient 1", "Constraint Gradient 1"}, tInput);
    tInput = tInput.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"vonmises mean", "vonmises mean"}, tInput);
    tInput = tInput.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"vonmises standard deviation", "vonmises standard deviation"}, tInput);

    // RANDOM QoIs
    tOuterFor = tOperation.child("For");
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
    tInnerFor = tOuterFor.child("For");
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
    tInput = tInnerFor.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"vonmises {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
                   "vonmises {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
}

TEST(PlatoTestXMLGenerator, ConstraintValueOperation_ErrorInvalidCode)
{
    XMLGen::ConstraintValueOperation tInterface;
    pugi::xml_document tDocument;
    XMLGen::Constraint tConstraint;
    ASSERT_THROW(tInterface.call(tConstraint, "plato_analyze", "Topology", "hippo", tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ConstraintValueOperation_PlatoMain)
{
    XMLGen::ConstraintValueOperation tInterface;
    pugi::xml_document tDocument;
    XMLGen::Constraint tConstraint;
    tConstraint.criterion("11");
    tConstraint.service("22");
    ASSERT_NO_THROW(tInterface.call(tConstraint, "platomain_1", "Topology", "platomain", tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    std::vector<std::string> tGoldKeys = {"Name", "PerformerName", "Input", "Output", "Output"};
    std::vector<std::string> tGoldValues = {"Compute Constraint Value", "platomain_1", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Topology", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Volume", "Criterion Value - criterion_11_service_22_scenario_"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);

    tOutput = tOutput.next_sibling("Output");
    ASSERT_FALSE(tOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Volume Gradient", "Criterion Gradient - criterion_11_service_22_scenario_"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, ConstraintValueOperation_PlatoAnalyze)
{
    XMLGen::ConstraintValueOperation tInterface;
    pugi::xml_document tDocument;
    XMLGen::Constraint tConstraint;
    tConstraint.criterion("2");
    tConstraint.service("3");
    ASSERT_NO_THROW(tInterface.call(tConstraint, "plato_analyze_1", "Topology", "plato_analyze", tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    std::vector<std::string> tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    std::vector<std::string> tGoldValues = {"Compute Constraint Value", "plato_analyze_1", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Topology", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Constraint Value", "Criterion Value - criterion_2_service_3_scenario_"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, ConstraintGradientOperation_ErrorInvalidCode)
{
    XMLGen::ConstraintGradientOperation tInterface;
    pugi::xml_document tDocument;
    XMLGen::Constraint tConstraint;
    ASSERT_THROW(tInterface.call(tConstraint, "performer", "Topology", "hippo", tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ConstraintGradientOperation_PlatoMain)
{
    XMLGen::ConstraintGradientOperation tInterface;
    pugi::xml_document tDocument;
    XMLGen::Constraint tConstraint;
    tConstraint.criterion("1");
    tConstraint.service("1");
    ASSERT_NO_THROW(tInterface.call(tConstraint, "platomain_1", "Topology", "platomain", tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    std::vector<std::string> tGoldKeys = {"Name", "PerformerName", "Input", "Output", "Output"};
    std::vector<std::string> tGoldValues = {"Compute Constraint Gradient", "platomain_1", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Topology", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Volume", "Criterion Value - criterion_1_service_1_scenario_"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);

    tOutput = tOutput.next_sibling("Output");
    ASSERT_FALSE(tOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Volume Gradient", "Criterion Gradient - criterion_1_service_1_scenario_"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, ConstraintGradientOperation_PlatoAnalyze)
{
    XMLGen::ConstraintGradientOperation tInterface;
    pugi::xml_document tDocument;
    XMLGen::Constraint tConstraint;
    tConstraint.criterion("1");
    tConstraint.service("1");
    ASSERT_NO_THROW(tInterface.call(tConstraint, "plato_analyze_1", "Topology", "plato_analyze", tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    std::vector<std::string> tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    std::vector<std::string> tGoldValues = {"Compute Constraint Gradient", "plato_analyze_1", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Topology", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Constraint Gradient", "Criterion Gradient - criterion_1_service_1_scenario_"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, SetKeyValue)
{
    std::unordered_map<std::string, std::string> tKeyToValueMap =
        { {"ValueName", "Constraint Value"}, {"ValueStageName", "Compute Constraint Value"},
          {"GradientName", "Constraint Gradient"}, {"GradientStageName", "Compute Constraint Gradient"},
          {"ReferenceValueName", "Design Volume"}, {"NormalizedTargetValue", ""}, {"AbsoluteTargetValue", ""} };

    // TEST 1: EMPTY VALUE -> RESULT = IGNORE
    XMLGen::set_key_value("AbsoluteTargetValue", "", tKeyToValueMap);
    ASSERT_STREQ("IGNORE", tKeyToValueMap.find("AbsoluteTargetValue")->second.c_str());

    // TEST 2: SET VALUE
    XMLGen::set_key_value("AbsoluteTargetValue", "10", tKeyToValueMap);
    ASSERT_STREQ("10", tKeyToValueMap.find("AbsoluteTargetValue")->second.c_str());
}

TEST(PlatoTestXMLGenerator, SetValueKeywordToIgnoreIfEmpty)
{
    // TEST 1: SET EMPTY VALUES TO IGNORE
    std::vector<std::string> tValues = {"hello", "10", "", ""};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    std::vector<std::string> tGold = {"hello", "10", "IGNORE", "IGNORE"};
    for(auto& tValue : tValues)
    {
        auto tIndex = &tValue - &tValues[0];
        ASSERT_STREQ(tGold[tIndex].c_str(), tValue.c_str());
    }

    // TEST 2: NO CHANGE
    tValues = {"hello", "10"};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    tGold = {"hello", "10"};
    for(auto& tValue : tValues)
    {
        auto tIndex = &tValue - &tValues[0];
        ASSERT_STREQ(tGold[tIndex].c_str(), tValue.c_str());
    }
}

TEST(PlatoTestXMLGenerator, TransformKeyTokens)
{
    std::unordered_map<std::string, std::string> tKeyToValueMap =
        { {"ValueName", "Constraint Value"}, {"ValueStageName", "Compute Constraint Value"},
          {"GradientName", "Constraint Gradient"}, {"GradientStageName", "Compute Constraint Gradient"},
          {"ReferenceValueName", "Design Volume"}, {"NormalizedTargetValue", "1"}, {"AbsoluteTargetValue", "10"} };
    auto tKeys = XMLGen::transform_key_tokens(tKeyToValueMap);

    std::vector<std::string> tGold = {"ValueName", "ValueStageName", "GradientName", "GradientStageName",
        "ReferenceValueName", "NormalizedTargetValue", "AbsoluteTargetValue"};
    ASSERT_EQ(7u, tKeys.size());
    for(auto& tKey : tKeys)
    {
        auto tItr = std::find(tGold.begin(), tGold.end(), tKey);
        ASSERT_TRUE(tItr != tGold.end());
        ASSERT_STREQ(tItr->c_str(), tKey.c_str());
    }
}

TEST(PlatoTestXMLGenerator, TransformValueTokens)
{
    std::unordered_map<std::string, std::string> tKeyToValueMap =
        { {"ValueName", "Constraint Value"}, {"ValueStageName", "Compute Constraint Value"},
          {"GradientName", "Constraint Gradient"}, {"GradientStageName", "Compute Constraint Gradient"},
          {"ReferenceValueName", "Design Volume"}, {"NormalizedTargetValue", "1"}, {"AbsoluteTargetValue", "10"} };
    auto tValues = XMLGen::transform_value_tokens(tKeyToValueMap);

    std::vector<std::string> tGold = {"Constraint Value", "Compute Constraint Value",
        "Constraint Gradient", "Compute Constraint Gradient", "Design Volume", "1", "10"};
    ASSERT_EQ(7u, tValues.size());
    for(auto& tValue : tValues)
    {
        auto tItr = std::find(tGold.begin(), tGold.end(), tValue);
        ASSERT_TRUE(tItr != tGold.end());
        ASSERT_STREQ(tItr->c_str(), tValue.c_str());
    }
}

TEST(PlatoTestXMLGenerator, AppendChilds)
{
    pugi::xml_document tDocument;
    auto tSharedData = tDocument.append_child("SharedData");
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Lower Bound Value", "Scalar", "Global", "1", "platomain", "platomain"};
    XMLGen::append_children(tKeys, tValues, tSharedData);
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
}

TEST(PlatoTestXMLGenerator, AppendAttributes)
{
    pugi::xml_document tDocument;
    auto tFor = tDocument.append_child("For");
    ASSERT_STREQ("For", tFor.name());
    std::vector<std::string> tKeys = {"var", "in"};
    std::vector<std::string> tValues = {"PerformerIndex", "Performers"};
    XMLGen::append_attributes(tKeys, tValues, tFor);
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tFor);
}

TEST(PlatoTestXMLGenerator, AppendQoiStatisticsSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tInputData.append(tService);
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.appendRandomQoI("vonmises", "element field");
    tInputData.mOutputMetaData.push_back(tOutputMetadata);
    XMLGen::append_qoi_statistics_shared_data(tInputData, tDocument);

    auto tSharedData = tDocument.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"vonmises mean", "Scalar", "Element Field", "platomain_1", "platomain_1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"vonmises standard deviation", "Scalar", "Element Field", "platomain_1", "platomain_1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

}

TEST(PlatoTestXMLGenerator, AppendNondeterministicSharedData)
{
    pugi::xml_document tDocument;
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Scalar", "Global", "1", "plato analyze {PerformerIndex}", "platomain"};
    XMLGen::append_multiperformer_shared_data(tKeys, tValues, tDocument);

    // TEST RESULTS AGAINS GOLD VALUES
    std::vector<std::string> tGoldOuterAttributeKeys = {"var", "in"};
    std::vector<std::string> tGoldOuterAttributeValues = {"PerformerIndex", "Performers"};
    auto tOuterForNode = tDocument.child("For");
    ASSERT_FALSE(tOuterForNode.empty());
    PlatoTestXMLGenerator::test_attributes(tGoldOuterAttributeKeys, tGoldOuterAttributeValues, tOuterForNode);

    auto tInnerForNode = tOuterForNode.child("For");
    ASSERT_FALSE(tInnerForNode.empty());
    std::vector<std::string> tGoldInnerAttributeKeys = {"var", "in"};
    std::vector<std::string> tGoldInnerAttributeValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldInnerAttributeKeys, tGoldInnerAttributeValues, tInnerForNode);

    auto tSharedData = tInnerForNode.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
}

TEST(PlatoTestXMLGenerator, AppendNondeterministicCriterionSharedData_ErrorEmptyObjectiveList)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_multiperformer_criterion_shared_data("Objective", tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendNondeterministicCriterionSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tInputData.append(tService);
    tService.id("2");
    tService.code("plato_analyze");
    tInputData.append(tService);

    XMLGen::append_multiperformer_criterion_shared_data("Objective", tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldOuterAttributeKeys = {"var", "in"};
    std::vector<std::string> tGoldOuterAttributeValues = {"PerformerIndex", "Performers"};
    std::vector<std::string> tGoldInnerAttributeKeys = {"var", "in"};
    std::vector<std::string> tGoldInnerAttributeValues = {"PerformerSampleIndex", "PerformerSamples"};

    std::vector<std::string> tTemp = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataKeys;
    tGoldSharedDataKeys.push_back(std::make_pair("Objective Value", tTemp));
    tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tGoldSharedDataKeys.push_back(std::make_pair("Objective Gradient", tTemp));

    tTemp = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Scalar", "Global", "1", "plato_analyze_2_{PerformerIndex}", "platomain_1"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataValues;
    tGoldSharedDataValues.push_back(std::make_pair("Objective Value", tTemp));
    tTemp = {"Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Scalar", "Nodal Field", "plato_analyze_2_{PerformerIndex}", "platomain_1"};
    tGoldSharedDataValues.push_back(std::make_pair("Objective Gradient", tTemp));

    auto tKeys = tGoldSharedDataKeys.begin();
    auto tValues = tGoldSharedDataValues.begin();
    for (auto &tChild : tDocument.children())
    {
        // TEST OUTER LOOP ATTRIBUTES
        ASSERT_STREQ("For", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldOuterAttributeKeys, tGoldOuterAttributeValues, tChild);

        // TEST INNER LOOP ATTRIBUTES
        auto tGoldInnerAttributeKeysItr = tGoldInnerAttributeKeys.begin();
        auto tGoldInnerAttributeValuesItr = tGoldInnerAttributeValues.begin();
        auto tInnerForNode = tChild.child("For");
        for (auto &tAttribute : tInnerForNode.attributes())
        {
            ASSERT_STREQ(tGoldInnerAttributeKeysItr->c_str(), tAttribute.name());
            std::advance(tGoldInnerAttributeKeysItr, 1);
            ASSERT_STREQ(tGoldInnerAttributeValuesItr->c_str(), tAttribute.value());
            std::advance(tGoldInnerAttributeValuesItr, 1);

            // TEST INNER LOOP SHARED DATA
            auto tSharedData = tInnerForNode.child("SharedData");
            ASSERT_FALSE(tSharedData.empty());
            ASSERT_STREQ("SharedData", tSharedData.name());
            PlatoTestXMLGenerator::test_children(tKeys->second, tValues->second, tSharedData);
        }
        std::advance(tKeys, 1);
        std::advance(tValues, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendQoiSharedDataForNondeterministicUsecase)
{
    XMLGen::InputData tInputData;
    pugi::xml_document tDocument;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tInputData.append(tService);
    tService.id("2");
    tService.code("plato_analyze");
    tInputData.append(tService);
    tService.id("3");
    tService.code("sierra");
    tInputData.append(tService);
    XMLGen::Output tOutputMetadata;
    tOutputMetadata.appendRandomQoI("VonMises", "element field");
    tOutputMetadata.serviceID("2");
    tInputData.mOutputMetaData.push_back(tOutputMetadata);

    ASSERT_NO_THROW(XMLGen::append_multiperformer_qoi_shared_data(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldOuterAttributeKeys = {"var", "in"};
    std::vector<std::string> tGoldOuterAttributeValues = {"PerformerIndex", "Performers"};
    std::vector<std::string> tGoldInnerAttributeKeys = {"var", "in"};
    std::vector<std::string> tGoldInnerAttributeValues = {"PerformerSampleIndex", "PerformerSamples"};

    std::vector<std::string> tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataKeys;
    tGoldSharedDataKeys.push_back(std::make_pair("vonmises", tTemp));
    tTemp = {"vonmises {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Scalar", "Element Field", "plato_analyze_2_{PerformerIndex}", "platomain_1"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataValues;
    tGoldSharedDataValues.push_back(std::make_pair("vonmises", tTemp));

    auto tKeys = tGoldSharedDataKeys.begin();
    auto tValues = tGoldSharedDataValues.begin();
    for (auto &tChild : tDocument.children())
    {
        // TEST OUTER LOOP ATTRIBUTES
        ASSERT_STREQ("For", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldOuterAttributeKeys, tGoldOuterAttributeValues, tChild);

        // TEST INNER LOOP ATTRIBUTES
        auto tGoldInnerAttributeKeysItr = tGoldInnerAttributeKeys.begin();
        auto tGoldInnerAttributeValuesItr = tGoldInnerAttributeValues.begin();
        auto tInnerForNode = tChild.child("For");
        for (auto &tAttribute : tInnerForNode.attributes())
        {
            ASSERT_STREQ(tGoldInnerAttributeKeysItr->c_str(), tAttribute.name());
            std::advance(tGoldInnerAttributeKeysItr, 1);
            ASSERT_STREQ(tGoldInnerAttributeValuesItr->c_str(), tAttribute.value());
            std::advance(tGoldInnerAttributeValuesItr, 1);

            // TEST INNER LOOP SHARED DATA
            auto tSharedData = tInnerForNode.child("SharedData");
            ASSERT_FALSE(tSharedData.empty());
            ASSERT_STREQ("SharedData", tSharedData.name());
            PlatoTestXMLGenerator::test_children(tKeys->second, tValues->second, tSharedData);
        }
        std::advance(tKeys, 1);
        std::advance(tValues, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendLowerBoundsSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tInputData.append(tService);
    XMLGen::append_lower_bounds_shared_data(tInputData, tDocument);

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tTemp = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataKeys;
    tGoldSharedDataKeys.push_back(std::make_pair("Lower Bound Value", tTemp));
    tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tGoldSharedDataKeys.push_back(std::make_pair("Lower Bound Vector", tTemp));

    tTemp = {"Lower Bound Value", "Scalar", "Global", "1", "platomain_1", "platomain_1"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataValues;
    tGoldSharedDataValues.push_back(std::make_pair("Lower Bound Value", tTemp));
    tTemp = {"Lower Bound Vector", "Scalar", "Nodal Field", "platomain_1", "platomain_1"};
    tGoldSharedDataValues.push_back(std::make_pair("Lower Bound Vector", tTemp));

    auto tKeys = tGoldSharedDataKeys.begin();
    auto tValues = tGoldSharedDataValues.begin();
    for (auto &tOuterChild : tDocument.children())
    {
        PlatoTestXMLGenerator::test_children(tKeys->second, tValues->second, tOuterChild);
        std::advance(tKeys, 1);
        std::advance(tValues, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendUpperBoundsSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tInputData.append(tService);
    XMLGen::append_upper_bounds_shared_data(tInputData, tDocument);

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tTemp = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataKeys;
    tGoldSharedDataKeys.push_back(std::make_pair("Upper Bound Value", tTemp));
    tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tGoldSharedDataKeys.push_back(std::make_pair("Upper Bound Vector", tTemp));

    tTemp = {"Upper Bound Value", "Scalar", "Global", "1", "platomain_1", "platomain_1"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataValues;
    tGoldSharedDataValues.push_back(std::make_pair("Upper Bound Value", tTemp));
    tTemp = {"Upper Bound Vector", "Scalar", "Nodal Field", "platomain_1", "platomain_1"};
    tGoldSharedDataValues.push_back(std::make_pair("Upper Bound Vector", tTemp));

    auto tKeys = tGoldSharedDataKeys.begin();
    auto tValues = tGoldSharedDataValues.begin();
    for (auto &tOuterChild : tDocument.children())
    {
        PlatoTestXMLGenerator::test_children(tKeys->second, tValues->second, tOuterChild);
        std::advance(tKeys, 1);
        std::advance(tValues, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendDesignVolumeSaredData)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tInputData.append(tService);
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tInputData.set(tOptimizationParameters);
    XMLGen::append_design_volume_shared_data(tInputData, tDocument);

    // TEST RESULTS AGAINST GOLD VALUES
    auto tSharedData = tDocument.child("SharedData");
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tGoldKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tGoldValues = {"Design Volume", "Scalar", "Global", "1", "platomain_1", "platomain_1"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSharedData);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tInputData.append(tService);

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tInputData.append(tCriterion);

    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tInputData.append(tScenario);

    XMLGen::Objective tObjective;
    tObjective.scenarioIDs.push_back("1");
    tObjective.criteriaIDs.push_back("1");
    tObjective.serviceIDs.push_back("1");
    tInputData.objective = tObjective;

    ASSERT_NO_THROW(XMLGen::append_objective_shared_data(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tTemp = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataKeys;
    tGoldSharedDataKeys.push_back(std::make_pair("Objective Value ID-0", tTemp));
    tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tGoldSharedDataKeys.push_back(std::make_pair("Objective Gradient ID-0", tTemp));
    tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tGoldSharedDataKeys.push_back(std::make_pair("Objective Gradient", tTemp));

    tTemp = {"Objective Value", "Scalar", "Global", "1", "platomain_1", "platomain_1"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataValues;
    tGoldSharedDataValues.push_back(std::make_pair("Objective Value ID-0", tTemp));
    tTemp = {"Objective Gradient", "Scalar", "Nodal Field", "platomain_1", "platomain_1"};
    tGoldSharedDataValues.push_back(std::make_pair("Objective Gradient ID-0", tTemp));

    tTemp = {"Objective Gradient", "Scalar", "Nodal Field", "platomain_1", "platomain_1"};
    tGoldSharedDataValues.push_back(std::make_pair("Objective Gradient", tTemp));

    auto tKeys = tGoldSharedDataKeys.begin();
    auto tValues = tGoldSharedDataValues.begin();
    for (auto &tOuterChild : tDocument.children())
    {
        PlatoTestXMLGenerator::test_children(tKeys->second, tValues->second, tOuterChild);
        std::advance(tKeys, 1);
        std::advance(tValues, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendConstraintSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tInputData.append(tService);

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tInputData.append(tCriterion);

    XMLGen::Constraint tConstraint;
    tConstraint.service("1");
    tConstraint.criterion("1");
    tConstraint.id("1");
    tInputData.constraints.push_back(tConstraint);

    ASSERT_NO_THROW(XMLGen::append_constraint_shared_data(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tTemp = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataKeys;
    tGoldSharedDataKeys.push_back(std::make_pair("Constraint Value", tTemp));
    tTemp = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tGoldSharedDataKeys.push_back(std::make_pair("Constraint Gradient", tTemp));

    tTemp = {"Constraint Value 1", "Scalar", "Global", "1", "platomain_1", "platomain_1"};
    std::vector<std::pair<std::string, std::vector<std::string>>> tGoldSharedDataValues;
    tGoldSharedDataValues.push_back(std::make_pair("Constraint Value", tTemp));
    tTemp = {"Constraint Gradient 1", "Scalar", "Nodal Field", "platomain_1", "platomain_1"};
    tGoldSharedDataValues.push_back(std::make_pair("Constraint Gradient", tTemp));

    auto tKeys = tGoldSharedDataKeys.begin();
    auto tValues = tGoldSharedDataValues.begin();
    for (auto &tOuterChild : tDocument.children())
    {
        PlatoTestXMLGenerator::test_children(tKeys->second, tValues->second, tOuterChild);
        std::advance(tKeys, 1);
        std::advance(tValues, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendControlSharedData)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    
    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tInputData.append(tService);
    XMLGen::append_control_shared_data(tInputData, tDocument);

    // TEST RESULTS AGAINST GOLD VALUES
    auto tSharedData = tDocument.child("SharedData");
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tGoldKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    std::vector<std::string> tGoldValues = {"Control", "Scalar", "Nodal Field", "platomain_1", "platomain_1"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSharedData);
}

TEST(PlatoTestXMLGenerator, AppendTopologySharedDataForNondeterministicUseCase_ErrorEmptyObjectiveList)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;

    XMLGen::append_control_shared_data(tInputData, tDocument);
    ASSERT_THROW(XMLGen::append_multiperformer_topology_shared_data(tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendTopologySharedDataForNondeterministicUseCase)
{
    pugi::xml_document tDocument;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    XMLGen::InputData tInputData;
    tInputData.append(tService);

    ASSERT_NO_THROW(XMLGen::append_multiperformer_topology_shared_data(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tSharedData = tDocument.child("SharedData");
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tGoldKeys = {"Name", "Type", "Layout", "OwnerName", "UserName", "For"};
    std::vector<std::string> tGoldValues = {"Topology", "Scalar", "Nodal Field", "platomain", "platomain", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tSharedData);

    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    auto tForNode = tSharedData.child("For");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tForNode);
    PlatoTestXMLGenerator::test_children({"UserName"}, {"plato_analyze_1_{PerformerIndex}"}, tForNode);
}

TEST(PlatoTestXMLGenerator, AppendPlatoMainPerformer)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tInputData.append(tService);
    int tID = 0;
    XMLGen::append_plato_main_performer(tInputData, tID, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tPerformer = tDocument.child("Performer");
    std::vector<std::string> tGoldKeys = {"Name", "Code", "PerformerID"};
    std::vector<std::string> tGoldValues = {"platomain_1", "platomain", "0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tPerformer);
}

TEST(PlatoTestXMLGenerator, AppendPhysicsPerformersForNondeterministicUsecase_ErrorEmptyObjectiveList)
{
    pugi::xml_document tDocument;
    int tID = 0;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_physics_performers_multiperformer_usecase(tInputData, tID, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendPhysicsPerformersForNondeterministicUsecase)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tInputData.append(tService);
    tService.id("2");
    tService.code("plato_analyze");
    tInputData.append(tService);
    int tID = 0;

    ASSERT_NO_THROW(XMLGen::append_physics_performers_multiperformer_usecase(tInputData, tID, tDocument));

    auto tPerformer = tDocument.child("Performer");
    ASSERT_FALSE(tPerformer.empty());
    ASSERT_STREQ("Performer", tPerformer.name());
    auto tPerformerID = tPerformer.child("PerformerID");
    ASSERT_FALSE(tPerformerID.empty());
    ASSERT_STREQ("PerformerID", tPerformerID.name());
    auto tFor = tPerformer.child("For");
    ASSERT_FALSE(tFor.empty());
    ASSERT_STREQ("For", tFor.name());
    std::vector<std::string> tKeys = {"var", "in"};
    std::vector<std::string> tValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tKeys, tValues, tFor);
    tKeys = {"Name", "Code"};
    tValues = {"plato_analyze_2_{PerformerIndex}", "plato_analyze"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tFor);
    tFor = tFor.next_sibling("For");
    ASSERT_TRUE(tFor.empty());
}

TEST(PlatoTestXMLGenerator, AppendTopologySharedDataForNondeterministicUsecase)
{
    pugi::xml_document tDocument;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    XMLGen::InputData tInputData;
    tInputData.append(tService);

    ASSERT_NO_THROW(XMLGen::append_multiperformer_topology_shared_data(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tPerformer = tDocument.child("Performer");
    std::vector<std::string> tGoldKeys = {"PerformerID", "For"};
    std::vector<std::string> tGoldValues = {"1", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tPerformer);

    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    auto tForNode = tPerformer.child("For");
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tForNode);

    tGoldKeys = {"Name", "Code"};
    tGoldValues = {"plato analyze_{PerformerIndex}", "plato_analyze"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tForNode);
}

TEST(PlatoTestXMLGenerator, AppendFilterControlOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::append_filter_control_operation(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    std::vector<std::string> tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    std::vector<std::string> tGoldValues = {"Filter Control", "platomain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tInput = tDocument.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    auto tOutput = tDocument.child("Output");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendFilterCriterionGradientSamplesOperation)
{
    pugi::xml_document tDocument;
    XMLGen::append_filter_criterion_gradient_samples_operation("Objective", "platomain_1", tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    std::vector<std::string> tGoldKeys = {"Name", "PerformerName", "Input", "For"};
    std::vector<std::string> tGoldValues = {"Filter Gradient", "platomain_1", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    auto tFor = tOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tInput = tFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
    
    auto tOutput = tFor.child("Output");
    ASSERT_FALSE(tOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendFilterCriterionGradientOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::append_filter_criterion_gradient_operation(tInputData, "Objective Gradient", "Objective Gradient", tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    std::vector<std::string> tGoldKeys = {"Name", "PerformerName", "Input", "Input", "Output"};
    std::vector<std::string> tGoldValues = {"Filter Gradient", "platomain", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tOutput = tOperation.child("Output");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Gradient", "Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);

    auto tInput = tOperation.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    tInput = tInput.next_sibling("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Gradient", "Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
}

TEST(PlatoTestXMLGenerator, AppendInitialGuessStage)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::append_initial_guess_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Initial Guess", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOperation = tStage.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "Output"};
    tGoldValues = {"Initialize Field", "platomain", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tOutput = tOperation.child("Output");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Initialized Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);

    tOutput = tStage.child("Output");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendNondeterministicOperation)
{
    pugi::xml_document tDocument;
    std::vector<std::string> tKeys = {"Name", "PerformerName"};
    auto tPerformerName = std::string("plato analyze") + " {PerformerIndex}";
    std::vector<std::string> tValues = {"Cache State", tPerformerName};
    XMLGen::append_nondeterministic_operation(tKeys, tValues, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOuterFor = tDocument.child("For");
    std::vector<std::string> tGoldKeys = {"var", "in"};
    std::vector<std::string> tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tOuterFor);

    auto tInnerFor = tOuterFor.child("For");
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tInnerFor);

    auto tOperation = tInnerFor.child("Operation");
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendCacheStateStageForNondeterministicUsecase_EmptyStage)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::Service tService;
    tService.cacheState("false");
    tInputData.append(tService);
    ASSERT_NO_THROW(XMLGen::append_cache_state_stage_for_nondeterministic_usecase(tInputData, tDocument));
    auto tStage = tDocument.child("Stage");
    ASSERT_TRUE(tStage.empty());
}

TEST(PlatoTestXMLGenerator, AppendCacheStateStageForNondeterministicUsecase)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::Service tService;
    tService.cacheState("true");
    tService.code("plato_analyze");
    tService.id("1");
    tInputData.append(tService);
    ASSERT_NO_THROW(XMLGen::append_cache_state_stage_for_nondeterministic_usecase(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "For"};
    std::vector<std::string> tGoldValues = {"Cache State", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterFor = tStage.child("For");
    ASSERT_FALSE(tOuterFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tOuterFor);

    auto tInnerFor = tOuterFor.child("For");
    ASSERT_FALSE(tInnerFor.empty());
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tInnerFor);

    auto tOperation = tInnerFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName"};
    tGoldValues = {"Cache State", "plato_analyze_1_{PerformerIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemStageForNondeterministicUsecase_EmptyStage)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::Service tService;
    tService.updateProblem("false");
    tInputData.append(tService);
    ASSERT_NO_THROW(XMLGen::append_update_problem_stage_for_nondeterministic_usecase(tInputData, tDocument));
    auto tStage = tDocument.child("Stage");
    ASSERT_TRUE(tStage.empty());
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemStageForNondeterministicUsecase)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::Service tService;
    tService.updateProblem("true");
    tService.code("plato_analyze");
    tService.id("1");
    tInputData.append(tService);
    ASSERT_NO_THROW(XMLGen::append_update_problem_stage_for_nondeterministic_usecase(tInputData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "For"};
    std::vector<std::string> tGoldValues = {"Update Problem", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterFor = tStage.child("For");
    ASSERT_FALSE(tOuterFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tOuterFor);

    auto tInnerFor = tOuterFor.child("For");
    ASSERT_FALSE(tInnerFor.empty());
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tInnerFor);

    auto tOperation = tInnerFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName"};
    tGoldValues = {"Update Problem", "plato_analyze_1_{PerformerIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendLowerBoundStage)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tInputData.set(tOptimizationParameters);
    XMLGen::append_lower_bound_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Input", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Set Lower Bounds", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterInput = tStage.child("Input");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Lower Bound Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterInput);

    auto tOperation = tStage.child("Operation");
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Compute Lower Bounds", "platomain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tInnerInput = tOperation.child("Input");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Lower Bound Vector", "Lower Bound Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInnerInput);
    auto tInnerOutput = tOperation.child("Output");
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Lower Bound Vector", "Lower Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInnerOutput);

    auto tOuterOutput = tStage.child("Output");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Lower Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendLowerBoundStage_TypeNotEqualToplogy)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "inverse");
    tInputData.set(tOptimizationParameters);
    XMLGen::append_lower_bound_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    std::vector<std::string> tGoldKeys = {"Name", "Output"};
    std::vector<std::string> tGoldValues = {"Set Lower Bounds", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterOutput = tStage.child("Output");
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Lower Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendUpperBoundStage)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tInputData.set(tOptimizationParameters);
    XMLGen::append_upper_bound_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Input", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Set Upper Bounds", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterInput = tStage.child("Input");
    ASSERT_FALSE(tOuterInput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Upper Bound Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterInput);

    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Compute Upper Bounds", "platomain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tInnerInput = tOperation.child("Input");
    ASSERT_FALSE(tInnerInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Upper Bound Vector", "Upper Bound Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInnerInput);
    auto tInnerOutput = tOperation.child("Output");
    ASSERT_FALSE(tInnerOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Upper Bound Vector", "Upper Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInnerOutput);

    auto tOuterOutput = tStage.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Upper Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendUpperBoundStage_TypeNotEqualToplogy)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "inverse");
    tInputData.set(tOptimizationParameters);
    XMLGen::append_upper_bound_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Output"};
    std::vector<std::string> tGoldValues = {"Set Upper Bounds", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterOutput = tStage.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Upper Bound Vector"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendDesignVolumeStage)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tInputData.set(tOptimizationParameters);
    XMLGen::append_design_volume_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Design Volume", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Output"};
    tGoldValues = {"Design Volume", "platomain", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tInnerOutput = tOperation.child("Output");
    ASSERT_FALSE(tInnerOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Design Volume", "Design Volume"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInnerOutput);

    auto tOuterOutput = tStage.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Design Volume"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendConstraintValueStage)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tInputData.append(tService);

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("volume");
    tInputData.append(tCriterion);

    XMLGen::Constraint tConstraint;
    tConstraint.id("3");
    tConstraint.service("1");
    tConstraint.criterion("1");
    tInputData.constraints.push_back(tConstraint);

    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tInputData.set(tOptimizationParameters);

    XMLGen::append_constraint_value_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Type", "Input", "Operation", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Compute Constraint Value 3", "volume", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tStageInput = tStage.child("Input");
    ASSERT_FALSE(tStageInput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageInput);

    auto tStageOutput = tStage.child("Output");
    ASSERT_FALSE(tStageOutput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Constraint Value 3"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOutput);

    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Filter Control", "platomain_1", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tFilterInput = tOperation.child("Input");
    ASSERT_FALSE(tFilterInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);
    auto tFilterOutput = tOperation.child("Output");
    ASSERT_FALSE(tFilterOutput.empty());
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);

    tOperation = tOperation.next_sibling("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output", "Output"};
    tGoldValues = {"Compute Constraint Value", "platomain_1", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tConstraintInput = tOperation.child("Input");
    ASSERT_FALSE(tConstraintInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Topology", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintInput);

    auto tConstraintOutput = tOperation.child("Output");
    ASSERT_FALSE(tConstraintOutput.empty());
    tGoldValues = {"Volume", "Criterion Value - criterion_1_service_1_scenario_"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintOutput);
    tConstraintOutput = tConstraintOutput.next_sibling("Output");
    ASSERT_FALSE(tConstraintOutput.empty());
    tGoldValues = {"Volume Gradient", "Criterion Gradient - criterion_1_service_1_scenario_"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintOutput);
}

TEST(PlatoTestXMLGenerator, AppendConstraintGradientStage)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tInputData.append(tService);

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("volume");
    tInputData.append(tCriterion);

    XMLGen::Constraint tConstraint;
    tConstraint.id("1");
    tConstraint.service("1");
    tConstraint.criterion("1");
    tInputData.constraints.push_back(tConstraint);

    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tInputData.set(tOptimizationParameters);

    XMLGen::append_constraint_gradient_stage(tInputData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Type", "Input", "Operation", "Operation", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Compute Constraint Gradient 1", "volume", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tOuterInput = tStage.child("Input");
    ASSERT_FALSE(tOuterInput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterInput);

    auto tOuterOutput = tStage.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Constraint Gradient 1"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOuterOutput);

    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Filter Control", "platomain_1", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tFilterInput = tOperation.child("Input");
    ASSERT_FALSE(tFilterInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);
    auto tFilterOutput = tOperation.child("Output");
    ASSERT_FALSE(tFilterOutput.empty());
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);

    tOperation = tOperation.next_sibling("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output", "Output"};
    tGoldValues = {"Compute Constraint Gradient", "platomain_1", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    auto tConstraintInput = tOperation.child("Input");
    ASSERT_FALSE(tConstraintInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Topology", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintInput);
    auto tConstraintOutput = tOperation.child("Output");
    ASSERT_FALSE(tConstraintOutput.empty());
    tGoldValues = {"Volume", "Criterion Value - criterion_1_service_1_scenario_"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintOutput);
    tConstraintOutput = tOperation.next_sibling("Output");
    ASSERT_FALSE(tConstraintOutput.empty());
    tGoldValues = {"Volume Gradient", "Constraint Gradient 1"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintOutput);

    tOperation = tOperation.next_sibling("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Input", "Output"};
    tGoldValues = {"Filter Gradient", "platomain_1", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);
    tFilterInput = tOperation.child("Input");
    ASSERT_FALSE(tFilterInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);
    tFilterInput = tFilterInput.next_sibling("Input");
    ASSERT_FALSE(tFilterInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Gradient", "Criterion Gradient - criterion_1_service_1_scenario_"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);
    tFilterOutput = tOperation.child("Output");
    ASSERT_FALSE(tFilterOutput.empty());
    tGoldValues = {"Filtered Gradient", "Constraint Gradient 1"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);
}

TEST(PlatoTestXMLGenerator, AppendNondeterministicParameters)
{
    pugi::xml_document tDocument;
    auto tOperation = tDocument.append_child("Operation");
    std::unordered_map<std::string, std::vector<std::string>> tTags =
        { { "0", {"traction_load_id_0_x_axis", "traction_load_id_0_y_axis", "traction_load_id_0_z_axis"} } };
    XMLGen::append_nondeterministic_parameters(tTags, tOperation);
    ASSERT_FALSE(tOperation.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"Parameter", "Parameter", "Parameter"};
    std::vector<std::string> tGoldValues = {"", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction_load_id_0_x_axis",
                   "{traction_load_id_0_x_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_0_y_axis",
                   "{traction_load_id_0_y_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_0_z_axis",
                   "{traction_load_id_0_z_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);
}

TEST(PlatoTestXMLGenerator, AppendSampleObjectiveValueOperation)
{
    XMLGen::InputData tXMLMetaData;

    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::NaturalBoundaryCondition tLoad1;
    tLoad1.id("1");
    tLoad1.is_random("true");
    tLoad1.type("traction");
    tLoad1.location_name("sideset");
    std::vector<std::string> tValues;
    tValues.push_back("1");
    tValues.push_back("2");
    tValues.push_back("3");
    tLoad1.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::NaturalBoundaryCondition tLoad2;
    tLoad2.id("2");
    tLoad2.is_random("true");
    tLoad2.type("traction");
    tLoad2.location_name("sideset");
    tValues.clear();
    tValues.push_back("4");
    tValues.push_back("5");
    tValues.push_back("6");
    tLoad2.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::NaturalBoundaryCondition tLoad3;
    tLoad3.id("3");
    tLoad3.type("traction");
    tLoad3.is_random("false");
    tLoad3.location_name("sideset");
    tValues.clear();
    tValues.push_back("7");
    tValues.push_back("8");
    tValues.push_back("9");
    tLoad3.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::NaturalBoundaryCondition tLoad4;
    tLoad4.id("1");
    tLoad4.is_random("true");
    tLoad4.type("traction");
    tLoad4.location_name("sideset");
    tValues.clear();
    tValues.push_back("11");
    tValues.push_back("12");
    tValues.push_back("13");
    tLoad4.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::NaturalBoundaryCondition tLoad5;
    tLoad5.id("2");
    tLoad5.is_random("true");
    tLoad5.type("traction");
    tLoad5.location_name("sideset");
    tValues.clear();
    tValues.push_back("14");
    tValues.push_back("15");
    tValues.push_back("16");
    tLoad5.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // Pose Criterion
    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tXMLMetaData.append(tCriterion);

    // Pose Service	
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tXMLMetaData.append(tService);
    
    // POSE OBJECTIVE
    XMLGen::Objective tObjective;
    tObjective.criteriaIDs.push_back("1");
    tObjective.criteriaIDs.push_back("1");
    tObjective.serviceIDs.push_back("1");
    tObjective.serviceIDs.push_back("1");
    tObjective.scenarioIDs.push_back("1");
    tObjective.scenarioIDs.push_back("2");
    
    // DEFINE XML GENERATOR INPUT DATA
    tXMLMetaData.objective = tObjective;
    tXMLMetaData.mRandomMetaData = tRandomMetaData;

    // CALL FUNCTION
    pugi::xml_document tDocument;
    auto tStage = tDocument.append_child("Stage");
    XMLGen::append_sample_objective_value_operation(tXMLMetaData, tStage);
    ASSERT_FALSE(tStage.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"For"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);
    auto tFor = tStage.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tOperation = tFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tFor = tOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tOperation = tFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Parameter", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Input", "Output"};
    tGoldValues = {"Compute Objective Value", "plato_analyze_1_{PerformerIndex}",
        "", "", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    // TEST PARAMETERS AGAINST GOLD VALUES
    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction_load_id_2_x_axis",
                   "{traction_load_id_2_x_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_2_y_axis",
                   "{traction_load_id_2_y_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_2_z_axis",
                   "{traction_load_id_2_z_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction_load_id_1_x_axis",
                   "{traction_load_id_1_x_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_1_y_axis",
                   "{traction_load_id_1_y_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_1_z_axis",
                   "{traction_load_id_1_z_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    // TEST OPERATION INPUTS AND OUTPUTS AGAINST GOLD VALUES
    auto tOperationInput = tOperation.child("Input");
    ASSERT_FALSE(tOperationInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Topology", "Topology" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationInput);

    auto tOperationOutput = tOperation.child("Output");
    ASSERT_FALSE(tOperationOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Objective Value", "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationOutput);
}

TEST(PlatoTestXMLGenerator, AppendEvaluateNondeterministicCriterionValueOperation)
{
    // CALL FUNCTION
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("objective_number_standard_deviations", "1");
    tXMLMetaData.set(tOptimizationParameters);
    pugi::xml_document tDocument;
    auto tStage = tDocument.append_child("Stage");
    XMLGen::append_evaluate_nondeterministic_objective_value_operation("Objective Value", tXMLMetaData, tStage);
    ASSERT_FALSE(tStage.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"Operation"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);
    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "For", "Output"};
    tGoldValues = {"Compute Non-Deterministic Objective Value", "platomain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tFor = tOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tInput = tFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Mean Plus 1 StdDev", "Objective Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveValueStageForNondeterministicUsecase)
{
    XMLGen::InputData tXMLMetaData;

    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::NaturalBoundaryCondition tLoad1;
    tLoad1.id("1");
    tLoad1.is_random("true");
    tLoad1.type("traction");
    tLoad1.location_name("sideset");
    std::vector<std::string> tValues;
    tValues.push_back("1");
    tValues.push_back("2");
    tValues.push_back("3");
    tLoad1.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::NaturalBoundaryCondition tLoad2;
    tLoad2.id("2");
    tLoad2.is_random("true");
    tLoad2.type("traction");
    tLoad2.location_name("sideset");
    tValues.clear();
    tValues.push_back("4");
    tValues.push_back("5");
    tValues.push_back("6");
    tLoad2.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::NaturalBoundaryCondition tLoad3;
    tLoad3.id("3");
    tLoad3.type("traction");
    tLoad3.is_random("false");
    tLoad3.location_name("sideset");
    tValues.clear();
    tValues.push_back("7");
    tValues.push_back("8");
    tValues.push_back("9");
    tLoad3.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::NaturalBoundaryCondition tLoad4;
    tLoad4.id("1");
    tLoad4.is_random("true");
    tLoad4.type("traction");
    tLoad4.location_name("sideset");
    tValues.clear();
    tValues.push_back("11");
    tValues.push_back("12");
    tValues.push_back("13");
    tLoad4.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::NaturalBoundaryCondition tLoad5;
    tLoad5.id("2");
    tLoad5.is_random("true");
    tLoad5.type("traction");
    tLoad5.location_name("sideset");
    tValues.clear();
    tValues.push_back("14");
    tValues.push_back("15");
    tValues.push_back("16");
    tLoad5.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // Pose Criterion
    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tXMLMetaData.append(tCriterion);

    // Pose Service	
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tXMLMetaData.append(tService);
    tService.id("2");
    tService.code("plato_analyze");
    tXMLMetaData.append(tService);
    
    // POSE OBJECTIVE
    XMLGen::Objective tObjective;
    tObjective.criteriaIDs.push_back("1");
    tObjective.criteriaIDs.push_back("1");
    tObjective.serviceIDs.push_back("1");
    tObjective.serviceIDs.push_back("2");
    tObjective.scenarioIDs.push_back("1");
    tObjective.scenarioIDs.push_back("2");
    
    // DEFINE XML GENERATOR INPUT DATA
    tXMLMetaData.objective = tObjective;
    tXMLMetaData.mRandomMetaData = tRandomMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("objective_number_standard_deviations", "1");
    tOptimizationParameters.append("optimization_type", "topology");
    tXMLMetaData.set(tOptimizationParameters);

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_objective_value_stage(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // ****** 1) TEST RESULTS AGAINST STAGE GOLD VALUES ******
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Input", "Operation", "For", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Compute Objective Value", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tStageInput= tStage.child("Input");
    ASSERT_FALSE(tStageInput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageInput);

    auto tStageOutput= tStage.child("Output");
    ASSERT_FALSE(tStageOutput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Objective Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOutput);

    // ****** 2) TEST RESULTS AGAINST FILTER OPERATION GOLD VALUES ******
    auto tStageOperation = tStage.child("Operation");
    ASSERT_FALSE(tStageOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Filter Control", "platomain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOperation);

    auto tFilterInput = tStageOperation.child("Input");
    ASSERT_FALSE(tFilterInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);

    auto tFilterOutput = tStageOperation.child("Output");
    ASSERT_FALSE(tFilterOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);

    // ****** 4) TEST RESULTS AGAINST SAMPLE OBJECTIVE GOLD VALUES ******
    auto tFor = tStage.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tOperation = tFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tFor = tOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tOperation = tFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Parameter", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Input", "Output"};
    tGoldValues = {"Compute Objective Value", "plato_analyze_1_{PerformerIndex}",
        "", "", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    // ****** 4.1) TEST PARAMETERS AGAINST GOLD VALUES ******
    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction_load_id_2_x_axis",
                   "{traction_load_id_2_x_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_2_y_axis",
                   "{traction_load_id_2_y_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_2_z_axis",
                   "{traction_load_id_2_z_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction_load_id_1_x_axis",
                   "{traction_load_id_1_x_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_1_y_axis",
                   "{traction_load_id_1_y_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_1_z_axis",
                   "{traction_load_id_1_z_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    // ****** 4.2) TEST SAMPLE OBJECTIVE OPERATION INPUTS AND OUTPUTS AGAINST GOLD VALUES ******
    auto tOperationInput = tOperation.child("Input");
    ASSERT_FALSE(tOperationInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Topology", "Topology" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationInput);

    auto tOperationOutput = tOperation.child("Output");
    ASSERT_FALSE(tOperationOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Objective Value", "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationOutput);

    // ****** 3) TEST RESULTS AGAINST EVALUATE RANDOM OBJECTIVE OPERATION GOLD VALUES ******
    tStageOperation = tStageOperation.next_sibling("Operation");
    ASSERT_FALSE(tStageOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "For", "Output"};
    tGoldValues = {"Compute Non-Deterministic Objective Value", "platomain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOperation);

    tFor = tStageOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tRandomObjectiveInput = tFor.child("Input");
    ASSERT_FALSE(tRandomObjectiveInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tRandomObjectiveInput);

    auto tRandomObjectiveOutput = tStageOperation.child("Output");
    ASSERT_FALSE(tRandomObjectiveOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Mean Plus 1 StdDev", "Objective Value"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tRandomObjectiveOutput);
}

TEST(PlatoTestXMLGenerator, AppendSampleObjectiveGradientOperation)
{
    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::NaturalBoundaryCondition tLoad1;
    tLoad1.id("1");
    tLoad1.is_random("true");
    tLoad1.type("traction");
    tLoad1.location_name("sideset");
    std::vector<std::string> tValues;
    tValues.push_back("1");
    tValues.push_back("2");
    tValues.push_back("3");
    tLoad1.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::NaturalBoundaryCondition tLoad2;
    tLoad2.id("2");
    tLoad2.is_random("true");
    tLoad2.type("traction");
    tLoad2.location_name("sideset");
    tValues.clear();
    tValues.push_back("4");
    tValues.push_back("5");
    tValues.push_back("6");
    tLoad2.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::NaturalBoundaryCondition tLoad3;
    tLoad3.id("3");
    tLoad3.type("traction");
    tLoad3.is_random("false");
    tLoad3.location_name("sideset");
    tValues.clear();
    tValues.push_back("7");
    tValues.push_back("8");
    tValues.push_back("9");
    tLoad3.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::NaturalBoundaryCondition tLoad4;
    tLoad4.id("1");
    tLoad4.is_random("true");
    tLoad4.type("traction");
    tLoad4.location_name("sideset");
    tValues.clear();
    tValues.push_back("11");
    tValues.push_back("12");
    tValues.push_back("13");
    tLoad4.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::NaturalBoundaryCondition tLoad5;
    tLoad5.id("2");
    tLoad5.is_random("true");
    tLoad5.type("traction");
    tLoad5.location_name("sideset");
    tValues.clear();
    tValues.push_back("14");
    tValues.push_back("15");
    tValues.push_back("16");
    tLoad5.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // Service
    XMLGen::InputData tXMLMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tXMLMetaData.append(tService);

    // POSE OBJECTIVE
    XMLGen::Objective tObjective;
    tObjective.serviceIDs.push_back("1");

    // DEFINE XML GENERATOR INPUT DATA
    tXMLMetaData.objective = tObjective;
    tXMLMetaData.mRandomMetaData = tRandomMetaData;

    // CALL FUNCTION
    pugi::xml_document tDocument;
    auto tStage = tDocument.append_child("Stage");
    XMLGen::append_sample_objective_gradient_operation(tXMLMetaData, tStage);
    ASSERT_FALSE(tStage.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"For"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);
    auto tFor = tStage.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("Operation").child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tOperation = tFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Parameter", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Input", "Output"};
    tGoldValues = {"Compute Objective Gradient", "plato_analyze_1_{PerformerIndex}",
        "", "", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    // TEST PARAMETERS AGAINST GOLD VALUES
    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction_load_id_2_x_axis",
                   "{traction_load_id_2_x_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_2_y_axis",
                   "{traction_load_id_2_y_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_2_z_axis",
                   "{traction_load_id_2_z_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction_load_id_1_x_axis",
                   "{traction_load_id_1_x_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_1_y_axis",
                   "{traction_load_id_1_y_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_1_z_axis",
                   "{traction_load_id_1_z_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    // TEST OPERATION INPUTS AND OUTPUTS AGAINST GOLD VALUES
    auto tOperationInput = tOperation.child("Input");
    ASSERT_FALSE(tOperationInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Topology", "Topology" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationInput);

    auto tOperationOutput = tOperation.child("Output");
    ASSERT_FALSE(tOperationOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Objective Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationOutput);
}

TEST(PlatoTestXMLGenerator, AppendEvaluateNondeterministicCriterionGradientOperation)
{
    // CALL FUNCTION
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("objective_number_standard_deviations", "2");
    tXMLMetaData.set(tOptimizationParameters);
    pugi::xml_document tDocument;
    auto tStage = tDocument.append_child("Stage");
    XMLGen::append_evaluate_nondeterministic_objective_gradient_operation("Objective Gradient ID-0", tXMLMetaData, tStage);
    ASSERT_FALSE(tStage.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    std::vector<std::string> tGoldKeys = {"Operation"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);
    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "For", "Output"};
    tGoldValues = {"Compute Non-Deterministic Objective Gradient", "platomain", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    auto tFor = tOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tInput = tFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldValues = {"Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Mean Plus 2 StdDev Gradient", "Objective Gradient ID-0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveGradientStageForNondeterministicUsecase)
{
    XMLGen::InputData tXMLMetaData;

    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::NaturalBoundaryCondition tLoad1;
    tLoad1.id("1");
    tLoad1.is_random("true");
    tLoad1.type("traction");
    tLoad1.location_name("sideset");
    std::vector<std::string> tValues;
    tValues.push_back("1");
    tValues.push_back("2");
    tValues.push_back("3");
    tLoad1.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::NaturalBoundaryCondition tLoad2;
    tLoad2.id("2");
    tLoad2.is_random("true");
    tLoad2.type("traction");
    tLoad2.location_name("sideset");
    tValues.clear();
    tValues.push_back("4");
    tValues.push_back("5");
    tValues.push_back("6");
    tLoad2.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::NaturalBoundaryCondition tLoad3;
    tLoad3.id("3");
    tLoad3.type("traction");
    tLoad3.is_random("false");
    tLoad3.location_name("sideset");
    tValues.clear();
    tValues.push_back("7");
    tValues.push_back("8");
    tValues.push_back("9");
    tLoad3.load_values(tValues);
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::NaturalBoundaryCondition tLoad4;
    tLoad4.id("1");
    tLoad4.is_random("true");
    tLoad4.type("traction");
    tLoad4.location_name("sideset");
    tValues.clear();
    tValues.push_back("11");
    tValues.push_back("12");
    tValues.push_back("13");
    tLoad4.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::NaturalBoundaryCondition tLoad5;
    tLoad5.id("2");
    tLoad5.is_random("true");
    tLoad5.type("traction");
    tLoad5.location_name("sideset");
    tValues.clear();
    tValues.push_back("14");
    tValues.push_back("15");
    tValues.push_back("16");
    tLoad5.load_values(tValues);
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // Pose Criterion
    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tXMLMetaData.append(tCriterion);

    // Pose Service	
    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tXMLMetaData.append(tService);
    tService.id("2");
    tService.code("plato_analyze");
    tXMLMetaData.append(tService);

    // POSE OBJECTIVE
    XMLGen::Objective tObjective;
    tObjective.criteriaIDs.push_back("1");
    tObjective.criteriaIDs.push_back("1");
    tObjective.serviceIDs.push_back("2");
    tObjective.serviceIDs.push_back("2");
    tObjective.scenarioIDs.push_back("1");
    tObjective.scenarioIDs.push_back("2");

    // DEFINE XML GENERATOR INPUT DATA
    tXMLMetaData.objective = tObjective;
    tXMLMetaData.mRandomMetaData = tRandomMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("objective_number_standard_deviations", "3");
    tOptimizationParameters.append("filter_in_engine", "true");
    tOptimizationParameters.append("optimization_type", "topology");
    tXMLMetaData.set(tOptimizationParameters);

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_objective_gradient_stage(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // ****** 1) TEST RESULTS AGAINST STAGE GOLD VALUES ******
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    std::vector<std::string> tGoldKeys = {"Name", "Type", "Input", "Operation", "For", "For", "Operation", "Output"};
    std::vector<std::string> tGoldValues = {"Compute Objective Gradient", "maximize stiffness", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStage);

    auto tStageInput= tStage.child("Input");
    ASSERT_FALSE(tStageInput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageInput);

    auto tStageOutput= tStage.child("Output");
    ASSERT_FALSE(tStageOutput.empty());
    tGoldKeys = {"SharedDataName"};
    tGoldValues = {"Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOutput);

    // ****** 2) TEST RESULTS AGAINST FILTER OPERATION GOLD VALUES ******
    auto tStageOperation = tStage.child("Operation");
    ASSERT_FALSE(tStageOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Output"};
    tGoldValues = {"Filter Control", "platomain_1", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOperation);

    auto tFilterInput = tStageOperation.child("Input");
    ASSERT_FALSE(tFilterInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterInput);

    auto tFilterOutput = tStageOperation.child("Output");
    ASSERT_FALSE(tFilterOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Field", "Topology"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tFilterOutput);

    // ****** 3) TEST RESULTS AGAINST SAMPLE OBJECTIVE GRADIENT GOLD VALUES ******
    auto tStageOuterFor = tStage.child("For");
    ASSERT_FALSE(tStageOuterFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tStageOuterFor);

    auto tStageInnerFor = tStageOuterFor.child("Operation").child("For");
    ASSERT_FALSE(tStageInnerFor.empty());
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tStageInnerFor);

    auto tOperation = tStageInnerFor.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Parameter", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Input", "Output"};
    tGoldValues = {"Compute Objective Gradient", "plato_analyze_2_{PerformerIndex}",
        "", "", "", "", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperation);

    // ****** 3.1) TEST PARAMETERS AGAINST GOLD VALUES ******
    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction_load_id_2_x_axis",
                   "{traction_load_id_2_x_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_2_y_axis",
                   "{traction_load_id_2_y_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_2_z_axis",
                   "{traction_load_id_2_z_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldKeys = {"ArgumentName", "ArgumentValue"};
    tGoldValues = {"traction_load_id_1_x_axis",
                   "{traction_load_id_1_x_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_1_y_axis",
                   "{traction_load_id_1_y_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    tGoldValues = {"traction_load_id_1_z_axis",
                   "{traction_load_id_1_z_axis[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tParameter);

    // ****** 3.2) TEST OPERATION INPUTS AND OUTPUTS AGAINST GOLD VALUES ******
    auto tOperationInput = tOperation.child("Input");
    ASSERT_FALSE(tOperationInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Topology", "Topology" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationInput);

    auto tOperationOutput = tOperation.child("Output");
    ASSERT_FALSE(tOperationOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = { "Objective Gradient", "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}" };
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationOutput);

    // ****** 4) TEST RESULTS AGAINST EVALUATE RANDOM OBJECTIVE GRADIENT OPERATION GOLD VALUES ******
    tStageOperation = tStageOperation.next_sibling("Operation");
    ASSERT_FALSE(tStageOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "For", "Output"};
    tGoldValues = {"Compute Non-Deterministic Objective Gradient", "platomain_1", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOperation);

    auto tFor = tStageOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldKeys = {"var", "in"};
    tGoldValues = {"PerformerIndex", "Performers"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    tFor = tFor.child("For");
    ASSERT_FALSE(tFor.empty());
    tGoldValues = {"PerformerSampleIndex", "PerformerSamples"};
    PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValues, tFor);

    auto tForInput = tFor.child("Input");
    ASSERT_FALSE(tForInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tForInput);

    tForInput = tForInput.next_sibling("Input");
    ASSERT_FALSE(tForInput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tForInput);

    tOperationOutput = tStageOperation.child("Output");
    ASSERT_FALSE(tOperationOutput.empty());
    tGoldKeys = { "ArgumentName", "SharedDataName" };
    tGoldValues = {"Objective Mean Plus 3 StdDev Gradient", "Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOperationOutput);

    // ****** 5) TEST RESULTS AGAINST FILTER GRADIENT OPERATION GOLD VALUES ******
    tStageOperation = tStageOperation.next_sibling("Operation");
    ASSERT_FALSE(tStageOperation.empty());
    tGoldKeys = {"Name", "PerformerName", "Input", "Input", "Output"};
    tGoldValues = {"Filter Gradient", "platomain_1", "", "", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tStageOperation);

    auto tInput = tStageOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Field", "Control"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);

    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Gradient", "Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tInput);
    
    auto tOutput = tStageOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    tGoldKeys = {"ArgumentName", "SharedDataName"};
    tGoldValues = {"Filtered Gradient", "Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendDerivativeCheckerOptions)
{
    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("check_gradient", "true");
    tOptimizationParameters.append("check_hessian", "true");
    tOptimizationParameters.append("derivative_checker_initial_superscript", "1");
    tOptimizationParameters.append("derivative_checker_final_superscript", "8");
    tXMLMetaData.set(tOptimizationParameters);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_derivative_checker_options(tXMLMetaData, tOptimizerNode);
    ASSERT_FALSE(tOptimizerNode.empty());

    // ****** TEST RESULTS AGAINST OPTIMIZER NODE GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"CheckGradient", "CheckHessian", "UseUserInitialGuess", "Options"};
    std::vector<std::string> tGoldValues = {"true", "true", "True", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOptionsNode = tOptimizerNode.child("Options");
    tGoldKeys = {"DerivativeCheckerInitialSuperscript", "DerivativeCheckerFinalSuperscript"};
    tGoldValues = {"1", "8"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptionsNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOC_Options)
{
    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("max_iterations", "11");
    tXMLMetaData.set(tOptimizationParameters);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimality_criteria_options(tXMLMetaData, tOptimizerNode);
    ASSERT_FALSE(tOptimizerNode.empty());

    // ****** TEST RESULTS AGAINST OPTIMIZER NODE GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Options", "Convergence"};
    std::vector<std::string> tGoldValues = {"",""};

    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tConvergenceNode = tOptimizerNode.child("Convergence");
    tGoldKeys = {"MaxIterations"};
    tGoldValues = {"11"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConvergenceNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmMMA_Options)
{
    // 1) TEST CASE WHERE ONLY A FEW PARAMETERS ARE DEFINED
    pugi::xml_document tDocument1;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("max_iterations", "11");
    tOptimizationParameters.append("mma_move_limit", "0.2");
    tXMLMetaData.set(tOptimizationParameters);
    auto tOptimizerNode = tDocument1.append_child("Optimizer");
    XMLGen::append_method_moving_asymptotes_options(tXMLMetaData, tOptimizerNode);
    ASSERT_FALSE(tOptimizerNode.empty());

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Options"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOptionsNode = tOptimizerNode.child("Options");
    tGoldKeys = {"MaxNumOuterIterations", "MoveLimit"};
    tGoldValues = {"11", "0.2"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptionsNode);

    // 2) TEST CASE WHERE ALL THE PARAMETERS ARE DEFINED
    pugi::xml_document tDocument2;
    tOptimizationParameters.append("mma_asymptote_expansion", "2");
    tOptimizationParameters.append("mma_asymptote_contraction", "0.75");
    tOptimizationParameters.append("mma_max_sub_problem_iterations", "50");
    tOptimizationParameters.append("mma_control_stagnation_tolerance", "1e-3");
    tOptimizationParameters.append("mma_objective_stagnation_tolerance", "1e-8");
    tXMLMetaData.set(tOptimizationParameters);
    tOptimizerNode = tDocument2.append_child("Optimizer");
    XMLGen::append_method_moving_asymptotes_options(tXMLMetaData, tOptimizerNode);
    ASSERT_FALSE(tOptimizerNode.empty());

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    tGoldKeys = {"Options"};
    tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    tOptionsNode = tOptimizerNode.child("Options");
    tGoldKeys = {"MaxNumOuterIterations", "MoveLimit", "AsymptoteExpansion", "AsymptoteContraction",
        "MaxNumSubProblemIter", "ControlStagnationTolerance", "ObjectiveStagnationTolerance"};
    tGoldValues = {"11", "0.2", "2", "0.75", "50", "1e-3", "1e-8"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptionsNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOptions_ErrorOptimizerNotSupported)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_algorithm", "stochastic gradient descent");
    tXMLMetaData.set(tOptimizationParameters);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_THROW(XMLGen::append_optimization_algorithm_parameters_options(tXMLMetaData, tOptimizerNode), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOptionsKSBC)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("ks_trust_region_contraction_factor", "0.5");
    tOptimizationParameters.append("ks_trust_region_expansion_factor", "4.0");
    tOptimizationParameters.append("ks_disable_post_smoothing", "false");
    tOptimizationParameters.append("optimization_algorithm", "KSbc");
    tXMLMetaData.set(tOptimizationParameters);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_NO_THROW(XMLGen::append_optimization_algorithm_parameters_options(tXMLMetaData, tOptimizerNode));
    ASSERT_FALSE(tOptimizerNode.empty());

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Options","Convergence"};
    std::vector<std::string> tGoldValues = {"",""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOptionsNode = tOptimizerNode.child("Options");
    tGoldKeys = {"KSTrustRegionExpansionFactor", "KSTrustRegionContractionFactor", "DisablePostSmoothing"};
    tGoldValues = {"4.0", "0.5", "false"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptionsNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOptionsMMA)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("max_iterations", "11");
    tOptimizationParameters.append("mma_move_limit", "0.2");
    tOptimizationParameters.append("optimization_algorithm", "MmA");
    tXMLMetaData.set(tOptimizationParameters);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_NO_THROW(XMLGen::append_optimization_algorithm_parameters_options(tXMLMetaData, tOptimizerNode));
    ASSERT_FALSE(tOptimizerNode.empty());

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Options"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOptionsNode = tOptimizerNode.child("Options");
    tGoldKeys = {"MaxNumOuterIterations", "MoveLimit"};
    tGoldValues = {"11", "0.2"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptionsNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOptionsOC)
{
    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("max_iterations", "11");
    tOptimizationParameters.append("optimization_algorithm", "Oc");
    tXMLMetaData.set(tOptimizationParameters);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_NO_THROW(XMLGen::append_optimization_algorithm_parameters_options(tXMLMetaData, tOptimizerNode));
    ASSERT_FALSE(tOptimizerNode.empty());

    // ****** TEST RESULTS AGAINST OPTIMIZER NODE GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Options", "Convergence"};
    std::vector<std::string> tGoldValues = {"",""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tCovergenceNode = tOptimizerNode.child("Convergence");
    tGoldKeys = {"MaxIterations"};
    tGoldValues = {"11"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tCovergenceNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOption_ErrorOptimizerNotSupported)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_algorithm", "stochastic gradient descent");
    tXMLMetaData.set(tOptimizationParameters);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_THROW(XMLGen::append_optimization_algorithm_options(tXMLMetaData, tOptimizerNode), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOption_DerivativeChecker)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("check_hessian", "false");
    tOptimizationParameters.append("check_gradient", "true");
    tOptimizationParameters.append("derivative_checker_initial_superscript", "1");
    tOptimizationParameters.append("derivative_checker_final_superscript", "8");
    tOptimizationParameters.append("optimization_algorithm", "derivativechecker");
    tXMLMetaData.set(tOptimizationParameters);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_NO_THROW(XMLGen::append_optimization_algorithm_options(tXMLMetaData, tOptimizerNode));

    // ****** TEST RESULTS AGAINST OPTIMIZER NODE GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Package", "CheckGradient", "CheckHessian", "UseUserInitialGuess", "Options"};
    std::vector<std::string> tGoldValues = {"DerivativeChecker", "true", "false", "True", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOptionsNode = tOptimizerNode.child("Options");
    tGoldKeys = {"DerivativeCheckerInitialSuperscript", "DerivativeCheckerFinalSuperscript"};
    tGoldValues = {"1", "8"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptionsNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationAlgorithmOption)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("max_iterations", "12");
    tOptimizationParameters.append("optimization_algorithm", "mma");
    tXMLMetaData.set(tOptimizationParameters);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_NO_THROW(XMLGen::append_optimization_algorithm_options(tXMLMetaData, tOptimizerNode));

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Package", "Options"};
    std::vector<std::string> tGoldValues = {"MMA", ""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOptionsNode = tOptimizerNode.child("Options");
    tGoldKeys = {"MaxNumOuterIterations"};
    tGoldValues = {"12"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptionsNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationOutputOptions)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_output_options(tXMLMetaData, tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Output"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOutputNode = tOptimizerNode.child("Output");
    tGoldKeys = {"OutputStage"};
    tGoldValues = {"Output To File"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutputNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationCacheStateOptions)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Service tService;
    tService.cacheState("true");
    tXMLMetaData.append(tService);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_cache_stage_options(tXMLMetaData, tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"CacheStage"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOutputNode = tOptimizerNode.child("CacheStage");
    ASSERT_FALSE(tOutputNode.empty());
    tGoldKeys = {"Name"};
    tGoldValues = {"Cache State"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutputNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationUpdateProblemOptions)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Service tService;
    tService.updateProblem("true");
    tXMLMetaData.append(tService);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_update_problem_stage_options(tXMLMetaData, tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"UpdateProblemStage"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tOutputNode = tOptimizerNode.child("UpdateProblemStage");
    ASSERT_FALSE(tOutputNode.empty());
    tGoldKeys = {"Name"};
    tGoldValues = {"Update Problem"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOutputNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationVariablesOptions)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tXMLMetaData.set(tOptimizationParameters);
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_variables_options(tXMLMetaData, tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"OptimizationVariables"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tNode = tOptimizerNode.child("OptimizationVariables");
    tGoldKeys = {"ValueName", "InitializationStage", "FilteredName", "LowerBoundValueName", "LowerBoundVectorName",
        "UpperBoundValueName", "UpperBoundVectorName", "SetLowerBoundsStage", "SetUpperBoundsStage"};
    tGoldValues = {"Control", "Initial Guess", "Topology", "Lower Bound Value", "Lower Bound Vector",
        "Upper Bound Value", "Upper Bound Vector", "Set Lower Bounds", "Set Upper Bounds"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationObjectiveOptions)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;

    // Pose Criterion
    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tXMLMetaData.append(tCriterion);

    // Pose Service	
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tXMLMetaData.append(tService);

    // Pose Service	
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tXMLMetaData.append(tScenario);

    XMLGen::Objective tObjective;
    tObjective.criteriaIDs.push_back("1");
    tObjective.serviceIDs.push_back("1");
    tObjective.scenarioIDs.push_back("1");

    tXMLMetaData.objective = tObjective;
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_objective_options(tXMLMetaData, tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"Objective"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tObjectiveNode = tOptimizerNode.child("Objective");
    tGoldKeys = {"ValueName", "ValueStageName", "GradientName", "GradientStageName"};
    tGoldValues = {"Objective Value", "Compute Objective Value", "Objective Gradient",
        "Compute Objective Gradient"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tObjectiveNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationConstraintOptions)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tXMLMetaData.append(tCriterion);

    XMLGen::Service tService;
    tService.id("1");
    tXMLMetaData.append(tService);

    XMLGen::Constraint tConstraint;
    tConstraint.id("1");
    tConstraint.absoluteTarget("");  // EMPTY VALUE - IT WILL BE IGNORE
    tConstraint.relativeTarget("");
    tConstraint.criterion("1");
    tConstraint.service("1");

    tXMLMetaData.constraints.push_back(tConstraint);

    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tXMLMetaData.set(tOptimizationParameters);

    auto tOptimizerNode = tDocument.append_child("Optimizer");
    ASSERT_THROW(XMLGen::append_optimization_constraint_options(tXMLMetaData, tOptimizerNode), std::runtime_error);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    // CASE 1: NORMALIZED TARGET VALUE IS DEFINED
    tOptimizerNode = tDocument.append_child("Optimizer");
    tXMLMetaData.constraints[0].relativeTarget("1.0");
    XMLGen::append_optimization_constraint_options(tXMLMetaData, tOptimizerNode);

    std::vector<std::string> tGoldKeys = {"Constraint"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tConstraintNode = tOptimizerNode.child("Constraint");
    tGoldKeys = {"ValueName", "ValueStageName", "GradientName", "GradientStageName",
        "ReferenceValueName", "NormalizedTargetValue"};
    tGoldValues = {"Constraint Value 1", "Compute Constraint Value 1", "Constraint Gradient 1",
        "Compute Constraint Gradient 1", "Design Volume", "1.0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tConstraintNode);
}

TEST(PlatoTestXMLGenerator, AppendOptimizationBoundConstraintsOptions)
{
    pugi::xml_document tDocument;
    auto tOptimizerNode = tDocument.append_child("Optimizer");
    XMLGen::append_optimization_bound_constraints_options({"1.0", "0.0"}, tOptimizerNode);

    // ****** TEST RESULTS AGAINST GOLD VALUES ******
    std::vector<std::string> tGoldKeys = {"BoundConstraint"};
    std::vector<std::string> tGoldValues = {""};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tOptimizerNode);
    auto tBoundConstraintNode = tOptimizerNode.child("BoundConstraint");
    tGoldKeys = {"Upper", "Lower"};
    tGoldValues = {"1.0", "0.0"};
    PlatoTestXMLGenerator::test_children(tGoldKeys, tGoldValues, tBoundConstraintNode);
}

TEST(PlatoTestXMLGenerator, WriteInterfaceXmlFile_ErrorEmptyObjective)
{
    XMLGen::InputData tXMLMetaData;
    ASSERT_THROW(XMLGen::write_interface_xml_file(tXMLMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, WriteInterfaceXmlFile_ErrorMultipleObjectives)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tXMLMetaData.append(tScenario);
    tScenario.id("2");
    tXMLMetaData.append(tScenario);
    XMLGen::Objective tObjective1;
    tObjective1.scenarioIDs.push_back("1");
    tObjective1.scenarioIDs.push_back("2");
    tXMLMetaData.objective = tObjective1;
    ASSERT_THROW(XMLGen::write_interface_xml_file(tXMLMetaData), std::runtime_error);
}

}
// namespace XMLGeneratorInterfaceFileUnitTester
