/*
 * XMLGeneratorPlatoMainOperationFile_UnitTester.cpp
 *
 *  Created on: May 28, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorRandomMetadata.hpp"
#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, AppendDeterministicQoIToOutputOperation_non_multi_load_case)
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
    XMLGen::append_deterministic_qoi_to_output_operation_for_non_multi_load_case(tMetaData, tDocument);
    //tDocument.save_file("xml.txt", " ");

    auto tInput = tDocument.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    std::vector<std::string> tKeys = {"ArgumentName", "Layout"};
    std::vector<std::string> tValues = {"dispx_plato_analyze_1", "Nodal Field"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
}

TEST(PlatoTestXMLGenerator, AppendDeterministicQoIToOutputOperation_multi_load_case)
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
    tMetaData.objective.scenarioIDs.push_back("5");
    tMetaData.objective.scenarioIDs.push_back("6");

    pugi::xml_document tDocument;
    XMLGen::append_deterministic_qoi_to_output_operation_for_multi_load_case(tMetaData, tDocument);
    //tDocument.save_file("xml.txt", " ");

    auto tInput = tDocument.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    std::vector<std::string> tKeys = {"ArgumentName", "Layout"};
    std::vector<std::string> tValues = {"dispx_plato_analyze_1_scenario_5", "Nodal Field"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tValues = {"dispx_plato_analyze_1_scenario_6", "Nodal Field"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
}

TEST(PlatoTestXMLGenerator, AppendQoiStatisticsToOutputOperation)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::Output tOutputMetaData;
    tOutputMetaData.appendRandomQoI("vonmises", "element field");
    tXMLMetaData.mOutputMetaData.push_back(tOutputMetaData);

    pugi::xml_document tDocument;
    XMLGen::append_qoi_statistics_to_output_operation(tXMLMetaData, tDocument);

    auto tInput = tDocument.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"vonmises mean", "Element Field"}, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"vonmises standard deviation", "Element Field"}, tInput);
}

TEST(PlatoTestXMLGenerator, AppendAggregateDataToPlatoMainOperation_non_multi_load_case)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.criteriaIDs.push_back("3");
    tObjective.criteriaIDs.push_back("4");
    tObjective.weights.push_back("1.0");
    tObjective.weights.push_back("2.0");
    tObjective.multi_load_case = "false";
    tMetaData.objective = tObjective;

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_aggregate_data_to_plato_main_operation(tMetaData, tDocument));
    //tDocument.save_file("xml.txt", " ");

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Name", "Function", "Aggregate", "Aggregate", "Weighting"};
    std::vector<std::string> tValues = {"Aggregate Data", "Aggregator", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tAggregate = tOperation.child("Aggregate");
    ASSERT_FALSE(tAggregate.empty());
    ASSERT_STREQ("Aggregate", tAggregate.name());
    tKeys = {"Layout","Input","Input","Output"};
    tValues = {"Value","","",""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tAggregate);
    auto tInput = tAggregate.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tKeys = {"ArgumentName"};
    tValues = {"Value 1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tKeys = {"ArgumentName"};
    tValues = {"Value 2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
    auto tOutput = tAggregate.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    tKeys = {"ArgumentName"};
    tValues = {"Value"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);
    tAggregate = tAggregate.next_sibling("Aggregate");
    ASSERT_FALSE(tAggregate.empty());
    ASSERT_STREQ("Aggregate", tAggregate.name());
    tKeys = {"Layout","Input","Input","Output"};
    tValues = {"Nodal Field","","",""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tAggregate);
    tInput = tAggregate.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tKeys = {"ArgumentName"};
    tValues = {"Field 1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tKeys = {"ArgumentName"};
    tValues = {"Field 2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
    tOutput = tAggregate.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    tKeys = {"ArgumentName"};
    tValues = {"Field"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);
    auto tWeighting = tOperation.child("Weighting");
    ASSERT_FALSE(tWeighting.empty());
    ASSERT_STREQ("Weighting", tWeighting.name());
    auto tWeight = tWeighting.child("Weight");
    ASSERT_FALSE(tWeight.empty());
    ASSERT_STREQ("Weight", tWeight.name());
    tKeys = {"Value"};
    tValues = {"1.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tWeight);
    tWeight = tWeight.next_sibling("Weight");
    ASSERT_FALSE(tWeight.empty());
    ASSERT_STREQ("Weight", tWeight.name());
    tKeys = {"Value"};
    tValues = {"2.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tWeight);
}

TEST(PlatoTestXMLGenerator, AppendAggregateDataToPlatoMainOperation_multi_load_case)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.criteriaIDs.push_back("3");
    tObjective.criteriaIDs.push_back("4");
    tObjective.weights.push_back("1.0");
    tObjective.weights.push_back("2.0");
    tObjective.multi_load_case = "true";
    tMetaData.objective = tObjective;

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_aggregate_data_to_plato_main_operation(tMetaData, tDocument));
    //tDocument.save_file("xml.txt", " ");

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Name", "Function", "Aggregate", "Aggregate", "Weighting"};
    std::vector<std::string> tValues = {"Aggregate Data", "Aggregator", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tAggregate = tOperation.child("Aggregate");
    ASSERT_FALSE(tAggregate.empty());
    ASSERT_STREQ("Aggregate", tAggregate.name());
    tKeys = {"Layout","Input","Output"};
    tValues = {"Value","",""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tAggregate);
    auto tInput = tAggregate.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tKeys = {"ArgumentName"};
    tValues = {"Value 1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
    auto tOutput = tAggregate.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    tKeys = {"ArgumentName"};
    tValues = {"Value"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);
    tAggregate = tAggregate.next_sibling("Aggregate");
    ASSERT_FALSE(tAggregate.empty());
    ASSERT_STREQ("Aggregate", tAggregate.name());
    tKeys = {"Layout","Input","Output"};
    tValues = {"Nodal Field","",""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tAggregate);
    tInput = tAggregate.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    tKeys = {"ArgumentName"};
    tValues = {"Field 1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);
    tOutput = tAggregate.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    tKeys = {"ArgumentName"};
    tValues = {"Field"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);
    auto tWeighting = tOperation.child("Weighting");
    ASSERT_FALSE(tWeighting.empty());
    ASSERT_STREQ("Weighting", tWeighting.name());
    auto tWeight = tWeighting.child("Weight");
    ASSERT_FALSE(tWeight.empty());
    ASSERT_STREQ("Weight", tWeight.name());
    tKeys = {"Value"};
    tValues = {"1.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tWeight);
}

TEST(PlatoTestXMLGenerator, AppendSetLowerBoundsToPlatoMainOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("discretization", "density");
    tOptimizationParameters.append("optimization_type", "topology");
    tOptimizationParameters.addFixedBlockID("1");
    tOptimizationParameters.addFixedBlockID("2");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_set_lower_bounds_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Discretization", "Input", "Output", "FixedBlocks", "FixedBlocks"};
    std::vector<std::string> tValues = {"SetLowerBounds", "Compute Lower Bounds", "density", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_STREQ("Input", tInput.name());
    tKeys = {"ArgumentName"}; tValues = {"Lower Bound Value"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_STREQ("Output", tOutput.name());
    tKeys = {"ArgumentName"}; tValues = {"Lower Bound Vector"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);

    auto tFixedBlocks = tOperation.child("FixedBlocks");
    ASSERT_FALSE(tFixedBlocks.empty());
    ASSERT_STREQ("FixedBlocks", tFixedBlocks.name());
    auto tIndexNode = tFixedBlocks.child("Index");
    ASSERT_FALSE(tIndexNode.empty());
    ASSERT_STREQ(tIndexNode.child_value(), "1");

    tIndexNode = tIndexNode.next_sibling("Index");
    ASSERT_FALSE(tIndexNode.empty());
    ASSERT_STREQ(tIndexNode.child_value(), "2");
}

TEST(PlatoTestXMLGenerator, AppendSetUpperBoundsToPlatoMainOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("discretization", "density");
    tOptimizationParameters.append("optimization_type", "topology");
    tOptimizationParameters.addFixedBlockID("1");
    tOptimizationParameters.addFixedBlockID("2");
    tOptimizationParameters.addFixedSidesetID("11");
    tOptimizationParameters.addFixedSidesetID("12");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_set_upper_bounds_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Discretization", "Input", "Output",
        "FixedBlocks", "FixedBlocks", "FixedSidesets", "FixedSidesets"};
    std::vector<std::string> tValues = {"SetUpperBounds", "Compute Upper Bounds", "density", "", "",
        "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_STREQ("Input", tInput.name());
    tKeys = {"ArgumentName"}; tValues = {"Upper Bound Value"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_STREQ("Output", tOutput.name());
    tKeys = {"ArgumentName"}; tValues = {"Upper Bound Vector"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);

    auto tFixedBlocks = tOperation.child("FixedBlocks");
    ASSERT_FALSE(tFixedBlocks.empty());
    ASSERT_STREQ("FixedBlocks", tFixedBlocks.name());
    auto tIndexNode = tFixedBlocks.child("Index");
    ASSERT_FALSE(tIndexNode.empty());
    ASSERT_STREQ(tIndexNode.child_value(), "1");

    tIndexNode = tIndexNode.next_sibling("Index");
    ASSERT_FALSE(tIndexNode.empty());
    ASSERT_STREQ(tIndexNode.child_value(), "2");

    auto tFixedSidesets = tOperation.child("FixedSidesets");
    ASSERT_FALSE(tFixedSidesets.empty());
    ASSERT_STREQ("FixedSidesets", tFixedSidesets.name());
    tIndexNode = tFixedSidesets.child("Index");
    ASSERT_FALSE(tIndexNode.empty());
    ASSERT_STREQ(tIndexNode.child_value(), "11");

    tIndexNode = tIndexNode.next_sibling("Index");
    ASSERT_FALSE(tIndexNode.empty());
    ASSERT_STREQ(tIndexNode.child_value(), "12");
}

TEST(PlatoTestXMLGenerator, AppendFixedBlocksIdentificationNumbersToOperation_NoFixedBlocks)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::append_fixed_blocks_identification_numbers_to_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    auto tFixedBlocks = tDocument.child("FixedBlocks");
    ASSERT_TRUE(tFixedBlocks.empty());
}

TEST(PlatoTestXMLGenerator, AppendFixedBlocksIdentificationNumbersToOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.addFixedBlockID("1");
    tOptimizationParameters.addFixedBlockID("2");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_fixed_blocks_identification_numbers_to_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    auto tFixedBlocks = tDocument.child("FixedBlocks");
    ASSERT_FALSE(tFixedBlocks.empty());
    ASSERT_STREQ("FixedBlocks", tFixedBlocks.name());
    auto tIndexNode = tFixedBlocks.child("Index");
    ASSERT_FALSE(tIndexNode.empty());
    ASSERT_STREQ(tIndexNode.child_value(), "1");

    tIndexNode = tIndexNode.next_sibling("Index");
    ASSERT_FALSE(tIndexNode.empty());
    ASSERT_STREQ(tIndexNode.child_value(), "2");
}

TEST(PlatoTestXMLGenerator, AppendFixedSidesetsIdentificationNumbersToOperation_NoFixedSidesets)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::append_fixed_sidesets_identification_numbers_to_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    auto tFixedBlocks = tDocument.child("FixedSidesets");
    ASSERT_TRUE(tFixedBlocks.empty());
}

TEST(PlatoTestXMLGenerator, AppendFixedSidesetsIdentificationNumbersToOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.addFixedSidesetID("1");
    tOptimizationParameters.addFixedSidesetID("2");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_fixed_sidesets_identification_numbers_to_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    auto tFixedSideSets = tDocument.child("FixedSidesets");
    ASSERT_FALSE(tFixedSideSets.empty());
    ASSERT_STREQ("FixedSidesets", tFixedSideSets.name());
    auto tIndexNode = tFixedSideSets.child("Index");
    ASSERT_FALSE(tIndexNode.empty());
    ASSERT_STREQ(tIndexNode.child_value(), "1");

    tIndexNode = tIndexNode.next_sibling("Index");
    ASSERT_FALSE(tIndexNode.empty());
    ASSERT_STREQ(tIndexNode.child_value(), "2");
}

TEST(PlatoTestXMLGenerator, AppendFixedNodesetsIdentificationNumbersToOperation_NoFixedNodesets)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::append_fixed_nodesets_identification_numbers_to_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    auto tFixedBlocks = tDocument.child("FixedNodesets");
    ASSERT_TRUE(tFixedBlocks.empty());
}

TEST(PlatoTestXMLGenerator, AppendFixedNodesetsIdentificationNumbersToOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.addFixedNodesetID("1");
    tOptimizationParameters.addFixedNodesetID("2");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_fixed_nodesets_identification_numbers_to_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    auto tFixedNodesets = tDocument.child("FixedNodesets");
    ASSERT_FALSE(tFixedNodesets.empty());
    ASSERT_STREQ("FixedNodesets", tFixedNodesets.name());
    auto tIndexNode = tFixedNodesets.child("Index");
    ASSERT_FALSE(tIndexNode.empty());
    ASSERT_STREQ(tIndexNode.child_value(), "1");

    tIndexNode = tIndexNode.next_sibling("Index");
    ASSERT_FALSE(tIndexNode.empty());
    ASSERT_STREQ(tIndexNode.child_value(), "2");
}

TEST(PlatoTestXMLGenerator, AppendComputeVolumeGradientToPlatoMainOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tXMLMetaData.append(tService);

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("VoLuMe");
    tXMLMetaData.append(tCriterion);

    XMLGen::Constraint tConstraint;
    tConstraint.criterion("1");
    tConstraint.service("1");
    tXMLMetaData.constraints.push_back(tConstraint);

    XMLGen::append_compute_volume_gradient_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "PenaltyModel", "Input", "Output", "Output", "SIMP"};
    std::vector<std::string> tValues = {"ComputeVolume", "Compute Constraint Gradient", "SIMP", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_STREQ("Input", tInput.name());
    tKeys = {"ArgumentName"}; tValues = {"Topology"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_STREQ("Output", tOutput.name());
    tKeys = {"ArgumentName"}; tValues = {"Volume"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);

    tOutput = tOutput.next_sibling("Output");
    ASSERT_STREQ("Output", tOutput.name());
    tKeys = {"ArgumentName"}; tValues = {"Volume Gradient"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);

    auto tSIMP = tOperation.child("SIMP");
    ASSERT_STREQ("SIMP", tSIMP.name());
    tKeys = {"PenaltyExponent", "MinimumValue"}; tValues = {"1.0", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSIMP);
}

TEST(PlatoTestXMLGenerator, AppendComputeVolumeToPlatoMainOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tXMLMetaData.append(tService);

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("VoLuMe");
    tXMLMetaData.append(tCriterion);

    XMLGen::Constraint tConstraint;
    tConstraint.criterion("1");
    tConstraint.service("1");
    tXMLMetaData.constraints.push_back(tConstraint);

    XMLGen::append_compute_volume_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "PenaltyModel", "Input", "Output", "Output", "SIMP"};
    std::vector<std::string> tValues = {"ComputeVolume", "Compute Constraint Value", "SIMP", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_STREQ("Input", tInput.name());
    tKeys = {"ArgumentName"}; tValues = {"Topology"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_STREQ("Output", tOutput.name());
    tKeys = {"ArgumentName"}; tValues = {"Volume"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);

    tOutput = tOutput.next_sibling("Output");
    ASSERT_STREQ("Output", tOutput.name());
    tKeys = {"ArgumentName"}; tValues = {"Volume Gradient"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);

    auto tSIMP = tOperation.child("SIMP");
    ASSERT_STREQ("SIMP", tSIMP.name());
    tKeys = {"PenaltyExponent", "MinimumValue"}; tValues = {"1.0", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSIMP);
}

TEST(PlatoTestXMLGenerator, IsVolumeConstraintDefined_No)
{
    XMLGen::InputData tXMLMetaData;
    ASSERT_FALSE(XMLGen::is_volume_constraint_defined(tXMLMetaData));
}

TEST(PlatoTestXMLGenerator, IsVolumeConstraintDefined_ConstraintDefiendButItIsNotVolume)
{
    XMLGen::InputData tXMLMetaData;

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("Mass");
    tXMLMetaData.append(tCriterion);

    XMLGen::Constraint tConstraint;
    tConstraint.criterion("1");
    tXMLMetaData.constraints.push_back(tConstraint);

    ASSERT_FALSE(XMLGen::is_volume_constraint_defined(tXMLMetaData));
}

TEST(PlatoTestXMLGenerator, IsVolumeConstraintDefined1_Yes)
{
    XMLGen::InputData tXMLMetaData;

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("VoLuMe");
    tXMLMetaData.append(tCriterion);

    XMLGen::Constraint tConstraint;
    tConstraint.criterion("1");
    tXMLMetaData.constraints.push_back(tConstraint);

    ASSERT_TRUE(XMLGen::is_volume_constraint_defined(tXMLMetaData));
}

TEST(PlatoTestXMLGenerator, IsVolumeConstraintDefined2_Yes)
{
    XMLGen::InputData tXMLMetaData;

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("stress");
    tXMLMetaData.append(tCriterion);
    tCriterion.id("2");
    tCriterion.type("VoLuMe");
    tXMLMetaData.append(tCriterion);

    XMLGen::Constraint tConstraint;
    tConstraint.criterion("1");
    tXMLMetaData.constraints.push_back(tConstraint);
    tConstraint.criterion("2");
    tXMLMetaData.constraints.push_back(tConstraint);

    ASSERT_TRUE(XMLGen::is_volume_constraint_defined(tXMLMetaData));
}

TEST(PlatoTestXMLGenerator, AppendDesignVolumeToPlatoMainOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tXMLMetaData.append(tService);

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("VoLuMe");
    tXMLMetaData.append(tCriterion);

    XMLGen::Constraint tConstraint;
    tConstraint.criterion("1");
    tConstraint.service("1");
    tXMLMetaData.constraints.push_back(tConstraint);

    XMLGen::append_design_volume_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Output"};
    std::vector<std::string> tValues = {"DesignVolume", "Design Volume", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tOutput = tOperation.child("Output");
    ASSERT_STREQ("Output", tOutput.name());
    tKeys = {"ArgumentName"}; tValues = {"Design Volume"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);
}

TEST(PlatoTestXMLGenerator, IsVolumeConstraintDefinedAndComputedByPlatomain_UnconstrainedProblem)
{
    XMLGen::InputData tXMLMetaData;
    ASSERT_FALSE(XMLGen::is_volume_constraint_defined_and_computed_by_platomain(tXMLMetaData));
}

TEST(PlatoTestXMLGenerator, IsVolumeConstraintDefinedAndComputedByPlatomain_NoVolumeConstraint)
{
    XMLGen::InputData tXMLMetaData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tXMLMetaData.append(tService);

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("stress");
    tXMLMetaData.append(tCriterion);

    XMLGen::Constraint tConstraint;
    tConstraint.criterion("1");
    tConstraint.service("1");
    tXMLMetaData.constraints.push_back(tConstraint);

    ASSERT_FALSE(XMLGen::is_volume_constraint_defined_and_computed_by_platomain(tXMLMetaData));
}

TEST(PlatoTestXMLGenerator, IsVolumeConstraintDefinedAndComputedByPlatomain_VolumeDefinedButPerformerNotPlatoMain)
{
    XMLGen::InputData tXMLMetaData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("analyze");
    tXMLMetaData.append(tService);

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("volume");
    tXMLMetaData.append(tCriterion);

    XMLGen::Constraint tConstraint;
    tConstraint.criterion("1");
    tConstraint.service("1");
    tXMLMetaData.constraints.push_back(tConstraint);

    ASSERT_FALSE(XMLGen::is_volume_constraint_defined_and_computed_by_platomain(tXMLMetaData));
}

TEST(PlatoTestXMLGenerator, IsVolumeConstraintDefinedAndComputedByPlatomain_Yes)
{
    XMLGen::InputData tXMLMetaData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("PlatoMain");
    tXMLMetaData.append(tService);

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("volumE");
    tXMLMetaData.append(tCriterion);

    XMLGen::Constraint tConstraint;
    tConstraint.criterion("1");
    tConstraint.service("1");
    tXMLMetaData.constraints.push_back(tConstraint);

    ASSERT_TRUE(XMLGen::is_volume_constraint_defined_and_computed_by_platomain(tXMLMetaData));
}

TEST(PlatoTestXMLGenerator, AppendInitializeFieldToPlatoMainOperation_ReadFileKey)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("initial_guess_file_name", "dummy.exo");
    tOptimizationParameters.append("initial_guess_field_name", "Control");
    tOptimizationParameters.append("optimization_type", "topology");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_initialize_field_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "FromFieldOnInputMesh"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "FromFieldOnInputMesh", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("FromFieldOnInputMesh");
    ASSERT_STREQ("FromFieldOnInputMesh", tMethod.name());
    tKeys = {"Name", "VariableName"}; tValues = {"dummy.exo", "Control"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMethod);
}

TEST(PlatoTestXMLGenerator, AppendInitializeFieldToPlatoMainOperation_DensityKey)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("discretization", "density");
    tOptimizationParameters.append("optimization_type", "topology");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_initialize_field_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "Uniform", "Output"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "Uniform", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("Uniform");
    ASSERT_FALSE(tMethod.empty());
    ASSERT_STREQ("Uniform", tMethod.name());
    PlatoTestXMLGenerator::test_children({"Value"}, {"0.5"}, tMethod);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Initialized Field"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendInitializeFieldOperation_InvalidKey)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("discretization", "radial basis");
    tXMLMetaData.set(tOptimizationParameters);
    ASSERT_THROW(XMLGen::append_initialize_field_operation(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendInitializeFieldOperation_LevelSetKey)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("discretization", "levelset");
    tOptimizationParameters.append("levelset_initialization_method", "primitives");
    tXMLMetaData.set(tOptimizationParameters);
    tXMLMetaData.mesh.run_name = "dummy.exo";
    ASSERT_NO_THROW(XMLGen::append_initialize_field_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "PrimitivesLevelSet"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "PrimitivesLevelSet", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("PrimitivesLevelSet");
    ASSERT_FALSE(tMethod.empty());
    ASSERT_STREQ("PrimitivesLevelSet", tMethod.name());
    tKeys = {"BackgroundMeshName"}; tValues = {"dummy.exo"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMethod);
}

TEST(PlatoTestXMLGenerator, AppendInitializeFieldOperation_DensityKey)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("discretization", "density");
    tXMLMetaData.set(tOptimizationParameters);
    ASSERT_NO_THROW(XMLGen::append_initialize_field_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "Uniform", "Output"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "Uniform", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("Uniform");
    ASSERT_FALSE(tMethod.empty());
    ASSERT_STREQ("Uniform", tMethod.name());
    PlatoTestXMLGenerator::test_children({"Value"}, {"0.5"}, tMethod);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Initialized Field"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendInitializeLevelsetOperation_InvalidKey)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("levelset_initialization_method", "magic");
    tXMLMetaData.set(tOptimizationParameters);
    ASSERT_THROW(XMLGen::append_initialize_levelset_operation(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendInitializeLevelsetOperation_PrimitivesKey)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("levelset_initialization_method", "primitives");
    tXMLMetaData.set(tOptimizationParameters);
    tXMLMetaData.mesh.run_name = "dummy.exo";
    ASSERT_NO_THROW(XMLGen::append_initialize_levelset_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "PrimitivesLevelSet"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "PrimitivesLevelSet", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("PrimitivesLevelSet");
    ASSERT_FALSE(tMethod.empty());
    ASSERT_STREQ("PrimitivesLevelSet", tMethod.name());
    tKeys = {"BackgroundMeshName"}; tValues = {"dummy.exo"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMethod);
}

TEST(PlatoTestXMLGenerator, AppendInitializeLevelsetOperation_SwissCheeseKey)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mesh.run_name = "dummy.exo";
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("levelset_initialization_method", "swiss_cheese");
    tXMLMetaData.set(tOptimizationParameters);
    ASSERT_NO_THROW(XMLGen::append_initialize_levelset_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "SwissCheeseLevelSet", "CreateSpheres"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "SwissCheeseLevelSet", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("SwissCheeseLevelSet");
    ASSERT_FALSE(tMethod.empty());
    ASSERT_STREQ("SwissCheeseLevelSet", tMethod.name());
    tKeys = {"BackgroundMeshName", "CreateSpheres"}; tValues = {"dummy.exo", "false"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMethod);
}

TEST(PlatoTestXMLGenerator, AppendInitializeLevelsetSwissCheeseOperation_ErrorEmptyFileName)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    ASSERT_THROW(XMLGen::append_initialize_levelset_swiss_cheese_operation(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendInitializeLevelsetSwissCheeseOperation_NoSphereRadiusAndPackagingAndNodeSetDefinition)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mesh.run_name = "dummy.exo";
    ASSERT_NO_THROW(XMLGen::append_initialize_levelset_swiss_cheese_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "SwissCheeseLevelSet", "CreateSpheres"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "SwissCheeseLevelSet", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("SwissCheeseLevelSet");
    ASSERT_FALSE(tMethod.empty());
    ASSERT_STREQ("SwissCheeseLevelSet", tMethod.name());
    tKeys = {"BackgroundMeshName", "CreateSpheres"}; tValues = {"dummy.exo", "false"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMethod);
}

TEST(PlatoTestXMLGenerator, AppendInitializeLevelsetSwissCheeseOperation_NoSpherePackagingAndNodeSetDefinition)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mesh.run_name = "dummy.exo";
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("levelset_sphere_radius", "1.5");
    tXMLMetaData.set(tOptimizationParameters);
    ASSERT_NO_THROW(XMLGen::append_initialize_levelset_swiss_cheese_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "SwissCheeseLevelSet", "CreateSpheres"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "SwissCheeseLevelSet", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("SwissCheeseLevelSet");
    ASSERT_FALSE(tMethod.empty());
    ASSERT_STREQ("SwissCheeseLevelSet", tMethod.name());
    tKeys = {"BackgroundMeshName", "SphereRadius", "CreateSpheres"};
    tValues = {"dummy.exo", "1.5", "true"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMethod);
}

TEST(PlatoTestXMLGenerator, AppendInitializeLevelsetSwissCheeseOperation_NoNodeSetDefinition)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mesh.run_name = "dummy.exo";
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("levelset_sphere_radius", "1.5");
    tOptimizationParameters.append("levelset_sphere_packing_factor", "0.5");
    tXMLMetaData.set(tOptimizationParameters);
    ASSERT_NO_THROW(XMLGen::append_initialize_levelset_swiss_cheese_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "SwissCheeseLevelSet", "CreateSpheres"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "SwissCheeseLevelSet", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("SwissCheeseLevelSet");
    ASSERT_FALSE(tMethod.empty());
    ASSERT_STREQ("SwissCheeseLevelSet", tMethod.name());
    tKeys = {"BackgroundMeshName", "SphereRadius", "SpherePackingFactor", "CreateSpheres"};
    tValues = {"dummy.exo", "1.5", "0.5", "true"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMethod);
}

TEST(PlatoTestXMLGenerator, AppendInitializeLevelsetSwissCheeseOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mesh.run_name = "dummy.exo";
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.addLevelsetNodeset("nodeset 1");
    tOptimizationParameters.addLevelsetNodeset("nodeset 2");
    tXMLMetaData.set(tOptimizationParameters);
    ASSERT_NO_THROW(XMLGen::append_initialize_levelset_swiss_cheese_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "SwissCheeseLevelSet", "CreateSpheres"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "SwissCheeseLevelSet", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("SwissCheeseLevelSet");
    ASSERT_FALSE(tMethod.empty());
    ASSERT_STREQ("SwissCheeseLevelSet", tMethod.name());
    tKeys = {"BackgroundMeshName", "NodeSet", "NodeSet", "CreateSpheres"};
    tValues = {"dummy.exo", "nodeset 1", "nodeset 2", "false"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMethod);
}

TEST(PlatoTestXMLGenerator, AppendInitializeLevelsetPrimitivesOperation_ErrorEmptyFileName)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    ASSERT_THROW(XMLGen::append_initialize_levelset_primitives_operation(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendInitializeLevelsetPrimitivesOperation_NoMaterialBoxOption)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mesh.run_name = "dummy.exo";
    ASSERT_NO_THROW(XMLGen::append_initialize_levelset_primitives_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "PrimitivesLevelSet"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "PrimitivesLevelSet", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("PrimitivesLevelSet");
    ASSERT_FALSE(tMethod.empty());
    ASSERT_STREQ("PrimitivesLevelSet", tMethod.name());
    tKeys = {"BackgroundMeshName"}; tValues = {"dummy.exo"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMethod);
}

TEST(PlatoTestXMLGenerator, AppendInitializeLevelsetPrimitivesOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mesh.run_name = "dummy.exo";
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("levelset_material_box_min", "-10");
    tOptimizationParameters.append("levelset_material_box_max", "10");
    tXMLMetaData.set(tOptimizationParameters);
    ASSERT_NO_THROW(XMLGen::append_initialize_levelset_primitives_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "PrimitivesLevelSet", "MaterialBox"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "PrimitivesLevelSet", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("PrimitivesLevelSet");
    ASSERT_FALSE(tMethod.empty());
    ASSERT_STREQ("PrimitivesLevelSet", tMethod.name());
    tKeys = {"BackgroundMeshName", "MaterialBox"}; tValues = {"dummy.exo", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMethod);

    auto tMaterialBox = tMethod.child("MaterialBox");
    ASSERT_FALSE(tMaterialBox.empty());
    ASSERT_STREQ("MaterialBox", tMaterialBox.name());
    PlatoTestXMLGenerator::test_children({"MinCoords", "MaxCoords"}, {"-10", "10"}, tMaterialBox);
}

TEST(PlatoTestXMLGenerator, AppendLevelsetMaterialBox_DoNotDefine)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::append_levelset_material_box(tXMLMetaData, tDocument);
    auto tMaterialBox = tDocument.child("MaterialBox");
    ASSERT_TRUE(tMaterialBox.empty());
}

TEST(PlatoTestXMLGenerator, AppendLevelsetMaterialBox)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("levelset_material_box_min", "-10");
    tOptimizationParameters.append("levelset_material_box_max", "10");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_levelset_material_box(tXMLMetaData, tDocument);

    auto tMaterialBox = tDocument.child("MaterialBox");
    ASSERT_FALSE(tMaterialBox.empty());
    PlatoTestXMLGenerator::test_children({"MinCoords", "MaxCoords"}, {"-10", "10"}, tMaterialBox);
}

TEST(PlatoTestXMLGenerator, AppendInitializeFieldFromFileOperation_ErrorEmptyFileName)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    ASSERT_THROW(XMLGen::append_initialize_field_from_file_operation(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendInitializeFieldFromFileOperation1)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("initial_guess_file_name", "dummy.exo");
    tOptimizationParameters.append("initial_guess_field_name", "Control");
    tXMLMetaData.set(tOptimizationParameters);
    ASSERT_NO_THROW(XMLGen::append_initialize_field_from_file_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "FromFieldOnInputMesh"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "FromFieldOnInputMesh", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("FromFieldOnInputMesh");
    tKeys = {"Name", "VariableName"}; tValues = {"dummy.exo", "Control"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMethod);
}

TEST(PlatoTestXMLGenerator, AppendInitializeFieldFromFileOperation2)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("restart_iteration", "100");
    tOptimizationParameters.append("initial_guess_file_name", "dummy.exo");
    tOptimizationParameters.append("initial_guess_field_name", "Control");
    tXMLMetaData.set(tOptimizationParameters);
    ASSERT_NO_THROW(XMLGen::append_initialize_field_from_file_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "FromFieldOnInputMesh"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "FromFieldOnInputMesh", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("FromFieldOnInputMesh");
    tKeys = {"Name", "VariableName", "Iteration"};
    tValues = {"dummy.exo", "Control", "100"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMethod);
}

TEST(PlatoTestXMLGenerator, AppendInitializeDensityFieldOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("initial_density_value", "0.25");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_initialize_density_field_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Method", "Uniform", "Output"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "Uniform", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.child("Uniform");
    ASSERT_FALSE(tMethod.empty());
    ASSERT_STREQ("Uniform", tMethod.name());
    PlatoTestXMLGenerator::test_children({"Value"}, {"0.25"}, tMethod);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Initialized Field"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendFilterGradientToPlatoMainOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_filter_gradient_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Gradient", "Input", "Input", "Output"};
    std::vector<std::string> tValues = {"Filter", "Filter Gradient", "True", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Field"}, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Gradient"}, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Filtered Gradient"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendFilterControlToPlatoMainOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("optimization_type", "topology");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_filter_control_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Gradient", "Input", "Output"};
    std::vector<std::string> tValues = {"Filter", "Filter Control", "False", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Field"}, tInput);
    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Filtered Field"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemToPlatoMainOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Service tService;
    tService.updateProblem("true");
    tXMLMetaData.append(tService);
    XMLGen::append_update_problem_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    PlatoTestXMLGenerator::test_children({"Function", "Name"}, {"Update Problem", "Update Problem"}, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendStochasticQoiToPlatoMainOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Output tOutputMetaData;
    tOutputMetaData.appendRandomQoI("VonMises", "element field");
    tXMLMetaData.mOutputMetaData.push_back(tOutputMetaData);

    ASSERT_NO_THROW(XMLGen::append_qoi_statistics_to_plato_main_operation(tXMLMetaData, tDocument));

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name" , "Layout", "For", "Output", "Output"};
    std::vector<std::string> tValues = {"MeanPlusStdDev", "compute vonmises statistics",
        "Element Field", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tOuterFor = tOperation.child("For");
    ASSERT_FALSE(tOuterFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
    auto tInnerFor = tOuterFor.child("For");
    ASSERT_FALSE(tInnerFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
    auto tInput = tInnerFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    tKeys = {"ArgumentName", "Probability"};
    tValues = {"vonmises {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    PlatoTestXMLGenerator::test_children({"Statistic", "ArgumentName"}, {"mean", "vonmises mean"}, tOutput);
    tOutput = tOutput.next_sibling("Output");
    ASSERT_FALSE(tOutput.empty());
    PlatoTestXMLGenerator::test_children({"Statistic", "ArgumentName"}, {"std_dev", "vonmises standard deviation"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendStochasticCriterionValueOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::append_stochastic_criterion_value_operation(tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tCriterionValue = tDocument.child("CriterionValue");
    ASSERT_FALSE(tCriterionValue.empty());
    ASSERT_STREQ("CriterionValue", tCriterionValue.name());
    std::vector<std::string> tKeys = {"Layout", "For", "Output", "Output"};
    std::vector<std::string> tValues = {"Global", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tCriterionValue);

    auto tOuterFor = tCriterionValue.child("For");
    ASSERT_FALSE(tOuterFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
    auto tInnerFor = tOuterFor.child("For");
    ASSERT_FALSE(tInnerFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
    auto tInnerForInput = tInnerFor.append_child("Input");
    ASSERT_FALSE(tInnerForInput.empty());
    tKeys = {"ArgumentName", "Probability"};
    tValues ={"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInnerForInput);

    auto tOuterOutput = tCriterionValue.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    PlatoTestXMLGenerator::test_children({"Statistic", "ArgumentName"}, {"mean", "Objective Mean"}, tOuterOutput);
    tOuterOutput = tOuterOutput.next_sibling("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    PlatoTestXMLGenerator::test_children({"Statistic", "ArgumentName"}, {"std_dev", "Objective StdDev"}, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendStochasticCriterionGradientOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("objective_number_standard_deviations", "2");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_stochastic_criterion_gradient_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tCriterionGradient = tDocument.child("CriterionGradient");
    ASSERT_FALSE(tCriterionGradient.empty());
    ASSERT_STREQ("CriterionGradient", tCriterionGradient.name());
    std::vector<std::string> tKeys = {"Layout", "For", "Output"};
    std::vector<std::string> tValues = {"Nodal Field", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tCriterionGradient);

    auto tOuterFor = tCriterionGradient.child("For");
    ASSERT_FALSE(tOuterFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
    auto tInnerFor = tOuterFor.child("For");
    ASSERT_FALSE(tInnerFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
    auto tInnerForInput = tInnerFor.append_child("Input");
    ASSERT_FALSE(tInnerForInput.empty());
    tKeys = {"ArgumentName", "Probability"};
    tValues ={"Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInnerForInput);

    auto tOuterOutput = tCriterionGradient.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    PlatoTestXMLGenerator::test_children({"Statistic", "ArgumentName"}, {"mean_plus_2_std_dev", "Objective Mean Plus 2 StdDev Gradient"}, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendStochasticObjectiveGradientToPlatoMainOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("objective_number_standard_deviations", "2");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_stochastic_objective_gradient_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Layout", "CriterionValue", "CriterionGradient"};
    std::vector<std::string> tValues = {"MeanPlusStdDevGradient", "Compute Non-Deterministic Objective Gradient",
        "Nodal Field", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    // TEST CRITERION VALUE RESULTS AGAINST GOLD VALUES
    auto tCriterionValue = tOperation.child("CriterionValue");
    ASSERT_FALSE(tCriterionValue.empty());
    ASSERT_STREQ("CriterionValue", tCriterionValue.name());
    tKeys = {"Layout", "For", "Output", "Output"};
    tValues = {"Global", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tCriterionValue);

    auto tOuterFor = tCriterionValue.child("For");
    ASSERT_FALSE(tOuterFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
    auto tInnerFor = tOuterFor.child("For");
    ASSERT_FALSE(tInnerFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
    auto tInnerForInput = tInnerFor.append_child("Input");
    ASSERT_FALSE(tInnerForInput.empty());
    tKeys = {"ArgumentName", "Probability"};
    tValues ={"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInnerForInput);

    auto tOuterOutput = tCriterionValue.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    PlatoTestXMLGenerator::test_children({"Statistic", "ArgumentName"}, {"mean", "Objective Mean"}, tOuterOutput);
    tOuterOutput = tOuterOutput.next_sibling("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    PlatoTestXMLGenerator::test_children({"Statistic", "ArgumentName"}, {"std_dev", "Objective StdDev"}, tOuterOutput);

    // TEST CRITERION GRADIENT RESULTS AGAINST GOLD VALUES
    auto tCriterionGradient = tOperation.child("CriterionGradient");
    ASSERT_FALSE(tCriterionGradient.empty());
    ASSERT_STREQ("CriterionGradient", tCriterionGradient.name());
    tKeys = {"Layout", "For", "Output"};
    tValues = {"Nodal Field", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tCriterionGradient);

    tOuterFor = tCriterionGradient.child("For");
    ASSERT_FALSE(tOuterFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
    tInnerFor = tOuterFor.child("For");
    ASSERT_FALSE(tInnerFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
    tInnerForInput = tInnerFor.append_child("Input");
    ASSERT_FALSE(tInnerForInput.empty());
    tKeys = {"ArgumentName", "Probability"};
    tValues ={"Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInnerForInput);

    tOuterOutput = tCriterionGradient.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    PlatoTestXMLGenerator::test_children({"Statistic", "ArgumentName"}, {"mean_plus_2_std_dev", "Objective Mean Plus 2 StdDev Gradient"}, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendStochasticObjectiveValueToPlatoMainOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("objective_number_standard_deviations", "2");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_stochastic_objective_value_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Layout", "For", "Output", "Output", "Output"};
    std::vector<std::string> tValues = {"MeanPlusStdDev", "Compute Non-Deterministic Objective Value", "Scalar", "", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tOuterFor = tOperation.child("For");
    ASSERT_FALSE(tOuterFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
    auto tInnerFor = tOuterFor.child("For");
    ASSERT_FALSE(tInnerFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
    auto tInnerForInput = tInnerFor.append_child("Input");
    ASSERT_FALSE(tInnerForInput.empty());
    tKeys = {"ArgumentName", "Probability"};
    tValues ={"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInnerForInput);

    auto tOuterOutput = tOperation.child("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    PlatoTestXMLGenerator::test_children({"Statistic", "ArgumentName"}, {"mean", "Objective Mean"}, tOuterOutput);
    tOuterOutput = tOuterOutput.next_sibling("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    PlatoTestXMLGenerator::test_children({"Statistic", "ArgumentName"}, {"std_dev", "Objective StdDev"}, tOuterOutput);
    tOuterOutput = tOuterOutput.next_sibling("Output");
    ASSERT_FALSE(tOuterOutput.empty());
    PlatoTestXMLGenerator::test_children({"Statistic", "ArgumentName"}, {"mean_plus_2_std_dev", "Objective Mean Plus 2 StdDev"}, tOuterOutput);
}

TEST(PlatoTestXMLGenerator, AppendOutputToPlatoMainOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    ASSERT_NO_THROW(XMLGen::append_output_to_plato_main_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    PlatoTestXMLGenerator::test_children({"Function", "Name", "Input", "Input"}, {"PlatoMainOutput", "Plato Main Output", "", ""}, tOperation);
    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"topology", "Nodal Field"}, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"control", "Nodal Field"}, tInput);
}

TEST(PlatoTestXMLGenerator, AppendSurfaceExtractionToOutputOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    auto tOperation = tDocument.append_child("Operation");

    // CASE 1: EMPTY CHILD - NOT DEFINED
    XMLGen::append_surface_extraction_to_output_operation(tXMLMetaData, tOperation);
    ASSERT_TRUE(tOperation.child("SurfaceExtraction").empty());

    // CASE 2: CHILD IS DEFINED
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("output_method", "epu");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_surface_extraction_to_output_operation(tXMLMetaData, tOperation);

    auto tSurfaceExtraction = tOperation.child("SurfaceExtraction");
    ASSERT_FALSE(tSurfaceExtraction.empty());
    PlatoTestXMLGenerator::test_children({"OutputMethod", "Discretization", "Output"}, {"epu", "density", ""}, tSurfaceExtraction);

    auto tOutput = tSurfaceExtraction.child("Output");
    ASSERT_FALSE(tOutput.empty());
    PlatoTestXMLGenerator::test_children( {"Format"}, {"Exodus"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendDefaultQoiToOutputOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("platomain");
    tXMLMetaData.append(tService);
    tService.id("2");
    tService.code("plato_analyze");
    tXMLMetaData.append(tService);

    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tXMLMetaData.append(tScenario);

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("volume");
    tXMLMetaData.append(tCriterion);
    tCriterion.id("2");
    tCriterion.type("mechanical_compliance");
    tXMLMetaData.append(tCriterion);

    XMLGen::Constraint tConstraint;
    tConstraint.id("1");
    tConstraint.criterion("1");
    tConstraint.service("1");
    tConstraint.scenario("1");
    tXMLMetaData.constraints.push_back(tConstraint);

    XMLGen::Objective tObjective;
    tObjective.serviceIDs.push_back("2");
    tObjective.criteriaIDs.push_back("2");
    tObjective.scenarioIDs.push_back("1");
    tXMLMetaData.objective = tObjective;

    auto tOperation = tDocument.append_child("Operation");

    XMLGen::append_default_qoi_to_output_operation(tXMLMetaData, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"topology", "Nodal Field"}, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"control", "Nodal Field"}, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"objective gradient"}, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"constraint gradient 1"}, tInput);
}

TEST(PlatoTestXMLGenerator, AppendChildrenToOutputOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    auto tOperation = tDocument.append_child("Operation");

    // CASE 1: DEFAULT
    XMLGen::append_children_to_output_operation(tXMLMetaData, tOperation);
    ASSERT_FALSE(tOperation.empty());
    PlatoTestXMLGenerator::test_children({"Function", "Name"}, {"PlatoMainOutput", "PlatoMainOutput"}, tOperation);

    // CASE 1: ALL THE PARAMETERS ARE DEFINED
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("write_restart_file", "true");
    tOptimizationParameters.append("output_frequency", "10");
    tOptimizationParameters.append("max_iterations", "100");
    tXMLMetaData.set(tOptimizationParameters);
    tOperation = tDocument.append_child("Operation");
    XMLGen::append_children_to_output_operation(tXMLMetaData, tOperation);
    ASSERT_FALSE(tOperation.empty());
    PlatoTestXMLGenerator::test_children({"Function", "Name", "WriteRestart", "OutputFrequency", "MaxIterations"},
                                         {"PlatoMainOutput", "Plato Main Output", "true", "10", "100"}, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendNonDeterministicQoiInputsToOutputOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Output tOutputMetaData;
    tOutputMetaData.outputSamples("true");
    tOutputMetaData.appendRandomQoI("VonMises", "element field");
    tOutputMetaData.appendRandomQoI("Cauchy_Stress", "element field");
    tOutputMetaData.appendRandomQoI("dispX", "nodal field");
    tXMLMetaData.mOutputMetaData.push_back(tOutputMetaData);

    auto tOperation = tDocument.append_child("Operation");
    ASSERT_NO_THROW(XMLGen::append_stochastic_qoi_to_output_operation(tXMLMetaData, tOperation));
    ASSERT_FALSE(tOperation.empty());

    auto tFor = tOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"SampleIndex", "Samples"}, tFor);
    auto tInput = tFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"cauchy_stress {SampleIndex}", "Element Field"}, tInput);

    tFor = tFor.next_sibling("For");
    ASSERT_FALSE(tFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"SampleIndex", "Samples"}, tFor);
    tInput = tFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"dispx {SampleIndex}", "Nodal Field"}, tInput);

    tFor = tFor.next_sibling("For");
    ASSERT_FALSE(tFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"SampleIndex", "Samples"}, tFor);
    tInput = tFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"vonmises {SampleIndex}", "Element Field"}, tInput);
}

TEST(PlatoTestXMLGenerator, AppendDeterministicQoiInputsToOutputOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tXMLMetaData.append(tService);
    XMLGen::Output tOutputMetaData;
    tOutputMetaData.serviceID("1");
    tOutputMetaData.appendDeterminsiticQoI("VonMises", "element field");
    tOutputMetaData.appendDeterminsiticQoI("Cauchy_Stress", "element field");
    tOutputMetaData.appendDeterminsiticQoI("dispx", "nodal field");
    tXMLMetaData.mOutputMetaData.push_back(tOutputMetaData);

    auto tOperation = tDocument.append_child("Operation");
    ASSERT_NO_THROW(XMLGen::append_deterministic_qoi_to_output_operation(tXMLMetaData, tOperation));
    ASSERT_FALSE(tOperation.empty());

    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"cauchy_stress_plato_analyze_1", "Element Field"}, tInput);
    tInput = tInput.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"dispx_plato_analyze_1", "Nodal Field"}, tInput);
    tInput = tInput.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"vonmises_plato_analyze_1", "Element Field"}, tInput);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveGradientInputToOutputOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;

    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tXMLMetaData.append(tService);

    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tXMLMetaData.append(tScenario);
    tScenario.id("2");
    tXMLMetaData.append(tScenario);

    XMLGen::Criterion tCriterion;
    tCriterion.id("1");
    tCriterion.type("compliance");
    tXMLMetaData.append(tCriterion);

    XMLGen::Objective tObjective;
    tObjective.serviceIDs.push_back("1");
    tObjective.serviceIDs.push_back("1");
    tObjective.criteriaIDs.push_back("1");
    tObjective.criteriaIDs.push_back("1");
    tObjective.scenarioIDs.push_back("1");
    tObjective.scenarioIDs.push_back("2");
    tXMLMetaData.objective = tObjective;

    auto tOperation = tDocument.append_child("Operation");
    XMLGen::append_objective_gradient_to_output_operation(tXMLMetaData, tOperation);
    ASSERT_FALSE(tOperation.empty());

    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"objective gradient"}, tInput);
}

TEST(PlatoTestXMLGenerator, AppendConstraintGradientInputToOutputOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Constraint tConstraint1;
    tConstraint1.id("1");
    tConstraint1.criterion("1");
    tConstraint1.service("1");
    tConstraint1.scenario("1");
    XMLGen::Constraint tConstraint2;
    tConstraint2.id("2");
    tConstraint2.criterion("2");
    tConstraint2.service("2");
    tConstraint2.scenario("2");
    tXMLMetaData.constraints.push_back(tConstraint1);
    tXMLMetaData.constraints.push_back(tConstraint2);
    auto tOperation = tDocument.append_child("Operation");
    XMLGen::append_constraint_gradient_to_output_operation(tXMLMetaData, tOperation);

    ASSERT_FALSE(tOperation.empty());
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"constraint gradient 1"}, tInput);
    tInput = tInput.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"constraint gradient 2"}, tInput);
}

TEST(PlatoTestXMLGenerator, AppendFilterOptionsToPlatoMainOperation)
{
    pugi::xml_document tDocument1;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("filter_type", "kernel_then_tanh");
    tOptimizationParameters.append("filter_in_engine", "true");
    tOptimizationParameters.append("optimization_type", "topology");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_filter_options_to_plato_main_operation(tXMLMetaData, tDocument1);
    ASSERT_FALSE(tDocument1.empty());

    // CASE 1: USER DEFINED FILTER
    auto tFilterNode = tDocument1.child("Filter");
    ASSERT_STREQ("Filter", tFilterNode.name());
    PlatoTestXMLGenerator::test_children({"Name", "Scale"}, {"KernelThenTANH", "2.0"}, tFilterNode);

    // CASE 2: DEFAULT FILTER
    pugi::xml_document tDocument2;
    tOptimizationParameters.append("filter_type", "pde_filter");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::append_filter_options_to_plato_main_operation(tXMLMetaData, tDocument2);
    ASSERT_FALSE(tDocument2.empty());
    tFilterNode = tDocument2.child("Filter");
    ASSERT_STREQ("Filter", tFilterNode.name());
    PlatoTestXMLGenerator::test_children({"Name", "Scale"}, {"Kernel", "2.0"}, tFilterNode);
}

TEST(PlatoTestXMLGenerator, AppendFilterOptionsToOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("filter_radius_scale", "2.5");
    tXMLMetaData.set(tOptimizationParameters);
    auto tFilterNode = tDocument.append_child("Filter");
    XMLGen::append_filter_options_to_operation(tXMLMetaData, tFilterNode);
    ASSERT_FALSE(tFilterNode.empty());
    ASSERT_STREQ("Filter", tFilterNode.name());

    // CASE 1: JUST ONE PARAMETER
    PlatoTestXMLGenerator::test_children({"Scale"}, {"2.5"}, tFilterNode);

    // CASE 2: ALL PARAMETERS
    tOptimizationParameters.append("filter_radius_scale", "2.5");
    tOptimizationParameters.append("filter_radius_absolute", "1");
    tOptimizationParameters.append("filter_projection_start_iteration", "50");
    tOptimizationParameters.append("filter_projection_update_interval", "2");
    tOptimizationParameters.append("filter_use_additive_continuation", "true");
    tOptimizationParameters.append("filter_power", "3");
    tOptimizationParameters.append("filter_heaviside_min", "0.5");
    tOptimizationParameters.append("filter_heaviside_update", "1");
    tOptimizationParameters.append("filter_heaviside_max", "2");
    tXMLMetaData.set(tOptimizationParameters);
    tFilterNode = tDocument.append_child("Filter");
    XMLGen::append_filter_options_to_operation(tXMLMetaData, tFilterNode);
    ASSERT_FALSE(tFilterNode.empty());
    ASSERT_STREQ("Filter", tFilterNode.name());
    std::vector<std::string> tKeys = {"Scale", "Absolute", "StartIteration", "UpdateInterval",
        "UseAdditiveContinuation", "Power", "HeavisideMin", "HeavisideUpdate", "HeavisideMax"};
    std::vector<std::string> tValues = {"2.5", "1", "50", "2", "true", "3", "0.5", "1", "2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tFilterNode);
}

TEST(PlatoTestXMLGenerator, WriteStochasticPlatoMainOperationsXmlFile)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::OptimizationParameters tOptimizationParameters;
    tOptimizationParameters.append("discretization", "density");
    tOptimizationParameters.append("filter_type", "Kernel");
    tOptimizationParameters.append("filter_radius_scale", "2.0");
    tOptimizationParameters.append("objective_number_standard_deviations", "1");
    tOptimizationParameters.append("filter_in_engine", "true");
    tOptimizationParameters.append("optimization_type", "topology");
    tXMLMetaData.set(tOptimizationParameters);
    XMLGen::Service tService;
    tService.updateProblem("true");
    tXMLMetaData.append(tService);
    XMLGen::Output tOutputMetaData;
    tOutputMetaData.outputSamples("true");
    tOutputMetaData.appendRandomQoI("VonMises", "element field");
    tXMLMetaData.mOutputMetaData.push_back(tOutputMetaData);

    // add random metadata
    XMLGen::RandomMetaData tRandomMetaData;
    XMLGen::RandomLoadCase tRandomLoadCase;
    XMLGen::LoadCase tLoadCase;
    std::vector<XMLGen::NaturalBoundaryCondition> tLoads;
    XMLGen::NaturalBoundaryCondition tLoad;
    tLoads.push_back(tLoad);
    tLoadCase.loads = tLoads;
    tLoadCase.id = 1;
    tRandomLoadCase.first = 1;
    tRandomLoadCase.second = tLoadCase;
    tRandomMetaData.append(tRandomLoadCase);
    tRandomMetaData.finalize();
    tXMLMetaData.mRandomMetaData = tRandomMetaData;
    EXPECT_EQ(tRandomMetaData.numSamples(), 1u);

    XMLGen::write_plato_main_operations_xml_file(tXMLMetaData);

    auto tReadData = XMLGen::read_data_from_file("plato_main_operations.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><includefilename=\"defines.xml\"/><Filter><Name>Kernel</Name><Scale>2.0</Scale></Filter><Operation><Function>PlatoMainOutput</Function><Name>PlatoMainOutput</Name><Input><ArgumentName>topology</ArgumentName>")
    +"<Layout>NodalField</Layout></Input><Input><ArgumentName>control</ArgumentName><Layout>NodalField</Layout></Input><Input><ArgumentName>objectivegradient</ArgumentName></Input><Input><ArgumentName>vonmisesmean</ArgumentName><Layout>ElementField</Layout></Input><Input><ArgumentName>vonmisesstandarddeviation</ArgumentName>"
    +"<Layout>ElementField</Layout></Input><Forvar=\"SampleIndex\"in=\"Samples\"><Input><ArgumentName>vonmises{SampleIndex}</ArgumentName><Layout>ElementField</Layout></Input></For></Operation><Operation><Function>InitializeField</Function><Name>InitializeField</Name>"
    +"<Method>Uniform</Method><Uniform><Value>0.5</Value></Uniform><Output><ArgumentName>InitializedField</ArgumentName></Output></Operation><Operation><Function>SetLowerBounds</Function><Name>ComputeLowerBounds</Name><Discretization>density</Discretization>"
    +"<Input><ArgumentName>LowerBoundValue</ArgumentName></Input><Output><ArgumentName>LowerBoundVector</ArgumentName></Output></Operation><Operation><Function>SetUpperBounds</Function><Name>ComputeUpperBounds</Name><Discretization>density</Discretization>"
    +"<Input><ArgumentName>UpperBoundValue</ArgumentName></Input><Output><ArgumentName>UpperBoundVector</ArgumentName></Output></Operation><Operation><Function>CopyField</Function><Name>CopyField</Name><Input><ArgumentName>InputField</ArgumentName></Input><Output><ArgumentName>OutputField</ArgumentName></Output></Operation><Operation><Function>CopyValue</Function><Name>CopyValue</Name><Input><ArgumentName>InputValue</ArgumentName></Input><Output><ArgumentName>OutputValue</ArgumentName></Output></Operation><Operation><Function>MeanPlusStdDev</Function><Name>ComputeNon-DeterministicObjectiveValue</Name><Layout>Scalar</Layout>"
    +"<Forvar=\"PerformerIndex\"in=\"Performers\"><Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\"><Input><ArgumentName>ObjectiveValue{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</ArgumentName>"
    +"<Probability>{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</Probability></Input></For></For><Output><Statistic>mean</Statistic><ArgumentName>ObjectiveMean</ArgumentName></Output><Output><Statistic>std_dev</Statistic>"
    +"<ArgumentName>ObjectiveStdDev</ArgumentName></Output><Output><Statistic>mean_plus_1_std_dev</Statistic><ArgumentName>ObjectiveMeanPlus1StdDev</ArgumentName></Output></Operation><Operation><Function>MeanPlusStdDevGradient</Function>"
    +"<Name>ComputeNon-DeterministicObjectiveGradient</Name><Layout>NodalField</Layout><CriterionValue><Layout>Global</Layout><Forvar=\"PerformerIndex\"in=\"Performers\"><Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\"><Input>"
    +"<ArgumentName>ObjectiveValue{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</ArgumentName><Probability>{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</Probability></Input></For></For><Output><Statistic>mean</Statistic>"
    +"<ArgumentName>ObjectiveMean</ArgumentName></Output><Output><Statistic>std_dev</Statistic><ArgumentName>ObjectiveStdDev</ArgumentName></Output></CriterionValue><CriterionGradient><Layout>NodalField</Layout><Forvar=\"PerformerIndex\"in=\"Performers\">"
    +"<Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\"><Input><ArgumentName>ObjectiveGradient{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</ArgumentName><Probability>{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</Probability>"
    +"</Input></For></For><Output><Statistic>mean_plus_1_std_dev</Statistic><ArgumentName>ObjectiveMeanPlus1StdDevGradient</ArgumentName></Output></CriterionGradient></Operation><Operation><Function>MeanPlusStdDev</Function><Name>computevonmisesstatistics</Name>"
    +"<Layout>ElementField</Layout><Forvar=\"PerformerIndex\"in=\"Performers\"><Forvar=\"PerformerSampleIndex\"in=\"PerformerSamples\"><Input><ArgumentName>vonmises{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}</ArgumentName>"
    +"<Probability>{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}</Probability></Input></For></For><Output><Statistic>mean</Statistic><ArgumentName>vonmisesmean</ArgumentName></Output><Output><Statistic>std_dev</Statistic>"
    +"<ArgumentName>vonmisesstandarddeviation</ArgumentName></Output></Operation><Operation><Function>UpdateProblem</Function><Name>UpdateProblem</Name></Operation><Operation><Function>Filter</Function><Name>FilterControl</Name><Gradient>False</Gradient>"
    +"<Input><ArgumentName>Field</ArgumentName></Input><Output><ArgumentName>FilteredField</ArgumentName></Output></Operation><Operation><Function>Filter</Function><Name>FilterGradient</Name><Gradient>True</Gradient><Input><ArgumentName>Field</ArgumentName></Input>"
    +"<Input><ArgumentName>Gradient</ArgumentName></Input><Output><ArgumentName>FilteredGradient</ArgumentName></Output></Operation>";
    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());
 //   Plato::system("rm -f plato_main_operations.xml");
}

}
// namespace PlatoTestXMLGenerator
