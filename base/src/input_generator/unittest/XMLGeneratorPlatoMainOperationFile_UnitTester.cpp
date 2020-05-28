/*
 * XMLGeneratorPlatoMainOperationFile_UnitTester.cpp
 *
 *  Created on: May 28, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"

namespace XMLGen
{

void write_plato_main_operations_xml_file_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData)
{
    pugi::xml_document tDocument;

    XMLGen::append_attributes("include", {"filename"}, {"defines.xml"}, tDocument);
    XMLGen::append_filter_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_output_to_plato_main_operation(aXMLMetaData, tDocument);

    tDocument.save_file("plato_main_operations.xml", "  ");
}

}

namespace PlatoTestXMLGenerator
{

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
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Control"}, tInput);
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
    tXMLMetaData.output_method = "epu";
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
    XMLGen::Objective tObjective;
    tXMLMetaData.objectives.push_back(tObjective);
    XMLGen::Constraint tConstraint;
    tXMLMetaData.constraints.push_back(tConstraint);
    auto tOperation = tDocument.append_child("Operation");

    XMLGen::append_default_qoi_to_output_operation(tXMLMetaData, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Control"}, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Objective Gradient 0"}, tInput);
    tInput = tInput.next_sibling("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Constraint Gradient 0"}, tInput);
}

TEST(PlatoTestXMLGenerator, AppendChildrenToOutputOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    auto tOperation = tDocument.append_child("Operation");

    // CASE 1: DEFAULT
    XMLGen::append_children_to_output_operation(tXMLMetaData, tOperation);
    ASSERT_FALSE(tOperation.empty());
    PlatoTestXMLGenerator::test_children({"Function", "Name"}, {"PlatoMainOutput", "Plato Main Output"}, tOperation);

    // CASE 1: ALL THE PARAMETERS ARE DEFINED
    tXMLMetaData.write_restart_file = "true";
    tXMLMetaData.output_frequency = "10";
    tXMLMetaData.max_iterations = "100";
    tOperation = tDocument.append_child("Operation");
    XMLGen::append_children_to_output_operation(tXMLMetaData, tOperation);
    ASSERT_FALSE(tOperation.empty());
    PlatoTestXMLGenerator::test_children({"Function", "Name", "WriteRestart", "OutputFrequency", "MaxIterations"},
                                         {"PlatoMainOutput", "Plato Main Output", "true", "10", "100"}, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendNonDeterministicQoiInputsToOutputOperation_ErrorInvalidLyout)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mOutputMetaData.mRandomQuantitiesOfInterest.push_back(std::make_pair("Von Mises", "quadrature point field"));
    auto tOperation = tDocument.append_child("Operation");
    ASSERT_THROW(XMLGen::append_nondeterministic_qoi_inputs_to_output_operation(tXMLMetaData, tOperation), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendNonDeterministicQoiInputsToOutputOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mOutputMetaData.mRandomQuantitiesOfInterest.push_back(std::make_pair("Von Mises", "element field"));
    tXMLMetaData.mOutputMetaData.mRandomQuantitiesOfInterest.push_back(std::make_pair("Cauchy Stress", "element field"));
    tXMLMetaData.mOutputMetaData.mRandomQuantitiesOfInterest.push_back(std::make_pair("Displacement X", "nodal field"));
    auto tOperation = tDocument.append_child("Operation");
    ASSERT_NO_THROW(XMLGen::append_nondeterministic_qoi_inputs_to_output_operation(tXMLMetaData, tOperation));
    ASSERT_FALSE(tOperation.empty());

    auto tFor = tOperation.child("For");
    ASSERT_FALSE(tFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"SampleIndex", "Samples"}, tFor);
    auto tInput = tFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"Von Mises {SampleIndex}", "Element Field"}, tInput);

    tFor = tFor.next_sibling("For");
    ASSERT_FALSE(tFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"SampleIndex", "Samples"}, tFor);
    tInput = tFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"Cauchy Stress {SampleIndex}", "Element Field"}, tInput);

    tFor = tFor.next_sibling("For");
    ASSERT_FALSE(tFor.empty());
    PlatoTestXMLGenerator::test_attributes({"var", "in"}, {"SampleIndex", "Samples"}, tFor);
    tInput = tFor.child("Input");
    ASSERT_FALSE(tInput.empty());
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"Displacement X {SampleIndex}", "Nodal Field"}, tInput);
}

TEST(PlatoTestXMLGenerator, AppendDeterministicQoiInputsToOutputOperation_ErrorInvalidLyout)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mOutputMetaData.mDeterministicQuantitiesOfInterest.push_back(std::make_pair("Von Mises", "quadrature point field"));
    auto tOperation = tDocument.append_child("Operation");
    ASSERT_THROW(XMLGen::append_deterministic_qoi_inputs_to_output_operation(tXMLMetaData, tOperation), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendDeterministicQoiInputsToOutputOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mOutputMetaData.mDeterministicQuantitiesOfInterest.push_back(std::make_pair("Von Mises", "element field"));
    tXMLMetaData.mOutputMetaData.mDeterministicQuantitiesOfInterest.push_back(std::make_pair("Cauchy Stress", "element field"));
    tXMLMetaData.mOutputMetaData.mDeterministicQuantitiesOfInterest.push_back(std::make_pair("Displacement X", "nodal field"));
    auto tOperation = tDocument.append_child("Operation");
    ASSERT_NO_THROW(XMLGen::append_deterministic_qoi_inputs_to_output_operation(tXMLMetaData, tOperation));
    ASSERT_FALSE(tOperation.empty());

    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"Von Mises", "Element Field"}, tInput);
    tInput = tInput.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"Cauchy Stress", "Element Field"}, tInput);
    tInput = tInput.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "Layout"}, {"Displacement X", "Nodal Field"}, tInput);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveGradientInputToOutputOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Objective tObjective1;
    XMLGen::Objective tObjective2;
    tXMLMetaData.objectives.push_back(tObjective1);
    tXMLMetaData.objectives.push_back(tObjective2);
    auto tOperation = tDocument.append_child("Operation");
    XMLGen::append_objective_gradient_input_to_output_operation(tXMLMetaData, tOperation);
    ASSERT_FALSE(tOperation.empty());

    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Objective Gradient 0"}, tInput);
    tInput = tInput.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Objective Gradient 1"}, tInput);
}

TEST(PlatoTestXMLGenerator, AppendConstraintGradientInputToOutputOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Constraint tConstraint1;
    XMLGen::Constraint tConstraint2;
    tXMLMetaData.constraints.push_back(tConstraint1);
    tXMLMetaData.constraints.push_back(tConstraint2);
    auto tOperation = tDocument.append_child("Operation");
    XMLGen::append_constraint_gradient_input_to_output_operation(tXMLMetaData, tOperation);

    ASSERT_FALSE(tOperation.empty());
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Constraint Gradient 0"}, tInput);
    tInput = tInput.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Constraint Gradient 1"}, tInput);

}

TEST(PlatoTestXMLGenerator, AppendFilterToPlatoMainOperation)
{
    pugi::xml_document tDocument1;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.filter_type = "kernel then tanh";
    XMLGen::append_filter_to_plato_main_operation(tXMLMetaData, tDocument1);
    ASSERT_FALSE(tDocument1.empty());

    // CASE 1: USER DEFINED FILTER
    auto tFilterNode = tDocument1.child("Filter");
    ASSERT_STREQ("Filter", tFilterNode.name());
    PlatoTestXMLGenerator::test_children({"Name", "Scale"}, {"KernelThenTANH", "2.0"}, tFilterNode);

    // CASE 2: DEFAULT FILTER
    pugi::xml_document tDocument2;
    tXMLMetaData.filter_type = "pde filter";
    XMLGen::append_filter_to_plato_main_operation(tXMLMetaData, tDocument2);
    ASSERT_FALSE(tDocument2.empty());
    tFilterNode = tDocument2.child("Filter");
    ASSERT_STREQ("Filter", tFilterNode.name());
    PlatoTestXMLGenerator::test_children({"Name", "Scale"}, {"Kernel", "2.0"}, tFilterNode);
}

TEST(PlatoTestXMLGenerator, AppendFilterOptionsToOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.filter_radius_scale = "2.5";
    auto tFilterNode = tDocument.append_child("Filter");
    XMLGen::append_filter_options_to_operation(tXMLMetaData, tFilterNode);
    ASSERT_FALSE(tFilterNode.empty());
    ASSERT_STREQ("Filter", tFilterNode.name());

    // CASE 1: JUST ONE PARAMETER
    PlatoTestXMLGenerator::test_children({"Scale"}, {"2.5"}, tFilterNode);

    // CASE 2: ALL PARAMETERS
    tXMLMetaData.filter_radius_scale = "2.5";
    tXMLMetaData.filter_radius_absolute = "1";
    tXMLMetaData.filter_projection_start_iteration = "50";
    tXMLMetaData.filter_projection_update_interval = "2";
    tXMLMetaData.filter_use_additive_continuation = "true";
    tXMLMetaData.filter_power = "3";
    tXMLMetaData.filter_heaviside_min = "0.5";
    tXMLMetaData.filter_heaviside_update = "1";
    tXMLMetaData.filter_heaviside_max = "2";
    tFilterNode = tDocument.append_child("Filter");
    XMLGen::append_filter_options_to_operation(tXMLMetaData, tFilterNode);
    ASSERT_FALSE(tFilterNode.empty());
    ASSERT_STREQ("Filter", tFilterNode.name());
    std::vector<std::string> tKeys = {"Scale", "Absolute", "StartIteration", "UpdateInterval",
        "UseAdditiveContinuation", "Power", "HeavisideMin", "HeavisideUpdate", "HeavisideMax"};
    std::vector<std::string> tValues = {"2.5", "1", "50", "2", "true", "3", "0.5", "1", "2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tFilterNode);
}

TEST(PlatoTestXMLGenerator, WritePlatoMainOperationsXmlFile)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::write_plato_main_operations_xml_file_for_nondeterministic_usecase(tXMLMetaData);
}

}
// namespace PlatoTestXMLGenerator
