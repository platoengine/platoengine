/*
 * XMLGeneratorPlatoMainOperationFile_UnitTester.cpp
 *
 *  Created on: May 28, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"

namespace XMLGen
{

void append_update_problem_to_plato_main_operation
(pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children({"Function", "Name"}, {"Update Problem", "Update Problem"}, tOperation);
}

void append_filter_control_to_plato_main_operation
(pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children({"Function", "Name", "Gradient"}, {"Filter", "Filter Control", "False"}, tOperation);
    auto tInput = tOperation.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"Field"}, tInput);
    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children({"ArgumentName"}, {"Filtered Field"}, tOutput);
}

void append_filter_gradient_to_plato_main_operation
(pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children({"Function", "Name", "Gradient"}, {"Filter", "Filter Gradient", "True"}, tOperation);
    auto tInput = tOperation.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"Field"}, tInput);
    tInput = tOperation.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"Gradient"}, tInput);
    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children({"ArgumentName"}, {"Filtered Gradient"}, tOutput);
}

void append_initialize_density_field_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children({"Function", "Name", "Method"}, {"InitializeField", "Initialize Field", "Uniform"}, tOperation);

    auto tMethod = tOperation.append_child("Uniform");
    auto tValue = aXMLMetaData.initial_density_value.empty() ? "0.5" : aXMLMetaData.initial_density_value;
    XMLGen::append_children({"Value"}, {tValue}, tMethod);

    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children({"ArgumentName"}, {"Initialized Field"}, tOutput);
}

void append_initialize_field_from_file_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.run_mesh_name.empty())
    {
        THROWERR(std::string("Append Initialize Field From File Operation: ")
            + "Initial guess was supposed to be initialized by reading it from a user-specified file. "
            + "However, the 'filename' keyword is empty.")
    }

    if(aXMLMetaData.initial_guess_field_name.empty())
    {
        THROWERR(std::string("Append Initialize Field From File Operation: ")
            + "Initial guess was supposed to be initialized by reading it from a user-specified field. "
            + "However, the field's 'name' keyword is empty.")
    }

    auto tOperation = aDocument.append_child("Operation");
    std::vector<std::string> tKeys = {"Function", "Name", "Method"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "FromFieldOnInputMesh"};
    XMLGen::append_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.append_child("FromFieldOnInputMesh");
    tKeys = {"Name", "VariableName", "Iteration"};
    tValues = {aXMLMetaData.run_mesh_name, aXMLMetaData.initial_guess_field_name, aXMLMetaData.restart_iteration};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    XMLGen::append_children(tKeys, tValues, tMethod);
}

void append_levelset_material_box
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tAppendMaterialBox = !aXMLMetaData.levelset_material_box_min.empty() &&
        !aXMLMetaData.levelset_material_box_max.empty();
    if(tAppendMaterialBox)
    {
        auto tMaterialBox = aParentNode.append_child("MaterialBox");
        std::vector<std::string> tKeys = {"MinCoords", "MaxCoords"};
        std::vector<std::string> tValues = {aXMLMetaData.levelset_material_box_min,
            aXMLMetaData.levelset_material_box_max};
        XMLGen::append_children(tKeys, tValues, tMaterialBox);
    }
}

void append_initialize_levelset_primitives_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.run_mesh_name.empty())
    {
        THROWERR(std::string("Append Initialize Levelset Primitives Operation: ")
            + "Levelset field was supposed to be initialized by reading it from a user-specified file. "
            + "However, the 'background mesh' keyword is empty.")
    }

    auto tOperation = aDocument.append_child("Operation");
    std::vector<std::string> tKeys = {"Function", "Name", "Method"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "PrimitivesLevelSet"};
    XMLGen::append_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.append_child("PrimitivesLevelSet");
    XMLGen::append_children({"BackgroundMeshName"}, {aXMLMetaData.run_mesh_name}, tMethod);
    XMLGen::append_levelset_material_box(aXMLMetaData, tMethod);
}

void append_initialize_levelset_swiss_cheese_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    std::vector<std::string> tKeys = {"Function", "Name", "Method"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "SwissCheeseLevelSet"};
    XMLGen::append_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.append_child("SwissCheeseLevelSet");
    tKeys = {"BackgroundMeshName", "SphereRadius", "SpherePackingFactor"};
    tValues = {aXMLMetaData.run_mesh_name, aXMLMetaData.levelset_sphere_radius,
        aXMLMetaData.levelset_sphere_packing_factor};
    for(auto& tNodeSet : aXMLMetaData.levelset_nodesets)
    {
        tKeys.push_back("NodeSet"); tValues.push_back(tNodeSet);
    }
    auto tCreateLevelSetSpheres = aXMLMetaData.create_levelset_spheres.empty() ? "false" : "true";
    tKeys.push_back("CreateSpheres"); tValues.push_back(tCreateLevelSetSpheres);
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    XMLGen::append_children(tKeys, tValues, tMethod);
}

void append_initialize_levelset_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    XMLGen::ValidLevelSetInitKeys tValidKeys;
    auto tLowerKey = Plato::tolower(aXMLMetaData.levelset_initialization_method);
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerKey);
    if(tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Append Initialize Levelset Operation: ") + "Levelset initialization method '"
            + tLowerKey + "' is not supported.")
    }

    if(tItr->compare("primitives") == 0)
    {
        XMLGen::append_initialize_levelset_primitives_operation(aXMLMetaData, aDocument);
    }
    else if(tItr->compare("swiss cheese") == 0)
    {
        XMLGen::append_initialize_levelset_swiss_cheese_operation(aXMLMetaData, aDocument);
    }
}

void append_initialize_field_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    XMLGen::ValidDiscretizationKeys tValidKeys;
    auto tLowerKey = Plato::tolower(aXMLMetaData.discretization);
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerKey);
    if(tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Append Initialize Field to Plato Main Operation: ") + "Discretization method '"
            + tLowerKey + "' is not supported.")
    }

    if(tItr->compare("density") == 0)
    {
        XMLGen::append_initialize_density_field_operation(aXMLMetaData, aDocument);
    }
    else if(tItr->compare("levelset") == 0)
    {
        XMLGen::append_initialize_levelset_operation(aXMLMetaData, aDocument);
    }
}

void append_initialize_field_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.initial_guess_filename.empty())
    {
        XMLGen::append_initialize_field_operation(aXMLMetaData, aDocument);
    }
    else
    {
        XMLGen::append_initialize_field_from_file_operation(aXMLMetaData, aDocument);
    }
}

void write_plato_main_operations_xml_file_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData)
{
    pugi::xml_document tDocument;
    auto tInclude = tDocument.append_child("include");

    XMLGen::append_attributes({"filename"}, {"defines.xml"}, tInclude);
    XMLGen::append_filter_options_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_output_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_initialize_field_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_stochastic_objective_value_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_stochastic_objective_gradient_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_nondeterministic_qoi_statistics_to_plato_main_operation(aXMLMetaData, tDocument);

    XMLGen::append_update_problem_to_plato_main_operation(tDocument);
    XMLGen::append_filter_control_to_plato_main_operation(tDocument);
    XMLGen::append_filter_gradient_to_plato_main_operation(tDocument);

    tDocument.save_file("plato_main_operations.xml", "  ");
}

}

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, AppendInitializeLevelsetPrimitivesOperation_ErrorEmptyFileName)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    ASSERT_THROW(XMLGen::append_initialize_levelset_primitives_operation(tXMLMetaData, tDocument), std::runtime_error);
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
    tXMLMetaData.levelset_material_box_max = "10";
    tXMLMetaData.levelset_material_box_min = "-10";
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

TEST(PlatoTestXMLGenerator, AppendInitializeFieldFromFileOperation_ErrorEmptyFieldName)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.run_mesh_name = "dummy.exo";
    ASSERT_THROW(XMLGen::append_initialize_field_from_file_operation(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendInitializeFieldFromFileOperation1)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.run_mesh_name = "dummy.exo";
    tXMLMetaData.initial_guess_field_name = "Control";
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
    tXMLMetaData.run_mesh_name = "dummy.exo";
    tXMLMetaData.restart_iteration = "100";
    tXMLMetaData.initial_guess_field_name = "Control";
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
    tXMLMetaData.initial_density_value = "0.25";
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
    XMLGen::append_filter_gradient_to_plato_main_operation(tDocument);
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
    XMLGen::append_filter_control_to_plato_main_operation(tDocument);
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
    XMLGen::append_update_problem_to_plato_main_operation(tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    PlatoTestXMLGenerator::test_children({"Function", "Name"}, {"Update Problem", "Update Problem"}, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendNondeterministicQoiStatisticsToPlatoMainOperation_ErrorInvalidLayout)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mOutputMetaData.mRandomQuantitiesOfInterest.push_back(std::make_pair("Von Mises Stress", "quadrature point field"));
    ASSERT_THROW(XMLGen::append_nondeterministic_qoi_statistics_to_plato_main_operation(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendNondeterministicQoiStatisticsToPlatoMainOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mOutputMetaData.mRandomQuantitiesOfInterest.push_back(std::make_pair("Von Mises Stress", "element field"));
    ASSERT_NO_THROW(XMLGen::append_nondeterministic_qoi_statistics_to_plato_main_operation(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name" , "Layout", "For", "Output", "Output"};
    std::vector<std::string> tValues = {"MeanPlusStdDev", "Von Mises Stress Statistics",
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
    tValues = {"Von Mises Stress {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    PlatoTestXMLGenerator::test_children({"Statistic", "ArgumentName"}, {"mean", "Von Mises Stress Mean"}, tOutput);
    tOutput = tOutput.next_sibling("Output");
    ASSERT_FALSE(tOutput.empty());
    PlatoTestXMLGenerator::test_children({"Statistic", "ArgumentName"}, {"std_dev", "Von Mises Stress StdDev"}, tOutput);
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
    tXMLMetaData.objective_number_standard_deviations = "2";
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
    tXMLMetaData.objective_number_standard_deviations = "2";
    XMLGen::append_stochastic_objective_gradient_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Layout", "CriterionValue", "CriterionGradient"};
    std::vector<std::string> tValues = {"MeanPlusStdDevGradient", "Calculate Non-Deterministic Objective Gradient",
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
    tXMLMetaData.objective_number_standard_deviations = "2";
    XMLGen::append_stochastic_objective_value_to_plato_main_operation(tXMLMetaData, tDocument);
    ASSERT_FALSE(tDocument.empty());

    // TEST RESULTS AGAINST GOLD VALUES
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Layout", "For", "Output", "Output", "Output"};
    std::vector<std::string> tValues = {"MeanPlusStdDev", "Calculate Non-Deterministic Objective Value", "Scalar", "", "", "", ""};
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
    PlatoTestXMLGenerator::test_children({"Function", "Name"}, {"PlatoMainOutput", "PlatoMainOutput"}, tOperation);

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

TEST(PlatoTestXMLGenerator, AppendFilterOptionsToPlatoMainOperation)
{
    pugi::xml_document tDocument1;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.filter_type = "kernel then tanh";
    XMLGen::append_filter_options_to_plato_main_operation(tXMLMetaData, tDocument1);
    ASSERT_FALSE(tDocument1.empty());

    // CASE 1: USER DEFINED FILTER
    auto tFilterNode = tDocument1.child("Filter");
    ASSERT_STREQ("Filter", tFilterNode.name());
    PlatoTestXMLGenerator::test_children({"Name", "Scale"}, {"KernelThenTANH", "2.0"}, tFilterNode);

    // CASE 2: DEFAULT FILTER
    pugi::xml_document tDocument2;
    tXMLMetaData.filter_type = "pde filter";
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
    tXMLMetaData.discretization = "density";
    tXMLMetaData.objective_number_standard_deviations = "1";
    tXMLMetaData.mOutputMetaData.mRandomQuantitiesOfInterest.push_back(std::make_pair("Von Mises Stress", "element field"));
    XMLGen::write_plato_main_operations_xml_file_for_nondeterministic_usecase(tXMLMetaData);
}

}
// namespace PlatoTestXMLGenerator
