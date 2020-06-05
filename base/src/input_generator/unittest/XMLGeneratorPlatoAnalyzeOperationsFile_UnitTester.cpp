/*
 * XMLGeneratorPlatoAnalyzeOperationsFile_UnitTester.cpp
 *
 *  Created on: Jun 2, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorMaterialFunctionInterface.hpp"
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

TEST(PlatoTestXMLGenerator, AppendMaterialPropertiesToPlatoAnalyzeOperation)
{
    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic linear elastic");
    tMaterial1.property("youngs modulus", "1");
    tMaterial1.property("poissons ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic linear thermoelastic");
    tMaterial2.property("youngs modulus", "1");
    tMaterial2.property("poissons ratio", "0.3");
    tMaterial2.property("Thermal Expansion Coefficient", "1.0e-8");
    tMaterial2.property("Thermal Conductivity Coefficient", "9");
    tMaterial2.property("Reference Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic linear elastic");
    tMaterial3.property("youngs modulus", "1.1");
    tMaterial3.property("poissons ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic linear thermoelastic");
    tMaterial4.property("youngs modulus", "1");
    tMaterial4.property("poissons ratio", "0.3");
    tMaterial4.property("Thermal Expansion Coefficient", "1.0e-8");
    tMaterial4.property("Thermal Conductivity Coefficient", "9");
    tMaterial4.property("Reference Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial3});
    tMaterialSetTwo.insert({"2", tMaterial4});
    auto tRandomMaterialCaseTwo = std::make_pair(0.5, tMaterialSetTwo);

    // CONSTRUCT SAMPLES SET
    XMLGen::InputData tXMLMetaData;
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseOne));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseTwo));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.finalize());

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_material_properties_to_plato_analyze_operation(tXMLMetaData, tDocument);

    auto tParameter = tDocument.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    std::vector<std::string> tKeys = {"ArgumentName", "Target", "InitialValue"};
    std::vector<std::string> tValues = {"poissons ratio block-id-1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs modulus block-id-1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"reference temperature block-id-2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Reference Temperature", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal conductivity coefficient block-id-2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Conductivity Coefficient", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal expansion coefficient block-id-2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Expansion Coefficient", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons ratio block-id-2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs modulus block-id-2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);
}

TEST(PlatoTestXMLGenerator, MaterialFunctionInterface_ErrorInvalidCategory)
{
    pugi::xml_document tDocument;
    std::vector<std::pair<std::string,std::string>> tTags =
    { {"youngs modulus block-id-1", "youngs modulus"}, {"poissons ratio block-id-1", "poissons ratio"} };

    XMLGen::MaterialFunctionInterface tInterface;
    ASSERT_THROW(tInterface.call("viscoelastic", tTags, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, MaterialFunctionInterface_Elastic)
{
    pugi::xml_document tDocument;
    std::vector<std::pair<std::string,std::string>> tTags =
    { {"youngs modulus block-id-1", "youngs modulus"}, {"poissons ratio block-id-1", "poissons ratio"} };

    XMLGen::MaterialFunctionInterface tInterface;
    ASSERT_NO_THROW(tInterface.call("isotropic linear elastic", tTags, tDocument));

    auto tParameter = tDocument.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    std::vector<std::string> tKeys = {"ArgumentName", "Target", "InitialValue"};
    std::vector<std::string> tValues = {"youngs modulus block-id-1",
        "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons ratio block-id-1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);
}

TEST(PlatoTestXMLGenerator, MaterialFunctionInterface_ThermoElastic)
{
    pugi::xml_document tDocument;
    std::vector<std::pair<std::string,std::string>> tTags =
    { {"youngs modulus block-id-1", "youngs modulus"}, {"poissons ratio block-id-1", "poissons ratio"},
      {"thermal expansion coefficient block-id-1", "thermal expansion coefficient"},
      {"thermal conductivity coefficient block-id-1", "thermal conductivity coefficient"},
      {"reference temperature block-id-1", "reference temperature"} };

    XMLGen::MaterialFunctionInterface tInterface;
    ASSERT_NO_THROW(tInterface.call("isotropic linear thermoelastic", tTags, tDocument));

    auto tParameter = tDocument.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    std::vector<std::string> tKeys = {"ArgumentName", "Target", "InitialValue"};
    std::vector<std::string> tValues = {"youngs modulus block-id-1",
        "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons ratio block-id-1", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal expansion coefficient block-id-1", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Expansion Coefficient", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal conductivity coefficient block-id-1", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Conductivity Coefficient", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"reference temperature block-id-1", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Reference Temperature", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);
}

TEST(PlatoTestXMLGenerator, ReturnMaterialPropertyTagsForPlatoAnalyzeOperationXmlFile_ErrorNoSamples)
{
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_THROW(XMLGen::return_material_metadata_for_plato_analyze_operation_xml_file(tRandomMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ReturnMaterialPropertyTagsForPlatoAnalyzeOperationXmlFile)
{
    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic linear elastic");
    tMaterial1.property("youngs modulus", "1");
    tMaterial1.property("poissons ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic linear thermoelastic");
    tMaterial2.property("youngs modulus", "1");
    tMaterial2.property("poissons ratio", "0.3");
    tMaterial2.property("Thermal Expansion Coefficient", "1.0e-8");
    tMaterial2.property("Thermal Conductivity Coefficient", "9");
    tMaterial2.property("Reference Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic linear elastic");
    tMaterial3.property("youngs modulus", "1.1");
    tMaterial3.property("poissons ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic linear thermoelastic");
    tMaterial4.property("youngs modulus", "1");
    tMaterial4.property("poissons ratio", "0.3");
    tMaterial4.property("Thermal Expansion Coefficient", "1.0e-8");
    tMaterial4.property("Thermal Conductivity Coefficient", "9");
    tMaterial4.property("Reference Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial3});
    tMaterialSetTwo.insert({"2", tMaterial4});
    auto tRandomMaterialCaseTwo = std::make_pair(0.5, tMaterialSetTwo);

    // CONSTRUCT SAMPLES SET
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_NO_THROW(tRandomMetaData.append(tRandomMaterialCaseOne));
    ASSERT_NO_THROW(tRandomMetaData.append(tRandomMaterialCaseTwo));
    ASSERT_NO_THROW(tRandomMetaData.finalize());

    // CALL FUNCTION
    std::vector<std::string> tGoldBlockIDs = {"1", "2"};
    std::vector<std::string> tGoldCategories = {"isotropic linear elastic", "isotropic linear thermoelastic"};
    std::vector<std::string> tGoldMatPropTags = {"youngs modulus", "poissons ratio", "thermal expansion coefficient",
        "thermal conductivity coefficient", "reference temperature"};
    std::vector<std::string> tGoldArgumentTags = {"youngs modulus block-id-1", "poissons ratio block-id-1",
        "youngs modulus block-id-2", "poissons ratio block-id-2", "thermal expansion coefficient block-id-2",
        "thermal conductivity coefficient block-id-2", "reference temperature block-id-2"};
    auto tMaterials = XMLGen::return_material_metadata_for_plato_analyze_operation_xml_file(tRandomMetaData);
    for(auto& tMaterial : tMaterials)
    {
        auto tGoldBlockID = std::find(tGoldBlockIDs.begin(), tGoldBlockIDs.end(), tMaterial.first);
        ASSERT_TRUE(tGoldBlockID != tGoldBlockIDs.end());
        ASSERT_STREQ(tGoldBlockID->c_str(), tMaterial.first.c_str());

        auto tGoldCategory = std::find(tGoldCategories.begin(), tGoldCategories.end(), tMaterial.second.first);
        ASSERT_TRUE(tGoldCategory != tGoldCategories.end());
        ASSERT_STREQ(tGoldCategory->c_str(), tMaterial.second.first.c_str());

        for(auto& tTagsPair : tMaterial.second.second)
        {
            auto tGoldArgumentTag = std::find(tGoldArgumentTags.begin(), tGoldArgumentTags.end(), tTagsPair.first);
            ASSERT_TRUE(tGoldArgumentTag != tGoldArgumentTags.end());
            ASSERT_STREQ(tGoldArgumentTag->c_str(), tTagsPair.first.c_str());

            auto tGoldMatPropTag = std::find(tGoldMatPropTags.begin(), tGoldMatPropTags.end(), tTagsPair.second);
            ASSERT_TRUE(tGoldMatPropTag != tGoldMatPropTags.end());
            ASSERT_STREQ(tGoldMatPropTag->c_str(), tTagsPair.second.c_str());
        }
    }
}

TEST(PlatoTestXMLGenerator, AppendIsotropicLinearThermoElasticMaterialPropertiesToPlatoAnalyzeOperation_ErrorEmptyTags)
{
    pugi::xml_document tDocument;
    std::vector<std::pair<std::string,std::string>> tTags;
    ASSERT_THROW(XMLGen::append_isotropic_linear_thermoelastic_material_properties_to_plato_analyze_operation(tTags, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendIsotropicLinearThermoElasticMaterialPropertiesToPlatoAnalyzeOperation_ErrorInvalidTag)
{
    pugi::xml_document tDocument;
    std::vector<std::pair<std::string,std::string>> tTags =
    { {"youngs modulus block-id-1", "young modulu"}, {"poissons ratio block-id-1", "poissons ratio"},
      {"thermal expansion coefficient block-id-1", "ther expann cocient"},
      {"thermal conductivity coefficient block-id-1", "thermal conductivity coefficient"},
      {"reference temperature block-id-1", "reference temperature"} };
    ASSERT_THROW(XMLGen::append_isotropic_linear_thermoelastic_material_properties_to_plato_analyze_operation(tTags, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendIsotropicLinearThermoElasticMaterialPropertiesToPlatoAnalyzeOperation)
{
    pugi::xml_document tDocument;
    std::vector<std::pair<std::string,std::string>> tTags =
    { {"youngs modulus block-id-1", "youngs modulus"}, {"poissons ratio block-id-1", "poissons ratio"},
      {"thermal expansion coefficient block-id-1", "thermal expansion coefficient"},
      {"thermal conductivity coefficient block-id-1", "thermal conductivity coefficient"},
      {"reference temperature block-id-1", "reference temperature"} };
    XMLGen::append_isotropic_linear_thermoelastic_material_properties_to_plato_analyze_operation(tTags, tDocument);

    auto tParameter = tDocument.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    std::vector<std::string> tKeys = {"ArgumentName", "Target", "InitialValue"};
    std::vector<std::string> tValues = {"youngs modulus block-id-1",
        "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons ratio block-id-1", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal expansion coefficient block-id-1", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Expansion Coefficient", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal conductivity coefficient block-id-1", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Conductivity Coefficient", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"reference temperature block-id-1", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Reference Temperature", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);
}

TEST(PlatoTestXMLGenerator, AppendIsotropicLinearElasticMaterialPropertiesToPlatoAnalyzeOperation_ErrorEmptyTags)
{
    pugi::xml_document tDocument;
    std::vector<std::pair<std::string,std::string>> tTags;
    ASSERT_THROW(XMLGen::append_isotropic_linear_elastic_material_properties_to_plato_analyze_operation(tTags, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendIsotropicLinearElasticMaterialPropertiesToPlatoAnalyzeOperation_ErrorInvalidTag)
{
    pugi::xml_document tDocument;
    std::vector<std::pair<std::string,std::string>> tTags =
    { {"youngs modulus block-id-1", "young modulu"}, {"poissons ratio block-id-1", "poissons ratio"} };
    ASSERT_THROW(XMLGen::append_isotropic_linear_elastic_material_properties_to_plato_analyze_operation(tTags, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendIsotropicLinearElasticMaterialPropertiesToPlatoAnalyzeOperation)
{
    pugi::xml_document tDocument;
    std::vector<std::pair<std::string,std::string>> tTags =
    { {"youngs modulus block-id-1", "youngs modulus"}, {"poissons ratio block-id-1", "poissons ratio"} };
    XMLGen::append_isotropic_linear_elastic_material_properties_to_plato_analyze_operation(tTags, tDocument);

    auto tParameter = tDocument.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    std::vector<std::string> tKeys = {"ArgumentName", "Target", "InitialValue"};
    std::vector<std::string> tValues = {"youngs modulus block-id-1",
        "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons ratio block-id-1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);
}

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
