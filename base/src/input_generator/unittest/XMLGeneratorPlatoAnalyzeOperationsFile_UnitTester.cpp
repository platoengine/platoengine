/*
 * XMLGeneratorPlatoAnalyzeOperationsFile_UnitTester.cpp
 *
 *  Created on: Jun 2, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeUtilities.hpp"
#include "XMLGeneratorMaterialFunctionInterface.hpp"
#include "XMLGeneratorPlatoAnalyzeOperationsFileUtilities.hpp"

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, WritePlatoAnalyzeOperationsXmlFile)
{
    XMLGen::InputData tMetaData;
    tMetaData.max_iterations = "10";
    tMetaData.discretization = "density";
    tMetaData.optimization_algorithm = "oc";
    tMetaData.optimization_type = "topology";
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
    XMLGen::Service tService;
    tService.id("1");
    tService.code("plato_analyze");
    tService.performer("plato_analyze_1");
    tService.cacheState("false");
    tService.updateProblem("true");
    tMetaData.append(tService);
    tMetaData.mOutputMetaData.disableOutput();

    XMLGen::write_plato_analyze_operation_xml_file(tMetaData);

    auto tReadData = XMLGen::read_data_from_file("plato_analyze_operations.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><Operation><Function>UpdateProblem</Function><Name>UpdateProblem</Name></Operation>")
        +"<Operation><Function>ComputeCriterionValue</Function><Name>ComputeObjectiveValue</Name><Input><ArgumentName>Topology</ArgumentName></Input><Output><Argument>Value</Argument><ArgumentName>ObjectiveValue</ArgumentName></Output></Operation>"
        +"<Operation><Function>ComputeCriterionGradient</Function><Name>ComputeObjectiveGradient</Name><Input><ArgumentName>Topology</ArgumentName></Input><Output><Argument>Gradient</Argument><ArgumentName>ObjectiveGradient</ArgumentName></Output></Operation>"
        +"<Operation><Function>ComputeCriterionValue</Function><Name>ComputeConstraintValue</Name><Input><ArgumentName>Topology</ArgumentName></Input><Output><Argument>Value</Argument><ArgumentName>ConstraintValue</ArgumentName></Output></Operation>"
        +"<Operation><Function>ComputeCriterionGradient</Function><Name>ComputeConstraintGradient</Name><Input><ArgumentName>Topology</ArgumentName></Input><Output><Argument>Gradient</Argument><ArgumentName>ConstraintGradient</ArgumentName></Output></Operation>";
    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());
    Plato::system("rm -f plato_analyze_operations.xml");
}

TEST(PlatoTestXMLGenerator, WritePlatoAnalyzeOperationXmlFileForNondeterministicUsecase)
{
    // POSE INPUTS
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mProblemUpdateFrequency = "5";
    tXMLMetaData.optimization_type = "topology";
    XMLGen::Service tService;
    tService.id("1");
    tService.updateProblem("true");
    tService.code("plato_analyze");
    tService.performer("plato_analyze_1");
    tXMLMetaData.append(tService);
    XMLGen::Constraint tConstraint;
    tConstraint.code("plato_analyze");
    tXMLMetaData.constraints.push_back(tConstraint);
    XMLGen::Objective tObjective;
    tObjective.code_name = "plato_analyze";
    tXMLMetaData.objectives.push_back(tObjective);
    tXMLMetaData.mOutputMetaData.serviceID("1");
    tXMLMetaData.mOutputMetaData.appendDeterminsiticQoI("dispx", "nodal field");
    tXMLMetaData.mOutputMetaData.appendDeterminsiticQoI("dispy", "nodal field");
    tXMLMetaData.mOutputMetaData.appendDeterminsiticQoI("dispz", "nodal field");

    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic linear elastic");
    tMaterial1.property("youngs_modulus", "1");
    tMaterial1.property("poissons_ratio", "0.3");
    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    auto tRandomMaterialCase1 = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic linear elastic");
    tMaterial2.property("youngs_modulus", "1.1");
    tMaterial2.property("poissons_ratio", "0.33");
    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial2});
    auto tRandomMaterialCase2 = std::make_pair(0.5, tMaterialSetTwo);

    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.load_id = "1";
    tLoad1.mIsRandom = true;
    tLoad1.type = "traction";
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.load_id = "2";
    tLoad2.mIsRandom = true;
    tLoad2.type = "traction";
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoad2.values.push_back("6");
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::Load tLoad3;
    tLoad3.load_id = "3";
    tLoad3.type = "traction";
    tLoad3.mIsRandom = false;
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("7");
    tLoad3.values.push_back("8");
    tLoad3.values.push_back("9");
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad4;
    tLoad4.load_id = "1";
    tLoad4.mIsRandom = true;
    tLoad4.type = "traction";
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("11");
    tLoad4.values.push_back("12");
    tLoad4.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::Load tLoad5;
    tLoad5.load_id = "2";
    tLoad5.mIsRandom = true;
    tLoad5.type = "traction";
    tLoad5.app_name = "sideset";
    tLoad5.values.push_back("14");
    tLoad5.values.push_back("15");
    tLoad5.values.push_back("16");
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCase1));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCase2));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.finalize());

    // CALL FUNCTION
    XMLGen::write_plato_analyze_operation_xml_file(tXMLMetaData);
    auto tData = XMLGen::read_data_from_file("plato_analyze_operations.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><Operation><Function>WriteOutput</Function><Name>WriteOutput</Name><Output><ArgumentName>SolutionX</ArgumentName></Output>")
    +"<Output><ArgumentName>SolutionY</ArgumentName></Output><Output><ArgumentName>SolutionZ</ArgumentName></Output></Operation><Operation><Function>UpdateProblem</Function><Name>UpdateProblem</Name>"
    +"</Operation><Operation><Function>ComputeCriterionValue</Function><Name>ComputeObjectiveValue</Name><Input><ArgumentName>Topology</ArgumentName></Input><Output><Argument>Value</Argument><ArgumentName>ObjectiveValue</ArgumentName></Output>"
    +"<Parameter><ArgumentName>traction_load_id_2_x_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID2]:Values(0)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_2_y_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID2]:Values(1)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_2_z_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID2]:Values(2)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_1_x_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID1]:Values(0)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_1_y_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID1]:Values(1)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_1_z_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID1]:Values(2)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>poissons_ratio_block_id_1</ArgumentName><Target>[PlatoProblem]:[MaterialModel]:[IsotropicLinearElastic]:PoissonsRatio</Target><InitialValue>0.0</InitialValue></Parameter>"
    +"<Parameter><ArgumentName>youngs_modulus_block_id_1</ArgumentName><Target>[PlatoProblem]:[MaterialModel]:[IsotropicLinearElastic]:YoungsModulus</Target><InitialValue>0.0</InitialValue></Parameter></Operation>"
    +"<Operation><Function>ComputeCriterionGradient</Function><Name>ComputeObjectiveGradient</Name><Input><ArgumentName>Topology</ArgumentName></Input><Output><Argument>Gradient</Argument><ArgumentName>ObjectiveGradient</ArgumentName></Output>"
    +"<Parameter><ArgumentName>traction_load_id_2_x_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID2]:Values(0)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_2_y_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID2]:Values(1)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_2_z_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID2]:Values(2)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_1_x_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID1]:Values(0)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_1_y_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID1]:Values(1)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_1_z_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID1]:Values(2)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>poissons_ratio_block_id_1</ArgumentName><Target>[PlatoProblem]:[MaterialModel]:[IsotropicLinearElastic]:PoissonsRatio</Target><InitialValue>0.0</InitialValue></Parameter>"
    +"<Parameter><ArgumentName>youngs_modulus_block_id_1</ArgumentName><Target>[PlatoProblem]:[MaterialModel]:[IsotropicLinearElastic]:YoungsModulus</Target><InitialValue>0.0</InitialValue></Parameter></Operation>"
    +"<Operation><Function>ComputeCriterionValue</Function><Name>ComputeConstraintValue</Name><Input><ArgumentName>Topology</ArgumentName></Input><Output><Argument>Value</Argument><ArgumentName>ConstraintValue</ArgumentName></Output>"
    +"<Parameter><ArgumentName>traction_load_id_2_x_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID2]:Values(0)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_2_y_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID2]:Values(1)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter>"+"<Parameter><ArgumentName>traction_load_id_2_z_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID2]:Values(2)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_1_x_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID1]:Values(0)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_1_y_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID1]:Values(1)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_1_z_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID1]:Values(2)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>poissons_ratio_block_id_1</ArgumentName><Target>[PlatoProblem]:[MaterialModel]:[IsotropicLinearElastic]:PoissonsRatio</Target><InitialValue>0.0</InitialValue></Parameter>"
    +"<Parameter><ArgumentName>youngs_modulus_block_id_1</ArgumentName><Target>[PlatoProblem]:[MaterialModel]:[IsotropicLinearElastic]:YoungsModulus</Target><InitialValue>0.0</InitialValue></Parameter>"
    +"</Operation><Operation><Function>ComputeCriterionGradient</Function><Name>ComputeConstraintGradient</Name><Input><ArgumentName>Topology</ArgumentName></Input><Output><Argument>Gradient</Argument><ArgumentName>ConstraintGradient</ArgumentName>"
    +"</Output><Parameter><ArgumentName>traction_load_id_2_x_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID2]:Values(0)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_2_y_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID2]:Values(1)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_2_z_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID2]:Values(2)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_1_x_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID1]:Values(0)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_1_y_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID1]:Values(1)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>traction_load_id_1_z_axis</ArgumentName><Target>[PlatoProblem]:[NaturalBoundaryConditions]:[RandomTractionVectorBoundaryConditionwithID1]:Values(2)</Target><InitialValue>0.0</InitialValue>"
    +"</Parameter><Parameter><ArgumentName>poissons_ratio_block_id_1</ArgumentName><Target>[PlatoProblem]:[MaterialModel]:[IsotropicLinearElastic]:PoissonsRatio</Target><InitialValue>0.0</InitialValue></Parameter>"
    +"<Parameter><ArgumentName>youngs_modulus_block_id_1</ArgumentName><Target>[PlatoProblem]:[MaterialModel]:[IsotropicLinearElastic]:YoungsModulus</Target><InitialValue>0.0</InitialValue></Parameter></Operation>";
    ASSERT_STREQ(tGold.c_str(), tData.str().c_str());
    Plato::system("rm -f plato_analyze_operations.xml");
}

TEST(PlatoTestXMLGenerator, AppendRandomTractionVectorToPlatoAnalyzeOperation)
{
    // POSE RANDOM LOADS
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.load_id = "1";
    tLoad1.mIsRandom = true;
    tLoad1.type = "traction";
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.load_id = "2";
    tLoad2.mIsRandom = true;
    tLoad2.type = "traction";
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoad2.values.push_back("6");
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::Load tLoad3;
    tLoad3.load_id = "3";
    tLoad3.mIsRandom = false;
    tLoad3.type = "traction";
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("7");
    tLoad3.values.push_back("8");
    tLoad3.values.push_back("9");
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad4;
    tLoad4.load_id = "1";
    tLoad4.mIsRandom = true;
    tLoad4.type = "traction";
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("11");
    tLoad4.values.push_back("12");
    tLoad4.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::Load tLoad5;
    tLoad5.load_id = "2";
    tLoad5.mIsRandom = true;
    tLoad5.type = "traction";
    tLoad5.app_name = "sideset";
    tLoad5.values.push_back("14");
    tLoad5.values.push_back("15");
    tLoad5.values.push_back("16");
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    XMLGen::InputData tXMLMetaData;
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.finalize());

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_random_traction_vector_to_plato_analyze_operation(tXMLMetaData, tDocument);

    auto tParameter = tDocument.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    std::vector<std::string> tKeys = {"ArgumentName", "Target", "InitialValue"};
    std::vector<std::string> tValues =
        {"traction_load_id_2_x_axis", "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition with ID 2]:Values(0)", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling();
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tKeys = {"ArgumentName", "Target", "InitialValue"};
    tValues = {"traction_load_id_2_y_axis", "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition with ID 2]:Values(1)", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling();
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tKeys = {"ArgumentName", "Target", "InitialValue"};
    tValues = {"traction_load_id_2_z_axis", "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition with ID 2]:Values(2)", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling();
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tKeys = {"ArgumentName", "Target", "InitialValue"};
    tValues = {"traction_load_id_1_x_axis", "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition with ID 1]:Values(0)", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling();
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tKeys = {"ArgumentName", "Target", "InitialValue"};
    tValues = {"traction_load_id_1_y_axis", "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition with ID 1]:Values(1)", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling();
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tKeys = {"ArgumentName", "Target", "InitialValue"};
    tValues = {"traction_load_id_1_z_axis", "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition with ID 1]:Values(2)", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    // EXPECT NEXT SIBLING TO BE EMPTY SINCE PREVIOUS PARAMETER IS THE LAST SIBLING ON THE LIST
    tParameter = tParameter.next_sibling();
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendLoadAndMaterialPropertiesToPlatoAnalyzeConstraintValueOperation)
{
    // POSE INPUTS
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.optimization_type = "topology";
    XMLGen::Constraint tConstraint;
    tConstraint.code("plato_analyze");
    tXMLMetaData.constraints.push_back(tConstraint);

    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic linear elastic");
    tMaterial1.property("youngs_modulus", "1");
    tMaterial1.property("poissons_ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic linear elastic");
    tMaterial2.property("youngs_modulus", "1");
    tMaterial2.property("poissons_ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCase1 = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic linear elastic");
    tMaterial3.property("youngs_modulus", "1.1");
    tMaterial3.property("poissons_ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic linear elastic");
    tMaterial4.property("youngs_modulus", "1");
    tMaterial4.property("poissons_ratio", "0.3");

    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial3});
    tMaterialSetTwo.insert({"2", tMaterial4});
    auto tRandomMaterialCase2 = std::make_pair(0.5, tMaterialSetTwo);

    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.load_id = "1";
    tLoad1.mIsRandom = true;
    tLoad1.type = "traction";
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1");
    tLoad1.values.push_back("2");
    tLoad1.values.push_back("3");
    tLoadCase1.loads.push_back(tLoad1);
    XMLGen::Load tLoad2;
    tLoad2.load_id = "2";
    tLoad2.mIsRandom = true;
    tLoad2.type = "traction";
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("4");
    tLoad2.values.push_back("5");
    tLoad2.values.push_back("6");
    tLoadCase1.loads.push_back(tLoad2);
    XMLGen::Load tLoad3;
    tLoad3.load_id = "3";
    tLoad3.type = "traction";
    tLoad3.mIsRandom = false;
    tLoad3.app_name = "sideset";
    tLoad3.values.push_back("7");
    tLoad3.values.push_back("8");
    tLoad3.values.push_back("9");
    tLoadCase1.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad4;
    tLoad4.mIsRandom = true;
    tLoad4.load_id = "1";
    tLoad4.type = "traction";
    tLoad4.app_name = "sideset";
    tLoad4.values.push_back("11");
    tLoad4.values.push_back("12");
    tLoad4.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad4);
    XMLGen::Load tLoad5;
    tLoad5.mIsRandom = true;
    tLoad5.load_id = "2";
    tLoad5.type = "traction";
    tLoad5.app_name = "sideset";
    tLoad5.values.push_back("14");
    tLoad5.values.push_back("15");
    tLoad5.values.push_back("16");
    tLoadCase2.loads.push_back(tLoad5);
    tLoadCase2.loads.push_back(tLoad3); // append deterministic load
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCase1));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCase2));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.finalize());

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_compute_constraint_value_to_plato_analyze_operation(tXMLMetaData, tDocument);

    // TEST OPERATION I/O ARGUMENTS
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = { "Function", "Name", "Input", "Output", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Parameter", "Parameter", "Parameter", "Parameter", "Parameter" };
    std::vector<std::string> tValues = { "ComputeCriterionValue", "Compute Constraint Value", "", "",
        "", "", "", "", "", "", "", "", "", "" };
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({ "ArgumentName" }, { "Topology" }, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"Argument", "ArgumentName"}, {"Value", "Constraint Value"}, tOutput);

    // TEST RANDOM PARAMETERS
    auto tParameter = tOperation.child("Parameter");
    std::vector<std::string> tGoldArgumentNames = {"traction_load_id_1_x_axis", "traction_load_id_1_y_axis", "traction_load_id_1_z_axis",
        "traction_load_id_2_x_axis", "traction_load_id_2_y_axis", "traction_load_id_2_z_axis", "poissons_ratio_block_id_1",
        "poissons_ratio_block_id_2", "youngs_modulus_block_id_1", "youngs_modulus_block_id_2"};
    std::vector<std::string> tGoldTargetNames =
        {"[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition with ID 2]:Values(0)",
         "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition with ID 2]:Values(1)",
         "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition with ID 2]:Values(2)",
         "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition with ID 1]:Values(0)",
         "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition with ID 1]:Values(1)",
         "[Plato Problem]:[Natural Boundary Conditions]:[Random Traction Vector Boundary Condition with ID 1]:Values(2)",
         "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Poissons Ratio",
         "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Youngs Modulus"};
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());

        auto tGoldArgName = std::find(tGoldArgumentNames.begin(), tGoldArgumentNames.end(), tParameter.child("ArgumentName").child_value());
        ASSERT_TRUE(tGoldArgName != tGoldArgumentNames.end());
        ASSERT_STREQ(tGoldArgName->c_str(), tParameter.child("ArgumentName").child_value());

        auto tGoldTarget = std::find(tGoldTargetNames.begin(), tGoldTargetNames.end(), tParameter.child("Target").child_value());
        ASSERT_TRUE(tGoldTarget != tGoldTargetNames.end());
        ASSERT_STREQ(tGoldTarget->c_str(), tParameter.child("Target").child_value());

        ASSERT_STREQ("0.0", tParameter.child("InitialValue").child_value());
        tParameter = tParameter.next_sibling();
    }
}

TEST(PlatoTestXMLGenerator, AppendMaterialPropertiesToPlatoAnalyzeOperation)
{
    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic linear elastic");
    tMaterial1.property("youngs_modulus", "1");
    tMaterial1.property("poissons_ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic linear thermoelastic");
    tMaterial2.property("youngs_modulus", "1");
    tMaterial2.property("poissons_ratio", "0.3");
    tMaterial2.property("Thermal_Expansivity", "1.0e-8");
    tMaterial2.property("Thermal_Conductivity", "9");
    tMaterial2.property("Reference_Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic linear elastic");
    tMaterial3.property("youngs_modulus", "1.1");
    tMaterial3.property("poissons_ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic linear thermoelastic");
    tMaterial4.property("youngs_modulus", "1");
    tMaterial4.property("poissons_ratio", "0.3");
    tMaterial4.property("Thermal_Expansivity", "1.0e-8");
    tMaterial4.property("Thermal_Conductivity", "9");
    tMaterial4.property("Reference_Temperature", "1e-4");

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
    XMLGen::append_random_material_properties_to_plato_analyze_operation(tXMLMetaData, tDocument);

    auto tParameter = tDocument.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    std::vector<std::string> tKeys = {"ArgumentName", "Target", "InitialValue"};
    std::vector<std::string> tValues = {"poissons_ratio_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs_modulus_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"reference_temperature_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Reference Temperature", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal_conductivity_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Conductivity", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal_expansivity_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Expansivity", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons_ratio_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs_modulus_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);
}

TEST(PlatoTestXMLGenerator, MaterialFunctionInterface_ErrorInvalidCategory)
{
    pugi::xml_document tDocument;
    std::vector<std::pair<std::string,std::string>> tTags =
    { {"youngs_modulus_block_id_1", "youngs_modulus"}, {"poissons_ratio_block_id_1", "poissons_ratio"} };

    XMLGen::MaterialFunctionInterface tInterface;
    ASSERT_THROW(tInterface.call("viscoelastic", tTags, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, MaterialFunctionInterface_Elastic)
{
    pugi::xml_document tDocument;
    std::vector<std::pair<std::string,std::string>> tTags =
    { {"youngs_modulus_block_id_1", "youngs_modulus"}, {"poissons_ratio_block_id_1", "poissons_ratio"} };

    XMLGen::MaterialFunctionInterface tInterface;
    ASSERT_NO_THROW(tInterface.call("isotropic linear elastic", tTags, tDocument));

    auto tParameter = tDocument.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    std::vector<std::string> tKeys = {"ArgumentName", "Target", "InitialValue"};
    std::vector<std::string> tValues = {"youngs_modulus_block_id_1",
        "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons_ratio_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);
}

TEST(PlatoTestXMLGenerator, MaterialFunctionInterface_ThermoElastic)
{
    pugi::xml_document tDocument;
    std::vector<std::pair<std::string,std::string>> tTags =
    { {"youngs_modulus_block_id_1", "youngs_modulus"}, {"poissons_ratio_block_id_1", "poissons_ratio"},
      {"thermal_expansivity_block_id_1", "thermal_expansivity"},
      {"thermal_conductivity_block_id_1", "thermal_conductivity"},
      {"reference_temperature_block_id_1", "reference_temperature"} };

    XMLGen::MaterialFunctionInterface tInterface;
    ASSERT_NO_THROW(tInterface.call("isotropic linear thermoelastic", tTags, tDocument));

    auto tParameter = tDocument.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    std::vector<std::string> tKeys = {"ArgumentName", "Target", "InitialValue"};
    std::vector<std::string> tValues = {"youngs_modulus_block_id_1",
        "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons_ratio_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal_expansivity_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Expansivity", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal_conductivity_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Conductivity", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"reference_temperature_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Reference Temperature", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);
}

TEST(PlatoTestXMLGenerator, MaterialFunctionInterface_OrthotropicLinearElastic)
{
    pugi::xml_document tDocument;
    std::vector<std::pair<std::string,std::string>> tTags =
    {
      {"youngs_modulus_x_block_id_1", "youngs_modulus_x"}, {"youngs_modulus_y_block_id_1", "youngs_modulus_y"},
      {"youngs_modulus_z_block_id_1", "youngs_modulus_z"}, {"poissons_ratio_xy_block_id_1", "poissons_ratio_xy"},
      {"poissons_ratio_xz_block_id_1", "poissons_ratio_xz"}, {"poissons_ratio_yz_block_id_1", "poissons_ratio_yz"},
      {"shear_modulus_xy_block_id_1", "shear_modulus_xy"}, {"shear_modulus_xz_block_id_1", "shear_modulus_xz"},
      {"shear_modulus_yz_block_id_1", "shear_modulus_yz"}
    };

    XMLGen::MaterialFunctionInterface tInterface;
    ASSERT_NO_THROW(tInterface.call("orthotropic linear elastic", tTags, tDocument));

    // TEST RESULTS
    auto tParameter = tDocument.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    std::vector<std::string> tKeys = {"ArgumentName", "Target", "InitialValue"};
    std::vector<std::string> tValues = {"youngs_modulus_x_block_id_1",
        "[Plato Problem]:[Material Model]:[Orthotropic Linear Elastic]:Youngs Modulus X", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs_modulus_y_block_id_1", "[Plato Problem]:[Material Model]:[Orthotropic Linear Elastic]:Youngs Modulus Y", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs_modulus_z_block_id_1", "[Plato Problem]:[Material Model]:[Orthotropic Linear Elastic]:Youngs Modulus Z", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons_ratio_xy_block_id_1", "[Plato Problem]:[Material Model]:[Orthotropic Linear Elastic]:Poissons Ratio XY", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons_ratio_xz_block_id_1", "[Plato Problem]:[Material Model]:[Orthotropic Linear Elastic]:Poissons Ratio XZ", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons_ratio_yz_block_id_1", "[Plato Problem]:[Material Model]:[Orthotropic Linear Elastic]:Poissons Ratio YZ", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"shear_modulus_xy_block_id_1", "[Plato Problem]:[Material Model]:[Orthotropic Linear Elastic]:Shear Modulus XY", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"shear_modulus_xz_block_id_1", "[Plato Problem]:[Material Model]:[Orthotropic Linear Elastic]:Shear Modulus XZ", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"shear_modulus_yz_block_id_1", "[Plato Problem]:[Material Model]:[Orthotropic Linear Elastic]:Shear Modulus YZ", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, ReturnMaterialPropertyTagsForPlatoAnalyzeOperationXmlFile_ErrorNoSamples)
{
    XMLGen::RandomMetaData tRandomMetaData;
    ASSERT_THROW(XMLGen::return_random_material_metadata_for_plato_analyze_operation_xml_file(tRandomMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, ReturnMaterialPropertyTagsForPlatoAnalyzeOperationXmlFile)
{
    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic linear elastic");
    tMaterial1.property("youngs_modulus", "1");
    tMaterial1.property("poissons_ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic linear thermoelastic");
    tMaterial2.property("youngs_modulus", "1");
    tMaterial2.property("poissons_ratio", "0.3");
    tMaterial2.property("Thermal_Expansivity", "1.0e-8");
    tMaterial2.property("Thermal_Conductivity", "9");
    tMaterial2.property("Reference_Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic linear elastic");
    tMaterial3.property("youngs_modulus", "1.1");
    tMaterial3.property("poissons_ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic linear thermoelastic");
    tMaterial4.property("youngs_modulus", "1");
    tMaterial4.property("poissons_ratio", "0.3");
    tMaterial4.property("Thermal_Expansivity", "1.0e-8");
    tMaterial4.property("Thermal_Conductivity", "9");
    tMaterial4.property("Reference_Temperature", "1e-4");

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
    std::vector<std::string> tGoldMatPropTags = {"youngs_modulus", "poissons_ratio", "thermal_expansivity",
        "thermal_conductivity", "reference_temperature"};
    std::vector<std::string> tGoldArgumentTags = {"youngs_modulus_block_id_1", "poissons_ratio_block_id_1",
        "youngs_modulus_block_id_2", "poissons_ratio_block_id_2", "thermal_expansivity_block_id_2",
        "thermal_conductivity_block_id_2", "reference_temperature_block_id_2"};
    auto tMaterials = XMLGen::return_random_material_metadata_for_plato_analyze_operation_xml_file(tRandomMetaData);
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
    Plato::system("rm -f amgx.json");
}

TEST(PlatoTestXMLGenerator, AppendDeterminsiticQoI_InvalidKey)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(tInputData.mOutputMetaData.appendDeterminsiticQoI("fluid pressure", "fpressure"), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemToPlatoAnalyzeOperation_ErrorEmptyUpdateFrequencyKey)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::Service tService;
    tService.updateProblem("true");
    tInputData.append(tService);
    ASSERT_THROW(XMLGen::append_update_problem_to_plato_analyze_operation(tInputData, tDocument), std::runtime_error);
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemToPlatoAnalyzeOperation_DoNotWriteUpdateProblemOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    XMLGen::Service tService;
    tService.updateProblem("false");
    tInputData.append(tService);
    ASSERT_NO_THROW(XMLGen::append_update_problem_to_plato_analyze_operation(tInputData, tDocument));
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemToPlatoAnalyzeOperation)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.mProblemUpdateFrequency = "5";
    XMLGen::Service tService;
    tService.updateProblem("true");
    tInputData.append(tService);
    ASSERT_NO_THROW(XMLGen::append_update_problem_to_plato_analyze_operation(tInputData, tDocument));
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());

    PlatoTestXMLGenerator::test_children({"Function", "Name"}, {"UpdateProblem", "Update Problem"}, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendComputeObjectiveValueToPlatoAnalyzeOperation)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.code_name = "plato_analyze";
    tMetaData.objectives.push_back(tObjective);
    tMetaData.optimization_type = "topology";

    pugi::xml_document tDocument;
    XMLGen::append_compute_objective_value_to_plato_analyze_operation(tMetaData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Input", "Output"};
    std::vector<std::string> tValues = {"ComputeCriterionValue", "Compute Objective Value", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    auto tOutput = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"Argument","ArgumentName"}, {"Value", "Objective Value"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendComputeObjectiveGradientToPlatoAnalyzeOperation)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.code_name = "plato_analyze";
    tMetaData.objectives.push_back(tObjective);
    tMetaData.optimization_type = "topology";

    pugi::xml_document tDocument;
    XMLGen::append_compute_objective_gradient_to_plato_analyze_operation(tMetaData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Input", "Output"};
    std::vector<std::string> tValues = {"ComputeCriterionGradient", "Compute Objective Gradient", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    auto tOutput = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"Argument", "ArgumentName"}, {"Gradient", "Objective Gradient"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendComputeConstraintValueToPlatoAnalyzeOperation)
{
    XMLGen::InputData tMetaData;
    XMLGen::Constraint tConstraint;
    tConstraint.code("plato_analyze");
    tMetaData.constraints.push_back(tConstraint);
    tMetaData.optimization_type = "topology";

    pugi::xml_document tDocument;
    XMLGen::append_compute_constraint_value_to_plato_analyze_operation(tMetaData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Input", "Output"};
    std::vector<std::string> tValues = {"ComputeCriterionValue", "Compute Constraint Value", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    auto tOutput = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"Argument","ArgumentName"}, {"Value", "Constraint Value"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendComputeConstraintGradientToPlatoAnalyzeOperation)
{
    XMLGen::InputData tMetaData;
    XMLGen::Constraint tConstraint;
    tConstraint.code("plato_analyze");
    tMetaData.optimization_type = "topology";
    tMetaData.constraints.push_back(tConstraint);

    pugi::xml_document tDocument;
    XMLGen::append_compute_constraint_gradient_to_plato_analyze_operation(tMetaData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Input", "Output"};
    std::vector<std::string> tValues = {"ComputeCriterionGradient", "Compute Constraint Gradient", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    auto tOutput = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"Argument", "ArgumentName"}, {"Gradient", "Constraint Gradient"}, tOutput);
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

TEST(PlatoTestXMLGenerator, AppendComputeRandomConstraintValueToPlatoAnalyzeOperation_EmptyOptimizationType)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_compute_constraint_value_to_plato_analyze_operation(tInputData, tDocument), std::runtime_error);
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomConstraintValueToPlatoAnalyzeOperation_EmptyConstraint)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::append_compute_constraint_value_to_plato_analyze_operation(tInputData, tDocument);
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomConstraintValueToPlatoAnalyzeOperation_NotPlatoAnalyzePerformer)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::Constraint tConstraint;
    tConstraint.code("sierra_sd");
    tInputData.constraints.push_back(tConstraint);
    XMLGen::append_compute_constraint_value_to_plato_analyze_operation(tInputData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomConstraintValueToPlatoAnalyzeOperation)
{
    // POSE INPUTS
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.optimization_type = "topology";
    XMLGen::Constraint tConstraint;
    tConstraint.code("plato_analyze");
    tXMLMetaData.constraints.push_back(tConstraint);

    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic linear elastic");
    tMaterial1.property("youngs_modulus", "1");
    tMaterial1.property("poissons_ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic linear thermoelastic");
    tMaterial2.property("youngs_modulus", "1");
    tMaterial2.property("poissons_ratio", "0.3");
    tMaterial2.property("Thermal_Expansivity", "1.0e-8");
    tMaterial2.property("Thermal_Conductivity", "9");
    tMaterial2.property("Reference_Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic linear elastic");
    tMaterial3.property("youngs_modulus", "1.1");
    tMaterial3.property("poissons_ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic linear thermoelastic");
    tMaterial4.property("youngs_modulus", "1");
    tMaterial4.property("poissons_ratio", "0.3");
    tMaterial4.property("Thermal_Expansivity", "1.0e-8");
    tMaterial4.property("Thermal_Conductivity", "9");
    tMaterial4.property("Reference_Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial3});
    tMaterialSetTwo.insert({"2", tMaterial4});
    auto tRandomMaterialCaseTwo = std::make_pair(0.5, tMaterialSetTwo);

    // CONSTRUCT SAMPLES SET
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseOne));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseTwo));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.finalize());

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_compute_constraint_value_to_plato_analyze_operation(tXMLMetaData, tDocument);

    // TEST OPERATION I/O PARAMETERS
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = { "Function", "Name", "Input", "Output", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Parameter", "Parameter" };
    std::vector<std::string> tValues = { "ComputeCriterionValue", "Compute Constraint Value", "", "", "", "", "", "", "", "", "" };
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({ "ArgumentName" }, { "Topology" }, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"Argument", "ArgumentName"}, {"Value", "Constraint Value"}, tOutput);

    // TEST RANDOM PARAMETERS
    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tKeys = {"ArgumentName", "Target", "InitialValue"};
    tValues = {"poissons_ratio_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs_modulus_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"reference_temperature_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Reference Temperature", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal_conductivity_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Conductivity", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal_expansivity_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Expansivity", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons_ratio_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs_modulus_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomConstraintGradientToPlatoAnalyzeOperation_EmptyOptimizationType)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_compute_constraint_gradient_to_plato_analyze_operation(tInputData, tDocument), std::runtime_error);
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomConstraintGradientToPlatoAnalyzeOperation_Empty)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::append_compute_constraint_gradient_to_plato_analyze_operation(tInputData, tDocument);
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomConstraintGradientToPlatoAnalyzeOperation_NotPlatoAnalyzePerformer)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::Constraint tConstraint;
    tConstraint.code("sierra_sd");
    tInputData.constraints.push_back(tConstraint);
    XMLGen::append_compute_constraint_gradient_to_plato_analyze_operation(tInputData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomConstraintGradientToPlatoAnalyzeOperation)
{
    // POSE INPUTS
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.optimization_type = "topology";
    XMLGen::Constraint tConstraint;
    tConstraint.code("plato_analyze");
    tXMLMetaData.constraints.push_back(tConstraint);

    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic linear elastic");
    tMaterial1.property("youngs_modulus", "1");
    tMaterial1.property("poissons_ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic linear thermoelastic");
    tMaterial2.property("youngs_modulus", "1");
    tMaterial2.property("poissons_ratio", "0.3");
    tMaterial2.property("Thermal_Expansivity", "1.0e-8");
    tMaterial2.property("Thermal_Conductivity", "9");
    tMaterial2.property("Reference_Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic linear elastic");
    tMaterial3.property("youngs_modulus", "1.1");
    tMaterial3.property("poissons_ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic linear thermoelastic");
    tMaterial4.property("youngs_modulus", "1");
    tMaterial4.property("poissons_ratio", "0.3");
    tMaterial4.property("Thermal_Expansivity", "1.0e-8");
    tMaterial4.property("Thermal_Conductivity", "9");
    tMaterial4.property("Reference_Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial3});
    tMaterialSetTwo.insert({"2", tMaterial4});
    auto tRandomMaterialCaseTwo = std::make_pair(0.5, tMaterialSetTwo);

    // CONSTRUCT SAMPLES SET
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseOne));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseTwo));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.finalize());

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_compute_constraint_gradient_to_plato_analyze_operation(tXMLMetaData, tDocument);

    // TEST OPERATION I/O ARGUMENTS
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = { "Function", "Name", "Input", "Output", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Parameter", "Parameter" };
    std::vector<std::string> tValues = { "ComputeCriterionGradient", "Compute Constraint Gradient", "", "", "", "", "", "", "", "", "" };
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({ "ArgumentName" }, { "Topology" }, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"Argument", "ArgumentName"}, {"Gradient", "Constraint Gradient"}, tOutput);

    // TEST RANDOM PARAMETERS
    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tKeys = {"ArgumentName", "Target", "InitialValue"};
    tValues = {"poissons_ratio_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs_modulus_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"reference_temperature_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Reference Temperature", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal_conductivity_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Conductivity", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal_expansivity_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Expansivity", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons_ratio_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs_modulus_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomObjectiveValueToPlatoAnalyzeOperation_EmptyOptimizationType)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_compute_objective_value_to_plato_analyze_operation(tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomObjectiveValueToPlatoAnalyzeOperation_Empty)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    ASSERT_NO_THROW(XMLGen::append_compute_objective_value_to_plato_analyze_operation(tInputData, tDocument));
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomObjectiveValueToPlatoAnalyzeOperation_NotPlatoAnalyzePerformer)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::Objective tObjective;
    tObjective.code_name = "sierra_sd";
    tInputData.objectives.push_back(tObjective);
    ASSERT_NO_THROW(XMLGen::append_compute_objective_value_to_plato_analyze_operation(tInputData, tDocument));

    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomObjectiveValueToPlatoAnalyzeOperation)
{
    // POSE INPUTS
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.optimization_type = "topology";
    XMLGen::Objective tObjective;
    tObjective.code_name = "plato_analyze";
    tXMLMetaData.objectives.push_back(tObjective);

    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic linear elastic");
    tMaterial1.property("youngs_modulus", "1");
    tMaterial1.property("poissons_ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic linear thermoelastic");
    tMaterial2.property("youngs_modulus", "1");
    tMaterial2.property("poissons_ratio", "0.3");
    tMaterial2.property("Thermal_Expansivity", "1.0e-8");
    tMaterial2.property("Thermal_Conductivity", "9");
    tMaterial2.property("Reference_Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic linear elastic");
    tMaterial3.property("youngs_modulus", "1.1");
    tMaterial3.property("poissons_ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic linear thermoelastic");
    tMaterial4.property("youngs_modulus", "1");
    tMaterial4.property("poissons_ratio", "0.3");
    tMaterial4.property("Thermal_Expansivity", "1.0e-8");
    tMaterial4.property("Thermal_Conductivity", "9");
    tMaterial4.property("Reference_Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial3});
    tMaterialSetTwo.insert({"2", tMaterial4});
    auto tRandomMaterialCaseTwo = std::make_pair(0.5, tMaterialSetTwo);

    // CONSTRUCT SAMPLES SET
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseOne));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseTwo));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.finalize());

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_compute_objective_value_to_plato_analyze_operation(tXMLMetaData, tDocument);

    // TEST OPERATION INPUTS
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = { "Function", "Name", "Input", "Output", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Parameter", "Parameter" };
    std::vector<std::string> tValues = { "ComputeCriterionValue", "Compute Objective Value", "", "", "", "", "", "", "", "", "" };
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({ "ArgumentName" }, { "Topology" }, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"Argument", "ArgumentName"}, {"Value", "Objective Value"}, tOutput);

    // TEST RANDOM PARAMETERS
    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tKeys = {"ArgumentName", "Target", "InitialValue"};
    tValues = {"poissons_ratio_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs_modulus_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"reference_temperature_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Reference Temperature", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal_conductivity_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Conductivity", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal_expansivity_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Expansivity", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons_ratio_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs_modulus_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomObjectiveGradientToPlatoAnalyzeOperation_EmptyOptimizationType)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    ASSERT_THROW(XMLGen::append_compute_objective_gradient_to_plato_analyze_operation(tInputData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomObjectiveGradientToPlatoAnalyzeOperation_Empty)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    ASSERT_NO_THROW(XMLGen::append_compute_objective_gradient_to_plato_analyze_operation(tInputData, tDocument));
    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomObjectiveGradientToPlatoAnalyzeOperation_NotPlatoAnalyzePerformer)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tInputData;
    tInputData.optimization_type = "topology";
    XMLGen::Objective tObjective;
    tObjective.code_name = "sierra_sd";
    tInputData.objectives.push_back(tObjective);
    ASSERT_NO_THROW(XMLGen::append_compute_objective_gradient_to_plato_analyze_operation(tInputData, tDocument));

    auto tOperation = tDocument.child("Operation");
    ASSERT_TRUE(tOperation.empty());
}

TEST(PlatoTestXMLGenerator, AppendComputeRandomObjectiveGradientToPlatoAnalyzeOperation)
{
    // POSE INPUTS
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.optimization_type = "topology";
    XMLGen::Objective tObjective;
    tObjective.code_name = "plato_analyze";
    tXMLMetaData.objectives.push_back(tObjective);

    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("2");
    tMaterial1.category("isotropic linear elastic");
    tMaterial1.property("youngs_modulus", "1");
    tMaterial1.property("poissons_ratio", "0.3");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");
    tMaterial2.category("isotropic linear thermoelastic");
    tMaterial2.property("youngs_modulus", "1");
    tMaterial2.property("poissons_ratio", "0.3");
    tMaterial2.property("Thermal_Expansivity", "1.0e-8");
    tMaterial2.property("Thermal_Conductivity", "9");
    tMaterial2.property("Reference_Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    tMaterialSetOne.insert({"2", tMaterial2});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial3;
    tMaterial3.id("2");
    tMaterial3.category("isotropic linear elastic");
    tMaterial3.property("youngs_modulus", "1.1");
    tMaterial3.property("poissons_ratio", "0.33");
    XMLGen::Material tMaterial4;
    tMaterial4.id("2");
    tMaterial4.category("isotropic linear thermoelastic");
    tMaterial4.property("youngs_modulus", "1");
    tMaterial4.property("poissons_ratio", "0.3");
    tMaterial4.property("Thermal_Expansivity", "1.0e-8");
    tMaterial4.property("Thermal_Conductivity", "9");
    tMaterial4.property("Reference_Temperature", "1e-4");

    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial3});
    tMaterialSetTwo.insert({"2", tMaterial4});
    auto tRandomMaterialCaseTwo = std::make_pair(0.5, tMaterialSetTwo);

    // CONSTRUCT SAMPLES SET
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseOne));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseTwo));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.finalize());

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_compute_objective_gradient_to_plato_analyze_operation(tXMLMetaData, tDocument);

    // TEST OPERATION I/O ARGUMENTS
    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = { "Function", "Name", "Input", "Output", "Parameter", "Parameter",
        "Parameter", "Parameter", "Parameter", "Parameter", "Parameter" };
    std::vector<std::string> tValues = { "ComputeCriterionGradient", "Compute Objective Gradient", "", "", "", "", "", "", "", "", "" };
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.child("Input");
    ASSERT_FALSE(tInput.empty());
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({ "ArgumentName" }, { "Topology" }, tInput);

    auto tOutput = tOperation.child("Output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"Argument", "ArgumentName"}, {"Gradient", "Objective Gradient"}, tOutput);

    // TEST RANDOM PARAMETERS
    auto tParameter = tOperation.child("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tKeys = {"ArgumentName", "Target", "InitialValue"};
    tValues = {"poissons_ratio_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs_modulus_block_id_1", "[Plato Problem]:[Material Model]:[Isotropic Linear Elastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"reference_temperature_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Reference Temperature", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal_conductivity_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Conductivity", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"thermal_expansivity_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Thermal Expansivity", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"poissons_ratio_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Poissons Ratio", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);

    tParameter = tParameter.next_sibling("Parameter");
    ASSERT_FALSE(tParameter.empty());
    ASSERT_STREQ("Parameter", tParameter.name());
    tValues = {"youngs_modulus_block_id_2", "[Plato Problem]:[Material Model]:[Isotropic Linear Thermoelastic]:Youngs Modulus", "0.0"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tParameter);
}

TEST(PlatoTestXMLGenerator, IsAnyObjectiveComputedByPlatoAnalyze)
{
    // NO OBJECTIVE DEFINED
    XMLGen::InputData tInputData;
    ASSERT_FALSE(XMLGen::is_any_objective_computed_by_plato_analyze(tInputData));

    XMLGen::Objective tObjective1;
    tObjective1.code_name = "sierra_sd";
    tInputData.objectives.push_back(tObjective1);
    ASSERT_FALSE(XMLGen::is_any_objective_computed_by_plato_analyze(tInputData));

    XMLGen::Objective tObjective2;
    tObjective2.code_name = "plato_analyze";
    tInputData.objectives.push_back(tObjective2);
    ASSERT_TRUE(XMLGen::is_any_objective_computed_by_plato_analyze(tInputData));

    tObjective2.code_name = "sierra_sd";
    tInputData.objectives.pop_back();
    tInputData.objectives.push_back(tObjective2);
    ASSERT_FALSE(XMLGen::is_any_objective_computed_by_plato_analyze(tInputData));

    tObjective2.code_name = "PLATO_Analyze";
    tInputData.objectives.pop_back();
    tInputData.objectives.push_back(tObjective2);
    ASSERT_TRUE(XMLGen::is_any_objective_computed_by_plato_analyze(tInputData));
}

TEST(PlatoTestXMLGenerator, IsPlatoAnalyzePerformer)
{
    ASSERT_FALSE(XMLGen::is_plato_analyze_code("plato_main"));
    ASSERT_TRUE(XMLGen::is_plato_analyze_code("plato_analyze"));
    ASSERT_TRUE(XMLGen::is_plato_analyze_code("plAto_anAlyZe"));
    ASSERT_FALSE(XMLGen::is_plato_analyze_code("plato analyze"));
    ASSERT_FALSE(XMLGen::is_plato_analyze_code("plAto anAlyZe"));
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
