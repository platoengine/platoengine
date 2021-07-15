/*
 * XMLGeneratorSrom_UnitTester.cpp
 *
 *  Created on: Jun 25, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester.hpp"

#include "Plato_SromXMLGenTools.hpp"
#include "XMLGeneratorRandomMetadata.hpp"

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, CheckRandomLoadIdentificationNumbers)
{
    XMLGen::InputData tInputMetadata;

    // POSE LOAD SCENARIO 1
    XMLGen::Scenario tScenario1;
    tScenario1.id("1");
    std::vector<std::string> tLoadIDs = {"1","2"};
    tScenario1.setLoadIDs(tLoadIDs);
    tInputMetadata.append(tScenario1);

    // POSE LOAD SCENARIO 2
    XMLGen::Scenario tScenario2;
    tScenario2.id("2");
    tLoadIDs = {"3", "4"};
    tScenario2.setLoadIDs(tLoadIDs);
    tInputMetadata.append(tScenario2);

    tInputMetadata.objective.scenarioIDs.push_back("1");
    tInputMetadata.objective.scenarioIDs.push_back("2");

    // POSE UNCERTAINTIES
    XMLGen::Uncertainty tUQCase1;
    tUQCase1.id("1");
    tUQCase1.category("load");
    tInputMetadata.uncertainties.push_back(tUQCase1);

    // NO ERROR THROWN
    ASSERT_NO_THROW(Plato::srom::check_random_load_identification_numbers(tInputMetadata));

    // ERROR THROWN
    tInputMetadata.uncertainties[0].id("0");
    ASSERT_THROW(Plato::srom::check_random_load_identification_numbers(tInputMetadata), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, CheckOutputMaterialSetType_Error)
{
    // POSE LOAD CASE 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("1");
    XMLGen::Material tMaterial2;
    tMaterial2.id("2");

    // APPEND LOAD CASES
    XMLGen::InputData tInputMetadata;
    tInputMetadata.materials.push_back(tMaterial1);
    tInputMetadata.materials.push_back(tMaterial2);

    // POSE UNCERTAINTIES
    XMLGen::Uncertainty tUQCase;
    tUQCase.id("1");
    tUQCase.category("material");
    tInputMetadata.uncertainties.push_back(tUQCase);

    // NO ERROR THROWN
    ASSERT_NO_THROW(Plato::srom::check_random_material_identification_numbers(tInputMetadata));

    // ERROR THROWN
    tInputMetadata.uncertainties[0].id("10");
    ASSERT_THROW(Plato::srom::check_random_material_identification_numbers(tInputMetadata), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, CheckOutputLoadSetType_Error)
{
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    tLoadCase1.loads.push_back(XMLGen::Load());
    tLoadCase1.loads[0].type("traction");
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    tLoadCase2.loads.push_back(XMLGen::Load());
    tLoadCase2.loads[0].type("pressure");
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    XMLGen::RandomMetaData tMetaData;
    ASSERT_NO_THROW(tMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tMetaData.finalize());
    ASSERT_EQ(2u, tMetaData.numSamples());

    ASSERT_THROW(Plato::srom::check_output_load_set_types(tMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, CheckOutputLoadSetSize_Error)
{
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    tLoadCase1.loads.push_back(XMLGen::Load());
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    tLoadCase2.loads.push_back(XMLGen::Load());
    tLoadCase2.loads.push_back(XMLGen::Load());
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    XMLGen::RandomMetaData tMetaData;
    ASSERT_NO_THROW(tMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tMetaData.finalize());
    ASSERT_EQ(2u, tMetaData.numSamples());

    ASSERT_THROW(Plato::srom::check_output_load_set_size(tMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, CheckOutputLoadSetApplicationName_Error)
{
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    tLoadCase1.loads.push_back(XMLGen::Load());
    tLoadCase1.loads[0].location_name("sideset");
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    tLoadCase2.loads.push_back(XMLGen::Load());
    tLoadCase2.loads[0].location_name("nodeset");
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    XMLGen::RandomMetaData tMetaData;
    ASSERT_NO_THROW(tMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tMetaData.finalize());
    ASSERT_EQ(2u, tMetaData.numSamples());

    ASSERT_THROW(Plato::srom::check_output_load_set_application_name(tMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, PostprocessMaterialOutputs_ErrorEmptyRandomMaterialCases)
{
    Plato::srom::OutputMetaData tOutput;
    XMLGen::InputData tXMLGenMetaData;
    ASSERT_THROW(Plato::srom::postprocess_material_outputs(tOutput, tXMLGenMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, BuildMaterialSet_ErrorEmptyMaterialList)
{
    Plato::srom::RandomMaterialCase tRandMaterialCase;
    EXPECT_THROW(Plato::srom::build_material_set(tRandMaterialCase), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, BuildMaterialSet)
{
    // POSE DATA
    Plato::srom::RandomMaterial tRandMaterial1;
    tRandMaterial1.blockID("0");
    tRandMaterial1.materialID("10");
    tRandMaterial1.category("isotropic linear elastic");
    tRandMaterial1.append("youngs_modulus", "homogeneous", "1");
    tRandMaterial1.append("poissons_ratio", "homogeneous", "0.3");

    Plato::srom::RandomMaterial tRandMaterial2;
    tRandMaterial2.blockID("1");
    tRandMaterial2.materialID("11");
    tRandMaterial2.category("isotropic linear elastic");
    tRandMaterial2.append("youngs_modulus", "homogeneous", "2");
    tRandMaterial2.append("poissons_ratio", "homogeneous", "0.33");

    Plato::srom::RandomMaterialCase tRandMaterialCase;
    tRandMaterialCase.caseID("0");
    tRandMaterialCase.probability(0.5);
    tRandMaterialCase.append("10", tRandMaterial1);
    tRandMaterialCase.append("11", tRandMaterial2);

    // CALL FUNCTION
    auto tMaterialSet = Plato::srom::build_material_set(tRandMaterialCase);

    // TEST DATA
    const double tTolerance = 1e-6;
    ASSERT_NEAR(0.5, tMaterialSet.first, tTolerance);

    std::vector<std::string> tGoldPropertiesMatOne = {"0.3" , "1"};
    std::vector<std::string> tGoldPropertiesMatTwo = {"0.33" , "2"};

    for(auto& tPair : tMaterialSet.second)
    {
        // unordered map; thus, if statement
        if(tPair.first == "0")
        {
            ASSERT_STREQ("10", tPair.second.id().c_str());
            ASSERT_STREQ("isotropic linear elastic", tPair.second.materialModel().c_str());
            auto tTags = tPair.second.tags();
            for(auto& tTag : tTags)
            {
                auto tTagIndex = &tTag - &tTags[0];
                ASSERT_STREQ(tGoldPropertiesMatOne[tTagIndex].c_str(), tPair.second.property(tTag).c_str());
            }
        }
        else
        {
            ASSERT_STREQ("11", tPair.second.id().c_str());
            ASSERT_STREQ("isotropic linear elastic", tPair.second.materialModel().c_str());
            auto tTags = tPair.second.tags();
            for(auto& tTag : tTags)
            {
                auto tTagIndex = &tTag - &tTags[0];
                ASSERT_STREQ(tGoldPropertiesMatTwo[tTagIndex].c_str(), tPair.second.property(tTag).c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, PostprocessMaterialOutputs)
{
    Plato::srom::OutputMetaData tOutput;
    tOutput.usecase(Plato::srom::usecase::MATERIAL);

    // CASE 1
    Plato::srom::RandomMaterial tRandMaterial1;
    tRandMaterial1.blockID("0");
    tRandMaterial1.materialID("10");
    tRandMaterial1.category("isotropic linear elastic");
    tRandMaterial1.append("youngs_modulus", "homogeneous", "1");
    tRandMaterial1.append("poissons_ratio", "homogeneous", "0.3");

    Plato::srom::RandomMaterial tRandMaterial2;
    tRandMaterial2.blockID("1");
    tRandMaterial2.materialID("11");
    tRandMaterial2.category("isotropic linear elastic");
    tRandMaterial2.append("youngs_modulus", "homogeneous", "2");
    tRandMaterial2.append("poissons_ratio", "homogeneous", "0.33");

    Plato::srom::RandomMaterialCase tRandMaterialCase1;
    tRandMaterialCase1.caseID("0");
    tRandMaterialCase1.probability(0.5);
    tRandMaterialCase1.append("10", tRandMaterial1);
    tRandMaterialCase1.append("11", tRandMaterial2);

    // CASE 2
    Plato::srom::RandomMaterial tRandMaterial3;
    tRandMaterial3.blockID("0");
    tRandMaterial3.materialID("10");
    tRandMaterial3.category("isotropic linear elastic");
    tRandMaterial3.append("youngs_modulus", "homogeneous", "1.1");
    tRandMaterial3.append("poissons_ratio", "homogeneous", "0.31");

    Plato::srom::RandomMaterial tRandMaterial4;
    tRandMaterial4.blockID("1");
    tRandMaterial4.materialID("11");
    tRandMaterial4.category("isotropic linear elastic");
    tRandMaterial4.append("youngs_modulus", "homogeneous", "2.2");
    tRandMaterial4.append("poissons_ratio", "homogeneous", "0.35");

    Plato::srom::RandomMaterialCase tRandMaterialCase2;
    tRandMaterialCase2.caseID("1");
    tRandMaterialCase2.probability(0.5);
    tRandMaterialCase2.append("10", tRandMaterial3);
    tRandMaterialCase2.append("11", tRandMaterial4);

    // APPEND CASES
    tOutput.append(tRandMaterialCase1);
    tOutput.append(tRandMaterialCase2);

    // TEST
    XMLGen::InputData tXMLGenMetaData;
    EXPECT_NO_THROW(Plato::srom::postprocess_material_outputs(tOutput, tXMLGenMetaData));
    EXPECT_NO_THROW(tXMLGenMetaData.mRandomMetaData.finalize());

    std::vector<std::string> tBlockIDs = {"0", "1"};
    std::vector<std::vector<std::string>> tGoldMatIDs = { {"10", "11"}, {"10", "11"} };
    std::vector<std::vector<std::string>> tGoldCategoryIDs = { {"isotropic linear elastic", "isotropic linear elastic"}, {"isotropic linear elastic", "isotropic linear elastic"} };
    std::vector<std::vector<std::vector<std::string>>> tGoldProperties =
        {
          { {"0.3" , "1"}  , {"0.33", "2"  } },
          { {"0.31", "1.1"}, {"0.35", "2.2"} }
        };
    ASSERT_EQ(2u, tXMLGenMetaData.mRandomMetaData.numSamples());
    auto tRandomSamples = tXMLGenMetaData.mRandomMetaData.samples();
    for(auto& tRandomSample : tRandomSamples)
    {
        auto tSampleIndex = &tRandomSample - &tRandomSamples[0];
        for(auto& tBlockID : tBlockIDs)
        {
            auto tBlockIndex = &tBlockID - &tBlockIDs[0];
            auto tMaterial = tRandomSample.material(tBlockID);
            ASSERT_STREQ(tGoldMatIDs[tSampleIndex][tBlockIndex].c_str(), tMaterial.id().c_str());
            ASSERT_STREQ(tGoldCategoryIDs[tSampleIndex][tBlockIndex].c_str(), tMaterial.materialModel().c_str());
            auto tTags = tMaterial.tags();
            for(auto& tTag : tTags)
            {
                auto tTagIndex = &tTag - &tTags[0];
                ASSERT_STREQ(tGoldProperties[tSampleIndex][tBlockIndex][tTagIndex].c_str(), tMaterial.property(tTag).c_str());
            }
        }
    }
}

TEST(PlatoTestXMLGenerator, RandomMetaData_Set_ErrorUndefinedLoadCaseID)
{
    XMLGen::RandomMetaData tMetaData;
    auto tLoadSet = std::make_pair(0.5, XMLGen::LoadCase());
    EXPECT_THROW(tMetaData.append(tLoadSet), std::runtime_error);
}


TEST(PlatoTestXMLGenerator, RandomMetaData_Append_ErrorUndefinedBlockID)
{
    XMLGen::RandomMetaData tMetaData;
    XMLGen::Material tMaterial;
    XMLGen::MaterialSet tMaterialMap;
    tMaterialMap.insert({"", tMaterial});
    auto tRandomMaterialCase = std::make_pair(0.5, tMaterialMap);
    EXPECT_THROW(tMetaData.append(tRandomMaterialCase), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, RandomMetaData_Append_ErrorUndefinedMaterialID)
{
    XMLGen::RandomMetaData tMetaData;
    XMLGen::Material tMaterial;
    XMLGen::MaterialSet tMaterialMap;
    tMaterialMap.insert({"1", tMaterial});
    auto tRandomMaterialCase = std::make_pair(0.5, tMaterialMap);
    EXPECT_THROW(tMetaData.append(tRandomMaterialCase), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, RandomMetaData_Append_ErrorUndefinedCategory)
{
    XMLGen::RandomMetaData tMetaData;
    XMLGen::Material tMaterial;
    tMaterial.id("2");
    tMaterial.materialModel("");
    XMLGen::MaterialSet tMaterialMap;
    tMaterialMap.insert({"1", tMaterial});
    auto tRandomMaterialCase = std::make_pair(0.5, tMaterialMap);
    EXPECT_THROW(tMetaData.append(tRandomMaterialCase), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, RandomMetaData_Append_ErrorUndefinedProperties)
{
    XMLGen::RandomMetaData tMetaData;
    XMLGen::Material tMaterial;
    tMaterial.id("2");
    tMaterial.materialModel("isotropic linear elastic");
    XMLGen::MaterialSet tMaterialMap;
    tMaterialMap.insert({"1", tMaterial});
    auto tRandomMaterialCase = std::make_pair(0.5, tMaterialMap);
    EXPECT_THROW(tMetaData.append(tRandomMaterialCase), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, SetBlockIdentificationNumber_Error1)
{
    // ERROR - EMPTY MATERIAL ID
    Plato::srom::Material tMaterial;
    tMaterial.category("isotropic linear elastic");
    tMaterial.append("poissons_ratio", "homogeneous", "0.35");
    tMaterial.append("youngs_modulus", "homogeneous", "2.5e8");

    std::unordered_map<std::string, std::string> tMap;
    EXPECT_THROW(Plato::srom::set_block_identification_number(tMap, tMaterial), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, SetBlockIdentificationNumber_Error2)
{
    // ERROR - MATERIAL ID IS NOT OWN BY ANY BLOCK ON THE LIST
    Plato::srom::Material tMaterial;
    tMaterial.materialID("1");
    tMaterial.category("isotropic linear elastic");
    tMaterial.append("poissons_ratio", "homogeneous", "0.35");
    tMaterial.append("youngs_modulus", "homogeneous", "2.5e8");

    std::unordered_map<std::string, std::string> tMap;
    EXPECT_THROW(Plato::srom::set_block_identification_number(tMap, tMaterial), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, SetBlockIdentificationNumber_Error3)
{
    // ERROR - EMPTY BLOCK ID
    Plato::srom::Material tMaterial;
    tMaterial.materialID("1");
    tMaterial.category("isotropic linear elastic");
    tMaterial.append("poissons_ratio", "homogeneous", "0.35");
    tMaterial.append("youngs_modulus", "homogeneous", "2.5e8");

    std::unordered_map<std::string, std::string> tMap;
    tMap.insert({"1", ""});
    EXPECT_THROW(Plato::srom::set_block_identification_number(tMap, tMaterial), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, SetBlockIdentificationNumber1)
{
    Plato::srom::Material tMaterial;
    tMaterial.materialID("1");
    tMaterial.category("isotropic linear elastic");
    tMaterial.append("poissons_ratio", "homogeneous", "0.35");
    tMaterial.append("youngs_modulus", "homogeneous", "2.5e8");

    std::unordered_map<std::string, std::string> tMap;
    tMap.insert({"1", "10"});
    EXPECT_TRUE(tMaterial.blockID().empty());
    EXPECT_NO_THROW(Plato::srom::set_block_identification_number(tMap, tMaterial));
    ASSERT_STREQ("10", tMaterial.blockID().c_str());
}

TEST(PlatoTestXMLGenerator, SetBlockIdentificationNumber2)
{
    Plato::srom::Material tMaterial;
    tMaterial.materialID("1");
    tMaterial.category("isotropic linear elastic");
    tMaterial.append("poissons_ratio", "homogeneous", "0.35");
    tMaterial.append("youngs_modulus", "homogeneous", "2.5e8");

    std::unordered_map<std::string, std::string> tMap;
    tMap.insert({"1", "10"});
    tMap.insert({"2", "1"});
    tMap.insert({"13", "14"});
    EXPECT_TRUE(tMaterial.blockID().empty());
    EXPECT_NO_THROW(Plato::srom::set_block_identification_number(tMap, tMaterial));
    ASSERT_STREQ("10", tMaterial.blockID().c_str());
}

TEST(PlatoTestXMLGenerator, BuildBlockIDtoMaterialIDmap_Error1)
{
    XMLGen::InputData tInputMetaData;
    EXPECT_THROW(Plato::srom::build_material_id_to_block_id_map(tInputMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, BuildBlockIDtoMaterialIDmap_Error2)
{
    XMLGen::Material tMaterial;
    tMaterial.id("30");
    tMaterial.materialModel("isotropic linear elastic");
    tMaterial.property("youngs_modulus", "0.5");
    tMaterial.property("poissons_ratio", "0.3");

    XMLGen::InputData tInputMetaData;
    tInputMetaData.materials.push_back(tMaterial);

    EXPECT_THROW(Plato::srom::build_material_id_to_block_id_map(tInputMetaData), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, BuildBlockIDtoMaterialIDmap)
{
    // BUILD MATERIALS AND BLOCKS
    XMLGen::Material tMaterial1;
    tMaterial1.id("30");
    tMaterial1.materialModel("isotropic linear elastic");
    tMaterial1.property("youngs_modulus", "0.5");
    tMaterial1.property("poissons_ratio", "0.3");

    XMLGen::Material tMaterial2;
    tMaterial2.id("3");
    tMaterial2.materialModel("isotropic linear elastic");
    tMaterial2.property("youngs_modulus", "0.5");
    tMaterial2.property("poissons_ratio", "0.3");

    XMLGen::Block tBlock1;
    tBlock1.block_id = "1";
    tBlock1.material_id = "30";
    tBlock1.element_type = "tet4";

    XMLGen::Block tBlock2;
    tBlock2.block_id = "2";
    tBlock2.material_id = "3";
    tBlock2.element_type = "tet4";

    // APPEND MATERIALS AND BLOCKS
    XMLGen::InputData tInputMetaData;
    tInputMetaData.blocks.push_back(tBlock1);
    tInputMetaData.blocks.push_back(tBlock2);
    tInputMetaData.materials.push_back(tMaterial1);
    tInputMetaData.materials.push_back(tMaterial2);

    // BUILD MAP
    auto tMap = Plato::srom::build_material_id_to_block_id_map(tInputMetaData);

    // TEST RESULTS
    ASSERT_STREQ("1", tMap.find("30")->second.c_str());
    ASSERT_STREQ("2", tMap.find("3")->second.c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessNondeterministicMaterialInputs_Error1)
{
    // ERROR - INPUT MATERIAL CONTAINER IS EMPTY
    XMLGen::InputData tInputMetadata;
    Plato::srom::InputMetaData tSromInputs;
    EXPECT_THROW(Plato::srom::preprocess_material_inputs(tInputMetadata, tSromInputs), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, PreprocessNondeterministicMaterialInputs_Error2)
{
    // 1. POSE DATA
    XMLGen::Uncertainty tCase1;
    tCase1.category("load");
    tCase1.attribute("x");
    tCase1.distribution("beta");
    tCase1.id("2");
    tCase1.lower("-2");
    tCase1.upper("2");
    tCase1.mean("0");
    tCase1.std("0.2");
    tCase1.samples("12");
    tCase1.tag("angle_variation");;

    XMLGen::Material tMaterial;
    tMaterial.id("30");
    tMaterial.materialModel("isotropic linear elastic");
    tMaterial.property("youngs_modulus", "0.5");
    tMaterial.property("poissons_ratio", "0.3");

    XMLGen::InputData tMetadata;
    tMetadata.materials.push_back(tMaterial);
    tMetadata.uncertainties.push_back(tCase1);

    // ERROR - NO RANDOM MATERIAL PROPERTIES
    Plato::srom::InputMetaData tSromInputs;
    EXPECT_THROW(Plato::srom::preprocess_material_inputs(tMetadata, tSromInputs), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, PreprocessNondeterministicMaterialInputs)
{
    // 1. POSE UNCERTAINTY DATA
    XMLGen::Uncertainty tCase1;
    tCase1.category("load");
    tCase1.attribute("x");
    tCase1.distribution("beta");
    tCase1.id("2");
    tCase1.lower("-2");
    tCase1.upper("2");
    tCase1.mean("0");
    tCase1.std("0.2");
    tCase1.samples("12");
    tCase1.tag("angle_variation");;

    XMLGen::Uncertainty tCase2;
    tCase2.category("load");
    tCase2.attribute("y");
    tCase2.distribution("beta");
    tCase2.id("20");
    tCase2.lower("-20");
    tCase2.upper("20");
    tCase2.mean("2");
    tCase2.std("2");
    tCase2.samples("8");
    tCase2.tag("angle_variation");;

    XMLGen::Uncertainty tCase3;
    tCase3.category("material");
    tCase3.attribute("homogeneous");
    tCase3.distribution("beta");
    tCase3.id("3");
    tCase3.lower("0.2");
    tCase3.upper("0.35");
    tCase3.mean("0.27");
    tCase3.std("0.05");
    tCase3.samples("6");
    tCase3.tag("poissons_ratio");;

    XMLGen::InputData tMetadata;
    tMetadata.uncertainties.push_back(tCase1);
    tMetadata.uncertainties.push_back(tCase2);
    tMetadata.uncertainties.push_back(tCase3);

    // 2. POSE MATERIAL DATA
    XMLGen::Material tMaterial1;
    tMaterial1.id("30");
    tMaterial1.materialModel("isotropic linear elastic");
    tMaterial1.property("youngs_modulus", "0.5");
    tMaterial1.property("poissons_ratio", "0.3");
    tMetadata.materials.push_back(tMaterial1);

    XMLGen::Material tMaterial2;
    tMaterial2.id("3");
    tMaterial2.materialModel("isotropic linear elastic");
    tMaterial2.property("youngs_modulus", "1");
    tMaterial2.property("poissons_ratio", "0.3");
    tMetadata.materials.push_back(tMaterial2);

    // 3. POSE MATERIAL BLOCKS
    XMLGen::Block tBlock1;
    tBlock1.block_id = "1";
    tBlock1.material_id = "30";
    tBlock1.element_type = "tet4";
    tMetadata.blocks.push_back(tBlock1);

    XMLGen::Block tBlock2;
    tBlock2.block_id = "2";
    tBlock2.material_id = "3";
    tBlock2.element_type = "tet4";
    tMetadata.blocks.push_back(tBlock2);

    // 3. CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    tSromInputs.usecase(Plato::srom::usecase::MATERIAL);
    EXPECT_NO_THROW(Plato::srom::preprocess_material_inputs(tMetadata, tSromInputs));
    ASSERT_EQ(2u, tSromInputs.materials().size());

    // 4.1 TEST RESULTS
    auto tMaterials = tSromInputs.materials();

    // 4.2 DETERMINISTIC MATERIAL
    ASSERT_EQ(2u, tMaterials[0].tags().size());
    ASSERT_STREQ("1", tMaterials[0].blockID().c_str());
    ASSERT_STREQ("30", tMaterials[0].materialID().c_str());
    ASSERT_STREQ("isotropic linear elastic", tMaterials[0].category().c_str());
    ASSERT_STREQ("poissons_ratio", tMaterials[0].tags()[0].c_str());
    ASSERT_STREQ("youngs_modulus", tMaterials[0].tags()[1].c_str());

    EXPECT_FALSE(tMaterials[0].isRandom());
    EXPECT_TRUE(tMaterials[0].isDeterministic());
    EXPECT_TRUE(tMaterials[0].randomVars().empty());
    ASSERT_STREQ("poissons_ratio", tMaterials[0].deterministicVars()[0].tag().c_str());
    ASSERT_STREQ("0.3", tMaterials[0].deterministicVars()[0].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[0].deterministicVars()[0].attribute().c_str());
    ASSERT_STREQ("youngs_modulus", tMaterials[0].deterministicVars()[1].tag().c_str());
    ASSERT_STREQ("0.5", tMaterials[0].deterministicVars()[1].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[0].deterministicVars()[1].attribute().c_str());

    // 4.3 NON-DETERMINISTIC MATERIAL
    ASSERT_STREQ("2", tMaterials[1].blockID().c_str());
    ASSERT_STREQ("3", tMaterials[1].materialID().c_str());
    ASSERT_STREQ("isotropic linear elastic", tMaterials[1].category().c_str());
    ASSERT_STREQ("poissons_ratio", tMaterials[1].tags()[0].c_str());
    ASSERT_STREQ("youngs_modulus", tMaterials[1].tags()[1].c_str());

    EXPECT_TRUE(tMaterials[1].isRandom());
    EXPECT_FALSE(tMaterials[1].isDeterministic());
    ASSERT_EQ(1u, tMaterials[1].deterministicVars().size());
    ASSERT_STREQ("youngs_modulus", tMaterials[1].deterministicVars()[0].tag().c_str());
    ASSERT_STREQ("1", tMaterials[1].deterministicVars()[0].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[0].attribute().c_str());

    ASSERT_EQ(1u, tMaterials[1].randomVars().size());
    ASSERT_EQ(0, tMaterials[1].randomVars()[0].id());
    EXPECT_TRUE(tMaterials[1].randomVars()[0].filename().empty());
    ASSERT_STREQ("poissons_ratio", tMaterials[1].randomVars()[0].tag().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].randomVars()[0].attribute().c_str());
    ASSERT_STREQ("0.05", tMaterials[1].randomVars()[0].deviation().c_str());
    ASSERT_STREQ("beta", tMaterials[1].randomVars()[0].distribution().c_str());
    ASSERT_STREQ("6", tMaterials[1].randomVars()[0].samples().c_str());
    ASSERT_STREQ("0.27", tMaterials[1].randomVars()[0].mean().c_str());
    ASSERT_STREQ("0.2", tMaterials[1].randomVars()[0].lower().c_str());
    ASSERT_STREQ("0.35", tMaterials[1].randomVars()[0].upper().c_str());
}

TEST(PlatoTestXMLGenerator, AppendRandomMaterial_Error)
{
    XMLGen::Material tMaterial;
    Plato::srom::Material tSromMaterial;
    Plato::srom::RandomMatPropMap tEmptyMap;
    EXPECT_THROW(Plato::srom::append_material_properties(tMaterial, tEmptyMap, tSromMaterial), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendRandomMaterial_DeterministicMaterialCase)
{
    // 1. BUILD RANDOM MATERIAL VARIABLE
    XMLGen::Uncertainty tCase1;
    tCase1.id("3");
    tCase1.attribute("homogeneous");
    tCase1.tag("poissons_ratio");
    tCase1.category("material");
    tCase1.distribution("beta");
    tCase1.lower("0.2");
    tCase1.upper("0.35");
    tCase1.mean("0.27");
    tCase1.samples("6");
    tCase1.std("0.05");

    // 2. BUILD RANDOM MATERIAL MAP
    std::vector<XMLGen::Uncertainty> tRandomVars;
    tRandomVars.push_back(tCase1);
    auto tRandomMaterialMap = Plato::srom::build_material_id_to_random_material_map(tRandomVars);

    // 3. BUILD DETERMINISTIC MATERIAL METADATA
    XMLGen::Material tMaterial;
    tMaterial.id("30");
    tMaterial.materialModel("isotropic linear elastic");
    tMaterial.property("youngs_modulus", "0.5");
    tMaterial.property("poissons_ratio", "0.3");

    // 4. APPEND MATERIAL TO SROM MATERIAL METADATA
    Plato::srom::Material tSromMaterial;
    EXPECT_NO_THROW(Plato::srom::append_material_properties(tMaterial, tRandomMaterialMap, tSromMaterial));

    // 5. TEST RESULTS
    ASSERT_FALSE(tSromMaterial.isRandom());
    ASSERT_TRUE(tSromMaterial.isDeterministic());
    ASSERT_TRUE(tSromMaterial.randomVars().empty());
    ASSERT_EQ(2u, tSromMaterial.deterministicVars().size());
    ASSERT_STREQ("30", tSromMaterial.materialID().c_str());
    ASSERT_STREQ("isotropic linear elastic", tSromMaterial.category().c_str());
    ASSERT_STREQ("poissons_ratio", tSromMaterial.tags()[0].c_str());
    ASSERT_STREQ("youngs_modulus", tSromMaterial.tags()[1].c_str());

    ASSERT_STREQ("0.3", tSromMaterial.deterministicVars()[0].value().c_str());
    ASSERT_STREQ("poissons_ratio", tSromMaterial.deterministicVars()[0].tag().c_str());
    ASSERT_STREQ("homogeneous", tSromMaterial.deterministicVars()[0].attribute().c_str());
    ASSERT_STREQ("0.5", tSromMaterial.deterministicVars()[1].value().c_str());
    ASSERT_STREQ("youngs_modulus", tSromMaterial.deterministicVars()[1].tag().c_str());
    ASSERT_STREQ("homogeneous", tSromMaterial.deterministicVars()[1].attribute().c_str());
}

TEST(PlatoTestXMLGenerator, AppendRandomMaterial_RandomMaterialCase)
{
    // 1. BUILD RANDOM MATERIAL MAP
    XMLGen::Uncertainty tCase1;
    tCase1.id("3");
    tCase1.attribute("homogeneous");
    tCase1.tag("poissons_ratio");;
    tCase1.category("material");
    tCase1.distribution("beta");
    tCase1.lower("0.2");
    tCase1.upper("0.35");
    tCase1.mean("0.27");
    tCase1.samples("6");
    tCase1.std("0.05");

    XMLGen::Uncertainty tCase2;
    tCase2.id("30");
    tCase2.attribute("homogeneous");
    tCase2.tag("youngs_modulus");;
    tCase2.category("material");
    tCase2.distribution("beta");
    tCase2.lower("1");
    tCase2.upper("2");
    tCase2.mean("0.2");
    tCase2.samples("8");
    tCase2.std("0.15");

    std::vector<XMLGen::Uncertainty> tRandomVars;
    tRandomVars.push_back(tCase1);
    tRandomVars.push_back(tCase2);
    auto tRandomMaterialMap = Plato::srom::build_material_id_to_random_material_map(tRandomVars);

    // 2. BUILD MATERIAL METADATA
    XMLGen::Material tMaterial;
    tMaterial.id("30");
    tMaterial.materialModel("isotropic linear elastic");
    tMaterial.property("youngs_modulus", "0.5");
    tMaterial.property("poissons_ratio", "0.3");

    // 3. APPEND MATERIAL TO SROM MATERIAL METADATA
    Plato::srom::Material tSromMaterial;
    EXPECT_NO_THROW(Plato::srom::append_material_properties(tMaterial, tRandomMaterialMap, tSromMaterial));

    // 4. TEST RESULTS
    ASSERT_TRUE(tSromMaterial.isRandom());
    ASSERT_FALSE(tSromMaterial.isDeterministic());
    ASSERT_EQ(1u, tSromMaterial.randomVars().size());
    ASSERT_EQ(1u, tSromMaterial.deterministicVars().size());
    ASSERT_STREQ("30", tSromMaterial.materialID().c_str());
    ASSERT_STREQ("isotropic linear elastic", tSromMaterial.category().c_str());
    ASSERT_STREQ("youngs_modulus", tSromMaterial.tags()[0].c_str());
    ASSERT_STREQ("poissons_ratio", tSromMaterial.tags()[1].c_str());

    ASSERT_STREQ("0.3", tSromMaterial.deterministicVars()[0].value().c_str());
    ASSERT_STREQ("poissons_ratio", tSromMaterial.deterministicVars()[0].tag().c_str());
    ASSERT_STREQ("homogeneous", tSromMaterial.deterministicVars()[0].attribute().c_str());

    ASSERT_EQ(0, tSromMaterial.randomVars()[0].id());
    ASSERT_TRUE(tSromMaterial.randomVars()[0].filename().empty());
    ASSERT_STREQ("0.2", tSromMaterial.randomVars()[0].mean().c_str());
    ASSERT_STREQ("2", tSromMaterial.randomVars()[0].upper().c_str());
    ASSERT_STREQ("1", tSromMaterial.randomVars()[0].lower().c_str());
    ASSERT_STREQ("8", tSromMaterial.randomVars()[0].samples().c_str());
    ASSERT_STREQ("0.15", tSromMaterial.randomVars()[0].deviation().c_str());
    ASSERT_STREQ("beta", tSromMaterial.randomVars()[0].distribution().c_str());
    ASSERT_STREQ("youngs_modulus", tSromMaterial.randomVars()[0].tag().c_str());
    ASSERT_STREQ("homogeneous", tSromMaterial.randomVars()[0].attribute().c_str());
}

TEST(PlatoTestXMLGenerator, BuildMaterialIdToRandomMaterialMap_Error)
{
    XMLGen::Uncertainty tCase1;
    tCase1.id("3");
    tCase1.attribute("homogeneous");
    tCase1.tag("poissons_ratio");;
    tCase1.category("material");
    tCase1.distribution("beta");
    tCase1.lower("0.2");
    tCase1.upper("0.35");
    tCase1.mean("0.27");
    tCase1.samples("6");
    tCase1.std("0.05");

    XMLGen::Uncertainty tCase2;
    tCase2.category("load");
    tCase2.attribute("y");
    tCase2.distribution("beta");
    tCase2.id("20");
    tCase2.lower("-20");
    tCase2.upper("20");
    tCase2.mean("2");
    tCase2.std("2");
    tCase2.samples("8");
    tCase2.tag("angle_variation");;

    std::vector<XMLGen::Uncertainty> tRandomVars;
    tRandomVars.push_back(tCase1);
    tRandomVars.push_back(tCase2);

    EXPECT_THROW(Plato::srom::build_material_id_to_random_material_map(tRandomVars), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, BuildMaterialIdToRandomMaterialMap)
{
    XMLGen::Uncertainty tCase1;
    tCase1.id("3");
    tCase1.attribute("homogeneous");
    tCase1.tag("poissons_ratio");;
    tCase1.category("material");
    tCase1.distribution("beta");
    tCase1.lower("0.2");
    tCase1.upper("0.35");
    tCase1.mean("0.27");
    tCase1.samples("6");
    tCase1.std("0.05");

    XMLGen::Uncertainty tCase2;
    tCase2.id("30");
    tCase2.attribute("homogeneous");
    tCase2.tag("youngs_modulus");;
    tCase2.category("material");
    tCase2.distribution("beta");
    tCase2.lower("1");
    tCase2.upper("2");
    tCase2.mean("0.2");
    tCase2.samples("8");
    tCase2.std("0.15");

    std::vector<XMLGen::Uncertainty> tRandomVars;
    tRandomVars.push_back(tCase1);
    tRandomVars.push_back(tCase2);

    auto tRandomVarMap = Plato::srom::build_material_id_to_random_material_map(tRandomVars);
    ASSERT_EQ(2u, tRandomVarMap.size());
    std::vector<std::string> tGoldIDs = {"3", "30"};
    std::vector<std::string> tGoldTags = {"poissons_ratio", "youngs_modulus"};
    auto tGoldIdIterator = tGoldIDs.begin();
    auto tGoldTagIterator = tGoldTags.begin();
    for(auto& tRandomVar : tRandomVarMap)
    {
        ASSERT_STREQ(tGoldIdIterator->c_str(), tRandomVar.first.c_str());
        ASSERT_STREQ(tGoldTagIterator->c_str(), tRandomVar.second.begin()->first.c_str());
        std::advance(tGoldIdIterator, 1);
        std::advance(tGoldTagIterator, 1);
    }

    // TEST MATERIAL 1
    ASSERT_TRUE(tRandomVarMap.find("3")->second.find("poissons_ratio")->second.filename().empty());
    ASSERT_STREQ("3", tRandomVarMap.find("3")->second.find("poissons_ratio")->second.id().c_str());
    ASSERT_STREQ("0.27", tRandomVarMap.find("3")->second.find("poissons_ratio")->second.mean().c_str());
    ASSERT_STREQ("0.2", tRandomVarMap.find("3")->second.find("poissons_ratio")->second.lower().c_str());
    ASSERT_STREQ("0.35", tRandomVarMap.find("3")->second.find("poissons_ratio")->second.upper().c_str());
    ASSERT_STREQ("6", tRandomVarMap.find("3")->second.find("poissons_ratio")->second.samples().c_str());
    ASSERT_STREQ("homogeneous", tRandomVarMap.find("3")->second.find("poissons_ratio")->second.attribute().c_str());
    ASSERT_STREQ("beta", tRandomVarMap.find("3")->second.find("poissons_ratio")->second.distribution().c_str());
    ASSERT_STREQ("material", tRandomVarMap.find("3")->second.find("poissons_ratio")->second.category().c_str());
    ASSERT_STREQ("0.05", tRandomVarMap.find("3")->second.find("poissons_ratio")->second.std().c_str());

    // TEST MATERIAL 2
    ASSERT_TRUE(tRandomVarMap.find("30")->second.find("youngs_modulus")->second.filename().empty());
    ASSERT_STREQ("30", tRandomVarMap.find("30")->second.find("youngs_modulus")->second.id().c_str());
    ASSERT_STREQ("0.2", tRandomVarMap.find("30")->second.find("youngs_modulus")->second.mean().c_str());
    ASSERT_STREQ("1", tRandomVarMap.find("30")->second.find("youngs_modulus")->second.lower().c_str());
    ASSERT_STREQ("2", tRandomVarMap.find("30")->second.find("youngs_modulus")->second.upper().c_str());
    ASSERT_STREQ("8", tRandomVarMap.find("30")->second.find("youngs_modulus")->second.samples().c_str());
    ASSERT_STREQ("homogeneous", tRandomVarMap.find("30")->second.find("youngs_modulus")->second.attribute().c_str());
    ASSERT_STREQ("beta", tRandomVarMap.find("30")->second.find("youngs_modulus")->second.distribution().c_str());
    ASSERT_STREQ("material", tRandomVarMap.find("30")->second.find("youngs_modulus")->second.category().c_str());
    ASSERT_STREQ("0.15", tRandomVarMap.find("30")->second.find("youngs_modulus")->second.std().c_str());
}

TEST(PlatoTestXMLGenerator, SplitUncertaintiesIntoCategories_Error1)
{
    // EMPTY UNCERTAINTY LIST
    XMLGen::InputData tMetadata;
    EXPECT_THROW(Plato::srom::split_uncertainties_into_categories(tMetadata), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, SplitUncertaintiesIntoCategories_Error2)
{
    // UNSUPPORTED CATEGORY, I.E. USE CASE
    XMLGen::InputData tMetadata;
    tMetadata.uncertainties.push_back(XMLGen::Uncertainty());
    tMetadata.uncertainties[0].category("boundary condition");
    EXPECT_THROW(Plato::srom::split_uncertainties_into_categories(tMetadata), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, SplitUncertaintiesIntoCategories)
{
    // 1. POSE DATA
    XMLGen::Uncertainty tCase1;
    tCase1.category("load");
    tCase1.attribute("x");
    tCase1.distribution("beta");
    tCase1.id("2");
    tCase1.lower("-2");
    tCase1.upper("2");
    tCase1.mean("0");
    tCase1.std("0.2");
    tCase1.samples("12");
    tCase1.tag("angle_variation");;

    XMLGen::Uncertainty tCase2;
    tCase2.category("load");
    tCase2.attribute("y");
    tCase2.distribution("beta");
    tCase2.id("20");
    tCase2.lower("-20");
    tCase2.upper("20");
    tCase2.mean("2");
    tCase2.std("2");
    tCase2.samples("8");
    tCase2.tag("angle_variation");;

    XMLGen::Uncertainty tCase3;
    tCase3.category("material");
    tCase3.attribute("homogeneous");
    tCase3.distribution("beta");
    tCase3.id("3");
    tCase3.lower("0.2");
    tCase3.upper("0.35");
    tCase3.mean("0.27");
    tCase3.std("0.05");
    tCase3.samples("6");
    tCase3.tag("poissons_ratio");;

    XMLGen::InputData tMetadata;
    tMetadata.uncertainties.push_back(tCase1);
    tMetadata.uncertainties.push_back(tCase2);
    tMetadata.uncertainties.push_back(tCase3);

    auto tCategoriesMap = Plato::srom::split_uncertainties_into_categories(tMetadata);
    ASSERT_EQ(2u, tCategoriesMap.size());

    // 2.1 TEST LOADS
    auto tLoads = tCategoriesMap.find(Plato::srom::category::LOAD);
    ASSERT_EQ(2u, tLoads->second.size());
    ASSERT_STREQ("load", tLoads->second[0].category().c_str());
    ASSERT_STREQ("angle_variation", tLoads->second[0].tag().c_str());
    ASSERT_STREQ("x", tLoads->second[0].attribute().c_str());
    ASSERT_STREQ("2", tLoads->second[0].id().c_str());
    ASSERT_STREQ("2", tLoads->second[0].upper().c_str());
    ASSERT_STREQ("-2", tLoads->second[0].lower().c_str());
    ASSERT_STREQ("0", tLoads->second[0].mean().c_str());
    ASSERT_STREQ("0.2", tLoads->second[0].std().c_str());
    ASSERT_STREQ("12", tLoads->second[0].samples().c_str());
    ASSERT_STREQ("beta", tLoads->second[0].distribution().c_str());

    ASSERT_STREQ("load", tLoads->second[1].category().c_str());
    ASSERT_STREQ("angle_variation", tLoads->second[1].tag().c_str());
    ASSERT_STREQ("y", tLoads->second[1].attribute().c_str());
    ASSERT_STREQ("20", tLoads->second[1].id().c_str());
    ASSERT_STREQ("20", tLoads->second[1].upper().c_str());
    ASSERT_STREQ("-20", tLoads->second[1].lower().c_str());
    ASSERT_STREQ("2", tLoads->second[1].mean().c_str());
    ASSERT_STREQ("2", tLoads->second[1].std().c_str());
    ASSERT_STREQ("8", tLoads->second[1].samples().c_str());
    ASSERT_STREQ("beta", tLoads->second[1].distribution().c_str());

    // 2.2 TEST MATERIALS
    auto tMaterials = tCategoriesMap.find(Plato::srom::category::MATERIAL);
    ASSERT_EQ(1u, tMaterials->second.size());
    ASSERT_STREQ("material", tMaterials->second[0].category().c_str());
    ASSERT_STREQ("poissons_ratio", tMaterials->second[0].tag().c_str());
    ASSERT_STREQ("homogeneous", tMaterials->second[0].attribute().c_str());
    ASSERT_STREQ("3", tMaterials->second[0].id().c_str());
    ASSERT_STREQ("0.35", tMaterials->second[0].upper().c_str());
    ASSERT_STREQ("0.2", tMaterials->second[0].lower().c_str());
    ASSERT_STREQ("0.27", tMaterials->second[0].mean().c_str());
    ASSERT_STREQ("0.05", tMaterials->second[0].std().c_str());
    ASSERT_STREQ("6", tMaterials->second[0].samples().c_str());
    ASSERT_STREQ("beta", tMaterials->second[0].distribution().c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessNondeterministicLoadInputs_Error)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
      "begin service 1\n"
      "  code platomain\n"
      "  number_processors 1\n"
      "end service\n"
      "begin service 2\n"
      "  code plato_analyze\n"
      "  number_processors 1\n"
      "end service\n"
      "begin criterion 1\n"
      "  type mechanical_compliance\n"
      "end criterion\n"
      "begin scenario 1\n"
      "  physics steady_state_mechanics\n"
      "  dimensions 3\n"
      "  loads 10\n"
      "  boundary_conditions 1 2 3\n"
      "  material 1\n"
      "end scenario\n"
      "begin objective\n"
      "  scenarios 1\n"
      "  criteria 1\n"
      "  services 2\n"
      "  type weighted_sum\n"
      "  weights 1\n"
      "end objective\n"
      "begin boundary_condition 1\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispx\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 2\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispy\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 3\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispz\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin load 10\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_2\n"
      "  value 0 -5e4 0\n"
      "end load\n"
      "begin block 1\n"
      "  material 1\n"
      "end block\n"
      "begin material 1\n"
      "  material_model isotropic_linear_elastic\n"
      "  poissons_ratio 0.33\n"
      "  youngs_modulus 1e6\n"
      "end material\n"
      "begin optimization parameters\n"
      "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tInputSS));

    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    auto tInputData = tTester.getInputData();

    Plato::srom::InputMetaData tSromInputs;
    EXPECT_THROW(Plato::srom::preprocess_load_inputs(tInputData, tSromInputs), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, PreprocessNondeterministicLoadInputs)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
      "begin service 1\n"
      "  code platomain\n"
      "  number_processors 1\n"
      "end service\n"
      "begin service 2\n"
      "  code plato_analyze\n"
      "  number_processors 1\n"
      "end service\n"
      "begin criterion 1\n"
      "  type mechanical_compliance\n"
      "end criterion\n"
      "begin scenario 1\n"
      "  physics steady_state_mechanics\n"
      "  dimensions 3\n"
      "  loads 10\n"
      "  boundary_conditions 1 2 3\n"
      "  material 1\n"
      "end scenario\n"
      "begin objective\n"
      "  scenarios 1\n"
      "  criteria 1\n"
      "  services 2\n"
      "  type weighted_sum\n"
      "  weights 1\n"
      "end objective\n"
      "begin boundary_condition 1\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispx\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 2\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispy\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 3\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispz\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin load 10\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_2\n"
      "  value 0 -5e4 0\n"
      "end load\n"
      "begin block 1\n"
      "  material 1\n"
      "end block\n"
      "begin material 1\n"
      "  material_model isotropic_linear_elastic\n"
      "  poissons_ratio 0.33\n"
      "  youngs_modulus 1e6\n"
      "end material\n"
      "begin uncertainty\n"
      "  category load\n"
      "  tag angle_variation\n"
      "  load_id 10\n"
      "  attribute X\n"
      "  distribution beta\n"
      "  mean 0.0\n"
      "  upper_bound 45.0\n"
      "  lower_bound -45.0\n"
      "  standard_deviation 22.5\n"
      "  number_samples 2\n"
      "end uncertainty\n"
      "begin optimization parameters\n"
      "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tInputSS));

    auto tInputData = tTester.getInputData();

    Plato::srom::InputMetaData tSromInputs;
    Plato::srom::preprocess_load_inputs(tInputData, tSromInputs);

    // 1 TEST RESULTS
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_FALSE(tLoads.empty());
    ASSERT_TRUE(tMaterials.empty());

    // 1.1. TEST INTEGERS
    ASSERT_EQ(1u, tLoads.size());
    ASSERT_EQ(2147483647, tLoads[0].mAppID);
    ASSERT_STREQ("ss_2", tLoads[0].mAppName.c_str());

    // 1.2. TEST STRINGS
    ASSERT_STREQ("ss_2", tLoads[0].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[0].mAppType.c_str());
    ASSERT_STREQ("10", tLoads[0].mLoadID.c_str());
    ASSERT_STREQ("traction", tLoads[0].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[0].c_str());
    ASSERT_STREQ("-5e4", tLoads[0].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[2].c_str());

    // 1.3. TEST STATISTICS
    ASSERT_FALSE(tLoads[0].mRandomVars.empty());
    ASSERT_EQ(1u, tLoads[0].mRandomVars.size());
    ASSERT_EQ(0, tLoads[0].mRandomVars[0].id());

    ASSERT_STREQ("angle_variation", tLoads[0].mRandomVars[0].tag().c_str());
    ASSERT_STREQ("x", tLoads[0].mRandomVars[0].attribute().c_str());
    ASSERT_STREQ("22.5", tLoads[0].mRandomVars[0].deviation().c_str());
    ASSERT_STREQ("beta", tLoads[0].mRandomVars[0].distribution().c_str());
    ASSERT_STREQ("-45.0", tLoads[0].mRandomVars[0].lower().c_str());
    ASSERT_STREQ("45.0", tLoads[0].mRandomVars[0].upper().c_str());
    ASSERT_STREQ("0.0", tLoads[0].mRandomVars[0].mean().c_str());
    ASSERT_STREQ("2", tLoads[0].mRandomVars[0].samples().c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_Error)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
      "begin service 1\n"
      "  code platomain\n"
      "  number_processors 1\n"
      "end service\n"
      "begin service 2\n"
      "  code plato_analyze\n"
      "  number_processors 1\n"
      "end service\n"
      "begin criterion 1\n"
      "  type mechanical_compliance\n"
      "end criterion\n"
      "begin scenario 1\n"
      "  physics steady_state_mechanics\n"
      "  dimensions 3\n"
      "  loads 10\n"
      "  boundary_conditions 1 2 3\n"
      "  material 1\n"
      "end scenario\n"
      "begin objective\n"
      "  scenarios 1\n"
      "  criteria 1\n"
      "  services 2\n"
      "  type weighted_sum\n"
      "  weights 1\n"
      "end objective\n"
      "begin boundary_condition 1\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispx\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 2\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispy\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 3\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispz\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin load 10\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_2\n"
      "  value 0 -5e4 0\n"
      "end load\n"
      "begin block 1\n"
      "  material 1\n"
      "end block\n"
      "begin material 1\n"
      "  material_model isotropic_linear_elastic\n"
      "  poissons_ratio 0.33\n"
      "  youngs_modulus 1e6\n"
      "end material\n"
      "begin uncertainty\n"
      "  category load\n"
      "  tag angle_variation\n"
      "  load_id 10\n"
      "  attribute X\n"
      "  distribution beta\n"
      "  mean 0.0\n"
      "  upper_bound 45.0\n"
      "  lower_bound -45.0\n"
      "  standard_deviation 22.5\n"
      "  number_samples 2\n"
      "end uncertainty\n"
      "begin optimization parameters\n"
      "end optimization parameters\n";
    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tInputSS));

    auto tInputData = tTester.getInputData();

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    tSromInputs.usecase(Plato::srom::usecase::UNDEFINED);
    EXPECT_THROW(Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_Loads)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
      "begin service 1\n"
      "  code platomain\n"
      "  number_processors 1\n"
      "end service\n"
      "begin service 2\n"
      "  code plato_analyze\n"
      "  number_processors 1\n"
      "end service\n"
      "begin criterion 1\n"
      "  type mechanical_compliance\n"
      "end criterion\n"
      "begin scenario 1\n"
      "  physics steady_state_mechanics\n"
      "  dimensions 3\n"
      "  loads 10\n"
      "  boundary_conditions 1 2 3\n"
      "  material 1\n"
      "end scenario\n"
      "begin objective\n"
      "  scenarios 1\n"
      "  criteria 1\n"
      "  services 2\n"
      "  type weighted_sum\n"
      "  weights 1\n"
      "end objective\n"
      "begin boundary_condition 1\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispx\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 2\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispy\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 3\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispz\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin load 10\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_2\n"
      "  value 0 -5e4 0\n"
      "end load\n"
      "begin block 1\n"
      "  material 1\n"
      "end block\n"
      "begin material 1\n"
      "  material_model isotropic_linear_elastic\n"
      "  poissons_ratio 0.33\n"
      "  youngs_modulus 1e6\n"
      "end material\n"
      "begin uncertainty\n"
      "  category load\n"
      "  tag angle_variation\n"
      "  load_id 10\n"
      "  attribute X\n"
      "  distribution beta\n"
      "  mean 0.0\n"
      "  upper_bound 45.0\n"
      "  lower_bound -45.0\n"
      "  standard_deviation 22.5\n"
      "  number_samples 2\n"
      "end uncertainty\n"
      "begin optimization parameters\n"
      "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tInputSS));
    auto tInputData = tTester.getInputData();

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    tSromInputs.usecase(Plato::srom::usecase::LOAD);
    Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs);

    // 1 TEST RESULTS
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_FALSE(tLoads.empty());
    ASSERT_TRUE(tMaterials.empty());

    // 1.1. TEST INTEGERS
    ASSERT_EQ(1u, tLoads.size());
    ASSERT_EQ(2147483647, tLoads[0].mAppID); // not defined, thus it takes the value std::numeric_limit::max
    ASSERT_STREQ("ss_2", tLoads[0].mAppName.c_str());

    // 1.2. TEST STRINGS
    ASSERT_STREQ("ss_2", tLoads[0].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[0].mAppType.c_str());
    ASSERT_STREQ("10", tLoads[0].mLoadID.c_str());
    ASSERT_STREQ("traction", tLoads[0].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[0].c_str());
    ASSERT_STREQ("-5e4", tLoads[0].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[2].c_str());

    // 1.3. TEST STATISTICS
    ASSERT_FALSE(tLoads[0].mRandomVars.empty());
    ASSERT_EQ(1u, tLoads[0].mRandomVars.size());
    ASSERT_EQ(0, tLoads[0].mRandomVars[0].id());

    ASSERT_STREQ("angle_variation", tLoads[0].mRandomVars[0].tag().c_str());
    ASSERT_STREQ("x", tLoads[0].mRandomVars[0].attribute().c_str());
    ASSERT_STREQ("22.5", tLoads[0].mRandomVars[0].deviation().c_str());
    ASSERT_STREQ("beta", tLoads[0].mRandomVars[0].distribution().c_str());
    ASSERT_STREQ("-45.0", tLoads[0].mRandomVars[0].lower().c_str());
    ASSERT_STREQ("45.0", tLoads[0].mRandomVars[0].upper().c_str());
    ASSERT_STREQ("0.0", tLoads[0].mRandomVars[0].mean().c_str());
    ASSERT_STREQ("2", tLoads[0].mRandomVars[0].samples().c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_Materials)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
      "begin service 1\n"
      "  code platomain\n"
      "  number_processors 1\n"
      "end service\n"
      "begin service 2\n"
      "  code plato_analyze\n"
      "  number_processors 1\n"
      "end service\n"
      "begin criterion 1\n"
      "  type mechanical_compliance\n"
      "end criterion\n"
      "begin scenario 1\n"
      "  physics steady_state_mechanics\n"
      "  dimensions 3\n"
      "  loads 10\n"
      "  boundary_conditions 1 2 3\n"
      "  material 1\n"
      "end scenario\n"
      "begin objective\n"
      "  scenarios 1\n"
      "  criteria 1\n"
      "  services 2\n"
      "  type weighted_sum\n"
      "  weights 1\n"
      "end objective\n"
      "begin boundary_condition 1\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispx\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 2\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispy\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 3\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispz\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin load 10\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_2\n"
      "  value 0 -5e4 0\n"
      "end load\n"
      "begin block 1\n"
      "  material 1\n"
      "end block\n"
      "begin material 1\n"
      "  penalty_exponent 3\n"
      "  material_model isotropic_linear_thermoelastic\n"
      "  poissons_ratio 0.33\n"
      "  youngs_modulus 1e6\n"
      "  thermal_conductivity .02\n"
      "  mass_density .001\n"
      "end material\n"
      "begin uncertainty\n"
      "  category material\n"
      "  tag thermal_conductivity\n"
      "  material_id 1\n"
      "  attribute homogeneous\n"
      "  distribution beta\n"
      "  mean 0.02\n"
      "  upper_bound 0.04\n"
      "  lower_bound 0.01\n"
      "  standard_deviation 0.0075\n"
      "  number_samples 8\n"
      "end uncertainty\n"
      "begin optimization parameters\n"
      "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMaterials(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tInputSS));
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    auto tInputData = tTester.getInputData();
    EXPECT_FALSE(tInputData.uncertainties.empty());
    tSromInputs.usecase(Plato::srom::usecase::MATERIAL);
    Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs);

    // 1 TEST RESULTS
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_TRUE(tLoads.empty());
    ASSERT_FALSE(tMaterials.empty());

    // 1.1. TEST INTEGERS
    ASSERT_EQ(1u, tMaterials.size());
    EXPECT_TRUE(tMaterials[0].isRandom());
    EXPECT_FALSE(tMaterials[0].isDeterministic());

    // 1.2. TEST STRINGS
    ASSERT_STREQ("1", tMaterials[0].blockID().c_str());
    ASSERT_STREQ("1", tMaterials[0].materialID().c_str());
    ASSERT_STREQ("isotropic_linear_thermoelastic", tMaterials[0].category().c_str());
    ASSERT_EQ(5u, tMaterials[0].tags().size());

    // 1.3. TEST STATISTICS
    ASSERT_FALSE(tMaterials[0].randomVars().empty());
    ASSERT_FALSE(tMaterials[0].deterministicVars().empty());
    ASSERT_EQ(1u, tMaterials[0].randomVars().size());
    ASSERT_EQ(4u, tMaterials[0].deterministicVars().size());
    ASSERT_EQ(0, tMaterials[0].randomVars()[0].id());

    ASSERT_STREQ("8", tMaterials[0].randomVars()[0].samples().c_str());
    ASSERT_STREQ("0.02", tMaterials[0].randomVars()[0].mean().c_str());
    ASSERT_STREQ("0.01", tMaterials[0].randomVars()[0].lower().c_str());
    ASSERT_STREQ("0.04", tMaterials[0].randomVars()[0].upper().c_str());
    ASSERT_STREQ("0.0075", tMaterials[0].randomVars()[0].deviation().c_str());
    ASSERT_STREQ("beta", tMaterials[0].randomVars()[0].distribution().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[0].randomVars()[0].attribute().c_str());
    ASSERT_STREQ("thermal_conductivity", tMaterials[0].randomVars()[0].tag().c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_Materials_1RandomAnd1Deterministic)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
      "begin service 1\n"
      "  code platomain\n"
      "  number_processors 1\n"
      "end service\n"
      "begin service 2\n"
      "  code plato_analyze\n"
      "  number_processors 1\n"
      "end service\n"
      "begin criterion 1\n"
      "  type mechanical_compliance\n"
      "end criterion\n"
      "begin scenario 1\n"
      "  physics steady_state_mechanics\n"
      "  dimensions 3\n"
      "  loads 10\n"
      "  boundary_conditions 1 2 3\n"
      "  material 1\n"
      "end scenario\n"
      "begin objective\n"
      "  scenarios 1\n"
      "  criteria 1\n"
      "  services 2\n"
      "  type weighted_sum\n"
      "  weights 1\n"
      "end objective\n"
      "begin boundary_condition 1\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispx\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 2\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispy\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 3\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispz\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin load 10\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_2\n"
      "  value 0 -5e4 0\n"
      "end load\n"
      "begin block 1\n"
      "  material 1\n"
      "end block\n"
      "begin block 2\n"
      "  material 2\n"
      "end block\n"
      "begin material 1\n"
      "  material_model isotropic_linear_thermoelastic\n"
      "  poissons_ratio 0.33\n"
      "  youngs_modulus 1e6\n"
      "  thermal_conductivity .02\n"
      "  mass_density .001\n"
      "  penalty_exponent 3\n"
      "end material\n"
      "begin material 2\n"
      "  material_model isotropic_linear_elastic\n"
      "  penalty_exponent 3\n"
      "  poissons_ratio 0.28\n"
      "  youngs_modulus 5e6\n"
      "end material\n"
      "begin uncertainty\n"
      "  category material\n"
      "  tag thermal_conductivity\n"
      "  material_id 1\n"
      "  attribute homogeneous\n"
      "  distribution beta\n"
      "  mean 0.02\n"
      "  upper_bound 0.04\n"
      "  lower_bound 0.01\n"
      "  standard_deviation 0.0075\n"
      "  number_samples 8\n"
      "end uncertainty\n"
      "begin optimization parameters\n"
      "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMaterials(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tInputSS));

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    auto tInputData = tTester.getInputData();
    EXPECT_FALSE(tInputData.uncertainties.empty());
    tSromInputs.usecase(Plato::srom::usecase::MATERIAL);
    Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs);

    // 1 TEST RESULTS
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_TRUE(tLoads.empty());
    ASSERT_FALSE(tMaterials.empty());

    // 1.1. TEST INTEGERS
    ASSERT_EQ(2u, tMaterials.size());
    EXPECT_TRUE(tMaterials[0].isRandom());
    EXPECT_FALSE(tMaterials[0].isDeterministic());

    // 1.2. TEST STRINGS
    ASSERT_STREQ("1", tMaterials[0].blockID().c_str());
    ASSERT_STREQ("isotropic_linear_thermoelastic", tMaterials[0].category().c_str());
    ASSERT_STREQ("1", tMaterials[0].materialID().c_str());
    ASSERT_EQ(5u, tMaterials[0].tags().size());

    // 1.3. TEST STATISTICS
    ASSERT_FALSE(tMaterials[0].randomVars().empty());
    ASSERT_FALSE(tMaterials[0].deterministicVars().empty());
    ASSERT_EQ(1u, tMaterials[0].randomVars().size());
    ASSERT_EQ(4u, tMaterials[0].deterministicVars().size());
    ASSERT_EQ(0, tMaterials[0].randomVars()[0].id());

    ASSERT_STREQ("8", tMaterials[0].randomVars()[0].samples().c_str());
    ASSERT_STREQ("0.02", tMaterials[0].randomVars()[0].mean().c_str());
    ASSERT_STREQ("0.01", tMaterials[0].randomVars()[0].lower().c_str());
    ASSERT_STREQ("0.04", tMaterials[0].randomVars()[0].upper().c_str());
    ASSERT_STREQ("0.0075", tMaterials[0].randomVars()[0].deviation().c_str());
    ASSERT_STREQ("beta", tMaterials[0].randomVars()[0].distribution().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[0].randomVars()[0].attribute().c_str());
    ASSERT_STREQ("thermal_conductivity", tMaterials[0].randomVars()[0].tag().c_str());

    // 2.1. TEST DETERMINISTIC MATERIAL
    EXPECT_FALSE(tMaterials[1].isRandom());
    EXPECT_TRUE(tMaterials[1].isDeterministic());

    // 2.2. TEST STRINGS
    ASSERT_STREQ("2", tMaterials[1].blockID().c_str());
    ASSERT_STREQ("isotropic_linear_elastic", tMaterials[1].category().c_str());
    ASSERT_STREQ("2", tMaterials[1].materialID().c_str());
    ASSERT_EQ(3u, tMaterials[1].tags().size());

    ASSERT_STREQ("3", tMaterials[1].deterministicVars()[0].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[0].attribute().c_str());
    ASSERT_STREQ("penalty_exponent", tMaterials[1].deterministicVars()[0].tag().c_str());

    ASSERT_STREQ("0.28", tMaterials[1].deterministicVars()[1].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[1].attribute().c_str());
    ASSERT_STREQ("poissons_ratio", tMaterials[1].deterministicVars()[1].tag().c_str());

    ASSERT_STREQ("5e6", tMaterials[1].deterministicVars()[2].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[2].attribute().c_str());
    ASSERT_STREQ("youngs_modulus", tMaterials[1].deterministicVars()[2].tag().c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_MaterialsPlusLoad)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
      "begin service 1\n"
      "  code platomain\n"
      "  number_processors 1\n"
      "end service\n"
      "begin service 2\n"
      "  code plato_analyze\n"
      "  number_processors 1\n"
      "end service\n"
      "begin criterion 1\n"
      "  type mechanical_compliance\n"
      "end criterion\n"
      "begin scenario 1\n"
      "  physics steady_state_mechanics\n"
      "  dimensions 3\n"
      "  loads 10\n"
      "  boundary_conditions 1 2 3\n"
      "  material 1\n"
      "end scenario\n"
      "begin objective\n"
      "  scenarios 1\n"
      "  criteria 1\n"
      "  services 2\n"
      "  type weighted_sum\n"
      "  weights 1\n"
      "end objective\n"
      "begin boundary_condition 1\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispx\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 2\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispy\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 3\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispz\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin load 10\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_2\n"
      "  value 0 -5e4 0\n"
      "end load\n"
      "begin block 1\n"
      "  material 1\n"
      "end block\n"
      "begin material 1\n"
      "  material_model isotropic_linear_thermoelastic\n"
      "  poissons_ratio 0.33\n"
      "  youngs_modulus 1e6\n"
      "  thermal_conductivity .02\n"
      "  mass_density .001\n"
      "  penalty_exponent 3\n"
      "end material\n"
      "begin uncertainty\n"
      "  category material\n"
      "  tag thermal_conductivity\n"
      "  material_id 1\n"
      "  attribute homogeneous\n"
      "  distribution beta\n"
      "  mean 0.02\n"
      "  upper_bound 0.04\n"
      "  lower_bound 0.01\n"
      "  standard_deviation 0.0075\n"
      "  number_samples 8\n"
      "end uncertainty\n"
      "begin uncertainty\n"
      "  category load\n"
      "  tag angle_variation\n"
      "  load_id 10\n"
      "  attribute X\n"
      "  distribution beta\n"
      "  mean 0.0\n"
      "  upper_bound 45.0\n"
      "  lower_bound -45.0\n"
      "  standard_deviation 22.5\n"
      "  number_samples 2\n"
      "end uncertainty\n"
      "begin optimization parameters\n"
      "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMaterials(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tInputSS));

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    auto tInputData = tTester.getInputData();
    EXPECT_FALSE(tInputData.uncertainties.empty());
    EXPECT_EQ(2u, tInputData.uncertainties.size());
    tSromInputs.usecase(Plato::srom::usecase::MATERIAL_PLUS_LOAD);
    Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs);

    // 1 TEST RESULTS
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_FALSE(tLoads.empty());
    ASSERT_FALSE(tMaterials.empty());

    // 1.1. TEST MATERIAL INTEGERS
    ASSERT_EQ(1u, tMaterials.size());
    EXPECT_TRUE(tMaterials[0].isRandom());
    EXPECT_FALSE(tMaterials[0].isDeterministic());

    // 1.2. TEST MATERIAL STRINGS
    ASSERT_STREQ("isotropic_linear_thermoelastic", tMaterials[0].category().c_str());
    ASSERT_STREQ("1", tMaterials[0].materialID().c_str());
    ASSERT_EQ(5u, tMaterials[0].tags().size());

    // 1.3. TEST MATERIAL STATISTICS
    ASSERT_FALSE(tMaterials[0].randomVars().empty());
    ASSERT_FALSE(tMaterials[0].deterministicVars().empty());
    ASSERT_EQ(1u, tMaterials[0].randomVars().size());
    ASSERT_EQ(4u, tMaterials[0].deterministicVars().size());
    ASSERT_EQ(0, tMaterials[0].randomVars()[0].id());

    ASSERT_STREQ("8", tMaterials[0].randomVars()[0].samples().c_str());
    ASSERT_STREQ("0.02", tMaterials[0].randomVars()[0].mean().c_str());
    ASSERT_STREQ("0.01", tMaterials[0].randomVars()[0].lower().c_str());
    ASSERT_STREQ("0.04", tMaterials[0].randomVars()[0].upper().c_str());
    ASSERT_STREQ("0.0075", tMaterials[0].randomVars()[0].deviation().c_str());
    ASSERT_STREQ("beta", tMaterials[0].randomVars()[0].distribution().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[0].randomVars()[0].attribute().c_str());
    ASSERT_STREQ("thermal_conductivity", tMaterials[0].randomVars()[0].tag().c_str());

    // 2.1. TEST LOAD INTEGERS
    ASSERT_EQ(1u, tLoads.size());
    ASSERT_EQ(2147483647, tLoads[0].mAppID); // not defined, thus it takes the value std::numeric_limit::max
    ASSERT_STREQ("ss_2", tLoads[0].mAppName.c_str());

    // 2.2. TEST LOAD STRINGS
    ASSERT_STREQ("ss_2", tLoads[0].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[0].mAppType.c_str());
    ASSERT_STREQ("10", tLoads[0].mLoadID.c_str());
    ASSERT_STREQ("traction", tLoads[0].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[0].c_str());
    ASSERT_STREQ("-5e4", tLoads[0].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[2].c_str());

    // 2.3. TEST LOAD STATISTICS
    ASSERT_FALSE(tLoads[0].mRandomVars.empty());
    ASSERT_EQ(1u, tLoads[0].mRandomVars.size());
    ASSERT_EQ(0, tLoads[0].mRandomVars[0].id());

    ASSERT_STREQ("angle_variation", tLoads[0].mRandomVars[0].tag().c_str());
    ASSERT_STREQ("x", tLoads[0].mRandomVars[0].attribute().c_str());
    ASSERT_STREQ("22.5", tLoads[0].mRandomVars[0].deviation().c_str());
    ASSERT_STREQ("beta", tLoads[0].mRandomVars[0].distribution().c_str());
    ASSERT_STREQ("-45.0", tLoads[0].mRandomVars[0].lower().c_str());
    ASSERT_STREQ("45.0", tLoads[0].mRandomVars[0].upper().c_str());
    ASSERT_STREQ("0.0", tLoads[0].mRandomVars[0].mean().c_str());
    ASSERT_STREQ("2", tLoads[0].mRandomVars[0].samples().c_str());
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_MaterialsPlusLoad_1RandomAnd1Deterministic)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
      "begin service 1\n"
      "  code platomain\n"
      "  number_processors 1\n"
      "end service\n"
      "begin service 2\n"
      "  code plato_analyze\n"
      "  number_processors 1\n"
      "end service\n"
      "begin criterion 1\n"
      "  type mechanical_compliance\n"
      "end criterion\n"
      "begin scenario 1\n"
      "  physics steady_state_mechanics\n"
      "  dimensions 3\n"
      "  loads 1 10\n"
      "  boundary_conditions 1 2 3\n"
      "  material 1\n"
      "end scenario\n"
      "begin objective\n"
      "  scenarios 1\n"
      "  criteria 1\n"
      "  services 2\n"
      "  type weighted_sum\n"
      "  weights 1\n"
      "end objective\n"
      "begin boundary_condition 1\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispx\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 2\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispy\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 3\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispz\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin load 10\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_2\n"
      "  value 0 -5e4 0\n"
      "end load\n"
      "begin load 1\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_3\n"
      "  value 0 -1e4 0\n"
      "end load\n"
      "begin block 1\n"
      "  material 1\n"
      "end block\n"
      "begin block 2\n"
      "  material 2\n"
      "end block\n"
      "begin material 1\n"
      "  material_model isotropic_linear_thermoelastic\n"
      "  poissons_ratio 0.33\n"
      "  youngs_modulus 1e6\n"
      "  thermal_conductivity .02\n"
      "  mass_density .001\n"
      "  penalty_exponent 3\n"
      "end material\n"
      "begin material 2\n"
      "  material_model isotropic_linear_elastic\n"
      "  poissons_ratio 0.28\n"
      "  youngs_modulus 5e6\n"
      "  penalty_exponent 3\n"
      "end material\n"
      "begin uncertainty\n"
      "  category material\n"
      "  tag thermal_conductivity\n"
      "  material_id 1\n"
      "  attribute homogeneous\n"
      "  distribution beta\n"
      "  mean 0.02\n"
      "  upper_bound 0.04\n"
      "  lower_bound 0.01\n"
      "  standard_deviation 0.0075\n"
      "  number_samples 8\n"
      "end uncertainty\n"
      "begin uncertainty\n"
      "  category load\n"
      "  tag angle_variation\n"
      "  load_id 10\n"
      "  attribute X\n"
      "  distribution beta\n"
      "  mean 0.0\n"
      "  upper_bound 45.0\n"
      "  lower_bound -45.0\n"
      "  standard_deviation 22.5\n"
      "  number_samples 2\n"
      "end uncertainty\n"
      "begin optimization parameters\n"
      "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMaterials(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tInputSS));

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    auto tInputData = tTester.getInputData();
    EXPECT_FALSE(tInputData.uncertainties.empty());
    EXPECT_EQ(2u, tInputData.uncertainties.size());
    tSromInputs.usecase(Plato::srom::usecase::MATERIAL_PLUS_LOAD);
    Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs);

    // 1 TEST RESULTS
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_FALSE(tLoads.empty());
    ASSERT_FALSE(tMaterials.empty());

    // 1.1. TEST MATERIAL INTEGERS
    ASSERT_EQ(2u, tMaterials.size());
    EXPECT_TRUE(tMaterials[0].isRandom());
    EXPECT_FALSE(tMaterials[0].isDeterministic());

    // 1.2. TEST MATERIAL STRINGS
    ASSERT_STREQ("1", tMaterials[0].blockID().c_str());
    ASSERT_STREQ("1", tMaterials[0].materialID().c_str());
    ASSERT_STREQ("isotropic_linear_thermoelastic", tMaterials[0].category().c_str());
    ASSERT_EQ(5u, tMaterials[0].tags().size());

    // 1.3. TEST MATERIAL STATISTICS
    ASSERT_FALSE(tMaterials[0].randomVars().empty());
    ASSERT_FALSE(tMaterials[0].deterministicVars().empty());
    ASSERT_EQ(1u, tMaterials[0].randomVars().size());
    ASSERT_EQ(4u, tMaterials[0].deterministicVars().size());
    ASSERT_EQ(0, tMaterials[0].randomVars()[0].id());

    ASSERT_STREQ("8", tMaterials[0].randomVars()[0].samples().c_str());
    ASSERT_STREQ("0.02", tMaterials[0].randomVars()[0].mean().c_str());
    ASSERT_STREQ("0.01", tMaterials[0].randomVars()[0].lower().c_str());
    ASSERT_STREQ("0.04", tMaterials[0].randomVars()[0].upper().c_str());
    ASSERT_STREQ("0.0075", tMaterials[0].randomVars()[0].deviation().c_str());
    ASSERT_STREQ("beta", tMaterials[0].randomVars()[0].distribution().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[0].randomVars()[0].attribute().c_str());
    ASSERT_STREQ("thermal_conductivity", tMaterials[0].randomVars()[0].tag().c_str());

    // 1.4 TEST DETERMINISTIC MATERIAL
    EXPECT_FALSE(tMaterials[1].isRandom());
    EXPECT_TRUE(tMaterials[1].isDeterministic());

    ASSERT_STREQ("2", tMaterials[1].blockID().c_str());
    ASSERT_STREQ("2", tMaterials[1].materialID().c_str());
    ASSERT_STREQ("isotropic_linear_elastic", tMaterials[1].category().c_str());
    ASSERT_EQ(3u, tMaterials[1].tags().size());

    ASSERT_STREQ("3", tMaterials[1].deterministicVars()[0].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[0].attribute().c_str());
    ASSERT_STREQ("penalty_exponent", tMaterials[1].deterministicVars()[0].tag().c_str());

    ASSERT_STREQ("0.28", tMaterials[1].deterministicVars()[1].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[1].attribute().c_str());
    ASSERT_STREQ("poissons_ratio", tMaterials[1].deterministicVars()[1].tag().c_str());

    ASSERT_STREQ("5e6", tMaterials[1].deterministicVars()[2].value().c_str());
    ASSERT_STREQ("homogeneous", tMaterials[1].deterministicVars()[2].attribute().c_str());
    ASSERT_STREQ("youngs_modulus", tMaterials[1].deterministicVars()[2].tag().c_str());

    // 2.1. TEST LOAD INTEGERS
    ASSERT_EQ(2u, tLoads.size());
    ASSERT_EQ(2147483647, tLoads[0].mAppID); // not defined, thus it takes the value std::numeric_limit::max
    ASSERT_STREQ("ss_2", tLoads[0].mAppName.c_str());

    // 2.2. TEST LOAD STRINGS
    ASSERT_STREQ("ss_2", tLoads[0].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[0].mAppType.c_str());
    ASSERT_STREQ("10", tLoads[0].mLoadID.c_str());
    ASSERT_STREQ("traction", tLoads[0].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[0].c_str());
    ASSERT_STREQ("-5e4", tLoads[0].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[2].c_str());

    // 2.3. TEST LOAD STATISTICS
    ASSERT_FALSE(tLoads[0].mRandomVars.empty());
    ASSERT_EQ(1u, tLoads[0].mRandomVars.size());
    ASSERT_EQ(0, tLoads[0].mRandomVars[0].id());

    ASSERT_STREQ("angle_variation", tLoads[0].mRandomVars[0].tag().c_str());
    ASSERT_STREQ("x", tLoads[0].mRandomVars[0].attribute().c_str());
    ASSERT_STREQ("22.5", tLoads[0].mRandomVars[0].deviation().c_str());
    ASSERT_STREQ("beta", tLoads[0].mRandomVars[0].distribution().c_str());
    ASSERT_STREQ("-45.0", tLoads[0].mRandomVars[0].lower().c_str());
    ASSERT_STREQ("45.0", tLoads[0].mRandomVars[0].upper().c_str());
    ASSERT_STREQ("0.0", tLoads[0].mRandomVars[0].mean().c_str());
    ASSERT_STREQ("2", tLoads[0].mRandomVars[0].samples().c_str());

    // 2.4 TEST DETERMINISTIC LOAD
    ASSERT_STREQ("1", tLoads[1].mLoadID.c_str());
    ASSERT_STREQ("ss_3", tLoads[1].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[1].mAppType.c_str());
    ASSERT_STREQ("traction", tLoads[1].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[1].mValues[0].c_str());
    ASSERT_STREQ("-1e4", tLoads[1].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[1].mValues[2].c_str());
    ASSERT_EQ(std::numeric_limits<int>::max(), tLoads[1].mAppID);
    ASSERT_TRUE(tLoads[1].mRandomVars.empty());
}

TEST(PlatoTestXMLGenerator, PreprocessSromProblemInputs_Loads_1RandomAnd1Deterministic)
{
    // Generate interface.xml using the new writer
    XMLGenerator_UnitTester tTester;
    std::istringstream tInputSS;
    std::string tStringInput =
      "begin service 1\n"
      "  code platomain\n"
      "  number_processors 1\n"
      "end service\n"
      "begin service 2\n"
      "  code plato_analyze\n"
      "  number_processors 1\n"
      "end service\n"
      "begin criterion 1\n"
      "  type mechanical_compliance\n"
      "end criterion\n"
      "begin scenario 1\n"
      "  physics steady_state_mechanics\n"
      "  dimensions 3\n"
      "  loads 1 10\n"
      "  boundary_conditions 1 2 3\n"
      "  material 1\n"
      "end scenario\n"
      "begin objective\n"
      "  scenarios 1\n"
      "  criteria 1\n"
      "  services 2\n"
      "  type weighted_sum\n"
      "  weights 1\n"
      "end objective\n"
      "begin boundary_condition 1\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispx\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 2\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispy\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin boundary_condition 3\n"
      "  type fixed_value\n"
      "  location_type nodeset\n"
      "  location_name ns_1\n"
      "  degree_of_freedom dispz\n"
      "  value 0\n"
      "end boundary_condition\n"
      "begin load 10\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_2\n"
      "  value 0 -5e4 0\n"
      "end load\n"
      "begin load 1\n"
      "  type traction\n"
      "  location_type sideset\n"
      "  location_name ss_3\n"
      "  value 0 -1e4 0\n"
      "end load\n"
      "begin block 1\n"
      "  material 1\n"
      "end block\n"
      "begin material 1\n"
      "  material_model isotropic_linear_elastic\n"
      "  poissons_ratio 0.33\n"
      "  youngs_modulus 1e6\n"
      "  penalty_exponent 3\n"
      "end material\n"
      "begin uncertainty\n"
      "  category load\n"
      "  tag angle_variation\n"
      "  load_id 10\n"
      "  attribute X\n"
      "  distribution beta\n"
      "  mean 0.0\n"
      "  upper_bound 45.0\n"
      "  lower_bound -45.0\n"
      "  standard_deviation 22.5\n"
      "  number_samples 2\n"
      "end uncertainty\n"
      "begin optimization parameters\n"
      "end optimization parameters\n";

    tInputSS.str(tStringInput);
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseObjective(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseServices(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseCriteria(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseMaterials(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseLoads(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseBCs(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseUncertainties(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    EXPECT_TRUE(tTester.publicParseBlocks(tInputSS));
    tInputSS.clear();
    tInputSS.seekg(0);
    ASSERT_NO_THROW(tTester.publicParseScenarios(tInputSS));

    auto tInputData = tTester.getInputData();

    // CALL FUNCTION
    Plato::srom::InputMetaData tSromInputs;
    tSromInputs.usecase(Plato::srom::usecase::LOAD);
    Plato::srom::preprocess_srom_problem_inputs(tInputData, tSromInputs);

    // 1 TEST RANDOM LOAD
    auto tLoads = tSromInputs.loads();
    auto tMaterials = tSromInputs.materials();
    ASSERT_FALSE(tLoads.empty());
    ASSERT_TRUE(tMaterials.empty());

    // 1.1. TEST INTEGERS
    ASSERT_EQ(2u, tLoads.size());
    ASSERT_EQ(std::numeric_limits<int>::max(), tLoads[0].mAppID);
    ASSERT_EQ(std::numeric_limits<int>::max(), tLoads[1].mAppID);
    ASSERT_STREQ("ss_2", tLoads[0].mAppName.c_str());
    ASSERT_STREQ("ss_3", tLoads[1].mAppName.c_str());

    // 1.2. TEST STRINGS - LOAD 1
    ASSERT_STREQ("ss_2", tLoads[0].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[0].mAppType.c_str());
    ASSERT_STREQ("10", tLoads[0].mLoadID.c_str());
    ASSERT_STREQ("traction", tLoads[0].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[0].c_str());
    ASSERT_STREQ("-5e4", tLoads[0].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[0].mValues[2].c_str());

    // 1.3. TEST STATISTICS - LOAD 1
    ASSERT_FALSE(tLoads[0].mRandomVars.empty());
    ASSERT_EQ(1u, tLoads[0].mRandomVars.size());
    ASSERT_EQ(0, tLoads[0].mRandomVars[0].id());

    ASSERT_STREQ("angle_variation", tLoads[0].mRandomVars[0].tag().c_str());
    ASSERT_STREQ("x", tLoads[0].mRandomVars[0].attribute().c_str());
    ASSERT_STREQ("22.5", tLoads[0].mRandomVars[0].deviation().c_str());
    ASSERT_STREQ("beta", tLoads[0].mRandomVars[0].distribution().c_str());
    ASSERT_STREQ("-45.0", tLoads[0].mRandomVars[0].lower().c_str());
    ASSERT_STREQ("45.0", tLoads[0].mRandomVars[0].upper().c_str());
    ASSERT_STREQ("0.0", tLoads[0].mRandomVars[0].mean().c_str());
    ASSERT_STREQ("2", tLoads[0].mRandomVars[0].samples().c_str());

    // 2 TEST DETERMINISTIC LOAD
    ASSERT_STREQ("1", tLoads[1].mLoadID.c_str());
    ASSERT_STREQ("ss_3", tLoads[1].mAppName.c_str());
    ASSERT_STREQ("sideset", tLoads[1].mAppType.c_str());
    ASSERT_STREQ("traction", tLoads[1].mLoadType.c_str());
    ASSERT_STREQ("0", tLoads[1].mValues[0].c_str());
    ASSERT_STREQ("-1e4", tLoads[1].mValues[1].c_str());
    ASSERT_STREQ("0", tLoads[1].mValues[2].c_str());
    ASSERT_EQ(std::numeric_limits<int>::max(), tLoads[1].mAppID);
    ASSERT_TRUE(tLoads[1].mRandomVars.empty());

}

}
// namespace PlatoTestXMLGenerator
