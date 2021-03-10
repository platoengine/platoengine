/*
 * XMLGeneratorPlatoMainInputDeckFile_UnitTester.cpp
 *
 *  Created on: June 2, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorPlatoMainInputFileUtilities.hpp"


namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, AppendBlockMetadataToPlatoMainInputDeck_EmptyBlockNode)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::append_block_metadata_to_mesh_node(tXMLMetaData, tDocument);

    auto tBlock = tDocument.child("block");
    ASSERT_TRUE(tBlock.empty());
}

TEST(PlatoTestXMLGenerator, AppendBlockMetadataToPlatoMainInputDeck_ErrorEmptyBlockID)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Block tBlock;
    tXMLMetaData.blocks.push_back(tBlock);
    ASSERT_THROW(XMLGen::append_block_metadata_to_mesh_node(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendBlockMetadataToPlatoMainInputDeck_ErrorEmptyMaterialID)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Block tBlock;
    tBlock.block_id = "1";
    tXMLMetaData.blocks.push_back(tBlock);
    ASSERT_THROW(XMLGen::append_block_metadata_to_mesh_node(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendBlockMetadataToPlatoMainInputDeck)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Block tBlock1;
    tBlock1.block_id = "1";
    tBlock1.material_id = "11";
    tXMLMetaData.blocks.push_back(tBlock1);
    XMLGen::Block tBlock2;
    tBlock2.block_id = "2";
    tBlock2.material_id = "12";
    tXMLMetaData.blocks.push_back(tBlock2);
    ASSERT_NO_THROW(XMLGen::append_block_metadata_to_mesh_node(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tBlock = tDocument.child("block");
    ASSERT_FALSE(tBlock.empty());
    ASSERT_STREQ("block", tBlock.name());
    std::vector<std::string> tKeys = {"index", "material", "integration"};
    std::vector<std::string> tValues = {"1", "11", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tBlock);

    auto tIntegration = tBlock.child("integration");
    ASSERT_FALSE(tIntegration.empty());
    ASSERT_STREQ("integration", tIntegration.name());
    tKeys = {"type", "order"};
    tValues = {"gauss", "2"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tIntegration);
}

TEST(PlatoTestXMLGenerator, AppendMeshMetadataToPlatoMainInputDeck_ErrorEmptyMeshFile)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    ASSERT_THROW(XMLGen::append_mesh_metadata_to_plato_main_input_deck(tXMLMetaData, tDocument), std::runtime_error);
    ASSERT_FALSE(tDocument.empty());
}

TEST(PlatoTestXMLGenerator, AppendMeshMetadataToPlatoMainInputDeck)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mesh.run_name = "dummy.exo";
    ASSERT_NO_THROW(XMLGen::append_mesh_metadata_to_plato_main_input_deck(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tMesh = tDocument.child("mesh");
    ASSERT_FALSE(tMesh.empty());
    ASSERT_STREQ("mesh", tMesh.name());
    std::vector<std::string> tKeys = {"type", "format", "ignore_node_map", "ignore_element_map", "mesh"};
    std::vector<std::string> tValues = {"unstructured", "exodus", "true", "true", "dummy.exo"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMesh);
}

TEST(PlatoTestXMLGenerator, AppendOutputMetadataToPlatoMainInputDeck)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    ASSERT_NO_THROW(XMLGen::append_output_metadata_to_plato_main_input_deck(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tOutput = tDocument.child("output");
    ASSERT_FALSE(tOutput.empty());
    ASSERT_STREQ("output", tOutput.name());
    std::vector<std::string> tKeys = {"file", "format"};
    std::vector<std::string> tValues = {"platomain", "exodus"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOutput);
}

TEST(PlatoTestXMLGenerator, WritePlatoMainInputDeck)
{
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mesh.run_name = "dummy.exo";
    XMLGen::Block tBlock1;
    tBlock1.block_id = "1";
    tBlock1.material_id = "11";
    tXMLMetaData.blocks.push_back(tBlock1);
    XMLGen::Block tBlock2;
    tBlock2.block_id = "2";
    tBlock2.material_id = "12";
    tXMLMetaData.blocks.push_back(tBlock2);
    ASSERT_NO_THROW(XMLGen::write_plato_main_input_deck_file(tXMLMetaData));

    auto tReadData = XMLGen::read_data_from_file("plato_main_input_deck.xml");
    auto tGold =
      std::string("<?xmlversion=\"1.0\"?><mesh><type>unstructured</type><format>exodus</format>") + 
        "<ignore_node_map>true</ignore_node_map><ignore_element_map>true</ignore_element_map><mesh>dummy.exo</mesh><block><index>1</index>" +
        "<material>11</material><integration><type>gauss</type><order>2</order></integration></block></mesh>" +
        "<output><file>platomain</file><format>exodus</format></output>";
    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());
    Plato::system("rm -f plato_main_input_deck.xml");
}

}
