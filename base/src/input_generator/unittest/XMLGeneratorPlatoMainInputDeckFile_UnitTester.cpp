/*
 * XMLGeneratorPlatoMainInputDeckFile_UnitTester.cpp
 *
 *  Created on: Jun 2, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "pugixml.hpp"
#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{

/*
void write_plato_main_input_deck_file
(const XMLGen::InputData& aInputData)
{
    pugi::xml_document doc;
    pugi::xml_node tmp_node1, tmp_node2, tmp_node3;

    // Version entry
    pugi::xml_node tmp_node = doc.append_child(pugi::node_declaration);
    tmp_node.set_name("xml");
    pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
    tmp_att.set_value("1.0");

    // mesh
    pugi::xml_node mesh_node = doc.append_child("mesh");
    addChild(mesh_node, "type", "unstructured");
    addChild(mesh_node, "format", "exodus");

    addChild(mesh_node, "ignore_node_map", "true");

    addChild(mesh_node, "mesh", m_InputData.run_mesh_name.c_str());
    // just need one block specified here
    if(m_InputData.blocks.size() > 0)
    {
        tmp_node = mesh_node.append_child("block");
        addChild(tmp_node, "index", m_InputData.blocks[0].block_id.c_str());
        tmp_node1 = tmp_node.append_child("integration");
        addChild(tmp_node1, "type", "gauss");
        addChild(tmp_node1, "order", "2");
        addChild(tmp_node, "material", m_InputData.blocks[0].material_id.c_str());
    }

    // output
    tmp_node = doc.append_child("output");
    addChild(tmp_node, "file", "platomain");
    addChild(tmp_node, "format", "exodus");

    // Write the file to disk
    doc.save_file("plato_main_input_deck.xml", "  ");
}
*/

void append_mesh_metadata_to_plato_main_input_deck
(const XMLGen::InputData& aInputData,
 pugi::xml_document& aDocument)
{
    if(aInputData.run_mesh_name.empty())
    {
        THROWERR("Append Mesh Metadata To Plato Main Input Deck: Run mesh name in XMLGen::InputData is empty.");
    }

    auto tMesh = aDocument.append_child("mesh");
    std::vector<std::string> tKeys = {"type", "format", "ignore_node_map", "mesh"};
    std::vector<std::string> tValues = {"unstructured", "exodus", "true", aInputData.run_mesh_name};
    XMLGen::append_children(tKeys, tValues, tMesh);
}

void append_block_metadata_to_plato_main_input_deck
(const XMLGen::InputData& aInputData,
 pugi::xml_node& aParentNode)
{
    if(!aInputData.blocks.empty())
    {
        if(aInputData.blocks[0].block_id.empty())
        {
            THROWERR("Append Block Identification Number To Plato Main Input Deck: Block ID in XMLGen::InputData.blocks[0] is empty.")
        }

        if(aInputData.blocks[0].material_id.empty())
        {
            THROWERR("Append Block Identification Number To Plato Main Input Deck: Material ID in XMLGen::InputData.blocks[0] is empty.")
        }

        // note: just need one block specified here
        auto tBlockNode = aParentNode.append_child("block");
        std::vector<std::string> tKeys = {"index", "material"};
        std::vector<std::string> tValues = {aInputData.blocks[0].block_id, aInputData.blocks[0].material_id};
        XMLGen::append_children(tKeys, tValues, tBlockNode);

        auto tIntegration = tBlockNode.append_child("integration");
        tKeys = {"type", "order"}; tValues = {"gauss", "2"};
        XMLGen::append_children(tKeys, tValues, tIntegration);
    }
}

}

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, AppendBlockMetadataToPlatoMainInputDeck_EmptyBlockNode)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::append_block_metadata_to_plato_main_input_deck(tXMLMetaData, tDocument);

    auto tBlock = tDocument.child("block");
    ASSERT_TRUE(tBlock.empty());
}

TEST(PlatoTestXMLGenerator, AppendBlockMetadataToPlatoMainInputDeck_ErrorEmptyBlockID)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Block tBlock;
    tXMLMetaData.blocks.push_back(tBlock);
    ASSERT_THROW(XMLGen::append_block_metadata_to_plato_main_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendBlockMetadataToPlatoMainInputDeck_ErrorEmptyMaterialID)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Block tBlock;
    tBlock.block_id = "1";
    tXMLMetaData.blocks.push_back(tBlock);
    ASSERT_THROW(XMLGen::append_block_metadata_to_plato_main_input_deck(tXMLMetaData, tDocument), std::runtime_error);
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
    ASSERT_NO_THROW(XMLGen::append_block_metadata_to_plato_main_input_deck(tXMLMetaData, tDocument));
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

    tDocument.save_file("dummy.xml", " ");
}

TEST(PlatoTestXMLGenerator, AppendMeshMetadataToPlatoMainInputDeck_ErrorEmptyMeshFile)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    ASSERT_THROW(XMLGen::append_mesh_metadata_to_plato_main_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendMeshMetadataToPlatoMainInputDeck)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.run_mesh_name = "dummy.exo";
    ASSERT_NO_THROW(XMLGen::append_mesh_metadata_to_plato_main_input_deck(tXMLMetaData, tDocument));
    ASSERT_FALSE(tDocument.empty());

    auto tMesh = tDocument.child("mesh");
    ASSERT_FALSE(tMesh.empty());
    ASSERT_STREQ("mesh", tMesh.name());
    std::vector<std::string> tKeys = {"type", "format", "ignore_node_map", "mesh"};
    std::vector<std::string> tValues = {"unstructured", "exodus", "true", "dummy.exo"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tMesh);
}

}
