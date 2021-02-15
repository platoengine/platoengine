/*
 * XMLGeneratorPlatoMainInputFileUtilities.cpp
 *
 *  Created on: June 2, 2020
 */

#include "pugixml.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorPlatoMainInputFileUtilities.hpp"

namespace XMLGen
{

bool sierra_sd_service_exists(const XMLGen::InputData &aInputData)
{
    for(auto &tService : aInputData.services())
    {
        if(tService.code() == "sierra_sd")
        {
            return true;
        }
    }
    return false;
}

void append_mesh_metadata_to_plato_main_input_deck
(const XMLGen::InputData &aInputData,
 pugi::xml_node &aDocument)
{
    if (aInputData.mesh.run_name.empty())
    {
        THROWERR("Append Mesh Metadata To Plato Main Input Deck: Run mesh name in XMLGen::InputData is empty.");
    }

    auto tMesh = aDocument.append_child("mesh");
    std::vector<std::string> tKeys = { "type", "format", "ignore_node_map", "ignore_element_map", "mesh" };
    std::string tIgnoreFlag = sierra_sd_service_exists(aInputData) ? "false" : "true";
    std::vector<std::string> tValues = { "unstructured", "exodus", tIgnoreFlag, tIgnoreFlag, aInputData.mesh.run_name };
    XMLGen::append_children(tKeys, tValues, tMesh);
    XMLGen::append_block_metadata_to_mesh_node(aInputData, tMesh);
}

void append_block_metadata_to_mesh_node
(const XMLGen::InputData &aInputData,
 pugi::xml_node &aParentNode)
{
    if (!aInputData.blocks.empty())
    {
        if (aInputData.blocks[0].block_id.empty())
        {
            THROWERR("Append Block Identification Number To Plato Main Input Deck: Block ID in XMLGen::InputData.blocks[0] is empty.")
        }

        if (aInputData.blocks[0].material_id.empty())
        {
            THROWERR("Append Block Identification Number To Plato Main Input Deck: Material ID in XMLGen::InputData.blocks[0] is empty.")
        }

        // note: just need one block specified here
        auto tBlockNode = aParentNode.append_child("block");
        std::vector<std::string> tKeys = { "index", "material" };
        std::vector<std::string> tValues = { aInputData.blocks[0].block_id, aInputData.blocks[0].material_id };
        XMLGen::append_children(tKeys, tValues, tBlockNode);

        auto tIntegration = tBlockNode.append_child("integration");
        tKeys = { "type", "order" };
        tValues = { "gauss", "2" };
        XMLGen::append_children(tKeys, tValues, tIntegration);
    }
}

void append_output_metadata_to_plato_main_input_deck
(const XMLGen::InputData &aInputData,
 pugi::xml_node &aDocument)
{
    auto tOutput = aDocument.append_child("output");
    std::vector<std::string> tKeys = { "file", "format" };
    std::vector<std::string> tValues = { "platomain", "exodus" };
    XMLGen::append_children(tKeys, tValues, tOutput);
}

void write_plato_main_input_deck_file(const XMLGen::InputData &aInputData)
{
    pugi::xml_document tDocument;
    append_version_entry(tDocument);
    append_mesh_metadata_to_plato_main_input_deck(aInputData, tDocument);
    append_output_metadata_to_plato_main_input_deck(aInputData, tDocument);
    tDocument.save_file("plato_main_input_deck.xml", "  ");
}

}
//namespace XMLGen
