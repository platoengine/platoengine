/*
 * XMLGeneratorPlatoMainInputFileUtilities.hpp
 *
 *  Created on: June 2, 2020
 */

#pragma once

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{
  void append_mesh_metadata_to_plato_main_input_deck(const XMLGen::InputData& aInputData, pugi::xml_node& aDocument);
  void append_block_metadata_to_mesh_node(const XMLGen::InputData& aInputData, pugi::xml_node& aNode);
  void append_output_metadata_to_plato_main_input_deck(const XMLGen::InputData& aInputData, pugi::xml_node& aDocument);
  void write_plato_main_input_deck_file(const XMLGen::InputData& aInputData);
}
// namespace XMLGen
