/*
 * XMLGeneratorPlatoESPInputFileUtilities.cpp
 *
 *  Created on: Jan 27, 2021
 */

#include "pugixml.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorPlatoESPInputFileUtilities.hpp"

namespace XMLGen
{

void write_plato_esp_input_deck_file(const XMLGen::InputData &aInputData)
{
    if(aInputData.optimization_parameters().optimization_type() == "shape")
    {
        pugi::xml_document doc;
        pugi::xml_node tmp_node1 = doc.append_child(pugi::node_declaration);
        tmp_node1.set_name("xml");
        pugi::xml_attribute tmp_att = tmp_node1.append_attribute("version");
        tmp_att.set_value("1.0");
        tmp_node1 = doc.append_child("ESP");
        addChild(tmp_node1, "ModelFileName", aInputData.optimization_parameters().csm_opt_file());
        addChild(tmp_node1, "TessFileName", aInputData.optimization_parameters().csm_tesselation_file());
        addChild(tmp_node1, "ParameterIndex", "0");
        doc.save_file("plato_esp_input_deck.xml", "  ");
    }
}

}
//namespace XMLGen
