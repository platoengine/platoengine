/*
 * XMLGeneratorPlatoESPOperationsFileUtilities.cpp
 *
 *  Created on: Jan 27, 2021
 */

#include "pugixml.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorPlatoESPOperationsFileUtilities.hpp"

namespace XMLGen
{

void write_plato_esp_operations_file(const XMLGen::InputData &aInputData)
{
    if(aInputData.optimization_parameters().optimizationType() == OT_SHAPE)
    {
        pugi::xml_document doc;
        pugi::xml_node tmp_node, tmp_node1;
    
        tmp_node = doc.append_child(pugi::node_declaration);
        tmp_node.set_name("xml");
        pugi::xml_attribute tmp_att = tmp_node.append_attribute("version");
        tmp_att.set_value("1.0");
        tmp_node = doc.append_child("Timers");
        addChild(tmp_node, "time", "true");
        tmp_node = doc.append_child("Operation");
        addChild(tmp_node, "Name", "Compute Parameter Sensitivity on Change");
        addChild(tmp_node, "Function", "ComputeParameterSensitivity");
        addChild(tmp_node, "OnChange", "true");
        tmp_node1 = tmp_node.append_child("Input");
        addChild(tmp_node1, "ArgumentName", "Parameters");
        tmp_node1 = tmp_node.append_child("Output");
        addChild(tmp_node1, "ArgumentName", "Parameter Sensitivity");
        tmp_node1 = tmp_node.append_child("Parameter");
        addChild(tmp_node1, "ArgumentName", "Parameter Index");
        addChild(tmp_node1, "InitialValue", "0");
        doc.save_file("plato_esp_operations.xml", "  ");
    }
}

}
//namespace XMLGen
