/*
 * XMLGeneratorPlatoMainOperationFile_UnitTester.cpp
 *
 *  Created on: May 28, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorUtilities.hpp"

namespace XMLGen
{

void append_filter_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document &aDocument)
{
    // light input file key to lato main operation XML file key map,
    // i.e. map<light_input_file_key,plato_main_operation_file_key>
    std::unordered_map<std::string, std::string> tValidFilters =
    { {"identity", "Identity"}, {"kernel", "Kernel"}, {"kernel then heaviside", "KernelThenHeaviside"},
      {"kernel then tanh", "KernelThenTANH"} };

    auto tItr = tValidFilters.find(aXMLMetaData.filter_type);
    auto tFilterName = tItr != tValidFilters.end() ? tItr->second : "Kernel";
    auto tFilterNode = aDocument.append_child("Filter");
    XMLGen::append_children({"Name"}, {tFilterName}, tFilterNode);
}

void append_filter_options_to_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    std::vector<std::string> tKeys = {"Scale", "Absolute", "StartIteration", "UpdateInterval",
        "UseAdditiveContinuation", "Power", "HeavisideMin", "HeavisideUpdate", "HeavisideMax"};
    std::vector<std::string> tValues = {aXMLMetaData.filter_radius_scale, aXMLMetaData.filter_radius_absolute,
        aXMLMetaData.filter_projection_start_iteration, aXMLMetaData.filter_projection_update_interval,
        aXMLMetaData.filter_use_additive_continuation, aXMLMetaData.filter_power, aXMLMetaData.filter_heaviside_min,
        aXMLMetaData.filter_heaviside_update, aXMLMetaData.filter_heaviside_max};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    XMLGen::append_children(tKeys, tValues, aParentNode);
}

void append_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document &aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children({"Function", "Name"}, {"PlatoMainOutput", "Plato Main Output"}, tOperation);
    auto tOperationInput = tOperation.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"Topology"}, tOperation);
    tOperationInput = tOperation.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"Objective Gradient"}, tOperation);
    tOperationInput = tOperation.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"Control"}, tOperation);
}

void write_plato_main_operations_xml_file_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData)
{
    pugi::xml_document tDocument;
    XMLGen::append_attributes("include", {"filename"}, {"defines.xml"}, tDocument);

    tDocument.save_file("plato_main_operations.xml", "  ");
}

}

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, WritePlatoMainOperationsXmlFile)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::write_plato_main_operations_xml_file_for_nondeterministic_usecase(tXMLMetaData);
}

}
// namespace PlatoTestXMLGenerator
