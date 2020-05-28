/*
 * XMLGeneratorPlatoMainOperationFileUtilities.cpp
 *
 *  Created on: May 28, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"

namespace XMLGen
{

/******************************************************************************/
void append_filter_options_to_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    std::vector<std::string> tKeys = {"Scale", "Absolute", "StartIteration", "UpdateInterval",
        "UseAdditiveContinuation", "Power", "HeavisideMin", "HeavisideUpdate", "HeavisideMax"};

    auto tScale = aXMLMetaData.filter_radius_scale.empty() ? std::string("2.0") : aXMLMetaData.filter_radius_scale;
    std::vector<std::string> tValues = {tScale, aXMLMetaData.filter_radius_absolute,
        aXMLMetaData.filter_projection_start_iteration, aXMLMetaData.filter_projection_update_interval,
        aXMLMetaData.filter_use_additive_continuation, aXMLMetaData.filter_power, aXMLMetaData.filter_heaviside_min,
        aXMLMetaData.filter_heaviside_update, aXMLMetaData.filter_heaviside_max};

    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    XMLGen::append_children(tKeys, tValues, aParentNode);
}
// function append_filter_options_to_operation
/******************************************************************************/

/******************************************************************************/
void append_filter_to_plato_main_operation
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
    XMLGen::append_filter_options_to_operation(aXMLMetaData, tFilterNode);
}
// function append_filter_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_constraint_gradient_input_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tIndex = std::to_string(&tConstraint - &aXMLMetaData.constraints[0]);
        auto tInput = aParentNode.append_child("Input");
        auto tSharedDatatName = std::string("Constraint Gradient ") + tIndex;
        XMLGen::append_children( { "ArgumentName" }, { tSharedDatatName }, tInput);
    }
}
// function append_constraint_gradient_input_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_objective_gradient_input_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    for(auto& tObjective : aXMLMetaData.objectives)
    {
        auto tIndex = std::to_string(&tObjective - &aXMLMetaData.objectives[0]);
        auto tInput = aParentNode.append_child("Input");
        auto tSharedDatatName = std::string("Objective Gradient ") + tIndex;
        XMLGen::append_children( { "ArgumentName" }, { tSharedDatatName }, tInput);
    }
}
// function append_objective_gradient_input_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_deterministic_qoi_inputs_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    std::unordered_map<std::string, std::string> tValidOutputLayouts =
        { {"element field", "Element Field"}, {"nodal field", "Nodal Field"} };

    for(auto& tPair : aXMLMetaData.mOutputMetaData.mDeterministicQuantitiesOfInterest)
    {
        auto tValidLayoutItr = tValidOutputLayouts.find(tPair.second);
        if(tValidLayoutItr == tValidOutputLayouts.end())
        {
            THROWERR(std::string("Append Deterministic QOI Inputs to Output Operation: ")
                + "QOI '" + tPair.first + "' has unsupported layout '" + tPair.second + "'.")
        }
        auto tInput= aParentNode.append_child("Input");
        XMLGen::append_children( { "ArgumentName", "Layout" }, { tPair.first, tValidLayoutItr->second }, tInput);
    }
}
// function append_deterministic_qoi_inputs_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_nondeterministic_qoi_inputs_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    std::unordered_map<std::string, std::string> tValidOutputLayouts =
        { {"element field", "Element Field"}, {"nodal field", "Nodal Field"} };

    for(auto& tPair : aXMLMetaData.mOutputMetaData.mRandomQuantitiesOfInterest)
    {
        auto tFor = aParentNode.append_child("For");
        XMLGen::append_attributes({"var", "in"}, {"SampleIndex", "Samples"}, tFor);
        auto tValidLayoutItr = tValidOutputLayouts.find(tPair.second);
        if(tValidLayoutItr == tValidOutputLayouts.end())
        {
            THROWERR(std::string("Append Nondeterministic QOI Inputs to Output Operation: ")
                + "QOI '" + tPair.first + "' has unsupported layout '" + tPair.second + "'.")
        }
        auto tInput= tFor.append_child("Input");
        auto tSharedDataName = tPair.first + " {SampleIndex}";
        XMLGen::append_children( { "ArgumentName", "Layout" }, { tSharedDataName, tValidLayoutItr->second }, tInput);
    }
}
// function append_nondeterministic_qoi_inputs_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_children_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    std::vector<std::string> tKeys = {"Function", "Name", "WriteRestart", "OutputFrequency", "MaxIterations"};
    std::vector<std::string> tValues = {"PlatoMainOutput", "Plato Main Output", aXMLMetaData.write_restart_file,
        aXMLMetaData.output_frequency, aXMLMetaData.max_iterations};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    XMLGen::append_children(tKeys, tValues, aParentNode);
}
// function append_children_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_default_qoi_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    auto tInput = aParentNode.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"Topology"}, tInput);
    tInput = aParentNode.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"Control"}, tInput);
    XMLGen::append_objective_gradient_input_to_output_operation(aXMLMetaData, aParentNode);
    XMLGen::append_constraint_gradient_input_to_output_operation(aXMLMetaData, aParentNode);
}
// function append_default_qoi_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_surface_extraction_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    if (!aXMLMetaData.output_method.empty())
    {
        auto tSurfaceExtraction = aParentNode.append_child("SurfaceExtraction");
        std::vector<std::string> tKeys = { "OutputMethod", "Discretization" };
        auto tDiscretization = aXMLMetaData.discretization.empty() ? "density" : aXMLMetaData.discretization;
        std::vector<std::string> tValues = { aXMLMetaData.output_method, tDiscretization };
        XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
        XMLGen::append_children(tKeys, tValues, tSurfaceExtraction);

        auto tOutput = tSurfaceExtraction.append_child("Output");
        XMLGen::append_children( { "Format" }, { "Exodus" }, tOutput);
    }
}
// function append_surface_extraction_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_output_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document &aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children_to_output_operation(aXMLMetaData, tOperation);
    XMLGen::append_default_qoi_to_output_operation(aXMLMetaData, tOperation);
    XMLGen::append_deterministic_qoi_inputs_to_output_operation(aXMLMetaData, tOperation);
    XMLGen::append_nondeterministic_qoi_inputs_to_output_operation(aXMLMetaData, tOperation);
    XMLGen::append_surface_extraction_to_output_operation(aXMLMetaData, tOperation);
}
// function append_output_to_plato_main_operation
/******************************************************************************/

}
// namespace XMLGen
