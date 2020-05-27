/*
 * XMLGeneratorInterfaceFileUtilities.cpp
 *
 *  Created on: May 26, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"

namespace XMLGen
{

/******************************************************************************/
void append_lower_bounds_shared_data
(pugi::xml_document& aDocument)
{
    // shared data - lower bound value
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Lower Bound Value", "Scalar", "Global", "1", "PlatoMain", "PlatoMain"};
    auto tSharedDataNode = aDocument.append_child("SharedData");
    XMLGen::append_children(tKeys, tValues, tSharedDataNode);

    // shared data - lower bound vector
    tValues = {"Lower Bound Vector", "Scalar", "Nodal Field", "IGNORE", "PlatoMain", "PlatoMain"};
    tSharedDataNode = aDocument.append_child("SharedData");
    XMLGen::append_children(tKeys, tValues, tSharedDataNode);
}
// function append_lower_bounds_shared_data
/******************************************************************************/

/******************************************************************************/
void append_upper_bounds_shared_data
(pugi::xml_document& aDocument)
{
    // shared data - upper bound value
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Upper Bound Value", "Scalar", "Global", "1", "PlatoMain", "PlatoMain"};
    auto tSharedDataNode = aDocument.append_child("SharedData");
    XMLGen::append_children(tKeys, tValues, tSharedDataNode);

    // shared data - upper bound vector
    tValues = {"Upper Bound Vector", "Scalar", "Nodal Field", "IGNORE", "PlatoMain", "PlatoMain"};
    tSharedDataNode = aDocument.append_child("SharedData");
    XMLGen::append_children(tKeys, tValues, tSharedDataNode);
}
// function append_upper_bounds_shared_data
/******************************************************************************/

/******************************************************************************/
void append_design_volume_shared_data
(pugi::xml_document& aDocument)
{
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Reference Value", "Scalar", "Global", "1", "PlatoMain", "PlatoMain"};
    auto tSharedDataNode = aDocument.append_child("SharedData");
    XMLGen::append_children(tKeys, tValues, tSharedDataNode);
}
// function append_design_volume_shared_data
/******************************************************************************/

/******************************************************************************/
void append_criterion_shared_data
(const std::string& aCriterion,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument,
 std::string aOwnerName)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR(std::string("Append Criterion Shared Data: ") + "Objective function list is empty.")
    }

    // shared data - deterministic criterion value
    auto tTag = aCriterion + " Value";
    auto tOwnerName = aOwnerName.empty() ? aXMLMetaData.objectives[0].performer_name : aOwnerName;
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {tTag, "Scalar", "Global", "1", tOwnerName, "PlatoMain"};
    auto tSharedDataNode = aDocument.append_child("SharedData");
    XMLGen::append_children(tKeys, tValues, tSharedDataNode);

    // shared data - deterministic criterion gradient
    tTag = aCriterion + " Gradient";
    tValues = {tTag, "Scalar", "Nodal Field", "IGNORE", tOwnerName, "PlatoMain"};
    tSharedDataNode = aDocument.append_child("SharedData");
    XMLGen::append_children(tKeys, tValues, tSharedDataNode);
}
// function append_criterion_shared_data
/******************************************************************************/

/******************************************************************************/
void append_control_shared_data
(pugi::xml_document& aDocument)
{
    auto tSharedData = aDocument.append_child("SharedData");
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Control", "Scalar", "Nodal Field", "IGNORE", "PlatoMain", "PlatoMain"};
    XMLGen::append_children(tKeys, tValues, tSharedData);
}
// function append_control_shared_data
/******************************************************************************/

/******************************************************************************/
void append_plato_main_performer
(pugi::xml_document& aDocument)
{
    auto tPerformerNode = aDocument.append_child("Performer");
    XMLGen::append_children( {"Name", "Code", "PerformerID"}, {"PlatoMain", "PlatoMain", "0"}, tPerformerNode);
}
// function append_plato_main_performer
/******************************************************************************/

/******************************************************************************/
void append_filter_control_operation
(pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"},{"Filter Control", "PlatoMain"}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Field", "Control"}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Filtered Field", "Topology"}, tOutputNode);
}
// function append_filter_control_operation
/******************************************************************************/

/******************************************************************************/
void append_filter_criterion_gradient_operation
(const std::string& aCriterionName,
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"},{"Filter Gradient", "PlatoMain"}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Field", "Control"}, tInputNode);
    tInputNode = tOperationNode.append_child("Input");
    auto tSharedDataName = aCriterionName + " Gradient";
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Gradient", tSharedDataName}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Filtered Gradient", tSharedDataName}, tOutputNode);
}
// function append_filter_criterion_gradient_operation
/******************************************************************************/

/******************************************************************************/
void append_initial_field_operation
(pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"},{"Initialize Field", "PlatoMain"}, tOperationNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "Initialized Field"},{"SharedDataName", "Control"}, tOutputNode);
}
// function append_initial_field_operation
/******************************************************************************/

/******************************************************************************/
void append_initial_guess_stage
(pugi::xml_document& aDocument)
{
    auto tStageNode = aDocument.append_child("Stage");
    XMLGen::append_children({"Name"},{"Initial Guess"}, tStageNode);
    XMLGen::append_initial_field_operation(tStageNode);
    auto tOutputNode = tStageNode.append_child("Output");
    XMLGen::append_children({"SharedDataName"},{"Control"}, tOutputNode);
}
// function append_initial_guess_stage
/******************************************************************************/

/******************************************************************************/
void append_update_problem_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR(std::string("Append Update Problem Stage: ") + "Objective function list is empty.")
    }

    for(auto &tObjective : aXMLMetaData.objectives)
    {
        auto tStageNode = aDocument.append_child("Stage");
        auto tIndex = &tObjective - &aXMLMetaData.objectives[0];
        auto tStageName = std::string("Update Problem : ") + tObjective.performer_name + " " + std::to_string(tIndex);
        XMLGen::append_children( { "Name" }, { tStageName }, tStageNode);

        auto tOperationNode = tStageNode.append_child("Operation");
        XMLGen::append_children( { "Name", "PerformerName" }, { "Update Problem", tObjective.performer_name }, tOperationNode);
    }
}
// function append_update_problem_stage
/******************************************************************************/

/******************************************************************************/
void append_lower_bound_operation
(pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Calculate Lower Bounds", "PlatoMain"}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Lower Bound Vector", "Lower Bound Vector"}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Lower Bound Vector", "Lower Bound Vector"}, tOutputNode);
}
// function append_lower_bound_operation
/******************************************************************************/

/******************************************************************************/
void append_lower_bound_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tStageNode = aDocument.append_child("Stage");
    XMLGen::append_children({"Name"}, {"Set Lower Bounds"}, tStageNode);
    auto tLower = Plato::tolower(aXMLMetaData.optimization_type);
    if(tLower.compare("topology") == 0)
    {
        auto tInputNode = tStageNode.append_child("Input");
        XMLGen::append_children({"SharedDataName"}, {"Lower Bound Value"}, tInputNode);
        XMLGen::append_lower_bound_operation(tStageNode);
    }
    auto tOutputNode = tStageNode.append_child("Output");
    XMLGen::append_children({"SharedDataName"}, {"Lower Bound Vector"}, tOutputNode);
}
// function append_lower_bound_stage
/******************************************************************************/

/******************************************************************************/
void append_upper_bound_operation
(pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Calculate Upper Bounds", "PlatoMain"}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Upper Bound Vector", "Upper Bound Vector"}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Upper Bound Vector", "Upper Bound Vector"}, tOutputNode);
}
// function append_upper_bound_operation
/******************************************************************************/

/******************************************************************************/
void append_upper_bound_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tStageNode = aDocument.append_child("Stage");
    XMLGen::append_children({"Name"}, {"Set Upper Bounds"}, tStageNode);
    auto tLower = Plato::tolower(aXMLMetaData.optimization_type);
    if(tLower.compare("topology") == 0)
    {
        auto tInputNode = tStageNode.append_child("Input");
        XMLGen::append_children({"SharedDataName"}, {"Upper Bound Value"}, tInputNode);
        XMLGen::append_upper_bound_operation(tStageNode);
    }
    auto tOutputNode = tStageNode.append_child("Output");
    XMLGen::append_children({"SharedDataName"}, {"Upper Bound Vector"}, tOutputNode);
}
// function append_upper_bound_stage
/******************************************************************************/

/******************************************************************************/
void append_design_volume_operation
(pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Calculate Design Domain Volume", "PlatoMain"}, tOperationNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Reference Value", "Reference Value"}, tOutputNode);
}
// function append_design_volume_stage
/******************************************************************************/

/******************************************************************************/
void append_design_volume_stage
(pugi::xml_document& aDocument)
{
    auto tStageNode = aDocument.append_child("Stage");
    XMLGen::append_children({"Name"}, {"Calculate Design Domain Volume"}, tStageNode);
    XMLGen::append_design_volume_operation(tStageNode);
    auto tOutputNode = tStageNode.append_child("Output");
    XMLGen::append_children({"SharedDataName"}, {"Reference Value"}, tOutputNode);
}
// function append_design_volume_stage
/******************************************************************************/

/******************************************************************************/
void append_constraint_value_operation
(const std::string& aPerformerName,
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Calculate Constraint Value", aPerformerName}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Constraint Value", "Constraint Value"}, tOutputNode);
}
// function append_constraint_value_operation
/******************************************************************************/

/******************************************************************************/
void append_constraint_value_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tStageNode = aDocument.append_child("Stage");
        auto tIndex = &tConstraint - &aXMLMetaData.constraints[0];
        auto tStageName = std::string("Calculate Constraint Value ") + std::to_string(tIndex);
        XMLGen::append_children({"Name", "Type"}, {tStageName, tConstraint.type}, tStageNode);
        auto tInputNode = tStageNode.append_child("Input");
        XMLGen::append_children({"SharedDataName"}, {"Control"}, tInputNode);

        XMLGen::append_filter_control_operation(tStageNode);
        XMLGen::append_constraint_value_operation(tConstraint.mPerformerName, tStageNode);

        auto tOutputNode = tStageNode.append_child("Output");
        XMLGen::append_children({"SharedDataName"}, {"Constraint Value"}, tOutputNode);
    }
}
// function append_constraint_value_stage
/******************************************************************************/

/******************************************************************************/
void append_constraint_gradient_operation
(const std::string& aPerformerName,
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Calculate Constraint Gradient", aPerformerName}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Constraint Gradient", "Constraint Gradient"}, tOutputNode);
}
// function append_constraint_gradient_operation
/******************************************************************************/

/******************************************************************************/
void append_constraint_gradient_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tStageNode = aDocument.append_child("Stage");
        auto tIndex = &tConstraint - &aXMLMetaData.constraints[0];
        auto tStageName = std::string("Calculate Constraint Gradient ") + std::to_string(tIndex);
        XMLGen::append_children({"Name", "Type"}, {tStageName, tConstraint.type}, tStageNode);
        auto tInputNode = tStageNode.append_child("Input");
        XMLGen::append_children({"SharedDataName"}, {"Control"}, tInputNode);

        XMLGen::append_filter_control_operation(tStageNode);
        XMLGen::append_constraint_gradient_operation(tConstraint.mPerformerName, tStageNode);
        XMLGen::append_filter_criterion_gradient_operation("Constraint", tStageNode);

        auto tOutputNode = tStageNode.append_child("Output");
        XMLGen::append_children({"SharedDataName"}, {"Constraint Gradient"}, tOutputNode);
    }
}
// function append_constraint_gradient_stage
/******************************************************************************/

}
// namespace XMLGen
