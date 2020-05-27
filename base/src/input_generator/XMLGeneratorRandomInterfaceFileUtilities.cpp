/*
 * XMLGeneratorRandomInterfaceFileUtilities.cpp
 *
 *  Created on: May 25, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDefinesFileUtilities.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"
#include "XMLGeneratorRandomInterfaceFileUtilities.hpp"

namespace XMLGen
{

/******************************************************************************/
void append_nondeterministic_shared_data
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 pugi::xml_document& aDocument)
{
    auto tForNode = aDocument.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tForNode);
    tForNode = tForNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tForNode);
    auto tSharedDataNode = tForNode.append_child("SharedData");
    XMLGen::append_children(aKeys, aValues, tSharedDataNode);
}
// function append_nondeterministic_shared_data
/******************************************************************************/

/******************************************************************************/
void append_criterion_shared_data_for_nondeterministic_usecase
(const std::string& aCriterion,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR(std::string("Append Criterion Shared Data For Nondeterministic Use Case: ")
            + "Objective function list is empty.")
    }

    // shared data - nondeterministic criterion value
    auto tOwnerName = aXMLMetaData.objectives[0].performer_name + " {PerformerIndex}";
    auto tTag = aCriterion + " Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {tTag, "Scalar", "Global", "1", tOwnerName, "PlatoMain"};
    XMLGen::append_nondeterministic_shared_data(tKeys, tValues, aDocument);

    // shared data - nondeterministic criterion gradient
    tTag = aCriterion + " Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    tValues = {tTag, "Scalar", "Nodal Field", "IGNORE", tOwnerName, "PlatoMain"};
    XMLGen::append_nondeterministic_shared_data(tKeys, tValues, aDocument);
}
//function append_criterion_shared_data_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
void append_qoi_shared_data_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR(std::string("Append QOI Shared Data For Nondeterministic Use Case: ")
            + "Objective function list is empty.")
    }
    // shared data - QOI statistics
    // 1 loop over QOI
    // 2 define tag/name and set "Name" key
    auto tOwnerName = aXMLMetaData.objectives[0].performer_name + " {PerformerIndex}";
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Von Mises {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}", "Scalar",
                                        "Element Field", "IGNORE", tOwnerName, "PlatoMain"};
    XMLGen::append_nondeterministic_shared_data(tKeys, tValues, aDocument);
}
//function append_qoi_shared_data_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
void append_topology_shared_data_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR(std::string("Append Topology Shared Data for a Nondeterministic Use Case: ")
            + "Objective function list is empty.")
    }

    auto tSharedData = aDocument.append_child("SharedData");
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Topology", "Scalar", "Nodal Field", "IGNORE", "PlatoMain", "PlatoMain"};
    XMLGen::append_children(tKeys, tValues, tSharedData);
    auto tForNode = tSharedData.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tForNode);
    auto tUserName = aXMLMetaData.objectives[0].performer_name + " {PerformerIndex}";
    XMLGen::append_children({"UserName"}, {tUserName}, tForNode);
}
//function append_topology_shared_data_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
void append_physics_performers_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR(std::string("Append Physics Performer for a Nondeterministic Use Case: ")
            + "Objective function list is empty.")
    }

    for(auto& tObjective : aXMLMetaData.objectives)
    {
        const int tID = (&tObjective - &aXMLMetaData.objectives[0]) + 1;
        auto tPerformerNode = aDocument.append_child("Performer");
        XMLGen::append_children( { "PerformerID" }, { std::to_string(tID) }, tPerformerNode);
        auto tForNode = tPerformerNode.append_child("For");
        XMLGen::append_attributes( { "var", "in" }, { "PerformerIndex", "Performers" }, tForNode);
        auto tPerformerName = tObjective.performer_name + " {PerformerIndex}";
        XMLGen::append_children( { "Name", "Code" }, { tPerformerName, tObjective.code_name }, tForNode);
    }
}
// function append_physics_performers_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
void append_shared_data_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    XMLGen::append_control_shared_data(aDocument);
    XMLGen::append_lower_bounds_shared_data(aDocument);
    XMLGen::append_upper_bounds_shared_data(aDocument);
    XMLGen::append_design_volume_shared_data(aDocument);

    XMLGen::append_criterion_shared_data("Objective", aXMLMetaData, aDocument, "PlatoMain");
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        XMLGen::append_criterion_shared_data("Constraint", aXMLMetaData, aDocument, tConstraint.mPerformerName);
    }

    XMLGen::append_qoi_shared_data_for_nondeterministic_usecase(aXMLMetaData, aDocument);
    XMLGen::append_topology_shared_data_for_nondeterministic_usecase(aXMLMetaData, aDocument);
    XMLGen::append_criterion_shared_data_for_nondeterministic_usecase("Objective", aXMLMetaData, aDocument);
}
// function append_shared_data_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
void append_filter_criterion_gradient_samples_operation
(const std::string& aCriterionName,
 pugi::xml_node& aParentNode)
{
    auto tForNode = aParentNode.append_child("For");
    XMLGen::append_attributes( { "var", "in" }, { "PerformerSampleIndex", "PerformerSamples" }, tForNode);
    auto tOperationNode = tForNode.append_child("Operation");
    tForNode = tOperationNode.append_child("For");
    XMLGen::append_attributes( { "var", "in" }, { "PerformerIndex", "Performers" }, tForNode);
    tOperationNode = tForNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"},{"Filter Gradient", "PlatoMain"}, tOperationNode);

    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Field", "Control"}, tInputNode);
    tInputNode = tOperationNode.append_child("Input");
    auto tSharedDataName = aCriterionName + " Gradient" + " {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Gradient", tSharedDataName}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Filtered Gradient", tSharedDataName}, tOutputNode);
}
// function append_filter_criterion_gradient_samples_operation
/******************************************************************************/

/******************************************************************************/
void append_nondeterministic_operation
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 pugi::xml_node& aParentNode)
{
    auto tForNode = aParentNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tForNode);
    tForNode = tForNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tForNode);
    auto tOperationNode = tForNode.append_child("Operation");
    XMLGen::append_children(aKeys, aValues, tOperationNode);
}
// function append_nondeterministic_operation
/******************************************************************************/

/******************************************************************************/
void append_cache_state_stage_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR(std::string("Append Cache State Stage for a Nondeterministic Use Case: ")
            + "Objective function list is empty.")
    }

    for (auto &tObjective : aXMLMetaData.objectives)
    {
        auto tStageNode = aDocument.append_child("Stage");
        auto tIndex = &tObjective - &aXMLMetaData.objectives[0];
        auto tStageName = std::string("Cache State : ") + tObjective.performer_name + " " + std::to_string(tIndex);
        XMLGen::append_children( { "Name" }, { tStageName }, tStageNode);

        auto tPerformerName = tObjective.performer_name + " {PerformerIndex}";
        std::vector<std::string> tKeys = { "Name", "PerformerName" };
        std::vector<std::string> tValues = { "Cache State", tPerformerName };
        XMLGen::append_nondeterministic_operation(tKeys, tValues, tStageNode);
    }
}
// function append_cache_state_stage_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
void append_update_problem_stage_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR(std::string("Append Update Problem Stage for a Nondeterministic Use Case: ")
            + "Objective function list is empty.")
    }

    for (auto &tObjective : aXMLMetaData.objectives)
    {
        auto tStageNode = aDocument.append_child("Stage");
        auto tIndex = &tObjective - &aXMLMetaData.objectives[0];
        auto tStageName = std::string("Update Problem : ") + tObjective.performer_name + " " + std::to_string(tIndex);
        XMLGen::append_children( { "Name" }, { tStageName }, tStageNode);

        auto tPerformerName = tObjective.performer_name + " {PerformerIndex}";
        std::vector<std::string> tKeys = { "Name", "PerformerName" };
        std::vector<std::string> tValues = { "Update Problem", tPerformerName };
        XMLGen::append_nondeterministic_operation(tKeys, tValues, tStageNode);
    }
}
// function append_update_problem_stage_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
void append_nondeterministic_parameters
(const std::unordered_map<std::string, std::vector<std::string>>& aTagsMap,
 pugi::xml_node& aParentNode)
{
    for(auto& tPair : aTagsMap)
    {
        for(auto& tTag : tPair.second)
        {
            auto tParameterNode = aParentNode.append_child("Parameter");
            auto tValue = std::string("{") + tTag + "[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}";
            XMLGen::append_children({"ArgumentName", "ArgumentValue"}, {tTag, tValue}, tParameterNode);
        }
    }
}
// function append_nondeterministic_parameters
/******************************************************************************/

/******************************************************************************/
void append_sample_objective_value_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    for(auto& tObjective : aXMLMetaData.objectives)
    {
        auto tForNode = aParentNode.append_child("For");
        XMLGen::append_attributes( { "var", "in" }, { "PerformerSampleIndex", "PerformerSamples" }, tForNode);
        auto tOperationNode = tForNode.append_child("Operation");
        tForNode = tOperationNode.append_child("For");
        XMLGen::append_attributes( { "var", "in" }, { "PerformerIndex", "Performers" }, tForNode);
        tOperationNode = tForNode.append_child("Operation");
        auto tPerformerName = tObjective.performer_name + " {PerformerIndex}";
        XMLGen::append_children( { "Name", "PerformerName" }, { "Compute Objective Value", tPerformerName }, tOperationNode);

        auto tLoadTags = XMLGen::return_random_tractions_tags_for_define_xml_file(aXMLMetaData.mRandomMetaData);
        XMLGen::append_nondeterministic_parameters(tLoadTags, tOperationNode);
        auto tMaterialTags = XMLGen::return_material_properties_tags_for_define_xml_file(aXMLMetaData.mRandomMetaData);
        XMLGen::append_nondeterministic_parameters(tMaterialTags, tOperationNode);

        auto tInputNode = tOperationNode.append_child("Input");
        XMLGen::append_children( { "ArgumentName", "SharedDataName" }, { "Topology", "Topology" }, tInputNode);
        auto tOutputNode = tOperationNode.append_child("Output");
        auto tArgumentName = std::string("Objective Value");
        auto tSharedDataName = tArgumentName + " {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
        XMLGen::append_children( { "ArgumentName", "SharedDataName" }, { tArgumentName, tSharedDataName }, tOutputNode);
    }
}
// function append_sample_objective_value_operation
/******************************************************************************/

/******************************************************************************/
void append_evaluate_nondeterministic_criterion_value_operation
(const std::string& aCriterionName,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    auto tOperationName = std::string("Calculate Non-Deterministic ") + aCriterionName + " Value";
    XMLGen::append_children({"Name", "PerformerName"}, {tOperationName, "PlatoMain"}, tOperationNode);

    auto tForNode = tOperationNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tForNode);
    tForNode = tForNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tForNode);

    auto tInputNode = tForNode.append_child("Input");
    auto tDataName = aCriterionName + " Value " + " {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    XMLGen::append_children({ "ArgumentName", "SharedDataName" }, { tDataName, tDataName }, tInputNode);

    auto tOutputNode = tOperationNode.append_child("Output");
    auto tSharedDataName = aCriterionName + " Value";
    auto tArgumentName = aCriterionName + " Mean Plus " + aXMLMetaData.objective_number_standard_deviations + " StdDev";
    XMLGen::append_children({ "ArgumentName", "SharedDataName" }, { tArgumentName, tSharedDataName }, tOutputNode);
}
// function append_evaluate_nondeterministic_criterion_value_operation
/******************************************************************************/

/******************************************************************************/
void append_objective_value_stage_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tStageNode = aDocument.append_child("Stage");
    XMLGen::append_children({"Name"}, {"Calculate Objective Value"}, tStageNode);
    auto tInputNode = tStageNode.append_child("Input");
    XMLGen::append_children({"SharedDataName"}, {"Control"}, tInputNode);

    XMLGen::append_filter_control_operation(tStageNode);
    XMLGen::append_sample_objective_value_operation(aXMLMetaData, tStageNode);
    XMLGen::append_evaluate_nondeterministic_criterion_value_operation("Objective", aXMLMetaData, tStageNode);

    auto tOutputNode = tStageNode.append_child("Output");
    XMLGen::append_children({"SharedDataName"}, {"Objective Value"}, tOutputNode);
}
// function append_objective_value_stage_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
void append_sample_objective_gradient_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    for(auto& tObjective : aXMLMetaData.objectives)
    {
        auto tForNode = aParentNode.append_child("For");
        XMLGen::append_attributes( { "var", "in" }, { "PerformerSampleIndex", "PerformerSamples" }, tForNode);
        auto tOperationNode = tForNode.append_child("Operation");
        tForNode = tOperationNode.append_child("For");
        XMLGen::append_attributes( { "var", "in" }, { "PerformerIndex", "Performers" }, tForNode);
        tOperationNode = tForNode.append_child("Operation");
        auto tOperationName = std::string("Compute Objective Gradient");
        auto tPerformerName = tObjective.performer_name + " {PerformerIndex}";
        XMLGen::append_children( { "Name", "PerformerName" }, { tOperationName, tPerformerName }, tOperationNode);

        auto tLoadTags = XMLGen::return_random_tractions_tags_for_define_xml_file(aXMLMetaData.mRandomMetaData);
        XMLGen::append_nondeterministic_parameters(tLoadTags, tOperationNode);
        auto tMaterialTags = XMLGen::return_material_properties_tags_for_define_xml_file(aXMLMetaData.mRandomMetaData);
        XMLGen::append_nondeterministic_parameters(tMaterialTags, tOperationNode);

        auto tInputNode = tOperationNode.append_child("Input");
        XMLGen::append_children( { "ArgumentName", "SharedDataName" }, { "Topology", "Topology" }, tInputNode);
        auto tOutputNode = tOperationNode.append_child("Output");
        auto tArgumentName = std::string("Objective Gradient");
        auto tSharedDataName = tArgumentName + " {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
        XMLGen::append_children( { "ArgumentName", "SharedDataName" }, { tArgumentName, tSharedDataName }, tOutputNode);
    }
}
// function append_sample_objective_gradient_operation
/******************************************************************************/

/******************************************************************************/
void append_evaluate_nondeterministic_criterion_gradient_operation
(const std::string& aCriterionName,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    auto tOperationName = std::string("Calculate Non-Deterministic ") + aCriterionName + " Gradient";
    XMLGen::append_children({"Name", "PerformerName"}, {tOperationName, "PlatoMain"}, tOperationNode);
    auto tForNode = tOperationNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tForNode);
    tForNode = tForNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tForNode);

    auto tInputNode = tForNode.append_child("Input");
    auto tDataName = aCriterionName + " Value " + "{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    XMLGen::append_children({ "ArgumentName", "SharedDataName" }, { tDataName, tDataName }, tInputNode);
    tInputNode = tForNode.append_child("Input");
    tDataName = aCriterionName + " Gradient " + "{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    XMLGen::append_children({ "ArgumentName", "SharedDataName" }, { tDataName, tDataName }, tInputNode);

    auto tOutputNode = tOperationNode.append_child("Output");
    auto tSharedDataName = aCriterionName + " Gradient";
    auto tArgumentName = aCriterionName + " Mean Plus " + aXMLMetaData.objective_number_standard_deviations + " StdDev Gradient";
    XMLGen::append_children({ "ArgumentName", "SharedDataName" }, { tArgumentName, tSharedDataName }, tOutputNode);
}
// function append_evaluate_nondeterministic_criterion_gradient_operation
/******************************************************************************/

/******************************************************************************/
void append_objective_gradient_stage_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tStageNode = aDocument.append_child("Stage");
    XMLGen::append_children({"Name"}, {"Calculate Objective Gradient"}, tStageNode);
    auto tInputNode = tStageNode.append_child("Input");
    XMLGen::append_children({"SharedDataName"}, {"Control"}, tInputNode);

    XMLGen::append_filter_control_operation(tStageNode);
    XMLGen::append_sample_objective_gradient_operation(aXMLMetaData, tStageNode);
    XMLGen::append_filter_criterion_gradient_samples_operation("Objective", tStageNode);
    XMLGen::append_evaluate_nondeterministic_criterion_gradient_operation("Objective", aXMLMetaData, tStageNode);

    auto tOutputNode = tStageNode.append_child("Output");
    XMLGen::append_children({"SharedDataName"}, {"Objective Gradient"}, tOutputNode);
}
// function append_objective_gradient_stage_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
void append_stages_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    // deterministic stages
    XMLGen::append_design_volume_stage(aDocument);
    XMLGen::append_initial_guess_stage(aDocument);
    XMLGen::append_lower_bound_stage(aXMLMetaData, aDocument);
    XMLGen::append_upper_bound_stage(aXMLMetaData, aDocument);

    // nondeterministic stages
    XMLGen::append_cache_state_stage_for_nondeterministic_usecase(aXMLMetaData, aDocument);
    XMLGen::append_update_problem_stage_for_nondeterministic_usecase(aXMLMetaData, aDocument);

    // criteria stages
    XMLGen::append_constraint_value_stage(aXMLMetaData, aDocument);
    XMLGen::append_constraint_gradient_stage(aXMLMetaData, aDocument);
    XMLGen::append_objective_value_stage_for_nondeterministic_usecase(aXMLMetaData, aDocument);
    XMLGen::append_objective_gradient_stage_for_nondeterministic_usecase(aXMLMetaData, aDocument);
}
// function append_stages_for_nondeterministic_usecase
/******************************************************************************/

}
// namespace XMLGen
