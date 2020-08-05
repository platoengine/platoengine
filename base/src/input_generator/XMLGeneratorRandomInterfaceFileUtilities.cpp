/*
 * XMLGeneratorRandomInterfaceFileUtilities.cpp
 *
 *  Created on: May 25, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorDefinesFileUtilities.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"
#include "XMLGeneratorRandomInterfaceFileUtilities.hpp"

namespace XMLGen
{

/******************************************************************************/
void append_multiperformer_shared_data
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
// function append_multiperformer_shared_data
/******************************************************************************/

/******************************************************************************/
void append_multiperformer_criterion_shared_data
(const std::string& aCriterion,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.services().empty())
    {
        THROWERR("Append Criterion Shared Data For Nondeterministic Use Case: Services list is empty.")
    }

    // shared data - nondeterministic criterion value
    for (auto &tService : aXMLMetaData.services())
    {
        auto tOwnerName = tService.performer() + "_{PerformerIndex}";
        auto tTag = aCriterion + " Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
        std::vector<std::string> tKeys = { "Name", "Type", "Layout", "Size", "OwnerName", "UserName" };
        std::vector<std::string> tValues = { tTag, "Scalar", "Global", "1", tOwnerName, "platomain" };
        XMLGen::append_multiperformer_shared_data(tKeys, tValues, aDocument);

        // shared data - nondeterministic criterion gradient
        tTag = aCriterion + " Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
        tValues = { tTag, "Scalar", "Nodal Field", "IGNORE", tOwnerName, "platomain" };
        XMLGen::append_multiperformer_shared_data(tKeys, tValues, aDocument);
    }
}
//function append_multiperformer_criterion_shared_data
/******************************************************************************/

/******************************************************************************/
void append_qoi_statistics_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tOutputDataIDs = aXMLMetaData.mOutputMetaData.randomIDs();
    for(auto& tOutputDataID : tOutputDataIDs)
    {
        auto tLayout = aXMLMetaData.mOutputMetaData.randomLayout(tOutputDataID);
        auto tMeanSharedData = tOutputDataID + " mean";
        std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
        std::vector<std::string> tValues = {tMeanSharedData, "Scalar", tLayout, "IGNORE", "platomain", "platomain"};
        auto tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);

        auto tStdDevSharedData = tOutputDataID + " standard deviation";
        tValues = {tStdDevSharedData, "Scalar", tLayout, "IGNORE", "platomain", "platomain"};
        tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);
    }
}
//function append_qoi_statistics_shared_data
/******************************************************************************/

/******************************************************************************/
void append_multiperformer_qoi_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.services().empty())
    {
        THROWERR("Append Nondeterministic QoI Shared Data: list of 'services' is empty.")
    }

    // TODO: LOOP OVER OUTPUT METADATA. FIRST, OUTPUT METADATA NEEDS TO BE REFACTORED TO SUPPORT MULTIPLE serviceS - I.E. SUPPORT STD::VECTOR
    auto tIDs = aXMLMetaData.mOutputMetaData.randomIDs();
    auto tServiceID = aXMLMetaData.mOutputMetaData.serviceID();
    for(auto& tID : tIDs)
    {
        auto tLayout = aXMLMetaData.mOutputMetaData.randomLayout(tID);
        auto tSharedDataName = aXMLMetaData.mOutputMetaData.randomSharedDataName(tID);
        auto tOwnerName = aXMLMetaData.service(tServiceID).performer() + "_{PerformerIndex}";
        std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
        std::vector<std::string> tValues = {tSharedDataName, "Scalar", tLayout, "IGNORE", tOwnerName, "platomain"};
        XMLGen::append_multiperformer_shared_data(tKeys, tValues, aDocument);
    }
}
//function append_multiperformer_qoi_shared_data
/******************************************************************************/

/******************************************************************************/
void append_multiperformer_topology_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.services().empty())
    {
        THROWERR("Append Topology Shared Data for a Nondeterministic Use Case: Services list is empty.")
    }

    auto tSharedData = aDocument.append_child("SharedData");
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Topology", "Scalar", "Nodal Field", "IGNORE", "platomain", "platomain"};
    XMLGen::append_children(tKeys, tValues, tSharedData);

    for(auto& tService : aXMLMetaData.services())
    {
        auto tForNode = tSharedData.append_child("For");
        XMLGen::append_attributes( { "var", "in" }, { "PerformerIndex", "Performers" }, tForNode);
        auto tUserName = tService.performer() + "_{PerformerIndex}";
        XMLGen::append_children( { "UserName" }, { tUserName }, tForNode);
    }
}
//function append_multiperformer_topology_shared_data
/******************************************************************************/

/******************************************************************************/
void append_physics_performers_multiperformer_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.services().empty())
    {
        THROWERR("Append Physics Performer for a Nondeterministic Use Case: Services list is empty.")
    }

    for(auto& tService : aXMLMetaData.services())
    {
        const int tID = (&tService - &aXMLMetaData.services()[0]) + 1;
        auto tPerformerNode = aDocument.append_child("Performer");
        XMLGen::append_children( { "PerformerID" }, { std::to_string(tID) }, tPerformerNode);
        auto tForNode = tPerformerNode.append_child("For");
        XMLGen::append_attributes( { "var", "in" }, { "PerformerIndex", "Performers" }, tForNode);
        auto tPerformerName = tService.performer() + "_{PerformerIndex}";
        XMLGen::append_children( { "Name", "Code" }, { tPerformerName, tService.code() }, tForNode);
    }
}
// function append_physics_performers_multiperformer_usecase
/******************************************************************************/

/******************************************************************************/
void append_filter_criterion_gradient_samples_operation
(const std::string& aCriterionName,
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"},{"Filter Gradient", "platomain"}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Field", "Control"}, tInputNode);

    auto tForNode = tOperationNode.append_child("For");
    XMLGen::append_attributes( { "var", "in" }, { "PerformerSampleIndex", "PerformerSamples" }, tForNode);
    tForNode = tForNode.append_child("For");
    XMLGen::append_attributes( { "var", "in" }, { "PerformerIndex", "Performers" }, tForNode);

    tInputNode = tForNode.append_child("Input");
    auto tSharedDataName = aCriterionName + " Gradient" + " {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Gradient", tSharedDataName}, tInputNode);
    auto tOutputNode = tForNode.append_child("Output");
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
    for (auto &tService : aXMLMetaData.services())
    {
        if (!tService.cacheState())
        {
            continue;
        }
        auto tStageNode = aDocument.append_child("Stage");
        XMLGen::append_children( { "Name" }, { "Cache State" }, tStageNode);
        auto tPerformerName = tService.performer() + "_{PerformerIndex}";
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
    for (auto &tService : aXMLMetaData.services())
    {
        if (!tService.updateProblem())
        {
            continue;
        }
        auto tStageNode = aDocument.append_child("Stage");
        XMLGen::append_children( { "Name" }, { "Update Problem" }, tStageNode);
        auto tPerformerName = tService.performer() + "_{PerformerIndex}";
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
(const std::string& aPerformerName,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tForNode = aParentNode.append_child("For");
    XMLGen::append_attributes( { "var", "in" }, { "PerformerSampleIndex", "PerformerSamples" }, tForNode);
    auto tOperationNode = tForNode.append_child("Operation");
    tForNode = tOperationNode.append_child("For");
    XMLGen::append_attributes( { "var", "in" }, { "PerformerIndex", "Performers" }, tForNode);
    tOperationNode = tForNode.append_child("Operation");
    auto tPerformerName = aPerformerName + "_{PerformerIndex}";
    XMLGen::append_children( { "Name", "PerformerName" }, { "Compute Objective Value", tPerformerName }, tOperationNode);

    auto tLoadTags = XMLGen::return_random_tractions_tags_for_define_xml_file(aXMLMetaData.mRandomMetaData);
    XMLGen::append_nondeterministic_parameters(tLoadTags, tOperationNode);
    auto tMaterialTags = XMLGen::return_material_property_tags_for_define_xml_file(aXMLMetaData.mRandomMetaData);
    XMLGen::append_nondeterministic_parameters(tMaterialTags, tOperationNode);

    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children( { "ArgumentName", "SharedDataName" }, { "Topology", "Topology" }, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    auto tArgumentName = std::string("Objective Value");
    auto tSharedDataName = tArgumentName + " {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
    XMLGen::append_children( { "ArgumentName", "SharedDataName" }, { tArgumentName, tSharedDataName }, tOutputNode);
}
// function append_sample_objective_value_operation
/******************************************************************************/

/******************************************************************************/
void append_evaluate_nondeterministic_objective_value_operation
(const std::string& aOutputSharedDataName,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    auto tOperationName = std::string("Compute Non-Deterministic Objective Value");
    XMLGen::append_children({"Name", "PerformerName"}, {tOperationName, "platomain"}, tOperationNode);

    auto tForNode = tOperationNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tForNode);
    tForNode = tForNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tForNode);

    auto tInputNode = tForNode.append_child("Input");
    auto tDataName = std::string("Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    XMLGen::append_children({ "ArgumentName", "SharedDataName" }, { tDataName, tDataName }, tInputNode);

    auto tOutputNode = tOperationNode.append_child("Output");
    auto tArgumentName = std::string("Objective Mean Plus ") + aXMLMetaData.objective_number_standard_deviations + " StdDev";
    XMLGen::append_children({ "ArgumentName", "SharedDataName" }, { tArgumentName, aOutputSharedDataName }, tOutputNode);
}
// function append_evaluate_nondeterministic_objective_value_operation
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
        auto tPerformerName = tObjective.mPerformerName + "_{PerformerIndex}";
        XMLGen::append_children( { "Name", "PerformerName" }, { tOperationName, tPerformerName }, tOperationNode);

        auto tLoadTags = XMLGen::return_random_tractions_tags_for_define_xml_file(aXMLMetaData.mRandomMetaData);
        XMLGen::append_nondeterministic_parameters(tLoadTags, tOperationNode);
        auto tMaterialTags = XMLGen::return_material_property_tags_for_define_xml_file(aXMLMetaData.mRandomMetaData);
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
void append_evaluate_nondeterministic_objective_gradient_operation
(const std::string& aOutputSharedDataName,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    auto tOperationName = std::string("Compute Non-Deterministic Objective Gradient");
    XMLGen::append_children({"Name", "PerformerName"}, {tOperationName, "platomain"}, tOperationNode);
    auto tForNode = tOperationNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tForNode);
    tForNode = tForNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tForNode);

    auto tInputNode = tForNode.append_child("Input");
    auto tDataName = std::string("Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    XMLGen::append_children({ "ArgumentName", "SharedDataName" }, { tDataName, tDataName }, tInputNode);
    tInputNode = tForNode.append_child("Input");
    tDataName = std::string("Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}");
    XMLGen::append_children({ "ArgumentName", "SharedDataName" }, { tDataName, tDataName }, tInputNode);

    auto tOutputNode = tOperationNode.append_child("Output");
    auto tArgumentName = std::string("Objective Mean Plus ") + aXMLMetaData.objective_number_standard_deviations + " StdDev Gradient";
    XMLGen::append_children({ "ArgumentName", "SharedDataName" }, { tArgumentName, aOutputSharedDataName }, tOutputNode);
}
// function append_evaluate_nondeterministic_objective_gradient_operation
/******************************************************************************/

}
// namespace XMLGen
