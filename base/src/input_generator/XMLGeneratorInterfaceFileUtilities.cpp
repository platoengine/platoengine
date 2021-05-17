/*
 * XMLGeneratorInterfaceFileUtilities.cpp
 *
 *  Created on: May 26, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeProblem.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"
#include "XMLGeneratorPlatoMainConstraintValueOperationInterface.hpp"
#include "XMLGeneratorPlatoMainConstraintGradientOperationInterface.hpp"
#include <tuple>

namespace XMLGen
{

/******************************************************************************/
void write_interface_xml_file
(const XMLGen::InputData& aMetaData)
{
    pugi::xml_document tDocument;
    int tNextPerformerID = 0;

    XMLGen::append_include_defines_xml_data(aMetaData, tDocument);
    XMLGen::append_console_data(aMetaData, tDocument);
    XMLGen::append_performer_data(aMetaData, tNextPerformerID, tDocument);
    XMLGen::append_shared_data(aMetaData, tDocument);
    XMLGen::append_stages(aMetaData, tDocument);
    XMLGen::append_optimizer_options(aMetaData, tDocument);

    tDocument.save_file("interface.xml", "  ");
}
/******************************************************************************/

/******************************************************************************/
void append_performer_data
(const XMLGen::InputData& aMetaData,
 int &aNextPerformerID,
 pugi::xml_node& aParentNode)
{
    XMLGen::append_plato_main_performer(aMetaData, aNextPerformerID, aParentNode);

    // note: multiperformer use case currently only works with Plato Analyze, and is only used currently with the robust optimization workflow
    if(XMLGen::is_robust_optimization_problem(aMetaData))
        XMLGen::append_physics_performers_multiperformer_usecase(aMetaData, aNextPerformerID, aParentNode);
    else
        XMLGen::append_physics_performers(aMetaData, aNextPerformerID, aParentNode);

    XMLGen::append_esp_performers(aMetaData, aNextPerformerID, aParentNode);
}
/******************************************************************************/

/******************************************************************************/
void append_console_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_node& aParentNode)
{
    auto tNode = aParentNode.append_child("Console");
    XMLGen::append_children({"Enabled"}, {"true"}, tNode);
    XMLGen::append_children({"Verbose"}, {aMetaData.optimization_parameters().verbose()}, tNode);
}
/******************************************************************************/

/******************************************************************************/
void append_compute_qoi_statistics_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    auto tQoIIDs = aXMLMetaData.mOutputMetaData[0].randomIDs();
    for (auto &tQoIID : tQoIIDs)
    {
        auto tOperationNode = aParentNode.append_child("Operation");
        auto tOperationName = "compute " + tQoIID + " statistics";
        XMLGen::append_children({"Name", "PerformerName"}, {tOperationName, tFirstPlatoMainPerformer}, tOperationNode);

        auto tForNode = tOperationNode.append_child("For");
        XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tForNode);
        tForNode = tForNode.append_child("For");
        XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tForNode);

        auto tInputNode = tForNode.append_child("Input");
        auto tDataName = tQoIID + " {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}";
        XMLGen::append_children({ "ArgumentName", "SharedDataName" }, { tDataName, tDataName }, tInputNode);

        auto tMeanName = tQoIID + " mean";
        auto tOutputNode = tOperationNode.append_child("Output");
        XMLGen::append_children({ "ArgumentName", "SharedDataName" }, { tMeanName, tMeanName }, tOutputNode);

        auto tStdDevName = tQoIID + " standard deviation";
        tOutputNode = tOperationNode.append_child("Output");
        XMLGen::append_children({ "ArgumentName", "SharedDataName" }, { tStdDevName, tStdDevName }, tOutputNode);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_esp_performers
(const XMLGen::InputData& aXMLMetaData,
 int &aNextPerformerID,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        for(auto& tService : aXMLMetaData.mPerformerServices)
        {
            if(tService.code() == "plato_esp")
            {
                auto tPerformerNode = aParentNode.append_child("Performer");
                addChild(tPerformerNode, "PerformerID", std::to_string(aNextPerformerID));
                aNextPerformerID++;
                auto tForNode = tPerformerNode.append_child("For");
                tForNode.append_attribute("var") = "I";
                tForNode.append_attribute("in") = "Parameters";
                addChild(tForNode, "Name", "plato_esp_{I}");
            }
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_physics_performers
(const XMLGen::InputData& aXMLMetaData,
 int &aNextPerformerID,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.mPerformerServices.empty())
    {
        THROWERR("Append Physics Performer: Services list is empty.")
    }

    std::vector<std::string> tKeywords = { "Name", "Code", "PerformerID" };
    for(auto& tService : aXMLMetaData.mPerformerServices)
    {
        if(tService.code() != "plato_esp")
        {
            auto tPerformerNode = aParentNode.append_child("Performer");
            std::vector<std::string> tValues = { tService.performer(), tService.code(), std::to_string(aNextPerformerID) };
            aNextPerformerID++;
            XMLGen::append_children( tKeywords, tValues, tPerformerNode);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_qoi_shared_data_for_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aParentNode)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    for(auto &tOutputMetaData : aXMLMetaData.mOutputMetaData)
    {
        auto tOutputIDs = tOutputMetaData.deterministicIDs();
        auto tServiceID = tOutputMetaData.serviceID();
        auto tOwnerName = aXMLMetaData.service(tServiceID).performer();
        for(auto& tID : tOutputIDs)
        {
            auto tLayout = tOutputMetaData.deterministicLayout(tID);
            for(size_t i=0; i<aXMLMetaData.objective.scenarioIDs.size(); ++i)
            {
                auto tSharedDataName = tOutputMetaData.deterministicSharedDataName(tID);
                auto tScenarioIndex = std::to_string(i);
                auto tScenarioID = aXMLMetaData.objective.scenarioIDs[i];
                tSharedDataName += std::string("_") + tOwnerName;
                tSharedDataName += "_scenario_" + tScenarioID;
                std::vector<std::string> tValues = {tSharedDataName, "Scalar", tLayout, "IGNORE", tOwnerName, tFirstPlatoMainPerformer};
                auto tSharedDataNode = aParentNode.append_child("SharedData");
                XMLGen::append_children(tKeys, tValues, tSharedDataNode);
            }                        
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_qoi_shared_data_for_non_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aParentNode)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    for(auto &tOutputMetaData : aXMLMetaData.mOutputMetaData)
    {
        auto tOutputIDs = tOutputMetaData.deterministicIDs();
        auto tServiceID = tOutputMetaData.serviceID();
        auto tOwnerName = aXMLMetaData.service(tServiceID).performer();
        for(auto& tID : tOutputIDs)
        {
            auto tLayout = tOutputMetaData.deterministicLayout(tID);
            auto tSharedDataName = tOutputMetaData.deterministicSharedDataName(tID);
            tSharedDataName += std::string("_") + tOwnerName;
            std::vector<std::string> tValues = {tSharedDataName, "Scalar", tLayout, "IGNORE", tOwnerName, tFirstPlatoMainPerformer};
            auto tSharedDataNode = aParentNode.append_child("SharedData");
            XMLGen::append_children(tKeys, tValues, tSharedDataNode);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_qoi_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aParentNode)
{
    if(aXMLMetaData.services().empty())
    {
        THROWERR("Append QoI Shared Data: list of 'services' is empty.")
    }

    if(aXMLMetaData.objective.multi_load_case == "true")
    {
        append_qoi_shared_data_for_multi_load_case(aXMLMetaData, aParentNode);
    }
    else
    {
        append_qoi_shared_data_for_non_multi_load_case(aXMLMetaData, aParentNode);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_topology_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        return;
    }
    if(aXMLMetaData.services().empty())
    {
        THROWERR("Append Topology Shared Data: Services list is empty.")
    }

    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    auto tSharedData = aDocument.append_child("SharedData");
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName"};
    if(aXMLMetaData.optimization_parameters().filter_in_engine() == "false")
    {
        std::string tFirstPlatoAnalyzePerformer = aXMLMetaData.getFirstPlatoAnalyzePerformer();
        std::vector<std::string> tValues = {"Topology", "Scalar", "Nodal Field", "IGNORE", tFirstPlatoAnalyzePerformer};
        XMLGen::append_children(tKeys, tValues, tSharedData);
        tKeys = {"UserName"};
        tValues = {tFirstPlatoAnalyzePerformer};
        XMLGen::append_children(tKeys, tValues, tSharedData);
        tKeys = {"UserName"};
        tValues = {tFirstPlatoMainPerformer};
        XMLGen::append_children(tKeys, tValues, tSharedData);
    }
    else
    {
        std::vector<std::string> tValues = {"Topology", "Scalar", "Nodal Field", "IGNORE", tFirstPlatoMainPerformer};
        XMLGen::append_children(tKeys, tValues, tSharedData);
        for(auto& tService : aXMLMetaData.services())
        {
            tKeys = {"UserName"};
            tValues = {tService.performer()};
            XMLGen::append_children(tKeys, tValues, tSharedData);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    XMLGen::append_design_variables_shared_data(aMetaData, aDocument);
    XMLGen::append_lower_bounds_shared_data(aMetaData, aDocument);
    XMLGen::append_upper_bounds_shared_data(aMetaData, aDocument);
    XMLGen::append_design_volume_shared_data(aMetaData, aDocument);
    XMLGen::append_parameter_sensitivity_shared_data(aMetaData, aDocument);
    XMLGen::append_criteria_shared_data(aMetaData, aDocument);
    XMLGen::append_constraint_shared_data(aMetaData, aDocument);
    XMLGen::append_objective_shared_data(aMetaData, aDocument);
    XMLGen::append_normalization_shared_data(aMetaData, aDocument);

    if(XMLGen::is_robust_optimization_problem(aMetaData))
    {
        XMLGen::append_qoi_statistics_shared_data(aMetaData, aDocument);
        XMLGen::append_multiperformer_qoi_shared_data(aMetaData, aDocument);
        XMLGen::append_multiperformer_topology_shared_data(aMetaData, aDocument);
        XMLGen::append_multiperformer_criterion_shared_data("Objective", aMetaData, aDocument);
    }
    else
    {
        XMLGen::append_qoi_shared_data(aMetaData, aDocument);
        XMLGen::append_topology_shared_data(aMetaData, aDocument);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_stages
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    // deterministic stages
    XMLGen::append_design_volume_stage(aXMLMetaData, aDocument);
    XMLGen::append_initial_guess_stage(aXMLMetaData, aDocument);
    XMLGen::append_lower_bound_stage(aXMLMetaData, aDocument);
    XMLGen::append_upper_bound_stage(aXMLMetaData, aDocument);
    XMLGen::append_plato_main_output_stage(aXMLMetaData, aDocument);

    // nondeterministic stages
    if(XMLGen::is_robust_optimization_problem(aXMLMetaData))
    {
        XMLGen::append_cache_state_stage_for_nondeterministic_usecase(aXMLMetaData, aDocument);
        XMLGen::append_update_problem_stage_for_nondeterministic_usecase(aXMLMetaData, aDocument);
    }
    else
    {
        XMLGen::append_cache_state_stage(aXMLMetaData, aDocument);
        XMLGen::append_update_problem_stage(aXMLMetaData, aDocument);
    }

    // constraint stages
    XMLGen::append_constraint_value_stage(aXMLMetaData, aDocument);
    XMLGen::append_constraint_gradient_stage(aXMLMetaData, aDocument);

    // objective stages
    XMLGen::append_objective_value_stage(aXMLMetaData, aDocument);
    XMLGen::append_objective_gradient_stage(aXMLMetaData, aDocument);
}
/******************************************************************************/

/******************************************************************************/
int num_cache_states
(const std::vector<XMLGen::Service> &aServices)
{
    int tNumCacheStates = 0;
    for(auto &tService : aServices)
    {
        if(tService.cacheState())
        {
            tNumCacheStates++;
        }
    }
    return tNumCacheStates;
}
/******************************************************************************/

/******************************************************************************/
void append_cache_state_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    int tNumCacheStates = num_cache_states(aXMLMetaData.services());
    if(tNumCacheStates > 0)
    {
        auto tStageNode = aDocument.append_child("Stage");
        XMLGen::append_children( { "Name" }, { "Cache State" }, tStageNode);
        auto tParentNode = tStageNode;
        if(tNumCacheStates > 1)
        {
            tParentNode = tStageNode.append_child("Operation");
        }
        for (auto &tService : aXMLMetaData.services())
        {
            if (!tService.cacheState())
            {
                continue;
            }
            auto tOperationNode = tParentNode.append_child("Operation");
            std::vector<std::string> tKeys = { "Name", "PerformerName" };
            std::vector<std::string> tValues = { "Cache State", tService.performer() };
            XMLGen::append_children(tKeys, tValues, tOperationNode);
            for(auto &tOutput : aXMLMetaData.mOutputMetaData)
            {
                auto tOutputService = tOutput.serviceID();
                if(tOutputService == tService.id())
                {
                    if(aXMLMetaData.objective.multi_load_case == "true")
                    {
                        for(size_t i=0; i<aXMLMetaData.objective.scenarioIDs.size(); ++i)
                        {
                            auto tScenarioIndex = std::to_string(i);
                            tKeys = {"ArgumentName", "SharedDataName"};
                            for(auto &tCurData : tOutput.deterministicIDs())
                            {
                                auto tOutputNode = tOperationNode.append_child("Output");
                                tValues = {tCurData + tScenarioIndex, tCurData + "_" + tService.performer() +
                                              "_scenario_" + aXMLMetaData.objective.scenarioIDs[i]};
                                XMLGen::append_children(tKeys, tValues, tOutputNode);
                            }
                        }                        
                    }
                    else
                    {
                        tKeys = {"ArgumentName", "SharedDataName"};
                        for(auto &tCurData : tOutput.deterministicIDs())
                        {
                            auto tOutputNode = tOperationNode.append_child("Output");
                            tValues = {tCurData + "0", tCurData + "_" + tService.performer()};
                            XMLGen::append_children(tKeys, tValues, tOutputNode);
                        }
                    }
                    break;
                }
            }
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_update_problem_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(need_update_problem_stage(aXMLMetaData))
    {
        auto tStageNode = aDocument.append_child("Stage");
        XMLGen::append_children( { "Name" }, { "Update Problem" }, tStageNode);
        for (auto &tService : aXMLMetaData.services())
        {
            if (!tService.updateProblem())
            {
                continue;
            }
            auto tOperationNode = tStageNode.append_child("Operation");
            std::vector<std::string> tKeys = { "Name", "PerformerName" };
            std::vector<std::string> tValues = { "Update Problem", tService.performer() };
            XMLGen::append_children(tKeys, tValues, tOperationNode);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
std::string get_design_variable_name
(const XMLGen::InputData& aXMLMetaData)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        return "";
    }
    else
    {
        if(aXMLMetaData.optimization_parameters().filter_in_engine() == "false")
        {
            return "Control";
        }
    }
    return "Topology";
}
/******************************************************************************/

/******************************************************************************/
std::string get_objective_value_operation_name(XMLGen::Service &aService)
{
    if(aService.code() == "sierra_sd")
    {
        return "Compute Objective";
    }
    else
    {
        return "Compute Objective Value";
    }
    return "";
}
/******************************************************************************/

/******************************************************************************/
std::string get_objective_gradient_operation_name(XMLGen::Service &aService)
{
    if(aService.code() == "sierra_sd")
    {
        return "Compute Gradient";
    }
    else
    {
        return "Compute Objective Gradient";
    }
    return "";
}
/******************************************************************************/

/******************************************************************************/
std::string get_objective_value_operation_output_name(XMLGen::Service &aService)
{
    if(aService.code() == "sierra_sd")
    {
        return "Internal Energy";
    }
    else
    {
        return "Objective Value";
    }
    return "";
}
/******************************************************************************/

/******************************************************************************/
std::string get_objective_gradient_operation_output_name(XMLGen::Service &aService)
{
    if(aService.code() == "sierra_sd")
    {
        return "Internal Energy Gradient";
    }
    else
    {
        return "Objective Gradient";
    }
    return "";
}
/******************************************************************************/
  
/******************************************************************************/
void append_objective_value_operation_for_non_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode,
 bool aCalculatingNormalizationFactor)
{
    XMLGen::Objective tObjective = aXMLMetaData.objective;
    for (size_t i=0; i<tObjective.criteriaIDs.size(); ++i)
    {
        std::string tCriterionID = tObjective.criteriaIDs[i];
        std::string tServiceID = tObjective.serviceIDs[i];
        std::string tScenarioID = tObjective.scenarioIDs[i];
        XMLGen::Service tService = aXMLMetaData.service(tServiceID); 

        auto tOperationNode = aParentNode.append_child("Operation");
        auto tOperationName = get_objective_value_operation_name(tService);
        XMLGen::append_children({"Name", "PerformerName"}, {tOperationName, tService.performer()}, tOperationNode);
        auto tDesignVariableName = get_design_variable_name(aXMLMetaData);
        if(tDesignVariableName != "")
        {
            auto tOperationInput = tOperationNode.append_child("Input");
            XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", tDesignVariableName}, tOperationInput);
        }
        auto tOperationOutput = tOperationNode.append_child("Output");

        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);
        auto tOutputSharedData = std::string("Criterion Value - ") + tIdentifierString;
        if(aCalculatingNormalizationFactor)
        {
            tOutputSharedData = std::string("Initial ") + tOutputSharedData;
        }
        auto tOperationOutputName = get_objective_value_operation_output_name(tService);
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tOperationOutputName, tOutputSharedData}, tOperationOutput);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_objective_value_operation_for_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode,
 bool aCalculatingNormalizationFactor)
{
    XMLGen::Objective tObjective = aXMLMetaData.objective;
    if(tObjective.criteriaIDs.size() > 0)
    {
        std::string tCriterionID = tObjective.criteriaIDs[0];
        std::string tServiceID = tObjective.serviceIDs[0];
        std::string tCompoundScenarioID = get_compound_scenario_id(tObjective.scenarioIDs);
        XMLGen::Service tService = aXMLMetaData.service(tServiceID); 

        auto tOperationNode = aParentNode.append_child("Operation");
        auto tOperationName = get_objective_value_operation_name(tService);
        XMLGen::append_children({"Name", "PerformerName"}, {tOperationName, tService.performer()}, tOperationNode);
        auto tDesignVariableName = get_design_variable_name(aXMLMetaData);
        if(tDesignVariableName != "")
        {
            auto tOperationInput = tOperationNode.append_child("Input");
            XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", tDesignVariableName}, tOperationInput);
        }
        auto tOperationOutput = tOperationNode.append_child("Output");

        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tCompoundScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);
        auto tOutputSharedData = std::string("Criterion Value - ") + tIdentifierString;
        if(aCalculatingNormalizationFactor)
        {
            tOutputSharedData = std::string("Initial ") + tOutputSharedData;
        }
        auto tOperationOutputName = get_objective_value_operation_output_name(tService);
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tOperationOutputName, tOutputSharedData}, tOperationOutput);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_objective_value_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode,
 bool aCalculatingNormalizationFactor)
{
    pugi::xml_node tParentNode = aParentNode;
    XMLGen::Objective tObjective = aXMLMetaData.objective;

    bool tMultiObjective = (tObjective.criteriaIDs.size() > 1 &&
                            tObjective.multi_load_case != "true");

    // If there is more than one sub-objective add an
    // outer "Operation" block so the sub-objectives
    // will be executed in parallel.
    if(tMultiObjective)
        tParentNode = aParentNode.append_child("Operation");
 
    if(tObjective.multi_load_case == "true")
    {
        append_objective_value_operation_for_multi_load_case(aXMLMetaData, tParentNode, aCalculatingNormalizationFactor);
    }
    else
    {
        append_objective_value_operation_for_non_multi_load_case(aXMLMetaData, tParentNode, aCalculatingNormalizationFactor);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_aggregate_objective_value_operation_for_non_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    XMLGen::Objective tObjective = aXMLMetaData.objective;
    for (size_t i=0; i<tObjective.criteriaIDs.size(); ++i)
    {
        std::string tCriterionID = tObjective.criteriaIDs[i];
        std::string tServiceID = tObjective.serviceIDs[i];
        std::string tScenarioID = tObjective.scenarioIDs[i];
        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);
        auto tOperationInput = aParentNode.append_child("Input");
        auto tArgName = std::string("Value ") + std::to_string(i+1);
        auto tOutputSharedData = std::string("Criterion Value - ") + tIdentifierString;
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgName, tOutputSharedData}, tOperationInput);
        if(aXMLMetaData.normalizeInAggregator())
        {
            tOperationInput = aParentNode.append_child("Input");
            tArgName = std::string("Normal ") + std::to_string(i+1);
            tOutputSharedData = std::string("Initial ") + tOutputSharedData;
            XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgName, tOutputSharedData}, tOperationInput);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_aggregate_objective_value_operation_for_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    XMLGen::Objective tObjective = aXMLMetaData.objective;
    if(tObjective.criteriaIDs.size() > 0)
    {
        std::string tCriterionID = tObjective.criteriaIDs[0];
        std::string tServiceID = tObjective.serviceIDs[0];
        std::string tCompoundScenarioID = get_compound_scenario_id(tObjective.scenarioIDs);
        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tCompoundScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);
        auto tOperationInput = aParentNode.append_child("Input");
        auto tOutputSharedData = std::string("Criterion Value - ") + tIdentifierString;
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Value 1", tOutputSharedData}, tOperationInput);
        if(aXMLMetaData.normalizeInAggregator())
        {
            tOperationInput = aParentNode.append_child("Input");
            tOutputSharedData = std::string("Initial ") + tOutputSharedData;
            XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Normal 1", tOutputSharedData}, tOperationInput);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_aggregate_objective_value_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    pugi::xml_node tParentNode = aParentNode;
    XMLGen::Objective tObjective = aXMLMetaData.objective;
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();

    auto tOperationNode = tParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Aggregate Data", tFirstPlatoMainPerformer}, tOperationNode);

    if(tObjective.multi_load_case == "true")
    {
        append_aggregate_objective_value_operation_for_multi_load_case(aXMLMetaData, tOperationNode);
    }
    else
    {
        append_aggregate_objective_value_operation_for_non_multi_load_case(aXMLMetaData, tOperationNode);
    }

    auto tOperationOutput = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Value", "Objective Value"}, tOperationOutput);
}
/******************************************************************************/

/******************************************************************************/
void append_aggregate_objective_gradient_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    pugi::xml_node tParentNode = aParentNode;
    XMLGen::Objective tObjective = aXMLMetaData.objective;
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();

    std::string tType;
    if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
        tType = "Value";
    else
        tType = "Field";

    auto tOperationNode = tParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Aggregate Data", tFirstPlatoMainPerformer}, tOperationNode);
    
    if(tObjective.multi_load_case == "true")
    {
        append_aggregate_objective_gradient_operation_for_multi_load_case(aXMLMetaData, tOperationNode, tType);
    }
    else
    {
        append_aggregate_objective_gradient_operation_for_non_multi_load_case(aXMLMetaData, tOperationNode, tType);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_aggregate_objective_gradient_operation_for_non_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode,
 std::string &aType)
{
    XMLGen::Objective tObjective = aXMLMetaData.objective;
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();

    for (size_t i=0; i<tObjective.criteriaIDs.size(); ++i)
    {
        std::string tCriterionID = tObjective.criteriaIDs[i];
        std::string tServiceID = tObjective.serviceIDs[i];
        std::string tScenarioID = tObjective.scenarioIDs[i];
        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);

        auto tOperationInput = aParentNode.append_child("Input");
        auto tArgName = aType + " " + std::to_string(i+1);
        auto tOutputSharedData = std::string("Criterion Gradient - ") + tIdentifierString;
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgName, tOutputSharedData}, tOperationInput);
        if(aXMLMetaData.normalizeInAggregator())
        {
            tOperationInput = aParentNode.append_child("Input");
            tArgName = std::string("Normal ") + std::to_string(i+1);
            tOutputSharedData = std::string("Initial Criterion Value - ") + tIdentifierString;
            XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgName, tOutputSharedData}, tOperationInput);
        }
    }

    auto tOperationOutput = aParentNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {aType, "Objective Gradient"}, tOperationOutput);
}
/******************************************************************************/

/******************************************************************************/
void append_aggregate_objective_gradient_operation_for_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode,
 std::string &aType)
{
    XMLGen::Objective tObjective = aXMLMetaData.objective;
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();

    if(tObjective.criteriaIDs.size() > 0)
    {
        std::string tCriterionID = tObjective.criteriaIDs[0];
        std::string tServiceID = tObjective.serviceIDs[0];
        std::string tCompoundScenarioID = get_compound_scenario_id(tObjective.scenarioIDs);
        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tCompoundScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);

        auto tOperationInput = aParentNode.append_child("Input");
        std::string tArgName = aType + " 1";
        auto tOutputSharedData = std::string("Criterion Gradient - ") + tIdentifierString;
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgName, tOutputSharedData}, tOperationInput);
        if(aXMLMetaData.normalizeInAggregator())
        {
            tOperationInput = aParentNode.append_child("Input");
            tArgName = "Normal 1";
            tOutputSharedData = std::string("Initial Criterion Value - ") + tIdentifierString;
            XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgName, tOutputSharedData}, tOperationInput);
        }
    }

    auto tOperationOutput = aParentNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {aType, "Objective Gradient"}, tOperationOutput);
}
/******************************************************************************/

/******************************************************************************/
void append_objective_value_stage_for_topology_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tObjective = aXMLMetaData.objective;
    auto tStageNode = aDocument.append_child("Stage");
    XMLGen::append_children( { "Name" }, { "Compute Objective Value" }, tStageNode);
    auto tStageInputNode = tStageNode.append_child("Input");
    XMLGen::append_children( { "SharedDataName" }, { "Control" }, tStageInputNode);
    XMLGen::append_filter_control_operation(aXMLMetaData, tStageNode);
    XMLGen::append_enforce_bounds_operation(aXMLMetaData, tStageNode);

    if(XMLGen::is_robust_optimization_problem(aXMLMetaData))
        XMLGen::append_sample_objective_value_operation(aXMLMetaData, tStageNode);
    else
        XMLGen::append_objective_value_operation(aXMLMetaData, tStageNode, false);

    if(XMLGen::is_robust_optimization_problem(aXMLMetaData))
    {
        auto tObjectiveName = "Objective Value";
        XMLGen::append_evaluate_nondeterministic_objective_value_operation(tObjectiveName, aXMLMetaData, tStageNode);
    }

    if(!XMLGen::is_robust_optimization_problem(aXMLMetaData))
    {
        if(aXMLMetaData.needToAggregate())
        {
            XMLGen::append_aggregate_objective_value_operation(aXMLMetaData, tStageNode);
        }
        else
        {
            std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
            std::string tCriterionID = aXMLMetaData.objective.criteriaIDs[0];
            std::string tServiceID = aXMLMetaData.objective.serviceIDs[0];
            std::string tScenarioID = "";
            if(aXMLMetaData.objective.multi_load_case == "true")
            {
                tScenarioID = get_compound_scenario_id(aXMLMetaData.objective.scenarioIDs);
            }
            else
            {
                tScenarioID = aXMLMetaData.objective.scenarioIDs[0];
            }
            ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
            auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);
            auto tInputValue = "Criterion Value - " + tIdentifierString;

            append_copy_value_operation(tFirstPlatoMainPerformer, tInputValue, "Objective Value", tStageNode);
        }
    }

    auto tStageOutputNode = tStageNode.append_child("Output");
    XMLGen::append_children( { "SharedDataName" }, { "Objective Value" }, tStageOutputNode);
}
/******************************************************************************/

/******************************************************************************/
std::string get_compound_scenario_id(const std::vector<std::string> &aScenarioIDs)
{
    std::string tReturn = "";
    for(auto tScenarioID : aScenarioIDs)
    {
        tReturn += tScenarioID;
    }
    return tReturn;
}
/******************************************************************************/

/******************************************************************************/
void append_objective_value_stage_for_shape_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    auto tObjective = aXMLMetaData.objective;
    auto tStageNode = aDocument.append_child("Stage");
    XMLGen::append_children( { "Name" }, { "Compute Objective Value" }, tStageNode);
    auto tStageInputNode = tStageNode.append_child("Input");
    XMLGen::append_children( { "SharedDataName" }, { "Design Parameters" }, tStageInputNode);
    XMLGen::append_update_geometry_on_change_operation(tFirstPlatoMainPerformer, tStageNode);
    auto tOuterOperationNode = tStageNode.append_child("Operation");
    for (size_t i=0; i<tObjective.criteriaIDs.size(); ++i)
    {
        std::string tCriterionID = tObjective.criteriaIDs[i];
        std::string tServiceID = tObjective.serviceIDs[i];
        std::string tScenarioID = tObjective.scenarioIDs[i];
        XMLGen::Service tService = aXMLMetaData.service(tServiceID); 
        XMLGen::append_reinitialize_on_change_operation(tService.performer(), tOuterOperationNode);
    }

    if(XMLGen::is_robust_optimization_problem(aXMLMetaData))
        XMLGen::append_sample_objective_value_operation(aXMLMetaData, tStageNode);
    else
        XMLGen::append_objective_value_operation(aXMLMetaData, tStageNode, false);


    if(XMLGen::is_robust_optimization_problem(aXMLMetaData))
    {
        auto tObjectiveName = "Objective Value";
        XMLGen::append_evaluate_nondeterministic_objective_value_operation(tObjectiveName, aXMLMetaData, tStageNode);
    }

    if(!XMLGen::is_robust_optimization_problem(aXMLMetaData))
    {
        if(aXMLMetaData.needToAggregate())
        {
            XMLGen::append_aggregate_objective_value_operation(aXMLMetaData, tStageNode);
        }
        else
        {
            std::string tCriterionID = aXMLMetaData.objective.criteriaIDs[0];
            std::string tServiceID = aXMLMetaData.objective.serviceIDs[0];
            std::string tScenarioID = "";
            if(aXMLMetaData.objective.multi_load_case == "true")
            {
                tScenarioID = get_compound_scenario_id(aXMLMetaData.objective.scenarioIDs);
            }
            else
            {
                tScenarioID = aXMLMetaData.objective.scenarioIDs[0];
            }
            ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
            auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);
            auto tInputValue = "Criterion Value - " + tIdentifierString;

            append_copy_value_operation(tFirstPlatoMainPerformer, tInputValue, "Objective Value", tStageNode);
        }
    }

    auto tStageOutputNode = tStageNode.append_child("Output");
    XMLGen::append_children( { "SharedDataName" }, { "Objective Value" }, tStageOutputNode);
}
/******************************************************************************/

/******************************************************************************/
void append_objective_value_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        append_objective_value_stage_for_topology_problem(aXMLMetaData, aDocument);
    }
    else if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        append_objective_value_stage_for_shape_problem(aXMLMetaData, aDocument);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_objective_gradient_operation_for_non_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    XMLGen::Objective tObjective = aXMLMetaData.objective;
    auto tDesignVariableName = get_design_variable_name(aXMLMetaData);
    for (size_t i=0; i<tObjective.criteriaIDs.size(); ++i)
    {
        std::string tCriterionID = tObjective.criteriaIDs[i];
        std::string tServiceID = tObjective.serviceIDs[i];
        std::string tScenarioID = tObjective.scenarioIDs[i];
        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);
        XMLGen::Service tService = aXMLMetaData.service(tObjective.serviceIDs[i]); 
        auto tOperationNode = aParentNode.append_child("Operation");
        auto tOperationName = get_objective_gradient_operation_name(tService);
        XMLGen::append_children({"Name", "PerformerName"}, {tOperationName, tService.performer()}, tOperationNode);
        if(tDesignVariableName != "")
        {
            auto tOperationInput = tOperationNode.append_child("Input");
            XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", tDesignVariableName}, tOperationInput);
            auto tOperationOutput = tOperationNode.append_child("Output");
            auto tOutputSharedData = std::string("Criterion Gradient - ") + tIdentifierString;
            auto tOperationOutputName = get_objective_gradient_operation_output_name(tService);
            XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tOperationOutputName, tOutputSharedData}, tOperationOutput);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_objective_gradient_operation_for_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    XMLGen::Objective tObjective = aXMLMetaData.objective;
    auto tDesignVariableName = get_design_variable_name(aXMLMetaData);
    if(tObjective.scenarioIDs.size() > 0)
    {
        std::string tCriterionID = tObjective.criteriaIDs[0];
        std::string tServiceID = tObjective.serviceIDs[0];
        std::string tScenarioID = get_compound_scenario_id(aXMLMetaData.objective.scenarioIDs);
        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);
        XMLGen::Service tService = aXMLMetaData.service(tServiceID); 
        auto tOperationNode = aParentNode.append_child("Operation");
        auto tOperationName = get_objective_gradient_operation_name(tService);
        XMLGen::append_children({"Name", "PerformerName"}, {tOperationName, tService.performer()}, tOperationNode);
        if(tDesignVariableName != "")
        {
            auto tOperationInput = tOperationNode.append_child("Input");
            XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", tDesignVariableName}, tOperationInput);
            auto tOperationOutput = tOperationNode.append_child("Output");
            auto tOutputSharedData = std::string("Criterion Gradient - ") + tIdentifierString;
            auto tOperationOutputName = get_objective_gradient_operation_output_name(tService);
            XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tOperationOutputName, tOutputSharedData}, tOperationOutput);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_objective_gradient_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    pugi::xml_node tParentNode = aParentNode;
    XMLGen::Objective tObjective = aXMLMetaData.objective;

    bool tMultiObjective = (tObjective.criteriaIDs.size() > 1 &&
                            tObjective.multi_load_case != "true");

    // If there is more than one sub-objective add an
    // outer "Operation" block so the sub-objectives
    // will be executed in parallel.
    if(tMultiObjective)
        tParentNode = aParentNode.append_child("Operation");

    if(tObjective.multi_load_case == "true")
    {
        append_objective_gradient_operation_for_multi_load_case(aXMLMetaData, tParentNode);
    }
    else
    {
        append_objective_gradient_operation_for_non_multi_load_case(aXMLMetaData, tParentNode);
    }
}
/******************************************************************************/

/******************************************************************************/
std::string get_filter_objective_criterion_gradient_input_shared_data_name
(const XMLGen::InputData& aXMLMetaData)
{
    std::string tReturnString = "Objective Gradient";

    if(!XMLGen::is_robust_optimization_problem(aXMLMetaData) &&
       !aXMLMetaData.needToAggregate())
    {
        std::string tCriterionID = aXMLMetaData.objective.criteriaIDs[0];
        std::string tServiceID = aXMLMetaData.objective.serviceIDs[0];
        std::string tScenarioID = "";
        if(aXMLMetaData.objective.multi_load_case == "true")
        {
            tScenarioID = get_compound_scenario_id(aXMLMetaData.objective.scenarioIDs);
        }
        else
        {
            tScenarioID = aXMLMetaData.objective.scenarioIDs[0];
        }
        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);
        tReturnString = "Criterion Gradient - " + tIdentifierString;
    }
    return tReturnString;
}
/******************************************************************************/

/******************************************************************************/
std::string get_filter_constraint_criterion_gradient_input_shared_data_name
(const XMLGen::Constraint &aConstraint)
{
    std::string tCriterionID = aConstraint.criterion();
    std::string tServiceID = aConstraint.service();
    std::string tScenarioID = aConstraint.scenario();
    ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
    auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);

    auto tReturnString = std::string("Criterion Gradient - ") + tIdentifierString;
    return tReturnString;
}
/******************************************************************************/

/******************************************************************************/
void append_objective_gradient_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        append_objective_gradient_stage_for_topology_problem(aXMLMetaData, aDocument);
    }
    else if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        append_objective_gradient_stage_for_shape_problem(aXMLMetaData, aDocument);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_objective_gradient_stage_for_topology_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tObjective = aXMLMetaData.objective;
    auto tStageNode = aDocument.append_child("Stage");
    XMLGen::append_children({"Name"}, {"Compute Objective Gradient"}, tStageNode);
    auto tStageInputNode = tStageNode.append_child("Input");
    XMLGen::append_children({"SharedDataName"}, {"Control"}, tStageInputNode);
    XMLGen::append_filter_control_operation(aXMLMetaData, tStageNode);
    XMLGen::append_enforce_bounds_operation(aXMLMetaData, tStageNode);

    if(XMLGen::is_robust_optimization_problem(aXMLMetaData))
    {
        XMLGen::append_sample_objective_gradient_operation(aXMLMetaData, tStageNode);
        auto tSharedDataName = std::string("Objective Gradient");
        XMLGen::append_evaluate_nondeterministic_objective_gradient_operation(tSharedDataName, aXMLMetaData, tStageNode);
    }
    else
    {
        XMLGen::append_objective_gradient_operation(aXMLMetaData, tStageNode);
        if(aXMLMetaData.needToAggregate())
        {
            XMLGen::append_aggregate_objective_gradient_operation(aXMLMetaData, tStageNode);
        }
    }

    if(aXMLMetaData.optimization_parameters().filter_in_engine() == "true")
    {
        auto tInputMetaDataTag = get_filter_objective_criterion_gradient_input_shared_data_name(aXMLMetaData);
        XMLGen::append_filter_criterion_gradient_operation(aXMLMetaData, tInputMetaDataTag, "Objective Gradient", tStageNode);
    }

    if(aXMLMetaData.optimization_parameters().filter_in_engine() == "false" &&
       !aXMLMetaData.needToAggregate())
    {
        std::string tCriterionID = tObjective.criteriaIDs[0];
        std::string tServiceID = tObjective.serviceIDs[0];
        std::string tScenarioID = tObjective.scenarioIDs[0];
        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);

        std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
        append_copy_field_operation(tFirstPlatoMainPerformer, std::string("Criterion Gradient - ") + tIdentifierString, "Objective Gradient", tStageNode);
    }

    auto tStageOutputNode = tStageNode.append_child("Output");
    XMLGen::append_children({"SharedDataName"}, {"Objective Gradient"}, tStageOutputNode);
}
/******************************************************************************/

/******************************************************************************/
void append_objective_gradient_stage_for_shape_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    auto tObjective = aXMLMetaData.objective;
    auto tStageNode = aDocument.append_child("Stage");
    XMLGen::append_children({"Name"}, {"Compute Objective Gradient"}, tStageNode);
    auto tStageInputNode = tStageNode.append_child("Input");
    XMLGen::append_children({"SharedDataName"}, {"Design Parameters"}, tStageInputNode);
    XMLGen::append_update_geometry_on_change_operation(tFirstPlatoMainPerformer, tStageNode);
    auto tOuterOperationNode = tStageNode.append_child("Operation");
    for (size_t i=0; i<tObjective.criteriaIDs.size(); ++i)
    {
        std::string tCriterionID = tObjective.criteriaIDs[i];
        std::string tServiceID = tObjective.serviceIDs[i];
        std::string tScenarioID = tObjective.scenarioIDs[i];
        XMLGen::Service tService = aXMLMetaData.service(tServiceID); 
        XMLGen::append_reinitialize_on_change_operation(tService.performer(), tOuterOperationNode);
    }

    if(XMLGen::is_robust_optimization_problem(aXMLMetaData))
    {
        XMLGen::append_sample_objective_gradient_operation(aXMLMetaData, tStageNode);
        auto tSharedDataName = std::string("Objective Gradient");
        XMLGen::append_evaluate_nondeterministic_objective_gradient_operation(tSharedDataName, aXMLMetaData, tStageNode);
    }
    else
    {
        tOuterOperationNode = tStageNode.append_child("Operation");
        XMLGen::append_objective_gradient_operation(aXMLMetaData, tOuterOperationNode);
        append_compute_shape_sensitivity_on_change_operation(tOuterOperationNode);
        tOuterOperationNode = tStageNode.append_child("Operation");
        for (size_t i=0; i<tObjective.criteriaIDs.size(); ++i)
        {
            std::string tCriterionID = tObjective.criteriaIDs[i];
            std::string tServiceID = tObjective.serviceIDs[i];
            std::string tScenarioID = tObjective.scenarioIDs[i];
            XMLGen::Service tService = aXMLMetaData.service(tServiceID); 
            ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
            auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);
            auto tSharedDataName = "Criterion Gradient - " + tIdentifierString;
            append_compute_objective_sensitivity_operation(tService.performer(), tSharedDataName, tOuterOperationNode);
        }
        if(aXMLMetaData.needToAggregate())
        {
            XMLGen::append_aggregate_objective_gradient_operation(aXMLMetaData, tStageNode);
        }
    }

    if(!aXMLMetaData.needToAggregate())
    {
        std::string tCriterionID = tObjective.criteriaIDs[0];
        std::string tServiceID = tObjective.serviceIDs[0];
        std::string tScenarioID = "";
        if(aXMLMetaData.objective.multi_load_case == "true")
        {
            tScenarioID = get_compound_scenario_id(aXMLMetaData.objective.scenarioIDs);
        }
        else
        {
            tScenarioID = aXMLMetaData.objective.scenarioIDs[0];
        }
        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);
        auto tSharedDataName = "Criterion Gradient - " + tIdentifierString;
        append_copy_value_operation(tFirstPlatoMainPerformer, tSharedDataName, "Objective Gradient", tStageNode);

    }

    auto tStageOutputNode = tStageNode.append_child("Output");
    XMLGen::append_children({"SharedDataName"}, {"Objective Gradient"}, tStageOutputNode);
}
/******************************************************************************/

/******************************************************************************/
void append_objective_gradient_to_plato_main_output_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tInput = aParentNode.append_child("Input");
    auto tSharedDataName = std::string("Objective Gradient");
    auto tArgumentName  = XMLGen::to_lower(tSharedDataName);
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgumentName, tSharedDataName}, tInput);
}
// function append_objective_gradient_to_plato_main_output_stage
/******************************************************************************/

/******************************************************************************/
void append_constraint_gradient_to_plato_main_output_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tInput = aParentNode.append_child("Input");
        auto tSharedDataName = std::string("Constraint Gradient ") + tConstraint.id();
        auto tArgumentName  = XMLGen::to_lower(tSharedDataName);
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgumentName, tSharedDataName}, tInput);
    }
}
// function append_constraint_gradient_to_plato_main_output_stage
/******************************************************************************/

/******************************************************************************/
inline void append_qoi_to_random_write_output_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::ValidPerformerOutputKeys tValidKeys;
    const XMLGen::Output &tOutputMetadata = aMetaData.mOutputMetaData[0];
    auto tServiceID = tOutputMetadata.serviceID();
    auto tCodeName = aMetaData.service(tServiceID).code();
    auto tOutputQoIs = tOutputMetadata.randomIDs();
    for(auto& tQoI : tOutputQoIs)
    {
        auto tOutput = aParentNode.append_child("Output");
        auto tArgumentName = tValidKeys.argument(tCodeName, tQoI);
        auto tSharedDataName = tOutputMetadata.randomSharedDataName(tQoI);
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgumentName, tSharedDataName}, tOutput);
    }
}
// function append_random_write_output_operation
/******************************************************************************/

/******************************************************************************/
inline void append_random_write_output_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_node& aParentNode)
{
    const XMLGen::Output &tOutputMetadata = aMetaData.mOutputMetaData[0];
    if(tOutputMetadata.randomIDs().empty())
    {
        return;
    }

    auto tServiceID = tOutputMetadata.serviceID();
    auto &tService = aMetaData.service(tServiceID);
    if(tService.code() == "plato_analyze")
    {
        auto tForNode = aParentNode.append_child("For");
        XMLGen::append_attributes( { "var", "in" }, { "PerformerSampleIndex", "PerformerSamples" }, tForNode);
        auto tOperationNode = tForNode.append_child("Operation");
        tForNode = tOperationNode.append_child("For");
        XMLGen::append_attributes( { "var", "in" }, { "PerformerIndex", "Performers" }, tForNode);

        tOperationNode = tForNode.append_child("Operation");
        tServiceID = tOutputMetadata.serviceID();
        auto tBasePerformerName = aMetaData.service(tServiceID).performer();
        auto tPerformerName = tBasePerformerName + "_{PerformerIndex}";
        XMLGen::append_children( { "Name", "PerformerName" }, { "Write Output", tPerformerName }, tOperationNode);
        XMLGen::append_qoi_to_random_write_output_operation(aMetaData, tOperationNode);
    }
}
// function append_random_qoi_outputs
/******************************************************************************/

/******************************************************************************/
inline void append_qoi_to_deterministic_write_output_operation
(const XMLGen::InputData& aMetaData,
 const XMLGen::Output& aOutputMetadata,
 pugi::xml_node& aParentNode)
{
    XMLGen::ValidPerformerOutputKeys tValidKeys;
    auto tServiceID = aOutputMetadata.serviceID();
    auto tCodeName = aMetaData.service(tServiceID).code();
    auto tPerformerName = aMetaData.service(tServiceID).performer();
    auto tOutputQoIs = aOutputMetadata.deterministicIDs();
    for(auto& tQoI : tOutputQoIs)
    {
        auto tOutput = aParentNode.append_child("Output");
        auto tArgumentName = tValidKeys.argument(tCodeName, tQoI);
        auto tSharedDataName = aOutputMetadata.deterministicSharedDataName(tQoI);
        tSharedDataName += std::string("_") + tPerformerName;
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgumentName, tSharedDataName}, tOutput);
    }
}
// function append_deterministic_qoi_outputs

/******************************************************************************/
inline void append_deterministic_write_output_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_node& aParentNode)
{
    int tNumPlatoAnalyzeOutputs = 0;
    for(auto &tCurOutput : aMetaData.mOutputMetaData)
    {
        auto tServiceID = tCurOutput.serviceID();
        auto &tService = aMetaData.service(tServiceID);
        if(tService.code() == "plato_analyze")
        {
            tNumPlatoAnalyzeOutputs++;
        }
    }
    pugi::xml_node tCurParentNode = aParentNode;
    if(tNumPlatoAnalyzeOutputs > 1)
    {
        tCurParentNode = aParentNode.append_child("Operation");
    }

    for(auto &tOutputMetadata : aMetaData.mOutputMetaData)
    {
        auto tServiceID = tOutputMetadata.serviceID();
        auto &tService = aMetaData.service(tServiceID);
        if(tService.code() == "plato_analyze")
        {
            if(!tOutputMetadata.deterministicIDs().empty())
            {
                auto tOperationNode = tCurParentNode.append_child("Operation");
                auto tPerformerName = aMetaData.service(tServiceID).performer();
                XMLGen::append_children( { "Name", "PerformerName" }, { "Write Output", tPerformerName }, tOperationNode);
                if(aMetaData.optimization_parameters().filter_in_engine() != "true")
                {
                    auto tTopologyNode = tOperationNode.append_child("Output");
                    XMLGen::append_children( { "ArgumentName", "SharedDataName" }, { "Topology", "Topology"}, tTopologyNode);
                }

                XMLGen::append_qoi_to_deterministic_write_output_operation(aMetaData, tOutputMetadata, tOperationNode);
            }
        }
    }
}
// function append_random_qoi_outputs
/******************************************************************************/

/******************************************************************************/
void append_write_ouput_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_node& aParentNode)
{
    if(aMetaData.mOutputMetaData[0].isOutputDisabled())
    {
        return;
    }
    XMLGen::append_random_write_output_operation(aMetaData, aParentNode);
    XMLGen::append_deterministic_write_output_operation(aMetaData, aParentNode);
}
// function append_write_ouput_operation
/******************************************************************************/

/******************************************************************************/
inline void append_default_qoi_to_plato_main_output_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tInput = aParentNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"topology", "Topology"}, tInput);
    tInput = aParentNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"control", "Control"}, tInput);
    XMLGen::append_objective_gradient_to_plato_main_output_stage(aXMLMetaData, aParentNode);
    XMLGen::append_constraint_gradient_to_plato_main_output_stage(aXMLMetaData, aParentNode);
}
// function append_default_qoi_to_plato_main_output_stage
/******************************************************************************/

/******************************************************************************/
inline void append_random_qoi_samples_to_plato_main_output_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    const XMLGen::Output &tOutputMetadata = aXMLMetaData.mOutputMetaData[0];
    if(tOutputMetadata.outputSamples() == false)
    {
        return;
    }

    auto tQoIIDs = tOutputMetadata.randomIDs();
    for(auto& tID : tQoIIDs)
    {
        auto tOuterFor = aParentNode.append_child("For");
        XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
        auto tInnerFor = tOuterFor.append_child("For");
        XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
        auto tInput = tInnerFor.append_child("Input");
        auto tArgumentName = tOutputMetadata.randomArgumentName(tID);
        auto tSharedDataName = tOutputMetadata.randomSharedDataName(tID);
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgumentName, tSharedDataName}, tInput);
    }
}
// function append_random_qoi_samples_to_plato_main_output_stage
/******************************************************************************/

/******************************************************************************/
inline void append_random_qoi_statistics_to_plato_main_output_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tQoIIDs = aXMLMetaData.mOutputMetaData[0].randomIDs();
    for (auto &tID : tQoIIDs)
    {
        auto tMeanName = tID + " mean";
        auto tInput = aParentNode.append_child("Input");
        XMLGen::append_children( { "ArgumentName", "SharedDataName" }, { tMeanName, tMeanName }, tInput);

        auto tStdDevName = tID + " standard deviation";
        tInput = aParentNode.append_child("Input");
        XMLGen::append_children( { "ArgumentName", "SharedDataName" }, { tStdDevName, tStdDevName }, tInput);
    }
}
// function append_random_qoi_statistics_to_plato_main_output_stage
/******************************************************************************/

/******************************************************************************/
void append_deterministic_qoi_to_plato_main_output_stage_for_non_multi_load_case 
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    for(auto &tOutputMetaData : aXMLMetaData.mOutputMetaData)
    {
        auto tQoIIDs = tOutputMetaData.deterministicIDs();
        for(auto& tID : tQoIIDs)
        {
            auto tInput = aParentNode.append_child("Input");
            auto tArgumentName = tOutputMetaData.deterministicArgumentName(tID);
            auto tSharedDataName = tOutputMetaData.deterministicSharedDataName(tID);
            auto tOwnerString = std::string("_") + aXMLMetaData.service(tOutputMetaData.serviceID()).performer();
            tSharedDataName += tOwnerString;
            tArgumentName += tOwnerString;
            XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgumentName, tSharedDataName}, tInput);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_deterministic_qoi_to_plato_main_output_stage_for_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    for(auto &tOutputMetaData : aXMLMetaData.mOutputMetaData)
    {
        auto tQoIIDs = tOutputMetaData.deterministicIDs();
        for(auto& tID : tQoIIDs)
        {
            for(auto tScenarioID : aXMLMetaData.objective.scenarioIDs)
            {
                auto tInput = aParentNode.append_child("Input");
                auto tArgumentName = tOutputMetaData.deterministicArgumentName(tID);
                auto tSharedDataName = tOutputMetaData.deterministicSharedDataName(tID);
                auto tOwnerString = std::string("_") + aXMLMetaData.service(tOutputMetaData.serviceID()).performer();
                tSharedDataName += tOwnerString;
                tSharedDataName += "_scenario_" + tScenarioID;
                tArgumentName += tOwnerString;
                tArgumentName += "_scenario_" + tScenarioID;
                XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgumentName, tSharedDataName}, tInput);
            }
        }
    }
}
/******************************************************************************/

/******************************************************************************/
inline void append_deterministic_qoi_to_plato_main_output_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.objective.multi_load_case == "true")
    {
        append_deterministic_qoi_to_plato_main_output_stage_for_multi_load_case(aXMLMetaData, aParentNode);
    }
    else
    {
        append_deterministic_qoi_to_plato_main_output_stage_for_non_multi_load_case(aXMLMetaData, aParentNode);
    }
}
// function append_deterministic_qoi_to_plato_main_output_stage
/******************************************************************************/

/******************************************************************************/
inline void append_platomain_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    auto tOperation = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"},{"PlatoMainOutput", tFirstPlatoMainPerformer}, tOperation);
    XMLGen::append_default_qoi_to_plato_main_output_stage(aXMLMetaData, tOperation);
    XMLGen::append_deterministic_qoi_to_plato_main_output_stage(aXMLMetaData, tOperation);
    XMLGen::append_random_qoi_samples_to_plato_main_output_stage(aXMLMetaData, tOperation);
    XMLGen::append_random_qoi_statistics_to_plato_main_output_stage(aXMLMetaData, tOperation);
}
/******************************************************************************/

/******************************************************************************/
void append_plato_main_output_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.mOutputMetaData[0].isOutputDisabled())
    {
        return;
    }
    if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        auto tOutputStage = aDocument.append_child("Stage");
        XMLGen::append_children({"Name"}, {"Output To File"}, tOutputStage);
        XMLGen::append_write_ouput_operation(aXMLMetaData, tOutputStage);
        XMLGen::append_compute_qoi_statistics_operation(aXMLMetaData, tOutputStage);
        XMLGen::append_platomain_output_operation(aXMLMetaData, tOutputStage);
    }
    else if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
        auto tOutputStage = aDocument.append_child("Stage");
        XMLGen::append_children({"Name"}, {"Output To File"}, tOutputStage);
        auto tOperation = tOutputStage.append_child("Operation");
        XMLGen::append_children({"Name", "PerformerName"}, {"CSMMeshOutput", tFirstPlatoMainPerformer}, tOperation);
    }
}
// function append_plato_main_output_stage
/******************************************************************************/

/******************************************************************************/
void append_lower_bounds_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    std::string tFirstPlatoMainPerformer = aMetaData.getFirstPlatoMainPerformer();
    // shared data - lower bound value
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Lower Bound Value", "Scalar", "Global", "1", tFirstPlatoMainPerformer, tFirstPlatoMainPerformer};
    auto tSharedDataNode = aDocument.append_child("SharedData");
    XMLGen::append_children(tKeys, tValues, tSharedDataNode);

    // shared data - lower bound vector
    if(aMetaData.optimization_parameters().optimization_type() == "topology")
    {
        tValues = {"Lower Bound Vector", "Scalar", "Nodal Field", "IGNORE", tFirstPlatoMainPerformer, tFirstPlatoMainPerformer};
        tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);
    }
    else if(aMetaData.optimization_parameters().optimization_type() == "shape")
    {
        tValues = {"Lower Bound Vector", "Scalar", "Global", aMetaData.optimization_parameters().num_shape_design_variables(), tFirstPlatoMainPerformer, tFirstPlatoMainPerformer};
        tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);
    }
}
// function append_lower_bounds_shared_data
/******************************************************************************/

/******************************************************************************/
void append_upper_bounds_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    std::string tFirstPlatoMainPerformer = aMetaData.getFirstPlatoMainPerformer();
    // shared data - upper bound value
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Upper Bound Value", "Scalar", "Global", "1", tFirstPlatoMainPerformer, tFirstPlatoMainPerformer};
    auto tSharedDataNode = aDocument.append_child("SharedData");
    XMLGen::append_children(tKeys, tValues, tSharedDataNode);

    // shared data - upper bound vector
    if(aMetaData.optimization_parameters().optimization_type() == "topology")
    {
        tValues = {"Upper Bound Vector", "Scalar", "Nodal Field", "IGNORE", tFirstPlatoMainPerformer, tFirstPlatoMainPerformer};
        tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);
    }
    else if(aMetaData.optimization_parameters().optimization_type() == "shape")
    {
        tValues = {"Upper Bound Vector", "Scalar", "Global", aMetaData.optimization_parameters().num_shape_design_variables(), tFirstPlatoMainPerformer, tFirstPlatoMainPerformer};
        tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);
    }
}
// function append_upper_bounds_shared_data
/******************************************************************************/

/******************************************************************************/
void append_design_volume_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    if(aMetaData.optimization_parameters().optimization_type() == "topology")
    {
        std::string tFirstPlatoMainPerformer = aMetaData.getFirstPlatoMainPerformer();
        std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
        std::vector<std::string> tValues = {"Design Volume", "Scalar", "Global", "1", tFirstPlatoMainPerformer, tFirstPlatoMainPerformer};
        auto tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);
    }
}
// function append_design_volume_shared_data
/******************************************************************************/

/******************************************************************************/
void append_parameter_sensitivity_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    if(aMetaData.optimization_parameters().optimization_type() == "shape")
    {
        std::string tFirstPlatoMainPerformer = aMetaData.getFirstPlatoMainPerformer();
        auto tForNode = aDocument.append_child("For");
        tForNode.append_attribute("var") = "I";
        tForNode.append_attribute("in") = "Parameters";
        auto tTmpNode = tForNode.append_child("SharedData");
        addChild(tTmpNode, "Name", "Parameter Sensitivity {I}");
        addChild(tTmpNode, "Type", "Scalar");
        addChild(tTmpNode, "Layout", "Global");
        addChild(tTmpNode, "Dynamic", "true");
        addChild(tTmpNode, "OwnerName", "plato_esp_{I}");
        addChild(tTmpNode, "UserName", tFirstPlatoMainPerformer);
        for(auto& tService : aMetaData.mPerformerServices)
        {
            if(tService.code() != "plato_esp")
            {
                addChild(tTmpNode, "UserName", tService.performer());
            }
        }
    }
}
// function append_parameter_sensitivity_shared_data
/******************************************************************************/

/******************************************************************************/
    void append_criteria_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();

    auto tConcretizedCriteria = aXMLMetaData.getConcretizedCriteria();

    for(auto& tCriterion : tConcretizedCriteria)
    {      
        std::string tCriterionID = std::get<0>(tCriterion);
        std::string tServiceID = std::get<1>(tCriterion);
        std::string tScenarioID = std::get<2>(tCriterion);
        std::string tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tCriterion);

        XMLGen::Service tService = aXMLMetaData.service(tServiceID);
        std::string tOwnerName = tService.performer();

        // shared data - deterministic criterion value
        std::string tTag = std::string("Criterion Value - ") + tIdentifierString;
        std::vector<std::string> tKeys = { "Name", "Type", "Layout", "Size", "OwnerName", "UserName" };
        std::vector<std::string> tValues = { tTag, "Scalar", "Global", "1", tOwnerName, tFirstPlatoMainPerformer };
        auto tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);

        // shared data - deterministic criterion gradient
        if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
        {
            tTag = std::string("Criterion Gradient - ") + tIdentifierString;
            tValues = { tTag, "Scalar", "Nodal Field", "IGNORE", tOwnerName, tFirstPlatoMainPerformer };
            tSharedDataNode = aDocument.append_child("SharedData");
            XMLGen::append_children(tKeys, tValues, tSharedDataNode);
        }
        else if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
        {
            tTag = std::string("Criterion Gradient - ") + tIdentifierString;
            tValues = { tTag, "Scalar", "Global", aXMLMetaData.optimization_parameters().num_shape_design_variables(), tOwnerName, tFirstPlatoMainPerformer };
            tSharedDataNode = aDocument.append_child("SharedData");
            XMLGen::append_children(tKeys, tValues, tSharedDataNode);
        }
    }
}
// function append_criteria_shared_data
/******************************************************************************/

/******************************************************************************/
void append_constraint_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    auto tConstraints = aXMLMetaData.constraints;

    for(auto& tConstraint : tConstraints)
    {
        std::string tTag = std::string("Constraint Value ") + tConstraint.id();
        std::vector<std::string> tKeys = { "Name", "Type", "Layout", "Size", "OwnerName", "UserName" };
        std::vector<std::string> tValues = {tTag, "Scalar", "Global", "1", tFirstPlatoMainPerformer, tFirstPlatoMainPerformer };
        auto tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);

        if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
        {
            tTag = std::string("Constraint Gradient ") + tConstraint.id();
            tValues = { tTag, "Scalar", "Nodal Field", "IGNORE", tFirstPlatoMainPerformer, tFirstPlatoMainPerformer };
            tSharedDataNode = aDocument.append_child("SharedData");
            XMLGen::append_children(tKeys, tValues, tSharedDataNode);
        }
        else if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
        {
            tTag = std::string("Constraint Gradient ") + tConstraint.id();
            tValues = { tTag, "Scalar", "Global", aXMLMetaData.optimization_parameters().num_shape_design_variables(), tFirstPlatoMainPerformer, tFirstPlatoMainPerformer };
            tSharedDataNode = aDocument.append_child("SharedData");
            XMLGen::append_children(tKeys, tValues, tSharedDataNode);
        }
    }
}
// function append_constraint_shared_data
/******************************************************************************/

/******************************************************************************/
void append_objective_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();

    std::vector<std::string> tKeys = { "Name", "Type", "Layout", "Size", "OwnerName", "UserName" };
    std::vector<std::string> tValues = {"Objective Value", "Scalar", "Global", "1", tFirstPlatoMainPerformer, tFirstPlatoMainPerformer };
    auto tSharedDataNode = aDocument.append_child("SharedData");
    XMLGen::append_children(tKeys, tValues, tSharedDataNode);

    if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        tValues = {"Objective Gradient", "Scalar", "Nodal Field", "IGNORE", tFirstPlatoMainPerformer, tFirstPlatoMainPerformer };
        tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);
    }
    else if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        tValues = {"Objective Gradient", "Scalar", "Global", aXMLMetaData.optimization_parameters().num_shape_design_variables(), tFirstPlatoMainPerformer, tFirstPlatoMainPerformer };
        tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);
    }
}
// function append_objective_shared_data
/******************************************************************************/

/******************************************************************************/
void append_normalization_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    bool tNormalizeInAggregator = aXMLMetaData.normalizeInAggregator();
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();

    if(tNormalizeInAggregator)
    {
        bool tMultiLoadCase = (aXMLMetaData.objective.multi_load_case == "true");
        int tNumEntries = aXMLMetaData.objective.criteriaIDs.size();
        if(tMultiLoadCase)
        {
            tNumEntries = 1;
        }
        for(int i=0; i<tNumEntries; ++i)
        {
            std::string tCriterionID = aXMLMetaData.objective.criteriaIDs[i];
            std::string tServiceID = aXMLMetaData.objective.serviceIDs[i];
            std::string tScenarioID = aXMLMetaData.objective.scenarioIDs[i];
            if(tMultiLoadCase)
            {
                tScenarioID = get_compound_scenario_id(aXMLMetaData.objective.scenarioIDs);
            }
            ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
            auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);
            auto tValueNameString = "Initial Criterion Value - " + tIdentifierString;
            auto &tService = aXMLMetaData.service(tServiceID);
            auto tOwnerName = tService.performer();

            std::vector<std::string> tKeys = { "Name", "Type", "Layout", "Size", "OwnerName", "UserName" };
            std::vector<std::string> tValues = {tValueNameString, "Scalar", "Global", "1", tOwnerName, tFirstPlatoMainPerformer };
            auto tSharedDataNode = aDocument.append_child("SharedData");
            XMLGen::append_children(tKeys, tValues, tSharedDataNode);
        }
    }
}
// function append_normalization_shared_data
/******************************************************************************/

/******************************************************************************/
void append_design_variables_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    if(aMetaData.optimization_parameters().optimization_type() == "topology")
    {
        XMLGen::append_control_shared_data(aMetaData, aDocument);
    }
    else if(aMetaData.optimization_parameters().optimization_type() == "shape")
    {
        XMLGen::append_design_parameters_shared_data(aMetaData, aDocument);
    }
}
// function append_design_variables_shared_data
/******************************************************************************/

/******************************************************************************/
void append_design_parameters_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    std::string tFirstPlatoMainPerformer = aMetaData.getFirstPlatoMainPerformer();
    auto tTmpNode = aDocument.append_child("SharedData");
    addChild(tTmpNode, "Name", "Design Parameters");
    addChild(tTmpNode, "Type", "Scalar");
    addChild(tTmpNode, "Layout", "Global");
    addChild(tTmpNode, "Size", aMetaData.optimization_parameters().num_shape_design_variables());
    addChild(tTmpNode, "OwnerName", tFirstPlatoMainPerformer);
    addChild(tTmpNode, "UserName", tFirstPlatoMainPerformer);
    for(auto& tService : aMetaData.mPerformerServices)
    {
        if(tService.code() != "plato_esp")
        {
            addChild(tTmpNode, "UserName", tService.performer());
        }
    }
}
// append_design_parameters_shared_data append_control_shared_data
/******************************************************************************/

/******************************************************************************/
void append_control_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    std::string tFirstPlatoMainPerformer = aMetaData.getFirstPlatoMainPerformer();
    auto tSharedData = aDocument.append_child("SharedData");
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Control", "Scalar", "Nodal Field", "IGNORE", tFirstPlatoMainPerformer, tFirstPlatoMainPerformer};
    XMLGen::append_children(tKeys, tValues, tSharedData);
    if(aMetaData.optimization_parameters().filter_in_engine() == "false")
    {
        for(auto& tService : aMetaData.services())
        {
            if(tService.code() == "plato_analyze")
            {
                tKeys = {"UserName"};
                tValues = {tService.performer()};
                XMLGen::append_children(tKeys, tValues, tSharedData);
            }
        }
    }
}
// function append_control_shared_data
/******************************************************************************/

/******************************************************************************/
void append_plato_main_performer
(const XMLGen::InputData& aXMLMetaData,
 int &aNextPerformerID,
 pugi::xml_node& aNode)
{
    // The platomain optimizer should always be the first service in the list.
    const XMLGen::Service &tService = aXMLMetaData.service(0);
    auto tPerformerNode = aNode.append_child("Performer");
    XMLGen::append_children( {"Name", "Code", "PerformerID"}, {tService.performer(), tService.code(), std::to_string(aNextPerformerID)}, tPerformerNode);
    aNextPerformerID++;
}
// function append_plato_main_performer
/******************************************************************************/

/******************************************************************************/
void append_filter_control_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.optimization_parameters().filter_in_engine() != "false")
    {
        std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
        auto tOperationNode = aParentNode.append_child("Operation");
        XMLGen::append_children({"Name", "PerformerName"},{"Filter Control", tFirstPlatoMainPerformer}, tOperationNode);
        auto tInputNode = tOperationNode.append_child("Input");
        XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Field", "Control"}, tInputNode);
        auto tOutputNode = tOperationNode.append_child("Output");
        XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Filtered Field", "Topology"}, tOutputNode);
    }
}
// function append_filter_control_operation
/******************************************************************************/

/******************************************************************************/
void append_enforce_bounds_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.optimization_parameters().enforce_bounds() == "true")
    {
        std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
        auto tOperationNode = aParentNode.append_child("Operation");
        XMLGen::append_children({"Name", "PerformerName"},{"EnforceBounds", tFirstPlatoMainPerformer}, tOperationNode);
        auto tInputNode = tOperationNode.append_child("Input");
        XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Lower Bound Vector", "Lower Bound Vector"}, tInputNode);
        tInputNode = tOperationNode.append_child("Input");
        XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Upper Bound Vector", "Upper Bound Vector"}, tInputNode);
        tInputNode = tOperationNode.append_child("Input");
        XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Topology", "Topology"}, tInputNode);
        auto tOutputNode = tOperationNode.append_child("Output");
        XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Topology", "Topology"}, tOutputNode);
    }
}
// function append_enforce_bounds_operation
/******************************************************************************/

/******************************************************************************/
void append_filter_criterion_gradient_operation
(const XMLGen::InputData& aXMLMetaData,
 const std::string& aInputSharedDataName,
 const std::string& aOutputSharedDataName,
 pugi::xml_node& aParentNode)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"},{"Filter Gradient", tFirstPlatoMainPerformer}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Field", "Control"}, tInputNode);
    tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Gradient", aInputSharedDataName}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Filtered Gradient", aOutputSharedDataName}, tOutputNode);
}
// function append_filter_criterion_gradient_operation
/******************************************************************************/

/******************************************************************************/
void append_initial_field_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"},{"Initialize Field", tFirstPlatoMainPerformer}, tOperationNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Initialized Field", "Control"}, tOutputNode);
}
// function append_initial_field_operation
/******************************************************************************/

/******************************************************************************/
void append_compute_normalization_factor_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.normalizeInAggregator())
    {
        XMLGen::append_filter_control_operation(aXMLMetaData, aParentNode);
        XMLGen::append_enforce_bounds_operation(aXMLMetaData, aParentNode);
        XMLGen::append_objective_value_operation(aXMLMetaData, aParentNode, true);
    }
}
// function append_compute_normalization_factor_operation
/******************************************************************************/

/******************************************************************************/
void append_initial_guess_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        auto tStageNode = aDocument.append_child("Stage");
        XMLGen::append_children({"Name"},{"Initial Guess"}, tStageNode);
        XMLGen::append_initial_field_operation(aXMLMetaData, tStageNode);
        XMLGen::append_compute_normalization_factor_operation(aXMLMetaData, tStageNode);
        auto tOutputNode = tStageNode.append_child("Output");
        XMLGen::append_children({"SharedDataName"},{"Control"}, tOutputNode);
    }
    else if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        auto tStageNode = aDocument.append_child("Stage");
        XMLGen::append_children({"Name"},{"Initialize Design Parameters"}, tStageNode);
        XMLGen::append_initial_values_operation(aXMLMetaData, tStageNode);
        XMLGen::append_initialize_geometry_operation(aXMLMetaData, tStageNode);
        auto tOutputNode = tStageNode.append_child("Output");
        XMLGen::append_children({"SharedDataName"},{"Design Parameters"}, tOutputNode);
    }
}
// function append_initial_guess_stage
/******************************************************************************/

/******************************************************************************/
void append_initialize_geometry_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"},{"Initialize Geometry", tFirstPlatoMainPerformer}, tOperationNode);
}
// function append_initialize_geometry_operation
/******************************************************************************/

/******************************************************************************/
void append_initial_values_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"},{"Initialize Values", tFirstPlatoMainPerformer}, tOperationNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Values", "Design Parameters"}, tOutputNode);
    tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Lower Bounds", "Lower Bound Vector"}, tOutputNode);
    tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Upper Bounds", "Upper Bound Vector"}, tOutputNode);
}
// function append_initial_values_operation
/******************************************************************************/

/******************************************************************************/
void append_lower_bound_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Lower Bounds", tFirstPlatoMainPerformer}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Lower Bound Value", "Lower Bound Value"}, tInputNode);
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
    auto tLower = Plato::tolower(aXMLMetaData.optimization_parameters().optimization_type());
    if(tLower.compare("topology") == 0)
    {
        auto tInputNode = tStageNode.append_child("Input");
        XMLGen::append_children({"SharedDataName"}, {"Lower Bound Value"}, tInputNode);
        XMLGen::append_lower_bound_operation(aXMLMetaData, tStageNode);
    }
    auto tOutputNode = tStageNode.append_child("Output");
    XMLGen::append_children({"SharedDataName"}, {"Lower Bound Vector"}, tOutputNode);
}
// function append_lower_bound_stage
/******************************************************************************/

/******************************************************************************/
void append_upper_bound_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Upper Bounds", tFirstPlatoMainPerformer}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Upper Bound Value", "Upper Bound Value"}, tInputNode);
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
    auto tLower = Plato::tolower(aXMLMetaData.optimization_parameters().optimization_type());
    if(tLower.compare("topology") == 0)
    {
        auto tInputNode = tStageNode.append_child("Input");
        XMLGen::append_children({"SharedDataName"}, {"Upper Bound Value"}, tInputNode);
        XMLGen::append_upper_bound_operation(aXMLMetaData, tStageNode);
    }
    auto tOutputNode = tStageNode.append_child("Output");
    XMLGen::append_children({"SharedDataName"}, {"Upper Bound Vector"}, tOutputNode);
}
// function append_upper_bound_stage
/******************************************************************************/

/******************************************************************************/
void append_design_volume_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Design Volume", tFirstPlatoMainPerformer}, tOperationNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Design Volume", "Design Volume"}, tOutputNode);
}
// function append_design_volume_operation
/******************************************************************************/

/******************************************************************************/
void append_design_volume_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        auto tStageNode = aDocument.append_child("Stage");
        XMLGen::append_children({"Name"}, {"Design Volume"}, tStageNode);
        XMLGen::append_design_volume_operation(aXMLMetaData, tStageNode);
        auto tOutputNode = tStageNode.append_child("Output");
        XMLGen::append_children({"SharedDataName"}, {"Design Volume"}, tOutputNode);
    }
}
// function append_design_volume_stage
/******************************************************************************/

/******************************************************************************/
void append_constraint_value_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        append_constraint_stage_for_topology_problem(aXMLMetaData, aDocument);
    }
    else if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        append_constraint_stage_for_shape_problem(aXMLMetaData, aDocument);
    }
}
// function append_constraint_value_stage
/******************************************************************************/

/******************************************************************************/
void append_constraint_stage_for_topology_problem 
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tDesignVariableName = get_design_variable_name(aXMLMetaData);
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    XMLGen::ConstraintValueOperation tValueOperationInterface;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        std::string tCriterionID = tConstraint.criterion();
        std::string tServiceID = tConstraint.service();
        std::string tScenarioID = tConstraint.scenario();
        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);

        auto tStageNode = aDocument.append_child("Stage");
        auto tStageName = std::string("Compute Constraint Value ") + tConstraint.id();
        XMLGen::append_children({"Name"}, {tStageName}, tStageNode);
        auto tInputNode = tStageNode.append_child("Input");
        XMLGen::append_children({"SharedDataName"}, {"Control"}, tInputNode);

        auto tService = aXMLMetaData.service(tConstraint.service()); 
        XMLGen::append_filter_control_operation(aXMLMetaData, tStageNode);
        XMLGen::append_enforce_bounds_operation(aXMLMetaData, tStageNode);
        tValueOperationInterface.call(tConstraint, tService.performer(), tDesignVariableName, tService.code(), tStageNode);

        append_copy_value_operation(tFirstPlatoMainPerformer, std::string("Criterion Value - ") + tIdentifierString, std::string("Constraint Value ") + tConstraint.id(), tStageNode);

        auto tOutputNode = tStageNode.append_child("Output");
        auto tSharedDataName = std::string("Constraint Value ") + tConstraint.id();
        XMLGen::append_children({"SharedDataName"}, {tSharedDataName}, tOutputNode);
    }
}
// function append_constraint_stage_for_topology_problem
/******************************************************************************/

/******************************************************************************/
void append_update_geometry_on_change_operation 
(const std::string &aFirstPlatoMainPerformer, 
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Update Geometry on Change", aFirstPlatoMainPerformer}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"SharedDataName", "ArgumentName"}, {"Design Parameters", "Parameters"}, tInputNode);
}
/******************************************************************************/

/******************************************************************************/
void append_reinitialize_on_change_operation 
(const std::string &aPerformer, 
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Reinitialize on Change", aPerformer}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"SharedDataName", "ArgumentName"}, {"Design Parameters", "Parameters"}, tInputNode);
}
/******************************************************************************/

/******************************************************************************/
void append_constraint_stage_for_shape_problem 
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tDesignVariableName = get_design_variable_name(aXMLMetaData);
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    XMLGen::ConstraintValueOperation tValueOperationInterface;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        std::string tCriterionID = tConstraint.criterion();
        std::string tServiceID = tConstraint.service();
        std::string tScenarioID = tConstraint.scenario();
        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);

        auto tStageNode = aDocument.append_child("Stage");
        auto tStageName = std::string("Compute Constraint Value ") + tConstraint.id();
        XMLGen::append_children({"Name"}, {tStageName}, tStageNode);
        auto tInputNode = tStageNode.append_child("Input");
        XMLGen::append_children({"SharedDataName"}, {"Design Parameters"}, tInputNode);

        auto tService = aXMLMetaData.service(tConstraint.service()); 
        XMLGen::append_update_geometry_on_change_operation(tFirstPlatoMainPerformer, tStageNode);
        XMLGen::append_reinitialize_on_change_operation(tService.performer(), tStageNode);
        tValueOperationInterface.call(tConstraint, tService.performer(), tDesignVariableName, tService.code(), tStageNode);

        append_copy_value_operation(tFirstPlatoMainPerformer, std::string("Criterion Value - ") + tIdentifierString, std::string("Constraint Value ") + tConstraint.id(), tStageNode);

        auto tOutputNode = tStageNode.append_child("Output");
        auto tSharedDataName = std::string("Constraint Value ") + tConstraint.id();
        XMLGen::append_children({"SharedDataName"}, {tSharedDataName}, tOutputNode);
    }
}
// function append_constraint_stage_for_shape_problem
/******************************************************************************/

/******************************************************************************/
void append_constraint_gradient_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "topology") 
    {
        append_constraint_gradient_stage_for_topology_problem(aXMLMetaData, aDocument);
    }
    else if(aXMLMetaData.optimization_parameters().optimization_type() == "shape") 
    {
        append_constraint_gradient_stage_for_shape_problem(aXMLMetaData, aDocument);
    }
}
// function append_constraint_gradient_stage
/******************************************************************************/

/******************************************************************************/
void append_constraint_gradient_stage_for_topology_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tDesignVariableName = get_design_variable_name(aXMLMetaData);
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    XMLGen::ConstraintGradientOperation tGradOperationInterface;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tStageNode = aDocument.append_child("Stage");
        auto tStageName = std::string("Compute Constraint Gradient ") + tConstraint.id();
        XMLGen::append_children({"Name"}, {tStageName}, tStageNode);
        auto tInputNode = tStageNode.append_child("Input");
        XMLGen::append_children({"SharedDataName"}, {"Control"}, tInputNode);

        auto tService = aXMLMetaData.service(tConstraint.service()); 
        XMLGen::append_filter_control_operation(aXMLMetaData, tStageNode);
        XMLGen::append_enforce_bounds_operation(aXMLMetaData, tStageNode);
        tGradOperationInterface.call(tConstraint, tService.performer(), tDesignVariableName, tService.code(), tStageNode);

        std::string tOutputSharedData = "Constraint Gradient " + tConstraint.id();
        if(aXMLMetaData.optimization_parameters().filter_in_engine() == "false")
        {
            auto tSharedDataName = get_filter_constraint_criterion_gradient_input_shared_data_name(tConstraint);
            append_copy_field_operation(tFirstPlatoMainPerformer, tSharedDataName, tOutputSharedData, tStageNode);
        }
        else
        {
            auto tSharedDataName = get_filter_constraint_criterion_gradient_input_shared_data_name(tConstraint);
            XMLGen::append_filter_criterion_gradient_operation(aXMLMetaData, tSharedDataName, tOutputSharedData, tStageNode);
        }

        auto tOutputNode = tStageNode.append_child("Output");
        XMLGen::append_children({"SharedDataName"}, {tOutputSharedData}, tOutputNode);
    }
}
// function append_constraint_gradient_stage_for_topology_problem
/******************************************************************************/

/******************************************************************************/
void append_compute_constraint_gradient_operation
(const std::string &aPerformer,
 pugi::xml_node &aParent)
{
    auto tOperationNode = aParent.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Constraint Gradient", aPerformer}, tOperationNode);
}
/******************************************************************************/

/******************************************************************************/
void append_compute_shape_sensitivity_on_change_operation
(pugi::xml_node &aParent)
{
    auto tForNode = aParent.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"I", "Parameters"}, tForNode);
    auto tOperationNode = tForNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Parameter Sensitivity on Change", "plato_esp_{I}"}, tOperationNode);
    auto tParamNode = tOperationNode.append_child("Parameter");
    XMLGen::append_children({"ArgumentName", "ArgumentValue"}, {"Parameter Index", "{I-1}"}, tParamNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Parameters", "Design Parameters"}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Parameter Sensitivity", "Parameter Sensitivity {I}"}, tOutputNode);
}
/******************************************************************************/

/******************************************************************************/
void append_compute_constraint_sensitivity_operation
(const std::string &aPerformer,
 const std::string &aSharedDataName,
 pugi::xml_node &aParent)
{
    auto tOperationNode = aParent.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Constraint Sensitivity", aPerformer}, tOperationNode);
    auto tForNode = tOperationNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"I", "Parameters"}, tForNode);
    auto tInputNode = tForNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Parameter Sensitivity {I}", "Parameter Sensitivity {I}"}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Criterion Sensitivity", aSharedDataName}, tOutputNode);
}
/******************************************************************************/

/******************************************************************************/
void append_compute_objective_sensitivity_operation
(const std::string &aPerformer,
 const std::string &aSharedDataName,
 pugi::xml_node &aParent)
{
    auto tOperationNode = aParent.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Objective Sensitivity", aPerformer}, tOperationNode);
    auto tForNode = tOperationNode.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"I", "Parameters"}, tForNode);
    auto tInputNode = tForNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Parameter Sensitivity {I}", "Parameter Sensitivity {I}"}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Criterion Sensitivity", aSharedDataName}, tOutputNode);
}
/******************************************************************************/

/******************************************************************************/
void append_copy_value_operation
(const std::string &aPerformer,
 const std::string &aInputSharedDataName,
 const std::string &aOutputSharedDataName,
 pugi::xml_node &aParent)
{
    auto tOperationNode = aParent.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Copy Value", aPerformer}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"InputValue", aInputSharedDataName}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"OutputValue", aOutputSharedDataName}, tOutputNode);
}
/******************************************************************************/

/******************************************************************************/
void append_copy_field_operation
(const std::string &aPerformer,
 const std::string &aInputSharedDataName,
 const std::string &aOutputSharedDataName,
 pugi::xml_node &aParent)
{
    auto tOperationNode = aParent.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Copy Field", aPerformer}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"InputField", aInputSharedDataName}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"OutputField", aOutputSharedDataName}, tOutputNode);
}
/******************************************************************************/

/******************************************************************************/
void append_constraint_gradient_stage_for_shape_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tDesignVariableName = get_design_variable_name(aXMLMetaData);
    std::string tFirstPlatoMainPerformer = aXMLMetaData.getFirstPlatoMainPerformer();
    XMLGen::ConstraintGradientOperation tGradOperationInterface;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        std::string tCriterionID = tConstraint.criterion();
        std::string tServiceID = tConstraint.service();
        std::string tScenarioID = tConstraint.scenario();
        ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
        auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);
        auto tSharedDataName = std::string("Criterion Gradient - ") + tIdentifierString;
        auto tService = aXMLMetaData.service(tConstraint.service()); 
        auto tStageNode = aDocument.append_child("Stage");
        auto tStageName = std::string("Compute Constraint Gradient ") + tConstraint.id();
        auto tStageOutputName = std::string("Constraint Gradient ") + tConstraint.id();
        XMLGen::append_children({"Name"}, {tStageName}, tStageNode);
        auto tInputNode = tStageNode.append_child("Input");
        XMLGen::append_children({"SharedDataName"}, {"Design Parameters"}, tInputNode);

        XMLGen::append_update_geometry_on_change_operation(tFirstPlatoMainPerformer, tStageNode);
        XMLGen::append_reinitialize_on_change_operation(tService.performer(), tStageNode);
        auto tOuterOperationNode = tStageNode.append_child("Operation");
        append_compute_constraint_gradient_operation(tService.performer(), tOuterOperationNode);
        append_compute_shape_sensitivity_on_change_operation(tOuterOperationNode);
        append_compute_constraint_sensitivity_operation(tService.performer(), tSharedDataName, tStageNode);
        append_copy_value_operation(tFirstPlatoMainPerformer, tSharedDataName, tStageOutputName, tStageNode);
        auto tOutputNode = tStageNode.append_child("Output");
        XMLGen::append_children({"SharedDataName"}, {"Constraint Gradient " + tConstraint.id()}, tOutputNode);
    }
}
// function append_constraint_gradient_stage_for_shape_problem
/******************************************************************************/

/******************************************************************************/
void append_derivative_checker_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tCheckGradient = aXMLMetaData.optimization_parameters().check_gradient().empty() ? std::string("true") : aXMLMetaData.optimization_parameters().check_gradient();
    std::vector<std::string> tKeys = {"CheckGradient", "CheckHessian", "UseUserInitialGuess"};
    std::vector<std::string> tValues = {tCheckGradient, aXMLMetaData.optimization_parameters().check_hessian(), "True"};
    XMLGen::append_children(tKeys, tValues, aParentNode);

    auto tOptionsNode = aParentNode.append_child("Options");
    tKeys = {"DerivativeCheckerInitialSuperscript", "DerivativeCheckerFinalSuperscript"};
    tValues = {aXMLMetaData.optimization_parameters().derivative_checker_initial_superscript(), aXMLMetaData.optimization_parameters().derivative_checker_final_superscript()};
    XMLGen::append_children(tKeys, tValues, tOptionsNode);
}
// function append_derivative_checker_options
/******************************************************************************/

/******************************************************************************/
void append_optimality_criteria_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::vector<std::string> tKeys = {"OCControlStagnationTolerance", 
                                      "OCObjectiveStagnationTolerance",
                                      "OCGradientTolerance"};
    std::vector<std::string> tValues = {aXMLMetaData.optimization_parameters().oc_control_stagnation_tolerance(), 
                                        aXMLMetaData.optimization_parameters().oc_objective_stagnation_tolerance(),
                                        aXMLMetaData.optimization_parameters().oc_gradient_tolerance()};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    auto tOptionsNode = aParentNode.child("Options");
    if(tOptionsNode.empty())
    {
        tOptionsNode = aParentNode.append_child("Options");
    }
    XMLGen::append_children(tKeys, tValues, tOptionsNode);

    auto tConvergenceNode = aParentNode.append_child("Convergence");
    XMLGen::append_children({"MaxIterations"}, {aXMLMetaData.optimization_parameters().max_iterations()}, tConvergenceNode);
}
// function append_optimality_criteria_options
/******************************************************************************/

/******************************************************************************/
void append_method_moving_asymptotes_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::vector<std::string> tKeys = {"MaxNumOuterIterations", "MoveLimit", "AsymptoteExpansion", "AsymptoteContraction",
         "MaxNumSubProblemIter", "ControlStagnationTolerance", "ObjectiveStagnationTolerance"};
    std::vector<std::string> tValues = {aXMLMetaData.optimization_parameters().max_iterations(), aXMLMetaData.optimization_parameters().mma_move_limit(), aXMLMetaData.optimization_parameters().mma_asymptote_expansion(),
        aXMLMetaData.optimization_parameters().mma_asymptote_contraction(), aXMLMetaData.optimization_parameters().mma_max_sub_problem_iterations(), aXMLMetaData.optimization_parameters().mma_control_stagnation_tolerance(),
        aXMLMetaData.optimization_parameters().mma_objective_stagnation_tolerance()};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    auto tOptionsNode = aParentNode.append_child("Options");
    XMLGen::append_children(tKeys, tValues, tOptionsNode);
}
// function append_method_moving_asymptotes_options
/******************************************************************************/

/******************************************************************************/
void append_globally_convergent_method_moving_asymptotes_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::vector<std::string> tKeys = 
                                      {"GCMMAInnerKKTTolerance", 
                                       "GCMMAOuterKKTTolerance", 
                                       "GCMMAInnerControlStagnationTolerance", 
                                       "GCMMAOuterControlStagnationTolerance", 
                                       "GCMMAOuterObjectiveStagnationTolerance", 
                                       "GCMMAMaxInnerIterations", 
                                       "GCMMAOuterStationarityTolerance", 
                                       "GCMMAInitialMovingAsymptoteScaleFactor"};
    std::vector<std::string> tValues = {
             aXMLMetaData.optimization_parameters().gcmma_inner_kkt_tolerance(), 
             aXMLMetaData.optimization_parameters().gcmma_outer_kkt_tolerance(), 
             aXMLMetaData.optimization_parameters().gcmma_inner_control_stagnation_tolerance(), 
             aXMLMetaData.optimization_parameters().gcmma_outer_control_stagnation_tolerance(), 
             aXMLMetaData.optimization_parameters().gcmma_outer_objective_stagnation_tolerance(), 
             aXMLMetaData.optimization_parameters().gcmma_max_inner_iterations(), 
             aXMLMetaData.optimization_parameters().gcmma_outer_stationarity_tolerance(), 
             aXMLMetaData.optimization_parameters().gcmma_initial_moving_asymptotes_scale_factor()};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    auto tOptionsNode = aParentNode.append_child("Options");
    XMLGen::append_children(tKeys, tValues, tOptionsNode);
}
// function append_globally_convergent_method_moving_asymptotes_options
/******************************************************************************/

/******************************************************************************/
void append_trust_region_kelley_sachs_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::vector<std::string> tKeys = {"MaxNumOuterIterations",
                                      "KSTrustRegionExpansionFactor", 
                                      "KSTrustRegionContractionFactor", 
                                      "KSMaxTrustRegionIterations", 
                                      "KSInitialRadiusScale",
                                      "KSMaxRadiusScale", 
                                      "HessianType", 
                                      "MinTrustRegionRadius", 
                                      "LimitedMemoryStorage", 
                                      "KSOuterGradientTolerance", 
                                      "KSOuterStationarityTolerance",
                                      "KSOuterStagnationTolerance", 
                                      "KSOuterControlStagnationTolerance", 
                                      "KSOuterActualReductionTolerance", 
                                      "ProblemUpdateFrequency", 
                                      "DisablePostSmoothing",
                                      "KSTrustRegionRatioLow",       
                                      "KSTrustRegionRatioMid", 
                                      "KSTrustRegionRatioUpper"};
    std::vector<std::string> tValues = {aXMLMetaData.optimization_parameters().max_iterations(),
                                        aXMLMetaData.optimization_parameters().ks_trust_region_expansion_factor(), 
                                        aXMLMetaData.optimization_parameters().ks_trust_region_contraction_factor(), 
                                        aXMLMetaData.optimization_parameters().ks_max_trust_region_iterations(),
                                        aXMLMetaData.optimization_parameters().ks_initial_radius_scale(), 
                                        aXMLMetaData.optimization_parameters().ks_max_radius_scale(), 
                                        aXMLMetaData.optimization_parameters().hessian_type(), 
                                        aXMLMetaData.optimization_parameters().ks_min_trust_region_radius(), 
                                        aXMLMetaData.optimization_parameters().limited_memory_storage(),
                                        aXMLMetaData.optimization_parameters().ks_outer_gradient_tolerance(), 
                                        aXMLMetaData.optimization_parameters().ks_outer_stationarity_tolerance(), 
                                        aXMLMetaData.optimization_parameters().ks_outer_stagnation_tolerance(), 
                                        aXMLMetaData.optimization_parameters().ks_outer_control_stagnation_tolerance(),
                                        aXMLMetaData.optimization_parameters().ks_outer_actual_reduction_tolerance(), 
                                        aXMLMetaData.optimization_parameters().problem_update_frequency(), 
                                        aXMLMetaData.optimization_parameters().ks_disable_post_smoothing(), 
                                        aXMLMetaData.optimization_parameters().ks_trust_region_ratio_low(),
                                        aXMLMetaData.optimization_parameters().ks_trust_region_ratio_mid(), 
                                        aXMLMetaData.optimization_parameters().ks_trust_region_ratio_high()};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    auto tOptionsNode = aParentNode.append_child("Options");
    XMLGen::append_children(tKeys, tValues, tOptionsNode);
    auto tConvergenceNode = aParentNode.append_child("Convergence");
    XMLGen::append_children({"MaxIterations"}, {aXMLMetaData.optimization_parameters().max_iterations()}, tConvergenceNode);
}
// function append_trust_region_kelley_sachs_options
/******************************************************************************/

/******************************************************************************/
void append_augmented_lagrangian_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::vector<std::string> tKeys = {"AugLagPenaltyParam", 
                                      "AugLagPenaltyParamScaleFactor"};
    std::vector<std::string> tValues = {aXMLMetaData.optimization_parameters().al_penalty_parameter(), 
                                        aXMLMetaData.optimization_parameters().al_penalty_scale_factor()};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    auto tOptionsNode = aParentNode.child("Options");
    if(tOptionsNode.empty())
    {
        tOptionsNode = aParentNode.append_child("Options");
    }
    XMLGen::append_children(tKeys, tValues, tOptionsNode);
}
// function append_augmented_lagrangian_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_algorithm_parameters_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tLower = XMLGen::to_lower(aXMLMetaData.optimization_parameters().optimization_algorithm());
    if(tLower.compare("oc") == 0)
    {
        XMLGen::append_optimality_criteria_options(aXMLMetaData, aParentNode);
    }
    else if(tLower.compare("mma") == 0)
    {
        XMLGen::append_method_moving_asymptotes_options(aXMLMetaData, aParentNode);
    }
    else if(tLower.compare("gcmma") == 0)
    {
        XMLGen::append_globally_convergent_method_moving_asymptotes_options(aXMLMetaData, aParentNode);
    }
    else if(tLower.compare("ksbc") == 0)
    {
        XMLGen::append_trust_region_kelley_sachs_options(aXMLMetaData, aParentNode);
    }
    else if(tLower.compare("ksal") == 0)
    {
        XMLGen::append_trust_region_kelley_sachs_options(aXMLMetaData, aParentNode);
        XMLGen::append_augmented_lagrangian_options(aXMLMetaData, aParentNode);
    }
    else if(tLower.compare("derivativechecker") == 0)
    {
        XMLGen::append_derivative_checker_options(aXMLMetaData, aParentNode);
    }
    else
    {
        THROWERR(std::string("Append Optimization Algorithm Options: ") + "Optimization algorithm '"
            + aXMLMetaData.optimization_parameters().optimization_algorithm() + "' is not supported.")
    }
}
// function append_optimization_algorithm_parameters_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_algorithm_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::unordered_map<std::string, std::string> tValidOptimizers =
        { {"oc", "OC"}, {"mma", "MMA"}, {"gcmma", "GCMMA"}, {"ksbc", "KSBC"}, {"ksal", "KSAL"} ,
          {"rol ksbc", "ROL KSBC"}, {"rol ksal", "ROL KSAL"}, {"derivativechecker", "DerivativeChecker"} };

    auto tLower = Plato::tolower(aXMLMetaData.optimization_parameters().optimization_algorithm());
    auto tOptimizerItr = tValidOptimizers.find(tLower);
    if(tOptimizerItr == tValidOptimizers.end())
    {
        THROWERR(std::string("Append Optimization Algorithm Option: Optimization algorithm '")
            + aXMLMetaData.optimization_parameters().optimization_algorithm() + "' is not supported.")
    }

    XMLGen::append_children( { "Package" }, { tOptimizerItr->second }, aParentNode);
    XMLGen::append_optimization_algorithm_parameters_options(aXMLMetaData, aParentNode);
}
// function append_optimization_algorithm_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_output_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.mOutputMetaData.size() > 0 && 
       aXMLMetaData.mOutputMetaData[0].isOutputDisabled())
    {
        return;
    }
    auto tNode = aParentNode.append_child("Output");
    append_children({"OutputStage"}, {"Output To File"}, tNode);
}
// function append_optimization_output_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_cache_stage_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(num_cache_states(aXMLMetaData.services()) > 0)
    {
        auto tNode = aParentNode.append_child("CacheStage");
        append_children({"Name"}, {"Cache State"}, tNode);
    }
}
// function append_optimization_cache_stage_options
/******************************************************************************/

/******************************************************************************/
bool need_update_problem_stage
(const XMLGen::InputData& aXMLMetaData)
{
    for (auto &tService : aXMLMetaData.services())
    {
        if (tService.updateProblem())
        {
            return true;
        }
    }
    return false;
}
/******************************************************************************/

/******************************************************************************/
void append_optimization_update_problem_stage_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(!need_update_problem_stage(aXMLMetaData))
    {
        return;
    }
    auto tNode = aParentNode.append_child("UpdateProblemStage");
    append_children({"Name"}, {"Update Problem"}, tNode);
}
// function append_optimization_cache_stage_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_variables_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::vector<std::string> tKeys =
        {"ValueName", "InitializationStage", "FilteredName", "LowerBoundValueName", "LowerBoundVectorName",
         "UpperBoundValueName", "UpperBoundVectorName", "SetLowerBoundsStage", "SetUpperBoundsStage"};
    std::vector<std::string> tValues;
    if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        tValues =
            {"Control", "Initial Guess", "Topology", "Lower Bound Value", "Lower Bound Vector",
             "Upper Bound Value", "Upper Bound Vector", "Set Lower Bounds", "Set Upper Bounds"};
    }
    else if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        tValues =
            {"Design Parameters", "Initialize Design Parameters", "Topology", "Lower Bound Value", "Lower Bound Vector",
             "Upper Bound Value", "Upper Bound Vector", "Set Lower Bounds", "Set Upper Bounds"};
    }
    auto tNode = aParentNode.append_child("OptimizationVariables");
    XMLGen::append_children(tKeys, tValues, tNode);
}
// function append_optimization_variables_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_objective_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::unordered_map<std::string, std::string> tKeyToValueMap =
        { {"ValueName", ""}, {"ValueStageName", ""}, {"GradientName", ""}, {"GradientStageName", ""} };

    std::string tValueNameString = "Objective Value";

    tKeyToValueMap.find("ValueName")->second = tValueNameString;
    tKeyToValueMap.find("ValueStageName")->second = std::string("Compute Objective Value");
    tKeyToValueMap.find("GradientName")->second = std::string("Objective Gradient");
    tKeyToValueMap.find("GradientStageName")->second = std::string("Compute Objective Gradient");

    auto tKeys = XMLGen::transform_key_tokens(tKeyToValueMap);
    auto tValues = XMLGen::transform_value_tokens(tKeyToValueMap);
    auto tNode = aParentNode.append_child("Objective");
    XMLGen::append_children(tKeys, tValues, tNode);
}
// function append_optimization_objective_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_constraint_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    for (auto &tConstraint : aXMLMetaData.constraints)
    {
        std::unordered_map<std::string, std::string> tKeyToValueMap;
        if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
        {
            tKeyToValueMap =
            { {"ValueName", ""}, {"ValueStageName", ""}, {"GradientName", ""}, {"GradientStageName", ""},
              {"ReferenceValueName", "Design Volume"} };
        }
        else if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
        {
            tKeyToValueMap =
            { {"ValueName", ""}, {"ValueStageName", ""}, {"GradientName", ""}, {"GradientStageName", ""} };
        }

        tKeyToValueMap.find("ValueName")->second = std::string("Constraint Value ") + tConstraint.id();
        tKeyToValueMap.find("ValueStageName")->second = std::string("Compute Constraint Value ") + tConstraint.id();
        tKeyToValueMap.find("GradientName")->second = std::string("Constraint Gradient ") + tConstraint.id();
        tKeyToValueMap.find("GradientStageName")->second = std::string("Compute Constraint Gradient ") + tConstraint.id();
        if(tConstraint.absoluteTarget().length() > 0)
            tKeyToValueMap["AbsoluteTargetValue"] = tConstraint.absoluteTarget();
        else if(tConstraint.relativeTarget().length() > 0)
            tKeyToValueMap["NormalizedTargetValue"] = tConstraint.relativeTarget();
        else
            THROWERR("Append Optimization Constraint Options: Constraint target was not set.")

        auto tKeys = XMLGen::transform_key_tokens(tKeyToValueMap);
        auto tValues = XMLGen::transform_value_tokens(tKeyToValueMap);
        auto tNode = aParentNode.append_child("Constraint");
        XMLGen::append_children(tKeys, tValues, tNode);
    }
}
// function append_optimization_constraint_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_bound_constraints_options
(const std::vector<std::string>& aValues,
 pugi::xml_node& aParentNode)
{
    std::vector<std::string> tKeys = {"Upper", "Lower"};
    auto tNode = aParentNode.append_child("BoundConstraint");
    XMLGen::append_children(tKeys, aValues, tNode);
}
// function append_optimization_bound_constraints_options
/******************************************************************************/

/******************************************************************************/
void append_optimizer_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tOptimizerNode = aParentNode.append_child("Optimizer");
    XMLGen::append_optimization_algorithm_options(aXMLMetaData, tOptimizerNode);
    XMLGen::append_optimization_update_problem_stage_options(aXMLMetaData, tOptimizerNode);
    XMLGen::append_optimization_cache_stage_options(aXMLMetaData, tOptimizerNode);
    XMLGen::append_optimization_output_options(aXMLMetaData, tOptimizerNode);
    XMLGen::append_optimization_variables_options(aXMLMetaData, tOptimizerNode);
    XMLGen::append_optimization_objective_options(aXMLMetaData, tOptimizerNode);
    XMLGen::append_optimization_constraint_options(aXMLMetaData, tOptimizerNode);
    XMLGen::append_optimization_bound_constraints_options({"1.0", "0.0"}, tOptimizerNode);
}
// function append_optimizer_options
/******************************************************************************/

}
// namespace XMLGen
