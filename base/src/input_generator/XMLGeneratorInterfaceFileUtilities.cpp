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
void append_objective_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument,
 std::string aOwnerName)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR(std::string("Append Criterion Shared Data: ") + "Objective function list is empty.")
    }

    for(auto& tObjective : aXMLMetaData.objectives)
    {
        auto tIndex = &tObjective - &aXMLMetaData.objectives[0];
        // shared data - deterministic criterion value
        auto tTag = std::string("Objective Value ") + std::to_string(tIndex);
        auto tOwnerName = aOwnerName.empty() ? tObjective.performer_name : aOwnerName;
        std::vector<std::string> tKeys = { "Name", "Type", "Layout", "Size", "OwnerName", "UserName" };
        std::vector<std::string> tValues = { tTag, "Scalar", "Global", "1", tOwnerName, "PlatoMain" };
        auto tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);

        // shared data - deterministic criterion gradient
        tTag = std::string("Objective Gradient ") + std::to_string(tIndex);
        tValues = { tTag, "Scalar", "Nodal Field", "IGNORE", tOwnerName, "PlatoMain" };
        tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);
    }
}
// function append_objective_shared_data
/******************************************************************************/

/******************************************************************************/
void append_constraint_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument,
 std::string aOwnerName)
{
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tIndex = &tConstraint - &aXMLMetaData.constraints[0];
        // shared data - deterministic criterion value
        auto tTag = std::string("Constraint Value ") + std::to_string(tIndex);
        auto tOwnerName = aOwnerName.empty() ? tConstraint.mPerformerName : aOwnerName;
        std::vector<std::string> tKeys = { "Name", "Type", "Layout", "Size", "OwnerName", "UserName" };
        std::vector<std::string> tValues = { tTag, "Scalar", "Global", "1", tOwnerName, "PlatoMain" };
        auto tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);

        // shared data - deterministic criterion gradient
        tTag = std::string("Constraint Gradient ") + std::to_string(tIndex);
        tValues = { tTag, "Scalar", "Nodal Field", "IGNORE", tOwnerName, "PlatoMain" };
        tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);
    }
}
// function append_constraint_shared_data
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
(const std::string& aSharedDataName,
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"},{"Filter Gradient", "PlatoMain"}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Field", "Control"}, tInputNode);
    tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Gradient", aSharedDataName}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Filtered Gradient", aSharedDataName}, tOutputNode);
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
    XMLGen::append_children({"ArgumentName", "SharedDataName"},{"Initialized Field", "Control"}, tOutputNode);
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
 const std::string& aSharedDataName,
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Calculate Constraint Value", aPerformerName}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Constraint Value", aSharedDataName}, tOutputNode);
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
        auto tSharedDataName = std::string("Constraint Value ") + std::to_string(tIndex);
        XMLGen::append_constraint_value_operation(tConstraint.mPerformerName, tSharedDataName, tStageNode);

        auto tOutputNode = tStageNode.append_child("Output");
        XMLGen::append_children({"SharedDataName"}, {tSharedDataName}, tOutputNode);
    }
}
// function append_constraint_value_stage
/******************************************************************************/

/******************************************************************************/
void append_constraint_gradient_operation
(const std::string& aPerformerName,
 const std::string& aSharedDataName,
 pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Calculate Constraint Gradient", aPerformerName}, tOperationNode);
    auto tInputNode = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tInputNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Constraint Gradient", aSharedDataName}, tOutputNode);
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
        auto tSharedDataName = std::string("Constraint Gradient ") + std::to_string(tIndex);
        XMLGen::append_constraint_gradient_operation(tConstraint.mPerformerName, tSharedDataName, tStageNode);
        XMLGen::append_filter_criterion_gradient_operation(tSharedDataName, tStageNode);

        auto tOutputNode = tStageNode.append_child("Output");
        XMLGen::append_children({"SharedDataName"}, {tSharedDataName}, tOutputNode);
    }
}
// function append_constraint_gradient_stage
/******************************************************************************/

/******************************************************************************/
void append_derivative_checker_parameters_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::vector<std::string> tKeys = {"CheckGradient", "CheckHessian", "UseUserInitialGuess"};
    std::vector<std::string> tValues = {aXMLMetaData.check_gradient, aXMLMetaData.check_hessian, "True"};
    XMLGen::append_children(tKeys, tValues, aParentNode);

    auto tOptionsNode = aParentNode.append_child("Options");
    tKeys = {"DerivativeCheckerInitialSuperscript", "DerivativeCheckerFinalSuperscript"};
    tValues = {aXMLMetaData.mDerivativeCheckerInitialSuperscript, aXMLMetaData.mDerivativeCheckerFinalSuperscript};
    XMLGen::append_children(tKeys, tValues, tOptionsNode);
}
// function append_derivative_checker_parameters_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_algorithm_oc_parameters_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tConvergenceNode = aParentNode.append_child("Convergence");
    XMLGen::append_children({"MaxIterations"}, {aXMLMetaData.max_iterations}, tConvergenceNode);
}
// function append_optimization_algorithm_oc_parameters_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_algorithm_mma_parameters_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::vector<std::string> tKeys = {"MaxNumOuterIterations", "MoveLimit", "AsymptoteExpansion", "AsymptoteContraction",
         "MaxNumSubProblemIter", "ControlStagnationTolerance", "ObjectiveStagnationTolerance"};
    std::vector<std::string> tValues = {aXMLMetaData.max_iterations, aXMLMetaData.mMMAMoveLimit, aXMLMetaData.mMMAAsymptoteExpansion,
        aXMLMetaData.mMMAAsymptoteContraction, aXMLMetaData.mMMAMaxNumSubProblemIterations, aXMLMetaData.mMMAControlStagnationTolerance,
        aXMLMetaData.mMMAObjectiveStagnationTolerance};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    auto tOptionsNode = aParentNode.append_child("Options");
    XMLGen::append_children(tKeys, tValues, tOptionsNode);
}
// function append_optimization_algorithm_mma_parameters_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_algorithm_parameters_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tLower = Plato::tolower(aXMLMetaData.optimization_algorithm);
    if(tLower.compare("oc") == 0)
    {
        XMLGen::append_optimization_algorithm_oc_parameters_options(aXMLMetaData, aParentNode);
    }
    else if(tLower.compare("mma") == 0)
    {
        XMLGen::append_optimization_algorithm_mma_parameters_options(aXMLMetaData, aParentNode);
    }
    else if(tLower.compare("derivativechecker") == 0)
    {
        XMLGen::append_derivative_checker_parameters_options(aXMLMetaData, aParentNode);
    }
    else
    {
        THROWERR(std::string("Append Optimization Algorithm Options: ") + "Optimization algorithm '"
            + aXMLMetaData.optimization_algorithm + "' is not supported.")
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
        { {"oc", "OC"}, {"mma", "MMA"}, {"ksbc", "KSBC"}, {"ksal", "KSAL"} ,
          {"rol ksbc", "ROL KSBC"}, {"rol ksal", "ROL KSAL"}, {"derivativechecker", "DerivativeChecker"} };

    auto tLower = Plato::tolower(aXMLMetaData.optimization_algorithm);
    auto tOptimizerItr = tValidOptimizers.find(tLower);
    if(tOptimizerItr == tValidOptimizers.end())
    {
        THROWERR(std::string("Append Optimization Algorithm Option: Optimization algorithm '")
            + aXMLMetaData.optimization_algorithm + "' is not supported.")
    }

    XMLGen::append_children( { "Package" }, { tOptimizerItr->second }, aParentNode);
    XMLGen::append_optimization_algorithm_parameters_options(aXMLMetaData, aParentNode);
}
// function append_optimization_algorithm_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_output_options
(pugi::xml_node& aParentNode)
{
    auto tNode = aParentNode.append_child("Output");
    append_children({"OutputStage"}, {"Output To File"}, tNode);
}
// function append_optimization_output_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_cache_stage_options
(pugi::xml_node& aParentNode)
{
    auto tNode = aParentNode.append_child("CacheStage");
    append_children({"Name"}, {"Cache State"}, tNode);
}
// function append_optimization_cache_stage_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_update_problem_stage_options
(pugi::xml_node& aParentNode)
{
    auto tNode = aParentNode.append_child("UpdateProblemStage");
    append_children({"Name"}, {"Update Problem"}, tNode);
}
// function append_optimization_cache_stage_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_variables_options
(pugi::xml_node& aParentNode)
{
    std::vector<std::string> tKeys =
        {"ValueName", "InitializationStage", "FilteredName", "LowerBoundValueName", "LowerBoundVectorName",
         "UpperBoundValueName", "UpperBoundVectorName", "SetLowerBoundsStage", "SetUpperBoundsStage"};
    std::vector<std::string> tValues =
        {"Control", "Initial Guess", "Topology", "Lower Bound Value", "Lower Bound Vector",
         "Upper Bound Value", "Upper Bound Vector", "Set Lower Bounds", "Set Upper Bounds"};
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

    for (auto &tObjective : aXMLMetaData.objectives)
    {
        auto tIndex = std::to_string(&tObjective - &aXMLMetaData.objectives[0]);

        tKeyToValueMap.find("ValueName")->second = std::string("Objective Value ") + tIndex;
        tKeyToValueMap.find("ValueStageName")->second = std::string("Calculate Objective Value ") + tIndex;
        tKeyToValueMap.find("GradientName")->second = std::string("Objective Gradient ") + tIndex;
        tKeyToValueMap.find("GradientStageName")->second = std::string("Calculate Objective Gradient ") + tIndex;

        auto tKeys = XMLGen::transform_key_tokens(tKeyToValueMap);
        auto tValues = XMLGen::transform_value_tokens(tKeyToValueMap);
        auto tNode = aParentNode.append_child("Objective");
        XMLGen::append_children(tKeys, tValues, tNode);
    }
}
// function append_optimization_objective_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_constraint_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::unordered_map<std::string, std::string> tKeyToValueMap =
        { {"ValueName", ""}, {"ValueStageName", ""}, {"GradientName", ""}, {"GradientStageName", ""},
          {"ReferenceValueName", "Reference Value"}, {"NormalizedTargetValue", ""}, {"AbsoluteTargetValue", ""} };

    for (auto &tConstraint : aXMLMetaData.constraints)
    {
        auto tIndex = std::to_string(&tConstraint - &aXMLMetaData.constraints[0]);

        tKeyToValueMap.find("ValueName")->second = std::string("Constraint Value ") + tIndex;
        tKeyToValueMap.find("ValueStageName")->second = std::string("Calculate Constraint Value ") + tIndex;
        tKeyToValueMap.find("GradientName")->second = std::string("Constraint Gradient ") + tIndex;
        tKeyToValueMap.find("GradientStageName")->second = std::string("Calculate Constraint Gradient ") + tIndex;
        XMLGen::set_key_value("AbsoluteTargetValue", tConstraint.mAbsoluteTargetValue, tKeyToValueMap);
        XMLGen::set_key_value("NormalizedTargetValue", tConstraint.mNormalizedTargetValue, tKeyToValueMap);

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
    XMLGen::append_optimization_update_problem_stage_options(tOptimizerNode);
    XMLGen::append_optimization_cache_stage_options(tOptimizerNode);
    XMLGen::append_optimization_output_options(tOptimizerNode);
    XMLGen::append_optimization_variables_options(tOptimizerNode);
    XMLGen::append_optimization_objective_options(aXMLMetaData, tOptimizerNode);
    XMLGen::append_optimization_constraint_options(aXMLMetaData, tOptimizerNode);
    XMLGen::append_optimization_bound_constraints_options({"1.0", "0.0"}, tOptimizerNode);
}
// function append_optimizer_options
/******************************************************************************/

}
// namespace XMLGen
