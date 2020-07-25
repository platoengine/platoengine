/*
 * XMLGeneratorInterfaceFileUtilities.cpp
 *
 *  Created on: May 26, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"
#include "XMLGeneratorPlatoMainConstraintValueOperationInterface.hpp"
#include "XMLGeneratorPlatoMainConstraintGradientOperationInterface.hpp"

namespace XMLGen
{

/******************************************************************************/
void append_objective_gradient_to_plato_main_output_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    for(auto& tObjective : aXMLMetaData.objectives)
    {
        auto tInput = aParentNode.append_child("Input");
        auto tSharedDataName = std::string("Objective Gradient ID-") + tObjective.name;
        auto tArgumentName  = XMLGen::to_lower(tSharedDataName);
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgumentName, tSharedDataName}, tInput);
    }
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
        auto tSharedDataName = std::string("Constraint Gradient ID-") + tConstraint.name();
        auto tArgumentName  = XMLGen::to_lower(tSharedDataName);
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgumentName, tSharedDataName}, tInput);
    }
}
// function append_constraint_gradient_to_plato_main_output_stage
/******************************************************************************/

/******************************************************************************/
void append_default_qoi_to_plato_main_output_stage
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
void append_random_qoi_samples_to_plato_main_output_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tQoIIDs = aXMLMetaData.mOutputMetaData.randomIDs();
    for(auto& tID : tQoIIDs)
    {
        auto tOuterFor = aParentNode.append_child("For");
        XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
        auto tInnerFor = tOuterFor.append_child("For");
        XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
        auto tInput = tInnerFor.append_child("Input");
        auto tArgumentName = aXMLMetaData.mOutputMetaData.randomArgumentName(tID);
        auto tSharedDataName = aXMLMetaData.mOutputMetaData.randomSharedDataName(tID);
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgumentName, tSharedDataName}, tInput);
    }
}
// function append_random_qoi_samples_to_plato_main_output_stage
/******************************************************************************/

/******************************************************************************/
void append_deterministic_qoi_to_plato_main_output_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tQoIIDs = aXMLMetaData.mOutputMetaData.deterministicIDs();
    for(auto& tID : tQoIIDs)
    {
        auto tInput = aParentNode.append_child("Input");
        auto tArgumentName = aXMLMetaData.mOutputMetaData.deterministicArgumentName(tID);
        auto tSharedDataName = aXMLMetaData.mOutputMetaData.deterministicSharedDataName(tID);
        XMLGen::append_children({"ArgumentName", "SharedDataName"}, {tArgumentName, tSharedDataName}, tInput);
    }
}
// function append_deterministic_qoi_to_plato_main_output_stage
/******************************************************************************/

/******************************************************************************/
void append_plato_main_output_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.mOutputMetaData.outputData() == false)
    {
        return;
    }
    auto tOutputStage = aDocument.append_child("Stage");
    XMLGen::append_children({"Name"}, {"Output To File"}, tOutputStage);
    auto tOutputOperation = tOutputStage.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"},{"PlatoMainOutput", "platomain"}, tOutputOperation);
    XMLGen::append_default_qoi_to_plato_main_output_stage(aXMLMetaData, tOutputOperation);
    XMLGen::append_deterministic_qoi_to_plato_main_output_stage(aXMLMetaData, tOutputOperation);
    XMLGen::append_random_qoi_samples_to_plato_main_output_stage(aXMLMetaData, tOutputOperation);
}
// function append_plato_main_output_stage
/******************************************************************************/

/******************************************************************************/
void append_lower_bounds_shared_data
(pugi::xml_document& aDocument)
{
    // shared data - lower bound value
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Lower Bound Value", "Scalar", "Global", "1", "platomain", "platomain"};
    auto tSharedDataNode = aDocument.append_child("SharedData");
    XMLGen::append_children(tKeys, tValues, tSharedDataNode);

    // shared data - lower bound vector
    tValues = {"Lower Bound Vector", "Scalar", "Nodal Field", "IGNORE", "platomain", "platomain"};
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
    std::vector<std::string> tValues = {"Upper Bound Value", "Scalar", "Global", "1", "platomain", "platomain"};
    auto tSharedDataNode = aDocument.append_child("SharedData");
    XMLGen::append_children(tKeys, tValues, tSharedDataNode);

    // shared data - upper bound vector
    tValues = {"Upper Bound Vector", "Scalar", "Nodal Field", "IGNORE", "platomain", "platomain"};
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
    std::vector<std::string> tValues = {"Design Volume", "Scalar", "Global", "1", "platomain", "platomain"};
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
        THROWERR("Append Criterion Shared Data: Objective function list is empty. "
            + "Plato optimization problems must have at least one objective function defined.")
    }

    for(auto& tObjective : aXMLMetaData.objectives)
    {
        // shared data - deterministic criterion value
        auto tTag = std::string("Objective Value ID-") + tObjective.name;
        auto tOwnerName = aOwnerName.empty() ? tObjective.mPerformerName : aOwnerName;
        std::vector<std::string> tKeys = { "Name", "Type", "Layout", "Size", "OwnerName", "UserName" };
        std::vector<std::string> tValues = { tTag, "Scalar", "Global", "1", tOwnerName, "platomain" };
        auto tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);

        // shared data - deterministic criterion gradient
        tTag = std::string("Objective Gradient ID-") + tObjective.name;
        tValues = { tTag, "Scalar", "Nodal Field", "IGNORE", tOwnerName, "platomain" };
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
        // shared data - deterministic criterion value
        auto tTag = std::string("Constraint Value ID-") + tConstraint.name();
        auto tOwnerName = aOwnerName.empty() ? tConstraint.performer() : aOwnerName;
        std::vector<std::string> tKeys = { "Name", "Type", "Layout", "Size", "OwnerName", "UserName" };
        std::vector<std::string> tValues = { tTag, "Scalar", "Global", "1", tOwnerName, "platomain" };
        auto tSharedDataNode = aDocument.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);

        // shared data - deterministic criterion gradient
        tTag = std::string("Constraint Gradient ID-") + tConstraint.name();
        tValues = { tTag, "Scalar", "Nodal Field", "IGNORE", tOwnerName, "platomain" };
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
    std::vector<std::string> tValues = {"Control", "Scalar", "Nodal Field", "IGNORE", "platomain", "platomain"};
    XMLGen::append_children(tKeys, tValues, tSharedData);
}
// function append_control_shared_data
/******************************************************************************/

/******************************************************************************/
void append_plato_main_performer
(pugi::xml_document& aDocument)
{
    auto tPerformerNode = aDocument.append_child("Performer");
    XMLGen::append_children( {"Name", "Code", "PerformerID"}, {"platomain", "platomain", "0"}, tPerformerNode);
}
// function append_plato_main_performer
/******************************************************************************/

/******************************************************************************/
void append_filter_control_operation
(pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"},{"Filter Control", "platomain"}, tOperationNode);
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
    XMLGen::append_children({"Name", "PerformerName"},{"Filter Gradient", "platomain"}, tOperationNode);
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
    XMLGen::append_children({"Name", "PerformerName"},{"Initialize Field", "platomain"}, tOperationNode);
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
void append_lower_bound_operation
(pugi::xml_node& aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Lower Bounds", "platomain"}, tOperationNode);
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
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Upper Bounds", "platomain"}, tOperationNode);
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
    XMLGen::append_children({"Name", "PerformerName"}, {"Design Volume", "platomain"}, tOperationNode);
    auto tOutputNode = tOperationNode.append_child("Output");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Design Volume", "Design Volume"}, tOutputNode);
}
// function append_design_volume_stage
/******************************************************************************/

/******************************************************************************/
void append_design_volume_stage
(pugi::xml_document& aDocument)
{
    auto tStageNode = aDocument.append_child("Stage");
    XMLGen::append_children({"Name"}, {"Design Volume"}, tStageNode);
    XMLGen::append_design_volume_operation(tStageNode);
    auto tOutputNode = tStageNode.append_child("Output");
    XMLGen::append_children({"SharedDataName"}, {"Design Volume"}, tOutputNode);
}
// function append_design_volume_stage
/******************************************************************************/

/******************************************************************************/
void append_constraint_value_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    XMLGen::ConstraintValueOperation tValueOperationInterface;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tStageNode = aDocument.append_child("Stage");
        auto tStageName = std::string("Compute Constraint Value ID-") + tConstraint.name();
        XMLGen::append_children({"Name", "Type"}, {tStageName, tConstraint.category()}, tStageNode);
        auto tInputNode = tStageNode.append_child("Input");
        XMLGen::append_children({"SharedDataName"}, {"Control"}, tInputNode);

        XMLGen::append_filter_control_operation(tStageNode);
        tValueOperationInterface.call(tConstraint, tStageNode);

        auto tOutputNode = tStageNode.append_child("Output");
        auto tSharedDataName = std::string("Constraint Value ID-") + tConstraint.name();
        XMLGen::append_children({"SharedDataName"}, {tSharedDataName}, tOutputNode);
    }
}
// function append_constraint_value_stage
/******************************************************************************/

/******************************************************************************/
void append_constraint_gradient_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    XMLGen::ConstraintGradientOperation tGradOperationInterface;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tStageNode = aDocument.append_child("Stage");
        auto tStageName = std::string("Compute Constraint Gradient ID-") + tConstraint.name();
        XMLGen::append_children({"Name", "Type"}, {tStageName, tConstraint.category()}, tStageNode);
        auto tInputNode = tStageNode.append_child("Input");
        XMLGen::append_children({"SharedDataName"}, {"Control"}, tInputNode);

        XMLGen::append_filter_control_operation(tStageNode);
        tGradOperationInterface.call(tConstraint, tStageNode);
        auto tSharedDataName = std::string("Constraint Gradient ID-") + tConstraint.name();
        XMLGen::append_filter_criterion_gradient_operation(tSharedDataName, tStageNode);

        auto tOutputNode = tStageNode.append_child("Output");
        XMLGen::append_children({"SharedDataName"}, {tSharedDataName}, tOutputNode);
    }
}
// function append_constraint_gradient_stage
/******************************************************************************/

/******************************************************************************/
void append_derivative_checker_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tCheckGradient = aXMLMetaData.check_gradient.empty() ? std::string("true") : aXMLMetaData.check_gradient;
    std::vector<std::string> tKeys = {"CheckGradient", "CheckHessian", "UseUserInitialGuess"};
    std::vector<std::string> tValues = {tCheckGradient, aXMLMetaData.check_hessian, "True"};
    XMLGen::append_children(tKeys, tValues, aParentNode);

    auto tOptionsNode = aParentNode.append_child("Options");
    tKeys = {"DerivativeCheckerInitialSuperscript", "DerivativeCheckerFinalSuperscript"};
    tValues = {aXMLMetaData.mDerivativeCheckerInitialSuperscript, aXMLMetaData.mDerivativeCheckerFinalSuperscript};
    XMLGen::append_children(tKeys, tValues, tOptionsNode);
}
// function append_derivative_checker_options
/******************************************************************************/

/******************************************************************************/
void append_optimality_criteria_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tConvergenceNode = aParentNode.append_child("Convergence");
    XMLGen::append_children({"MaxIterations"}, {aXMLMetaData.max_iterations}, tConvergenceNode);
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
    std::vector<std::string> tValues = {aXMLMetaData.max_iterations, aXMLMetaData.mMMAMoveLimit, aXMLMetaData.mMMAAsymptoteExpansion,
        aXMLMetaData.mMMAAsymptoteContraction, aXMLMetaData.mMMAMaxNumSubProblemIterations, aXMLMetaData.mMMAControlStagnationTolerance,
        aXMLMetaData.mMMAObjectiveStagnationTolerance};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    auto tOptionsNode = aParentNode.append_child("Options");
    XMLGen::append_children(tKeys, tValues, tOptionsNode);
}
// function append_method_moving_asymptotes_options
/******************************************************************************/

/******************************************************************************/
void append_trust_region_kelley_sachs_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::vector<std::string> tKeys = {"KSTrustRegionExpansionFactor", "KSTrustRegionContractionFactor", "KSMaxTrustRegionIterations", "KSInitialRadiusScale",
         "KSMaxRadiusScale", "HessianType", "MinTrustRegionRadius", "LimitedMemoryStorage", "KSOuterGradientTolerance", "KSOuterStationarityTolerance",
         "KSOuterStagnationTolerance", "KSOuterControlStagnationTolerance", "KSOuterActualReductionTolerance", "ProblemUpdateFrequency", "DisablePostSmoothing",
         "KSTrustRegionRatioLow", "KSTrustRegionRatioMid", "KSTrustRegionRatioUpper"};
    std::vector<std::string> tValues = {aXMLMetaData.mTrustRegionExpansionFactor, aXMLMetaData.mTrustRegionContractionFactor, aXMLMetaData.mMaxTrustRegionIterations,
        aXMLMetaData.mInitialRadiusScale, aXMLMetaData.mMaxRadiusScale, aXMLMetaData.mHessianType, aXMLMetaData.mMinTrustRegionRadius, aXMLMetaData.mLimitedMemoryStorage,
        aXMLMetaData.mOuterGradientToleranceKS, aXMLMetaData.mOuterStationarityToleranceKS, aXMLMetaData.mOuterStagnationToleranceKS, aXMLMetaData.mOuterControlStagnationToleranceKS,
        aXMLMetaData.mOuterActualReductionToleranceKS, aXMLMetaData.mProblemUpdateFrequency, aXMLMetaData.mDisablePostSmoothingKS, aXMLMetaData.mTrustRegionRatioLowKS,
        aXMLMetaData.mTrustRegionRatioMidKS, aXMLMetaData.mTrustRegionRatioUpperKS};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    auto tOptionsNode = aParentNode.append_child("Options");
    XMLGen::append_children(tKeys, tValues, tOptionsNode);
}
// function append_trust_region_kelley_sachs_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_algorithm_parameters_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tLower = XMLGen::to_lower(aXMLMetaData.optimization_algorithm);
    if(tLower.compare("oc") == 0)
    {
        XMLGen::append_optimality_criteria_options(aXMLMetaData, aParentNode);
    }
    else if(tLower.compare("mma") == 0)
    {
        XMLGen::append_method_moving_asymptotes_options(aXMLMetaData, aParentNode);
    }
    else if(tLower.compare("ksbc") == 0)
    {
        XMLGen::append_trust_region_kelley_sachs_options(aXMLMetaData, aParentNode);
    }
    else if(tLower.compare("derivativechecker") == 0)
    {
        XMLGen::append_derivative_checker_options(aXMLMetaData, aParentNode);
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
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(!aXMLMetaData.mOutputMetaData.outputData())
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
    if(!aXMLMetaData.scenario(0u).cacheState())
    {
        return;
    }
    auto tNode = aParentNode.append_child("CacheStage");
    append_children({"Name"}, {"Cache State"}, tNode);
}
// function append_optimization_cache_stage_options
/******************************************************************************/

/******************************************************************************/
void append_optimization_update_problem_stage_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(!aXMLMetaData.scenario(0u).updateProblem())
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
        tKeyToValueMap.find("ValueName")->second = std::string("Objective Value ID-") + tObjective.name;
        tKeyToValueMap.find("ValueStageName")->second = std::string("Compute Objective Value ID-") + tObjective.name;
        tKeyToValueMap.find("GradientName")->second = std::string("Objective Gradient ID-") + tObjective.name;
        tKeyToValueMap.find("GradientStageName")->second = std::string("Compute Objective Gradient ID-") + tObjective.name;

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
          {"ReferenceValueName", "Design Volume"}, {"NormalizedTargetValue", ""}, {"AbsoluteTargetValue", ""} };

    for (auto &tConstraint : aXMLMetaData.constraints)
    {
        tKeyToValueMap.find("ValueName")->second = std::string("Constraint Value ID-") + tConstraint.name();
        tKeyToValueMap.find("ValueStageName")->second = std::string("Compute Constraint Value ID-") + tConstraint.name();
        tKeyToValueMap.find("GradientName")->second = std::string("Constraint Gradient ID-") + tConstraint.name();
        tKeyToValueMap.find("GradientStageName")->second = std::string("Compute Constraint Gradient ID-") + tConstraint.name();
        XMLGen::set_key_value("AbsoluteTargetValue", tConstraint.absoluteTarget(), tKeyToValueMap);
        XMLGen::set_key_value("NormalizedTargetValue", tConstraint.normalizedTarget(), tKeyToValueMap);

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
    XMLGen::append_optimization_variables_options(tOptimizerNode);
    XMLGen::append_optimization_objective_options(aXMLMetaData, tOptimizerNode);
    XMLGen::append_optimization_constraint_options(aXMLMetaData, tOptimizerNode);
    XMLGen::append_optimization_bound_constraints_options({"1.0", "0.0"}, tOptimizerNode);
}
// function append_optimizer_options
/******************************************************************************/

}
// namespace XMLGen
