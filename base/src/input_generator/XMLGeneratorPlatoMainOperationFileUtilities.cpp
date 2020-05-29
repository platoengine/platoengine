/*
 * XMLGeneratorPlatoMainOperationFileUtilities.cpp
 *
 *  Created on: May 28, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
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
void append_filter_options_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document &aDocument)
{
    XMLGen::ValidFilterKeys tValidKeys;
    auto tItr = tValidKeys.mKeys.find(aXMLMetaData.filter_type);
    auto tFilterName = tItr != tValidKeys.mKeys.end() ? tItr->second : "Kernel";
    auto tFilterNode = aDocument.append_child("Filter");
    XMLGen::append_children({"Name"}, {tFilterName}, tFilterNode);
    XMLGen::append_filter_options_to_operation(aXMLMetaData, tFilterNode);
}
// function append_filter_options_to_plato_main_operation
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
    XMLGen::ValidLayoutKeys tValidKeys;
    for(auto& tPair : aXMLMetaData.mOutputMetaData.mDeterministicQuantitiesOfInterest)
    {
        auto tValidLayoutItr = tValidKeys.mKeys.find(tPair.second);
        if(tValidLayoutItr == tValidKeys.mKeys.end())
        {
            THROWERR(std::string("Append Deterministic QOI Inputs to Output Operation: ")
                + "QOI '" + tPair.first + "' layout '" + tPair.second + "' is not supported.")
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
    XMLGen::ValidLayoutKeys tValidKeys;
    for(auto& tPair : aXMLMetaData.mOutputMetaData.mRandomQuantitiesOfInterest)
    {
        auto tFor = aParentNode.append_child("For");
        XMLGen::append_attributes({"var", "in"}, {"SampleIndex", "Samples"}, tFor);
        auto tValidLayoutItr = tValidKeys.mKeys.find(tPair.second);
        if(tValidLayoutItr == tValidKeys.mKeys.end())
        {
            THROWERR(std::string("Append Nondeterministic QOI Inputs to Output Operation: ")
                + "QOI '" + tPair.first + "' layout '" + tPair.second + "' is not supported.")
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
    std::vector<std::string> tValues = {"PlatoMainOutput", "PlatoMainOutput", aXMLMetaData.write_restart_file,
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

/******************************************************************************/
void append_stochastic_objective_value_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document &aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    std::vector<std::string> tKeys = {"Function", "Name", "Layout"};
    std::vector<std::string> tValues = {"MeanPlusStdDev", "Calculate Non-Deterministic Objective Value", "Scalar"};
    XMLGen::append_children(tKeys, tValues, tOperation);

    auto tOuterFor = tOperation.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
    auto tInnerFor = tOuterFor.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
    auto tInnerForInput = tInnerFor.append_child("Input");
    tKeys = {"ArgumentName", "Probability"};
    tValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    XMLGen::append_children(tKeys, tValues, tInnerForInput);

    auto tOuterOutput = tOperation.append_child("Output");
    XMLGen::append_children({"Statistic", "ArgumentName"}, {"mean", "Objective Mean"}, tOuterOutput);
    tOuterOutput = tOperation.append_child("Output");
    XMLGen::append_children({"Statistic", "ArgumentName"}, {"std_dev", "Objective StdDev"}, tOuterOutput);
    tOuterOutput = tOperation.append_child("Output");
    auto tStatistics = std::string("mean_plus_") + aXMLMetaData.objective_number_standard_deviations + "_std_dev";
    auto tArgumentName = std::string("Objective Mean Plus ") + aXMLMetaData.objective_number_standard_deviations + " StdDev";
    XMLGen::append_children({"Statistic", "ArgumentName"}, {tStatistics, tArgumentName}, tOuterOutput);
}
// function append_stochastic_objective_value_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_stochastic_criterion_gradient_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tCriterionGradient = aParentNode.append_child("CriterionGradient");
    XMLGen::append_children({"Layout"}, {"Nodal Field"}, tCriterionGradient);

    auto tOuterFor = tCriterionGradient.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
    auto tInnerFor = tOuterFor.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
    auto tInnerForInput = tInnerFor.append_child("Input");
    std::vector<std::string> tKeys = {"ArgumentName", "Probability"};
    std::vector<std::string> tValues = {"Objective Gradient {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    XMLGen::append_children(tKeys, tValues, tInnerForInput);

    auto tOuterOutput = tCriterionGradient.append_child("Output");
    auto tStatistics = std::string("mean_plus_") + aXMLMetaData.objective_number_standard_deviations + "_std_dev";
    auto tArgumentName = std::string("Objective Mean Plus ")
        + aXMLMetaData.objective_number_standard_deviations + " StdDev Gradient";
    XMLGen::append_children({"Statistic", "ArgumentName"}, {tStatistics, tArgumentName}, tOuterOutput);
}
// function append_stochastic_criterion_gradient_operation
/******************************************************************************/

/******************************************************************************/
void append_stochastic_criterion_value_operation
(pugi::xml_node& aParentNode)
{
    auto tCriterionValue = aParentNode.append_child("CriterionValue");
    XMLGen::append_children({"Layout"}, {"Global"}, tCriterionValue);

    auto tOuterFor = tCriterionValue.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
    auto tInnerFor = tOuterFor.append_child("For");
    XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
    auto tInnerForInput = tInnerFor.append_child("Input");
    std::vector<std::string> tKeys = {"ArgumentName", "Probability"};
    std::vector<std::string> tValues = {"Objective Value {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
        "{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
    XMLGen::append_children(tKeys, tValues, tInnerForInput);

    auto tOuterOutput = tCriterionValue.append_child("Output");
    XMLGen::append_children({"Statistic", "ArgumentName"}, {"mean", "Objective Mean"}, tOuterOutput);
    tOuterOutput = tCriterionValue.append_child("Output");
    XMLGen::append_children({"Statistic", "ArgumentName"}, {"std_dev", "Objective StdDev"}, tOuterOutput);
}
// function append_stochastic_criterion_value_operation
/******************************************************************************/

/******************************************************************************/
void append_stochastic_objective_gradient_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document &aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    std::vector<std::string> tKeys = {"Function", "Name", "Layout"};
    std::vector<std::string> tValues = {"MeanPlusStdDevGradient", "Calculate Non-Deterministic Objective Gradient", "Nodal Field"};
    XMLGen::append_children(tKeys, tValues, tOperation);
    XMLGen::append_stochastic_criterion_value_operation(tOperation);
    XMLGen::append_stochastic_criterion_gradient_operation(aXMLMetaData, tOperation);
}
// function append_stochastic_objective_gradient_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_nondeterministic_qoi_statistics_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document &aDocument)
{
    XMLGen::ValidLayoutKeys tValidKeys;
    for(auto& tPair : aXMLMetaData.mOutputMetaData.mRandomQuantitiesOfInterest)
    {
        auto tOperation = aDocument.append_child("Operation");
        auto tValidLayoutItr = tValidKeys.mKeys.find(tPair.second);
        if(tValidLayoutItr == tValidKeys.mKeys.end())
        {
            THROWERR(std::string("Append Nondeterministic QOI to Statistics Operation: ")
                + "QOI '" + tPair.first + "' layout '" + tPair.second + "' is not supported.")
        }
        auto tName = tPair.first + " Statistics";
        std::vector<std::string> tKeys = {"Function", "Name" , "Layout"};
        std::vector<std::string> tValues = { "MeanPlusStdDev", tName, tValidLayoutItr->second };
        XMLGen::append_children(tKeys, tValues, tOperation);

        auto tOuterFor = tOperation.append_child("For");
        XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
        auto tInnerFor = tOuterFor.append_child("For");
        XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
        auto tInput = tInnerFor.append_child("Input");
        tKeys = {"ArgumentName", "Probability"};
        tValues = {tPair.first + " {PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}",
            "{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
        XMLGen::append_children(tKeys, tValues, tInput);

        auto tOutput = tOperation.append_child("Output");
        auto tArgumentName = tPair.first + " Mean";
        XMLGen::append_children({"Statistic", "ArgumentName"}, {"mean", tArgumentName}, tOutput);
        tOutput = tOperation.append_child("Output");
        tArgumentName = tPair.first + " StdDev";
        XMLGen::append_children({"Statistic", "ArgumentName"}, {"std_dev", tArgumentName}, tOutput);
    }
}
// function append_nondeterministic_qoi_statistics_to_plato_main_operation
/******************************************************************************/

}
// namespace XMLGen
