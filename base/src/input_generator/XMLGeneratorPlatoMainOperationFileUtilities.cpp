/*
 * XMLGeneratorPlatoMainOperationFileUtilities.cpp
 *
 *  Created on: May 28, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeProblem.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"

namespace XMLGen
{

/******************************************************************************/
void write_plato_main_operations_xml_file
(const XMLGen::InputData& aMetaData)
{
    pugi::xml_document tDocument;

    XMLGen::append_include_defines_xml_data(aMetaData, tDocument);
    XMLGen::append_filter_options_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_output_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_initialize_field_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_set_lower_bounds_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_set_upper_bounds_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_copy_field_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_copy_value_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_design_volume_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_compute_volume_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_compute_volume_gradient_to_plato_main_operation(aMetaData, tDocument);

    if(XMLGen::is_robust_optimization_problem(aMetaData))
    {
        XMLGen::append_stochastic_objective_value_to_plato_main_operation(aMetaData, tDocument);
        XMLGen::append_stochastic_objective_gradient_to_plato_main_operation(aMetaData, tDocument);
        XMLGen::append_qoi_statistics_to_plato_main_operation(aMetaData, tDocument);
    }

    XMLGen::append_update_problem_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_filter_control_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_filter_gradient_to_plato_main_operation(aMetaData, tDocument);
    //if(!XMLGen::is_robust_optimization_problem(aMetaData))
    if(aMetaData.needToAggregate())
    {
        XMLGen::append_aggregate_data_to_plato_main_operation(aMetaData, tDocument);
    }

    XMLGen::append_csm_mesh_output_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_initialize_geometry_operation_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_update_geometry_on_change_operation_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_enforce_bounds_operation_to_plato_main_operation(aMetaData, tDocument);

    tDocument.save_file("plato_main_operations.xml", "  ");
}
/******************************************************************************/

/******************************************************************************/
bool is_volume_constraint_defined
(const XMLGen::InputData& aXMLMetaData)
{
    auto tVolumeConstraintDefined = false;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tCriterion = aXMLMetaData.criterion(tConstraint.criterion());
        auto tType = Plato::tolower(tCriterion.type());
        tVolumeConstraintDefined = tType.compare("volume") == 0;
        if (tVolumeConstraintDefined == true)
        {
            break;
        }
    }
    return tVolumeConstraintDefined;
}
// function is_volume_constraint_defined
/******************************************************************************/

/******************************************************************************/
bool is_volume_constraint_defined_and_computed_by_platomain
(const XMLGen::InputData& aXMLMetaData)
{
    auto tIsVolumeDefinedAndComputedByPlatoMain = false;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tCriterion = aXMLMetaData.criterion(tConstraint.criterion());
        auto tType = Plato::tolower(tCriterion.type());
        auto tService = aXMLMetaData.service(tConstraint.service());
        auto tCode = Plato::tolower(tService.code());

        auto tIsVolumeConstraintDefined = tType.compare("volume") == 0;
        auto tIsVolumeComputedByPlatoMain = tCode.compare("platomain") == 0;
        tIsVolumeDefinedAndComputedByPlatoMain = tIsVolumeConstraintDefined && tIsVolumeComputedByPlatoMain;
        if (tIsVolumeDefinedAndComputedByPlatoMain == true)
        {
            break;
        }
    }
    return tIsVolumeDefinedAndComputedByPlatoMain;
}
//function is_volume_constraint_defined_and_computed_by_platomain
/******************************************************************************/

/******************************************************************************/
void append_filter_options_to_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    std::vector<std::string> tKeys = {"Scale", "Absolute", "StartIteration", "UpdateInterval",
        "UseAdditiveContinuation", "Power", "HeavisideMin", "HeavisideUpdate", "HeavisideMax"};

    auto tScale = aXMLMetaData.optimization_parameters().filter_radius_scale().empty() ? std::string("2.0") : aXMLMetaData.optimization_parameters().filter_radius_scale();
    std::vector<std::string> tValues = {tScale, aXMLMetaData.optimization_parameters().filter_radius_absolute(),
        aXMLMetaData.optimization_parameters().filter_projection_start_iteration(), aXMLMetaData.optimization_parameters().filter_projection_update_interval(),
        aXMLMetaData.optimization_parameters().filter_use_additive_continuation(), aXMLMetaData.optimization_parameters().filter_power(), aXMLMetaData.optimization_parameters().filter_heaviside_min(),
        aXMLMetaData.optimization_parameters().filter_heaviside_update(), aXMLMetaData.optimization_parameters().filter_heaviside_max()};

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
    if(aXMLMetaData.optimization_parameters().filter_in_engine() == "true" &&
       aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        XMLGen::ValidFilterKeys tValidKeys;
        auto tValue = tValidKeys.value(aXMLMetaData.optimization_parameters().filter_type());
        auto tFilterName = tValue.empty() ? "Kernel" : tValue;
        auto tFilterNode = aDocument.append_child("Filter");
        XMLGen::append_children({"Name"}, {tFilterName}, tFilterNode);
        XMLGen::append_filter_options_to_operation(aXMLMetaData, tFilterNode);
    }
}
// function append_filter_options_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_enforce_bounds_operation_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document &aDocument)
{
    if(aXMLMetaData.optimization_parameters().enforce_bounds() == "true")
    {
        auto tOperationNode = aDocument.append_child("Operation");
        XMLGen::append_children({"Name","Function"}, {"EnforceBounds","EnforceBounds"}, tOperationNode);
        auto tInputNode = tOperationNode.append_child("Input");
        XMLGen::append_children({"ArgumentName"}, {"Upper Bound Vector"}, tInputNode);
        tInputNode = tOperationNode.append_child("Input");
        XMLGen::append_children({"ArgumentName"}, {"Lower Bound Vector"}, tInputNode);
        tInputNode = tOperationNode.append_child("Input");
        XMLGen::append_children({"ArgumentName"}, {"Topology"}, tInputNode);
        auto tOutputNode = tOperationNode.append_child("Output");
        XMLGen::append_children({"ArgumentName"}, {"Topology"}, tOutputNode);
    }
}
// function append_enforce_bounds_operation_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_constraint_gradient_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tInput = aParentNode.append_child("Input");
        auto tArgumentName = std::string("constraint gradient ") + tConstraint.id();
        XMLGen::append_children( { "ArgumentName" }, { tArgumentName }, tInput);
    }
}
// function append_constraint_gradient_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_objective_gradient_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
/*
    bool tNeedToAggregate = aXMLMetaData.needToAggregate();

    if(tNeedToAggregate)
    {
*/
 // I think this will always be "objective gradient" but I am leaving the old code
 // in just in case we run into a situation when that isn't the case.
        auto tInput = aParentNode.append_child("Input");
        XMLGen::append_children( { "ArgumentName" }, { "objective gradient" }, tInput);
/*
    }
    else
    {
        for(size_t i=0; i<aXMLMetaData.objective.criteriaIDs.size(); ++i)
        {
            std::string tCriterionID = aXMLMetaData.objective.criteriaIDs[i];
            std::string tServiceID = aXMLMetaData.objective.serviceIDs[i];
            std::string tScenarioID = aXMLMetaData.objective.scenarioIDs[i];
            ConcretizedCriterion tConcretizedCriterion(tCriterionID,tServiceID,tScenarioID);
            auto tIdentifierString = XMLGen::get_concretized_criterion_identifier_string(tConcretizedCriterion);

            auto tInput = aParentNode.append_child("Input");
            std::string tName = std::string("Criterion Gradient - ");
            tName += tIdentifierString;
            auto tArgumentName = XMLGen::to_lower(tName);
            XMLGen::append_children( { "ArgumentName" }, { tArgumentName }, tInput);
        }
    }
*/
}
// function append_objective_gradient_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_qoi_statistics_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    for(auto &tOutputMetadata : aXMLMetaData.mOutputMetaData)
    {
        auto tOutputQoIs = tOutputMetadata.randomIDs();
        for(auto& tOutputQoI : tOutputQoIs)
        {
            auto tLayout = tOutputMetadata.randomLayout(tOutputQoI);
            auto tValidLayout = XMLGen::check_data_layout(tLayout);
            auto tArgumentName = tOutputQoI + " mean";
            auto tInput= aParentNode.append_child("Input");
            XMLGen::append_children( { "ArgumentName", "Layout" }, { tArgumentName, tValidLayout }, tInput);

            tArgumentName = tOutputQoI + " standard deviation";
            tInput= aParentNode.append_child("Input");
            XMLGen::append_children( { "ArgumentName", "Layout" }, { tArgumentName, tValidLayout }, tInput);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_deterministic_qoi_to_output_operation_for_non_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    for(auto &tOutputMetadata : aXMLMetaData.mOutputMetaData)
    {
        auto tIDs = tOutputMetadata.deterministicIDs();
        for(auto& tID : tIDs)
        {
            auto tInput= aParentNode.append_child("Input");
            auto tLayout = tOutputMetadata.deterministicLayout(tID);
            auto tArgumentName = tOutputMetadata.deterministicArgumentName(tID);
            auto tPerformerString = std::string("_") + 
                                    aXMLMetaData.service(tOutputMetadata.serviceID()).performer();
            tArgumentName += tPerformerString;
            XMLGen::append_children( { "ArgumentName", "Layout" }, { tArgumentName, tLayout }, tInput);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_deterministic_qoi_to_output_operation_for_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    for(auto &tOutputMetadata : aXMLMetaData.mOutputMetaData)
    {
        auto tIDs = tOutputMetadata.deterministicIDs();
        for(auto& tID : tIDs)
        {
            auto tLayout = tOutputMetadata.deterministicLayout(tID);
            for(auto tScenarioID : aXMLMetaData.objective.scenarioIDs)
            {
                auto tArgumentName = tOutputMetadata.deterministicArgumentName(tID);
                auto tInput = aParentNode.append_child("Input");
                auto tPerformerString = std::string("_") + 
                                        aXMLMetaData.service(tOutputMetadata.serviceID()).performer();
                tArgumentName += tPerformerString;
                tArgumentName += "_scenario_" + tScenarioID;
                XMLGen::append_children( { "ArgumentName", "Layout" }, { tArgumentName, tLayout }, tInput);
            }
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_deterministic_qoi_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    if(aXMLMetaData.objective.multi_load_case == "true")
    {
        append_deterministic_qoi_to_output_operation_for_multi_load_case(aXMLMetaData, aParentNode);
    }
    else
    {
        append_deterministic_qoi_to_output_operation_for_non_multi_load_case(aXMLMetaData, aParentNode);
    }
}
// function append_deterministic_qoi_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_stochastic_qoi_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    for(auto &tOutputMetadata : aXMLMetaData.mOutputMetaData)
    {
        if(tOutputMetadata.outputSamples() == false)
        {
            return;
        }

        auto tIDs = tOutputMetadata .randomIDs();
        for(auto& tID : tIDs)
        {
            auto tFor = aParentNode.append_child("For");
            XMLGen::append_attributes({"var", "in"}, {"SampleIndex", "Samples"}, tFor);

            auto tInput= tFor.append_child("Input");
            auto tLayout = tOutputMetadata .randomLayout(tID);
            auto tArgumentName = tID + " {SampleIndex}";
            XMLGen::append_children( { "ArgumentName", "Layout" }, { tArgumentName, tLayout }, tInput);
        }
    }
}
// function append_stochastic_qoi_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_children_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    std::vector<std::string> tKeys = {"Function", "Name", "WriteRestart", "OutputFrequency", "MaxIterations", "RestartFieldName"};
    std::vector<std::string> tValues = {"PlatoMainOutput", "PlatoMainOutput", aXMLMetaData.optimization_parameters().write_restart_file(),
        aXMLMetaData.optimization_parameters().output_frequency(), aXMLMetaData.optimization_parameters().max_iterations(), aXMLMetaData.optimization_parameters().initial_guess_field_name()};
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
    auto tLayout = XMLGen::return_output_qoi_data_layout("topology");
    auto tValidLayoutKeyword = XMLGen::check_data_layout_keyword(tLayout);
    XMLGen::append_children({"ArgumentName", "Layout"}, {"topology", tValidLayoutKeyword}, tInput);

    tInput = aParentNode.append_child("Input");
    tLayout = XMLGen::return_output_qoi_data_layout("control");
    tValidLayoutKeyword = XMLGen::check_data_layout_keyword(tLayout);
    XMLGen::append_children({"ArgumentName", "Layout"}, {"control", tValidLayoutKeyword}, tInput);
    XMLGen::append_objective_gradient_to_output_operation(aXMLMetaData, aParentNode);
    XMLGen::append_constraint_gradient_to_output_operation(aXMLMetaData, aParentNode);
}
// function append_default_qoi_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_surface_extraction_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    if (!aXMLMetaData.optimization_parameters().output_method().empty())
    {
        auto tSurfaceExtraction = aParentNode.append_child("SurfaceExtraction");
        std::vector<std::string> tKeys = { "OutputMethod", "Discretization" };
        auto tDiscretization = aXMLMetaData.optimization_parameters().discretization().empty() ? "density" : aXMLMetaData.optimization_parameters().discretization();
        std::vector<std::string> tValues = { aXMLMetaData.optimization_parameters().output_method(), tDiscretization };
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
    if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        return;
    }
    for(auto &tOutputMetadata : aXMLMetaData.mOutputMetaData)
    {
        if(tOutputMetadata.isOutputDisabled())
        {
            return;
        }
    }
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children_to_output_operation(aXMLMetaData, tOperation);
    XMLGen::append_default_qoi_to_output_operation(aXMLMetaData, tOperation);
    XMLGen::append_qoi_statistics_to_output_operation(aXMLMetaData, tOperation);
    XMLGen::append_stochastic_qoi_to_output_operation(aXMLMetaData, tOperation);
    XMLGen::append_deterministic_qoi_to_output_operation(aXMLMetaData, tOperation);
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
    std::vector<std::string> tValues = {"MeanPlusStdDev", "Compute Non-Deterministic Objective Value", "Scalar"};
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
    auto tStatistics = std::string("mean_plus_") + aXMLMetaData.optimization_parameters().objective_number_standard_deviations() + "_std_dev";
    auto tArgumentName = std::string("Objective Mean Plus ") + aXMLMetaData.optimization_parameters().objective_number_standard_deviations() + " StdDev";
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
    auto tStatistics = std::string("mean_plus_") + aXMLMetaData.optimization_parameters().objective_number_standard_deviations() + "_std_dev";
    auto tArgumentName = std::string("Objective Mean Plus ")
        + aXMLMetaData.optimization_parameters().objective_number_standard_deviations() + " StdDev Gradient";
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
    std::vector<std::string> tValues = {"MeanPlusStdDevGradient", "Compute Non-Deterministic Objective Gradient", "Nodal Field"};
    XMLGen::append_children(tKeys, tValues, tOperation);
    XMLGen::append_stochastic_criterion_value_operation(tOperation);
    XMLGen::append_stochastic_criterion_gradient_operation(aXMLMetaData, tOperation);
}
// function append_stochastic_objective_gradient_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_qoi_statistics_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document &aDocument)
{
    for(auto &tOutputMetadata : aXMLMetaData.mOutputMetaData)
    {
        auto tOutputDataIDs = tOutputMetadata.randomIDs();
        for(auto& tOutputDataID : tOutputDataIDs)
        {
            auto tDataLayout = tOutputMetadata.randomLayout(tOutputDataID);
            auto tSupportedDataLayout = XMLGen::check_data_layout(tDataLayout);
            auto tOperationName = "compute " + tOutputDataID + " statistics";
            std::vector<std::string> tKeys = {"Function", "Name" , "Layout"};
            std::vector<std::string> tValues = { "MeanPlusStdDev", tOperationName, tSupportedDataLayout };
            auto tOperation = aDocument.append_child("Operation");
            XMLGen::append_children(tKeys, tValues, tOperation);

            auto tOuterFor = tOperation.append_child("For");
            XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
            auto tInnerFor = tOuterFor.append_child("For");
            XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
            auto tInput = tInnerFor.append_child("Input");
            tKeys = {"ArgumentName", "Probability"};
            auto tArgumentName = tOutputMetadata.randomArgumentName(tOutputDataID);
            tValues = {tArgumentName, "{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
            XMLGen::append_children(tKeys, tValues, tInput);

            auto tOutput = tOperation.append_child("Output");
            tArgumentName = tOutputDataID + " mean";
            XMLGen::append_children({"Statistic", "ArgumentName"}, {"mean", tArgumentName}, tOutput);
            tOutput = tOperation.append_child("Output");
            tArgumentName = tOutputDataID + " standard deviation";
            XMLGen::append_children({"Statistic", "ArgumentName"}, {"std_dev", tArgumentName}, tOutput);
        }
    }
}
// function append_qoi_statistics_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_aggregate_data_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children({"Name", "Function"}, {"Aggregate Data", "Aggregator"}, tOperation);
    XMLGen::Objective tObjective = aXMLMetaData.objective;
    // Aggregate values
    auto tAggregateNode = tOperation.append_child("Aggregate");
    XMLGen::append_children({"Layout"}, {"Value"}, tAggregateNode);

    int tNumEntries = tObjective.criteriaIDs.size();
    if(tObjective.multi_load_case == "true")
    {
        tNumEntries = 1;
    }
    for (int i=0; i<tNumEntries; ++i)
    {
        auto tInput = tAggregateNode.append_child("Input");
        auto tArgName = std::string("Value ") + std::to_string(i+1);
        XMLGen::append_children({"ArgumentName"}, {tArgName}, tInput);
    }
    auto tOutput = tAggregateNode.append_child("Output");
    XMLGen::append_children({"ArgumentName"}, {"Value"}, tOutput);

    // Aggregate fields
    tAggregateNode = tOperation.append_child("Aggregate");
    XMLGen::append_children({"Layout"}, {"Nodal Field"}, tAggregateNode);

    for (int i=0; i<tNumEntries; ++i)
    {
        auto tInput = tAggregateNode.append_child("Input");
        auto tArgName = std::string("Field ") + std::to_string(i+1);
        XMLGen::append_children({"ArgumentName"}, {tArgName}, tInput);
    }
    tOutput = tAggregateNode.append_child("Output");
    XMLGen::append_children({"ArgumentName"}, {"Field"}, tOutput);

    // Weighting and normalization
    bool tNeedToDoWeightingInAggregator = aXMLMetaData.needToDoWeightingInAggregator();
    auto tWeightingNode = tOperation.append_child("Weighting");
    for (int i=0; i<tNumEntries; ++i)
    {
        auto tWeight = tWeightingNode.append_child("Weight");
        if(tNeedToDoWeightingInAggregator)
        {
            XMLGen::append_children({"Value"}, {tObjective.weights[i]}, tWeight);
        }
        else
        {
            XMLGen::append_children({"Value"}, {"1.0"}, tWeight);
        }
    }
    if(aXMLMetaData.normalizeInAggregator())
    {
        auto tNormals = tWeightingNode.append_child("Normals");
        for (int i=0; i<tNumEntries; ++i)
        {
            auto tInput = tNormals.append_child("Input");
            auto tArgName = std::string("Normal ") + std::to_string(i+1);
            XMLGen::append_children({"ArgumentName"}, {tArgName}, tInput);
        }
    }
}
// function append_aggregate_data_to_plato_main_operation
/******************************************************************************/
/******************************************************************************/
void append_update_problem_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    bool tNeedUpdate = false;
    for(auto &tService : aXMLMetaData.services())
    {
        if(tService.updateProblem())
        {
            tNeedUpdate = true;
            break;
        }
    }
    if(!tNeedUpdate)
    {
        return;
    }
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children({"Function", "Name"}, {"Update Problem", "Update Problem"}, tOperation);
}
// function append_update_problem_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_filter_control_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children({"Function", "Name", "Gradient"}, {"Filter", "Filter Control", "False"}, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children({"ArgumentName"}, {"Field"}, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children({"ArgumentName"}, {"Filtered Field"}, tOutput);
    }
}
// function append_filter_control_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_initialize_geometry_operation_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        pugi::xml_node tmp_node = aDocument.append_child("Operation");
        addChild(tmp_node, "Function", "SystemCall");
        addChild(tmp_node, "Name", "Initialize Geometry");
        addChild(tmp_node, "Command", "plato-cli geometry esp");
        addChild(tmp_node, "Argument", std::string("--input ") + aXMLMetaData.optimization_parameters().csm_file());
        addChild(tmp_node, "Argument", std::string("--output-model ") + aXMLMetaData.optimization_parameters().csm_opt_file());
        addChild(tmp_node, "Argument", std::string("--output-mesh ") + aXMLMetaData.optimization_parameters().csm_exodus_file());
        addChild(tmp_node, "Argument", std::string("--tesselation ") + aXMLMetaData.optimization_parameters().csm_tesselation_file());
    }
}
// function append_initialize_geometry_operation_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_update_geometry_on_change_operation_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        pugi::xml_node tmp_node = aDocument.append_child("Operation");
        addChild(tmp_node, "Function", "SystemCall");
        addChild(tmp_node, "Name", "Update Geometry on Change");
        addChild(tmp_node, "Command", "plato-cli geometry esp");
        addChild(tmp_node, "OnChange", "true");
        addChild(tmp_node, "Argument", std::string("--input ") + aXMLMetaData.optimization_parameters().csm_file());
        addChild(tmp_node, "Argument", std::string("--output-model ") + aXMLMetaData.optimization_parameters().csm_opt_file());
        addChild(tmp_node, "Argument", std::string("--output-mesh ") + aXMLMetaData.optimization_parameters().csm_exodus_file());
        addChild(tmp_node, "Argument", std::string("--tesselation ") + aXMLMetaData.optimization_parameters().csm_tesselation_file());
        addChild(tmp_node, "Argument", "--parameters");
        addChild(tmp_node, "AppendInput", "true");
        pugi::xml_node tmp_node1 = tmp_node.append_child("Input");
        addChild(tmp_node1, "ArgumentName", "Parameters");
    }
}
// function append_update_geometry_on_change_operation_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_csm_mesh_output_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        pugi::xml_node tmp_node = aDocument.append_child("Operation");
        addChild(tmp_node, "Function", "CSMMeshOutput");
        addChild(tmp_node, "Name", "CSMMeshOutput");
        addChild(tmp_node, "BaseMeshName", aXMLMetaData.optimization_parameters().csm_exodus_file());
        addChild(tmp_node, "OutputFrequency", aXMLMetaData.optimization_parameters().output_frequency());
        addChild(tmp_node, "MaxIterations", aXMLMetaData.optimization_parameters().max_iterations());
    }
}
// function append_csm_mesh_output_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_filter_gradient_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children({"Function", "Name", "Gradient"}, {"Filter", "Filter Gradient", "True"}, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children({"ArgumentName"}, {"Field"}, tInput);
        tInput = tOperation.append_child("Input");
        XMLGen::append_children({"ArgumentName"}, {"Gradient"}, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children({"ArgumentName"}, {"Filtered Gradient"}, tOutput);
    }
}
// function append_filter_gradient_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_initialize_density_field_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children({"Function", "Name", "Method"}, {"InitializeField", "Initialize Field", "Uniform"}, tOperation);

    auto tMethod = tOperation.append_child("Uniform");
    auto tValue = aXMLMetaData.optimization_parameters().initial_density_value().empty() ? "0.5" : aXMLMetaData.optimization_parameters().initial_density_value();
    XMLGen::append_children({"Value"}, {tValue}, tMethod);

    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children({"ArgumentName"}, {"Initialized Field"}, tOutput);
}
// function append_initialize_density_field_operation
/******************************************************************************/

/******************************************************************************/
void append_initialize_field_from_file_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().initial_guess_file_name().empty())
    {
        THROWERR(std::string("Append Initialize Field From File Operation: ")
            + "Initial guess was supposed to be initialized by reading it from an user-specified file. "
            + "However, the 'filename' keyword is empty.")
    }

    if(aXMLMetaData.optimization_parameters().initial_guess_field_name().empty())
    {
        THROWERR(std::string("Append Initialize Field From File Operation: ")
            + "Initial guess was supposed to be initialized by reading it from an user-specified field. "
            + "However, the field's 'name' keyword is empty.")
    }

    auto tOperation = aDocument.append_child("Operation");
    std::vector<std::string> tKeys = {"Function", "Name", "Method"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "FromFieldOnInputMesh"};
    XMLGen::append_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.append_child("FromFieldOnInputMesh");
    tKeys = {"Name", "VariableName", "Iteration"};
    tValues = {aXMLMetaData.optimization_parameters().initial_guess_file_name(), aXMLMetaData.optimization_parameters().initial_guess_field_name(), aXMLMetaData.optimization_parameters().restart_iteration()};
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    XMLGen::append_children(tKeys, tValues, tMethod);
}
// function append_initialize_field_from_file_operation
/******************************************************************************/

/******************************************************************************/
void append_levelset_material_box
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tAppendMaterialBox = !aXMLMetaData.optimization_parameters().levelset_material_box_min().empty() &&
        !aXMLMetaData.optimization_parameters().levelset_material_box_max().empty();
    if(tAppendMaterialBox)
    {
        auto tMaterialBox = aParentNode.append_child("MaterialBox");
        std::vector<std::string> tKeys = {"MinCoords", "MaxCoords"};
        std::vector<std::string> tValues = {aXMLMetaData.optimization_parameters().levelset_material_box_min(),
            aXMLMetaData.optimization_parameters().levelset_material_box_max()};
        XMLGen::append_children(tKeys, tValues, tMaterialBox);
    }
}
// function append_levelset_material_box
/******************************************************************************/

/******************************************************************************/
void append_initialize_levelset_primitives_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.mesh.run_name.empty())
    {
        THROWERR(std::string("Append Initialize Levelset Primitives Operation: ")
            + "Levelset field was supposed to be initialized by reading it from an user-specified file. "
            + "However, the 'background mesh' keyword is empty.")
    }

    auto tOperation = aDocument.append_child("Operation");
    std::vector<std::string> tKeys = {"Function", "Name", "Method"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "PrimitivesLevelSet"};
    XMLGen::append_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.append_child("PrimitivesLevelSet");
    XMLGen::append_children({"BackgroundMeshName"}, {aXMLMetaData.mesh.run_name}, tMethod);
    XMLGen::append_levelset_material_box(aXMLMetaData, tMethod);
}
// function append_initialize_levelset_primitives_operation
/******************************************************************************/

/******************************************************************************/
void append_initialize_levelset_swiss_cheese_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.mesh.run_name.empty())
    {
        THROWERR(std::string("Append Initialize Levelset Swiss Cheese Operation: ")
            + "Levelset field was supposed to be initialized by writing it into an user-specified file. "
            + "However, the 'background mesh' keyword is empty.")
    }

    auto tOperation = aDocument.append_child("Operation");
    std::vector<std::string> tKeys = {"Function", "Name", "Method"};
    std::vector<std::string> tValues = {"InitializeField", "Initialize Field", "SwissCheeseLevelSet"};
    XMLGen::append_children(tKeys, tValues, tOperation);

    auto tMethod = tOperation.append_child("SwissCheeseLevelSet");
    tKeys = {"BackgroundMeshName", "SphereRadius", "SpherePackingFactor"};
    tValues = {aXMLMetaData.mesh.run_name, aXMLMetaData.optimization_parameters().levelset_sphere_radius(),
        aXMLMetaData.optimization_parameters().levelset_sphere_packing_factor()};
    for(auto& tNodeSet : aXMLMetaData.optimization_parameters().levelset_nodesets())
    {
        tKeys.push_back("NodeSet"); tValues.push_back(tNodeSet);
    }

    auto tDefineCreateLevelSetSpheresKeyword =
        aXMLMetaData.optimization_parameters().levelset_sphere_radius().empty() && aXMLMetaData.optimization_parameters().levelset_sphere_packing_factor().empty();
    auto tCreateLevelSetSpheres = tDefineCreateLevelSetSpheresKeyword ? "false" : "true";
    tKeys.push_back("CreateSpheres"); tValues.push_back(tCreateLevelSetSpheres);
    XMLGen::set_value_keyword_to_ignore_if_empty(tValues);
    XMLGen::append_children(tKeys, tValues, tMethod);
}
// function append_initialize_levelset_swiss_cheese_operation
/******************************************************************************/

/******************************************************************************/
void append_initialize_levelset_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    XMLGen::ValidLevelSetInitKeys tValidKeys;
    auto tLowerKey = Plato::tolower(aXMLMetaData.optimization_parameters().levelset_initialization_method());
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerKey);
    if(tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Append Initialize Levelset Operation: ") + "Levelset initialization method '"
            + tLowerKey + "' is not supported.")
    }

    if(tItr->compare("primitives") == 0)
    {
        XMLGen::append_initialize_levelset_primitives_operation(aXMLMetaData, aDocument);
    }
    else if(tItr->compare("swiss_cheese") == 0)
    {
        XMLGen::append_initialize_levelset_swiss_cheese_operation(aXMLMetaData, aDocument);
    }
}
// function append_initialize_levelset_operation
/******************************************************************************/

/******************************************************************************/
void append_initialize_field_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    XMLGen::ValidDiscretizationKeys tValidKeys;
    auto tValue = tValidKeys.value(aXMLMetaData.optimization_parameters().discretization());
    if(tValue.empty())
    {
        THROWERR(std::string("Append Initialize Field to Plato Main Operation: ") + "Discretization method '" + tValue + "' is not supported.")
    }

    if(tValue.compare("density") == 0)
    {
        XMLGen::append_initialize_density_field_operation(aXMLMetaData, aDocument);
    }
    else if(tValue.compare("levelset") == 0)
    {
        XMLGen::append_initialize_levelset_operation(aXMLMetaData, aDocument);
    }
}
// function append_initialize_field_operation
/******************************************************************************/

/******************************************************************************/
void append_initialize_field_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
    {
        append_initialize_data_for_shape_problem(aXMLMetaData, aDocument);
    }
    else if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        append_initialize_data_for_topology_problem(aXMLMetaData, aDocument);
    }
}
// function append_initialize_field_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_initialize_data_for_topology_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().initial_guess_file_name().empty())
    {
        XMLGen::append_initialize_field_operation(aXMLMetaData, aDocument);
    }
    else
    {
        XMLGen::append_initialize_field_from_file_operation(aXMLMetaData, aDocument);
    }
}
// function append_initialize_data_for_topology_problem
/******************************************************************************/

/******************************************************************************/
void append_initialize_data_for_shape_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    pugi::xml_node tmp_node = aDocument.append_child("Operation");
    addChild(tmp_node, "Function", "InitializeValues");
    addChild(tmp_node, "Name", "Initialize Values");
    pugi::xml_node tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Values");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Lower Bounds");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Upper Bounds");
    addChild(tmp_node, "Method", "ReadFromCSMFile");
    addChild(tmp_node, "CSMFileName", aXMLMetaData.optimization_parameters().csm_file());
}
// function append_initialize_data_for_topology_problem
/******************************************************************************/

/******************************************************************************/
void append_design_volume_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(XMLGen::is_volume_constraint_defined(aXMLMetaData))
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children({"Function", "Name"}, {"DesignVolume", "Design Volume"}, tOperation);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children({"ArgumentName"}, {"Design Volume"}, tOutput);
    }
}
// function append_design_volume_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_compute_volume_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(XMLGen::is_volume_constraint_defined_and_computed_by_platomain(aXMLMetaData))
    {
        auto tOperation = aDocument.append_child("Operation");
        std::vector<std::string> tKeys = {"Function", "Name", "PenaltyModel"};
        std::vector<std::string> tValues = {"ComputeVolume", "Compute Constraint Value", "SIMP"};
        XMLGen::append_children(tKeys, tValues, tOperation);

        tKeys = {"ArgumentName"}; tValues = {"Topology"};
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children(tKeys, tValues, tInput);

        tKeys = {"ArgumentName"}; tValues = {"Volume"};
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children(tKeys, tValues, tOutput);

        tKeys = {"ArgumentName"}; tValues = {"Volume Gradient"};
        tOutput = tOperation.append_child("Output");
        XMLGen::append_children(tKeys, tValues, tOutput);

        tKeys = {"PenaltyExponent", "MinimumValue"}; tValues = {"1.0", "0.0"};
        auto tSIMP = tOperation.append_child("SIMP");
        XMLGen::append_children(tKeys, tValues, tSIMP);
    }
}
// function append_compute_volume_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_compute_volume_gradient_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(XMLGen::is_volume_constraint_defined_and_computed_by_platomain(aXMLMetaData))
    {
        auto tOperation = aDocument.append_child("Operation");
        std::vector<std::string> tKeys = {"Function", "Name", "PenaltyModel"};
        std::vector<std::string> tValues = {"ComputeVolume", "Compute Constraint Gradient", "SIMP"};
        XMLGen::append_children(tKeys, tValues, tOperation);

        tKeys = {"ArgumentName"}; tValues = {"Topology"};
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children(tKeys, tValues, tInput);

        tKeys = {"ArgumentName"}; tValues = {"Volume"};
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children(tKeys, tValues, tOutput);

        tKeys = {"ArgumentName"}; tValues = {"Volume Gradient"};
        tOutput = tOperation.append_child("Output");
        XMLGen::append_children(tKeys, tValues, tOutput);

        tKeys = {"PenaltyExponent", "MinimumValue"}; tValues = {"1.0", "0.0"};
        auto tSIMP = tOperation.append_child("SIMP");
        XMLGen::append_children(tKeys, tValues, tSIMP);
    }
}
// function append_compute_volume_gradient_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_fixed_blocks_identification_numbers_to_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.optimization_parameters().fixed_block_ids().size() > 0)
    {
        auto tFixedBlocks = aParentNode.append_child("FixedBlocks");
        for(auto& tID : aXMLMetaData.optimization_parameters().fixed_block_ids())
        {
            XMLGen::append_children({"Index"}, {tID}, tFixedBlocks);
        }
    }
}
// function append_fixed_blocks_identification_numbers_to_operation
/******************************************************************************/

/******************************************************************************/
void append_fixed_sidesets_identification_numbers_to_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.optimization_parameters().fixed_sideset_ids().size() > 0)
    {
        auto tFixedSideSet = aParentNode.append_child("FixedSidesets");
        for(auto& tID : aXMLMetaData.optimization_parameters().fixed_sideset_ids())
        {
            XMLGen::append_children({"Index"}, {tID}, tFixedSideSet);
        }
    }
}
// function append_fixed_sidesets_identification_numbers_to_operation
/******************************************************************************/

/******************************************************************************/
void append_fixed_nodesets_identification_numbers_to_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.optimization_parameters().fixed_nodeset_ids().size() > 0)
    {
        auto tFixedNodeSet = aParentNode.append_child("FixedNodesets");
        for(auto& tID : aXMLMetaData.optimization_parameters().fixed_nodeset_ids())
        {
            XMLGen::append_children({"Index"}, {tID}, tFixedNodeSet);
        }
    }
}
// function append_fixed_nodesets_identification_numbers_to_operation
/******************************************************************************/

/******************************************************************************/
void append_set_lower_bounds_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        auto tOperation = aDocument.append_child("Operation");
        std::vector<std::string> tKeys = {"Function", "Name", "Discretization"};
        std::vector<std::string> tValues = {"SetLowerBounds", "Compute Lower Bounds", aXMLMetaData.optimization_parameters().discretization()};
        XMLGen::append_children(tKeys, tValues, tOperation);

        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children({"ArgumentName"}, {"Lower Bound Value"}, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children({"ArgumentName"}, {"Lower Bound Vector"}, tOutput);

        XMLGen::append_fixed_blocks_identification_numbers_to_operation(aXMLMetaData, tOperation);
        XMLGen::append_fixed_sidesets_identification_numbers_to_operation(aXMLMetaData, tOperation);
        XMLGen::append_fixed_nodesets_identification_numbers_to_operation(aXMLMetaData, tOperation);
    }
}
// function append_set_lower_bounds_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_copy_field_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    std::vector<std::string> tKeys = {"Function", "Name"};
    std::vector<std::string> tValues = {"CopyField", "Copy Field"};
    XMLGen::append_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"InputField"}, tInput);
    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children({"ArgumentName"}, {"OutputField"}, tOutput);
}
// function append_copy_field_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_copy_value_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    std::vector<std::string> tKeys = {"Function", "Name"};
    std::vector<std::string> tValues = {"CopyValue", "Copy Value"};
    XMLGen::append_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"InputValue"}, tInput);
    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children({"ArgumentName"}, {"OutputValue"}, tOutput);
}
// function append_copy_value_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_set_upper_bounds_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
    {
        auto tOperation = aDocument.append_child("Operation");
        std::vector<std::string> tKeys = {"Function", "Name", "Discretization"};
        std::vector<std::string> tValues = {"SetUpperBounds", "Compute Upper Bounds", aXMLMetaData.optimization_parameters().discretization()};
        XMLGen::append_children(tKeys, tValues, tOperation);

        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children({"ArgumentName"}, {"Upper Bound Value"}, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children({"ArgumentName"}, {"Upper Bound Vector"}, tOutput);

        XMLGen::append_fixed_blocks_identification_numbers_to_operation(aXMLMetaData, tOperation);
        XMLGen::append_fixed_sidesets_identification_numbers_to_operation(aXMLMetaData, tOperation);
        XMLGen::append_fixed_nodesets_identification_numbers_to_operation(aXMLMetaData, tOperation);
    }
}
// function append_set_upper_bounds_to_plato_main_operation
/******************************************************************************/

}
// namespace XMLGen
