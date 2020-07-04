/*
 * XMLGeneratorPlatoMainOperationFileUtilities.cpp
 *
 *  Created on: May 28, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorParserUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"

namespace XMLGen
{

/******************************************************************************/
void write_plato_main_operations_xml_file_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData)
{
    pugi::xml_document tDocument;
    auto tInclude = tDocument.append_child("include");
    XMLGen::append_attributes({"filename"}, {"defines.xml"}, tInclude);

    XMLGen::append_filter_options_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_output_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_initialize_field_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_set_lower_bounds_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_set_upper_bounds_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_design_volume_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_volume_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_volume_gradient_to_plato_main_operation(aXMLMetaData, tDocument);

    XMLGen::append_stochastic_objective_value_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_stochastic_objective_gradient_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_nondeterministic_qoi_statistics_to_plato_main_operation(aXMLMetaData, tDocument);

    XMLGen::append_update_problem_to_plato_main_operation(aXMLMetaData, tDocument);
    XMLGen::append_filter_control_to_plato_main_operation(tDocument);
    XMLGen::append_filter_gradient_to_plato_main_operation(tDocument);

    tDocument.save_file("plato_main_operations.xml", "  ");
}
// function write_plato_main_operations_xml_file_for_nondeterministic_usecase
/******************************************************************************/

/******************************************************************************/
bool is_volume_constraint_defined
(const XMLGen::InputData& aXMLMetaData)
{
    auto tVolumeConstraintDefined = false;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tLowerCategory = Plato::tolower(tConstraint.category());
        tVolumeConstraintDefined = tLowerCategory.compare("volume") == 0;
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
        auto tLowerCategory = Plato::tolower(tConstraint.category());
        auto tIsVolumeConstraintDefined = tLowerCategory.compare("volume") == 0;
        auto tLowerCode = Plato::tolower(tConstraint.code());
        auto tIsVolumeComputedByPlatoMain = tLowerCode.compare("platomain") == 0;
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
void append_constraint_gradient_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tInput = aParentNode.append_child("Input");
        auto tName = std::string("Constraint Gradient ID-") + tConstraint.name();
        auto tArgumentName = XMLGen::to_lower(tName);
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
    for(auto& tObjective : aXMLMetaData.objectives)
    {
        auto tInput = aParentNode.append_child("Input");
        auto tName = std::string("Objective Gradient ID-") + tObjective.name;
        auto tArgumentName = XMLGen::to_lower(tName);
        XMLGen::append_children( { "ArgumentName" }, { tArgumentName }, tInput);
    }
}
// function append_objective_gradient_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_deterministic_qoi_inputs_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    XMLGen::ValidLayoutKeys tValidLayouts;
    auto tIDs = aXMLMetaData.mOutputMetaData.deterministicIDs();
    for(auto& tID : tIDs)
    {
        auto tInput= aParentNode.append_child("Input");
        auto tLayout = aXMLMetaData.mOutputMetaData.deterministicLayout(tID);
        auto tValidLayout = tValidLayouts.mKeys.find(tLayout);
        auto tArgumentName = aXMLMetaData.mOutputMetaData.deterministicArgumentName(tID);
        XMLGen::append_children( { "ArgumentName", "Layout" }, { tArgumentName, tValidLayout->second }, tInput);
    }
}
// function append_deterministic_qoi_inputs_to_output_operation
/******************************************************************************/

/******************************************************************************/
void append_nondeterministic_qoi_inputs_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode)
{
    XMLGen::ValidLayoutKeys tValidLayouts;
    auto tIDs = aXMLMetaData.mOutputMetaData.randomIDs();
    for(auto& tID : tIDs)
    {
        auto tFor = aParentNode.append_child("For");
        XMLGen::append_attributes({"var", "in"}, {"SampleIndex", "Samples"}, tFor);

        auto tInput= tFor.append_child("Input");
        auto tLayout = aXMLMetaData.mOutputMetaData.randomLayout(tID);
        auto tValidLayout = tValidLayouts.mKeys.find(tLayout);
        auto tArgumentName = tID + " {SampleIndex}";
        XMLGen::append_children( { "ArgumentName", "Layout" }, { tArgumentName, tValidLayout->second }, tInput);
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
    XMLGen::append_children({"ArgumentName"}, {"topology"}, tInput);
    tInput = aParentNode.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"control"}, tInput);
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
    if(!aXMLMetaData.mOutputMetaData.outputData())
    {
        return;
    }
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
    std::vector<std::string> tValues = {"MeanPlusStdDevGradient", "Compute Non-Deterministic Objective Gradient", "Nodal Field"};
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
    auto tIDs = aXMLMetaData.mOutputMetaData.randomIDs();
    for(auto& tID : tIDs)
    {
        auto tLayout = aXMLMetaData.mOutputMetaData.randomLayout(tID);
        auto tValidLayoutItr = tValidKeys.mKeys.find(tLayout);
        auto tName = tID + " Statistics";
        std::vector<std::string> tKeys = {"Function", "Name" , "Layout"};
        std::vector<std::string> tValues = { "MeanPlusStdDev", tName, tValidLayoutItr->second };
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children(tKeys, tValues, tOperation);

        auto tOuterFor = tOperation.append_child("For");
        XMLGen::append_attributes({"var", "in"}, {"PerformerIndex", "Performers"}, tOuterFor);
        auto tInnerFor = tOuterFor.append_child("For");
        XMLGen::append_attributes({"var", "in"}, {"PerformerSampleIndex", "PerformerSamples"}, tInnerFor);
        auto tInput = tInnerFor.append_child("Input");
        tKeys = {"ArgumentName", "Probability"};
        auto tArgumentName = aXMLMetaData.mOutputMetaData.randomArgumentName(tID);
        tValues = {tArgumentName, "{Probabilities[{PerformerIndex*NumSamplesPerPerformer+PerformerSampleIndex}]}"};
        XMLGen::append_children(tKeys, tValues, tInput);

        auto tOutput = tOperation.append_child("Output");
        tArgumentName = tID + " Mean";
        XMLGen::append_children({"Statistic", "ArgumentName"}, {"mean", tArgumentName}, tOutput);
        tOutput = tOperation.append_child("Output");
        tArgumentName = tID + " StdDev";
        XMLGen::append_children({"Statistic", "ArgumentName"}, {"std_dev", tArgumentName}, tOutput);
    }
}
// function append_nondeterministic_qoi_statistics_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_update_problem_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(!aXMLMetaData.mScenarioMetaData.updateProblem())
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
(pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children({"Function", "Name", "Gradient"}, {"Filter", "Filter Control", "False"}, tOperation);
    auto tInput = tOperation.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"Field"}, tInput);
    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children({"ArgumentName"}, {"Filtered Field"}, tOutput);
}
// function append_filter_control_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_filter_gradient_to_plato_main_operation
(pugi::xml_document& aDocument)
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
    auto tValue = aXMLMetaData.initial_density_value.empty() ? "0.5" : aXMLMetaData.initial_density_value;
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
    if(aXMLMetaData.initial_guess_filename.empty())
    {
        THROWERR(std::string("Append Initialize Field From File Operation: ")
            + "Initial guess was supposed to be initialized by reading it from an user-specified file. "
            + "However, the 'filename' keyword is empty.")
    }

    if(aXMLMetaData.initial_guess_field_name.empty())
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
    tValues = {aXMLMetaData.initial_guess_filename, aXMLMetaData.initial_guess_field_name, aXMLMetaData.restart_iteration};
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
    auto tAppendMaterialBox = !aXMLMetaData.levelset_material_box_min.empty() &&
        !aXMLMetaData.levelset_material_box_max.empty();
    if(tAppendMaterialBox)
    {
        auto tMaterialBox = aParentNode.append_child("MaterialBox");
        std::vector<std::string> tKeys = {"MinCoords", "MaxCoords"};
        std::vector<std::string> tValues = {aXMLMetaData.levelset_material_box_min,
            aXMLMetaData.levelset_material_box_max};
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
    if(aXMLMetaData.run_mesh_name.empty())
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
    XMLGen::append_children({"BackgroundMeshName"}, {aXMLMetaData.run_mesh_name}, tMethod);
    XMLGen::append_levelset_material_box(aXMLMetaData, tMethod);
}
// function append_initialize_levelset_primitives_operation
/******************************************************************************/

/******************************************************************************/
void append_initialize_levelset_swiss_cheese_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.run_mesh_name.empty())
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
    tValues = {aXMLMetaData.run_mesh_name, aXMLMetaData.levelset_sphere_radius,
        aXMLMetaData.levelset_sphere_packing_factor};
    for(auto& tNodeSet : aXMLMetaData.levelset_nodesets)
    {
        tKeys.push_back("NodeSet"); tValues.push_back(tNodeSet);
    }

    auto tDefineCreateLevelSetSpheresKeyword =
        aXMLMetaData.levelset_sphere_radius.empty() && aXMLMetaData.levelset_sphere_packing_factor.empty();
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
    auto tLowerKey = Plato::tolower(aXMLMetaData.levelset_initialization_method);
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
    else if(tItr->compare("swiss cheese") == 0)
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
    auto tLowerKey = Plato::tolower(aXMLMetaData.discretization);
    auto tItr = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), tLowerKey);
    if(tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Append Initialize Field to Plato Main Operation: ") + "Discretization method '"
            + tLowerKey + "' is not supported.")
    }

    if(tItr->compare("density") == 0)
    {
        XMLGen::append_initialize_density_field_operation(aXMLMetaData, aDocument);
    }
    else if(tItr->compare("levelset") == 0)
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
    if(aXMLMetaData.initial_guess_filename.empty())
    {
        XMLGen::append_initialize_field_operation(aXMLMetaData, aDocument);
    }
    else
    {
        XMLGen::append_initialize_field_from_file_operation(aXMLMetaData, aDocument);
    }
}
// function append_initialize_field_to_plato_main_operation
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
    for(auto& tID : aXMLMetaData.fixed_block_ids)
    {
        auto tFixedBlocks = aParentNode.append_child("FixedBlocks");
        XMLGen::append_children({"Index"}, {tID}, tFixedBlocks);
    }
}
// function append_fixed_blocks_identification_numbers_to_operation
/******************************************************************************/

/******************************************************************************/
void append_fixed_sidesets_identification_numbers_to_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    for(auto& tID : aXMLMetaData.fixed_sideset_ids)
    {
        auto tFixedSideSet = aParentNode.append_child("FixedSidesets");
        XMLGen::append_children({"Index"}, {tID}, tFixedSideSet);
    }
}
// function append_fixed_sidesets_identification_numbers_to_operation
/******************************************************************************/

/******************************************************************************/
void append_fixed_nodesets_identification_numbers_to_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    for(auto& tID : aXMLMetaData.fixed_nodeset_ids)
    {
        auto tFixedNodeSet = aParentNode.append_child("FixedNodesets");
        XMLGen::append_children({"Index"}, {tID}, tFixedNodeSet);
    }
}
// function append_fixed_nodesets_identification_numbers_to_operation
/******************************************************************************/

/******************************************************************************/
void append_set_lower_bounds_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    std::vector<std::string> tKeys = {"Function", "Name", "Discretization"};
    std::vector<std::string> tValues = {"SetLowerBounds", "Compute Lower Bounds", aXMLMetaData.discretization};
    XMLGen::append_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"Lower Bound Value"}, tInput);
    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children({"ArgumentName"}, {"Lower Bound Vector"}, tOutput);

    XMLGen::append_fixed_blocks_identification_numbers_to_operation(aXMLMetaData, tOperation);
    XMLGen::append_fixed_sidesets_identification_numbers_to_operation(aXMLMetaData, tOperation);
    XMLGen::append_fixed_nodesets_identification_numbers_to_operation(aXMLMetaData, tOperation);
}
// function append_set_lower_bounds_to_plato_main_operation
/******************************************************************************/

/******************************************************************************/
void append_set_upper_bounds_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    std::vector<std::string> tKeys = {"Function", "Name", "Discretization"};
    std::vector<std::string> tValues = {"SetUpperBounds", "Compute Upper Bounds", aXMLMetaData.discretization};
    XMLGen::append_children(tKeys, tValues, tOperation);

    auto tInput = tOperation.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"Upper Bound Value"}, tInput);
    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children({"ArgumentName"}, {"Upper Bound Vector"}, tOutput);

    XMLGen::append_fixed_blocks_identification_numbers_to_operation(aXMLMetaData, tOperation);
    XMLGen::append_fixed_sidesets_identification_numbers_to_operation(aXMLMetaData, tOperation);
    XMLGen::append_fixed_nodesets_identification_numbers_to_operation(aXMLMetaData, tOperation);
}
// function append_set_upper_bounds_to_plato_main_operation
/******************************************************************************/

}
// namespace XMLGen
