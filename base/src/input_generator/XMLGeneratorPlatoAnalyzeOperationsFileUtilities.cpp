/*
 * XMLGeneratorPlatoAnalyzeOperationsFileUtilities.cpp
 *
 *  Created on: Jun 4, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorPlatoAnalyzeProblem.hpp"
#include "XMLGeneratorDefinesFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeUtilities.hpp"
#include "XMLGeneratorMaterialFunctionInterface.hpp"
#include "XMLGeneratorPlatoAnalyzeOperationsFileUtilities.hpp"

namespace XMLGen
{

/******************************************************************************/
void append_compute_objective_value_to_plato_analyze_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    if(XMLGen::is_any_objective_computed_by_plato_analyze(aMetaData))
    {
        if(aMetaData.optimization_parameters().optimization_type() == "topology")
        {
            append_compute_objective_value_operation_for_topology_problem(aMetaData, aDocument);
        }
        else if(aMetaData.optimization_parameters().optimization_type() == "shape")
        {
            append_compute_objective_value_operation_for_shape_problem(aMetaData, aDocument);
        }
        else
        {
            THROWERR("Append Compute Objective Value to Plato Analyze Operation: Unknown optimization type.")
        }
    }
    else
    {
        THROWERR("Append Compute Objective Value to Plato Analyze Operation: No objectives computed by Plato Analyze.")
    }
}
/******************************************************************************/

/******************************************************************************/
void append_compute_objective_value_operation_for_shape_problem
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children( { "Function", "Name", "Criterion" }, { "ComputeCriterionValue", "Compute Objective Value", "My Objective" }, tOperation);
    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children( { "Argument" }, { "Value" }, tOutput);
    XMLGen::append_children( { "ArgumentName" }, { "Objective Value" }, tOutput);

    if(XMLGen::is_robust_optimization_problem(aMetaData))
    {
        XMLGen::append_random_traction_vector_to_plato_analyze_operation(aMetaData, tOperation);
        XMLGen::append_random_material_properties_to_plato_analyze_operation(aMetaData, tOperation);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_compute_objective_value_operation_for_topology_problem
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children( { "Function", "Name", "Criterion" }, { "ComputeCriterionValue", "Compute Objective Value", "My Objective" }, tOperation);
    auto tInput = tOperation.append_child("Input");
    XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children( { "Argument" }, { "Value" }, tOutput);
    XMLGen::append_children( { "ArgumentName" }, { "Objective Value" }, tOutput);

    if(XMLGen::is_robust_optimization_problem(aMetaData))
    {
        XMLGen::append_random_traction_vector_to_plato_analyze_operation(aMetaData, tOperation);
        XMLGen::append_random_material_properties_to_plato_analyze_operation(aMetaData, tOperation);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_compute_objective_gradient_operation_for_topology_problem
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children( { "Function", "Name", "Criterion" }, { "ComputeCriterionGradient", "Compute Objective Gradient", "My Objective"}, tOperation);
    auto tInput = tOperation.append_child("Input");
    XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children( { "Argument" }, { "Gradient" }, tOutput);
    XMLGen::append_children( { "ArgumentName" }, { "Objective Gradient" }, tOutput);

    if(XMLGen::is_robust_optimization_problem(aMetaData))
    {
        XMLGen::append_random_traction_vector_to_plato_analyze_operation(aMetaData, tOperation);
        XMLGen::append_random_material_properties_to_plato_analyze_operation(aMetaData, tOperation);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_compute_objective_gradient_operation_for_shape_problem
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children( { "Function", "Name", "Criterion" }, { "ComputeCriterionX", "Compute Objective Gradient", "My Objective"}, tOperation);

    if(XMLGen::is_robust_optimization_problem(aMetaData))
    {
        XMLGen::append_random_traction_vector_to_plato_analyze_operation(aMetaData, tOperation);
        XMLGen::append_random_material_properties_to_plato_analyze_operation(aMetaData, tOperation);
    }

    pugi::xml_node tmp_node = aDocument.append_child("Operation");
    addChild(tmp_node, "Name", "Compute Objective Sensitivity");
    addChild(tmp_node, "Function", "MapCriterionGradientX");
    addChild(tmp_node, "Criterion", "My Objective");
    pugi::xml_node tForNode = tmp_node.append_child("For");
    tForNode.append_attribute("var") = "I";
    tForNode.append_attribute("in") = "Parameters";
    pugi::xml_node tmp_node1 = tForNode.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Parameter Sensitivity {I}");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Criterion Sensitivity");
}
/******************************************************************************/

/******************************************************************************/
void append_compute_constraint_value_operation_for_shape_problem
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children( { "Function", "Name", "Criterion" }, { "ComputeCriterionValue", "Compute Constraint Value", "My Constraint" }, tOperation);
    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children( { "Argument" }, { "Value" }, tOutput);
    XMLGen::append_children( { "ArgumentName" }, { "Constraint Value" }, tOutput);

    if(XMLGen::is_robust_optimization_problem(aMetaData))
    {
        XMLGen::append_random_traction_vector_to_plato_analyze_operation(aMetaData, tOperation);
        XMLGen::append_random_material_properties_to_plato_analyze_operation(aMetaData, tOperation);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_compute_constraint_value_operation_for_topology_problem
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children( { "Function", "Name", "Criterion" }, { "ComputeCriterionValue", "Compute Constraint Value", "My Constraint" }, tOperation);
    auto tInput = tOperation.append_child("Input");
    XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children( { "Argument" }, { "Value" }, tOutput);
    XMLGen::append_children( { "ArgumentName" }, { "Constraint Value" }, tOutput);

    if(XMLGen::is_robust_optimization_problem(aMetaData))
    {
        XMLGen::append_random_traction_vector_to_plato_analyze_operation(aMetaData, tOperation);
        XMLGen::append_random_material_properties_to_plato_analyze_operation(aMetaData, tOperation);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_compute_constraint_gradient_operation_for_topology_problem
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children( { "Function", "Name", "Criterion" }, { "ComputeCriterionGradient", "Compute Constraint Gradient", "My Constraint" }, tOperation);
    auto tInput = tOperation.append_child("Input");
    XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
    auto tOutput = tOperation.append_child("Output");
    XMLGen::append_children( { "Argument" }, { "Gradient" }, tOutput);
    XMLGen::append_children( { "ArgumentName" }, { "Constraint Gradient" }, tOutput);

    if(XMLGen::is_robust_optimization_problem(aMetaData))
    {
        XMLGen::append_random_traction_vector_to_plato_analyze_operation(aMetaData, tOperation);
        XMLGen::append_random_material_properties_to_plato_analyze_operation(aMetaData, tOperation);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_compute_constraint_gradient_operation_for_shape_problem
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children( { "Function", "Name", "Criterion" }, { "ComputeCriterionX", "Compute Constraint Gradient", "My Constraint" }, tOperation);

    if(XMLGen::is_robust_optimization_problem(aMetaData))
    {
        XMLGen::append_random_traction_vector_to_plato_analyze_operation(aMetaData, tOperation);
        XMLGen::append_random_material_properties_to_plato_analyze_operation(aMetaData, tOperation);
    }

    pugi::xml_node tmp_node = aDocument.append_child("Operation");
    addChild(tmp_node, "Name", "Compute Constraint Sensitivity");
    addChild(tmp_node, "Function", "MapCriterionGradientX");
    addChild(tmp_node, "Criterion", "My Constraint");
    pugi::xml_node tForNode = tmp_node.append_child("For");
    tForNode.append_attribute("var") = "I";
    tForNode.append_attribute("in") = "Parameters";
    pugi::xml_node tmp_node1 = tForNode.append_child("Input");
    addChild(tmp_node1, "ArgumentName", "Parameter Sensitivity {I}");
    tmp_node1 = tmp_node.append_child("Output");
    addChild(tmp_node1, "ArgumentName", "Criterion Sensitivity");
}
/******************************************************************************/

/******************************************************************************/
void append_reinit_on_change_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    if(aMetaData.optimization_parameters().optimization_type() == "shape")
    {
        pugi::xml_node tmp_node = aDocument.append_child("Operation");
        addChild(tmp_node, "Name", "Reinitialize on Change");
        addChild(tmp_node, "Function", "Reinitialize");
        addChild(tmp_node, "OnChange", "true");
        pugi::xml_node tmp_node1 = tmp_node.append_child("Input");
        addChild(tmp_node1, "ArgumentName", "Parameters");
        addChild(tmp_node1, "SharedDataName", "Design Parameters");
    }
}
/******************************************************************************/

/******************************************************************************/
void append_mesh_map_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    if(aMetaData.optimization_parameters().needsMeshMap())
    {
        auto tMeshMap = aDocument.append_child("MeshMap");
        XMLGen::append_children( { "FilterFirst" }, { aMetaData.optimization_parameters().filter_before_symmetry_enforcement() }, tMeshMap);
        auto tFilter = tMeshMap.append_child("Filter");
        XMLGen::append_children( { "Type", "Radius" }, { "Linear", aMetaData.optimization_parameters().mesh_map_filter_radius() }, tFilter);
        auto tLinearMap = tMeshMap.append_child("LinearMap");
        XMLGen::append_children( { "Type" }, { "SymmetryPlane" }, tLinearMap);
        auto tOrigin = tLinearMap.append_child("Origin");
        XMLGen::append_children( { "X", "Y", "Z" }, { aMetaData.optimization_parameters().symmetryOrigin()[0], aMetaData.optimization_parameters().symmetryOrigin()[1], aMetaData.optimization_parameters().symmetryOrigin()[2] }, tOrigin);
        auto tNormal = tLinearMap.append_child("Normal");
        XMLGen::append_children( { "X", "Y", "Z" }, { aMetaData.optimization_parameters().symmetryNormal()[0], aMetaData.optimization_parameters().symmetryNormal()[1], aMetaData.optimization_parameters().symmetryNormal()[2] }, tNormal);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_compute_objective_gradient_to_plato_analyze_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    if(XMLGen::is_any_objective_computed_by_plato_analyze(aMetaData))
    {
        if(aMetaData.optimization_parameters().optimization_type() == "topology")
        {
            append_compute_objective_gradient_operation_for_topology_problem(aMetaData, aDocument);
        }
        else if(aMetaData.optimization_parameters().optimization_type() == "shape")
        {
            append_compute_objective_gradient_operation_for_shape_problem(aMetaData, aDocument);
        }
        else
        {
            THROWERR("Append Compute Objective Gradient to Plato Analyze Operation: Unknown optimization type.")
        }
    }
    else
    {
        THROWERR("Append Compute Objective Gradient to Plato Analyze Operation: No objectives computed by Plato Analyze.")
    }
}

/******************************************************************************/
void append_compute_constraint_value_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(XMLGen::is_any_constraint_computed_by_plato_analyze(aXMLMetaData))
    {
        if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
        {
            append_compute_constraint_value_operation_for_topology_problem(aXMLMetaData, aDocument);
        }
        else if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
        {
            append_compute_constraint_value_operation_for_shape_problem(aXMLMetaData, aDocument);
        }
        else
        {
            THROWERR("Append Compute Constraint Value to Plato Analyze Operation: Unknown optimization type.")
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_compute_constraint_gradient_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(XMLGen::is_any_constraint_computed_by_plato_analyze(aXMLMetaData))
    {
        if(aXMLMetaData.optimization_parameters().optimization_type() == "topology")
        {
            append_compute_constraint_gradient_operation_for_topology_problem(aXMLMetaData, aDocument);
        }
        else if(aXMLMetaData.optimization_parameters().optimization_type() == "shape")
        {
            append_compute_constraint_gradient_operation_for_shape_problem(aXMLMetaData, aDocument);
        }
        else
        {
            THROWERR("Append Compute Constraint Gradient to Plato Analyze Operation: Unknown optimization type.")
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_update_problem_to_plato_analyze_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    if(aMetaData.optimization_parameters().optimization_type() == "shape")
    {
        return;
    }
    bool tNeedUpdate = false;
    for(auto &tService : aMetaData.services())
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

    if(aMetaData.optimization_parameters().problem_update_frequency().empty())
    {
        THROWERR("Append Update Problem to Plato Analyze Operation: 'update problem frequency' keyword is empty.")
    }

    auto tIsUpdateFrequencyGreaterThanZero = std::stoi(aMetaData.optimization_parameters().problem_update_frequency()) > 0;
    if (!aMetaData.optimization_parameters().problem_update_frequency().empty() && tIsUpdateFrequencyGreaterThanZero)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "UpdateProblem", "Update Problem" }, tOperation);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_compute_solution_to_plato_analyze_operation
(pugi::xml_document& aDocument)
{
    auto tOperation = aDocument.append_child("Operation");
    XMLGen::append_children({"Function", "Name"}, {"Compute Solution", "Compute Displacement Solution"}, tOperation);
    auto tInput = tOperation.append_child("Input");
    XMLGen::append_children({"ArgumentName"}, {"Topology"}, tInput);
}
/******************************************************************************/

/******************************************************************************/
XMLGen::Analyze::MaterialPropertyMetadata
return_random_material_metadata_for_plato_analyze_operation_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData)
{
    if(aRandomMetaData.samples().empty())
    {
        THROWERR(std::string("Return Material Property Tags For Plato Analyze Operation XML File: Samples ")
            + "vector in input random metadata structure is empty.")
    }

    // MaterialTags = map< block_id, pair< material_category, vector< pair< material_property_argument_name_tag, material_property_tag > > > >
    XMLGen::Analyze::MaterialPropertyMetadata tMap;
    auto tSample = aRandomMetaData.sample(0);
    auto tBlockIDs = tSample.materialBlockIDs();
    for(auto& tID : tBlockIDs)
    {
        auto tMaterial = tSample.material(tID);
        auto tMaterialPropertiesTags = tMaterial.tags();
        tMap[tID] = std::make_tuple(tMaterial.name(), tMaterial.materialModel(), std::vector<std::pair<std::string, std::string>>());
        //tMap[tID] = std::make_pair(tMaterial.materialModel(), std::vector<std::pair<std::string, std::string>>());
        for(auto& tTag : tMaterialPropertiesTags)
        {
            auto tMaterialPropertyTag = Plato::tolower(tTag);
            auto tArgumentNameTag = tMaterialPropertyTag + "_block_id_" + tID;
            std::get<2>(tMap[tID]).push_back(std::make_pair(tArgumentNameTag, tMaterialPropertyTag));
            //tMap[tID].second.push_back(std::make_pair(tArgumentNameTag, tMaterialPropertyTag));
        }
    }

    return tMap;
}
/******************************************************************************/

/******************************************************************************/
void append_random_material_properties_to_plato_analyze_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::MaterialFunctionInterface tMatFuncInterface;
    auto tMaterialTags =
        XMLGen::return_random_material_metadata_for_plato_analyze_operation_xml_file(aMetaData.mRandomMetaData);
    for (auto &tMaterial : tMaterialTags)
    {
        tMatFuncInterface.call(std::get<0>(tMaterial.second), std::get<1>(tMaterial.second), 
                               std::get<2>(tMaterial.second), aParentNode);
    }
}
/******************************************************************************/

/******************************************************************************/
void append_random_traction_vector_to_plato_analyze_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_node& aParentNode)
{
    std::vector<std::string> tKeys = {"ArgumentName", "Target", "InitialValue"};
    auto tLoadsTags = XMLGen::return_random_tractions_tags_for_define_xml_file(aMetaData.mRandomMetaData);
    for(auto& tPair : tLoadsTags)
    {
        for(auto& tArgumentNameTag : tPair.second)
        {
            auto tDim = &tArgumentNameTag - &tPair.second[0];
            auto tValue = std::string("Values(") + std::to_string(tDim) + ")";
            auto tTarget = std::string("[Plato Problem]:[Natural Boundary Conditions]:[") + tPair.first + "]:" + tValue;
            std::vector<std::string> tValues = { tArgumentNameTag, tTarget, "0.0" };
            auto tParameter = aParentNode.append_child("Parameter");
            XMLGen::append_children(tKeys, tValues, tParameter);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_write_output_to_plato_analyze_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_node& aParentNode)
{
    if(aMetaData.mOutputMetaData.size() == 0)
    {
        return;
    }
    const XMLGen::Output &tOutputMetadata = aMetaData.mOutputMetaData[0];
    if(tOutputMetadata.isOutputDisabled())
    {
        return;
    }
    if(aMetaData.optimization_parameters().optimization_type() == "shape")
    {
        return;
    }

    auto tServiceID = tOutputMetadata.serviceID();
    auto tCodeName = aMetaData.service(tServiceID).code();
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Function", "Name"}, {"WriteOutput", "Write Output"}, tOperationNode);

    if(aMetaData.optimization_parameters().filter_in_engine() == "false")
    {
        auto tOutput = tOperationNode.append_child("Output");
        XMLGen::append_children({"ArgumentName"}, {"Topology"}, tOutput);
    }

    XMLGen::ValidPerformerOutputKeys tValidKeys;
    auto tOutputQoIs = tOutputMetadata.outputIDs();
    for(auto& tQoI : tOutputQoIs)
    {
        auto tOutput = tOperationNode.append_child("Output");
        auto tArgumentName = tValidKeys.argument(tCodeName, tQoI);
        XMLGen::append_children({"ArgumentName"}, {tArgumentName}, tOutput);
    }
}
/******************************************************************************/

/******************************************************************************/
void write_plato_analyze_operation_xml_file
(const XMLGen::InputData& aXMLMetaData)
{
    pugi::xml_document tDocument;
    XMLGen::append_include_defines_xml_data(aXMLMetaData, tDocument);
    XMLGen::append_reinit_on_change_data(aXMLMetaData, tDocument);
    XMLGen::append_mesh_map_data(aXMLMetaData, tDocument);
    XMLGen::append_write_output_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_update_problem_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_objective_value_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_objective_gradient_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_constraint_value_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_constraint_gradient_to_plato_analyze_operation(aXMLMetaData, tDocument);
    std::string tServiceID = get_plato_analyze_service_id(aXMLMetaData);
    std::string tFilename = std::string("plato_analyze_") + tServiceID + "_operations.xml";
    tDocument.save_file(tFilename.c_str(), "  ");
}
/******************************************************************************/

/******************************************************************************/
void write_amgx_input_file(const XMLGen::InputData& aMetaData)
{
    std::vector<XMLGen::Scenario> tScenarios = aMetaData.scenarios();
    bool tAtLeastOneScenarioIncludesPlasticity       = false;
    bool tAtLeastOneScenarioIncludesThermoplasticity = false;
    for (unsigned int tIndex = 0; tIndex < tScenarios.size(); ++tIndex)
    {
        if (tScenarios[tIndex].physics() == "plasticity")
            tAtLeastOneScenarioIncludesPlasticity = true;
        else if (tScenarios[tIndex].physics() == "thermoplasticity")
            tAtLeastOneScenarioIncludesThermoplasticity = true;
    }
    
    if (tAtLeastOneScenarioIncludesThermoplasticity)
        XMLGen::write_amgx_input_file_for_thermoplasticity(aMetaData);
    else if (tAtLeastOneScenarioIncludesPlasticity)
        XMLGen::write_amgx_input_file_for_plasticity(aMetaData);
    else
        XMLGen::write_default_amgx_input_file(aMetaData);
}
/******************************************************************************/

/******************************************************************************/
void write_default_amgx_input_file(const XMLGen::InputData& aMetaData)
{
    FILE *tFilePointer = fopen("amgx.json", "w");
    if(tFilePointer)
    {
        fprintf(tFilePointer, "{\n");
        fprintf(tFilePointer, "\"config_version\": 2,\n");
        fprintf(tFilePointer, "\"solver\": {\n");
        fprintf(tFilePointer, "\"preconditioner\": {\n");
        fprintf(tFilePointer, "\"print_grid_stats\": 1,\n");
        fprintf(tFilePointer, "\"algorithm\": \"AGGREGATION\",\n");
        fprintf(tFilePointer, "\"print_vis_data\": 0,\n");
        fprintf(tFilePointer, "\"max_matching_iterations\": 50,\n");
        fprintf(tFilePointer, "\"max_unassigned_percentage\": 0.01,\n");
        fprintf(tFilePointer, "\"solver\": \"AMG\",\n");
        fprintf(tFilePointer, "\"smoother\": {\n");
        fprintf(tFilePointer, "\"relaxation_factor\": 0.78,\n");
        fprintf(tFilePointer, "\"scope\": \"jacobi\",\n");
        fprintf(tFilePointer, "\"solver\": \"BLOCK_JACOBI\",\n");
        fprintf(tFilePointer, "\"monitor_residual\": 0,\n");
        fprintf(tFilePointer, "\"print_solve_stats\": 0\n");
        fprintf(tFilePointer, "},\n");
        fprintf(tFilePointer, "\"print_solve_stats\": 0,\n");
        fprintf(tFilePointer, "\"dense_lu_num_rows\": 64,\n");
        fprintf(tFilePointer, "\"presweeps\": 1,\n");
        fprintf(tFilePointer, "\"selector\": \"SIZE_8\",\n");
        fprintf(tFilePointer, "\"coarse_solver\": \"DENSE_LU_SOLVER\",\n");
        fprintf(tFilePointer, "\"coarsest_sweeps\": 2,\n");
        fprintf(tFilePointer, "\"max_iters\": 1,\n");
        fprintf(tFilePointer, "\"monitor_residual\": 0,\n");
        fprintf(tFilePointer, "\"store_res_history\": 0,\n");
        fprintf(tFilePointer, "\"scope\": \"amg\",\n");
        fprintf(tFilePointer, "\"max_levels\": 100,\n");
        fprintf(tFilePointer, "\"postsweeps\": 1,\n");
        fprintf(tFilePointer, "\"cycle\": \"W\"\n");
        fprintf(tFilePointer, "},\n");
        fprintf(tFilePointer, "\"solver\": \"PBICGSTAB\",\n");
        fprintf(tFilePointer, "\"print_solve_stats\": 0,\n");
        fprintf(tFilePointer, "\"obtain_timings\": 0,\n");
        fprintf(tFilePointer, "\"max_iters\": 1000,\n");
        fprintf(tFilePointer, "\"monitor_residual\": 1,\n");
        fprintf(tFilePointer, "\"convergence\": \"ABSOLUTE\",\n");
        fprintf(tFilePointer, "\"scope\": \"main\",\n");

        std::string tTolerance = "1e-12";
        std::string tScenarioID = aMetaData.objective.scenarioIDs[0];
        auto &tScenario = aMetaData.scenario(tScenarioID);
        if(tScenario.solverTolerance().length() > 0)
            tTolerance = tScenario.solverTolerance();

        fprintf(tFilePointer, "\"tolerance\": %s,\n", tTolerance.c_str());
        fprintf(tFilePointer, "\"norm\": \"L2\"\n");
        fprintf(tFilePointer, "}\n");
        fprintf(tFilePointer, "}\n");
        fclose(tFilePointer);
    }
}
/******************************************************************************/

/******************************************************************************/
void write_amgx_input_file_for_plasticity(const XMLGen::InputData& aMetaData)
{
    FILE *tFilePointer = fopen("amgx.json", "w");
    if(tFilePointer)
    {
        fprintf(tFilePointer, "{\n");
        fprintf(tFilePointer, "\"config_version\": 2,\n");
        fprintf(tFilePointer, "\"solver\": {\n");
        fprintf(tFilePointer, "\"preconditioner\": {\n");
        fprintf(tFilePointer, "\"print_grid_stats\": 1,\n");
        fprintf(tFilePointer, "\"algorithm\": \"AGGREGATION\",\n");
        fprintf(tFilePointer, "\"print_vis_data\": 0,\n");
        fprintf(tFilePointer, "\"max_matching_iterations\": 50,\n");
        fprintf(tFilePointer, "\"max_unassigned_percentage\": 0.01,\n");
        fprintf(tFilePointer, "\"solver\": \"AMG\",\n");
        fprintf(tFilePointer, "\"smoother\": {\n");
        fprintf(tFilePointer, "\"relaxation_factor\": 0.78,\n");
        fprintf(tFilePointer, "\"scope\": \"jacobi\",\n");
        fprintf(tFilePointer, "\"solver\": \"MULTICOLOR_GS\",\n");
        fprintf(tFilePointer, "\"symmetric_GS\": 1,\n");
        fprintf(tFilePointer, "\"monitor_residual\": 0,\n");
        fprintf(tFilePointer, "\"print_solve_stats\": 0\n");
        fprintf(tFilePointer, "},\n");
        fprintf(tFilePointer, "\"print_solve_stats\": 0,\n");
        fprintf(tFilePointer, "\"dense_lu_num_rows\": 128,\n");
        fprintf(tFilePointer, "\"presweeps\": 1,\n");
        fprintf(tFilePointer, "\"selector\": \"SIZE_8\",\n");
        fprintf(tFilePointer, "\"coarse_solver\": \"DENSE_LU_SOLVER\",\n");
        fprintf(tFilePointer, "\"coarsest_sweeps\": 2,\n");
        fprintf(tFilePointer, "\"max_iters\": 1,\n");
        fprintf(tFilePointer, "\"monitor_residual\": 0,\n");
        fprintf(tFilePointer, "\"store_res_history\": 0,\n");
        fprintf(tFilePointer, "\"scope\": \"amg\",\n");
        fprintf(tFilePointer, "\"max_levels\": 100,\n");
        fprintf(tFilePointer, "\"postsweeps\": 1,\n");
        fprintf(tFilePointer, "\"cycle\": \"W\"\n");
        fprintf(tFilePointer, "},\n");
        fprintf(tFilePointer, "\"solver\": \"FGMRES\",\n");
        fprintf(tFilePointer, "\"gmres_n_restart\": 1000,\n");
        fprintf(tFilePointer, "\"print_solve_stats\": 0,\n");
        fprintf(tFilePointer, "\"obtain_timings\": 0,\n");
        fprintf(tFilePointer, "\"max_iters\": 1000,\n");
        fprintf(tFilePointer, "\"monitor_residual\": 1,\n");
        fprintf(tFilePointer, "\"convergence\": \"ABSOLUTE\",\n");
        fprintf(tFilePointer, "\"scope\": \"main\",\n");

        std::string tTolerance = "1e-12";
        std::string tScenarioID = aMetaData.objective.scenarioIDs[0];
        auto &tScenario = aMetaData.scenario(tScenarioID);
        if(tScenario.solverTolerance().length() > 0)
            tTolerance = tScenario.solverTolerance();

        fprintf(tFilePointer, "\"tolerance\": %s,\n", tTolerance.c_str());
        fprintf(tFilePointer, "\"norm\": \"L2\"\n");
        fprintf(tFilePointer, "}\n");
        fprintf(tFilePointer, "}\n");
        fclose(tFilePointer);
    }
}
/******************************************************************************/

/******************************************************************************/
void write_amgx_input_file_for_thermoplasticity(const XMLGen::InputData& aMetaData)
{
    FILE *tFilePointer = fopen("amgx.json", "w");
    if(tFilePointer)
    {
        fprintf(tFilePointer, "{\n");
        fprintf(tFilePointer, "\"config_version\": 2,\n");
        fprintf(tFilePointer, "\"solver\": {\n");
        fprintf(tFilePointer, "\"preconditioner\": {\n");
        fprintf(tFilePointer, "\"print_grid_stats\": 1,\n");
        fprintf(tFilePointer, "\"algorithm\": \"AGGREGATION\",\n");
        fprintf(tFilePointer, "\"print_vis_data\": 0,\n");
        fprintf(tFilePointer, "\"max_matching_iterations\": 50,\n");
        fprintf(tFilePointer, "\"max_unassigned_percentage\": 0.01,\n");
        fprintf(tFilePointer, "\"solver\": \"AMG\",\n");
        fprintf(tFilePointer, "\"smoother\": {\n");
        fprintf(tFilePointer, "\"relaxation_factor\": 0.78,\n");
        fprintf(tFilePointer, "\"scope\": \"jacobi\",\n");
        fprintf(tFilePointer, "\"solver\": \"MULTICOLOR_GS\",\n");
        fprintf(tFilePointer, "\"symmetric_GS\": 0,\n");
        fprintf(tFilePointer, "\"monitor_residual\": 0,\n");
        fprintf(tFilePointer, "\"print_solve_stats\": 0\n");
        fprintf(tFilePointer, "},\n");
        fprintf(tFilePointer, "\"print_solve_stats\": 0,\n");
        fprintf(tFilePointer, "\"dense_lu_num_rows\": 128,\n");
        fprintf(tFilePointer, "\"presweeps\": 1,\n");
        fprintf(tFilePointer, "\"selector\": \"SIZE_8\",\n");
        fprintf(tFilePointer, "\"coarse_solver\": \"DENSE_LU_SOLVER\",\n");
        fprintf(tFilePointer, "\"coarsest_sweeps\": 2,\n");
        fprintf(tFilePointer, "\"max_iters\": 1,\n");
        fprintf(tFilePointer, "\"monitor_residual\": 0,\n");
        fprintf(tFilePointer, "\"store_res_history\": 0,\n");
        fprintf(tFilePointer, "\"scope\": \"amg\",\n");
        fprintf(tFilePointer, "\"max_levels\": 100,\n");
        fprintf(tFilePointer, "\"postsweeps\": 1,\n");
        fprintf(tFilePointer, "\"cycle\": \"W\"\n");
        fprintf(tFilePointer, "},\n");
        fprintf(tFilePointer, "\"solver\": \"FGMRES\",\n");
        fprintf(tFilePointer, "\"gmres_n_restart\": 1000,\n");
        fprintf(tFilePointer, "\"print_solve_stats\": 0,\n");
        fprintf(tFilePointer, "\"obtain_timings\": 0,\n");
        fprintf(tFilePointer, "\"max_iters\": 1000,\n");
        fprintf(tFilePointer, "\"monitor_residual\": 1,\n");
        fprintf(tFilePointer, "\"convergence\": \"ABSOLUTE\",\n");
        fprintf(tFilePointer, "\"scope\": \"main\",\n");

        std::string tTolerance = "1e-12";
        std::string tScenarioID = aMetaData.objective.scenarioIDs[0];
        auto &tScenario = aMetaData.scenario(tScenarioID);
        if(tScenario.solverTolerance().length() > 0)
            tTolerance = tScenario.solverTolerance();

        fprintf(tFilePointer, "\"tolerance\": %s,\n", tTolerance.c_str());
        fprintf(tFilePointer, "\"norm\": \"L2\"\n");
        fprintf(tFilePointer, "}\n");
        fprintf(tFilePointer, "}\n");
        fclose(tFilePointer);
    }
}
/******************************************************************************/

}
// namespace XMLGen
