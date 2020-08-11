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
    auto tIsTopologyOptimization = XMLGen::is_topology_optimization_problem(aMetaData.optimization_type);
    auto tIsPlatoAnalyzePerformer = XMLGen::is_any_objective_computed_by_plato_analyze(aMetaData);
    auto tAppendComputeObjectiveValueOperation = tIsTopologyOptimization && tIsPlatoAnalyzePerformer;

    if(tAppendComputeObjectiveValueOperation)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "ComputeCriterionValue", "Compute Objective Value" }, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children( { "Argument" }, { "Value" }, tOutput);
        XMLGen::append_children( { "ArgumentName" }, { "Objective Value" }, tOutput);

        if(XMLGen::Analyze::is_robust_optimization_problem(aMetaData))
        {
            XMLGen::append_random_traction_vector_to_plato_analyze_operation(aMetaData, tOperation);
            XMLGen::append_random_material_properties_to_plato_analyze_operation(aMetaData, tOperation);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_compute_objective_gradient_to_plato_analyze_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsTopologyOptimization = XMLGen::is_topology_optimization_problem(aMetaData.optimization_type);
    auto tIsPlatoAnalyzePerformer = XMLGen::is_any_objective_computed_by_plato_analyze(aMetaData);
    auto tAppendComputeObjectiveGradientOperation = tIsTopologyOptimization && tIsPlatoAnalyzePerformer;

    if(tAppendComputeObjectiveGradientOperation)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "ComputeCriterionGradient", "Compute Objective Gradient" }, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children( { "Argument" }, { "Gradient" }, tOutput);
        XMLGen::append_children( { "ArgumentName" }, { "Objective Gradient" }, tOutput);

        if(XMLGen::Analyze::is_robust_optimization_problem(aMetaData))
        {
            XMLGen::append_random_traction_vector_to_plato_analyze_operation(aMetaData, tOperation);
            XMLGen::append_random_material_properties_to_plato_analyze_operation(aMetaData, tOperation);
        }
    }
}

/******************************************************************************/
void append_compute_constraint_value_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsTopologyOptimization = XMLGen::is_topology_optimization_problem(aXMLMetaData.optimization_type);
    auto tIsPlatoAnalyzePerformer = XMLGen::is_any_constraint_computed_by_plato_analyze(aXMLMetaData);
    auto tAppendComputeConstraintValueOperation = tIsTopologyOptimization && tIsPlatoAnalyzePerformer;

    if(tAppendComputeConstraintValueOperation)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "ComputeCriterionValue", "Compute Constraint Value" }, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children( { "Argument" }, { "Value" }, tOutput);
        XMLGen::append_children( { "ArgumentName" }, { "Constraint Value" }, tOutput);

        if(XMLGen::Analyze::is_robust_optimization_problem(aXMLMetaData))
        {
            XMLGen::append_random_traction_vector_to_plato_analyze_operation(aXMLMetaData, tOperation);
            XMLGen::append_random_material_properties_to_plato_analyze_operation(aXMLMetaData, tOperation);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_compute_constraint_gradient_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsTopologyOptimization = XMLGen::is_topology_optimization_problem(aXMLMetaData.optimization_type);
    auto tIsPlatoAnalyzePerformer = XMLGen::is_any_constraint_computed_by_plato_analyze(aXMLMetaData);
    auto tAppendComputeConstraintGradientOperation = tIsTopologyOptimization && tIsPlatoAnalyzePerformer;

    if(tAppendComputeConstraintGradientOperation)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "ComputeCriterionGradient", "Compute Constraint Gradient" }, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children( { "Argument" }, { "Gradient" }, tOutput);
        XMLGen::append_children( { "ArgumentName" }, { "Constraint Gradient" }, tOutput);

        if(XMLGen::Analyze::is_robust_optimization_problem(aXMLMetaData))
        {
            XMLGen::append_random_traction_vector_to_plato_analyze_operation(aXMLMetaData, tOperation);
            XMLGen::append_random_material_properties_to_plato_analyze_operation(aXMLMetaData, tOperation);
        }
    }
}
/******************************************************************************/

/******************************************************************************/
void append_update_problem_to_plato_analyze_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    if(!aMetaData.service(0u).updateProblem())
    {
        return;
    }
    if(aMetaData.mProblemUpdateFrequency.empty())
    {
        THROWERR("Append Update Problem to Plato Analyze Operation: 'update problem frequency' keyword is empty.")
    }

    auto tIsUpdateFrequencyGreaterThanZero = std::stoi(aMetaData.mProblemUpdateFrequency) > 0;
    if (!aMetaData.mProblemUpdateFrequency.empty() && tIsUpdateFrequencyGreaterThanZero)
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
        tMap[tID] = std::make_pair(tMaterial.category(), std::vector<std::pair<std::string, std::string>>());
        for(auto& tTag : tMaterialPropertiesTags)
        {
            auto tMaterialPropertyTag = Plato::tolower(tTag);
            auto tArgumentNameTag = tMaterialPropertyTag + "_block_id_" + tID;
            tMap[tID].second.push_back(std::make_pair(tArgumentNameTag, tMaterialPropertyTag));
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
        tMatFuncInterface.call(tMaterial.second.first, tMaterial.second.second, aParentNode);
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
    if(aMetaData.mOutputMetaData.isOutputDisabled())
    {
        return;
    }

    auto tServiceID = aMetaData.mOutputMetaData.serviceID();
    auto tCodeName = aMetaData.service(tServiceID).code();
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Function", "Name"}, {"WriteOutput", "Write Output"}, tOperationNode);

    XMLGen::ValidPerformerOutputKeys tValidKeys;
    auto tOutputQoIs = aMetaData.mOutputMetaData.outputIDs();
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
    XMLGen::append_write_output_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_update_problem_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_objective_value_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_objective_gradient_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_constraint_value_to_plato_analyze_operation(aXMLMetaData, tDocument);
    XMLGen::append_compute_constraint_gradient_to_plato_analyze_operation(aXMLMetaData, tDocument);
    tDocument.save_file("plato_analyze_operations.xml", "  ");
}
/******************************************************************************/

/******************************************************************************/
void write_amgx_input_file()
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
        fprintf(tFilePointer, "\"tolerance\": 1e-12,\n");
        fprintf(tFilePointer, "\"norm\": \"L2\"\n");
        fprintf(tFilePointer, "}\n");
        fprintf(tFilePointer, "}\n");
        fclose(tFilePointer);
    }
}
/******************************************************************************/

}
// namespace XMLGen
