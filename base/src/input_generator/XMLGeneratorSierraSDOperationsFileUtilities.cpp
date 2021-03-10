/*
 * XMLGeneratorSierraSDInputDeckUtilities.cpp
 *
 *  Created on: Feb 1, 2021
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorSierraSDInputDeckUtilities.hpp"

namespace XMLGen
{

/**************************************************************************/
void append_cache_state_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperationNode = aDocument.append_child("Operation");
    append_children({"Function","Name"}, {"Cache State","Cache State"}, tOperationNode);
    for(size_t k=0; k<aMetaData.objective.scenarioIDs.size(); ++k)
    {
        auto tScenarioID = aMetaData.objective.scenarioIDs[k];
        auto tServiceID = aMetaData.objective.serviceIDs[k];
        for(auto &tOutput : aMetaData.mOutputMetaData)
        {
   
            if(tOutput.serviceID() == tServiceID)
            {
                XMLGen::Service tService = aMetaData.service(tServiceID);
                if(aMetaData.objective.multi_load_case == "true")
                {
                    auto &tScenario = aMetaData.scenario(aMetaData.objective.scenarioIDs[0]);
                    for(size_t i=0; i<tScenario.loadIDs().size(); ++i)
                    {
                        auto tLoadIndex = std::to_string(i);
                        for(auto &tCurData : tOutput.outputIDs())
                        {
                            auto tOutputNode = tOperationNode.append_child("Output");
                            XMLGen::append_children({"ArgumentName"}, {tCurData + tLoadIndex}, tOutputNode);
                        }
                    }                        
                }
                else
                {
                    for(auto &tCurData : tOutput.outputIDs())
                    {
                        auto tOutputNode = tOperationNode.append_child("Output");
                        append_children({"ArgumentName"}, {tCurData + "0"}, tOutputNode);
                    }
                }
                break;
            } 
        }
    }
}
/**************************************************************************/
void append_update_problem_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    auto tOperationNode = aDocument.append_child("Operation");
    append_children({"Function", "Name"}, {"Update Problem", "Update Problem"}, tOperationNode);
}
/**************************************************************************/
void append_SIMP_penalty_model
(const XMLGen::Scenario& aScenario,
 pugi::xml_node& aParentNode)
{
    auto tSIMPNode = aParentNode.append_child("SIMP");
    if(aScenario.materialPenaltyExponent() != "")
    {
        append_children({"PenaltyExponent"}, {aScenario.materialPenaltyExponent()}, tSIMPNode);
    }
    if(aScenario.minErsatzMaterialConstant() != "")
    {
        addChild(tSIMPNode, "MinimumValue", aScenario.minErsatzMaterialConstant());
    }
    else
    {
        addChild(tSIMPNode, "MinimumValue", "0.001");
    }
}
/**************************************************************************/
void append_displacement_operation
(const XMLGen::Scenario& aScenario,
 pugi::xml_document& aDocument)
{
    auto tOperationNode = aDocument.append_child("Operation");
    append_children({"Function", "Name","PenaltyModel"}, {"Displacement", "Compute Displacement","SIMP"}, tOperationNode);
    auto tTopologyNode = tOperationNode.append_child("Topology");
    append_children({"Name"}, {"Topology"}, tTopologyNode);
    append_SIMP_penalty_model(aScenario, tOperationNode);
}
/**************************************************************************/
void append_internal_energy_operation
(const XMLGen::Scenario& aScenario,
 pugi::xml_document& aDocument)
{
    auto tOperationNode = aDocument.append_child("Operation");
    append_children({"Function", "Name","PenaltyModel"}, {"InternalEnergy", "Compute Objective","SIMP"}, tOperationNode);
    auto tTopologyNode = tOperationNode.append_child("Topology");
    append_children({"Name"}, {"Topology"}, tTopologyNode);
    append_SIMP_penalty_model(aScenario, tOperationNode);
    auto tOutputNode = tOperationNode.append_child("OutputValue");
    append_children({"Name"}, {"Internal Energy"}, tOutputNode);
}
/**************************************************************************/
void append_internal_energy_gradient_operation
(const XMLGen::Scenario& aScenario,
 pugi::xml_document& aDocument)
{
    auto tOperationNode = aDocument.append_child("Operation");
    append_children({"Function", "Name","PenaltyModel"}, {"InternalEnergyGradient", "Compute Gradient","SIMP"}, tOperationNode);
    auto tTopologyNode = tOperationNode.append_child("Topology");
    append_children({"Name"}, {"Topology"}, tTopologyNode);
    append_SIMP_penalty_model(aScenario, tOperationNode);
    auto tOutputNode = tOperationNode.append_child("OutputGradient");
    append_children({"Name"}, {"Internal Energy Gradient"}, tOutputNode);
}
/**************************************************************************/
void append_internal_energy_hessian_operation
(const XMLGen::InputData& aMetaData,
 const XMLGen::Scenario& aScenario,
 pugi::xml_document& aDocument)
{
    auto tAlgorithm = aMetaData.optimization_parameters().optimization_algorithm();
    if(tAlgorithm =="ksbc" ||
       tAlgorithm == "ksal" ||
       tAlgorithm == "rol ksal" ||
       tAlgorithm == "rol ksbc")
    {
        auto tOperationNode = aDocument.append_child("Operation");
        append_children({"Function", "Name","PenaltyModel"}, {"InternalEnergyHessian", "Compute HessianTimesVector","SIMP"}, tOperationNode);
        auto tTopologyNode = tOperationNode.append_child("Topology");
        append_children({"Name"}, {"Topology"}, tTopologyNode);
        auto tDescentNode = tOperationNode.append_child("DescentDirection");
        append_children({"Name"}, {"Descent Direction"}, tDescentNode);
        append_SIMP_penalty_model(aScenario, tOperationNode);
        auto tOutputNode = tOperationNode.append_child("OutputHessian");
        append_children({"Name"}, {"HessianTimesVector"}, tOutputNode);
    }
}
/**************************************************************************/
void append_surface_area_operation
(const XMLGen::InputData& aMetaData,
 const XMLGen::Scenario& aScenario,
 pugi::xml_document& aDocument)
{
    auto tDiscretization = aMetaData.optimization_parameters().discretization();
    if(tDiscretization == "levelset")
    {
        auto tOperationNode = aDocument.append_child("Operation");
        append_children({"Function", "Name"}, {"Compute Surface Area", "Compute Surface Area"}, tOperationNode);
        auto tTopologyNode = tOperationNode.append_child("Topology");
        append_children({"Name"}, {"Topology"}, tTopologyNode);
        auto tOutputNode = tOperationNode.append_child("OutputValue");
        append_children({"Name"}, {"SurfaceArea"}, tOutputNode);
    }
}
/**************************************************************************/
void append_surface_area_gradient_operation
(const XMLGen::InputData& aMetaData,
 const XMLGen::Scenario& aScenario,
 pugi::xml_document& aDocument)
{
    auto tDiscretization = aMetaData.optimization_parameters().discretization();
    if(tDiscretization == "levelset")
    {
        auto tOperationNode = aDocument.append_child("Operation");
        append_children({"Function", "Name"}, {"Compute Surface Area Gradient", "Compute Surface Area Gradient"}, tOperationNode);
        auto tTopologyNode = tOperationNode.append_child("Topology");
        append_children({"Name"}, {"Topology"}, tTopologyNode);
        auto tOutputNode = tOperationNode.append_child("OutputValue");
        append_children({"Name"}, {"SurfaceAreaGradient"}, tOutputNode);
    }
}
/**************************************************************************/
void add_operations
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    for(size_t i=0; i<aMetaData.objective.serviceIDs.size(); ++i)
    {
        auto tServiceID = aMetaData.objective.serviceIDs[i];
        auto &tService = aMetaData.service(tServiceID);
        if(tService.code() == "sierra_sd")
        {
            auto tScenarioID = aMetaData.objective.scenarioIDs[i];
            auto &tScenario = aMetaData.scenario(tScenarioID);
            append_version_entry(aDocument);
            append_update_problem_operation(aMetaData, aDocument);
            append_cache_state_operation(aMetaData, aDocument);
            append_displacement_operation(tScenario, aDocument);
            append_internal_energy_operation(tScenario, aDocument);
            append_internal_energy_gradient_operation(tScenario, aDocument);
            append_internal_energy_hessian_operation(aMetaData, tScenario, aDocument);
            append_surface_area_operation(aMetaData, tScenario, aDocument);
            append_surface_area_gradient_operation(aMetaData, tScenario, aDocument);

        }
    }
}
/******************************************************************************/
void write_sierra_sd_operation_xml_file
(const XMLGen::InputData& aXMLMetaData)
{
    pugi::xml_document tDocument;
    add_operations(aXMLMetaData, tDocument);
    std::string tServiceID = get_salinas_service_id(aXMLMetaData);
    std::string tFilename = std::string("sierra_sd_") + tServiceID + "_operations.xml";
    tDocument.save_file(tFilename.c_str(), "  ");
}
/******************************************************************************/


}
// namespace XMLGen
