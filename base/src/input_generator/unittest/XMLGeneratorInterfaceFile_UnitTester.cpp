/*
 * XMLGeneratorInterfaceFile_UnitTester.cpp
 *
 *  Created on: Jul 24, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorPlatoAnalyzeUtilities.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"
#include "XMLGeneratorPlatoMainInputFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeOperationsFileUtilities.hpp"
#include "XMLGeneratorAnalyzeUncertaintyLaunchScriptUtilities.hpp"

namespace XMLGen
{

void append_physics_performers
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.scenarios().empty())
    {
        THROWERR("Append Physics Performer: Scenarios list is empty.")
    }

    std::vector<std::string> tKeywords = { "Name", "Code", "PerformerID" };
    for(auto& tScenario : aXMLMetaData.scenarios())
    {
        const int tID = (&tScenario - &aXMLMetaData.scenarios()[0]) + 1;
        auto tPerformerNode = aParentNode.append_child("Performer");
        std::vector<std::string> tValues = { std::to_string(tID), tScenario.performer(), tScenario.code() };
        XMLGen::append_children( tKeywords, tValues, tPerformerNode);
    }
}

void append_qoi_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aParentNode)
{
    if(aXMLMetaData.scenarios().empty())
    {
        THROWERR("Append QoI Shared Data: list of 'scenarios' is empty.")
    }

    XMLGen::ValidLayoutKeys tValidLayouts;
    auto tOutputIDs = aXMLMetaData.mOutputMetaData.deterministicIDs();
    auto tScenarioID = aXMLMetaData.mOutputMetaData.scenarioID();
    for(auto& tID : tOutputIDs)
    {
        auto tLayout = aXMLMetaData.mOutputMetaData.deterministicLayout(tID);
        auto tValidLayoutItr = tValidLayouts.mKeys.find(tLayout);
        if(tValidLayoutItr == tValidLayouts.mKeys.end())
        {
            THROWERR("Append QoI Shared Data: Unexpected error while searching valid layout '"
                + tLayout + "' of quantity of interest with tag '" + tID + "'.")
        }
        auto tSharedDataName = aXMLMetaData.mOutputMetaData.deterministicSharedDataName(tID);
        auto tOwnerName = aXMLMetaData.scenario(tScenarioID).performer();
        std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
        std::vector<std::string> tValues = {tSharedDataName, "Scalar", tValidLayoutItr->second, "IGNORE", tOwnerName, "platomain"};
        auto tSharedDataNode = aParentNode.append_child("SharedData");
        XMLGen::append_children(tKeys, tValues, tSharedDataNode);
    }
}

void append_topology_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    if(aXMLMetaData.scenarios().empty())
    {
        THROWERR("Append Topology Shared Data: Scenarios list is empty.")
    }

    auto tSharedData = aDocument.append_child("SharedData");
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName", "UserName"};
    std::vector<std::string> tValues = {"Topology", "Scalar", "Nodal Field", "IGNORE", "platomain", "platomain"};
    for(auto& tScenario : aXMLMetaData.scenarios())
    {
        std::vector<std::string> tValues =
            {"Topology", "Scalar", "Nodal Field", "IGNORE", "platomain", "platomain", tScenario.performer()};
        XMLGen::append_children(tKeys, tValues, tSharedData);
    }
}

void append_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    XMLGen::append_control_shared_data(aDocument);
    XMLGen::append_lower_bounds_shared_data(aDocument);
    XMLGen::append_upper_bounds_shared_data(aDocument);
    XMLGen::append_design_volume_shared_data(aDocument);
    XMLGen::append_qoi_shared_data(aMetaData, aDocument);
    XMLGen::append_topology_shared_data(aMetaData, aDocument);
    XMLGen::append_objective_shared_data(aMetaData, aDocument);
    XMLGen::append_constraint_shared_data(aMetaData, aDocument);
}

void append_cache_state_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    for (auto &tScenario : aXMLMetaData.scenarios())
    {
        if (!tScenario.cacheState())
        {
            continue;
        }
        auto tStageNode = aDocument.append_child("Stage");
        auto tStageName = std::string("Cache State ID-") + tScenario.performer();
        XMLGen::append_children( { "Name" }, { tStageName }, tStageNode);
        auto tPerformerName = tScenario.performer();
        std::vector<std::string> tKeys = { "Name", "PerformerName" };
        std::vector<std::string> tValues = { "Cache State", tPerformerName };
        auto tOperationNode = tStageNode.append_child("Operation");
        XMLGen::append_children(tKeys, tValues, tOperationNode);
    }
}

void append_update_problem_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    for (auto &tScenario : aXMLMetaData.scenarios())
    {
        if (!tScenario.updateProblem())
        {
            continue;
        }
        auto tStageNode = aDocument.append_child("Stage");
        auto tStageName = std::string("Update Problem ID-") + tScenario.performer();
        XMLGen::append_children( { "Name" }, { tStageName }, tStageNode);
        auto tPerformerName = tScenario.performer();
        std::vector<std::string> tKeys = { "Name", "PerformerName" };
        std::vector<std::string> tValues = { "Update Problem", tPerformerName };
        auto tOperationNode = tStageNode.append_child("Operation");
        XMLGen::append_children(tKeys, tValues, tOperationNode);
    }
}

void append_objective_value_operation
(const XMLGen::Objective& aObjective,
 pugi::xml_node &aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Objective Value", aObjective.performer()}, tOperationNode);
    auto tOperationInput = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tOperationInput);
    auto tOperationOutput = tOperationNode.append_child("Output");
    auto tOutputSharedData = std::string("Objective Value ID-") + aObjective.name;
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Objective Value", tOutputSharedData}, tOperationOutput);
}

void append_objective_value_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    for(auto& tObjective : aXMLMetaData.objectives)
    {
        auto tStageNode = aDocument.append_child("Stage");
        auto tName = std::string("Compute Objective Value ID-") + tObjective.name;
        XMLGen::append_children( { "Name", "Type" }, { tName, tObjective.category() }, tStageNode);
        auto tStageInput = tStageNode.append_child("Input");
        XMLGen::append_children( { "SharedDataName" }, { "Control" }, tStageInput);
        XMLGen::append_filter_control_operation(tStageNode);
        XMLGen::append_objective_value_operation(tObjective, tStageNode);
        auto tStageOutput = tStageNode.append_child("Output");
        auto tOutputSharedData = std::string("Objective Value ID-") + tObjective.name;
        XMLGen::append_children( { "SharedDataName" }, { tOutputSharedData }, tStageOutput);
    }
}

void append_objective_gradient_operation
(const XMLGen::Objective& aObjective,
 pugi::xml_node &aParentNode)
{
    auto tOperationNode = aParentNode.append_child("Operation");
    XMLGen::append_children({"Name", "PerformerName"}, {"Compute Objective Gradient", aObjective.performer()}, tOperationNode);
    auto tOperationInput = tOperationNode.append_child("Input");
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tOperationInput);
    auto tOperationOutput = tOperationNode.append_child("Output");
    auto tOutputSharedData = std::string("Objective Gradient ID-") + aObjective.name;
    XMLGen::append_children({"ArgumentName", "SharedDataName"}, {"Objective Gradient", tOutputSharedData}, tOperationOutput);
}

void append_objective_gradient_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    for(auto& tObjective : aXMLMetaData.objectives)
    {
        auto tStageNode = aDocument.append_child("Stage");
        auto tStageName = std::string("Compute Objective Gradient ID-") + tObjective.name;
        XMLGen::append_children({"Name", "Type"}, {tStageName, tObjective.category()}, tStageNode);
        auto tInputNode = tStageNode.append_child("Input");
        XMLGen::append_children({"SharedDataName"}, {"Control"}, tInputNode);

        XMLGen::append_filter_control_operation(tStageNode);
        XMLGen::append_objective_gradient_operation(tObjective, tStageNode);
        auto tSharedDataName = std::string("Objective Gradient ID-") + tObjective.name;
        XMLGen::append_filter_criterion_gradient_operation(tSharedDataName, tStageNode);

        auto tOutputNode = tStageNode.append_child("Output");
        XMLGen::append_children({"SharedDataName"}, {tSharedDataName}, tOutputNode);
    }
}

void append_stages
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    XMLGen::append_design_volume_stage(aDocument);
    XMLGen::append_initial_guess_stage(aDocument);
    XMLGen::append_lower_bound_stage(aXMLMetaData, aDocument);
    XMLGen::append_upper_bound_stage(aXMLMetaData, aDocument);
    XMLGen::append_plato_main_output_stage(aXMLMetaData, aDocument);
    XMLGen::append_cache_state_stage(aXMLMetaData, aDocument);
    XMLGen::append_update_problem_stage(aXMLMetaData, aDocument);
    XMLGen::append_constraint_value_stage(aXMLMetaData, aDocument);
    XMLGen::append_constraint_gradient_stage(aXMLMetaData, aDocument);
    XMLGen::append_objective_value_stage(aXMLMetaData, aDocument);
    XMLGen::append_objective_gradient_stage(aXMLMetaData, aDocument);
}

void write_interface_xml_file
(const XMLGen::InputData& aMetaData)
{
    pugi::xml_document tDocument;
    auto tNode = tDocument.append_child("Console");
    XMLGen::append_children({"Verbose"}, {aMetaData.mVerbose}, tNode);
    XMLGen::append_plato_main_performer(tDocument);
    XMLGen::append_physics_performers(aMetaData, tDocument);
    XMLGen::append_shared_data(aMetaData, tDocument);
    XMLGen::append_stages(aMetaData, tDocument);
    XMLGen::append_optimizer_options(aMetaData, tDocument);
    tDocument.save_file("interface.xml", "  ");
}

void write_plato_main_operations_xml_file
(const XMLGen::InputData& aMetaData)
{
    pugi::xml_document tDocument;
    XMLGen::append_filter_options_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_output_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_initialize_field_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_set_lower_bounds_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_set_upper_bounds_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_design_volume_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_compute_volume_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_compute_volume_gradient_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_update_problem_to_plato_main_operation(aMetaData, tDocument);
    XMLGen::append_filter_control_to_plato_main_operation(tDocument);
    XMLGen::append_filter_gradient_to_plato_main_operation(tDocument);
    tDocument.save_file("plato_main_operations.xml", "  ");
}

void append_compute_objective_value_to_plato_analyze_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsPlatoAnalyzePerformer = XMLGen::is_any_objective_computed_by_plato_analyze(aMetaData);
    if(tIsPlatoAnalyzePerformer)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "ComputeObjectiveValue", "Compute Objective Value" }, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children( { "ArgumentName" }, { "Objective Value" }, tOutput);
    }
}

void append_compute_objective_gradient_to_plato_analyze_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsPlatoAnalyzePerformer = XMLGen::is_any_objective_computed_by_plato_analyze(aMetaData);
    if(tIsPlatoAnalyzePerformer)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "ComputeObjectiveGradient", "Compute Objective Gradient" }, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children( { "ArgumentName" }, { "Objective Gradient" }, tOutput);
    }
}

void append_compute_constraint_value_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsPlatoAnalyzePerformer = XMLGen::is_any_constraint_computed_by_plato_analyze(aXMLMetaData);
    if(tIsPlatoAnalyzePerformer)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "ComputeConstraintValue", "Compute Constraint Value" }, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children( { "ArgumentName" }, { "Constraint Value" }, tOutput);
    }
}

void append_compute_constraint_gradient_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsPlatoAnalyzePerformer = XMLGen::is_any_constraint_computed_by_plato_analyze(aXMLMetaData);
    if(tIsPlatoAnalyzePerformer)
    {
        auto tOperation = aDocument.append_child("Operation");
        XMLGen::append_children( { "Function", "Name" }, { "ComputeConstraintGradient", "Compute Constraint Gradient" }, tOperation);
        auto tInput = tOperation.append_child("Input");
        XMLGen::append_children( { "ArgumentName" }, { "Topology" }, tInput);
        auto tOutput = tOperation.append_child("Output");
        XMLGen::append_children( { "ArgumentName" }, { "Constraint Gradient" }, tOutput);
    }
}

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

namespace Analyze
{

inline void write_optimization_problem
(const XMLGen::InputData& aMetaData)
{
    XMLGen::write_interface_xml_file(aMetaData);
    XMLGen::write_plato_main_operations_xml_file(aMetaData);
    XMLGen::write_plato_main_input_deck_file(aMetaData);
    XMLGen::write_plato_analyze_operation_xml_file(aMetaData);
    XMLGen::write_amgx_input_file();
    XMLGen::write_plato_analyze_input_deck_file(aMetaData);
    XMLGen::generate_launch_script(aMetaData);
    std::cout << "Successfully wrote XML files." << std::endl;
}

}
// namespace Analyze

}
// namespace XMLGen

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, WritePlatoAnalyzeOperationsXmlFile)
{
    XMLGen::InputData tMetaData;
    tMetaData.max_iterations = "10";
    tMetaData.discretization = "density";
    tMetaData.optimization_algorithm = "oc";
    tMetaData.mProblemUpdateFrequency = "5";
    XMLGen::Objective tObjective;
    tObjective.name = "1";
    tObjective.type = "compliance";
    tObjective.code_name = "plato_analyze";
    tObjective.mPerformerName = "plato_analyze_1";
    tMetaData.objectives.push_back(tObjective);
    XMLGen::Constraint tConstraint;
    tConstraint.code("plato_analyze");
    tMetaData.constraints.push_back(tConstraint);
    XMLGen::Scenario tScenario;
    tScenario.performer("plato_analyze_1");
    tScenario.cacheState("false");
    tScenario.updateProblem("true");
    tMetaData.append(tScenario);

    XMLGen::write_plato_analyze_operation_xml_file(tMetaData);

    auto tReadData = XMLGen::read_data_from_file("plato_analyze_operations.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><Operation><Function>UpdateProblem</Function><Name>UpdateProblem</Name></Operation><Operation><Function>ComputeObjectiveValue</Function><Name>ComputeObjectiveValue</Name>")
        +"<Input><ArgumentName>Topology</ArgumentName></Input><Output><ArgumentName>ObjectiveValue</ArgumentName></Output></Operation><Operation><Function>ComputeObjectiveGradient</Function><Name>ComputeObjectiveGradient</Name>"
        +"<Input><ArgumentName>Topology</ArgumentName></Input><Output><ArgumentName>ObjectiveGradient</ArgumentName></Output></Operation><Operation><Function>ComputeConstraintValue</Function><Name>ComputeConstraintValue</Name>"
        +"<Input><ArgumentName>Topology</ArgumentName></Input><Output><ArgumentName>ConstraintValue</ArgumentName></Output></Operation><Operation><Function>ComputeConstraintGradient</Function><Name>ComputeConstraintGradient</Name>"
        +"<Input><ArgumentName>Topology</ArgumentName></Input><Output><ArgumentName>ConstraintGradient</ArgumentName></Output></Operation>";
    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());
    Plato::system("rm -f plato_analyze_operations.xml");
}

TEST(PlatoTestXMLGenerator, AppendComputeConstraintGradientToPlatoAnalyzeOperation)
{
    XMLGen::InputData tMetaData;
    XMLGen::Constraint tConstraint;
    tConstraint.code("plato_analyze");
    tMetaData.constraints.push_back(tConstraint);

    pugi::xml_document tDocument;
    XMLGen::append_compute_constraint_gradient_to_plato_analyze_operation(tMetaData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Input", "Output"};
    std::vector<std::string> tValues = {"ComputeConstraintGradient", "Compute Constraint Gradient", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    auto tOutput = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Constraint Gradient"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendComputeConstraintValueToPlatoAnalyzeOperation)
{
    XMLGen::InputData tMetaData;
    XMLGen::Constraint tConstraint;
    tConstraint.code("plato_analyze");
    tMetaData.constraints.push_back(tConstraint);

    pugi::xml_document tDocument;
    XMLGen::append_compute_constraint_value_to_plato_analyze_operation(tMetaData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Input", "Output"};
    std::vector<std::string> tValues = {"ComputeConstraintValue", "Compute Constraint Value", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    auto tOutput = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Constraint Value"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendComputeObjectiveGradientToPlatoAnalyzeOperation)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.code_name = "plato_analyze";
    tMetaData.objectives.push_back(tObjective);

    pugi::xml_document tDocument;
    XMLGen::append_compute_objective_gradient_to_plato_analyze_operation(tMetaData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Input", "Output"};
    std::vector<std::string> tValues = {"ComputeObjectiveGradient", "Compute Objective Gradient", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    auto tOutput = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Objective Gradient"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendComputeObjectiveValueToPlatoAnalyzeOperation)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.code_name = "plato_analyze";
    tMetaData.objectives.push_back(tObjective);

    pugi::xml_document tDocument;
    XMLGen::append_compute_objective_value_to_plato_analyze_operation(tMetaData, tDocument);

    auto tOperation = tDocument.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    std::vector<std::string> tKeys = {"Function", "Name", "Input", "Output"};
    std::vector<std::string> tValues = {"ComputeObjectiveValue", "Compute Objective Value", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tInput = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Topology"}, tInput);
    auto tOutput = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName"}, {"Objective Value"}, tOutput);
}

TEST(PlatoTestXMLGenerator, WritePlatoMainOperationsXmlFile)
{
    XMLGen::InputData tMetaData;
    tMetaData.max_iterations = "10";
    tMetaData.discretization = "density";
    tMetaData.optimization_algorithm = "oc";
    XMLGen::Objective tObjective;
    tObjective.name = "1";
    tObjective.type = "compliance";
    tObjective.mPerformerName = "plato_analyze_1";
    tMetaData.objectives.push_back(tObjective);
    XMLGen::Scenario tScenario;
    tScenario.performer("plato_analyze_1");
    tScenario.cacheState("false");
    tScenario.updateProblem("true");
    tMetaData.append(tScenario);

    ASSERT_NO_THROW(XMLGen::write_plato_main_operations_xml_file(tMetaData));

    auto tReadData = XMLGen::read_data_from_file("plato_main_operations.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><Filter><Name>Kernel</Name><Scale>2.0</Scale></Filter><Operation><Function>InitializeField</Function><Name>InitializeField</Name><Method>Uniform</Method><Uniform><Value>0.5</Value></Uniform>")
    +"<Output><ArgumentName>InitializedField</ArgumentName></Output></Operation><Operation><Function>SetLowerBounds</Function><Name>ComputeLowerBounds</Name><Discretization>density</Discretization><Input><ArgumentName>LowerBoundValue</ArgumentName>"
    +"</Input><Output><ArgumentName>LowerBoundVector</ArgumentName></Output></Operation><Operation><Function>SetUpperBounds</Function><Name>ComputeUpperBounds</Name><Discretization>density</Discretization><Input><ArgumentName>UpperBoundValue</ArgumentName>"
    +"</Input><Output><ArgumentName>UpperBoundVector</ArgumentName></Output></Operation><Operation><Function>UpdateProblem</Function><Name>UpdateProblem</Name></Operation><Operation><Function>Filter</Function><Name>FilterControl</Name>"
    +"<Gradient>False</Gradient><Input><ArgumentName>Field</ArgumentName></Input><Output><ArgumentName>FilteredField</ArgumentName></Output></Operation><Operation><Function>Filter</Function><Name>FilterGradient</Name><Gradient>True</Gradient>"
    +"<Input><ArgumentName>Field</ArgumentName></Input><Input><ArgumentName>Gradient</ArgumentName></Input><Output><ArgumentName>FilteredGradient</ArgumentName></Output></Operation>";
    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());
    Plato::system("rm -f plato_main_operations.xml");
}

TEST(PlatoTestXMLGenerator, WriteInterfaceXmlFile)
{
    XMLGen::InputData tMetaData;
    tMetaData.max_iterations = "10";
    tMetaData.optimization_algorithm = "oc";
    XMLGen::Objective tObjective;
    tObjective.name = "1";
    tObjective.type = "compliance";
    tObjective.mPerformerName = "plato_analyze_1";
    tMetaData.objectives.push_back(tObjective);
    XMLGen::Scenario tScenario;
    tScenario.performer("plato_analyze_1");
    tScenario.cacheState("false");
    tScenario.updateProblem("true");
    tMetaData.append(tScenario);

    ASSERT_NO_THROW(XMLGen::write_interface_xml_file(tMetaData));

    auto tReadData = XMLGen::read_data_from_file("interface.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><Console><Verbose>false</Verbose></Console><Performer><Name>platomain</Name><Code>platomain</Code><PerformerID>0</PerformerID></Performer><Performer><Name>1</Name><Code>plato_analyze_1</Code>")
    +"<PerformerID></PerformerID></Performer><SharedData><Name>Control</Name><Type>Scalar</Type><Layout>NodalField</Layout><OwnerName>platomain</OwnerName><UserName>platomain</UserName></SharedData><SharedData><Name>LowerBoundValue</Name>"
    +"<Type>Scalar</Type><Layout>Global</Layout><Size>1</Size><OwnerName>platomain</OwnerName><UserName>platomain</UserName></SharedData><SharedData><Name>LowerBoundVector</Name><Type>Scalar</Type><Layout>NodalField</Layout><OwnerName>platomain</OwnerName>"
    +"<UserName>platomain</UserName></SharedData><SharedData><Name>UpperBoundValue</Name><Type>Scalar</Type><Layout>Global</Layout><Size>1</Size><OwnerName>platomain</OwnerName><UserName>platomain</UserName></SharedData>"
    +"<SharedData><Name>UpperBoundVector</Name><Type>Scalar</Type><Layout>NodalField</Layout><OwnerName>platomain</OwnerName><UserName>platomain</UserName></SharedData><SharedData><Name>DesignVolume</Name><Type>Scalar</Type>"
    +"<Layout>Global</Layout><Size>1</Size><OwnerName>platomain</OwnerName><UserName>platomain</UserName></SharedData><SharedData><Name>Topology</Name><Type>Scalar</Type><Layout>NodalField</Layout><OwnerName>platomain</OwnerName>"
    +"<UserName>platomain</UserName><UserName>plato_analyze_1</UserName></SharedData><SharedData><Name>ObjectiveValueID-1</Name><Type>Scalar</Type><Layout>Global</Layout><Size>1</Size><OwnerName>plato_analyze_1</OwnerName>"
    +"<UserName>platomain</UserName></SharedData><SharedData><Name>ObjectiveGradientID-1</Name><Type>Scalar</Type><Layout>NodalField</Layout><OwnerName>plato_analyze_1</OwnerName><UserName>platomain</UserName></SharedData>"
    +"<Stage><Name>DesignVolume</Name><Operation><Name>DesignVolume</Name><PerformerName>platomain</PerformerName><Output><ArgumentName>DesignVolume</ArgumentName><SharedDataName>DesignVolume</SharedDataName></Output></Operation>"
    +"<Output><SharedDataName>DesignVolume</SharedDataName></Output></Stage><Stage><Name>InitialGuess</Name><Operation><Name>InitializeField</Name><PerformerName>platomain</PerformerName><Output><ArgumentName>InitializedField</ArgumentName>"
    +"<SharedDataName>Control</SharedDataName></Output></Operation><Output><SharedDataName>Control</SharedDataName></Output></Stage><Stage><Name>SetLowerBounds</Name><Output><SharedDataName>LowerBoundVector</SharedDataName></Output></Stage>"
    +"<Stage><Name>SetUpperBounds</Name><Output><SharedDataName>UpperBoundVector</SharedDataName></Output></Stage><Stage><Name>UpdateProblemID-plato_analyze_1</Name><Operation><Name>UpdateProblem</Name><PerformerName>plato_analyze_1</PerformerName>"
    +"</Operation></Stage><Stage><Name>ComputeObjectiveValueID-1</Name><Type>compliance</Type><Input><SharedDataName>Control</SharedDataName></Input><Operation><Name>FilterControl</Name><PerformerName>platomain</PerformerName>"
    +"<Input><ArgumentName>Field</ArgumentName><SharedDataName>Control</SharedDataName></Input><Output><ArgumentName>FilteredField</ArgumentName><SharedDataName>Topology</SharedDataName></Output></Operation>"
    +"<Operation><Name>ComputeObjectiveValue</Name><PerformerName>plato_analyze_1</PerformerName><Input><ArgumentName>Topology</ArgumentName><SharedDataName>Topology</SharedDataName></Input><Output><ArgumentName>ObjectiveValue</ArgumentName>"
    +"<SharedDataName>ObjectiveValueID-1</SharedDataName></Output></Operation><Output><SharedDataName>ObjectiveValueID-1</SharedDataName></Output></Stage><Stage><Name>ComputeObjectiveGradientID-1</Name><Type>compliance</Type>"
    +"<Input><SharedDataName>Control</SharedDataName></Input><Operation><Name>FilterControl</Name><PerformerName>platomain</PerformerName><Input><ArgumentName>Field</ArgumentName><SharedDataName>Control</SharedDataName></Input>"
    +"<Output><ArgumentName>FilteredField</ArgumentName><SharedDataName>Topology</SharedDataName></Output></Operation><Operation><Name>ComputeObjectiveGradient</Name><PerformerName>plato_analyze_1</PerformerName><Input><ArgumentName>Topology</ArgumentName>"
    +"<SharedDataName>Topology</SharedDataName></Input><Output><ArgumentName>ObjectiveGradient</ArgumentName><SharedDataName>ObjectiveGradientID-1</SharedDataName></Output></Operation><Operation><Name>FilterGradient</Name><PerformerName>platomain</PerformerName>"
    +"<Input><ArgumentName>Field</ArgumentName><SharedDataName>Control</SharedDataName></Input><Input><ArgumentName>Gradient</ArgumentName><SharedDataName>ObjectiveGradientID-1</SharedDataName></Input><Output><ArgumentName>FilteredGradient</ArgumentName>"
    +"<SharedDataName>ObjectiveGradientID-1</SharedDataName></Output></Operation><Output><SharedDataName>ObjectiveGradientID-1</SharedDataName></Output></Stage><Optimizer><Package>OC</Package><Convergence><MaxIterations>10</MaxIterations></Convergence>"
    +"<UpdateProblemStage><Name>UpdateProblem</Name></UpdateProblemStage><OptimizationVariables><ValueName>Control</ValueName><InitializationStage>InitialGuess</InitializationStage><FilteredName>Topology</FilteredName>"
    +"<LowerBoundValueName>LowerBoundValue</LowerBoundValueName><LowerBoundVectorName>LowerBoundVector</LowerBoundVectorName><UpperBoundValueName>UpperBoundValue</UpperBoundValueName><UpperBoundVectorName>UpperBoundVector</UpperBoundVectorName>"
    +"<SetLowerBoundsStage>SetLowerBounds</SetLowerBoundsStage><SetUpperBoundsStage>SetUpperBounds</SetUpperBoundsStage></OptimizationVariables><Objective><GradientStageName>ComputeObjectiveGradientID-1</GradientStageName>"
    +"<GradientName>ObjectiveGradientID-1</GradientName><ValueStageName>ComputeObjectiveValueID-1</ValueStageName><ValueName>ObjectiveValueID-1</ValueName></Objective><BoundConstraint><Upper>1.0</Upper><Lower>0.0</Lower></BoundConstraint></Optimizer>";
    ASSERT_STREQ(tGold.c_str(), tReadData.str().c_str());
    Plato::system("rm -f interface.xml");
}

TEST(PlatoTestXMLGenerator, AppendObjectiveGradientStage)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.name = "1";
    tObjective.type = "compliance";
    tObjective.mPerformerName = "plato_analyze_1";
    tMetaData.objectives.push_back(tObjective);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_objective_gradient_stage(tMetaData, tDocument));

    // STAGE INPUTS
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    ASSERT_STREQ("Stage", tStage.name());
    auto tName = tStage.child("Name");
    ASSERT_STREQ("Compute Objective Gradient ID-1", tName.child_value());
    auto tType = tStage.child("Type");
    ASSERT_STREQ("compliance", tType.child_value());
    auto tInput = tStage.child("Input");
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({"SharedDataName"}, {"Control"}, tInput);

    // FILTER CONTROL OPERATION
    auto tOperation = tStage.child("Operation");
    std::vector<std::string> tKeys = {"Name", "PerformerName", "Input", "Output"};
    std::vector<std::string> tValues = {"Filter Control", "platomain", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Field", "Control"}, tOpInputs);
    auto tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Filtered Field", "Topology"}, tOpOutputs);

    // OBJECTIVE VALUE OPERATION
    tOperation = tOperation.next_sibling("Operation");
    tValues = {"Compute Objective Gradient", "plato_analyze_1", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tOpInputs);
    tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Objective Gradient", "Objective Gradient ID-1"}, tOpOutputs);

    // FILTER GRADIENT OPERATION
    tOperation = tOperation.next_sibling("Operation");
    tKeys = {"Name", "PerformerName", "Input", "Input", "Output"};
    tValues = {"Filter Gradient", "platomain", "", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Field", "Control"}, tOpInputs);
    tOpInputs = tOpInputs.next_sibling("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Gradient", "Objective Gradient ID-1"}, tOpInputs);
    tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Filtered Gradient", "Objective Gradient ID-1"}, tOpOutputs);

    // STAGE OUTPUT
    auto tOutput = tStage.child("Output");
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"SharedDataName"}, {"Objective Gradient ID-1"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveValueStage)
{
    XMLGen::InputData tMetaData;
    XMLGen::Objective tObjective;
    tObjective.name = "1";
    tObjective.type = "compliance";
    tObjective.mPerformerName = "plato_analyze_1";
    tMetaData.objectives.push_back(tObjective);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_objective_value_stage(tMetaData, tDocument));

    // STAGE INPUTS
    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    ASSERT_STREQ("Stage", tStage.name());
    auto tName = tStage.child("Name");
    ASSERT_STREQ("Compute Objective Value ID-1", tName.child_value());
    auto tType = tStage.child("Type");
    ASSERT_STREQ("compliance", tType.child_value());
    auto tInput = tStage.child("Input");
    ASSERT_STREQ("Input", tInput.name());
    PlatoTestXMLGenerator::test_children({"SharedDataName"}, {"Control"}, tInput);

    // FILTER CONTROL OPERATION
    auto tOperation = tStage.child("Operation");
    std::vector<std::string> tKeys = {"Name", "PerformerName", "Input", "Output"};
    std::vector<std::string> tValues = {"Filter Control", "platomain", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    auto tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Field", "Control"}, tOpInputs);
    auto tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Filtered Field", "Topology"}, tOpOutputs);

    // OBJECTIVE VALUE OPERATION
    tOperation = tOperation.next_sibling("Operation");
    tValues = {"Compute Objective Value", "plato_analyze_1", "", ""};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tOperation);
    tOpInputs = tOperation.child("Input");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Topology", "Topology"}, tOpInputs);
    tOpOutputs = tOperation.child("Output");
    PlatoTestXMLGenerator::test_children({"ArgumentName", "SharedDataName"}, {"Objective Value", "Objective Value ID-1"}, tOpOutputs);

    // STAGE OUTPUT
    auto tOutput = tStage.child("Output");
    ASSERT_STREQ("Output", tOutput.name());
    PlatoTestXMLGenerator::test_children({"SharedDataName"}, {"Objective Value ID-1"}, tOutput);
}

TEST(PlatoTestXMLGenerator, AppendCacheStateStage)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.performer("plato_analyze_1");
    tScenario.cacheState("true");
    tMetaData.append(tScenario);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_cache_state_stage(tMetaData, tDocument));

    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    ASSERT_STREQ("Stage", tStage.name());
    PlatoTestXMLGenerator::test_children({"Name", "Operation"}, {"Cache State ID-plato_analyze_1", ""}, tStage);
    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    PlatoTestXMLGenerator::test_children({"Name", "PerformerName"}, {"Cache State", "plato_analyze_1"}, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendUpdateProblemStage)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.performer("plato_analyze_1");
    tScenario.updateProblem("true");
    tMetaData.append(tScenario);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_update_problem_stage(tMetaData, tDocument));

    auto tStage = tDocument.child("Stage");
    ASSERT_FALSE(tStage.empty());
    ASSERT_STREQ("Stage", tStage.name());
    PlatoTestXMLGenerator::test_children({"Name", "Operation"}, {"Update Problem ID-plato_analyze_1", ""}, tStage);
    auto tOperation = tStage.child("Operation");
    ASSERT_FALSE(tOperation.empty());
    ASSERT_STREQ("Operation", tOperation.name());
    PlatoTestXMLGenerator::test_children({"Name", "PerformerName"}, {"Update Problem", "plato_analyze_1"}, tOperation);
}

TEST(PlatoTestXMLGenerator, AppendSharedData)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tScenario.performer("plato_analyze_1");
    tMetaData.append(tScenario);
    tMetaData.mOutputMetaData.scenarioID("1");
    tMetaData.mOutputMetaData.appendDeterminsiticQoI("dispx", "nodal field");
    XMLGen::Objective tObjective;
    tObjective.name = "1";
    tObjective.mPerformerName = "plato_analyze_1";
    tMetaData.objectives.push_back(tObjective);

    pugi::xml_document tDocument;
    XMLGen::append_shared_data(tMetaData, tDocument);

    // TEST
    auto tSharedData = tDocument.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"Control", "Scalar", "Nodal Field", "platomain", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    tValues = {"Lower Bound Value", "Scalar", "Global", "1", "platomain", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"Lower Bound Vector", "Scalar", "Nodal Field", "platomain", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    tValues = {"Upper Bound Value", "Scalar", "Global", "1", "platomain", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"Upper Bound Vector", "Scalar", "Nodal Field", "platomain", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    tValues = {"Design Volume", "Scalar", "Global", "1", "platomain", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"dispx", "Scalar", "Nodal Field", "plato_analyze_1", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName", "UserName"};
    tValues = {"Topology", "Scalar", "Nodal Field", "platomain", "platomain", "plato_analyze_1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "Size", "OwnerName", "UserName"};
    tValues = {"Objective Value ID-1", "Scalar", "Global", "1", "plato_analyze_1", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);

    tSharedData = tSharedData.next_sibling("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    tValues = {"Objective Gradient ID-1", "Scalar", "Nodal Field", "plato_analyze_1", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
}

TEST(PlatoTestXMLGenerator, AppendTopologySharedData_ErrorEmptyScenario)
{
    XMLGen::InputData tMetaData;
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_topology_shared_data(tMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendTopologySharedData)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tScenario.performer("plato_analyze_1");
    tMetaData.append(tScenario);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_topology_shared_data(tMetaData, tDocument));

    auto tSharedData = tDocument.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName", "UserName"};
    std::vector<std::string> tValues = {"Topology", "Scalar", "Nodal Field", "platomain", "platomain", "plato_analyze_1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
    tSharedData = tSharedData.next_sibling("Performer");
    ASSERT_TRUE(tSharedData.empty());
}

TEST(PlatoTestXMLGenerator, AppendQoISharedData_ErrorEmptyScenario)
{
    XMLGen::InputData tMetaData;
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_qoi_shared_data(tMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendQoISharedData_EmptySharedData)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tScenario.performer("plato_analyze_1");
    tMetaData.append(tScenario);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_qoi_shared_data(tMetaData, tDocument));

    auto tSharedData = tDocument.child("SharedData");
    ASSERT_TRUE(tSharedData.empty());
}

TEST(PlatoTestXMLGenerator, AppendQoISharedData_ErrorDidNotMatchOuputScenarioID)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tScenario.performer("plato_analyze_1");
    tMetaData.append(tScenario);
    tMetaData.mOutputMetaData.scenarioID("2");
    tMetaData.mOutputMetaData.appendDeterminsiticQoI("dispx", "nodal field");

    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_qoi_shared_data(tMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendQoISharedData)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tScenario.performer("plato_analyze_1");
    tMetaData.append(tScenario);
    tMetaData.mOutputMetaData.scenarioID("1");
    tMetaData.mOutputMetaData.appendDeterminsiticQoI("dispx", "nodal field");

    pugi::xml_document tDocument;
    XMLGen::append_qoi_shared_data(tMetaData, tDocument);

    auto tSharedData = tDocument.child("SharedData");
    ASSERT_FALSE(tSharedData.empty());
    ASSERT_STREQ("SharedData", tSharedData.name());
    std::vector<std::string> tKeys = {"Name", "Type", "Layout", "OwnerName", "UserName"};
    std::vector<std::string> tValues = {"dispx", "Scalar", "Nodal Field", "plato_analyze_1", "platomain"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tSharedData);
    tSharedData = tSharedData.next_sibling("Performer");
    ASSERT_TRUE(tSharedData.empty());
}

TEST(PlatoTestXMLGenerator, AppendPhysicsPerformers_EmptyScenario)
{
    XMLGen::InputData tMetaData;
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_physics_performers(tMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendPhysicsPerformers)
{
    XMLGen::InputData tMetaData;
    XMLGen::Scenario tScenario;
    tScenario.id("1");
    tScenario.code("plato_analyze");
    tScenario.performer("plato_analyze_1");
    tMetaData.append(tScenario);

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_physics_performers(tMetaData, tDocument));

    auto tPerformer = tDocument.child("Performer");
    ASSERT_FALSE(tPerformer.empty());
    ASSERT_STREQ("Performer", tPerformer.name());
    std::vector<std::string> tKeys = {"Name", "Code", "PerformerID"};
    std::vector<std::string> tValues = {"plato_analyze", "plato_analyze_1", "1"};
    PlatoTestXMLGenerator::test_children(tKeys, tValues, tPerformer);
    tPerformer = tPerformer.next_sibling("Performer");
    ASSERT_TRUE(tPerformer.empty());
}

}
// namespace PlatoTestXMLGenerator
