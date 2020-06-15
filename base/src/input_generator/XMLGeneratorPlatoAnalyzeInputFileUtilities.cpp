/*
 * XMLGeneratorPlatoAnalyzeInputFileUtilities.cpp
 *
 *  Created on: Jun 15, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorPlatoAnalyzeUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorAnalyzeNaturalBCFunctionInterface.hpp"
#include "XMLGeneratorAnalyzeNaturalBCTagFunctionInterface.hpp"
#include "XMLGeneratorAnalyzeMaterialModelFunctionInterface.hpp"
#include "XMLGeneratorAnalyzeAppendCriterionFunctionInterface.hpp"
#include "XMLGeneratorAnalyzeEssentialBCFunctionInterface.hpp"
#include "XMLGeneratorAnalyzeEssentialBCTagFunctionInterface.hpp"

namespace XMLGen
{

/**********************************************************************************/
void check_input_mesh_file_keyword
(const XMLGen::InputData& aXMLMetaData)
{
    if (aXMLMetaData.run_mesh_name.empty())
    {
        THROWERR("Check Input Mesh File Keyword: Input mesh filename is empty.")
    }
}
// function check_input_mesh_file_keyword
/**********************************************************************************/

/**********************************************************************************/
void check_objective_container_is_defined
(const XMLGen::InputData& aXMLMetaData)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR("Check Objective is Defined: Objective container is empty.")
    }
}
// function check_objective_container_is_defined
/**********************************************************************************/

/**********************************************************************************/
std::vector<std::string> return_list_of_objective_functions
(const XMLGen::InputData& aXMLMetaData)
{
    std::vector<std::string> tTokens;
    for(auto& tObjective : aXMLMetaData.objectives)
    {
        auto tToken = std::string("my ") + Plato::tolower(tObjective.type);
        tTokens.push_back(tToken);
    }
    return tTokens;
}
// function return_list_of_objective_functions
/**********************************************************************************/

/**********************************************************************************/
std::vector<std::string> return_list_of_objective_weights
(const XMLGen::InputData& aXMLMetaData)
{
    std::vector<std::string> tTokens;
    for(auto& tObjective : aXMLMetaData.objectives)
    {
        auto tWeight = tObjective.weight.empty() ? "1.0" : tObjective.weight;
        tTokens.push_back(tWeight);
    }
    return tTokens;
}
// function return_list_of_objective_weights
/**********************************************************************************/

/**********************************************************************************/
std::vector<std::string> return_list_of_constraint_functions
(const XMLGen::InputData& aXMLMetaData)
{
    std::vector<std::string> tTokens;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tToken = std::string("my ") + Plato::tolower(tConstraint.type);
        tTokens.push_back(tToken);
    }
    return tTokens;
}
// function return_list_of_constraint_functions
/**********************************************************************************/

/**********************************************************************************/
std::vector<std::string> return_list_of_constraint_weights
(const XMLGen::InputData& aXMLMetaData)
{
    std::vector<std::string> tTokens;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto tWeight = tConstraint.weight.empty() ? "1.0" : tConstraint.weight;
        tTokens.push_back(tWeight);
    }
    return tTokens;
}
// function return_list_of_constraint_weights
/**********************************************************************************/

/**********************************************************************************/
std::string transform_tokens_for_plato_analyze_input_deck
(const std::vector<std::string> &aTokens)
{
    if(aTokens.empty())
    {
        THROWERR("Transform Tokens for Plato Analyze Input Deck: Input list of tokens is empty.")
    }

    std::string tOutput("{");
    auto tEndIndex = aTokens.size() - 1u;
    auto tEndIterator = std::next(aTokens.begin(), tEndIndex);
    for(auto tItr = aTokens.begin(); tItr != tEndIterator; ++tItr)
    {
        auto tIndex = std::distance(aTokens.begin(), tItr);
        tOutput += aTokens[tIndex] + ", ";
    }
    tOutput += aTokens[tEndIndex] + "}";
    return tOutput;
}
// function transform_tokens_for_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
void append_problem_description_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    XMLGen::ValidSpatialDimsKeys tValidKeys;
    auto tSpatialDim = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), aXMLMetaData.mPhysicsMetaData.mSpatialDims);
    if (tSpatialDim == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Append Problem Description to Plato Analyze Input Deck: Invalid spatial dimensions '")
            + aXMLMetaData.mPhysicsMetaData.mSpatialDims + "'.  Only three and two dimensional problems are supported in Plato Analyze.")
    }
    XMLGen::check_input_mesh_file_keyword(aXMLMetaData);

    auto tProblem = aDocument.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Problem"}, tProblem);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Physics", "string", "Plato Driver"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tProblem);
    tValues = {"Spatial Dimension", "int", tSpatialDim->c_str()};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tProblem);
    tValues = {"Input Mesh", "string", aXMLMetaData.run_mesh_name};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tProblem);
}
// function append_problem_description_to_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
void append_physics_parameter_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::ValidAnalyzePhysicsKeys tValidKeys;
    auto tLowerPhysics = Plato::tolower(aXMLMetaData.mPhysicsMetaData.mPhysics);
    auto tPhysics = tValidKeys.mKeys.find(tLowerPhysics);
    if (tPhysics == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Append Physics to Plato Analyze Input Deck: Physics '")
            + tLowerPhysics + "' is not supported in Plato Analyze.")
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Physics", "string", tPhysics->second.first};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_physics_parameter_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_pde_constraint_parameter_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::ValidAnalyzePhysicsKeys tValidKeys;
    auto tLowerPhysics = Plato::tolower(aXMLMetaData.mPhysicsMetaData.mPhysics);
    auto tPhysics = tValidKeys.mKeys.find(tLowerPhysics);
    if (tPhysics == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Append PDE Constraint to Plato Analyze Input Deck: Physics '")
            + tLowerPhysics + "' is not supported in Plato Analyze.")
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"PDE Constraint", "string", tPhysics->second.second};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_physics_parameter_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_constraint_parameter_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(XMLGen::is_any_constraint_computed_by_plato_analyze(aXMLMetaData) == false)
    {
        return;
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Constraint", "string", "My Constraint"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_constraint_parameter_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_objective_parameter_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(XMLGen::is_any_objective_computed_by_plato_analyze(aXMLMetaData) == false)
    {
        return;
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Objective", "string", "My Objective"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_objective_parameter_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_self_adjoint_parameter_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::check_objective_container_is_defined(aXMLMetaData);

    std::string tIsSelfAdjoint = "false";
    if(aXMLMetaData.objectives.size() == 1u)
    {
        XMLGen::ValidAnalyzeCriteriaKeys tValidKeys;
        auto tLowerCriterion = Plato::tolower(aXMLMetaData.objectives[0].type);
        auto tItr = tValidKeys.mKeys.find(tLowerCriterion);
        if (tItr == tValidKeys.mKeys.end())
        {
            THROWERR(std::string("Append Self Adjoint Parameter to Plato Analyze Input Deck: Criterion '")
                + tLowerCriterion + "' is not supported.")
        }
        tIsSelfAdjoint = tItr->second.second ? "true" : "false";
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Self-Adjoint", "bool", tIsSelfAdjoint};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_self_adjoint_parameter_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_plato_problem_description_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tProblem = aDocument.child("ParameterList");
    if(tProblem.empty())
    {
        THROWERR("Append Plato Problem To Plato Analyze Input Deck: Parameter List with name 'Problem' is empty.")
    }
    auto tPlatoProblem = tProblem.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Plato Problem"}, tPlatoProblem);
    XMLGen::append_physics_parameter_to_plato_problem(aXMLMetaData, tPlatoProblem);
    XMLGen::append_pde_constraint_parameter_to_plato_problem(aXMLMetaData, tPlatoProblem);
    XMLGen::append_constraint_parameter_to_plato_problem(aXMLMetaData, tPlatoProblem);
    XMLGen::append_objective_parameter_to_plato_problem(aXMLMetaData, tPlatoProblem);
    XMLGen::append_self_adjoint_parameter_to_plato_problem(aXMLMetaData, tPlatoProblem);
}
// function append_plato_problem_description_to_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
void append_plato_problem_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tProblem = aDocument.child("ParameterList");
    if(tProblem.empty())
    {
        THROWERR("Append Plato Problem To Plato Analyze Input Deck: Parameter List with name 'Problem' is empty.")
    }
    auto tPlatoProblem = tProblem.child("ParameterList");
    if(tPlatoProblem.empty())
    {
        THROWERR("Append Plato Problem To Plato Analyze Input Deck: Parameter List with name 'Plato Problem' is empty.")
    }
    XMLGen::append_objective_criteria_to_plato_analyze_input_deck(aXMLMetaData, tPlatoProblem);
    XMLGen::append_constraint_criteria_to_plato_analyze_input_deck(aXMLMetaData, tPlatoProblem);
    XMLGen::append_partial_differential_equation_to_plato_analyze_input_deck(aXMLMetaData, tPlatoProblem);
    XMLGen::append_material_model_to_plato_analyze_input_deck(aXMLMetaData, tPlatoProblem);
    XMLGen::append_natural_boundary_conditions_to_plato_analyze_input_deck(aXMLMetaData, tPlatoProblem);
    XMLGen::append_essential_boundary_conditions_to_plato_analyze_input_deck(aXMLMetaData, tPlatoProblem);
}
// function append_plato_problem_to_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
void append_weighted_sum_objective_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::append_attributes({"name"}, {"My Objective"}, aParentNode);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "Weighted Sum"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_weighted_sum_objective_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_functions_to_weighted_sum_objective
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tTokens = XMLGen::return_list_of_objective_functions(aXMLMetaData);
    auto tFunctions = XMLGen::transform_tokens_for_plato_analyze_input_deck(tTokens);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Functions", "Array(string)", tFunctions};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_functions_to_weighted_sum_objective
/**********************************************************************************/

/**********************************************************************************/
void append_weights_to_weighted_sum_objective
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tTokens = XMLGen::return_list_of_objective_weights(aXMLMetaData);
    auto tWeights = XMLGen::transform_tokens_for_plato_analyze_input_deck(tTokens);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Weights", "Array(double)", tWeights};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_weights_to_weighted_sum_objective
/**********************************************************************************/

/**********************************************************************************/
void append_objective_criteria_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::AppendCriterionParameters<XMLGen::Objective> tFunctionInterface;
    for(auto& tObjective : aXMLMetaData.objectives)
    {
        tFunctionInterface.call(tObjective, aParentNode);
    }
}
// function append_objective_criteria_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_objective_criteria_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tObjective = aParentNode.append_child("ParameterList");
    XMLGen::append_weighted_sum_objective_to_plato_problem(aXMLMetaData, tObjective);
    XMLGen::append_functions_to_weighted_sum_objective(aXMLMetaData, tObjective);
    XMLGen::append_weights_to_weighted_sum_objective(aXMLMetaData, tObjective);
    XMLGen::append_objective_criteria_to_plato_problem(aXMLMetaData, aParentNode);
}
// function append_objective_criteria_to_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
void append_constraint_criteria_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::AppendCriterionParameters<XMLGen::Constraint> tFunctionInterface;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        tFunctionInterface.call(tConstraint, aParentNode);
    }
}
// function append_constraint_criteria_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_weighted_sum_constraint_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::append_attributes({"name"}, {"My Constraint"}, aParentNode);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "Weighted Sum"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_weighted_sum_constraint_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_functions_to_weighted_sum_constraint
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tTokens = XMLGen::return_list_of_constraint_functions(aXMLMetaData);
    auto tFunctions = XMLGen::transform_tokens_for_plato_analyze_input_deck(tTokens);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Functions", "Array(string)", tFunctions};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_functions_to_weighted_sum_constraint
/**********************************************************************************/

/**********************************************************************************/
void append_weights_to_weighted_sum_constraint
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tTokens = XMLGen::return_list_of_constraint_weights(aXMLMetaData);
    auto tWeights = XMLGen::transform_tokens_for_plato_analyze_input_deck(tTokens);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Weights", "Array(double)", tWeights};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_weights_to_weighted_sum_constraint
/**********************************************************************************/

/**********************************************************************************/
void append_constraint_criteria_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tObjective = aParentNode.append_child("ParameterList");
    XMLGen::append_weighted_sum_constraint_to_plato_problem(aXMLMetaData, tObjective);
    XMLGen::append_functions_to_weighted_sum_constraint(aXMLMetaData, tObjective);
    XMLGen::append_weights_to_weighted_sum_constraint(aXMLMetaData, tObjective);
    XMLGen::append_constraint_criteria_to_plato_problem(aXMLMetaData, aParentNode);
}
// function append_constraint_criteria_to_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
void append_partial_differential_equation_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::ValidAnalyzePhysicsKeys tValidKeys;
    auto tLowerPhysics = Plato::tolower(aXMLMetaData.mPhysicsMetaData.mPhysics);
    auto tPhysicsItr = tValidKeys.mKeys.find(tLowerPhysics);
    if(tPhysicsItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Append Partial Differential Equation to Plato Analyze Input Deck: Physics '")
            + tLowerPhysics + "' is not supported in Plato Analyze.")
    }
    auto tPhysics = aParentNode.append_child("ParameterList");
    auto tPDECategory = tPhysicsItr->second.second;
    XMLGen::append_attributes({"name"}, {tPDECategory}, tPhysics);
    XMLGen::Private::append_simp_penalty_function(aXMLMetaData.mPhysicsMetaData, tPhysics);
}
// function append_partial_differential_equation_to_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
void append_material_model_to_plato_problem
(const std::vector<XMLGen::Material>& aMaterials,
 pugi::xml_node& aParentNode)
{
    if(aMaterials.empty())
    {
        THROWERR("Append Material Model to Plato Problem: Material container is empty.")
    }

    XMLGen::AppendMaterialModelParameters tMaterialInterface;
    for(auto& tMaterial : aMaterials)
    {
        tMaterialInterface.call(tMaterial, aParentNode);
    }
}
// function append_material_model_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_material_model_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tIsRandomUseCase = aXMLMetaData.mRandomMetaData.samples().empty() ? false : true;
    if(tIsRandomUseCase)
    {
        auto tRandomMaterials = aXMLMetaData.mRandomMetaData.materials();
        XMLGen::append_material_model_to_plato_problem(tRandomMaterials, aParentNode);
    }
    else
    {
        XMLGen::append_material_model_to_plato_problem(aXMLMetaData.materials, aParentNode);
    }
}
// function append_material_model_to_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
void append_natural_boundary_conditions_to_plato_problem
(const XMLGen::LoadCase& aLoadCase,
 pugi::xml_node& aParentNode)
{
    auto tLowerPerformer = Plato::tolower(aLoadCase.mPerformerName);
    if (tLowerPerformer.compare("plato_analyze") != 0)
    {
        return;
    }

    auto tNaturalBC = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Natural Boundary Conditions"}, tNaturalBC);

    XMLGen::AppendNaturalBoundaryCondition tNaturalBCFuncInterface;
    XMLGen::NaturalBoundaryConditionTag tNaturalBCNameFuncInterface;
    for(auto& tLoad : aLoadCase.loads)
    {
        auto tName = tNaturalBCNameFuncInterface.call(tLoad);
        tNaturalBCFuncInterface.call(tName, tLoad, tNaturalBC);
    }
}
// function append_natural_boundary_conditions_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_natural_boundary_conditions_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tIsRandomUseCase = aXMLMetaData.mRandomMetaData.samples().empty() ? false : true;
    if(tIsRandomUseCase)
    {
        auto tRandomLoads = aXMLMetaData.mRandomMetaData.loadcase();
        XMLGen::append_natural_boundary_conditions_to_plato_problem(tRandomLoads, aParentNode);
    }
    else
    {
        for (auto &tLoadCase : aXMLMetaData.load_cases)
        {
            XMLGen::append_natural_boundary_conditions_to_plato_problem(tLoadCase, aParentNode);
        }
    }
}
// function append_natural_boundary_conditions_to_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
void append_essential_boundary_conditions_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tEssentialBC = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Essential Boundary Conditions"}, tEssentialBC);

    XMLGen::EssentialBoundaryConditionTag tTagInterface;
    XMLGen::AppendEssentialBoundaryCondition tFuncInterface;
    for (auto &tBC : aXMLMetaData.bcs)
    {
        auto tName = tTagInterface.call(tBC);
        tFuncInterface.call(tName, tBC, tEssentialBC);
    }
}
// function append_essential_boundary_conditions_to_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
void write_plato_analyze_input_deck_file
(const XMLGen::InputData& aXMLMetaData)
{
    pugi::xml_document tDocument;

    XMLGen::append_problem_description_to_plato_analyze_input_deck(aXMLMetaData, tDocument);
    XMLGen::append_plato_problem_description_to_plato_analyze_input_deck(aXMLMetaData, tDocument);
    XMLGen::append_plato_problem_to_plato_analyze_input_deck(aXMLMetaData, tDocument);

    tDocument.save_file("plato_analyze_input_deck.xml", "  ");
}
// function write_plato_analyze_input_deck_file
/**********************************************************************************/

}
// namespace XMLGen
