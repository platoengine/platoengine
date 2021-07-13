/*
 * XMLGeneratorPlatoAnalyzeInputFileUtilities.cpp
 *
 *  Created on: Jun 15, 2020
 */

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorPlatoAnalyzeUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorAnalyzePhysicsFunctionInterface.hpp"
#include "XMLGeneratorAnalyzeLoadFunctionInterface.hpp"
#include "XMLGeneratorAnalyzeLoadTagFunctionInterface.hpp"
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
    if (aXMLMetaData.mesh.run_name.empty())
    {
        THROWERR("Check Input Mesh File Keyword: Input mesh filename is empty.")
    }
}
// function check_input_mesh_file_keyword
/**********************************************************************************/

/**********************************************************************************/
void is_objective_container_empty
(const XMLGen::InputData& aXMLMetaData)
{
    if(aXMLMetaData.objective.serviceIDs.size() == 0)
    {
        THROWERR("Check Objective is Defined: Objective container is empty.")
    }
}
// function is_objective_container_empty
/**********************************************************************************/

/**********************************************************************************/
std::vector<std::string> return_list_of_objective_functions
(const XMLGen::InputData& aXMLMetaData)
{
    std::vector<std::string> tTokens;
    for(size_t i=0; i<aXMLMetaData.objective.criteriaIDs.size(); ++i)
    {
        auto tCriterion = aXMLMetaData.criterion(aXMLMetaData.objective.criteriaIDs[i]);
        auto tCriterionType = Plato::tolower(tCriterion.type());
        if(tCriterionType == "composite")
        {
            for(auto &tCriterionID : tCriterion.criterionIDs())
            {
                auto tSubCriterion = aXMLMetaData.criterion(tCriterionID);
                auto tSubCriterionType = Plato::tolower(tSubCriterion.type());
                auto tToken = std::string("my_") + tSubCriterionType + "_criterion_id_" + tCriterionID;
                tTokens.push_back(tToken);
            }
        }
        else
        {
            auto tToken = std::string("my_") + tCriterionType + "_criterion_id_" + aXMLMetaData.objective.criteriaIDs[i];
            tTokens.push_back(tToken);
        }
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
    for(size_t i=0; i<aXMLMetaData.objective.criteriaIDs.size(); ++i)
    {
        auto tCriterion = aXMLMetaData.criterion(aXMLMetaData.objective.criteriaIDs[i]);
        auto tCriterionType = Plato::tolower(tCriterion.type());
        if(tCriterionType == "composite")
        {
            for(auto &tWeight : tCriterion.criterionWeights())
            {
                auto tReturnWeight = tWeight.empty() ? "1.0" : tWeight;
                tTokens.push_back(tReturnWeight);
            }
        }
        else
        {
            // If it isn't a composite criterion we won't be relying on Plato Analyze
            // to do the weighting.
            tTokens.push_back("1.0");
        }
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
        auto &tCriterion = aXMLMetaData.criterion(tConstraint.criterion());
        auto tCriterionType = Plato::tolower(tCriterion.type());
        if(tCriterionType == "composite")
        {
            for(auto &tCriterionID : tCriterion.criterionIDs())
            {
                auto tSubCriterion = aXMLMetaData.criterion(tCriterionID);
                auto tSubCriterionType = Plato::tolower(tSubCriterion.type());
                auto tToken = std::string("my ") + tSubCriterionType;
                tTokens.push_back(tToken);
            }
        }
        else
        {
            auto tToken = std::string("my ") + tCriterionType;
            tTokens.push_back(tToken);
        }
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
        auto tCriterion = aXMLMetaData.criterion(tConstraint.criterion());
        auto tCriterionType = Plato::tolower(tCriterion.type());
        if(tCriterionType == "composite")
        {
            for(auto &tWeight : tCriterion.criterionWeights())
            {
                auto tReturnWeight = tWeight.empty() ? "1.0" : tWeight;
                tTokens.push_back(tReturnWeight);
            }
        }
        else
        {
            auto tCurrentWeight = tConstraint.weight();
            auto tProposedWeight = tCurrentWeight.empty() ? "1.0" : tCurrentWeight;
            tTokens.push_back(tProposedWeight);
        }
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
        tOutput += aTokens[tIndex] + ",";
    }
    tOutput += aTokens[tEndIndex] + "}";

    std::replace(tOutput.begin(), tOutput.end(), ' ', ','); // guard against the possibility of white spaces between two consecutive number characters

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
    auto tSpatialDim = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), aXMLMetaData.scenario(0u).dimensions());
    if (tSpatialDim == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Append Problem Description to Plato Analyze Input Deck: Invalid spatial dimensions '")
            + aXMLMetaData.scenario(0u).dimensions() + "'.  Only three and two dimensional problems are supported in Plato Analyze.")
    }
    XMLGen::check_input_mesh_file_keyword(aXMLMetaData);

    auto tProblem = aDocument.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Problem"}, tProblem);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Physics", "string", "Plato Driver"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tProblem);
    tValues = {"Spatial Dimension", "int", tSpatialDim->c_str()};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tProblem);
    tValues = {"Input Mesh", "string", aXMLMetaData.mesh.run_name};
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
    auto tPhysicsTag = tValidKeys.physics(aXMLMetaData.scenario(0u).physics());
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Physics", "string", tPhysicsTag};
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
    auto tPDE = tValidKeys.pde(aXMLMetaData.scenario(0u).physics());
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"PDE Constraint", "string", tPDE};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}
// function append_physics_parameter_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_self_adjoint_parameter_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::is_objective_container_empty(aXMLMetaData);

    std::string tIsSelfAdjoint = "false";
    if(aXMLMetaData.objective.criteriaIDs.size() == 1u)
    {
        XMLGen::ValidAnalyzeCriteriaKeys tValidKeys;
        auto &tCriterion = aXMLMetaData.criterion(aXMLMetaData.objective.criteriaIDs[0]);
        auto tLowerCriterion = Plato::tolower(tCriterion.type());
        auto tItr = tValidKeys.mKeys.find(tLowerCriterion);
        if (tItr == tValidKeys.mKeys.end())
        {
            THROWERR(std::string("Append Self Adjoint Parameter to Plato Analyze Input Deck: Criterion '")
                + tLowerCriterion + "' is not supported.")
        }
        tIsSelfAdjoint = tItr->second.second ? "true" : "false";
    }
    if(aXMLMetaData.optimization_parameters().optimizationType() == OT_SHAPE)
    {
        tIsSelfAdjoint = "false";
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
    XMLGen::append_criteria_list_to_plato_analyze_input_deck(aXMLMetaData, tPlatoProblem);
    XMLGen::append_physics_to_plato_analyze_input_deck(aXMLMetaData, tPlatoProblem);
    XMLGen::append_spatial_model_to_plato_analyze_input_deck(aXMLMetaData, tPlatoProblem);
    XMLGen::append_material_models_to_plato_analyze_input_deck(aXMLMetaData, tPlatoProblem);
    XMLGen::append_loads_to_plato_analyze_input_deck(aXMLMetaData, tPlatoProblem);
    XMLGen::append_essential_boundary_conditions_to_plato_analyze_input_deck(aXMLMetaData, tPlatoProblem);
}
// function append_plato_problem_to_plato_analyze_input_deck
/**********************************************************************************/
void append_criteria_list_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tCriteriaList = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Criteria"}, tCriteriaList);
    XMLGen::append_objective_criteria_to_criteria_list(aXMLMetaData, tCriteriaList);
    XMLGen::append_constraint_criteria_to_criteria_list(aXMLMetaData, tCriteriaList);
}

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
 const std::vector<std::string> &aObjectiveFunctions,
 pugi::xml_node& aParentNode)
{
//    auto tTokens = XMLGen::return_list_of_objective_functions(aXMLMetaData);
    auto tFunctions = XMLGen::transform_tokens_for_plato_analyze_input_deck(aObjectiveFunctions);
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
pugi::xml_node append_objective_criteria_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    pugi::xml_node tReturn;
    XMLGen::AppendCriterionParameters<XMLGen::Criterion> tFunctionInterface;
    for(auto& tCriteriaID : aXMLMetaData.objective.criteriaIDs)
    {
        auto &tCriterion = aXMLMetaData.criterion(tCriteriaID);
        auto tCriterionType = Plato::tolower(tCriterion.type());
        if(tCriterionType == "composite")
        {
            for(auto &tCriterionID : tCriterion.criterionIDs())
            {
                auto tSubCriterion = aXMLMetaData.criterion(tCriterionID);
                tReturn = tFunctionInterface.call(tSubCriterion, aParentNode);
            }
        }
        else
        {
            tReturn = tFunctionInterface.call(tCriterion, aParentNode);
        }
    }
    return tReturn;
}
// function append_objective_criteria_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_objective_criteria_to_criteria_list
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(XMLGen::is_any_objective_computed_by_plato_analyze(aXMLMetaData) == false)
    {
        return;
    }

    auto tObjectiveFunctions = XMLGen::return_list_of_objective_functions(aXMLMetaData);
    if(tObjectiveFunctions.size() > 1)
    {
        auto tObjective = aParentNode.append_child("ParameterList");
        XMLGen::append_weighted_sum_objective_to_plato_problem(aXMLMetaData, tObjective);
        XMLGen::append_functions_to_weighted_sum_objective(aXMLMetaData, tObjectiveFunctions, tObjective);
        XMLGen::append_weights_to_weighted_sum_objective(aXMLMetaData, tObjective);
        XMLGen::append_objective_criteria_to_plato_problem(aXMLMetaData, aParentNode);
    }
    else
    {
        auto tObjective = XMLGen::append_objective_criteria_to_plato_problem(aXMLMetaData, aParentNode);
        // Change the name to "My Objective"
        tObjective.remove_attribute("name");
        XMLGen::append_attributes({"name"}, {"My Objective"}, tObjective);
    }
}
// function append_objective_criteria_to_criteria_list
/**********************************************************************************/

/**********************************************************************************/
pugi::xml_node append_constraint_criteria_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    pugi::xml_node tReturn;
    XMLGen::AppendCriterionParameters<XMLGen::Criterion> tFunctionInterface;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        auto &tCriterion = aXMLMetaData.criterion(tConstraint.criterion());
        auto tCriterionType = Plato::tolower(tCriterion.type());
        if(tCriterionType == "composite")
        {
            for(auto &tCriterionID : tCriterion.criterionIDs())
            {
                auto tSubCriterion = aXMLMetaData.criterion(tCriterionID);
                tReturn = tFunctionInterface.call(tSubCriterion, aParentNode);
            }
        }
        else
        {
            tReturn = tFunctionInterface.call(tCriterion, aParentNode);
        }
    }
    return tReturn;
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
 const std::vector<std::string> &aConstraintFunctions,
 pugi::xml_node& aParentNode)
{
    auto tFunctions = XMLGen::transform_tokens_for_plato_analyze_input_deck(aConstraintFunctions);
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
void append_constraint_criteria_to_criteria_list
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(XMLGen::is_any_constraint_computed_by_plato_analyze(aXMLMetaData) == false)
    {
        return;
    }
    auto tContraintFunctions = XMLGen::return_list_of_constraint_functions(aXMLMetaData);
    if(tContraintFunctions.size() > 1)
    {
        auto tConstraint = aParentNode.append_child("ParameterList");
        XMLGen::append_weighted_sum_constraint_to_plato_problem(aXMLMetaData, tConstraint);
        XMLGen::append_functions_to_weighted_sum_constraint(aXMLMetaData, tContraintFunctions, tConstraint);
        XMLGen::append_weights_to_weighted_sum_constraint(aXMLMetaData, tConstraint);
        XMLGen::append_constraint_criteria_to_plato_problem(aXMLMetaData, aParentNode);
    }
    else
    {
        auto tConstraint = XMLGen::append_constraint_criteria_to_plato_problem(aXMLMetaData, aParentNode);
        // Change the name to "My Constraint"
        tConstraint.remove_attribute("name");
        XMLGen::append_attributes({"name"}, {"My Constraint"}, tConstraint);
    }
}
// function append_constraint_criteria_to_criteria_list
/**********************************************************************************/

/**********************************************************************************/
void append_physics_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::AnalyzePhysicsFunctionInterface tPhysicsInterface;
    tPhysicsInterface.call(aXMLMetaData.scenario(0u), aXMLMetaData.mOutputMetaData[0], aParentNode);
}
// function append_physics_to_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
void append_spatial_model_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.blocks.empty())
    {
        THROWERR("Append Spatial Model to Plato Problem: Block container is empty.")
    }

    auto tSpatialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Spatial Model"}, tSpatialModel);
    auto tDomains = tSpatialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Domains"}, tDomains);
    for(auto& tBlock : aXMLMetaData.blocks)
    {
        auto tCurDomain = tDomains.append_child("ParameterList");
        XMLGen::append_attributes({"name"}, {std::string("Block ") + tBlock.block_id}, tCurDomain);
        std::vector<std::string> tKeys = {"name", "type", "value"};
        std::vector<std::string> tValues = {"Element Block", "string", tBlock.name};
        XMLGen::append_parameter_plus_attributes(tKeys, tValues, tCurDomain);

        auto tMaterials = aXMLMetaData.materials;
        std::vector<std::string> tMaterialIDs;
        bool tMaterialFound = false;
        XMLGen::Material tMaterial;

        for(auto tMat : tMaterials)
        {
            if(tMat.id() == tBlock.material_id)
            {
                tMaterial = tMat;
                tMaterialFound = true;
                break;
            }

        }
        if(!tMaterialFound)
        {
            THROWERR("Append Spatial Model to Plato Analyze Input Deck: Block " + tBlock.block_id +
                    " lists material with material_id " + tBlock.material_id + " but no material with ID " + tBlock.material_id + " exists")
        }

        tValues = {"Material Model", "string", tMaterial.name()};
        XMLGen::append_parameter_plus_attributes(tKeys, tValues, tCurDomain);
    }
}
// function append_spatial_model_to_plato_problem
/**********************************************************************************/
/**********************************************************************************/
void append_material_model_to_plato_problem
(const std::vector<XMLGen::Material>& aMaterials,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(aMaterials.empty())
    {
        THROWERR("Append Material Model to Plato Problem: Material container is empty.")
    }

    auto tMaterialModels = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Material Models"}, tMaterialModels);

    XMLGen::append_pressure_and_temperature_scaling_to_material_models(aXMLMetaData, tMaterialModels);

    XMLGen::AppendMaterialModelParameters tMaterialInterface;
    for(auto& tMaterial : aMaterials)
    {
        tMaterialInterface.call(tMaterial, tMaterialModels);
    }
}
// function append_material_model_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
/**********************************************************************************/
void append_pressure_and_temperature_scaling_to_material_models
(const XMLGen::InputData& aXMLMetaData,
       pugi::xml_node&    aMaterialModels)
{
    auto tScenarios = aXMLMetaData.scenarios();
    if (tScenarios.size() == 1)
    {
        std::string tPhysics = tScenarios[0].physics();
        if (tPhysics == "plasticity")
        {
            std::string tPressureScaling = tScenarios[0].pressureScaling();
            std::vector<std::string> tKeys = {"name", "type", "value"};
            std::vector<std::string> tValues = {"Pressure Scaling", "double", tPressureScaling};
            XMLGen::append_parameter_plus_attributes(tKeys, tValues, aMaterialModels);
        }
        else if (tPhysics == "thermoplasticity")
        {
            std::string tPressureScaling = tScenarios[0].pressureScaling();
            std::string tTemperatureScaling = tScenarios[0].temperatureScaling();
            
            std::vector<std::string> tKeys = {"name", "type", "value"};
            std::vector<std::string> tValuesPressureScaling = {"Pressure Scaling", "double", tPressureScaling};
            XMLGen::append_parameter_plus_attributes(tKeys, tValuesPressureScaling, aMaterialModels);

            std::vector<std::string> tValuesTemperatureScaling = {"Temperature Scaling", "double", tTemperatureScaling};
            XMLGen::append_parameter_plus_attributes(tKeys, tValuesTemperatureScaling, aMaterialModels);
        }
    }
}
// function append_pressure_and_temperature_scaling_to_material_models
/**********************************************************************************/

/**********************************************************************************/
void append_material_models_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(!aXMLMetaData.mRandomMetaData.empty() && aXMLMetaData.mRandomMetaData.materialSamplesDrawn())
    {
        auto tRandomMaterials = aXMLMetaData.mRandomMetaData.materials();
        XMLGen::append_material_model_to_plato_problem(tRandomMaterials, aXMLMetaData, aParentNode);
    }
    else
    {
        XMLGen::append_material_model_to_plato_problem(aXMLMetaData.materials, aXMLMetaData, aParentNode);
    }
}
// function append_material_models_to_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
void append_spatial_model_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::append_spatial_model_to_plato_problem(aXMLMetaData, aParentNode);
}
// function append_spatial_model_to_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
void append_loads_to_plato_problem
(const std::string &aPhysics,
 const std::vector<XMLGen::Load> &aLoads,
 std::vector<pugi::xml_node> &aParentNodes)
{
    XMLGen::AppendLoad tLoadFuncInterface;
    XMLGen::LoadTag tLoadNameFuncInterface;
    for(auto& tLoad : aLoads)
    {
        auto tName = tLoadNameFuncInterface.call(tLoad);
        pugi::xml_node tParentNode;
        XMLGen::get_load_parent_node(aPhysics, tLoad, aParentNodes, tParentNode);
        tLoadFuncInterface.call(tName, tLoad, tParentNode);
    }
}
// function append_loads_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void get_load_parent_node
(const std::string &aPhysics,
 const XMLGen::Load &aLoad,
 const std::vector<pugi::xml_node> &aParentNodes,
 pugi::xml_node &aParentNode)
{
    XMLGen::ValidPhysicsLoadCombinations tPhysicsToLoadMap;
    std::vector<std::string> tParentNames = tPhysicsToLoadMap.get_parent_load_node_names(aPhysics, aLoad.type()); 
    for(auto &tCurParentNode : aParentNodes)
    {
        if(tParentNames[tParentNames.size() - 1].compare(tCurParentNode.attribute("name").value()) == 0)
        {
            aParentNode = tCurParentNode;
            return;
        }
    } 
    THROWERR("Couldn't find valid parent node for " + aLoad.type() + " load.")
}
/**********************************************************************************/

/**********************************************************************************/
void get_scenario_list_from_objectives_and_constraints
(const XMLGen::InputData& aXMLMetaData,
 std::vector<XMLGen::Scenario>& aScenarioList)
{
    std::set<std::string> tScenarioIDs;
    for(auto &tScenarioID : aXMLMetaData.objective.scenarioIDs)
    {
        if(tScenarioIDs.find(tScenarioID) == tScenarioIDs.end())
        {
            tScenarioIDs.insert(tScenarioID);
            aScenarioList.push_back(aXMLMetaData.scenario(tScenarioID));
        }
    }
    for(auto &tConstraint : aXMLMetaData.constraints)
    {
        if(tConstraint.scenario() != "" &&
           tScenarioIDs.find(tConstraint.scenario()) == tScenarioIDs.end())
        {
            tScenarioIDs.insert(tConstraint.scenario());
            aScenarioList.push_back(aXMLMetaData.scenario(tConstraint.scenario()));
        }
    }
}
/**********************************************************************************/

/**********************************************************************************/
std::string get_essential_boundary_condition_block_title(XMLGen::Scenario &aScenario)
{
    XMLGen::ValidEssentialBoundaryConditionBlockTitleKeys tValidTitleMap;
    std::string tReturnValue = tValidTitleMap.value(aScenario.physics());
    return tReturnValue;
}
/**********************************************************************************/

/**********************************************************************************/
void append_deterministic_loads_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::vector<XMLGen::Scenario> tScenarioList;
    get_scenario_list_from_objectives_and_constraints(aXMLMetaData, tScenarioList);
    for (auto &tScenario : tScenarioList)
    {
        std::vector<pugi::xml_node> tParentNodes;
        XMLGen::create_load_parent_nodes(tScenario, aParentNode, tParentNodes);

        std::vector<XMLGen::Load> tScenarioLoads = aXMLMetaData.scenarioLoads(tScenario.id());
        XMLGen::append_loads_to_plato_problem(tScenario.physics(), tScenarioLoads, tParentNodes);
    }
}
// function append_deterministic_loads_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void create_load_parent_nodes
(const XMLGen::Scenario &aScenario,
 pugi::xml_node &aParentNode,
 std::vector<pugi::xml_node> &aParentNodes)
{
    XMLGen::ValidPhysicsLoadCombinations tPhysicsToLoadMap;
    std::set<std::vector<std::string>> tParentNames;
    tPhysicsToLoadMap.get_parent_names(aScenario.physics(), tParentNames);
    size_t tParentCounter = 0;
    pugi::xml_node tCurrentParent;
    auto tItr = tParentNames.begin();
    while(tItr != tParentNames.end())
    {
        std::vector<std::string> tCurrentNames = *tItr;
        if (tCurrentNames.size() == 1)
        {
            auto tLoadParent = aParentNode.append_child("ParameterList");
            XMLGen::append_attributes({"name"}, {tCurrentNames[0]}, tLoadParent);
            aParentNodes.push_back(tLoadParent);
        }
        else if (tCurrentNames.size() == 2)
        {
            if (tParentCounter == 0)// Need to create the first parameter list
            {
                tCurrentParent = aParentNode.append_child("ParameterList");
                XMLGen::append_attributes({"name"}, {tCurrentNames[0]}, tCurrentParent);
            }
            // Append to the parent that was previously created
            auto tLoadParent = tCurrentParent.append_child("ParameterList");
            XMLGen::append_attributes({"name"}, {tCurrentNames[1]}, tLoadParent);
            aParentNodes.push_back(tLoadParent);
        }
        else
        {
            THROWERR("Only two levels of load parent parameter lists are allowed.")
        }
        tItr++;
        tParentCounter++;
    }
}
/**********************************************************************************/

/**********************************************************************************/
void get_ebc_vector_for_scenario
(const XMLGen::InputData& aXMLMetaData,
 const XMLGen::Scenario &aScenario,
 std::vector<XMLGen::EssentialBoundaryCondition> &aEBCVector)
{
    for(auto &tbcID : aScenario.bcIDs())
    {
        for(size_t i=0; i<aXMLMetaData.ebcs.size(); ++i)
        {
            if(aXMLMetaData.ebcs[i].value("id").compare(tbcID) == 0)
            {
                aEBCVector.push_back(aXMLMetaData.ebcs[i]);
                break;
            }
        }
    }
}
/**********************************************************************************/

/**********************************************************************************/
void append_random_loads_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.mRandomMetaData.loadSamplesDrawn())
    {
        auto tRandomLoads = aXMLMetaData.mRandomMetaData.loadcase();
        auto &tScenario = aXMLMetaData.scenario(aXMLMetaData.objective.scenarioIDs[0]);
        std::vector<pugi::xml_node> tParentNodes;
        XMLGen::create_load_parent_nodes(tScenario, aParentNode, tParentNodes);
        XMLGen::append_loads_to_plato_problem(tScenario.physics(), tRandomLoads.loads, tParentNodes);
    }
    else
    {
        XMLGen::append_deterministic_loads_to_plato_problem(aXMLMetaData, aParentNode);
    }
}
// function append_random_loads_to_plato_problem
/**********************************************************************************/

/**********************************************************************************/
void append_loads_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(aXMLMetaData.mRandomMetaData.samplesDrawn())
    {
        XMLGen::append_random_loads_to_plato_problem(aXMLMetaData, aParentNode);
    }
    else
    {
        XMLGen::append_deterministic_loads_to_plato_problem(aXMLMetaData, aParentNode);
    }
}
// function append_loads_to_plato_analyze_input_deck
/**********************************************************************************/

/**********************************************************************************/
// Return map from supported essential boundary condition block name to list of 
// essential boundary conditions.
std::unordered_map<std::string, std::vector<XMLGen::EssentialBoundaryCondition>>
get_map_from_essential_bcs_block_name_to_essential_bcs_list(
    const XMLGen::InputData &tXMLMetaData,
    const XMLGen::Scenario &tScenario)
{
    XMLGen::ValidEssentialBoundaryConditionBlockNames tMap;
    std::unordered_map<std::string, std::vector<XMLGen::EssentialBoundaryCondition>> tEssBCsBlockNameToEssBCs;

    auto tPhysics = tScenario.physics();

    for (auto &tbcID : tScenario.bcIDs())
    {
        for (auto &tEBC : tXMLMetaData.ebcs)
        {
            if (tEBC.value("id") == tbcID)
            {
                auto tDofName = tEBC.degree_of_freedom();
                auto tBlockName = tMap.blockName(tPhysics, tDofName);
                tEssBCsBlockNameToEssBCs[tBlockName].push_back(tEBC);
                break;
            }
        }
    }

    return tEssBCsBlockNameToEssBCs;
}
//function get_map_from_essential_bcs_block_name_to_essential_bcs_list
/**********************************************************************************/

/**********************************************************************************/
// Append each set of essential boundary conditions to the plato analyze input deck.
void append_analyze_essential_boundary_conditions(
    const std::unordered_map<std::string, std::vector<XMLGen::EssentialBoundaryCondition>> &aMap,
    const XMLGen::Scenario &aScenario,
    pugi::xml_node &aParentNode)
{
    XMLGen::EssentialBoundaryConditionTag tTagInterface;
    XMLGen::AppendEssentialBoundaryCondition tFuncInterface;
    for (auto &tPair : aMap)
    {
        auto tEssentialBC = aParentNode.append_child("ParameterList");
        XMLGen::append_attributes({"name"}, {tPair.first}, tEssentialBC);

        for (auto &tBC : tPair.second)
        {
            auto tName = tTagInterface.call(tBC);
            tFuncInterface.call(tName, aScenario.physics(), tBC, tEssentialBC);
        }
    }
}
//function append_analyze_essential_boundary_conditions
/**********************************************************************************/

/**********************************************************************************/
void append_essential_boundary_conditions_to_plato_analyze_input_deck(
    const XMLGen::InputData& aXMLMetaData,
    pugi::xml_node& aParentNode)
{
    std::vector<XMLGen::Scenario> tScenarioList;
    XMLGen::get_scenario_list_from_objectives_and_constraints(aXMLMetaData, tScenarioList);
    for (auto &tScenario : tScenarioList)
    {
        auto tEssBCsBlockNameToEssBCs = XMLGen::get_map_from_essential_bcs_block_name_to_essential_bcs_list(aXMLMetaData, tScenario);
        XMLGen::append_analyze_essential_boundary_conditions(tEssBCsBlockNameToEssBCs, tScenario, aParentNode);
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

    std::string tServiceID = get_plato_analyze_service_id(aXMLMetaData);
    std::string tFilename = std::string("plato_analyze_") + tServiceID + "_input_deck.xml";
    tDocument.save_file(tFilename.c_str(), "  ");
}
// function write_plato_analyze_input_deck_file
/**********************************************************************************/

/**********************************************************************************/
std::string get_plato_analyze_service_id
(const XMLGen::InputData& aXMLMetaData)
{
    std::string tReturn = "";

    if(aXMLMetaData.objective.serviceIDs.size() > 0)
    {
        tReturn = aXMLMetaData.objective.serviceIDs[0];
    }
    else if(aXMLMetaData.constraints.size() > 0 &&
            aXMLMetaData.constraints[0].service().size() > 0)
    {
        tReturn = aXMLMetaData.constraints[0].service();
    }
    return tReturn;
}
// function get_plato_analyze_service_id
/**********************************************************************************/

}
// namespace XMLGen
