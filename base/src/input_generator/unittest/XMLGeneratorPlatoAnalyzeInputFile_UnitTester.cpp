/*
 * XMLGeneratorPlatoAnalyzeInputFile_UnitTester.cpp
 *
 *  Created on: Jun 8, 2020
 */

#include <gtest/gtest.h>

#include "pugixml.hpp"

#include <typeindex>

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorPlatoAnalyzeUtilities.hpp"

namespace XMLGen
{

struct ValidAnalyzePDEKeys
{
    std::unordered_map<std::string, std::string> mKeys = { { "elliptic", "Elliptic" }, { "parabolic", "Parabolic" }, { "hyperbolic", "Hyperbolic" },
        {"plasticity", "Infinite Strain Plasticity"} };
};

struct ValidAnalyzePhysicsKeys
{
    std::unordered_map<std::string, std::string> mKeys = { { "mechanical", "Mechanical" }, { "electromechanical", "Electromechanical" },
        { "thermal", "Thermal" }, {"stabilized mechanical", "Stabilized Mechanical"}, {"stabilized thermomechanical", "Stabilized Thermomechanical"},
        { "thermomechanical", "Thermomechanical" }, { "thermomechanical", "Thermomechanical" } };
};

struct ValidAnalyzeCriteriaKeys
{

    std::unordered_map<std::string, std::string> mKeys = { { "volume", "Volume" }, { "maximize stiffness", "Internal Elastic Energy" },
      { "local stress", "Stress Constraint Quadratic" }, {"stress p-norm", "Stress P-Norm"}, {"effective energy", "Effective Energy"},
      { "stress constrained mass minimization", "Stress Constraint General" }, { "maximize heat conduction", "Internal Thermal Energy" },
      { "flux p-norm", "Flux P-Norm" }, { "thermo-elastic energy", "Internal Thermoelastic Energy"}, { "electro-elastic energy", "Internal Electroelastic Energy"} };


    std::unordered_map<std::string, bool> mSelfAdjoint = { { "volume", false }, { "maximize stiffness", true }, { "local stress", false },
      { "stress p-norm", false }, { "effective energy", true }, { "stress constrained mass minimization ", false }, { "maximize heat conduction", true },
      { "flux p-norm", false }, { "thermo-elastic energy", false }, { "electro-elastic energy", false } };
};

struct ValidSpatialDimsKeys
{
    std::vector<std::string> mKeys = { "3", "2" };
};

std::string return_random_traction_load_name
(const XMLGen::Load& aLoad)
{
    std::string tOutput;
    if(aLoad.mIsRandom)
    {
        tOutput = std::string("Random Traction Vector Boundary Condition ") + aLoad.load_id;
    }
    else
    {
        tOutput = std::string("Traction Vector Boundary Condition ") + aLoad.load_id;
    }
    return tOutput;
}

std::string return_pressure_load_name
(const XMLGen::Load& aLoad)
{
    std::string tOutput;
    if(aLoad.mIsRandom)
    {
        tOutput = std::string("Random Pressure Boundary Condition ") + aLoad.load_id;
    }
    else
    {
        tOutput = std::string("Pressure Boundary Condition ") + aLoad.load_id;
    }
    return tOutput;
}

std::string return_surface_potential_load_name
(const XMLGen::Load& aLoad)
{
    std::string tOutput;
    if(aLoad.mIsRandom)
    {
        tOutput = std::string("Random Surface Potential Boundary Condition ") + aLoad.load_id;
    }
    else
    {
        tOutput = std::string("Surface Potential Boundary Condition ") + aLoad.load_id;
    }
    return tOutput;
}

std::string return_surface_flux_load_name
(const XMLGen::Load& aLoad)
{
    std::string tOutput;
    if(aLoad.mIsRandom)
    {
        tOutput = std::string("Random Surface Flux Boundary Condition ") + aLoad.load_id;
    }
    else
    {
        tOutput = std::string("Surface Flux Boundary Condition ") + aLoad.load_id;
    }
    return tOutput;
}

void append_parameter_plus_attributes
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 pugi::xml_node& aParentNode)
{
    auto tChild = aParentNode.append_child("Parameter");
    XMLGen::append_attributes(aKeys, aValues, tChild);
}

void append_problem_description_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    XMLGen::ValidSpatialDimsKeys tValidKeys;
    auto tSpatialDim = std::find(tValidKeys.mKeys.begin(), tValidKeys.mKeys.end(), aXMLMetaData.mPhysicsMetaData.mSpatialDims);
    if (tSpatialDim == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Append Problem Description to Plato Analyze Input Deck: Invalid spatial dimensions '") + tSpatialDim->c_str()
            + "'.  Only three and two dimensional problems are supported in Plato Analyze.")
    }
    auto tProblemParamList = aDocument.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Problem"}, tProblemParamList);
    std::vector<std::string> tKeys = {"Physics", "Spatial Dimension", "Input Mesh"};
    std::vector<std::string> tValues = {"Plato Driver", tSpatialDim->c_str(), aXMLMetaData.run_mesh_name};
    XMLGen::append_children(tKeys, tValues, tProblemParamList);
}

void append_physics_parameter_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::ValidAnalyzePhysicsKeys tValidKeys;
    auto tPhysics = tValidKeys.mKeys.find(Plato::tolower(aXMLMetaData.mPhysicsMetaData.mCategory));
    if (tPhysics == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Append Physics to Plato Analyze Input Deck: Physics category '") + tPhysics->first + "' is not supported.")
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Physics", "string", tPhysics->second};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

void append_pde_constraint_parameter_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::ValidAnalyzePDEKeys tValidKeys;
    auto tPDE = tValidKeys.mKeys.find(Plato::tolower(aXMLMetaData.mPhysicsMetaData.mPDE));
    if (tPDE == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Append PDE Constraint to Plato Analyze Input Deck: PDE category '")
            + aXMLMetaData.mPhysicsMetaData.mPDE + "' is not supported.")
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"PDE Constraint", "string", tPDE->second};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

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

void append_objective_parameter_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    if(XMLGen::is_any_constraint_computed_by_plato_analyze(aXMLMetaData) == false)
    {
        return;
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Objective", "string", "My Objective"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

void append_self_adjoint_parameter_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    std::string tIsSelfAdjoint = "false";
    if(aXMLMetaData.objectives.size() == 1u)
    {
        XMLGen::ValidAnalyzeCriteriaKeys tValidKeys;
        auto tItr = tValidKeys.mSelfAdjoint.find(Plato::tolower(aXMLMetaData.objectives[0].type));
        if (tItr == tValidKeys.mSelfAdjoint.end())
        {
            THROWERR(std::string("Append Self Adjoint Parameter to Plato Analyze Input Deck: Objective '")
                + aXMLMetaData.objectives[0].type + "' is not supported.")
        }
        tIsSelfAdjoint = tItr->second ? "true" : "false";
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Self-Adjoint", "bool", tIsSelfAdjoint};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

void append_plato_problem_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tProblem = aDocument.child("ParameterList");
    auto tPlatoProblem = tProblem.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Plato Problem"}, tPlatoProblem);
    XMLGen::append_physics_parameter_to_plato_problem(aXMLMetaData, tPlatoProblem);
    XMLGen::append_pde_constraint_parameter_to_plato_problem(aXMLMetaData, tPlatoProblem);
    XMLGen::append_constraint_parameter_to_plato_problem(aXMLMetaData, tPlatoProblem);
    XMLGen::append_objective_parameter_to_plato_problem(aXMLMetaData, tPlatoProblem);
    XMLGen::append_self_adjoint_parameter_to_plato_problem(aXMLMetaData, tPlatoProblem);
}

void append_weighted_sum_objective_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::append_attributes({"name"}, {"My Objective"}, aParentNode);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "Weighted Sum"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

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

std::vector<std::string> get_list_of_objective_functions
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

std::vector<std::string> get_list_of_objective_weights
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

void append_functions_to_weighted_sum_objective
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tTokens = XMLGen::get_list_of_objective_functions(aXMLMetaData);
    auto tFunctions = XMLGen::transform_tokens_for_plato_analyze_input_deck(tTokens);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Functions", "Array(string)", tFunctions};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

void append_weights_to_weighted_sum_objective
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tTokens = XMLGen::get_list_of_objective_weights(aXMLMetaData);
    auto tWeights = XMLGen::transform_tokens_for_plato_analyze_input_deck(tTokens);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Weights", "Array(string)", tWeights};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

template<typename Criterion>
std::string is_criterion_supported_in_plato_analyze
(const Criterion& aCriterion)
{
    XMLGen::ValidAnalyzeCriteriaKeys tValidKeys;
    auto tItr = tValidKeys.mKeys.find(Plato::tolower(aCriterion.type));
    if (tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Is Criterion Supported in Plato Analyze: Criterion '") + aCriterion.type + "' is not supported.")
    }
    return tItr->second;
}

template<typename Criterion>
void append_simp_penalty_function
(const Criterion& aCriterion,
 pugi::xml_node& aParentNode)
{
    auto tPenaltyFunction = aParentNode.append_child("Penalty Function");
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "SIMP"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tPenaltyFunction);
    tValues = {"Exponent", "double", aCriterion.mPenaltyParam};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tPenaltyFunction);
    tValues = {"Minimum Value", "double", aCriterion.mMinimumErsatzValue};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tPenaltyFunction);
}

template<typename Criterion>
void append_scalar_function_criterion
(const Criterion& aCriterion,
 pugi::xml_node& aParentNode)
{
    auto tDesignCriterionName = XMLGen::is_criterion_supported_in_plato_analyze(aCriterion);

    auto tName = std::string("my ") + Plato::tolower(aCriterion.type);
    auto tObjective = aParentNode.append_child("ParameterList");
    std::vector<std::string> tKeys = {"name"};
    std::vector<std::string> tValues = {tName};
    XMLGen::append_attributes(tKeys, tValues, tObjective);

    tKeys = {"name", "type", "value"}; tValues = {"Type", "string", "Scalar Function"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tObjective);
    tValues = {"Scalar Function Type", "string", tDesignCriterionName};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tObjective);
    XMLGen::append_simp_penalty_function(aCriterion, tObjective);
}

template<typename Criterion>
void append_pnorm_criterion
(const Criterion& aCriterion,
 pugi::xml_node& aParentNode)
{
    XMLGen::append_scalar_function_criterion(aCriterion, aParentNode);
    auto tCriterion = aParentNode.child("ParameterList");
    if(tCriterion.empty())
    {
        THROWERR("Append P-Norm Criterion: Criterion parameter list is empty. Most likely, "
            + "there was an error appending the scalar function criterion.")
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Exponent", "double", aCriterion.mPnormExponent};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tCriterion);
}

namespace Analyze
{

/*!< material function pointer type */
typedef void (*MaterialFunction)(void);

/*!< criterion function pointer type */
typedef void (*CriterionFunction)(void);

/*!< map from material model category to material function used to append material properties and \n
 * respective values, i.e. map<material_model, material_function> */
typedef std::unordered_map<std::string, MaterialFunction> MaterialFunctionMap;

/*!< map from design criterion category to design criterion function used to append design \n
 * criterion and respective parameters, i.e. map<design_criterion_category, criterion_function> */
typedef std::unordered_map<std::string, CriterionFunction> CriterionFunctionMap;
}

/******************************************************************************//**
 * \struct The goal of this C++ struct is to provide an interface for the \n
 * functions used to append design criteria to plato_analyze_input_deck.xml file. \n
 * This interface reduces cyclomatic complexity due to having multiple design \n
 * criterion implemented in Plato Analyze.
**********************************************************************************/
template<typename CriterionType>
struct CriterionFunctionInterface
{
private:
    /*!< map from design criterion category to function used to append design criterion and respective parameters */
    std::unordered_map<std::string, std::pair<XMLGen::Analyze::CriterionFunction, std::type_index>> mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert functions used to append design criterion parameters to design \n
     *   criterion function map.
     **********************************************************************************/
    void insert()
    {
        // volume
        auto tFuncIndex = std::type_index(typeid(append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("volume",
          std::make_pair((XMLGen::Analyze::CriterionFunction)append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // maximize stiffness
        tFuncIndex = std::type_index(typeid(append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("maximize stiffness",
          std::make_pair((XMLGen::Analyze::CriterionFunction)append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // stress p-norm
        tFuncIndex = std::type_index(typeid(append_pnorm_criterion<CriterionType>));
        mMap.insert(std::make_pair("stress p-norm",
          std::make_pair((XMLGen::Analyze::CriterionFunction)append_pnorm_criterion<CriterionType>, tFuncIndex)));

        // effective energy
        tFuncIndex = std::type_index(typeid(append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("effective energy",
          std::make_pair((XMLGen::Analyze::CriterionFunction)append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // maximize heat conduction
        tFuncIndex = std::type_index(typeid(append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("maximize heat conduction",
          std::make_pair((XMLGen::Analyze::CriterionFunction)append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // thermo-elastic energy
        tFuncIndex = std::type_index(typeid(append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("thermo-elastic energy",
          std::make_pair((XMLGen::Analyze::CriterionFunction)append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // electro-elastic energy
        tFuncIndex = std::type_index(typeid(append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("electro-elastic energy",
          std::make_pair((XMLGen::Analyze::CriterionFunction)append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // flux p-norm
        tFuncIndex = std::type_index(typeid(append_pnorm_criterion<CriterionType>));
        mMap.insert(std::make_pair("flux p-norm",
          std::make_pair((XMLGen::Analyze::CriterionFunction)append_pnorm_criterion<CriterionType>, tFuncIndex)));
    }

public:
    /******************************************************************************//**
     * \fn CriterionFunctionInterface
     * \brief Default constructor
    **********************************************************************************/
    CriterionFunctionInterface()
    {
        this->insert();
    }

    /******************************************************************************//**
     * \fn call
     * \tparam CriterionType criterion function metadata C++ structure type
     * \brief Append criterion function parameters to plato_analyze_input_deck.xml file.
     * \param [in]     Criterion    criterion metadata
     * \param [in/out] aParentNode  pugi::xml_node
    **********************************************************************************/
    void call
    (const CriterionType& aCriterion,
     pugi::xml_node &aParentNode)
    {
        if(Plato::tolower(aCriterion.mPerformerName).compare("plato_analyze") != 0)
        {
            return;
        }

        auto tLowerFuncLabel = Plato::tolower(aCriterion.type);
        auto tMapItr = mMap.find(tLowerFuncLabel);
        if(tMapItr == mMap.end())
        {
            THROWERR(std::string("Criterion Function Interface: Did not find criterion function with tag '") + tLowerFuncLabel + "' in list.")
        }
        auto tTypeCastedFunc = reinterpret_cast<void(*)(const CriterionType&, pugi::xml_node&)>(tMapItr->second.first);
        if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
        {
            THROWERR(std::string("Criterion Function Interface: Reinterpret cast for criterion function with tag '") + tLowerFuncLabel + "' failed.")
        }
        tTypeCastedFunc(aCriterion, aParentNode);
    }
};
// struct CriterionFunctionInterface

void append_objective_criteria_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::CriterionFunctionInterface<XMLGen::Objective> tFunctionInterface;
    for(auto& tObjective : aXMLMetaData.objectives)
    {
        tFunctionInterface.call(tObjective, aParentNode);
    }
}

void append_objective_criteria_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tProblem = aDocument.child("ParameterList");
    auto tPlatoProblem = tProblem.child("ParameterList");
    auto tObjective = tPlatoProblem.append_child("ParameterList");
    XMLGen::append_weighted_sum_objective_to_plato_problem(aXMLMetaData, tObjective);
    XMLGen::append_functions_to_weighted_sum_objective(aXMLMetaData, tObjective);
    XMLGen::append_weights_to_weighted_sum_objective(aXMLMetaData, tObjective);
    XMLGen::append_objective_criteria_to_plato_problem(aXMLMetaData, tPlatoProblem);
}

void append_constraint_criteria_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::CriterionFunctionInterface<XMLGen::Constraint> tFunctionInterface;
    for(auto& tConstraint : aXMLMetaData.constraints)
    {
        tFunctionInterface.call(tConstraint, aParentNode);
    }
}

void append_weighted_sum_constraint_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    XMLGen::append_attributes({"name"}, {"My Constraint"}, aParentNode);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "Weighted Sum"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

std::vector<std::string> get_list_of_constraint_functions
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

std::vector<std::string> get_list_of_constraint_weights
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

void append_functions_to_weighted_sum_constraint
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tTokens = XMLGen::get_list_of_constraint_functions(aXMLMetaData);
    auto tFunctions = XMLGen::transform_tokens_for_plato_analyze_input_deck(tTokens);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Functions", "Array(string)", tFunctions};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

void append_weights_to_weighted_sum_constraint
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode)
{
    auto tTokens = XMLGen::get_list_of_constraint_weights(aXMLMetaData);
    auto tWeights = XMLGen::transform_tokens_for_plato_analyze_input_deck(tTokens);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Weights", "Array(string)", tWeights};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

void append_constraint_criteria_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tProblem = aDocument.child("ParameterList");
    auto tPlatoProblem = tProblem.child("ParameterList");
    auto tObjective = tPlatoProblem.append_child("ParameterList");
    XMLGen::append_weighted_sum_constraint_to_plato_problem(aXMLMetaData, tObjective);
    XMLGen::append_functions_to_weighted_sum_constraint(aXMLMetaData, tObjective);
    XMLGen::append_weights_to_weighted_sum_constraint(aXMLMetaData, tObjective);
    XMLGen::append_constraint_criteria_to_plato_problem(aXMLMetaData, tPlatoProblem);
}

void append_partial_differential_equation_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tProblem = aDocument.child("ParameterList");
    auto tPlatoProblem = tProblem.child("ParameterList");
    auto tPhysics = tPlatoProblem.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Elliptic"}, tPhysics);
    XMLGen::append_simp_penalty_function(aXMLMetaData.mPhysicsMetaData, tPhysics);
}

void append_material_properties_to_plato_analyze_material_model
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tTags = aMaterial.tags();
    XMLGen::ValidAnalyzeMaterialPropertyKeys tValidKeys;
    std::vector<std::string> tKeys = {"name", "type", "value"};

    for(auto& tTag : tTags)
    {
        auto tItr = tValidKeys.mKeys.find(Plato::tolower(tTag));
        if (tItr == tValidKeys.mKeys.end())
        {
            THROWERR(std::string("Append Material Properties To Plato Analyze Material Model: Material property '") + tTag + "' is not supported.")
        }
        auto tMaterialTag = tItr->second.first;
        auto tValueType = tItr->second.second;
        std::vector<std::string> tValues = {tMaterialTag, tValueType, aMaterial.property(tTag)};
        XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
    }
}

void append_isotropic_linear_elastic_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Material Model"}, tMaterialModel);
    auto tIsotropicLinearElasticMaterial = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Isotropic Linear Elastic"}, tIsotropicLinearElasticMaterial);
    XMLGen::append_material_properties_to_plato_analyze_material_model(aMaterial, tIsotropicLinearElasticMaterial);
}

void append_isotropic_linear_thermal_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Material Model"}, tMaterialModel);
    auto tIsotropicLinearElasticMaterial = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Isotropic Linear Thermal"}, tIsotropicLinearElasticMaterial);
    XMLGen::append_material_properties_to_plato_analyze_material_model(aMaterial, tIsotropicLinearElasticMaterial);
}

void append_isotropic_linear_thermoelastic_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Material Model"}, tMaterialModel);
    auto tIsotropicLinearElasticMaterial = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Isotropic Linear Thermoelastic"}, tIsotropicLinearElasticMaterial);
    XMLGen::append_material_properties_to_plato_analyze_material_model(aMaterial, tIsotropicLinearElasticMaterial);
}

void append_isotropic_linear_electroelastic_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Material Model"}, tMaterialModel);
    auto tIsotropicLinearElasticMaterial = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Isotropic Linear Electroelastic"}, tIsotropicLinearElasticMaterial);
    XMLGen::append_material_properties_to_plato_analyze_material_model(aMaterial, tIsotropicLinearElasticMaterial);
}

void append_orthotropic_linear_elastic_material_to_plato_problem
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    auto tMaterialModel = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Material Model"}, tMaterialModel);
    auto tIsotropicLinearElasticMaterial = tMaterialModel.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Orthotropic Linear Elastic"}, tIsotropicLinearElasticMaterial);
    XMLGen::append_material_properties_to_plato_analyze_material_model(aMaterial, tIsotropicLinearElasticMaterial);
}

/******************************************************************************//**
 * \struct The goal of this C++ struct is to provide an interface for the \n
 * functions used to append material models to plato_analyze_input_deck.xml file. \n
 * This interface reduces cyclomatic complexity due to having multiple material \n
 * models implemented in Plato Analyze.
**********************************************************************************/
struct MaterialFunctionInterface
{
private:
    /*!< map from material model category to function used to append material properties and respective values */
    std::unordered_map<std::string, std::pair<XMLGen::Analyze::MaterialFunction, std::type_index>> mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert material functions to material model function map.
     **********************************************************************************/
    void insert()
    {
        // orthotropic linear elastic material
        auto tFuncIndex = std::type_index(typeid(append_orthotropic_linear_elastic_material_to_plato_problem));
        mMap.insert(std::make_pair("volume",
          std::make_pair((XMLGen::Analyze::MaterialFunction)append_orthotropic_linear_elastic_material_to_plato_problem, tFuncIndex)));

        // isotropic linear electroelastic material
        tFuncIndex = std::type_index(typeid(append_isotropic_linear_electroelastic_material_to_plato_problem));
        mMap.insert(std::make_pair("volume",
          std::make_pair((XMLGen::Analyze::MaterialFunction)append_isotropic_linear_electroelastic_material_to_plato_problem, tFuncIndex)));

        // isotropic linear thermoelastic material
        tFuncIndex = std::type_index(typeid(append_isotropic_linear_thermoelastic_material_to_plato_problem));
        mMap.insert(std::make_pair("volume",
          std::make_pair((XMLGen::Analyze::MaterialFunction)append_isotropic_linear_thermoelastic_material_to_plato_problem, tFuncIndex)));

        // isotropic linear thermal material
        tFuncIndex = std::type_index(typeid(append_isotropic_linear_thermal_material_to_plato_problem));
        mMap.insert(std::make_pair("volume",
          std::make_pair((XMLGen::Analyze::MaterialFunction)append_isotropic_linear_thermal_material_to_plato_problem, tFuncIndex)));

        // isotropic linear elastic material
        tFuncIndex = std::type_index(typeid(append_isotropic_linear_elastic_material_to_plato_problem));
        mMap.insert(std::make_pair("volume",
          std::make_pair((XMLGen::Analyze::MaterialFunction)append_isotropic_linear_elastic_material_to_plato_problem, tFuncIndex)));
    }

public:
    /******************************************************************************//**
     * \fn MaterialFunctionInterface
     * \brief Default constructor
    **********************************************************************************/
    MaterialFunctionInterface()
    {
        this->insert();
    }

    /******************************************************************************//**
     * \fn call
     * \brief Append material model and corresponding parameters to plato_analyze_input_deck.xml file.
     * \param [in]     aMaterial    material model metadata
     * \param [in/out] aParentNode  pugi::xml_node
    **********************************************************************************/
    void call
    (const XMLGen::Material& aMaterial,
     pugi::xml_node &aParentNode)
    {
        auto tCategory = Plato::tolower(aMaterial.category());
        if(Plato::tolower(tCategory).compare("plato_analyze") != 0)
        {
            return;
        }

        auto tMapItr = mMap.find(tCategory);
        if(tMapItr == mMap.end())
        {
            THROWERR(std::string("Material Function Interface: Did not find material function with tag '") + tCategory + "' in list.")
        }
        auto tTypeCastedFunc = reinterpret_cast<void(*)(const XMLGen::Material&, pugi::xml_node&)>(tMapItr->second.first);
        if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
        {
            THROWERR(std::string("Material Function Interface: Reinterpret cast for material function with tag '") + tCategory + "' failed.")
        }
        tTypeCastedFunc(aMaterial, aParentNode);
    }
};
// struct MaterialFunctionInterface

void append_material_model_to_plato_problem
(const std::vector<XMLGen::Material>& aMaterials,
 pugi::xml_document& aDocument)
{
    auto tProblem = aDocument.child("ParameterList");
    auto tPlatoProblem = tProblem.child("ParameterList");
    XMLGen::MaterialFunctionInterface tMaterialInterface;
    for(auto& tMaterial : aMaterials)
    {
        tMaterialInterface.call(tMaterial, tPlatoProblem);
    }
}

void append_material_model_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsRandomUseCase = aXMLMetaData.mRandomMetaData.samples().empty() ? false : true;
    if(tIsRandomUseCase)
    {
        auto tRandomMaterials = aXMLMetaData.mRandomMetaData.materials();
        XMLGen::append_material_model_to_plato_problem(tRandomMaterials, aDocument);
    }
    else
    {
        XMLGen::append_material_model_to_plato_problem(aXMLMetaData.materials, aDocument);
    }
}

void append_traction_load_to_plato_problem
(const XMLGen::Load& aLoad,
 pugi::xml_node &aParentNode)
{
    auto tLoad = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Natural Boundary Conditions"}, tLoad);
    auto tTraction = tLoad.append_child("ParameterList");
    // TODO: MATCH LOAD NAMES IN PLATO_ANALYZE_OPERATION.XML FILE IN HERE,
    // E.G. "Random Traction Vector Boundary Condition 1",
    // SEE XMLGeneratorPlatoAnalyzeOperationsFileUtilities.cpp LINE 284
    //XMLGen::append_attributes({"name"}, {"Isotropic Linear Electroelastic"}, tTraction);
}

void append_natural_boundary_conditions_to_plato_problem
(const XMLGen::LoadCase& aLoadCase,
 pugi::xml_document& aDocument)
{
    auto tProblem = aDocument.child("ParameterList");
    auto tPlatoProblem = tProblem.child("ParameterList");
    for(auto& tLoad : aLoadCase.loads)
    {
        // TODO: FINISH
    }
}

void append_natural_boundary_conditions_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument)
{
    auto tIsRandomUseCase = aXMLMetaData.mRandomMetaData.samples().empty() ? false : true;
    if(tIsRandomUseCase)
    {
        auto tRandomLoads = aXMLMetaData.mRandomMetaData.loadcase();
    }
    else
    {
    }
}

void write_plato_analyze_input_deck_file
(const XMLGen::InputData& aXMLMetaData)
{
    pugi::xml_document tDocument;
    XMLGen::append_problem_description_to_plato_analyze_input_deck(aXMLMetaData, tDocument);
    XMLGen::append_plato_problem_to_plato_analyze_input_deck(aXMLMetaData, tDocument);
    XMLGen::append_objective_criteria_to_plato_analyze_input_deck(aXMLMetaData, tDocument);
    XMLGen::append_constraint_criteria_to_plato_analyze_input_deck(aXMLMetaData, tDocument);
    XMLGen::append_partial_differential_equation_to_plato_analyze_input_deck(aXMLMetaData, tDocument);
    XMLGen::append_material_model_to_plato_analyze_input_deck(aXMLMetaData, tDocument);
}

}

namespace Plato
{

}
// namespace Plato
