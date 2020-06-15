/*
 * XMLGeneratorPlatoAnalyzeInputFile_UnitTester.cpp
 *
 *  Created on: Jun 8, 2020
 */

#include <gtest/gtest.h>

#include "XMLGenerator_UnitTester_Tools.hpp"

#include "pugixml.hpp"

#include <typeindex>

#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorPlatoAnalyzeUtilities.hpp"

namespace XMLGen
{

namespace Analyze
{

/*!< criterion function pointer type */
typedef void (*CriterionFunc)(void);

/*!< material function pointer type */
typedef void (*MaterialModelFunc)(void);

/*!< natural boundary condition function pointer type */
typedef void (*NaturalBCFunc)(void);

/*!< define essential boundary condition function pointer type */
typedef void (*EssentialBCFunc)(void);

/*!< define natural boundary condition tag function pointer type */
typedef std::string (*NaturalBCTagFunc)(void);

/*!< define essential boundary condition tag function pointer type */
typedef std::string (*EssentialBCTagFunc)(void);

/*!< map from design criterion category to design criterion function used to append design \n
 * criterion and respective parameters, i.e. map<design_criterion_category, criterion_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::CriterionFunc, std::type_index>> CriterionFuncMap;

/*!< map from material model category to material function used to append material properties and \n
 * respective values, i.e. map<material_model, material_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::MaterialModelFunc, std::type_index>> MaterialModelFuncMap;

/*!< map from natural boundary condition (bc) category to natural bc function used to append, \n
 * natural bc parameters, i.e. map<natural_bc_category, append_natural_bc_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::NaturalBCFunc, std::type_index>> NaturalBCFuncMap;

/*!< map from natural boundary condition (bc) category to natural bc function used to define \n
 * the natural bcs tags, i.e. map<natural_bc_category, define_natural_bc_tag_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::NaturalBCTagFunc, std::type_index>> NaturalBCTagFuncMap;

/*!< map from essential boundary condition (bc) category to essential bc function used to append, \n
 * essential bc parameters, i.e. map<essential_bc_category, append_essential_bc_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::EssentialBCFunc, std::type_index>> EssentialBCFuncMap;

/*!< map from essential boundary condition (bc) category to essential bc function used to define \n
 * the essential bcs tags, i.e. map<essential_bc_category, define_essential_bc_tag_function> */
typedef std::unordered_map<std::string, std::pair<XMLGen::Analyze::EssentialBCTagFunc, std::type_index>> EssentialBCTagFuncMap;

}

std::string return_traction_load_name
(const XMLGen::Load& aLoad)
{
    std::string tOutput;
    if(aLoad.mIsRandom)
    {
        tOutput = std::string("Random Traction Vector Boundary Condition with ID ") + aLoad.load_id;
    }
    else
    {
        tOutput = std::string("Traction Vector Boundary Condition with ID ") + aLoad.load_id;
    }
    return tOutput;
}

std::string return_pressure_load_name
(const XMLGen::Load& aLoad)
{
    std::string tOutput;
    if(aLoad.mIsRandom)
    {
        tOutput = std::string("Random Uniform Pressure Boundary Condition with ID ") + aLoad.load_id;
    }
    else
    {
        tOutput = std::string("Uniform Pressure Boundary Condition with ID ") + aLoad.load_id;
    }
    return tOutput;
}

std::string return_surface_potential_load_name
(const XMLGen::Load& aLoad)
{
    std::string tOutput;
    if(aLoad.mIsRandom)
    {
        tOutput = std::string("Random Uniform Surface Potential Boundary Condition with ID ") + aLoad.load_id;
    }
    else
    {
        tOutput = std::string("Uniform Surface Potential Boundary Condition with ID ") + aLoad.load_id;
    }
    return tOutput;
}

std::string return_surface_flux_load_name
(const XMLGen::Load& aLoad)
{
    std::string tOutput;
    if(aLoad.mIsRandom)
    {
        tOutput = std::string("Random Uniform Surface Flux Boundary Condition with ID ") + aLoad.load_id;
    }
    else
    {
        tOutput = std::string("Uniform Surface Flux Boundary Condition with ID ") + aLoad.load_id;
    }
    return tOutput;
}

/******************************************************************************//**
 * \struct The goal of this C++ struct is to provide an interface for the \n
 * functions used to define natural boundary conditions' tags for \n
 * plato_analyze_input_deck.xml. This interface reduces cyclomatic complexity \n
 * due to multiple natural boundary conditions categories in Plato Analyze.
**********************************************************************************/
struct NaturalBoundaryConditionTag
{
private:
    /*!< map from natural boundary condition category to function used to define its tag */
    XMLGen::Analyze::NaturalBCTagFuncMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert functions to natural boundary condition functions map.
     **********************************************************************************/
    void insert()
    {
        // traction load
        auto tFuncIndex = std::type_index(typeid(return_traction_load_name));
        mMap.insert(std::make_pair("traction",
          std::make_pair((XMLGen::Analyze::NaturalBCTagFunc)return_traction_load_name, tFuncIndex)));

        // uniform pressure load
        tFuncIndex = std::type_index(typeid(return_pressure_load_name));
        mMap.insert(std::make_pair("uniform pressure",
          std::make_pair((XMLGen::Analyze::NaturalBCTagFunc)return_pressure_load_name, tFuncIndex)));

        // uniform surface potential
        tFuncIndex = std::type_index(typeid(return_surface_potential_load_name));
        mMap.insert(std::make_pair("uniform surface potential",
          std::make_pair((XMLGen::Analyze::NaturalBCTagFunc)return_surface_potential_load_name, tFuncIndex)));

        // uniform surface flux
        tFuncIndex = std::type_index(typeid(return_surface_flux_load_name));
        mMap.insert(std::make_pair("uniform surface flux",
          std::make_pair((XMLGen::Analyze::NaturalBCTagFunc)return_surface_flux_load_name, tFuncIndex)));
    }

public:
    /******************************************************************************//**
     * \fn NaturalBoundaryConditionTag
     * \brief Default constructor
    **********************************************************************************/
    NaturalBoundaryConditionTag()
    {
        this->insert();
    }

    /******************************************************************************//**
     * \fn call
     * \brief Return natural boundary condition name for plato_analyze_input_deck.xml.
     * \param [in] aLoad natural boundary condition metadata
     * \return natural boundary condition name
    **********************************************************************************/
    std::string call(const XMLGen::Load& aLoad) const
    {
        auto tCategory = Plato::tolower(aLoad.type);
        auto tMapItr = mMap.find(tCategory);
        if(tMapItr == mMap.end())
        {
            THROWERR(std::string("Natural Boundary Condition Name Function Interface: Did not find natural boundary condition function with tag '")
                + tCategory + "' in list.")
        }
        auto tTypeCastedFunc = reinterpret_cast<std::string(*)(const XMLGen::Load&)>(tMapItr->second.first);
        if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
        {
            THROWERR(std::string("Natural Boundary Condition Name Function Interface: Reinterpret cast for natural boundary condition function with tag '")
                + tCategory + "' failed.")
        }
        auto tName = tTypeCastedFunc(aLoad);
        return tName;
    }
};
// struct NaturalBoundaryConditionTag

void append_parameter_plus_attributes
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 pugi::xml_node& aParentNode)
{
    auto tChild = aParentNode.append_child("Parameter");
    XMLGen::append_attributes(aKeys, aValues, tChild);
}

void check_input_mesh_file_keyword
(const XMLGen::InputData& aXMLMetaData)
{
    if (aXMLMetaData.run_mesh_name.empty())
    {
        THROWERR("Check Input Mesh File Keyword: Input mesh filename is empty.")
    }
}

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
    if(XMLGen::is_any_objective_computed_by_plato_analyze(aXMLMetaData) == false)
    {
        return;
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Objective", "string", "My Objective"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

void check_objective_container_is_defined
(const XMLGen::InputData& aXMLMetaData)
{
    if(aXMLMetaData.objectives.empty())
    {
        THROWERR("Check Objective is Defined: Objective container is empty.")
    }
}

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
            THROWERR(std::string("Append Self Adjoint Parameter to Plato Analyze Input Deck: Criterion '") + tLowerCriterion + "' is not supported.")
        }
        tIsSelfAdjoint = tItr->second.second ? "true" : "false";
    }
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Self-Adjoint", "bool", tIsSelfAdjoint};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

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
    std::vector<std::string> tValues = {"Weights", "Array(double)", tWeights};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

template<typename Criterion>
std::string is_criterion_supported_in_plato_analyze
(const Criterion& aCriterion)
{
    XMLGen::ValidAnalyzeCriteriaKeys tValidKeys;
    auto tLowerCriterion = Plato::tolower(aCriterion.type);
    auto tItr = tValidKeys.mKeys.find(tLowerCriterion);
    if (tItr == tValidKeys.mKeys.end())
    {
        THROWERR(std::string("Is Criterion Supported in Plato Analyze: Criterion '") + tLowerCriterion + "' is not supported.")
    }
    return tItr->second.first;
}

template<typename MetaData>
void append_simp_penalty_function
(const MetaData& aMetadata,
 pugi::xml_node& aParentNode)
{
    auto tPenaltyFunction = aParentNode.append_child("Penalty Function");
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "SIMP"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tPenaltyFunction);
    tValues = {"Exponent", "double", aMetadata.mPenaltyParam};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tPenaltyFunction);
    tValues = {"Minimum Value", "double", aMetadata.mMinimumErsatzValue};
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

/******************************************************************************//**
 * \struct The goal of this C++ struct is to provide an interface for the \n
 * functions used to append design criteria paraemters to plato_analyze_input_deck.xml. \n
 * This interface reduces cyclomatic complexity due to having multiple design \n
 * criteria in Plato Analyze.
**********************************************************************************/
template<typename CriterionType>
struct AppendCriterionParameters
{
private:
    /*!< map from design criterion category to function used to append design criterion and respective parameters */
    XMLGen::Analyze::CriterionFuncMap mMap;

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
          std::make_pair((XMLGen::Analyze::CriterionFunc)append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // maximize stiffness
        tFuncIndex = std::type_index(typeid(append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("maximize stiffness",
          std::make_pair((XMLGen::Analyze::CriterionFunc)append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // stress p-norm
        tFuncIndex = std::type_index(typeid(append_pnorm_criterion<CriterionType>));
        mMap.insert(std::make_pair("stress p-norm",
          std::make_pair((XMLGen::Analyze::CriterionFunc)append_pnorm_criterion<CriterionType>, tFuncIndex)));

        // effective energy
        tFuncIndex = std::type_index(typeid(append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("effective energy",
          std::make_pair((XMLGen::Analyze::CriterionFunc)append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // maximize heat conduction
        tFuncIndex = std::type_index(typeid(append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("maximize heat conduction",
          std::make_pair((XMLGen::Analyze::CriterionFunc)append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // thermo-elastic energy
        tFuncIndex = std::type_index(typeid(append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("thermo-elastic energy",
          std::make_pair((XMLGen::Analyze::CriterionFunc)append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // electro-elastic energy
        tFuncIndex = std::type_index(typeid(append_scalar_function_criterion<CriterionType>));
        mMap.insert(std::make_pair("electro-elastic energy",
          std::make_pair((XMLGen::Analyze::CriterionFunc)append_scalar_function_criterion<CriterionType>, tFuncIndex)));

        // flux p-norm
        tFuncIndex = std::type_index(typeid(append_pnorm_criterion<CriterionType>));
        mMap.insert(std::make_pair("flux p-norm",
          std::make_pair((XMLGen::Analyze::CriterionFunc)append_pnorm_criterion<CriterionType>, tFuncIndex)));
    }

public:
    /******************************************************************************//**
     * \fn AppendCriterionParameters
     * \brief Default constructor
    **********************************************************************************/
    AppendCriterionParameters()
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
    void call(const CriterionType& aCriterion, pugi::xml_node &aParentNode) const
    {
        auto tLowerPerformer = Plato::tolower(aCriterion.mPerformerName);
        if(tLowerPerformer.compare("plato_analyze") != 0)
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
// struct AppendCriterionParameters

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
    std::vector<std::string> tValues = {"Weights", "Array(double)", tWeights};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, aParentNode);
}

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
    XMLGen::append_simp_penalty_function(aXMLMetaData.mPhysicsMetaData, tPhysics);
}

void check_material_property_tags_container
(const XMLGen::Material& aMaterial)
{
    auto tTags = aMaterial.tags();
    if(tTags.empty())
    {
        THROWERR("Check Material Property Tags Container: Material tags container is empty, "
            + "i.e. material properties are not defined.")
    }
}

void append_material_properties_to_plato_analyze_material_model
(const XMLGen::Material& aMaterial,
 pugi::xml_node& aParentNode)
{
    XMLGen::check_material_property_tags_container(aMaterial);
    XMLGen::ValidAnalyzeMaterialPropertyKeys tValidKeys;
    auto tLowerMaterialModel = Plato::tolower(aMaterial.category());
    auto tMaterialModelItr = tValidKeys.mKeys.find(tLowerMaterialModel);
    if(tMaterialModelItr == tValidKeys.mKeys.end())
    {
        THROWERR("Append Material Properties To Plato Analyze Material Model: Material model '"
            + tLowerMaterialModel + "' is not supported in Plato Analyze.")
    }

    auto tTags = aMaterial.tags();
    std::vector<std::string> tKeys = {"name", "type", "value"};
    for(auto& tTag : tTags)
    {
        auto tLowerTag = Plato::tolower(tTag);
        auto tItr = tMaterialModelItr->second.find(tLowerTag);
        if (tItr == tMaterialModelItr->second.end())
        {
            THROWERR(std::string("Append Material Properties To Plato Analyze Material Model: Material property with tag '")
                + tTag + "' is not supported in Plato Analyze by material model '" + tLowerMaterialModel + "'.")
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
 * functions used to append material models to plato_analyze_input_deck.xml. \n
 * This interface reduces cyclomatic complexity due to having multiple material \n
 * models in Plato Analyze.
**********************************************************************************/
struct AppendMaterialModelParameters
{
private:
    /*!< map from material model category to function used to append material properties and respective properties */
    XMLGen::Analyze::MaterialModelFuncMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert material functions to material model function map.
     **********************************************************************************/
    void insert()
    {
        // orthotropic linear elastic material
        auto tFuncIndex = std::type_index(typeid(append_orthotropic_linear_elastic_material_to_plato_problem));
        mMap.insert(std::make_pair("orthotropic linear elastic",
          std::make_pair((XMLGen::Analyze::MaterialModelFunc)append_orthotropic_linear_elastic_material_to_plato_problem, tFuncIndex)));

        // isotropic linear electroelastic material
        tFuncIndex = std::type_index(typeid(append_isotropic_linear_electroelastic_material_to_plato_problem));
        mMap.insert(std::make_pair("isotropic linear electroelastic",
          std::make_pair((XMLGen::Analyze::MaterialModelFunc)append_isotropic_linear_electroelastic_material_to_plato_problem, tFuncIndex)));

        // isotropic linear thermoelastic material
        tFuncIndex = std::type_index(typeid(append_isotropic_linear_thermoelastic_material_to_plato_problem));
        mMap.insert(std::make_pair("isotropic linear thermoelastic",
          std::make_pair((XMLGen::Analyze::MaterialModelFunc)append_isotropic_linear_thermoelastic_material_to_plato_problem, tFuncIndex)));

        // isotropic linear thermal material
        tFuncIndex = std::type_index(typeid(append_isotropic_linear_thermal_material_to_plato_problem));
        mMap.insert(std::make_pair("isotropic linear thermal",
          std::make_pair((XMLGen::Analyze::MaterialModelFunc)append_isotropic_linear_thermal_material_to_plato_problem, tFuncIndex)));

        // isotropic linear elastic material
        tFuncIndex = std::type_index(typeid(append_isotropic_linear_elastic_material_to_plato_problem));
        mMap.insert(std::make_pair("isotropic linear elastic",
          std::make_pair((XMLGen::Analyze::MaterialModelFunc)append_isotropic_linear_elastic_material_to_plato_problem, tFuncIndex)));
    }

public:
    /******************************************************************************//**
     * \fn AppendMaterialModelParameters
     * \brief Default constructor
    **********************************************************************************/
    AppendMaterialModelParameters()
    {
        this->insert();
    }

    /******************************************************************************//**
     * \fn call
     * \brief Append material model and corresponding parameters to plato_analyze_input_deck.xml file.
     * \param [in]     aMaterial    material model metadata
     * \param [in/out] aParentNode  pugi::xml_node
    **********************************************************************************/
    void call(const XMLGen::Material& aMaterial, pugi::xml_node &aParentNode) const
    {
        auto tLowerPerformer = Plato::tolower(aMaterial.performer());
        if(Plato::tolower(tLowerPerformer).compare("plato_analyze") != 0)
        {
            return;
        }

        auto tCategory = Plato::tolower(aMaterial.category());
        auto tMapItr = mMap.find(tCategory);
        if(tMapItr == mMap.end())
        {
            THROWERR(std::string("Material Model Function Interface: Did not find material model function with tag '") + tCategory
                + "' in list. " + "Material model is not supported in Plato Analyze.")
        }
        auto tTypeCastedFunc = reinterpret_cast<void(*)(const XMLGen::Material&, pugi::xml_node&)>(tMapItr->second.first);
        if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
        {
            THROWERR(std::string("Material Model Function Interface: Reinterpret cast for material function with tag '") + tCategory + "' failed.")
        }
        tTypeCastedFunc(aMaterial, aParentNode);
    }
};
// struct AppendMaterialModelParameters

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

std::string return_natural_boundary_condition_vector_values
(const XMLGen::Load& aLoad)
{
    std::vector<std::string> tTokens;
    for(auto& tToken : aLoad.values)
    {
        tTokens.push_back(tToken);
    }
    auto tOutput = XMLGen::transform_tokens_for_plato_analyze_input_deck(tTokens);
    return tOutput;
}

void append_uniform_vector_valued_load_to_plato_problem
(const std::string& aName,
 const XMLGen::Load& aLoad,
 pugi::xml_node &aParentNode)
{
    auto tVecValuedLoad = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {aName}, tVecValuedLoad);
    std::vector<std::string> tKeys = {"name", "type","value"};
    std::vector<std::string> tValues = {"Type", "string", "Uniform"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tVecValuedLoad);

    auto tVecValues = XMLGen::return_natural_boundary_condition_vector_values(aLoad);
    tValues = {"Values", "Array(double)", tVecValues};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tVecValuedLoad);
    tValues = {"Sides", "string", aLoad.app_name};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tVecValuedLoad);
}

void append_uniform_single_valued_load_to_plato_problem
(const std::string& aName,
 const XMLGen::Load& aLoad,
 pugi::xml_node& aParentNode)
{
    auto tUniformSingleValuedLoad = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {aName}, tUniformSingleValuedLoad);
    std::vector<std::string> tKeys = {"name", "type","value"};
    std::vector<std::string> tValues = {"Type", "string", "Uniform"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tUniformSingleValuedLoad);

    if(aLoad.values.size() > 1u)
    {
        THROWERR(std::string("Append Uniform Single Valued Load to Plato Problem: Single valued functions" )
            + " are expected to have one value. However uniform load with name '" + aName + "' has '"
            + std::to_string(aLoad.values.size()) + "' values.");
    }
    tValues = {"Value", "double", aLoad.values[0]};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tUniformSingleValuedLoad);
    tValues = {"Sides", "string", aLoad.app_name};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tUniformSingleValuedLoad);
}

/******************************************************************************//**
 * \struct The goal of this C++ struct is to provide an interface for the \n
 * functions used to define natural boundary conditions' names for \n
 * plato_analyze_input_deck.xml. This interface reduces cyclomatic complexity \n
 * due to multiple natural boundary conditions categories in Plato Analyze.
**********************************************************************************/
struct AppendNaturalBoundaryCondition
{
private:
    /*!< map from natural boundary condition category to function used to define its properties */
    XMLGen::Analyze::NaturalBCFuncMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert natural boundary condition functions to map.
     **********************************************************************************/
    void insert()
    {
        // traction load
        auto tFuncIndex = std::type_index(typeid(append_uniform_vector_valued_load_to_plato_problem));
        mMap.insert(std::make_pair("traction",
          std::make_pair((XMLGen::Analyze::NaturalBCFunc)append_uniform_vector_valued_load_to_plato_problem, tFuncIndex)));

        // uniform pressure load
        tFuncIndex = std::type_index(typeid(append_uniform_single_valued_load_to_plato_problem));
        mMap.insert(std::make_pair("uniform pressure",
          std::make_pair((XMLGen::Analyze::NaturalBCFunc)append_uniform_single_valued_load_to_plato_problem, tFuncIndex)));

        // uniform surface potential
        tFuncIndex = std::type_index(typeid(append_uniform_single_valued_load_to_plato_problem));
        mMap.insert(std::make_pair("uniform surface potential",
          std::make_pair((XMLGen::Analyze::NaturalBCFunc)append_uniform_single_valued_load_to_plato_problem, tFuncIndex)));

        // uniform surface flux
        tFuncIndex = std::type_index(typeid(append_uniform_single_valued_load_to_plato_problem));
        mMap.insert(std::make_pair("uniform surface flux",
          std::make_pair((XMLGen::Analyze::NaturalBCFunc)append_uniform_single_valued_load_to_plato_problem, tFuncIndex)));
    }

public:
    /******************************************************************************//**
     * \fn AppendNaturalBoundaryCondition
     * \brief Default constructor
    **********************************************************************************/
    AppendNaturalBoundaryCondition()
    {
        this->insert();
    }

    /******************************************************************************//**
     * \fn call
     * \brief Append natural boundary condition parameters to plato_analyze_input_deck.xml.
     * \param [in]     aName        natural boundary condition identification name
     * \param [in]     aLoad        natural boundary condition metadata
     * \param [in/out] aParentNode  pugi::xml_node
    **********************************************************************************/
    void call(const std::string& aName, const XMLGen::Load& aLoad, pugi::xml_node& aParentNode) const
    {
        auto tCategory = Plato::tolower(aLoad.type);
        auto tMapItr = mMap.find(tCategory);
        if(tMapItr == mMap.end())
        {
            THROWERR(std::string("Natural Boundary Condition Function Interface: Did not find natural boundary condition function with tag '")
                + tCategory + "' in natural boundary condition function list.")
        }
        auto tTypeCastedFunc = reinterpret_cast<void(*)(const std::string&, const XMLGen::Load&, pugi::xml_node&)>(tMapItr->second.first);
        if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
        {
            THROWERR(std::string("Natural Boundary Condition Interface: Reinterpret cast for natural boundary condition function with tag '")
                + tCategory + "' failed.")
        }
        tTypeCastedFunc(aName, aLoad, aParentNode);
    }
};
// struct AppendNaturalBoundaryCondition

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

std::string return_displacement_bc_name
(const XMLGen::BC& aBC)
{
    std::string tOutput;
    if(aBC.mIsRandom)
    {
        tOutput = std::string("Random Displacement Boundary Condition with ID ") + aBC.bc_id;
    }
    else
    {
        tOutput = std::string("Displacement Boundary Condition with ID ") + aBC.bc_id;
    }
    return tOutput;
}

std::string return_potential_bc_name
(const XMLGen::BC& aBC)
{
    std::string tOutput;
    if(aBC.mIsRandom)
    {
        tOutput = std::string("Random Potential Boundary Condition with ID ") + aBC.bc_id;
    }
    else
    {
        tOutput = std::string("Potential Boundary Condition with ID ") + aBC.bc_id;
    }
    return tOutput;
}

std::string return_velocity_bc_name
(const XMLGen::BC& aBC)
{
    std::string tOutput;
    if(aBC.mIsRandom)
    {
        tOutput = std::string("Random Velocity Boundary Condition with ID ") + aBC.bc_id;
    }
    else
    {
        tOutput = std::string("Velocity Boundary Condition with ID ") + aBC.bc_id;
    }
    return tOutput;
}

std::string return_temperature_bc_name
(const XMLGen::BC& aBC)
{
    std::string tOutput;
    if(aBC.mIsRandom)
    {
        tOutput = std::string("Random Temperature Boundary Condition with ID ") + aBC.bc_id;
    }
    else
    {
        tOutput = std::string("Temperature Boundary Condition with ID ") + aBC.bc_id;
    }
    return tOutput;
}

/******************************************************************************//**
 * \struct The goal of this C++ struct is to provide an interface for the \n
 * functions used to define essential boundary conditions' tags for \n
 * plato_analyze_input_deck.xml. This interface reduces cyclomatic complexity \n
 * due to multiple essential boundary conditions categories in Plato Analyze.
**********************************************************************************/
struct EssentialBoundaryConditionTag
{
private:
    /*!< map from Neumman load category to function used to define its tag */
    XMLGen::Analyze::EssentialBCTagFuncMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert functions to essential boundary condition functions map.
     **********************************************************************************/
    void insert()
    {
        // temperature
        auto tFuncIndex = std::type_index(typeid(return_temperature_bc_name));
        mMap.insert(std::make_pair("temperature",
          std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)return_temperature_bc_name, tFuncIndex)));

        // velocity
        tFuncIndex = std::type_index(typeid(return_velocity_bc_name));
        mMap.insert(std::make_pair("velocity",
          std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)return_velocity_bc_name, tFuncIndex)));

        // potential
        tFuncIndex = std::type_index(typeid(return_potential_bc_name));
        mMap.insert(std::make_pair("potential",
          std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)return_potential_bc_name, tFuncIndex)));

        // displacement
        tFuncIndex = std::type_index(typeid(return_displacement_bc_name));
        mMap.insert(std::make_pair("displacement",
          std::make_pair((XMLGen::Analyze::EssentialBCTagFunc)return_displacement_bc_name, tFuncIndex)));
    }

public:
    /******************************************************************************//**
     * \fn EssentialBoundaryConditionTag
     * \brief Default constructor
    **********************************************************************************/
    EssentialBoundaryConditionTag()
    {
        this->insert();
    }

    /******************************************************************************//**
     * \fn call
     * \brief Return essential boundary condition tag for plato_analyze_input_deck.xml.
     * \param [in] aBC essential boundary condition metadata
     * \return essential boundary condition tag
    **********************************************************************************/
    std::string call(const XMLGen::BC& aBC) const
    {
        auto tTag = Plato::tolower(aBC.type);
        auto tMapItr = mMap.find(tTag);
        if(tMapItr == mMap.end())
        {
            THROWERR(std::string("Essential Boundary Condition Tag Function Interface: Did not find essential ")
                + "boundary condition function with tag '" + tTag + "' in essential boundary condition function list.")
        }
        auto tTypeCastedFunc = reinterpret_cast<std::string(*)(const XMLGen::BC&)>(tMapItr->second.first);
        if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
        {
            THROWERR(std::string("Essential Boundary Condition Tag Function Interface: Reinterpret cast for essential ")
                + "boundary condition function with tag '" + tTag + "' failed.")
        }
        auto tName = tTypeCastedFunc(aBC);
        return tName;
    }
};
// struct EssentialBoundaryConditionTag

struct ValidDofsKeys
{
    /*!< map from physics to map from degree of freedom name to degree of freedom index, i.e. map<physics, map<dof_name, dof_index>> */
    std::unordered_map<std::string, std::unordered_map<std::string,std::string>> mKeys =
        {
            {"mechanical", { {"dispx", "0"}, {"dispy", "1"}, {"dispz", "2"} } },
            {"thermal", { {"temp", "0"} } }, {"electrostatics", { {"potential", "0"} } },
            {"thermalmechanics", { {"dispx", "0"}, {"dispy", "1"}, {"dispz", "2"}, {"temp", "3"} } },
            {"electromechanics", { {"dispx", "0"}, {"dispy", "1"}, {"dispz", "2"}, {"potential", "3"} } }
        };
};
// struct ValidDofsKeys

void check_essential_boundary_condition_application_name_keyword
(const XMLGen::BC& aBC)
{
    if(aBC.app_name.empty())
    {
        THROWERR(std::string("Check Essential Boundary Condition Application Set Name: ")
            + "Application set name, e.g. sideset or nodeset, for Essential Boundary Condition "
            + "with identification number '" + aBC.bc_id + "' is empty.")
    }
}

void check_essential_boundary_condition_value_keyword
(const XMLGen::BC& aBC)
{
    if(aBC.value.empty())
    {
        THROWERR(std::string("Check Essential Boundary Condition Value: Value parameter for Essential Boundary Condition ")
            + "with identification number '" + aBC.bc_id + "' is empty.")
    }
}

void append_rigid_essential_boundary_condition_to_plato_problem
(const std::string& aName,
 const XMLGen::BC& aBC,
 pugi::xml_node &aParentNode)
{
    XMLGen::ValidDofsKeys tValidDofs;
    auto tLowerPhysics = Plato::tolower(aBC.mPhysics);
    auto tDofsKeysItr = tValidDofs.mKeys.find(tLowerPhysics);
    if(tDofsKeysItr == tValidDofs.mKeys.end())
    {
        THROWERR(std::string("Append Rigid Essential Boundary Condition to Plato Problem: ")
            + "Physics '" + tLowerPhysics + "' is not supported in Plato Analyze.")
    }
    XMLGen::check_essential_boundary_condition_application_name_keyword(aBC);

    std::vector<std::string> tKeys = {"name", "type", "value"};
    for(auto& tDofNameItr : tDofsKeysItr->second)
    {
        auto tBCName = aName + " applied to Dof with tag " + Plato::toupper(tDofNameItr.first);
        auto tEssentialBoundaryCondParentNode = aParentNode.append_child("ParameterList");
        XMLGen::append_attributes({"name"}, {tBCName}, tEssentialBoundaryCondParentNode);
        std::vector<std::string> tValues = {"Type", "string", "Zero Value"};
        XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
        tValues = {"Index", "int", tDofNameItr.second};
        XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
        tValues = {"Sides", "string", aBC.app_name};
        XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
    }
}

void append_zero_value_essential_boundary_condition_to_plato_problem
(const std::string& aName,
 const XMLGen::BC& aBC,
 pugi::xml_node &aParentNode)
{
    XMLGen::ValidDofsKeys tValidDofs;
    auto tLowerPhysics = Plato::tolower(aBC.mPhysics);
    auto tDofsKeysItr = tValidDofs.mKeys.find(tLowerPhysics);
    if(tDofsKeysItr == tValidDofs.mKeys.end())
    {
        THROWERR(std::string("Append Zero Value Essential Boundary Condition to Plato Problem: ")
            + "Physics '" + tLowerPhysics + "' is not supported in Plato Analyze.")
    }

    auto tLowerDof = Plato::tolower(aBC.dof);
    auto tDofNameItr = tDofsKeysItr->second.find(tLowerDof);
    if(tDofNameItr == tDofsKeysItr->second.end())
    {
        THROWERR(std::string("Append Zero Value Essential Boundary Condition to Plato Problem: ")
            + "Degree of Freedom tag/key '" + tLowerDof + "' is not supported for physics '" + tLowerPhysics + "'.")
    }
    XMLGen::check_essential_boundary_condition_application_name_keyword(aBC);

    auto tBCName = aName + " applied to Dof with tag " + Plato::toupper(tDofNameItr->first);
    auto tEssentialBoundaryCondParentNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {tBCName}, tEssentialBoundaryCondParentNode);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "Zero Value"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
    tValues = {"Index", "int", tDofNameItr->second};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
    tValues = {"Sides", "string", aBC.app_name};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
}

void append_fixed_value_essential_boundary_condition_to_plato_problem
(const std::string& aName,
 const XMLGen::BC& aBC,
 pugi::xml_node &aParentNode)
{
    XMLGen::ValidDofsKeys tValidDofs;
    auto tLowerPhysics = Plato::tolower(aBC.mPhysics);
    auto tDofsKeysItr = tValidDofs.mKeys.find(tLowerPhysics);
    if(tDofsKeysItr == tValidDofs.mKeys.end())
    {
        THROWERR(std::string("Append Fixed Value Essential Boundary Condition to Plato Problem: ")
            + "Physics '" + tLowerPhysics + "' is not supported in Plato Analyze.")
    }

    auto tLowerDof = Plato::tolower(aBC.dof);
    auto tDofNameItr = tDofsKeysItr->second.find(tLowerDof);
    if(tDofNameItr == tDofsKeysItr->second.end())
    {
        THROWERR(std::string("Append Fixed Value Essential Boundary Condition to Plato Problem: ")
            + "Degree of Freedom tag/key '" + tLowerDof + "' is not supported for physics '" + tLowerPhysics + "'.")
    }
    XMLGen::check_essential_boundary_condition_value_keyword(aBC);
    XMLGen::check_essential_boundary_condition_application_name_keyword(aBC);

    auto tBCName = aName + " applied to Dof with tag " + Plato::toupper(tDofNameItr->first);
    auto tEssentialBoundaryCondParentNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {tBCName}, tEssentialBoundaryCondParentNode);
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Type", "string", "Fixed Value"};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
    tValues = {"Index", "int", tDofNameItr->second};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
    tValues = {"Sides", "string", aBC.app_name};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
    tValues = {"Value", "double", aBC.value};
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tEssentialBoundaryCondParentNode);
}

/******************************************************************************//**
 * \struct The goal of this C++ struct is to provide an interface for the \n
 * functions used to define essential boundary conditions' names for \n
 * plato_analyze_input_deck.xml. This interface reduces cyclomatic complexity \n
 * due to multiple essential boundary conditions categories in Plato Analyze.
**********************************************************************************/
struct AppendEssentialBoundaryCondition
{
private:
    /*!< map from essential boundary condition category to function used to append its properties */
    XMLGen::Analyze::EssentialBCFuncMap mMap;

    /******************************************************************************//**
     * \fn insert
     * \brief Insert essential boundary condition functions to map.
     **********************************************************************************/
    void insert()
    {
        // rigid
        auto tFuncIndex = std::type_index(typeid(append_rigid_essential_boundary_condition_to_plato_problem));
        mMap.insert(std::make_pair("rigid",
          std::make_pair((XMLGen::Analyze::EssentialBCFunc)append_rigid_essential_boundary_condition_to_plato_problem, tFuncIndex)));

        // zero value
        tFuncIndex = std::type_index(typeid(append_zero_value_essential_boundary_condition_to_plato_problem));
        mMap.insert(std::make_pair("zero value",
          std::make_pair((XMLGen::Analyze::EssentialBCFunc)append_zero_value_essential_boundary_condition_to_plato_problem, tFuncIndex)));

        // fixed value
        tFuncIndex = std::type_index(typeid(append_fixed_value_essential_boundary_condition_to_plato_problem));
        mMap.insert(std::make_pair("fixed value",
          std::make_pair((XMLGen::Analyze::EssentialBCFunc)append_fixed_value_essential_boundary_condition_to_plato_problem, tFuncIndex)));

        // insulated
        tFuncIndex = std::type_index(typeid(append_zero_value_essential_boundary_condition_to_plato_problem));
        mMap.insert(std::make_pair("insulated",
          std::make_pair((XMLGen::Analyze::EssentialBCFunc)append_zero_value_essential_boundary_condition_to_plato_problem, tFuncIndex)));
    }

public:
    /******************************************************************************//**
     * \fn AppendEssentialBoundaryCondition
     * \brief Default constructor
    **********************************************************************************/
    AppendEssentialBoundaryCondition()
    {
        this->insert();
    }

    /******************************************************************************//**
     * \fn call
     * \brief Append essential boundary condition parameters to plato_analyze_input_deck.xml.
     * \param [in]     aName        essential boundary condition name
     * \param [in]     aBC          essential boundary condition metadata
     * \param [in/out] aParentNode  pugi::xml_node
    **********************************************************************************/
    void call(const std::string& aName, const XMLGen::BC& aBC, pugi::xml_node& aParentNode) const
    {
        auto tLowerPerformer = Plato::tolower(aBC.mPerformerName);
        if(tLowerPerformer.compare("plato_analyze") != 0)
        {
            return;
        }

        auto tCategory = Plato::tolower(aBC.mCategory);
        auto tMapItr = mMap.find(tCategory);
        if(tMapItr == mMap.end())
        {
            THROWERR(std::string("Essential Boundary Condition Function Interface: Did not find essential boundary condition function with tag '")
                + tCategory + "' in list. Essential boundary condition '" + tCategory + "' is not supported in Plato Analyze.")
        }
        auto tTypeCastedFunc = reinterpret_cast<void(*)(const std::string&, const XMLGen::BC&, pugi::xml_node&)>(tMapItr->second.first);
        if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
        {
            THROWERR(std::string("Essential Boundary Condition Interface: Reinterpret cast for essential boundary condition function with tag '")
                + tCategory + "' failed.")
        }
        tTypeCastedFunc(aName, aBC, aParentNode);
    }
};
// struct AppendEssentialBoundaryCondition

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

void write_plato_analyze_input_deck_file
(const XMLGen::InputData& aXMLMetaData)
{
    pugi::xml_document tDocument;

    XMLGen::append_problem_description_to_plato_analyze_input_deck(aXMLMetaData, tDocument);
    XMLGen::append_plato_problem_description_to_plato_analyze_input_deck(aXMLMetaData, tDocument);
    XMLGen::append_plato_problem_to_plato_analyze_input_deck(aXMLMetaData, tDocument); // TODO: FINISH UNIT TESTING THIS FUNCTION

    tDocument.save_file("plato_analyze_input_deck.xml", "  ");
}

}

namespace PlatoTestXMLGenerator
{

TEST(PlatoTestXMLGenerator, WritePlatoAnalyzeInputDeckFile)
{
    // POSE PROBLEM ESSENTIAL BC
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_2";
    tBC.type = "displacement";
    tBC.mPhysics = "mechanical";
    tBC.mCategory = "rigid";
    tBC.mPerformerName = "plato_analyze";

    // POSE DESIGN INTENTS
    XMLGen::Objective tObjective;
    tObjective.type = "maximize stiffness";
    tObjective.mPerformerName = "plato_analyze";
    XMLGen::Constraint tConstraint;
    tConstraint.type = "volume";
    tConstraint.mPenaltyParam = "1.0";
    tConstraint.mMinimumErsatzValue = "0.0";
    tConstraint.mPerformerName = "plato_analyze";

    // POSE MATERIAL
    XMLGen::Material tMaterial;
    tMaterial.performer("plato_analyze");
    tMaterial.category("isotropic linear elastic");
    tMaterial.property("youngs modulus", "1e9");
    tMaterial.property("poissons ratio", "0.3");

    // POSE NATURAL BC
    XMLGen::Load tLoad;
    tLoad.type = "traction";
    tLoad.load_id = "1";
    tLoad.app_name = "ss_1";
    tLoad.values = {"1.0", "2.0", "3.0"};
    XMLGen::LoadCase tLoadCase;
    tLoadCase.mPerformerName = "plato_analyze";
    tLoadCase.loads.push_back(tLoad);

    // SET INPUT METADATA
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.bcs.push_back(tBC);
    tXMLMetaData.materials.push_back(tMaterial);
    tXMLMetaData.load_cases.push_back(tLoadCase);
    tXMLMetaData.objectives.push_back(tObjective);
    tXMLMetaData.constraints.push_back(tConstraint);
    tXMLMetaData.run_mesh_name = "lbracket.exo";
    tXMLMetaData.mPhysicsMetaData.mSpatialDims = "3";
    tXMLMetaData.mPhysicsMetaData.mPhysics = "mechanical";

    // CALL FUNCTION
    XMLGen::write_plato_analyze_input_deck_file(tXMLMetaData);

    // TEST OUTPUT
    auto tData = XMLGen::read_data_from_file("plato_analyze_input_deck.xml");
    auto tGold = std::string("<?xmlversion=\"1.0\"?><ParameterListname=\"Problem\"><Parametername=\"Physics\"type=\"string\"value=\"PlatoDriver\"/><Parametername=\"SpatialDimension\"type=\"int\"value=\"3\"/>")
    +"<Parametername=\"InputMesh\"type=\"string\"value=\"lbracket.exo\"/><ParameterListname=\"PlatoProblem\"><Parametername=\"Physics\"type=\"string\"value=\"Mechanical\"/><Parametername=\"PDEConstraint\"type=\"string\"value=\"Elliptic\"/>"
    +"<Parametername=\"Constraint\"type=\"string\"value=\"MyConstraint\"/><Parametername=\"Objective\"type=\"string\"value=\"MyObjective\"/><Parametername=\"Self-Adjoint\"type=\"bool\"value=\"true\"/><ParameterListname=\"MyObjective\">"
    +"<Parametername=\"Type\"type=\"string\"value=\"WeightedSum\"/><Parametername=\"Functions\"type=\"Array(string)\"value=\"{mymaximizestiffness}\"/><Parametername=\"Weights\"type=\"Array(double)\"value=\"{1.0}\"/></ParameterList>"
    +"<ParameterListname=\"mymaximizestiffness\"><Parametername=\"Type\"type=\"string\"value=\"ScalarFunction\"/><Parametername=\"ScalarFunctionType\"type=\"string\"value=\"InternalElasticEnergy\"/><PenaltyFunction>"
    +"<Parametername=\"Type\"type=\"string\"value=\"SIMP\"/><Parametername=\"Exponent\"type=\"double\"value=\"3.0\"/><Parametername=\"MinimumValue\"type=\"double\"value=\"1e-9\"/></PenaltyFunction></ParameterList>"
    +"<ParameterListname=\"MyConstraint\"><Parametername=\"Type\"type=\"string\"value=\"WeightedSum\"/><Parametername=\"Functions\"type=\"Array(string)\"value=\"{myvolume}\"/><Parametername=\"Weights\"type=\"Array(double)\"value=\"{1.0}\"/>"
    +"</ParameterList><ParameterListname=\"myvolume\"><Parametername=\"Type\"type=\"string\"value=\"ScalarFunction\"/><Parametername=\"ScalarFunctionType\"type=\"string\"value=\"Volume\"/><PenaltyFunction>"
    +"<Parametername=\"Type\"type=\"string\"value=\"SIMP\"/><Parametername=\"Exponent\"type=\"double\"value=\"1.0\"/><Parametername=\"MinimumValue\"type=\"double\"value=\"0.0\"/></PenaltyFunction></ParameterList>"
    +"<ParameterListname=\"Elliptic\"><PenaltyFunction><Parametername=\"Type\"type=\"string\"value=\"SIMP\"/><Parametername=\"Exponent\"type=\"double\"value=\"3.0\"/><Parametername=\"MinimumValue\"type=\"double\"value=\"1e-9\"/>"
    +"</PenaltyFunction></ParameterList><ParameterListname=\"MaterialModel\"><ParameterListname=\"IsotropicLinearElastic\"><Parametername=\"PoissonsRatio\"type=\"double\"value=\"0.3\"/><Parametername=\"YoungsModulus\"type=\"double\"value=\"1e9\"/>"
    +"</ParameterList></ParameterList><ParameterListname=\"NaturalBoundaryConditions\"><ParameterListname=\"TractionVectorBoundaryConditionwithID1\"><Parametername=\"Type\"type=\"string\"value=\"Uniform\"/>"
    +"<Parametername=\"Values\"type=\"Array(double)\"value=\"{1.0,2.0,3.0}\"/><Parametername=\"Sides\"type=\"string\"value=\"ss_1\"/></ParameterList></ParameterList><ParameterListname=\"EssentialBoundaryConditions\">"
    +"<ParameterListname=\"DisplacementBoundaryConditionwithID1appliedtoDofwithtagDISPZ\"><Parametername=\"Type\"type=\"string\"value=\"ZeroValue\"/><Parametername=\"Index\"type=\"int\"value=\"2\"/>"
    +"<Parametername=\"Sides\"type=\"string\"value=\"ss_2\"/></ParameterList><ParameterListname=\"DisplacementBoundaryConditionwithID1appliedtoDofwithtagDISPY\"><Parametername=\"Type\"type=\"string\"value=\"ZeroValue\"/>"
    +"<Parametername=\"Index\"type=\"int\"value=\"1\"/><Parametername=\"Sides\"type=\"string\"value=\"ss_2\"/></ParameterList><ParameterListname=\"DisplacementBoundaryConditionwithID1appliedtoDofwithtagDISPX\">"
    +"<Parametername=\"Type\"type=\"string\"value=\"ZeroValue\"/><Parametername=\"Index\"type=\"int\"value=\"0\"/><Parametername=\"Sides\"type=\"string\"value=\"ss_2\"/></ParameterList></ParameterList></ParameterList></ParameterList>";
    ASSERT_STREQ(tGold.c_str(), tData.str().c_str());
    Plato::system("rm -f plato_analyze_input_deck.xml");
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_ErrorEmptyAppName)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.mPhysics = "mechanical";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Displacement Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_ErrorInvalidPhysics)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.mPhysics = "cfd";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Displacement Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_ErrorInvalidCategory)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "mechanical";
    tBC.mCategory = "pin";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Displacement Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryZeroValue_ErrorInvalidPhysics)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "cfd";
    tBC.mCategory = "zero value";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryZeroValue_ErrorEmptyDof)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "zero value";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryZeroValue_ErrorInvalidDof)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "dispx";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "zero value";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryRigid)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "mechanical";
    tBC.mCategory = "rigid";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Displacement Boundary Condition with ID 1", tBC, tDocument));

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Zero Value"}, {"Index", "int", "2"}, {"Sides", "string", "ss_1"},
          {"Type", "string", "Zero Value"}, {"Index", "int", "1"}, {"Sides", "string", "ss_1"},
          {"Type", "string", "Zero Value"}, {"Index", "int", "0"}, {"Sides", "string", "ss_1"} };
    std::vector<std::string> tGoldParameterListNames =
        {"Displacement Boundary Condition with ID 1 applied to Dof with tag DISPZ",
         "Displacement Boundary Condition with ID 1 applied to Dof with tag DISPY",
         "Displacement Boundary Condition with ID 1 applied to Dof with tag DISPX"};

    auto tParamList = tDocument.child("ParameterList");
    auto tGoldValuesItr = tGoldValues.begin();
    auto tGoldParameterListNamesItr = tGoldParameterListNames.begin();
    while(!tParamList.empty())
    {
        ASSERT_FALSE(tParamList.empty());
        ASSERT_STREQ("ParameterList", tParamList.name());
        PlatoTestXMLGenerator::test_attributes({"name"}, {tGoldParameterListNamesItr->c_str()}, tParamList);

        auto tParameter = tParamList.child("Parameter");
        while(!tParameter.empty())
        {
            ASSERT_FALSE(tParameter.empty());
            ASSERT_STREQ("Parameter", tParameter.name());
            PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
            tParameter = tParameter.next_sibling();
            std::advance(tGoldValuesItr, 1);
        }
        tParamList = tParamList.next_sibling();
        std::advance(tGoldParameterListNamesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryZeroValue)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "temp";
    tBC.app_name = "ss_2";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "zero value";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument));

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Zero Value"}, {"Index", "int", "0"}, {"Sides", "string", "ss_2"} };

    auto tParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Thermal Boundary Condition with ID 1 applied to Dof with tag TEMP"}, tParamList);

    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryFixedValue_ErrorInvalidPhysics)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "cfd";
    tBC.mCategory = "fixed value";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryFixedValue_ErrorEmptyDof)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "fixed value";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryFixedValue_ErrorInvalidDof)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "dispx";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "fixed value";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryFixedValue_ErrorEmptyValue)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "temp";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "fixed value";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryFixedValue)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "temp";
    tBC.value = "10.0";
    tBC.app_name = "ss_2";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "fixed value";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument));

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Fixed Value"}, {"Index", "int", "0"}, {"Sides", "string", "ss_2"}, {"Value", "double", "10.0"} };

    auto tParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Thermal Boundary Condition with ID 1 applied to Dof with tag TEMP"}, tParamList);

    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryInsulated_ErrorInvalidPhysics)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "temp";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "cfd";
    tBC.mCategory = "insulated";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryInsulated_ErrorEmptyDof)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "insulated";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryInsulated_ErrorInvalidDof)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "dispx";
    tBC.app_name = "ss_1";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "insulated";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_CategoryInsulated)
{
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.dof = "temp";
    tBC.app_name = "ss_11";
    tBC.mPhysics = "thermal";
    tBC.mCategory = "insulated";
    tBC.mPerformerName = "plato_analyze";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Thermal Boundary Condition with ID 1", tBC, tDocument));
    tDocument.save_file("dummy.xml", "  ");

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Zero Value"}, {"Index", "int", "0"}, {"Sides", "string", "ss_11"} };

    auto tParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Thermal Boundary Condition with ID 1 applied to Dof with tag TEMP"}, tParamList);

    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryCondition_PerformerNotAnalyze_DoNothing)
{
    XMLGen::BC tBC;
    tBC.type = "displacement";
    tBC.bc_id = "1";
    tBC.mPerformerName = "sierra";
    pugi::xml_document tDocument;

    XMLGen::AppendEssentialBoundaryCondition tInterface;
    tInterface.call("Displacement Boundary Condition with ID 1", tBC, tDocument);
    auto tEssentialBCParamList = tDocument.child("ParameterList");
    ASSERT_TRUE(tEssentialBCParamList.empty());
}

TEST(PlatoTestXMLGenerator, AppendEssentialBoundaryConditionsToPlatoAnalyzeInputDeck)
{
    // POSE PROBLEM
    XMLGen::BC tBC;
    tBC.bc_id = "1";
    tBC.app_name = "ss_1";
    tBC.type = "displacement";
    tBC.mPhysics = "mechanical";
    tBC.mCategory = "rigid";
    tBC.mPerformerName = "plato_analyze";
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.bcs.push_back(tBC);

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_essential_boundary_conditions_to_plato_analyze_input_deck(tXMLMetaData, tDocument);
    tDocument.save_file("dummy.xml", " ");

    // TEST
    auto tEssentialBC = tDocument.child("ParameterList");
    ASSERT_FALSE(tEssentialBC.empty());
    ASSERT_STREQ("ParameterList", tEssentialBC.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Essential Boundary Conditions"}, tEssentialBC);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Zero Value"}, {"Index", "int", "2"}, {"Sides", "string", "ss_1"},
          {"Type", "string", "Zero Value"}, {"Index", "int", "1"}, {"Sides", "string", "ss_1"},
          {"Type", "string", "Zero Value"}, {"Index", "int", "0"}, {"Sides", "string", "ss_1"} };
    std::vector<std::string> tGoldParameterListNames =
        {"Displacement Boundary Condition with ID 1 applied to Dof with tag DISPZ",
         "Displacement Boundary Condition with ID 1 applied to Dof with tag DISPY",
         "Displacement Boundary Condition with ID 1 applied to Dof with tag DISPX"};

    auto tParamList = tEssentialBC.child("ParameterList");
    auto tGoldValuesItr = tGoldValues.begin();
    auto tGoldParameterListNamesItr = tGoldParameterListNames.begin();
    while(!tParamList.empty())
    {
        ASSERT_FALSE(tParamList.empty());
        ASSERT_STREQ("ParameterList", tParamList.name());
        PlatoTestXMLGenerator::test_attributes({"name"}, {tGoldParameterListNamesItr->c_str()}, tParamList);

        auto tParameter = tParamList.child("Parameter");
        while(!tParameter.empty())
        {
            ASSERT_FALSE(tParameter.empty());
            ASSERT_STREQ("Parameter", tParameter.name());
            PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
            tParameter = tParameter.next_sibling();
            std::advance(tGoldValuesItr, 1);
        }
        tParamList = tParamList.next_sibling();
        std::advance(tGoldParameterListNamesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, EssentialBoundaryConditionTag_InvalidTag)
{
    XMLGen::EssentialBoundaryConditionTag tInterface;
    XMLGen::BC tBC;
    tBC.type = "fluid velocity";
    ASSERT_THROW(tInterface.call(tBC), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, EssentialBoundaryConditionTag)
{
    XMLGen::EssentialBoundaryConditionTag tInterface;
    XMLGen::BC tBC;

    // TEST 1
    tBC.type = "displacement";
    tBC.bc_id = "1";
    auto tName = tInterface.call(tBC);
    ASSERT_STREQ("Displacement Boundary Condition with ID 1", tName.c_str());

    // TEST 2
    tBC.type = "temperature";
    tBC.bc_id = "1";
    tName = tInterface.call(tBC);
    ASSERT_STREQ("Temperature Boundary Condition with ID 1", tName.c_str());

    // TEST 3
    tBC.type = "potential";
    tBC.bc_id = "1";
    tName = tInterface.call(tBC);
    ASSERT_STREQ("Potential Boundary Condition with ID 1", tName.c_str());

    // TEST 4
    tBC.type = "velocity";
    tBC.bc_id = "1";
    tName = tInterface.call(tBC);
    ASSERT_STREQ("Velocity Boundary Condition with ID 1", tName.c_str());
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryConditionsToPlatoAnalyzeInputDeck_DoNotAppend_PerformerIsNotAnalyze)
{
    XMLGen::Load tLoad;
    tLoad.type = "traction";
    tLoad.load_id = "1";
    tLoad.app_name = "ss_1";
    tLoad.values = {"1.0", "2.0", "3.0"};
    XMLGen::LoadCase tLoadCase;
    tLoadCase.mPerformerName = "sierra";
    tLoadCase.loads.push_back(tLoad);
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.load_cases.push_back(tLoadCase);

    pugi::xml_document tDocument;
    XMLGen::append_natural_boundary_conditions_to_plato_analyze_input_deck(tXMLMetaData, tDocument);
    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_TRUE(tLoadParamList.empty());
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryConditionsToPlatoAnalyzeInputDeck)
{
    XMLGen::Load tLoad;
    tLoad.type = "traction";
    tLoad.load_id = "1";
    tLoad.app_name = "ss_1";
    tLoad.values = {"1.0", "2.0", "3.0"};
    XMLGen::LoadCase tLoadCase;
    tLoadCase.mPerformerName = "plato_analyze";
    tLoadCase.loads.push_back(tLoad);
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.load_cases.push_back(tLoadCase);

    pugi::xml_document tDocument;
    XMLGen::append_natural_boundary_conditions_to_plato_analyze_input_deck(tXMLMetaData, tDocument);

    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tLoadParamList.empty());
    ASSERT_STREQ("ParameterList", tLoadParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Natural Boundary Conditions"}, tLoadParamList);

    auto tTraction = tLoadParamList.child("ParameterList");
    ASSERT_FALSE(tTraction.empty());
    ASSERT_STREQ("ParameterList", tTraction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Traction Vector Boundary Condition with ID 1"}, tTraction);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Uniform"}, {"Values", "Array(double)", "{1.0, 2.0, 3.0}"}, {"Sides", "string", "ss_1"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tLoadParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryConditionsToPlatoAnalyzeInputDeck_RandomUseCase)
{
    // POSE LOAD SET 1
    XMLGen::LoadCase tLoadCase1;
    tLoadCase1.id = "1";
    XMLGen::Load tLoad1;
    tLoad1.mIsRandom = true;
    tLoad1.type = "traction";
    tLoad1.load_id = "1";
    tLoad1.app_name = "sideset";
    tLoad1.values.push_back("1.0");
    tLoad1.values.push_back("2.0");
    tLoad1.values.push_back("3.0");
    tLoadCase1.loads.push_back(tLoad1);
    auto tLoadSet1 = std::make_pair(0.5, tLoadCase1);

    // POSE LOAD SET 2
    XMLGen::LoadCase tLoadCase2;
    tLoadCase2.id = "2";
    XMLGen::Load tLoad2;
    tLoad2.mIsRandom = true;
    tLoad2.type = "traction";
    tLoad1.load_id = "1";
    tLoad2.app_name = "sideset";
    tLoad2.values.push_back("11");
    tLoad2.values.push_back("12");
    tLoad2.values.push_back("13");
    tLoadCase2.loads.push_back(tLoad2);
    auto tLoadSet2 = std::make_pair(0.5, tLoadCase2);

    // CONSTRUCT SAMPLES SET
    XMLGen::InputData tXMLMetaData;
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tLoadSet1));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tLoadSet2));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.finalize());

    // CALL FUNCTION
    pugi::xml_document tDocument;
    XMLGen::append_natural_boundary_conditions_to_plato_analyze_input_deck(tXMLMetaData, tDocument);

    // TEST
    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tLoadParamList.empty());
    ASSERT_STREQ("ParameterList", tLoadParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Natural Boundary Conditions"}, tLoadParamList);

    auto tTraction = tLoadParamList.child("ParameterList");
    ASSERT_FALSE(tTraction.empty());
    ASSERT_STREQ("ParameterList", tTraction.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Random Traction Vector Boundary Condition with ID 1"}, tTraction);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Uniform"}, {"Values", "Array(double)", "{1.0, 2.0, 3.0}"}, {"Sides", "string", "ss_1"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tLoadParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryCondition_ErrorInvalidType)
{
    XMLGen::Load tLoad;
    tLoad.type = "nonsense";
    pugi::xml_document tDocument;
    XMLGen::AppendNaturalBoundaryCondition tInterface;
    ASSERT_THROW(tInterface.call("Traction Vector Boundary Condition 1", tLoad, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryCondition_Traction)
{
    XMLGen::Load tLoad;
    tLoad.type = "traction";
    tLoad.load_id = "1";
    tLoad.app_name = "ss_1";
    tLoad.values = {"1.0", "2.0", "3.0"};
    pugi::xml_document tDocument;
    XMLGen::AppendNaturalBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Traction Vector Boundary Condition 1", tLoad, tDocument));

    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tLoadParamList.empty());
    ASSERT_STREQ("ParameterList", tLoadParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Traction Vector Boundary Condition 1"}, tLoadParamList);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Uniform"}, {"Values", "Array(double)", "{1.0, 2.0, 3.0}"}, {"Sides", "string", "ss_1"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tLoadParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryCondition_UniformPressure)
{
    XMLGen::Load tLoad;
    tLoad.type = "uniform pressure";
    tLoad.load_id = "1";
    tLoad.app_name = "ss_1";
    tLoad.values = {"1.0"};
    pugi::xml_document tDocument;
    XMLGen::AppendNaturalBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Uniform Pressure Boundary Condition 1", tLoad, tDocument));

    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tLoadParamList.empty());
    ASSERT_STREQ("ParameterList", tLoadParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Uniform Pressure Boundary Condition 1"}, tLoadParamList);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Uniform"}, {"Value", "double", "1.0"}, {"Sides", "string", "ss_1"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tLoadParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryCondition_UniformSurfacePotential)
{
    XMLGen::Load tLoad;
    tLoad.type = "uniform surface potential";
    tLoad.load_id = "1";
    tLoad.app_name = "ss_1";
    tLoad.values = {"1.0"};
    pugi::xml_document tDocument;
    XMLGen::AppendNaturalBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Uniform Surface Potential Boundary Condition 1", tLoad, tDocument));

    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tLoadParamList.empty());
    ASSERT_STREQ("ParameterList", tLoadParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Uniform Surface Potential Boundary Condition 1"}, tLoadParamList);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Uniform"}, {"Value", "double", "1.0"}, {"Sides", "string", "ss_1"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tLoadParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendNaturalBoundaryCondition_UniformSurfaceFlux)
{
    XMLGen::Load tLoad;
    tLoad.type = "uniform surface flux";
    tLoad.load_id = "1";
    tLoad.app_name = "ss_1";
    tLoad.values = {"1.0"};
    pugi::xml_document tDocument;
    XMLGen::AppendNaturalBoundaryCondition tInterface;
    ASSERT_NO_THROW(tInterface.call("Uniform Surface Flux Boundary Condition 1", tLoad, tDocument));

    auto tLoadParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tLoadParamList.empty());
    ASSERT_STREQ("ParameterList", tLoadParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Uniform Surface Flux Boundary Condition 1"}, tLoadParamList);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Uniform"}, {"Value", "double", "1.0"}, {"Sides", "string", "ss_1"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tLoadParamList.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, NaturalBoundaryConditionTag_ErrorInvalidType)
{
    XMLGen::NaturalBoundaryConditionTag tInterface;
    XMLGen::Load tLoad;
    tLoad.type = "nonsense";
    ASSERT_THROW(tInterface.call(tLoad), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, NaturalBoundaryConditionTag)
{
    XMLGen::NaturalBoundaryConditionTag tInterface;
    XMLGen::Load tLoad;

    // TRACTION TEST
    tLoad.type = "traction";
    tLoad.load_id = "1";
    auto tName = tInterface.call(tLoad);
    ASSERT_STREQ("Traction Vector Boundary Condition with ID 1", tName.c_str());

    tLoad.mIsRandom = true;
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Random Traction Vector Boundary Condition with ID 1", tName.c_str());

    // PRESSURE TEST
    tLoad.mIsRandom = false;
    tLoad.type = "uniform pressure";
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Uniform Pressure Boundary Condition with ID 1", tName.c_str());

    tLoad.mIsRandom = true;
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Random Uniform Pressure Boundary Condition with ID 1", tName.c_str());

    // SURFACE POTENTIAL TEST
    tLoad.mIsRandom = false;
    tLoad.type = "uniform surface potential";
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Uniform Surface Potential Boundary Condition with ID 1", tName.c_str());

    tLoad.mIsRandom = true;
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Random Uniform Surface Potential Boundary Condition with ID 1", tName.c_str());

    // SURFACE POTENTIAL TEST
    tLoad.mIsRandom = false;
    tLoad.type = "uniform surface flux";
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Uniform Surface Flux Boundary Condition with ID 1", tName.c_str());

    tLoad.mIsRandom = true;
    tName = tInterface.call(tLoad);
    ASSERT_STREQ("Random Uniform Surface Flux Boundary Condition with ID 1", tName.c_str());
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_ErrorEmptyMaterialContainer)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    ASSERT_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_Empty_MaterialIsNotFromAnalyzePerformer)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.performer("sierra");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));
    auto tParamList = tDocument.child("ParameterList");
    ASSERT_TRUE(tParamList.empty());
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_ErrorInvalidMatModel)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.performer("plato_analyze");
    tMaterial.category("isotropic linear viscoelastic");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_ErrorMatPropAreNotDefined)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.performer("plato_analyze");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_ErrorInvalidProperty)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.performer("plato_analyze");
    tMaterial.category("isotropic linear elastic");
    tMaterial.property("youngs modulus", "1e9");
    tMaterial.property("poissons ratio", "0.3");
    tMaterial.property("piezoelectric coupling 33", "123");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_IsotropicLinearElasticMatModel)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.performer("plato_analyze");
    tMaterial.category("isotropic linear elastic");
    tMaterial.property("youngs modulus", "1e9");
    tMaterial.property("poissons ratio", "0.3");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tMaterialModelParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tMaterialModelParamList.empty());
    ASSERT_STREQ("ParameterList", tMaterialModelParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Material Model"}, tMaterialModelParamList);
    auto tMyMaterialModel = tMaterialModelParamList.child("Isotropic Linear Elastic");
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Isotropic Linear Elastic"}, tMyMaterialModel);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Poissons Ratio", "double", "0.3"}, {"Youngs Modulus", "double", "1e9"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tMyMaterialModel.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_IsotropicLinearThermalMatModel)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.performer("plato_analyze");
    tMaterial.category("isotropic linear thermal");
    tMaterial.property("thermal conductivity coefficient", "10");
    tMaterial.property("mass density", "200");
    tMaterial.property("specific heat", "20");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tMaterialModelParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tMaterialModelParamList.empty());
    ASSERT_STREQ("ParameterList", tMaterialModelParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Material Model"}, tMaterialModelParamList);
    auto tMyMaterialModel = tMaterialModelParamList.child("Isotropic Linear Thermal");
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Isotropic Linear Thermal"}, tMyMaterialModel);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Mass Density", "double", "200"}, {"Specific Heat", "double", "20"}, {"Thermal Conductivity Coefficient", "double", "10"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tMyMaterialModel.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_OrthotropicLinearElasticMatModel)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.performer("plato_analyze");
    tMaterial.category("orthotropic linear elastic");
    tMaterial.property("youngs modulus x", "1.0");
    tMaterial.property("youngs modulus y", "2.0");
    tMaterial.property("youngs modulus z", "3.0");
    tMaterial.property("poissons ratio xy", "0.3");
    tMaterial.property("poissons ratio xz", "0.4");
    tMaterial.property("poissons ratio yz", "0.25");
    tMaterial.property("shear modulus ratio xy", "1.3");
    tMaterial.property("shear modulus ratio xz", "1.4");
    tMaterial.property("shear modulus ratio yz", "1.25");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tMaterialModelParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tMaterialModelParamList.empty());
    ASSERT_STREQ("ParameterList", tMaterialModelParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Material Model"}, tMaterialModelParamList);
    auto tMyMaterialModel = tMaterialModelParamList.child("Orthotropic Linear Elastic");
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Orthotropic Linear Elastic"}, tMyMaterialModel);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Poissons Ratio XY", "double", "0.3"}, {"Poissons Ratio XZ", "double", "0.4"}, {"Poissons Ratio YZ", "double", "0.25"},
          {"Shear Modulus XY", "double", "1.3"}, {"Shear Modulus XZ", "double", "1.4"}, {"Shear Modulus YZ", "double", "1.25"},
          {"Youngs Modulus X", "double", "1.0"}, {"Youngs Modulus Y", "double", "2.0"}, {"Youngs Modulus Z", "double", "3.0"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tMyMaterialModel.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_IsotropicLinearThermoelasticMatModel)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.performer("plato_analyze");
    tMaterial.category("isotropic linear thermoelastic");
    tMaterial.property("thermal conductivity coefficient", "1.0");
    tMaterial.property("youngs modulus", "2.3");
    tMaterial.property("poissons ratio", "0.3");
    tMaterial.property("thermal expansion coefficient", "0.4");
    tMaterial.property("reference temperature", "1.25");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tMaterialModelParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tMaterialModelParamList.empty());
    ASSERT_STREQ("ParameterList", tMaterialModelParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Material Model"}, tMaterialModelParamList);
    auto tMyMaterialModel = tMaterialModelParamList.child("Isotropic Linear Thermoelastic");
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Isotropic Linear Thermoelastic"}, tMyMaterialModel);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Poissons Ratio", "double", "0.3"}, {"Reference Temperature", "double", "1.25"}, {"Thermal Conductivity Coefficient", "double", "1.0"},
          {"Thermal Expansion Coefficient", "double", "0.4"}, {"Youngs Modulus", "double", "2.3"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tMyMaterialModel.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_IsotropicLinearElectroelasticMatModel)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    XMLGen::Material tMaterial;
    tMaterial.performer("plato_analyze");
    tMaterial.category("isotropic linear electroelastic");
    tMaterial.property("youngs modulus", "2.3");
    tMaterial.property("poissons ratio", "0.3");
    tMaterial.property("dielectric permittivity 11", "1.0");
    tMaterial.property("dielectric permittivity 33", "0.4");
    tMaterial.property("piezoelectric coupling 15", "1.25");
    tMaterial.property("piezoelectric coupling 33", "2.25");
    tMaterial.property("piezoelectric coupling 31", "3.25");
    tMaterial.property("thermal expansion", "0.25");
    tXMLMetaData.materials.push_back(tMaterial);
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tMaterialModelParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tMaterialModelParamList.empty());
    ASSERT_STREQ("ParameterList", tMaterialModelParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Material Model"}, tMaterialModelParamList);
    auto tMyMaterialModel = tMaterialModelParamList.child("Isotropic Linear Electroelastic");
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Isotropic Linear Electroelastic"}, tMyMaterialModel);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"p11", "double", "1.0"}, {"p33", "double", "0.4"}, {"e15", "double", "1.25"},
          {"e31", "double", "3.25"}, {"e33", "double", "2.25"}, {"Poissons Ratio", "double", "0.3"},
          {"Alpha", "double", "0.25"}, {"Youngs Modulus", "double", "2.3"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tMyMaterialModel.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendMaterialModelToPlatoAnalyzeInputDeck_RandomIsotropicLinearElasticMatModel)
{
    // POSE INPUTS
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.optimization_type = "topology";
    XMLGen::Objective tObjective;
    tObjective.mPerformerName = "plato_analyze";
    tXMLMetaData.objectives.push_back(tObjective);

    // POSE MATERIAL SET 1
    XMLGen::Material tMaterial1;
    tMaterial1.id("1");
    tMaterial1.category("isotropic linear elastic");
    tMaterial1.property("youngs modulus", "1");
    tMaterial1.property("poissons ratio", "0.3");
    XMLGen::MaterialSet tMaterialSetOne;
    tMaterialSetOne.insert({"1", tMaterial1});
    auto tRandomMaterialCaseOne = std::make_pair(0.5, tMaterialSetOne);

    // POSE MATERIAL SET 2
    XMLGen::Material tMaterial2;
    tMaterial2.id("1");
    tMaterial2.category("isotropic linear elastic");
    tMaterial2.property("youngs modulus", "1.1");
    tMaterial2.property("poissons ratio", "0.33");
    XMLGen::MaterialSet tMaterialSetTwo;
    tMaterialSetTwo.insert({"1", tMaterial2});
    auto tRandomMaterialCaseTwo = std::make_pair(0.5, tMaterialSetTwo);

    // CONSTRUCT SAMPLES SET
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseOne));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.append(tRandomMaterialCaseTwo));
    ASSERT_NO_THROW(tXMLMetaData.mRandomMetaData.finalize());

    // CALL FUNCTION WITH RANDOM MATERIAL
    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_material_model_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tMaterialModelParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tMaterialModelParamList.empty());
    ASSERT_STREQ("ParameterList", tMaterialModelParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Material Model"}, tMaterialModelParamList);
    auto tMyMaterialModel = tMaterialModelParamList.child("Isotropic Linear Elastic");
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Isotropic Linear Elastic"}, tMyMaterialModel);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Poissons Ratio", "double", "0.3"}, {"Youngs Modulus", "double", "1.0"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tMyMaterialModel.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendPartialDifferentialEquationToPlatoAnalyzeInputDeck_ErrorInvalidPhysics)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mPhysicsMetaData.mPhysics = "mechanical";
    ASSERT_NO_THROW(XMLGen::append_partial_differential_equation_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tPDE = tDocument.child("ParameterList");
    ASSERT_FALSE(tPDE.empty());
    ASSERT_STREQ("ParameterList", tPDE.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Elliptic"}, tPDE);

    auto tPenaltyFunc = tPDE.child("Penalty Function");
    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "SIMP"}, {"Exponent", "double", "3.0"}, {"Minimum Value", "double", "1e-9"} };
    auto tGoldValuesItr = tGoldValues.begin();
    auto tParameter = tPenaltyFunc.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendPartialDifferentialEquationToPlatoAnalyzeInputDeck)
{
    pugi::xml_document tDocument;
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mPhysicsMetaData.mPhysics = "computational fluid dynamics";
    ASSERT_THROW(XMLGen::append_partial_differential_equation_to_plato_analyze_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveParameterToPlatoProblem_DoNotAppendParameter)
{
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.run_mesh_name = "lbracket.exo";
    tXMLMetaData.mPhysicsMetaData.mPhysics = "mechanical";
    tXMLMetaData.mPhysicsMetaData.mSpatialDims = "2";
    XMLGen::Objective tObjective;
    tObjective.type = "maximize stiffness";
    tObjective.mPerformerName = "sierra";
    tXMLMetaData.objectives.push_back(tObjective);

    pugi::xml_document tDocument;
    XMLGen::append_objective_parameter_to_plato_problem(tXMLMetaData, tDocument);
    auto tParameter = tDocument.child("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendConstraintParameterToPlatoProblem_DoNotAppendParameter)
{
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.run_mesh_name = "lbracket.exo";
    tXMLMetaData.mPhysicsMetaData.mPhysics = "mechanical";
    tXMLMetaData.mPhysicsMetaData.mSpatialDims = "2";
    XMLGen::Constraint tConstraint;
    tConstraint.type = "maximize stiffness";
    tConstraint.mPerformerName = "sierra";
    tXMLMetaData.constraints.push_back(tConstraint);

    pugi::xml_document tDocument;
    XMLGen::append_constraint_parameter_to_plato_problem(tXMLMetaData, tDocument);
    auto tParameter = tDocument.child("Parameter");
    ASSERT_TRUE(tParameter.empty());
}

TEST(PlatoTestXMLGenerator, AppendSelfAdjointParameterToPlatoProblem_ErrorEmptyObjective)
{
    XMLGen::InputData tXMLMetaData;
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_self_adjoint_parameter_to_plato_problem(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendSelfAdjointParameterToPlatoProblem_ErrorInvalidCriterion)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::Objective tObjective;
    tObjective.type = "maximize thrust";
    tXMLMetaData.objectives.push_back(tObjective);
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_self_adjoint_parameter_to_plato_problem(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendPDEConstraintParameterToPlatoProblem_ErrorInvalidPhysics)
{
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mPhysicsMetaData.mPhysics = "computational fluid dynamics";
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_pde_constraint_parameter_to_plato_problem(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendPhysicsParameterToPlatoProblem_ErrorInvalidPhysics)
{
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mPhysicsMetaData.mPhysics = "computational fluid dynamics";
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_physics_parameter_to_plato_problem(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendProblemDescriptionToPlatoAnalyzeInputDeck_ErrorInvalidSpatialDim)
{
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mPhysicsMetaData.mSpatialDims = "1";
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_problem_description_to_plato_analyze_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendProblemDescriptionToPlatoAnalyzeInputDeck_ErrorEmptyMeshFile)
{
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.mPhysicsMetaData.mSpatialDims = "2";
    pugi::xml_document tDocument;
    ASSERT_THROW(XMLGen::append_problem_description_to_plato_analyze_input_deck(tXMLMetaData, tDocument), std::runtime_error);
}

TEST(PlatoTestXMLGenerator, AppendObjectiveCriteriaToPlatoAnalyzeInputDeck)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::Objective tObjective1;
    tObjective1.type = "maximize stiffness";
    tObjective1.mPerformerName = "plato_analyze";
    tXMLMetaData.objectives.push_back(tObjective1);
    XMLGen::Objective tObjective2;
    tObjective2.type = "volume";
    tObjective2.mPenaltyParam = "1.0";
    tObjective2.mMinimumErsatzValue = "0.0";
    tObjective2.mPerformerName = "plato_analyze";
    tXMLMetaData.objectives.push_back(tObjective2);

    pugi::xml_document tDocument;
    XMLGen::append_objective_criteria_to_plato_analyze_input_deck(tXMLMetaData, tDocument);

    // TEST MY OBJECTIVE
    auto tParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"My Objective"}, tParamList);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Weighted Sum"}, {"Functions", "Array(string)", "{my maximize stiffness, my volume}"}, {"Weights", "Array(double)", "{1.0, 1.0}"} };
    auto tGoldValuesItr = tGoldValues.begin();

    auto tChild = tParamList.child("Parameter");
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ("Parameter", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }

    // TEST MY OBJECTIVE 1 - 'my maximize stiffness'
    tParamList = tParamList.next_sibling("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"my maximize stiffness"}, tParamList);
    tGoldValues = { {"Type", "string", "Scalar Function"}, {"Scalar Function Type", "string", "Internal Elastic Energy"}, {} };
    tGoldValuesItr = tGoldValues.begin();

    tChild = tParamList.child("Parameter");
    std::vector<std::string> tGoldChildName = {"Parameter", "Parameter", "Penalty Function"};
    auto tGoldChildItr = tGoldChildName.begin();
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ(tGoldChildItr->c_str(), tChild.name());
        if (tGoldChildItr->compare("Parameter") == 0)
        {
            // TEST PARAMETER CHILDREN, SKIP PENALTY FUNCTION CHILDREN (TEST BELOW)
            PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        }
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
        std::advance(tGoldChildItr, 1);
    }

    auto tPenaltyModel = tParamList.child("Penalty Function");
    tGoldValues = { {"Type", "string", "SIMP"}, {"Exponent", "double", "3.0"}, {"Minimum Value", "double", "1e-9"} };
    tGoldValuesItr = tGoldValues.begin();
    tChild = tPenaltyModel.child("Parameter");
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ("Parameter", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }

    // TEST MY OBJECTIVE 2 - 'my volume'
    tParamList = tParamList.next_sibling("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"my volume"}, tParamList);
    tGoldValues = { {"Type", "string", "Scalar Function"}, {"Scalar Function Type", "string", "Volume"}, {} };
    tGoldValuesItr = tGoldValues.begin();

    tChild = tParamList.child("Parameter");
    tGoldChildItr = tGoldChildName.begin();
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ(tGoldChildItr->c_str(), tChild.name());
        if (tGoldChildItr->compare("Parameter") == 0)
        {
            // TEST PARAMETER CHILDREN, SKIP PENALTY FUNCTION CHILDREN (TEST BELOW)
            PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        }
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
        std::advance(tGoldChildItr, 1);
    }

    tPenaltyModel = tParamList.child("Penalty Function");
    tGoldValues = { {"Type", "string", "SIMP"}, {"Exponent", "double", "1.0"}, {"Minimum Value", "double", "0.0"} };
    tGoldValuesItr = tGoldValues.begin();
    tChild = tPenaltyModel.child("Parameter");
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ("Parameter", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendConstraintCriteriaToPlatoAnalyzeInputDeck)
{
    XMLGen::InputData tXMLMetaData;
    XMLGen::Constraint tConstraint;
    tConstraint.type = "stress p-norm";
    tConstraint.weight = "0.5";
    tConstraint.mPerformerName = "plato_analyze";
    tXMLMetaData.constraints.push_back(tConstraint);

    pugi::xml_document tDocument;
    XMLGen::append_constraint_criteria_to_plato_analyze_input_deck(tXMLMetaData, tDocument);

    // TEST MY CONSTRAINT
    auto tParamList = tDocument.child("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"My Constraint"}, tParamList);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Type", "string", "Weighted Sum"}, {"Functions", "Array(string)", "{my stress p-norm}"},
          {"Weights", "Array(double)", "{0.5}"}, {"Exponent", "double", "6"} };
    auto tGoldValuesItr = tGoldValues.begin();

    auto tChild = tParamList.child("Parameter");
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ("Parameter", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }

    // TEST MY OBJECTIVE 1 - 'my stress p-norm'
    tParamList = tParamList.next_sibling("ParameterList");
    ASSERT_FALSE(tParamList.empty());
    ASSERT_STREQ("ParameterList", tParamList.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"my stress p-norm"}, tParamList);
    tGoldValues = { {"Type", "string", "Scalar Function"}, {"Scalar Function Type", "string", "Stress P-Norm"}, {} };
    tGoldValuesItr = tGoldValues.begin();

    tChild = tParamList.child("Parameter");
    std::vector<std::string> tGoldChildName = {"Parameter", "Parameter", "Penalty Function"};
    auto tGoldChildItr = tGoldChildName.begin();
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ(tGoldChildItr->c_str(), tChild.name());
        if (tGoldChildItr->compare("Parameter") == 0)
        {
            // TEST PARAMETER CHILDREN, SKIP PENALTY FUNCTION CHILDREN (TEST BELOW)
            PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        }
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
        std::advance(tGoldChildItr, 1);
    }

    auto tPenaltyModel = tParamList.child("Penalty Function");
    tGoldValues = { {"Type", "string", "SIMP"}, {"Exponent", "double", "3.0"}, {"Minimum Value", "double", "1e-9"} };
    tGoldValuesItr = tGoldValues.begin();
    tChild = tPenaltyModel.child("Parameter");
    while(!tChild.empty())
    {
        ASSERT_FALSE(tChild.empty());
        ASSERT_STREQ("Parameter", tChild.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tChild);
        tChild = tChild.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendProblemDescriptionToPlatoAnalyzeInputDeck)
{
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.run_mesh_name = "lbracket.exo";
    tXMLMetaData.mPhysicsMetaData.mSpatialDims = "2";

    pugi::xml_document tDocument;
    ASSERT_NO_THROW(XMLGen::append_problem_description_to_plato_analyze_input_deck(tXMLMetaData, tDocument));

    auto tProblem = tDocument.child("ParameterList");
    ASSERT_FALSE(tProblem.empty());
    ASSERT_STREQ("ParameterList", tProblem.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Problem"}, tProblem);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Physics", "string", "Plato Driver"}, {"Spatial Dimension", "int", "2"}, {"Input Mesh", "string", "lbracket.exo"} };
    auto tGoldValuesItr = tGoldValues.begin();

    auto tParameter = tProblem.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

TEST(PlatoTestXMLGenerator, AppendPlatoProblemToPlatoAnalyzeInputDeck)
{
    XMLGen::InputData tXMLMetaData;
    tXMLMetaData.run_mesh_name = "lbracket.exo";
    tXMLMetaData.mPhysicsMetaData.mPhysics = "mechanical";
    tXMLMetaData.mPhysicsMetaData.mSpatialDims = "2";
    XMLGen::Objective tObjective;
    tObjective.type = "maximize stiffness";
    tObjective.mPerformerName = "plato_analyze";
    tXMLMetaData.objectives.push_back(tObjective);
    XMLGen::Constraint tConstraint;
    tConstraint.type = "volume";
    tConstraint.mPerformerName = "plato_analyze";
    tXMLMetaData.constraints.push_back(tConstraint);

    pugi::xml_document tDocument;
    auto tProblem = tDocument.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"Problem"}, tProblem);
    XMLGen::append_plato_problem_description_to_plato_analyze_input_deck(tXMLMetaData, tDocument);

    tProblem = tDocument.child("ParameterList");
    ASSERT_FALSE(tProblem.empty());
    ASSERT_STREQ("ParameterList", tProblem.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Problem"}, tProblem);

    auto tPlatoProblem = tProblem.child("ParameterList");
    ASSERT_FALSE(tPlatoProblem.empty());
    ASSERT_STREQ("ParameterList", tPlatoProblem.name());
    PlatoTestXMLGenerator::test_attributes({"name"}, {"Plato Problem"}, tPlatoProblem);

    std::vector<std::string> tGoldKeys = {"name", "type", "value"};
    std::vector<std::vector<std::string>> tGoldValues =
        { {"Physics", "string", "Mechanical"}, {"PDE Constraint", "string", "Elliptic"}, {"Constraint", "string", "My Constraint"},
          {"Objective", "string", "My Objective"}, {"Self-Adjoint", "bool", "true"} };
    auto tGoldValuesItr = tGoldValues.begin();

    auto tParameter = tPlatoProblem.child("Parameter");
    while(!tParameter.empty())
    {
        ASSERT_FALSE(tParameter.empty());
        ASSERT_STREQ("Parameter", tParameter.name());
        PlatoTestXMLGenerator::test_attributes(tGoldKeys, tGoldValuesItr.operator*(), tParameter);
        tParameter = tParameter.next_sibling();
        std::advance(tGoldValuesItr, 1);
    }
}

}
// namespace PlatoTestXMLGenerator
