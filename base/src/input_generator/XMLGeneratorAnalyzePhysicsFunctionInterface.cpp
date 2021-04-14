/*
 * XMLGeneratorAnalyzePhysicsFunctionInterface.cpp
 *
 *  Created on: Jul 23, 2020
 */

#include "XMLG_Macros.hpp"
#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorAnalyzePhysicsFunctionInterface.hpp"

namespace XMLGen
{

namespace Private
{

/******************************************************************************//**
 * \fn append_simp_penalty_function
 * \brief Append Solid Isotropic Material with Penalization function inputs.
 * \param [in]     aMetadata    Service metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_simp_penalty_function
(const XMLGen::Scenario& aMetadata,
 pugi::xml_node& aParentNode)
{
    auto tPenaltyFunction = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes( { "name" }, { "Penalty Function" }, tPenaltyFunction);
    std::vector<std::string> tKeys = { "name", "type", "value" };
    std::vector<std::string> tValues = { "Type", "string", "SIMP" };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tPenaltyFunction);

    auto tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.materialPenaltyExponent());
    tValues = { "Exponent", "double", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tPenaltyFunction);
    tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.minErsatzMaterialConstant());
    tValues = { "Minimum Value", "double", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tPenaltyFunction);
    tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.additiveContinuation());
    tValues = { "Additive Continuation", "double", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tPenaltyFunction);
}

/******************************************************************************//**
 * \fn append_parabolic_pde_time_step_option
 * \brief Append parabolic partial differential equation (PDE) time step
 * integration inputs.
 * \param [in]     aMetadata    Scenario metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_parabolic_pde_time_step_option
(const XMLGen::Scenario& aMetadata,
 pugi::xml_node& aParentNode)
{
    auto tTimeStepNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes( { "name" }, { "Time Integration" }, tTimeStepNode);
    std::vector<std::string> tKeys = { "name", "type", "value" };
    auto tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.numTimeSteps());
    std::vector<std::string> tValues = { "Number Time Steps", "int", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepNode);
    tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.timeStep());
    tValues = { "Time Step", "double", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepNode);
}

/******************************************************************************//**
 * \fn append_hyperbolic_pde_time_step_option
 * \brief Append hyperbolic parabolic partial differential equation (PDE) time
 * step integration inputs.
 * \param [in]     aMetadata    Scenario metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_hyperbolic_pde_time_step_option
(const XMLGen::Scenario& aMetadata,
 pugi::xml_node& aParentNode)
{
    auto tTimeStepNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes( { "name" }, { "Time Integration" }, tTimeStepNode);
    std::vector<std::string> tKeys = { "name", "type", "value" };
    auto tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.numTimeSteps());
    std::vector<std::string> tValues = { "Number Time Steps", "int", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepNode);
    tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.timeStep());
    tValues = { "Time Step", "double", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepNode);
    tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.newmarkBeta());
    tValues = { "Newmark Beta", "double", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepNode);
    tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.newmarkGamma());
    tValues = { "Newmark Gamma", "double", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepNode);
}

/******************************************************************************//**
 * \fn append_stabilized_elliptic_pde_time_step_option
 * \brief Append stabilized elliptic partial differential equation (PDE) time
 * stepping routine inputs.
 * \param [in]     aMetadata    Scenario metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_stabilized_elliptic_pde_time_step_option
(const XMLGen::Scenario& aMetadata,
 pugi::xml_node& aParentNode)
{
    auto tTimeStepNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes( { "name" }, { "Time Stepping" }, tTimeStepNode);
    std::vector<std::string> tKeys = { "name", "type", "value" };
    auto tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.numTimeSteps());
    std::vector<std::string> tValues = { "Number Time Steps", "int", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepNode);
    tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.timeStep());
    tValues = { "Time Step", "double", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepNode);
}

/******************************************************************************//**
 * \fn append_pseudo_time_step_option
 * \brief Append plasticity physics pseudo time step routine inputs.
 * \param [in]     aMetadata    Scenario metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_plasticity_pseudo_time_step_option
(const XMLGen::Scenario& aMetadata,
 pugi::xml_node& aParentNode)
{
    auto tTimeStepNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes( { "name" }, { "Time Stepping" }, tTimeStepNode);
    std::vector<std::string> tKeys = { "name", "type", "value" };
    auto tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.timeStepExpansion());
    std::vector<std::string> tValues = { "Expansion Multiplier", "double", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepNode);
    tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.numTimeSteps());
    tValues = { "Initial Num. Pseudo Time Steps", "int", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepNode);
    tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.maxNumTimeSteps());
    tValues = { "Maximum Num. Pseudo Time Steps", "int", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepNode);
}

/******************************************************************************//**
 * \fn append_stabilized_elliptic_newton_solver_option
 * \brief Append stabilized elliptic Newton solver inputs.
 * \param [in]     aMetadata    Scenario metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_stabilized_elliptic_newton_solver_option
(const XMLGen::Scenario& aMetadata,
 pugi::xml_node& aParentNode)
{
    auto tTimeStepping = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes( { "name" }, { "Newton Iteration" }, tTimeStepping);
    std::vector<std::string> tKeys = { "name", "type", "value" };
    auto tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.solverMaxNumIterations());
    std::vector<std::string> tValues = { "Number Iterations", "int", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepping);
}

/******************************************************************************//**
 * \fn append_newton_raphson_solver_option
 * \brief Append Newton solver inputs (e.g. plasticity use case).
 * \param [in]     aMetadata    Scenario metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_newton_raphson_solver_option
(const XMLGen::Scenario& aMetadata,
 pugi::xml_node& aParentNode)
{
    auto tTimeStepping = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes( { "name" }, { "Newton-Raphson" }, tTimeStepping);
    std::vector<std::string> tKeys = { "name", "type", "value" };
    auto tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.solverConvergenceCriterion());
    std::vector<std::string> tValues = { "Stop Measure", "string", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepping);
    tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.newtonSolverTolerance());
    tValues = { "Stopping Tolerance", "double", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepping);
    tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.solverMaxNumIterations());
    tValues = { "Maximum Number Iterations", "int", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepping);
}

/******************************************************************************//**
 * \fn transform_analyze_output_keywords
 * \brief Return valid output keywords for Plato Analyze.
 * \param [in] aKeywords list of output keywords parse from the plato input deck
 * \return valid output keywords for Plato Analyze
**********************************************************************************/
inline std::vector<std::string>
transform_analyze_output_keywords
(const std::vector<std::string>& aKeywords)
{
    std::vector<std::string> tOutput;
    XMLGen::ValidAnalyzeOutputKeys tValidKeys;
    for(auto& tKey : aKeywords)
    {
        auto tValue = tValidKeys.value(tKey);
        if(!tValue.empty())
        {
            tOutput.push_back(tValue);
        }
    }
    return tOutput;
}

/******************************************************************************//**
 * \fn append_plottable_option
 * \brief Append plottable output keywords.
 * \param [in]     aOutput      Output metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_plottable_option
(const XMLGen::Output& aOutput,
 pugi::xml_node &aParentNode)
{
    auto tOutputQoIs = aOutput.outputIDs();
    if(tOutputQoIs.empty())
    {
        return;
    }

    auto tValidAnalyzeOutputKeywords = XMLGen::Private::transform_analyze_output_keywords(tOutputQoIs);
    auto tTransformQoIIDs = XMLGen::transform_tokens(tValidAnalyzeOutputKeywords);
    tTransformQoIIDs.insert(0u, "{");
    tTransformQoIIDs.insert(tTransformQoIIDs.size(), "}");
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Plottable", "Array(string)", tTransformQoIIDs};
    auto tParameter = aParentNode.append_child("Parameter");
    XMLGen::append_attributes(tKeys, tValues, tParameter);
}

/******************************************************************************//**
 * \fn append_state_gradient_projection_residual_to_analyze_input_deck
 * \brief Append state gradient projection residual inputs
 * \param [in]     aMetadata    Scenario metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_state_gradient_projection_residual_to_analyze_input_deck
(const XMLGen::Scenario& aMetadata,
 pugi::xml_node &aParentNode)
{
    auto tPhysicsNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"State Gradient Projection"}, tPhysicsNode);
    XMLGen::Private::append_simp_penalty_function(aMetadata, tPhysicsNode);
}

/******************************************************************************//**
 * \fn append_elliptic_pde_to_analyze_input_deck
 * \brief Append elliptic partial differential equation (PDE) inputs.
 * \param [in]     aScenario    Scenario metadata
 * \param [in]     aOutput      Output metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_elliptic_pde_to_analyze_input_deck
(const XMLGen::Scenario& aScenario,
 const XMLGen::Output& aOutput,
 pugi::xml_node &aParentNode)
{
    auto tPhysicsTag = aScenario.physics();
    XMLGen::ValidAnalyzePhysicsKeys tValidKeys;
    auto tPDECategory = tValidKeys.pde(tPhysicsTag);
    auto tPhysicsNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {tPDECategory}, tPhysicsNode);
    XMLGen::Private::append_plottable_option(aOutput, tPhysicsNode);
    XMLGen::Private::append_simp_penalty_function(aScenario, tPhysicsNode);
}

/******************************************************************************//**
 * \fn append_hyperbolic_pde_to_analyze_input_deck
 * \brief Append hyperbolic partial differential equation (PDE) inputs.
 * \param [in]     aScenario    Scenario metadata
 * \param [in]     aOutput      Output metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_hyperbolic_pde_to_analyze_input_deck
(const XMLGen::Scenario& aScenario,
 const XMLGen::Output& aOutput,
 pugi::xml_node &aParentNode)
{
    auto tPhysicsTag = aScenario.physics();
    XMLGen::ValidAnalyzePhysicsKeys tValidKeys;
    auto tPDECategory = tValidKeys.pde(tPhysicsTag);
    auto tPhysicsNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {tPDECategory}, tPhysicsNode);
    XMLGen::Private::append_plottable_option(aOutput, tPhysicsNode);
    XMLGen::Private::append_simp_penalty_function(aScenario, tPhysicsNode);
    XMLGen::Private::append_hyperbolic_pde_time_step_option(aScenario, aParentNode);
}

/******************************************************************************//**
 * \fn append_stabilized_elliptic_pde_to_analyze_input_deck
 * \brief Append stabilized elliptic partial differential equation (PDE) inputs.
 * \param [in]     aScenario    Scenario metadata
 * \param [in]     aOutput      Output metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_stabilized_elliptic_pde_to_analyze_input_deck
(const XMLGen::Scenario& aScenario,
 const XMLGen::Output& aOutput,
 pugi::xml_node &aParentNode)
{
    auto tPhysicsTag = aScenario.physics();
    XMLGen::ValidAnalyzePhysicsKeys tValidKeys;
    auto tPDECategory = tValidKeys.pde(tPhysicsTag);
    auto tPhysicsNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {tPDECategory}, tPhysicsNode);
    XMLGen::Private::append_plottable_option(aOutput, tPhysicsNode);
    XMLGen::Private::append_simp_penalty_function(aScenario, tPhysicsNode);
    XMLGen::Private::append_stabilized_elliptic_pde_time_step_option(aScenario, aParentNode);
    XMLGen::Private::append_stabilized_elliptic_newton_solver_option(aScenario, aParentNode);
}

/******************************************************************************//**
 * \fn append_parabolic_residual_to_analyze_input_deck
 * \brief Append parabolic partial differential equation (PDE) inputs.
 * \param [in]     aScenario    Scenario metadata
 * \param [in]     aOutput      Output metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_parabolic_residual_to_analyze_input_deck
(const XMLGen::Scenario& aScenario,
 const XMLGen::Output& aOutput,
 pugi::xml_node &aParentNode)
{
    auto tPhysicsTag = aScenario.physics();
    XMLGen::ValidAnalyzePhysicsKeys tValidKeys;
    auto tPDECategory = tValidKeys.pde(tPhysicsTag);
    auto tPhysicsNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {tPDECategory}, tPhysicsNode);
    XMLGen::Private::append_plottable_option(aOutput, tPhysicsNode);
    XMLGen::Private::append_simp_penalty_function(aScenario, tPhysicsNode);
    XMLGen::Private::append_parabolic_pde_time_step_option(aScenario, aParentNode);
}

/******************************************************************************//**
 * \fn append_elliptic_plasticity_pde_to_analyze_input_deck
 * \brief Append elliptic plasticity partial differential equation (PDE) inputs.
 * \param [in]     aScenario    Scenario metadata
 * \param [in]     aOutput      Output metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_elliptic_plasticity_pde_to_analyze_input_deck
(const XMLGen::Scenario& aScenario,
 const XMLGen::Output& aOutput,
 pugi::xml_node &aParentNode)
{
    auto tPhysicsTag = aScenario.physics();
    XMLGen::ValidAnalyzePhysicsKeys tValidKeys;
    auto tPDECategory = tValidKeys.pde(tPhysicsTag);
    auto tPhysicsNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {tPDECategory}, tPhysicsNode);
    XMLGen::Private::append_plottable_option(aOutput, tPhysicsNode);
    XMLGen::Private::append_simp_penalty_function(aScenario, tPhysicsNode);
    XMLGen::Private::append_state_gradient_projection_residual_to_analyze_input_deck(aScenario, aParentNode);
    XMLGen::Private::append_plasticity_pseudo_time_step_option(aScenario, aParentNode);
    XMLGen::Private::append_newton_raphson_solver_option(aScenario, aParentNode);
}

/******************************************************************************//**
 * \fn append_elliptic_thermoplasticity_pde_to_analyze_input_deck
 * \brief Append elliptic thermoplasticity partial differential equation (PDE) inputs.
 * \param [in]     aScenario    Scenario metadata
 * \param [in]     aOutput      Output metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
inline void append_elliptic_thermoplasticity_pde_to_analyze_input_deck
(const XMLGen::Scenario& aScenario,
 const XMLGen::Output& aOutput,
 pugi::xml_node &aParentNode)
{
    auto tPhysicsTag = aScenario.physics();
    XMLGen::ValidAnalyzePhysicsKeys tValidKeys;
    auto tPDECategory = tValidKeys.pde(tPhysicsTag);
    auto tPhysicsNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {tPDECategory}, tPhysicsNode);
    XMLGen::Private::append_plottable_option(aOutput, tPhysicsNode);
    XMLGen::Private::append_simp_penalty_function(aScenario, tPhysicsNode);
    XMLGen::Private::append_state_gradient_projection_residual_to_analyze_input_deck(aScenario, aParentNode);
    XMLGen::Private::append_plasticity_pseudo_time_step_option(aScenario, aParentNode);
    XMLGen::Private::append_newton_raphson_solver_option(aScenario, aParentNode);
}

}
// namespace Private

void AnalyzePhysicsFunctionInterface::insertEllipticPhysics()
{
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_elliptic_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("steady_state_mechanics",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_elliptic_pde_to_analyze_input_deck, tFuncIndex)));

    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_elliptic_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("steady_state_thermal",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_elliptic_pde_to_analyze_input_deck, tFuncIndex)));

    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_elliptic_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("steady_state_thermomechanics",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_elliptic_pde_to_analyze_input_deck, tFuncIndex)));

    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_elliptic_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("steady_state_electromechanics",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_elliptic_pde_to_analyze_input_deck, tFuncIndex)));
}

void AnalyzePhysicsFunctionInterface::insertParabolicPhysics()
{
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_parabolic_residual_to_analyze_input_deck));
    mMap.insert(std::make_pair("transient_thermal",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_parabolic_residual_to_analyze_input_deck, tFuncIndex)));

    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_parabolic_residual_to_analyze_input_deck));
    mMap.insert(std::make_pair("transient_thermomechanics",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_parabolic_residual_to_analyze_input_deck, tFuncIndex)));
}

void AnalyzePhysicsFunctionInterface::insertHyperbolicPhysics()
{
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_hyperbolic_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("transient_mechanics",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_hyperbolic_pde_to_analyze_input_deck, tFuncIndex)));
}

void AnalyzePhysicsFunctionInterface::insertPlasticityPhysics()
{
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_elliptic_plasticity_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("plasticity",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_elliptic_plasticity_pde_to_analyze_input_deck, tFuncIndex)));

    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_elliptic_thermoplasticity_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("thermoplasticity",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_elliptic_thermoplasticity_pde_to_analyze_input_deck, tFuncIndex)));
}

void AnalyzePhysicsFunctionInterface::insertStabilizedEllipticPhysics()
{
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_stabilized_elliptic_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("stabilized_mechanics",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_stabilized_elliptic_pde_to_analyze_input_deck, tFuncIndex)));
}

void AnalyzePhysicsFunctionInterface::insert()
{
    this->insertEllipticPhysics();
    this->insertParabolicPhysics();
    this->insertHyperbolicPhysics();
    this->insertPlasticityPhysics();
    this->insertStabilizedEllipticPhysics();
}

AnalyzePhysicsFunctionInterface::AnalyzePhysicsFunctionInterface()
{
    this->insert();
}

void AnalyzePhysicsFunctionInterface::call(const XMLGen::Scenario& aScenarioMetaData,
          const XMLGen::Output& aOutputMetaData,
          pugi::xml_node &aParentNode) const
{
/* Scenario doesn't have a code member and we should only be getting in here for plato_analyze anyway.
    auto tCode = XMLGen::to_lower(aScenarioMetaData.code());
    if(tCode.compare("plato_analyze") != 0)
    {
        return;
    }
*/

    auto tPhysics = XMLGen::to_lower(aScenarioMetaData.physics());
    auto tMapItr = mMap.find(tPhysics);
    if(tMapItr == mMap.end())
    {
        THROWERR(std::string("Physics Function Interface: Did not find physics function with tag '") + tPhysics
            + "' in function list. " + "Physics '" + tPhysics + "' is not supported in Plato Analyze.")
    }
    auto tTypeCastedFunc = reinterpret_cast<void(*)(const XMLGen::Scenario&, const XMLGen::Output&, pugi::xml_node&)>(tMapItr->second.first);
    if(tMapItr->second.second == std::type_index(typeid(tTypeCastedFunc)))
    {
        THROWERR(std::string("Physics Function Interface: Reinterpret cast for physics function with tag '") + tPhysics + "' failed.")
    }
    tTypeCastedFunc(aScenarioMetaData, aOutputMetaData, aParentNode);
}

}
// namespace XMLGen
