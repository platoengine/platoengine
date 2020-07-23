/*
 * XMLGeneratorPhysicsFunctionInterface.cpp
 *
 *  Created on: Jul 23, 2020
 */

#include "XMLG_Macros.hpp"
#include "XMLGeneratorUtilities.hpp"
#include "XMLGeneratorValidInputKeys.hpp"
#include "XMLGeneratorPhysicsFunctionInterface.hpp"

namespace XMLGen
{

namespace Private
{

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

inline void append_pseudo_time_step_option
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
    tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.solverTolerance());
    tValues = { "Stopping Tolerance", "double", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepping);
    tPropertyValue = XMLGen::set_value_keyword_to_ignore_if_empty(aMetadata.solverMaxNumIterations());
    tValues = { "Maximum Number Iterations", "int", tPropertyValue };
    XMLGen::append_parameter_plus_attributes(tKeys, tValues, tTimeStepping);
}

inline void append_plottable_option
(const XMLGen::Output& aOutput,
 pugi::xml_node &aParentNode)
{
    auto tQoiIDs = aOutput.deterministicIDs();
    if(tQoiIDs.empty())
    {
        return;
    }

    auto tTransformQoiIDs = XMLGen::transform_tokens(tQoiIDs);
    tTransformQoiIDs.insert(0u, "{");
    tTransformQoiIDs.insert(tTransformQoiIDs.size(), "}");
    std::vector<std::string> tKeys = {"name", "type", "value"};
    std::vector<std::string> tValues = {"Plottable", "Array(string)", tTransformQoiIDs};
    auto tParameter = aParentNode.append_child("Parameter");
    XMLGen::append_attributes(tKeys, tValues, tParameter);
}

inline void append_state_gradient_projection_residual_to_analyze_input_deck
(const XMLGen::Scenario& aScenario,
 pugi::xml_node &aParentNode)
{
    auto tPhysicsNode = aParentNode.append_child("ParameterList");
    XMLGen::append_attributes({"name"}, {"State Gradient Projection"}, tPhysicsNode);
    XMLGen::Private::append_simp_penalty_function(aScenario, tPhysicsNode);
}

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

inline void append_plasticity_residual_to_analyze_input_deck
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
    XMLGen::Private::append_pseudo_time_step_option(aScenario, aParentNode);
    XMLGen::Private::append_newton_raphson_solver_option(aScenario, aParentNode);
}

}
// namespace Private

void PhysicsFunctionInterface::insertEllipticPhysics()
{
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_elliptic_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("mechanical",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_elliptic_pde_to_analyze_input_deck, tFuncIndex)));

    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_elliptic_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("thermal",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_elliptic_pde_to_analyze_input_deck, tFuncIndex)));

    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_elliptic_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("thermomechanical",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_elliptic_pde_to_analyze_input_deck, tFuncIndex)));

    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_elliptic_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("electromechanical",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_elliptic_pde_to_analyze_input_deck, tFuncIndex)));
}

void PhysicsFunctionInterface::insertParabolicPhysics()
{
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_parabolic_residual_to_analyze_input_deck));
    mMap.insert(std::make_pair("heat conduction",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_parabolic_residual_to_analyze_input_deck, tFuncIndex)));

    tFuncIndex = std::type_index(typeid(XMLGen::Private::append_parabolic_residual_to_analyze_input_deck));
    mMap.insert(std::make_pair("transient thermomechanics",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_parabolic_residual_to_analyze_input_deck, tFuncIndex)));
}

void PhysicsFunctionInterface::insertHyperbolicPhysics()
{
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_hyperbolic_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("transient mechanics",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_hyperbolic_pde_to_analyze_input_deck, tFuncIndex)));
}

void PhysicsFunctionInterface::insertPlasticityPhysics()
{
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_plasticity_residual_to_analyze_input_deck));
    mMap.insert(std::make_pair("plasticity",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_plasticity_residual_to_analyze_input_deck, tFuncIndex)));
}

void PhysicsFunctionInterface::insertStabilizedEllipticPhysics()
{
    auto tFuncIndex = std::type_index(typeid(XMLGen::Private::append_stabilized_elliptic_pde_to_analyze_input_deck));
    mMap.insert(std::make_pair("stabilized mechanics",
      std::make_pair((XMLGen::Analyze::MaterialModelFunc)XMLGen::Private::append_stabilized_elliptic_pde_to_analyze_input_deck, tFuncIndex)));
}

void PhysicsFunctionInterface::insert()
{
    this->insertEllipticPhysics();
    this->insertParabolicPhysics();
    this->insertHyperbolicPhysics();
    this->insertPlasticityPhysics();
    this->insertStabilizedEllipticPhysics();
}

PhysicsFunctionInterface::PhysicsFunctionInterface()
{
    this->insert();
}

void PhysicsFunctionInterface::call(const XMLGen::Scenario& aScenarioMetaData,
          const XMLGen::Output& aOutputMetaData,
          pugi::xml_node &aParentNode) const
{
    auto tCode = XMLGen::to_lower(aScenarioMetaData.code());
    if(tCode.compare("plato_analyze") != 0)
    {
        return;
    }

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
