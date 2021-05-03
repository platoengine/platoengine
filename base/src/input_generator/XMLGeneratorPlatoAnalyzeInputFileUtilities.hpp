/*
 * XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp
 *
 *  Created on: Jun 15, 2020
 */

#pragma once

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \fn check_input_mesh_file_keyword
 * \brief Check if input filename keyword is defined, i.e. is not empty.
 * \param [in] aXMLMetaData Plato problem input data
**********************************************************************************/
void check_input_mesh_file_keyword
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn is_objective_container_empty
 * \brief Check if objective list is defined, i.e. is not empty.
 * \param [in] aXMLMetaData Plato problem input data
**********************************************************************************/
void is_objective_container_empty
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn return_list_of_objective_functions
 * \brief Return list of objective functions to be used in optimization problem.
 * \param [in] aXMLMetaData Plato problem input data
**********************************************************************************/
std::vector<std::string> return_list_of_objective_functions
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn return_list_of_objective_weights
 * \brief Return list of objective function weights to be used in optimization problem.
 * \param [in] aXMLMetaData Plato problem input data
**********************************************************************************/
std::vector<std::string> return_list_of_objective_weights
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn return_list_of_constraint_functions
 * \brief Return list of constraint functions to be used in optimization problem.
 * \param [in] aXMLMetaData Plato problem input data
**********************************************************************************/
std::vector<std::string> return_list_of_constraint_functions
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn return_list_of_constraint_weights
 * \brief Return list of constraint function weights to be used in optimization problem.
 * \param [in] aXMLMetaData Plato problem input data
**********************************************************************************/
std::vector<std::string> return_list_of_constraint_weights
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn transform_tokens_for_plato_analyze_input_deck
 * \brief Transform token list into a single string that can be appended to plato \n
 * analyze's input deck.
 * \param [in] aTokens token list
**********************************************************************************/
std::string transform_tokens_for_plato_analyze_input_deck
(const std::vector<std::string> &aTokens);

/******************************************************************************//**
 * \fn append_problem_description_to_plato_analyze_input_deck
 * \brief Append plato analyze's problem description to input deck.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_problem_description_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_plato_problem_description_to_plato_analyze_input_deck
 * \brief Append plato analyze's plato problem description to input deck.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_plato_problem_description_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_plato_problem_to_plato_analyze_input_deck
 * \brief Append plato problem to input deck.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_plato_problem_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_physics_parameter_to_plato_problem
 * \brief Append plato analyze's physics parameter to plato problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_physics_parameter_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_pde_constraint_parameter_to_plato_problem
 * \brief Append plato analyze's partial differential equation (PDE) parameter \n
 * to plato problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_pde_constraint_parameter_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_self_adjoint_parameter_to_plato_problem
 * \brief Append plato analyze's self adjoint parameter to plato problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_self_adjoint_parameter_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn get_ebc_vector_for_scenario
 * \brief Utility function to retrieve the ebcs used in a scenario
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in]     aScenario The scenario of interest
 * \param [out]    aEBCVector  EBC vector to populate
**********************************************************************************/
void get_ebc_vector_for_scenario
(const XMLGen::InputData& aXMLMetaData,
 const XMLGen::Scenario &aScenario,
 std::vector<XMLGen::EssentialBoundaryCondition> &aEBCVector);

/******************************************************************************//**
 * \fn append_weighted_sum_objective_to_plato_problem
 * \brief Append weighted sum objective function to plato problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_weighted_sum_objective_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_functions_to_weighted_sum_objective
 * \brief Append criterion function parameters to weighted sum objective parameter \n
 * list inside the plato problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in]     aObjectiveFunctions list of objective criterion function names
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_functions_to_weighted_sum_objective
(const XMLGen::InputData& aXMLMetaData,
 const std::vector<std::string> &aObjectiveFunctions,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_weights_to_weighted_sum_objective
 * \brief Append criteria weights to weighted sum objective parameter list inside \n
 * the plato problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_weights_to_weighted_sum_objective
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_objective_criteria_to_plato_problem
 * \brief Append objective criterion parameters to plato problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
pugi::xml_node append_objective_criteria_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_objective_criteria_to_criteria_list
 * \brief Append objective criteria to criteria parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_objective_criteria_to_criteria_list
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_criteria_list_to_plato_analyze_input_deck
 * \brief Append criteria to problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_criteria_list_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_constraint_criteria_to_plato_problem
 * \brief Append constraint criteria to plato problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
pugi::xml_node append_constraint_criteria_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_weighted_sum_constraint_to_plato_problem
 * \brief Append weighted sum constraint function to plato problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_weighted_sum_constraint_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_functions_to_weighted_sum_constraint
 * \brief Append criterion function parameters to weighted sum constraint parameter \n
 * list inside the plato problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in]     aConstraintFunctions list of contraint functions
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_functions_to_weighted_sum_constraint
(const XMLGen::InputData& aXMLMetaData,
 const std::vector<std::string> &aConstraintFunctions,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_weights_to_weighted_sum_constraint
 * \brief Append criteria weights to weighted sum constraint parameter list inside \n
 * the plato problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_weights_to_weighted_sum_constraint
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_constraint_criteria_to_criteria_list
 * \brief Append constraint criteria to criteria parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_constraint_criteria_to_criteria_list
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_physics_to_plato_analyze_input_deck
 * \brief Append partial differential equation (pde) to problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_physics_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_spatial_model_to_plato_problem
 * \brief Append spatial model to plato problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_spatial_model_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_material_model_to_plato_problem
 * \brief Append material model to plato problem parameter list.
 * \param [in]     aMaterials List of materials
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_material_model_to_plato_problem
(const std::vector<XMLGen::Material>& aMaterials,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_pressure_and_temperature_scaling_to_material_models
 * \brief Append material model to plato problem parameter list.
 * \param [in]     aXMLMetaData     Plato problem input data
 * \param [in/out] aMaterialModels  pugi::xml_node
**********************************************************************************/
void append_pressure_and_temperature_scaling_to_material_models
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aMaterialModels);

/******************************************************************************//**
 * \fn append_material_models_to_plato_analyze_input_deck
 * \brief Append material models to problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_material_models_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_spatial_model_to_plato_analyze_input_deck
 * \brief Append spatial model to problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_spatial_model_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_material_model_to_plato_analyze_input_deck
 * \brief Append material model to problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_material_model_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_natural_boundary_conditions_to_plato_problem
 * \brief Append natural boundary condition to plato problem parameter list.
 * \param [in]     aPhysics     name of the current physics
 * \param [in]     aLoadCase    loads relating to the current scenario
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_natural_boundary_conditions_to_plato_problem
(const std::string &aPhysics,
 const std::vector<XMLGen::NaturalBoundaryCondition> &aLoads,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_deterministic_natural_boundary_conditions_to_plato_problem
 * \brief Append deterministic natural boundary condition to problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_deterministic_natural_boundary_conditions_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_random_natural_boundary_conditions_to_plato_problem
 * \brief Append random natural boundary condition to problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_random_natural_boundary_conditions_to_plato_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_natural_boundary_conditions_to_plato_analyze_input_deck
 * \brief Append natural boundary condition to problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_natural_boundary_conditions_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_essential_boundary_conditions_to_plato_analyze_input_deck
 * \brief Append natural boundary condition to problem parameter list.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_essential_boundary_conditions_to_plato_analyze_input_deck
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn write_plato_analyze_input_deck_file
 * \brief Write plato analyze input file.
 * \param [in] aXMLMetaData Plato problem input data
**********************************************************************************/
void write_plato_analyze_input_deck_file
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn get_nbc_parent_node
 * \brief get the appropiate nbc parent node	
 * \param [in] aPhysics String indicating the physics
 * \param [in] aLoad Current load/NBC being added
 * \param [in] aParentNodes List of potential parent nodes
**********************************************************************************/
void get_nbc_parent_node
(const std::string &aPhysics,
 const XMLGen::NaturalBoundaryCondition &aLoad,
 const std::vector<pugi::xml_node> &aParentNodes,
 pugi::xml_node &aParentNode);

/******************************************************************************//**
 * \fn create_natural_boundary_condition_parent_nodes
 * \brief create the natural boundary condition parent nodes for the current physics
 * \param [in] tScenario The current scenario
 * \param [in] aParentNode Parent node that will contain the NBC blocks
 * \param [in/out] tParentNodes List of parent nodes created by this function
**********************************************************************************/
void create_natural_boundary_condition_parent_nodes
(const XMLGen::Scenario &aScenario,
 pugi::xml_node &aParentNode,
 std::vector<pugi::xml_node> &aParentNodes);

/******************************************************************************//**
 * \fn get_plato_analyze_service_id
 * \brief get the id of the service for the performer used in objective or constraint
 * \param [in] aXMLMetaData Plato problem input data
**********************************************************************************/
std::string get_plato_analyze_service_id
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn get_scenario_list_from_objectives_and_constraints
 * \brief get the list of scenarios relevant to this plato analyze performer
 * \param [in] aXMLMetaData Plato problem input data
**********************************************************************************/
void get_scenario_list_from_objectives_and_constraints
(const XMLGen::InputData& aXMLMetaData,
 std::vector<XMLGen::Scenario>& aScenarioList);

/******************************************************************************//**
 * \fn get_essential_boundary_condition_block_title
 * \brief get the title for the essential boundary condition block based on the physics
 * \param [in] aScenario Scenario input data
**********************************************************************************/
std::string get_essential_boundary_condition_block_title(XMLGen::Scenario &aScenario);

}

// namespace XMLGen
