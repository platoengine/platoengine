/*
 * XMLGeneratorInterfaceFileUtilities.hpp
 *
 *  Created on: May 26, 2020
 */

#pragma once

#include <string>
#include <vector>

#include "pugixml.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \fn append_lower_bounds_shared_data
 * \brief Append lower bounds shared data to PUGI XML document.
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_lower_bounds_shared_data
(pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_upper_bounds_shared_data
 * \brief Append upper bounds shared data to PUGI XML document.
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_upper_bounds_shared_data
(pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_design_volume_shared_data
 * \brief Append design volume shared data to PUGI XML document.
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_design_volume_shared_data
(pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_criterion_shared_data
 * \brief Append criterion value and gradient shared data to PUGI XML document.
 * \param [in]     aCriterion   criterion name
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
 * \param [in]     aOwnerName   name of the application that owns the data (default = "")
**********************************************************************************/
void append_criterion_shared_data
(const std::string& aCriterion,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument,
 std::string aOwnerName = "");

/******************************************************************************//**
 * \fn append_control_shared_data
 * \brief Append control shared data to PUGI XML document.
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_control_shared_data
(pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_plato_main_performer
 * \brief Append Plato Main performer information to PUGI XML document.
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_plato_main_performer
(pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_filter_control_operation
 * \brief Append filter control operation to PUGI XML document.
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_filter_control_operation
(pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_filter_criterion_gradient_operation
 * \brief Append filter criterion gradient operation to PUGI XML document.
 * \param [in]     aCriterionName criterion, e.g. objective, constraint, name
 * \param [in/out] aParentNode    pugi::xml_node
**********************************************************************************/
void append_filter_criterion_gradient_operation
(const std::string& aCriterionName,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_initial_guess_stage
 * \brief Append initial field operation to PUGI XML document.
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_initial_field_operation
(pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_initial_guess_stage
 * \brief Append initial guess stage to PUGI XML document.
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_initial_guess_stage
(pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_update_problem_stage
 * \brief Append initial guess stage to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_document
**********************************************************************************/
void append_update_problem_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_lower_bound_operation
 * \brief Append lower bound operation to PUGI XML document.
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_lower_bound_operation
(pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_lower_bound_stage
 * \brief Append lower bound stage to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_document
**********************************************************************************/
void append_lower_bound_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_upper_bound_operation
 * \brief Append upper bound stage to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_upper_bound_operation
(pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_upper_bound_stage
 * \brief Append upper bound stage to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_upper_bound_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_upper_bound_operation
 * \brief Append upper bound operation to PUGI XML document.
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_design_volume_operation
(pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_design_volume_stage
 * \brief Append design volume stage to PUGI XML document.
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_design_volume_stage
(pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_constraint_value_operation
 * \brief Append constraint value operation to PUGI XML document.
 * \param [in]     aPerformerName operation performer's name
 * \param [in/out] aParentNode    pugi::xml_node
**********************************************************************************/
void append_constraint_value_operation
(const std::string& aPerformerName,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_constraint_value_stage
 * \brief Append constraint value stage to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_constraint_value_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_constraint_gradient_operation
 * \brief Append constraint gradient operation to PUGI XML document.
 * \param [in]     aPerformerName operation performer's name
 * \param [in/out] aParentNode    pugi::xml_node
**********************************************************************************/
void append_constraint_gradient_operation
(const std::string& aPerformerName,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_constraint_gradient_stage
 * \brief Append constraint gradient stage to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_constraint_gradient_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_derivative_checker_parameters_options
 * \brief Append derivative checker options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_derivative_checker_parameters_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_algorithm_oc_parameters_options
 * \brief Append optimality criteria algorithm's options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_optimization_algorithm_oc_parameters_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_algorithm_mma_parameters_options
 * \brief Append Method of Moving Asymptotes (MMA) algorithm's options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_optimization_algorithm_mma_parameters_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_algorithm_parameters_options
 * \brief Append optimization algorithm's options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_optimization_algorithm_parameters_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

 /******************************************************************************//**
  * \fn append_optimization_algorithm_options
  * \brief Append optimization algorithm's options to PUGI XML document.
  * \param [in]     aXMLMetaData Plato problem input data
  * \param [in/out] aParentNode  pugi::xml_node
 **********************************************************************************/
void append_optimization_algorithm_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_algorithm_options
 * \brief Append optimization output options to PUGI XML document.
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_optimization_output_options
(pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_cache_stage_options
 * \brief Append optimization cache state options to PUGI XML document.
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_optimization_cache_stage_options
(pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_update_problem_stage_options
 * \brief Append optimization update problem options to PUGI XML document.
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_optimization_update_problem_stage_options
(pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_variables_options
 * \brief Append optimization variables options to PUGI XML document.
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_optimization_variables_options
(pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_objective_options
 * \brief Append optimization objective options to PUGI XML document.
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_optimization_objective_options
(pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_constraint_options
 * \brief Append optimization constraint options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_optimization_constraint_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_bound_constraints_options
 * \brief Append optimization bound constraints options to PUGI XML document.
 * \param [in]     aValues     upper and lower bounds, i.e. {upper_value, lower_value}
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_optimization_bound_constraints_options
(const std::vector<std::string>& aValues,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimizer_options
 * \brief Append optimizer's options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_optimizer_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

}
// namespace XMLGen
