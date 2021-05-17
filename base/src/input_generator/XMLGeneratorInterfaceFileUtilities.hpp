/*
 * XMLGeneratorInterfaceFileUtilities.hpp
 *
 *  Created on: May 26, 2020
 */

#pragma once

#include <string>
#include <vector>

#include "pugixml.hpp"
#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \fn write_interface_xml_file
 * \brief Write interface.xml file.
 * \param [in]  aXMLMetaData Plato problem input metadata
**********************************************************************************/
void write_interface_xml_file
(const XMLGen::InputData& aMetaData);

/******************************************************************************//**
 * \fn append_physics_performers
 * \brief Append physics performers to interface.xml file.
 * \param [in]     aXMLMetaData Plato problem input metadata
 * \param [in]     aNextPerformerID Next available performer id
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_physics_performers
(const XMLGen::InputData& aXMLMetaData,
 int &aNextPerformerID,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_performer_data
 * \brief Append performers to interface.xml file.
 * \param [in]     aXMLMetaData Plato problem input metadata
 * \param [in]     aNextPerformerID Next available performer id
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_performer_data
(const XMLGen::InputData& aMetaData,
 int &aNextPerformerID,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_console_data
 * \brief Append console data to interface.xml file.
 * \param [in]     aXMLMetaData Plato problem input metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_console_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_esp_performers
 * \brief Append esp shape derivative performers to interface.xml file.
 * \param [in]     aXMLMetaData Plato problem input metadata
 * \param [in]     aNextPerformerID Next available performer id
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_esp_performers
(const XMLGen::InputData& aXMLMetaData,
 int &aNextPerformerID,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_qoi_shared_data
 * \brief Append quantities of interest shared data to interface.xml file.
 * \param [in]     aXMLMetaData Plato problem input metadata
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_qoi_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aParentNode);

/******************************************************************************//**
 * \fn append_topology_shared_data
 * \brief Append topology shared data to interface.xml file.
 * \param [in]     aXMLMetaData Plato problem input metadata
 * \param [in/out] aDocument    parent xml document
**********************************************************************************/
void append_topology_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_shared_data
 * \brief Append shared data to interface.xml file.
 * \param [in]     aXMLMetaData Plato problem input metadata
 * \param [in/out] aDocument    parent xml document
**********************************************************************************/
void append_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_cache_state_stage
 * \brief Append cache state stage to interface.xml file.
 * \param [in]     aXMLMetaData Plato problem input metadata
 * \param [in/out] aDocument    parent xml document
**********************************************************************************/
void append_cache_state_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_update_problem_stage
 * \brief Append update problem stage to interface.xml file.
 * \param [in]     aXMLMetaData Plato problem input metadata
 * \param [in/out] aDocument    parent xml document
**********************************************************************************/
void append_update_problem_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_objective_value_stage
 * \brief Append objective value stage to interface.xml file.
 * \param [in]     aXMLMetaData Plato problem input metadata
 * \param [in/out] aDocument    parent xml document
**********************************************************************************/
void append_objective_value_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_stages
 * \brief Append stages to interface.xml file.
 * \param [in]     aXMLMetaData Plato problem input metadata
 * \param [in/out] aDocument    parent xml document
**********************************************************************************/
void append_stages
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_objective_gradient_stage
 * \brief Append objective gradient stage to interface.xml file.
 * \param [in]     aXMLMetaData Plato problem input metadata
 * \param [in/out] aDocument    parent xml document
**********************************************************************************/
void append_objective_gradient_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_write_ouput_operation
 * \brief Append write output operation to output stage.
 * \param [in]     aMetaData    Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_write_ouput_operation
(const XMLGen::InputData& aMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_plato_main_output_stage
 * \brief Append plato main output stage to interface.xml file.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_plato_main_output_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_lower_bounds_shared_data
 * \brief Append lower bounds shared data to PUGI XML document.
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_lower_bounds_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_upper_bounds_shared_data
 * \brief Append upper bounds shared data to PUGI XML document.
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_upper_bounds_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_design_volume_shared_data
 * \brief Append design volume shared data to PUGI XML document.
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_design_volume_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_objective_shared_data
 * \brief Append objective value and gradient shared data to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
 * \param [in]     aOwnerName   name of the application that owns the data (default = "")
**********************************************************************************/
void append_objective_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_normalization_shared_data
 * \brief Append normalization variable shared data to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
 * \param [in]     aOwnerName   name of the application that owns the data (default = "")
**********************************************************************************/
void append_normalization_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_criteria_shared_data
 * \brief Append shared data for criteria values and gradients to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_criteria_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_constraint_shared_data
 * \brief Append shared data for criteria values and gradients to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_constraint_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_control_shared_data
 * \brief Append control shared data to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_control_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_plato_main_performer
 * \brief Append Plato Main performer information to PUGI XML document.
 * \param [in] aNextPerformerID Next available performer id
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_plato_main_performer
(const XMLGen::InputData& aMetaData,
 int &aNextPerformerID,
 pugi::xml_node& aDocument);

/******************************************************************************//**
 * \fn append_compute_qoi_statistics_operation
 * \brief Append compute QoI statistics operation to output stage
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_compute_qoi_statistics_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_initialize_geometry_operation
 * \brief Append initialize geometry operation for shape optimization problems
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_initialize_geometry_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_initial_values_operation
 * \brief Append initialize values operation for shape optimization problems
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_initial_values_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_update_geometry_on_change_operation
 * \brief Append operation for updating geometry for shape optimization
 * \param [in]     aFirstPlatoMainPerformer PlatoMain performer name 
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_update_geometry_on_change_operation 
(const std::string &aFirstPlatoMainPerformer, 
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_reinitialize_on_change_operation
 * \brief Append operation for reinitializing performer after geometry is updated
 * \param [in]     aPerformer Performer name
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_reinitialize_on_change_operation 
(const std::string &aPerformer, 
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_design_parameters_shared_data
 * \brief Append design parameters shared data for shape optimization problems
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_design_parameters_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_constraint_stage_for_topology_problem
 * \brief Append constaint stage for topology optimization problem
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_constraint_stage_for_topology_problem 
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_objective_gradient_stage_for_topology_problem
 * \brief Append objective gradient stage for topology optimization problem
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_objective_gradient_stage_for_topology_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_objective_gradient_stage_for_shape_problem
 * \brief Append objective gradient stage for shape optimization problem
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_objective_gradient_stage_for_shape_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_objective_value_stage_for_topology_problem
 * \brief Append objective value stage for topology optimization problem
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_objective_value_stage_for_topology_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_constraint_gradient_stage_for_topology_problem
 * \brief Append constaint gradient stage for topology optimization problem
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_constraint_gradient_stage_for_topology_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_copy_value_operation
 * \brief Append operation copying a value from one shared data to another
 * \param [in]     aPlatoMainPerformer performer doing the copy
 * \param [in]     aInputSharedDataName name of input shared data
 * \param [in]     aOutputSharedDataName name of output shared data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_copy_value_operation
(const std::string &aPerformer,
 const std::string &aInputSharedDataName,
 const std::string &aOutputSharedDataName,
 pugi::xml_node &aParent);

/******************************************************************************//**
 * \fn append_copy_field_operation
 * \brief Append operation copying a field from one shared data to another
 * \param [in]     aPlatoMainPerformer performer doing the copy
 * \param [in]     aInputSharedDataName name of input shared data
 * \param [in]     aOutputSharedDataName name of output shared data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_copy_field_operation
(const std::string &aPerformer,
 const std::string &aInputSharedDataName,
 const std::string &aOutputSharedDataName,
 pugi::xml_node &aParent);

/******************************************************************************//**
 * \fn append_compute_objective_sensitivity_operation
 * \brief Append objective sensitivity operation for shape optimization problem
 * \param [in]     aPerformer Name of current performer
 * \param [in]     aSharedDataName Name of shared data to store result
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_compute_objective_sensitivity_operation
(const std::string &aPerformer,
 const std::string &aSharedDataName,
 pugi::xml_node &aParent);

/******************************************************************************//**
 * \fn append_compute_constraint_sensitivity_operation
 * \brief Append constraint sensitivity operation for shape optimization problem
 * \param [in]     aPerformer Name of current performer
 * \param [in]     aSharedDataName Name of shared data to store result
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_compute_constraint_sensitivity_operation
(const std::string &aPerformer,
 const std::string &aSharedDataName,
 pugi::xml_node &aParent);

/******************************************************************************//**
 * \fn append_compute_shape_sensitivity_on_change_operation
 * \brief Append compute shape sensitivity on change operation for shape optimization problem
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_compute_shape_sensitivity_on_change_operation
(pugi::xml_node &aParent);

/******************************************************************************//**
 * \fn append_compute_constraint_gradient_operation
 * \brief Append constaint gradient operation for shape optimization problem
 * \param [in]     aPerformer Name of performer calculating the constraint gradient
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_compute_constraint_gradient_operation
(const std::string &aPerformer,
 pugi::xml_node &aParent);

/******************************************************************************//**
 * \fn append_constraint_gradient_stage_for_shape_problem
 * \brief Append constaint gradient stage for shape optimization problem
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_constraint_gradient_stage_for_shape_problem
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_constraint_stage_for_shape_problem
 * \brief Append constaint stage for shape optimization problem
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_constraint_stage_for_shape_problem 
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_design_variables_shared_data
 * \brief Append design variables shared data
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_design_variables_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_parameter_sensitivity_shared_data
 * \brief Append parameters sensitivity shared data for shape optimization problems
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  parent xml node
**********************************************************************************/
void append_parameter_sensitivity_shared_data
(const XMLGen::InputData& aMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_filter_control_operation
 * \brief Append filter control operation to PUGI XML document.
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_filter_control_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_enforce_bounds_operation
 * \brief Append enforce bounds operation to PUGI XML document.
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_enforce_bounds_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_filter_criterion_gradient_operation
 * \brief Append filter criterion gradient operation to PUGI XML document.
 * \param [in]     aSharedDataName criterion gradient shared data name
 * \param [in/out] aParentNode     pugi::xml_node
**********************************************************************************/
void append_filter_criterion_gradient_operation
(const XMLGen::InputData& aXMLMetaData,
 const std::string& aInputSharedDataName,
 const std::string& aOutputSharedDataName,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_initial_guess_stage
 * \brief Append initial field operation to PUGI XML document.
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_initial_field_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_initial_guess_stage
 * \brief Append initial guess stage to PUGI XML document.
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_initial_guess_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_lower_bound_operation
 * \brief Append lower bound operation to PUGI XML document.
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_lower_bound_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

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
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

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
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_design_volume_stage
 * \brief Append design volume stage to PUGI XML document.
 * \param [in/out] aDocument pugi::xml_document
**********************************************************************************/
void append_design_volume_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_aggregate_objective_value_operation
 * \brief Append aggregate objective value operation to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_aggregate_objective_value_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_aggregate_objective_gradient_operation
 * \brief Append aggregate objective gradient operation to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_aggregate_objective_gradient_operation
(const XMLGen::InputData& aXMLMetaData,
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
 * \fn append_constraint_gradient_stage
 * \brief Append constraint gradient stage to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_constraint_gradient_stage
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_derivative_checker_options
 * \brief Append derivative checker options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_derivative_checker_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimality_criteria_options
 * \brief Append optimality criteria algorithm's options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_optimality_criteria_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_method_moving_asymptotes_options
 * \brief Append Method of Moving Asymptotes (MMA) algorithm's options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_method_moving_asymptotes_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_trust_region_kelley_sachs_options
 * \brief Append options for trust region Kelley-Sachs optimization algorithm.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_trust_region_kelley_sachs_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_augmented_lagrangian_options
 * \brief Append options for augmented lagrangian algorithm.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_augmented_lagrangian_options
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
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_optimization_output_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_cache_stage_options
 * \brief Append optimization cache state options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_optimization_cache_stage_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_update_problem_stage_options
 * \brief Append optimization update problem options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_optimization_update_problem_stage_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_variables_options
 * \brief Append optimization variables options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_optimization_variables_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_optimization_objective_options
 * \brief Append optimization objective options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_optimization_objective_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

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

/******************************************************************************//**
 * \fn append_compute_normalization_factor_operation
 * \brief Append operation for calculating normalization factor to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_compute_normalization_factor_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn get_filter_objective_criterion_gradient_input_shared_data_name
 * \brief Get the name for the input shared data for objective gradient filtering
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [out]    Name          Returned name
**********************************************************************************/
std::string get_filter_objective_criterion_gradient_input_shared_data_name
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn get_filter_constraint_criterion_gradient_input_shared_data_name
 * \brief Get the name for the input shared data for constraint gradient filtering
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [out]    Name          Returned name
**********************************************************************************/
std::string get_filter_constraint_criterion_gradient_input_shared_data_name
(const XMLGen::Constraint &aConstraint);

/******************************************************************************//**
 * \fn get_design_variable_name
 * \brief Get the name for the design variable to be used in calculations
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [out]    Name          Returned name
**********************************************************************************/
std::string get_design_variable_name
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn append_objective_value_operation_for_multi_load_case
 * \brief Append the objective value operation for multi-load case problems
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [in]     aParentNode Parent to add data to
 * \param [in]     aCalculatingNormalizationFactor Flag telling if we are calculating the objective normalization factor
**********************************************************************************/
void append_objective_value_operation_for_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode,
 bool aCalculatingNormalizationFactor);

/******************************************************************************//**
 * \fn append_objective_value_operation_for_non_multi_load_case
 * \brief Append the objective value operation for non multi-load case problems
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [in]     aParentNode Parent to add data to
 * \param [in]     aCalculatingNormalizationFactor Flag telling if we are calculating the objective normalization factor
**********************************************************************************/
void append_objective_value_operation_for_non_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode,
 bool aCalculatingNormalizationFactor);

/******************************************************************************//**
 * \fn append_gradient_value_operation_for_multi_load_case
 * \brief Append the gradient value operation for multi-load case problems
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [in]     aParentNode Parent to add data to
 * \param [in]     aCalculatingNormalizationFactor Flag telling if we are calculating the gradient normalization factor
**********************************************************************************/
void append_gradient_value_operation_for_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode);

/******************************************************************************//**
 * \fn append_gradient_value_operation_for_non_multi_load_case
 * \brief Append the gradient value operation for non multi-load case problems
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [in]     aParentNode Parent to add data to
 * \param [in]     aCalculatingNormalizationFactor Flag telling if we are calculating the gradient normalization factor
**********************************************************************************/
void append_gradient_value_operation_for_non_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode);

/******************************************************************************//**
 * \fn append_aggregate_objective_value_operation_for_multi_load_case
 * \brief Append the aggregate objective value operation for multi-load case problems
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [in]     aParentNode Parent to add data to
**********************************************************************************/
void append_aggregate_objective_value_operation_for_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode);

/******************************************************************************//**
 * \fn append_aggregate_objective_value_operation_for_non_multi_load_case
 * \brief Append the aggregate objective value operation for non multi-load case problems
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [in]     aParentNode Parent to add data to
**********************************************************************************/
void append_aggregate_objective_value_operation_for_non_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode);

/******************************************************************************//**
 * \fn append_aggregate_objective_gradient_operation_for_multi_load_case
 * \brief Append the aggregate objective gradient operation for multi-load case problems
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [in]     aParentNode Parent to add data to
 * \param [in]     aType type of data being aggregated
**********************************************************************************/
void append_aggregate_objective_gradient_operation_for_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode,
 std::string &aType);

/******************************************************************************//**
 * \fn append_aggregate_objective_gradient_operation_for_non_multi_load_case
 * \brief Append the aggregate objective gradient operation for non multi-load case problems
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [in]     aParentNode Parent to add data to
 * \param [in]     aType type of data being aggregated
**********************************************************************************/
void append_aggregate_objective_gradient_operation_for_non_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode,
 std::string &aType);

/******************************************************************************//**
 * \fn getCompoundScenarioID
 * \brief Build a compound scenario id from a list of ids
 * \param [in]     aScenarioIDs  List of ids to build compound id from
**********************************************************************************/
std::string get_compound_scenario_id
(const std::vector<std::string> &aScenarioIDs);

/******************************************************************************//**
 * \fn get_objective_value_operation_name
 * \brief Get the name of the Compute Objective Value operation based on the service provider
 * \param [in] aService The service provider
**********************************************************************************/
std::string get_objective_value_operation_name
(XMLGen::Service &aService);

/******************************************************************************//**
 * \fn get_objective_gradient_operation_name
 * \brief Get the name of the Compute Objective Gradient operation based on the service provider
 * \param [in] aService The service provider
**********************************************************************************/
std::string get_objective_gradient_operation_name
(XMLGen::Service &aService);

/******************************************************************************//**
 * \fn get_objective_value_operation_output_name
 * \brief Get the name of the Compute Objective Value operation output based on the service provider
 * \param [in] aService The service provider
**********************************************************************************/
std::string get_objective_value_operation_output_name
(XMLGen::Service &aService);

/******************************************************************************//**
 * \fn get_objective_gradient_operation_output_name
 * \brief Get the name of the Compute Objective Gradient operation output based on the service provider
 * \param [in] aService The service provider
**********************************************************************************/
std::string get_objective_gradient_operation_output_name
(XMLGen::Service &aService);

/******************************************************************************//**
 * \fn num_cache_states
 * \brief Return the number of services with the cache state flag set
 * \param [in] aServices List of services to check
**********************************************************************************/
int num_cache_states
(const std::vector<XMLGen::Service> &aServices);

/******************************************************************************//**
 * \fn append_globally_convergent_method_moving_asymptotes_options
 * \brief Append GCMMA options
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [in]     aParentNode Parent to add data to
**********************************************************************************/
void append_globally_convergent_method_moving_asymptotes_options
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn need_update_problem_stage
 * \brief Return whether any of the serivces have the update problem flag set to true
 * \param [in]     aXMLMetaData  Plato problem input data
**********************************************************************************/
bool need_update_problem_stage
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn append_qoi_shared_data_for_multi_load_case
 * \brief Append quantities of interest shared data for multi load case problems
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [in]     aParentNode Parent to add data to
**********************************************************************************/
void append_qoi_shared_data_for_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aParentNode);

/******************************************************************************//**
 * \fn append_qoi_shared_data_for_non_multi_load_case
 * \brief Append quantities of interest shared data for non multi load case problems
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [in]     aParentNode Parent to add data to
**********************************************************************************/
void append_qoi_shared_data_for_non_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aParentNode);

/******************************************************************************//**
 * \fn append_deterministic_qoi_to_plato_main_output_stage_for_non_multi_load_case
 * \brief Append quantities of interest to output stage for non multi load case problems
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [in]     aParentNode Parent to add data to
**********************************************************************************/
void append_deterministic_qoi_to_plato_main_output_stage_for_non_multi_load_case 
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_deterministic_qoi_to_plato_main_output_stage_for_multi_load_case
 * \brief Append quantities of interest to output stage for multi load case problems
 * \param [in]     aXMLMetaData  Plato problem input data
 * \param [in]     aParentNode Parent to add data to
**********************************************************************************/
void append_deterministic_qoi_to_plato_main_output_stage_for_multi_load_case
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

}
// namespace XMLGen
