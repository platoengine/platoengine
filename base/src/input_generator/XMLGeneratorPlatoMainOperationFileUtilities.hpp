/*
 * XMLGeneratorPlatoMainOperationFileUtilities.hpp
 *
 *  Created on: May 28, 2020
 */

#pragma once

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \fn append_filter_options_to_operation
 * \brief Append filter options to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_filter_options_to_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode);

/******************************************************************************//**
 * \fn append_filter_to_plato_main_operation
 * \brief Append filter operation to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_filter_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document &aDocument);

/******************************************************************************//**
 * \fn append_constraint_gradient_input_to_output_operation
 * \brief Append constrain gradient inputs to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_constraint_gradient_input_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode);

/******************************************************************************//**
 * \fn append_objective_gradient_input_to_output_operation
 * \brief Append objective gradient inputs to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_objective_gradient_input_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode);

/******************************************************************************//**
 * \fn append_deterministic_qoi_inputs_to_output_operation
 * \brief Append deterministic Quantities of Interest (QoI) inputs to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_deterministic_qoi_inputs_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode);

/******************************************************************************//**
 * \fn append_nondeterministic_qoi_inputs_to_output_operation
 * \brief Append nondeterministic Quantities of Interest (QoI) inputs to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_nondeterministic_qoi_inputs_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode);

/******************************************************************************//**
 * \fn append_children_to_output_operation
 * \brief Append output operation children to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_children_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode);

/******************************************************************************//**
 * \fn append_children_to_output_operation
 * \brief Append default Quantities of Interest (QoI) inputs to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_default_qoi_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode);

/******************************************************************************//**
 * \fn append_surface_extraction_to_output_operation
 * \brief Append surface extraction operation to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_surface_extraction_to_output_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node &aParentNode);

/******************************************************************************//**
 * \fn append_output_to_plato_main_operation
 * \brief Append Plato main output operation to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_output_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document &aDocument);

/******************************************************************************//**
 * \fn append_stochastic_objective_value_to_plato_main_operation
 * \brief Append stochastic objective value operation to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_stochastic_objective_value_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document &aDocument);

/******************************************************************************//**
 * \fn append_stochastic_criterion_gradient_operation
 * \brief Append stochastic criterion gradient operation to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_stochastic_criterion_gradient_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_stochastic_criterion_value_operation
 * \brief Append stochastic criterion value operation to PUGI XML document.
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_stochastic_criterion_value_operation
(pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_stochastic_objective_gradient_to_plato_main_operation
 * \brief Append stochastic criterion value operation to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_document
**********************************************************************************/
void append_stochastic_objective_gradient_to_plato_main_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document &aDocument);

}
// namespace XMLGen
