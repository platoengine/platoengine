/*
 * XMLGeneratorRandomInterfaceFileUtilities.hpp
 *
 *  Created on: May 25, 2020
 */

#pragma once

#include <string>
#include <vector>

#include "pugixml.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \fn append_nondeterministic_shared_data
 * \brief Append nondeterministic shared data keys and values to PUGI XML document.
 * \param [in]     aKeys      keys to append
 * \param [in]     aValues    values to append
 * \param [in/out] aDocument  pugi::xml_document
**********************************************************************************/
void append_nondeterministic_shared_data
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_criterion_shared_data_for_nondeterministic_usecase
 * \brief Append shared data associated with an optimization criterion to PUGI XML document.
 * \param [in]     aCriterion   criterion name
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_criterion_shared_data_for_nondeterministic_usecase
(const std::string& aCriterion,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_nondeterministic_qoi_shared_data
 * \brief Append Quantities of Interest (QOI) shared data to PUGI XML document.\n
 * QOI denote problem quantities the user requested statistics to be computed.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_nondeterministic_qoi_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_topology_shared_data_for_nondeterministic_usecase
 * \brief Append Topology shared data to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_topology_shared_data_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_physics_performers_for_nondeterministic_usecase
 * \brief Append physics performers information to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_physics_performers_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_shared_data_for_nondeterministic_usecase
 * \brief Append shared data associated with a nondeterministic use case to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_shared_data_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_filter_criterion_gradient_samples_operation
 * \brief Append filter criterion gradient samples operation to PUGI XML document.
 * \param [in]     aCriterionName criterion, e.g. objective, constraint, name
 * \param [in/out] aParentNode    pugi::xml_node
**********************************************************************************/
void append_filter_criterion_gradient_samples_operation
(const std::string& aCriterionName,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_nondeterministic_operation
 * \brief Append a nondeterministic operation to PUGI XML document.
 * \param [in]     aKeys       keys to append
 * \param [in]     aValues     keys' values to append
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_nondeterministic_operation
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_cache_state_stage_for_nondeterministic_usecase
 * \brief Append cache state stage for a nondeterministic use case to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_cache_state_stage_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_update_problem_stage_for_nondeterministic_usecase
 * \brief Append update problem stage for a nondeterministic use case to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_update_problem_stage_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_nondeterministic_parameters
 * \brief Append nondeterministic parameters to PUGI XML document.
 * \param [in]     aTagsMap    parameter identification number to tag map
 * \param [in/out] aParentNode pugi::xml_node
**********************************************************************************/
void append_nondeterministic_parameters
(const std::unordered_map<std::string, std::vector<std::string>>& aTagsMap,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_sample_objective_value_operation
 * \brief Append sample objective value operation to PUGI XML document.
 * \param [in]     aPerformerName operation's performer name
 * \param [in]     aXMLMetaData   Plato problem input data
 * \param [in/out] aParentNode    pugi::xml_node
**********************************************************************************/
void append_sample_objective_value_operation
(const std::string& aPerformerName,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_evaluate_nondeterministic_objective_value_operation
 * \brief Append evaluate nondeterministic objective value operation to PUGI XML document.
 * \param [in]     aOutputSharedDataName output objective value shared data name
 * \param [in]     aXMLMetaData          Plato problem input data
 * \param [in/out] aParentNode           pugi::xml_node
**********************************************************************************/
void append_evaluate_nondeterministic_objective_value_operation
(const std::string& aOutputSharedDataName,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_objective_value_stage_for_nondeterministic_usecase
 * \brief Append nondeterministic objective value stage to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_objective_value_stage_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_sample_objective_gradient_operation
 * \brief Append sample criterion gradient operation to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_sample_objective_gradient_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_evaluate_nondeterministic_objective_gradient_operation
 * \brief Append evaluate nondeterministic objective gradient operation to PUGI XML document.
 * \param [in]     aOutputSharedDataName output objective gradient shared data name
 * \param [in]     aXMLMetaData          Plato problem input data
 * \param [in/out] aParentNode           pugi::xml_node
**********************************************************************************/
void append_evaluate_nondeterministic_objective_gradient_operation
(const std::string& aOutputSharedDataName,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_node& aParentNode);

/******************************************************************************//**
 * \fn append_objective_gradient_stage_for_nondeterministic_usecase
 * \brief Append objective gradient stage to PUGI XML document.
 * \param [in]     aXMLMetaData   Plato problem input data
 * \param [in/out] aParentNode    pugi::xml_node
**********************************************************************************/
void append_objective_gradient_stage_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_stages_for_nondeterministic_usecase
 * \brief Append stages for a nondeterministic use case to the PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aParentNode  pugi::xml_node
**********************************************************************************/
void append_stages_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn write_interface_xml_file_for_nondeterministic_usecase
 * \brief Write interface.xml file for a nondeterministic use case.
 * \param [in] aXMLMetaData Plato problem input data
**********************************************************************************/
void write_interface_xml_file_for_nondeterministic_usecase
(const XMLGen::InputData& aXMLMetaData);

}
// namespace XMLGen
