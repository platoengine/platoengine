/*
 * XMLGeneratorRandomInterfaceFileUtilities.hpp
 *
 *  Created on: May 25, 2020
 */

#pragma once

#include <string>
#include <vector>

#include <XMLGeneratorDataStruct.hpp>

#include "pugixml.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \fn append_multiperformer_shared_data
 * \brief Append nondeterministic shared data keys and values to PUGI XML document.
 * \param [in]     aKeys      keys to append
 * \param [in]     aValues    values to append
 * \param [in/out] aDocument  pugi::xml_document
**********************************************************************************/
void append_multiperformer_shared_data
(const std::vector<std::string>& aKeys,
 const std::vector<std::string>& aValues,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_multiperformer_criterion_shared_data
 * \brief Append shared data associated with an optimization criterion to PUGI XML document.
 * \param [in]     aCriterion   criterion name
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_multiperformer_criterion_shared_data
(const std::string& aCriterion,
 const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_qoi_statistics_shared_data
 * \brief Append Quantities of Interest (QoI) statistics shared data xml document. \n
 * The QoI are assigned in the 'compute_quantity_of_interest_statistics' keyword \n
 * inside the output block.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_qoi_statistics_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_multiperformer_qoi_shared_data
 * \brief Append Quantities of Interest (QOI) shared data to PUGI XML document.\n
 * QOI denote problem quantities the user requested statistics to be computed.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_multiperformer_qoi_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_multiperformer_topology_shared_data
 * \brief Append Topology shared data to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_multiperformer_topology_shared_data
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_physics_performers_multiperformer_usecase
 * \brief Append physics performers information to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in]     aNextPerformerID Next available performer id
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_physics_performers_multiperformer_usecase
(const XMLGen::InputData& aXMLMetaData,
 int &aNextPerformerID,
 pugi::xml_node& aNode);

/******************************************************************************//**
 * \fn append_filter_criterion_gradient_samples_operation
 * \brief Append filter criterion gradient samples operation to PUGI XML document.
 * \param [in]     aCriterionName criterion, e.g. objective, constraint, name
 * \param [in]     aPerformerName Performer name
 * \param [in/out] aParentNode    pugi::xml_node
**********************************************************************************/
void append_filter_criterion_gradient_samples_operation
(const std::string& aCriterionName,
 const std::string& aPerformerName,
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
 * \param [in]     aXMLMetaData   Plato problem input data
 * \param [in/out] aParentNode    pugi::xml_node
**********************************************************************************/
void append_sample_objective_value_operation
(const XMLGen::InputData& aXMLMetaData,
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
 * \fn get_random_objective_service_id
 * \brief Get the service used for the random objective.
 * \param [in]     aXMLMetaData          Plato problem input data
 * \param [out]    serviceID           service ID string
**********************************************************************************/
std::string get_random_objective_service_id
(const XMLGen::InputData& aXMLMetaData);
}
// namespace XMLGen
