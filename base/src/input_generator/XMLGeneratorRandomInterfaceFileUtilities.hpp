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
 * \fn append_qoi_shared_data_for_nondeterministic_usecase
 * \brief Append Quantities of Interest (QOI) shared data to PUGI XML document.\n
 * QOI denote problem quantities the user requested statistics to be computed.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_qoi_shared_data_for_nondeterministic_usecase
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

}
// namespace XMLGen
