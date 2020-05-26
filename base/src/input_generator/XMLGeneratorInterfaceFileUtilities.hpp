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
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_plato_main_performer
(pugi::xml_document& aDocument);

}
// namespace XMLGen
