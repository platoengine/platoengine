/*
 * XMLGeneratorAnalyzeDefinesFileUtilities.hpp
 *
 *  Created on: May 20, 2020
 */

#pragma once

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"
#include "XMLGeneratorRandomMetadata.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \fn append_basic_attributes_to_define_xml_file
 * \brief Append basic attributes, e.g. number of samples and performers, to define XML file.
 * \param [in]     aRandomMetaData       random samples metadata
 * \param [in]     aUncertaintyMetaData  uncertainty metadata
 * \param [in/out] aDocument             pugi::xml_document
**********************************************************************************/
void append_basic_attributes_to_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData,
 const XMLGen::UncertaintyMetaData& aUncertaintyMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn allocate_random_tractions_container_for_define_xml_file
 * \brief Return allocated random tractions container.
 * \param [in]     aRandomMetaData       random samples metadata
 * \return random tractions container
**********************************************************************************/
std::vector<std::vector<std::vector<std::string>>>
allocate_random_tractions_container_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData);

/******************************************************************************//**
 * \fn prepare_probabilities_for_define_xml_file
 * \brief Return probabilities in the format needed by the define XML file.
 * \param [in] aRandomMetaData random samples metadata
 * \return formatted probabilities container
**********************************************************************************/
std::vector<std::string>
prepare_probabilities_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData);

/******************************************************************************//**
 * \fn prepare_tractions_for_define_xml_file
 * \brief Return traction loads in the format needed by the define XML file.
 * \param [in] aRandomMetaData random samples metadata
 * \return formatted traction loads container
**********************************************************************************/
std::vector<std::vector<std::vector<std::string>>>
prepare_tractions_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData);

/******************************************************************************//**
 * \fn append_probabilities_to_define_xml_file
 * \brief Append probabilities to pugi::xml_document.
 * \param [in]     aProbabilities probabilities container
 * \param [in/out] aDocument      pugi::xml_document
**********************************************************************************/
void append_probabilities_to_define_xml_file
(const std::vector<std::string>& aProbabilities,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn return_random_tractions_tags_for_define_xml_file
 * \brief Return random tractions' tags used to define parameters inside the define.xml file.
 * \param [in] aRandomMetaData random samples metadata
 * \return traction load index to tags map, i.e. map<load index, vector<tags>>
**********************************************************************************/
std::unordered_map<std::string, std::vector<std::string>>
return_random_tractions_tags_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData);

/******************************************************************************//**
 * \fn append_tractions_to_define_xml_file
 * \brief Append formated tractions to pugi::xml_document.
 * \param [in]     aTags      traction loads' tags
 * \param [in]     aValues    traction loads' values
 * \param [in/out] aDocument  pugi::xml_document
**********************************************************************************/
void append_tractions_to_define_xml_file
(const std::unordered_map<std::string, std::vector<std::string>>& aTags,
 const std::vector<std::vector<std::vector<std::string>>>& aValues,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn return_material_properties_tags_for_define_xml_file
 * \brief Return material properties' tags used to define parameters inside the \n
 * define.xml file.
 * \param [in] aRandomMetaData random samples metadata
 * \return traction block identification number to material properties' tags map, \n
 * i.e. map<block id, vector<tags>>
**********************************************************************************/
std::unordered_map<std::string, std::vector<std::string>>
return_material_properties_tags_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData);

/******************************************************************************//**
 * \fn prepare_material_properties_for_define_xml_file
 * \brief Return materials in the format needed by the define XML file.
 * \param [in] aRandomMetaData random samples metadata
 * \return formatted formatted materials
**********************************************************************************/
std::unordered_map<std::string, std::vector<std::string>>
prepare_material_properties_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData);

/******************************************************************************//**
 * \fn append_material_properties_to_define_xml_file
 * \brief Append formated material properties to pugi::xml_document.
 * \param [in]     aMaterials set of formatted material properties
 * \param [in/out] aDocument  pugi::xml_document
**********************************************************************************/
void append_material_properties_to_define_xml_file
(const std::unordered_map<std::string, std::vector<std::string>>& aMaterials,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn write_define_xml_file
 * \brief Write define XML file.
 * \param [in] aRandomMetaData      random samples metadata
 * \param [in] aUncertaintyMetaData uncertainty metadata
**********************************************************************************/
void write_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData,
 const XMLGen::UncertaintyMetaData& aUncertaintyMetaData);

}
// namespace XMLGen
