/*
 * XMLGeneratorDefinesFileUtilities.hpp
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
 * \fn prepare_probabilities_for_define_xml_file
 * \brief Return probabilities in the format needed by the define XML file.
 * \param [in] aRandomMetaData random samples metadata
 * \return formatted probabilities container
**********************************************************************************/
std::vector<std::string>
prepare_probabilities_for_define_xml_file
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
 * \return traction load index to tags map, i.e. map<load_index, vector<argument_name_tag>>
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
 * \fn return_material_property_tags_for_define_xml_file
 * \brief Return map from element block identification number to list of material \n
 * property argument name tags used to define parameters in the define.xml file.
 * \param [in] aRandomMetaData random samples metadata
 * \return map from element block identification number to list of material property \n
 * argument name tags, i.e. map<elem_block_id, vector<material_property_argument_name_tag>>
**********************************************************************************/
std::unordered_map<std::string, std::vector<std::string>>
return_material_property_tags_for_define_xml_file
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
 * \param [in] aMetaData  input meta data
**********************************************************************************/
void write_define_xml_file
(const XMLGen::InputData& aMetaData);

/******************************************************************************//**
 * \fn add_robust_optimization_data_to_define_xml_file
 * \brief Add robust optimization parameters to defines.xml
 * \param [in] aMetaData  input meta data
 * \param [in] aDocument  document to add to
**********************************************************************************/
void add_robust_optimization_data_to_define_xml_file
(const XMLGen::InputData& aMetaData,
 pugi::xml_document &aDocument);

/******************************************************************************//**
 * \fn add_shape_optimization_data_to_define_xml_file
 * \brief Add shape optimization parameters to defines.xml
 * \param [in] aMetaData  input meta data
 * \param [in] aDocument  document to add to
**********************************************************************************/
void add_shape_optimization_data_to_define_xml_file
(const XMLGen::InputData& aMetaData,
 pugi::xml_document &aDocument);

/******************************************************************************//**
 * \fn allocate_map_from_random_load_identifier_to_load_samples
 * \brief Allocate map from random load identifier to samples' load component values, \n
 * i.e. map< random_load_identifier, vector[DimIndex][SampleIndex] >.
 * \param [in] aRandomMetaData      random samples metadata
 * \return map from random load identifier to samples' load component values
**********************************************************************************/
std::map<std::string, std::vector<std::vector<std::string>>>
allocate_map_from_random_load_identifier_to_load_samples
(const XMLGen::RandomMetaData& aRandomMetaData);

/******************************************************************************//**
 * \fn prepare_tractions_for_define_xml_file
 * \brief Return initialized map from random load identifier to samples' load \n
 * component values, i.e. map< random_load_identifier, vector[DimIndex][SampleIndex] >.
 * \param [in] aRandomMetaData      random samples metadata
 * \return map from random load identifier to samples' load component values
**********************************************************************************/
std::map<std::string, std::vector<std::vector<std::string>>>
prepare_tractions_for_define_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData);

/******************************************************************************//**
 * \fn append_tractions_to_define_xml_file
 * \brief Append random traction loads to define.xml file.
 * \param [in]     aTags     map from random load identifier to random load tags
 * \param [in]     aValues   map from random load identifier to samples' load component values
 * \param [in/out] aDocument xml document
**********************************************************************************/
void append_tractions_to_define_xml_file
(const std::unordered_map<std::string, std::vector<std::string>>& aTags,
 const std::map<std::string, std::vector<std::vector<std::string>>>& aValues,
 pugi::xml_document& aDocument);

}
// namespace XMLGen
