/*
 * XMLGeneratorPlatoAnalyzeOperationsFileUtilities.hpp
 *
 *  Created on: Jun 4, 2020
 */

#pragma once

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{

namespace Analyze
{
/*!< information used to identify materials, \n
 * i.e. pair<material_category, vector<pair<material_property_argument_name_tag, material_property_tag> > > */
using MaterialInfo = std::pair<std::string, std::vector<std::pair<std::string, std::string>>>;

/*!< map from element block identification number to material metadata, i.e. map< block_id, MaterialInfo > */
using MaterialMetadata = std::unordered_map<std::string, XMLGen::Analyze::MaterialInfo>;
}
// namespace Analyze

/******************************************************************************//**
 * \fn is_plato_analyze_performer
 * \brief Returns true if Plato Analyze is used to compute physical quantities of \n
 * interest during optimization, i.e. Plato Analyze is an active performer.
 * \param [in] aPerformer physics app name, i.e. performer name
 * \return Flag
**********************************************************************************/
bool is_plato_analyze_performer(const std::string& aPerformer);

/******************************************************************************//**
 * \fn is_plato_analyze_performer
 * \brief Returns true if solving a topology optimization problem.
 * \param [in] aProblemType optimization problem type, i.e. shape, topology, inverse, others.
 * \return Flag
**********************************************************************************/
bool is_topology_optimization_problem(const std::string& aProblemType);

/******************************************************************************//**
 * \fn is_any_objective_computed_by_plato_analyze
 * \brief Returns true if any objective is computed by a Plato Analyze performer.
 * \param [in] aXMLMetaData Plato problem input data
 * \return Flag
**********************************************************************************/
bool is_any_objective_computed_by_plato_analyze
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn is_any_constraint_computed_by_plato_analyze
 * \brief Returns true if any constraint is computed by a Plato Analyze performer.
 * \param [in] aXMLMetaData Plato problem input data
 * \return Flag
**********************************************************************************/
bool is_any_constraint_computed_by_plato_analyze
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn append_update_problem_to_plato_analyze_operation
 * \brief Append update problem operation to PUGI XML document. The update problem \n
 * operation is used to enable safe non-Plato parameter, e.g. physics-only parameters \n
 * updates during optimization.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_update_problem_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_compute_solution_to_plato_analyze_operation
 * \brief Append compute solution operation to PUGI XML document.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_compute_solution_to_plato_analyze_operation
(pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_compute_objective_value_to_plato_analyze_operation
 * \brief Append compute objective value operation to PUGI XML document. The \n
 * operation is only appended if Plato Analyze is responsible for its evaluation.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_compute_objective_value_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_compute_objective_gradient_to_plato_analyze_operation
 * \brief Append compute objective gradient operation to PUGI XML document. The \n
 * operation is only appended if Plato Analyze is responsible for its evaluation.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_compute_objective_gradient_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_compute_constraint_value_to_plato_analyze_operation
 * \brief Append compute constraint value operation to PUGI XML document. The \n
 * operation is only appended if Plato Analyze is responsible for its evaluation.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_compute_constraint_value_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_compute_constraint_gradient_to_plato_analyze_operation
 * \brief Append compute constraint gradient operation to PUGI XML document. The \n
 * operation is only appended if Plato Analyze is responsible for its evaluation.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_compute_constraint_gradient_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_write_output_to_plato_analyze_operation
 * \brief Append write output operation to PUGI XML document. The operation is only \n
 * appended if the user specified output quantities of interest for visualization.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_write_output_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn return_material_property_tags_for_plato_analyze_operation_xml_file
 * \brief Return material metadata needed to write the plato analyze operation xml file.
 * \param [in] aRandomMetaData random samples metadata
 * \return map from element block identification number to material property tags, \n
 * i.e. map< block_id, pair< material_category, vector< pair<material_property_argument_name_tag, material_property_tag> > > >
**********************************************************************************/
XMLGen::Analyze::MaterialMetadata
return_material_metadata_for_plato_analyze_operation_xml_file
(const XMLGen::RandomMetaData& aRandomMetaData);

/******************************************************************************//**
 * \fn append_isotropic_linear_elastic_material_properties_to_plato_analyze_operation
 * \brief Append material properties for an isotropic linear elastic material plus \n
 * corresponding material property values to the plato analyze operation xml file.
 * \param [in]     aMaterialTags list of material tags, i.e. \n
 * vector<pair<material_property_argument_name_tag, material_property_tag>>
 * \param [in/out] aDocument     pugi::xml_document
**********************************************************************************/
void append_isotropic_linear_elastic_material_properties_to_plato_analyze_operation
(const std::vector<std::pair<std::string, std::string>>& aMaterialTags,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_isotropic_linear_thermoelastic_material_properties_to_plato_analyze_operation
 * \brief Append material properties for an isotropic linear thermoelastic material plus \n
 * corresponding material property values to the plato analyze operation xml file.
 * \param [in]     aMaterialTags list of material tags, i.e. \n
 * vector<pair<material_property_argument_name_tag, material_property_tag>>
 * \param [in/out] aDocument     pugi::xml_document
**********************************************************************************/
void append_isotropic_linear_thermoelastic_material_properties_to_plato_analyze_operation
(const std::vector<std::pair<std::string, std::string>>& aMaterialTags,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn append_material_properties_to_plato_analyze_operation
 * \brief Append material properties to the plato analyze operation xml file.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void append_material_properties_to_plato_analyze_operation
(const XMLGen::InputData& aXMLMetaData,
 pugi::xml_document& aDocument);

/******************************************************************************//**
 * \fn write_amgx_input_file
 * \brief Write AMGX input .json file. This file is used to assign values for the \n
 * linear solver parameters. Interested readers can find more information on AMGX \n
 * in \see{https://github.com/NVIDIA/AMGX}.
 * \param [in]     aXMLMetaData Plato problem input data
 * \param [in/out] aDocument    pugi::xml_document
**********************************************************************************/
void write_amgx_input_file();

}
// namespace XMLGen
