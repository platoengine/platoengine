/*
 * XMLGeneratorPlatoAnalyzeUtilities.hpp
 *
 *  Created on: Jun 8, 2020
 */

#pragma once

#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \fn is_plato_analyze_code
 * \brief Returns true if Plato Analyze is used to compute physical quantities of \n
 * interest during optimization, i.e. Plato Analyze is an active performer.
 * \param [in] aCode simulation code name
 * \return Flag
**********************************************************************************/
bool is_plato_analyze_code(const std::string& aCode);

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
 * \fn return_constraints_computed_by_plato_analyze
 * \brief Returns categories of constraints computed by plato analyze.
 * \param [in] aXMLMetaData Plato problem input data
 * \return list of constraint categories computed by plato analyze
**********************************************************************************/
std::vector<std::string>
return_constraints_computed_by_plato_analyze
(const XMLGen::InputData& aXMLMetaData);

/******************************************************************************//**
 * \fn return_objectives_computed_by_plato_analyze
 * \brief Returns categories of objectives computed by plato analyze.
 * \param [in] aXMLMetaData Plato problem input data
 * \return list of objectives categories computed by plato analyze
**********************************************************************************/
std::vector<std::string>
return_objectives_computed_by_plato_analyze
(const XMLGen::InputData& aXMLMetaData);

}
// namespace Plato
