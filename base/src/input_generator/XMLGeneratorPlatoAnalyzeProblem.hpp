/*
 * XMLGeneratorPlatoAnalyzeProblem.hpp
 *
 *  Created on: Jun 15, 2020
 */

#pragma once

#include <iostream>

#include "XMLGeneratorDefinesFileUtilities.hpp"
#include "XMLGeneratorLaunchScriptUtilities.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"
#include "XMLGeneratorRandomInterfaceFileUtilities.hpp"
#include "XMLGeneratorPlatoMainInputFileUtilities.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeOperationsFileUtilities.hpp"
#include "XMLGeneratorAnalyzeUncertaintyLaunchScriptUtilities.hpp"

namespace XMLGen
{

namespace Analyze
{

/******************************************************************************//**
 * \fn is_robust_optimization_problem
 * \brief Write input files needed to solve optimization problems with Plato Analyze.
 * \param [in] aInputData input metadata
**********************************************************************************/
inline bool is_robust_optimization_problem
(const XMLGen::InputData& aMetaData)
{
    if(aMetaData.mRandomMetaData.empty())
        return false;
    else
        return true;
}
// function is_robust_optimization_problem

/******************************************************************************//**
 * \fn write_optimization_problem
 * \brief Write input files needed to solve optimization problems with Plato Analyze.
 * \param [in] aInputData input metadata
**********************************************************************************/
inline void write_optimization_problem
(const XMLGen::InputData& aMetaData)
{
    if(XMLGen::Analyze::is_robust_optimization_problem(aMetaData))
        XMLGen::write_define_xml_file(aMetaData.mRandomMetaData, aMetaData.m_UncertaintyMetaData);

    XMLGen::write_interface_xml_file(aMetaData);
    XMLGen::write_plato_main_operations_xml_file(aMetaData);
    XMLGen::write_plato_analyze_operation_xml_file(aMetaData);
    XMLGen::write_plato_main_input_deck_file(aMetaData);
    XMLGen::write_plato_analyze_input_deck_file(aMetaData);
    XMLGen::write_amgx_input_file(aMetaData.service(0u));
    XMLGen::generate_launch_script(aMetaData);
}

}
// namespace Analyze

}
// namespace XMLGen
