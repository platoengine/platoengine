/*
 * XMLGeneratorProblem.hpp
 *
 *  Created on: Nov 20, 2020
 */

#pragma once

#include <iostream>

//#include "XMLGeneratorDefinesFileUtilities.hpp"
//#include "XMLGeneratorLaunchScriptUtilities.hpp"
#include "XMLGeneratorInterfaceFileUtilities.hpp"
//#include "XMLGeneratorRandomInterfaceFileUtilities.hpp"
#include "XMLGeneratorPlatoMainInputFileUtilities.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeOperationsFileUtilities.hpp"
#include "XMLGeneratorAnalyzeUncertaintyLaunchScriptUtilities.hpp"

namespace XMLGen
{

namespace Problem
{

/******************************************************************************//**
 * \fn write_optimization_problem
 * \brief Write input files needed to solve optimization problems with Plato Analyze.
 * \param [in] aInputData input metadata
**********************************************************************************/
inline void write_optimization_problem
(const XMLGen::InputData& aMetaData,
 const std::vector<XMLGen::InputData>& aPreProcessedMetaData)
{
    if(XMLGen::Analyze::is_robust_optimization_problem(aMetaData))
        XMLGen::write_define_xml_file(aMetaData.mRandomMetaData, aMetaData.m_UncertaintyMetaData);

    XMLGen::write_interface_xml_file(aMetaData);
    XMLGen::generate_launch_script(aMetaData);
    XMLGen::write_plato_main_operations_xml_file(aMetaData);
    XMLGen::write_plato_main_input_deck_file(aMetaData);
    XMLGen::write_amgx_input_file(aMetaData);

    for(auto tCurMetaData : aPreProcessedMetaData)
    {
        XMLGen::write_plato_analyze_operation_xml_file(tCurMetaData);
        XMLGen::write_plato_analyze_input_deck_file(tCurMetaData);
    }
}

/*
inline void write_service_operation_files
(const XMLGen::InputData& aMetaData,
 const XMLGen::Service &aService)
{
    if(aService.code() == "platomain")
    {
        XMLGen::write_plato_main_operations_xml_file(aMetaData, aService);

    }
    else if(aService.code() == "plato_analyze")
    {
    }
}
*/

}
// namespace Problem

}
// namespace XMLGen
