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
 * \fn is_robust_optimization_problem
 * \brief Write input files needed to solve optimization problems with Plato Analyze.
 * \param [in] aInputData input metadata
**********************************************************************************/
/*
inline bool is_robust_optimization_problem
(const XMLGen::InputData& aMetaData)
{
    if(aMetaData.mRandomMetaData.empty())
        return false;
    else
        return true;
}
*/
// function is_robust_optimization_problem

/******************************************************************************//**
 * \fn write_optimization_problem
 * \brief Write input files needed to solve optimization problems with Plato Analyze.
 * \param [in] aInputData input metadata
**********************************************************************************/
inline void write_optimization_problem
(const XMLGen::InputData& aMetaData)
{
/* There will be cases other than robust opitimzation where we will need to 
 * write the define_xml file so the logic here needs to be expanded
 */
//    if(XMLGen::Analyze::is_robust_optimization_problem(aMetaData))
//        XMLGen::write_define_xml_file(aMetaData.mRandomMetaData, aMetaData.m_UncertaintyMetaData);

    XMLGen::write_interface_xml_file(aMetaData);
    for(auto tService : aMetaData.services())
    {
        XMLGen::write_service_operation_files(aMetaData, tService);
        XMLGen::write_service_input_files(aMetaData, tService);
        XMLGen::write_service_auxilliary_files(aMetaData, tService);
    }
    XMLGen::generate_launch_script(aMetaData);
}

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

}
// namespace Problem

}
// namespace XMLGen
