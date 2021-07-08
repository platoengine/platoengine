/*
 * XMLGeneratorProblem.hpp
 *
 *  Created on: Nov 20, 2020
 */

#pragma once

#include <iostream>

#include "XMLGeneratorInterfaceFileUtilities.hpp"
#include "XMLGeneratorPlatoMainInputFileUtilities.hpp"
#include "XMLGeneratorPlatoESPInputFileUtilities.hpp"
#include "XMLGeneratorPlatoESPOperationsFileUtilities.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorSierraSDOperationsFileUtilities.hpp"
#include "XMLGeneratorSierraSDInputDeckUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeOperationsFileUtilities.hpp"
#include "XMLGeneratorAnalyzeUncertaintyLaunchScriptUtilities.hpp"

namespace XMLGen
{

namespace Problem
{


/******************************************************************************//**
 * \fn write_performer_operation_xml_file
 * \brief Write the operations file for the performer in the metada
 * \param [in] aInputData input metadata
**********************************************************************************/
inline void write_performer_operation_xml_file
(const XMLGen::InputData& aMetaData)
{
    if(aMetaData.services().size() > 0)
    {
        if(aMetaData.services()[0].code() == "plato_analyze")
        {
            if(aMetaData.services()[0].id() == "helmholtz")
            {
                XMLGen::write_plato_analyze_helmholtz_operation_xml_file(aMetaData);
            }
            else
            {
                XMLGen::write_plato_analyze_operation_xml_file(aMetaData);
            }
        }
        else if(aMetaData.services()[0].code() == "sierra_sd")
        {
            XMLGen::write_sierra_sd_operation_xml_file(aMetaData);
        }
    }
}

/******************************************************************************//**
 * \fn write_performer_input_deck_file
 * \brief Write the input deck file for the performer in the metada
 * \param [in] aInputData input metadata
**********************************************************************************/
inline void write_performer_input_deck_file
(const XMLGen::InputData& aMetaData)
{
    if(aMetaData.services().size() > 0)
    {
        if(aMetaData.services()[0].code() == "plato_analyze")
        {
            if(aMetaData.services()[0].id() == "helmholtz")
            {
                XMLGen::write_plato_analyze_helmholtz_input_deck_file(aMetaData);
            }
            else
            {
                XMLGen::write_plato_analyze_input_deck_file(aMetaData);
            }
        }
        else if(aMetaData.services()[0].code() == "sierra_sd")
        {
            XMLGen::write_sierra_sd_input_deck(aMetaData);
        }
    }
}

/******************************************************************************//**
 * \fn write_optimization_problem
 * \brief Write input files needed to solve optimization problems with Plato Analyze.
 * \param [in] aInputData input metadata
**********************************************************************************/
inline void write_optimization_problem
(const XMLGen::InputData& aMetaData,
 const std::vector<XMLGen::InputData>& aPreProcessedMetaData)
{
    XMLGen::write_define_xml_file(aMetaData);
    XMLGen::write_interface_xml_file(aMetaData);
    XMLGen::generate_launch_script(aMetaData);
    XMLGen::write_plato_main_operations_xml_file(aMetaData);
    XMLGen::write_plato_main_input_deck_file(aMetaData);
    XMLGen::write_amgx_input_file(aMetaData);
    XMLGen::write_plato_esp_input_deck_file(aMetaData);
    XMLGen::write_plato_esp_operations_file(aMetaData);

    for(auto tCurMetaData : aPreProcessedMetaData)
    {
        write_performer_operation_xml_file(tCurMetaData);
        write_performer_input_deck_file(tCurMetaData);
    }
}

}
// namespace Problem

}
// namespace XMLGen
