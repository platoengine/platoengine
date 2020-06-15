/*
 * XMLGeneratorPlatoAnalyzeProblem.hpp
 *
 *  Created on: Jun 15, 2020
 */

#pragma once

#include <iostream>

#include "XMLGeneratorDefinesFileUtilities.hpp"
#include "XMLGeneratorLaunchScriptUtilities.hpp"
#include "XMLGeneratorAnalyzeUncertaintyLaunchScriptUtilities.hpp"
#include "XMLGeneratorRandomInterfaceFileUtilities.hpp"
#include "XMLGeneratorPlatoMainInputFileUtilities.hpp"
#include "XMLGeneratorPlatoMainOperationFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeInputFileUtilities.hpp"
#include "XMLGeneratorPlatoAnalyzeOperationsFileUtilities.hpp"

namespace XMLGen
{

namespace Analyze
{

inline void write_plato_analyze_optimization_problem
(const XMLGen::InputData& aInputData)
{
    XMLGen::write_define_xml_file(aInputData.mRandomMetaData, aInputData.m_UncertaintyMetaData);
    XMLGen::write_interface_xml_file_for_nondeterministic_usecase(aInputData);
    XMLGen::write_plato_main_operations_xml_file_for_nondeterministic_usecase(aInputData);
    XMLGen::write_plato_main_input_deck_file(aInputData);
    XMLGen::write_plato_analyze_operation_xml_file_for_nondeterministic_usecase(aInputData);
    XMLGen::write_amgx_input_file();
    XMLGen::write_plato_analyze_input_deck_file(aInputData);
    XMLGen::generate_launch_script(aInputData);
    std::cout << "Successfully wrote XML files." << std::endl;
}
// function write_plato_analyze_optimization_problem

}
// namespace Analyze

}
// namespace XMLGen
