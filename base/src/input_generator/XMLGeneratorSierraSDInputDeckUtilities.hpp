/*
 * XMLGeneratorSierraSDInputDeckUtilities.hpp
 *
 *  Created on: Feb 6, 2021
 */

#pragma once

#include <fstream>

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{

/******************************************************************************//**
 * \fn write_sierra_sd_operation_xml_file
 * \brief Write SierraSD operations to plato_analyze_operation.xml file.
 * \param [in] aMetaData Plato problem input data
**********************************************************************************/
void write_sierra_sd_input_deck
(const XMLGen::InputData& aXMLMetaData);

void augment_sierra_sd_input_deck_with_plato_problem_description(const XMLGen::InputData &aXMLMetaData, std::istream &inputDeck, std::ostream &outfile);

}
// namespace XMLGen
