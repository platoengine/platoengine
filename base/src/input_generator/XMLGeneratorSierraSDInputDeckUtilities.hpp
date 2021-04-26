/*
 * XMLGeneratorSierraSDInputDeckUtilities.hpp
 *
 *  Created on: Feb 6, 2021
 */

#pragma once

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


}
// namespace XMLGen
