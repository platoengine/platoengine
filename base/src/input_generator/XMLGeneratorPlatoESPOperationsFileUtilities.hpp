/*
 * XMLGeneratorPlatoESPOperationsFileUtilities.hpp
 *
 *  Created on: Jan 27, 2021
 */

#pragma once

#include "pugixml.hpp"

#include "XMLGeneratorDataStruct.hpp"

namespace XMLGen
{
  void write_plato_esp_operations_file(const XMLGen::InputData& aInputData);
}
// namespace XMLGen
