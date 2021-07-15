/*
 * XMLGeneratorBoundaryMetadata.hpp
 *
 *  Created on: May 14, 2020
 */

#pragma once

#include <string>
#include <vector>
#include "XMLGeneratorLoadMetadata.hpp"

namespace XMLGen
{

struct LoadCase
{
    std::vector<XMLGen::Load> loads;
    std::string id; // Same as Scenario id these loads belong to
};

}
// namespace XMLGen
