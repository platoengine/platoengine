/*
 * XMLGeneratorBoundaryMetadata.hpp
 *
 *  Created on: May 14, 2020
 */

#pragma once

#include <string>
#include <vector>

namespace XMLGen
{

struct Load
{
    bool mIsRandom = false;
    std::string mName;
    std::string type; // traction, heat flux, force, pressure ...
    std::string app_type; // nodeset or sideset
    std::string app_id; // nodeset/sideset id
    std::string app_name; // nodeset/sideset name
    std::vector<std::string> values;
    std::string dof;  // x, y, or z
    std::string load_id;
};

struct LoadCase
{
    std::vector<XMLGen::Load> loads;
    std::string id; // note LoadCase id is not necessarily shared with its Loads
};

struct BC
{
    std::string type;     // temperature, displacement
    std::string app_type; // nodeset or sideset
    std::string app_id; // nodeset/sideset id
    std::string app_name; // nodeset/sideset name
    std::string dof;
    std::string bc_id;
    std::string value;
};

}
// namespace XMLGen
