#pragma once

/* A collection of builder factories for Data.
 */

#include "PSL_ParameterDataEnums.hpp"
#include <string>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class DataSequence;
class DataFlow;
class Classifier;

DataSequence* load_data_sequence(const std::string& filename, AbstractAuthority* authority);

Classifier* build_classifier(const data_flow_t::data_flow_t& flow_type, AbstractAuthority* authority);
DataFlow* build_data_flow(const data_flow_t::data_flow_t& flow_type, AbstractAuthority* authority);

DataFlow* load_data_flow(const std::string& filename, AbstractAuthority* authority);
Classifier* load_classifier(const std::string& filename, AbstractAuthority* authority);

}
