#pragma once

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class Preprocessor;
class DataSequence;

Preprocessor* build_preprocessor(const preprocessor_t::preprocessor_t& type,
                                 AbstractAuthority* authority,
                                 DataSequence* sequence);

}
