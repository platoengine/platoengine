#pragma once

/* Abstract builder function to take enumeration to build class.
 *
 */

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;
class DistanceVoter;

DistanceVoter* build_distance_voter(const distance_voter_t::distance_voter_t& type, AbstractAuthority* authority);

}
