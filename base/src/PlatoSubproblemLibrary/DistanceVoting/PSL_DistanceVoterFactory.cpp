#include "PSL_DistanceVoterFactory.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_DistanceVoter.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_CountingVoter.hpp"
#include "PSL_LinearDistanceVoter.hpp"
#include "PSL_InverseDistanceVoter.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{

DistanceVoter* build_distance_voter(const distance_voter_t::distance_voter_t& type, AbstractAuthority* authority)
{
    DistanceVoter* result = NULL;

    switch(type)
    {
        case distance_voter_t::distance_voter_t::counting_voter:
        {
            result = new CountingVoter(authority);
            break;
        }
        case distance_voter_t::distance_voter_t::linear_distance_voter:
        {
            result = new LinearDistanceVoter(authority);
            break;
        }
        case distance_voter_t::distance_voter_t::inverse_distance_voter:
        {
            result = new InverseDistanceVoter(authority);
            break;
        }
        default:
        {
            authority->utilities->fatal_error("PlatoSubproblemLibrary could not match enum to DistanceVoter. Aborting.\n\n");
            break;
        }
    }

    return result;
}

}


