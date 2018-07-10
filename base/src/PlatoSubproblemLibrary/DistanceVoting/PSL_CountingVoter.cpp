#include "PSL_CountingVoter.hpp"

#include "PSL_DistanceVoter.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Random.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

CountingVoter::CountingVoter(AbstractAuthority* authority) :
        DistanceVoter(authority, distance_voter_t::distance_voter_t::counting_voter)
{
}
CountingVoter::~CountingVoter()
{
}

int CountingVoter::choose_output(const std::vector<int>& instances_per_class,
                                 const std::vector<double>& distances_per_instance,
                                 const std::vector<int>& classes_per_instance)
{
    return rand_max_index(instances_per_class);
}

}
