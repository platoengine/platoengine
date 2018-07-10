#include "PSL_InverseDistanceVoter.hpp"

#include "PSL_DistanceVoter.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Random.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

InverseDistanceVoter::InverseDistanceVoter(AbstractAuthority* authority) :
        DistanceVoter(authority, distance_voter_t::distance_voter_t::inverse_distance_voter),
        m_tolerance(1e-3)
{
}
InverseDistanceVoter::~InverseDistanceVoter()
{
}

int InverseDistanceVoter::choose_output(const int& output_class_size,
                                        const std::vector<double>& distances_per_instance,
                                        const std::vector<int>& classes_per_instance)
{
    std::vector<double> vote_magnitude_per_class(output_class_size, 0.);

    // get size
    const int num_input = distances_per_instance.size();
    assert(num_input == int(classes_per_instance.size()));

    // for each input
    for(int i = 0; i < num_input; i++)
    {
        vote_magnitude_per_class[classes_per_instance[i]] += 1. / (distances_per_instance[i] + m_tolerance);
    }

    return rand_max_index(vote_magnitude_per_class);
}

}
