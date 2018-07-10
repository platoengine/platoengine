#include "PSL_LinearDistanceVoter.hpp"

#include "PSL_DistanceVoter.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Random.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

LinearDistanceVoter::LinearDistanceVoter(AbstractAuthority* authority) :
        DistanceVoter(authority, distance_voter_t::distance_voter_t::linear_distance_voter),
        m_tolerance(1e-3)
{
}
LinearDistanceVoter::~LinearDistanceVoter()
{
}

int LinearDistanceVoter::choose_output(const int& output_class_size,
                                       const std::vector<double>& distances_per_instance,
                                       const std::vector<int>& classes_per_instance)
{
    std::vector<double> vote_magnitude_per_class(output_class_size, 0.);

    // get size
    const int num_input = distances_per_instance.size();
    assert(num_input == int(classes_per_instance.size()));

    // get max
    const double max_distance = max(distances_per_instance);

    // for each input
    for(int i = 0; i < num_input; i++)
    {
        vote_magnitude_per_class[classes_per_instance[i]] += 1. - (distances_per_instance[i] / (max_distance + m_tolerance));
    }

    return rand_max_index(vote_magnitude_per_class);
}

}
