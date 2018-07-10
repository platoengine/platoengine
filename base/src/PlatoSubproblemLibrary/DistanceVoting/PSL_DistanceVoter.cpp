#include "PSL_DistanceVoter.hpp"
#include "PSL_AbstractAuthority.hpp"

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{

DistanceVoter::DistanceVoter(AbstractAuthority* authority, const distance_voter_t::distance_voter_t& type) :
        m_authority(authority),
        m_type(type)
{
}
DistanceVoter::~DistanceVoter()
{
}
distance_voter_t::distance_voter_t DistanceVoter::get_distance_voter_type()
{
    return m_type;
}

int DistanceVoter::choose_output(const int& output_class_size,
                                 const std::vector<double>& distances_per_instance,
                                 const std::vector<int>& classes_per_instance)
{
    const int num_input = distances_per_instance.size();
    assert(num_input == int(classes_per_instance.size()));

    // count each class
    std::vector<int> class_count(output_class_size, 0);
    for(int i = 0; i < num_input; i++)
    {
        assert(0 <= classes_per_instance[i]);
        assert(classes_per_instance[i] < output_class_size);
        class_count[classes_per_instance[i]]++;
    }

    return choose_output(class_count, distances_per_instance, classes_per_instance);
}
int DistanceVoter::choose_output(const std::vector<int>& instances_per_class,
                                         const std::vector<double>& distances_per_instance,
                                         const std::vector<int>& classes_per_instance)
{
    return choose_output(instances_per_class.size(), distances_per_instance, classes_per_instance);
}

}
