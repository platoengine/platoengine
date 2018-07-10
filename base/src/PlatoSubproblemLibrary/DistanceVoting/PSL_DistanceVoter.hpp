#pragma once

#include "PSL_ParameterDataEnums.hpp"
#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;

class DistanceVoter
{
public:
    DistanceVoter(AbstractAuthority* authority, const distance_voter_t::distance_voter_t& type);
    virtual ~DistanceVoter();
    distance_voter_t::distance_voter_t get_distance_voter_type();

    // implement at least one of these
    virtual int choose_output(const int& output_class_size,
                              const std::vector<double>& distances_per_instance,
                              const std::vector<int>& classes_per_instance);
    virtual int choose_output(const std::vector<int>& instances_per_class,
                              const std::vector<double>& distances_per_instance,
                              const std::vector<int>& classes_per_instance);

protected:
    AbstractAuthority* m_authority;
    distance_voter_t::distance_voter_t m_type;

};

}
