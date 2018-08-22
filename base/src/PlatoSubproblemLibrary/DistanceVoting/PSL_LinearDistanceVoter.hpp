#pragma once

/* Chooses nearest class by maximizer of a linear distance from center metric.
 */

#include "PSL_DistanceVoter.hpp"
#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;

class LinearDistanceVoter : public DistanceVoter
{
public:
    LinearDistanceVoter(AbstractAuthority* authority);
    virtual ~LinearDistanceVoter();

    virtual int choose_output(const int& output_class_size,
                              const std::vector<double>& distances_per_instance,
                              const std::vector<int>& classes_per_instance);

protected:
    double m_tolerance;

};

}
