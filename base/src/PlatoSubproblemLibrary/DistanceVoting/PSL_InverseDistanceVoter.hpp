#pragma once

#include "PSL_DistanceVoter.hpp"
#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;

class InverseDistanceVoter : public DistanceVoter
{
public:
    InverseDistanceVoter(AbstractAuthority* authority);
    virtual ~InverseDistanceVoter();

    // implement at least one of these
    virtual int choose_output(const int& output_class_size,
                              const std::vector<double>& distances_per_instance,
                              const std::vector<int>& classes_per_instance);

protected:
    double m_tolerance;

};

}
