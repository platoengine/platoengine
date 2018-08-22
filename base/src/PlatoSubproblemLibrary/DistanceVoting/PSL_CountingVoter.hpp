#pragma once

/* chooses most frequent occurrence.
 */

#include "PSL_DistanceVoter.hpp"
#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;

class CountingVoter : public DistanceVoter
{
public:
    CountingVoter(AbstractAuthority* authority);
    virtual ~CountingVoter();

    // implement at least one of these
    virtual int choose_output(const std::vector<int>& instances_per_class,
                              const std::vector<double>& distances_per_instance,
                              const std::vector<int>& classes_per_instance);

protected:

};

}
