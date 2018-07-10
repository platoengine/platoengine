#pragma once

#include "PSL_DiscreteGlobalOptimizer.hpp"
#include <vector>

namespace PlatoSubproblemLibrary
{
class AbstractAuthority;

class RandomSearch : public DiscreteGlobalOptimizer
{
public:
    RandomSearch(AbstractAuthority* authority);
    virtual ~RandomSearch();

    void default_parameters();
    void set_num_trials(int p) { m_num_trials = p; }

    virtual double find_min(std::vector<double>& best_parameters);

protected:
    int m_num_trials;

};

}
