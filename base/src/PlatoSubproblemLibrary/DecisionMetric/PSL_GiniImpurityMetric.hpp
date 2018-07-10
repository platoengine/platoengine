#pragma once

#include "PSL_DecisionMetric.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class GiniImpurityMetric : public DecisionMetric
{
public:
    GiniImpurityMetric();
    virtual ~GiniImpurityMetric();

    virtual double measure(const std::vector<int>& num_class_members);
    virtual double lower_cutoff_upscale();

protected:

};

}
