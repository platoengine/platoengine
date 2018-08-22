#pragma once

/* Entropy-based decision metric.
 */

#include "PSL_DecisionMetric.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class EntropyMetric : public DecisionMetric
{
public:
    EntropyMetric();
    virtual ~EntropyMetric();

    virtual double measure(const std::vector<int>& num_class_members);
    virtual double lower_cutoff_upscale();

protected:

};

}
