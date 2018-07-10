#include "PSL_EntropyMetric.hpp"

#include "PSL_DecisionMetric.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_FreeHelpers.hpp"

#include <vector>
#include <cmath>
#include <math.h>
#include <cstddef>
#include <algorithm>

namespace PlatoSubproblemLibrary
{

EntropyMetric::EntropyMetric() :
        DecisionMetric(decision_metric_t::decision_metric_t::entropy_metric)
{
}

EntropyMetric::~EntropyMetric()
{
}

double EntropyMetric::measure(const std::vector<int>& num_class_members)
{
    const size_t num_classes = num_class_members.size();
    const double population_size = std::max(sum(num_class_members), 1);

    std::vector<double> p_log_p(num_classes);
    for(size_t c = 0u; c < num_classes; c++)
    {
        assert(0 <= num_class_members[c]);
        if(num_class_members[c] == 0)
        {
            p_log_p[c] = 0.;
        }
        else
        {
            const double prob = double(num_class_members[c]) / population_size;
            p_log_p[c] = prob * log2(prob);
        }
    }

    const double entropy = -1. * sum(p_log_p);
    return entropy;
}

double EntropyMetric::lower_cutoff_upscale()
{
    return 3.;
}

}
