#include "PSL_GiniImpurityMetric.hpp"

#include "PSL_DecisionMetric.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_FreeHelpers.hpp"

#include <vector>
#include <cstddef>
#include <algorithm>

namespace PlatoSubproblemLibrary
{

GiniImpurityMetric::GiniImpurityMetric() :
        DecisionMetric(decision_metric_t::decision_metric_t::gini_impurity_metric)
{
}

GiniImpurityMetric::~GiniImpurityMetric()
{
}

double GiniImpurityMetric::measure(const std::vector<int>& num_class_members)
{
    const size_t num_classes = num_class_members.size();
    const double population_size = sum(num_class_members);
    if(population_size < 1.)
    {
        return 0.;
    }

    std::vector<double> squared_class_probabilities(num_classes);
    for(size_t c = 0u; c < num_classes; c++)
    {
        const double prob = double(num_class_members[c]) / population_size;
        squared_class_probabilities[c] = prob * prob;
    }

    const double impurity = 1. - sum(squared_class_probabilities);
    return impurity;
}

double GiniImpurityMetric::lower_cutoff_upscale()
{
    return 1.;
}

}
