#pragma once

#include "PSL_ParameterDataEnums.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

class DecisionMetric
{
public:
    DecisionMetric(decision_metric_t::decision_metric_t type);
    virtual ~DecisionMetric();
    decision_metric_t::decision_metric_t get_type();

    virtual double measure(const std::vector<int>& num_class_members) = 0;
    virtual double lower_cutoff_upscale() = 0;

protected:
    decision_metric_t::decision_metric_t m_type;

};

}

