#pragma once

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class GlobalUtilities;
}
class DecisionMetric;

DecisionMetric* build_decision_metric(const decision_metric_t::decision_metric_t& type, AbstractInterface::GlobalUtilities* util);

}
