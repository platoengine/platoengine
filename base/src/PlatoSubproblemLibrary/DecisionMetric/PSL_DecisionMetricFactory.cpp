#include "PSL_DecisionMetricFactory.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_DecisionMetric.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_GiniImpurityMetric.hpp"
#include "PSL_EntropyMetric.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{

DecisionMetric* build_decision_metric(const decision_metric_t::decision_metric_t& type, AbstractInterface::GlobalUtilities* util)
{
    DecisionMetric* result = NULL;

    switch(type)
    {
        case decision_metric_t::decision_metric_t::gini_impurity_metric:
        {
            result = new GiniImpurityMetric;
            break;
        }
        case decision_metric_t::decision_metric_t::entropy_metric:
        {
            result = new EntropyMetric;
            break;
        }
        default:
        {
            util->fatal_error("PlatoSubproblemLibrary could not match enum to DecisionMetric. Aborting.\n\n");
            break;
        }
    }

    return result;
}

}


