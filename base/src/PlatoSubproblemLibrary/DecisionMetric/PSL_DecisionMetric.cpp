#include "PSL_DecisionMetric.hpp"

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{

DecisionMetric::DecisionMetric(decision_metric_t::decision_metric_t type) :
        m_type(type)
{
}
DecisionMetric::~DecisionMetric()
{
}
decision_metric_t::decision_metric_t DecisionMetric::get_type()
{
    return m_type;
}

}


