#include "PSL_LogisticFunction.hpp"

#include "PSL_InvertibleUnitRange.hpp"

#include <cmath>
#include <math.h>
#include <cassert>

namespace PlatoSubproblemLibrary
{

LogisticFunction::LogisticFunction(const double& negative_scale, const double& positive_scale) :
        InvertibleUnitRange(invertible_unit_range_t::invertible_unit_range_t::logistic_function, negative_scale, positive_scale)
{

}
LogisticFunction::~LogisticFunction()
{
}

double LogisticFunction::evaluate(const double& x)
{
    const double scale = (x >= 0. ? m_positive_scale : m_negative_scale);
    return 1. / (1. + exp(-scale * x));
}
double LogisticFunction::invert(const double& y)
{
    assert(0.<y);
    assert(y<1.);
    const double scale = (y >= .5 ? m_positive_scale : m_negative_scale);
    return (1. / scale) * log(y / (1. - y));
}

}

