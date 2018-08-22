#include "PSL_AlgebraicFunction.hpp"

#include "PSL_InvertibleUnitRange.hpp"

#include <cmath>
#include <math.h>
#include <cassert>

namespace PlatoSubproblemLibrary
{

AlgebraicFunction::AlgebraicFunction(const double& negative_scale, const double& positive_scale) :
        InvertibleUnitRange(invertible_unit_range_t::invertible_unit_range_t::algebraic_function, negative_scale, positive_scale)
{

}
AlgebraicFunction::~AlgebraicFunction()
{
}

double AlgebraicFunction::evaluate(const double& x)
{
    const double scaled_x = x * (x >= 0. ? m_positive_scale : m_negative_scale);
    const double sqrt_term = sqrt(1. + scaled_x * scaled_x);
    return 0.5 * (scaled_x + sqrt_term) / sqrt_term;
}
double AlgebraicFunction::invert(const double& y)
{
    assert(0.<y);
    assert(y<1.);
    const double scale = (y >= .5 ? m_positive_scale : m_negative_scale);
    return (1. / (2. * scale)) * (2. * y - 1.) / sqrt(y * (1. - y));
}

}

