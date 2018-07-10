// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#include "PSL_PolynomialTentFunction.hpp"

#include "PSL_Point.hpp"
#include "PSL_Abstract_BoundedSupportFunction.hpp"
#include "PSL_ParameterData.hpp"

#include <cmath> // for pow, sqrt, fabs

namespace PlatoSubproblemLibrary
{

PolynomialTentFunction::PolynomialTentFunction() :
        Abstract_BoundedSupportFunction(),
        m_radius(-1),
        m_radius_squared(-1),
        m_weighting_penalty(-1)
{
}

PolynomialTentFunction::~PolynomialTentFunction()
{
}

void PolynomialTentFunction::build(double support, ParameterData* input_data)
{
    m_radius = support;
    m_radius_squared = m_radius * m_radius;
    m_weighting_penalty = input_data->get_penalty();
}

double PolynomialTentFunction::evaluate(Point* center, Point* other)
{
    double result_squared = pow((*center)(0u) - (*other)(0u), 2.)
                            + pow((*center)(1u) - (*other)(1u), 2.)
                            + pow((*center)(2u) - (*other)(2u), 2.);

    if(result_squared < m_radius_squared)
    {
        return pow(m_radius - sqrt(result_squared), m_weighting_penalty);
    }
    return -1.;
}

double PolynomialTentFunction::get_support()
{
    return m_radius;
}

}
