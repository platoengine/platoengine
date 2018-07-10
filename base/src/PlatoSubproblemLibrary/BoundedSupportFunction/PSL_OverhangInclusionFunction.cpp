// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#include "PSL_OverhangInclusionFunction.hpp"

#include "PSL_Point.hpp"
#include "PSL_Abstract_BoundedSupportFunction.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_FreeHelpers.hpp"

#include <iostream>
#include <math.h>
#include <cmath> // for pow, sqrt, fabs

namespace PlatoSubproblemLibrary
{

OverhangInclusionFunction::OverhangInclusionFunction() :
        Abstract_BoundedSupportFunction(),
        m_radius(-1.),
        m_radius_squared(-1.),
        m_max_overhang_angle(-1.),
        m_normalized_build_direction(NULL)
{
}

OverhangInclusionFunction::~OverhangInclusionFunction()
{
    safe_free(m_normalized_build_direction);
}

void OverhangInclusionFunction::build(double support, ParameterData* input_data)
{
    m_radius = support;
    m_radius_squared = m_radius * m_radius;
    m_max_overhang_angle = input_data->get_max_overhang_angle();

    // normalize build direction
    const double build_direction_x = input_data->get_build_direction_x();
    const double build_direction_y = input_data->get_build_direction_y();
    const double build_direction_z = input_data->get_build_direction_z();
    const double build_norm = sqrt(build_direction_x * build_direction_x + build_direction_y * build_direction_y
                                   + build_direction_z * build_direction_z);
    assert(build_norm != 0.);
    const std::vector<double> nbd = {build_direction_x / build_norm, build_direction_y / build_norm, build_direction_z
                                                                                                     / build_norm};
    m_normalized_build_direction = new Point(0u, nbd);
}

double OverhangInclusionFunction::evaluate(Point* center, Point* other)
{
    double result_squared = pow((*center)(0u) - (*other)(0u), 2.) + pow((*center)(1u) - (*other)(1u), 2.)
                            + pow((*center)(2u) - (*other)(2u), 2.);

    if(result_squared < m_radius_squared)
    {
        const double overhang = compute_overhang_angle(center, other, m_normalized_build_direction);
        if(overhang < m_max_overhang_angle)
        {
            return 1.;
        }
    }
    return -1.;
}

double OverhangInclusionFunction::get_support()
{
    return m_radius;
}

}
