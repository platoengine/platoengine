/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
*/

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
