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

