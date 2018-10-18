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

/*
 * Plato_TrustRegionUtilities.hpp
 *
 *  Created on: Oct 18, 2018
 */

#pragma once

#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Compute Cauchy point: \$f p_{C} = -1.0 \frac{\Delta}{\Vert{g}\Vert} g\$f
 * @param [in] aTrustRegionRadius trust region radius (\$f\Delta\$f)
 * @param [in] aGradient gradient (\$f -1.0*g \$f)
 * @param [out] aCauchyPoint Cauchy point () (\$f p_{C} \$f)
**********************************************************************************/
template<typename ScalarType>
void compute_cauchy_point(const ScalarType & aTrustRegionRadius,
                          const Plato::MultiVector<ScalarType> & aGradient,
                          Plato::MultiVector<ScalarType> & aCauchyPoint)
{
    // cauchy_point = -1.0 * Trust_Region_Radius * (gradient/norm(gradient))
    Plato::update(static_cast<ScalarType>(1), aGradient, static_cast<ScalarType>(0), aCauchyPoint);
    ScalarType tNormGradient = Plato::norm(aGradient);
    ScalarType tCauchyScale = static_cast<ScalarType>(-1.0) * aTrustRegionRadius / tNormGradient;
    Plato::scale(tCauchyScale, aCauchyPoint);
}
// function compute_cauchy_point

} // namespace Plato
