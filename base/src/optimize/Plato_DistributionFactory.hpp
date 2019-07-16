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
 * Plato_DistributionFactory.hpp
 *
 *  Created on: Aug 28, 2018
 */

#ifndef PLATO_DISTRIBUTIONFACTORY_HPP_
#define PLATO_DISTRIBUTIONFACTORY_HPP_

#include "Plato_Macros.hpp"
#include "Plato_BetaDistribution.hpp"
#include "Plato_SromProbDataStruct.hpp"
#include "Plato_NormalDistribution.hpp"
#include "Plato_UniformDistribution.hpp"

#include <cmath>
#include <stdlib.h>

namespace Plato
{

// factory constructor
template<typename ScalarType, typename OrdinalType>
std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>>
build_distrubtion(const Plato::SromInputs<ScalarType, OrdinalType>& aInput)
{
    // grab values from input struct
    const ScalarType tMean = aInput.mMean;
    const ScalarType tMax = aInput.mUpperBound;
    const ScalarType tMin = aInput.mLowerBound;
    const ScalarType tVariance = aInput.mVariance;
    const ScalarType tStandardDeviation = std::sqrt(tVariance);

    // make
    std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> tDistribution;
    switch(aInput.mDistribution)
    {
        case DistrubtionName::type_t::beta:
        {
            tDistribution = std::make_shared < Plato::BetaDistribution<ScalarType, OrdinalType> > (tMin, tMax, tMean, tVariance);
            break;
        }
        case DistrubtionName::type_t::normal:
        {
            tDistribution = std::make_shared < Plato::NormalDistribution<ScalarType, OrdinalType> > (tMean, tStandardDeviation);
            break;
        }
        case DistrubtionName::type_t::uniform:
        {
            tDistribution = std::make_shared < Plato::UniformDistribution<ScalarType, OrdinalType> > (tMin, tMax);
            break;
        }
        default:
        {
            THROWERR("UNDEFINED DISTRIBUTION. OPTIONS ARE: BETA, NORMAL AND UNIFORM\n")
            break;
        }
    }

    return tDistribution;
}
// function build_distrubtion

} // namespace Plato

#endif /* PLATO_DISTRIBUTIONFACTORY_HPP_ */
