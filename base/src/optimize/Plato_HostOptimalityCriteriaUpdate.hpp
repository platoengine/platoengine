/*
 * Plato_HostOptimalityCriteriaUpdate.hpp
 *
 *  Created on: May 6, 2018
 */

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

#ifndef PLATO_HOSTOPTIMALITYCRITERIAUPDATE_HPP_
#define PLATO_HOSTOPTIMALITYCRITERIAUPDATE_HPP_

#include <cmath>
#include <algorithm>

#include "Plato_Vector.hpp"
#include "Plato_OptimalityCriteriaUpdate.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class HostOptimalityCriteriaUpdate : public OptimalityCriteriaUpdate<ScalarType, OrdinalType>
{
public:
    /********************************************************************************/
    explicit HostOptimalityCriteriaUpdate(const ScalarType & aMoveLimit,
                                          const ScalarType & aScaleFactor,
                                          const ScalarType & aDampingPower) :
            mMoveLimit(aMoveLimit),
            mScaleFactor(aScaleFactor),
            mDampingPower(aDampingPower)
    /********************************************************************************/
    {
    }

    /********************************************************************************/
    virtual ~HostOptimalityCriteriaUpdate()
    /********************************************************************************/
    {
    }

    /********************************************************************************/
    void reset(const ScalarType & aMoveLimit, const ScalarType & aScaleFactor, const ScalarType & aDampingPower)
    /********************************************************************************/
    {
        mMoveLimit = aMoveLimit;
        mScaleFactor = aScaleFactor;
        mDampingPower = aDampingPower;
    }

    /********************************************************************************/
    void update(const ScalarType & aTrialDual,
                const Plato::Vector<ScalarType, OrdinalType> & aLowerBounds,
                const Plato::Vector<ScalarType, OrdinalType> & aUpperBounds,
                const Plato::Vector<ScalarType, OrdinalType> & aPreviousControls,
                const Plato::Vector<ScalarType, OrdinalType> & aObjectiveGradient,
                const Plato::Vector<ScalarType, OrdinalType> & aInequalityGradient,
                Plato::Vector<ScalarType, OrdinalType> & aTrialControls)
    /********************************************************************************/
    {
        OrdinalType tNumControls = aPreviousControls.size();
        for(OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
        {
            if(aInequalityGradient[tControlIndex] == static_cast<ScalarType>(0))
            {
                aTrialControls[tControlIndex] = aPreviousControls[tControlIndex];
            }
            else
            {
                ScalarType tMyDesignVariableOffset = ( mScaleFactor * ( aUpperBounds[tControlIndex]
                        - aLowerBounds[tControlIndex] ) ) - aLowerBounds[tControlIndex];
                ScalarType tMyValue = -aObjectiveGradient[tControlIndex]
                        / (aTrialDual * aInequalityGradient[tControlIndex]);
                ScalarType tFabsValue = std::abs(tMyValue);
                ScalarType tSignValue = copysign(1.0, tMyValue);
                ScalarType tMyTrialControlValue = ((aPreviousControls[tControlIndex] + tMyDesignVariableOffset)
                        * tSignValue * std::pow(tFabsValue, mDampingPower)) - tMyDesignVariableOffset;

                ScalarType tMyControlValue = aPreviousControls[tControlIndex] + mMoveLimit;
                tMyTrialControlValue = std::min(tMyControlValue, tMyTrialControlValue);
                tMyControlValue = aPreviousControls[tControlIndex] - mMoveLimit;
                tMyTrialControlValue = std::max(tMyControlValue, tMyTrialControlValue);
                tMyTrialControlValue = std::min(aUpperBounds[tControlIndex], tMyTrialControlValue);
                tMyTrialControlValue = std::max(aLowerBounds[tControlIndex], tMyTrialControlValue);
                aTrialControls[tControlIndex] = tMyTrialControlValue;
            }
        }
    }

    /********************************************************************************/
    std::shared_ptr<Plato::OptimalityCriteriaUpdate<ScalarType, OrdinalType>> create() const
        /********************************************************************************/
    {
        std::shared_ptr<Plato::OptimalityCriteriaUpdate<ScalarType, OrdinalType>> tCopy =
                std::make_shared<HostOptimalityCriteriaUpdate<ScalarType, OrdinalType>>(mMoveLimit, mScaleFactor, mDampingPower);
        return (tCopy);
    }

private:
    ScalarType mMoveLimit;
    ScalarType mScaleFactor;
    ScalarType mDampingPower;

private:
    HostOptimalityCriteriaUpdate(const Plato::HostOptimalityCriteriaUpdate<ScalarType, OrdinalType>&);
    Plato::HostOptimalityCriteriaUpdate<ScalarType, OrdinalType> & operator=(const Plato::HostOptimalityCriteriaUpdate<ScalarType, OrdinalType>&);
};
// class HostOptimalityCriteriaUpdate

} // namespace Plato

#endif /* PLATO_HOSTOPTIMALITYCRITERIAUPDATE_HPP_ */
