/*
 * Plato_DeviceOptimalityCriteriaUpdate.hpp
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

#ifndef PLATO_DEVICEOPTIMALITYCRITERIAUPDATE_HPP_
#define PLATO_DEVICEOPTIMALITYCRITERIAUPDATE_HPP_

#include <cmath>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_KokkosTypes.hpp"
#include "Plato_OptimalityCriteriaUpdate.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class DeviceOptimalityCriteriaUpdate : public OptimalityCriteriaUpdate<ScalarType, OrdinalType>
{
public:
    /********************************************************************************/
    DeviceOptimalityCriteriaUpdate(const ScalarType & aMoveLimit,
                                   const ScalarType & aScaleFactor,
                                   const ScalarType & aDampingPower) :
            mMoveLimit(aMoveLimit),
            mScaleFactor(aScaleFactor),
            mDampingPower(aDampingPower)
    /********************************************************************************/
    {
    }

    /********************************************************************************/
    virtual ~DeviceOptimalityCriteriaUpdate()
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
        assert(aLowerBounds.size() == aUpperBounds.size());
        assert(aLowerBounds.size() == aTrialControls.size());
        assert(aLowerBounds.size() == aPreviousControls.size());
        assert(aLowerBounds.size() == aObjectiveGradient.size());
        assert(aLowerBounds.size() == aInequalityGradient.size());

        ScalarType* tTrialControls = aTrialControls.data();
        const ScalarType* tLowerBounds = aLowerBounds.data();
        const ScalarType* tUpperBounds = aUpperBounds.data();
        const ScalarType* tObjGradient = aObjectiveGradient.data();
        const ScalarType* tPrevControls = aPreviousControls.data();
        const ScalarType* tInqGradient = aInequalityGradient.data();

        ScalarType tMoveLimit = mMoveLimit;
        ScalarType tScaleFactor = mScaleFactor;
        ScalarType tDampingPower = mDampingPower;
        OrdinalType tNumControls = aTrialControls.size();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tNumControls), KOKKOS_LAMBDA(const OrdinalType & aControlIndex)
        {
            ScalarType tOffset = ( tScaleFactor * ( tUpperBounds[aControlIndex] - tLowerBounds[aControlIndex] ) )
                  - tLowerBounds[aControlIndex];
            ScalarType tMyValue = -tObjGradient[aControlIndex] / (aTrialDual * tInqGradient[aControlIndex]);
            ScalarType tFabsValue = fabs(tMyValue);
            ScalarType tSignValue = copysign(1.0, tMyValue);
            ScalarType tTrialControlVal = ((tPrevControls[aControlIndex] + tOffset)
                    * tSignValue * pow(tFabsValue, tDampingPower)) - tOffset;

            ScalarType tPrevControlValue = tPrevControls[aControlIndex] + tMoveLimit;
            tTrialControlVal = fmin(tPrevControlValue, tTrialControlVal);
            tPrevControlValue = tPrevControls[aControlIndex] - tMoveLimit;
            tTrialControlVal = fmax(tPrevControlValue, tTrialControlVal);
            tTrialControlVal = fmin(tUpperBounds[aControlIndex], tTrialControlVal);
            tTrialControlVal = fmax(tLowerBounds[aControlIndex], tTrialControlVal);
            tTrialControls[aControlIndex] = tTrialControlVal;
        }, "OptimalityCriteriaSubProblem::KokkosUpdate");
    }

    /********************************************************************************/
    std::shared_ptr<Plato::OptimalityCriteriaUpdate<ScalarType, OrdinalType>> create() const
        /********************************************************************************/
    {
        std::shared_ptr<Plato::OptimalityCriteriaUpdate<ScalarType, OrdinalType>> tCopy =
                std::make_shared<DeviceOptimalityCriteriaUpdate<ScalarType, OrdinalType>>(mMoveLimit, mScaleFactor, mDampingPower);
        return (tCopy);
    }

private:
    ScalarType mMoveLimit;
    ScalarType mScaleFactor;
    ScalarType mDampingPower;

private:
    DeviceOptimalityCriteriaUpdate(const Plato::DeviceOptimalityCriteriaUpdate<ScalarType, OrdinalType>&);
    Plato::DeviceOptimalityCriteriaUpdate<ScalarType, OrdinalType> & operator=(const Plato::DeviceOptimalityCriteriaUpdate<ScalarType, OrdinalType>&);
};
// class DeviceOptimalityCriteriaUpdate

} // namespace Plato

#endif /* PLATO_DEVICEOPTIMALITYCRITERIAUPDATE_HPP_ */
