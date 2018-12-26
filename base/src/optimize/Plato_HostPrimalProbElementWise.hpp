/*
 * Plato_HostPrimalProbElementWise.hpp
 *
 *  Created on: May 8, 2018
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

#ifndef PLATO_HOSTPRIMALPROBELEMENTWISE_HPP_
#define PLATO_HOSTPRIMALPROBELEMENTWISE_HPP_

#include <cmath>
#include <cassert>
#include <algorithm>

#include "Plato_PrimalProbElementWiseFunctions.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class HostPrimalProbElementWise : public Plato::PrimalProbElementWiseFunctions<ScalarType, OrdinalType>
{
public:
    HostPrimalProbElementWise()
    {
    }
    virtual ~HostPrimalProbElementWise()
    {
    }

    void updateSigmaCoeff(const std::map<Plato::element_wise::constant_t, ScalarType> & aConstants,
                          const Plato::Vector<ScalarType, OrdinalType> & aCurrentControls,
                          const Plato::Vector<ScalarType, OrdinalType> & aPreviousControls,
                          const Plato::Vector<ScalarType, OrdinalType> & aAntepenultimateControls,
                          const Plato::Vector<ScalarType, OrdinalType> & aUpperBounds,
                          const Plato::Vector<ScalarType, OrdinalType> & aLowerBounds,
                          const Plato::Vector<ScalarType, OrdinalType> & aPreviousSigma,
                          Plato::Vector<ScalarType, OrdinalType> & aCurrentSigma) const
    {
        assert(aCurrentSigma.size() == aUpperBounds.size());
        assert(aCurrentSigma.size() == aLowerBounds.size());
        assert(aCurrentSigma.size() == aCurrentSigma.size());
        assert(aCurrentSigma.size() == aPreviousSigma.size());
        assert(aCurrentSigma.size() == aCurrentControls.size());
        assert(aCurrentSigma.size() == aPreviousControls.size());
        assert(aCurrentSigma.size() == aAntepenultimateControls.size());

        const ScalarType tExpansionFactor = aConstants.find(Plato::element_wise::ExpansionFactor)->second;
        const ScalarType tContractionFactor = aConstants.find(Plato::element_wise::ContractionFactor)->second;
        const ScalarType tLowerBoundScaleFactor = aConstants.find(Plato::element_wise::LowerBoundScaleFactor)->second;
        const ScalarType tUpperBoundScaleFactor = aConstants.find(Plato::element_wise::UpperBoundScaleFactor)->second;

        const OrdinalType tNumberElements = aCurrentSigma.size();
        for(OrdinalType tIndex = 0; tIndex < tNumberElements; tIndex++)
        {
            ScalarType tValue = (aCurrentControls[tIndex] - aPreviousControls[tIndex])
                    * (aPreviousControls[tIndex] - aAntepenultimateControls[tIndex]);

            if(tValue > static_cast<ScalarType>(0))
            {
                aCurrentSigma[tIndex] = tExpansionFactor * aPreviousSigma[tIndex];
            }
            else if(tValue < static_cast<ScalarType>(0))
            {
                aCurrentSigma[tIndex] = tContractionFactor * aPreviousSigma[tIndex];
            }

            // check that lower bound is satisfied
            const ScalarType tBoundsMisfit = aUpperBounds[tIndex] - aLowerBounds[tIndex];
            ScalarType tSlopeValue = tLowerBoundScaleFactor * tBoundsMisfit;
            tValue = tBoundsMisfit <= static_cast<ScalarType>(0) ? tLowerBoundScaleFactor : tSlopeValue;
            aCurrentSigma[tIndex] = std::max(tValue, aCurrentSigma[tIndex]);

            // check that upper bound is satisfied
            tSlopeValue = tUpperBoundScaleFactor * tBoundsMisfit;
            tValue = tBoundsMisfit <= static_cast<ScalarType>(0) ? tUpperBoundScaleFactor : tSlopeValue;
            aCurrentSigma[tIndex] = std::min(tValue, aCurrentSigma[tIndex]);
        }
    }

    void updateObjectiveGlobalizationCoeff(const Plato::Vector<ScalarType, OrdinalType> & aDeltaControl,
                                           const Plato::Vector<ScalarType, OrdinalType> & aCurrentSigma,
                                           const Plato::Vector<ScalarType, OrdinalType> & aCurrentGradient,
                                           Plato::Vector<ScalarType, OrdinalType> & aOutputOne,
                                           Plato::Vector<ScalarType, OrdinalType> & aOutputTwo) const
    {
        assert(aDeltaControl.size() == aOutputOne.size());
        assert(aDeltaControl.size() == aOutputTwo.size());
        assert(aDeltaControl.size() == aCurrentSigma.size());
        assert(aDeltaControl.size() == aCurrentGradient.size());

        const OrdinalType tNumElements = aOutputOne.size();
        for(OrdinalType tIndex = 0; tIndex < tNumElements; tIndex++)
        {
            ScalarType tNumerator = aDeltaControl[tIndex] * aDeltaControl[tIndex];
            ScalarType tDenominator = (aCurrentSigma[tIndex] * aCurrentSigma[tIndex])
                    - (aDeltaControl[tIndex] * aDeltaControl[tIndex]);
            aOutputOne[tIndex] = tNumerator / tDenominator;

            tNumerator = ( (aCurrentSigma[tIndex] * aCurrentSigma[tIndex])
                    * aCurrentGradient[tIndex] * aDeltaControl[tIndex] )
                    + ( aCurrentSigma[tIndex] * std::abs(aCurrentGradient[tIndex])
                            * (aDeltaControl[tIndex] * aDeltaControl[tIndex]) );
            aOutputTwo[tIndex] = tNumerator / tDenominator;
        }
    }

    void updateConstraintGlobalizationCoeff(const Plato::Vector<ScalarType, OrdinalType> & aDeltaControl,
                                            const Plato::Vector<ScalarType, OrdinalType> & aCurrentSigma,
                                            const Plato::Vector<ScalarType, OrdinalType> & aCurrentGradient,
                                            Plato::Vector<ScalarType, OrdinalType> & aOutputOne,
                                            Plato::Vector<ScalarType, OrdinalType> & aOutputTwo) const
    {
        assert(aOutputOne.size() == aOutputTwo.size());
        assert(aOutputOne.size() == aDeltaControl.size());
        assert(aOutputOne.size() == aCurrentSigma.size());
        assert(aOutputOne.size() == aCurrentGradient.size());

        const OrdinalType tNumElements = aOutputOne.size();
        for(OrdinalType tIndex = 0; tIndex < tNumElements; tIndex++)
        {
            ScalarType tNumerator = aDeltaControl[tIndex] * aDeltaControl[tIndex];
            ScalarType tDenominator = (aCurrentSigma[tIndex] * aCurrentSigma[tIndex])
                    - (aDeltaControl[tIndex] * aDeltaControl[tIndex]);
            aOutputOne[tIndex] = tNumerator / tDenominator;

            tNumerator = ((aCurrentSigma[tIndex] * aCurrentSigma[tIndex])
                    * aCurrentGradient[tIndex] * aDeltaControl[tIndex])
                    + (aCurrentSigma[tIndex] * std::abs(aCurrentGradient[tIndex])
                            * (aDeltaControl[tIndex] * aDeltaControl[tIndex]));
            aOutputTwo[tIndex] = tNumerator / tDenominator;
        }
    }

    void computeKarushKuhnTuckerObjectiveConditions(const Plato::Vector<ScalarType, OrdinalType> & aControl,
                                                    const Plato::Vector<ScalarType, OrdinalType> & aObjectiveGradient,
                                                    const Plato::Vector<ScalarType, OrdinalType> & aConstraintGradientTimesDual,
                                                    Plato::Vector<ScalarType, OrdinalType> & aOutputOne,
                                                    Plato::Vector<ScalarType, OrdinalType> & aOutputTwo) const
    {
        assert(aControl.size() == aOutputOne.size());
        assert(aControl.size() == aOutputTwo.size());
        assert(aControl.size() == aObjectiveGradient.size());
        assert(aControl.size() == aConstraintGradientTimesDual.size());

        const OrdinalType tNumControls = aControl.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            aOutputOne[tIndex] = aObjectiveGradient[tIndex] + aConstraintGradientTimesDual[tIndex];
            aOutputOne[tIndex] = std::max(static_cast<ScalarType>(0), aOutputOne[tIndex]);
            aOutputOne[tIndex] = (static_cast<ScalarType>(1) + aControl[tIndex]) * aOutputOne[tIndex];
            aOutputOne[tIndex] = aOutputOne[tIndex] * aOutputOne[tIndex];

            aOutputTwo[tIndex] = aObjectiveGradient[tIndex] + aConstraintGradientTimesDual[tIndex];
            aOutputTwo[tIndex] = std::max(static_cast<ScalarType>(0), -aOutputTwo[tIndex]);
            aOutputTwo[tIndex] = (static_cast<ScalarType>(1) - aControl[tIndex]) * aOutputTwo[tIndex];
            aOutputTwo[tIndex] = aOutputTwo[tIndex] * aOutputTwo[tIndex];
        }
    }

private:
    HostPrimalProbElementWise(const Plato::HostPrimalProbElementWise<ScalarType, OrdinalType> & aRhs);
    Plato::HostPrimalProbElementWise<ScalarType, OrdinalType> & operator=(const Plato::HostPrimalProbElementWise<ScalarType, OrdinalType> & aRhs);
};
// class HostPrimalProbElementWise

} // namespace Plato

#endif /* PLATO_HOSTPRIMALPROBELEMENTWISE_HPP_ */
