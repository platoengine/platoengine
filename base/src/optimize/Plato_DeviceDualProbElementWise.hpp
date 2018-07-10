/*
 * Plato_DeviceDualProbElementWise.hpp
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

#ifndef PLATO_DEVICEDUALPROBELEMENTWISE_HPP_
#define PLATO_DEVICEDUALPROBELEMENTWISE_HPP_

#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_KokkosTypes.hpp"
#include "Plato_PrimalProbElementWiseFunctions.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class DeviceDualProbElementWise : public Plato::DualProbElementWiseFunctions<ScalarType, OrdinalType>
{
public:
    DeviceDualProbElementWise()
    {
    }
    virtual ~DeviceDualProbElementWise()
    {
    }

    void updateTrialControl(const Plato::Vector<ScalarType, OrdinalType> & aTermA,
                            const Plato::Vector<ScalarType, OrdinalType> & aTermB,
                            const Plato::Vector<ScalarType, OrdinalType> & aLowerBounds,
                            const Plato::Vector<ScalarType, OrdinalType> & aUpperBounds,
                            const Plato::Vector<ScalarType, OrdinalType> & aLowerAsymptotes,
                            const Plato::Vector<ScalarType, OrdinalType> & aUpperAsymptotes,
                            Plato::Vector<ScalarType, OrdinalType> & aOutput) const
    {
        assert(aOutput.size() == aTermA.size());
        assert(aOutput.size() == aTermB.size());
        assert(aOutput.size() == aLowerBounds.size());
        assert(aOutput.size() == aUpperBounds.size());
        assert(aOutput.size() == aLowerAsymptotes.size());
        assert(aOutput.size() == aUpperAsymptotes.size());

        ScalarType* tOutputData = aOutput.data();
        const ScalarType* tTermA_Data = aTermA.data();
        const ScalarType* tTermB_Data = aTermB.data();
        const ScalarType* tUpperBoundsData = aUpperBounds.data();
        const ScalarType* tLowerBoundsData = aLowerBounds.data();
        const ScalarType* tLowerAsymptotesData = aLowerAsymptotes.data();
        const ScalarType* tUpperAsymptotesData = aUpperAsymptotes.data();

        const OrdinalType tNumElements = aOutput.size();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tNumElements), KOKKOS_LAMBDA(const OrdinalType & aIndex)
        {
            ScalarType tSqrtTermA = sqrt(tTermA_Data[aIndex]);
            ScalarType tSqrtTermB = sqrt(tTermB_Data[aIndex]);
            ScalarType tNumerator = (tLowerAsymptotesData[aIndex] * tSqrtTermA)
                    + (tUpperAsymptotesData[aIndex] * tSqrtTermB);
            ScalarType tDenominator = (tSqrtTermA + tSqrtTermB);

            tOutputData[aIndex] = tNumerator / tDenominator;
            // Project trial control to feasible set
            tOutputData[aIndex] = fmax(tOutputData[aIndex], tLowerBoundsData[aIndex]);
            tOutputData[aIndex] = fmin(tOutputData[aIndex], tUpperBoundsData[aIndex]);
        }, "DeviceDualProbElementWise::updateTrialControl");
    }

    void updateGradientCoeff(const Plato::Vector<ScalarType, OrdinalType> & aTrialControls,
                             const Plato::Vector<ScalarType, OrdinalType> & aLowerAsymptotes,
                             const Plato::Vector<ScalarType, OrdinalType> & aUpperAsymptotes,
                             const Plato::Vector<ScalarType, OrdinalType> & aPcoeff,
                             const Plato::Vector<ScalarType, OrdinalType> & aQcoeff,
                             Plato::Vector<ScalarType, OrdinalType> & aOutputOne,
                             Plato::Vector<ScalarType, OrdinalType> & aOutputTwo) const
    {
        assert(aTrialControls.size() == aPcoeff.size());
        assert(aTrialControls.size() == aQcoeff.size());
        assert(aTrialControls.size() == aOutputOne.size());
        assert(aTrialControls.size() == aOutputOne.size());
        assert(aTrialControls.size() == aLowerAsymptotes.size());
        assert(aTrialControls.size() == aUpperAsymptotes.size());

        ScalarType* tOutputOneData = aOutputOne.data();
        ScalarType* tOutputTwoData = aOutputTwo.data();
        const ScalarType* tPcoeffData = aPcoeff.data();
        const ScalarType* tQcoeffData = aQcoeff.data();
        const ScalarType* tTrialControlsData = aTrialControls.data();
        const ScalarType* tLowerAsymptotesData = aLowerAsymptotes.data();
        const ScalarType* tUpperAsymptotesData = aUpperAsymptotes.data();

        const OrdinalType tNumControls = aTrialControls.size();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tNumControls), KOKKOS_LAMBDA(const OrdinalType & aIndex)
        {
            tOutputOneData[aIndex] = tPcoeffData[aIndex] / (tUpperAsymptotesData[aIndex] - tTrialControlsData[aIndex]);
            tOutputTwoData[aIndex] = tQcoeffData[aIndex] / (tTrialControlsData[aIndex] - tLowerAsymptotesData[aIndex]);
        }, "DeviceDualProbElementWise::updateGradientCoeff");
    }

    void updateObjectiveCoeff(const ScalarType & aGlobalizationFactor,
                              const Plato::Vector<ScalarType, OrdinalType> & aCurrentSigma,
                              const Plato::Vector<ScalarType, OrdinalType> & aCurrentGradient,
                              Plato::Vector<ScalarType, OrdinalType> & aPcoeff,
                              Plato::Vector<ScalarType, OrdinalType> & aQcoeff,
                              Plato::Vector<ScalarType, OrdinalType> & aRcoeff) const
    {
        assert(aCurrentSigma.size() == aPcoeff.size());
        assert(aCurrentSigma.size() == aQcoeff.size());
        assert(aCurrentSigma.size() == aRcoeff.size());
        assert(aCurrentSigma.size() == aCurrentGradient.size());

        ScalarType* tPcoeffData = aPcoeff.data();
        ScalarType* tQcoeffData = aQcoeff.data();
        ScalarType* tRcoeffData = aRcoeff.data();
        const ScalarType* tCurrentSigmaData = aCurrentSigma.data();
        const ScalarType* tCurrentGradientData = aCurrentGradient.data();

        OrdinalType tNumElements = aCurrentSigma.size();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tNumElements), KOKKOS_LAMBDA(const OrdinalType & aIndex)
        {
            ScalarType tCurrentSigmaTimesCurrentSigma = tCurrentSigmaData[aIndex] * tCurrentSigmaData[aIndex];
            ScalarType tValue = tCurrentSigmaTimesCurrentSigma
                    * fmax(static_cast<ScalarType>(0), tCurrentGradientData[aIndex])
                    + ((aGlobalizationFactor * tCurrentSigmaData[aIndex]) / static_cast<ScalarType>(4));
            tPcoeffData[aIndex] = tValue;

            tValue = tCurrentSigmaTimesCurrentSigma
                    * fmax(static_cast<ScalarType>(0), -tCurrentGradientData[aIndex])
                    + ((aGlobalizationFactor * tCurrentSigmaData[aIndex]) / static_cast<ScalarType>(4));
            tQcoeffData[aIndex] = tValue;

            tRcoeffData[aIndex] = (tPcoeffData[aIndex] + tQcoeffData[aIndex]) / tCurrentSigmaData[aIndex];
        }, "DeviceDualProbElementWise::updateObjectiveCoeff");
    }

    void updateConstraintCoeff(const ScalarType & aGlobalizationFactor,
                               const Plato::Vector<ScalarType, OrdinalType> & aCurrentSigma,
                               const Plato::Vector<ScalarType, OrdinalType> & aCurrentGradient,
                               Plato::Vector<ScalarType, OrdinalType> & aPcoeff,
                               Plato::Vector<ScalarType, OrdinalType> & aQcoeff,
                               Plato::Vector<ScalarType, OrdinalType> & aRcoeff) const
    {
        assert(aCurrentSigma.size() == aPcoeff.size());
        assert(aCurrentSigma.size() == aQcoeff.size());
        assert(aCurrentSigma.size() == aRcoeff.size());
        assert(aCurrentSigma.size() == aCurrentGradient.size());

        ScalarType* tPcoeffData = aPcoeff.data();
        ScalarType* tQcoeffData = aQcoeff.data();
        ScalarType* tRcoeffData = aRcoeff.data();
        const ScalarType* tCurrentSigmaData = aCurrentSigma.data();
        const ScalarType* tCurrentGradientData = aCurrentGradient.data();

        const OrdinalType tNumControls = aCurrentSigma.size();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tNumControls), KOKKOS_LAMBDA(const OrdinalType & aIndex)
        {
            ScalarType tCurrentSigmaTimesCurrentSigma = tCurrentSigmaData[aIndex] * tCurrentSigmaData[aIndex];
            ScalarType tValue = tCurrentSigmaTimesCurrentSigma
                    * fmax(static_cast<ScalarType>(0), tCurrentGradientData[aIndex])
                    + ((aGlobalizationFactor * tCurrentSigmaData[aIndex]) / static_cast<ScalarType>(4));
            tPcoeffData[aIndex] = tValue;

            tValue = tCurrentSigmaTimesCurrentSigma
                    * fmax(static_cast<ScalarType>(0), -tCurrentGradientData[aIndex])
                    + ((aGlobalizationFactor * tCurrentSigmaData[aIndex]) / static_cast<ScalarType>(4));
            tQcoeffData[aIndex] = tValue;

            tRcoeffData[aIndex] = (tPcoeffData[aIndex] + tQcoeffData[aIndex]) / tCurrentSigmaData[aIndex];
        }, "DeviceDualProbElementWise::updateConstraintCoeff");
    }

    void updateMovingAsymptotesCoeff(const Plato::Vector<ScalarType, OrdinalType> & aTrialControl,
                                     const Plato::Vector<ScalarType, OrdinalType> & aLowerAsymptotes,
                                     const Plato::Vector<ScalarType, OrdinalType> & aUpperAsymptotes,
                                     const Plato::Vector<ScalarType, OrdinalType> & aObjectivePcoeff,
                                     const Plato::Vector<ScalarType, OrdinalType> & aObjectiveQcoeff,
                                     const Plato::Vector<ScalarType, OrdinalType> & aDualTimesConstraintPcoeff,
                                     const Plato::Vector<ScalarType, OrdinalType> & aDualTimesConstraintQcoeff,
                                     Plato::Vector<ScalarType, OrdinalType> & aOutputOne,
                                     Plato::Vector<ScalarType, OrdinalType> & aOutputTwo) const
    {
        assert(aTrialControl.size() == aOutputOne.size());
        assert(aTrialControl.size() == aOutputTwo.size());
        assert(aTrialControl.size() == aLowerAsymptotes.size());
        assert(aTrialControl.size() == aUpperAsymptotes.size());
        assert(aTrialControl.size() == aObjectivePcoeff.size());
        assert(aTrialControl.size() == aObjectiveQcoeff.size());
        assert(aTrialControl.size() == aDualTimesConstraintPcoeff.size());
        assert(aTrialControl.size() == aDualTimesConstraintQcoeff.size());

        ScalarType* tOutputOneData = aOutputOne.data();
        ScalarType* tOutputTwoData = aOutputTwo.data();
        const ScalarType* tTrialControlData = aTrialControl.data();
        const ScalarType* tLowerAsymptotesData = aLowerAsymptotes.data();
        const ScalarType* tUpperAsymptotesData = aUpperAsymptotes.data();
        const ScalarType* tObjectivePcoeffData = aObjectivePcoeff.data();
        const ScalarType* tObjectiveQcoeffData = aObjectiveQcoeff.data();
        const ScalarType* tDualTimesConstraintPcoeffData = aDualTimesConstraintPcoeff.data();
        const ScalarType* tDualTimesConstraintQcoeffData = aDualTimesConstraintQcoeff.data();

        const OrdinalType tNumControls = aTrialControl.size();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tNumControls), KOKKOS_LAMBDA(const OrdinalType & aIndex)
        {
            ScalarType tNumerator = tObjectivePcoeffData[aIndex] + tDualTimesConstraintPcoeffData[aIndex];
            ScalarType tDenominator = tUpperAsymptotesData[aIndex] - tTrialControlData[aIndex];
            tOutputOneData[aIndex] = tNumerator / tDenominator;

            tNumerator = tObjectiveQcoeffData[aIndex] + tDualTimesConstraintQcoeffData[aIndex];
            tDenominator = tTrialControlData[aIndex] - tLowerAsymptotesData[aIndex];
            tOutputTwoData[aIndex] = tNumerator / tDenominator;
        }, "DeviceDualProbElementWise::updateMovingAsymptotesCoeff");
    }

private:
    DeviceDualProbElementWise(const Plato::DeviceDualProbElementWise<ScalarType, OrdinalType> & aRhs);
    Plato::DeviceDualProbElementWise<ScalarType, OrdinalType> & operator=(const Plato::DeviceDualProbElementWise<ScalarType, OrdinalType> & aRhs);
};
// class DeviceDualProbElementWise

} // namespace Plato

#endif /* PLATO_DEVICEDUALPROBELEMENTWISE_HPP_ */
