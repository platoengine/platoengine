/*
 * Plato_HostDualProbElementWise.hpp
 *
 *  Created on: May 8, 2018
 */

#ifndef PLATO_HOSTDUALPROBELEMENTWISE_HPP_
#define PLATO_HOSTDUALPROBELEMENTWISE_HPP_

#include <cmath>
#include <cassert>
#include <algorithm>

#include "Plato_Vector.hpp"
#include "Plato_DualProbElementWiseFunctions.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class HostDualProbElementWise : public Plato::DualProbElementWiseFunctions<ScalarType, OrdinalType>
{
public:
    HostDualProbElementWise()
    {
    }
    virtual ~HostDualProbElementWise()
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

        const OrdinalType tNumElements = aOutput.size();
        for(OrdinalType tIndex = 0; tIndex < tNumElements; tIndex++)
        {
            ScalarType tSqrtTermA = std::sqrt(aTermA[tIndex]);
            ScalarType tSqrtTermB = std::sqrt(aTermB[tIndex]);
            ScalarType tNumerator = (aLowerAsymptotes[tIndex] * tSqrtTermA) + (aUpperAsymptotes[tIndex] * tSqrtTermB);
            ScalarType tDenominator = (tSqrtTermA + tSqrtTermB);

            aOutput[tIndex] = tNumerator / tDenominator;
            // Project trial control to feasible set
            aOutput[tIndex] = std::max(aOutput[tIndex], aLowerBounds[tIndex]);
            aOutput[tIndex] = std::min(aOutput[tIndex], aUpperBounds[tIndex]);
        }
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

        const OrdinalType tNumControls = aTrialControls.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            aOutputOne[tIndex] = aPcoeff[tIndex] / (aUpperAsymptotes[tIndex] - aTrialControls[tIndex]);
            aOutputTwo[tIndex] = aQcoeff[tIndex] / (aTrialControls[tIndex] - aLowerAsymptotes[tIndex]);
        }
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

        OrdinalType tNumElements = aCurrentSigma.size();
        for(OrdinalType tIndex = 0; tIndex < tNumElements; tIndex++)
        {
            ScalarType tCurrentSigmaTimesCurrentSigma = aCurrentSigma[tIndex] * aCurrentSigma[tIndex];
            ScalarType tValue = tCurrentSigmaTimesCurrentSigma
                    * std::max(static_cast<ScalarType>(0), aCurrentGradient[tIndex])
                    + ((aGlobalizationFactor * aCurrentSigma[tIndex]) / static_cast<ScalarType>(4));
            aPcoeff[tIndex] = tValue;

            tValue = tCurrentSigmaTimesCurrentSigma
                    * std::max(static_cast<ScalarType>(0), -aCurrentGradient[tIndex])
                    + ((aGlobalizationFactor * aCurrentSigma[tIndex]) / static_cast<ScalarType>(4));
            aQcoeff[tIndex] = tValue;

            aRcoeff[tIndex] = (aPcoeff[tIndex] + aQcoeff[tIndex]) / aCurrentSigma[tIndex];
        }
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

        const OrdinalType tNumControls = aCurrentSigma.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            ScalarType tCurrentSigmaTimesCurrentSigma = aCurrentSigma[tIndex] * aCurrentSigma[tIndex];
            ScalarType tValue = tCurrentSigmaTimesCurrentSigma
                    * std::max(static_cast<ScalarType>(0), aCurrentGradient[tIndex])
                    + ((aGlobalizationFactor * aCurrentSigma[tIndex]) / static_cast<ScalarType>(4));
            aPcoeff[tIndex] = tValue;

            tValue = tCurrentSigmaTimesCurrentSigma
                    * std::max(static_cast<ScalarType>(0), -aCurrentGradient[tIndex])
                    + ((aGlobalizationFactor * aCurrentSigma[tIndex]) / static_cast<ScalarType>(4));
            aQcoeff[tIndex] = tValue;

            aRcoeff[tIndex] = (aPcoeff[tIndex] + aQcoeff[tIndex]) / aCurrentSigma[tIndex];
        }
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

        const OrdinalType tNumControls = aTrialControl.size();
        for(OrdinalType tIndex = 0; tIndex < tNumControls; tIndex++)
        {
            ScalarType tNumerator = aObjectivePcoeff[tIndex] + aDualTimesConstraintPcoeff[tIndex];
            ScalarType tDenominator = aUpperAsymptotes[tIndex] - aTrialControl[tIndex];
            aOutputOne[tIndex] = tNumerator / tDenominator;

            tNumerator = aObjectiveQcoeff[tIndex] + aDualTimesConstraintQcoeff[tIndex];
            tDenominator = aTrialControl[tIndex] - aLowerAsymptotes[tIndex];
            aOutputTwo[tIndex] = tNumerator / tDenominator;
        }
    }

private:
    HostDualProbElementWise(const Plato::HostDualProbElementWise<ScalarType, OrdinalType> & aRhs);
    Plato::HostDualProbElementWise<ScalarType, OrdinalType> & operator=(const Plato::HostDualProbElementWise<ScalarType, OrdinalType> & aRhs);
};
// class HostDualProbElementWise

} // namespace Plato

#endif /* PLATO_HOSTDUALPROBELEMENTWISE_HPP_ */
