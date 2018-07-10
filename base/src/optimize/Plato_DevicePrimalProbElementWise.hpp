/*
 * Plato_DevicePrimalProbElementWise.hpp
 *
 *  Created on: May 8, 2018
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_DEVICEPRIMALPROBELEMENTWISE_HPP_
#define PLATO_DEVICEPRIMALPROBELEMENTWISE_HPP_

#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_KokkosTypes.hpp"
#include "Plato_PrimalProbElementWiseFunctions.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class DevicePrimalProbElementWise : public Plato::PrimalProbElementWiseFunctions<ScalarType, OrdinalType>
{
public:
    DevicePrimalProbElementWise()
    {
    }
    virtual ~DevicePrimalProbElementWise()
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

        ScalarType* tCurrentSigmaData = aCurrentSigma.data();
        const ScalarType* tUpperBoundsData = aUpperBounds.data();
        const ScalarType* tLowerBoundsData = aLowerBounds.data();
        const ScalarType* tPreviousSigmaData = aPreviousSigma.data();
        const ScalarType* tCurrentControlData = aCurrentControls.data();
        const ScalarType* tPreviousControlData = aPreviousControls.data();
        const ScalarType* tAntepenultimateControlsData = aAntepenultimateControls.data();

        const OrdinalType tNumElements = aCurrentSigma.size();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tNumElements), KOKKOS_LAMBDA(const OrdinalType & aIndex)
        {
            ScalarType tValue = (tCurrentControlData[aIndex] - tPreviousControlData[aIndex])
                    * (tPreviousControlData[aIndex] - tAntepenultimateControlsData[aIndex]);

            ScalarType tConditionOne = tExpansionFactor * tPreviousSigmaData[aIndex];
            ScalarType tConditionTwo = tContractionFactor * tPreviousSigmaData[aIndex];
            tCurrentSigmaData[aIndex] = tValue > static_cast<ScalarType>(0) ? tConditionOne : tConditionTwo;

            // check that lower bound is satisfied
            tValue = tLowerBoundScaleFactor * (tUpperBoundsData[aIndex] - tLowerBoundsData[aIndex]);
            tCurrentSigmaData[aIndex] = fmax(tValue, tCurrentSigmaData[aIndex]);

            // check that upper bound is satisfied
            tValue = tUpperBoundScaleFactor * (tUpperBoundsData[aIndex] - tLowerBoundsData[aIndex]);
            tCurrentSigmaData[aIndex] = fmin(tValue, tCurrentSigmaData[aIndex]);
        }, "DevicePrimalProbElementWise::updateSigmaCoeff");
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

        ScalarType* tOutputOneData = aOutputOne.data();
        ScalarType* tOutputTwoData = aOutputTwo.data();
        const ScalarType* tDeltaControlData = aDeltaControl.data();
        const ScalarType* tCurrentSigmaData = aCurrentSigma.data();
        const ScalarType* tCurrentGradientData = aCurrentGradient.data();

        const OrdinalType tNumElements = aOutputOne.size();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tNumElements), KOKKOS_LAMBDA(const OrdinalType & aIndex)
        {
            ScalarType tNumerator = tDeltaControlData[aIndex] * tDeltaControlData[aIndex];
            ScalarType tDenominator = (tCurrentSigmaData[aIndex] * tCurrentSigmaData[aIndex])
                    - (tDeltaControlData[aIndex] * tDeltaControlData[aIndex]);
            tOutputOneData[aIndex] = tNumerator / tDenominator;

            tNumerator = ( (tCurrentSigmaData[aIndex] * tCurrentSigmaData[aIndex])
                    * tCurrentGradientData[aIndex] * tDeltaControlData[aIndex] )
                    + ( tCurrentSigmaData[aIndex] * fabs(tCurrentGradientData[aIndex])
                            * (tDeltaControlData[aIndex] * tDeltaControlData[aIndex]) );
            tOutputTwoData[aIndex] = tNumerator / tDenominator;
        }, "DevicePrimalProbElementWise::updateObjectiveGlobalizationCoeff");
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

        ScalarType* tOutputOneData = aOutputOne.data();
        ScalarType* tOutputTwoData = aOutputTwo.data();
        const ScalarType* tDeltaControlData = aDeltaControl.data();
        const ScalarType* tCurrentSigmaData = aCurrentSigma.data();
        const ScalarType* tCurrentGradientData = aCurrentGradient.data();

        const OrdinalType tNumElements = aOutputOne.size();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tNumElements), KOKKOS_LAMBDA(const OrdinalType & aIndex)
        {
            ScalarType tNumerator = tDeltaControlData[aIndex] * tDeltaControlData[aIndex];
            ScalarType tDenominator = (tCurrentSigmaData[aIndex] * tCurrentSigmaData[aIndex])
                    - (tDeltaControlData[aIndex] * tDeltaControlData[aIndex]);
            tOutputOneData[aIndex] = tNumerator / tDenominator;

            tNumerator = ((tCurrentSigmaData[aIndex] * tCurrentSigmaData[aIndex])
                    * tCurrentGradientData[aIndex] * tDeltaControlData[aIndex])
                    + (tCurrentSigmaData[aIndex] * fabs(tCurrentGradientData[aIndex])
                            * (tDeltaControlData[aIndex] * tDeltaControlData[aIndex]));
            tOutputTwoData[aIndex] = tNumerator / tDenominator;
        }, "DevicePrimalProbElementWise::updateConstraintGlobalizationCoeff");
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

        ScalarType* tOutputOneData = aOutputOne.data();
        ScalarType* tOutputTwoData = aOutputTwo.data();
        const ScalarType* tControlData = aControl.data();
        const ScalarType* tObjectiveGradientData = aObjectiveGradient.data();
        const ScalarType* tConstraintGradientTimesDualData = aConstraintGradientTimesDual.data();

        const OrdinalType tNumControls = aControl.size();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tNumControls), KOKKOS_LAMBDA(const OrdinalType & aIndex)
        {
            tOutputOneData[aIndex] = tObjectiveGradientData[aIndex] + tConstraintGradientTimesDualData[aIndex];
            tOutputOneData[aIndex] = fmax(static_cast<ScalarType>(0), tOutputOneData[aIndex]);
            tOutputOneData[aIndex] = (static_cast<ScalarType>(1) + tControlData[aIndex]) * tOutputOneData[aIndex];
            tOutputOneData[aIndex] = tOutputOneData[aIndex] * tOutputOneData[aIndex];

            tOutputTwoData[aIndex] = tObjectiveGradientData[aIndex] + tConstraintGradientTimesDualData[aIndex];
            tOutputTwoData[aIndex] = fmax(static_cast<ScalarType>(0), -tOutputTwoData[aIndex]);
            tOutputTwoData[aIndex] = (static_cast<ScalarType>(1) - tControlData[aIndex]) * tOutputTwoData[aIndex];
            tOutputTwoData[aIndex] = tOutputTwoData[aIndex] * tOutputTwoData[aIndex];
        }, "DevicePrimalProbElementWise::computeKarushKuhnTuckerObjectiveConditions");
    }

private:
    DevicePrimalProbElementWise(const Plato::DevicePrimalProbElementWise<ScalarType, OrdinalType> & aRhs);
    Plato::DevicePrimalProbElementWise<ScalarType, OrdinalType> & operator=(const Plato::DevicePrimalProbElementWise<ScalarType, OrdinalType> & aRhs);
};
// class DevicePrimalProbElementWise

} // namespace Plato

#endif /* PLATO_DEVICEPRIMALPROBELEMENTWISE_HPP_ */
