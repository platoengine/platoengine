/*
 * Plato_DualProblemStageMng.hpp
 *
 *  Created on: Nov 4, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_DUALPROBLEMSTAGEMNG_HPP_
#define PLATO_DUALPROBLEMSTAGEMNG_HPP_

#include <cmath>
#include <vector>
#include <limits>
#include <memory>
#include <numeric>
#include <cassert>
#include <algorithm>

#include "Plato_Vector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_ReductionOperations.hpp"
#include "Plato_HostDualProbElementWise.hpp"
#include "Plato_DeviceDualProbElementWise.hpp"
#include "Plato_ConservativeConvexSeparableAppxDataMng.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class DualProblemStageMng
{
public:
    explicit DualProblemStageMng(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory) :
            mEpsilon(1e-6),
            mObjectiveCoefficientA(1),
            mObjectiveCoefficientR(1),
            mTrialAuxiliaryVariableZ(0),
            mCurrentObjectiveFunctionValue(std::numeric_limits<ScalarType>::max()),
            mDualWorkVector(),
            mControlWorkVectorOne(),
            mControlWorkVectorTwo(),
            mTermA(aDataFactory.control().create()),
            mTermB(aDataFactory.control().create()),
            mTrialControl(aDataFactory.control().create()),
            mLowerAsymptotes(aDataFactory.control().create()),
            mUpperAsymptotes(aDataFactory.control().create()),
            mDualTimesCoefficientsP(aDataFactory.control().create()),
            mDualTimesCoefficientsQ(aDataFactory.control().create()),
            mObjectiveCoefficientsP(aDataFactory.control().create()),
            mObjectiveCoefficientsQ(aDataFactory.control().create()),
            mTrialControlLowerBounds(aDataFactory.control().create()),
            mTrialControlUpperBounds(aDataFactory.control().create()),
            mConstraintCoefficientsA(aDataFactory.dual().create()),
            mConstraintCoefficientsC(aDataFactory.dual().create()),
            mConstraintCoefficientsD(aDataFactory.dual().create()),
            mConstraintCoefficientsR(aDataFactory.dual().create()),
            mTrialAuxiliaryVariableY(aDataFactory.dual().create()),
            mCurrentConstraintValues(aDataFactory.dual().create()),
            mDualReductionOperations(aDataFactory.getDualReductionOperations().create()),
            mControlReductionOperations(aDataFactory.getControlReductionOperations().create()),
            mElementWiseFunctions(nullptr),
            mConstraintCoefficientsP(),
            mConstraintCoefficientsQ()
    {
        this->initialize(aDataFactory);
    }
    virtual ~DualProblemStageMng()
    {
    }

    // NOTE: DUAL PROBLEM CONSTRAINT COEFFICIENTS A
    const Plato::MultiVector<ScalarType, OrdinalType> & getConstraintCoefficientsA() const
    {
        assert(mConstraintCoefficientsA.get() != nullptr);

        return (mConstraintCoefficientsA.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getConstraintCoefficientsA(const OrdinalType & aVectorIndex) const
    {
        assert(mConstraintCoefficientsA.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mConstraintCoefficientsA->getNumVectors());

        return (mConstraintCoefficientsA->operator [](aVectorIndex));
    }
    void setConstraintCoefficientsA(const ScalarType & aValue)
    {
        assert(mConstraintCoefficientsA.get() != nullptr);
        assert(mConstraintCoefficientsA->getNumVectors() > static_cast<OrdinalType>(0));

        OrdinalType tNumVectors = mConstraintCoefficientsA->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            mConstraintCoefficientsA->operator [](tVectorIndex).fill(aValue);
        }
    }
    void setConstraintCoefficientsA(const OrdinalType & aVectorIndex, const ScalarType & aValue)
    {
        assert(mConstraintCoefficientsA.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mConstraintCoefficientsA->getNumVectors());

        mConstraintCoefficientsA->operator [](aVectorIndex).fill(aValue);
    }
    void setConstraintCoefficientsA(const OrdinalType & aVectorIndex,
                                    const OrdinalType & aElementIndex,
                                    const ScalarType & aValue)
    {
        assert(mConstraintCoefficientsA.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mConstraintCoefficientsA->getNumVectors());
        assert(aElementIndex >= static_cast<OrdinalType>(0));
        assert(aElementIndex < mConstraintCoefficientsA->operator [](aVectorIndex).size());

        mConstraintCoefficientsA->operator*().operator()(aVectorIndex, aElementIndex) = aValue;
    }
    void setConstraintCoefficientsA(const OrdinalType & aVectorIndex,
                                    const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mConstraintCoefficientsA.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mConstraintCoefficientsA->getNumVectors());

        mConstraintCoefficientsA->operator [](aVectorIndex).update(1., aInput, 0.);
    }
    void setConstraintCoefficientsA(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mConstraintCoefficientsA->getNumVectors());
        Plato::update(1., aInput, 0., *mConstraintCoefficientsA);
    }

    // NOTE: DUAL PROBLEM CONSTRAINT COEFFICIENTS C
    const Plato::MultiVector<ScalarType, OrdinalType> & getConstraintCoefficientsC() const
    {
        assert(mConstraintCoefficientsC.get() != nullptr);

        return (mConstraintCoefficientsC.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getConstraintCoefficientsC(const OrdinalType & aVectorIndex) const
    {
        assert(mConstraintCoefficientsC.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mConstraintCoefficientsC->getNumVectors());

        return (mConstraintCoefficientsC->operator [](aVectorIndex));
    }
    void setConstraintCoefficientsC(const ScalarType & aValue)
    {
        assert(mConstraintCoefficientsC.get() != nullptr);
        assert(mConstraintCoefficientsC->getNumVectors() > static_cast<OrdinalType>(0));

        OrdinalType tNumVectors = mConstraintCoefficientsC->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            mConstraintCoefficientsC->operator [](tVectorIndex).fill(aValue);
        }
    }
    void setConstraintCoefficientsC(const OrdinalType & aVectorIndex, const ScalarType & aValue)
    {
        assert(mConstraintCoefficientsC.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mConstraintCoefficientsC->getNumVectors());

        mConstraintCoefficientsC->operator [](aVectorIndex).fill(aValue);
    }
    void setConstraintCoefficientsC(const OrdinalType & aVectorIndex,
                                    const OrdinalType & aElementIndex,
                                    const ScalarType & aValue)
    {
        assert(mConstraintCoefficientsC.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mConstraintCoefficientsC->getNumVectors());
        assert(aElementIndex >= static_cast<OrdinalType>(0));
        assert(aElementIndex < mConstraintCoefficientsC->operator [](aVectorIndex).size());

        mConstraintCoefficientsC->operator*().operator()(aVectorIndex, aElementIndex) = aValue;
    }
    void setConstraintCoefficientsC(const OrdinalType & aVectorIndex,
                                    const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mConstraintCoefficientsC.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mConstraintCoefficientsC->getNumVectors());

        mConstraintCoefficientsC->operator [](aVectorIndex).update(1., aInput, 0.);
    }
    void setConstraintCoefficientsC(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mConstraintCoefficientsC->getNumVectors());
        Plato::update(1., aInput, 0., *mConstraintCoefficientsC);
    }

    // NOTE: DUAL PROBLEM CONSTRAINT COEFFICIENTS D
    const Plato::MultiVector<ScalarType, OrdinalType> & getConstraintCoefficientsD() const
    {
        assert(mConstraintCoefficientsD.get() != nullptr);

        return (mConstraintCoefficientsD.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getConstraintCoefficientsD(const OrdinalType & aVectorIndex) const
    {
        assert(mConstraintCoefficientsD.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mConstraintCoefficientsD->getNumVectors());

        return (mConstraintCoefficientsD->operator [](aVectorIndex));
    }
    void setConstraintCoefficientsD(const ScalarType & aValue)
    {
        assert(mConstraintCoefficientsD.get() != nullptr);
        assert(mConstraintCoefficientsD->getNumVectors() > static_cast<OrdinalType>(0));

        OrdinalType tNumVectors = mConstraintCoefficientsD->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            mConstraintCoefficientsD->operator [](tVectorIndex).fill(aValue);
        }
    }
    void setConstraintCoefficientsD(const OrdinalType & aVectorIndex, const ScalarType & aValue)
    {
        assert(mConstraintCoefficientsD.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mConstraintCoefficientsD->getNumVectors());

        mConstraintCoefficientsD->operator [](aVectorIndex).fill(aValue);
    }
    void setConstraintCoefficientsD(const OrdinalType & aVectorIndex,
                                    const OrdinalType & aElementIndex,
                                    const ScalarType & aValue)
    {
        assert(mConstraintCoefficientsD.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mConstraintCoefficientsD->getNumVectors());
        assert(aElementIndex >= static_cast<OrdinalType>(0));
        assert(aElementIndex < mConstraintCoefficientsD->operator [](aVectorIndex).size());

        mConstraintCoefficientsD->operator*().operator()(aVectorIndex, aElementIndex) = aValue;
    }
    void setConstraintCoefficientsD(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mConstraintCoefficientsD.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mConstraintCoefficientsD->getNumVectors());

        mConstraintCoefficientsD->operator [](aVectorIndex).update(1., aInput, 0.);
    }
    void setConstraintCoefficientsD(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mConstraintCoefficientsD->getNumVectors());
        Plato::update(1., aInput, 0., *mConstraintCoefficientsD);
    }

    ScalarType getObjectiveCoefficientsR() const
    {
        return (mObjectiveCoefficientR);
    }
    const Plato::MultiVector<ScalarType, OrdinalType> & getLowerAsymptotes() const
    {
        assert(mLowerAsymptotes.get() != nullptr);
        return (mLowerAsymptotes.operator*());
    }
    const Plato::MultiVector<ScalarType, OrdinalType> & getUpperAsymptotes() const
    {
        assert(mUpperAsymptotes.get() != nullptr);
        return (mUpperAsymptotes.operator*());
    }
    const Plato::MultiVector<ScalarType, OrdinalType> & getTrialControlLowerBounds() const
    {
        assert(mTrialControlLowerBounds.get() != nullptr);
        return (mTrialControlLowerBounds.operator*());
    }
    const Plato::MultiVector<ScalarType, OrdinalType> & getTrialControlUpperBounds() const
    {
        assert(mTrialControlUpperBounds.get() != nullptr);
        return (mTrialControlUpperBounds.operator*());
    }
    const Plato::MultiVector<ScalarType, OrdinalType> & getObjectiveCoefficientsP() const
    {
        assert(mObjectiveCoefficientsP.get() != nullptr);
        return (mObjectiveCoefficientsP.operator*());
    }
    const Plato::MultiVector<ScalarType, OrdinalType> & getObjectiveCoefficientsQ() const
    {
        assert(mObjectiveCoefficientsQ.get() != nullptr);
        return (mObjectiveCoefficientsQ.operator*());
    }
    const Plato::MultiVector<ScalarType, OrdinalType> & getConstraintCoefficientsP(const OrdinalType & aConstraintIndex) const
    {
        assert(mConstraintCoefficientsP.empty() == false);
        assert(mConstraintCoefficientsP[aConstraintIndex].get() != nullptr);
        return (mConstraintCoefficientsP[aConstraintIndex].operator*());
    }
    const Plato::MultiVector<ScalarType, OrdinalType> & getConstraintCoefficientsQ(const OrdinalType & aConstraintIndex) const
    {
        assert(mConstraintCoefficientsQ.empty() == false);
        assert(mConstraintCoefficientsQ[aConstraintIndex].get() != nullptr);
        return (mConstraintCoefficientsQ[aConstraintIndex].operator*());
    }
    const Plato::MultiVector<ScalarType, OrdinalType> & getConstraintCoefficientsR() const
    {
        assert(mConstraintCoefficientsR.get() != nullptr);
        return (mConstraintCoefficientsR.operator*());
    }

    // UPDATE DUAL PROBLEM DATA
    void update(const Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        // Update Current Objective Function Value
        mCurrentObjectiveFunctionValue = aDataMng.getCurrentObjectiveFunctionValue();

        // Update Current Constraint Values
        Plato::update(static_cast<ScalarType>(1),
                      aDataMng.getCurrentConstraintValues(),
                      static_cast<ScalarType>(0),
                      mCurrentConstraintValues.operator*());

        // Update Moving Asymptotes
        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentSigma = aDataMng.getCurrentSigma();
        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = aDataMng.getCurrentControl();
        Plato::update(static_cast<ScalarType>(1), tCurrentControl, static_cast<ScalarType>(0), *mLowerAsymptotes);
        Plato::update(static_cast<ScalarType>(-1), tCurrentSigma, static_cast<ScalarType>(1), *mLowerAsymptotes);
        Plato::update(static_cast<ScalarType>(1), tCurrentControl, static_cast<ScalarType>(0), *mUpperAsymptotes);
        Plato::update(static_cast<ScalarType>(1), tCurrentSigma, static_cast<ScalarType>(1), *mUpperAsymptotes);

        // Update Trial Control Bounds
        const ScalarType tScaleFactor = aDataMng.getDualProblemBoundsScaleFactor();
        Plato::update(static_cast<ScalarType>(1), tCurrentControl, static_cast<ScalarType>(0), *mTrialControlLowerBounds);
        Plato::update(-tScaleFactor, tCurrentSigma, static_cast<ScalarType>(1), *mTrialControlLowerBounds);
        Plato::update(static_cast<ScalarType>(1), tCurrentControl, static_cast<ScalarType>(0), *mTrialControlUpperBounds);
        Plato::update(tScaleFactor, tCurrentSigma, static_cast<ScalarType>(1), *mTrialControlUpperBounds);
    }

    // EVALUATE DUAL OBJECTIVE FUNCTION
    ScalarType evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aDual,
                                 ScalarType aTolerance = std::numeric_limits<ScalarType>::max())
    {
        assert(aDual.getNumVectors() == static_cast<OrdinalType>(1));

        this->computeDualTimesConstraintCoefficientTerms(aDual);
        this->computeTrialControl(aDual);
        this->computeTrialAuxiliaryVariables(aDual);

        const ScalarType tObjectiveTerm = mObjectiveCoefficientR + (mTrialAuxiliaryVariableZ * mObjectiveCoefficientA)
                + (mEpsilon * mTrialAuxiliaryVariableZ * mTrialAuxiliaryVariableZ);

        const ScalarType tConstraintSummationTerm = this->computeConstraintContribution(aDual);

        ScalarType tMovingAsymptotesTerm = this->computeMovingAsymptotesContribution();

        // Add all contributions to dual objective function
        ScalarType tOutput = static_cast<ScalarType>(-1)
                * (tObjectiveTerm + tConstraintSummationTerm + tMovingAsymptotesTerm);
        return (tOutput);
    }

    // COMPUTE DUAL GRADIENT
    void computeGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aDual,
                         Plato::MultiVector<ScalarType, OrdinalType> & aDualGradient)
    {
        assert(aDual.getNumVectors() == static_cast<OrdinalType>(1));

        const OrdinalType tDualVectorIndex = 0;
        Plato::Vector<ScalarType, OrdinalType> & tDualGradient = aDualGradient[tDualVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tDual = aDual[tDualVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tAuxiliaryVariableY = (*mTrialAuxiliaryVariableY)[tDualVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tCoefficientsR = (*mConstraintCoefficientsR)[tDualVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tCoefficientsA = (*mConstraintCoefficientsA)[tDualVectorIndex];

        OrdinalType tNumConstraints = tDual.size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            tDualGradient[tConstraintIndex] = tCoefficientsR[tConstraintIndex] - tAuxiliaryVariableY[tConstraintIndex]
                    - (tCoefficientsA[tConstraintIndex] * mTrialAuxiliaryVariableZ);

            const Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintCoefficientsP =
                    mConstraintCoefficientsP[tConstraintIndex].operator*();
            const Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintCoefficientsQ =
                    mConstraintCoefficientsQ[tConstraintIndex].operator*();

            const OrdinalType tNumControlVectors = mTrialControl->getNumVectors();
            std::vector<ScalarType> tMyStorageOne(tNumControlVectors);
            std::vector<ScalarType> tMyStorageTwo(tNumControlVectors);
            for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
            {
                mControlWorkVectorOne->fill(static_cast<ScalarType>(0));
                mControlWorkVectorTwo->fill(static_cast<ScalarType>(0));
                const Plato::Vector<ScalarType, OrdinalType> & tMyTrialControl = (*mTrialControl)[tVectorIndex];
                const Plato::Vector<ScalarType, OrdinalType> & tMyLowerAsymptotes = (*mLowerAsymptotes)[tVectorIndex];
                const Plato::Vector<ScalarType, OrdinalType> & tMyUpperAsymptotes = (*mUpperAsymptotes)[tVectorIndex];
                const Plato::Vector<ScalarType, OrdinalType> & tMyCoefficientsP = tMyConstraintCoefficientsP[tVectorIndex];
                const Plato::Vector<ScalarType, OrdinalType> & tMyCoefficientsQ = tMyConstraintCoefficientsQ[tVectorIndex];

                mElementWiseFunctions->updateGradientCoeff(tMyTrialControl,
                                                           tMyLowerAsymptotes,
                                                           tMyUpperAsymptotes,
                                                           tMyCoefficientsP,
                                                           tMyCoefficientsQ,
                                                           *mControlWorkVectorOne,
                                                           *mControlWorkVectorTwo);

                tMyStorageOne[tVectorIndex] = mControlReductionOperations->sum(mControlWorkVectorOne.operator*());
                tMyStorageTwo[tVectorIndex] = mControlReductionOperations->sum(mControlWorkVectorTwo.operator*());
            }

            const ScalarType tInitialValue = 0;
            const ScalarType tGlobalSumP = std::accumulate(tMyStorageOne.begin(), tMyStorageOne.end(), tInitialValue);
            const ScalarType tGlobalSumQ = std::accumulate(tMyStorageTwo.begin(), tMyStorageTwo.end(), tInitialValue);
            // Add contribution to dual gradient
            tDualGradient[tConstraintIndex] = static_cast<ScalarType>(-1)
                    * (tDualGradient[tConstraintIndex] + tGlobalSumP + tGlobalSumQ);
        }
    }
    // APPLY VECTOR TO DUAL HESSIAN
    void applyVectorToHessian(const Plato::MultiVector<ScalarType, OrdinalType> & aDual,
                              const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                              Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        return;
    }

    // GET OPTIMAL TRIAL CONTROL FROM DUAL PROBLEM
    void getTrialControl(Plato::MultiVector<ScalarType, OrdinalType> & aInput) const
    {
        Plato::update(static_cast<ScalarType>(1), *mTrialControl, static_cast<ScalarType>(0), aInput);
    }
    void updateObjectiveCoefficients(const Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mObjectiveCoefficientR = mCurrentObjectiveFunctionValue;
        const ScalarType tGlobalizationFactor = aDataMng.getDualObjectiveGlobalizationFactor();
        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentSigma = aDataMng.getCurrentSigma();

        const OrdinalType tNumVectors = tCurrentSigma.getNumVectors();
        std::vector<ScalarType> tStorage(tNumVectors);

        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            mControlWorkVectorOne->fill(static_cast<ScalarType>(0));
            Plato::Vector<ScalarType, OrdinalType> & tMyObjectiveCoefficientsP = mObjectiveCoefficientsP->operator[](tVectorIndex);
            Plato::Vector<ScalarType, OrdinalType> & tMyObjectiveCoefficientsQ = mObjectiveCoefficientsQ->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyCurrentSigma = tCurrentSigma[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tMyCurrentObjectiveGradient = aDataMng.getCurrentObjectiveGradient(tVectorIndex);

            mElementWiseFunctions->updateObjectiveCoeff(tGlobalizationFactor,
                                                        tMyCurrentSigma,
                                                        tMyCurrentObjectiveGradient,
                                                        tMyObjectiveCoefficientsP,
                                                        tMyObjectiveCoefficientsQ,
                                                        *mControlWorkVectorOne);

            tStorage[tVectorIndex] = mControlReductionOperations->sum(*mControlWorkVectorOne);
        }

        const ScalarType tInitialValue = 0;
        const ScalarType tValue = std::accumulate(tStorage.begin(), tStorage.end(), tInitialValue);
        mObjectiveCoefficientR = mObjectiveCoefficientR - tValue;
    }
    void updateConstraintCoefficients(const Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        assert(aDataMng.getNumDualVectors() == static_cast<OrdinalType>(1));

        const OrdinalType tDualVectorIndex = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentConstraintValues =
                mCurrentConstraintValues->operator[](tDualVectorIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalizationFactor =
                aDataMng.getConstraintGlobalizationFactors(tDualVectorIndex);
        Plato::Vector<ScalarType, OrdinalType> & tConstraintCoefficientsR =
                mConstraintCoefficientsR.operator*()[tDualVectorIndex];

        const OrdinalType tNumConstraints = tConstraintCoefficientsR.size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            tConstraintCoefficientsR[tConstraintIndex] = tCurrentConstraintValues[tConstraintIndex];
            const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentConstraintGradients =
                    aDataMng.getCurrentConstraintGradients(tConstraintIndex);
            Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintCoeffP =
                    mConstraintCoefficientsP[tConstraintIndex].operator*();
            Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintCoeffQ =
                    mConstraintCoefficientsQ[tConstraintIndex].operator*();
            assert(tCurrentConstraintGradients.getNumVectors() == aDataMng.getNumControlVectors());

            const ScalarType tMyGlobalizationFactor = tGlobalizationFactor[tConstraintIndex];  
            const OrdinalType tNumControlVectors = tCurrentConstraintGradients.getNumVectors();
            std::vector<ScalarType> tStorage(tNumControlVectors);
            for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
            {
                mControlWorkVectorOne->fill(static_cast<ScalarType>(0));
                Plato::Vector<ScalarType, OrdinalType> & tMyCoeffP = tMyConstraintCoeffP[tVectorIndex];
                Plato::Vector<ScalarType, OrdinalType> & tMyCoeffQ = tMyConstraintCoeffQ[tVectorIndex];
                const Plato::Vector<ScalarType, OrdinalType> & tMyCurrentSigma = aDataMng.getCurrentSigma(tVectorIndex);
                const Plato::Vector<ScalarType, OrdinalType> & tMyCurrentGradient = tCurrentConstraintGradients[tVectorIndex];

                mElementWiseFunctions->updateConstraintCoeff(tMyGlobalizationFactor,
                                                             tMyCurrentSigma,
                                                             tMyCurrentGradient,
                                                             tMyCoeffP,
                                                             tMyCoeffQ,
                                                             *mControlWorkVectorOne);

                tStorage[tVectorIndex] = mControlReductionOperations->sum(mControlWorkVectorOne.operator*());
            }

            const ScalarType tInitialValue = 0;
            const ScalarType tValue = std::accumulate(tStorage.begin(), tStorage.end(), tInitialValue);
            tConstraintCoefficientsR[tConstraintIndex] = tConstraintCoefficientsR[tConstraintIndex] - tValue;
        }
    }
    void initializeAuxiliaryVariables(const Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        assert(aDataMng.getNumDualVectors() == static_cast<OrdinalType>(1));

        const OrdinalType tDualVectorIndex = 0;
        Plato::Vector<ScalarType> & tAuxiliaryVariablesY =
                mTrialAuxiliaryVariableY->operator[](tDualVectorIndex);
        const Plato::Vector<ScalarType> & tCoefficientsA =
                mConstraintCoefficientsA->operator[](tDualVectorIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tCurrentConstraintValues =
                aDataMng.getCurrentConstraintValues(tDualVectorIndex);

        const OrdinalType tNumConstraints = mDualWorkVector->size();
        const ScalarType tMaxCoefficientA = mDualReductionOperations->max(tCoefficientsA);
        if(tMaxCoefficientA > static_cast<ScalarType>(0))
        {
            mDualWorkVector->fill(static_cast<ScalarType>(0));
            for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
            {
                if(tCoefficientsA[tIndex] > static_cast<ScalarType>(0))
                {
                    ScalarType tValue = std::max(static_cast<ScalarType>(0), tCurrentConstraintValues[tIndex]);
                    (*mDualWorkVector)[tIndex] = tValue / tCoefficientsA[tIndex];
                    tAuxiliaryVariablesY[tIndex] = 0;
                }
                else
                {
                    tAuxiliaryVariablesY[tIndex] =
                            std::max(static_cast<ScalarType>(0), tCurrentConstraintValues[tIndex]);
                }
            }
            mTrialAuxiliaryVariableZ = mDualReductionOperations->max(mDualWorkVector.operator*());
        }
        else
        {
            for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
            {
                tAuxiliaryVariablesY[tIndex] = tCurrentConstraintValues[tIndex];
            }
            mTrialAuxiliaryVariableZ = 0;
        }
    }
    void checkConstraintCoefficients()
    {
        const OrdinalType tDualVectorIndex = 0;
        ScalarType tMinCoeffA = mDualReductionOperations->min(mConstraintCoefficientsA->operator[](tDualVectorIndex));
        assert(tMinCoeffA >= static_cast<ScalarType>(0));
        ScalarType tMinCoeffC = mDualReductionOperations->min(mConstraintCoefficientsC->operator[](tDualVectorIndex));
        assert(tMinCoeffC >= static_cast<ScalarType>(0));
        ScalarType tMinCoeffD = mDualReductionOperations->min(mConstraintCoefficientsD->operator[](tDualVectorIndex));
        assert(tMinCoeffD >= static_cast<ScalarType>(0));
    }

private:
    void initialize(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory)
    {
        const OrdinalType tDualVectorIndex = 0;
        mDualWorkVector = aDataFactory.dual(tDualVectorIndex).create();

        const OrdinalType tControlVectorIndex = 0;
        mControlWorkVectorOne = aDataFactory.control(tControlVectorIndex).create();
        mControlWorkVectorTwo = aDataFactory.control(tControlVectorIndex).create();

        const OrdinalType tNumConstraints = aDataFactory.dual(tDualVectorIndex).size();
        mConstraintCoefficientsP.resize(tNumConstraints);
        mConstraintCoefficientsQ.resize(tNumConstraints);
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            mConstraintCoefficientsP[tIndex] = aDataFactory.control().create();
            mConstraintCoefficientsQ[tIndex] = aDataFactory.control().create();
        }

        Plato::fill(static_cast<ScalarType>(0), mConstraintCoefficientsA.operator*());
        Plato::fill(static_cast<ScalarType>(1), mConstraintCoefficientsD.operator*());
        Plato::fill(static_cast<ScalarType>(1e3), mConstraintCoefficientsC.operator*());

        // Allocate element wise functions
        Plato::MemorySpace::type_t tMemorySpace = aDataFactory.getMemorySpace();
        switch(tMemorySpace)
        {
            case Plato::MemorySpace::DEVICE:
            {
                mElementWiseFunctions = std::make_shared<Plato::DeviceDualProbElementWise<ScalarType, OrdinalType>>();
                break;
            }
            default:
            case Plato::MemorySpace::HOST:
            {
                mElementWiseFunctions = std::make_shared<Plato::HostDualProbElementWise<ScalarType, OrdinalType>>();
                break;
            }
        }
    }
    ScalarType computeMovingAsymptotesContribution()
    {
        const OrdinalType tNumVectors = mTrialControl->getNumVectors();
        std::vector<ScalarType> tMySumP(tNumVectors);
        std::vector<ScalarType> tMySumQ(tNumVectors);

        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            mControlWorkVectorOne->fill(static_cast<ScalarType>(0));
            mControlWorkVectorTwo->fill(static_cast<ScalarType>(0));
            const Plato::Vector<ScalarType, OrdinalType> & tMyTrialControl = mTrialControl->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyLowerAsymptotes =
                    mLowerAsymptotes->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyUpperAsymptotes =
                    mUpperAsymptotes->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyObjectiveCoefficientsP =
                    mObjectiveCoefficientsP->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyObjectiveCoefficientsQ =
                    mObjectiveCoefficientsQ->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyDualTimesCoefficientsP =
                    mDualTimesCoefficientsP->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyDualTimesCoefficientsQ =
                    mDualTimesCoefficientsQ->operator[](tVectorIndex);

            mElementWiseFunctions->updateMovingAsymptotesCoeff(tMyTrialControl,
                                                               tMyLowerAsymptotes,
                                                               tMyUpperAsymptotes,
                                                               tMyObjectiveCoefficientsP,
                                                               tMyObjectiveCoefficientsQ,
                                                               tMyDualTimesCoefficientsP,
                                                               tMyDualTimesCoefficientsQ,
                                                               *mControlWorkVectorOne,
                                                               *mControlWorkVectorTwo);

            tMySumP[tVectorIndex] = mControlReductionOperations->sum(mControlWorkVectorOne.operator*());
            tMySumQ[tVectorIndex] = mControlReductionOperations->sum(mControlWorkVectorTwo.operator*());
        }

        const ScalarType tInitialValue = 0;
        const ScalarType tGlobalSumP = std::accumulate(tMySumP.begin(), tMySumP.end(), tInitialValue);
        const ScalarType tGlobalSumQ = std::accumulate(tMySumQ.begin(), tMySumQ.end(), tInitialValue);
        const ScalarType tMovingAsymptotesTerm = tGlobalSumP + tGlobalSumQ;

        return (tMovingAsymptotesTerm);
    }
    // Compute trial controls based on the following explicit expression:
    // \[ x(\lambda)=\frac{u_j^k\mathtt{b}^{1/2}+l_j^k\mathtt{a}^{1/2}}{(\mathtt{a}^{1/2}+\mathtt{b}^{1/2})} \],
    // where
    //      \[ \mathtt{a}=(p_{0j}+\lambda^{\intercal}p_j) \] and [ \mathtt{b}=(q_{0j}+\lambda^{\intercal}q_j) ]
    //      and j=1\dots,n_{x}
    // Here, x denotes the trial control vector
    void computeTrialControl(const Plato::MultiVector<ScalarType, OrdinalType> & aDual)
    {
        assert(aDual.getNumVectors() == static_cast<OrdinalType>(1));

        Plato::update(static_cast<ScalarType>(1), *mObjectiveCoefficientsP, static_cast<ScalarType>(0), *mTermA);
        Plato::update(static_cast<ScalarType>(1), *mDualTimesCoefficientsP, static_cast<ScalarType>(1), *mTermA);
        Plato::update(static_cast<ScalarType>(1), *mObjectiveCoefficientsQ, static_cast<ScalarType>(0), *mTermB);
        Plato::update(static_cast<ScalarType>(1), *mDualTimesCoefficientsQ, static_cast<ScalarType>(1), *mTermB);

        const OrdinalType tNumControlVectors = mTrialControl->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tMyTrialControl = mTrialControl->operator[](tVectorIndex);
            
            const Plato::Vector<ScalarType, OrdinalType> & tMyTermA = mTermA->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyTermB = mTermB->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyLowerAsymptotes =
                    mLowerAsymptotes->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyUpperAsymptotes =
                    mUpperAsymptotes->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyLowerBounds =
                    mTrialControlLowerBounds->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyUpperBounds =
                    mTrialControlUpperBounds->operator[](tVectorIndex);

            mElementWiseFunctions->updateTrialControl(tMyTermA,
                                                      tMyTermB,
                                                      tMyLowerBounds,
                                                      tMyUpperBounds,
                                                      tMyLowerAsymptotes,
                                                      tMyUpperAsymptotes,
                                                      tMyTrialControl);
        }
    }
    /*!
     * Update auxiliary variables based on the following expression:
     *  \[ y_i(\lambda)=\frac{\lambda_i-c_i}{2d_i} \]
     *  and
     *  \[ z(\lambda)=\frac{\lambda^{\intercal}a-a_0}{2\varepsilon} \]
     */
    void computeTrialAuxiliaryVariables(const Plato::MultiVector<ScalarType, OrdinalType> & aDual)
    {
        assert(aDual.getNumVectors() == static_cast<OrdinalType>(1));

        OrdinalType tNumVectors = aDual.getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyDual = aDual[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tMyCoefficientsC = (*mConstraintCoefficientsC)[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tMyCoefficientsD = (*mConstraintCoefficientsD)[tVectorIndex];
            Plato::Vector<ScalarType, OrdinalType> & tMyAuxiliaryVariablesY = (*mTrialAuxiliaryVariableY)[tVectorIndex];

            const OrdinalType tNumDual = tMyAuxiliaryVariablesY.size();
            for(OrdinalType tIndex = 0; tIndex < tNumDual; tIndex++)
            {
                ScalarType tDualMinusConstraintCoefficientC = tMyDual[tIndex] - tMyCoefficientsC[tIndex];
                tMyAuxiliaryVariablesY[tIndex] = tDualMinusConstraintCoefficientC / tMyCoefficientsD[tIndex];
                // Project auxiliary variables Y to feasible set (Y >= 0)
                tMyAuxiliaryVariablesY[tIndex] = std::max(tMyAuxiliaryVariablesY[tIndex], static_cast<ScalarType>(0));
            }
        }
        ScalarType tDualDotConstraintCoefficientA = Plato::dot(aDual, *mConstraintCoefficientsA);
        mTrialAuxiliaryVariableZ = (tDualDotConstraintCoefficientA - mObjectiveCoefficientA)
                / (static_cast<ScalarType>(2) * mEpsilon);
        // Project auxiliary variables Z to feasible set (Z >= 0)
        mTrialAuxiliaryVariableZ = std::max(mTrialAuxiliaryVariableZ, static_cast<ScalarType>(0));
    }
    /*! Compute: \sum_{i=1}^{m}\left( c_iy_i + \frac{1}{2}d_iy_i^2 \right) -
     * \lambda^{T}y - (\lambda^{T}a)z + \lambda^{T}r, where m is the number of constraints
     **/
    ScalarType computeConstraintContribution(const Plato::MultiVector<ScalarType, OrdinalType> & aDual)
    {
        assert(aDual.getNumVectors() == static_cast<OrdinalType>(1));

        const OrdinalType tNumVectors = aDual.getNumVectors();
        std::vector<ScalarType> tStorage(tNumVectors);
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            mDualWorkVector->fill(static_cast<ScalarType>(0));
            const Plato::Vector<ScalarType, OrdinalType> & tMyDual = aDual[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tMyCoefficientsC = (*mConstraintCoefficientsC)[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tMyCoefficientsD = (*mConstraintCoefficientsD)[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tMyAuxiliaryVariablesY =
                    (*mTrialAuxiliaryVariableY)[tVectorIndex];

            const OrdinalType tNumDuals = tMyDual.size();
            for(OrdinalType tIndex = 0; tIndex < tNumDuals; tIndex++)
            {
                ScalarType tValueOne = tMyCoefficientsC[tIndex] * tMyAuxiliaryVariablesY[tIndex];
                ScalarType tValueTwo = tMyCoefficientsD[tIndex] * tMyAuxiliaryVariablesY[tIndex] * tMyAuxiliaryVariablesY[tIndex];
                (*mDualWorkVector)[tIndex] = tValueOne + tValueTwo;
            }

            tStorage[tVectorIndex] = mDualReductionOperations->sum(mDualWorkVector.operator*());
        }

        const ScalarType tInitialValue = 0;
        ScalarType tConstraintSummationTerm = std::accumulate(tStorage.begin(), tStorage.end(), tInitialValue);

        // Add additional contributions to inequality summation term
        ScalarType tDualDotConstraintCoeffR = Plato::dot(aDual, *mConstraintCoefficientsR);
        ScalarType tDualDotConstraintCoeffA = Plato::dot(aDual, *mConstraintCoefficientsA);
        ScalarType tDualDotTrialAuxiliaryVariableY = Plato::dot(aDual, *mTrialAuxiliaryVariableY);
        ScalarType tOutput = tConstraintSummationTerm - tDualDotTrialAuxiliaryVariableY
                - (tDualDotConstraintCoeffA * mTrialAuxiliaryVariableZ) + tDualDotConstraintCoeffR;

        return (tOutput);
    }
    /*
     * Compute \lambda_j\times{p}_j and \lambda_j\times{q}_j, where
     * j=1,\dots,N_{c}. Here, N_{c} denotes the number of constraints.
     **/
    void computeDualTimesConstraintCoefficientTerms(const Plato::MultiVector<ScalarType, OrdinalType> & aDual)
    {
        const OrdinalType tDualVectorIndex = 0;
        Plato::fill(static_cast<ScalarType>(0), mDualTimesCoefficientsP.operator*());
        Plato::fill(static_cast<ScalarType>(0), mDualTimesCoefficientsQ.operator*());
        const Plato::Vector<ScalarType, OrdinalType> & tDual = aDual[tDualVectorIndex];

        const ScalarType tBeta = 1;
        const OrdinalType tNumConstraints = tDual.size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintCoefficientsP =
                    mConstraintCoefficientsP[tConstraintIndex].operator*();
            const Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintCoefficientsQ =
                    mConstraintCoefficientsQ[tConstraintIndex].operator*();

            const OrdinalType tNumControlVectors = tMyConstraintCoefficientsP.getNumVectors();
            for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
            {
                const Plato::Vector<ScalarType, OrdinalType> & tMyCoefficientsP =
                        tMyConstraintCoefficientsP[tVectorIndex];
                Plato::Vector<ScalarType, OrdinalType> & tMyDualTimesCoefficientsP =
                        (*mDualTimesCoefficientsP)[tVectorIndex];
                tMyDualTimesCoefficientsP.update(tDual[tConstraintIndex], tMyCoefficientsP, tBeta);

                const Plato::Vector<ScalarType, OrdinalType> & tMyCoefficientsQ =
                        tMyConstraintCoefficientsQ[tVectorIndex];
                Plato::Vector<ScalarType, OrdinalType> & tMyDualTimesCoefficientsQ =
                        (*mDualTimesCoefficientsQ)[tVectorIndex];
                tMyDualTimesCoefficientsQ.update(tDual[tConstraintIndex], tMyCoefficientsQ, tBeta);
            }
        }
    }

private:
    ScalarType mEpsilon;
    ScalarType mObjectiveCoefficientA;
    ScalarType mObjectiveCoefficientR;
    ScalarType mTrialAuxiliaryVariableZ;
    ScalarType mCurrentObjectiveFunctionValue;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mDualWorkVector;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWorkVectorOne;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWorkVectorTwo;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTermA;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTermB;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerAsymptotes;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperAsymptotes;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDualTimesCoefficientsP;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDualTimesCoefficientsQ;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mObjectiveCoefficientsP;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mObjectiveCoefficientsQ;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialControlLowerBounds;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialControlUpperBounds;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mConstraintCoefficientsA;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mConstraintCoefficientsC;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mConstraintCoefficientsD;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mConstraintCoefficientsR;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialAuxiliaryVariableY;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentConstraintValues;

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mDualReductionOperations;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductionOperations;
    std::shared_ptr<Plato::DualProbElementWiseFunctions<ScalarType, OrdinalType>> mElementWiseFunctions;

    std::vector<std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>>> mConstraintCoefficientsP;
    std::vector<std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>>> mConstraintCoefficientsQ;

private:
    DualProblemStageMng(const Plato::DualProblemStageMng<ScalarType, OrdinalType> & aRhs);
    Plato::DualProblemStageMng<ScalarType, OrdinalType> & operator=(const Plato::DualProblemStageMng<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_DUALPROBLEMSTAGEMNG_HPP_ */
