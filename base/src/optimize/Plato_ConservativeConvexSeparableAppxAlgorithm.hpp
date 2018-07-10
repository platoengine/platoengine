/*
 * Plato_ConservativeConvexSeparableAppxAlgorithm.hpp
 *
 *  Created on: Nov 4, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_CONSERVATIVECONVEXSEPARABLEAPPXALGORITHM_HPP_
#define PLATO_CONSERVATIVECONVEXSEPARABLEAPPXALGORITHM_HPP_

#include <memory>
#include <cassert>
#include <iostream>
#include <stdexcept>
#include <algorithm>

#include "Plato_Vector.hpp"
#include "Plato_ErrorChecks.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_MultiVectorList.hpp"
#include "Plato_PrimalProblemStageMng.hpp"
#include "Plato_ConservativeConvexSeparableAppxDataMng.hpp"
#include "Plato_ConservativeConvexSeparableAppxStageMng.hpp"
#include "Plato_ConservativeConvexSeparableApproximation.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class ConservativeConvexSeparableAppxAlgorithm
{
public:
    ConservativeConvexSeparableAppxAlgorithm(const std::shared_ptr<Plato::ConservativeConvexSeparableAppxStageMng<ScalarType, OrdinalType>> & aStageMng,
                                             const std::shared_ptr<Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType>> & aDataMng,
                                             const std::shared_ptr<Plato::ConservativeConvexSeparableApproximation<ScalarType, OrdinalType>> & aSubProblem) :
            mMaxNumIterations(500),
            mNumIterationsDone(0),
            mStagnationTolerance(1e-8),
            mStationarityTolerance(1e-4),
            mObjectiveStagnationTolerance(1e-6),
            mMovingAsymptoteExpansionFactor(1.2),
            mMovingAsymptoteContractionFactor(0.4),
            mInitialMovingAsymptoteScaleFactor(0.5),
            mKarushKuhnTuckerConditionsTolerance(1e-5),
            mMovingAsymptoteUpperBoundScaleFactor(10),
            mMovingAsymptoteLowerBoundScaleFactor(0.01),
            mStoppingCriterion(Plato::ccsa::stop_t::NOT_CONVERGED),
            mDualWork(),
            mControlWork(),
            mPreviousSigma(),
            mAntepenultimateControl(),
            mWorkMultiVectorList(),
            mDataMng(aDataMng),
            mStageMng(aStageMng),
            mSubProblem(aSubProblem)
    {
        this->initialize(aDataMng.operator*());
    }
    ~ConservativeConvexSeparableAppxAlgorithm()
    {
    }

    OrdinalType getMaxNumIterations() const
    {
        return (mMaxNumIterations);
    }
    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumIterations = aInput;
    }
    OrdinalType getNumIterationsDone() const
    {
        return (mNumIterationsDone);
    }

    ScalarType getStagnationTolerance() const
    {
        return (mStagnationTolerance);
    }
    void setStagnationTolerance(const ScalarType & aInput)
    {
        mStagnationTolerance = aInput;
    }
    ScalarType getStationarityTolerance() const
    {
        return (mStationarityTolerance);
    }
    void setStationarityTolerance(const ScalarType & aInput)
    {
        mStationarityTolerance = aInput;
    }
    ScalarType getObjectiveStagnationTolerance() const
    {
        return (mObjectiveStagnationTolerance);
    }
    void setObjectiveStagnationTolerance(const ScalarType & aInput)
    {
        mObjectiveStagnationTolerance = aInput;
    }
    ScalarType getMovingAsymptoteExpansionFactor() const
    {
        return (mMovingAsymptoteExpansionFactor);
    }
    void setMovingAsymptoteExpansionFactor(const ScalarType & aInput)
    {
        mMovingAsymptoteExpansionFactor = aInput;
    }
    ScalarType getMovingAsymptoteContractionFactor() const
    {
        return (mMovingAsymptoteContractionFactor);
    }
    void setMovingAsymptoteContractionFactor(const ScalarType & aInput)
    {
        mMovingAsymptoteContractionFactor = aInput;
    }
    ScalarType getInitialMovingAsymptoteScaleFactor() const
    {
        return (mInitialMovingAsymptoteScaleFactor);
    }
    void setInitialMovingAsymptoteScaleFactor(const ScalarType & aInput)
    {
        mInitialMovingAsymptoteScaleFactor = aInput;
    }
    ScalarType getKarushKuhnTuckerConditionsTolerance() const
    {
        return (mKarushKuhnTuckerConditionsTolerance);
    }
    void setKarushKuhnTuckerConditionsTolerance(const ScalarType & aInput)
    {
        mKarushKuhnTuckerConditionsTolerance = aInput;
    }
    ScalarType getMovingAsymptoteUpperBoundScaleFactor() const
    {
        return (mMovingAsymptoteUpperBoundScaleFactor);
    }
    void setMovingAsymptoteUpperBoundScaleFactor(const ScalarType & aInput)
    {
        mMovingAsymptoteUpperBoundScaleFactor = aInput;
    }
    ScalarType getMovingAsymptoteLowerBoundScaleFactor() const
    {
        return (mMovingAsymptoteLowerBoundScaleFactor);
    }
    void setMovingAsymptoteLowerBoundScaleFactor(const ScalarType & aInput)
    {
        mMovingAsymptoteLowerBoundScaleFactor = aInput;
    }

    Plato::ccsa::stop_t getStoppingCriterion() const
    {
        return (mStoppingCriterion);
    }
    void setStoppingCriterion(const Plato::ccsa::stop_t & aInput)
    {
        mStoppingCriterion = aInput;
    }

    void solve()
    {
        this->checkInitialGuess();

        const Plato::MultiVector<ScalarType, OrdinalType> & tControl = mDataMng->getCurrentControl();
        const ScalarType tSMCurrentObjectiveFunctionValue = mStageMng->evaluateObjective(tControl);
        mDataMng->setCurrentObjectiveFunctionValue(tSMCurrentObjectiveFunctionValue);
        mStageMng->evaluateConstraints(tControl, mDualWork.operator*());
        mDataMng->setCurrentConstraintValues(mDualWork.operator*());
        mDataMng->setDual(mDualWork.operator*());
        mSubProblem->initializeAuxiliaryVariables(mDataMng.operator*());

        while(1)
        {
            const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = mDataMng->getCurrentControl();
            mStageMng->cacheData();
            mStageMng->computeGradient(tCurrentControl, mControlWork.operator*());
            mDataMng->setCurrentObjectiveGradient(mControlWork.operator*());
            mStageMng->computeConstraintGradients(tCurrentControl, mWorkMultiVectorList.operator*());
            mDataMng->setCurrentConstraintGradients(mWorkMultiVectorList.operator*());

            if(this->checkStoppingCriteria() == true)
            {
                break;
            }

            this->updateSigmaParameters();

            const Plato::MultiVector<ScalarType, OrdinalType> & tPreviousControl = mDataMng->getPreviousControl();
            Plato::update(static_cast<ScalarType>(1),
                          tPreviousControl,
                          static_cast<ScalarType>(0),
                          mAntepenultimateControl.operator*());
            Plato::update(static_cast<ScalarType>(1),
                          tCurrentControl,
                          static_cast<ScalarType>(0),
                          mControlWork.operator*());
            mDataMng->setPreviousControl(mControlWork.operator*());

            const ScalarType tDMCurrentObjectiveFunctionValue = mDataMng->getCurrentObjectiveFunctionValue();
            mDataMng->setPreviousObjectiveFunctionValue(tDMCurrentObjectiveFunctionValue);

            mSubProblem->solve(mStageMng.operator*(), mDataMng.operator*());
            mStageMng->update(mDataMng.operator*());

            mNumIterationsDone++;
        }
    }

private:
    void initialize(const Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        // Allocate Core Data Structures
        mDualWork = aDataMng.getDual().create();
        mControlWork = aDataMng.getCurrentControl().create();
        mPreviousSigma = aDataMng.getCurrentControl().create();
        mAntepenultimateControl = aDataMng.getCurrentControl().create();
        mWorkMultiVectorList = aDataMng.getCurrentConstraintGradients().create();

        // Check Bounds
        try
        {
            const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getControlLowerBounds();
            const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getControlUpperBounds();
            Plato::error::checkBounds(tLowerBounds, tUpperBounds);
        }
        catch(const std::invalid_argument & tErrorMsg)
        {
            std::cout << tErrorMsg.what() << std::flush;
            std::abort();
        }
    }
    void checkInitialGuess()
    {
        try
        {
            bool tIsInitialGuessSet = mDataMng->isInitialGuessSet();
            Plato::error::checkInitialGuessIsSet(tIsInitialGuessSet);
        }
        catch(const std::invalid_argument & tErrorMsg)
        {
            std::cout << tErrorMsg.what() << std::flush;
            std::abort();
        }

        try
        {
            const Plato::MultiVector<ScalarType, OrdinalType> & tControl = mDataMng->getCurrentControl();
            const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getControlLowerBounds();
            const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getControlUpperBounds();
            Plato::error::checkInitialGuess(tControl, tLowerBounds, tUpperBounds);
        }
        catch(const std::invalid_argument & tErrorMsg)
        {
            std::cout << tErrorMsg.what() << std::flush;
            std::abort();
        }

        const Plato::MultiVector<ScalarType, OrdinalType> & tControl = mDataMng->getCurrentControl();
        const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getControlUpperBounds();
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tWorkMultiVector = tControl.create();
        Plato::update(static_cast<ScalarType>(1), tControl, static_cast<ScalarType>(0), *tWorkMultiVector);
        mDataMng->bounds().project(tLowerBounds, tUpperBounds, *tWorkMultiVector);
        mDataMng->setCurrentControl(*tWorkMultiVector);
    }
    bool checkStoppingCriteria()
    {
        bool tStop = false;

        mDataMng->computeStagnationMeasure();
        mDataMng->computeFeasibilityMeasure();
        mDataMng->computeStationarityMeasure();
        mDataMng->computeNormInactiveGradient();
        mDataMng->computeObjectiveStagnationMeasure();

        const Plato::MultiVector<ScalarType, OrdinalType> & tDual = mDataMng->getDual();
        const Plato::MultiVector<ScalarType, OrdinalType> & tControl = mDataMng->getCurrentControl();
        mDataMng->computeKarushKuhnTuckerConditionsInexactness(tControl, tDual);

        const ScalarType tStagnationMeasure = mDataMng->getStagnationMeasure();
        const ScalarType tStationarityMeasure = mDataMng->getStationarityMeasure();
        const ScalarType tObjectiveStagnationMeasure = mDataMng->getObjectiveStagnationMeasure();
        const ScalarType t_KKT_ConditionsInexactness = mDataMng->getKarushKuhnTuckerConditionsInexactness();

        if(tStagnationMeasure < this->getStagnationTolerance())
        {
            tStop = true;
            this->setStoppingCriterion(Plato::ccsa::stop_t::CONTROL_STAGNATION);
        }
        else if(tStationarityMeasure < this->getStationarityTolerance())
        {
            tStop = true;
            this->setStoppingCriterion(Plato::ccsa::stop_t::STATIONARITY_TOLERANCE);
        }
        else if(t_KKT_ConditionsInexactness < this->getKarushKuhnTuckerConditionsTolerance())
        {
            tStop = true;
            this->setStoppingCriterion(Plato::ccsa::stop_t::KKT_CONDITIONS_TOLERANCE);
        }
        else if(mNumIterationsDone >= this->getMaxNumIterations())
        {
            tStop = true;
            this->setStoppingCriterion(Plato::ccsa::stop_t::MAX_NUMBER_ITERATIONS);
        }
        else if(tObjectiveStagnationMeasure < this->getObjectiveStagnationTolerance())
        {
            tStop = true;
            this->setStoppingCriterion(Plato::ccsa::stop_t::OBJECTIVE_STAGNATION);
        }

        return (tStop);
    }
    void updateSigmaParameters()
    {
        assert(mControlWork.get() != nullptr);
        assert(mPreviousSigma.get() != nullptr);

        const Plato::MultiVector<ScalarType, OrdinalType> & tDMCurrentSigma = mDataMng->getCurrentSigma();
        Plato::update(static_cast<ScalarType>(1), tDMCurrentSigma, static_cast<ScalarType>(0), *mPreviousSigma);

        const OrdinalType tNumIterationsDone = this->getNumIterationsDone();
        if(tNumIterationsDone < static_cast<OrdinalType>(2))
        {
            const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getControlUpperBounds();
            Plato::update(static_cast<ScalarType>(1), tUpperBounds, static_cast<ScalarType>(0), *mControlWork);
            const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getControlLowerBounds();
            Plato::update(static_cast<ScalarType>(-1), tLowerBounds, static_cast<ScalarType>(1), *mControlWork);
            const ScalarType tInitialMovingAsymptoteScaleFactor = this->getInitialMovingAsymptoteScaleFactor();
            Plato::scale(tInitialMovingAsymptoteScaleFactor, mControlWork.operator*());
            mDataMng->setCurrentSigma(mControlWork.operator*());
        }
        else
        {
            const ScalarType tExpansionFactor = this->getMovingAsymptoteExpansionFactor();
            const ScalarType tContractionFactor = this->getMovingAsymptoteContractionFactor();
            const ScalarType tLowerBoundScaleFactor = this->getMovingAsymptoteLowerBoundScaleFactor();
            const ScalarType tUpperBoundScaleFactor = this->getMovingAsymptoteUpperBoundScaleFactor();
            std::map<Plato::element_wise::constant_t, ScalarType> tConstants;
            tConstants[Plato::element_wise::ExpansionFactor] = tExpansionFactor;
            tConstants[Plato::element_wise::ContractionFactor] = tContractionFactor;
            tConstants[Plato::element_wise::LowerBoundScaleFactor] = tLowerBoundScaleFactor;
            tConstants[Plato::element_wise::UpperBoundScaleFactor] = tUpperBoundScaleFactor;
            
            const OrdinalType tNumVectors = mControlWork->getNumVectors();
            Plato::fill(static_cast<ScalarType>(0), mControlWork.operator*());
            for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
            {
                const Plato::Vector<ScalarType, OrdinalType> & tCurrentControl = mDataMng->getCurrentControl(tVectorIndex);
                const Plato::Vector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getControlUpperBounds(tVectorIndex);
                const Plato::Vector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getControlLowerBounds(tVectorIndex);
                const Plato::Vector<ScalarType, OrdinalType> & tPreviousControl = mDataMng->getPreviousControl(tVectorIndex);
                const Plato::Vector<ScalarType, OrdinalType> & tPreviousSigma = mPreviousSigma->operator[](tVectorIndex);
                const Plato::Vector<ScalarType, OrdinalType> & tAntepenultimateControl = mAntepenultimateControl->operator[](tVectorIndex);

                Plato::Vector<ScalarType, OrdinalType> & tCurrentSigma = mControlWork->operator[](tVectorIndex);
                tCurrentSigma.update(static_cast<ScalarType>(1), tPreviousSigma, static_cast<ScalarType>(0));

                mDataMng->elementWiseFunctions().updateSigmaCoeff(tConstants,
                                                                  tCurrentControl,
                                                                  tPreviousControl,
                                                                  tAntepenultimateControl,
                                                                  tUpperBounds,
                                                                  tLowerBounds,
                                                                  tPreviousSigma,
                                                                  tCurrentSigma);
            }
            mDataMng->setCurrentSigma(mControlWork.operator*());
        }
    }

private:
    OrdinalType mMaxNumIterations;
    OrdinalType mNumIterationsDone;

    ScalarType mStagnationTolerance;
    ScalarType mStationarityTolerance;
    ScalarType mObjectiveStagnationTolerance;
    ScalarType mMovingAsymptoteExpansionFactor;
    ScalarType mMovingAsymptoteContractionFactor;
    ScalarType mInitialMovingAsymptoteScaleFactor;
    ScalarType mKarushKuhnTuckerConditionsTolerance;
    ScalarType mMovingAsymptoteUpperBoundScaleFactor;
    ScalarType mMovingAsymptoteLowerBoundScaleFactor;

    Plato::ccsa::stop_t mStoppingCriterion;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDualWork;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlWork;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousSigma;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAntepenultimateControl;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mWorkMultiVectorList;

    std::shared_ptr<Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType>> mDataMng;
    std::shared_ptr<Plato::ConservativeConvexSeparableAppxStageMng<ScalarType, OrdinalType>> mStageMng;
    std::shared_ptr<Plato::ConservativeConvexSeparableApproximation<ScalarType, OrdinalType>> mSubProblem;

private:
    ConservativeConvexSeparableAppxAlgorithm(const Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> & aRhs);
    Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> & operator=(const Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_CONSERVATIVECONVEXSEPARABLEAPPXALGORITHM_HPP_ */
