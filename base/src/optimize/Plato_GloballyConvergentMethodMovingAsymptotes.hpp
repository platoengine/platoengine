/*
 * Plato_GloballyConvergentMethodMovingAsymptotes.hpp
 *
 *  Created on: Nov 4, 2017
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

#ifndef PLATO_GLOBALLYCONVERGENTMETHODMOVINGASYMPTOTES_HPP_
#define PLATO_GLOBALLYCONVERGENTMETHODMOVINGASYMPTOTES_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <cassert>
#include <numeric>
#include <algorithm>
#include <mpi.h>

#include "Plato_Vector.hpp"
#include "Plato_BoundsBase.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_ReductionOperations.hpp"
#include "Plato_DualSolverInterface.hpp"
#include "Plato_ConservativeConvexSeparableAppxDataMng.hpp"
#include "Plato_ConservativeConvexSeparableAppxStageMng.hpp"
#include "Plato_ConservativeConvexSeparableApproximation.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class GloballyConvergentMethodMovingAsymptotes : public Plato::ConservativeConvexSeparableApproximation<ScalarType, OrdinalType>
{
public:
    explicit GloballyConvergentMethodMovingAsymptotes(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory) :
            mMaxNumIterations(5),
            mNumIterationsDone(0),
            mControlStagnationTolerance(1e-8),
            mObjectiveStagnationTolerance(1e-8),
            mMinObjectiveGlobalizationFactor(1e-5),
            mCurrentTrialObjectiveFunctionValue(std::numeric_limits<ScalarType>::max()),
            mPreviousTrialObjectiveFunctionValue(std::numeric_limits<ScalarType>::max()),
            mKarushKuhnTuckerConditionsTolerance(5e-4),
            mStoppingCriterion(Plato::ccsa::stop_t::NOT_CONVERGED),
            mDualWorkOne(),
            mControlWorkOne(),
            mControlWorkTwo(),
            mTrialDual(aDataFactory->dual().create()),
            mActiveSet(aDataFactory->control().create()),
            mControlWork(aDataFactory->control().create()),
            mInactiveSet(aDataFactory->control().create()),
            mDeltaControl(aDataFactory->control().create()),
            mTrialControl(aDataFactory->control().create()),
            mPreviousTrialControl(aDataFactory->control().create()),
            mTrialConstraintValues(aDataFactory->dual().create()),
            mMinConstraintGlobalizationFactors(aDataFactory->dual().create()),
            mDualSolver(std::make_shared<Plato::DualSolverInterface<ScalarType, OrdinalType>>(aDataFactory.operator*())),
            mControlReductionOperations(aDataFactory->getControlReductionOperations().create())
    {
        this->initialize();
    }
    virtual ~GloballyConvergentMethodMovingAsymptotes()
    {
    }

    /******************************************************************************//**
     * @brief Return maximum number of inner iterations
     * @return maximum number of inner iterations
    **********************************************************************************/
    OrdinalType getMaxNumIterations() const
    {
        return (mMaxNumIterations);
    }

    /******************************************************************************//**
     * @brief Set maximum number of inner iterations
     * @param [in] aInput maximum number of inner iterations
    **********************************************************************************/
    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumIterations = aInput;
    }

    /******************************************************************************//**
     * @brief Return number of inner iterations done
     * @return number of inner iterations done
    **********************************************************************************/
    OrdinalType getNumIterationsDone() const
    {
        return (mNumIterationsDone);
    }

    /******************************************************************************//**
     * @brief Set number of inner iterations done
     * @param [in] aInput number of inner iterations done
    **********************************************************************************/
    void setNumIterationsDone(const OrdinalType & aInput)
    {
        mNumIterationsDone = aInput;
    }

    /******************************************************************************//**
     * @brief Return control stagnation tolerance for GCMMA subproblem
     * @return control stagnation tolerance for GCMMA subproblem
    **********************************************************************************/
    ScalarType getControlStagnationTolerance() const
    {
        return (mControlStagnationTolerance);
    }

    /******************************************************************************//**
     * @brief Set control stagnation tolerance for GCMMA subproblem
     * @param [in] aInput control stagnation tolerance for GCMMA subproblem
    **********************************************************************************/
    void setControlStagnationTolerance(const ScalarType & aInput)
    {
        mControlStagnationTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Return stagnation tolerance for first-order objective approximation
     * @return stagnation tolerance for first-order objective approximation
    **********************************************************************************/
    ScalarType getObjectiveStagnationTolerance() const
    {
        return (mObjectiveStagnationTolerance);
    }

    /******************************************************************************//**
     * @brief Set stagnation tolerance for first-order objective approximation
     * @param [in] aInput stagnation tolerance for first-order objective approximation
    **********************************************************************************/
    void setObjectiveStagnationTolerance(const ScalarType & aInput)
    {
        mObjectiveStagnationTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Return Karush-Kuhn-Tucker (KKT) tolerance for GCMMA subproblem
     * @return Karush-Kuhn-Tucker (KKT) tolerance for GCMMA subproblem
    **********************************************************************************/
    ScalarType getKarushKuhnTuckerConditionsTolerance() const
    {
        return (mKarushKuhnTuckerConditionsTolerance);
    }

    /******************************************************************************//**
     * @brief Set Karush-Kuhn-Tucker (KKT) tolerance for GCMMA subproblem
     * @param [in] aInput Karush-Kuhn-Tucker (KKT) tolerance for GCMMA subproblem
    **********************************************************************************/
    void setKarushKuhnTuckerConditionsTolerance(const ScalarType & aInput)
    {
        mKarushKuhnTuckerConditionsTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Return GCMMA subproblem stopping criterion
     * @return GCMMA subproblem stopping criterion
    **********************************************************************************/
    Plato::ccsa::stop_t getStoppingCriterion() const
    {
        return (mStoppingCriterion);
    }

    /******************************************************************************//**
     * @brief Set GCMMA subproblem stopping criterion
     * @param [in] aInput GCMMA subproblem stopping criterion
    **********************************************************************************/
    void setStoppingCriterion(const Plato::ccsa::stop_t & aInput)
    {
        mStoppingCriterion = aInput;
    }

    /******************************************************************************//**
     * @brief Set algorithm for dual problem
     * @param [in] aInput algorithm for dual problem
    **********************************************************************************/
    void setDualSolver(const std::shared_ptr<Plato::DualProblemSolver<ScalarType, OrdinalType>> & aInput)
    {
        mDualSolver = aInput;
    }

    /******************************************************************************//**
     * @brief Solve GCMMA subproblem
     * @param [in] aStageMng main interface between user-defined criteria
     * @param [in] aDataMng data manager for CCSA algorithm
    **********************************************************************************/
    void solve(Plato::ConservativeConvexSeparableAppxStageMng<ScalarType, OrdinalType> & aStageMng,
               Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mDualSolver->update(aDataMng);

        OrdinalType tIterations = 0;
        this->setNumIterationsDone(tIterations);
        while(1)
        {
            mDualSolver->updateObjectiveCoefficients(aDataMng);
            mDualSolver->updateConstraintCoefficients(aDataMng);
            mDualSolver->solve(mTrialDual.operator*(), mTrialControl.operator*());

            const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = aDataMng.getControlLowerBounds();
            const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = aDataMng.getControlUpperBounds();
            aDataMng.bounds().project(tLowerBounds, tUpperBounds, *mTrialControl);
            aDataMng.bounds().computeActiveAndInactiveSets(*mTrialControl,
                                                           tLowerBounds,
                                                           tUpperBounds,
                                                           *mActiveSet,
                                                           *mInactiveSet);
            aDataMng.setActiveSet(mActiveSet.operator*());
            aDataMng.setInactiveSet(mInactiveSet.operator*());

            mPreviousTrialObjectiveFunctionValue = mCurrentTrialObjectiveFunctionValue;
            mCurrentTrialObjectiveFunctionValue = aStageMng.evaluateObjective(mTrialControl.operator*());
            aStageMng.evaluateConstraints(mTrialControl.operator*(), mTrialConstraintValues.operator*());

            const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = aDataMng.getCurrentControl();
            Plato::update(static_cast<ScalarType>(1), *mTrialControl, static_cast<ScalarType>(0), *mDeltaControl);
            Plato::update(static_cast<ScalarType>(-1), tCurrentControl, static_cast<ScalarType>(1), *mDeltaControl);
            this->updateObjectiveGlobalizationFactor(aDataMng);
            this->updateConstraintGlobalizationFactors(aDataMng);

            tIterations++;
            this->setNumIterationsDone(tIterations);
            if(this->checkStoppingCriteria(aDataMng) == true)
            {
                break;
            }
            Plato::update(static_cast<ScalarType>(1), *mTrialControl, static_cast<ScalarType>(0), *mPreviousTrialControl);
        }

        this->checkGlobalizationFactors(aDataMng);

        aDataMng.setDual(mTrialDual.operator*());
        aDataMng.setCurrentControl(mTrialControl.operator*());
        aDataMng.setCurrentObjectiveFunctionValue(mCurrentTrialObjectiveFunctionValue);
        aDataMng.setCurrentConstraintValues(mTrialConstraintValues.operator*());
    }
    void initializeAuxiliaryVariables(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mDualSolver->initializeAuxiliaryVariables(aDataMng);
    }

private:
    void initialize()
    {
        const OrdinalType tVectorIndex = 0;
        mDualWorkOne = mTrialDual->operator[](tVectorIndex).create();
        mControlWorkOne = mTrialControl->operator[](tVectorIndex).create();
        mControlWorkTwo = mTrialControl->operator[](tVectorIndex).create();
        Plato::fill(static_cast<ScalarType>(1e-5), mMinConstraintGlobalizationFactors.operator*());
    }
    void updateObjectiveGlobalizationFactor(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mControlWorkOne->fill(static_cast<ScalarType>(0));
        mControlWorkTwo->fill(static_cast<ScalarType>(0));

        const OrdinalType tNumVectors = mTrialControl->getNumVectors();
        std::vector<ScalarType> tStorageOne(tNumVectors, 0);
        std::vector<ScalarType> tStorageTwo(tNumVectors, 0);

        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tDeltaControl = mDeltaControl->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentSigma = aDataMng.getCurrentSigma(tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tCurrentGradient = aDataMng.getCurrentObjectiveGradient(tVectorIndex);

            aDataMng.elementWiseFunctions().updateObjectiveGlobalizationCoeff(tDeltaControl,
                                                                              tCurrentSigma,
                                                                              tCurrentGradient,
                                                                              *mControlWorkOne,
                                                                              *mControlWorkTwo);

            tStorageOne[tVectorIndex] = mControlReductionOperations->sum(mControlWorkOne.operator*());
            tStorageTwo[tVectorIndex] = mControlReductionOperations->sum(mControlWorkTwo.operator*());
        }

        const ScalarType tInitialValue = 0;
        ScalarType tFunctionEvaluationW = std::accumulate(tStorageOne.begin(), tStorageOne.end(), tInitialValue);
        tFunctionEvaluationW = static_cast<ScalarType>(0.5) * tFunctionEvaluationW;
        ScalarType tFunctionEvaluationV = std::accumulate(tStorageTwo.begin(), tStorageTwo.end(), tInitialValue);
        const ScalarType tCurrentObjectiveValue = aDataMng.getCurrentObjectiveFunctionValue();
        tFunctionEvaluationV = tCurrentObjectiveValue + tFunctionEvaluationV;

        ScalarType tGlobalizationFactor = aDataMng.getDualObjectiveGlobalizationFactor();
        const ScalarType tCcsaFunctionValue = tFunctionEvaluationV + (tGlobalizationFactor * tFunctionEvaluationW);

        const ScalarType tActualOverPredictedReduction = (mCurrentTrialObjectiveFunctionValue - tCcsaFunctionValue)
                / tFunctionEvaluationW;
        if(tActualOverPredictedReduction > static_cast<ScalarType>(0))
        {
            ScalarType tValueOne = static_cast<ScalarType>(10) * tGlobalizationFactor;
            ScalarType tValueTwo = static_cast<ScalarType>(1.1)
                    * (tGlobalizationFactor + tActualOverPredictedReduction);
            tGlobalizationFactor = std::min(tValueOne, tValueTwo);
        }

        aDataMng.setDualObjectiveGlobalizationFactor(tGlobalizationFactor);
    }
    void updateConstraintGlobalizationFactors(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        assert(aDataMng.getNumDualVectors() == static_cast<OrdinalType>(1));

        bool tUpdateConstraintGlobalizationFactors = false;
        const OrdinalType tDualVectorIndex = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tConstraintValues = aDataMng.getCurrentConstraintValues(tDualVectorIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tTrialConstraintValues = mTrialConstraintValues->operator[](tDualVectorIndex);
        const Plato::Vector<ScalarType, OrdinalType> & tGlobalizationFactors = aDataMng.getConstraintGlobalizationFactors(tDualVectorIndex);

        const OrdinalType tNumConstraints = aDataMng.getNumConstraints();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            mControlWorkOne->fill(static_cast<ScalarType>(0));
            mControlWorkTwo->fill(static_cast<ScalarType>(0));

            const OrdinalType tNumVectors = mTrialControl->getNumVectors();
            std::vector<ScalarType> tStorageOne(tNumVectors, 0);
            std::vector<ScalarType> tStorageTwo(tNumVectors, 0);
            for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
            {
                const Plato::Vector<ScalarType, OrdinalType> & tCurrentSigma = aDataMng.getCurrentSigma(tVectorIndex);
                const Plato::Vector<ScalarType, OrdinalType> & tDeltaControl = mDeltaControl->operator[](tVectorIndex);
                const Plato::Vector<ScalarType, OrdinalType> & tConstraintGradient =
                        aDataMng.getCurrentConstraintGradients(tConstraintIndex, tVectorIndex);
                assert(tConstraintGradient.size() == tDeltaControl.size());

                aDataMng.elementWiseFunctions().updateConstraintGlobalizationCoeff(tDeltaControl,
                                                                                   tCurrentSigma,
                                                                                   tConstraintGradient,
                                                                                   *mControlWorkOne,
                                                                                   *mControlWorkTwo);

                tStorageOne[tVectorIndex] = mControlReductionOperations->sum(mControlWorkOne.operator*());
                tStorageTwo[tVectorIndex] = mControlReductionOperations->sum(mControlWorkTwo.operator*());
            }

            const ScalarType tInitialValue = 0;
            ScalarType tFunctionEvaluationW = std::accumulate(tStorageOne.begin(), tStorageOne.end(), tInitialValue);
            tFunctionEvaluationW = static_cast<ScalarType>(0.5) * tFunctionEvaluationW;

            ScalarType tFunctionEvaluationV = std::accumulate(tStorageTwo.begin(), tStorageTwo.end(), tInitialValue);
            tFunctionEvaluationV = tFunctionEvaluationV + tConstraintValues[tConstraintIndex];

            ScalarType tCcsaFunctionValue =
                    tFunctionEvaluationV + (tGlobalizationFactors[tConstraintIndex] * tFunctionEvaluationW);
            ScalarType tActualOverPredictedReduction =
                    (tTrialConstraintValues[tConstraintIndex] - tCcsaFunctionValue) / tFunctionEvaluationW;

            if(tActualOverPredictedReduction > static_cast<ScalarType>(0))
            {
                tUpdateConstraintGlobalizationFactors = true;
                ScalarType tValueOne = static_cast<ScalarType>(10) * tGlobalizationFactors[tConstraintIndex];
                ScalarType tValueTwo = static_cast<ScalarType>(1.1)
                        * (tGlobalizationFactors[tConstraintIndex] + tActualOverPredictedReduction);
                mDualWorkOne->operator[](tConstraintIndex) = std::min(tValueOne, tValueTwo);
            }
        }
        if(tUpdateConstraintGlobalizationFactors == true)
        {
            aDataMng.setConstraintGlobalizationFactors(tDualVectorIndex, mDualWorkOne.operator*());
        }
    }
    bool checkStoppingCriteria(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        aDataMng.computeKarushKuhnTuckerConditionsInexactness(mTrialControl.operator*(), mTrialDual.operator*());
        const ScalarType t_KKT_ConditionsInexactness = aDataMng.getKarushKuhnTuckerConditionsInexactness();

        const ScalarType tObjectiveStagnation =
                std::abs(mCurrentTrialObjectiveFunctionValue - mPreviousTrialObjectiveFunctionValue);

        Plato::update(static_cast<ScalarType>(1),
                      mTrialControl.operator*(),
                      static_cast<ScalarType>(0),
                      mControlWork.operator*());
        Plato::update(static_cast<ScalarType>(-1),
                      mPreviousTrialControl.operator*(),
                      static_cast<ScalarType>(1),
                      mControlWork.operator*());

        const OrdinalType tNumControlVectors = mControlWork->getNumVectors();
        std::vector<ScalarType> tStorage(tNumControlVectors);
        for(OrdinalType tIndex = 0; tIndex < tNumControlVectors; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tVector = mControlWork->operator[](tIndex);
            tStorage[tIndex] = mControlReductionOperations->max(tVector);
            tStorage[tIndex] = std::abs(tStorage[tIndex]);
        }
        ScalarType tControlStagnation = *std::max_element(tStorage.begin(), tStorage.end());

        const OrdinalType tNumIterationsDone = this->getNumIterationsDone();

        bool tStop = false;
        if(tNumIterationsDone >= this->getMaxNumIterations())
        {
            tStop = true;
            this->setStoppingCriterion(Plato::ccsa::MAX_NUMBER_ITERATIONS);
        }
        else if(t_KKT_ConditionsInexactness < this->getKarushKuhnTuckerConditionsTolerance())
        {
            tStop = true;
            this->setStoppingCriterion(Plato::ccsa::KKT_CONDITIONS_TOLERANCE);
        }
        else if(tObjectiveStagnation < this->getControlStagnationTolerance())
        {
            tStop = true;
            this->setStoppingCriterion(Plato::ccsa::OBJECTIVE_STAGNATION);
        }
        else if(tControlStagnation < this->getControlStagnationTolerance())
        {
            tStop = true;
            this->setStoppingCriterion(Plato::ccsa::CONTROL_STAGNATION);
        }

        return (tStop);
    }
    void checkGlobalizationFactors(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        ScalarType tObjectiveGlobalizationFactor = aDataMng.getDualObjectiveGlobalizationFactor();
        ScalarType tValue = static_cast<ScalarType>(0.1) * tObjectiveGlobalizationFactor;
        tObjectiveGlobalizationFactor = std::max(tValue, mMinObjectiveGlobalizationFactor);
        aDataMng.setDualObjectiveGlobalizationFactor(tObjectiveGlobalizationFactor);

        const Plato::MultiVector<ScalarType, OrdinalType> & tConstraintGlobalizationFactors =
                aDataMng.getConstraintGlobalizationFactors();
        const OrdinalType tNumDualVectors = tConstraintGlobalizationFactors.getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumDualVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyConstraintGlobalizationFactors =
                    tConstraintGlobalizationFactors[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tMyMinConstraintGlobalizationFactors =
                    mMinConstraintGlobalizationFactors->operator[](tVectorIndex);
            const OrdinalType tNumConstraints = tMyConstraintGlobalizationFactors.size();
            for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
            {
                tValue = static_cast<ScalarType>(0.1) * tMyConstraintGlobalizationFactors[tIndex];
                mDualWorkOne->operator[](tIndex) = std::max(tValue, tMyMinConstraintGlobalizationFactors[tIndex]);
            }
            aDataMng.setConstraintGlobalizationFactors(tVectorIndex, mDualWorkOne.operator*());
        }
    }

private:
    OrdinalType mMaxNumIterations;
    OrdinalType mNumIterationsDone;

    ScalarType mControlStagnationTolerance;
    ScalarType mObjectiveStagnationTolerance;
    ScalarType mMinObjectiveGlobalizationFactor;
    ScalarType mCurrentTrialObjectiveFunctionValue;
    ScalarType mPreviousTrialObjectiveFunctionValue;
    ScalarType mKarushKuhnTuckerConditionsTolerance;

    Plato::ccsa::stop_t mStoppingCriterion;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mDualWorkOne;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWorkOne;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWorkTwo;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialDual;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mActiveSet;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlWork;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mInactiveSet;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDeltaControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousTrialControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialConstraintValues;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mMinConstraintGlobalizationFactors;

    std::shared_ptr<Plato::DualProblemSolver<ScalarType, OrdinalType>> mDualSolver;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductionOperations;

private:
    GloballyConvergentMethodMovingAsymptotes(const Plato::GloballyConvergentMethodMovingAsymptotes<ScalarType, OrdinalType> & aRhs);
    Plato::GloballyConvergentMethodMovingAsymptotes<ScalarType, OrdinalType> & operator=(const Plato::GloballyConvergentMethodMovingAsymptotes<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_GLOBALLYCONVERGENTMETHODMOVINGASYMPTOTES_HPP_ */
