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
 * Plato_KelleySachsAugmentedLagrangian.hpp
 *
 *  Created on: Oct 21, 2017
 */

#ifndef PLATO_KELLEYSACHSAUGMENTEDLAGRANGIAN_HPP_
#define PLATO_KELLEYSACHSAUGMENTEDLAGRANGIAN_HPP_

#include <cmath>
#include <memory>
#include <sstream>
#include <cassert>
#include <iostream>
#include <algorithm>

#include "Plato_Types.hpp"
#include "Plato_ErrorChecks.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_OptimizersIO.hpp"
#include "Plato_KelleySachsStepMng.hpp"
#include "Plato_KelleySachsAlgorithm.hpp"
#include "Plato_ProjectedSteihaugTointPcg.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_AugmentedLagrangianStageMng.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class KelleySachsAugmentedLagrangian : public Plato::KelleySachsAlgorithm<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Default constructor
     * @param [in] aDataFactory manage allocation of linear algebra data structures
     * @param [in] aDataMng trust region augmented Lagrangian data base manager
     * @param [in] aStageMng manage evaluation of augmented Lagrangian criteria
    **********************************************************************************/
    KelleySachsAugmentedLagrangian(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory,
                                   const std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> & aDataMng,
                                   const std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>> & aStageMng) :
            Plato::KelleySachsAlgorithm<ScalarType, OrdinalType>(*aDataFactory),
            mPrintDiagnostics(false),
            mGammaConstant(1),
            mMinPenaltyParameter(1e-4),
            mOptimalityTolerance(1e-4),
            mFeasibilityTolerance(1e-4),
            mOutputData(),
            mGradient(aDataFactory->control().create()),
            mCurrentConstraintValues(aDataFactory->dual().create()),
            mStepMng(std::make_shared<Plato::KelleySachsStepMng<ScalarType, OrdinalType>>(*aDataFactory)),
            mSolver(std::make_shared<Plato::ProjectedSteihaugTointPcg<ScalarType, OrdinalType>>(*aDataFactory)),
            mDataMng(aDataMng),
            mStageMng(aStageMng),
            mVerbose(false)
    {
        this->initialize();
    }

    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    virtual ~KelleySachsAugmentedLagrangian()
    {
    }

    /******************************************************************************//**
     * @brief Enable output of diagnostics (i.e. optimization problem status)
    **********************************************************************************/
    void enableDiagnostics()
    {
        mPrintDiagnostics = true;
    }

    /******************************************************************************//**
     * @brief Set minimum Lagrange multipliers' penalty value
     * @param [in] aInput minimum Lagrange multipliers' penalty value
    **********************************************************************************/
    void setMinPenaltyParameter(const ScalarType & aInput)
    {
        mMinPenaltyParameter = aInput;
    }

    /******************************************************************************//**
     * @brief Set optimality stopping tolerance
     * @param [in] aInput optimality stopping tolerance
    **********************************************************************************/
    void setOptimalityTolerance(const ScalarType & aInput)
    {
        mOptimalityTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set feasibility stopping tolerance
     * @param [in] aInput feasibility stopping tolerance
    **********************************************************************************/
    void setFeasibilityTolerance(const ScalarType & aInput)
    {
        mFeasibilityTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set minimum trust region radius
     * @param [in] aInput minimum trust region radius
    **********************************************************************************/
    void setMinTrustRegionRadius(const ScalarType & aInput)
    {
        mStepMng->setMinTrustRegionRadius(aInput);
    }

    /******************************************************************************//**
     * @brief Set maximum trust region radius
     * @param [in] aInput maximum trust region radius
    **********************************************************************************/
    void setMaxTrustRegionRadius(const ScalarType & aInput)
    {
        mStepMng->setMaxTrustRegionRadius(aInput);
    }

    /******************************************************************************//**
     * @brief Set trust region expansion factor
     * @param [in] aInput trust region expansion factor
    **********************************************************************************/
    void setTrustRegionExpansion(const ScalarType & aInput)
    {
        assert(aInput > static_cast<ScalarType>(0));
        assert(aInput > static_cast<ScalarType>(1));
        mStepMng->setTrustRegionExpansion(aInput);
    }

    /******************************************************************************//**
     * @brief Set trust region contraction factor
     * @param [in] aInput trust region contraction factor
    **********************************************************************************/
    void setTrustRegionContraction(const ScalarType & aInput)
    {
        assert(aInput > static_cast<ScalarType>(0));
        assert(aInput < static_cast<ScalarType>(1));
        mStepMng->setTrustRegionContraction(aInput);
    }

    /******************************************************************************//**
     * @brief Set Lagrange multipliers' penalty contraction factor
     * @param [in] aInput Lagrange multipliers' penalty contraction factor
    **********************************************************************************/
    void setPenaltyParameterScaleFactor(const ScalarType & aInput)
    {
        mStageMng->setPenaltyParameterScaleFactor(aInput);
    }

    /******************************************************************************//**
     * @brief Set relative tolerance constant for norm of augmented Lagrangian gradient
     * @param [in] aInput relative tolerance constant for norm of augmented Lagrangian gradient
    **********************************************************************************/
    void setNormAugmentedLagrangianGradientRelativeToleranceConstant(const ScalarType & aInput)
    {
        mGammaConstant = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of trust region subproblem iterations
     * @param [in] aInput maximum number of trust region subproblem iterations
    **********************************************************************************/
    void setMaxNumTrustRegionSubProblemIterations(const OrdinalType & aInput)
    {
        mStepMng->setMaxNumTrustRegionSubProblemIterations(aInput);
    }

    /******************************************************************************//**
     * @brief Return reference to data manager
     * @return trust region algorithm's data manager
    **********************************************************************************/
    const Plato::TrustRegionAlgorithmDataMng<ScalarType,OrdinalType> & getDataMng() const
    {
        return (*mDataMng);
    }

    /******************************************************************************//**
     * @brief Return reference to trust region step manager
     * @return trust region step manager
    **********************************************************************************/
    const Plato::KelleySachsStepMng<ScalarType,OrdinalType> & getStepMng() const
    {
        return (*mStepMng);
    }

    /******************************************************************************//**
     * @brief Return reference to stage manager
     * @return trust region algorithm's stage manager
    **********************************************************************************/
    const Plato::AugmentedLagrangianStageMng<ScalarType,OrdinalType> & getStageMng() const
    {
        return (*mStageMng);
    }

    /******************************************************************************//**
     * @brief Solve constrained optimization problem
    **********************************************************************************/
    void solve()
    {
        this->openOutputFile();
        this->checkInitialGuess(*mDataMng);
        this->setInitialStateData();
        this->initializeTrustRegionRadius(*mDataMng, *mStepMng);
        this->outputDiagnostics();

        OrdinalType tIteration = 0;
        while(1)
        {
            tIteration++;
            this->setNumIterationsDone(tIteration);
            mDataMng->cacheCurrentStageData();
            mStageMng->cacheCurrentCriteriaValues();
            // Compute adaptive constants to ensure superlinear convergence
            this->computeAdaptiveConstant(*mDataMng, *mStepMng);
            // Solve trust region subproblem
            mStepMng->solveSubProblem(*mDataMng, *mStageMng, *mSolver);
            // Update mid objective, control, and gradient information if necessary
            this->update();
            // Update stage manager data
            mStageMng->updateOptimizationData(mDataMng.operator*());
            // check penalty parameter, update if necessary
            bool tStoppingCriterionMet = this->checkPenaltyParameter();
            // output diagnostics
            this->outputDiagnostics();
            if(tStoppingCriterionMet == true)
            {
                this->outputMyStoppingCriterion();
                this->closeOutputFile();
                break;
            }
        }
    }

private:
    /******************************************************************************//**
     * @brief Open output file (i.e. diagnostics file)
    **********************************************************************************/
    void openOutputFile()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                const OrdinalType tNumConstraints = mStageMng->getNumConstraints();
                mOutputData.mConstraintValues.clear();
                mOutputData.mConstraintValues.resize(tNumConstraints);
                mOutputStream.open("plato_ksal_algorithm_diagnostics.txt");
                Plato::print_ksal_diagnostics_header(mOutputData, mOutputStream, mPrintDiagnostics);
            }
        }
    }

    /******************************************************************************//**
     * @brief Close output file (i.e. diagnostics file)
    **********************************************************************************/
    void closeOutputFile()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                mOutputStream.close();
            }
        }
    }

    /******************************************************************************//**
     * @brief Print stopping criterion into diagnostics file.
    **********************************************************************************/
    void outputMyStoppingCriterion()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                std::string tReason;
                Plato::algorithm::stop_t tStopCriterion = this->getStoppingCriterion();
                Plato::get_stop_criterion(tStopCriterion, tReason);
                mOutputStream << tReason.c_str();
            }
        }
    }

    /******************************************************************************//**
     * @brief Print diagnostics for Kelley-Sachs augmented Lagrangian algorithm
    **********************************************************************************/
    void outputDiagnostics()
    {
        if(mPrintDiagnostics == false)
        {
            return;
        }

        const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
        if(tMyCommWrapper.myProcID() == 0)
        {
            mOutputData.mNumIter = this->getNumIterationsDone();
            mOutputData.mNumIterPCG = mSolver->getNumIterationsDone();
            mOutputData.mNumLineSearchIter = this->getNumLineSearchItrDone();
            mOutputData.mObjFuncCount = mDataMng->getNumObjectiveFunctionEvaluations();
            mOutputData.mAugLagFuncValue = mDataMng->getCurrentObjectiveFunctionValue();
            mOutputData.mNumTrustRegionIter = mStepMng->getNumTrustRegionSubProblemItrDone();

            mOutputData.mPenalty = mStageMng->getPenaltyParameter();
            mOutputData.mActualRed = mStepMng->getActualReduction();
            mOutputData.mAredOverPred = mStepMng->getActualOverPredictedReduction();
            mOutputData.mTrustRegionRadius = mStepMng->getTrustRegionRadius();
            mOutputData.mStationarityMeasure = mDataMng->getStationarityMeasure();
            mOutputData.mNormAugLagFuncGrad = mDataMng->getNormProjectedGradient();
            mOutputData.mControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
            mOutputData.mObjectiveStagnationMeasure = mDataMng->getObjectiveStagnationMeasure();

            mOutputData.mObjFuncValue = mStageMng->getCurrentObjectiveFunctionValue();
            mOutputData.mNormObjFuncGrad = mStageMng->getNormObjectiveFunctionGradient();

            const OrdinalType tCONSTRAINT_VECTOR_INDEX = 0;
            const OrdinalType tNumConstraints = mStageMng->getNumConstraints();
            for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
            {
                mOutputData.mConstraintValues[tIndex] =
                        mStageMng->getCurrentConstraintValues(tCONSTRAINT_VECTOR_INDEX, tIndex);
            }

            Plato::print_ksal_diagnostics(mOutputData, mOutputStream, mPrintDiagnostics);
        }
    }

    /******************************************************************************//**
     * @brief Check and initialize control lower and upper bounds
    **********************************************************************************/
    void initialize()
    {
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

    /******************************************************************************//**
     * @brief Compute initial objective value and gradient.
    **********************************************************************************/
    void setInitialStateData()
    {
        this->initializeMaxTrustRegionRadius(*mStepMng);
        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = mDataMng->getCurrentControl();
        ScalarType tCurrentAugLagFuncValue = mStageMng->evaluateAugmentedLagrangian(tCurrentControl);
        mDataMng->setCurrentObjectiveFunctionValue(tCurrentAugLagFuncValue);

        mStageMng->cacheData();
        mStageMng->computeGradient(tCurrentControl, *mGradient);
        mDataMng->setCurrentGradient(*mGradient);
        mDataMng->computeNormProjectedGradient();
        mDataMng->computeStationarityMeasure();
    }

    /******************************************************************************//**
     * @brief Update trial control if current mid-point actual reduction tolerance is violated.
    **********************************************************************************/
    void update()
    {
        // Perform application-based continuation
        const bool tContinuationDone = this->performContinuation(*mStepMng, *mStageMng);
        // Check mid-point solution
        this->checkMidPointSolution();
        // Apply post smoothing operation
        const bool tControlUpdated = this->applyPostSmoothing(tContinuationDone);
        // Update current state
        this->updateCurrentState(tControlUpdated);
        // Compute stationarity measure
        mDataMng->computeStationarityMeasure();
        // Compute norm of projected gradient
        mDataMng->computeNormProjectedGradient();
        // Compute control stagnation measure
        mDataMng->computeControlStagnationMeasure();
        // Compute objective stagnation measure
        mDataMng->computeObjectiveStagnationMeasure();
        // compute gradient inexactness bound
        ScalarType tNormProjectedGradient = mDataMng->getNormProjectedGradient();
        mStepMng->updateGradientInexactnessTolerance(tNormProjectedGradient);
    }

    /******************************************************************************//**
     * @brief Check mid-point solution, avoid evaluation of feasible constraints.
    **********************************************************************************/
    void checkMidPointSolution()
    {
        // Update objectives and inequality constraints
        mStageMng->updateCurrentCriteriaValues();
        // Compute mid-objective function. Feasible point detected, do not include feasible constraints.
        const ScalarType tAugLagFuncValue = mStageMng->getCurrentAugmenteLagrangianFunctionValue();
        if(tAugLagFuncValue < static_cast<ScalarType>(0))
        {
            this->recomputeMidPointObjFunc();
            mStageMng->updateCurrentCriteriaValues();
        }
        // Compute gradient at new midpoint
        mStageMng->cacheData();
        const Plato::MultiVector<ScalarType, OrdinalType> & tMidControl = mStepMng->getMidPointControls();
        mStageMng->computeGradient(tMidControl, *mGradient /* mid point gradient */);
    }

    /******************************************************************************//**
     * @brief Update current criteria values, controls and gradient
     * @param [in] aSmoothingDone: true = post-smoothing done; false = post-smoothing was not done
    **********************************************************************************/
    void updateCurrentState(const bool & aPostSmoothingDone)
    {
        if(aPostSmoothingDone == true)
        {
            // Update current gradient since control was updated
            mStageMng->cacheData();
            mStageMng->updateCurrentCriteriaValues();
            const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = mDataMng->getCurrentControl();
            mStageMng->computeGradient(tCurrentControl, *mGradient /* new/current gradient */);
            mDataMng->setCurrentGradient(*mGradient);
        }
        else
        {
            this->acceptMidPoint(*mGradient /* mid point gradient */, *mStepMng, *mDataMng);
        }
    }

    /******************************************************************************//**
     * @brief Apply post trust region subproblem smoothing step
     * @param [in] aDidContinuation flag: true = continuation done, false = continuation not done
     * @return Flag indicating if mid-point control was updated (updated = true, not updated = false)
    **********************************************************************************/
    bool applyPostSmoothing(const bool & aDidContinuation)
    {
        bool tControlUpdated = false;

        bool tContinuationNotDone = aDidContinuation == false;
        bool tPostSmoothingActive = this->isPostSmoothingActive() == true;
        bool tNumTrustRegionIterLessThanMaxNumIter = mStepMng->getNumTrustRegionSubProblemItrDone()
                != mStepMng->getMaxNumTrustRegionSubProblemIterations();
        bool tMaxNumTrustRegionIterDone = tNumTrustRegionIterLessThanMaxNumIter == false;
        bool tMidObjFuncValGreaterOldObjFuncVal =
                mStepMng->getMidPointObjectiveFunctionValue() > mDataMng->getPreviousObjectiveFunctionValue();
        if(tNumTrustRegionIterLessThanMaxNumIter && tPostSmoothingActive && tContinuationNotDone)
        {
            tControlUpdated = this->updateControl(*mGradient /* mid point gradient */, *mStepMng, *mDataMng, *mStageMng);
        }
        else if(tMaxNumTrustRegionIterDone  && tMidObjFuncValGreaterOldObjFuncVal && tPostSmoothingActive && tContinuationNotDone)
        {
            tControlUpdated = this->updateControl(*mGradient /* mid point gradient */, *mStepMng, *mDataMng, *mStageMng);
        }

        return (tControlUpdated);
    }

    /******************************************************************************//**
     * @brief Recompute mid-objective function value since negative value was detected.
    **********************************************************************************/
    void recomputeMidPointObjFunc()
    {
        const Plato::MultiVector<ScalarType, OrdinalType> & tMidControl = mStepMng->getMidPointControls();
        const ScalarType tTolerance = mStepMng->getObjectiveInexactnessTolerance();
        const ScalarType tMidObjFuncValue = mStageMng->evaluateObjective(tMidControl, tTolerance);
        mStepMng->setMidPointObjectiveFunctionValue(tMidObjFuncValue);
    }

    bool checkPenaltyParameter()
    {
        bool tStop = false;
        mStageMng->getCurrentConstraintValues(*mCurrentConstraintValues);
        const ScalarType tNormConstraintValues = Plato::norm(*mCurrentConstraintValues);
        const ScalarType tDynamicFeasibilityTolerance = mStageMng->getDynamicFeasibilityTolerance();
        const ScalarType tMyFeasibilityTolerance = std::max(mFeasibilityTolerance, tDynamicFeasibilityTolerance);
        if(tNormConstraintValues <= tMyFeasibilityTolerance)
        {

            if(tNormConstraintValues <= mFeasibilityTolerance)
            {
                tStop = this->checkStoppingCriteria();
                return (tStop);
            }
            else
            {
                mStageMng->updateLagrangeMultipliers();
            }
        }
        else
        {
            mStageMng->updatePenaltyParameter();
            const ScalarType tCurrentPenaltyParameter = mStageMng->getPenaltyParameter();
            if(tCurrentPenaltyParameter < mMinPenaltyParameter)
            {
                this->setStoppingCriterion(Plato::algorithm::stop_t::SMALL_PENALTY_PARAMETER);
                tStop = true;
                return (tStop);
            }
        }

        tStop = this->checkStoppingCriteria();
        return (tStop);
    }

    bool checkStoppingCriteria()
    {
        bool tStop = false;
        if(this->checkNaN() == true)
        {
            // Stop optimization algorithm: NaN number detected
            tStop = true;
            mDataMng->resetCurrentStageDataToPreviousStageData();
        }
        else
        {
            tStop = this->checkStoppingTolerance(*mDataMng, *mStepMng);
        }

        return (tStop);
    }

    bool checkNaN()
    {
        const ScalarType tActualReduction = mStepMng->getActualReduction();
        const ScalarType tStagnationMeasure = mStageMng->getStagnationMeasure();
        const ScalarType tStationarityMeasure = mDataMng->getStationarityMeasure();
        const ScalarType tControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
        const ScalarType tNormProjectedAugmentedLagrangianGradient = mDataMng->getNormProjectedGradient();

        bool tNaN_Detected = false;
        if(std::isfinite(tStationarityMeasure) == false)
        {
            tNaN_Detected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_STATIONARITY_MEASURE);
        }
        else if(std::isfinite(tNormProjectedAugmentedLagrangianGradient) == false)
        {
            tNaN_Detected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_NORM_GRADIENT);
        }
        else if(std::isfinite(tActualReduction) == false)
        {
            tNaN_Detected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_ACTUAL_REDUCTION);
        }
        else if(std::isfinite(tStagnationMeasure) == false)
        {
            tNaN_Detected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_OBJECTIVE_STAGNATION);
        }
        else if(std::isfinite(tControlStagnationMeasure) == false)
        {
            tNaN_Detected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_CONTROL_STAGNATION);
        }

        return (tNaN_Detected);
    }

private:
    bool mPrintDiagnostics;
    std::ofstream mOutputStream;

    ScalarType mGammaConstant;
    ScalarType mMinPenaltyParameter;
    ScalarType mOptimalityTolerance;
    ScalarType mFeasibilityTolerance;

    Plato::OutputDataKSAL<ScalarType, OrdinalType> mOutputData;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentConstraintValues;

    std::shared_ptr<Plato::KelleySachsStepMng<ScalarType, OrdinalType>> mStepMng;
    std::shared_ptr<Plato::ProjectedSteihaugTointPcg<ScalarType,OrdinalType>> mSolver;
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType,OrdinalType>> mDataMng;
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType,OrdinalType>> mStageMng;

    bool mVerbose;

private:
    KelleySachsAugmentedLagrangian(const Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> & aRhs);
    Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> & operator=(const Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_KELLEYSACHSAUGMENTEDLAGRANGIAN_HPP_ */
