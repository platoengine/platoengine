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
 * Plato_KelleySachsBoundConstrained.hpp
 *
 *  Created on: Dec 20, 2017
 */

#ifndef PLATO_KELLEYSACHSBOUNDCONSTRAINED_HPP_
#define PLATO_KELLEYSACHSBOUNDCONSTRAINED_HPP_

#include <cmath>
#include <memory>
#include <sstream>
#include <cassert>
#include <iostream>
#include <algorithm>

#include "Plato_Types.hpp"
#include "Plato_ErrorChecks.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_OptimizersIO.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_KelleySachsStepMng.hpp"
#include "Plato_KelleySachsAlgorithm.hpp"
#include "Plato_ProjectedSteihaugTointPcg.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_ReducedSpaceTrustRegionStageMng.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class KelleySachsBoundConstrained : public Plato::KelleySachsAlgorithm<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aDataFactory enables construction of core vector and multi-vector data structures
     * @param [in] aDataMng data manager for Kelley-Sachs algorithms
     * @param [in] aStageMng interface to objective and constraint evaluations
    **********************************************************************************/
    KelleySachsBoundConstrained(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory,
                                const std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> & aDataMng,
                                const std::shared_ptr<Plato::ReducedSpaceTrustRegionStageMng<ScalarType, OrdinalType>> & aStageMng) :
            Plato::KelleySachsAlgorithm<ScalarType, OrdinalType>(*aDataFactory),
            mPrintDiagnostics(false),
            mOptimalityTolerance(1e-5),
            mOutputData(),
            mGradient(aDataFactory->control().create()),
            mControlWorkVector(aDataFactory->control().create()),
            mStepMng(std::make_shared<Plato::KelleySachsStepMng<ScalarType, OrdinalType>>(*aDataFactory)),
            mSolver(std::make_shared<Plato::ProjectedSteihaugTointPcg<ScalarType, OrdinalType>>(*aDataFactory)),
            mDataMng(aDataMng),
            mStageMng(aStageMng)
    {
        this->initialize();
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~KelleySachsBoundConstrained()
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
     * @brief Set optimality tolerance, i.e. \f$\Vert\nabla{F}(x)\Vert\leq\epsilon\f$
     * @param [in] aInput optimality tolerance
    **********************************************************************************/
    void setOptimalityTolerance(const ScalarType & aInput)
    {
        mOptimalityTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of trust region subproblem iterations
     * @return [in] aInput maximum number of trust region subproblem iterations
    **********************************************************************************/
    void setMaxNumTrustRegionSubProblemIterations(const OrdinalType & aInput)
    {
        mStepMng->setMaxNumTrustRegionSubProblemIterations(aInput);
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
     * @brief Set trust region radius expansion factor
     * @param [in] aInput trust region radius expansion factor
    **********************************************************************************/
    void setTrustRegionExpansion(const ScalarType & aInput)
    {
        mStepMng->setTrustRegionExpansion(aInput);
    }

    /******************************************************************************//**
     * @brief Set trust region radius contraction factor
     * @param [in] aInput trust region radius contraction factor
    **********************************************************************************/
    void setTrustRegionContraction(const ScalarType & aInput)
    {
        mStepMng->setTrustRegionContraction(aInput);
    }

    /******************************************************************************//**
     * @brief Set actual over predicted reduction middle bound
     * @param [in] aInput actual over predicted reduction middle bound
    **********************************************************************************/
    void setActualOverPredictedReductionMidBound(const ScalarType & aInput)
    {
        mStepMng->setActualOverPredictedReductionMidBound(aInput);
    }

    /******************************************************************************//**
     * @brief Set actual over predicted reduction lower bound
     * @param [in] aInput actual over predicted reduction lower bound
    **********************************************************************************/
    void setActualOverPredictedReductionLowerBound(const ScalarType & aInput)
    {
        mStepMng->setActualOverPredictedReductionLowerBound(aInput);
    }

    /******************************************************************************//**
     * @brief Set actual over predicted reduction upper bound
     * @param [in] aInput actual over predicted reduction upper bound
    **********************************************************************************/
    void setActualOverPredictedReductionUpperBound(const ScalarType & aInput)
    {
        mStepMng->setActualOverPredictedReductionUpperBound(aInput);
    }

    /******************************************************************************//**
     * @brief Return reference to data manager
     * @return trust region algorithm's data manager
    **********************************************************************************/
    const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & getDataMng() const
    {
        return (*mDataMng);
    }

    /******************************************************************************//**
     * @brief Return reference to trust region step algorithm manager
     * @return trust region step algorithm manager
    **********************************************************************************/
    const Plato::KelleySachsStepMng<ScalarType, OrdinalType> & getStepMng() const
    {
        return (*mStepMng);
    }

    /******************************************************************************//**
     * @brief Solve bound constrained optimization problem
    **********************************************************************************/
    void solve()
    {
        this->openOutputFile();
        this->checkInitialGuess(*mDataMng);
        this->setInitialStateData();
        mStageMng->updateOptimizationData(*mDataMng);
        this->outputDiagnostics();

        OrdinalType tIteration = 0;
        while(1)
        {
            tIteration++;
            this->setNumIterationsDone(tIteration);
            mDataMng->cacheCurrentStageData();
            // Compute adaptive constants to ensure superlinear convergence
            this->computeAdaptiveConstant(*mDataMng, *mStepMng);
            // Solve trust region subproblem
            mStepMng->solveSubProblem(*mDataMng, *mStageMng, *mSolver);
            // Update mid objective, control, and gradient information if necessary
            this->update();
            // Update stage manager data
            mStageMng->updateOptimizationData(*mDataMng);
            // output diagnostics
            this->outputDiagnostics();
            if(this->checkStoppingCriteria() == true)
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
                mOutputStream.open("plato_ksbc_algorithm_diagnostics.txt");
                Plato::print_ksbc_diagnostics_header(mOutputData, mOutputStream, mPrintDiagnostics);
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
     * @brief Print diagnostics for Kelley-Sachs bound constrained algorithm
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
            mOutputData.mObjFuncValue = mDataMng->getCurrentObjectiveFunctionValue();
            mOutputData.mObjFuncCount = mDataMng->getNumObjectiveFunctionEvaluations();
            mOutputData.mNumLineSearchIter = this->getNumLineSearchItrDone();
            mOutputData.mNumTrustRegionIter = mStepMng->getNumTrustRegionSubProblemItrDone();

            mOutputData.mActualRed = mStepMng->getActualReduction();
            mOutputData.mAredOverPred = mStepMng->getActualOverPredictedReduction();
            mOutputData.mNormObjFuncGrad = mDataMng->getNormProjectedGradient();
            mOutputData.mTrustRegionRadius = mStepMng->getTrustRegionRadius();
            mOutputData.mStationarityMeasure = mDataMng->getStationarityMeasure();
            mOutputData.mControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
            mOutputData.mObjectiveStagnationMeasure = mDataMng->getObjectiveStagnationMeasure();

            Plato::print_ksbc_diagnostics(mOutputData, mOutputStream, mPrintDiagnostics);
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
     * @brief Compute initial objective value and gradient.
    **********************************************************************************/
    void setInitialStateData()
    {
        this->initializeMaxTrustRegionRadius(*mStepMng);
        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = mDataMng->getCurrentControl();
        ScalarType tTolerance = mStepMng->getObjectiveInexactnessTolerance();
        ScalarType tCurrentObjFuncValue = mStageMng->evaluateObjective(tCurrentControl, tTolerance);
        mDataMng->setCurrentObjectiveFunctionValue(tCurrentObjFuncValue);
        mStageMng->cacheData();
        mStageMng->computeGradient(tCurrentControl, *mGradient);
        mDataMng->setCurrentGradient(*mGradient);
        mDataMng->computeNormProjectedGradient();
        this->initializeTrustRegionRadius(*mDataMng, *mStepMng);
        mDataMng->computeStationarityMeasure();
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
        catch(const std::invalid_argument & tError)
        {
            std::cout << tError.what() << std::flush;
            throw tError;
        }
    }

    /******************************************************************************//**
     * @brief Update trial control if current mid-point actual reduction tolerance is violated.
    **********************************************************************************/
    void update()
    {
        // Perform application-based continuation
        const bool tContinuationDone = this->performContinuation(*mStepMng, *mStageMng);
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
        // compute gradient inexactness bound for next trust region sub-problem solve
        ScalarType tNormProjectedGradient = mDataMng->getNormProjectedGradient();
        mStepMng->updateGradientInexactnessTolerance(tNormProjectedGradient);
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
        // Compute gradient at new midpoint
        mStageMng->cacheData();
        const Plato::MultiVector<ScalarType, OrdinalType> & tMidControl = mStepMng->getMidPointControls();
        mStageMng->computeGradient(tMidControl, *mGradient /* mid point gradient */);

        bool tContinuationNotDone = aDidContinuation == false;
        bool tPostSmoothingActive = this->isPostSmoothingActive() == true;
        bool tNumTrustRegionIterLessThanMaxNumIter = mStepMng->getNumTrustRegionSubProblemItrDone()
                != mStepMng->getMaxNumTrustRegionSubProblemIterations();
        if(tNumTrustRegionIterLessThanMaxNumIter && tPostSmoothingActive && tContinuationNotDone)
        {
            tControlUpdated = this->updateControl(*mGradient /* mid point gradient */, *mStepMng, *mDataMng, *mStageMng);
        }
        return (tControlUpdated);
    }

    /******************************************************************************//**
     * @brief Check stopping criteria
     * @return stopping criteria flag (true = stop algorithm, false = continue to next iteration)
    **********************************************************************************/
    bool checkStoppingCriteria()
    {
        bool tStop = false;
        if(this->checkNaN() == true)
        {
            // Stop optimization algorithm: NaN criterion measure detected
            tStop = true;
            mDataMng->resetCurrentStageDataToPreviousStageData();
        }
        else
        {
            tStop = this->checkStoppingTolerance(*mDataMng, *mStepMng);
        }
        return (tStop);
    }

    /******************************************************************************//**
     * @brief Check for NaN values
     * @return NaN flag (true = NaN identified, false = NaN not identified)
    **********************************************************************************/
    bool checkNaN()
    {
        const ScalarType tStationarityMeasure = mDataMng->getStationarityMeasure();
        const ScalarType tOptimalityMeasure = mDataMng->getNormProjectedGradient();
        const ScalarType tControlStagnationMeasure = mDataMng->getControlStagnationMeasure();

        bool tNaN_ValueDetected = false;
        if(std::isfinite(tStationarityMeasure) == false)
        {
            tNaN_ValueDetected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_STATIONARITY_MEASURE);
        }
        else if(std::isfinite(tOptimalityMeasure) == false)
        {
            tNaN_ValueDetected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_OBJECTIVE_GRADIENT);
        }
        else if(std::isfinite(tControlStagnationMeasure) == false)
        {
            tNaN_ValueDetected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_CONTROL_STAGNATION);
        }

        return (tNaN_ValueDetected);
    }

private:
    bool mPrintDiagnostics;
    std::ofstream mOutputStream;
    ScalarType mOptimalityTolerance;

    Plato::OutputDataKSBC<ScalarType, OrdinalType> mOutputData;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mControlWorkVector;

    std::shared_ptr<Plato::KelleySachsStepMng<ScalarType, OrdinalType>> mStepMng;
    std::shared_ptr<Plato::ProjectedSteihaugTointPcg<ScalarType,OrdinalType>> mSolver;
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType,OrdinalType>> mDataMng;
    std::shared_ptr<Plato::ReducedSpaceTrustRegionStageMng<ScalarType,OrdinalType>> mStageMng;

private:
    KelleySachsBoundConstrained(const Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType>&);
    Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType> & operator=(const Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType>&);
}; // class KelleySachsBoundConstrained

} // namespace Plato

#endif /* PLATO_KELLEYSACHSBOUNDCONSTRAINED_HPP_ */
