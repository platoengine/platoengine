/*
 * Plato_ConservativeConvexSeparableAppxAlgorithm.hpp
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

#ifndef PLATO_CONSERVATIVECONVEXSEPARABLEAPPXALGORITHM_HPP_
#define PLATO_CONSERVATIVECONVEXSEPARABLEAPPXALGORITHM_HPP_

#include <memory>
#include <cassert>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>

#include "Plato_Vector.hpp"
#include "Plato_ErrorChecks.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_OptimizersIO.hpp"
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
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aStageMng stage manager for Conservative Convex Separable Approximation (CCSA) algorithms
     * @param [in] aDataMng data manager for CCSA algorithms
     * @param [in] aSubProblem CCSA subproblem (e.g. MMA, GCMMA)
    **********************************************************************************/
    ConservativeConvexSeparableAppxAlgorithm(const std::shared_ptr<Plato::ConservativeConvexSeparableAppxStageMng<ScalarType, OrdinalType>> & aStageMng,
                                             const std::shared_ptr< Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType>> & aDataMng,
                                             const std::shared_ptr<Plato::ConservativeConvexSeparableApproximation<ScalarType, OrdinalType>> & aSubProblem) :
            mPrintDiagnostics(false),
            mOutputStream(),
            mMaxNumOuterIter(500),
            mNumOuterIterDone(0),
            mStationarityTolerance(1e-4),
            mControlStagnationTolerance(1e-8),
            mObjectiveStagnationTolerance(1e-6),
            mMovingAsymptoteExpansionFactor(1.2),
            mMovingAsymptoteContractionFactor(0.4),
            mInitialMovingAsymptoteScaleFactor(0.5),
            mKarushKuhnTuckerConditionsTolerance(1e-5),
            mMovingAsymptoteUpperBoundScaleFactor(10),
            mMovingAsymptoteLowerBoundScaleFactor(0.01),
            mStopCriterion(Plato::ccsa::stop_t::NOT_CONVERGED),
            mDualWork(),
            mControlWork(),
            mPreviousSigma(),
            mAntepenultimateControl(),
            mWorkMultiVectorList(),
            mOutputData(),
            mDataMng(aDataMng),
            mStageMng(aStageMng),
            mSubProblem(aSubProblem)
    {
        this->initialize(*aDataMng);
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~ConservativeConvexSeparableAppxAlgorithm()
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
     * @brief Return maximum number of outer optimization iterations
     * @return maximum number of outer optimization iterations
    **********************************************************************************/
    OrdinalType getMaxNumIterations() const
    {
        return (mMaxNumOuterIter);
    }

    /******************************************************************************//**
     * @brief Set maximum number of outer optimization iterations
     * @param [in] aInput maximum number of outer optimization iterations
    **********************************************************************************/
    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumOuterIter = aInput;
    }

    /******************************************************************************//**
     * @brief Return number of outer optimization iterations done
     * @return number of outer optimization iterations done
    **********************************************************************************/
    OrdinalType getNumIterationsDone() const
    {
        return (mNumOuterIterDone);
    }

    /******************************************************************************//**
     * @brief Return control stagnation tolerance
     * @return control stagnation tolerance
    **********************************************************************************/
    ScalarType getControlStagnationTolerance() const
    {
        return (mControlStagnationTolerance);
    }

    /******************************************************************************//**
     * @brief Set control stagnation tolerance
     * @param [in] aInput control stagnation tolerance
    **********************************************************************************/
    void setControlStagnationTolerance(const ScalarType & aInput)
    {
        mControlStagnationTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Return stationarity tolerance (i.e. norm of the trial step)
     * @return stationarity tolerance
    **********************************************************************************/
    ScalarType getStationarityTolerance() const
    {
        return (mStationarityTolerance);
    }

    /******************************************************************************//**
     * @brief Set stationarity tolerance (i.e. norm of the trial step)
     * @param [in] aInput stationarity tolerance
    **********************************************************************************/
    void setStationarityTolerance(const ScalarType & aInput)
    {
        mStationarityTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Return objective function stagnation tolerance
     * @return objective function stagnation tolerance
    **********************************************************************************/
    ScalarType getObjectiveStagnationTolerance() const
    {
        return (mObjectiveStagnationTolerance);
    }

    /******************************************************************************//**
     * @brief Set objective function stagnation tolerance
     * @param [in] aInput objective function stagnation tolerance
    **********************************************************************************/
    void setObjectiveStagnationTolerance(const ScalarType & aInput)
    {
        mObjectiveStagnationTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Return moving asymptotes' expansion factor
     * @return moving asymptotes expansion factor
    **********************************************************************************/
    ScalarType getMovingAsymptoteExpansionFactor() const
    {
        return (mMovingAsymptoteExpansionFactor);
    }

    /******************************************************************************//**
     * @brief Set moving asymptotes' expansion factor
     * @param [in] aInput moving asymptotes expansion factor
    **********************************************************************************/
    void setMovingAsymptoteExpansionFactor(const ScalarType & aInput)
    {
        mMovingAsymptoteExpansionFactor = aInput;
    }

    /******************************************************************************//**
     * @brief Return moving asymptotes' contraction factor
     * @return moving asymptotes contraction factor
    **********************************************************************************/
    ScalarType getMovingAsymptoteContractionFactor() const
    {
        return (mMovingAsymptoteContractionFactor);
    }

    /******************************************************************************//**
     * @brief Set moving asymptotes' contraction factor
     * @param [in] aInput moving asymptotes contraction factor
    **********************************************************************************/
    void setMovingAsymptoteContractionFactor(const ScalarType & aInput)
    {
        mMovingAsymptoteContractionFactor = aInput;
    }

    /******************************************************************************//**
     * @brief Return initial moving asymptotes' scale factor
     * @return moving initial asymptotes scale factor
    **********************************************************************************/
    ScalarType getInitialMovingAsymptoteScaleFactor() const
    {
        return (mInitialMovingAsymptoteScaleFactor);
    }

    /******************************************************************************//**
     * @brief Set initial moving asymptotes' scale factor
     * @param [in] aInput initial moving asymptotes scale factor
    **********************************************************************************/
    void setInitialMovingAsymptoteScaleFactor(const ScalarType & aInput)
    {
        mInitialMovingAsymptoteScaleFactor = aInput;
    }

    /******************************************************************************//**
     * @brief Return Karush-Kuhn-Tucker (KKT) inexactness tolerance
     * @return Karush-Kuhn-Tucker inexactness tolerance
    **********************************************************************************/
    ScalarType getKarushKuhnTuckerConditionsTolerance() const
    {
        return (mKarushKuhnTuckerConditionsTolerance);
    }
    /******************************************************************************//**
     * @brief Set Karush-Kuhn-Tucker (KKT) inexactness tolerance
     * @param [in] aInput Karush-Kuhn-Tucker inexactness tolerance
    **********************************************************************************/
    void setKarushKuhnTuckerConditionsTolerance(const ScalarType & aInput)
    {
        mKarushKuhnTuckerConditionsTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Return scale factor for upper bound on moving asymptotes
     * @return scale factor
    **********************************************************************************/
    ScalarType getMovingAsymptoteUpperBoundScaleFactor() const
    {
        return (mMovingAsymptoteUpperBoundScaleFactor);
    }

    /******************************************************************************//**
     * @brief Set scale factor for upper bound on moving asymptotes
     * @param [in] aInput scale factor
    **********************************************************************************/
    void setMovingAsymptoteUpperBoundScaleFactor(const ScalarType & aInput)
    {
        mMovingAsymptoteUpperBoundScaleFactor = aInput;
    }

    /******************************************************************************//**
     * @brief Return scale factor for lower bound on moving asymptotes
     * @return scale factor
    **********************************************************************************/
    ScalarType getMovingAsymptoteLowerBoundScaleFactor() const
    {
        return (mMovingAsymptoteLowerBoundScaleFactor);
    }

    /******************************************************************************//**
     * @brief Set scale factor for lower bound on moving asymptotes
     * @param [in] aInput scale factor
    **********************************************************************************/
    void setMovingAsymptoteLowerBoundScaleFactor(const ScalarType & aInput)
    {
        mMovingAsymptoteLowerBoundScaleFactor = aInput;
    }

    /******************************************************************************//**
     * @brief Get stopping criterion
     * @return stopping criterion
    **********************************************************************************/
    Plato::ccsa::stop_t getStoppingCriterion() const
    {
        return (mStopCriterion);
    }

    /******************************************************************************//**
     * @brief Set stopping criterion
     * @param [in] stopping criterion
    **********************************************************************************/
    void setStoppingCriterion(const Plato::ccsa::stop_t & aInput)
    {
        mStopCriterion = aInput;
    }

    /******************************************************************************//**
     * @brief Return reference to data manager
     * @return conservative convex separable approximation (CCSA) algorithm data manager
    **********************************************************************************/
    const Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & getDataMng() const
    {
        return (*mDataMng);
    }

    /******************************************************************************//**
     * @brief Solve optimization problem using Optimality criteria algorithm
    **********************************************************************************/
    void solve()
    {
        this->openOutputFile();
        this->checkInitialGuess();
        this->setInitialProblemData();

        while(1)
        {
            this->computeCurrentGradients();
            mStageMng->update(*mDataMng);

            this->outputDiagnostics();
            if(this->checkStoppingCriteria() == true)
            {
                this->outputMyStoppingCriterion();
                this->closeOutputFile();
                break;
            }

            this->updateSigmaParameters();
            this->saveProblemData();

            mSubProblem->solve(*mStageMng, *mDataMng);
            mNumOuterIterDone++;
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
                const OrdinalType tNumConstraints = mDataMng->getNumConstraints();
                mOutputData.mConstraints.clear();
                mOutputData.mConstraints.resize(tNumConstraints);
                mOutputStream.open("plato_ccsa_algorithm_diagnostics.txt");
                Plato::print_ccsa_diagnostics_header(mOutputData, mOutputStream, mPrintDiagnostics);
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
     * @brief Print diagnostics for Optimality Criteria algorithm
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
            mOutputData.mKKTMeasure = mDataMng->getKarushKuhnTuckerConditionsInexactness();
            mOutputData.mObjFuncValue = mDataMng->getCurrentObjectiveFunctionValue();
            mOutputData.mObjFuncCount = mDataMng->getNumObjectiveFunctionEvaluations();
            mOutputData.mNormObjFuncGrad = mDataMng->getNormInactiveGradient();
            mOutputData.mStationarityMeasure = mDataMng->getStationarityMeasure();
            mOutputData.mControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
            mOutputData.mObjectiveStagnationMeasure = mDataMng->getObjectiveStagnationMeasure();

            const OrdinalType tDUAL_VECTOR_INDEX = 0;
            const OrdinalType tNumConstraints = mDataMng->getNumConstraints();
            for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
            {
                mOutputData.mConstraints[tIndex] = mDataMng->getCurrentConstraintValue(tDUAL_VECTOR_INDEX, tIndex);
            }

            Plato::print_ccsa_diagnostics(mOutputData, mOutputStream, mPrintDiagnostics);
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
                Plato::get_ccsa_stop_criterion(mStopCriterion, tReason);
                mOutputStream << tReason.c_str();
            }
        }
    }

    /******************************************************************************//**
     * @brief Compute current objective and constraint gradients.
    **********************************************************************************/
    void computeCurrentGradients()
    {
        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = mDataMng->getCurrentControl();
        mStageMng->cacheData();
        mStageMng->computeGradient(tCurrentControl, *mControlWork);
        mDataMng->setCurrentObjectiveGradient(*mControlWork);
        mStageMng->computeConstraintGradients(tCurrentControl, *mWorkMultiVectorList);
        mDataMng->setCurrentConstraintGradients(*mWorkMultiVectorList);
    }

    /******************************************************************************//**
     * @brief Compute initial objective and constraint values and initialize auxiliary variables.
    **********************************************************************************/
    void setInitialProblemData()
    {
        const Plato::MultiVector<ScalarType, OrdinalType> & tControl = mDataMng->getCurrentControl();
        const ScalarType tInitialObjFuncValue = mStageMng->evaluateObjective(tControl);
        mDataMng->setCurrentObjectiveFunctionValue(tInitialObjFuncValue);
        mStageMng->evaluateConstraints(tControl, *mDualWork);
        mDataMng->setCurrentConstraintValues(*mDualWork);
        mDataMng->setDual(*mDualWork);
        mSubProblem->initializeAuxiliaryVariables(*mDataMng);
    }

    /******************************************************************************//**
     * @brief Save previous and antepenultimate criteria values and controls.
    **********************************************************************************/
    void saveProblemData()
    {
        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = mDataMng->getCurrentControl();
        const Plato::MultiVector<ScalarType, OrdinalType> & tPreviousControl = mDataMng->getPreviousControl();
        Plato::update(static_cast<ScalarType>(1), tPreviousControl, static_cast<ScalarType>(0), *mAntepenultimateControl);
        Plato::update(static_cast<ScalarType>(1), tCurrentControl, static_cast<ScalarType>(0), *mControlWork);
        mDataMng->setPreviousControl(*mControlWork);

        const ScalarType tCurrentObjFuncValue = mDataMng->getCurrentObjectiveFunctionValue();
        mDataMng->setPreviousObjectiveFunctionValue(tCurrentObjFuncValue);
    }

    /******************************************************************************//**
     * @brief Initialize bounds and check if the bounds are properly defined.
     * @param [in] aDataMng data manager for CCSA algorithms
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Check if initial guess is within upper and lower bounds
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Check if a stopping criterion is met.
     * @return flag instructing algorithm to stop, true or false
    **********************************************************************************/
    bool checkStoppingCriteria()
    {
        bool tStop = false;

        mDataMng->computeFeasibilityMeasure();
        mDataMng->computeStationarityMeasure();
        mDataMng->computeNormInactiveGradient();
        mDataMng->computeControlStagnationMeasure();
        mDataMng->computeObjectiveStagnationMeasure();

        const Plato::MultiVector<ScalarType, OrdinalType> & tDual = mDataMng->getDual();
        const Plato::MultiVector<ScalarType, OrdinalType> & tControl = mDataMng->getCurrentControl();
        mDataMng->computeKarushKuhnTuckerConditionsInexactness(tControl, tDual);

        const ScalarType tStationarityMeasure = mDataMng->getStationarityMeasure();
        const ScalarType tControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
        const ScalarType tObjectiveStagnationMeasure = mDataMng->getObjectiveStagnationMeasure();
        const ScalarType t_KKT_ConditionsInexactness = mDataMng->getKarushKuhnTuckerConditionsInexactness();

        if(tControlStagnationMeasure < this->getControlStagnationTolerance())
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
        else if(mNumOuterIterDone >= this->getMaxNumIterations())
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

    /******************************************************************************//**
     * @brief Update sigma coefficients
    **********************************************************************************/
    void updateSigmaParameters()
    {
        assert(mControlWork.get() != nullptr);
        assert(mPreviousSigma.get() != nullptr);

        const Plato::MultiVector<ScalarType, OrdinalType> & tDMCurrentSigma = mDataMng->getCurrentSigma();
        Plato::update(static_cast<ScalarType>(1), tDMCurrentSigma, static_cast<ScalarType>(0), *mPreviousSigma);

        const OrdinalType tNumIterationsDone = this->getNumIterationsDone();
        if(tNumIterationsDone < static_cast<OrdinalType>(2))
        {
            this->computeInitialSigmaCoefficients();
        }
        else
        {
            this->computeSigmaCoefficients();
        }
    }

    /******************************************************************************//**
     * @brief Compute initial sigma coefficients (i.e. first optimization iteration).
    **********************************************************************************/
    void computeInitialSigmaCoefficients()
    {
        const OrdinalType tNumVectors = mControlWork->getNumVectors();
        const ScalarType tMultiplier = this->getInitialMovingAsymptoteScaleFactor();

        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tCurrentSigma = mControlWork->operator[](tIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getControlUpperBounds(tIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getControlLowerBounds(tIndex);
            mDataMng->elementWiseFunctions().updateInitialSigmaCoeff(tMultiplier, tUpperBounds, tLowerBounds, tCurrentSigma);
        }

        mDataMng->setCurrentSigma(*mControlWork);
    }

    /******************************************************************************//**
     * @brief Compute sigma coefficients if optimization iteration is greater than two.
    **********************************************************************************/
    void computeSigmaCoefficients()
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

private:
    bool mPrintDiagnostics;
    std::ofstream mOutputStream;

    OrdinalType mMaxNumOuterIter;
    OrdinalType mNumOuterIterDone;

    ScalarType mStationarityTolerance;
    ScalarType mControlStagnationTolerance;
    ScalarType mObjectiveStagnationTolerance;
    ScalarType mMovingAsymptoteExpansionFactor;
    ScalarType mMovingAsymptoteContractionFactor;
    ScalarType mInitialMovingAsymptoteScaleFactor;
    ScalarType mKarushKuhnTuckerConditionsTolerance;
    ScalarType mMovingAsymptoteUpperBoundScaleFactor;
    ScalarType mMovingAsymptoteLowerBoundScaleFactor;

    Plato::ccsa::stop_t mStopCriterion;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDualWork;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlWork;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousSigma;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAntepenultimateControl;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mWorkMultiVectorList;

    Plato::OutputDataCCSA<ScalarType, OrdinalType> mOutputData;
    std::shared_ptr<Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType>> mDataMng;
    std::shared_ptr<Plato::ConservativeConvexSeparableAppxStageMng<ScalarType, OrdinalType>> mStageMng;
    std::shared_ptr<Plato::ConservativeConvexSeparableApproximation<ScalarType, OrdinalType>> mSubProblem;

private:
    ConservativeConvexSeparableAppxAlgorithm(const Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> & aRhs);
    Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> & operator=(const Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_CONSERVATIVECONVEXSEPARABLEAPPXALGORITHM_HPP_ */
