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
 * Plato_MethodMovingAsymptotes.hpp
 *
 *  Created on: Sep 1, 2019
 */

#pragma once

#include "Plato_Console.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_AugmentedLagrangian.hpp"
#include "Plato_MethodMovingAsymptotesIO.hpp"
#include "Plato_MethodMovingAsymptotesDataMng.hpp"
#include "Plato_MethodMovingAsymptotesCriterion.hpp"
#include "Plato_MethodMovingAsymptotesOperations.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotes
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aObjective objective criterion interface
     * @param [in] aConstraints list of constraint criteria interfaces
     * @param [in] aDataFactory data factory used to allocate core data structure
    **********************************************************************************/
    MethodMovingAsymptotes(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> &aObjective,
                           const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> &aConstraints,
                           const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mPrintDiagnostics(false),
        mOutputStream(),
        mIterationCount(0),
        mNumObjFuncEvals(0),
        mMaxNumIterations(100),
        mProblemUpdateFrequency(0),
        mMaxNumTrustRegionIterations(50),
        mMaxNumSubProblemIterations(100),
        mInitialAugLagPenalty(1),
        mAugLagPenaltyMultiplier(1.025),
        mOptimalityTolerance(1e-6),
        mFeasibilityTolerance(1e-4),
        mControlStagnationTolerance(1e-6),
        mObjectiveStagnationTolerance(1e-8),
        mStoppingCriterion(Plato::algorithm::stop_t::NOT_CONVERGED),
        mObjective(aObjective),
        mConstraints(aConstraints),
        mMMAData(std::make_shared<Plato::ApproximationFunctionData<ScalarType, OrdinalType>>(aDataFactory)),
        mDataMng(std::make_shared<Plato::MethodMovingAsymptotesDataMng<ScalarType, OrdinalType>>(aDataFactory)),
        mOperations(std::make_shared<Plato::MethodMovingAsymptotesOperations<ScalarType, OrdinalType>>(aDataFactory)),
        mConstrAppxFuncList(std::make_shared<Plato::CriterionList<ScalarType, OrdinalType>>())
    {
        this->initialize(aDataFactory);
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~MethodMovingAsymptotes()
    {
    }

    /******************************************************************************//**
     * @brief Return optimization iteration count
     * @return optimization iteration count
    **********************************************************************************/
    OrdinalType getNumIterations() const
    {
        return mIterationCount;
    }

    /******************************************************************************//**
     * @brief Return number of objective function evaluations
     * @return number of objective function evaluations
    **********************************************************************************/
    OrdinalType getNumObjFuncEvals() const
    {
        return mNumObjFuncEvals;
    }

    /******************************************************************************//**
     * @brief Return optimal objective function value
     * @return optimal objective function value
    **********************************************************************************/
    ScalarType getOptimalObjectiveValue() const
    {
        return (mDataMng->getCurrentObjectiveValue());
    }

    /******************************************************************************//**
     * @brief Return norm of the objective function gradient
     * @return norm of the objective function gradient
    **********************************************************************************/
    ScalarType getNormObjectiveGradient() const
    {
        return (mDataMng->getNormObjectiveGradient());
    }

    /******************************************************************************//**
     * @brief Return control stagnation measure
     * @return control stagnation measure
    **********************************************************************************/
    ScalarType getControlStagnationMeasure() const
    {
        return (mDataMng->getControlStagnationMeasure());
    }

    /******************************************************************************//**
     * @brief Return objective stagnation measure
     * @return objective stagnation measure
    **********************************************************************************/
    ScalarType getObjectiveStagnationMeasure() const
    {
        return (mDataMng->getObjectiveStagnationMeasure());
    }

    /******************************************************************************//**
     * @brief Return optimal constraint value
     * @param constraint index
     * @return optimal constraint value
    **********************************************************************************/
    ScalarType getOptimalConstraintValue(const OrdinalType& aIndex) const
    {
        return (mDataMng->getCurrentConstraintValue(aIndex));
    }

    /******************************************************************************//**
     * @brief Return stopping criterion
     * @return stopping criterion
    **********************************************************************************/
    Plato::algorithm::stop_t getStoppingCriterion() const
    {
        return (mStoppingCriterion);
    }

    /******************************************************************************//**
     * @brief Print algorithm's diagnostics (i.e. optimization problem status)
     * @param [in] aInput print diagnostics if true; if false, do not print diagnostics.
    **********************************************************************************/
    void enableDiagnostics(const bool & aInput)
    {
        mPrintDiagnostics = aInput;
    }

    /******************************************************************************//**
     * @brief Enable bound constraint optimization
    **********************************************************************************/
    void enableBoundConstrainedOptimization()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            mConstrAppxFuncs[tIndex]->setObjFuncAppxFuncMultiplier(0.0);
            mConstrAppxFuncs[tIndex]->setConstraintAppxFuncMultiplier(0.0);
        }
    }

    /******************************************************************************//**
     * @brief Set maximum number of iterations
     * @param [in] aInput maximum number of iterations
    **********************************************************************************/
    void setMaxNumIterations(const OrdinalType& aInput)
    {
        mMaxNumIterations = aInput;
    }

    /******************************************************************************//**
     * @brief Set update problem, i.e. continuation, frequency
     * @param [in] aInput update problem, i.e. continuation, frequency
    **********************************************************************************/
    void setUpdateFrequency(const OrdinalType& aInput)
    {
        mProblemUpdateFrequency = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of subproblem iterations
     * @param [in] aInput maximum number of subproblem iterations
    **********************************************************************************/
    void setMaxNumSubProblemIterations(const OrdinalType& aInput)
    {
        mMaxNumSubProblemIterations = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of trust region iterations
     * @param [in] aInput maximum number of trust region iterations
    **********************************************************************************/
    void setMaxNumTrustRegionIterations(const OrdinalType& aInput)
    {
        mMaxNumTrustRegionIterations = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of trust region iterations
     * @param [in] aInput maximum number of trust region iterations
    **********************************************************************************/
    void setMoveLimit(const ScalarType& aInput)
    {
        mOperations->setMoveLimit(aInput);
    }

    /******************************************************************************//**
     * @brief Set moving asymptotes expansion parameter
     * @param [in] aInput moving asymptotes expansion parameter
    **********************************************************************************/
    void setAsymptoteExpansionParameter(const ScalarType& aInput)
    {
        mOperations->setAsymptoteExpansionParameter(aInput);
    }

    /******************************************************************************//**
     * @brief Set moving asymptotes contraction parameter
     * @param [in] aInput moving asymptotes contraction parameter
    **********************************************************************************/
    void setAsymptoteContractionParameter(const ScalarType& aInput)
    {
        mOperations->setAsymptoteContractionParameter(aInput);
    }

    /******************************************************************************//**
     * @brief Set scaling on initial moving asymptotes
     * @param [in] aInput scaling on initial moving asymptotes
    **********************************************************************************/
    void setInitialAymptoteScaling(const ScalarType& aInput)
    {
        mOperations->setInitialAymptoteScaling(aInput);
    }

    /******************************************************************************//**
     * @brief Set scaling on optimization variables bounds for subproblem
     * @param [in] aInput scaling on optimization variables bounds for subproblem
    **********************************************************************************/
    void setSubProblemBoundsScaling(const ScalarType& aInput)
    {
        mOperations->setSubProblemBoundsScaling(aInput);
    }

    /******************************************************************************//**
     * @brief Set initial penalty on Augmented Lagrangian function
     * @param [in] aInput initial penalty on Augmented Lagrangian function
    **********************************************************************************/
    void setInitialAugLagPenalty(const ScalarType& aInput)
    {
        mInitialAugLagPenalty = aInput;
    }

    /******************************************************************************//**
     * @brief Set reduction parameter for augmented Lagrangian penalty
     * @param [in] aInput reduction parameter for augmented Lagrangian penalty
    **********************************************************************************/
    void setAugLagSubProbPenaltyMultiplier(const ScalarType& aInput)
    {
        mAugLagPenaltyMultiplier = aInput;
    }

    /******************************************************************************//**
     * @brief Set optimality tolerance
     * @param [in] aInput optimality tolerance
    **********************************************************************************/
    void setOptimalityTolerance(const ScalarType& aInput)
    {
        mOptimalityTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set feasibility tolerance
     * @param [in] aInput feasibility tolerance
    **********************************************************************************/
    void setFeasibilityTolerance(const ScalarType& aInput)
    {
        mFeasibilityTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set feasibility tolerance for augmened Lagrangian subproblem.
     * @param [in] aInput feasibility tolerance
    **********************************************************************************/
    void setSubProblemFeasibilityTolerance(const ScalarType& aInput)
    {
        mSubProblemFeasibilityTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set control stagnation tolerance
     * @param [in] aInput control stagnation tolerance
    **********************************************************************************/
    void setControlStagnationTolerance(const ScalarType& aInput)
    {
        mControlStagnationTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set objective function stagnation tolerance
     * @param [in] aInput objective function stagnation tolerance
    **********************************************************************************/
    void setObjectiveStagnationTolerance(const ScalarType& aInput)
    {
        mObjectiveStagnationTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set 2D container of lower bound on optimization variables
     * @param [in] aInput const reference 2D container of lower bound on optimization variables
    **********************************************************************************/
    void setControlLowerBounds(const Plato::MultiVector<ScalarType, OrdinalType> &aInput)
    {
        mDataMng->setControlLowerBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Set 2D container of upper bound on optimization variables
     * @param [in] aInput const reference 2D container of upper bound on optimization variables
    **********************************************************************************/
    void setControlUpperBounds(const Plato::MultiVector<ScalarType, OrdinalType> &aInput)
    {
        mDataMng->setControlUpperBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Set 2D container of initial optimization variables
     * @param [in] aInput 2D container of initial optimization variables
    **********************************************************************************/
    void setInitialGuess(const Plato::MultiVector<ScalarType, OrdinalType> &aInput)
    {
        mDataMng->setCurrentControls(aInput);
    }

    /******************************************************************************//**
     * @brief Set constraint normalization values
     * @param [in] aIndex constraint index
     * @param [in] aInput constraint normalization values
    **********************************************************************************/
    void setConstraintNormalization(const OrdinalType & aIndex, const ScalarType & aValue)
    {
        mDataMng->setConstraintNormalization(aIndex, aValue);
    }

    /******************************************************************************//**
     * @brief Set constraint normalization parameters
     * @param [in] aInput reference to 1D container of constraint normalization parameters
    **********************************************************************************/
    void setConstraintNormalizationParams(Plato::Vector<ScalarType, OrdinalType>& aInput)
    {
        mDataMng->setConstraintNormalizationParams(aInput);
    }

    /******************************************************************************//**
     * @brief Set output diagnostics flag for augmented Lagrangian subproblem. 
     * @param [out] aInput boolean flag
    **********************************************************************************/
    void outputSubProblemDiagnostics(const bool aInput)
    {
        if(aInput == true)
        {
            mSubProblemSolver->enableDiagnostics();
        }
    }

    /******************************************************************************//**
     * @brief Gather 2D container of optimal optimization variables
     * @param [out] aInput reference to 2D container of optimal optimization variables
    **********************************************************************************/
    void getSolution(Plato::MultiVector<ScalarType, OrdinalType>& aInput) const
    {
        Plato::update(static_cast<ScalarType>(1.0), mDataMng->getCurrentControls(), static_cast<ScalarType>(0.0), aInput);
    }

    /******************************************************************************//**
     * @brief Gather 1D container of optimal constraint values
     * @param [out] aInput reference to 1D container of optimal constraint values
    **********************************************************************************/
    void getOptimalConstraintValues(Plato::Vector<ScalarType, OrdinalType>& aInput) const
    {
        mDataMng->getCurrentConstraintValues(aInput);
    }

    /******************************************************************************//**
     * @brief Solve problem using the Method of Moving Asymptotes (MMA) algorithm.
    **********************************************************************************/
    void solve()
    {
        this->openOutputFile();
        this->setSubProblemOptions();

        mOperations->initialize(*mDataMng);
        while(true)
        {
            this->updateState();
            this->updateSubProblem();
            mDataMng->cacheState();
            this->solveSubProblem();
            mIterationCount++;

            bool tStop = this->checkStoppingCriteria();
            if(tStop == true)
            {
                this->updateState();
                mDataMng->cacheState();
                this->printStoppingCriterion();
                this->closeOutputFile();
                break;
            }
        }
    }

private:
    /******************************************************************************//**
     * @brief Initialize core internal operations and data structures.
    **********************************************************************************/
    void initialize(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory)
    {
        this->initializeApproximationFunctions(aDataFactory);
        mSubProblemSolver = std::make_shared<Plato::AugmentedLagrangian<ScalarType, OrdinalType>>(mObjAppxFunc, mConstrAppxFuncList, aDataFactory);
    }

    /******************************************************************************//**
     * @brief Set algorithmic options for Method Of Moving Asymptotes (MMA) subproblem
    **********************************************************************************/
    void setSubProblemOptions()
    {
        mSubProblemSolver->disablePostSmoothing();
        mSubProblemSolver->setPenaltyParameter(mInitialAugLagPenalty);
        mSubProblemSolver->setFeasibilityTolerance(mSubProblemFeasibilityTolerance);
        mSubProblemSolver->setControlStagnationTolerance(mControlStagnationTolerance);
        mSubProblemSolver->setMaxNumOuterIterations(mMaxNumSubProblemIterations);
        mSubProblemSolver->setPenaltyParameterScaleFactor(mAugLagPenaltyMultiplier);
        mSubProblemSolver->setMaxNumTrustRegionSubProblemIterations(mMaxNumTrustRegionIterations);
    }

    /******************************************************************************//**
     * @brief Initialize Method of Moving Asymptotes (MMA) approximation functions.
    **********************************************************************************/
    void initializeApproximationFunctions(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory)
    {
        mObjAppxFunc = std::make_shared<Plato::MethodMovingAsymptotesCriterion<ScalarType, OrdinalType>>(aDataFactory);
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            mConstrAppxFuncs.push_back(std::make_shared<Plato::MethodMovingAsymptotesCriterion<ScalarType, OrdinalType>>(aDataFactory));
            mConstrAppxFuncList->add(mConstrAppxFuncs[tIndex]);
        }
    }

    /******************************************************************************//**
     * @brief Open output file (i.e. diagnostics file)
    **********************************************************************************/
    void openOutputFile()
    {
        if (mPrintDiagnostics == false)
        {
            return;
        }

        const Plato::CommWrapper &tMyCommWrapper = mDataMng->getCommWrapper();
        if (tMyCommWrapper.myProcID() == 0)
        {
            const OrdinalType tNumConstraints = mDataMng->getNumConstraints();
            mOutputData.mConstraints.clear();
            mOutputData.mConstraints.resize(tNumConstraints);
            mOutputStream.open("plato_mma_algorithm_diagnostics.txt");
            Plato::print_mma_diagnostics_header(mOutputData, mOutputStream);
        }
    }

    /******************************************************************************//**
     * @brief Close output file (i.e. diagnostics file)
    **********************************************************************************/
    void closeOutputFile()
    {
        if (mPrintDiagnostics == false)
        {
            return;
        }

        const Plato::CommWrapper &tMyCommWrapper = mDataMng->getCommWrapper();
        if (tMyCommWrapper.myProcID() == 0)
        {
            mOutputStream.close();
        }
    }

    /******************************************************************************//**
     * @brief Print MMA diagnostics to file
    **********************************************************************************/
    void printDiagnostics()
    {
        if(mPrintDiagnostics == false)
        {
            return;
        }

        const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
        if(tMyCommWrapper.myProcID() == 0)
        {
            mOutputData.mNumIter = mIterationCount;
            mOutputData.mObjFuncCount = mNumObjFuncEvals;
            mOutputData.mObjFuncValue = mDataMng->getCurrentObjectiveValue();
            mOutputData.mNormObjFuncGrad = mDataMng->getNormObjectiveGradient();
            mOutputData.mControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
            mOutputData.mObjectiveStagnationMeasure = mDataMng->getObjectiveStagnationMeasure();

            const OrdinalType tNumConstraints = mDataMng->getNumConstraints();
            for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
            {
                mOutputData.mConstraints[tConstraintIndex] = mDataMng->getCurrentConstraintValue(tConstraintIndex);
            }

            Plato::print_mma_diagnostics(mOutputData, mOutputStream);

            std::stringstream tConsoleStream;
            Plato::print_mma_diagnostics_header(mOutputData, tConsoleStream);
            Plato::print_mma_diagnostics(mOutputData, tConsoleStream);
            Plato::Console::Alert(tConsoleStream.str());
        }
    }

    /******************************************************************************//**
     * @brief Print stopping criterion to diagnostics file.
    **********************************************************************************/
    void printStoppingCriterion()
    {
        if(mPrintDiagnostics == true)
        {
            const Plato::CommWrapper& tMyCommWrapper = mDataMng->getCommWrapper();
            if(tMyCommWrapper.myProcID() == 0)
            {
                std::string tReason;
                Plato::print_mma_stop_criterion(mStoppingCriterion, tReason);
                mOutputStream << tReason.c_str();
            }
        }
    }

    /******************************************************************************//**
     * @brief Evaluate objective criterion and its gradient.
    **********************************************************************************/
    void evaluateObjective()
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = mDataMng->getCurrentControls();
        ScalarType tObjFuncValue = mObjective->value(tCurrentControls);
        mNumObjFuncEvals++;
        mObjective->cacheData();
        mDataMng->setCurrentObjectiveValue(tObjFuncValue);
        Plato::MultiVector<ScalarType, OrdinalType> &tCurrentObjGrad = mDataMng->getCurrentObjectiveGradient();
        mObjective->gradient(tCurrentControls, tCurrentObjGrad);
    }

    /******************************************************************************//**
     * @brief Evaluate constraint criteria and respective gradients.
    **********************************************************************************/
    void evaluateConstraints()
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = mDataMng->getCurrentControls();
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const ScalarType tValue = (*mConstraints)[tIndex].value(tCurrentControls);
            (*mConstraints)[tIndex].cacheData();
            mDataMng->setCurrentConstraintValue(tIndex, tValue);
            Plato::MultiVector<ScalarType, OrdinalType> &tCurrentConstraintGrad = mDataMng->getCurrentConstraintGradient(tIndex);
            (*mConstraints)[tIndex].gradient(tCurrentControls, tCurrentConstraintGrad);
        }
    }

    /******************************************************************************//**
     * @brief Update Method of Moving Asymptotes (MMA) subproblem.
    **********************************************************************************/
    void updateSubProblem()
    {
        this->updateAsymptotes();
        mOperations->updateSubProblemBounds(*mDataMng);
        mOperations->updateObjectiveApproximationFunctionData(*mDataMng);
        mOperations->updateConstraintApproximationFunctionsData(*mDataMng);
        this->updateObjectiveApproximationFunction();
        this->updateConstraintApproximationFunctions();
    }

    /******************************************************************************//**
     * @brief Update current state and stopping measures
    **********************************************************************************/
    void updateState()
    {
        this->evaluateObjective();
        this->evaluateConstraints();
        this->performContinuation();
        mDataMng->computeStoppingMeasures();
        this->printDiagnostics();
    }

    /******************************************************************************//**
     * @brief If active, safely allow application to perform continuation on its data.
    **********************************************************************************/
    void performContinuation()
    {
        const bool tIsContinuationEnabled = mProblemUpdateFrequency > static_cast<OrdinalType>(0);
        const auto IterationCountOneBase = mIterationCount + static_cast<OrdinalType>(1);
        bool tPerformContinuation = tIsContinuationEnabled ? (IterationCountOneBase % mProblemUpdateFrequency) == static_cast<OrdinalType>(0) : false;
        if (tPerformContinuation)
        {
            mObjective->updateProblem(mDataMng->getCurrentControls());
            const OrdinalType tNumConstraints = mConstraints->size();
            for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
            {
                (*mConstraints)[tIndex].updateProblem(mDataMng->getCurrentControls());
            }
        }
    }

    /******************************************************************************//**
     * @brief Update moving asymptotes.
    **********************************************************************************/
    void updateAsymptotes()
    {
        if(mIterationCount >= static_cast<OrdinalType>(2))
        {
            mOperations->updateCurrentAsymptotesMultipliers(*mDataMng);
            mOperations->updateCurrentAsymptotes(*mDataMng);
        }
        else
        {
            mOperations->updateInitialAsymptotes(*mDataMng);
        }
    }

    /******************************************************************************//**
     * @brief Update approximation functions associated with the objective criterion.
    **********************************************************************************/
    void updateObjectiveApproximationFunction()
    {
        mMMAData->mCurrentNormalizedCriterionValue = 1.0;
        Plato::update(static_cast<ScalarType>(1), mDataMng->getCurrentControls(), static_cast<ScalarType>(0), *mMMAData->mCurrentControls);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getLowerAsymptotes(), static_cast<ScalarType>(0), *mMMAData->mLowerAsymptotes);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getUpperAsymptotes(), static_cast<ScalarType>(0), *mMMAData->mUpperAsymptotes);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getObjFuncAppxFunctionP(), static_cast<ScalarType>(0), *mMMAData->mAppxFunctionP);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getObjFuncAppxFunctionQ(), static_cast<ScalarType>(0), *mMMAData->mAppxFunctionQ);

        mObjAppxFunc->update(*mMMAData);
    }

    /******************************************************************************//**
     * @brief Update approximation functions associated with the constraint criteria
    **********************************************************************************/
    void updateConstraintApproximationFunctions()
    {
        Plato::update(static_cast<ScalarType>(1), mDataMng->getCurrentControls(), static_cast<ScalarType>(0), *mMMAData->mCurrentControls);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getLowerAsymptotes(), static_cast<ScalarType>(0), *mMMAData->mLowerAsymptotes);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getUpperAsymptotes(), static_cast<ScalarType>(0), *mMMAData->mUpperAsymptotes);

        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const ScalarType tConstraintValue = mDataMng->getCurrentConstraintValue(tIndex);
            const ScalarType tValue = mDataMng->getConstraintNormalization(tIndex);
            const ScalarType tConstraintNormalization = std::abs(tValue) <= std::numeric_limits<ScalarType>::epsilon() ? static_cast<ScalarType>(1) : tValue;
            mMMAData->mCurrentNormalizedCriterionValue = tConstraintValue / tConstraintNormalization;
            Plato::update(static_cast<ScalarType>(1), mDataMng->getConstraintAppxFunctionP(tIndex), static_cast<ScalarType>(0), *mMMAData->mAppxFunctionP);
            Plato::update(static_cast<ScalarType>(1), mDataMng->getConstraintAppxFunctionQ(tIndex), static_cast<ScalarType>(0), *mMMAData->mAppxFunctionQ);
            mConstrAppxFuncs[tIndex]->update(*mMMAData);
        }
    }

    /******************************************************************************//**
     * @brief Solve Method of Moving Asymptotes (MMA) subproblem
    **********************************************************************************/
    void solveSubProblem()
    {
        mSubProblemSolver->resetParameters();
        mSubProblemSolver->setInitialGuess(mDataMng->getCurrentControls());
        mSubProblemSolver->setControlLowerBounds(mDataMng->getSubProblemControlLowerBounds());
        mSubProblemSolver->setControlUpperBounds(mDataMng->getSubProblemControlUpperBounds());
        mSubProblemSolver->solve();
        mSubProblemSolver->getSolution(mDataMng->getCurrentControls());
    }

    /******************************************************************************//**
     * @brief Check stopping criteria
     * @return stopping criterion met, yes or no
    **********************************************************************************/
    bool checkStoppingCriteria()
    {
        bool tStop = false;
        const ScalarType tFeasibilityMeasure = mDataMng->getFeasibilityMeasure();
        const ScalarType tOptimalityMeasure = mDataMng->getNormObjectiveGradient();
        const ScalarType tControlStagnation = mDataMng->getControlStagnationMeasure();
        const ScalarType tObjectiveStagnation = mDataMng->getObjectiveStagnationMeasure();

        if(mIterationCount == mMaxNumIterations)
        {
            mStoppingCriterion = Plato::algorithm::MAX_NUMBER_ITERATIONS;
            tStop = true;
        }
        else if(tControlStagnation <= mControlStagnationTolerance)
        {
            mStoppingCriterion = Plato::algorithm::CONTROL_STAGNATION;
            tStop = true;
        }
        else if(tObjectiveStagnation <= mObjectiveStagnationTolerance)
        {
            mStoppingCriterion = Plato::algorithm::OBJECTIVE_STAGNATION;
            tStop = true;
        }
        else if(tOptimalityMeasure <= mOptimalityTolerance && tFeasibilityMeasure <= mFeasibilityTolerance)
        {
            mStoppingCriterion = Plato::algorithm::OPTIMALITY_AND_FEASIBILITY;
            tStop = true;
        }

        return (tStop);
    }

private:
    bool mPrintDiagnostics; /*!< output MMA diagnostics to text file (default=false) */
    std::ofstream mOutputStream; /*!< output string stream with diagnostics */

    OrdinalType mIterationCount; /*!< number of optimization iterations */
    OrdinalType mNumObjFuncEvals; /*!< number of objective function evaluations */
    OrdinalType mMaxNumIterations; /*!< maximum number of optimization iterations */
    OrdinalType mProblemUpdateFrequency; /*!< problem update, i.e. continuation, frequency */
    OrdinalType mMaxNumTrustRegionIterations; /*!< maximum number of trust region subproblem iterations */
    OrdinalType mMaxNumSubProblemIterations; /*!< maximum number of MMA subproblem iterations iterations */

    ScalarType mInitialAugLagPenalty; /*!< initial augmented Lagragian penalty parameter */
    ScalarType mAugLagPenaltyMultiplier; /*!< augmented Lagragian penalty reduction parameter */
    ScalarType mOptimalityTolerance; /*!< tolerance on the norm of the objective gradient - primary stopping tolerance */
    ScalarType mFeasibilityTolerance; /*!< feasibility tolerance - primary stopping tolerance */
    ScalarType mSubProblemFeasibilityTolerance = 1e-8; /*!< feasibility tolerance for augmented Lagrangian subproblem */
    ScalarType mControlStagnationTolerance; /*!< control stagnation tolerance - secondary stopping tolerance */
    ScalarType mObjectiveStagnationTolerance; /*!< objective stagnation tolerance - secondary stopping tolerance */
    Plato::algorithm::stop_t mStoppingCriterion; /*!< stopping criterion */

    Plato::OutputDataMMA<ScalarType, OrdinalType> mOutputData; /*!< output data structure */

    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mObjective; /*!< objective criterion interface */
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> mConstraints; /*!< constraint criteria interface */

    std::shared_ptr<Plato::ApproximationFunctionData<ScalarType, OrdinalType>> mMMAData; /*!< structure with approximation function's data */
    std::shared_ptr<Plato::AugmentedLagrangian<ScalarType, OrdinalType>> mSubProblemSolver; /*!< MMA subproblem solver */
    std::shared_ptr<Plato::MethodMovingAsymptotesDataMng<ScalarType, OrdinalType>> mDataMng; /*!< MMA data manager */
    std::shared_ptr<Plato::MethodMovingAsymptotesOperations<ScalarType, OrdinalType>> mOperations; /*!< interface to MMA core operations */

    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> mConstrAppxFuncList; /*!< list of constraint criteria */
    std::shared_ptr<Plato::MethodMovingAsymptotesCriterion<ScalarType, OrdinalType>> mObjAppxFunc; /*!< objective criterion approximation function */
    std::vector<std::shared_ptr<Plato::MethodMovingAsymptotesCriterion<ScalarType, OrdinalType>>> mConstrAppxFuncs; /*!< list of constraint criteria approximation function */

private:
    MethodMovingAsymptotes(const Plato::MethodMovingAsymptotes<ScalarType, OrdinalType> & aRhs);
    Plato::MethodMovingAsymptotes<ScalarType, OrdinalType> & operator=(const Plato::MethodMovingAsymptotes<ScalarType, OrdinalType> & aRhs);
};
// class MethodMovingAsymptotes

}
// namespace Plato
