/*
 * Plato_OptimalityCriteria.hpp
 *
 *  Created on: Oct 17, 2017
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

#ifndef PLATO_OPTIMALITYCRITERIA_HPP_
#define PLATO_OPTIMALITYCRITERIA_HPP_

#include <sstream>
#include <iostream>

#include "Plato_Vector.hpp"
#include "Plato_HostBounds.hpp"
#include "Plato_ErrorChecks.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DeviceBounds.hpp"
#include "Plato_OptimizersIO.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_OptimalityCriteriaDataMng.hpp"
#include "Plato_OptimalityCriteriaSubProblem.hpp"
#include "Plato_OptimalityCriteriaStageMngBase.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class OptimalityCriteria
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aDataMng data manager for the Optimality Criteria algorithm
     * @param [in] aStageMng stage manager for the Optimality Criteria algorithm
     * @param [in] aSubProblem optimality criteria subproblem
    **********************************************************************************/
    OptimalityCriteria(const std::shared_ptr<Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType>> & aDataMng,
                       const std::shared_ptr<Plato::OptimalityCriteriaStageMngBase<ScalarType, OrdinalType>> & aStageMng,
                       const std::shared_ptr<Plato::OptimalityCriteriaSubProblem<ScalarType, OrdinalType>> & aSubProblem) :
            mPrintDiagnostics(false),
            mOutputStream(),
            mMaxNumIterations(50),
            mNumIterationsDone(0),
            mProblemUpdateFrequency(0), // zero for no update
            mFeasibilityTolerance(1e-5),
            mControlStagnationTolerance(1e-2),
            mObjectiveGradientTolerance(1e-8),
            mObjectiveStagnationTolerance(1e-5),
            mOutputData(),
            mBounds(nullptr),
            mDataMng(aDataMng),
            mStageMng(aStageMng),
            mSubProblem(aSubProblem)
    {
        this->initialize();
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~OptimalityCriteria()
    {
    }

    /******************************************************************************//**
     * @brief Enable output of diagnostics (i.e. optimization problem status)
    **********************************************************************************/
    void enableDiagnostics()
    {
        mPrintDiagnostics = true;
    }

    //! specify outer loop frequency of update. Zero if no updates
    void setProblemUpdateFrequency(const OrdinalType& aInput)
    {
        mProblemUpdateFrequency = aInput;
    }

    /******************************************************************************//**
     * @brief Get number of optimization iterations done.
     * @return number of optimization iterations done
    **********************************************************************************/
    OrdinalType getNumIterationsDone() const
    {
        return (mNumIterationsDone);
    }

    /******************************************************************************//**
     * @brief Get max number of optimization iterations.
     * @return max number of optimization iterations
    **********************************************************************************/
    OrdinalType getMaxNumIterations() const
    {
        return (mMaxNumIterations);
    }

    /******************************************************************************//**
     * @brief Get control stagnation tolerance
     * @return control stagnation tolerance
    **********************************************************************************/
    ScalarType getControlStagnationTolerance() const
    {
        return (mControlStagnationTolerance);
    }

    /******************************************************************************//**
     * @brief Get feasibility tolerance
     * @return feasibility tolerance
    **********************************************************************************/
    ScalarType getFeasibilityTolerance() const
    {
        return (mFeasibilityTolerance);
    }

    /******************************************************************************//**
     * @brief Get objective gradient tolerance
     * @return objective gradient tolerance
    **********************************************************************************/
    ScalarType getObjectiveGradientTolerance() const
    {
        return (mObjectiveGradientTolerance);
    }

    /******************************************************************************//**
     * @brief Get objective stagnation tolerance
     * @return objective stagnation tolerance
    **********************************************************************************/
    ScalarType getObjectiveStagnationTolerance() const
    {
        return (mObjectiveStagnationTolerance);
    }

    /******************************************************************************//**
     * @brief Set maximum number of optimization iterations
     * @param [in] aInput maximum number of optimization iterations
    **********************************************************************************/
    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumIterations = aInput;
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
     * @brief Set feasibility tolerance
     * @param [in] aInput feasibility tolerance
    **********************************************************************************/
    void setFeasibilityTolerance(const ScalarType & aInput)
    {
        mFeasibilityTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set objective gradient tolerance
     * @param [in] aInput objective gradient tolerance
    **********************************************************************************/
    void setObjectiveGradientTolerance(const ScalarType & aInput)
    {
        mObjectiveGradientTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Set objective stagnation tolerance
     * @param [in] aInput objective stagnation tolerance
    **********************************************************************************/
    void setObjectiveStagnationTolerance(const ScalarType & aInput)
    {
        mObjectiveStagnationTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Return reference to data manager
     * @return optimality criteria algorithm data manager
    **********************************************************************************/
    const Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType> & getDataMng() const
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

        mNumIterationsDone = 0;
        while(1)
        {
            this->updateProblem();
            mStageMng->update(*mDataMng);

            this->computeStoppingMetrics();
            this->outputDiagnostics();

            if(this->isStoppingCriteriaSatisfied() == true)
            {
                this->outputMyStoppingCriterion();
                this->closeOutputFile();
                break;
            }

            this->storeCurrentStageData();
            mSubProblem->solve(*mDataMng, *mStageMng);

            mNumIterationsDone++;
        }
    }

private:
    /******************************************************************************//**
     * @brief Invoke update problem stage if needed.
     **********************************************************************************/
    void updateProblem()
    {
        // don't update if frequency == 0
        bool tHaveProblemUpdateFrequency = (0 < mProblemUpdateFrequency);
        if(!tHaveProblemUpdateFrequency)
        {
            return;
        }

        // only update at requested frequency
        bool tIsIterationToUpdate = (mNumIterationsDone % mProblemUpdateFrequency) == 0;
        if(!tIsIterationToUpdate)
        {
            return;
        }

        // invoke advance continuation
        mStageMng->updateProblem(*mDataMng);
    }

    /******************************************************************************//**
     * @brief Compute metrics used to decide if algorithm converged.
    **********************************************************************************/
    void computeStoppingMetrics()
    {
        mDataMng->computeMaxInequalityValue();
        mDataMng->computeNormObjectiveGradient();
        mDataMng->computeControlStagnationMeasure();
        mDataMng->computeObjectiveStagnationMeasure();
    }

    /******************************************************************************//**
     * @brief Open output file
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
                mOutputStream.open("plato_optimality_criteria_diagnostics.txt");
                Plato::print_oc_diagnostics_header(mOutputData, mOutputStream);
            }
        }
    }

    /******************************************************************************//**
     * @brief Close output file
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
                Plato::algorithm::stop_t tCriterion = mDataMng->getStopCriterion();
                Plato::get_stop_criterion(tCriterion, tReason);
                mOutputStream << tReason.c_str();
            }
        }
    }

    /******************************************************************************//**
     * @brief Initialize bounds based on memory space
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
            THROWERR(tErrorMsg.what())
        }

        Plato::MemorySpace::type_t tMemorySpace = mDataMng->getMemorySpace();
        switch(tMemorySpace)
        {
            case Plato::MemorySpace::DEVICE:
            {
                mBounds = std::make_shared<Plato::DeviceBounds<ScalarType, OrdinalType>>();
                break;
            }
            default:
            case Plato::MemorySpace::HOST:
            {
                mBounds = std::make_shared<Plato::HostBounds<ScalarType, OrdinalType>>();
                break;
            }
        }
    }

    /******************************************************************************//**
     * @brief Check if a stopping criterion is met.
     * @return flag instructing algorithm to stop, true or false
    **********************************************************************************/
    bool isStoppingCriteriaSatisfied()
    {
        bool tStop = false;
        ScalarType tMaxInequalityValue = mDataMng->getMaxInequalityValue();
        ScalarType tNormObjectiveGradient = mDataMng->getNormObjectiveGradient();
        ScalarType tControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
        ScalarType tObjectiveStagnationMeasure = mDataMng->getObjectiveStagnationMeasure();

        if(this->getNumIterationsDone() >= this->getMaxNumIterations())
        {
            tStop = true;
            mDataMng->setStopCriterion(Plato::algorithm::MAX_NUMBER_ITERATIONS);
        }
        else if(tControlStagnationMeasure < this->getControlStagnationTolerance())
        {
            tStop = true;
            mDataMng->setStopCriterion(Plato::algorithm::CONTROL_STAGNATION);
        }
        else if(tObjectiveStagnationMeasure < this->getObjectiveStagnationTolerance())
        {
            tStop = true;
            mDataMng->setStopCriterion(Plato::algorithm::OBJECTIVE_STAGNATION);
        }
        else if(tNormObjectiveGradient < this->getObjectiveGradientTolerance()
                && tMaxInequalityValue < this->getFeasibilityTolerance())
        {
            tStop = true;
            mDataMng->setStopCriterion(Plato::algorithm::OPTIMALITY_AND_FEASIBILITY);
        }

        return (tStop);
    }

    /******************************************************************************//**
     * @brief Store primal information associated with current optimization iteration
    **********************************************************************************/
    void storeCurrentStageData()
    {
        const ScalarType tObjectiveValue = mDataMng->getCurrentObjectiveValue();
        mDataMng->setPreviousObjectiveValue(tObjectiveValue);

        const OrdinalType tNumVectors = mDataMng->getNumControlVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tControl = mDataMng->getCurrentControl(tVectorIndex);
            mDataMng->setPreviousControl(tVectorIndex, tControl);
        }

        const OrdinalType tNumConstraints = mDataMng->getNumConstraints();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            ScalarType tMyCurrentConstraintValue = mDataMng->getCurrentConstraintValues(tConstraintIndex);
            mDataMng->setPreviousConstraintValue(tConstraintIndex, tMyCurrentConstraintValue);
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
            THROWERR(tErrorMsg.what())
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
            THROWERR(tErrorMsg.what())
        }

        const Plato::MultiVector<ScalarType, OrdinalType> & tControl = mDataMng->getCurrentControl();
        const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getControlUpperBounds();
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tWorkMultiVector = tControl.create();
        Plato::update(static_cast<ScalarType>(1), tControl, static_cast<ScalarType>(0), *tWorkMultiVector);
        mBounds->project(tLowerBounds, tUpperBounds, *tWorkMultiVector);
        mDataMng->setCurrentControl(*tWorkMultiVector);
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
            mOutputData.mObjFuncValue = mDataMng->getCurrentObjectiveValue();
            mOutputData.mNormObjFuncGrad = mDataMng->getNormObjectiveGradient();
            mOutputData.mObjFuncCount = mDataMng->getNumObjectiveFunctionEvaluations();
            mOutputData.mControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
            mOutputData.mObjectiveStagnationMeasure = mDataMng->getObjectiveStagnationMeasure();

            const OrdinalType tNumConstraints = mDataMng->getNumConstraints();
            for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
            {
                mOutputData.mConstraints[tIndex] = mDataMng->getCurrentConstraintValues(tIndex);
            }

            Plato::print_oc_diagnostics(mOutputData, mOutputStream);
        }
    }

private:
    bool mPrintDiagnostics; /*!< flag: true = enable diagnostics & false = disable diagnostics */
    std::ofstream mOutputStream; /*!< output file */

    OrdinalType mMaxNumIterations; /*!< maximum number of outer optimization iterations */
    OrdinalType mNumIterationsDone; /*!< number of outer optimization iterations done */
    OrdinalType mProblemUpdateFrequency; /*!< problem update frequency */

    ScalarType mFeasibilityTolerance; /*!< feasibility tolerance */
    ScalarType mControlStagnationTolerance; /*!< control stagnation tolerance */
    ScalarType mObjectiveGradientTolerance; /*!< optimality tolerance */
    ScalarType mObjectiveStagnationTolerance; /*!< objective stagnation tolerance */

    Plato::OutputDataOC<ScalarType, OrdinalType> mOutputData; /*!< output data struct */
    std::shared_ptr<Plato::BoundsBase<ScalarType, OrdinalType>> mBounds; /*!< box constraint interface */
    std::shared_ptr<Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType>> mDataMng; /*!< optimization data manager */
    std::shared_ptr<Plato::OptimalityCriteriaStageMngBase<ScalarType, OrdinalType>> mStageMng; /*!< criteria evaluation manager */
    std::shared_ptr<Plato::OptimalityCriteriaSubProblem<ScalarType, OrdinalType>> mSubProblem; /*!< optimality criteria udate manager */

private:
    OptimalityCriteria(const Plato::OptimalityCriteria<ScalarType, OrdinalType>&);
    Plato::OptimalityCriteria<ScalarType, OrdinalType> & operator=(const Plato::OptimalityCriteria<ScalarType, OrdinalType>&);
};
// class OptimalityCriteria

} // namespace Plato

#endif /* PLATO_OPTIMALITYCRITERIA_HPP_ */
