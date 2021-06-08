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
 * Plato_AugmentedLagrangian.hpp
 *
 *  Created on: Nov 4, 2018
 */

#pragma once

#include "Plato_Macros.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_OptimizersIO.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_AugmentedLagrangianStageMng.hpp"
#include "Plato_KelleySachsBoundConstrained.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class AugmentedLagrangian
{
public:
    /******************************************************************************//**
     * @brief Default constructor
     * @param [in] aObjective interface to objective function
     * @param [in] aConstraints interface to inequality constraints
     * @param [in] aDataFactory factory for linear algebra data structure
    **********************************************************************************/
    AugmentedLagrangian(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aObjective,
                        const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aConstraints,
                        const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory) :
            mPrintDiagnostics(false),
            mNumAugLagIter(0),
            mMaxNumAugLagIter(100),
            mMaxNumAugLagSubProbIter(5),
            mObjFuncStagnation(std::numeric_limits<ScalarType>::max()),
            mControlStagnation(std::numeric_limits<ScalarType>::max()),
            mCurrentObjFuncValue(0.0),
            mPreviousObjFuncValue(0.0),
            mOptimalityTolerance(1e-4),
            mControlStagnationTol(std::numeric_limits<ScalarType>::epsilon()),
            mActualReductionTol(std::numeric_limits<ScalarType>::epsilon()),
            mFeasibilityTolerance(1e-4),
            mDynamicAugLagProbTolerance(5e-2),
            mDynamicFeasibilityTolerance(1e-1),
            mStoppingCriterion(Plato::algorithm::stop_t::NOT_CONVERGED),
            mOutputData(),
            mCurrentControl(aDataFactory->control().create()),
            mPreviousControl(aDataFactory->control().create()),
            mControlReductionOperations(aDataFactory->getControlReductionOperations().create()),
            mDataMng(std::make_shared<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>>(aDataFactory)),
            mStageMng(std::make_shared<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>>(aDataFactory, aObjective, aConstraints)),
            mOptimizer(std::make_shared<Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType>>(aDataFactory, mDataMng, mStageMng))
    {
        this->initialize(aDataFactory.operator*());
    }

    /******************************************************************************//**
     * @brief Engine constructor
     * @param [in] aDataFactory factory for linear algebra data structure
     * @param [in] aDataMng linear algebra data manager
     * @param [in] aStageMng criteria evaluation manager
    **********************************************************************************/
    AugmentedLagrangian(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory,
                        const std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> & aDataMng,
                        const std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>> & aStageMng) :
            mPrintDiagnostics(false),
            mNumAugLagIter(0),
            mMaxNumAugLagIter(100),
            mMaxNumAugLagSubProbIter(5),
            mObjFuncStagnation(std::numeric_limits<ScalarType>::max()),
            mControlStagnation(std::numeric_limits<ScalarType>::max()),
            mCurrentObjFuncValue(0.0),
            mPreviousObjFuncValue(0.0),
            mOptimalityTolerance(1e-4),
            mControlStagnationTol(std::numeric_limits<ScalarType>::epsilon()),
            mActualReductionTol(std::numeric_limits<ScalarType>::epsilon()),
            mFeasibilityTolerance(1e-4),
            mDynamicAugLagProbTolerance(5e-2),
            mDynamicFeasibilityTolerance(1e-1),
            mStoppingCriterion(Plato::algorithm::stop_t::NOT_CONVERGED),
            mOutputData(),
            mCurrentControl(aDataFactory->control().create()),
            mPreviousControl(aDataFactory->control().create()),
            mControlReductionOperations(aDataFactory->getControlReductionOperations().create()),
            mDataMng(aDataMng),
            mStageMng(aStageMng),
            mOptimizer(std::make_shared<Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType>>(aDataFactory, mDataMng, mStageMng))
    {
        this->initialize(aDataFactory.operator*());
    }

    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    ~AugmentedLagrangian()
    {
    }

    /******************************************************************************//**
     * @brief Specify if Hessian information is provided
     * @param [in] aInput true = information available; false = information is not available
    **********************************************************************************/
    void setMeanNorm(const bool & aInput)
    {
        if(aInput == true)
        {
            mDataMng->enableMeanNorm();
            mStageMng->enableMeanNorm();
        }
    }

    /******************************************************************************//**
     * @brief Enable mean norm calculations
     * @param [in] aInput true = enable; false = disable
    **********************************************************************************/
    void setHaveHessian(const bool & aInput)
    {
        mStageMng->setHaveHessian(aInput);
    }

    /******************************************************************************//**
     * @brief Set penalty parameter, /f$\mu\in(0,1)/f$
     * @param [in] aInput penalty parameter
    **********************************************************************************/
    void setPenaltyParameter(const ScalarType & aInput)
    {
        mStageMng->setPenaltyParameter(aInput);
    }

    /******************************************************************************//**
     * @brief Set lower bound on penalty parameter, /f$\mu_{lb}\in(0,1)/f$
     * @param [in] aInput lower bound on penalty parameter
    **********************************************************************************/
    void setPenaltyParameterLowerBound(const ScalarType & aInput)
    {
        mStageMng->setPenaltyParameterLowerBound(aInput);
    }

    /******************************************************************************//**
     * @brief Set maximum number of outer iterations
     * @param [in] aInput maximum number of outer iterations
    **********************************************************************************/
    void setMaxNumOuterIterations(const OrdinalType & aInput)
    {
        mMaxNumAugLagIter = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of augmented Lagrangian subproblem iterations
     * @param [in] aInput maximum number of augmented Lagrangian subproblem iterations
    **********************************************************************************/
    void setMaxNumAugLagSubProbIter(const OrdinalType & aInput)
    {
        mMaxNumAugLagSubProbIter = aInput;
        mOptimizer->setMaxNumIterations(mMaxNumAugLagSubProbIter);
    }

    /******************************************************************************//**
     * @brief Set maximum number of post-smoothing iterations
     * @param [in] aInput maximum number of post-smoothing iterations
    **********************************************************************************/
    void setMaxNumPostSmoothingIter(const OrdinalType & aInput)
    {
        mOptimizer->setMaxNumLineSearchIterations(aInput);
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
     * @brief Set stopping tolerance based on norm of the projected gradient.
     * @param [in] aInput stopping tolerance based on norm of the projected gradient
    **********************************************************************************/
    void setGradientTolerance(const ScalarType & aInput)
    {
        mOptimizer->setGradientTolerance(aInput);
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance based on norm of the projected descent direction.
     * @param [in] aInput stopping tolerance based on norm of the projected descent direction
    **********************************************************************************/
    void setStationarityTolerance(const ScalarType & aInput)
    {
        mOptimalityTolerance = aInput;
        mOptimizer->setStationarityTolerance(aInput);
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance based on the objective function stagnation metric
     * @param [in] aInput stopping tolerance based on the objective function stagnation
    **********************************************************************************/
    void setObjectiveStagnationTolerance(const ScalarType & aInput)
    {
        mOptimizer->setObjectiveStagnationTolerance(aInput);
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance based on the control stagnation metric
     * @param [in] aInput stopping tolerance based on the control stagnation
    **********************************************************************************/
    void setControlStagnationTolerance(const ScalarType & aInput)
    {
        mControlStagnationTol = aInput;
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance based on the augmented Lagrangian actual reduction metric
     * @param [in] aInput stopping tolerance based on the augmented Lagrangian actual reduction metric
    **********************************************************************************/
    void setAugLagActualReductionTolerance(const ScalarType & aInput)
    {
        mActualReductionTol = aInput;
    }

    /******************************************************************************//**
     * @brief Set stopping tolerance based on minimum actual reduction
     * @param [in] aInput stopping tolerance based on minimum actual reduction
    **********************************************************************************/
    void setActualReductionTolerance(const ScalarType & aInput)
    {
        mOptimizer->setActualReductionTolerance(aInput);
    }

    /******************************************************************************//**
     * @brief Set actual over predicted reduction middle bound
     * @param [in] aInput actual over predicted reduction middle bound
    **********************************************************************************/
    void setActualOverPredictedReductionMidBound(const ScalarType & aInput)
    {
        mOptimizer->setActualOverPredictedReductionMidBound(aInput);
    }

    /******************************************************************************//**
     * @brief Set actual over predicted reduction lower bound
     * @param [in] aInput actual over predicted reduction lower bound
    **********************************************************************************/
    void setActualOverPredictedReductionLowerBound(const ScalarType & aInput)
    {
        mOptimizer->setActualOverPredictedReductionLowerBound(aInput);
    }

    /******************************************************************************//**
     * @brief Set actual over predicted reduction upper bound
     * @param [in] aInput actual over predicted reduction upper bound
    **********************************************************************************/
    void setActualOverPredictedReductionUpperBound(const ScalarType & aInput)
    {
        mOptimizer->setActualOverPredictedReductionUpperBound(aInput);
    }

    /******************************************************************************//**
     * @brief Set minimum trust region radius
     * @param [in] aInput minimum trust region radius
    **********************************************************************************/
    void setMinTrustRegionRadius(const ScalarType & aInput)
    {
        mOptimizer->setMinTrustRegionRadius(aInput);
    }

    /******************************************************************************//**
     * @brief Set maximum trust region radius
     * @param [in] aInput maximum trust region radius
    **********************************************************************************/
    void setMaxTrustRegionRadius(const ScalarType & aInput)
    {
        mOptimizer->setMaxTrustRegionRadius(aInput);
    }

    /******************************************************************************//**
     * @brief Set trust region expansion factor
     * @param [in] aInput trust region expansion factor
    **********************************************************************************/
    void setTrustRegionExpansion(const ScalarType & aInput)
    {
        assert(aInput > static_cast<ScalarType>(0));
        assert(aInput > static_cast<ScalarType>(1));
        mOptimizer->setTrustRegionExpansion(aInput);
    }

    /******************************************************************************//**
     * @brief Set trust region contraction factor
     * @param [in] aInput trust region contraction factor
    **********************************************************************************/
    void setTrustRegionContraction(const ScalarType & aInput)
    {
        assert(aInput > static_cast<ScalarType>(0));
        assert(aInput < static_cast<ScalarType>(1));
        mOptimizer->setTrustRegionContraction(aInput);
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
     * @brief Set maximum number of trust region subproblem iterations
     * @param [in] aInput maximum number of trust region subproblem iterations
    **********************************************************************************/
    void setMaxNumTrustRegionSubProblemIterations(const OrdinalType & aInput)
    {
        mOptimizer->setMaxNumTrustRegionSubProblemIterations(aInput);
    }

    /******************************************************************************//**
     * @brief Set control initial guess
     * @param [in] aInput initial guess
    **********************************************************************************/
    void setInitialGuess(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        mDataMng->setInitialGuess(aInput);
    }

    /******************************************************************************//**
     * @brief Set control lower bounds
     * @param [in] aInput lower bounds
    **********************************************************************************/
    void setControlLowerBounds(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        mDataMng->setControlLowerBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Set control upper bounds
     * @param [in] aInput upper bounds
    **********************************************************************************/
    void setControlUpperBounds(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        mDataMng->setControlUpperBounds(aInput);
    }

    /******************************************************************************//**
     * @brief Enable output of diagnostics (i.e. optimization problem status)
    **********************************************************************************/
    void enableDiagnostics()
    {
        mPrintDiagnostics = true;
        mOptimizer->enableDiagnostics();
    }

    /******************************************************************************//**
     * @brief Disable post smoothing operation
    **********************************************************************************/
    void disablePostSmoothing()
    {
        mOptimizer->disablePostSmoothing();
    }

    /******************************************************************************//**
     * @brief Set criteria Hessians to LBFGS.
     * @param [in] aMaxMemory memory size (default = 8)
    **********************************************************************************/
    void setCriteriaHessiansLBFGS(OrdinalType aMaxMemory = 8)
    {
        mStageMng->setHaveHessian(true);
        mStageMng->setObjectiveHessianLBFGS(aMaxMemory);
        mStageMng->setConstraintHessiansLBFGS(aMaxMemory);
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
     * @brief Return reference to stage manager
     * @return trust region algorithm's stage manager
    **********************************************************************************/
    const Plato::AugmentedLagrangianStageMng<ScalarType,OrdinalType> & getStageMng() const
    {
        return (*mStageMng);
    }

    /******************************************************************************//**
     * @brief Return reference to trust region step manager
     * @return trust region step manager
    **********************************************************************************/
    const Plato::KelleySachsStepMng<ScalarType,OrdinalType> & getStepMng() const
    {
        return (mOptimizer->getStepMng());
    }

    /******************************************************************************//**
     * @brief Get control solution
     * param [in/out] aSolution solution to optimization problem
    **********************************************************************************/
    void getSolution(Plato::MultiVector<ScalarType, OrdinalType> &aSolution) const
    {
        const Plato::MultiVector<ScalarType, OrdinalType> & tSolution = mDataMng->getCurrentControl();
        Plato::update(static_cast<ScalarType>(1), tSolution, static_cast<ScalarType>(0), aSolution);
    }

    /******************************************************************************//**
     * @brief Return number of outer optimization iterations
     * @return number of outer optimization iterations
    **********************************************************************************/
    OrdinalType getNumOuterIterDone() const
    {
        return (mNumAugLagIter);
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
     * @brief Reset optimizer parameters to initial values, i.e. values at iteration 0.
    **********************************************************************************/
    void resetParameters()
    {
        mNumAugLagIter = 0;
        //mStageMng->resetParameters();
        mOptimizer->resetParameters();
    }

    /******************************************************************************//**
     * @brief Solve constrained optimization problem
    **********************************************************************************/
    void solve()
    {
        this->openOutputFile();
        this->outputDiagnostics();

        while(true)
        {
            mNumAugLagIter++;
            // solve augmented Lagrangian problem
            mOptimizer->setStationarityTolerance(mDynamicAugLagProbTolerance);
            mOptimizer->solve(mOutputStream);
            mOptimizer->useCurrentTrustRegionRadius();
            this->cacheState();
            // check stopping criteria
            bool tStop = this->checkStoppingCriteria();
            // output diagnostics
            this->outputDiagnostics();
            if(tStop == true)
            {
                this->outputMyStoppingCriterion();
                this->closeOutputFile();
                break;
            }
        }
    }

private:
    /******************************************************************************//**
     * @brief Initialize default parameters
     * @param [in] aDataFactory factory for linear algebra data structure
    **********************************************************************************/
    void initialize(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory)
    {
        const OrdinalType tVectorIndex = 0;
        mOptimizer->setMaxNumIterations(mMaxNumAugLagSubProbIter);
        mControlWorkVector = aDataFactory.control(tVectorIndex).create();
        mDynamicAugLagProbTolerance = std::pow(mStageMng->getPenaltyParameter(), static_cast<ScalarType>(0.05));
    }

    /******************************************************************************//**
     * @brief Check stopping criteria
     * @return stopping criterion met, yes or no
    **********************************************************************************/
    bool checkStoppingCriteria()
    {
        bool tStop = false;

        mStageMng->computeNormConstraintVector();
        const ScalarType tNormConstraint = mStageMng->getNormConstraintVector();
        if(tNormConstraint <= mDynamicFeasibilityTolerance)
        {
            tStop = this->testConvergence();
            mStageMng->updateLagrangeMultipliers();
            this->updateDynamicTolerancesBasedOnConvergenceCheck();
        }
        else
        {
            mStageMng->updatePenaltyParameter();
            this->updateDynamicTolerancesBasedOnPenaltyUpdate();
        }

        if(tStop == false)
        {
            tStop = this->checkSecondaryStoppingCriteria();
        }

        return (tStop);
    }

    /******************************************************************************//**
     * @brief Update dynamic optimality and feasibility tolerances due to convergence check
    **********************************************************************************/
    void updateDynamicTolerancesBasedOnConvergenceCheck()
    {
        const ScalarType tPenalty = static_cast<ScalarType>(1) / mStageMng->getPenaltyParameter();
        mDynamicAugLagProbTolerance = mDynamicAugLagProbTolerance / tPenalty;
        mDynamicFeasibilityTolerance = mDynamicFeasibilityTolerance / std::pow(tPenalty, static_cast<ScalarType>(0.9));
    }

    /******************************************************************************//**
     * @brief Update dynamic optimality and feasibility tolerances due to penalty update
    **********************************************************************************/
    void updateDynamicTolerancesBasedOnPenaltyUpdate()
    {
        const ScalarType tPenalty = static_cast<ScalarType>(1) / mStageMng->getPenaltyParameter();
        mDynamicAugLagProbTolerance = static_cast<ScalarType>(1) / tPenalty;
        mDynamicFeasibilityTolerance = static_cast<ScalarType>(1) / std::pow(tPenalty, static_cast<ScalarType>(0.1));
    }

    /******************************************************************************//**
     * @brief Test for convergence
     * @return convergence criterion satisfied, yes or no
    **********************************************************************************/
    bool testConvergence()
    {
        bool tStop = false;

        const ScalarType tNormConstraint = mStageMng->getNormConstraintVector();
        bool tFeasibilitySatisfied = tNormConstraint <= mFeasibilityTolerance;
        const ScalarType tStationarityMeasure = mDataMng->getStationarityMeasure();
        bool tOptimalitySatisfied = tStationarityMeasure <= mOptimalityTolerance;

        if(tFeasibilitySatisfied && tOptimalitySatisfied)
        {
            mStoppingCriterion = Plato::algorithm::OPTIMALITY_AND_FEASIBILITY;
            tStop = true;
        }

        return (tStop);
    }

    /******************************************************************************//**
     * @brief Check secondary stopping criteria
     * @return criterion satisfied, yes or no
    **********************************************************************************/
    bool checkSecondaryStoppingCriteria()
    {
        bool tStop = false;

        this->computeControlStagnationMeasure();
        mObjFuncStagnation = std::abs(mCurrentObjFuncValue - mPreviousObjFuncValue);

        if(mNumAugLagIter == mMaxNumAugLagIter)
        {
            mStoppingCriterion = Plato::algorithm::MAX_NUMBER_ITERATIONS;
            tStop = true;
        }
        else if(mControlStagnation <= mControlStagnationTol)
        {
            mStoppingCriterion = Plato::algorithm::CONTROL_STAGNATION;
            tStop = true;
        }
        else if(mObjFuncStagnation <= mActualReductionTol)
        {
            mStoppingCriterion = Plato::algorithm::ACTUAL_REDUCTION_TOLERANCE;
            tStop = true;
        }

        return (tStop);
    }

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
                mOptimizer->setNumConstraints(tNumConstraints);
                mOutputData.mConstraintValues.clear();
                mOutputData.mConstraintValues.resize(tNumConstraints);
                mOutputStream.open("plato_ksal_algorithm_diagnostics.txt");
                Plato::print_ksal_diagnostics_header(mOutputData, mOutputStream);
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
                Plato::get_stop_criterion(mStoppingCriterion, tReason);
                mOutputStream << tReason.c_str();
            }
        }
    }

    /******************************************************************************//**
     * @brief Print diagnostics for Kelley-Sachs augmented Lagrangian optimization algorithm
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
            mOutputData.mNumIter = mNumAugLagIter;
            mOutputData.mObjFuncCount = mDataMng->getNumObjectiveFunctionEvaluations();

            mOutputData.mPenalty = static_cast<ScalarType>(1) / mStageMng->getPenaltyParameter();
            mOutputData.mStationarityMeasure = mDataMng->getStationarityMeasure();
            mOutputData.mControlStagnationMeasure = mControlStagnation;
            mOutputData.mObjectiveStagnationMeasure = mObjFuncStagnation;

            mOutputData.mObjFuncValue = mStageMng->getCurrentObjectiveFunctionValue();
            mOutputData.mNormObjFuncGrad = mStageMng->getNormObjectiveFunctionGradient();
            mOutputData.mNormAugLagFuncGrad = mStageMng->getNormAugmentedLagrangianGradient();
            mOutputData.mAugLagFuncValue = mStageMng->getCurrentAugmenteLagrangianFunctionValue();

            const OrdinalType tCONSTRAINT_VECTOR_INDEX = 0;
            const OrdinalType tNumConstraints = mStageMng->getNumConstraints();
            for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
            {
                mOutputData.mConstraintValues[tIndex] =
                        mStageMng->getCurrentConstraintValues(tCONSTRAINT_VECTOR_INDEX, tIndex);
            }

            Plato::print_ksal_outer_diagnostics(mOutputData, mOutputStream);
        }
    }

    /******************************************************************************//**
     * @brief Cache current state data, which includes the objective function value and the controls.
    **********************************************************************************/
    void cacheState()
    {
        mPreviousObjFuncValue = mCurrentObjFuncValue;
        mCurrentObjFuncValue = mStageMng->getCurrentObjectiveFunctionValue();
        Plato::update(1., *mCurrentControl, 0., *mPreviousControl);
        Plato::update(1., mDataMng->getCurrentControl(), 0., *mCurrentControl);
    }
    
    /******************************************************************************//**
     * @brief Compute control stagnation measure, i.e. \f$ \max(x_{i+1}-x_i) \f$, where
     *        \f$ x \f$ is the vector of control variables.
    **********************************************************************************/
    void computeControlStagnationMeasure()
    {
        OrdinalType tNumVectors = mCurrentControl->getNumVectors();
        std::vector<ScalarType> tStorage(tNumVectors, std::numeric_limits<ScalarType>::min());
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyCurrentControl = mCurrentControl->operator[](tIndex);
            mControlWorkVector->update(1., tMyCurrentControl, 0.);
            const Plato::Vector<ScalarType, OrdinalType> & tMyPreviousControl = mPreviousControl->operator[](tIndex);
            mControlWorkVector->update(-1., tMyPreviousControl, 1.);
            mControlWorkVector->modulus();
            tStorage[tIndex] = mControlReductionOperations->max(*mControlWorkVector);
        }
        mControlStagnation = *std::max_element(tStorage.begin(), tStorage.end());
    }

private:
    bool mPrintDiagnostics; /*!< flag to enable problem statistics output (default=false) */
    std::ofstream mOutputStream;  /*!< output stream for diagnostics */

    OrdinalType mNumAugLagIter; /*!< number of augmented Lagrangian outer iterations */
    OrdinalType mMaxNumAugLagIter; /*!< maximum number of augmented Lagrangian outer iterations */
    OrdinalType mMaxNumAugLagSubProbIter; /*!< maximum number of augmented Lagrangian subproblem iterations */

    ScalarType mObjFuncStagnation;
    ScalarType mControlStagnation;
    ScalarType mCurrentObjFuncValue; /*!< current objective function ( f(x) ) value  */
    ScalarType mPreviousObjFuncValue; /*!< previous objective function ( f(x) ) value  */
    ScalarType mOptimalityTolerance; /*!< optimality tolerance - primary stopping tolerance */
    ScalarType mControlStagnationTol; /*!< control stagnation tolerance - secondary stopping tolerance */
    ScalarType mActualReductionTol; /*!< objective stagnation tolerance - secondary stopping tolerance */
    ScalarType mFeasibilityTolerance; /*!< feasibility tolerance - primary stopping tolerance */
    ScalarType mDynamicAugLagProbTolerance; /*!< dynamic optimality tolerance for augmented Lagrangian subproblem */
    ScalarType mDynamicFeasibilityTolerance; /*!< dynamic feasibility tolerance for augmented Lagrangian problem */

    Plato::algorithm::stop_t mStoppingCriterion; /*!< stopping criterion */
    Plato::OutputDataKSAL<ScalarType, OrdinalType> mOutputData; /*!< output data structure */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWorkVector;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousControl;

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductionOperations;

    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> mDataMng; /*!< state data manager */
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>> mStageMng; /*!< stage manager - manages criteria evaluations */
    std::shared_ptr<Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType>> mOptimizer; /*!< optimizer/solver for augmented Lagrangian subproblem */
};
// class AugmentedLagrangian

} // namespace Plato
