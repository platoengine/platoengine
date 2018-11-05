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
 * Plato_KelleySachsAugmentedLagrangianLightInterface.hpp
 *
 *  Created on: Oct 3, 2018
 */

#pragma once

#include "Plato_AugmentedLagrangian.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Output structure for Kelley-Sachs Augmented Lagrangian (KSAL) trust region algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmOutputsKSAL
{
    OrdinalType mNumOuterIter; /*!< number of outer iterations */
    OrdinalType mNumObjFuncEval; /*!< number of objective function evaluations */
    OrdinalType mNumObjGradEval; /*!< number of objective gradient evaluations */

    ScalarType mObjFuncValue; /*!< objective function value */
    ScalarType mNormObjFuncGrad; /*!< norm of the objective function gradient */
    ScalarType mActualReduction; /*!< actual reduction */
    ScalarType mPenaltyParameter; /*!< penalty parameter */
    ScalarType mStationarityMeasure; /*!< norm of the descent direction */
    ScalarType mCurrentTrustRegionRadius; /*!< current trust region radius */
    ScalarType mControlStagnationMeasure; /*!< norm of the difference between two subsequent control fields */
    ScalarType mObjectiveStagnationMeasure; /*!< measures stagnation in two subsequent objective function evaluations */

    std::string mStopCriterion; /*!< stopping criterion */

    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mConstraints; /*!< constraint values */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mSolution; /*!< optimal solution */
};
// struct AlgorithmOutputsKSAL

/******************************************************************************//**
 * @brief Input structure for Kelley-Sachs Augmented Lagrangian (KSAL) trust region algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmInputsKSAL
{
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    AlgorithmInputsKSAL() :
            mHaveHessian(true),
            mPrintDiagnostics(false),
            mDisablePostSmoothing(false),
            mMaxNumOuterIter(50),
            mMaxNumAugLagSubProbIter(5),
            mMaxNumPostSmoothingIter(5),
            mMaxTrustRegionSubProblemIter(25),
            mMaxTrustRegionRadius(1e2),
            mMinTrustRegionRadius(1e-6),
            mTrustRegionExpansionFactor(4),
            mTrustRegionContractionFactor(0.75),
            mInitialPenaltyParameter(0.05),
            mPenaltyParameterScaleFactor(2),
            mGradientTolerance(1e-4),
            mFeasibilityTolerance(1e-4),
            mStationarityTolerance(1e-4),
            mActualReductionTolerance(1e-8),
            mControlStagnationTolerance(1e-16),
            mObjectiveStagnationTolerance(1e-8),
            mActualOverPredictedReductionMidBound(0.25),
            mActualOverPredictedReductionLowerBound(0.10),
            mActualOverPredictedReductionUpperBound(0.75),
            mCommWrapper(),
            mMemorySpace(Plato::MemorySpace::HOST),
            mDual(nullptr),
            mLowerBounds(nullptr),
            mUpperBounds(nullptr),
            mInitialGuess(nullptr),
            mReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>())
    {
        mCommWrapper.useDefaultComm();
    }

    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    virtual ~AlgorithmInputsKSAL()
    {
    }

    bool mHaveHessian; /*!< flag to specify that Hessian information is available (default=true) */
    bool mPrintDiagnostics; /*!< flag to enable problem statistics output (default=false) */
    bool mDisablePostSmoothing; /*!< flag to disable post smoothing operation (default=false) */

    OrdinalType mMaxNumOuterIter; /*!< maximum number of outer iterations */
    OrdinalType mMaxNumAugLagSubProbIter; /*!< maximum number of augmented Lagrangian subproblem iterations */
    OrdinalType mMaxNumPostSmoothingIter; /*!< maximum number of outer line search iterations */
    OrdinalType mMaxTrustRegionSubProblemIter; /*!< maximum number of trust region sub problem iterations */

    ScalarType mMaxTrustRegionRadius; /*!< maximum trust region radius */
    ScalarType mMinTrustRegionRadius; /*!< minimum trust region radius */
    ScalarType mTrustRegionExpansionFactor; /*!< trust region radius expansion factor */
    ScalarType mTrustRegionContractionFactor; /*!< trust region radius contraction factor */

    ScalarType mInitialPenaltyParameter; /*!< initial penalty parameter */
    ScalarType mPenaltyParameterScaleFactor; /*!< penalty parameter scale factor */

    ScalarType mGradientTolerance; /*!< gradient norm tolerance */
    ScalarType mFeasibilityTolerance; /*!< feasibility tolerance */
    ScalarType mStationarityTolerance; /*!< descent direction norm tolerance */
    ScalarType mActualReductionTolerance; /*!< actual reduction tolerance */
    ScalarType mControlStagnationTolerance; /*!< control stagnation tolerance */
    ScalarType mObjectiveStagnationTolerance; /*!< objective function stagnation tolerance */
    ScalarType mActualOverPredictedReductionMidBound; /*!< actual over predicted reduction middle bound */
    ScalarType mActualOverPredictedReductionLowerBound; /*!< actual over predicted reduction lower bound */
    ScalarType mActualOverPredictedReductionUpperBound; /*!< actual over predicted reduction upper bound */

    Plato::CommWrapper mCommWrapper; /*!< distributed memory communication wrapper */
    Plato::MemorySpace::type_t mMemorySpace; /*!< memory space: HOST (default) OR DEVICE */

    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mDual; /*!< Lagrange multipliers */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mLowerBounds; /*!< lower bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mUpperBounds; /*!< upper bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mInitialGuess; /*!< initial guess */

    /*!< operations which require communication across processors, e.g. max, min, global sum */
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mReductionOperations;
};
// struct AlgorithmInputsKSAL

/******************************************************************************//**
 * @brief Set Kelley-Sachs Augmented Lagrangian (KSAL) trust region algorithm inputs
 * @param [in] aInputs Kelley-Sachs Augmented Lagrangian trust region algorithm inputs
 * @param [in,out] aAlgorithm Kelley-Sachs Augmented Lagrangian trust region algorithm interface
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_ksal_algorithm_inputs(const Plato::AlgorithmInputsKSAL<ScalarType, OrdinalType> & aInputs,
                                      Plato::AugmentedLagrangian<ScalarType, OrdinalType> & aAlgorithm)
{
    if(aInputs.mPrintDiagnostics == true)
    {
        aAlgorithm.enableDiagnostics();
    }

    if(aInputs.mDisablePostSmoothing == true)
    {
        aAlgorithm.disablePostSmoothing();
    }

    aAlgorithm.setHaveHessian(aInputs.mHaveHessian);
    aAlgorithm.setPenaltyParameter(aInputs.mInitialPenaltyParameter);
    aAlgorithm.setPenaltyParameterScaleFactor(aInputs.mPenaltyParameterScaleFactor);

    aAlgorithm.setMaxNumOuterIterations(aInputs.mMaxNumOuterIter);
    aAlgorithm.setMaxNumAugLagSubProbIter(aInputs.mMaxNumAugLagSubProbIter);
    aAlgorithm.setMaxNumPostSmoothingIter(aInputs.mMaxNumPostSmoothingIter);
    aAlgorithm.setMaxNumTrustRegionSubProblemIterations(aInputs.mMaxTrustRegionSubProblemIter);

    aAlgorithm.setMaxTrustRegionRadius(aInputs.mMaxTrustRegionRadius);
    aAlgorithm.setMinTrustRegionRadius(aInputs.mMinTrustRegionRadius);
    aAlgorithm.setTrustRegionExpansion(aInputs.mTrustRegionExpansionFactor);
    aAlgorithm.setTrustRegionContraction(aInputs.mTrustRegionContractionFactor);

    aAlgorithm.setGradientTolerance(aInputs.mGradientTolerance);
    aAlgorithm.setFeasibilityTolerance(aInputs.mFeasibilityTolerance);
    aAlgorithm.setStationarityTolerance(aInputs.mStationarityTolerance);
    aAlgorithm.setActualReductionTolerance(aInputs.mActualReductionTolerance);
    aAlgorithm.setControlStagnationTolerance(aInputs.mControlStagnationTolerance);
    aAlgorithm.setObjectiveStagnationTolerance(aInputs.mObjectiveStagnationTolerance);
}
// function set_ksal_algorithm_inputs

/******************************************************************************//**
 * @brief Set Kelley-Sachs Augmented Lagrangian (KSAL) trust region algorithm outputs
 * @param [in] aAlgorithm Kelley-Sachs Augmented Lagrangian trust region algorithm interface
 * @param [in,out] aOutputs Kelley-Sachs Augmented Lagrangian trust region algorithm outputs
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_ksal_algorithm_outputs(const Plato::AugmentedLagrangian<ScalarType, OrdinalType> & aAlgorithm,
                                       Plato::AlgorithmOutputsKSAL<ScalarType, OrdinalType> & aOutputs)
{
    aOutputs.mNumOuterIter = aAlgorithm.getNumOuterIterDone();
    aOutputs.mNumObjFuncEval = aAlgorithm.getDataMng().getNumObjectiveFunctionEvaluations();
    aOutputs.mNumObjGradEval = aAlgorithm.getDataMng().getNumObjectiveGradientEvaluations();

    aOutputs.mObjFuncValue = aAlgorithm.getDataMng().getCurrentObjectiveFunctionValue();
    aOutputs.mNormObjFuncGrad = aAlgorithm.getDataMng().getNormProjectedGradient();
    aOutputs.mActualReduction = aAlgorithm.getStepMng().getActualReduction();
    aOutputs.mPenaltyParameter = aAlgorithm.getStageMng().getPenaltyParameter();
    aOutputs.mStationarityMeasure = aAlgorithm.getDataMng().getStationarityMeasure();
    aOutputs.mCurrentTrustRegionRadius = aAlgorithm.getStepMng().getTrustRegionRadius();
    aOutputs.mControlStagnationMeasure = aAlgorithm.getDataMng().getControlStagnationMeasure();
    aOutputs.mObjectiveStagnationMeasure = aAlgorithm.getDataMng().getObjectiveStagnationMeasure();

    Plato::get_stop_criterion(aAlgorithm.getStoppingCriterion(), aOutputs.mStopCriterion);

    const Plato::MultiVector<ScalarType, OrdinalType> & tSolution = aAlgorithm.getDataMng().getCurrentControl();
    aOutputs.mSolution = tSolution.create();
    Plato::update(static_cast<ScalarType>(1), tSolution, static_cast<ScalarType>(0), *aOutputs.mSolution);

    const OrdinalType tDualVectorIndex = 0;
    const Plato::Vector<ScalarType, OrdinalType> & tDual = aAlgorithm.getDataMng().getDual(tDualVectorIndex);
    aOutputs.mConstraints = tDual.create();
    aAlgorithm.getStageMng().getCurrentConstraintValues(tDualVectorIndex, *aOutputs.mConstraints);
}
// function set_ksal_algorithm_outputs

/******************************************************************************//**
 * @brief Kelley-Sachs Augmented Lagrangian (KSAL) trust region algorithm interface
 * @param [in] aObjective user-defined objective function
 * @param [in] aConstraints user-defined list of constraints
 * @param [in] aInputs Kelley-Sachs Augmented Lagrangian trust region algorithm input structure
 * @param [in,out] aOutputs Kelley-Sachs Augmented Lagrangian trust region algorithm output structure
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void solve_ksal(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aObjective,
                       const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aConstraints,
                       const Plato::AlgorithmInputsKSAL<ScalarType, OrdinalType> & aInputs,
                       Plato::AlgorithmOutputsKSAL<ScalarType, OrdinalType> & aOutputs)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tDataFactory;
    tDataFactory = std::make_shared<Plato::DataFactory<ScalarType, OrdinalType>>(aInputs.mMemorySpace);
    tDataFactory->setCommWrapper(aInputs.mCommWrapper);
    tDataFactory->allocateDual(*aInputs.mDual);
    tDataFactory->allocateControl(*aInputs.mInitialGuess);
    tDataFactory->allocateControlReductionOperations(*aInputs.mReductionOperations);

    // ********* ALLOCATE AUGMENTED LAGRANGIAN ALGORITHM *********
    Plato::AugmentedLagrangian<ScalarType, OrdinalType> tAlgorithm(aObjective, aConstraints, tDataFactory);
    tAlgorithm.setInitialGuess(*aInputs.mInitialGuess);
    tAlgorithm.setControlLowerBounds(*aInputs.mLowerBounds);
    tAlgorithm.setControlUpperBounds(*aInputs.mUpperBounds);

    // ********* SOLVE OPTIMIZATION PROBLEM AND SAVE SOLUTION *********
    Plato::set_ksal_algorithm_inputs(aInputs, tAlgorithm);
    tAlgorithm.solve();
    Plato::set_ksal_algorithm_outputs(tAlgorithm, aOutputs);
}
// function solve_ksal

} // namespace Plato
