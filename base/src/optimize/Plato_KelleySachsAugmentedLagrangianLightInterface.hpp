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

#include "Plato_DataFactory.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_AugmentedLagrangianStageMng.hpp"
#include "Plato_KelleySachsAugmentedLagrangian.hpp"

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
            mPrintDiagnostics(false),
            mMaxNumOuterIter(500),
            mMaxTrustRegionSubProblemIter(25),
            mMaxNumOuterLineSearchUpdates(10),
            mMaxTrustRegionRadius(1e2),
            mMinTrustRegionRadius(1e-8),
            mTrustRegionExpansionFactor(2),
            mTrustRegionContractionFactor(0.75),
            mMinPenaltyParameter(1e-5),
            mInitialPenaltyParameter(0.1),
            mPenaltyParameterScaleFactor(1.2),
            mFeasibilityTolerance(1e-4),
            mOuterGradientTolerance(1e-8),
            mOuterStationarityTolerance(1e-8),
            mOuterActualReductionTolerance(1e-12),
            mOuterControlStagnationTolerance(1e-16),
            mOuterObjectiveStagnationTolerance(1e-8),
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

    bool mPrintDiagnostics; /*!< flag to enable problem statistics output (default=false) */

    OrdinalType mMaxNumOuterIter; /*!< maximum number of outer iterations */
    OrdinalType mMaxTrustRegionSubProblemIter; /*!< maximum number of trust region sub problem iterations */
    OrdinalType mMaxNumOuterLineSearchUpdates; /*!< maximum number of outer line search iterations */

    ScalarType mMaxTrustRegionRadius; /*!< maximum trust region radius */
    ScalarType mMinTrustRegionRadius; /*!< minimum trust region radius */
    ScalarType mTrustRegionExpansionFactor; /*!< trust region radius expansion factor */
    ScalarType mTrustRegionContractionFactor; /*!< trust region radius contraction factor */

    ScalarType mMinPenaltyParameter; /*!< minimum penalty parameter */
    ScalarType mInitialPenaltyParameter; /*!< initial penalty parameter */
    ScalarType mPenaltyParameterScaleFactor; /*!< penalty parameter scale factor */

    ScalarType mFeasibilityTolerance; /*!< feasibility tolerance */
    ScalarType mOuterGradientTolerance; /*!< gradient norm tolerance */
    ScalarType mOuterStationarityTolerance; /*!< descent direction norm tolerance */
    ScalarType mOuterActualReductionTolerance; /*!< actual reduction tolerance */
    ScalarType mOuterControlStagnationTolerance; /*!< control stagnation tolerance */
    ScalarType mOuterObjectiveStagnationTolerance; /*!< objective function stagnation tolerance */

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
 * @param [in,out] aStageMng interface to user-defined criteria
 * @param [in,out] aAlgorithm Kelley-Sachs Augmented Lagrangian trust region algorithm interface
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_ksal_algorithm_inputs(const Plato::AlgorithmInputsKSAL<ScalarType, OrdinalType> & aInputs,
                                      Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType> & aStageMng,
                                      Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> & aAlgorithm)
{
/*    if(aInputs.mPrintDiagnostics == true)
    {
        aAlgorithm.enableDiagnostics();
    }*/

    aAlgorithm.setMinPenaltyParameter(aInputs.mMinPenaltyParameter);
    aStageMng.setPenaltyParameter(aInputs.mInitialPenaltyParameter);
    aStageMng.setPenaltyParameterScaleFactor(aInputs.mPenaltyParameterScaleFactor);

    aAlgorithm.setMaxNumIterations(aInputs.mMaxNumOuterIter);
    aAlgorithm.setMaxNumUpdates(aInputs.mMaxNumOuterLineSearchUpdates);
    aAlgorithm.setMaxNumTrustRegionSubProblemIterations(aInputs.mMaxTrustRegionSubProblemIter);

    aAlgorithm.setMaxTrustRegionRadius(aInputs.mMaxTrustRegionRadius);
    aAlgorithm.setMinTrustRegionRadius(aInputs.mMinTrustRegionRadius);
    aAlgorithm.setTrustRegionExpansion(aInputs.mTrustRegionExpansionFactor);
    aAlgorithm.setTrustRegionContraction(aInputs.mTrustRegionContractionFactor);

    aAlgorithm.setGradientTolerance(aInputs.mOuterGradientTolerance);
    aAlgorithm.setFeasibilityTolerance(aInputs.mFeasibilityTolerance);
    aAlgorithm.setStationarityTolerance(aInputs.mOuterStationarityTolerance);
    aAlgorithm.setActualReductionTolerance(aInputs.mOuterActualReductionTolerance);
    aAlgorithm.setObjectiveStagnationTolerance(aInputs.mOuterObjectiveStagnationTolerance);
    aAlgorithm.setControlStagnationTolerance(aInputs.mOuterControlStagnationTolerance);
}
// function set_ksal_algorithm_inputs

/******************************************************************************//**
 * @brief Set Kelley-Sachs Augmented Lagrangian (KSAL) trust region algorithm outputs
 * @param [in] aAlgorithm Kelley-Sachs Augmented Lagrangian trust region algorithm interface
 * @param [in,out] aOutputs Kelley-Sachs Augmented Lagrangian trust region algorithm outputs
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_ksal_algorithm_outputs(const Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> & aAlgorithm,
                                       Plato::AlgorithmOutputsKSAL<ScalarType, OrdinalType> & aOutputs)
{
    aOutputs.mNumOuterIter = aAlgorithm.getNumIterationsDone();
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
    tDataFactory->allocateControl(*aInputs.mInitialGuess);
    tDataFactory->allocateControlReductionOperations(*aInputs.mReductionOperations);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> tDataMng;
    tDataMng = std::make_shared<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>>(tDataFactory);
    tDataMng->setInitialGuess(*aInputs.mInitialGuess);
    tDataMng->setControlLowerBounds(*aInputs.mLowerBounds);
    tDataMng->setControlUpperBounds(*aInputs.mUpperBounds);

    // ********* ALLOCATE AUGMENTED LAGRANGIAN STAGE MANAGER *********
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>> tStageMng;
    tStageMng = std::make_shared<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>>(tDataFactory, aObjective, aConstraints);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM, SOLVE OPTIMIZATION PROBLEM, AND SAVE SOLUTION *********
    Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    Plato::set_ksal_algorithm_inputs(aInputs, *tStageMng, tAlgorithm);
    tAlgorithm.solve();
    Plato::set_ksal_algorithm_outputs(tAlgorithm, aOutputs);
}
// function solve_ksal

} // namespace Plato
