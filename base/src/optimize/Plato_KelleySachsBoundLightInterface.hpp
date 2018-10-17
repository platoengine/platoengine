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
 * Plato_KelleySachsBoundLightInterface.hpp
 *
 *  Created on: Sep 29, 2018
 */

#pragma once

#include <string>
#include <memory>

#include "Plato_DataFactory.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_KelleySachsBoundConstrained.hpp"
#include "Plato_ReducedSpaceTrustRegionStageMng.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Output data structure for Kelley-Sachs Bound Constrained (KSBC) trust region algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmOutputsKSBC
{
    OrdinalType mNumOuterIter; /*!< number of outer iterations */
    OrdinalType mNumObjFuncEval; /*!< number of objective function evaluations */
    OrdinalType mNumObjGradEval; /*!< number of objective gradient evaluations */

    ScalarType mObjFuncValue; /*!< objective function value */
    ScalarType mNormObjFuncGrad; /*!< norm of the objective function gradient */
    ScalarType mActualReduction; /*!< actual reduction */
    ScalarType mStationarityMeasure; /*!< norm of the descent direction */
    ScalarType mCurrentTrustRegionRadius; /*!< current trust region radius */
    ScalarType mControlStagnationMeasure; /*!< norm of the difference between two subsequent control fields */
    ScalarType mObjectiveStagnationMeasure; /*!< measures stagnation in two subsequent objective function evaluations */

    std::string mStopCriterion; /*!< stopping criterion */

    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mSolution; /*!< optimal solution */
};
// struct AlgorithmOutputsKSBC

/******************************************************************************//**
 * @brief Input data structure for Kelley-Sachs Bound Constrained (KSBC) trust region algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmInputsKSBC
{
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    AlgorithmInputsKSBC() :
            mHaveHessian(true),
            mPrintDiagnostics(false),
            mMaxNumOuterIter(500),
            mMaxTrustRegionSubProblemIter(25),
            mMaxNumOuterLineSearchUpdates(10),
            mMaxTrustRegionRadius(1e2),
            mMinTrustRegionRadius(1e-8),
            mTrustRegionExpansionFactor(4),
            mTrustRegionContractionFactor(0.75),
            mOuterGradientTolerance(1e-8),
            mOuterStationarityTolerance(1e-8),
            mOuterActualReductionTolerance(1e-12),
            mOuterControlStagnationTolerance(1e-16),
            mOuterObjectiveStagnationTolerance(1e-12),
            mCommWrapper(),
            mMemorySpace(Plato::MemorySpace::HOST),
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
    virtual ~AlgorithmInputsKSBC()
    {
    }

    bool mHaveHessian; /*!< flag to specify that Hessian information is available (default=true) */
    bool mPrintDiagnostics; /*!< flag to enable problem statistics output (default=false) */

    OrdinalType mMaxNumOuterIter; /*!< maximum number of outer iterations */
    OrdinalType mMaxTrustRegionSubProblemIter; /*!< maximum number of trust region sub problem iterations */
    OrdinalType mMaxNumOuterLineSearchUpdates; /*!< maximum number of outer line search iterations */

    ScalarType mMaxTrustRegionRadius; /*!< maximum trust region radius */
    ScalarType mMinTrustRegionRadius; /*!< minimum trust region radius */
    ScalarType mTrustRegionExpansionFactor; /*!< trust region radius expansion factor */
    ScalarType mTrustRegionContractionFactor; /*!< trust region radius contraction factor */

    ScalarType mOuterGradientTolerance; /*!< gradient norm tolerance */
    ScalarType mOuterStationarityTolerance; /*!< descent direction norm tolerance */
    ScalarType mOuterActualReductionTolerance; /*!< actual reduction tolerance */
    ScalarType mOuterControlStagnationTolerance; /*!< control stagnation tolerance */
    ScalarType mOuterObjectiveStagnationTolerance; /*!< objective function stagnation tolerance */

    Plato::CommWrapper mCommWrapper; /*!< distributed memory communication wrapper */
    Plato::MemorySpace::type_t mMemorySpace; /*!< memory space: HOST (default) OR DEVICE */

    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mLowerBounds; /*!< lower bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mUpperBounds; /*!< upper bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mInitialGuess; /*!< initial guess */
    
    /*!< operations which require communication across processors, e.g. max, min, global sum */
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mReductionOperations;
};
// struct AlgorithmInputsKSBC

/******************************************************************************//**
 * @brief Set Kelley-Sachs Bound Constrained (KSBC) trust region algorithm inputs
 * @param [in] aInputs Kelley-Sachs Bound Constrained trust region algorithm inputs
 * @param [in,out] aAlgorithm Kelley-Sachs Bound Constrained trust region algorithm interface
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_ksbc_algorithm_inputs(const Plato::AlgorithmInputsKSBC<ScalarType, OrdinalType> & aInputs,
                                      Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType> & aAlgorithm)
{
    if(aInputs.mPrintDiagnostics == true)
    {
        aAlgorithm.enableDiagnostics();
    }

    aAlgorithm.setMaxNumIterations(aInputs.mMaxNumOuterIter);
    aAlgorithm.setMaxNumUpdates(aInputs.mMaxNumOuterLineSearchUpdates);
    aAlgorithm.setMaxNumTrustRegionSubProblemIterations(aInputs.mMaxTrustRegionSubProblemIter);

    aAlgorithm.setMaxTrustRegionRadius(aInputs.mMaxTrustRegionRadius);
    aAlgorithm.setMinTrustRegionRadius(aInputs.mMinTrustRegionRadius);
    aAlgorithm.setTrustRegionExpansion(aInputs.mTrustRegionExpansionFactor);
    aAlgorithm.setTrustRegionContraction(aInputs.mTrustRegionContractionFactor);

    aAlgorithm.setGradientTolerance(aInputs.mOuterGradientTolerance);
    aAlgorithm.setStationarityTolerance(aInputs.mOuterStationarityTolerance);
    aAlgorithm.setActualReductionTolerance(aInputs.mOuterActualReductionTolerance);
    aAlgorithm.setControlStagnationTolerance(aInputs.mOuterControlStagnationTolerance);
    aAlgorithm.setObjectiveStagnationTolerance(aInputs.mOuterObjectiveStagnationTolerance);
}
// function set_ksbc_algorithm_inputs

/******************************************************************************//**
 * @brief Set Kelley-Sachs Bound Constrained (KSBC) trust region algorithm outputs
 * @param [in] aAlgorithm Kelley-Sachs Bound Constrained trust region algorithm interface
 * @param [in,out] aOutputs Kelley-Sachs Bound Constrained trust region algorithm outputs
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_ksbc_algorithm_outputs(const Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType> & aAlgorithm,
                                       Plato::AlgorithmOutputsKSBC<ScalarType, OrdinalType> & aOutputs)
{
    aOutputs.mNumOuterIter = aAlgorithm.getNumIterationsDone();
    aOutputs.mNumObjFuncEval = aAlgorithm.getDataMng().getNumObjectiveFunctionEvaluations();
    aOutputs.mNumObjGradEval = aAlgorithm.getDataMng().getNumObjectiveGradientEvaluations();

    aOutputs.mObjFuncValue = aAlgorithm.getDataMng().getCurrentObjectiveFunctionValue();
    aOutputs.mNormObjFuncGrad = aAlgorithm.getDataMng().getNormProjectedGradient();
    aOutputs.mActualReduction = aAlgorithm.getStepMng().getActualReduction();
    aOutputs.mStationarityMeasure = aAlgorithm.getDataMng().getStationarityMeasure();
    aOutputs.mCurrentTrustRegionRadius = aAlgorithm.getStepMng().getTrustRegionRadius();
    aOutputs.mControlStagnationMeasure = aAlgorithm.getDataMng().getControlStagnationMeasure();
    aOutputs.mObjectiveStagnationMeasure = aAlgorithm.getDataMng().getObjectiveStagnationMeasure();

    Plato::get_stop_criterion(aAlgorithm.getStoppingCriterion(), aOutputs.mStopCriterion);

    const Plato::MultiVector<ScalarType, OrdinalType> & tSolution = aAlgorithm.getDataMng().getCurrentControl();
    aOutputs.mSolution = tSolution.create();
    Plato::update(static_cast<ScalarType>(1), tSolution, static_cast<ScalarType>(0), *aOutputs.mSolution);
}
// function set_ksbc_algorithm_outputs

/******************************************************************************//**
 * @brief Kelley-Sachs Bound Constrained (KSBC) trust region algorithm interface
 * @param [in] aObjective list of user-defined objective functions
 * @param [in] aInputs Kelley-Sachs Bound Constrained trust region algorithm inputs
 * @param [in,out] aOutputs Kelley-Sachs Bound Constrained trust region algorithm outputs
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void solve_ksbc(const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aObjective,
                       const Plato::AlgorithmInputsKSBC<ScalarType, OrdinalType> & aInputs,
                       Plato::AlgorithmOutputsKSBC<ScalarType, OrdinalType> & aOutputs)
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

    // ********* ALLOCATE REDUCED SPACE STAGE MANAGER *********
    std::shared_ptr<Plato::ReducedSpaceTrustRegionStageMng<ScalarType, OrdinalType>> tStageMng;
    tStageMng = std::make_shared<Plato::ReducedSpaceTrustRegionStageMng<ScalarType, OrdinalType>>(tDataFactory, aObjective);
    tStageMng->setHaveHessian(aInputs.mHaveHessian);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM, SOLVE OPTIMIZATION PROBLEM, AND SAVE SOLUTION *********
    Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    Plato::set_ksbc_algorithm_inputs(aInputs, tAlgorithm);
    tAlgorithm.solve();
    Plato::set_ksbc_algorithm_outputs(tAlgorithm, aOutputs);
}
// function solve_ksbc

} // namespace Plato
