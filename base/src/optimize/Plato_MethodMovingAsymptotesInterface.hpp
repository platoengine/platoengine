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
 * Plato_MethodMovingAsymptotesInterface.hpp
 *
 *  Created on: Sep 1, 2019
 */

#pragma once

#include "Plato_MethodMovingAsymptotes.hpp"
#include "Plato_MethodMovingAsymptotesIO_Data.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Set Method of Moving Asymptotes (MMA) algorithm inputs
 * @param [in] aInputs Method of Moving Asymptotes algorithm inputs
 * @param [in,out] aAlgorithm Method of Moving Asymptotes algorithm interface
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_mma_algorithm_inputs(const Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> & aInputs,
                                     Plato::MethodMovingAsymptotes<ScalarType, OrdinalType> & aAlgorithm)
{
    if(aInputs.mPrintMMADiagnostics == true)
    {
        aAlgorithm.enableDiagnostics(aInputs.mPrintMMADiagnostics);
    }
    aAlgorithm.outputSubProblemDiagnostics(aInputs.mPrintAugLagSubProbDiagnostics);

    aAlgorithm.setInitialGuess(*aInputs.mInitialGuess);
    aAlgorithm.setControlLowerBounds(*aInputs.mLowerBounds);
    aAlgorithm.setControlUpperBounds(*aInputs.mUpperBounds);
    aAlgorithm.setConstraintNormalizationParams(*aInputs.mConstraintNormalizationParams);

    aAlgorithm.setUpdateFrequency(aInputs.mUpdateFrequency);
    aAlgorithm.setMaxNumIterations(aInputs.mMaxNumSolverIter);
    aAlgorithm.setMaxNumSubProblemIterations(aInputs.mMaxNumSubProblemIter);
    aAlgorithm.setMaxNumTrustRegionIterations(aInputs.mMaxNumTrustRegionIter);

    aAlgorithm.setMoveLimit(aInputs.mMoveLimit);
    aAlgorithm.setInitialAugLagPenalty(aInputs.mInitialAugLagPenalty);
    aAlgorithm.setInitialAymptoteScaling(aInputs.mInitialAymptoteScaling);
    aAlgorithm.setAsymptoteExpansionParameter(aInputs.mAsymptoteExpansion);
    aAlgorithm.setSubProblemBoundsScaling(aInputs.mSubProblemBoundsScaling);
    aAlgorithm.setAsymptoteContractionParameter(aInputs.mAsymptoteContraction);
    aAlgorithm.setAugLagSubProbPenaltyMultiplier(aInputs.mAugLagSubProbPenaltyMultiplier);

    aAlgorithm.setOptimalityTolerance(aInputs.mOptimalityTolerance);
    aAlgorithm.setFeasibilityTolerance(aInputs.mFeasibilityTolerance);
    aAlgorithm.setControlStagnationTolerance(aInputs.mControlStagnationTolerance);
    aAlgorithm.setObjectiveStagnationTolerance(aInputs.mObjectiveStagnationTolerance);
    aAlgorithm.setSubProblemFeasibilityTolerance(aInputs.mAugLagSubProbFeasibilityTolerance);
}
// function set_mma_algorithm_inputs

/******************************************************************************//**
 * @brief Set Method of Moving Asymptotes (MMA) algorithm outputs
 * @param [in] aAlgorithm Method of Moving Asymptotes algorithm interface
 * @param [in,out] aOutputs Method of Moving Asymptotes algorithm outputs
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_mma_algorithm_outputs(const Plato::MethodMovingAsymptotes<ScalarType, OrdinalType> & aAlgorithm,
                                      Plato::AlgorithmOutputsMMA<ScalarType, OrdinalType> & aOutputs)
{
    aAlgorithm.getSolution(*aOutputs.mSolution);
    aAlgorithm.getOptimalConstraintValues(*aOutputs.mConstraints);

    aOutputs.mNumSolverIter = aAlgorithm.getNumIterations();
    aOutputs.mNumObjFuncEval = aAlgorithm.getNumObjFuncEvals();

    aOutputs.mObjFuncValue = aAlgorithm.getOptimalObjectiveValue();
    aOutputs.mNormObjFuncGrad = aAlgorithm.getNormObjectiveGradient();
    aOutputs.mControlStagnationMeasure = aAlgorithm.getControlStagnationMeasure();
    aOutputs.mObjectiveStagnationMeasure = aAlgorithm.getObjectiveStagnationMeasure();

    Plato::print_mma_stop_criterion(aAlgorithm.getStoppingCriterion(), aOutputs.mStopCriterion);
}
// function set_mma_algorithm_outputs

/******************************************************************************//**
 * @brief Method of Moving Asymptotes (MMA) algorithm interface
 * @param [in] aObjective user-defined objective function
 * @param [in] aConstraints user-defined list of constraints
 * @param [in] aInputs Method of Moving Asymptotes algorithm inputs
 * @param [in,out] aOutputs Method of Moving Asymptotes algorithm outputs
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void solve_mma(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aObjective,
                      const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aConstraints,
                      const Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> & aInputs,
                      Plato::AlgorithmOutputsMMA<ScalarType, OrdinalType> & aOutputs)
{
    // ********* ALLOCATE DATA STRUCTURES *********
    std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tDataFactory;
    tDataFactory = std::make_shared<Plato::DataFactory<ScalarType, OrdinalType>>();
    tDataFactory->setCommWrapper(aInputs.mCommWrapper);
    tDataFactory->allocateControl(*aInputs.mInitialGuess);
    tDataFactory->allocateDual(*aInputs.mConstraintNormalizationParams);
    tDataFactory->allocateDualReductionOperations(*aInputs.mDualReductionOperations);
    tDataFactory->allocateControlReductionOperations(*aInputs.mControlReductionOperations);

    // ********* ALLOCATE OPTIMALITY CRITERIA ALGORITHM AND SOLVE OPTIMIZATION PROBLEM *********
    Plato::MethodMovingAsymptotes<ScalarType, OrdinalType> tAlgorithm(aObjective, aConstraints, tDataFactory);
    Plato::set_mma_algorithm_inputs(aInputs, tAlgorithm);
    tAlgorithm.solve();

    aOutputs.mSolution = tDataFactory->control().create();
    aOutputs.mConstraints = tDataFactory->dual(0 /* vector index */).create();
    Plato::set_mma_algorithm_outputs(tAlgorithm, aOutputs);
}
// function solve_mma

}
// namespace Plato
