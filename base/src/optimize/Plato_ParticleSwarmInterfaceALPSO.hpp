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
 * Plato_ParticleSwarmInterfaceALPSO.hpp
 *
 *  Created on: Jan 24, 2019
 */

#pragma once

#include "Plato_ParticleSwarmAlgorithmALPSO.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Set augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm inputs
 * @param [in] aInputs inputs for ALPSO algorithm
 * @param [in,out] aAlgorithm ALPSO algorithm interface
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_alpso_algorithm_inputs(const Plato::InputDataALPSO<ScalarType,OrdinalType> & aInputs,
                                       Plato::ParticleSwarmAlgorithmALPSO<ScalarType,OrdinalType> & aAlgorithm)
{
    if(aInputs.mOutputDiagnostics == true)
    {
        aAlgorithm.enableDiagnostics();
    }

    if(aInputs.mOutputParticleDiagnostics == true)
    {
        aAlgorithm.enableParticleDiagnostics();
    }

    if(aInputs.mDisableStdDevStoppingTol == true)
    {
        aAlgorithm.disableStdDevStoppingTolerance();
    }

    aAlgorithm.setMaxNumOuterIterations(aInputs.mMaxNumOuterIter);
    aAlgorithm.setMaxNumInnerIterations(aInputs.mMaxNumInnerIter);
    aAlgorithm.setMaxNumConsecutiveFailures(aInputs.mMaxNumConsecutiveFailures);
    aAlgorithm.setMaxNumConsecutiveSuccesses(aInputs.mMaxNumConsecutiveSuccesses);

    aAlgorithm.setTimeStep(aInputs.mTimeStep);
    aAlgorithm.setLowerBounds(*aInputs.mParticlesLowerBounds);
    aAlgorithm.setUpperBounds(*aInputs.mParticlesUpperBounds);
    aAlgorithm.setInertiaMultiplier(aInputs.mInertiaMultiplier);
    aAlgorithm.setRandomNumMultiplier(aInputs.mRandomNumMultiplier);
    aAlgorithm.setMeanAugLagFuncTolerance(aInputs.mMeanBestAugLagFuncTolerance);
    aAlgorithm.setSocialBehaviorMultiplier(aInputs.mSocialBehaviorMultiplier);
    aAlgorithm.setStdDevAugLagFuncTolerance(aInputs.mStdDevBestAugLagFuncTolerance);
    aAlgorithm.setBestAugLagFuncTolerance(aInputs.mGlobalBestAugLagFuncTolerance);
    aAlgorithm.setPenaltyExpansionMultiplier(aInputs.mPenaltyExpansionMultiplier);
    aAlgorithm.setPenaltyMultiplierUpperBound(aInputs.mPenaltyMultiplierUpperBound);
    aAlgorithm.setCognitiveBehaviorMultiplier(aInputs.mCognitiveBehaviorMultiplier);
    aAlgorithm.setPenaltyContractionMultiplier(aInputs.mPenaltyContractionMultiplier);
    aAlgorithm.setTrustRegionExpansionMultiplier(aInputs.mTrustRegionExpansionMultiplier);
    aAlgorithm.setFeasibilityInexactnessTolerance(aInputs.mFeasibilityInexactnessTolerance);
    aAlgorithm.setTrustRegionContractionMultiplier(aInputs.mTrustRegionContractionMultiplier);
}
// function set_alpso_algorithm_inputs

/******************************************************************************//**
 * @brief Set Augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm outputs
 * @param [in] aAlgorithm ALPSO algorithm interface
 * @param [in,out] aOutputs outputs from ALPSO algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_alpso_algorithm_outputs(const Plato::ParticleSwarmAlgorithmALPSO<ScalarType, OrdinalType> & aAlgorithm,
                                        Plato::OutputDataALPSO<ScalarType, OrdinalType> & aOutputs)
{
    aOutputs.mStopCriterion = aAlgorithm.getStoppingCriterion();

    aOutputs.mNumOuterIter = aAlgorithm.getNumIterations();
    aOutputs.mNumAugLagFuncEval = aAlgorithm.getNumAugLagFuncEvaluations();

    // AUGMENTED LAGRANGIAN FUNCTION DIAGNOSTICS
    aOutputs.mMeanBestAugLagFuncValue = aAlgorithm.getMeanCurrentBestAugLagValues();
    aOutputs.mGlobalBestAugLagFuncValue = aAlgorithm.getCurrentGlobalBestAugLagValue();
    aOutputs.mStdDevBestAugLagFuncValue = aAlgorithm.getStdDevCurrentBestAugLagValues();

    // PARTILCES DIAGNOSTICS
    aAlgorithm.getMeanCurrentBestParticlePositions(aOutputs.mMeanBestParticles);
    aAlgorithm.getStdDevCurrentBestParticlePositions(aOutputs.mStdDevBestParticles);
    aAlgorithm.getCurrentGlobalBestParticlePosition(aOutputs.mGlobalBestParticles);

    // CONSTRAINTS DIAGNOSTICS
    aAlgorithm.getMeanCurrentBestConstraintValues(aOutputs.mMeanBestConstraintValues);
    aAlgorithm.getStdDevCurrentBestConstraintValues(aOutputs.mStdDevBestConstraintValues);
    aAlgorithm.getCurrentGlobalBestConstraintValues(aOutputs.mGlobalBestConstraintValues);
}
// function set_alpso_algorithm_outputs

/******************************************************************************//**
 * @brief Augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm light interface
 * @param [in] aObjective user-defined grad-free objective function
 * @param [in] aConstraints user-defined list of grad-free constraints
 * @param [in] aInputs ALPSO algorithm inputs
 * @param [in,out] aOutputs ALPSO algorithm outputs
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void solve_alpso(const std::shared_ptr<Plato::GradFreeCriterion<ScalarType, OrdinalType>> & aObjective,
                        const std::shared_ptr<Plato::GradFreeCriteriaList<ScalarType, OrdinalType>> & aConstraints,
                        const Plato::InputDataALPSO<ScalarType, OrdinalType> & aInputs,
                        Plato::OutputDataALPSO<ScalarType, OrdinalType> & aOutputs)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tFactory;
    tFactory = std::make_shared<Plato::DataFactory<ScalarType, OrdinalType>>(aInputs.mMemorySpace);
    tFactory->setCommWrapper(aInputs.mCommWrapper);
    tFactory->allocateDual(*aInputs.mDual);
    tFactory->allocateControl(*aInputs.mParticles);
    tFactory->allocateObjFuncValues(*aInputs.mCriteriaEvals);
    tFactory->allocateControlReductionOperations(*aInputs.mControlReductions);
    tFactory->allocateObjFuncReductionOperations(*aInputs.mCriteriaReductions);

    // ********* ALLOCATE AUGMENTED LAGRANGIAN ALGORITHM *********
    Plato::ParticleSwarmAlgorithmALPSO<ScalarType, OrdinalType> tAlgorithm(tFactory, aObjective, aConstraints);
    tAlgorithm.setCustomOutput(aInputs.mCustomOutput);

    // ********* SOLVE OPTIMIZATION PROBLEM AND SAVE SOLUTION *********
    Plato::set_alpso_algorithm_inputs(aInputs, tAlgorithm);
    tAlgorithm.solve();
    Plato::set_alpso_algorithm_outputs(tAlgorithm, aOutputs);

    // ********* OUTPUT SOLUTION TO TEXT FILE *********
    if(aInputs.mOutputSolution == true && aInputs.mCommWrapper.myProcID() == 0)
    {
        Plato::pso::print_solution(*aOutputs.mGlobalBestParticles, *aOutputs.mMeanBestParticles, *aOutputs.mStdDevBestParticles);
    }
}
// function solve_alpso

} // namespace Plato
