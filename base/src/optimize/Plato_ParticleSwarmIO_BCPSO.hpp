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
 * Plato_ParticleSwarmIO_BCPSO.hpp
 *
 *  Created on: Jan 24, 2019
 */

#pragma once

#include <vector>
#include <string>

#include "Plato_MultiVector.hpp"
#include "Plato_CustomOutput.hpp"
#include "Plato_CommWrapper.hpp"
#include "Plato_ParticleSwarmTypes.hpp"
#include "Plato_ReductionOperations.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Diagnostic data for the Particle Swarm Optimization (PSO) algorithm
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct DiagnosticsBCPSO
{
    /******************************************************************************//**
     * @brief Constructor
     **********************************************************************************/
    DiagnosticsBCPSO() :
        mNumIter(0),
        mObjFuncCount(0),
        mNumConstraints(0),
        mTrustRegionMultiplier(0),
        mMeanCurrentBestObjFuncValues(0),
        mCurrentGlobalBestObjFuncValue(0),
        mStdDevCurrentBestObjFuncValues(0)
    {
    }

    /******************************************************************************//**
     * @brief Destructor
     **********************************************************************************/
    ~DiagnosticsBCPSO()
    {
    }

    OrdinalType mNumIter; /*!< number of outer iterations */
    OrdinalType mObjFuncCount; /*!< number of objective function evaluations */
    OrdinalType mNumConstraints; /*!< number of constraints - only needed for ALPSO output */

    ScalarType mTrustRegionMultiplier; /*!< trust region multiplier */
    ScalarType mMeanCurrentBestObjFuncValues; /*!< mean - objective function value */
    ScalarType mCurrentGlobalBestObjFuncValue; /*!< best - objective function value */
    ScalarType mStdDevCurrentBestObjFuncValues; /*!< standard deviation - objective function value */
};
// struct DiagnosticsBCPSO

/******************************************************************************//**
 * @brief Inputs from Bound Constrained Particle Swarm Optimization (BCPSO) algorithm
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct OutputDataBCPSO
{
    std::string mStopCriterion; /*!< stopping criterion */

    OrdinalType mNumOuterIter; /*!< number of outer iterations */
    OrdinalType mNumObjFuncEval; /*!< number of objective function evaluations */

    ScalarType mMeanBestObjFuncValue; /*!< objective function mean from best particle set */
    ScalarType mStdDevBestObjFuncValue; /*!< objective function standard deviation from best particle set */
    ScalarType mGlobalBestObjFuncValue; /*!< global best objective function value */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mMeanBestParticles; /*!< particle dimension's standard deviation values from best particle set */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mStdDevBestParticles; /*!< particle dimension's mean values from best particle set */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mGlobalBestParticles; /*!< global best particle dimension's */
};
// struct OutputDataBCPSO

/******************************************************************************//**
 * @brief Inputs for Bound Constrained Particle Swarm Optimization (BCPSO) algorithm
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct InputDataBCPSO
{
    /******************************************************************************//**
     * @brief Default constructor
     **********************************************************************************/
    InputDataBCPSO() :
        mOutputSolution(false),
        mReadRestartFile(false),
        mWriteRestartFile(false),
        mOutputDiagnostics(false),
        mOutputParticleDiagnostics(false),
        mDisableStdDevStoppingTol(false),
        mOutputStageName(),
        mNumParticles(10),
        mMaxNumIterations(1e3),
        mMaxNumConsecutiveFailures(10),
        mMaxNumConsecutiveSuccesses(10),
        mTimeStep(1),
        mMeanBestObjFuncTolerance(5e-4),
        mStdDevBestObjFuncTolerance(1e-6),
        mGlobalBestObjFuncTolerance(1e-10),
        mTrustRegionMultiplierTolerance(1e-8),
        mInertiaMultiplier(0.9),
        mRandomNumMultiplier(0.1),
        mSocialBehaviorMultiplier(0.8),
        mCognitiveBehaviorMultiplier(0.8),
        mTrustRegionExpansionMultiplier(4.0),
        mTrustRegionContractionMultiplier(0.75),
        mMemorySpace(Plato::MemorySpace::HOST),
        mCriteriaEvals(),
        mParticlesLowerBounds(),
        mParticlesUpperBounds(),
        mParticles(),
        mCustomOutput(std::make_shared<Plato::CustomOutput<ScalarType, OrdinalType>>()),
        mControlReductions(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>()),
        mCriteriaReductions(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>())
    {
        mCommWrapper.useDefaultComm();
    }

    /******************************************************************************//**
     * @brief Default destructor
     **********************************************************************************/
    ~InputDataBCPSO()
    {
    }

    bool mOutputSolution; /*!< flag to enable solution output (default=false) */
    bool mReadRestartFile; /*!< flag - read restart file (default=false) */
    bool mWriteRestartFile; /*!< flag - write restart file (default=false) */
    bool mOutputDiagnostics; /*!< flag to enable algorithm diagnostics (default=false) */
    bool mOutputParticleDiagnostics; /*!< flag to enable particle data diagnostics (default=false) */
    bool mDisableStdDevStoppingTol; /*!< flag to disable the stopping tolerance based on the standard deviation (default=false) */

    std::string mOutputStageName; /*!< output stage name */

    OrdinalType mNumParticles; /*!< number of particles */
    OrdinalType mMaxNumIterations; /*!< maximum number of iterations */
    OrdinalType mMaxNumConsecutiveFailures; /*!< maximum number of consecutive failures, global best F(x_{i+1}) == F(x_{i}) */
    OrdinalType mMaxNumConsecutiveSuccesses; /*!< maximum number of consecutive successes, global best F(x_{i+1}) < F(x_{i}) */

    ScalarType mTimeStep; /*!< time step \Delta{t} */
    ScalarType mMeanBestObjFuncTolerance; /*!< mean objective function stopping tolerance */
    ScalarType mStdDevBestObjFuncTolerance; /*!< standard deviation stopping tolerance */
    ScalarType mGlobalBestObjFuncTolerance; /*!< best objective function stopping tolerance */
    ScalarType mTrustRegionMultiplierTolerance; /*!< stopping tolerance on the trust region multiplier */
    ScalarType mInertiaMultiplier; /*!< inertia multiplier */
    ScalarType mRandomNumMultiplier; /*!< random number multiplier (used to find an unique particle position) */
    ScalarType mSocialBehaviorMultiplier; /*!< social behavior multiplier */
    ScalarType mCognitiveBehaviorMultiplier; /*!< cognite behavior multiplier */
    ScalarType mTrustRegionExpansionMultiplier; /*!< trust region expansion multiplier */
    ScalarType mTrustRegionContractionMultiplier; /*!< trust region contraction multiplier */

    Plato::CommWrapper mCommWrapper; /*!< distributed memory communication wrapper */
    Plato::MemorySpace::type_t mMemorySpace; /*!< memory space: HOST (default) OR DEVICE */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCriteriaEvals; /*!< criteria evaluations */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mParticlesLowerBounds; /*!< particles' lower bounds */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mParticlesUpperBounds; /*!< particles' upper bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mParticles; /*!< particles */

    std::shared_ptr<Plato::CustomOutput<ScalarType, OrdinalType>> mCustomOutput; /*!< custom output interface */

    /*!< operations that require communication across processors, e.g. max, min, global sum */
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductions;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mCriteriaReductions;
};
// struct InputDataBCPSO

}// namespace Plato
