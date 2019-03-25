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
 * Plato_ParticleSwarmIO_ALPSO.hpp
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
 * @brief Diagnostic data for the augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct DiagnosticsALPSO
{
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    DiagnosticsALPSO() :
        mNumIter(0),
        mNumConstraints(0),
        mAugLagFuncCount(0),
        mMeanCurrentBestObjFuncValues(0),
        mCurrentGlobalBestObjFuncValue(0),
        mStdDevCurrentBestObjFuncValues(0),
        mMeanCurrentBestAugLagValues(0),
        mCurrentGlobalBestAugLagValue(0),
        mStdDevCurrentBestAugLagValues(0),
        mMeanCurrentBestConstraintValues(),
        mCurrentGlobalBestConstraintValues(),
        mStdDevCurrentBestConstraintValues(),
        mMeanCurrentPenaltyMultipliers(),
        mStdDevCurrentPenaltyMultipliers(),
        mMeanCurrentLagrangeMultipliers(),
        mStdDevCurrentLagrangeMultipliers()
    {
    }

    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aNumConstraints number of constraints
    **********************************************************************************/
    explicit DiagnosticsALPSO(const OrdinalType & aNumConstraints) :
        mNumIter(0),
        mNumConstraints(aNumConstraints),
        mAugLagFuncCount(0),
        mMeanCurrentBestObjFuncValues(0),
        mCurrentGlobalBestObjFuncValue(0),
        mStdDevCurrentBestObjFuncValues(0),
        mMeanCurrentBestAugLagValues(0),
        mCurrentGlobalBestAugLagValue(0),
        mStdDevCurrentBestAugLagValues(0),
        mMeanCurrentBestConstraintValues(std::vector<ScalarType>(aNumConstraints, 0)),
        mCurrentGlobalBestConstraintValues(std::vector<ScalarType>(aNumConstraints, 0)),
        mStdDevCurrentBestConstraintValues(std::vector<ScalarType>(aNumConstraints, 0)),
        mMeanCurrentPenaltyMultipliers(std::vector<ScalarType>(aNumConstraints, 0)),
        mStdDevCurrentPenaltyMultipliers(std::vector<ScalarType>(aNumConstraints, 0)),
        mMeanCurrentLagrangeMultipliers(std::vector<ScalarType>(aNumConstraints, 0)),
        mStdDevCurrentLagrangeMultipliers(std::vector<ScalarType>(aNumConstraints, 0))
    {
    }

    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    ~DiagnosticsALPSO()
    {
    }

    OrdinalType mNumIter;  /*!< number of outer iterations */
    OrdinalType mNumConstraints;  /*!< number of constraints */
    OrdinalType mAugLagFuncCount;  /*!< number of augmented Lagrangian function evaluations */

    ScalarType mMeanCurrentBestObjFuncValues;  /*!< mean objective function value */
    ScalarType mCurrentGlobalBestObjFuncValue;  /*!< best objective function value */
    ScalarType mStdDevCurrentBestObjFuncValues;  /*!< standard deviation for objective function value */

    ScalarType mMeanCurrentBestAugLagValues;  /*!< mean augmented Lagrangian function */
    ScalarType mCurrentGlobalBestAugLagValue;  /*!< best augmented Lagrangian function */
    ScalarType mStdDevCurrentBestAugLagValues;  /*!< standard deviation for augmented Lagrangian function */

    std::vector<ScalarType> mMeanCurrentBestConstraintValues;  /*!< mean constraint values */
    std::vector<ScalarType> mCurrentGlobalBestConstraintValues;  /*!< constraint values at global best particle location */
    std::vector<ScalarType> mStdDevCurrentBestConstraintValues;  /*!< standard deviation for constraint values */

    std::vector<ScalarType> mMeanCurrentPenaltyMultipliers;  /*!< mean penalty multipliers */
    std::vector<ScalarType> mStdDevCurrentPenaltyMultipliers;  /*!< standard deviation for penalty multipliers */

    std::vector<ScalarType> mMeanCurrentLagrangeMultipliers;  /*!< mean Lagrange multipliers */
    std::vector<ScalarType> mStdDevCurrentLagrangeMultipliers;  /*!< standard deviation for Lagrange multipliers */
};
// struct DiagnosticsALPSO

/******************************************************************************//**
 * @brief Inputs from augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct OutputDataALPSO
{
    std::string mStopCriterion; /*!< stopping criterion */

    OrdinalType mNumOuterIter; /*!< number of outer iterations */
    OrdinalType mNumAugLagFuncEval; /*!< number of augmented Lagrangian function evaluations */

    ScalarType mMeanBestAugLagFuncValue; /*!< augmented Lagrangian function mean from best particle set */
    ScalarType mStdDevBestAugLagFuncValue; /*!< augmented Lagrangian function standard deviation from best particle set */
    ScalarType mGlobalBestAugLagFuncValue; /*!< global best augmented Lagrangian function value */

    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mMeanBestParticles; /*!< particle dimension's standard deviation values from best particle set */
    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mStdDevBestParticles; /*!< particle dimension's mean values from best particle set */
    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mGlobalBestParticles; /*!< global best particle dimension's */

    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mMeanBestConstraintValues; /*!< constraint mean values from best particle set */
    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mStdDevBestConstraintValues; /*!< constraint standard deviation values from best particle set */
    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mGlobalBestConstraintValues; /*!< global best constraint values */
};
// struct OutputDataALPSO

/******************************************************************************//**
 * @brief Inputs for augmented Lagrangian Particle Swarm Optimization (ALPSO) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct InputDataALPSO
{
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    InputDataALPSO() :
            mOutputSolution(false),
            mOutputDiagnostics(false),
            mOutputParticleDiagnostics(false),
            mDisableStdDevStoppingTol(false),
            mOutputStageName(),
            mNumParticles(10),
            mMaxNumOuterIter(1e3),
            mMaxNumInnerIter(10),
            mMaxNumConsecutiveFailures(10),
            mMaxNumConsecutiveSuccesses(10),
            mTimeStep(1),
            mMeanBestAugLagFuncTolerance(5e-4),
            mStdDevBestAugLagFuncTolerance(1e-6),
            mGlobalBestAugLagFuncTolerance(1e-10),
            mTrustRegionMultiplierTolerance(1e-8),
            mInertiaMultiplier(0.9),
            mSocialBehaviorMultiplier(0.8),
            mCognitiveBehaviorMultiplier(0.8),
            mPenaltyExpansionMultiplier(2),
            mPenaltyMultiplierUpperBound(100),
            mPenaltyContractionMultiplier(0.5),
            mFeasibilityInexactnessTolerance(1e-4),
            mTrustRegionExpansionMultiplier(4.0),
            mTrustRegionContractionMultiplier(0.75),
            mMemorySpace(Plato::MemorySpace::HOST),
            mCriteriaEvals(),
            mParticlesLowerBounds(),
            mParticlesUpperBounds(),
            mDual(),
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
    ~InputDataALPSO()
    {
    }

    bool mOutputSolution; /*!< flag to output solution (default=false) */
    bool mOutputDiagnostics; /*!< flag to enable algorithm diagnostics (default=false) */
    bool mOutputParticleDiagnostics; /*!< flag to enable particle data diagnostics (default=false) */
    bool mDisableStdDevStoppingTol; /*!< flag to disable the stopping tolerance based on the standard deviation (default=false) */

    std::string mOutputStageName; /*!< output stage name */

    OrdinalType mNumParticles; /*!< number of particles */
    OrdinalType mMaxNumOuterIter; /*!< maximum number of outer iterations */
    OrdinalType mMaxNumInnerIter; /*!< maximum number of augmented Lagrangian subproblem iterations */
    OrdinalType mMaxNumConsecutiveFailures; /*!< maximum number of consecutive failures, global best F(x_{i+1}) == F(x_{i}) */
    OrdinalType mMaxNumConsecutiveSuccesses; /*!< maximum number of consecutive successes, global best F(x_{i+1}) < F(x_{i}) */

    ScalarType mTimeStep; /*!< time step \Delta{t} */
    ScalarType mMeanBestAugLagFuncTolerance; /*!< mean augmented Lagrangian function stopping tolerance */
    ScalarType mStdDevBestAugLagFuncTolerance; /*!< standard deviation stopping tolerance */
    ScalarType mGlobalBestAugLagFuncTolerance; /*!< best augmented Lagrangian function stopping tolerance */
    ScalarType mTrustRegionMultiplierTolerance; /*!< stopping tolerance on the trust region multiplier */
    ScalarType mInertiaMultiplier; /*!< inertia multiplier */
    ScalarType mSocialBehaviorMultiplier; /*!< social behavior multiplier */
    ScalarType mCognitiveBehaviorMultiplier; /*!< cognite behavior multiplier */
    ScalarType mPenaltyExpansionMultiplier; /*!< penalty expansion multiplier */
    ScalarType mPenaltyMultiplierUpperBound; /*!< upper bound on penalty multiplier */
    ScalarType mPenaltyContractionMultiplier; /*!< penalty contraction multiplier */
    ScalarType mFeasibilityInexactnessTolerance; /*!< feasibility inexactness tolerance */
    ScalarType mTrustRegionExpansionMultiplier; /*!< trust region expansion multiplier */
    ScalarType mTrustRegionContractionMultiplier; /*!< trust region contraction multiplier */

    Plato::CommWrapper mCommWrapper; /*!< distributed memory communication wrapper */
    Plato::MemorySpace::type_t mMemorySpace; /*!< memory space: HOST (default) OR DEVICE */
    std::vector<Plato::particle_swarm::constraint_t> mConstraintTypes; /*!< list of constraint types */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCriteriaEvals; /*!< criteria evaluations */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mParticlesLowerBounds; /*!< particles' lower bounds */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mParticlesUpperBounds; /*!< particles' upper bounds */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDual; /*!< Lagrange multipliers */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mParticles; /*!< particles */

    std::shared_ptr<Plato::CustomOutput<ScalarType,OrdinalType>> mCustomOutput;  /*!< custom output interface */

    /*!< operations which require communication across processors, e.g. max, min, global sum */
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mControlReductions;
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mCriteriaReductions;

};
// InputDataALPSO

} // namespace Plato
