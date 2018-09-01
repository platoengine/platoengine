/*
 * Plato_SolveUncertaintyProblem.hpp
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
// *************************************************************************
//@HEADER
*/

#pragma once

#include "Plato_MultiVector.hpp"
#include "Plato_Vector.hpp"
#include "Plato_DistributionFactory.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_SromObjective.hpp"
#include "Plato_ReductionOperations.hpp"
#include "Plato_SromConstraint.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_AugmentedLagrangianStageMng.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_StatisticsUtils.hpp"
#include "Plato_KelleySachsAugmentedLagrangian.hpp"

#include <vector>

namespace Plato
{

template<typename ScalarType>
struct UncertaintyOutputStruct
{
    ScalarType mSampleValue;
    ScalarType mSampleWeight;
};
// struct UncertaintyOutputStruct

template<typename ScalarType, typename OrdinalType>
struct AlgorithmParamStruct
{
    // Stopping criterion
    Plato::algorithm::stop_t mStop;
    // Input parameters
    OrdinalType mMaxNumIterations;
    ScalarType mStagnationTolerance;
    ScalarType mMinPenaltyParameter;
    ScalarType mMaxTrustRegionRadius;
    ScalarType mActualReductionTolerance;
    ScalarType mPenaltyParameterScaleFactor;
    // Output parameters
    ScalarType mObjectiveValue;
    ScalarType mConstraintValue;
    ScalarType mCumulativeDistributionMisfit;
    std::vector<ScalarType> mMomentMisfitTerms;

    /*! @brief Default constructor */
    AlgorithmParamStruct() :
            mStop(Plato::algorithm::NOT_CONVERGED),
            mMaxNumIterations(500),
            mStagnationTolerance(1e-12),
            mMinPenaltyParameter(5e-4),
            mMaxTrustRegionRadius(1e1),
            mActualReductionTolerance(1e-12),
            mPenaltyParameterScaleFactor(1.2),
            mObjectiveValue(std::numeric_limits<ScalarType>::max()),
            mConstraintValue(std::numeric_limits<ScalarType>::max()),
            mCumulativeDistributionMisfit(std::numeric_limits<ScalarType>::max()),
            mMomentMisfitTerms()
    {
    }
};
// struct AlgorithmParamStruct

template<typename ScalarType, typename OrdinalType>
void solve_uncertainty(const Plato::UncertaintyInputStruct<ScalarType, OrdinalType>& aInput,
                       Plato::AlgorithmParamStruct<ScalarType, OrdinalType>& aAlgorithmParam,
                       std::vector<UncertaintyOutputStruct<ScalarType> >& aOutput)
{
    // grab values from input struct
    const OrdinalType tNumSamples = aInput.mNumSamples;
    const ScalarType tLowerBound = aInput.mLowerBound;
    const ScalarType tUpperBound = aInput.mUpperBound;
    OrdinalType tMaxNumMoments = 0;
    if(aInput.mMaxNumDistributionMoments < 1)
    {
        tMaxNumMoments = 4;
    }
    else
    {
        tMaxNumMoments = aInput.mMaxNumDistributionMoments;
    }

    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tDataFactory =
            std::make_shared<Plato::DataFactory<ScalarType, OrdinalType>>();
    const OrdinalType tNumConstraints = 1;
    const OrdinalType tNumControlVectors = 2; // samples values and sample weights
    tDataFactory->allocateDual(tNumConstraints);
    tDataFactory->allocateControl(tNumSamples, tNumControlVectors);

    // build distribution
    std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> tDistribution = build_distrubtion<ScalarType, OrdinalType>(aInput);

    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT CRITERIA *********
    std::shared_ptr<Plato::SromObjective<ScalarType, OrdinalType> > tSromObjective =
            std::make_shared<Plato::SromObjective<ScalarType, OrdinalType> >(tDistribution, tMaxNumMoments);
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType> > tReductions = tDataFactory->getControlReductionOperations().create();
    std::shared_ptr<Plato::SromConstraint<ScalarType, OrdinalType> > tSromConstraint = std::make_shared<Plato::SromConstraint<ScalarType, OrdinalType> >(tReductions);
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType> > tConstraintList = std::make_shared<Plato::CriterionList<ScalarType, OrdinalType> >();
    tConstraintList->add(tSromConstraint);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType> > tStageMng =
            std::make_shared<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType> >(tDataFactory, tSromObjective, tConstraintList);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> > tDataMng =
            std::make_shared<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> >(tDataFactory);

    // ********* SET INTIAL GUESS FOR VECTOR OF SAMPLES *********
    Plato::StandardVector<ScalarType, OrdinalType> tInitialGuess(tNumSamples);
    Plato::make_uniform_sample<ScalarType,OrdinalType>(tInitialGuess);
    OrdinalType tVectorIndex = 0;
    tDataMng->setInitialGuess(tVectorIndex, tInitialGuess);
    // ********* SET INTIAL GUESS FOR VECTOR OF PROBABILITIES *********
    ScalarType tScalarValue = 1. / static_cast<ScalarType>(tNumSamples);
    tInitialGuess.fill(tScalarValue);
    tVectorIndex = 1;
    tDataMng->setInitialGuess(tVectorIndex, tInitialGuess);
    // ********* SET UPPER AND LOWER BOUNDS *********
    tScalarValue = 0;
    tDataMng->setControlLowerBounds(tScalarValue);
    tScalarValue = 1;
    tDataMng->setControlUpperBounds(tScalarValue);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM *********
    Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    tAlgorithm.setMaxNumIterations(aAlgorithmParam.mMaxNumIterations);
    tAlgorithm.setMinPenaltyParameter(aAlgorithmParam.mMinPenaltyParameter);
    tAlgorithm.setStagnationTolerance(aAlgorithmParam.mStagnationTolerance);
    tAlgorithm.setMaxTrustRegionRadius(aAlgorithmParam.mMaxTrustRegionRadius);
    tAlgorithm.setActualReductionTolerance(aAlgorithmParam.mActualReductionTolerance);
    tAlgorithm.solve();

    // transfer to output data structure
    const Plato::MultiVector<ScalarType, OrdinalType>& tFinalControl = tDataMng->getCurrentControl();
    aOutput.resize(tNumSamples);
    for(OrdinalType tIndex = 0; tIndex < tNumSamples; tIndex++)
    {
        // undo scaling
        const ScalarType tUnscaledSampleValue = tFinalControl(0, tIndex);
        aOutput[tIndex].mSampleValue = tLowerBound + (tUpperBound - tLowerBound) * tUnscaledSampleValue;
        // NOTE: The upper/lower bounds are not always defined. For example, normal dist'n what should be done?
        // confirmed accurate for beta dist'n alone

        aOutput[tIndex].mSampleWeight = tFinalControl(1, tIndex);
    }

    // transfer stopping criterion, objective and constraint values to algorithm parameter data structure
    aAlgorithmParam.mStop = tAlgorithm.getStoppingCriterion();
    aAlgorithmParam.mObjectiveValue = tDataMng->getCurrentObjectiveFunctionValue();
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double, size_t> tConstraintValues(tNumVectors, tNumConstraints);
    tStageMng->getCurrentConstraintValues(tConstraintValues);
    aAlgorithmParam.mConstraintValue = tConstraintValues(0,0);
}
// function solve_uncertainty

} // namespace Plato
