/*
 * Plato_SromProbDataStruct.hpp
 *
 *  Created on: Sep 27, 2018
 */

#pragma once

#include <limits>
#include <vector>

#include "Plato_Types.hpp"

namespace Plato
{

struct DistrubtionName
{
    enum type_t
    {
        beta = 0,
        normal = 1,
        uniform = 2,
    };
};
// struct DistrubtionName

template<typename ScalarType, typename OrdinalType = size_t>
struct UncertaintyInputStruct
{
    DistrubtionName::type_t mDistribution;
    ScalarType mMean;
    ScalarType mLowerBound;
    ScalarType mUpperBound;
    ScalarType mVariance;

    ScalarType mMomentErrorCriterionWeight;
    ScalarType mCumulativeDistributionFuncErrorWeight;

    OrdinalType mNumSamples;
    OrdinalType mMaxNumDistributionMoments; // if zero, then use default

    UncertaintyInputStruct() :   // default Constructor
            mDistribution(DistrubtionName::type_t::beta),
            mMean(0.),
            mLowerBound(0.),
            mUpperBound(0.),
            mVariance(0.),
            mMomentErrorCriterionWeight(1/1e2),
            mCumulativeDistributionFuncErrorWeight(1/1e2),
            mNumSamples(0),
            mMaxNumDistributionMoments(4)
    {
    }
};
// struct UncertaintyInputStruct

template<typename ScalarType>
struct UncertaintyOutputStruct
{
    // Primary outputs
    ScalarType mSampleValue;
    ScalarType mSampleWeight;
};
// struct UncertaintyOutputStruct

template<typename ScalarType>
struct SromProblemDiagnosticsStruct
{
    // Diagnostics - secondary outputs
    std::vector<ScalarType> mSromCDF;
    std::vector<ScalarType> mTrueCDF;
    std::vector<ScalarType> mSromMoments;
    std::vector<ScalarType> mTrueMoments;
    std::vector<ScalarType> mMomentErrors;
    ScalarType mCumulativeDistributionFunctionError;
};
// struct SromProblemOutputStruct

template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmParamStruct
{
    // Stopping criterion
    Plato::algorithm::stop_t mStop;
    // Input parameters
    OrdinalType mMaxNumIterations;
    ScalarType mStagnationTolerance;
    ScalarType mMinPenaltyParameter;
    ScalarType mFeasibilityTolerance;
    ScalarType mMaxTrustRegionRadius;
    ScalarType mActualReductionTolerance;
    ScalarType mPenaltyParameterScaleFactor;
    // Output parameters
    ScalarType mObjectiveValue;
    ScalarType mConstraintValue;
    /*! @brief Default constructor */
    AlgorithmParamStruct() :
            mStop(Plato::algorithm::NOT_CONVERGED),
            mMaxNumIterations(1000),
            mStagnationTolerance(1e-12),
            mMinPenaltyParameter(1e-5),
            mFeasibilityTolerance(1e-4),
            mMaxTrustRegionRadius(1e1),
            mActualReductionTolerance(1e-12),
            mPenaltyParameterScaleFactor(1.2),
            mObjectiveValue(std::numeric_limits<ScalarType>::max()),
            mConstraintValue(std::numeric_limits<ScalarType>::max())
    {
    }
};
// struct AlgorithmParamStruct

} // namespace Plato
