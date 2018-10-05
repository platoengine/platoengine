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
    OrdinalType mNumMonteCarloSamples;
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
            mNumMonteCarloSamples(1000),
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
    ScalarType mMinPenaltyParameter;
    ScalarType mFeasibilityTolerance;
    ScalarType mMaxTrustRegionRadius;
    ScalarType mActualReductionTolerance;
    ScalarType mPenaltyParameterScaleFactor;
    ScalarType mObjectiveStagnationTolerance;
    // Output parameters
    ScalarType mObjectiveValue;
    ScalarType mConstraintValue;
    /*! @brief Default constructor */
    AlgorithmParamStruct() :
            mStop(Plato::algorithm::NOT_CONVERGED),
            mMaxNumIterations(1000),
            mMinPenaltyParameter(1e-5),
            mFeasibilityTolerance(1e-4),
            mMaxTrustRegionRadius(1e1),
            mActualReductionTolerance(1e-12),
            mPenaltyParameterScaleFactor(1.2),
            mObjectiveStagnationTolerance(1e-12),
            mObjectiveValue(std::numeric_limits<ScalarType>::max()),
            mConstraintValue(std::numeric_limits<ScalarType>::max())
    {
    }
};
// struct AlgorithmParamStruct

} // namespace Plato
