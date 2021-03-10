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

enum struct SromInitialGuess
{
    random = 0, uniform = 1
};
// struct SromInitialGuess

enum struct DistributionName
{
    beta = 0, normal = 1, uniform = 2, undefined = 3
};
// struct DistributionName

template<typename ScalarType, typename OrdinalType = size_t>
struct SromInputs
{
    Plato::SromInitialGuess mInitialGuess; /*!< initial guess type, options: random, uniform */
    Plato::DistributionName mDistribution; /*!< distribution type, options: beta, normal, uniform */

    ScalarType mMean;       /*!< distribution's mean */
    ScalarType mVariance;   /*!< distribution's variance */
    ScalarType mLowerBound; /*!< distribution's lower bound */
    ScalarType mUpperBound; /*!< distribution's upper bound */

    ScalarType mMomentErrorCriterionWeight;            /*!< weight on moment misfit term in the SROM objective function */
    ScalarType mCorrelationErrorCriterionWeight;       /*!< weight on correlation misfit term in the SROM objective function */
    ScalarType mCumulativeDistributionFuncErrorWeight; /*!< weight on cumulative distribution function misfit term in the SROM objective function */

    OrdinalType mDimensions = 1;            /*!< random vector dimensions */
    OrdinalType mRandomSeed = 2;            /*!< random seed */
    OrdinalType mNumSamples;                /*!< number of SROM samples */
    OrdinalType mNumMonteCarloSamples;      /*!< number of Monte Carlo samples */
    OrdinalType mMaxNumDistributionMoments; /*!< number of raw moments to match in the SROM optimization problem, if zero, then use default = 4 */

    std::string mCorrelationMatrixFilename; /*!< name of the file containing the truth correlation matrix */

    SromInputs() :   // default Constructor
        mInitialGuess(Plato::SromInitialGuess::uniform),
        mDistribution(Plato::DistributionName::beta),
        mMean(0.),
        mVariance(0.),
        mLowerBound(0.),
        mUpperBound(0.),
        mMomentErrorCriterionWeight(1.0),
        mCorrelationErrorCriterionWeight(1.0),
        mCumulativeDistributionFuncErrorWeight(1.0),
        mNumSamples(0),
        mNumMonteCarloSamples(1000),
        mMaxNumDistributionMoments(4)
    {
    }
};
// struct UncertaintyInputStruct

template<typename ScalarType>
struct SromOutputs
{
    std::vector<ScalarType> mProbabilities;         /*!< SROM probabilities */
    std::vector<std::vector<ScalarType>> mSamples;  /*!< SROM samples */
};
// struct UncertaintyOutputStruct

template<typename ScalarType>
struct SromDiagnostics
{
    // Diagnostics - secondary outputs
    bool mOutputDiagnostics = false;       /*!< output diagnostic flag, default = false */

    std::vector<ScalarType> mSromCDF;      /*!< cumulative distribution function estimate */
    std::vector<ScalarType> mTrueCDF;      /*!< true cumulative distribution function */
    std::vector<ScalarType> mSromMoments;  /*!< raw moments estimates */
    std::vector<ScalarType> mTrueMoments;  /*!< true raw moments */
    std::vector<ScalarType> mMomentErrors; /*!< misfit between true and raw moment estimates */

    ScalarType mCumulativeDistributionFunctionError; /*!< misfit between true and cumulative distribution function estimate */
};
// struct SromProblemOutputStruct

} // namespace Plato
