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
 * Plato_StatisticsUtils.hpp
 *
 *  Created on: Jan 31, 2018
 */

#pragma once

#include <cmath>

#include "Plato_StandardVector.hpp"
#include "Plato_ReductionOperations.hpp"

#define _MATH_DEFINES_DEFINED

namespace Plato
{

/******************************************************************************//**
 * @brief Compute mean of the elements in the input vector
 * @param [in] aReductions common parallel programming operations (e.g. reductions)
 * @param [in] aInput input container
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline ScalarType mean(const Plato::ReductionOperations<ScalarType, OrdinalType> & aReductions,
                       const Plato::Vector<ScalarType, OrdinalType> & aInput)
{
    const OrdinalType tSize = 1;
    const OrdinalType tELEMENT_INDEX = 0;
    Plato::StandardVector<ScalarType, OrdinalType> tWork(tSize);

    tWork[tELEMENT_INDEX] = aInput.size(); /* local number of elements */
    const OrdinalType tGlobalNumElements = aReductions.sum(tWork);
    ScalarType tOutput = aReductions.sum(aInput);
    tOutput = tOutput / tGlobalNumElements;
    return (tOutput);
}
// function mean

/******************************************************************************//**
 * @brief Compute standard deviation of the elements in the input vector
 * @param [in] aMean mean of the elements in the input vector
 * @param [in] aInput input container
 * @param [in] aReductions common parallel programming operations (e.g. reductions)
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline ScalarType standard_deviation(const ScalarType & aMean,
                                     const Plato::Vector<ScalarType, OrdinalType> & aInput,
                                     const Plato::ReductionOperations<ScalarType, OrdinalType> & aReductions)
{
    const OrdinalType tSize = 1;
    const OrdinalType tELEMENT_INDEX = 0;
    Plato::StandardVector<ScalarType, OrdinalType> tWork(tSize);

    const OrdinalType tLocalNumElements = aInput.size();
    for(OrdinalType tIndex = 0; tIndex < tLocalNumElements; tIndex++)
    {
        const ScalarType tMisfit = aInput[tIndex] - aMean;
        tWork[tELEMENT_INDEX] += tMisfit * tMisfit;
    }

    ScalarType tOutput = aReductions.sum(tWork);
    tWork[tELEMENT_INDEX] = tLocalNumElements;
    const ScalarType tGlobalNumElements = aReductions.sum(tWork);
    tOutput = tOutput / (tGlobalNumElements - static_cast<OrdinalType>(1));
    tOutput = std::pow(tOutput, static_cast<ScalarType>(0.5));

    return (tOutput);
}
// function standard_deviation

template<typename OrdinalType>
inline OrdinalType factorial(const OrdinalType & aInput)
{
    OrdinalType tOutput = 1;
    for(OrdinalType tIndex = 1; tIndex <= aInput; tIndex++)
    {
        tOutput *= tIndex;
    }
    return (tOutput);
}
// function factorial

template<typename ScalarType>
inline ScalarType pochhammer_symbol(const ScalarType & aX, const ScalarType & aN)
{
    const ScalarType tCoefficientOne = std::tgamma(aX + aN);
    const ScalarType tCoefficientTwo = std::tgamma(aX);
    ScalarType tOutput = tCoefficientOne / tCoefficientTwo;
    tOutput = std::isfinite(tOutput) ? tOutput : static_cast<ScalarType>(0);
    return (tOutput);
}
// function pochhammer_symbol

template<typename ScalarType>
inline void shape_parameters(const ScalarType & aMinValue,
                             const ScalarType & aMaxValue,
                             const ScalarType & aMean,
                             const ScalarType & aSigma,
                             ScalarType & aAlpha,
                             ScalarType & aBeta)
{
    // Scale mean/variance to lie in [0,1] for the standard beta distribution:
    ScalarType tMeanStd = (aMean - aMinValue) / (aMaxValue - aMinValue);
    ScalarType tVarianceStd = (static_cast<ScalarType>(1) / (aMaxValue - aMinValue))
            * (static_cast<ScalarType>(1) / (aMaxValue - aMinValue)) * aSigma;
    // Compute shape parameters for Beta distributions based on standard mean/variance:
    aAlpha = tMeanStd
            * (tMeanStd * (static_cast<ScalarType>(1) - tMeanStd) / tVarianceStd - static_cast<ScalarType>(1));
    aBeta = (tMeanStd * (static_cast<ScalarType>(1) - tMeanStd) / tVarianceStd - static_cast<ScalarType>(1)) - aAlpha;
}
// function shape_parameters

template<typename ScalarType, typename OrdinalType>
inline void make_uniform_sample(Plato::Vector<ScalarType, OrdinalType>& aInitialGuess)
{
    assert(aInitialGuess.size() > static_cast<OrdinalType>(0));
    const OrdinalType tNumSample = aInitialGuess.size();
    for(OrdinalType tIndex = 0; tIndex < tNumSample; tIndex++)
    {
        ScalarType tValue = (1. / static_cast<ScalarType>(tNumSample + 1u));
        aInitialGuess[tIndex] = static_cast<ScalarType>(tIndex + 1u) * tValue;
    }
}
// function make_uniform_sample

} // namespace Plato
