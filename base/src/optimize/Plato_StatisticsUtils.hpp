/*
//\HEADER
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
// Questions? Contact the Plato team (plato3D-help\sandia.gov)
// *************************************************************************
//\HEADER
*/

/*
 * Plato_StatisticsUtils.hpp
 *
 *  Created on: Jan 31, 2018
 */

#pragma once

#include <cmath>
#include <cstdlib>

#include "Plato_Macros.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_ReductionOperations.hpp"

#define _MATH_DEFINES_DEFINED

namespace Plato
{

/******************************************************************************//**
 * \brief Compute mean of the elements in the input vector
 * \param [in] aReductions common parallel programming operations (e.g. reductions)
 * \param [in] aInput input container
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
 * \brief Compute standard deviation of the elements in the input vector
 * \param [in] aMean mean of the elements in the input vector
 * \param [in] aInput input container
 * \param [in] aReductions common parallel programming operations (e.g. reductions)
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
    assert(tGlobalNumElements > static_cast<OrdinalType>(0));
    if(tGlobalNumElements > static_cast<OrdinalType>(1))
    {
        tOutput = tOutput / (tGlobalNumElements - static_cast<OrdinalType>(1));
        tOutput = std::pow(tOutput, static_cast<ScalarType>(0.5));
    }
    else
    {
        tOutput = 0;
    }

    return (tOutput);
}
// function standard_deviation

/******************************************************************************//**
 * \brief Compute the factorial of a positive integer \f$ n \f$, denoted by \n
 * \f$ n! \f$, is the product of all positive integers less than or equal to \f$ n \f$.
 * \param [in] aInput positive integer
 * \return factorial product
**********************************************************************************/
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

/******************************************************************************//**
 * \brief Compute uniform initial guess, where \f$ x_i = \frac{1}{N}*i \f$,
 * where \f$ i\in\{1,N\} \f$ and \f$ N \f$ is the total number of samples.
 * \param [in\out] aInput sample-probability pairs
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void uniform_samples_initial_guess
(Plato::MultiVector<ScalarType, OrdinalType>& aInput)
{
    if(aInput.getNumVectors() <= static_cast<OrdinalType>(0))
    {
        THROWERR("Uniform Samples Initial Guess: input multi-vector is empty.")
    }
    // last dimension, i.e. N, is for the probabilities. random vectors dimensions are from 1 to N-1
    auto tRandomVectorDim = aInput.getNumVectors() - static_cast<OrdinalType>(1);
    for(decltype(tRandomVectorDim) tDim = 0; tDim < tRandomVectorDim; tDim++)
    {
        auto tNumSample = aInput[tDim].size();
        auto tValue = (static_cast<ScalarType>(1.0) / static_cast<ScalarType>(tNumSample + 1u));
        for(decltype(tNumSample) tSample = 0; tSample < tNumSample; tSample++)
        {
            aInput(tDim, tSample) = static_cast<ScalarType>(tSample + 1u + tDim) * tValue;
        }
    }
}
// function uniform_samples_initial_guess

/******************************************************************************//**
 * \brief Compute initial guess for random vector of samples, values are between 0.0 and 1.0.
 * \param [in]  aLower lower bounds
 * \param [in]  aUpper upper bounds
 * \param [out] aGuess samples initial guess
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void random_sample_initial_guess
(const Plato::MultiVector<ScalarType, OrdinalType>& aLower,
 const Plato::MultiVector<ScalarType, OrdinalType>& aUpper,
 Plato::MultiVector<ScalarType, OrdinalType>& aGuess)
{
    if(aLower.getNumVectors() <= 0)
    {
        THROWERR("Random Samples Initial Guess: Lower bound multi-vector is empty.")
    }
    if(aUpper.getNumVectors() <= 0)
    {
        THROWERR("Random Samples Initial Guess: Upper bound multi-vector is empty.")
    }
    if(aGuess.getNumVectors() <= 0)
    {
        THROWERR("Random Samples Initial Guess: Initial guess multi-vector is empty.")
    }
    if(aLower.getNumVectors() != aUpper.getNumVectors())
    {
        THROWERR(std::string("Random Samples Initial Guess: dimensions mismatch between lower and upper bound multi-vector. ")
            + "Number of Lower Bound Vectors is '" + std::to_string(aLower.getNumVectors()) + "' and number of Upper Bound "
            + "Vectors is '" + std::to_string(aUpper.getNumVectors()) + "'.")
    }
    if(aGuess.getNumVectors() != aUpper.getNumVectors())
    {
        THROWERR(std::string("Random Samples Initial Guess: dimensions mismatch between initial guess and lower/upper bound multi-vectors. ")
            + "Number of Initial Guess Vectors is '" + std::to_string(aGuess.getNumVectors()) + "' and number of Lower/Upper Bound Vectors is '"
            + std::to_string(aUpper.getNumVectors()) + "'.")
    }

    // last dimension, i.e. N, is for the probabilities. random vectors dimensions are from 1 to N-1
    auto tRandomVectorDim = aGuess.getNumVectors() - static_cast<OrdinalType>(1);
    for(decltype(tRandomVectorDim) tDim = 0; tDim < tRandomVectorDim; tDim++)
    {
        auto tNumSample = aGuess[tDim].size();
        for(decltype(tNumSample) tSample = 0; tSample < tNumSample; tSample++)
        {
            auto tNormalizedRandNum = static_cast<ScalarType>(std::rand()) / static_cast<ScalarType>(RAND_MAX);
            aGuess(tDim, tSample) = aLower(tDim, tSample) + ((aUpper(tDim, tSample) - aLower(tDim, tSample)) * tNormalizedRandNum);
        }
    }
}
// function random_sample_initial_guess

} // namespace Plato
