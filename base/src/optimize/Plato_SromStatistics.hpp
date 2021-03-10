/*
 * Plato_SromStatistics.hpp
 *
 *  Created on: Jan 31, 2018
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

#ifndef PLATO_SROMSTATISTICS_HPP_
#define PLATO_SROMSTATISTICS_HPP_

#include <cmath>
#include <cassert>

#include "Plato_Vector.hpp"

#define _MATH_DEFINES_DEFINED

namespace Plato
{

/******************************************************************************//**
 * \fn compute_correlation_misfit
 * \brief Compute misfit between Stochastic Reduced Order Model (SROM) correlation matrix \n
 * \f$ \hat{r_ij}(x) \f$ and the truth correlation matrix \f$ r_ij(x) \f$,  defined as \n
 *
 * \f$ \epsilon = \left( \frac{ \hat{r}_ij(x) - r_ij(x) }{ r_ij(x) } \right)^2\f$
 *
 * \param [in] aProbability sample probabilities
 * \param [in] aSamples     set of samples
 * \param [in] aCorrelation correlation matrix
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
ScalarType compute_correlation_misfit
(const Plato::MultiVector<ScalarType, OrdinalType> & aSromCorrelation,
 const Plato::MultiVector<ScalarType, OrdinalType> & aTruthCorrelation)
{
    auto tRandVecDim = aSromCorrelation.getNumVectors();
    if(tRandVecDim != aTruthCorrelation.getNumVectors())
    {
        THROWERR(std::string("Compute Correlation Misfit: Dimension mismatch, SROM and Truth Correlation matrix do not have ")
            + "the same number of rows, i.e. random vector dimensions. SROM Correlation matrix has '" + std::to_string(tRandVecDim)
            + "' rows while the Truth Correlation matrix has '" + std::to_string(aTruthCorrelation.getNumVectors()) + "' rows.")
    }

    // Diagonal entries are variance; thus, these entries are not considered in the misfit/error calculation.
    // Furthermore, the matrix is symmetric, only the upper triangular part is used to compute the misfit term.
    ScalarType tMisfit = 0;
    for(decltype(tRandVecDim) tDimI = 0; tDimI < tRandVecDim - static_cast<OrdinalType>(1); tDimI++)
    {
        for(decltype(tRandVecDim) tDimJ = tDimI + static_cast<OrdinalType>(1); tDimJ < tRandVecDim; tDimJ++)
        {
            auto tMyMisfit = (aSromCorrelation(tDimI, tDimJ) - aTruthCorrelation(tDimI, tDimJ)) / aTruthCorrelation(tDimI, tDimJ);
            tMisfit += (tMyMisfit*tMyMisfit);
        }
    }

    return tMisfit;
}
// function compute_correlation_misfit

/******************************************************************************//**
 * \fn compute_srom_correlation_matrix
 * \brief Compute Stochastic Reduced Order Model (SROM) correlation matrix, defined as \n
 *
 * \f$ \hat{r}_ij(x) = \sum_{k=1}^{N_{\mbox{samples}}} x_i^k x_j^k\f$
 *
 * \param [in] aProbability sample probabilities
 * \param [in] aSamples     set of samples
 * \param [in] aCorrelation correlation matrix
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void compute_srom_correlation_matrix
(const Plato::Vector<ScalarType, OrdinalType> & aProbabilities,
 const Plato::MultiVector<ScalarType, OrdinalType> & aSamples,
 Plato::MultiVector<ScalarType, OrdinalType> & aCorrelation)
{
    Plato::fill(0.0, aCorrelation);
    auto tNumSamples = aProbabilities.size();
    auto tRandVecDim = aCorrelation.getNumVectors();
    for (decltype(tNumSamples) tSampleIndex = 0; tSampleIndex < tNumSamples; tSampleIndex++)
    {
        for (decltype(tRandVecDim) tIndexI = 0; tIndexI < tRandVecDim; tIndexI++)
        {
            aSamples(tIndexI, tSampleIndex);
            for (decltype(tRandVecDim) tIndexJ = 0; tIndexJ < tRandVecDim; tIndexJ++)
            {
                aCorrelation(tIndexI, tIndexJ) += aProbabilities[tSampleIndex] * aSamples(tIndexI, tSampleIndex) * aSamples(tIndexJ, tSampleIndex);
            }
        }
    }
}
// function compute_srom_correlation_matrix

/******************************************************************************//**
 * \brief Evaluate cumulative distribution function (CDF) at this sample point
 * \param [in] aMySample input sample point
 * \param [in] aSigma curvature correction
 * \param [in] aSamples set of samples
 * \param [in] aSamplesProbability set of sample probabilities
 * \return CDF value given a sample computed with a stochastic reduced order model
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline ScalarType compute_srom_cdf(const ScalarType & aMySample,
                                   const ScalarType & aSigma,
                                   const Plato::Vector<ScalarType, OrdinalType> & aSamples,
                                   const Plato::Vector<ScalarType, OrdinalType> & aSamplesProbability)
{
    ScalarType tSum = 0;
    OrdinalType tNumSamples = aSamples.size();
    for(OrdinalType tIndexJ = 0; tIndexJ < tNumSamples; tIndexJ++)
    {
        ScalarType tValue = (aMySample - aSamples[tIndexJ]) / (aSigma * std::sqrt(static_cast<ScalarType>(2)));
        tSum = tSum + aSamplesProbability[tIndexJ] *
                (static_cast<ScalarType>(0.5) * (static_cast<ScalarType>(1) + std::erf(tValue)));
    }
    return (tSum);
}
// function compute_srom_cdf

/******************************************************************************//**
 * \brief Evaluate n-th order raw moment
 * \param [in] aOrder raw moment order
 * \param [in] aSamples set of samples
 * \param [in] aSamplesProbability set of sample probabilities
 * \return evaluation of the n-th order raw moment
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline ScalarType compute_raw_moment(const ScalarType & aOrder,
                                     const Plato::Vector<ScalarType, OrdinalType> & aSamples,
                                     const Plato::Vector<ScalarType, OrdinalType> & aSamplesProbability)
{
    assert(aOrder >= static_cast<OrdinalType>(0));
    assert(aSamples.size() == aSamplesProbability.size());

    ScalarType tOutput = 0;
    OrdinalType tNumSamples = aSamples.size();
    for(OrdinalType tSampleIndex = 0; tSampleIndex < tNumSamples; tSampleIndex++)
    {
        tOutput = tOutput + (aSamplesProbability[tSampleIndex] * std::pow(aSamples[tSampleIndex], aOrder));
    }
    return (tOutput);
}
// function compute_raw_moment

/******************************************************************************//**
 * \brief Evaluate n-th order central moment
 * \param [in] aOrder raw moment order
 * \param [in] aSamples set of samples
 * \param [in] aSamplesProbability set of sample probabilities
 * \return evaluation of the n-th order raw moment
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline ScalarType compute_central_moment(const ScalarType & aOrder,
                                         const Plato::Vector<ScalarType, OrdinalType> & aSamples,
                                         const Plato::Vector<ScalarType, OrdinalType> & aSampleProbabilities)
{
    assert(aOrder >= static_cast<OrdinalType>(0));
    assert(aSamples.size() == aSampleProbabilities.size());

    const ScalarType tOrderRawMoment= 1.0;
    const ScalarType tSampleMean =
            Plato::compute_raw_moment<ScalarType, OrdinalType>(tOrderRawMoment, aSamples, aSampleProbabilities);

    ScalarType tOutput = 0;
    OrdinalType tNumSamples = aSamples.size();
    for(OrdinalType tSampleIndex = 0; tSampleIndex < tNumSamples; tSampleIndex++)
    {
        ScalarType tSampleMinusMean = aSamples[tSampleIndex] - tSampleMean;
        tOutput = tOutput + (aSampleProbabilities[tSampleIndex] * std::pow(tSampleMinusMean, aOrder));
    }
    return (tOutput);
}
// function compute_central_moment

}
// namespace Plato

#endif /* PLATO_SROMSTATISTICS_HPP_ */
