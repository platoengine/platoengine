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
 * Plato_StatisticsOperationsUtilities.cpp
 *
 *  Created on: Jul 6, 2019
 */

#include <cmath>
#include <vector>

#include "PlatoApp.hpp"
#include "Plato_Macros.hpp"
#include "Plato_OperationsUtilities.hpp"
#include "Plato_StatisticsOperationsUtilities.hpp"

namespace Plato
{

double compute_sample_set_mean(const std::vector<Plato::SampleProbPair<double, double>>& aPairs)
{
    if(aPairs.empty() == true)
    {
        THROWERR("INPUT CONTAINER OF SAMPLE-PROBABILITY PAIRS IS EMPTY.\n")
    }

    double tMean = 0;
    for(auto tIterator = aPairs.begin(); tIterator != aPairs.end(); ++ tIterator)
    {
        tMean += tIterator->mProbability * tIterator->mSample;
    }
    return (tMean);
}
// function compute_sample_set_mean

void compute_sample_set_mean(const std::vector<Plato::SampleProbPair<double*, double>>& aPairs, double* aOutput)
{
    if(aPairs.empty() == true)
    {
        THROWERR("INPUT CONTAINER OF SAMPLE-PROBABILITY PAIRS IS EMPTY.\n")
    }

    Plato::zero(aPairs[0].mLength, aOutput);
    for(auto tIterator = aPairs.begin(); tIterator != aPairs.end(); ++tIterator)
    {
        for(size_t tIndex = 0; tIndex < tIterator->mLength; tIndex++)
        {
            aOutput[tIndex] += tIterator->mProbability * tIterator->mSample[tIndex];
        }
    }
}
// function compute_sample_set_mean

double compute_sample_set_standard_deviation(const double& aMean, const std::vector<Plato::SampleProbPair<double, double>>& aPairs)
{
    if(aPairs.empty() == true)
    {
        THROWERR("INPUT CONTAINER OF SAMPLE-PROBABILITY PAIRS IS EMPTY.\n")
    }

    if(std::isfinite(aMean) == false)
    {
        THROWERR("MEAN = " + std::to_string(aMean) + " IS NOT A FINITE NUMBER.\n")
    }

    // STD_DEV(X) = sqrt( E( (X - E(X))^2 )
    double tVariance = 0;
    for(auto tIterator = aPairs.begin(); tIterator != aPairs.end(); ++tIterator)
    {
        auto tSampleMinusMean = tIterator->mSample - aMean;
        tVariance += tIterator->mProbability * std::pow(tSampleMinusMean, 2.0);
    }

    const double tOutput = std::sqrt(tVariance);
    return (tOutput);
}
// function compute_sample_set_standard_deviation

void compute_sample_set_standard_deviation(const double* aMean,
                                           const std::vector<Plato::SampleProbPair<double*, double>>& aPairs,
                                           double* aOutput)
{
    // STD_DEV(X) = sqrt( E( (X - E(X))^2 )
    if(aPairs.empty() == true)
    {
        THROWERR("INPUT CONTAINER OF SAMPLE-PROBABILITY PAIRS IS EMPTY.\n")
    }

    // 1. COMPUTE VARIANCE = E( (X - E(X))^2 )
    for(auto tIterator = aPairs.begin(); tIterator != aPairs.end(); ++tIterator)
    {
        for(size_t tIndex = 0; tIndex < tIterator->mLength; tIndex++)
        {
            auto tSampleMinusMean = tIterator->mSample[tIndex] - aMean[tIndex];
            aOutput[tIndex] += tIterator->mProbability * std::pow(tSampleMinusMean, 2.0);
        }
    }

    // 1. COMPUTE STANDARD DEVIATION = SQRT(VARIANCE)
    for(size_t tIndex = 0; tIndex < aPairs.begin()->mLength; tIndex++)
    {
        auto tStdDev = std::sqrt(aOutput[tIndex]);
        aOutput[tIndex] = tStdDev;
    }
}
// function compute_sample_set_standard_deviation

void compute_sample_set_mean_plus_std_dev_gradient(const double& aCriterionMean,
                                                   const double& aCriterionStdDev,
                                                   const double& aStdDevMultiplier,
                                                   const std::vector<Plato::SampleProbPair<double, double>>& aCriterionValPairs,
                                                   const std::vector<Plato::SampleProbPair<double*, double>>& aCriterionGradPairs,
                                                   double* aOutput)
{
    if(aCriterionValPairs.empty() == true)
    {
        THROWERR("INPUT CONTAINER OF CRITERION VALUE SAMPLE-PROBABILITY PAIRS IS EMPTY.\n")
    }

    if(aCriterionGradPairs.empty() == true)
    {
        THROWERR("INPUT CONTAINER OF CRITERION GRADIENT SAMPLE-PROBABILITY PAIRS IS EMPTY.\n")
    }

    if(aCriterionValPairs.size() != aCriterionGradPairs.size())
    {
        THROWERR("CRITERION VALUE AND CRITERION GRADIENT CONTAINERS DO NOT HAVE THE SAME SIZE.\n")
    }

    if(std::isfinite(aCriterionMean) == false)
    {
        THROWERR("MEAN, MU = " + std::to_string(aCriterionMean) + ", IS NOT A FINITE NUMBER.\n")
    }

    if(std::isfinite(aCriterionStdDev) == false)
    {
        THROWERR("STANDARD DEVIATION, SIGMA = " + std::to_string(aCriterionStdDev) + ", IS NOT A FINITE NUMBER.\n")
    }

    if(std::isfinite(aStdDevMultiplier) == false)
    {
        THROWERR("STANDARD DEVIATION MULTIPLIER, k = " + std::to_string(aStdDevMultiplier) + ", IS NOT A FINITE NUMBER.\n")
    }

    auto tValueIter = aCriterionValPairs.begin();
    Plato::zero(aCriterionGradPairs[0].mLength, aOutput);
    for(auto tGradientIter = aCriterionGradPairs.begin(); tGradientIter != aCriterionGradPairs.end(); ++tGradientIter)
    {
        auto tStdDevMultOverStdDev = aStdDevMultiplier / aCriterionStdDev;
        auto tMultiplier = static_cast<double>(1.0) + (tStdDevMultOverStdDev * (tValueIter->mSample - aCriterionMean));

        for(size_t tIndex = 0; tIndex < tGradientIter->mLength; tIndex++)
        {
            aOutput[tIndex] += tMultiplier * tGradientIter->mProbability * tGradientIter->mSample[tIndex];
        }

        ++tValueIter;
    }
}
// function compute_sample_set_mean_plus_std_dev_gradient

}
// namespace Plato
