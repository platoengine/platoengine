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
 * Plato_StatisticsOperationsUtilities.hpp
 *
 *  Created on: Jul 6, 2019
 */

#pragma once

#include <vector>
#include <string>

class PlatoApp;

namespace Plato
{

template<typename SampleType, typename ProbabilityType>
struct SampleProbPair
{
    size_t mLength;
    SampleType mSample;
    ProbabilityType mProbability;
};

/******************************************************************************//**
 * @brief Compute the sample set's mean
 * @param [in] aPairs sample-probability pairs
 * @return sample set mean
 **********************************************************************************/
double compute_sample_set_mean(const std::vector<Plato::SampleProbPair<double, double>>& aPairs);

/******************************************************************************//**
 * @brief Compute the sample set's mean
 * NOTE: aOutput values are set to zero inside the function.
 * @param [in] aPairs sample-probability pairs
 * @param [out] aOutput output array
 **********************************************************************************/
void compute_sample_set_mean(const std::vector<Plato::SampleProbPair<double*, double>>& aPairs, double* aOutput);

/******************************************************************************//**
 * @brief Compute the sample set's standard deviation
 * @param [in] aMean sample set mean
 * @param [in] aPairs sample-probability pairs
 * @return standard deviation
 **********************************************************************************/
double compute_sample_set_standard_deviation
(const double& aMean, const std::vector<Plato::SampleProbPair<double, double>>& aPairs);

/******************************************************************************//**
 * @brief Compute the sample set's standard deviation
 * @param [in] aMean sample set mean
 * @param [in] aPairs sample-probability pairs
 * @param [out] aOutput standard deviation field
 **********************************************************************************/
void compute_sample_set_standard_deviation
(const double* aMean, const std::vector<Plato::SampleProbPair<double*, double>>& aPairs, double* aOutput);

/******************************************************************************//**
 * @brief Compute the gradient of the mean plus standard deviation criterion, i.e
 * compute \f$\frac{\partial{J}(x)}{\partial{x}}, where J(x) = E(f(x)) + k*Std(f(x))\f$.
 * Here, \f$f(x)\f$ denotes a criterion, \f$E\f$ denotes expectation and \f$Std\f$
 * denotes standard deviation.
 * NOTE: aOutput values are set to zero inside the function.
 * @param [in] aCriterionMean sample set mean
 * @param [in] aCriterionStdDev sample set standard deviation
 * @param [in] aStdDevMultiplier standard deviation multiplier
 * @param [in] aCriterionValPairs criterion value sample-probability pairs
 * @param [in] aCriterionGradPairs criterion gradient sample-probability pairs
 * @param [out] aOutput output gradient
 **********************************************************************************/
void compute_sample_set_mean_plus_std_dev_gradient(const double& aCriterionMean,
                                                   const double& aCriterionStdDev,
                                                   const double& aStdDevMultiplier,
                                                   const std::vector<Plato::SampleProbPair<double, double>>& aCriterionValPairs,
                                                   const std::vector<Plato::SampleProbPair<double*, double>>& aCriterionGradPairs,
                                                   double* aOutput);

}
// namespace Plato
