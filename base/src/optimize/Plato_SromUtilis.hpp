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
 * Plato_SromUtilis.hpp
 *
 *  Created on: Sep 27, 2018
 */

#pragma once

#include <vector>
#include <string>
#include <cassert>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "Plato_Macros.hpp"
#include "Plato_ErrorChecks.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_CommWrapper.hpp"
#include "Plato_Distribution.hpp"
#include "Plato_SromObjective.hpp"
#include "Plato_SromProbDataStruct.hpp"

namespace Plato
{

/******************************************************************************//**
 *
 * \brief Check cumulative distribution function container dimension.
 * \param [in] aNumSamples number of samples
 * \param [in] aCDF cumulative distribution function
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void check_cdf_container_size(const OrdinalType & aNumSamples, const Plato::Vector<ScalarType, OrdinalType> & aCDF)
{
    try
    {
        if(aCDF.size() != aNumSamples)
        {
            std::ostringstream tMessage;
            tMessage << "DIMENSION MISMATCH! EXPECTED CUMULATIVE DISTRIBUTION FUNCTION CONTAINER OF SIZE = "
            << aNumSamples << ". INPUT CUMULATIVE DISTRIBUTION FUNCTION CONTAINER HAS SIZE = " << aCDF.size()
            << ".\n";
            throw std::invalid_argument(tMessage.str().c_str());
        }
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        throw tErrorMsg;
    }
}

/******************************************************************************//**
 *
 * \brief Check sample's container dimension.
 * \param [in] aNumSamples number of samples
 * \param [in] aSamples samples
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void check_samples_container_size(const OrdinalType & aNumSamples, const Plato::Vector<ScalarType, OrdinalType> & aSamples)
{
    try
    {
        if(aSamples.size() != aNumSamples)
        {
            std::ostringstream tMessage;
            tMessage << "DIMENSION MISMATCH! EXPECTED SAMPLES CONTAINER OF SIZE = " << aNumSamples
            << ". INPUT SAMPLES CONTAINER HAS SIZE = " << aSamples.size() << ".\n";
            throw std::invalid_argument(tMessage.str().c_str());
        }
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        throw tErrorMsg;
    }
}

/******************************************************************************//**
 *
 * \brief Normalize sample value, i.e. map sample from [LB,UP] interval to [0,1] interval.
 * \param [in] aLowerBound lower bound (LB)
 * \param [in] aUpperBound upper bound (UB)
 * \param [in] aSampleValue unnormalized sample value
 * @return normalized sample value
 *
**********************************************************************************/
template<typename ScalarType>
inline ScalarType normalize(const ScalarType& aLowerBound, const ScalarType& aUpperBound, const ScalarType& aSampleValue)
{
    return ((aSampleValue - aLowerBound) / (aUpperBound - aLowerBound));
}

/******************************************************************************//**
 *
 * \brief Undo normalization, i.e. map sample from [0,1] interval to [LB,UP] interval.
 * \param [in] aLowerBound lower bound (LB)
 * \param [in] aUpperBound upper bound (UB)
 * \param [in] aSampleValue normalized sample value
 * @return unnormalized sample value
 *
**********************************************************************************/
template<typename ScalarType>
inline ScalarType undo_normalization(const ScalarType& aLowerBound, const ScalarType& aUpperBound, const ScalarType& aSampleValue)
{
    return (aLowerBound + (aUpperBound - aLowerBound) * aSampleValue);
}

/******************************************************************************//**
 *
 * \brief Check sample's container dimension.
 * \param [in] aDistribution probability distribution function interface
 * \param [in] aInputParamUQ statistics core input parameters
 * \param [in,out] aSamples monte carlo samples
 * \param [in,out] aCDF monte carlo cumulative distribution function values
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void compute_monte_carlo_data(const OrdinalType & aNumMonteCarloSamples,
                                     const Plato::Distribution<ScalarType, OrdinalType> & aDistribution,
                                     Plato::Vector<ScalarType, OrdinalType> & aSamples,
                                     Plato::Vector<ScalarType, OrdinalType> & aCDF)
{
    const OrdinalType tLength = aNumMonteCarloSamples + static_cast<OrdinalType>(1);
    try
    {
        Plato::check_cdf_container_size(tLength, aCDF);
        Plato::check_samples_container_size(tLength, aSamples);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        THROWERR(tErrorMsg.what())
    }

    const ScalarType tDelta = static_cast<ScalarType>(1.0) / aNumMonteCarloSamples;
    for(OrdinalType tIndex = 1; tIndex < tLength; tIndex++)
    {
        OrdinalType tPreviousIndex = tIndex - static_cast<OrdinalType>(1);
        aSamples[tIndex] = aSamples[tPreviousIndex] + tDelta;
        const ScalarType tValue = aDistribution.cdf(aSamples[tIndex]);
        aCDF[tIndex] = tValue;
    }
}

/******************************************************************************//**
 *
 * \brief Check sample's container dimension.
 * \param [in] aLowerBound range's lower bound
 * \param [in] aUpperBound range's upper bound
 * \param [in] aNormalizedSamples normalized sample values
 * \param [in,out] aUnnormalizedSamples unnnormalized sample values
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void compute_unnormalized_samples(const ScalarType & aLowerBound,
                                         const ScalarType & aUpperBound,
                                         const Plato::Vector<ScalarType, OrdinalType> & aNormalizedSamples,
                                         Plato::Vector<ScalarType, OrdinalType> & aUnnormalizedSamples)
{
    try
    {
        Plato::error::check_dimension_mismatch(aNormalizedSamples, aUnnormalizedSamples);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        THROWERR(tErrorMsg.what())
    }

    for(OrdinalType tIndex = 0; tIndex < aUnnormalizedSamples.size(); tIndex++)
    {
        aUnnormalizedSamples[tIndex] = Plato::undo_normalization(aLowerBound, aUpperBound, aNormalizedSamples[tIndex]);
    }
}

/******************************************************************************//**
 *
 * \brief Print cumulative distribution function output data to file
 * \param [in] aCommWrapper distributed memory communicator wrapper
 * \param [in] aSromCDF stochastic reduced order model cumulative distribution function values
 * \param [in] aMonteCarloCDF monte carlo cumulative distribution function values
 * \param [in] aSamples monte carlo samples
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void print_cumulative_distribution_function_to_file(const Plato::CommWrapper & aCommWrapper,
                                                           const Plato::Vector<ScalarType, OrdinalType> & aSromCDF,
                                                           const Plato::Vector<ScalarType, OrdinalType> & aMonteCarloCDF,
                                                           const Plato::Vector<ScalarType, OrdinalType> & aSamples)
{
    if(aCommWrapper.myProcID() == static_cast<OrdinalType>(0))
    {
        std::ofstream tOutputFile;
        tOutputFile.open("plato_cdf_output.txt");
        Plato::print_plato_license(tOutputFile);
        tOutputFile << std::right << "    Samples" << std::setw(18) << "MC-CDF" << std::setw(18) << "SROM-CDF" << "\n"
                                                                                                 << std::flush;
        const OrdinalType tNumSamples = aSamples.size();
        for(OrdinalType tIndex = 0; tIndex < tNumSamples; tIndex++)
        {
            tOutputFile << std::scientific << std::setprecision(8) << std::right << aSamples[tIndex] << std::setw(18)
            << aMonteCarloCDF[tIndex] << std::setw(18) << aSromCDF[tIndex] << "\n" << std::flush;
        }
        tOutputFile.close();
    }
}

/******************************************************************************//**
 *
 * \brief output cumulative distribution function data to file (main interface)
 * \param [in] aCommWrapper distributed memory communicator wrapper
 * \param [in] aSromCDF stochastic reduced order model cumulative distribution function values
 * \param [in] aMonteCarloCDF monte carlo cumulative distribution function values
 * \param [in] aSamples monte carlo samples
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void output_cumulative_distribution_function(const Plato::CommWrapper & aCommWrapper,
                                                    const Plato::Vector<ScalarType, OrdinalType> & aSromCDF,
                                                    const Plato::Vector<ScalarType, OrdinalType> & aMonteCarloCDF,
                                                    const Plato::Vector<ScalarType, OrdinalType> & aSamples)
{
    try
    {
        Plato::error::check_null_comm(aCommWrapper);
        Plato::error::check_dimension_mismatch(aSromCDF, aMonteCarloCDF);
        Plato::error::check_dimension_mismatch(aSamples, aMonteCarloCDF);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        THROWERR(tErrorMsg.what())
    }

    Plato::print_cumulative_distribution_function_to_file(aCommWrapper, aSromCDF, aMonteCarloCDF, aSamples);
}

/******************************************************************************//**
 *
 * \brief Compute cumulative distribution function from Stochastic Reduced Order Model (SROM) approximation.
 * \param [in] aSamplesMC Monte Carlo samples
 * \param [in] aSamplesSROM SROM samples from optimization problem
 * \param [in] aProbsSROM SROM sample probabilities from optimization problem
 * \param [in,out] aProbsSROM aSromCDF SROM cumulative distribution function
 * \param [in] aSigma smoothing parameter (Default = 1e-7)
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void compute_srom_cdf_plot(const Plato::Vector<ScalarType, OrdinalType>& aSamplesMC,
                                  const Plato::Vector<ScalarType, OrdinalType>& aSamplesSROM,
                                  const Plato::Vector<ScalarType, OrdinalType>& aProbsSROM,
                                  Plato::Vector<ScalarType, OrdinalType>& aSromCDF,
                                  ScalarType aSigma = 1e-7)
{
    try
    {
        Plato::error::check_dimension_mismatch(aSromCDF, aSamplesMC);
        Plato::error::check_dimension_mismatch(aSamplesSROM, aProbsSROM);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        THROWERR(tErrorMsg.what())
    }

    const ScalarType tConstant = std::sqrt(2);
    const OrdinalType tNumSamplesMC = aSamplesMC.size();
    const OrdinalType tNumSamplesSROM = aSamplesSROM.size();

    for(OrdinalType tIndexMC = 0; tIndexMC < tNumSamplesMC; tIndexMC++)
    {
        ScalarType tSum = 0;
        for(OrdinalType tIndexSROM = 0; tIndexSROM < tNumSamplesSROM; tIndexSROM++)
        {
            ScalarType tArg = (aSamplesMC[tIndexMC] - aSamplesSROM[tIndexSROM]) / (aSigma * tConstant);
            tSum = tSum + aProbsSROM[tIndexSROM] * (static_cast<ScalarType>(0.5) * (static_cast<ScalarType>(1) + erf(tArg)));
            tSum = tSum >= static_cast<ScalarType>(1.0) ? static_cast<ScalarType>(1.0) : tSum;
        }
        aSromCDF[tIndexMC] = tSum;
    }
}

/******************************************************************************//**
 *
 * \brief Save solution from the stochastic reduced order model (SROM) problem
 * \param [in]   aInputMetaData    SROM problem input metadata
 * \param [in]   aOptimalControls  SROM problem solution
 * \param [out]  aOutputMetaData   SROM problem output metadata
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void save_srom_solution
(const Plato::SromInputs<ScalarType, OrdinalType>& aInputMetaData,
 const Plato::MultiVector<ScalarType, OrdinalType>& aSolution,
 Plato::SromOutputs<ScalarType>& aOutputMetaData)
{
    // NOTE: The upper/lower bounds are not always defined. For example, normal dist'n what should be done?
    // confirmed accurate for beta and uniform dist'n alone
    auto tNumSamples = aInputMetaData.mNumSamples;
    auto tRandVecDim = aInputMetaData.mDimensions;
    for (decltype(tRandVecDim) tDimIndex = 0; tDimIndex < tRandVecDim; tDimIndex++)
    {
        aOutputMetaData.mSamples.push_back(std::vector<ScalarType>());
        for (decltype(tNumSamples) tSampleIndex = 0; tSampleIndex < tNumSamples; tSampleIndex++)
        {
            // undo scaling
            const auto tUnscaledValue = aSolution(tDimIndex, tSampleIndex);
            auto tScaledValue = aInputMetaData.mLowerBound
                + (aInputMetaData.mUpperBound - aInputMetaData.mLowerBound) * tUnscaledValue;
            aOutputMetaData.mSamples.rbegin()->push_back(tScaledValue);
        }
    }

    for (decltype(tNumSamples) tSampleIndex = 0; tSampleIndex < tNumSamples; tSampleIndex++)
    {
        aOutputMetaData.mProbabilities.push_back(aSolution(tRandVecDim, tSampleIndex));
    }
}

/******************************************************************************//**
 *
 * \brief Save diagnostics associated with the stochastic reduced order model (SROM) moment metrics
 * \param [in] aSromObjective objective function interface for SROM problem
 * \param [in,out] aOutput diagnostics associated with the SROM optimization problem
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void save_srom_moments_diagnostics(const Plato::SromObjective<ScalarType, OrdinalType>& aSromObjective,
                                          Plato::SromDiagnostics<ScalarType>& aOutput)
{
    // Resize if necessary
    const OrdinalType tNumMoments = aSromObjective.getMaxNumMoments();
    if(aOutput.mSromMoments.empty() == true || static_cast<OrdinalType>(aOutput.mSromMoments.size()) != tNumMoments)
    {
        aOutput.mSromMoments.clear();
        aOutput.mSromMoments.resize(tNumMoments);
    }
    if(aOutput.mTrueMoments.empty() == true || static_cast<OrdinalType>(aOutput.mTrueMoments.size()) != tNumMoments)
    {
        aOutput.mTrueMoments.clear();
        aOutput.mTrueMoments.resize(tNumMoments);
    }
    if(aOutput.mMomentErrors.empty() == true || static_cast<OrdinalType>(aOutput.mMomentErrors.size()) != tNumMoments)
    {
        aOutput.mMomentErrors.clear();
        aOutput.mMomentErrors.resize(tNumMoments);
    }

    // SET OUTPUT ASSOCIATED WITH MOMENT-BASED METRICS
    const OrdinalType tNUM_RANDOM_DIM = 1; // ONLY SINGLE DIMENSION SUPPORT
    for(OrdinalType tRandomDimIndex = 0; tRandomDimIndex < tNUM_RANDOM_DIM; tRandomDimIndex++)
    {
        for(OrdinalType tMomentIndex = 0; tMomentIndex < tNumMoments; tMomentIndex++)
        {
            aOutput.mSromMoments[tMomentIndex] = aSromObjective.getSromMoments()[tMomentIndex];
            aOutput.mTrueMoments[tMomentIndex] = aSromObjective.getTrueMoments()[tMomentIndex];
            aOutput.mMomentErrors[tMomentIndex] = aSromObjective.getMomentError(tRandomDimIndex)[tMomentIndex];
        }
    }
}

/******************************************************************************//**
 *
 * \brief Save diagnostics associated with the SROM CDF
 * \param [in] aSromObjective objective function interface for SROM problem
 * \param [in,out] aOutput diagnostics associated with the SROM optimization problem
 *
 * NOTE: CDF = cumulative distribution function; 2) SROM = stochastic reduced order model
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void save_srom_cdf_diagnostics(const Plato::SromObjective<ScalarType, OrdinalType>& aSromObjective,
                                      Plato::SromDiagnostics<ScalarType>& aOutput)
{
    // Resize if necessary
    const OrdinalType tNumSamples = aSromObjective.getNumSamples();
    if(aOutput.mSromCDF.empty() == true || static_cast<OrdinalType>(aOutput.mSromCDF.size()) != tNumSamples)
    {
        aOutput.mSromCDF.clear();
        aOutput.mSromCDF.resize(tNumSamples);
    }
    if(aOutput.mTrueCDF.empty() == true || static_cast<OrdinalType>(aOutput.mTrueCDF.size()) != tNumSamples)
    {
        aOutput.mTrueCDF.clear();
        aOutput.mTrueCDF.resize(tNumSamples);
    }

    // SET OUTPUT ASSOCIATED WITH CDF METRICS
    const OrdinalType tNUM_RANDOM_DIM = 1; // ONLY SINGLE DIMENSION SUPPORT
    aOutput.mCumulativeDistributionFunctionError = aSromObjective.getCumulativeDistributionFunctionError();
    for(OrdinalType tRandomDimIndex = 0; tRandomDimIndex < tNUM_RANDOM_DIM; tRandomDimIndex++)
    {
        for(OrdinalType tSampleIndex = 0; tSampleIndex < tNumSamples; tSampleIndex++)
        {
            aOutput.mSromCDF[tSampleIndex] = aSromObjective.getSromCDF(tRandomDimIndex)[tSampleIndex];
            aOutput.mTrueCDF[tSampleIndex] = aSromObjective.getTrueCDF(tRandomDimIndex)[tSampleIndex];
        }
    }
}

} // namespace Plato
