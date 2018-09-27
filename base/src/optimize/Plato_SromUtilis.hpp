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
 * @brief Check cumulative distribution function container dimension.
 * @param [in] aNumSamples number of samples
 * @param [in] aCDF cumulative distribution function
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
            tMessage << "\n\n ******** MESSAGE: DIMENSION MISMATCH! EXPECTED CUMULATIVE DISTRIBUTION FUNCTION CONTAINER OF SIZE = "
            << aNumSamples << ". INPUT CUMULATIVE DISTRIBUTION FUNCTION CONTAINER HAS SIZE = " << aCDF.size()
            << ". ABORT! ******** \n\n";
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
 * @brief Check sample's container dimension.
 * @param [in] aNumSamples number of samples
 * @param [in] aSamples samples
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
            tMessage << "\n\n ******** MESSAGE: DIMENSION MISMATCH! EXPECTED SAMPLES CONTAINER OF SIZE = " << aNumSamples
            << ". INPUT SAMPLES CONTAINER HAS SIZE = " << aSamples.size() << ". ABORT! ******** \n\n";
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
 * @brief Normalize sample value, i.e. map sample from [LB,UP] interval to [0,1] interval.
 * @param [in] aLowerBound lower bound (LB)
 * @param [in] aUpperBound upper bound (UB)
 * @param [in] aSampleValue unnormalized sample value
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
 * @brief Undo normalization, i.e. map sample from [0,1] interval to [LB,UP] interval.
 * @param [in] aLowerBound lower bound (LB)
 * @param [in] aUpperBound upper bound (UB)
 * @param [in] aSampleValue normalized sample value
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
 * @brief Check sample's container dimension.
 * @param [in] aDistribution probability distribution function interface
 * @param [in] aInputParamUQ statistics core input parameters
 * @param [in,out] aSamples monte carlo samples
 * @param [in,out] aCDF monte carlo cumulative distribution function values
 * @param [in] aPrint flag use to enable terminal output (default = false)
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void compute_monte_carlo_data(const OrdinalType & aNumMonteCarloSamples,
                                     const Plato::Distribution<ScalarType, OrdinalType> & aDistribution,
                                     Plato::Vector<ScalarType, OrdinalType> & aSamples,
                                     Plato::Vector<ScalarType, OrdinalType> & aCDF,
                                     bool aPrint = false)
{
    const OrdinalType tLength = aNumMonteCarloSamples + static_cast<OrdinalType>(1);
    try
    {
        Plato::check_cdf_container_size(tLength, aCDF);
        Plato::check_samples_container_size(tLength, aSamples);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        std::ostringstream tMessage;
        tMessage << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\nFUNCTION: " << __PRETTY_FUNCTION__ << "\nLINE: "
                 << __LINE__ << "\n ******** \n\n";
        tMessage << tErrorMsg.what();
        if(aPrint == true)
        {
            std::cout << tMessage.str().c_str() << std::flush;
        }
        throw std::invalid_argument(tMessage.str().c_str());
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
 * @brief Check sample's container dimension.
 * @param [in] aLowerBound range's lower bound
 * @param [in] aUpperBound range's upper bound
 * @param [in] aNormalizedSamples normalized sample values
 * @param [in,out] aUnnormalizedSamples unnnormalized sample values
 * @param [in] aPrint flag use to enable terminal output (default = false)
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void compute_unnormalized_samples(const ScalarType & aLowerBound,
                                         const ScalarType & aUpperBound,
                                         const Plato::Vector<ScalarType, OrdinalType> & aNormalizedSamples,
                                         Plato::Vector<ScalarType, OrdinalType> & aUnnormalizedSamples,
                                         bool aPrint = false)
{
    try
    {
        Plato::error::check_dimension_mismatch(aNormalizedSamples, aUnnormalizedSamples);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        std::ostringstream tMessage;
        tMessage << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\nFUNCTION: " << __PRETTY_FUNCTION__ << "\nLINE: "
                 << __LINE__ << "\n ******** \n\n";
        tMessage << tErrorMsg.what();
        if(aPrint == true)
        {
            std::cout << tMessage.str().c_str() << std::flush;
        }
        throw std::invalid_argument(tMessage.str().c_str());
    }

    for(OrdinalType tIndex = 0; tIndex < aUnnormalizedSamples.size(); tIndex++)
    {
        aUnnormalizedSamples[tIndex] = Plato::undo_normalization(aLowerBound, aUpperBound, aNormalizedSamples[tIndex]);
    }
}

/******************************************************************************//**
 *
 * @brief Print cumulative distribution function output data to file
 * @param [in] aCommWrapper distributed memory communicator wrapper
 * @param [in] aSromCDF stochastic reduced order model cumulative distribution function values
 * @param [in] aMonteCarloCDF monte carlo cumulative distribution function values
 * @param [in] aSamples monte carlo samples
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
 * @brief output cumulative distribution function data to file (main interface)
 * @param [in] aCommWrapper distributed memory communicator wrapper
 * @param [in] aSromCDF stochastic reduced order model cumulative distribution function values
 * @param [in] aMonteCarloCDF monte carlo cumulative distribution function values
 * @param [in] aSamples monte carlo samples
 * @param [in] aPrint flag use to enable terminal output (default = false)
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void output_cumulative_distribution_function(const Plato::CommWrapper & aCommWrapper,
                                                    const Plato::Vector<ScalarType, OrdinalType> & aSromCDF,
                                                    const Plato::Vector<ScalarType, OrdinalType> & aMonteCarloCDF,
                                                    const Plato::Vector<ScalarType, OrdinalType> & aSamples,
                                                    bool aPrint = false)
{
    try
    {
        Plato::error::check_null_comm(aCommWrapper);
        Plato::error::check_dimension_mismatch(aSromCDF, aMonteCarloCDF);
        Plato::error::check_dimension_mismatch(aSamples, aMonteCarloCDF);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        std::ostringstream tMessage;
        tMessage << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\nFUNCTION: " << __PRETTY_FUNCTION__ << "\nLINE: "
                 << __LINE__ << "\n ******** \n\n";
        tMessage << tErrorMsg.what();
        if(aPrint == true)
        {
            std::cout << tMessage.str().c_str() << std::flush;
        }
        throw std::invalid_argument(tMessage.str().c_str());
    }

    Plato::print_cumulative_distribution_function_to_file(aCommWrapper, aSromCDF, aMonteCarloCDF, aSamples);
}

/******************************************************************************//**
 *
 * @brief Compute cumulative distribution function from Stochastic Reduced Order Model (SROM) approximation.
 * @param [in] aSamplesMC Monte Carlo samples
 * @param [in] aSamplesSROM SROM samples from optimization problem
 * @param [in] aProbsSROM SROM sample probabilities from optimization problem
 * @param [in,out] aProbsSROM aSromCDF SROM cumulative distribution function
 * @param [in] aSigma smoothing parameter (Default = 1e-7)
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
        std::ostringstream tMessage;
        tMessage << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\nFUNCTION: " << __PRETTY_FUNCTION__ << "\nLINE: "
                 << __LINE__ << "\n ******** \n\n";
        tMessage << tErrorMsg.what();
        std::cout << tMessage.str().c_str() << std::flush;
        throw std::invalid_argument(tMessage.str().c_str());
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
 * @brief Save solution from the stochastic reduced order model (SROM) optimization problem
 * @param [in] aStatsInputs data structure with inputs for probability distribution function
 * @param [in] aOptimalControls optimal controls from SROM problem
 * @param [in,out] aOutput data structure used to save solution from SROM problem
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void save_srom_solution(const Plato::UncertaintyInputStruct<ScalarType, OrdinalType>& aStatsInputs,
                               const Plato::MultiVector<ScalarType, OrdinalType>& aOptimalControls,
                               std::vector<UncertaintyOutputStruct<ScalarType> >& aOutput)
{
    for(OrdinalType tIndex = 0; tIndex < aStatsInputs.mNumSamples; tIndex++)
    {
        // undo scaling
        const ScalarType tUnscaledSampleValue = aOptimalControls(0, tIndex);
        aOutput[tIndex].mSampleValue = aStatsInputs.mLowerBound
                + (aStatsInputs.mUpperBound - aStatsInputs.mLowerBound) * tUnscaledSampleValue;
        // NOTE: The upper/lower bounds are not always defined. For example, normal dist'n what should be done?
        // confirmed accurate for beta dist'n alone
        aOutput[tIndex].mSampleWeight = aOptimalControls(1, tIndex);
    }
}

/******************************************************************************//**
 *
 * @brief Save diagnostics associated with the stochastic reduced order model (SROM) moment metrics
 * @param [in] aSromObjective objective function interface for SROM problem
 * @param [in,out] aOutput diagnostics associated with the SROM optimization problem
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void save_srom_moments_diagnostics(const Plato::SromObjective<ScalarType, OrdinalType>& aSromObjective,
                                          Plato::SromProblemDiagnosticsStruct<ScalarType>& aOutput)
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
 * @brief Save diagnostics associated with the SROM CDF
 * @param [in] aSromObjective objective function interface for SROM problem
 * @param [in,out] aOutput diagnostics associated with the SROM optimization problem
 *
 * NOTE: CDF = cumulative distribution function; 2) SROM = stochastic reduced order model
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void save_srom_cdf_diagnostics(const Plato::SromObjective<ScalarType, OrdinalType>& aSromObjective,
                                      Plato::SromProblemDiagnosticsStruct<ScalarType>& aOutput)
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

/******************************************************************************//**
 *
 * @brief Output diagnostics for stochastic reduced order model (SROM) optimization problem
 * @param [in] aCommWrapper distributed memory communicator wrapper
 * @param [in] aSromDiagnostics SROM problem diagnostics
 * @param [in] aAlgorithmDiagnostics optimizer diagnostics
 * @param [in] aPrint flag use to enable terminal output (default = false)
 *
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void output_srom_diagnostics(const Plato::CommWrapper & aCommWrapper,
                                    const Plato::SromProblemDiagnosticsStruct<ScalarType>& aSromDiagnostics,
                                    const Plato::AlgorithmParamStruct<ScalarType, OrdinalType>& aAlgorithmDiagnostics,
                                    bool aPrint = false)
{
    try
    {
        Plato::error::check_null_comm(aCommWrapper);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        std::ostringstream tMessage;
        tMessage << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\nFUNCTION: " << __PRETTY_FUNCTION__ << "\nLINE: "
                 << __LINE__ << "\n ******** \n\n";
        tMessage << tErrorMsg.what();
        if(aPrint == true)
        {
            std::cout << tMessage.str().c_str() << std::flush;
        }
        throw std::invalid_argument(tMessage.str().c_str());
    }

    if(aCommWrapper.myProcID() == 0)
    {
        std::ofstream tOutputFile;
        tOutputFile.open("plato_srom_diagnostics.txt");
        Plato::print_plato_license(tOutputFile);
        tOutputFile << "Cumulative Distribution Function (CDF) Mismatch = " << std::setprecision(6) << std::scientific
        << aSromDiagnostics.mCumulativeDistributionFunctionError << "\n\n";

        tOutputFile << "--------------------------------\n";
        tOutputFile << "| Statistical Moments Mismatch |\n";
        tOutputFile << "--------------------------------\n";
        tOutputFile << "| Name" << std::setw(12) << "Order" << std::setw(10) << "Error" << std::setw(5) << "|\n";
        tOutputFile << "--------------------------------\n";
        std::vector<std::string> tMomentNames = {"Mean    ", "Variance", "Skewness", "Kurtosis"};
        const OrdinalType tMaxNumMoments = aSromDiagnostics.mMomentErrors.size();
        //NOTE: ONLY THE ERRORS IN THE FIRST FOUR MOMENTS ARE OUTPUTTED TO FILE
        OrdinalType tNumMomentsToOutput = tMaxNumMoments > tMomentNames.size() ? tMomentNames.size() : tMaxNumMoments;
        for(OrdinalType tMomentIndex = 0; tMomentIndex < tNumMomentsToOutput; tMomentIndex++)
        {
            const OrdinalType tMomentOrder = tMomentIndex + static_cast<OrdinalType>(1);
            std::string tMyName = tMomentIndex < tMaxNumMoments ? tMomentNames[tMomentIndex] : std::to_string(tMomentOrder) + "";
            tOutputFile << std::setprecision(3) << std::scientific << "| " << std::setw(8) << tMyName.c_str() << std::setw(6)
            << tMomentOrder << std::setw(14) << aSromDiagnostics.mMomentErrors[tMomentIndex] << " |\n";
        }
        tOutputFile << "--------------------------------\n\n";
        tOutputFile << "Stochastic Reduced Order Model (SROM) optimizer diagnostics.\n";
        tOutputFile << "Objective Function Value = " << std::setprecision(6) << std::scientific
        << aAlgorithmDiagnostics.mObjectiveValue << "\n";
        tOutputFile << "Constraint Value = " << std::setprecision(6) << std::scientific << aAlgorithmDiagnostics.mConstraintValue
        << "\n";
        tOutputFile.close();
    }
}

} // namespace Plato
