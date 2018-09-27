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
 * Plato_SolveUncertaintyProblem.hpp
 *
 * Created on: August 31, 2018
 */

#pragma once

#include "Plato_Vector.hpp"
#include "Plato_ErrorChecks.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_SromObjective.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_SromConstraint.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_StatisticsUtils.hpp"
#include "Plato_DistributionFactory.hpp"
#include "Plato_ReductionOperations.hpp"
#include "Plato_AugmentedLagrangianStageMng.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_KelleySachsAugmentedLagrangian.hpp"

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

namespace Plato
{

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

/******************************* INLINED FUNCTIONS *******************************/

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
 * @brief Check sample's container dimension.
 * @param [in] aDistribution probability distribution function interface
 * @param [in] aInputParamUQ statistics core input parameters
 * @param [in,out] aSamples monte carlo samples
 * @param [in,out] aCDF monte carlo cumulative distribution function values
 * @param [in] aPrint flag use to enable terminal output (default = false)
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void compute_monte_carlo_data(const Plato::Distribution<ScalarType, OrdinalType> & aDistribution,
                                     const Plato::UncertaintyInputStruct<ScalarType, OrdinalType> & aInputParamUQ,
                                     Plato::Vector<ScalarType, OrdinalType> & aSamples,
                                     Plato::Vector<ScalarType, OrdinalType> & aCDF,
                                     bool aPrint = false)
{
    const OrdinalType tNumMonteCarloSamples = aInputParamUQ.mNumSamples + static_cast<OrdinalType>(1);
    try
    {
        Plato::check_cdf_container_size(tNumMonteCarloSamples, aCDF);
        Plato::check_samples_container_size(tNumMonteCarloSamples, aSamples);
    }
    catch(const std::invalid_argument& tErrorMsg)
    {
        std::ostringstream tMessage;
        tMessage << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\nFUNCTION: " << __PRETTY_FUNCTION__
        << "\nLINE: " << __LINE__ << "\n ******** \n\n";
        tMessage << tErrorMsg.what();
        if(aPrint == true)
        {
            std::cout << tMessage.str().c_str() << std::flush;
        }
        throw std::invalid_argument(tMessage.str().c_str());
    }

    const ScalarType tDelta = static_cast<ScalarType>(1.0) / aInputParamUQ.mNumSamples;
    for(OrdinalType tIndex = 1; tIndex < tNumMonteCarloSamples; tIndex++)
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
        tMessage << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\nFUNCTION: " << __PRETTY_FUNCTION__
        << "\nLINE: " << __LINE__ << "\n ******** \n\n";
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
        tOutputFile << std::right << "    Samples" << std::setw(18) << "MC-CDF" << std::setw(18) << "SROM-CDF" << "\n" << std::flush;
        const OrdinalType tNumSamples = aSamples.size();
        for(OrdinalType tIndex = 0; tIndex < tNumSamples; tIndex++)
        {
            tOutputFile << std::scientific << std::setprecision(8) << std::right << aSamples[tIndex] << std::setw(18) << aMonteCarloCDF[tIndex]
            << std::setw(18) << aSromCDF[tIndex] << "\n" << std::flush;
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
        tMessage << "\n\n ********\n ERROR IN FILE: " << __FILE__ << "\nFUNCTION: " << __PRETTY_FUNCTION__ << "\nLINE: " << __LINE__
                 << "\n ******** \n\n";
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
void compute_srom_cdf_plot(const Plato::Vector<ScalarType, OrdinalType>& aSamplesMC,
                           const Plato::Vector<ScalarType, OrdinalType>& aSamplesSROM,
                           const Plato::Vector<ScalarType, OrdinalType>& aProbsSROM,
                           Plato::Vector<ScalarType, OrdinalType>& aSromCDF,
                           ScalarType aSigma = 1e-7)
{
    assert(aSromCDF.size() == aSamplesMC.size());
    assert(aSamplesSROM.size() == aProbsSROM.size());

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
void save_srom_solution(const Plato::UncertaintyInputStruct<ScalarType, OrdinalType>& aStatsInputs,
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
void save_srom_moments_diagnostics(const Plato::SromObjective<ScalarType, OrdinalType>& aSromObjective,
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
void save_srom_cdf_diagnostics(const Plato::SromObjective<ScalarType, OrdinalType>& aSromObjective,
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
 * @param [in] aSromDiagnostics SROM problem diagnostics
 * @param [in] aAlgorithmDiagnostics optimizer diagnostics
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
void output_srom_diagnostics(const Plato::SromProblemDiagnosticsStruct<ScalarType>& aSromDiagnostics,
                             const Plato::AlgorithmParamStruct<ScalarType, OrdinalType>& aAlgorithmDiagnostics)
{
    std::ofstream tOutputFile;
    tOutputFile.open("plato_srom_diagnostics.txt");
    Plato::print_plato_license(tOutputFile);
    tOutputFile << "Cumulative Distribution Function (CDF) Mismatch = "
            << std::setprecision(6) << std::scientific << aSromDiagnostics.mCumulativeDistributionFunctionError << "\n\n";

    tOutputFile << "--------------------------------\n";
    tOutputFile << "| Statistical Moments Mismatch |\n";
    tOutputFile << "--------------------------------\n";
    tOutputFile << "| Name" << std::setw(12) << "Order" << std::setw(10) << "Error" << std::setw(5) << "|\n";
    tOutputFile << "--------------------------------\n";
    std::vector<std::string> tMomentNames = {"Mean    ", "Variance", "Skewness", "Kurtosis"};
    const OrdinalType tMaxNumMoments = aSromDiagnostics.mMomentErrors.size();
    for(OrdinalType tMomentIndex = 0; tMomentIndex < tMaxNumMoments; tMomentIndex++)
    {
        const OrdinalType tMomentOrder = tMomentIndex + static_cast<OrdinalType>(1);
        std::string tMyName = tMomentIndex < tMaxNumMoments ? tMomentNames[tMomentIndex] : std::to_string(tMomentOrder) + "";
        tOutputFile << std::setprecision(3) << std::scientific << "| "<< std::setw(8) << tMyName.c_str() << std::setw(6) << tMomentOrder
                << std::setw(14) << aSromDiagnostics.mMomentErrors[tMomentIndex] << " |\n";
    }
    tOutputFile << "--------------------------------\n\n";
    tOutputFile << "Stochastic Reduced Order Model (SROM) optimizer diagnostics.\n";
    tOutputFile << "Objective Function Value = " << std::setprecision(6) << std::scientific
            << aAlgorithmDiagnostics.mObjectiveValue << "\n";
    tOutputFile << "Constraint Value = " << std::setprecision(6) << std::scientific
            << aAlgorithmDiagnostics.mConstraintValue << "\n";
    tOutputFile.close();
}

/******************************************************************************//**
 *
 * @brief Set optimizer parameters for stochastic reduced order model (SROM) problem
 * @param [in] aAlgorithmParam data structure with inputs/outputs for/from optimizer
 * @param [in,out] aAlgorithm optimizer interface
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
void set_srom_optimizer_parameters(const Plato::AlgorithmParamStruct<ScalarType, OrdinalType>& aAlgorithmParam,
                                   Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType>& aAlgorithm)
{
    aAlgorithm.setMaxNumIterations(aAlgorithmParam.mMaxNumIterations);
    aAlgorithm.setMinPenaltyParameter(aAlgorithmParam.mMinPenaltyParameter);
    aAlgorithm.setStagnationTolerance(aAlgorithmParam.mStagnationTolerance);
    aAlgorithm.setFeasibilityTolerance(aAlgorithmParam.mFeasibilityTolerance);
    aAlgorithm.setMaxTrustRegionRadius(aAlgorithmParam.mMaxTrustRegionRadius);
    aAlgorithm.setActualReductionTolerance(aAlgorithmParam.mActualReductionTolerance);
}

/******************************************************************************//**
 *
 * @brief Solve optimization problem to construct an optimal stochastic reduced order model (SROM)
 * @param [in] aStatisticInputs data structure with inputs for probability distribution function
 * @param [in,out] aAlgorithmParam data structure with inputs/outputs for/from optimization algorithm
 * @param [in,out] aSromDiagnostics diagnostics associated with the SROM optimization problem
 * @param [in,out] aOutput data structure with outputs from SROM optimization problem
 *
**********************************************************************************/
template<typename ScalarType, typename OrdinalType>
void solve_uncertainty(const Plato::UncertaintyInputStruct<ScalarType, OrdinalType>& aStatsInputs,
                       Plato::AlgorithmParamStruct<ScalarType, OrdinalType>& aAlgorithmParam,
                       Plato::SromProblemDiagnosticsStruct<ScalarType>& aSromDiagnostics,
                       std::vector<UncertaintyOutputStruct<ScalarType>>& aOutput)
{
    // grab values from input struct
    const OrdinalType tNumSamples = aStatsInputs.mNumSamples;
    const OrdinalType tMaxNumMoments = aStatsInputs.mMaxNumDistributionMoments;

    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tDataFactory =
            std::make_shared<Plato::DataFactory<ScalarType, OrdinalType>>();
    const OrdinalType tNumConstraints = 1;
    const OrdinalType tNumControlVectors = 2; // samples values and sample weights
    tDataFactory->allocateDual(tNumConstraints);
    tDataFactory->allocateControl(tNumSamples, tNumControlVectors);

    // build distribution
    std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> tDistribution =
            Plato::build_distrubtion<ScalarType, OrdinalType>(aStatsInputs);

    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT CRITERIA *********
    std::shared_ptr<Plato::SromObjective<ScalarType, OrdinalType> > tSromObjective =
            std::make_shared<Plato::SromObjective<ScalarType, OrdinalType> >(tDistribution, tMaxNumMoments, tNumSamples);
    tSromObjective->setMomentMisfitTermWeight(aStatsInputs.mMomentErrorCriterionWeight);
    tSromObjective->setCdfMisfitTermWeight(aStatsInputs.mCumulativeDistributionFuncErrorWeight);

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType> > tReductions = tDataFactory->getControlReductionOperations().create();
    std::shared_ptr<Plato::SromConstraint<ScalarType, OrdinalType> > tSromConstraint = std::make_shared<Plato::SromConstraint<ScalarType, OrdinalType> >(tReductions);
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType> > tConstraintList = std::make_shared<Plato::CriterionList<ScalarType, OrdinalType> >();
    tConstraintList->add(tSromConstraint);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType> > tStageMng =
            std::make_shared<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType> >(tDataFactory, tSromObjective, tConstraintList);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> > tDataMng =
            std::make_shared<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> >(tDataFactory);

    // ********* SET INTIAL GUESS FOR VECTOR OF SAMPLES *********
    Plato::StandardVector<ScalarType, OrdinalType> tInitialGuess(tNumSamples);
    Plato::make_uniform_sample<ScalarType,OrdinalType>(tInitialGuess);
    OrdinalType tVectorIndex = 0;
    tDataMng->setInitialGuess(tVectorIndex, tInitialGuess);
    // ********* SET INTIAL GUESS FOR VECTOR OF PROBABILITIES *********
    ScalarType tScalarValue = 1. / static_cast<ScalarType>(tNumSamples);
    tInitialGuess.fill(tScalarValue);
    tVectorIndex = 1;
    tDataMng->setInitialGuess(tVectorIndex, tInitialGuess);
    // ********* SET UPPER AND LOWER BOUNDS *********
    tScalarValue = 0;
    tDataMng->setControlLowerBounds(tScalarValue);
    tScalarValue = 1;
    tDataMng->setControlUpperBounds(tScalarValue);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM *********
    Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    Plato::set_srom_optimizer_parameters<ScalarType, OrdinalType>(aAlgorithmParam, tAlgorithm);
    tAlgorithm.solve();

    // transfer to output data structure
    const Plato::MultiVector<ScalarType, OrdinalType>& tFinalControl = tDataMng->getCurrentControl();
    aOutput.resize(tNumSamples);
    Plato::save_srom_solution<ScalarType,OrdinalType>(aStatsInputs, tFinalControl, aOutput);
    Plato::save_srom_cdf_diagnostics<ScalarType,OrdinalType>(*tSromObjective, aSromDiagnostics);
    Plato::save_srom_moments_diagnostics<ScalarType,OrdinalType>(*tSromObjective, aSromDiagnostics);

    // transfer stopping criterion, objective and constraint values to algorithm parameter data structure
    aAlgorithmParam.mStop = tAlgorithm.getStoppingCriterion();
    aAlgorithmParam.mObjectiveValue = tDataMng->getCurrentObjectiveFunctionValue();
    const OrdinalType tNumVectors = 1;
    Plato::StandardMultiVector<ScalarType, OrdinalType> tConstraintValues(tNumVectors, tNumConstraints);
    tStageMng->getCurrentConstraintValues(tConstraintValues);
    aAlgorithmParam.mConstraintValue = tConstraintValues(0,0);
}
// function solve_uncertainty

} // namespace Plato
