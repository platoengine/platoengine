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
 * Plato_SromSolve.hpp
 *
 *  Created on: Apr 28, 2020
 */

#pragma once

#include "Plato_Macros.hpp"
#include "Plato_SromHelpers.hpp"
#include "Plato_SromMetadata.hpp"
#include "Plato_SromProbDataStruct.hpp"
#include "Plato_SolveUncertaintyProblem.hpp"
#include "Plato_KelleySachsAugmentedLagrangianLightInterface.hpp"

namespace Plato
{

namespace srom
{

/******************************************************************************//**
 * \fn check_input_number_samples
 * \brief Check random variable's number of samples
 * \param [in] aMyRandomVar random variable metadata
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool check_input_number_samples(const Plato::srom::RandomVariable & aMyRandomVar)
{
    if(aMyRandomVar.samples().empty() == true)
    {
        PRINTERR("Check Input Number Samples: NUMBER OF SAMPLES IS NOT DEFINED.\n");
        return (false);
    }
    else if(std::atof(aMyRandomVar.samples().c_str()) <= 0)
    {
        std::ostringstream tMsg;
        tMsg << "Check Input Number Samples: NUMBER OF SAMPLES SHOULD BE GREATER THAN ZERO. " << "INPUT NUMBER OF SAMPLES = "
             << std::atoi(aMyRandomVar.samples().c_str()) << ".\n";
        PRINTERR(tMsg.str().c_str());
        return (false);
    }

    return (true);
}
// function check_input_number_samples

/******************************************************************************//**
 * \fn check_input_standard_deviation
 * \brief Check random variable's standard deviation
 * \param [in] aMyRandomVar random variable metadata
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool check_input_standard_deviation(const Plato::srom::RandomVariable & aMyRandomVar)
{
    if(aMyRandomVar.deviation().empty() == true)
    {
        PRINTERR("Check Input Standard Deviation: STANDARD DEVIATION IS NOT DEFINED.\n");
        return (false);
    }
    return (true);
}
// function check_input_standard_deviation

/******************************************************************************//**
 * \fn check_input_mean
 * \brief Check random variable's mean
 * \param [in] aMyRandomVar random variable metadata
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool check_input_mean(const Plato::srom::RandomVariable & aMyRandomVar)
{
    if(aMyRandomVar.mean().empty() == true)
    {
        PRINTERR("Check Input Mean: MEAN IS NOT DEFINED.\n");
        return (false);
    }
    return (true);
}
// function check_input_mean

/******************************************************************************//**
 * \fn check_input_lower_bound
 * \brief Check random variable's lower bound
 * \param [in] aMyRandomVar random variable metadata
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool check_input_lower_bound(const Plato::srom::RandomVariable & aMyRandomVar)
{
    if(aMyRandomVar.lower().empty() == true)
    {
        PRINTERR("Check Input Lower Bound: LOWER BOUND IS NOT DEFINED.\n");
        return (false);
    }
    return (true);
}
// function check_input_lower_bound

/******************************************************************************//**
 * \fn check_input_upper_bound
 * \brief Check random variable's upper bound
 * \param [in] aMyRandomVar random variable metadata
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool check_input_upper_bound(const Plato::srom::RandomVariable & aMyRandomVar)
{
    if(aMyRandomVar.upper().empty() == true)
    {
        PRINTERR("Check Input Upper Bound: UPPER BOUND IS NOT DEFINED.\n");
        return (false);
    }
    return (true);
}
// function check_input_upper_bound

/******************************************************************************//**
 * \fn define_random_samples_initial_guess_method
 * \brief Define method used to compute random samples initial guess.
 * \param [in]  aMyRandomVar random variable metadata
 * \param [out] aInput       Stochastic Reduced Order Model (SROM) problem metadata
**********************************************************************************/
inline void define_random_samples_initial_guess_method
(const Plato::srom::RandomVariable & aMyRandomVar,
 Plato::SromInputs<double> & aInput)
{
    if(aMyRandomVar.guess() == "random")
    {
        std::srand(aInput.mRandomSeed);
        aInput.mInitialGuess = Plato::SromInitialGuess::random;
    }
    else if(aMyRandomVar.guess() == "uniform")
    {
        aInput.mInitialGuess = Plato::SromInitialGuess::uniform;
    }
    else
    {
        THROWERR(std::string("Define Random Samples Initial Guess Method: Initial Guess Method '")
            + aMyRandomVar.guess() + "' is not supported. Options are: 'random' or 'uniform'.");
    }
}
// function define_random_samples_initial_guess_method

/******************************************************************************//**
 * \fn define_distribution
 * \brief Define the distribution associated with the input random variable
 * \param [in] aMyRandomVar random variable metadata
 * \param [out] aInput Stochastic Reduced Order Model (SROM) problem metadata
**********************************************************************************/
inline void define_distribution
(const Plato::srom::RandomVariable & aMyRandomVar,
 Plato::SromInputs<double> & aInput)
{
    if(aMyRandomVar.distribution() == "normal")
    {
        aInput.mDistribution = Plato::DistributionName::normal;
    }
    else if(aMyRandomVar.distribution() == "uniform")
    {
        aInput.mDistribution = Plato::DistributionName::uniform;
    }
    else if(aMyRandomVar.distribution() == "beta")
    {
        aInput.mDistribution = Plato::DistributionName::beta;
    }
    else
    {
        std::ostringstream tMsg;
        tMsg << "Define Distribution: DISTRIBUTION = " << aMyRandomVar.distribution()
             << " IS NOT DEFINED. OPTIONS ARE NORMAL, UNIFORM AND BETA.\n";
        THROWERR(tMsg.str().c_str());
    }
}
// function define_distribution

/******************************************************************************//**
 * \fn check_input_statistics
 * \brief Check random variable statistics
 * \param [in] aMyRandomVar random variable metadata
**********************************************************************************/
inline void check_input_statistics(const Plato::srom::RandomVariable & aMyRandomVar)
{
    std::locale tLocale;
    std::stringstream tOutput;
    auto tDistribution = aMyRandomVar.distribution();
    for(std::string::size_type tIndex = 0; tIndex < tDistribution.length(); ++tIndex)
    {
        tOutput << std::toupper(tDistribution[tIndex], tLocale);
    }

    if(aMyRandomVar.distribution() == "beta" || aMyRandomVar.distribution() == "normal")
    {
        bool tIsMeanDefined = Plato::srom::check_input_mean(aMyRandomVar);
        bool tIsLowerBoundDefined = Plato::srom::check_input_lower_bound(aMyRandomVar);
        bool tIsUpperBoundDefined = Plato::srom::check_input_upper_bound(aMyRandomVar);
        bool tIsNumSamplesDefined = Plato::srom::check_input_number_samples(aMyRandomVar);
        bool tIsStandardDeviationDefined = Plato::srom::check_input_standard_deviation(aMyRandomVar);
        bool tStatisticsDefined = tIsMeanDefined && tIsLowerBoundDefined && tIsUpperBoundDefined && tIsNumSamplesDefined
                && tIsStandardDeviationDefined;
        if(tStatisticsDefined == false)
        {
            std::ostringstream tMsg;
            tMsg << "Check Input Statistics: Input statistics metadata for distribution '" << tOutput.str().c_str()
                 << "' are not completely defined.\n";
            THROWERR(tMsg.str().c_str());
        }
    }
    else if(aMyRandomVar.distribution() == "uniform")
    {
        bool tIsLowerBoundDefined = Plato::srom::check_input_lower_bound(aMyRandomVar);
        bool tIsUpperBoundDefined = Plato::srom::check_input_upper_bound(aMyRandomVar);
        bool tIsNumSamplesDefined = Plato::srom::check_input_number_samples(aMyRandomVar);
        bool tStatisticsDefined = tIsLowerBoundDefined && tIsUpperBoundDefined && tIsNumSamplesDefined;
        if(tStatisticsDefined == false)
        {
            std::ostringstream tMsg;
            tMsg << "Check Input Statistics: Input statistics metadata for distribution '" << tOutput.str().c_str()
                 << "' are not completely defined.\n";
            THROWERR(tMsg.str().c_str());
        }
    }
}
// function check_input_statistics

/******************************************************************************//**
 * \fn define_input_statistics
 * \brief Define random variable's statistics
 * \param [in] aMyRandomVar random variable metadata
 * \param [out] aInput Stochastic Reduced Order Model (SROM) problem metadata
**********************************************************************************/
inline void define_input_statistics
(const Plato::srom::RandomVariable & aMyRandomVar,
 Plato::SromInputs<double> & aInputSromMetaData)
{
    Plato::srom::check_input_statistics(aMyRandomVar);
    aInputSromMetaData.mMean = std::stod(aMyRandomVar.mean());
    aInputSromMetaData.mLowerBound = std::stod(aMyRandomVar.lower());
    aInputSromMetaData.mUpperBound = std::stod(aMyRandomVar.upper());
    const auto tStdDev = std::stod(aMyRandomVar.deviation());
    aInputSromMetaData.mVariance = tStdDev * tStdDev;
    aInputSromMetaData.mRandomSeed = std::stoi(aMyRandomVar.seed());
    aInputSromMetaData.mNumSamples = std::stoi(aMyRandomVar.samples());
    aInputSromMetaData.mDimensions = std::stoi(aMyRandomVar.dimensions());
    aInputSromMetaData.mCorrelationMatrixFilename = aMyRandomVar.correlationFilename();
}
// function define_input_statistics

/******************************************************************************//**
 * \fn compute_uniform_random_variable_statistics
 * \brief Given a uniform distribution, solve stochastic reduced order model \n
 *   (SROM) problem and compute sample probability pairs.
 * \param [in]   aInputMetaData  SROM problem input metadata
 * \param [out]  aOutputMetaData SROM problem output metadata
**********************************************************************************/
inline void compute_uniform_random_variable_statistics
(const Plato::SromInputs<double> & aInputMetaData,
 Plato::SromOutputs<double>& aOutputMetaData)
{
    const auto tDelta = (aInputMetaData.mUpperBound - aInputMetaData.mLowerBound)
        / static_cast<double>(aInputMetaData.mNumSamples - 1);

    for (decltype(aInputMetaData.mDimensions) tDimIndex = 0; tDimIndex < aInputMetaData.mDimensions; tDimIndex++)
    {
        aOutputMetaData.mSamples.push_back(std::vector<double>());
        for (decltype(aInputMetaData.mNumSamples) tSampleIndex = 0; tSampleIndex < aInputMetaData.mNumSamples; tSampleIndex++)
        {
            auto tSample = aInputMetaData.mLowerBound + (static_cast<double>(tSampleIndex) * tDelta);
            aOutputMetaData.mSamples.rbegin()->push_back(tSample);
        }
    }

    aOutputMetaData.mProbabilities.resize(aInputMetaData.mNumSamples);
    const auto tProbability = static_cast<double>(1.0 / aInputMetaData.mNumSamples);
    std::fill(aOutputMetaData.mProbabilities.begin(), aOutputMetaData.mProbabilities.end(), tProbability);
}
// function compute_uniform_random_variable_statistics

/******************************************************************************//**
 * \fn compute_sample_probability_pairs
 * \brief Compute random variable's statistics
 * \param  [in] aInputMetaData input metadata for the Stochastic Reduced Order Model (SROM) problem
 * \return SROM problem output metadata
**********************************************************************************/
inline Plato::SromOutputs<double>
compute_sample_probability_pairs
(const Plato::SromInputs<double> & aInputMetaData)
{
    Plato::SromOutputs<double> tOutputMetaData;
    switch(aInputMetaData.mDistribution)
    {
        case Plato::DistributionName::beta:
        case Plato::DistributionName::normal:
        {
            Plato::AlgorithmInputsKSAL<double> tAlgoInputs;
            Plato::SromDiagnostics<double> tSromDiagnostics;
            tSromDiagnostics.mOutputDiagnostics = true;
            Plato::solve_srom_problem(aInputMetaData, tAlgoInputs, tSromDiagnostics, tOutputMetaData);
            break;
        }
        case Plato::DistributionName::uniform:
        {
            Plato::srom::compute_uniform_random_variable_statistics(aInputMetaData, tOutputMetaData);
            break;
        }
        default:
        case Plato::DistributionName::undefined:
        {
            THROWERR("Compute Random Variable Statistics: INPUT DISTRIBUTION IS NOT SUPPORTED. OPTIONS ARE BETA, NORMAL AND UNIFORM.\n");
        }
    }
    return tOutputMetaData;
}
// function compute_sample_probability_pairs

/******************************************************************************//**
 * \fn post_process_sample_probability_pairs
 * \brief Post-process sample probability pairs computed by solving the SROM problem
 * \param [in] aMySromSolution sample-probability pairs for this random variable - \n
 *  computed by solving the Stochastic Reduced Order Model (SROM) problem
 * \param [in] aMyVariable input variable
 * \param [out] aMyRandomVariable random variable
**********************************************************************************/
inline void post_process_sample_probability_pairs
(const Plato::SromOutputs<double> aMySromSolution,
 const Plato::srom::RandomVariable & aMyVariable,
 Plato::srom::SromVariable & aMyRandomVariable)
{
    if(aMySromSolution.mSamples.empty())
    {
        THROWERR("SROM output samples metadata is empty.");
    }
    if(aMySromSolution.mProbabilities.empty())
    {
        THROWERR("SROM output probability metadata is empty.");
    }

    aMyRandomVariable.mSampleProbPairs.mSamples.clear();
    aMyRandomVariable.mSampleProbPairs.mProbabilities.clear();
    aMyRandomVariable.mSampleProbPairs.mNumSamples = aMySromSolution.mProbabilities.size();

    aMyRandomVariable.mTag = aMyVariable.tag();
    aMyRandomVariable.mAttribute = aMyVariable.attribute();
    for(auto& tProb : aMySromSolution.mProbabilities)
    {
        aMyRandomVariable.mSampleProbPairs.mProbabilities.push_back(tProb);
    }

    for(auto& tDim : aMySromSolution.mSamples)
    {
        for(auto& tSample : tDim)
        {
            aMyRandomVariable.mSampleProbPairs.mSamples.push_back(tSample);
        }
    }
}
// function post_process_sample_probability_pairs

/******************************************************************************//**
 * \fn compute_stochastic_reduced_order_model
 * \brief Compute sample-probability pairs by solving the Stochastic Reduced Order \n
 *    Model (SROM) problem.
 * \param [in] tRandomVar random variable metadata
 * \return SROM random variable metadata
**********************************************************************************/
inline Plato::srom::SromVariable
compute_stochastic_reduced_order_model
(const Plato::srom::RandomVariable& tRandomVar)
{
    Plato::SromInputs<double> tSromInputs;
    Plato::srom::define_distribution(tRandomVar, tSromInputs);
    Plato::srom::define_input_statistics(tRandomVar, tSromInputs);
    Plato::srom::define_random_samples_initial_guess_method(tRandomVar, tSromInputs);

    auto tSromOutputs = Plato::srom::compute_sample_probability_pairs(tSromInputs);

    Plato::srom::SromVariable tSampleProbPair;
    Plato::srom::post_process_sample_probability_pairs(tSromOutputs, tRandomVar, tSampleProbPair);

    return tSampleProbPair;
}
// function compute_stochastic_reduced_order_model

/******************************************************************************//**
 * \fn set_random_variables_id
 * \brief Set random variable identification numbers
 * \param [out] aSetRandomVariables set of random variables
**********************************************************************************/
inline void set_random_variables_id
(std::vector<Plato::srom::RandomVariable> & aRandomVariableSet)
{
    for(auto& tRandomVar : aRandomVariableSet)
    {
        auto tIndex = &tRandomVar - &aRandomVariableSet[0];
        tRandomVar.id(tIndex);
    }
}
// function set_random_variables_id

/******************************************************************************//**
 * \fn post_process_sample_probability_pairs
 * \brief Post-process sample-probability pairs read from an user-defined file \n
 *   into an SROM variable.
 * \param [in] aRandomVar       random variable metadata
 * \param [in] aSampleProbPairs sample-probability pairs read from a file
 * \return sample-probability pairs in Plato format
**********************************************************************************/
inline Plato::srom::SromVariable
post_process_sample_probability_pairs
(const Plato::srom::RandomVariable& aRandomVar,
 const std::vector<DataPairs>& aSampleProbPairs)
{
    if(aSampleProbPairs.empty())
    {
        THROWERR("Post Process Sample Probability Pairs: List of sample-probability pairs read from a file is empty.")
    }

    Plato::srom::SromVariable tOutput;
    tOutput.mTag = aRandomVar.tag();
    tOutput.mAttribute = aRandomVar.attribute();

    const auto tNumSamples = aSampleProbPairs[0].second.size();
    tOutput.mSampleProbPairs.mNumSamples = tNumSamples;
    tOutput.mSampleProbPairs.mSamples.resize(tNumSamples);
    tOutput.mSampleProbPairs.mProbabilities.resize(tNumSamples);

    for(auto& tSample : aSampleProbPairs[0].second)
    {
        auto tIndex = &tSample - &aSampleProbPairs[0].second[0];
        tOutput.mSampleProbPairs.mSamples[tIndex] = tSample;
        tOutput.mSampleProbPairs.mProbabilities[tIndex] = aSampleProbPairs[1].second[tIndex];
    }
    return tOutput;
}

/******************************************************************************//**
 * \fn compute_sample_probability_pairs
 * \brief Compute the sample-probability pairs for the set of random variables
 * \param [in] aSetRandomVariables set of input random variables
 * \param [out] aMySampleProbPairs set of sample-probability pairs
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool compute_sample_probability_pairs
(const std::vector<Plato::srom::RandomVariable> & aSetRandomVariables,
 std::vector<Plato::srom::SromVariable> & aMySampleProbPairs)
{
    if(aSetRandomVariables.size() <= 0)
    {
        PRINTERR("Compute Sample Probability Pairs: Input set of random variables is empty.\n");
        return (false);
    }

    aMySampleProbPairs.clear();
    for(auto& tRandomVar : aSetRandomVariables)
    {
        tRandomVar.check();
        try
        {
            if (tRandomVar.filename().empty())
            {
                // solve srom problem: sample-probability pairs must be computed by plato
                auto tSampleProbPairs = Plato::srom::compute_stochastic_reduced_order_model(tRandomVar);
                aMySampleProbPairs.push_back(tSampleProbPairs);
            }
            else
            {
                // read sample-probability pairs from the user-provided file
                auto tFilename = tRandomVar.filename();
                auto tSampleProbPairsFromFile = Plato::srom::read_sample_probability_pairs(tFilename);
                auto tSampleProbPairs = Plato::srom::post_process_sample_probability_pairs(tRandomVar, tSampleProbPairsFromFile);
                aMySampleProbPairs.push_back(tSampleProbPairs);
            }
        }
        catch(std::exception& tError)
        {
            std::ostringstream tMsg;
            tMsg << "Compute Sample Probability Pairs: Sample probability pairs were not computed for random variable with tag '"
                << tRandomVar.tag() << "', attribute '" << tRandomVar.attribute() << "', and identification number '" << tRandomVar.id()
                << "' due to the following error: "  << tError.what();
            THROWERR(tMsg.str().c_str());
        }
    }

    return (true);
}
// funciton compute_sample_probability_pairs

}
// namespace srom

}
// namespace Plato
