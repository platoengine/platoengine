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
    if(aMyRandomVar.mStatistics.mNumSamples.empty() == true)
    {
        PRINTERR("Check Input Number Samples: NUMBER OF SAMPLES IS NOT DEFINED.\n");
        return (false);
    }
    else if(std::atof(aMyRandomVar.mStatistics.mNumSamples.c_str()) <= 0)
    {
        std::ostringstream tMsg;
        tMsg << "Check Input Number Samples: NUMBER OF SAMPLES SHOULD BE GREATER THAN ZERO. " << "INPUT NUMBER OF SAMPLES = "
             << std::atoi(aMyRandomVar.mStatistics.mNumSamples.c_str()) << ".\n";
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
    if(aMyRandomVar.mStatistics.mStandardDeviation.empty() == true)
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
    if(aMyRandomVar.mStatistics.mMean.empty() == true)
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
    if(aMyRandomVar.mStatistics.mLowerBound.empty() == true)
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
    if(aMyRandomVar.mStatistics.mUpperBound.empty() == true)
    {
        PRINTERR("Check Input Upper Bound: UPPER BOUND IS NOT DEFINED.\n");
        return (false);
    }
    return (true);
}
// function check_input_upper_bound

/******************************************************************************//**
 * \fn define_distribution
 * \brief Define the distribution associated with the input random variable
 * \param [in] aMyRandomVar random variable metadata
 * \param [out] aInput Stochastic Reduced Order Model (SROM) problem metadata
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool define_distribution(const Plato::srom::RandomVariable & aMyRandomVar, Plato::SromInputs<double> & aInput)
{
    if(aMyRandomVar.mStatistics.mDistribution == "normal")
    {
        aInput.mDistribution = Plato::DistrubtionName::normal;
    }
    else if(aMyRandomVar.mStatistics.mDistribution == "uniform")
    {
        aInput.mDistribution = Plato::DistrubtionName::uniform;
    }
    else if(aMyRandomVar.mStatistics.mDistribution == "beta")
    {
        aInput.mDistribution = Plato::DistrubtionName::beta;
    }
    else
    {
        std::ostringstream tMsg;
        tMsg << "Define Distribution: DISTRIBUTION = " << aMyRandomVar.mStatistics.mDistribution
             << " IS NOT DEFINED. OPTIONS ARE NORMAL, UNIFORM AND BETA.\n";
        PRINTERR(tMsg.str().c_str());
        return (false);
    }

    return (true);
}
// function define_distribution

/******************************************************************************//**
 * \fn check_input_statistics
 * \brief Check random variable statistics
 * \param [in] aMyRandomVar random variable metadata
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool check_input_statistics(const Plato::srom::RandomVariable & aMyRandomVar)
{
    std::locale tLocale;
    std::stringstream tOutput;
    for(std::string::size_type tIndex = 0; tIndex < aMyRandomVar.mStatistics.mDistribution.length(); ++tIndex)
    {
        tOutput << std::toupper(aMyRandomVar.mStatistics.mDistribution[tIndex], tLocale);
    }

    if(aMyRandomVar.mStatistics.mDistribution == "beta" || aMyRandomVar.mStatistics.mDistribution == "normal")
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
            tMsg << "Check Input Statistics: FULL SET OF INPUT STATISTICS FOR THE " << tOutput.str().c_str()
                 << " DISTRIBUTION IS NOT PROPERLY DEFINED.\n";
            PRINTERR(tMsg.str().c_str());
            return (false);
        }
    }
    else if(aMyRandomVar.mStatistics.mDistribution == "uniform")
    {
        bool tIsLowerBoundDefined = Plato::srom::check_input_lower_bound(aMyRandomVar);
        bool tIsUpperBoundDefined = Plato::srom::check_input_upper_bound(aMyRandomVar);
        bool tIsNumSamplesDefined = Plato::srom::check_input_number_samples(aMyRandomVar);
        bool tStatisticsDefined = tIsLowerBoundDefined && tIsUpperBoundDefined && tIsNumSamplesDefined;
        if(tStatisticsDefined == false)
        {
            std::ostringstream tMsg;
            tMsg << "Check Input Statistics: SET OF INPUT STATISTICS FOR THE " << tOutput.str().c_str()
                 << " DISTRIBUTION IS NOT PROPERLY DEFINED.\n";
            PRINTERR(tMsg.str().c_str());
            return (false);
        }
    }

    return (true);
}
// function check_input_statistics

/******************************************************************************//**
 * \fn define_input_statistics
 * \brief Define random variable's statistics
 * \param [in] aMyRandomVar random variable metadata
 * \param [out] aInput Stochastic Reduced Order Model (SROM) problem metadata
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool define_input_statistics(const Plato::srom::RandomVariable & aMyRandomVar, Plato::SromInputs<double> & aInput)
{
    if(Plato::srom::check_input_statistics(aMyRandomVar) == false)
    {
        return (false);
    }

    aInput.mMean = std::atof(aMyRandomVar.mStatistics.mMean.c_str());
    aInput.mLowerBound = std::atof(aMyRandomVar.mStatistics.mLowerBound.c_str());
    aInput.mUpperBound = std::atof(aMyRandomVar.mStatistics.mUpperBound.c_str());
    const double tStdDev = std::atof(aMyRandomVar.mStatistics.mStandardDeviation.c_str());
    aInput.mVariance = tStdDev * tStdDev;
    const int tNumSamples = std::atoi(aMyRandomVar.mStatistics.mNumSamples.c_str());
    aInput.mNumSamples = tNumSamples;

    return (true);
}
// function define_input_statistics

/******************************************************************************//**
 * \fn compute_uniform_random_variable_statistics
 * \brief Compute sample probability pairs for an uniform random variable
 * \param [in] aInputMetaData input metadata for the Stochastic Reduced Order Model (SROM) problem
 * \param [out] aOutputMetaData output metadata for the SROM problem
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool compute_uniform_random_variable_statistics(const Plato::SromInputs<double> & aInputMetaData,
                                                       std::vector<Plato::SromOutputs<double>> & aOutputMetaData)
{
    aOutputMetaData.clear();

    const double tSampleProbability = static_cast<double>(1.0 / aInputMetaData.mNumSamples);
    const double tDelta = (aInputMetaData.mUpperBound - aInputMetaData.mLowerBound) / static_cast<double>(aInputMetaData.mNumSamples - 1);
    for(size_t tIndex = 0; tIndex < aInputMetaData.mNumSamples; tIndex++)
    {
        Plato::SromOutputs<double> tSromOutputs;
        tSromOutputs.mSampleWeight = tSampleProbability;
        tSromOutputs.mSampleValue = aInputMetaData.mLowerBound + (static_cast<double>(tIndex) * tDelta);
        aOutputMetaData.push_back(tSromOutputs);
    }
    return (true);
}
// function compute_uniform_random_variable_statistics

/******************************************************************************//**
 * \fn compute_random_variable_statistics
 * \brief Compute random variable's statistics
 * \param [in] aInputMetaData input metadata for the Stochastic Reduced Order Model (SROM) problem
 * \param [out] aOutputMetaData output metadata for the SROM problem
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool compute_random_variable_statistics(const Plato::SromInputs<double> & aInputMetaData,
                                               std::vector<Plato::SromOutputs<double>> & aOutputMetaData)
{
    switch(aInputMetaData.mDistribution)
    {
        case Plato::DistrubtionName::beta:
        case Plato::DistrubtionName::normal:
        {
            // solve stochastic reduced order model sub-problem
            const bool tEnableOutput = true;
            Plato::AlgorithmInputsKSAL<double> tAlgoInputs;
            Plato::SromDiagnostics<double> tSromDiagnostics;
            Plato::solve_srom_problem(aInputMetaData, tAlgoInputs, tSromDiagnostics, aOutputMetaData, tEnableOutput);
            break;
        }
        case Plato::DistrubtionName::uniform:
        {
            Plato::srom::compute_uniform_random_variable_statistics(aInputMetaData, aOutputMetaData);
            break;
        }
        default:
        case Plato::DistrubtionName::undefined:
        {
            PRINTERR("Compute Random Variable Statistics: INPUT DISTRIBUTION IS NOT SUPPORTED. OPTIONS ARE BETA, NORMAL AND UNIFORM.\n");
            return (false);
        }
    }

    return (true);
}
// function compute_random_variable_statistics

/******************************************************************************//**
 * \fn post_process_sample_probability_pairs
 * \brief Post-process sample probability pairs computed by solving the SROM problem
 * \param [in] aMySromSolution sample-probability pairs for this random variable - \n
 *  computed by solving the Stochastic Reduced Order Model (SROM) problem
 * \param [in] aMyVariable input variable
 * \param [out] aMyRandomVariable random variable
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool post_process_sample_probability_pairs(const std::vector<Plato::SromOutputs<double>> aMySromSolution,
                                                  const Plato::srom::RandomVariable & aMyVariable,
                                                  Plato::srom::SromVariable & aMyRandomVariable)
{
    if(aMySromSolution.size() <= 0)
    {
        PRINTERR("SROM SOLUTION IS EMPTY.\n");
        return (false);
    }

    aMyRandomVariable.mSampleProbPairs.mSamples.clear();
    aMyRandomVariable.mSampleProbPairs.mProbabilities.clear();

    aMyRandomVariable.mTag = aMyVariable.mTag;
    aMyRandomVariable.mAttribute = aMyVariable.mAttribute;

    const size_t tNumSamples = aMySromSolution.size();
    aMyRandomVariable.mSampleProbPairs.mNumSamples = tNumSamples;
    aMyRandomVariable.mSampleProbPairs.mSamples.resize(tNumSamples);
    aMyRandomVariable.mSampleProbPairs.mProbabilities.resize(tNumSamples);

    for(size_t tIndex = 0; tIndex < tNumSamples; tIndex++)
    {
        aMyRandomVariable.mSampleProbPairs.mSamples[tIndex] = aMySromSolution[tIndex].mSampleValue;
        aMyRandomVariable.mSampleProbPairs.mProbabilities[tIndex] = aMySromSolution[tIndex].mSampleWeight;
    }

    return (true);
}
// function post_process_sample_probability_pairs

/******************************************************************************//**
 * \fn compute_sample_probability_pairs
 * \brief Compute the sample-probability pairs for the set of random variables
 * \param [in] aSetRandomVariables set of input random variables
 * \param [out] aMySampleProbPairs set of sample-probability pairs
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool compute_sample_probability_pairs(const std::vector<Plato::srom::RandomVariable> & aSetRandomVariables,
                                             std::vector<Plato::srom::SromVariable> & aMySampleProbPairs)
{
    if(aSetRandomVariables.size() <= 0)
    {
        PRINTERR("INPUT SET OF RANDOM VARIABLES IS EMPTY.\n");
        return (false);
    }

    std::ostringstream tMsg;
    aMySampleProbPairs.clear();

    const size_t tNumRandomVariables = aSetRandomVariables.size();
    for(size_t tRandomVarIndex = 0; tRandomVarIndex < tNumRandomVariables; tRandomVarIndex++)
    {
        // pose uncertainty
        Plato::SromInputs<double> tSromInputs;
        const Plato::srom::RandomVariable & tMyRandomVar = aSetRandomVariables[tRandomVarIndex];
        if(Plato::srom::define_distribution(tMyRandomVar, tSromInputs) == false)
        {
            tMsg << "Compute Sample Probability Pairs: PROBABILITY DISTIRBUTION WAS NOT DEFINED FOR RANDOM VARIABLE #"
                << tRandomVarIndex << ".\n";
            PRINTERR(tMsg.str().c_str());
            return (false);
        }

        if(Plato::srom::define_input_statistics(tMyRandomVar, tSromInputs) == false)
        {
            tMsg << "Compute Sample Probability Pairs: SET OF INPUT STATISTICS FOR THE SROM PROBLEM IS NOT PROPERLY DEFINED FOR RANDOM VARIABLE #"
                 << tRandomVarIndex << ".\n";
            PRINTERR(tMsg.str().c_str());
            return (false);
        }

        std::vector<Plato::SromOutputs<double>> tSromOutputs;
        if(Plato::srom::compute_random_variable_statistics(tSromInputs, tSromOutputs) == false)
        {
            tMsg << "Compute Sample Probability Pairs: STATISTICS FOR RANDOM VARIABLE #"
                << tRandomVarIndex << " WERE NOT COMPUTED.\n";
            PRINTERR(tMsg.str().c_str());
            return (false);
        }

        Plato::srom::SromVariable tMySampleProbPairs;
        if(Plato::srom::post_process_sample_probability_pairs(tSromOutputs, tMyRandomVar, tMySampleProbPairs) == false)
        {
            tMsg << "Compute Sample Probability Pairs: SAMPLE PROBABILITY PAIR POST PROCESSING FAILED FOR RANDOM VARIABLE #"
                << tRandomVarIndex << ".\n";
            PRINTERR(tMsg.str().c_str());
            return (false);
        }
        aMySampleProbPairs.push_back(tMySampleProbPairs);
    }

    return (true);
}
// funciton compute_sample_probability_pairs

}
// namespace srom

}
// namespace Plato
