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
 * Plato_Test_UncertainLoadGeneratorXML.cpp
 *
 *  Created on: Jun 1, 2019
 */

#include "gtest/gtest.h"

#include <map>
#include <cmath>
#include <locale>
#include <cctype>
#include <vector>
#include <string>
#include <cstdlib>
#include <iostream>

#include "../../base/src/tools/Plato_UniqueCounter.hpp"
#include "../../base/src/tools/Plato_Vector3DVariations.hpp"
#include "../../base/src/optimize/Plato_SromProbDataStruct.hpp"
#include "../../base/src/optimize/Plato_SolveUncertaintyProblem.hpp"
#include "../../base/src/optimize/Plato_KelleySachsAugmentedLagrangianLightInterface.hpp"

namespace Plato
{

struct RandomVariable
{
    std::string mType; /*!< random variable type, e.g. random rotation */
    std::string mSubType; /*!< random variable subtype, e.g. rotation axis */
    std::string mGlobalID; /*!< random variable global identifier */
    std::string mDistribution; /*!< probability distribution */
    std::string mMean; /*!< probability distribution mean */
    std::string mUpperBound; /*!< probability distribution upper bound */
    std::string mLowerBound; /*!< probability distribution lower bound */
    std::string mStandardDeviation; /*!< probability distribution standard deviation */
    std::string mNumSamples; /*!< number of samples */
};

struct SampleProbabilityPairs
{
    int mNumSamples; /*!< total number of samples */
    std::vector<double> mSamples; /*!< sample set */
    std::vector<double> mProbabilities; /*!< probability set  */
};

struct SromRandomVariable
{
    std::string mType; /*!< random variable type, e.g. random rotation */
    std::string mSubType; /*!< random variable subtype, e.g. rotation axis */
    Plato::SampleProbabilityPairs mSampleProbPair; /*!< sample-probability pair for this random variable */
};

struct RandomRotations
{
    double mProbability; /*!< probability associated with this random rotation */
    Plato::Vector3D mRotationAngles; /*!< vector of random rotations, e.g. /f$(\theta_x, \theta_y, \theta_z)/f$ */
};

struct RandomLoad
{
    int mLoadID;           /*!< random load vector global identifier */
    double mProbability;   /*!< probability associated with this random load */
    Plato::Vector3D mLoad; /*!< vector of random loads, e.g. /f$(f_x, f_y, f_z)/f$ */
};

struct RandomLoadCase
{
    int mLoadCaseID;                       /*!< random load case global identifier */
    double mProbability;                   /*!< probability associated with this random load case */
    std::vector<Plato::RandomLoad> mLoads; /*!< set of random loads associated with this random load case */
};

inline bool apply_rotation_matrix(const Plato::Vector3D& aRotatioAnglesInDegrees, Plato::Vector3D& aVectorToRotate)
{
    // compute cosine/sine
    const double tCosAngleX = cos(aRotatioAnglesInDegrees.mX * M_PI / 180.0);
    const double tSinAngleX = sin(aRotatioAnglesInDegrees.mX * M_PI / 180.0);
    const double tCosAngleY = cos(aRotatioAnglesInDegrees.mY * M_PI / 180.0);
    const double tSinAngleY = sin(aRotatioAnglesInDegrees.mY * M_PI / 180.0);
    const double tCosAngleZ = cos(aRotatioAnglesInDegrees.mZ * M_PI / 180.0);
    const double tSinAngleZ = sin(aRotatioAnglesInDegrees.mZ * M_PI / 180.0);

    // compute all the components associated with the rotation matrix
    const double tA11 =  tCosAngleY * tCosAngleZ;
    const double tA12 = (-tCosAngleX * tSinAngleZ) + (tSinAngleX * tSinAngleY * tCosAngleZ);
    const double tA13 =  (tSinAngleX * tSinAngleZ) + (tCosAngleX * tSinAngleY * tCosAngleZ);

    const double tA21 =  tCosAngleY * tSinAngleZ;
    const double tA22 =  (tCosAngleX * tCosAngleZ) + (tSinAngleX * tSinAngleY * tSinAngleZ);
    const double tA23 = -(tSinAngleX * tCosAngleZ) + (tCosAngleX * tSinAngleY * tSinAngleZ);

    const double tA31 = -tSinAngleY;
    const double tA32 = tSinAngleX * tCosAngleY;
    const double tA33 = tCosAngleX * tCosAngleY;

    // apply rotation matrix to vector
    const double tMagnitudeX = aVectorToRotate.mX;
    const double tMagnitudeY = aVectorToRotate.mY;
    const double tMagnitudeZ = aVectorToRotate.mZ;
    aVectorToRotate.mX = (tMagnitudeX * tA11) + (tMagnitudeY * tA12) + (tMagnitudeZ * tA13);
    aVectorToRotate.mY = (tMagnitudeX * tA21) + (tMagnitudeY * tA22) + (tMagnitudeZ * tA23);
    aVectorToRotate.mZ = (tMagnitudeX * tA31) + (tMagnitudeY * tA32) + (tMagnitudeZ * tA33);

    return (true);
}

/*inline bool register_all_load_ids(const std::vector<LoadCase> & aLoadCases, Plato::UniqueCounter & aUniqueLoadCounter)
{
    if(aLoadCases.size() <= 0)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF LOAD CASES IS EMPTY.\n";
        return (false);
    }

    const int tNumLoadCases = aLoadCases.size();
    for(int tPrivateLoadIndex = 0; tPrivateLoadIndex < tNumLoadCases; tPrivateLoadIndex++)
    {
        // register load case
        const int tMyLoadCaseId = std::atoi(aLoadCases[tPrivateLoadIndex].id.c_str());
        aUniqueLoadCounter.mark(tMyLoadCaseId);

        const std::vector<Load> & tMyLoadCaseLoads = aLoadCases[tPrivateLoadIndex].loads;
        const int tMyLoadCaseNumLoads = tMyLoadCaseLoads.size();
        for(int tLoadIndex = 0; tLoadIndex < tMyLoadCaseNumLoads; tLoadIndex++)
        {
            // register load id
            const int tMyLoadID = std::atoi(tMyLoadCaseLoads[tLoadIndex].load_id.c_str());
            aUniqueLoadCounter.mark(tMyLoadID);
        }
    }

    return (true);
}*/

inline bool define_distribution(const Plato::RandomVariable & aMyRandomVar, Plato::SromInputs<double> & aInput)
{
    if(aMyRandomVar.mDistribution == "normal")
    {
        aInput.mDistribution = Plato::DistrubtionName::normal;
    }
    else if(aMyRandomVar.mDistribution == "uniform")
    {
        aInput.mDistribution = Plato::DistrubtionName::uniform;
    }
    else if(aMyRandomVar.mDistribution == "beta")
    {
        aInput.mDistribution = Plato::DistrubtionName::beta;
    }
    else
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: DISTRIBUTION = " << aMyRandomVar.mDistribution
                 << " IS NOT DEFINED. OPTIONS ARE NORMAL, UNIFORM AND BETA.\n";
        return (false);
    }

    return (true);
}

inline bool check_input_mean(const Plato::RandomVariable & aMyRandomVar)
{
    if(aMyRandomVar.mMean.empty() == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: MEAN IS NOT DEFINED.\n";
        return (false);
    }
    return (true);
}

inline bool check_input_lower_bound(const Plato::RandomVariable & aMyRandomVar)
{
    if(aMyRandomVar.mLowerBound.empty() == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: LOWER BOUND IS NOT DEFINED.\n";
        return (false);
    }
    return (true);
}

inline bool check_input_upper_bound(const Plato::RandomVariable & aMyRandomVar)
{
    if(aMyRandomVar.mUpperBound.empty() == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: UPPER BOUND IS NOT DEFINED.\n";
        return (false);
    }
    return (true);
}

inline bool check_input_standard_deviation(const Plato::RandomVariable & aMyRandomVar)
{
    if(aMyRandomVar.mStandardDeviation.empty() == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: STANDARD DEVIATION IS NOT DEFINED.\n";
        return (false);
    }
    return (true);
}

inline bool check_input_number_samples(const Plato::RandomVariable & aMyRandomVar)
{
    if(aMyRandomVar.mNumSamples.empty() == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: NUMBER OF SAMPLES IS NOT DEFINED.\n";
        return (false);
    }
    else if(std::atof(aMyRandomVar.mNumSamples.c_str()) <= 0)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: NUMBER OF SAMPLES SHOULD BE GREATER THAN ZERO. "
                 << "INPUT NUMBER OF SAMPLES = " << std::atof(aMyRandomVar.mNumSamples.c_str()) << ".\n";
        return (false);
    }

    return (true);
}

inline bool check_input_statistics(const Plato::RandomVariable & aMyRandomVar)
{
    std::locale tLocale;
    std::stringstream tOutput;
    for(std::string::size_type tIndex = 0; tIndex < aMyRandomVar.mDistribution.length(); ++tIndex)
    {
        tOutput << std::toupper(aMyRandomVar.mDistribution[tIndex], tLocale);
    }

    if(aMyRandomVar.mDistribution == "beta" || aMyRandomVar.mDistribution == "normal")
    {
        bool tIsMeanDefined = Plato::check_input_mean(aMyRandomVar);
        bool tIsLowerBoundDefined = Plato::check_input_lower_bound(aMyRandomVar);
        bool tIsUpperBoundDefined = Plato::check_input_upper_bound(aMyRandomVar);
        bool tIsNumSamplesDefined = Plato::check_input_number_samples(aMyRandomVar);
        bool tIsStandardDeviationDefined = Plato::check_input_standard_deviation(aMyRandomVar);
        bool tStatisticsDefined = tIsMeanDefined && tIsLowerBoundDefined && tIsUpperBoundDefined && tIsNumSamplesDefined
                && tIsStandardDeviationDefined;
        if(tStatisticsDefined == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: FULL SET OF INPUT STATISTICS FOR THE " << tOutput.str().c_str()
                     << " DISTRIBUTION IS NOT PROPERLY DEFINED.\n";
            return (false);
        }
    }
    else if(aMyRandomVar.mDistribution == "uniform")
    {
        bool tIsLowerBoundDefined = Plato::check_input_lower_bound(aMyRandomVar);
        bool tIsUpperBoundDefined = Plato::check_input_upper_bound(aMyRandomVar);
        bool tIsNumSamplesDefined = Plato::check_input_number_samples(aMyRandomVar);
        bool tStatisticsDefined = tIsLowerBoundDefined && tIsUpperBoundDefined && tIsNumSamplesDefined;
        if(tStatisticsDefined == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: SET OF INPUT STATISTICS FOR THE " << tOutput.str().c_str()
                     << " DISTRIBUTION IS NOT PROPERLY DEFINED.\n";
            return (false);
        }
    }

    return (true);
}

inline bool define_input_statistics(const Plato::RandomVariable & aMyRandomVar, Plato::SromInputs<double> & aInput)
{
    if(Plato::check_input_statistics(aMyRandomVar) == false)
    {
        return (false);
    }

    aInput.mMean = std::atof(aMyRandomVar.mMean.c_str());
    aInput.mLowerBound = std::atof(aMyRandomVar.mLowerBound.c_str());
    aInput.mUpperBound = std::atof(aMyRandomVar.mUpperBound.c_str());
    const double tStdDev = std::atof(aMyRandomVar.mStandardDeviation.c_str());
    aInput.mVariance = tStdDev * tStdDev;
    const int tNumSamples = std::atoi(aMyRandomVar.mNumSamples.c_str());
    aInput.mNumSamples = tNumSamples;

    return (true);
}

inline bool post_process_sample_probability_pairs(const std::vector<Plato::SromOutputs<double>> aMySromSolution,
                                                  const Plato::RandomVariable & aMyRandomVariable,
                                                  Plato::SromRandomVariable & aMySromRandomVariable)
{
    if(aMySromSolution.size() <= 0)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: SROM SOLUTION IS EMPTY.\n";
        return (false);
    }

    aMySromRandomVariable.mSampleProbPair.mSamples.clear();
    aMySromRandomVariable.mSampleProbPair.mProbabilities.clear();

    aMySromRandomVariable.mType = aMyRandomVariable.mType;
    aMySromRandomVariable.mSubType = aMyRandomVariable.mSubType;

    const size_t tNumSamples = aMySromSolution.size();
    aMySromRandomVariable.mSampleProbPair.mNumSamples = tNumSamples;
    aMySromRandomVariable.mSampleProbPair.mSamples.resize(tNumSamples);
    aMySromRandomVariable.mSampleProbPair.mProbabilities.resize(tNumSamples);

    for(size_t tIndex = 0; tIndex < tNumSamples; tIndex++)
    {
        aMySromRandomVariable.mSampleProbPair.mSamples[tIndex] = aMySromSolution[tIndex].mSampleValue;
        aMySromRandomVariable.mSampleProbPair.mProbabilities[tIndex] = aMySromSolution[tIndex].mSampleWeight;
    }

    return (true);
}

inline bool compute_uniform_random_variable_statistics(const Plato::SromInputs<double> & aSromInputs,
                                                       std::vector<Plato::SromOutputs<double>> & aSromOutputSet)
{
    aSromOutputSet.clear();

    const double tSampleProbability = static_cast<double>(1.0 / aSromInputs.mNumSamples);
    const double tDelta = (aSromInputs.mUpperBound - aSromInputs.mLowerBound) / static_cast<double>(aSromInputs.mNumSamples - 1);
    for(size_t tIndex = 0; tIndex < aSromInputs.mNumSamples; tIndex++)
    {
        Plato::SromOutputs<double> tSromOutputs;
        tSromOutputs.mSampleWeight = tSampleProbability;
        tSromOutputs.mSampleValue = aSromInputs.mLowerBound + (static_cast<double>(tIndex) * tDelta);
        aSromOutputSet.push_back(tSromOutputs);
    }
    return (true);
}

inline bool compute_random_variable_statistics(const Plato::SromInputs<double> & aSromInputs,
                                               std::vector<Plato::SromOutputs<double>> & aSromOutputs)
{
    switch(aSromInputs.mDistribution)
    {
        case Plato::DistrubtionName::beta:
        case Plato::DistrubtionName::normal:
        {
            // solve stochastic reduced order model sub-problem
            const bool tEnableOutput = true;
            Plato::AlgorithmInputsKSAL<double> tAlgoInputs;
            Plato::SromDiagnostics<double> tSromDiagnostics;
            Plato::solve_srom_problem(aSromInputs, tAlgoInputs, tSromDiagnostics, aSromOutputs, tEnableOutput);
            break;
        }
        case Plato::DistrubtionName::uniform:
        {
            Plato::compute_uniform_random_variable_statistics(aSromInputs, aSromOutputs);
            break;
        }
        default:
        case Plato::DistrubtionName::undefined:
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: INPUT DISTRIBUTION IS NOT SUPPORTED. OPTIONS ARE BETA, NORMAL AND UNIFORM.\n";
            return (false);
        }
    }

    return (true);
}

inline bool compute_sample_probability_pairs(const std::vector<Plato::RandomVariable> & aMySetRandomVars,
                                             std::vector<Plato::SromRandomVariable> & aMySampleProbPairs)
{
    if(aMySetRandomVars.size() <= 0)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF RANDOM VARIABLES IS EMPTY.\n";
        return (false);
    }

    aMySampleProbPairs.clear();

    const size_t tMyNumRandomVars = aMySetRandomVars.size();
    for(size_t tRandomVarIndex = 0; tRandomVarIndex < tMyNumRandomVars; tRandomVarIndex++)
    {
        // pose uncertainty
        Plato::SromInputs<double> tSromInputs;
        const Plato::RandomVariable & tMyRandomVar = aMySetRandomVars[tRandomVarIndex];
        if(Plato::define_distribution(tMyRandomVar, tSromInputs) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: PROBABILITY DISTIRBUTION WAS NOT DEFINED FOR RANDOM VARIABLE #"
                     << tRandomVarIndex << ".\n";
            return (false);
        }

        if(Plato::define_input_statistics(tMyRandomVar, tSromInputs) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: SET OF INPUT STATISTICS FOR THE SROM PROBLEM IS NOT PROPERLY DEFINED FOR RANDOM VARIABLE #"
                     << tRandomVarIndex << ".\n";
            return (false);
        }

        std::vector<Plato::SromOutputs<double>> tSromOutputs;
        if(Plato::compute_random_variable_statistics(tSromInputs, tSromOutputs) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: STATISTICS FOR RANDOM VARIABLE #" << tRandomVarIndex << " WERE NOT COMPUTED.\n";
            return (false);
        }

        Plato::SromRandomVariable tMySampleProbPairs;
        if(Plato::post_process_sample_probability_pairs(tSromOutputs, tMyRandomVar, tMySampleProbPairs) == false)
        {
            std::cout<< "\nFILE: " <<  __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: SAMPLE PROBABILITY PAIR POST PROCESSING FAILED FOR RANDOM VARIABLE #"
                     << tRandomVarIndex << ".\n";
            return (false);
        }
        aMySampleProbPairs.push_back(tMySampleProbPairs);
    }

    return (true);
}

inline bool post_process_random_load(const std::vector<Plato::SromRandomVariable> & aMySampleProbPairs,
                                     std::map<Plato::axis3D::axis3D, Plato::SampleProbabilityPairs> & aRotationAxisToSampleProbPairs)
{
    if(aMySampleProbPairs.size() <= 0)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF RANDOM VARIABLES IS EMPTY.\n";
        return (false);
    }

    const size_t tNumRandomVariables = aMySampleProbPairs.size();
    for(size_t tRandVarIndex = 0; tRandVarIndex < tNumRandomVariables; tRandVarIndex++)
    {
        Plato::axis3D::axis3D tMyAxis = Plato::axis3D::axis3D::x;
        Plato::axis3D_stringToEnum(aMySampleProbPairs[tRandVarIndex].mSubType, tMyAxis);
        aRotationAxisToSampleProbPairs[tMyAxis] = aMySampleProbPairs[tRandVarIndex].mSampleProbPair;
    }

    return (true);
}

inline bool set_random_rotations_about_xyz(const Plato::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                           const Plato::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                           const Plato::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                           std::vector<Plato::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexI = 0; tIndexI < aMyXaxisSampleProbPairs.mSamples.size(); tIndexI++)
    {
        for(size_t tIndexJ = 0; tIndexJ < aMyYaxisSampleProbPairs.mSamples.size(); tIndexJ++)
        {
            for(size_t tIndexK = 0; tIndexK < aMyZaxisSampleProbPairs.mSamples.size(); tIndexK++)
            {
                Plato::RandomRotations tMyRandomRotations;
                tMyRandomRotations.mProbability = aMyXaxisSampleProbPairs.mProbabilities[tIndexI]
                        * aMyYaxisSampleProbPairs.mProbabilities[tIndexJ] * aMyZaxisSampleProbPairs.mProbabilities[tIndexK];
                tMyRandomRotations.mRotationAngles.mX = aMyXaxisSampleProbPairs.mSamples[tIndexI];
                tMyRandomRotations.mRotationAngles.mY = aMyYaxisSampleProbPairs.mSamples[tIndexJ];
                tMyRandomRotations.mRotationAngles.mZ = aMyZaxisSampleProbPairs.mSamples[tIndexK];
                aMyRandomRotations.push_back(tMyRandomRotations);
            }
        }
    }

    return (true);
}

inline bool set_random_rotations_about_xy(const Plato::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                          const Plato::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                          std::vector<Plato::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexI = 0; tIndexI < aMyXaxisSampleProbPairs.mSamples.size(); tIndexI++)
    {
        for(size_t tIndexJ = 0; tIndexJ < aMyYaxisSampleProbPairs.mSamples.size(); tIndexJ++)
        {
            Plato::RandomRotations tMyRandomRotations;
            tMyRandomRotations.mProbability = aMyXaxisSampleProbPairs.mProbabilities[tIndexI]
                    * aMyYaxisSampleProbPairs.mProbabilities[tIndexJ];
            tMyRandomRotations.mRotationAngles.mX = aMyXaxisSampleProbPairs.mSamples[tIndexI];
            tMyRandomRotations.mRotationAngles.mY = aMyYaxisSampleProbPairs.mSamples[tIndexJ];
            aMyRandomRotations.push_back(tMyRandomRotations);
        }
    }

    return (true);
}

inline bool set_random_rotations_about_xz(const Plato::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                          const Plato::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                          std::vector<Plato::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexI = 0; tIndexI < aMyXaxisSampleProbPairs.mSamples.size(); tIndexI++)
    {
        for(size_t tIndexK = 0; tIndexK < aMyZaxisSampleProbPairs.mSamples.size(); tIndexK++)
        {
            Plato::RandomRotations tMyRandomRotations;
            tMyRandomRotations.mProbability = aMyXaxisSampleProbPairs.mProbabilities[tIndexI]
                    * aMyZaxisSampleProbPairs.mProbabilities[tIndexK];
            tMyRandomRotations.mRotationAngles.mX = aMyXaxisSampleProbPairs.mSamples[tIndexI];
            tMyRandomRotations.mRotationAngles.mZ = aMyZaxisSampleProbPairs.mSamples[tIndexK];
            aMyRandomRotations.push_back(tMyRandomRotations);
        }
    }

    return (true);
}

inline bool set_random_rotations_about_yz(const Plato::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                          const Plato::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                          std::vector<Plato::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexJ = 0; tIndexJ < aMyYaxisSampleProbPairs.mSamples.size(); tIndexJ++)
    {
        for(size_t tIndexK = 0; tIndexK < aMyZaxisSampleProbPairs.mSamples.size(); tIndexK++)
        {
            Plato::RandomRotations tMyRandomRotations;
            tMyRandomRotations.mProbability = aMyYaxisSampleProbPairs.mProbabilities[tIndexJ]
                    * aMyZaxisSampleProbPairs.mProbabilities[tIndexK];
            tMyRandomRotations.mRotationAngles.mY = aMyYaxisSampleProbPairs.mSamples[tIndexJ];
            tMyRandomRotations.mRotationAngles.mZ = aMyZaxisSampleProbPairs.mSamples[tIndexK];
            aMyRandomRotations.push_back(tMyRandomRotations);
        }
    }

    return (true);
}

inline bool set_random_rotations_about_x(const Plato::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                         std::vector<Plato::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexI = 0; tIndexI < aMyXaxisSampleProbPairs.mSamples.size(); tIndexI++)
    {
        Plato::RandomRotations tMyRandomRotations;
        tMyRandomRotations.mProbability = aMyXaxisSampleProbPairs.mProbabilities[tIndexI];
        tMyRandomRotations.mRotationAngles.mX = aMyXaxisSampleProbPairs.mSamples[tIndexI];
        aMyRandomRotations.push_back(tMyRandomRotations);
    }

    return (true);
}

inline bool set_random_rotations_about_y(const Plato::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                         std::vector<Plato::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexJ = 0; tIndexJ < aMyYaxisSampleProbPairs.mSamples.size(); tIndexJ++)
    {
        Plato::RandomRotations tMyRandomRotations;
        tMyRandomRotations.mProbability = aMyYaxisSampleProbPairs.mProbabilities[tIndexJ];
        tMyRandomRotations.mRotationAngles.mY = aMyYaxisSampleProbPairs.mSamples[tIndexJ];
        aMyRandomRotations.push_back(tMyRandomRotations);
    }

    return (true);
}

inline bool set_random_rotations_about_z(const Plato::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                         std::vector<Plato::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexK = 0; tIndexK < aMyZaxisSampleProbPairs.mSamples.size(); tIndexK++)
    {
        Plato::RandomRotations tMyRandomRotations;
        tMyRandomRotations.mProbability = aMyZaxisSampleProbPairs.mProbabilities[tIndexK];
        tMyRandomRotations.mRotationAngles.mZ = aMyZaxisSampleProbPairs.mSamples[tIndexK];
        aMyRandomRotations.push_back(tMyRandomRotations);
    }

    return (true);
}

inline bool expand_random_rotations(const Plato::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                    const Plato::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                    const Plato::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                    std::vector<Plato::RandomRotations> & aMyRandomRotations)
{
    aMyRandomRotations.clear();

    const bool tRotateAboutX = aMyXaxisSampleProbPairs.mSamples.empty();
    const bool tRotateAboutY = aMyYaxisSampleProbPairs.mSamples.empty();
    const bool tRotateAboutZ = aMyZaxisSampleProbPairs.mSamples.empty();

    if((tRotateAboutX && tRotateAboutY && tRotateAboutZ) == true)
    {
        Plato::set_random_rotations_about_xyz(aMyXaxisSampleProbPairs, aMyYaxisSampleProbPairs, aMyZaxisSampleProbPairs, aMyRandomRotations);
    }
    else if((tRotateAboutX && tRotateAboutY && !tRotateAboutZ) == true)
    {
        Plato::set_random_rotations_about_xy(aMyXaxisSampleProbPairs, aMyYaxisSampleProbPairs, aMyRandomRotations);
    }
    else if((tRotateAboutX && !tRotateAboutY && tRotateAboutZ) == true)
    {
        Plato::set_random_rotations_about_xz(aMyXaxisSampleProbPairs, aMyZaxisSampleProbPairs, aMyRandomRotations);
    }
    else if((!tRotateAboutX && tRotateAboutY && tRotateAboutZ) == true)
    {
        Plato::set_random_rotations_about_yz(aMyYaxisSampleProbPairs, aMyZaxisSampleProbPairs, aMyRandomRotations);
    }
    else if((tRotateAboutX && !tRotateAboutY && !tRotateAboutZ) == true)
    {
        Plato::set_random_rotations_about_x(aMyXaxisSampleProbPairs, aMyRandomRotations);
    }
    else if((!tRotateAboutX && tRotateAboutY && !tRotateAboutZ) == true)
    {
        Plato::set_random_rotations_about_y(aMyYaxisSampleProbPairs, aMyRandomRotations);
    }
    else if((!tRotateAboutX && !tRotateAboutY && tRotateAboutZ) == true)
    {
        Plato::set_random_rotations_about_z(aMyZaxisSampleProbPairs, aMyRandomRotations);
    }
    else
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF SAMPLE PROBABILITY PAIRS ARE EMPTY. LOAD IS DETERMINISTIC.\n";
        return (false);
    }

    return (true);
}

inline bool expand_random_loads(const Plato::Vector3D & aOriginalLoad,
                                const std::vector<Plato::RandomRotations> & aMyRandomRotations,
                                Plato::UniqueCounter & aUniqueLoadCounter,
                                std::vector<Plato::RandomLoad> & aMyRandomLoads)
{
    if(aMyRandomRotations.empty())
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT VECTOR OF RANDOM ROTATIONS IS EMPTY.\n";
        return (false);
    }

    const size_t tNumRandomLoads = aMyRandomRotations.size();
    for(size_t tIndex = 0; tIndex < tNumRandomLoads; tIndex++)
    {
        Plato::RandomLoad tMyRandomLoad;
        tMyRandomLoad.mLoad = aOriginalLoad;
        tMyRandomLoad.mProbability = aMyRandomRotations[tIndex].mProbability;
        if(Plato::apply_rotation_matrix(aMyRandomRotations[tIndex].mRotationAngles, tMyRandomLoad.mLoad) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: APPLICATION OF ROTATION MATRIS WAS UNSUCCESSFUL.\n";
            return (false);
        }
        tMyRandomLoad.mLoadID = aUniqueLoadCounter.assign_next_unique();
        aMyRandomLoads.push_back(tMyRandomLoad);
    }

    return (true);
}

inline bool update_initial_random_load_case(const std::vector<Plato::RandomLoad> & aNewRandomLoads,
                                            std::vector<Plato::RandomLoadCase> & aOldRandomLoadCases,
                                            Plato::UniqueCounter & aUniqueLoadCounter)
{
    std::vector<Plato::RandomLoadCase> tNewSetRandomLoadCase;

    for(size_t tLoadIndex = 0; tLoadIndex < aNewRandomLoads.size(); tLoadIndex++)
    {
        Plato::RandomLoadCase tNewRandomLoadCase;
        const Plato::RandomLoad& tMyNewRandomLoad = aNewRandomLoads[tLoadIndex];
        tNewRandomLoadCase.mLoads.push_back(tMyNewRandomLoad);
        tNewRandomLoadCase.mProbability = tMyNewRandomLoad.mProbability;
        tNewRandomLoadCase.mLoadCaseID = aUniqueLoadCounter.assign_next_unique();
        tNewSetRandomLoadCase.push_back(tNewRandomLoadCase);
    } // index over new random loads

    aOldRandomLoadCases = tNewSetRandomLoadCase;

    return (true);
}

inline bool update_random_load_cases(const std::vector<Plato::RandomLoad> & aNewRandomLoads,
                                     std::vector<Plato::RandomLoadCase> & aOldRandomLoadCases,
                                     Plato::UniqueCounter & aUniqueLoadCounter)
{
    std::vector<Plato::RandomLoadCase> tNewSetRandomLoadCase;

    for(size_t tNewLoadIndex = 0; tNewLoadIndex < aNewRandomLoads.size(); tNewLoadIndex++)
    {
        const Plato::RandomLoad& tMyNewRandomLoad = aNewRandomLoads[tNewLoadIndex];
        std::vector<Plato::RandomLoadCase> tTempRandomLoadCases = aOldRandomLoadCases;
        for(size_t tOldLoadCaseIndex = 0; tOldLoadCaseIndex < aOldRandomLoadCases.size(); tOldLoadCaseIndex++)
        {
            tTempRandomLoadCases[tOldLoadCaseIndex].mLoads.push_back(tMyNewRandomLoad);
            Plato::RandomLoadCase& tMyTempRandomLoadCase = tTempRandomLoadCases[tOldLoadCaseIndex];
            tMyTempRandomLoadCase.mLoadCaseID = aUniqueLoadCounter.assign_next_unique();
            tMyTempRandomLoadCase.mProbability = tMyTempRandomLoadCase.mProbability * tMyNewRandomLoad.mProbability;
            tNewSetRandomLoadCase.push_back(tMyTempRandomLoadCase);
        } // index over old random load cases
    } // index over new random loads

    aOldRandomLoadCases = tNewSetRandomLoadCase;

    return (true);
}

inline bool expand_random_load_cases(const std::vector<Plato::RandomLoad> & aNewRandomLoads,
                                     std::vector<Plato::RandomLoadCase> & aOldRandomLoadCases,
                                     Plato::UniqueCounter & aUniqueLoadCounter)
{
    if(aNewRandomLoads.empty())
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: NEW VECTOR OF RANDOM LOADS IS EMPTY.\n";
        return (false);
    } // if-else statement
    else if(!aNewRandomLoads.empty() && aOldRandomLoadCases.empty())
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: NEW VECTOR OF RANDOM LOADS IS NOT EMPTY. "
                 << "HOWEVER, THE PREVIOUS SET OF LOAD CASES IS EMPTY. "
                 << "THIS USE CASE IS NOT VALID.\n";
        return (false);
    } // if-else statement

    if(aOldRandomLoadCases.empty())
    {
        if(Plato::update_initial_random_load_case(aNewRandomLoads, aOldRandomLoadCases, aUniqueLoadCounter) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: INITIAL RANDOM LOAD CASE WAS NOT UPDATED.\n";
            return (false);
        } // if statement
    } // if-else statement
    else
    {
        if(Plato::update_random_load_cases(aNewRandomLoads, aOldRandomLoadCases, aUniqueLoadCounter) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: NEW SET OF RANDOM LOAD CASES WAS NOT UPDATED.\n";
            return (false);
        } // if statement
    } // if-else statement

    return (true);
}

}

namespace PlatoUncertainLoadGeneratorXMLTest
{

TEST(PlatoTest, check_input_mean)
{
    Plato::RandomVariable tMyRandomVar;
    ASSERT_FALSE(Plato::check_input_mean(tMyRandomVar));

    tMyRandomVar.mMean = "5";
    ASSERT_TRUE(Plato::check_input_mean(tMyRandomVar));
}

TEST(PlatoTest, check_input_lower_bound)
{
    Plato::RandomVariable tMyRandomVar;
    ASSERT_FALSE(Plato::check_input_lower_bound(tMyRandomVar));

    tMyRandomVar.mLowerBound = "5";
    ASSERT_TRUE(Plato::check_input_lower_bound(tMyRandomVar));
}

TEST(PlatoTest, check_input_upper_bound)
{
    Plato::RandomVariable tMyRandomVar;
    ASSERT_FALSE(Plato::check_input_upper_bound(tMyRandomVar));

    tMyRandomVar.mUpperBound = "5";
    ASSERT_TRUE(Plato::check_input_upper_bound(tMyRandomVar));
}

TEST(PlatoTest, check_input_standard_deviation)
{
    Plato::RandomVariable tMyRandomVar;
    ASSERT_FALSE(Plato::check_input_standard_deviation(tMyRandomVar));

    tMyRandomVar.mStandardDeviation = "5";
    ASSERT_TRUE(Plato::check_input_standard_deviation(tMyRandomVar));
}

TEST(PlatoTest, check_input_number_samples)
{
    Plato::RandomVariable tMyRandomVar;
    ASSERT_FALSE(Plato::check_input_number_samples(tMyRandomVar));

    tMyRandomVar.mNumSamples = "-1";
    ASSERT_FALSE(Plato::check_input_number_samples(tMyRandomVar));

    tMyRandomVar.mNumSamples = "0";
    ASSERT_FALSE(Plato::check_input_number_samples(tMyRandomVar));

    tMyRandomVar.mNumSamples = "2";
    ASSERT_TRUE(Plato::check_input_number_samples(tMyRandomVar));
}

TEST(PlatoTest, check_input_statistics_uniform)
{
    // DEFINE INPUTS
    Plato::RandomVariable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mGlobalID = "1";
    tMyRandomVar.mDistribution = "uniform";

    // TEST UNDEFINED UPPER BOUND
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED LOWER BOUND
    tMyRandomVar.mLowerBound = "";
    tMyRandomVar.mUpperBound = "135";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED NUMBER OF SAMPLES
    tMyRandomVar.mNumSamples = "";
    tMyRandomVar.mLowerBound = "65";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST NUMBER OF SAMPLES = 0
    tMyRandomVar.mNumSamples = "0";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST NEGATIVE NUMBER OF SAMPLES
    tMyRandomVar.mNumSamples = "-1";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST SUCCESS
    tMyRandomVar.mNumSamples = "4";
    ASSERT_TRUE(Plato::check_input_statistics(tMyRandomVar));
}

TEST(PlatoTest, check_input_statistics_normal)
{
    // DEFINE INPUTS
    Plato::RandomVariable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mGlobalID = "1";
    tMyRandomVar.mNumSamples = "4";
    tMyRandomVar.mLowerBound = "65";
    tMyRandomVar.mUpperBound = "95";
    tMyRandomVar.mStandardDeviation = "5";
    tMyRandomVar.mDistribution = "normal";

    // TEST UNDEFINED MEAN
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED UPPER BOUND
    tMyRandomVar.mMean = "80";
    tMyRandomVar.mUpperBound = "";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED LOWER BOUND
    tMyRandomVar.mLowerBound = "";
    tMyRandomVar.mUpperBound = "95";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED NUMBER OF SAMPLES
    tMyRandomVar.mNumSamples = "";
    tMyRandomVar.mLowerBound = "65";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST NUMBER OF SAMPLES = 0
    tMyRandomVar.mNumSamples = "0";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST NEGATIVE NUMBER OF SAMPLES
    tMyRandomVar.mNumSamples = "-1";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED STANDARD DEVIATION
    tMyRandomVar.mNumSamples = "4";
    tMyRandomVar.mStandardDeviation = "";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST SUCCESS
    tMyRandomVar.mStandardDeviation = "5";
    ASSERT_TRUE(Plato::check_input_statistics(tMyRandomVar));
}

TEST(PlatoTest, check_input_statistics_beta)
{
    // DEFINE INPUTS
    Plato::RandomVariable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mGlobalID = "1";
    tMyRandomVar.mNumSamples = "4";
    tMyRandomVar.mLowerBound = "65";
    tMyRandomVar.mUpperBound = "95";
    tMyRandomVar.mStandardDeviation = "5";
    tMyRandomVar.mDistribution = "beta";

    // TEST UNDEFINED MEAN
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED UPPER BOUND
    tMyRandomVar.mMean = "80";
    tMyRandomVar.mUpperBound = "";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED LOWER BOUND
    tMyRandomVar.mLowerBound = "";
    tMyRandomVar.mUpperBound = "95";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED NUMBER OF SAMPLES
    tMyRandomVar.mNumSamples = "";
    tMyRandomVar.mLowerBound = "65";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST NUMBER OF SAMPLES = 0
    tMyRandomVar.mNumSamples = "0";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST NEGATIVE NUMBER OF SAMPLES
    tMyRandomVar.mNumSamples = "-1";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED STANDARD DEVIATION
    tMyRandomVar.mNumSamples = "4";
    tMyRandomVar.mStandardDeviation = "";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST SUCCESS
    tMyRandomVar.mStandardDeviation = "5";
    ASSERT_TRUE(Plato::check_input_statistics(tMyRandomVar));
}

TEST(PlatoTest, define_input_statistics)
{
    // DEFINE INPUTS
    Plato::RandomVariable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mGlobalID = "1";
    tMyRandomVar.mNumSamples = "4";
    tMyRandomVar.mLowerBound = "65";
    tMyRandomVar.mUpperBound = "95";
    tMyRandomVar.mStandardDeviation = "5";
    tMyRandomVar.mDistribution = "beta";

    // TEST UNDEFINED PARAMETER
    Plato::SromInputs<double> tSromInputs;
    ASSERT_FALSE(Plato::define_input_statistics(tMyRandomVar, tSromInputs));

    // TEST SUCCESS
    tMyRandomVar.mMean = "80";
    ASSERT_TRUE(Plato::define_input_statistics(tMyRandomVar, tSromInputs));

    ASSERT_EQ(4u, tSromInputs.mNumSamples);
    ASSERT_EQ(Plato::DistrubtionName::beta, tSromInputs.mDistribution);

    const double tTolerance = 1e-4;
    ASSERT_NEAR(80.0, tSromInputs.mMean, tTolerance);
    ASSERT_NEAR(25.0, tSromInputs.mVariance, tTolerance);
    ASSERT_NEAR(65.0, tSromInputs.mLowerBound, tTolerance);
    ASSERT_NEAR(95.0, tSromInputs.mUpperBound, tTolerance);
}

TEST(PlatoTest, define_distribution)
{
    Plato::RandomVariable tMyRandomVar;
    Plato::SromInputs<double> tSromInputs;

    tMyRandomVar.mDistribution = "normal";
    ASSERT_TRUE(Plato::define_distribution(tMyRandomVar, tSromInputs));
    ASSERT_EQ(Plato::DistrubtionName::normal, tSromInputs.mDistribution);

    tMyRandomVar.mDistribution = "beta";
    ASSERT_TRUE(Plato::define_distribution(tMyRandomVar, tSromInputs));
    ASSERT_EQ(Plato::DistrubtionName::beta, tSromInputs.mDistribution);

    tMyRandomVar.mDistribution = "uniform";
    ASSERT_TRUE(Plato::define_distribution(tMyRandomVar, tSromInputs));
    ASSERT_EQ(Plato::DistrubtionName::uniform, tSromInputs.mDistribution);

    tMyRandomVar.mDistribution = "lognormal";
    ASSERT_FALSE(Plato::define_distribution(tMyRandomVar, tSromInputs));
}

TEST(PlatoTest, compute_uniform_random_variable_statistics)
{
    Plato::SromInputs<double> tSromInputs;
    tSromInputs.mNumSamples = 4;
    tSromInputs.mLowerBound = 10.0;
    tSromInputs.mUpperBound = 20.0;
    tSromInputs.mDistribution = Plato::DistrubtionName::uniform;
    std::vector<Plato::SromOutputs<double>> tSromOutputs;
    ASSERT_TRUE(Plato::compute_uniform_random_variable_statistics(tSromInputs, tSromOutputs));

    // TEST RESULTS
    double tSum = 0;
    double tTolerance = 1e-4;
    std::vector<double> tGoldSamples = {10, 13.333333333333, 16.666666666667, 20.0};
    std::vector<double> tGoldProbabilities(tSromInputs.mNumSamples, 0.25);
    for(size_t tIndex = 0; tIndex < tSromInputs.mNumSamples; tIndex++)
    {
        tSum += tSromOutputs[tIndex].mSampleWeight;
        ASSERT_NEAR(tGoldSamples[tIndex], tSromOutputs[tIndex].mSampleValue, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tIndex], tSromOutputs[tIndex].mSampleWeight, tTolerance);
    }
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, compute_random_variable_statistics_error)
{
    Plato::SromInputs<double> tSromInputs;
    tSromInputs.mNumSamples = 4;
    tSromInputs.mLowerBound = 10.0;
    tSromInputs.mUpperBound = 20.0;
    tSromInputs.mDistribution = Plato::DistrubtionName::undefined;
    std::vector<Plato::SromOutputs<double>> tSromOutputs;
    ASSERT_FALSE(Plato::compute_random_variable_statistics(tSromInputs, tSromOutputs));
}

TEST(PlatoTest, compute_random_variable_statistics)
{
    Plato::SromInputs<double> tSromInputs;
    tSromInputs.mNumSamples = 4;
    tSromInputs.mLowerBound = 10.0;
    tSromInputs.mUpperBound = 20.0;
    tSromInputs.mDistribution = Plato::DistrubtionName::uniform;
    std::vector<Plato::SromOutputs<double>> tSromOutputs;
    ASSERT_TRUE(Plato::compute_random_variable_statistics(tSromInputs, tSromOutputs));

    // TEST RESULTS
    double tSum = 0;
    double tTolerance = 1e-4;
    std::vector<double> tGoldSamples = {10, 13.333333333333, 16.666666666667, 20.0};
    std::vector<double> tGoldProbabilities(tSromInputs.mNumSamples, 0.25);
    for(size_t tIndex = 0; tIndex < tSromInputs.mNumSamples; tIndex++)
    {
        tSum += tSromOutputs[tIndex].mSampleWeight;
        ASSERT_NEAR(tGoldSamples[tIndex], tSromOutputs[tIndex].mSampleValue, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tIndex], tSromOutputs[tIndex].mSampleWeight, tTolerance);
    }
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, compute_sample_probability_pairs_OneRandVar)
{
    // DEFINE INPUTS
    Plato::RandomVariable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mGlobalID = "1";
    tMyRandomVar.mDistribution = "beta";
    tMyRandomVar.mMean = "85";
    tMyRandomVar.mUpperBound = "65";
    tMyRandomVar.mLowerBound = "135";
    tMyRandomVar.mStandardDeviation = "15";
    tMyRandomVar.mNumSamples = "3";
    std::vector<Plato::RandomVariable> tRandomVarsSet;
    tRandomVarsSet.push_back(tMyRandomVar);

    // SOLVE SROM PROBLEM
    std::vector<Plato::SromRandomVariable> tMySampleProbPairs;
    Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs);

    // TEST RESULTS
    ASSERT_EQ(1u, tMySampleProbPairs.size());
    ASSERT_STREQ("x", tMySampleProbPairs[0].mSubType.c_str());
    ASSERT_STREQ("random rotation", tMySampleProbPairs[0].mType.c_str());
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPair.mNumSamples);
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPair.mSamples.size());
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPair.mProbabilities.size());

    // TEST RESULTS
    double tSum = 0;
    double tTolerance = 1e-4;
    const Plato::SampleProbabilityPairs& tSampleProbabilityPairs = tMySampleProbPairs[0].mSampleProbPair;
    std::vector<double> tGoldSamples = {102.6401761033302, 88.31771931721183, 76.83726384445947};
    std::vector<double> tGoldProbabilities = {0.18178392025984, 0.19071306890276, 0.62750198867198};
    for(size_t tIndex = 0; tIndex < tSampleProbabilityPairs.mNumSamples; tIndex++)
    {
        tSum += tSampleProbabilityPairs.mProbabilities[tIndex];
        ASSERT_NEAR(tGoldSamples[tIndex], tSampleProbabilityPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tIndex], tSampleProbabilityPairs.mProbabilities[tIndex], tTolerance);
    }
    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, compute_sample_probability_pairs_TwoRandVars)
{
    std::vector<Plato::RandomVariable> tRandomVarsSet;

    // DEFINE INPUTS
    Plato::RandomVariable tMyRandomVarOne;
    tMyRandomVarOne.mType = "random rotation";
    tMyRandomVarOne.mSubType = "x";
    tMyRandomVarOne.mGlobalID = "1";
    tMyRandomVarOne.mDistribution = "beta";
    tMyRandomVarOne.mMean = "85";
    tMyRandomVarOne.mLowerBound = "65";
    tMyRandomVarOne.mUpperBound = "135";
    tMyRandomVarOne.mStandardDeviation = "15";
    tMyRandomVarOne.mNumSamples = "3";
    tRandomVarsSet.push_back(tMyRandomVarOne);

    Plato::RandomVariable tMyRandomVarTwo;
    tMyRandomVarTwo.mType = "random rotation";
    tMyRandomVarTwo.mSubType = "y";
    tMyRandomVarTwo.mGlobalID = "1";
    tMyRandomVarTwo.mNumSamples = "4";
    tMyRandomVarTwo.mDistribution = "beta";
    tMyRandomVarTwo.mMean = "70";
    tMyRandomVarTwo.mLowerBound = "50";
    tMyRandomVarTwo.mUpperBound = "120";
    tMyRandomVarTwo.mStandardDeviation = "15";
    tRandomVarsSet.push_back(tMyRandomVarTwo);

    // SOLVE SROM PROBLEM
    std::vector<Plato::SromRandomVariable> tMySampleProbPairs;
    Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs);

    // TEST RESULTS - RANDOM VARIABLE ONE
    ASSERT_EQ(2u, tMySampleProbPairs.size());
    ASSERT_STREQ("x", tMySampleProbPairs[0].mSubType.c_str());
    ASSERT_STREQ("random rotation", tMySampleProbPairs[0].mType.c_str());
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPair.mNumSamples);
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPair.mSamples.size());
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPair.mProbabilities.size());

    double tSum = 0;
    double tTolerance = 1e-4;
    const Plato::SampleProbabilityPairs& tSampleProbabilityPairsOne = tMySampleProbPairs[0].mSampleProbPair;
    std::vector<double> tGoldSamplesOne = {79.56461506624, 95.1780010696, 104.3742043151};
    std::vector<double> tGoldProbabilitiesOne = {0.441549282785, 0.3256625620299, 0.2326524892665};
    for(size_t tIndex = 0; tIndex < tSampleProbabilityPairsOne.mNumSamples; tIndex++)
    {
        tSum += tSampleProbabilityPairsOne.mProbabilities[tIndex];
        ASSERT_NEAR(tGoldSamplesOne[tIndex], tSampleProbabilityPairsOne.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tGoldProbabilitiesOne[tIndex], tSampleProbabilityPairsOne.mProbabilities[tIndex], tTolerance);
    }
    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);

    // TEST RESULTS - RANDOM VARIABLE TWO
    ASSERT_STREQ("y", tMySampleProbPairs[1].mSubType.c_str());
    ASSERT_STREQ("random rotation", tMySampleProbPairs[1].mType.c_str());
    ASSERT_EQ(4u, tMySampleProbPairs[1].mSampleProbPair.mNumSamples);
    ASSERT_EQ(4u, tMySampleProbPairs[1].mSampleProbPair.mSamples.size());
    ASSERT_EQ(4u, tMySampleProbPairs[1].mSampleProbPair.mProbabilities.size());

    tSum = 0;
    tTolerance = 1e-4;
    const Plato::SampleProbabilityPairs& tSampleProbabilityPairsTwo = tMySampleProbPairs[1].mSampleProbPair;
    std::vector<double> tGoldSamplesTwo = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    std::vector<double> tGoldProbabilitiesTwo = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    for(size_t tIndex = 0; tIndex < tSampleProbabilityPairsTwo.mNumSamples; tIndex++)
    {
        tSum += tSampleProbabilityPairsTwo.mProbabilities[tIndex];
        ASSERT_NEAR(tGoldSamplesTwo[tIndex], tSampleProbabilityPairsTwo.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tGoldProbabilitiesTwo[tIndex], tSampleProbabilityPairsTwo.mProbabilities[tIndex], tTolerance);
    }
    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, compute_sample_probability_pairs_error_undefined_distribution)
{
    // DEFINE INPUTS
    Plato::RandomVariable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mGlobalID = "1";
    tMyRandomVar.mDistribution = "";
    tMyRandomVar.mMean = "85";
    tMyRandomVar.mUpperBound = "135";
    tMyRandomVar.mLowerBound = "65";
    tMyRandomVar.mStandardDeviation = "15";
    tMyRandomVar.mNumSamples = "3";
    std::vector<Plato::RandomVariable> tRandomVarsSet;
    tRandomVarsSet.push_back(tMyRandomVar);

    // SOLVE SROM PROBLEM
    std::vector<Plato::SromRandomVariable> tMySampleProbPairs;
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));
}

TEST(PlatoTest, compute_sample_probability_pairs_error_empty_random_var_set)
{
    // SOLVE SROM PROBLEM
    std::vector<Plato::RandomVariable> tRandomVarsSet;
    std::vector<Plato::SromRandomVariable> tMySampleProbPairs;
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));
}

TEST(PlatoTest, compute_sample_probability_pairs_error_undefined_input_statistics_uniform)
{
    // DEFINE INPUTS
    Plato::RandomVariable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mGlobalID = "1";
    tMyRandomVar.mDistribution = "uniform";
    tMyRandomVar.mLowerBound = "65";
    tMyRandomVar.mUpperBound = "";
    tMyRandomVar.mNumSamples = "3";
    std::vector<Plato::RandomVariable> tRandomVarsSet;
    tRandomVarsSet.push_back(tMyRandomVar);

    // TEST UNDEFINED UPPER BOUND
    std::vector<Plato::SromRandomVariable> tMySampleProbPairs;
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));

    // TEST UNDEFINED LOWER BOUND
    tRandomVarsSet[0].mLowerBound = "";
    tRandomVarsSet[0].mUpperBound = "135";
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));

    // TEST UNDEFINED NUMBER OF SAMPLES
    tRandomVarsSet[0].mNumSamples = "";
    tRandomVarsSet[0].mLowerBound = "65";
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));

    // TEST NUMBER OF SAMPLES = 0
    tRandomVarsSet[0].mNumSamples = "0";
    tRandomVarsSet[0].mLowerBound = "65";
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));

    // TEST NEGATIVE NUMBER OF SAMPLES
    tRandomVarsSet[0].mNumSamples = "-1";
    tRandomVarsSet[0].mLowerBound = "65";
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));
}

}
