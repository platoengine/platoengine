/*
 * Plato_Test_UncertainLoadGeneratorXML.cpp
 *
 *  Created on: Jun 1, 2019
 */

#include <gtest/gtest.h>
#include <sys/_types/_size_t.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "../../../base/src/input_generator/XMLGeneratorDataStruct.hpp"
#include "../../../base/src/optimize/Plato_KelleySachsAugmentedLagrangianLightInterface.hpp"
#include "../../../base/src/optimize/Plato_SolveUncertaintyProblem.hpp"
#include "../../../base/src/optimize/Plato_SromProbDataStruct.hpp"
#include "../../../base/src/tools/Plato_UniqueCounter.hpp"
#include "../../../base/src/tools/Plato_Vector3DVariations.hpp"

namespace Plato
{

struct Rotation
{
    enum type_t
    {
        xyz = 0, xy = 1 , xz = 2, yz = 3, x = 4, y = 5, z = 6
    };
};

struct RandomVariable
{
    std::string mType;     // currently always "angle variation"
    std::string mSubType;  // x, y, z
    std::string mGlobalID; // which random variable to vary
    std::string mDistribution; // normal, uniform, beta
    std::string mMean; // scalar value
    std::string mUpperBound; // scalar value
    std::string mLowerBound; // scalar value
    std::string mStandardDeviation; // scalar value
    std::string mNumSamples; // integer value
};

struct SampleProbabilityPairs
{
    std::vector<double> mSamples;
    std::vector<double> mProbabilities;
};

struct SromRandomVariable
{
    std::string mType;     // currently always "angle variation"
    std::string mSubType;  // x, y, z
    Plato::SampleProbabilityPairs mSampleProbPair;
};

struct RandomRotations
{
    double mProbability;
    Plato::Vector3D mRotationAngles;
};

struct RandomLoad
{
    int mLoadID;
    double mProbability;
    Plato::Vector3D mLoad;
};

struct RandomLoadCase
{
    int mLoadCaseID;
    double mProbability;
    std::vector<Plato::RandomLoad> mLoads;
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

inline bool register_all_load_ids(const std::vector<LoadCase> & aLoadCases, Plato::UniqueCounter & aUniqueLoadCounter)
{
    if(aLoadCases.size() <= 0)
    {
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
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
}

inline bool define_distribution(const Plato::RandomVariable & aMyRandomVar, Plato::SromInputs<double> & aInput)
{
    if(aMyRandomVar.mDistribution == "normal")
    {
        aInput.mDistribution = Plato::DistrubtionName::type_t::normal;
    }
    else if(aMyRandomVar.mDistribution == "uniform")
    {
        aInput.mDistribution = Plato::DistrubtionName::type_t::uniform;
    }
    else if(aMyRandomVar.mDistribution == "beta")
    {
        aInput.mDistribution = Plato::DistrubtionName::type_t::beta;
    }
    else
    {
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: UNDEFINE DISTRIBUTION. OPTIONS ARE NORMAL, UNIFORM, AND BETA.\n";
        return (false);
    }

    return (true);
}

inline bool post_process_sample_probability_pairs(const std::vector<Plato::SromOutputs<double>> aMySromSolution,
                                                  const Plato::RandomVariable & aMyRandomVariable,
                                                  Plato::SromRandomVariable & aMySromRandomVariable)
{
    if(aMySromSolution.size() <= 0)
    {
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: SROM SOLUTION IS EMPTY.\n";
        return (false);
    }

    aMySromRandomVariable.mSampleProbPair.mSamples.clear();
    aMySromRandomVariable.mSampleProbPair.mProbabilities.clear();

    aMySromRandomVariable.mType = aMyRandomVariable.mType;
    aMySromRandomVariable.mSubType = aMyRandomVariable.mSubType;

    const size_t tNumSamples = aMySromSolution.size();
    aMySromRandomVariable.mSampleProbPair.mSamples.resize(tNumSamples);
    aMySromRandomVariable.mSampleProbPair.mProbabilities.resize(tNumSamples);

    for(size_t tIndex = 0; tIndex < tNumSamples; tIndex++)
    {
        aMySromRandomVariable.mSampleProbPair.mSamples[tIndex] = aMySromSolution[tIndex].mSampleValue;
        aMySromRandomVariable.mSampleProbPair.mProbabilities[tIndex] = aMySromSolution[tIndex].mSampleWeight;
    }

    return (true);
}

inline bool compute_sample_probability_pairs(const std::vector<Plato::RandomVariable> & aMySetRandomVars,
                                             std::vector<Plato::SromRandomVariable> & aMySampleProbPairs)
{
    if(aMySetRandomVars.size() <= 0)
    {
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF RANDOM VARIABLES IS EMPTY.\n";
        return (false);
    }

    aMySampleProbPairs.clear();

    const size_t tMyNumRandomVars = aMySetRandomVars.size();
    aMySampleProbPairs.resize(tMyNumRandomVars);
    for(size_t tRandomVarIndex = 0; tRandomVarIndex < tMyNumRandomVars; tRandomVarIndex++)
    {
        // pose uncertainty
        Plato::SromInputs<double> tInput;
        const Plato::RandomVariable & tMyRandomVar = aMySetRandomVars[tRandomVarIndex];
        if(Plato::define_distribution(tMyRandomVar, tInput))
        {
            std::cout<< "\nFILE: " __FILE__
                     << "\nFUNCTION: " __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: PROBABILITY DISTIRBUTION WAS NOT DEFINE FOR RANDOM VARIABLE #" << tRandomVarIndex << ".\n";
            return (false);
        }

        tInput.mMean = std::atof(tMyRandomVar.mMean.c_str());
        tInput.mLowerBound = std::atof(tMyRandomVar.mLowerBound.c_str());
        tInput.mUpperBound = std::atof(tMyRandomVar.mUpperBound.c_str());
        const double tStdDev = std::atof(tMyRandomVar.mStandardDeviation.c_str());
        tInput.mVariance = tStdDev * tStdDev;
        const int tNumSamples = std::atoi(tMyRandomVar.mNumSamples.c_str());
        tInput.mNumSamples = tNumSamples;

        // solve uncertainty sub-problem
        const bool tEnableOutput = true;
        Plato::AlgorithmInputsKSAL<double> tAlgoInputs;
        Plato::SromDiagnostics<double> tSromDiagnostics;
        std::vector<Plato::SromOutputs<double>> tSromOutput;
        Plato::solve_uncertainty(tInput, tAlgoInputs, tSromDiagnostics, tSromOutput, tEnableOutput);

        Plato::SromRandomVariable & tMySampleProbPairs = aMySampleProbPairs[tRandomVarIndex];
        if(Plato::post_process_sample_probability_pairs(tSromOutput, tMyRandomVar, tMySampleProbPairs))
        {
            std::cout<< "\nFILE: " __FILE__
                     << "\nFUNCTION: " __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: SAMPLE PROBABILITY PAIR POST PROCESSING FAILED FOR RANDOM VARIABLE #" << tRandomVarIndex << ".\n";
            return (false);
        }
    }

    return (true);
}

inline bool post_process_random_load(const std::vector<Plato::SromRandomVariable> & aMySampleProbPairs,
                                     std::map<Plato::axis3D::axis3D, Plato::SampleProbabilityPairs> & aRotationAxisToSampleProbPairs)
{
    if(aMySampleProbPairs.size() <= 0)
    {
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
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
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
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
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT VECTOR OF RANDOM ROTATIONS IS EMPTY.\n";
        return (false);
    }

    const size_t tNumRandomLoads = aMyRandomRotations.size();
    for(const size_t tIndex = 0; tIndex < tNumRandomLoads; tIndex++)
    {
        Plato::RandomLoad tMyRandomLoad;
        tMyRandomLoad.mLoad = aOriginalLoad;
        tMyRandomLoad.mProbability = aMyRandomRotations[tIndex].mProbability;
        if(Plato::apply_rotation_matrix(aMyRandomRotations[tIndex].mRotationAngles, tMyRandomLoad.mLoad) == false)
        {
            std::cout<< "\nFILE: " __FILE__
                     << "\nFUNCTION: " __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: APPLICATION OF ROTATION MATRIS WAS UNSUCCESSFUL.\n";
            return (false)
        }
        tMyRandomLoad.mLoadID = aUniqueLoadCounter.assign_next_unique();
        aMyRandomLoads.push_back(tMyRandomLoad);
    }

    return (true);
}

inline bool expand_random_load_cases(const std::vector<Plato::RandomLoad> & aNewRandomLoads,
                                     std::vector<Plato::RandomLoadCase> & aOldRandomLoadCases,
                                     Plato::UniqueCounter & aUniqueLoadCounter)
{
    if(aNewRandomLoads.empty())
    {
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: NEW VECTOR OF RANDOM LOADS IS EMPTY.\n";
        return (false);
    }
    else if(!aNewRandomLoads.empty() && aOldRandomLoadCases.empty())
    {
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: NEW VECTOR OF RANDOM LOADS IS NOT EMPTY. "
                 << "HOWEVER, THE PREVIOUS SET OF LOAD CASES IS EMPTY. "
                 << "THIS USE CASE IS NOT VALID.\n";
        return (false);
    }

    std::vector<Plato::RandomLoadCase> tNewSetRandomLoadCase;

    if(aOldRandomLoadCases.empty())
    {
        for(size_t tLoadIndex = 0; tLoadIndex < aNewRandomLoads.size(); tLoadIndex++)
        {
            Plato::RandomLoadCase tNewRandomLoadCase;
            const Plato::RandomLoad& tMyNewRandomLoad = aNewRandomLoads[tLoadIndex];
            tNewRandomLoadCase.mLoads.push_back(tMyNewRandomLoad);
            tNewRandomLoadCase.mProbability = tMyNewRandomLoad.mProbability;
            tNewRandomLoadCase.mLoadCaseID = aUniqueLoadCounter.assign_next_unique();
            tNewSetRandomLoadCase.push_back(tNewRandomLoadCase);
        } // index over new random loads
    } // if-else
    else
    {
        for(size_t tNewLoadIndex = 0; tNewLoadIndex < aNewRandomLoads.size(); tNewLoadIndex++)
        {
            const Plato::RandomLoad& tMyNewRandomLoad = aNewRandomLoads[tNewLoadIndex];
            std::vector<Plato::RandomLoadCase> tTempRandomLoadCases = aOldRandomLoadCases;
            for(size_t tOldLoadCaseIndex = 0; tOldLoadCaseIndex < aOldRandomLoadCases.size(); tOldLoadCaseIndex++)
            {
                tTempRandomLoadCases[tOldLoadCaseIndex].mLoads.push_back(tMyNewRandomLoad);
                const Plato::RandomLoadCase& tMyTempRandomLoadCase = tTempRandomLoadCases[tOldLoadCaseIndex];
                tMyTempRandomLoadCase.mLoadCaseID = aUniqueLoadCounter.assign_next_unique();
                tMyTempRandomLoadCase.mProbability = tMyTempRandomLoadCase.mProbability * tMyNewRandomLoad.mProbability;
                tNewSetRandomLoadCase.push_back(tMyTempRandomLoadCase);
            } // index over old random load cases
        } // index over new random loads
    } // if-else

    aOldRandomLoadCases = tNewSetRandomLoadCase;

    return (true);
}

}
