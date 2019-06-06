/*
 * Plato_Test_UncertainLoadGeneratorXML.cpp
 *
 *  Created on: Jun 1, 2019
 */

#include <gtest/gtest.h>

#include <math.h>
#include <stdlib.h>
#include <cstdlib>

#include "Plato_UniqueCounter.hpp"
#include "XMLGeneratorDataStruct.hpp"
#include "Plato_SromProbDataStruct.hpp"
#include "Plato_Vector3DVariations.hpp"
#include "Plato_SolveUncertaintyProblem.hpp"
#include "Plato_KelleySachsAugmentedLagrangianLightInterface.hpp"

namespace Plato
{

struct Rotation
{
    enum type_t
    {
        xyz = 0, xy = 1 , xz = 2, yz = 3, x = 4, y = 5, z = 6
    };
};

struct RandomVar
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

struct SromRandomVar
{
    std::string mType;     // currently always "angle variation"
    std::string mSubType;  // x, y, z
    std::string mGlobalID; // which random variable to vary
    Plato::SampleProbabilityPairs mSampleProbPair;
};

struct RandomRotations
{
    double mProbability;
    Plato::Vector3D mRotationAngles
};

struct RandomLoad
{
    int mLoadID;
    double mProbability;
    Plato::Vector3D mLoad
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

inline bool register_random_variables(const std::vector<Plato::RandomVar> & aAllRandomVars,
                                      std::map<int, std::vector<Plato::RandomVar>> & aGlobalIdToRandomVar)
{
    if(aAllRandomVars.size() <= 0)
    {
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF RANDOM VARIABLES IS EMPTY.\n";
        return (false);
    }

    const int tNumRandomVars = aAllRandomVars.size();
    for(int tPrivateRandomVarIndex = 0; tPrivateRandomVarIndex < tNumRandomVars; tPrivateRandomVarIndex++)
    {
        // register random variables
        const int tMyGlobalId = std::atoi(aAllRandomVars[tPrivateRandomVarIndex].mGlobalID.c_str());
        aGlobalIdToRandomVar[tMyGlobalId].push_back(aAllRandomVars[tPrivateRandomVarIndex]);
    }

    return (true);
}

inline bool register_random_var_global_ids(std::map<int, std::vector<Plato::RandomVar>> & aGlobalIdToRandomVar,
                                           Plato::UniqueCounter & aUniqueRandomVarCounter)
{
    if(aGlobalIdToRandomVar.size() <= 0)
    {
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: GLOBAL ID TO RANDOM VARIABLE MAP IS EMPTY.\n";
        return (false);
    }

    std::map<int, std::vector<Plato::RandomVar>>::iterator tIterator;
    for(tIterator = aGlobalIdToRandomVar.begin(); tIterator != aGlobalIdToRandomVar.end(); ++tIterator)
    {
        const int tMyGLobalRandomVarId = tIterator->first;
        aUniqueRandomVarCounter.mark(tMyGLobalRandomVarId);
    }

    return (true);
}

inline bool define_distribution(const Plato::RandomVar & aMyRandomVar, Plato::UncertaintyInputStruct<double> & aInput)
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

inline bool post_process_sample_probability_pairs(const std::vector<Plato::UncertaintyOutputStruct<double>> aMySromSolution,
                                                  const Plato::RandomVar & tMyRandomVar,
                                                  Plato::SromRandomVar & aMySampleProbPairs)
{
    if(aMySromSolution.size() <= 0)
    {
        std::cout<< "\nFILE: " __FILE__
                 << "\nFUNCTION: " __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: SROM SOLUTION IS EMPTY.\n";
        return (false);
    }

    aMySampleProbPairs.mSampleProbPair.mSamples.clear();
    aMySampleProbPairs.mSampleProbPair.mProbabilities.clear();

    aMySampleProbPairs.mType = tMyRandomVar.mType;
    aMySampleProbPairs.mSubType = tMyRandomVar.mSubType;
    aMySampleProbPairs.mGlobalID = tMyRandomVar.mGlobalID;

    const size_t tNumSamples = aMySromSolution.size();
    aMySampleProbPairs.mSampleProbPair.mSamples.resize(tNumSamples);
    aMySampleProbPairs.mSampleProbPair.mProbabilities.resize(tNumSamples);

    for(size_t tIndex = 0; tIndex < tNumSamples; tIndex++)
    {
        aMySampleProbPairs.mSampleProbPair.mSamples[tIndex] = aMySromSolution[tIndex].mSampleValue;
        aMySampleProbPairs.mSampleProbPair.mProbabilities[tIndex] = aMySromSolution[tIndex].mSampleWeight;
    }

    return (true);
}

inline bool compute_sample_probability_pairs(const std::vector<Plato::RandomVar> & aMySetRandomVars,
                                             std::vector<Plato::SromRandomVar> & aMySampleProbPairs)
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
        Plato::UncertaintyInputStruct<double> tInput;
        const Plato::RandomVar & tMyRandomVar = aMySetRandomVars[tRandomVarIndex];
        if(Plato::define_distribution(tMyRandomVar, tInput))
        {
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
        Plato::SromProblemDiagnosticsStruct<double> tSromDiagnostics;
        std::vector<Plato::UncertaintyOutputStruct<double>> tSromOutput;
        Plato::solve_uncertainty(tInput, tAlgoInputs, tSromDiagnostics, tSromOutput, tEnableOutput);

        Plato::SromRandomVar & tMySampleProbPairs = aMySampleProbPairs[tRandomVarIndex];
        if(Plato::post_process_sample_probability_pairs(tSromOutput, tMyRandomVar, tMySampleProbPairs))
        {
            return (false);
        }
    }

    return (true);
}

inline bool post_process_random_load(const std::vector<Plato::SromRandomVar> & aMySampleProbPairs,
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
        Plato::apply_rotation_matrix(aMyRandomRotations[tIndex].mRotationAngles, tMyRandomLoad.mLoad);
        aMyRandomLoads.push_back(tMyRandomLoad);
    }

    return (true);
}

}
