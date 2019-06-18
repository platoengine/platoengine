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
#include <memory>

#include "../../base/src/tools/Plato_UniqueCounter.hpp"
#include "../../base/src/tools/Plato_Vector3DVariations.hpp"
#include "../../base/src/optimize/Plato_SromProbDataStruct.hpp"
#include "../../base/src/optimize/Plato_SolveUncertaintyProblem.hpp"
#include "../../base/src/optimize/Plato_KelleySachsAugmentedLagrangianLightInterface.hpp"
#include "../../base/src/input_generator/XMLGeneratorDataStruct.hpp"

namespace Plato
{

struct VariableType
{

enum type_t
{
    LOAD, MATERIAL, UNDEFINED
};

};

inline bool variable_type_string_to_enum(const std::string& aStringVarType, Plato::VariableType::type_t& aEnumVarType)
{
    if(aStringVarType == "material")
    {
        aEnumVarType = Plato::VariableType::MATERIAL;
    }
    else if(aStringVarType == "load")
    {
        aEnumVarType = Plato::VariableType::LOAD;
    }
    else
    {
        return (false);
    }

    return (true);
}

namespace srom
{

/******************************************************************************//**
 * @brief Statistics metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct Statistics
{
    std::string mNumSamples; /*!< number of samples */
    std::string mDistribution; /*!< probability distribution */
    std::string mMean; /*!< probability distribution mean */
    std::string mUpperBound; /*!< probability distribution upper bound */
    std::string mLowerBound; /*!< probability distribution lower bound */
    std::string mStandardDeviation; /*!< probability distribution standard deviation */
};

/******************************************************************************//**
 * @brief Variable metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct Variable
{
    std::string mType; /*!< random variable type, e.g. random rotation */
    std::string mSubType; /*!< random variable subtype, e.g. rotation axis */
    Plato::srom::Statistics mStatistics; /*!< statistics for this random variable */
};

/******************************************************************************//**
 * @brief Load metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct Load
{
    int mAppID; /*!< application set identifier */
    std::string mLoadID; /*!< global load identifier */
    std::string mAppType; /*!< application type, e.g. sideset, nodeset, etc. */
    std::string mLoadType; /*!< load type, e.g. pressure, traction, etc. */
    std::vector<std::string> mValues; /*!< load magnitudes, e.g. \f$F = \{F_x, F_y, F_z\}\f$. */
    std::vector<Plato::srom::Variable> mRandomVars; /*!< set of random variables, e.g. rotations, \f$\theta = \{\theta_x, \theta_y, \theta_z\}\f$. */
};

/******************************************************************************//**
 * @brief Sample-Probability pairs defined by the Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct SampleProbabilityPairs
{
    int mNumSamples; /*!< total number of samples */
    std::vector<double> mSamples; /*!< sample set */
    std::vector<double> mProbabilities; /*!< probability set  */
};

/******************************************************************************//**
 * @brief Random vriable metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct RandomVariable
{
    std::string mType; /*!< random variable type, e.g. random rotation */
    std::string mSubType; /*!< random variable subtype, e.g. rotation axis */
    Plato::srom::SampleProbabilityPairs mSampleProbPairs; /*!< sample-probability pair for this random variable */
};

/******************************************************************************//**
 * @brief Random rotations metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct RandomRotations
{
    double mProbability; /*!< probability associated with this random rotation */
    Plato::Vector3D mRotations; /*!< vector of random rotations, e.g. /f$(\theta_x, \theta_y, \theta_z)/f$ */
};

/******************************************************************************//**
 * @brief Random load metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct RandomLoad
{
    int mAppID; /*!< application set identifier */
    int mLoadID; /*!< load identifier */
    double mProbability; /*!< probability associated with this random load */
    std::string mAppType; /*!< application type, e.g. sideset, nodeset, etc. */
    std::string mLoadType; /*!< load type, e.g. pressure, traction, etc. */
    Plato::Vector3D mLoadValues; /*!< load components, e.g. /f$(f_x, f_y, f_z)/f$ */
};

/******************************************************************************//**
 * @brief Random load case metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct RandomLoadCase
{
    int mLoadCaseID; /*!< random load case global identifier */
    double mProbability; /*!< probability associated with this random load case */
    std::vector<Plato::srom::RandomLoad> mLoads; /*!< set of random loads associated with this random load case */
};

/******************************************************************************//**
 * @brief Output metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct OutputMetaData
{
    std::vector<Plato::srom::RandomLoadCase> mLoadCases; /*!< set of random load cases */
};

/******************************************************************************//**
 * @brief Input metadata for Stochastic Reduced Order Model (SROM) problem.
**********************************************************************************/
struct InputMetaData
{
    std::vector<Plato::srom::Load> mLoads; /*!< set of loads */
};

}
// namespace srom

inline bool check_vector3d_values(const Plato::Vector3D & aMyOriginalLoad)
{
    if(std::isfinite(aMyOriginalLoad.mX) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: X-COMPONENT IS NOT A FINITE NUMBER.\n";
        return (false);
    }
    else if(std::isfinite(aMyOriginalLoad.mY) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: Y-COMPONENT IS NOT A FINITE NUMBER.\n";
        return (false);
    }
    else if(std::isfinite(aMyOriginalLoad.mZ) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: Z-COMPONENT IS NOT A FINITE NUMBER.\n";
        return (false);
    }

    return (true);
}

inline bool initialize_load_id_counter(const std::vector<XMLGen::LoadCase> &aLoadCases,
                                       Plato::UniqueCounter &aUniqueCounter)
{
    if(aLoadCases.empty() == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF LOAD CASES IS EMPTY.\n";
        return (false);
    }

    aUniqueCounter.mark(0); // Mark 0 as true since we don't want to have a 0 ID
    for(size_t tIndex = 0; tIndex < aLoadCases.size(); tIndex++)
    {
        aUniqueCounter.mark(std::atoi(aLoadCases[tIndex].id.c_str()));
    }

    return (true);
}

inline bool expand_single_load_case(const XMLGen::LoadCase &aOldLoadCase,
                                    std::vector<XMLGen::LoadCase> &aNewLoadCaseList,
                                    Plato::UniqueCounter &aUniqueLoadIDCounter,
                                    std::map<int, std::vector<int> > &tOriginalToNewLoadCaseMap)
{
    if(aOldLoadCase.loads.empty() == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: LOAD CASE #" << aOldLoadCase.id << " HAS AN EMPTY LOADS SET.\n";
        return (false);
    }

    int tOriginalLoadCaseID = std::atoi(aOldLoadCase.id.c_str());
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < aOldLoadCase.loads.size(); tLoadCaseIndex++)
    {
        std::string tIDString = aOldLoadCase.id;
        int tCurLoadCaseID = std::atoi(tIDString.c_str());
        // If this is a multi-load load case we need to generate a new load case with a new id.
        if(tLoadCaseIndex > 0)
        {
            tCurLoadCaseID = aUniqueLoadIDCounter.assignNextUnique();
            tIDString = std::to_string(tCurLoadCaseID);
        }
        tOriginalToNewLoadCaseMap[tOriginalLoadCaseID].push_back(aNewLoadCaseList.size());
        XMLGen::LoadCase tNewLoadCase = aOldLoadCase;
        tNewLoadCase.id = tIDString;
        tNewLoadCase.loads[0] = aOldLoadCase.loads[tLoadCaseIndex];
        tNewLoadCase.loads.resize(1);
        aNewLoadCaseList.push_back(tNewLoadCase);
    }

    return (true);
}

inline bool expand_load_cases(const std::vector<XMLGen::LoadCase> &aInputLoadCases,
                              std::vector<XMLGen::LoadCase> &aNewLoadCaseList,
                              std::map<int, std::vector<int> > &aOriginalToNewLoadCaseMap)
{
    Plato::UniqueCounter tUniqueLoadIDCounter;
    if(Plato::initialize_load_id_counter(aInputLoadCases, tUniqueLoadIDCounter) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: FAILED TO INITIALIZE ORIGINAL SET OF LOAD IDENTIFIERS.\n";
        return (false);
    }

    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < aInputLoadCases.size(); tLoadCaseIndex++)
    {
        const XMLGen::LoadCase& tOldLoadCase = aInputLoadCases[tLoadCaseIndex];
        Plato::expand_single_load_case(tOldLoadCase, aNewLoadCaseList, tUniqueLoadIDCounter, aOriginalToNewLoadCaseMap);
    }

    return (true);
}

inline bool set_random_variable_statistics(const XMLGen::Uncertainty &aRandomVariable, Plato::srom::Statistics& aStatistics)
{
    bool tInputStatisticsError = aRandomVariable.mean.empty() || aRandomVariable.upper.empty() || aRandomVariable.lower.empty()
            || aRandomVariable.num_samples.empty() || aRandomVariable.distribution.empty() || aRandomVariable.standard_deviation.empty();
    if(tInputStatisticsError == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: STATISTICS FOR RANDOM VARIABLE #" << aRandomVariable.id << " WERE NOT PROPERLY DEFINED.\n";
        return (false);
    }

    aStatistics.mMean = aRandomVariable.mean;
    aStatistics.mUpperBound = aRandomVariable.upper;
    aStatistics.mLowerBound = aRandomVariable.lower;
    aStatistics.mNumSamples = aRandomVariable.num_samples;
    aStatistics.mDistribution = aRandomVariable.distribution;
    aStatistics.mStandardDeviation = aRandomVariable.standard_deviation;

    return (true);
}

inline bool create_deterministic_load_variable(const XMLGen::LoadCase &aLoadCase, Plato::srom::Load& aLoad)
{
    if(aLoadCase.loads.empty() == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: LOAD CASE #" << aLoadCase.id << " HAS AN EMPTY LOADS SET.\n";
        return (false);
    }

    aLoad.mValues.push_back(aLoadCase.loads[0].x);
    aLoad.mValues.push_back(aLoadCase.loads[0].y);
    aLoad.mValues.push_back(aLoadCase.loads[0].z);

    aLoad.mLoadID = aLoadCase.id;
    aLoad.mAppType = aLoadCase.loads[0].app_type;
    aLoad.mLoadType = aLoadCase.loads[0].type;
    aLoad.mAppID = std::atoi(aLoadCase.loads[0].app_id.c_str());

    return (true);
}

inline int get_or_create_random_load_variable(const XMLGen::LoadCase &aLoadCase,
                                              std::vector<Plato::srom::Load> &aRandomLoads)
{
    for(size_t tRandomLoadIndex = 0; tRandomLoadIndex < aRandomLoads.size(); ++tRandomLoadIndex)
    {
        if(aRandomLoads[tRandomLoadIndex].mLoadID == aLoadCase.id)
            return tRandomLoadIndex;
    }

    Plato::srom::Load tNewLoad;
    tNewLoad.mLoadType = aLoadCase.loads[0].type;
    tNewLoad.mAppType = aLoadCase.loads[0].app_type;
    tNewLoad.mAppID = std::atoi(aLoadCase.loads[0].app_id.c_str());
    tNewLoad.mLoadID = aLoadCase.id;
    tNewLoad.mValues.push_back(aLoadCase.loads[0].x);
    tNewLoad.mValues.push_back(aLoadCase.loads[0].y);
    tNewLoad.mValues.push_back(aLoadCase.loads[0].z);
    aRandomLoads.push_back(tNewLoad);

    return (aRandomLoads.size() - 1);
}

inline void add_random_variable_to_random_load(Plato::srom::Load &aRandomLoad,
                                               const XMLGen::Uncertainty &aRandomVariable)
{
    Plato::srom::Variable tNewVariable;
    tNewVariable.mType = aRandomVariable.type;
    tNewVariable.mSubType = aRandomVariable.axis;
    tNewVariable.mStatistics.mDistribution = aRandomVariable.distribution;
    tNewVariable.mStatistics.mLowerBound = aRandomVariable.lower;
    tNewVariable.mStatistics.mMean = aRandomVariable.mean;
    tNewVariable.mStatistics.mNumSamples = aRandomVariable.num_samples;
    tNewVariable.mStatistics.mStandardDeviation = aRandomVariable.standard_deviation;
    tNewVariable.mStatistics.mUpperBound = aRandomVariable.upper;
    aRandomLoad.mRandomVars.push_back(tNewVariable);
}

inline void create_random_loads_from_uncertainty(const XMLGen::Uncertainty& aRandomVariable,
                                                 const std::vector<XMLGen::LoadCase> &aNewLoadCases,
                                                 std::map<int, std::vector<int> > &aOriginalToNewLoadCaseMap,
                                                 std::set<int> &aRandomLoadIDs,
                                                 std::vector<Plato::srom::Load> &aRandomLoads)
{
    const int tRandomVarLoadID = std::atoi(aRandomVariable.id.c_str());
    for(size_t tIndexJ = 0; tIndexJ < aOriginalToNewLoadCaseMap[tRandomVarLoadID].size(); tIndexJ++)
    {
        const int tIndexIntoNewLoadCaseList = aOriginalToNewLoadCaseMap[tRandomVarLoadID][tIndexJ];
        std::string tCurLoadCaseIDString = aNewLoadCases[tIndexIntoNewLoadCaseList].id;
        const int tCurLoadCaseID = std::atoi(tCurLoadCaseIDString.c_str());
        aRandomLoadIDs.insert(tCurLoadCaseID);
        const int tIndexOfRandomLoad = get_or_create_random_load_variable(aNewLoadCases[tIndexIntoNewLoadCaseList], aRandomLoads);
        add_random_variable_to_random_load(aRandomLoads[tIndexOfRandomLoad], aRandomVariable);
    }
}

inline void create_random_load_variables(const std::vector<XMLGen::Uncertainty> &aRandomVariables,
                                         const std::vector<XMLGen::LoadCase> &aNewLoadCases,
                                         std::map<int, std::vector<int> > &aOriginalToNewLoadCaseMap,
                                         std::set<int> &aRandomLoadIDs,
                                         std::vector<Plato::srom::Load> &aLoad)
{
    const size_t tNumRandomVars = aRandomVariables.size();
    for(size_t tRandomVarIndex = 0; tRandomVarIndex < tNumRandomVars; ++tRandomVarIndex)
    {
        const XMLGen::Uncertainty& tRandomVariable = aRandomVariables[tRandomVarIndex];
        Plato::VariableType::type_t tVarType = Plato::VariableType::UNDEFINED;
        Plato::variable_type_string_to_enum(tRandomVariable.variable_type, tVarType);
        if(tVarType == Plato::VariableType::LOAD)
        {
            Plato::create_random_loads_from_uncertainty(tRandomVariable, aNewLoadCases, aOriginalToNewLoadCaseMap, aRandomLoadIDs, aLoad);
        }
    }
}

inline void create_deterministic_load_variables(const std::vector<XMLGen::LoadCase> &aNewLoadCases,
                                                const std::set<int> & aRandomLoadIDs,
                                                std::vector<Plato::srom::Load> &aLoad)
{
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < aNewLoadCases.size(); tLoadCaseIndex++)
    {
        const int tCurLoadCaseID = std::atoi(aNewLoadCases[tLoadCaseIndex].id.c_str());
        if(aRandomLoadIDs.find(tCurLoadCaseID) == aRandomLoadIDs.end())
        {
            Plato::srom::Load tNewLoad;
            Plato::create_deterministic_load_variable(aNewLoadCases[tLoadCaseIndex], tNewLoad);
            aLoad.push_back(tNewLoad);
        }
    }
}

inline bool generate_srom_load_inputs(const std::vector<XMLGen::LoadCase> &aInputLoadCases,
                                      const std::vector<XMLGen::Uncertainty> &aUncertainties,
                                      std::vector<Plato::srom::Load> &aLoads)
{
    std::vector<XMLGen::LoadCase> tNewSetLoadCases;
    std::map<int, std::vector<int> > tOriginalToNewLoadCaseMap;
    Plato::expand_load_cases(aInputLoadCases, tNewSetLoadCases, tOriginalToNewLoadCaseMap);

    std::set<int> tRandomLoadIDs;
    Plato::create_random_load_variables(aUncertainties,
                                        tNewSetLoadCases,
                                        tOriginalToNewLoadCaseMap,
                                        tRandomLoadIDs,
                                        aLoads);

    Plato::create_deterministic_load_variables(tNewSetLoadCases, tRandomLoadIDs, aLoads);

    return (true);
}

inline bool apply_rotation_matrix(const Plato::Vector3D& aRotatioAnglesInDegrees, Plato::Vector3D& aVectorToRotate)
{
    const bool tBadNumberDetected = Plato::check_vector3d_values(aVectorToRotate) == false;
    const bool tBadRotationDetected = Plato::check_vector3d_values(aRotatioAnglesInDegrees) == false;
    if(tBadRotationDetected || tBadNumberDetected)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: A NON-FINITE NUMBER WAS DETECTED.\n";
        return (false);
    }

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

inline bool define_distribution(const Plato::srom::Variable & aMyRandomVar, Plato::SromInputs<double> & aInput)
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
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: DISTRIBUTION = " << aMyRandomVar.mStatistics.mDistribution
                 << " IS NOT DEFINED. OPTIONS ARE NORMAL, UNIFORM AND BETA.\n";
        return (false);
    }

    return (true);
}

inline bool check_input_mean(const Plato::srom::Variable & aMyRandomVar)
{
    if(aMyRandomVar.mStatistics.mMean.empty() == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: MEAN IS NOT DEFINED.\n";
        return (false);
    }
    return (true);
}

inline bool check_input_lower_bound(const Plato::srom::Variable & aMyRandomVar)
{
    if(aMyRandomVar.mStatistics.mLowerBound.empty() == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: LOWER BOUND IS NOT DEFINED.\n";
        return (false);
    }
    return (true);
}

inline bool check_input_upper_bound(const Plato::srom::Variable & aMyRandomVar)
{
    if(aMyRandomVar.mStatistics.mUpperBound.empty() == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: UPPER BOUND IS NOT DEFINED.\n";
        return (false);
    }
    return (true);
}

inline bool check_input_standard_deviation(const Plato::srom::Variable & aMyRandomVar)
{
    if(aMyRandomVar.mStatistics.mStandardDeviation.empty() == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: STANDARD DEVIATION IS NOT DEFINED.\n";
        return (false);
    }
    return (true);
}

inline bool check_input_number_samples(const Plato::srom::Variable & aMyRandomVar)
{
    if(aMyRandomVar.mStatistics.mNumSamples.empty() == true)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: NUMBER OF SAMPLES IS NOT DEFINED.\n";
        return (false);
    }
    else if(std::atof(aMyRandomVar.mStatistics.mNumSamples.c_str()) <= 0)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: NUMBER OF SAMPLES SHOULD BE GREATER THAN ZERO. "
                 << "INPUT NUMBER OF SAMPLES = " << std::atof(aMyRandomVar.mStatistics.mNumSamples.c_str()) << ".\n";
        return (false);
    }

    return (true);
}

inline bool check_input_statistics(const Plato::srom::Variable & aMyRandomVar)
{
    std::locale tLocale;
    std::stringstream tOutput;
    for(std::string::size_type tIndex = 0; tIndex < aMyRandomVar.mStatistics.mDistribution.length(); ++tIndex)
    {
        tOutput << std::toupper(aMyRandomVar.mStatistics.mDistribution[tIndex], tLocale);
    }

    if(aMyRandomVar.mStatistics.mDistribution == "beta" || aMyRandomVar.mStatistics.mDistribution == "normal")
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
    else if(aMyRandomVar.mStatistics.mDistribution == "uniform")
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

inline bool define_input_statistics(const Plato::srom::Variable & aMyRandomVar, Plato::SromInputs<double> & aInput)
{
    if(Plato::check_input_statistics(aMyRandomVar) == false)
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

inline bool post_process_sample_probability_pairs(const std::vector<Plato::SromOutputs<double>> aMySromSolution,
                                                  const Plato::srom::Variable & aMyRandomVariable,
                                                  Plato::srom::RandomVariable & aMySromRandomVariable)
{
    if(aMySromSolution.size() <= 0)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: SROM SOLUTION IS EMPTY.\n";
        return (false);
    }

    aMySromRandomVariable.mSampleProbPairs.mSamples.clear();
    aMySromRandomVariable.mSampleProbPairs.mProbabilities.clear();

    aMySromRandomVariable.mType = aMyRandomVariable.mType;
    aMySromRandomVariable.mSubType = aMyRandomVariable.mSubType;

    const size_t tNumSamples = aMySromSolution.size();
    aMySromRandomVariable.mSampleProbPairs.mNumSamples = tNumSamples;
    aMySromRandomVariable.mSampleProbPairs.mSamples.resize(tNumSamples);
    aMySromRandomVariable.mSampleProbPairs.mProbabilities.resize(tNumSamples);

    for(size_t tIndex = 0; tIndex < tNumSamples; tIndex++)
    {
        aMySromRandomVariable.mSampleProbPairs.mSamples[tIndex] = aMySromSolution[tIndex].mSampleValue;
        aMySromRandomVariable.mSampleProbPairs.mProbabilities[tIndex] = aMySromSolution[tIndex].mSampleWeight;
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

inline bool compute_sample_probability_pairs(const std::vector<Plato::srom::Variable> & aSetRandomVariables,
                                             std::vector<Plato::srom::RandomVariable> & aMySampleProbPairs)
{
    if(aSetRandomVariables.size() <= 0)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF RANDOM VARIABLES IS EMPTY.\n";
        return (false);
    }

    aMySampleProbPairs.clear();

    const size_t tNumRandomVariables = aSetRandomVariables.size();
    for(size_t tRandomVarIndex = 0; tRandomVarIndex < tNumRandomVariables; tRandomVarIndex++)
    {
        // pose uncertainty
        Plato::SromInputs<double> tSromInputs;
        const Plato::srom::Variable & tMyRandomVar = aSetRandomVariables[tRandomVarIndex];
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

        Plato::srom::RandomVariable tMySampleProbPairs;
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

inline bool expand_load_sample_probability_pair(const std::vector<Plato::srom::RandomVariable> & aMySampleProbPairs,
                                                Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                                Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                                Plato::srom::SampleProbabilityPairs& aMyZaxisSampleProbPairs)
{
    if(aMySampleProbPairs.size() <= 0)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF RANDOM VARIABLES IS EMPTY.\n";
        return (false);
    }

    for(size_t tRandVarIndex = 0; tRandVarIndex < aMySampleProbPairs.size(); tRandVarIndex++)
    {
        Plato::axis3D::axis3D tMyAxis = Plato::axis3D::axis3D::x;
        Plato::axis3D_stringToEnum(aMySampleProbPairs[tRandVarIndex].mSubType, tMyAxis);
        if(tMyAxis == Plato::axis3D::axis3D::x)
        {
            aMyXaxisSampleProbPairs = aMySampleProbPairs[tRandVarIndex].mSampleProbPairs;
        }
        else if(tMyAxis == Plato::axis3D::axis3D::y)
        {
            aMyYaxisSampleProbPairs = aMySampleProbPairs[tRandVarIndex].mSampleProbPairs;
        }
        else
        {
            aMyZaxisSampleProbPairs = aMySampleProbPairs[tRandVarIndex].mSampleProbPairs;
        }
    }

    return (true);
}

inline bool compute_random_rotations_about_xyz(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                               const Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                               const Plato::srom::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                               std::vector<Plato::srom::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexI = 0; tIndexI < aMyXaxisSampleProbPairs.mSamples.size(); tIndexI++)
    {
        for(size_t tIndexJ = 0; tIndexJ < aMyYaxisSampleProbPairs.mSamples.size(); tIndexJ++)
        {
            for(size_t tIndexK = 0; tIndexK < aMyZaxisSampleProbPairs.mSamples.size(); tIndexK++)
            {
                Plato::srom::RandomRotations tMyRandomRotations;
                tMyRandomRotations.mProbability = aMyXaxisSampleProbPairs.mProbabilities[tIndexI]
                        * aMyYaxisSampleProbPairs.mProbabilities[tIndexJ] * aMyZaxisSampleProbPairs.mProbabilities[tIndexK];
                tMyRandomRotations.mRotations.mX = aMyXaxisSampleProbPairs.mSamples[tIndexI];
                tMyRandomRotations.mRotations.mY = aMyYaxisSampleProbPairs.mSamples[tIndexJ];
                tMyRandomRotations.mRotations.mZ = aMyZaxisSampleProbPairs.mSamples[tIndexK];
                aMyRandomRotations.push_back(tMyRandomRotations);
            }
        }
    }

    return (true);
}

inline bool compute_random_rotations_about_xy(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                              const Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                              std::vector<Plato::srom::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexI = 0; tIndexI < aMyXaxisSampleProbPairs.mSamples.size(); tIndexI++)
    {
        for(size_t tIndexJ = 0; tIndexJ < aMyYaxisSampleProbPairs.mSamples.size(); tIndexJ++)
        {
            Plato::srom::RandomRotations tMyRandomRotations;
            tMyRandomRotations.mProbability = aMyXaxisSampleProbPairs.mProbabilities[tIndexI]
                    * aMyYaxisSampleProbPairs.mProbabilities[tIndexJ];
            tMyRandomRotations.mRotations.mX = aMyXaxisSampleProbPairs.mSamples[tIndexI];
            tMyRandomRotations.mRotations.mY = aMyYaxisSampleProbPairs.mSamples[tIndexJ];
            aMyRandomRotations.push_back(tMyRandomRotations);
        }
    }

    return (true);
}

inline bool compute_random_rotations_about_xz(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                              const Plato::srom::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                              std::vector<Plato::srom::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexI = 0; tIndexI < aMyXaxisSampleProbPairs.mSamples.size(); tIndexI++)
    {
        for(size_t tIndexK = 0; tIndexK < aMyZaxisSampleProbPairs.mSamples.size(); tIndexK++)
        {
            Plato::srom::RandomRotations tMyRandomRotations;
            tMyRandomRotations.mProbability = aMyXaxisSampleProbPairs.mProbabilities[tIndexI]
                    * aMyZaxisSampleProbPairs.mProbabilities[tIndexK];
            tMyRandomRotations.mRotations.mX = aMyXaxisSampleProbPairs.mSamples[tIndexI];
            tMyRandomRotations.mRotations.mZ = aMyZaxisSampleProbPairs.mSamples[tIndexK];
            aMyRandomRotations.push_back(tMyRandomRotations);
        }
    }

    return (true);
}

inline bool compute_random_rotations_about_yz(const Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                              const Plato::srom::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                              std::vector<Plato::srom::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexJ = 0; tIndexJ < aMyYaxisSampleProbPairs.mSamples.size(); tIndexJ++)
    {
        for(size_t tIndexK = 0; tIndexK < aMyZaxisSampleProbPairs.mSamples.size(); tIndexK++)
        {
            Plato::srom::RandomRotations tMyRandomRotations;
            tMyRandomRotations.mProbability = aMyYaxisSampleProbPairs.mProbabilities[tIndexJ]
                    * aMyZaxisSampleProbPairs.mProbabilities[tIndexK];
            tMyRandomRotations.mRotations.mY = aMyYaxisSampleProbPairs.mSamples[tIndexJ];
            tMyRandomRotations.mRotations.mZ = aMyZaxisSampleProbPairs.mSamples[tIndexK];
            aMyRandomRotations.push_back(tMyRandomRotations);
        }
    }

    return (true);
}

inline bool compute_random_rotations_about_x(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                             std::vector<Plato::srom::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexI = 0; tIndexI < aMyXaxisSampleProbPairs.mSamples.size(); tIndexI++)
    {
        Plato::srom::RandomRotations tMyRandomRotations;
        tMyRandomRotations.mProbability = aMyXaxisSampleProbPairs.mProbabilities[tIndexI];
        tMyRandomRotations.mRotations.mX = aMyXaxisSampleProbPairs.mSamples[tIndexI];
        aMyRandomRotations.push_back(tMyRandomRotations);
    }

    return (true);
}

inline bool compute_random_rotations_about_y(const Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                             std::vector<Plato::srom::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexJ = 0; tIndexJ < aMyYaxisSampleProbPairs.mSamples.size(); tIndexJ++)
    {
        Plato::srom::RandomRotations tMyRandomRotations;
        tMyRandomRotations.mProbability = aMyYaxisSampleProbPairs.mProbabilities[tIndexJ];
        tMyRandomRotations.mRotations.mY = aMyYaxisSampleProbPairs.mSamples[tIndexJ];
        aMyRandomRotations.push_back(tMyRandomRotations);
    }

    return (true);
}

inline bool compute_random_rotations_about_z(const Plato::srom::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                             std::vector<Plato::srom::RandomRotations> & aMyRandomRotations)
{
    for(size_t tIndexK = 0; tIndexK < aMyZaxisSampleProbPairs.mSamples.size(); tIndexK++)
    {
        Plato::srom::RandomRotations tMyRandomRotations;
        tMyRandomRotations.mProbability = aMyZaxisSampleProbPairs.mProbabilities[tIndexK];
        tMyRandomRotations.mRotations.mZ = aMyZaxisSampleProbPairs.mSamples[tIndexK];
        aMyRandomRotations.push_back(tMyRandomRotations);
    }

    return (true);
}

inline bool expand_random_rotations(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
                                    const Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
                                    const Plato::srom::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
                                    std::vector<Plato::srom::RandomRotations> & aMyRandomRotations)
{
    aMyRandomRotations.clear();

    const bool tRotateAboutX = !aMyXaxisSampleProbPairs.mSamples.empty();
    const bool tRotateAboutY = !aMyYaxisSampleProbPairs.mSamples.empty();
    const bool tRotateAboutZ = !aMyZaxisSampleProbPairs.mSamples.empty();

    if(tRotateAboutX && tRotateAboutY && tRotateAboutZ)
    {
        Plato::compute_random_rotations_about_xyz(aMyXaxisSampleProbPairs, aMyYaxisSampleProbPairs, aMyZaxisSampleProbPairs, aMyRandomRotations);
    }
    else if(tRotateAboutX && tRotateAboutY && !tRotateAboutZ)
    {
        Plato::compute_random_rotations_about_xy(aMyXaxisSampleProbPairs, aMyYaxisSampleProbPairs, aMyRandomRotations);
    }
    else if(tRotateAboutX && !tRotateAboutY && tRotateAboutZ)
    {
        Plato::compute_random_rotations_about_xz(aMyXaxisSampleProbPairs, aMyZaxisSampleProbPairs, aMyRandomRotations);
    }
    else if(!tRotateAboutX && tRotateAboutY && tRotateAboutZ)
    {
        Plato::compute_random_rotations_about_yz(aMyYaxisSampleProbPairs, aMyZaxisSampleProbPairs, aMyRandomRotations);
    }
    else if(tRotateAboutX && !tRotateAboutY && !tRotateAboutZ)
    {
        Plato::compute_random_rotations_about_x(aMyXaxisSampleProbPairs, aMyRandomRotations);
    }
    else if(!tRotateAboutX && tRotateAboutY && !tRotateAboutZ)
    {
        Plato::compute_random_rotations_about_y(aMyYaxisSampleProbPairs, aMyRandomRotations);
    }
    else if(!tRotateAboutX && !tRotateAboutY && tRotateAboutZ)
    {
        Plato::compute_random_rotations_about_z(aMyZaxisSampleProbPairs, aMyRandomRotations);
    }
    else
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF SAMPLE PROBABILITY PAIRS ARE EMPTY. LOAD VECTOR IS DETERMINISTIC.\n";
        return (false);
    }

    return (true);
}

inline bool check_expand_random_loads_inputs(const Plato::Vector3D & aMyOriginalLoad,
                                             const std::vector<Plato::srom::RandomRotations> & aMyRandomRotations)
{
    if(aMyRandomRotations.empty())
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: VECTOR OF RANDOM ROTATIONS IS EMPTY.\n";
        return (false);
    }
    else if(Plato::check_vector3d_values(aMyOriginalLoad) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: A NON-FINITE NUMBER WAS DETECTED IN VECTOR 3D.\n";
        return (false);
    }

    return (true);
}

inline bool expand_random_loads(const Plato::Vector3D & aMyOriginalLoad,
                                const std::vector<Plato::srom::RandomRotations> & aMyRandomRotations,
                                std::vector<Plato::srom::RandomLoad> & aMyRandomLoads)
{
    if(Plato::check_expand_random_loads_inputs(aMyOriginalLoad, aMyRandomRotations) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: ONE OF THE INPUT ARGUMENTS IS NOT PROPERLY DEFINED.\n";
        return (false);
    }

    const size_t tNumRandomLoads = aMyRandomRotations.size();
    for(size_t tIndex = 0; tIndex < tNumRandomLoads; tIndex++)
    {
        Plato::srom::RandomLoad tMyRandomLoad;
        tMyRandomLoad.mLoadValues = aMyOriginalLoad;
        tMyRandomLoad.mProbability = aMyRandomRotations[tIndex].mProbability;
        if(Plato::apply_rotation_matrix(aMyRandomRotations[tIndex].mRotations, tMyRandomLoad.mLoadValues) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: APPLICATION OF ROTATION MATRIX WAS UNSUCCESSFUL.\n";
            return (false);
        }
        aMyRandomLoads.push_back(tMyRandomLoad);
    }

    return (true);
}

inline bool update_initial_random_load_case(const std::vector<Plato::srom::RandomLoad> & aNewSetRandomLoads,
                                            std::vector<Plato::srom::RandomLoadCase> & aOldRandomLoadCases)
{
    std::vector<Plato::srom::RandomLoadCase> tNewSetRandomLoadCase;

    for(size_t tLoadIndex = 0; tLoadIndex < aNewSetRandomLoads.size(); tLoadIndex++)
    {
        Plato::srom::RandomLoadCase tNewRandomLoadCase;
        const Plato::srom::RandomLoad& tMyNewRandomLoad = aNewSetRandomLoads[tLoadIndex];
        tNewRandomLoadCase.mLoads.push_back(tMyNewRandomLoad);
        tNewRandomLoadCase.mProbability = tMyNewRandomLoad.mProbability;
        tNewSetRandomLoadCase.push_back(tNewRandomLoadCase);
    } // index over new random loads

    aOldRandomLoadCases = tNewSetRandomLoadCase;

    return (true);
}

inline bool update_random_load_cases(const std::vector<Plato::srom::RandomLoad> & aNewSetRandomLoads,
                                     std::vector<Plato::srom::RandomLoadCase> & aOldRandomLoadCases)
{
    std::vector<Plato::srom::RandomLoadCase> tNewSetRandomLoadCase;

    for(size_t tNewLoadIndex = 0; tNewLoadIndex < aNewSetRandomLoads.size(); tNewLoadIndex++)
    {
        const Plato::srom::RandomLoad& tMyNewRandomLoad = aNewSetRandomLoads[tNewLoadIndex];
        std::vector<Plato::srom::RandomLoadCase> tTempRandomLoadCases = aOldRandomLoadCases;
        for(size_t tOldLoadCaseIndex = 0; tOldLoadCaseIndex < aOldRandomLoadCases.size(); tOldLoadCaseIndex++)
        {
            tTempRandomLoadCases[tOldLoadCaseIndex].mLoads.push_back(tMyNewRandomLoad);
            Plato::srom::RandomLoadCase& tMyTempRandomLoadCase = tTempRandomLoadCases[tOldLoadCaseIndex];
            tMyTempRandomLoadCase.mProbability = tMyTempRandomLoadCase.mProbability * tMyNewRandomLoad.mProbability;
            tNewSetRandomLoadCase.push_back(tMyTempRandomLoadCase);
        } // index over old random load cases
    } // index over new random loads

    aOldRandomLoadCases = tNewSetRandomLoadCase;

    return (true);
}

inline bool expand_random_load_cases(const std::vector<Plato::srom::RandomLoad> & aNewSetRandomLoads,
                                     std::vector<Plato::srom::RandomLoadCase> & aOldRandomLoadCases)
{
    if(aNewSetRandomLoads.empty())
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: THE NEW SET OF RANDOM LOADS IS EMPTY.\n";
        return (false);
    } // if statement

    if(aOldRandomLoadCases.empty())
    {
        if(Plato::update_initial_random_load_case(aNewSetRandomLoads, aOldRandomLoadCases) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: FUNCTION FAILED WHILE TRYING TO UPDATE THE FIRST RANDOM LOAD CASE.\n";
            return (false);
        } // if statement
    } // if-else statement
    else
    {
        if(Plato::update_random_load_cases(aNewSetRandomLoads, aOldRandomLoadCases) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: FUNCTION FAILED WHILE TRYING TO ADD A NEW RANDOM LOAD CASE.\n";
            return (false);
        } // if statement
    } // if-else statement

    return (true);
}

inline bool expand_random_and_deterministic_loads(const std::vector<Plato::srom::Load>& aLoads,
                                                  std::vector<Plato::srom::Load>& aRandomLoads,
                                                  std::vector<Plato::srom::Load>& aDeterministicLoads)
{
    if(aLoads.empty())
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF LOADS IS EMPTY.\n";
        return (false);
    } // if statement

    const size_t tNumLoads = aLoads.size();
    for(size_t tLoadIndex = 0; tLoadIndex < tNumLoads; tLoadIndex++)
    {
        const Plato::srom::Load& tLoad = aLoads[tLoadIndex];
        if(tLoad.mRandomVars.empty())
        {
            // deterministic load
            aDeterministicLoads.push_back(aLoads[tLoadIndex]);
        }
        else
        {
            // random load
            aRandomLoads.push_back(aLoads[tLoadIndex]);
        } // if-else statement
    }// for-loop

    return (true);
}

inline bool check_load_parameters(const Plato::srom::Load& aLoad)
{
    if(std::isfinite(aLoad.mAppID) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: APPLICATION IDENTIFIER IS NOT A FINITE NUMBER.\n";
        return (false);
    }

    if(aLoad.mAppType.empty())
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: APPLICATION TYPE IS NOT DEFINE.\n";
        return (false);
    }

    if(aLoad.mLoadType.empty())
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: LOAD TYPE IS NOT DEFINE.\n";
        return (false);
    }

    if(aLoad.mValues.empty())
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: LOAD VALUES/COMPONENTS ARE NOT DEFINE, VALUES VECTOR IS EMPTY.\n";
        return (false);
    }

    return (true);
}

inline bool set_load_components(const std::vector<std::string> & aInput, Plato::Vector3D & aOutput)
{
    if(aInput.empty())
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT LOAD VECTOR IS EMPTY.\n";
        return (false);
    }

    if(aInput.size() != static_cast<size_t>(3))
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT DIMENSIONS = " << aInput.size() << ". CURRENTLY, ONLY 3-DIM PROBLEMS ARE SUPPORTED.\n";
        return (false);
    }

    aOutput.mX = std::atof(aInput[0].c_str());
    aOutput.mY = std::atof(aInput[1].c_str());
    aOutput.mZ = std::atof(aInput[2].c_str());

    return (true);
}

inline bool set_random_load_parameters(const Plato::srom::Load & aOriginalLoad, std::vector<Plato::srom::RandomLoad> & aSetRandomLoads)
{
    if(Plato::check_load_parameters(aOriginalLoad) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: ONE OR MORE LOAD PARAMETERS ARE NOT DEFINED.\n";
        return (false);
    }

    for(size_t tLoadIndex = 0; tLoadIndex < aSetRandomLoads.size(); tLoadIndex++)
    {
        aSetRandomLoads[tLoadIndex].mAppID = aOriginalLoad.mAppID;
        aSetRandomLoads[tLoadIndex].mAppType = aOriginalLoad.mAppType;
        aSetRandomLoads[tLoadIndex].mLoadType = aOriginalLoad.mLoadType;
    }

    return (true);
}

inline bool generate_set_random_rotations(const std::vector<Plato::srom::RandomVariable> & aMySampleProbPairs,
                                          std::vector<Plato::srom::RandomRotations> & aMySetRandomRotation)
{
    Plato::srom::SampleProbabilityPairs tXaxisSampleProbPairs, tYaxisSampleProbPairs, tZaxisSampleProbPairs;
    if(Plato::expand_load_sample_probability_pair(aMySampleProbPairs,  tXaxisSampleProbPairs, tYaxisSampleProbPairs, tZaxisSampleProbPairs) == false)
    {
        std::cout << "\nFILE: " << __FILE__
                  << "\nFUNCTION: " << __PRETTY_FUNCTION__
                  << "\nLINE:" << __LINE__
                  << "\nMESSAGE: FAILED TO EXPAND SAMPLE-PROBABILITY PAIRS.\n";
        return (false);
    }

    Plato::expand_random_rotations(tXaxisSampleProbPairs, tYaxisSampleProbPairs, tZaxisSampleProbPairs, aMySetRandomRotation);

    return (true);
}

inline bool generate_set_random_loads(const Plato::srom::Load & aOriginalLoad,
                                      const std::vector<Plato::srom::RandomRotations> & aSetRandomRotations,
                                      std::vector<Plato::srom::RandomLoad> & aSetRandomLoads)
{
    Plato::Vector3D tMyOriginalLoadComponents;
    if(Plato::set_load_components(aOriginalLoad.mValues, tMyOriginalLoadComponents) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: FAILED TO SET ARRAY OF LOAD COMPONENTS.\n";
        return (false);
    }

    if(Plato::expand_random_loads(tMyOriginalLoadComponents, aSetRandomRotations, aSetRandomLoads) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: FAILED TO EXPAND SET OF RANDOM LOADS.\n";
        return (false);
    }

    if(Plato::set_random_load_parameters(aOriginalLoad, aSetRandomLoads) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: FAILED TO SET LOAD TYPE, APPLICATION TYPE AND APPLICATION IDENTIFIER.\n";
        return (false);
    }

    return (true);
}

inline void generate_load_case_identifiers(std::vector<Plato::srom::RandomLoadCase> & aSetLoadCases)
{
    Plato::UniqueCounter tLoadCounter, tLoadCaseCounter;
    tLoadCounter.mark(0); tLoadCaseCounter.mark(0);

    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < aSetLoadCases.size(); tLoadCaseIndex++)
    {
        Plato::srom::RandomLoadCase& tMyLoadCase = aSetLoadCases[tLoadCaseIndex];
        tMyLoadCase.mLoadCaseID = tLoadCaseCounter.assignNextUnique();
        for(size_t tLoadIndex = 0; tLoadIndex < tMyLoadCase.mLoads.size(); tLoadIndex++)
        {
            tMyLoadCase.mLoads[tLoadIndex].mLoadID = tLoadCounter.assignNextUnique();
        }
    }
}

inline bool check_deterministic_loads(const std::vector<Plato::srom::Load>& aDeterministicLoads)
{
    for(size_t tIndex = 0; tIndex < aDeterministicLoads.size(); tIndex++)
    {
        if(Plato::check_load_parameters(aDeterministicLoads[tIndex]) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: UNDEFINED PARAMETER FOR DETERMINISTIC LOAD #" << tIndex << ".\n";
            return (false);
        }
    }

    return (true);
}

inline void append_deterministic_loads(const std::vector<Plato::srom::Load>& aDeterministicLoads,
                                       std::vector<Plato::srom::RandomLoadCase> & aSetLoadCases)
{
    if(aDeterministicLoads.empty())
    {
        return; // return, there are no deterministic loads to append
    }

    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < aSetLoadCases.size(); tLoadCaseIndex++)
    {
        Plato::srom::RandomLoadCase& tRandomLoadCase = aSetLoadCases[tLoadCaseIndex];
        for(size_t tLoadIndex = 0; tLoadIndex < aDeterministicLoads.size(); tLoadIndex++)
        {
            Plato::srom::RandomLoad tLoad;
            tLoad.mProbability = 1.0;
            tLoad.mAppID = aDeterministicLoads[tLoadIndex].mAppID;
            tLoad.mAppType = aDeterministicLoads[tLoadIndex].mAppType;
            tLoad.mLoadType = aDeterministicLoads[tLoadIndex].mLoadType;
            Plato::set_load_components(aDeterministicLoads[tLoadIndex].mValues, tLoad.mLoadValues);
            tRandomLoadCase.mLoads.push_back(tLoad);
        }
    }
}

inline bool generate_output_random_load_cases(const std::vector<Plato::srom::Load>& aDeterministicLoads,
                                              std::vector<Plato::srom::RandomLoadCase> & aSetRandomLoadCases)
{
    if(aSetRandomLoadCases.empty())
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: SET OF RANDOM LOAD CASES IS EMPTY.\n";
        return (false);
    }

    if(Plato::check_deterministic_loads(aDeterministicLoads) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: AN ERROR WAS DETECTED WITH THE INPUT SET OF DETERMINISTIC LOADS.\n";
        return (false);
    }

    Plato::append_deterministic_loads(aDeterministicLoads, aSetRandomLoadCases);
    Plato::generate_load_case_identifiers(aSetRandomLoadCases);

    return (true);
}

inline bool generate_load_sroms(const Plato::srom::InputMetaData & aInput, Plato::srom::OutputMetaData & aOutput)
{
    aOutput.mLoadCases.clear();
    std::vector<Plato::srom::Load> tRandomLoads, tDeterministicLoads;
    if(Plato::expand_random_and_deterministic_loads(aInput.mLoads, tRandomLoads, tDeterministicLoads) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: FAILED TO GENERATE THE SETS OF RANDOM AND DETERMINISTIC LOADS.\n";
        return (false);
    }

    for(size_t tLoadIndex = 0; tLoadIndex < tRandomLoads.size(); tLoadIndex++)
    {
        std::vector<Plato::srom::RandomVariable> tMySampleProbPairs;
        if(Plato::compute_sample_probability_pairs(tRandomLoads[tLoadIndex].mRandomVars, tMySampleProbPairs) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: FAILED TO COMPUTE THE SAMPLE-PROBABILITY PAIRS FOR LOAD #" << tLoadIndex << ".\n";
            return (false);
        }

        std::vector<Plato::srom::RandomRotations> tMySetRandomRotation;
        if(Plato::generate_set_random_rotations(tMySampleProbPairs, tMySetRandomRotation) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: FAILED TO GENERATE SET OF RANDOM ROTATIONS FOR LOAD #" << tLoadIndex << ".\n";
            return (false);
        }

        std::vector<Plato::srom::RandomLoad> tMySetRandomLoads;
        if(Plato::generate_set_random_loads(tRandomLoads[tLoadIndex], tMySetRandomRotation, tMySetRandomLoads) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: FAILED TO GENERATE SET OF RANDOM LOADS FOR LOAD #" << tLoadIndex << ".\n";
            return (false);
        }

        if(Plato::expand_random_load_cases(tMySetRandomLoads, aOutput.mLoadCases) == false)
        {
            std::cout<< "\nFILE: " << __FILE__
                     << "\nFUNCTION: " << __PRETTY_FUNCTION__
                     << "\nLINE:" << __LINE__
                     << "\nMESSAGE: FAILED TO EXPAND RANDOM LOAD CASES FOR LOAD #" << tLoadIndex << ".\n";
            return (false);
        }
    }

    if(Plato::generate_output_random_load_cases(tDeterministicLoads, aOutput.mLoadCases) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: FAILED TO GENERATE SET OF OUTPUT RANDOM LOAD CASES.\n";
        return (false);
    }

    return(true);
}

}

namespace PlatoUncertainLoadGeneratorXMLTest
{

TEST(PlatoTest, check_input_mean)
{
    Plato::srom::Variable tMyRandomVar;
    ASSERT_FALSE(Plato::check_input_mean(tMyRandomVar));

    tMyRandomVar.mStatistics.mMean = "5";
    ASSERT_TRUE(Plato::check_input_mean(tMyRandomVar));
}

TEST(PlatoTest, check_input_lower_bound)
{
    Plato::srom::Variable tMyRandomVar;
    ASSERT_FALSE(Plato::check_input_lower_bound(tMyRandomVar));

    tMyRandomVar.mStatistics.mLowerBound = "5";
    ASSERT_TRUE(Plato::check_input_lower_bound(tMyRandomVar));
}

TEST(PlatoTest, check_input_upper_bound)
{
    Plato::srom::Variable tMyRandomVar;
    ASSERT_FALSE(Plato::check_input_upper_bound(tMyRandomVar));

    tMyRandomVar.mStatistics.mUpperBound = "5";
    ASSERT_TRUE(Plato::check_input_upper_bound(tMyRandomVar));
}

TEST(PlatoTest, check_input_standard_deviation)
{
    Plato::srom::Variable tMyRandomVar;
    ASSERT_FALSE(Plato::check_input_standard_deviation(tMyRandomVar));

    tMyRandomVar.mStatistics.mStandardDeviation = "5";
    ASSERT_TRUE(Plato::check_input_standard_deviation(tMyRandomVar));
}

TEST(PlatoTest, check_input_number_samples)
{
    Plato::srom::Variable tMyRandomVar;
    ASSERT_FALSE(Plato::check_input_number_samples(tMyRandomVar));

    tMyRandomVar.mStatistics.mNumSamples = "-1";
    ASSERT_FALSE(Plato::check_input_number_samples(tMyRandomVar));

    tMyRandomVar.mStatistics.mNumSamples = "0";
    ASSERT_FALSE(Plato::check_input_number_samples(tMyRandomVar));

    tMyRandomVar.mStatistics.mNumSamples = "2";
    ASSERT_TRUE(Plato::check_input_number_samples(tMyRandomVar));
}

TEST(PlatoTest, check_input_statistics_uniform)
{
    // DEFINE INPUTS
    Plato::srom::Variable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mStatistics.mDistribution = "uniform";

    // TEST UNDEFINED UPPER BOUND
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED LOWER BOUND
    tMyRandomVar.mStatistics.mLowerBound = "";
    tMyRandomVar.mStatistics.mUpperBound = "135";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED NUMBER OF SAMPLES
    tMyRandomVar.mStatistics.mNumSamples = "";
    tMyRandomVar.mStatistics.mLowerBound = "65";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST NUMBER OF SAMPLES = 0
    tMyRandomVar.mStatistics.mNumSamples = "0";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST NEGATIVE NUMBER OF SAMPLES
    tMyRandomVar.mStatistics.mNumSamples = "-1";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST SUCCESS
    tMyRandomVar.mStatistics.mNumSamples = "4";
    ASSERT_TRUE(Plato::check_input_statistics(tMyRandomVar));
}

TEST(PlatoTest, check_input_statistics_normal)
{
    // DEFINE INPUTS
    Plato::srom::Variable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mStatistics.mNumSamples = "4";
    tMyRandomVar.mStatistics.mLowerBound = "65";
    tMyRandomVar.mStatistics.mUpperBound = "95";
    tMyRandomVar.mStatistics.mStandardDeviation = "5";
    tMyRandomVar.mStatistics.mDistribution = "normal";

    // TEST UNDEFINED MEAN
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED UPPER BOUND
    tMyRandomVar.mStatistics.mMean = "80";
    tMyRandomVar.mStatistics.mUpperBound = "";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED LOWER BOUND
    tMyRandomVar.mStatistics.mLowerBound = "";
    tMyRandomVar.mStatistics.mUpperBound = "95";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED NUMBER OF SAMPLES
    tMyRandomVar.mStatistics.mNumSamples = "";
    tMyRandomVar.mStatistics.mLowerBound = "65";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST NUMBER OF SAMPLES = 0
    tMyRandomVar.mStatistics.mNumSamples = "0";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST NEGATIVE NUMBER OF SAMPLES
    tMyRandomVar.mStatistics.mNumSamples = "-1";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED STANDARD DEVIATION
    tMyRandomVar.mStatistics.mNumSamples = "4";
    tMyRandomVar.mStatistics.mNumSamples = "4";
    tMyRandomVar.mStatistics.mStandardDeviation = "";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST SUCCESS
    tMyRandomVar.mStatistics.mStandardDeviation = "5";
    ASSERT_TRUE(Plato::check_input_statistics(tMyRandomVar));
}

TEST(PlatoTest, check_input_statistics_beta)
{
    // DEFINE INPUTS
    Plato::srom::Variable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mStatistics.mNumSamples = "4";
    tMyRandomVar.mStatistics.mLowerBound = "65";
    tMyRandomVar.mStatistics.mUpperBound = "95";
    tMyRandomVar.mStatistics.mStandardDeviation = "5";
    tMyRandomVar.mStatistics.mDistribution = "beta";

    // TEST UNDEFINED MEAN
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED UPPER BOUND
    tMyRandomVar.mStatistics.mMean = "80";
    tMyRandomVar.mStatistics.mUpperBound = "";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED LOWER BOUND
    tMyRandomVar.mStatistics.mLowerBound = "";
    tMyRandomVar.mStatistics.mUpperBound = "95";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED NUMBER OF SAMPLES
    tMyRandomVar.mStatistics.mNumSamples = "";
    tMyRandomVar.mStatistics.mLowerBound = "65";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST NUMBER OF SAMPLES = 0
    tMyRandomVar.mStatistics.mNumSamples = "0";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST NEGATIVE NUMBER OF SAMPLES
    tMyRandomVar.mStatistics.mNumSamples = "-1";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST UNDEFINED STANDARD DEVIATION
    tMyRandomVar.mStatistics.mNumSamples = "4";
    tMyRandomVar.mStatistics.mStandardDeviation = "";
    ASSERT_FALSE(Plato::check_input_statistics(tMyRandomVar));

    // TEST SUCCESS
    tMyRandomVar.mStatistics.mStandardDeviation = "5";
    ASSERT_TRUE(Plato::check_input_statistics(tMyRandomVar));
}

TEST(PlatoTest, define_input_statistics)
{
    // DEFINE INPUTS
    Plato::srom::Variable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mStatistics.mNumSamples = "4";
    tMyRandomVar.mStatistics.mLowerBound = "65";
    tMyRandomVar.mStatistics.mUpperBound = "95";
    tMyRandomVar.mStatistics.mStandardDeviation = "5";
    tMyRandomVar.mStatistics.mDistribution = "beta";

    // TEST UNDEFINED PARAMETER
    Plato::SromInputs<double> tSromInputs;
    ASSERT_FALSE(Plato::define_input_statistics(tMyRandomVar, tSromInputs));

    // TEST SUCCESS
    tMyRandomVar.mStatistics.mMean = "80";
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
    Plato::srom::Variable tMyRandomVar;
    Plato::SromInputs<double> tSromInputs;

    tMyRandomVar.mStatistics.mDistribution = "normal";
    ASSERT_TRUE(Plato::define_distribution(tMyRandomVar, tSromInputs));
    ASSERT_EQ(Plato::DistrubtionName::normal, tSromInputs.mDistribution);

    tMyRandomVar.mStatistics.mDistribution = "beta";
    ASSERT_TRUE(Plato::define_distribution(tMyRandomVar, tSromInputs));
    ASSERT_EQ(Plato::DistrubtionName::beta, tSromInputs.mDistribution);

    tMyRandomVar.mStatistics.mDistribution = "uniform";
    ASSERT_TRUE(Plato::define_distribution(tMyRandomVar, tSromInputs));
    ASSERT_EQ(Plato::DistrubtionName::uniform, tSromInputs.mDistribution);

    tMyRandomVar.mStatistics.mDistribution = "lognormal";
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
    Plato::srom::Variable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mStatistics.mDistribution = "beta";
    tMyRandomVar.mStatistics.mMean = "85";
    tMyRandomVar.mStatistics.mUpperBound = "65";
    tMyRandomVar.mStatistics.mLowerBound = "135";
    tMyRandomVar.mStatistics.mStandardDeviation = "15";
    tMyRandomVar.mStatistics.mNumSamples = "3";
    std::vector<Plato::srom::Variable> tRandomVarsSet;
    tRandomVarsSet.push_back(tMyRandomVar);

    // SOLVE SROM PROBLEM
    std::vector<Plato::srom::RandomVariable> tMySampleProbPairs;
    Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs);

    // TEST RESULTS
    ASSERT_EQ(1u, tMySampleProbPairs.size());
    ASSERT_STREQ("x", tMySampleProbPairs[0].mSubType.c_str());
    ASSERT_STREQ("random rotation", tMySampleProbPairs[0].mType.c_str());
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mNumSamples);
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mSamples.size());
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mProbabilities.size());

    // TEST RESULTS
    double tSum = 0;
    double tTolerance = 1e-4;
    const Plato::srom::SampleProbabilityPairs& tSampleProbabilityPairs = tMySampleProbPairs[0].mSampleProbPairs;
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
    std::vector<Plato::srom::Variable> tRandomVarsSet;

    // DEFINE INPUTS
    Plato::srom::Variable tMyRandomVarOne;
    tMyRandomVarOne.mType = "random rotation";
    tMyRandomVarOne.mSubType = "x";
    tMyRandomVarOne.mStatistics.mDistribution = "beta";
    tMyRandomVarOne.mStatistics.mMean = "85";
    tMyRandomVarOne.mStatistics.mLowerBound = "65";
    tMyRandomVarOne.mStatistics.mUpperBound = "135";
    tMyRandomVarOne.mStatistics.mStandardDeviation = "15";
    tMyRandomVarOne.mStatistics.mNumSamples = "3";
    tRandomVarsSet.push_back(tMyRandomVarOne);

    Plato::srom::Variable tMyRandomVarTwo;
    tMyRandomVarTwo.mType = "random rotation";
    tMyRandomVarTwo.mSubType = "y";
    tMyRandomVarTwo.mStatistics.mNumSamples = "4";
    tMyRandomVarTwo.mStatistics.mDistribution = "beta";
    tMyRandomVarTwo.mStatistics.mMean = "70";
    tMyRandomVarTwo.mStatistics.mLowerBound = "50";
    tMyRandomVarTwo.mStatistics.mUpperBound = "120";
    tMyRandomVarTwo.mStatistics.mStandardDeviation = "15";
    tRandomVarsSet.push_back(tMyRandomVarTwo);

    // SOLVE SROM PROBLEM
    std::vector<Plato::srom::RandomVariable> tMySampleProbPairs;
    Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs);

    // TEST RESULTS - RANDOM VARIABLE ONE
    ASSERT_EQ(2u, tMySampleProbPairs.size());
    ASSERT_STREQ("x", tMySampleProbPairs[0].mSubType.c_str());
    ASSERT_STREQ("random rotation", tMySampleProbPairs[0].mType.c_str());
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mNumSamples);
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mSamples.size());
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mProbabilities.size());

    double tSum = 0;
    double tTolerance = 1e-4;
    const Plato::srom::SampleProbabilityPairs& tSampleProbabilityPairsOne = tMySampleProbPairs[0].mSampleProbPairs;
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
    ASSERT_EQ(4u, tMySampleProbPairs[1].mSampleProbPairs.mNumSamples);
    ASSERT_EQ(4u, tMySampleProbPairs[1].mSampleProbPairs.mSamples.size());
    ASSERT_EQ(4u, tMySampleProbPairs[1].mSampleProbPairs.mProbabilities.size());

    tSum = 0;
    tTolerance = 1e-4;
    const Plato::srom::SampleProbabilityPairs& tSampleProbabilityPairsTwo = tMySampleProbPairs[1].mSampleProbPairs;
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
    Plato::srom::Variable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mStatistics.mDistribution = "";
    tMyRandomVar.mStatistics.mMean = "85";
    tMyRandomVar.mStatistics.mUpperBound = "135";
    tMyRandomVar.mStatistics.mLowerBound = "65";
    tMyRandomVar.mStatistics.mStandardDeviation = "15";
    tMyRandomVar.mStatistics.mNumSamples = "3";
    std::vector<Plato::srom::Variable> tRandomVarsSet;
    tRandomVarsSet.push_back(tMyRandomVar);

    // SOLVE SROM PROBLEM
    std::vector<Plato::srom::RandomVariable> tMySampleProbPairs;
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));
}

TEST(PlatoTest, compute_sample_probability_pairs_error_empty_random_var_set)
{
    // SOLVE SROM PROBLEM
    std::vector<Plato::srom::Variable> tRandomVarsSet;
    std::vector<Plato::srom::RandomVariable> tMySampleProbPairs;
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));
}

TEST(PlatoTest, compute_sample_probability_pairs_error_undefined_input_statistics_uniform)
{
    // DEFINE INPUTS
    Plato::srom::Variable tMyRandomVar;
    tMyRandomVar.mType = "random rotation";
    tMyRandomVar.mSubType = "x";
    tMyRandomVar.mStatistics.mDistribution = "uniform";
    tMyRandomVar.mStatistics.mLowerBound = "65";
    tMyRandomVar.mStatistics.mUpperBound = "";
    tMyRandomVar.mStatistics.mNumSamples = "3";
    std::vector<Plato::srom::Variable> tRandomVarsSet;
    tRandomVarsSet.push_back(tMyRandomVar);

    // TEST UNDEFINED UPPER BOUND
    std::vector<Plato::srom::RandomVariable> tMySampleProbPairs;
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));

    // TEST UNDEFINED LOWER BOUND
    tRandomVarsSet[0].mStatistics.mLowerBound = "";
    tRandomVarsSet[0].mStatistics.mUpperBound = "135";
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));

    // TEST UNDEFINED NUMBER OF SAMPLES
    tRandomVarsSet[0].mStatistics.mNumSamples = "";
    tRandomVarsSet[0].mStatistics.mLowerBound = "65";
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));

    // TEST NUMBER OF SAMPLES = 0
    tRandomVarsSet[0].mStatistics.mNumSamples = "0";
    tRandomVarsSet[0].mStatistics.mLowerBound = "65";
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));

    // TEST NEGATIVE NUMBER OF SAMPLES
    tRandomVarsSet[0].mStatistics.mNumSamples = "-1";
    tRandomVarsSet[0].mStatistics.mLowerBound = "65";
    ASSERT_FALSE(Plato::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));
}

TEST(PlatoTest, post_process_random_load_error)
{
    // ERROR: ZERO INPUTS PROVIDED
    std::vector<Plato::srom::RandomVariable> tSromRandomVariableSet;
    Plato::srom::SampleProbabilityPairs tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs;
    ASSERT_FALSE(Plato::expand_load_sample_probability_pair(tSromRandomVariableSet, tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs));
}

TEST(PlatoTest, post_process_random_load_OneRandomRotation)
{
    Plato::srom::RandomVariable tRandomLoadX;
    tRandomLoadX.mType = "random rotation";
    tRandomLoadX.mSubType = "x";
    tRandomLoadX.mSampleProbPairs.mNumSamples = 4;
    tRandomLoadX.mSampleProbPairs.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tRandomLoadX.mSampleProbPairs.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    std::vector<Plato::srom::RandomVariable> tSromRandomVariableSet;
    tSromRandomVariableSet.push_back(tRandomLoadX);

    Plato::srom::SampleProbabilityPairs tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs;
    ASSERT_TRUE(Plato::expand_load_sample_probability_pair(tSromRandomVariableSet, tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs));

    // TEST RESULTS
    ASSERT_TRUE(tMyYaxisSampleProbPairs.mSamples.empty());
    ASSERT_TRUE(tMyZaxisSampleProbPairs.mSamples.empty());
    ASSERT_FALSE(tMyXaxisSampleProbPairs.mSamples.empty());

    ASSERT_EQ(4u, tMyXaxisSampleProbPairs.mNumSamples);

    const double tTolerance = 1e-4;
    for(size_t tIndex = 0; tIndex < tMyXaxisSampleProbPairs.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadX.mSampleProbPairs.mSamples[tIndex], tMyXaxisSampleProbPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadX.mSampleProbPairs.mProbabilities[tIndex], tMyXaxisSampleProbPairs.mProbabilities[tIndex], tTolerance);
    }
}

TEST(PlatoTest, post_process_random_load_TwoRandomRotations)
{
    Plato::srom::RandomVariable tRandomLoadX;
    tRandomLoadX.mType = "random rotation";
    tRandomLoadX.mSubType = "x";
    tRandomLoadX.mSampleProbPairs.mNumSamples = 4;
    tRandomLoadX.mSampleProbPairs.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tRandomLoadX.mSampleProbPairs.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    Plato::srom::RandomVariable tRandomLoadY;
    tRandomLoadY.mType = "random rotation";
    tRandomLoadY.mSubType = "y";
    tRandomLoadY.mSampleProbPairs.mNumSamples = 3;
    tRandomLoadY.mSampleProbPairs.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tRandomLoadY.mSampleProbPairs.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    std::vector<Plato::srom::RandomVariable> tSromRandomVariableSet;
    tSromRandomVariableSet.push_back(tRandomLoadX);
    tSromRandomVariableSet.push_back(tRandomLoadY);

    Plato::srom::SampleProbabilityPairs tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs;
    ASSERT_TRUE(Plato::expand_load_sample_probability_pair(tSromRandomVariableSet, tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs));

    // TEST RESULTS
    ASSERT_TRUE(tMyZaxisSampleProbPairs.mSamples.empty());
    ASSERT_FALSE(tMyYaxisSampleProbPairs.mSamples.empty());
    ASSERT_FALSE(tMyXaxisSampleProbPairs.mSamples.empty());

    // TEST RESULTS FOR ROTATION X SAMPLE-PROBABILITY PAIRS
    const double tTolerance = 1e-4;

    ASSERT_EQ(tRandomLoadX.mSampleProbPairs.mNumSamples, tMyXaxisSampleProbPairs.mNumSamples);
    for(size_t tIndex = 0; tIndex < tMyXaxisSampleProbPairs.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadX.mSampleProbPairs.mSamples[tIndex], tMyXaxisSampleProbPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadX.mSampleProbPairs.mProbabilities[tIndex], tMyXaxisSampleProbPairs.mProbabilities[tIndex], tTolerance);
    }

    // TEST RESULTS FOR ROTATION Y SAMPLE-PROBABILITY PAIRS
    ASSERT_EQ(tRandomLoadY.mSampleProbPairs.mNumSamples, tMyYaxisSampleProbPairs.mNumSamples);
    for(size_t tIndex = 0; tIndex < tMyYaxisSampleProbPairs.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadY.mSampleProbPairs.mSamples[tIndex], tMyYaxisSampleProbPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadY.mSampleProbPairs.mProbabilities[tIndex], tMyYaxisSampleProbPairs.mProbabilities[tIndex], tTolerance);
    }
}

TEST(PlatoTest, post_process_random_load_ThreeRandomRotations)
{
    Plato::srom::RandomVariable tRandomLoadX;
    tRandomLoadX.mType = "random rotation";
    tRandomLoadX.mSubType = "x";
    tRandomLoadX.mSampleProbPairs.mNumSamples = 4;
    tRandomLoadX.mSampleProbPairs.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tRandomLoadX.mSampleProbPairs.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    Plato::srom::RandomVariable tRandomLoadY;
    tRandomLoadY.mType = "random rotation";
    tRandomLoadY.mSubType = "y";
    tRandomLoadY.mSampleProbPairs.mNumSamples = 3;
    tRandomLoadY.mSampleProbPairs.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tRandomLoadY.mSampleProbPairs.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    Plato::srom::RandomVariable tRandomLoadZ;
    tRandomLoadZ.mType = "random rotation";
    tRandomLoadZ.mSubType = "z";
    tRandomLoadZ.mSampleProbPairs.mNumSamples = 4;
    tRandomLoadZ.mSampleProbPairs.mSamples = {10, 13.333333333333, 16.666666666667, 20.0};
    tRandomLoadZ.mSampleProbPairs.mProbabilities = {0.25, 0.25, 0.25, 0.25};

    std::vector<Plato::srom::RandomVariable> tSromRandomVariableSet;
    tSromRandomVariableSet.push_back(tRandomLoadX);
    tSromRandomVariableSet.push_back(tRandomLoadY);
    tSromRandomVariableSet.push_back(tRandomLoadZ);

    Plato::srom::SampleProbabilityPairs tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs;
    ASSERT_TRUE(Plato::expand_load_sample_probability_pair(tSromRandomVariableSet, tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs));

    // TEST RESULTS
    ASSERT_FALSE(tMyZaxisSampleProbPairs.mSamples.empty());
    ASSERT_FALSE(tMyYaxisSampleProbPairs.mSamples.empty());
    ASSERT_FALSE(tMyXaxisSampleProbPairs.mSamples.empty());

    // TEST RESULTS FOR ROTATION X SAMPLE-PROBABILITY PAIRS
    const double tTolerance = 1e-4;

    ASSERT_EQ(tRandomLoadX.mSampleProbPairs.mNumSamples, tMyXaxisSampleProbPairs.mNumSamples);
    for(size_t tIndex = 0; tIndex < tMyXaxisSampleProbPairs.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadX.mSampleProbPairs.mSamples[tIndex], tMyXaxisSampleProbPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadX.mSampleProbPairs.mProbabilities[tIndex], tMyXaxisSampleProbPairs.mProbabilities[tIndex], tTolerance);
    }

    // TEST RESULTS FOR ROTATION Y SAMPLE-PROBABILITY PAIRS
    ASSERT_EQ(tRandomLoadY.mSampleProbPairs.mNumSamples, tMyYaxisSampleProbPairs.mNumSamples);
    for(size_t tIndex = 0; tIndex < tMyYaxisSampleProbPairs.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadY.mSampleProbPairs.mSamples[tIndex], tMyYaxisSampleProbPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadY.mSampleProbPairs.mProbabilities[tIndex], tMyYaxisSampleProbPairs.mProbabilities[tIndex], tTolerance);
    }

    // TEST RESULTS FOR ROTATION Z SAMPLE-PROBABILITY PAIRS
    ASSERT_EQ(tRandomLoadZ.mSampleProbPairs.mNumSamples, tMyZaxisSampleProbPairs.mNumSamples);
    for(size_t tIndex = 0; tIndex < tMyZaxisSampleProbPairs.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadZ.mSampleProbPairs.mSamples[tIndex], tMyZaxisSampleProbPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadZ.mSampleProbPairs.mProbabilities[tIndex], tMyZaxisSampleProbPairs.mProbabilities[tIndex], tTolerance);
    }
}

TEST(PlatoTest, expand_random_rotations_about_xyz)
{
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    tSampleProbPairSetY.mNumSamples = 3;
    tSampleProbPairSetY.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetY.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 4;
    tSampleProbPairSetZ.mProbabilities = {0.25, 0.25, 0.25, 0.25};
    tSampleProbPairSetZ.mSamples = {10, 13.333333333333, 16.666666666667, 20.0};

    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST NUMBER OF ROTATION VECTORS
    const size_t tNumRotationVectors = tSampleProbPairSetX.mNumSamples * tSampleProbPairSetY.mNumSamples * tSampleProbPairSetZ.mNumSamples;
    ASSERT_EQ(tNumRotationVectors, tRandomRotationSet.size()); // expects 48 rotation vectors
    std::vector<std::vector<double>> tSetGoldRotations =
        { {62.92995363352, 79.56461506624, 10}, {62.92995363352, 79.56461506624, 13.333333333333}, {62.92995363352, 79.56461506624, 16.666666666667}, {62.92995363352, 79.56461506624, 20},
          {62.92995363352,  95.1780010696, 10}, {62.92995363352,  95.1780010696, 13.333333333333}, {62.92995363352,  95.1780010696, 16.666666666667}, {62.92995363352,  95.1780010696, 20},
          {62.92995363352, 104.3742043151, 10}, {62.92995363352, 104.3742043151, 13.333333333333}, {62.92995363352, 104.3742043151, 16.666666666667}, {62.92995363352, 104.3742043151, 20},
          {69.67128118964, 79.56461506624, 10}, {69.67128118964, 79.56461506624, 13.333333333333}, {69.67128118964, 79.56461506624, 16.666666666667}, {69.67128118964, 79.56461506624, 20},
          {69.67128118964,  95.1780010696, 10}, {69.67128118964,  95.1780010696, 13.333333333333}, {69.67128118964,  95.1780010696, 16.666666666667}, {69.67128118964,  95.1780010696, 20},
          {69.67128118964, 104.3742043151, 10}, {69.67128118964, 104.3742043151, 13.333333333333}, {69.67128118964, 104.3742043151, 16.666666666667}, {69.67128118964, 104.3742043151, 20},
          {66.03455388567, 79.56461506624, 10}, {66.03455388567, 79.56461506624, 13.333333333333}, {66.03455388567, 79.56461506624, 16.666666666667}, {66.03455388567, 79.56461506624, 20},
          {66.03455388567,  95.1780010696, 10}, {66.03455388567,  95.1780010696, 13.333333333333}, {66.03455388567,  95.1780010696, 16.666666666667}, {66.03455388567,  95.1780010696, 20},
          {66.03455388567, 104.3742043151, 10}, {66.03455388567, 104.3742043151, 13.333333333333}, {66.03455388567, 104.3742043151, 16.666666666667}, {66.03455388567, 104.3742043151, 20},
          { 96.2527627689, 79.56461506624, 10}, { 96.2527627689, 79.56461506624, 13.333333333333}, { 96.2527627689, 79.56461506624, 16.666666666667}, { 96.2527627689, 79.56461506624, 20},
          { 96.2527627689,  95.1780010696, 10}, { 96.2527627689,  95.1780010696, 13.333333333333}, { 96.2527627689,  95.1780010696, 16.666666666667}, { 96.2527627689,  95.1780010696, 20},
          { 96.2527627689, 104.3742043151, 10}, { 96.2527627689, 104.3742043151, 13.333333333333}, { 96.2527627689, 104.3742043151, 16.666666666667}, { 96.2527627689, 104.3742043151, 20} };

    std::vector<double> tGoldProbabilities = { 0.040214307576243, 0.040214307576243, 0.040214307576243, 0.040214307576243,
                                               0.029659870248084, 0.029659870248084, 0.029659870248084, 0.029659870248084,
                                               0.021188934342120, 0.021188934342120, 0.021188934342120, 0.021188934342120,
                                               0.021685307558066, 0.021685307558066, 0.021685307558066, 0.021685307558066,
                                               0.015993894890335, 0.015993894890335, 0.015993894890335, 0.015993894890335,
                                               0.011425997007791, 0.011425997007791, 0.011425997007791, 0.011425997007791,
                                               0.025396048040355, 0.025396048040355, 0.025396048040355, 0.025396048040355,
                                               0.018730733788290, 0.018730733788290, 0.018730733788290, 0.018730733788290,
                                               0.013381187614786, 0.013381187614786, 0.013381187614786, 0.013381187614786,
                                               0.023089690523626, 0.023089690523626, 0.023089690523626, 0.023089690523626,
                                               0.017029690830826, 0.017029690830826, 0.017029690830826, 0.017029690830826,
                                               0.012165966939936, 0.012165966939936, 0.012165966939936, 0.012165966939936};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations.mX, tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations.mY, tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_about_xy)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    tSampleProbPairSetY.mNumSamples = 3;
    tSampleProbPairSetY.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetY.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST OUTPUTS
    ASSERT_EQ(12u, tRandomRotationSet.size());
    std::vector<std::vector<double>> tSetGoldRotations =
        { {62.92995363352, 79.56461506624, 0}, {62.92995363352,  95.1780010696, 0}, {62.92995363352, 104.3742043151, 0},
          {69.67128118964, 79.56461506624, 0}, {69.67128118964,  95.1780010696, 0}, {69.67128118964, 104.3742043151, 0},
          {66.03455388567, 79.56461506624, 0}, {66.03455388567,  95.1780010696, 0}, {66.03455388567, 104.3742043151, 0},
          {96.25276276890, 79.56461506624, 0}, {96.25276276890,  95.1780010696, 0}, {96.25276276890, 104.3742043151, 0}};

    std::vector<double> tGoldProbabilities = { 0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                               0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                               0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                               0.0923587620945064, 0.0681187633233061, 0.0486638677597447 };

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations.mX, tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations.mY, tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_about_xz)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST OUTPUTS
    ASSERT_EQ(12u, tRandomRotationSet.size());
    std::vector<std::vector<double>> tSetGoldRotations =
        { {62.92995363352, 0, 79.56461506624}, {62.92995363352, 0, 95.1780010696}, {62.92995363352, 0, 104.3742043151},
          {69.67128118964, 0, 79.56461506624}, {69.67128118964, 0, 95.1780010696}, {69.67128118964, 0, 104.3742043151},
          {66.03455388567, 0, 79.56461506624}, {66.03455388567, 0, 95.1780010696}, {66.03455388567, 0, 104.3742043151},
          {96.25276276890, 0, 79.56461506624}, {96.25276276890, 0, 95.1780010696}, {96.25276276890, 0, 104.3742043151}};

    std::vector<double> tGoldProbabilities = { 0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                               0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                               0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                               0.0923587620945064, 0.0681187633233061, 0.0486638677597447 };

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations.mX, tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations.mY, tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_about_yz)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST OUTPUTS
    ASSERT_EQ(12u, tRandomRotationSet.size());
    std::vector<std::vector<double>> tSetGoldRotations =
        { {0, 62.92995363352, 79.56461506624}, {0, 62.92995363352, 95.1780010696}, {0, 62.92995363352, 104.3742043151},
          {0, 69.67128118964, 79.56461506624}, {0, 69.67128118964, 95.1780010696}, {0, 69.67128118964, 104.3742043151},
          {0, 66.03455388567, 79.56461506624}, {0, 66.03455388567, 95.1780010696}, {0, 66.03455388567, 104.3742043151},
          {0, 96.25276276890, 79.56461506624}, {0, 96.25276276890, 95.1780010696}, {0, 96.25276276890, 104.3742043151}};

    std::vector<double> tGoldProbabilities = { 0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                               0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                               0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                               0.0923587620945064, 0.0681187633233061, 0.0486638677597447 };

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations.mX, tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations.mY, tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_about_x)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST OUTPUTS
    ASSERT_EQ(4u, tRandomRotationSet.size());
    std::vector<std::vector<double>> tSetGoldRotations =
        { {62.92995363352, 0, 0}, {69.67128118964, 0, 0}, {66.03455388567, 0, 0}, {96.25276276890, 0, 0} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations.mX, tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations.mY, tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_about_y)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST OUTPUTS
    ASSERT_EQ(4u, tRandomRotationSet.size());
    std::vector<std::vector<double>> tSetGoldRotations =
        { {0, 62.92995363352, 0}, {0, 69.67128118964, 0}, {0, 66.03455388567, 0}, {0, 96.25276276890, 0} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations.mX, tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations.mY, tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_about_z)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 4;
    tSampleProbPairSetZ.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetZ.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST OUTPUTS
    ASSERT_EQ(4u, tRandomRotationSet.size());
    std::vector<std::vector<double>> tSetGoldRotations =
        { {0, 0, 62.92995363352}, {0, 0, 69.67128118964}, {0, 0, 66.03455388567}, {0, 0, 96.25276276890} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations.mX, tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations.mY, tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_error)
{
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_FALSE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));
}

TEST(PlatoTest, expand_random_loads_about_z)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 4;
    tSampleProbPairSetZ.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetZ.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(4u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
        { {-0.43537138598505315, 1.345530288126378, 1}, {-0.5903092061838121, 1.2851206328958531, 1},
          {-0.50760495686136231, 1.319976214850015, 1}, {-1.1029658951090628, 0.8851362800305180, 1} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_y)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(4u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
        { {1.34553028812638, 1, -0.435371385985053}, {1.28512063289585, 1, -0.590309206183812},
          {1.31997621485002, 1, -0.507604956861362}, {0.88513628003051, 1, -1.10296589510906} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_x)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(4u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
        { {1, -0.435371385985053, 1.34553028812638}, {1, -0.590309206183812, 1.28512063289585},
          {1, -0.507604956861362, 1.31997621485002}, {1, -1.10296589510906, 0.885136280030518} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_yz)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(12u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
    { {-0.739748540735107, 1.50440149320111, -0.435371385985053}, {-1.11735350148433, 1.24978914581067, -0.435371385985053}, {-1.30272801733653, 1.05515471335339, -0.435371385985053},
      {-0.750690333076780, 1.44499102589591, -0.590309206183812}, {-1.11190151802027, 1.18962601489664, -0.590309206183812}, {-1.28773109166515, 0.99663618068657, -0.590309206183812},
      {-0.744377061825103, 1.47927008946926, -0.507604956861362}, {-1.11504724126135, 1.22433935554046, -0.507604956861362}, {-1.29638412187704, 1.03040060962466, -0.507604956861362},
      {-0.823138118323529, 1.05162249518973, -1.102965895109060}, {-1.07580285019335, 0.79127394860574, -1.102965895109060}, {-1.18843346139817, 0.60917332677608, -1.102965895109060} };
    std::vector<double> tGoldProbabilities = {0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                              0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                              0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                              0.0923587620945064, 0.0681187633233061, 0.0486638677597447};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_xz)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(12u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
    { {0.609296803530850, 0.904602481171501, 1.34553028812638}, {0.343344489354037, 1.03521147857081, 1.34553028812638}, {0.173488310131751, 1.07677762327334, 1.34553028812638},
      {0.761671920821811, 0.876539128582960, 1.28512063289585}, {0.497650027509830, 1.04919464782510, 1.28512063289585}, {0.323575806212112, 1.11524152385909, 1.28512063289585},
      {0.680335616523720, 0.891519063800385, 1.31997621485002}, {0.415283283979423, 1.04173057278621, 1.31997621485002}, {0.243460611096492, 1.09470988077881, 1.31997621485002},
      {1.265849164342930, 0.783683392006009, 0.88513628003052}, {1.008214628020530, 1.09546201651137, 0.88513628003052}, {0.820183791860380, 1.24251048822264, 0.88513628003052} };
    std::vector<double> tGoldProbabilities = {0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                              0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                              0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                              0.0923587620945064, 0.0681187633233061, 0.0486638677597447};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_xy)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetY.mNumSamples = 3;
    tSampleProbPairSetY.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetY.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(12u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
    { {1.50440149320111, -0.435371385985053, -0.739748540735107}, {1.24978914581067, -0.435371385985053, -1.11735350148433}, {1.05515471335339, -0.435371385985053, -1.30272801733653},
      {1.44499102589591, -0.590309206183812, -0.750690333076780}, {1.18962601489664, -0.590309206183812, -1.11190151802027}, {0.99663618068657, -0.590309206183812, -1.28773109166515},
      {1.47927008946926, -0.507604956861362, -0.744377061825103}, {1.22433935554046, -0.507604956861362, -1.11504724126135}, {1.03040060962466, -0.507604956861362, -1.29638412187704},
      {1.05162249518973, -1.10296589510906,  -0.823138118323529}, {0.791273948605747, -1.10296589510906, -1.07580285019335}, {0.609173326776081,-1.10296589510906,  -1.18843346139817} };
    std::vector<double> tGoldProbabilities = {0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                              0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                              0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                              0.0923587620945064, 0.0681187633233061, 0.0486638677597447};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_xyz)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetY.mNumSamples = 3;
    tSampleProbPairSetY.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetY.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};
    tSampleProbPairSetZ.mNumSamples = 4;
    tSampleProbPairSetZ.mProbabilities = {0.25, 0.25, 0.25, 0.25};
    tSampleProbPairSetZ.mSamples = {10, 13.333333333333, 16.666666666667, 20.0};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(48u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
    { {1.55714770193223, -0.167520538583969, -0.739748540735107}, {1.56425370752726, -0.0766970336291921, -0.739748540735107}, {1.56606677372885, 0.0143859898095396, -0.739748540735107},
      {1.56258076569489, 0.105420335612506, -0.739748540735107}, {1.30640348820948, -0.211733508719819, -1.11735350148433}, {1.31650446891269, -0.135414681824415, -1.11735350148433},
      {1.32215081522277, -0.0586376545102008, -1.11735350148433}, {1.32332342169111, 0.0183377840655412, -1.11735350148433}, {1.11472599012243, -0.245531423227262, -1.30272801733653},
      {1.12711643277193, -0.180300470942008, -1.30272801733653}, {1.13569307019323, -0.114459438935836, -1.30272801733653}, {1.14042688176016, -0.0482311124196051, -1.30272801733653},
      {1.52554448324919, -0.330421024532408, -0.750690333076780}, {1.54217577737325, -0.241159481481218, -0.750690333076780}, {1.55358883683092, -0.151081931081903, -0.750690333076780},
      {1.55974504344169, -0.0604931672516919, -0.750690333076780}, {1.27405904056906, -0.374764693332327, -1.11190151802027}, {1.29369416329485, -0.300050705849966, -1.11190151802027},
      {1.30895183448128, -0.224321441757799, -1.11190151802027}, {1.31978042699877, -0.147833144914024, -1.11190151802027}, {1.08400115558644, -0.408277026351148, -1.28773109166515},
      {1.10590639504269, -0.344557224510759, -1.28773109166515}, {1.12406959732454, -0.279671550089624, -1.28773109166515}, {1.13842930390487, -0.213839555670947, -1.28773109166515},
      {1.54494132864207, -0.243020741670963, -0.744377061825103}, {1.55645793187998, -0.152779239808649, -0.744377061825103}, {1.56270797416156, -0.0620207813599240, -0.744377061825103},
      {1.56367030732212, 0.0289475357806807, -0.744377061825103}, {1.29388356538789, -0.287288999048993, -1.11504724126135}, {1.30839888887741, -0.211570312992648, -1.11504724126135},
      {1.31848700469088, -0.135135739968118, -1.11504724126135}, {1.32411377783769, -0.0582439103757674, -1.11504724126135}, {1.10289118480055, -0.320966108856280, -1.29638412187704},
      {1.11968778695736, -0.256295665752715, -1.29638412187704}, {1.13269572007193, -0.190757999244703, -1.29638412187704}, {1.14187096941614, -0.124574868050287, -1.29638412187704},
      {1.22717400421935, -0.903597034928209, -0.823138118323529}, {1.27763731503050, -0.830714469239849, -0.823138118323529}, {1.32377750559417, -0.755021028744024, -0.823138118323529},
      {1.36543845211078, -0.676772836083224, -0.823138118323529}, {0.970780737057986, -0.948806085600751, -1.07580285019335}, {1.02430649721405, -0.890754976006840, -1.07580285019335},
      {1.07436632842014, -0.829689833535944, -1.07580285019335}, {1.12079084405347, -0.765817283300505, -1.07580285019335}, {0.791446632851788, -0.980427526733487, -1.18843346139817},
      {0.847114421213299, -0.932750269472827, -1.18843346139817}, {0.899915842521188, -0.881916880555393, -1.18843346139817}, {0.949672233479613, -0.828099364078381, -1.18843346139817} };

    std::vector<double> tGoldProbabilities = { 0.040214307576243, 0.040214307576243, 0.040214307576243, 0.040214307576243,
                                               0.029659870248084, 0.029659870248084, 0.029659870248084, 0.029659870248084,
                                               0.021188934342120, 0.021188934342120, 0.021188934342120, 0.021188934342120,
                                               0.021685307558066, 0.021685307558066, 0.021685307558066, 0.021685307558066,
                                               0.015993894890335, 0.015993894890335, 0.015993894890335, 0.015993894890335,
                                               0.011425997007791, 0.011425997007791, 0.011425997007791, 0.011425997007791,
                                               0.025396048040355, 0.025396048040355, 0.025396048040355, 0.025396048040355,
                                               0.018730733788290, 0.018730733788290, 0.018730733788290, 0.018730733788290,
                                               0.013381187614786, 0.013381187614786, 0.013381187614786, 0.013381187614786,
                                               0.023089690523626, 0.023089690523626, 0.023089690523626, 0.023089690523626,
                                               0.017029690830826, 0.017029690830826, 0.017029690830826, 0.017029690830826,
                                               0.012165966939936, 0.012165966939936, 0.012165966939936, 0.012165966939936};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_error_1)
{
    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED SINCE SET OF RANDOM ROTATION IS EMPTY
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_FALSE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));
}

TEST(PlatoTest, expand_random_loads_error_2)
{
    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED DUE TO NaN X-COMPONENT
    Plato::Vector3D tMyOriginalLoad(std::numeric_limits<double>::quiet_NaN(), 1, 1);
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    Plato::srom::RandomRotations tMyRotation;
    tMyRotation.mRotations.mX = 0;
    tMyRotation.mRotations.mY = 0;
    tMyRotation.mRotations.mZ = 62.92995363352;
    tMyRandomRotationsSet.push_back(tMyRotation);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_FALSE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED DUE TO NaN Y-COMPONENT
    tMyOriginalLoad.mX = 1; tMyOriginalLoad.mY = std::numeric_limits<double>::quiet_NaN();
    ASSERT_FALSE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED DUE TO NaN Z-COMPONENT
    tMyOriginalLoad.mY = 1; tMyOriginalLoad.mZ = std::numeric_limits<double>::quiet_NaN();
    ASSERT_FALSE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));
}

TEST(PlatoTest, expand_random_loads_error_3)
{
    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED DUE TO NaN X-COMPONENT
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    Plato::srom::RandomRotations tMyRotation;
    tMyRotation.mRotations.mX = std::numeric_limits<double>::quiet_NaN(); tMyRotation.mRotations.mY = 0; tMyRotation.mRotations.mZ = 62.92995363352;
    tMyRandomRotationsSet.push_back(tMyRotation);
    ASSERT_FALSE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED DUE TO NaN Y-COMPONENT
    tMyRandomRotationsSet[0].mRotations.mX = 0; tMyRandomRotationsSet[0].mRotations.mY = std::numeric_limits<double>::quiet_NaN();
    ASSERT_FALSE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED DUE TO NaN Z-COMPONENT
    tMyRandomRotationsSet[0].mRotations.mY = 0; tMyRandomRotationsSet[0].mRotations.mZ = std::numeric_limits<double>::quiet_NaN();
    ASSERT_FALSE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));
}

TEST(PlatoTest, expand_random_load_cases_error)
{
    // CALL EXPAND RANDOM LOAD CASES FUNCTION - FAILURE IS EXPECTED DUE TO EMPTY ARRAY OF NEW RANDOM LOADS
    std::vector<Plato::srom::RandomLoad> tNewSetRandomLoads;
    std::vector<Plato::srom::RandomLoadCase> tOldRandomLoadCases;
    ASSERT_FALSE(Plato::expand_random_load_cases(tNewSetRandomLoads, tOldRandomLoadCases));
}

TEST(PlatoTest, expand_random_load_cases_one_load)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tNewSetRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION - FUNCTION BEING TESTED
    std::vector<Plato::srom::RandomLoadCase> tRandomLoadCases;
    ASSERT_TRUE(Plato::expand_random_load_cases(tNewSetRandomLoads, tRandomLoadCases));

    std::vector<std::vector<double>> tGoldLoads =
    { {0.609296803530850, 0.904602481171501, 1.34553028812638}, {0.343344489354037, 1.03521147857081, 1.34553028812638}, {0.173488310131751, 1.07677762327334, 1.34553028812638},
      {0.761671920821811, 0.876539128582960, 1.28512063289585}, {0.497650027509830, 1.04919464782510, 1.28512063289585}, {0.323575806212112, 1.11524152385909, 1.28512063289585},
      {0.680335616523720, 0.891519063800385, 1.31997621485002}, {0.415283283979423, 1.04173057278621, 1.31997621485002}, {0.243460611096492, 1.09470988077881, 1.31997621485002},
      {1.265849164342930, 0.783683392006009, 0.88513628003052}, {1.008214628020530, 1.09546201651137, 0.88513628003052}, {0.820183791860380, 1.24251048822264, 0.88513628003052} };
    std::vector<double> tGoldProbabilities = {0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                              0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                              0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                              0.0923587620945064, 0.0681187633233061, 0.0486638677597447};

    double tSum = 0;
    double tTolerance = 1e-4;
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tRandomLoadCases.size(); tLoadCaseIndex++)
    {
        const Plato::srom::RandomLoadCase& tRandomLoadCase = tRandomLoadCases[tLoadCaseIndex];
        const Plato::srom::RandomLoad& tRandomLoad = tRandomLoadCase.mLoads[0];

        ASSERT_NEAR(tGoldLoads[tLoadCaseIndex][0], tRandomLoad.mLoadValues.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tLoadCaseIndex][1], tRandomLoad.mLoadValues.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tLoadCaseIndex][2], tRandomLoad.mLoadValues.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tLoadCaseIndex], tRandomLoad.mProbability, tTolerance);
        tSum += tRandomLoadCase.mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_load_cases_two_load)
{
    // *** SET FIRST LOAD CASE ***
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tNewSetRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION - FUNCTION BEING TESTED
    std::vector<Plato::srom::RandomLoadCase> tRandomLoadCases;
    ASSERT_TRUE(Plato::expand_random_load_cases(tNewSetRandomLoads, tRandomLoadCases));

    // *** SET SECOND LOAD CASE ***
    tSampleProbPairSetX.mNumSamples = 0;
    tSampleProbPairSetX.mSamples.clear();
    tSampleProbPairSetX.mProbabilities.clear();
    tSampleProbPairSetZ.mNumSamples = 0;
    tSampleProbPairSetZ.mSamples.clear();
    tSampleProbPairSetZ.mProbabilities.clear();
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // COMPUTE RANDOM ROTATIONS SET
    tMyRandomRotationsSet.clear();
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    tNewSetRandomLoads.clear();
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION - FUNCTION BEING TESTED
    ASSERT_TRUE(Plato::expand_random_load_cases(tNewSetRandomLoads, tRandomLoadCases));

    std::vector<std::vector<std::vector<double>>> tGoldLoadCases =
      { { {0.609296803530850, 0.904602481171501, 1.34553028812638}, {1.34553028812638, 1, -0.435371385985053} }, { {0.343344489354037, 1.03521147857081, 1.34553028812638}, {1.34553028812638, 1, -0.435371385985053} },
        { {0.173488310131751, 1.07677762327334, 1.34553028812638}, {1.34553028812638, 1, -0.435371385985053} }, { {0.761671920821811, 0.876539128582960, 1.28512063289585}, {1.34553028812638, 1, -0.435371385985053} },
        { {0.497650027509830, 1.04919464782510, 1.28512063289585}, {1.34553028812638, 1, -0.435371385985053} }, { {0.323575806212112, 1.11524152385909, 1.28512063289585}, {1.34553028812638, 1, -0.435371385985053} },
        { {0.680335616523720, 0.891519063800385, 1.31997621485002}, {1.34553028812638, 1, -0.435371385985053} }, { {0.415283283979423, 1.04173057278621, 1.31997621485002}, {1.34553028812638, 1, -0.435371385985053} },
        { {0.243460611096492, 1.09470988077881, 1.31997621485002}, {1.34553028812638, 1, -0.435371385985053} }, { {1.265849164342930, 0.783683392006009, 0.88513628003052}, {1.34553028812638, 1, -0.435371385985053} },
        { {1.008214628020530, 1.09546201651137, 0.88513628003052}, {1.34553028812638, 1, -0.435371385985053} }, { {0.820183791860380, 1.24251048822264, 0.88513628003052}, {1.34553028812638, 1, -0.435371385985053} },

        { {0.609296803530850, 0.904602481171501, 1.34553028812638}, {1.28512063289585, 1, -0.590309206183812} }, { {0.343344489354037, 1.03521147857081, 1.34553028812638}, {1.28512063289585, 1, -0.590309206183812} },
        { {0.173488310131751, 1.07677762327334, 1.34553028812638}, {1.28512063289585, 1, -0.590309206183812} }, { {0.761671920821811, 0.876539128582960, 1.28512063289585}, {1.28512063289585, 1, -0.590309206183812} },
        { {0.497650027509830, 1.04919464782510, 1.28512063289585}, {1.28512063289585, 1, -0.590309206183812} }, { {0.323575806212112, 1.11524152385909, 1.28512063289585}, {1.28512063289585, 1, -0.590309206183812} },
        { {0.680335616523720, 0.891519063800385, 1.31997621485002}, {1.28512063289585, 1, -0.590309206183812} }, { {0.415283283979423, 1.04173057278621, 1.31997621485002}, {1.28512063289585, 1, -0.590309206183812} },
        { {0.243460611096492, 1.09470988077881, 1.31997621485002}, {1.28512063289585, 1, -0.590309206183812} }, { {1.265849164342930, 0.783683392006009, 0.88513628003052}, {1.28512063289585, 1, -0.590309206183812} },
        { {1.008214628020530, 1.09546201651137, 0.88513628003052}, {1.28512063289585, 1, -0.590309206183812} }, { {0.820183791860380, 1.24251048822264, 0.88513628003052}, {1.28512063289585, 1, -0.590309206183812} },

        { {0.609296803530850, 0.904602481171501, 1.34553028812638}, {1.31997621485002, 1, -0.507604956861362} }, { {0.343344489354037, 1.03521147857081, 1.34553028812638}, {1.31997621485002, 1, -0.507604956861362} },
        { {0.173488310131751, 1.07677762327334, 1.34553028812638}, {1.31997621485002, 1, -0.507604956861362} }, { {0.761671920821811, 0.876539128582960, 1.2851206328958}, {1.31997621485002, 1, -0.507604956861362} },
        { {0.497650027509830, 1.04919464782510, 1.28512063289585}, {1.31997621485002, 1, -0.507604956861362} }, { {0.323575806212112, 1.11524152385909, 1.28512063289585},  {1.31997621485002, 1, -0.507604956861362} },
        { {0.680335616523720, 0.891519063800385, 1.31997621485002}, {1.31997621485002, 1, -0.507604956861362} }, { {0.415283283979423, 1.04173057278621, 1.31997621485002}, {1.31997621485002, 1, -0.507604956861362} },
        { {0.243460611096492, 1.09470988077881, 1.31997621485002}, {1.31997621485002, 1, -0.507604956861362} }, { {1.265849164342930, 0.783683392006009, 0.8851362800305}, {1.31997621485002, 1, -0.507604956861362} },
        { {1.008214628020530, 1.09546201651137, 0.88513628003052}, {1.31997621485002, 1, -0.507604956861362} }, { {0.820183791860380, 1.24251048822264, 0.88513628003052},  {1.31997621485002, 1, -0.507604956861362} },

        { {0.609296803530850, 0.904602481171501, 1.34553028812638}, {0.88513628003051, 1, -1.10296589510906} }, { {0.343344489354037, 1.03521147857081, 1.34553028812638}, {0.88513628003051, 1, -1.10296589510906} },
        { {0.173488310131751, 1.07677762327334, 1.34553028812638}, {0.88513628003051, 1, -1.10296589510906} }, {{0.761671920821811, 0.876539128582960, 1.28512063289585}, {0.88513628003051, 1, -1.10296589510906} },
        { {0.497650027509830, 1.04919464782510, 1.28512063289585}, {0.88513628003051, 1, -1.10296589510906} }, { {0.323575806212112, 1.11524152385909, 1.28512063289585}, {0.88513628003051, 1, -1.10296589510906} },
        { {0.680335616523720, 0.891519063800385, 1.31997621485002}, {0.88513628003051, 1, -1.10296589510906} }, {  {0.415283283979423, 1.04173057278621, 1.31997621485002}, {0.88513628003051, 1, -1.10296589510906} },
        { {0.243460611096492, 1.09470988077881, 1.31997621485002}, {0.88513628003051, 1, -1.10296589510906} }, { {1.265849164342930, 0.783683392006009, 0.88513628003052}, {0.88513628003051, 1, -1.10296589510906} },
        { {1.008214628020530, 1.09546201651137, 0.88513628003052}, {0.88513628003051, 1, -1.10296589510906} }, { {0.820183791860380, 1.24251048822264, 0.88513628003052}, {0.88513628003051, 1, -1.10296589510906} } };

    std::vector<double> tGoldProbabilities = { 0.0586005901270716,  0.0432205850202652, 0.0308766737872556, 0.0315999925544024, 0.0233064233973704, 0.0166500483982540 ,
                                               0.0370073113714254,  0.0272945655330732, 0.0194991667913430, 0.0336464699279151, 0.0248157929979522, 0.0177283381243119,

                                               0.0315999925544023,  0.0233064233973704, 0.0166500483982540, 0.0170400934064482, 0.0125678394062182, 0.00897843186006081,
                                               0.0199559554137536,  0.0147184194860575, 0.0105148006886510, 0.0181436432107338, 0.0133817572871952, 0.00955989268223052,

                                               0.0370073113714254,  0.0272945655330731, 0.0194991667913430, 0.0199559554137537, 0.0147184194860575, 0.0105148006886510 ,
                                               0.0233707730924189,  0.0172370019353027, 0.0123140694550314, 0.0212483421493127, 0.0156716131426772, 0.0111957597635392,

                                               0.0336464699279150,  0.0248157929979521, 0.0177283381243119, 0.0181436432107338, 0.0133817572871952, 0.00955989268223053,
                                               0.0212483421493127,  0.0156716131426772, 0.0111957597635392, 0.0193186610604985, 0.0142483860833551, 0.0101790100454297 };

    double tSum = 0;
    double tTolerance = 1e-4;
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tRandomLoadCases.size(); tLoadCaseIndex++)
    {
        const Plato::srom::RandomLoadCase& tRandomLoadCase = tRandomLoadCases[tLoadCaseIndex];
        ASSERT_NEAR(tGoldProbabilities[tLoadCaseIndex], tRandomLoadCase.mProbability, tTolerance);
        for(size_t tLoadIndex = 0; tLoadIndex < tRandomLoadCase.mLoads.size(); tLoadIndex++)
        {
            const Plato::srom::RandomLoad& tRandomLoad = tRandomLoadCase.mLoads[tLoadIndex];
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][0], tRandomLoad.mLoadValues.mX, tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][1], tRandomLoad.mLoadValues.mY, tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][2], tRandomLoad.mLoadValues.mZ, tTolerance);
        }
        tSum += tRandomLoadCase.mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_and_deterministic_loads_error)
{
    // FUNCTION FAILED DUE TO EMPTY SET OF LOADS
    std::vector<Plato::srom::Load> tLoads;
    std::vector<Plato::srom::Load> tRandomLoads;
    std::vector<Plato::srom::Load> tDeterministicLoads;
    ASSERT_FALSE(Plato::expand_random_and_deterministic_loads(tLoads, tRandomLoads, tDeterministicLoads));
}

TEST(PlatoTest, expand_random_and_deterministic_loads_only_random_loads)
{
    // INPUTS
    std::vector<Plato::srom::Load> tLoads;
    Plato::srom::Load tLoad1;
    tLoad1.mAppID = 1;
    tLoad1.mAppType = "nodeset";
    tLoad1.mLoadID = 1;
    tLoad1.mLoadType = "traction";
    tLoad1.mValues = {"1", "2", "3"};
    Plato::srom::Variable tRandVar1;
    tRandVar1.mType = "random rotation";
    tRandVar1.mSubType = "x";
    tRandVar1.mStatistics.mMean = "60";
    tRandVar1.mStatistics.mNumSamples = "2";
    tRandVar1.mStatistics.mLowerBound = "40";
    tRandVar1.mStatistics.mUpperBound = "100";
    tRandVar1.mStatistics.mDistribution = "beta";
    tRandVar1.mStatistics.mStandardDeviation = "10";
    Plato::srom::Variable tRandVar2;
    tRandVar2.mType = "random rotation";
    tRandVar2.mSubType = "y";
    tRandVar2.mStatistics.mMean = "65";
    tRandVar2.mStatistics.mNumSamples = "2";
    tRandVar2.mStatistics.mLowerBound = "40";
    tRandVar2.mStatistics.mUpperBound = "100";
    tRandVar2.mStatistics.mDistribution = "beta";
    tRandVar2.mStatistics.mStandardDeviation = "10";
    tLoad1.mRandomVars.push_back(tRandVar2);
    tLoads.push_back(tLoad1);

    Plato::srom::Load tLoad2;
    tLoad2.mAppID = 2;
    tLoad2.mAppType = "nodeset";
    tLoad2.mLoadID = 2;
    tLoad2.mLoadType = "traction";
    tLoad2.mValues = {"1", "2", "3"};
    tRandVar1.mType = "random rotation";
    tRandVar1.mSubType = "x";
    tRandVar1.mStatistics.mMean = "70";
    tRandVar1.mStatistics.mNumSamples = "2";
    tRandVar1.mStatistics.mLowerBound = "45";
    tRandVar1.mStatistics.mUpperBound = "110";
    tRandVar1.mStatistics.mDistribution = "beta";
    tRandVar1.mStatistics.mStandardDeviation = "15";
    tRandVar2.mType = "random rotation";
    tRandVar2.mSubType = "z";
    tRandVar2.mStatistics.mMean = "75";
    tRandVar2.mStatistics.mNumSamples = "2";
    tRandVar2.mStatistics.mLowerBound = "50";
    tRandVar2.mStatistics.mUpperBound = "120";
    tRandVar2.mStatistics.mDistribution = "beta";
    tRandVar2.mStatistics.mStandardDeviation = "12";
    tLoad2.mRandomVars.push_back(tRandVar2);
    tLoads.push_back(tLoad2);

    // CALL FUNCTION TO BE TESTED
    std::vector<Plato::srom::Load> tRandomLoads;
    std::vector<Plato::srom::Load> tDeterministicLoads;
    ASSERT_TRUE(Plato::expand_random_and_deterministic_loads(tLoads, tRandomLoads, tDeterministicLoads));

    // TEST OUTPUT
    ASSERT_FALSE(tRandomLoads.empty());
    ASSERT_TRUE(tDeterministicLoads.empty());

    ASSERT_EQ(2u, tRandomLoads.size());
    for(size_t tLoadIndex = 0; tLoadIndex < tRandomLoads.size(); tLoadIndex++)
    {
        ASSERT_EQ(tLoads[tLoadIndex].mAppID, tRandomLoads[tLoadIndex].mAppID);
        ASSERT_EQ(tLoads[tLoadIndex].mLoadID, tRandomLoads[tLoadIndex].mLoadID);
        ASSERT_STREQ(tLoads[tLoadIndex].mAppType.c_str(), tRandomLoads[tLoadIndex].mAppType.c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mLoadType.c_str(), tRandomLoads[tLoadIndex].mLoadType.c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mValues[0].c_str(), tRandomLoads[tLoadIndex].mValues[0].c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mValues[1].c_str(), tRandomLoads[tLoadIndex].mValues[1].c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mValues[2].c_str(), tRandomLoads[tLoadIndex].mValues[2].c_str());

        for(size_t tVarIndex = 0; tVarIndex < tRandomLoads[tLoadIndex].mRandomVars.size(); tVarIndex++)
        {
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mType.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mType.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mSubType.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mSubType.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mMean.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mMean.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mNumSamples.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mNumSamples.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mLowerBound.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mLowerBound.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mUpperBound.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mUpperBound.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mDistribution.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mDistribution.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mStandardDeviation.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mStandardDeviation.c_str());
        }
    }
}

TEST(PlatoTest, expand_random_and_deterministic_loads)
{
    // RANDOM INPUTS
    std::vector<Plato::srom::Load> tLoads;
    Plato::srom::Load tLoad1;
    tLoad1.mAppID = 1;
    tLoad1.mAppType = "nodeset";
    tLoad1.mLoadID = 1;
    tLoad1.mLoadType = "traction";
    tLoad1.mValues = {"1", "2", "3"};
    Plato::srom::Variable tRandVar1;
    tRandVar1.mType = "random rotation";
    tRandVar1.mSubType = "x";
    tRandVar1.mStatistics.mMean = "60";
    tRandVar1.mStatistics.mNumSamples = "2";
    tRandVar1.mStatistics.mLowerBound = "40";
    tRandVar1.mStatistics.mUpperBound = "100";
    tRandVar1.mStatistics.mDistribution = "beta";
    tRandVar1.mStatistics.mStandardDeviation = "10";
    Plato::srom::Variable tRandVar2;
    tRandVar2.mType = "random rotation";
    tRandVar2.mSubType = "y";
    tRandVar2.mStatistics.mMean = "65";
    tRandVar2.mStatistics.mNumSamples = "2";
    tRandVar2.mStatistics.mLowerBound = "40";
    tRandVar2.mStatistics.mUpperBound = "100";
    tRandVar2.mStatistics.mDistribution = "beta";
    tRandVar2.mStatistics.mStandardDeviation = "10";
    tLoad1.mRandomVars.push_back(tRandVar2);
    tLoads.push_back(tLoad1);

    Plato::srom::Load tLoad2;
    tLoad2.mAppID = 2;
    tLoad2.mAppType = "nodeset";
    tLoad2.mLoadID = 2;
    tLoad2.mLoadType = "traction";
    tLoad2.mValues = {"1", "2", "3"};
    tRandVar1.mType = "random rotation";
    tRandVar1.mSubType = "x";
    tRandVar1.mStatistics.mMean = "70";
    tRandVar1.mStatistics.mNumSamples = "2";
    tRandVar1.mStatistics.mLowerBound = "45";
    tRandVar1.mStatistics.mUpperBound = "110";
    tRandVar1.mStatistics.mDistribution = "beta";
    tRandVar1.mStatistics.mStandardDeviation = "15";
    tRandVar2.mType = "random rotation";
    tRandVar2.mSubType = "z";
    tRandVar2.mStatistics.mMean = "75";
    tRandVar2.mStatistics.mNumSamples = "2";
    tRandVar2.mStatistics.mLowerBound = "50";
    tRandVar2.mStatistics.mUpperBound = "120";
    tRandVar2.mStatistics.mDistribution = "beta";
    tRandVar2.mStatistics.mStandardDeviation = "12";
    tLoad2.mRandomVars.push_back(tRandVar2);
    tLoads.push_back(tLoad2);

    // DETERMINISTIC INPUTS
    Plato::srom::Load tLoad3;
    tLoad3.mAppID = 3;
    tLoad3.mAppType = "SIDESET";
    tLoad3.mLoadID = 3;
    tLoad3.mLoadType = "pressure";
    tLoad3.mValues = {"1", "2", "3"};
    tLoads.push_back(tLoad3);

    // CALL FUNCTION TO BE TESTED
    std::vector<Plato::srom::Load> tRandomLoads;
    std::vector<Plato::srom::Load> tDeterministicLoads;
    ASSERT_TRUE(Plato::expand_random_and_deterministic_loads(tLoads, tRandomLoads, tDeterministicLoads));

    // TEST OUTPUT
    ASSERT_FALSE(tRandomLoads.empty());
    ASSERT_FALSE(tDeterministicLoads.empty());

    // TEST RANDOM LOADS
    ASSERT_EQ(2u, tRandomLoads.size());
    for(size_t tLoadIndex = 0; tLoadIndex < tRandomLoads.size(); tLoadIndex++)
    {
        ASSERT_EQ(tLoads[tLoadIndex].mAppID, tRandomLoads[tLoadIndex].mAppID);
        ASSERT_EQ(tLoads[tLoadIndex].mLoadID, tRandomLoads[tLoadIndex].mLoadID);
        ASSERT_STREQ(tLoads[tLoadIndex].mAppType.c_str(), tRandomLoads[tLoadIndex].mAppType.c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mLoadType.c_str(), tRandomLoads[tLoadIndex].mLoadType.c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mValues[0].c_str(), tRandomLoads[tLoadIndex].mValues[0].c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mValues[1].c_str(), tRandomLoads[tLoadIndex].mValues[1].c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mValues[2].c_str(), tRandomLoads[tLoadIndex].mValues[2].c_str());

        for(size_t tVarIndex = 0; tVarIndex < tRandomLoads[tLoadIndex].mRandomVars.size(); tVarIndex++)
        {
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mType.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mType.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mSubType.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mSubType.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mMean.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mMean.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mNumSamples.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mNumSamples.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mLowerBound.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mLowerBound.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mUpperBound.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mUpperBound.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mDistribution.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mDistribution.c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mStandardDeviation.c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mStatistics.mStandardDeviation.c_str());
        }
    }

    // TEST DETERMINISTIC LOADS
    ASSERT_EQ(1u, tDeterministicLoads.size());
    ASSERT_TRUE(tDeterministicLoads[0].mRandomVars.empty());
    ASSERT_EQ(tLoads[2].mAppID, tDeterministicLoads[0].mAppID);
    ASSERT_EQ(tLoads[2].mLoadID, tDeterministicLoads[0].mLoadID);
    ASSERT_STREQ(tLoads[2].mAppType.c_str(), tDeterministicLoads[0].mAppType.c_str());
    ASSERT_STREQ(tLoads[2].mLoadType.c_str(), tDeterministicLoads[0].mLoadType.c_str());
    ASSERT_STREQ(tLoads[2].mValues[0].c_str(), tDeterministicLoads[0].mValues[0].c_str());
    ASSERT_STREQ(tLoads[2].mValues[1].c_str(), tDeterministicLoads[0].mValues[1].c_str());
    ASSERT_STREQ(tLoads[2].mValues[2].c_str(), tDeterministicLoads[0].mValues[2].c_str());
}

TEST(PlatoTest, set_load_components_errors)
{
    // TEST ERROR: EMPTY INPUT ARRAY
    Plato::Vector3D tFloatLoad;
    std::vector<std::string> tStringLoad;
    ASSERT_FALSE(Plato::set_load_components(tStringLoad, tFloatLoad));

    // TEST ERROR: INPUT ARRAY SIZE IS NOT EQUAL TO 3
    tStringLoad = {"1", "2"};
    ASSERT_FALSE(Plato::set_load_components(tStringLoad, tFloatLoad));
}

TEST(PlatoTest, set_load_components)
{
    Plato::Vector3D tFloatLoad;
    std::vector<std::string> tStringLoad = {"1", "2", "3"};
    ASSERT_TRUE(Plato::set_load_components(tStringLoad, tFloatLoad));

    // TEST OUTPUT
    ASSERT_EQ(1.0, tFloatLoad.mX);
    ASSERT_EQ(2.0, tFloatLoad.mY);
    ASSERT_EQ(3.0, tFloatLoad.mZ);
}

TEST(PlatoTest, generate_set_random_rotations_errors)
{
    // TEST ERROR: EMPTY SET OF SAMPLE-PROBABILITY PAIRS
    std::vector<Plato::srom::RandomVariable> tMySampleProbPairs;
    std::vector<Plato::srom::RandomRotations> tMySetRandomRotation;
    ASSERT_FALSE(Plato::generate_set_random_rotations(tMySampleProbPairs, tMySetRandomRotation));
}

TEST(PlatoTest, generate_set_random_rotations)
{
    // SET INPUTS
    Plato::srom::RandomVariable tRandVar;
    tRandVar.mSubType = "x";
    tRandVar.mType = "random rotation";
    tRandVar.mSampleProbPairs.mNumSamples = 2;
    tRandVar.mSampleProbPairs.mSamples = {1, 2};
    tRandVar.mSampleProbPairs.mProbabilities = {0.5, 0.5};
    std::vector<Plato::srom::RandomVariable> tMySampleProbPairs;
    tMySampleProbPairs.push_back(tRandVar);

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tMySetRandomRotation;
    ASSERT_TRUE(Plato::generate_set_random_rotations(tMySampleProbPairs, tMySetRandomRotation));

    // TEST OUTPUT
    ASSERT_FALSE(tMySetRandomRotation.empty());
    ASSERT_EQ(0.5, tMySetRandomRotation[0].mProbability);
    ASSERT_EQ(1.0, tMySetRandomRotation[0].mRotations.mX);
    ASSERT_EQ(0.0, tMySetRandomRotation[0].mRotations.mY);
    ASSERT_EQ(0.0, tMySetRandomRotation[0].mRotations.mZ);

    ASSERT_EQ(0.5, tMySetRandomRotation[1].mProbability);
    ASSERT_EQ(2.0, tMySetRandomRotation[1].mRotations.mX);
    ASSERT_EQ(0.0, tMySetRandomRotation[1].mRotations.mY);
    ASSERT_EQ(0.0, tMySetRandomRotation[1].mRotations.mZ);
}

TEST(PlatoTest, check_load_parameters)
{
    // FAILED: APPLICATION ID IS NOT DEFINED
    Plato::srom::Load tLoad;
    ASSERT_FALSE(Plato::check_load_parameters(tLoad));

    // FAILED: APPLICATION TYPE IS NOT DEFINED
    tLoad.mAppID = 1;
    ASSERT_FALSE(Plato::check_load_parameters(tLoad));

    // FAILED: LOAD TYPE IS NOT DEFINED
    tLoad.mAppType = "sideset";
    ASSERT_FALSE(Plato::check_load_parameters(tLoad));

    // FAILED: LOAD VALUES/COMPONENTS ARE NOT DEFINED
    tLoad.mLoadType = "traction";
    ASSERT_FALSE(Plato::check_load_parameters(tLoad));

    // PASS
    tLoad.mValues = {"1", "2", "3"};
    ASSERT_TRUE(Plato::check_load_parameters(tLoad));
}

TEST(PlatoTest, check_deterministic_loads)
{
    // FAILED: APPLICATION ID IS NOT DEFINED
    Plato::srom::Load tLoad;
    std::vector<Plato::srom::Load> tDeterministicLoads;
    tDeterministicLoads.push_back(tLoad);
    ASSERT_FALSE(Plato::check_deterministic_loads(tDeterministicLoads));

    // FAILED: APPLICATION TYPE IS NOT DEFINED
    tDeterministicLoads[0].mAppID = 1;
    ASSERT_FALSE(Plato::check_deterministic_loads(tDeterministicLoads));

    // FAILED: LOAD TYPE IS NOT DEFINED
    tDeterministicLoads[0].mAppType = "sideset";
    ASSERT_FALSE(Plato::check_deterministic_loads(tDeterministicLoads));

    // FAILED: LOAD VALUES/COMPONENTS ARE NOT DEFINED
    tDeterministicLoads[0].mLoadType = "traction";
    ASSERT_FALSE(Plato::check_deterministic_loads(tDeterministicLoads));

    // PASS
    tDeterministicLoads[0].mValues = {"1", "2", "3"};
    ASSERT_TRUE(Plato::check_deterministic_loads(tDeterministicLoads));
}

TEST(PlatoTest, append_deterministic_loads)
{
    // SET INPUTS - CREATE SET OF RANDOM LOAD CASES
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tNewSetRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION
    std::vector<Plato::srom::RandomLoadCase> tRandomLoadCases;
    ASSERT_TRUE(Plato::expand_random_load_cases(tNewSetRandomLoads, tRandomLoadCases));
    ASSERT_EQ(12u, tRandomLoadCases.size());

    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tRandomLoadCases.size(); tLoadCaseIndex++)
    {
        ASSERT_EQ(1u, tRandomLoadCases[tLoadCaseIndex].mLoads.size());
    }

    // SET INPUTS - CREATE SET OF DETERMINISTIC LOADS
    Plato::srom::Load tLoadDet1;
    tLoadDet1.mAppID = 3;
    tLoadDet1.mAppType = "sideset";
    tLoadDet1.mLoadType = "pressure";
    tLoadDet1.mValues = {"1", "2", "3"};
    std::vector<Plato::srom::Load> tDeterministicLoads;
    tDeterministicLoads.push_back(tLoadDet1);

    // CALL FUNCTION TO BE TESTED
    Plato::append_deterministic_loads(tDeterministicLoads, tRandomLoadCases);

    // TEST OUTPUT
    const double tTolerance = 1e-6;
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tRandomLoadCases.size(); tLoadCaseIndex++)
    {
        ASSERT_EQ(2u, tRandomLoadCases[tLoadCaseIndex].mLoads.size());
        // EXPECT VALUES FOR DETERMINISTIC LOAD
        ASSERT_EQ(3, tRandomLoadCases[tLoadCaseIndex].mLoads[1].mAppID);
        ASSERT_NEAR(1.0, tRandomLoadCases[tLoadCaseIndex].mLoads[1].mProbability, tTolerance);
        ASSERT_NEAR(1.0, tRandomLoadCases[tLoadCaseIndex].mLoads[1].mLoadValues.mX, tTolerance);
        ASSERT_NEAR(2.0, tRandomLoadCases[tLoadCaseIndex].mLoads[1].mLoadValues.mY, tTolerance);
        ASSERT_NEAR(3.0, tRandomLoadCases[tLoadCaseIndex].mLoads[1].mLoadValues.mZ, tTolerance);
        ASSERT_STREQ(tLoadDet1.mAppType.c_str(), tRandomLoadCases[tLoadCaseIndex].mLoads[1].mAppType.c_str());
        ASSERT_STREQ(tLoadDet1.mLoadType.c_str(), tRandomLoadCases[tLoadCaseIndex].mLoads[1].mLoadType.c_str());
    }
}

TEST(PlatoTest, set_random_load_parameters_error)
{
    Plato::srom::Load tOriginalLoad;
    std::vector<Plato::srom::RandomLoad> tSetRandomLoads;
    ASSERT_FALSE(Plato::set_random_load_parameters(tOriginalLoad, tSetRandomLoads));
}

TEST(PlatoTest, set_random_load_parameters)
{
    // SET INPUTS
    Plato::srom::Load tOriginalLoad;
    tOriginalLoad.mAppID = 1;
    tOriginalLoad.mAppType = "sideset";
    tOriginalLoad.mLoadType = "traction";
    tOriginalLoad.mValues = {"1", "2", "3"};

    // CALL FUNCTION
    Plato::srom::RandomLoad tRandVar1;
    tRandVar1.mProbability = 0.5;
    std::vector<Plato::srom::RandomLoad> tSetRandomLoads;
    tSetRandomLoads.push_back(tRandVar1);
    ASSERT_TRUE(Plato::set_random_load_parameters(tOriginalLoad, tSetRandomLoads));

    // TEST OUTPUT
    ASSERT_EQ(tOriginalLoad.mAppID, tSetRandomLoads[0].mAppID);
    ASSERT_STREQ(tOriginalLoad.mAppType.c_str(), tSetRandomLoads[0].mAppType.c_str());
    ASSERT_STREQ(tOriginalLoad.mLoadType.c_str(), tSetRandomLoads[0].mLoadType.c_str());
}

TEST(PlatoTest, generate_set_random_loads_error)
{
    // ERROR: EMPTY ORIGINAL LOAD VALUES
    Plato::srom::Load tOriginalLoad;
    std::vector<Plato::srom::RandomLoad> tSetRandomLoads;
    std::vector<Plato::srom::RandomRotations> tSetRandomRotations;
    ASSERT_FALSE(Plato::generate_set_random_loads(tOriginalLoad, tSetRandomRotations, tSetRandomLoads));

    // ERROR: EMPTY SET OF RANDOM ROTATIONS
    tOriginalLoad.mValues = {"1", "2", "3"};
    ASSERT_FALSE(Plato::generate_set_random_loads(tOriginalLoad, tSetRandomRotations, tSetRandomLoads));

    // ERROR: LOAD PARAMETERS ARE NOT DEFINED
    Plato::srom::RandomRotations tRotations1;
    tRotations1.mRotations.mX = 62.92995363352; tRotations1.mRotations.mY = 0; tRotations1.mRotations.mZ = 0;
    tRotations1.mProbability = 0.3643018720139;
    tSetRandomRotations.push_back(tRotations1);
    Plato::srom::RandomRotations tRotations2;
    tRotations2.mRotations.mX = 69.67128118964; tRotations1.mRotations.mY = 0; tRotations1.mRotations.mZ = 0;
    tRotations2.mProbability = 0.1964474490484;
    tSetRandomRotations.push_back(tRotations2);
    Plato::srom::RandomRotations tRotations3;
    tRotations3.mRotations.mX = 66.03455388567; tRotations1.mRotations.mY = 0; tRotations1.mRotations.mZ = 0;
    tRotations3.mProbability = 0.2300630894941;
    tSetRandomRotations.push_back(tRotations3);
    Plato::srom::RandomRotations tRotations4;
    tRotations4.mRotations.mX = 96.25276276890; tRotations1.mRotations.mY = 0; tRotations1.mRotations.mZ = 0;
    tRotations4.mProbability = 0.2091697703866;
    tSetRandomRotations.push_back(tRotations4);
    ASSERT_FALSE(Plato::generate_set_random_loads(tOriginalLoad, tSetRandomRotations, tSetRandomLoads));
}

TEST(PlatoTest, generate_set_random_loads)
{
    // SET INPUPTS
    Plato::srom::Load tOriginalLoad;
    tOriginalLoad.mAppID = 1;
    tOriginalLoad.mLoadID = 1;
    tOriginalLoad.mAppType = "nodeset";
    tOriginalLoad.mLoadType = "traction";
    tOriginalLoad.mValues = {"1", "1", "1"};
    std::vector<Plato::srom::RandomRotations> tSetRandomRotations;
    Plato::srom::RandomRotations tRotations1;
    tRotations1.mRotations.mX = 62.92995363352; tRotations1.mRotations.mY = 0; tRotations1.mRotations.mZ = 0;
    tRotations1.mProbability = 0.3643018720139;
    tSetRandomRotations.push_back(tRotations1);
    Plato::srom::RandomRotations tRotations2;
    tRotations2.mRotations.mX = 69.67128118964; tRotations1.mRotations.mY = 0; tRotations1.mRotations.mZ = 0;
    tRotations2.mProbability = 0.1964474490484;
    tSetRandomRotations.push_back(tRotations2);
    Plato::srom::RandomRotations tRotations3;
    tRotations3.mRotations.mX = 66.03455388567; tRotations1.mRotations.mY = 0; tRotations1.mRotations.mZ = 0;
    tRotations3.mProbability = 0.2300630894941;
    tSetRandomRotations.push_back(tRotations3);
    Plato::srom::RandomRotations tRotations4;
    tRotations4.mRotations.mX = 96.25276276890; tRotations1.mRotations.mY = 0; tRotations1.mRotations.mZ = 0;
    tRotations4.mProbability = 0.2091697703866;
    tSetRandomRotations.push_back(tRotations4);

    std::vector<Plato::srom::RandomLoad> tSetRandomLoads;
    ASSERT_TRUE(Plato::generate_set_random_loads(tOriginalLoad, tSetRandomRotations, tSetRandomLoads));
    ASSERT_EQ(4u, tSetRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
        { {1, -0.435371385985053, 1.34553028812638}, {1, -0.590309206183812, 1.28512063289585},
          {1, -0.507604956861362, 1.31997621485002}, {1, -1.10296589510906, 0.885136280030518} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    const double tTolerance = 1e-6;
    for(size_t tIndex =0; tIndex < tSetRandomLoads.size(); tIndex++)
    {
        ASSERT_NEAR(tGoldProbabilities[tIndex], tSetRandomLoads[tIndex].mProbability, tTolerance);
        ASSERT_NEAR(tGoldLoads[tIndex][0], tSetRandomLoads[tIndex].mLoadValues.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tIndex][1], tSetRandomLoads[tIndex].mLoadValues.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tIndex][2], tSetRandomLoads[tIndex].mLoadValues.mZ, tTolerance);
    }
}

TEST(PlatoTest, generate_output_random_load_cases_errors)
{
    Plato::srom::Load tLoadDet1;
    tLoadDet1.mAppID = 3;
    tLoadDet1.mAppType = "sideset";
    tLoadDet1.mLoadID = 3;
    tLoadDet1.mLoadType = "pressure";
    tLoadDet1.mValues = {"1", "2", "3"};
    std::vector<Plato::srom::Load> tDeterministicLoads;
    tDeterministicLoads.push_back(tLoadDet1);

    // ERROR 1: EMPTY SET OF RANDOM LOAD CASES
    std::vector<Plato::srom::RandomLoadCase> tSetRandomLoadCases;
    ASSERT_FALSE(Plato::generate_output_random_load_cases(tDeterministicLoads, tSetRandomLoadCases));

    // ERROR 2: DETERMINISTIC LOAD PARAMETERS NOT DEFINED
    tDeterministicLoads[0].mAppType.clear(); // remove application type

    // SET INPUTS - CREATE SET OF RANDOM LOAD CASES
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tNewSetRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION
    ASSERT_TRUE(Plato::expand_random_load_cases(tNewSetRandomLoads, tSetRandomLoadCases));
    ASSERT_EQ(12u, tSetRandomLoadCases.size());

    ASSERT_FALSE(Plato::generate_output_random_load_cases(tDeterministicLoads, tSetRandomLoadCases));
}

TEST(PlatoTest, generate_output_random_load_cases)
{
    // SET INPUTS - CREATE SET OF DETERMINISTIC LOADS
    Plato::srom::Load tLoadDet1;
    tLoadDet1.mAppID = 3;
    tLoadDet1.mAppType = "sideset";
    tLoadDet1.mLoadID = 3;
    tLoadDet1.mLoadType = "pressure";
    tLoadDet1.mValues = {"1", "2", "3"};
    std::vector<Plato::srom::Load> tDeterministicLoads;
    tDeterministicLoads.push_back(tLoadDet1);

    // SET INPUTS - CREATE SET OF RANDOM LOAD CASES
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::srom::RandomLoad> tNewSetRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION
    std::vector<Plato::srom::RandomLoadCase> tSetRandomLoadCases;
    ASSERT_TRUE(Plato::expand_random_load_cases(tNewSetRandomLoads, tSetRandomLoadCases));
    ASSERT_EQ(12u, tSetRandomLoadCases.size());

    // CALL FUNCTION TO BE TESTED
    ASSERT_TRUE(Plato::generate_output_random_load_cases(tDeterministicLoads, tSetRandomLoadCases));

    // TEST OUTPUT
    std::vector<std::vector<std::vector<double>>> tGoldLoadCases =
    { { {0.609296803530850, 0.90460248117150, 1.34553028812638}, {1, 2, 3} }, { {0.343344489354037, 1.03521147857081, 1.34553028812638}, {1, 2, 3} },
      { {0.173488310131751, 1.07677762327334, 1.34553028812638}, {1, 2, 3} }, { {0.761671920821811, 0.87653912858296, 1.28512063289585}, {1, 2, 3} },
      { {0.497650027509830, 1.04919464782510, 1.28512063289585}, {1, 2, 3} }, { {0.323575806212112, 1.11524152385909, 1.28512063289585}, {1, 2, 3} },
      { {0.680335616523720, 0.89151906380038, 1.31997621485002}, {1, 2, 3} }, { {0.415283283979423, 1.04173057278621, 1.31997621485002}, {1, 2, 3} },
      { {0.243460611096492, 1.09470988077881, 1.31997621485002}, {1, 2, 3} }, { {1.265849164342930, 0.78368339200601, 0.88513628003052}, {1, 2, 3} },
      { {1.008214628020530, 1.09546201651137, 0.88513628003052}, {1, 2, 3} }, { {0.820183791860380, 1.24251048822264, 0.88513628003052}, {1, 2, 3} } };
    std::vector<double> tGoldLoadCasesProbs =
            {0.160857230304970, 0.118639480992335, 0.0847557373684797,  0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
             0.101584192161421, 0.0749229351531628, 0.0535247504591439, 0.0923587620945064, 0.0681187633233061, 0.0486638677597447};
    std::vector<int> tGoldLoadIDs = {1, 2, 3, 4, 5, 6 , 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
    std::vector<int> tGoldLoadCaseIDs = {1, 2, 3, 4, 5, 6 , 7, 8, 9, 10, 11, 12};

    const double tTolerance = 1e-6;
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tSetRandomLoadCases.size(); tLoadCaseIndex++)
    {
        const Plato::srom::RandomLoadCase& tLoadCase = tSetRandomLoadCases[tLoadCaseIndex];
        ASSERT_EQ(tGoldLoadCaseIDs[tLoadCaseIndex], tLoadCase.mLoadCaseID);
        ASSERT_NEAR(tGoldLoadCasesProbs[tLoadCaseIndex], tLoadCase.mProbability, tTolerance);
        const size_t tNumLoads = tLoadCase.mLoads.size();
        for(size_t tLoadIndex = 0; tLoadIndex < tNumLoads; tLoadIndex++)
        {
            size_t tIndex = tLoadCaseIndex * tNumLoads + tLoadIndex;
            ASSERT_EQ(tGoldLoadIDs[tIndex], tLoadCase.mLoads[tLoadIndex].mLoadID);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][0], tLoadCase.mLoads[tLoadIndex].mLoadValues.mX, tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][1], tLoadCase.mLoads[tLoadIndex].mLoadValues.mY, tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][2], tLoadCase.mLoads[tLoadIndex].mLoadValues.mZ, tTolerance);
        }
    }
}

TEST(PlatoTest, generate_load_sroms_error)
{
    Plato::srom::InputMetaData tInputs;
    Plato::srom::OutputMetaData tOutputs;
    ASSERT_FALSE(Plato::generate_load_sroms(tInputs, tOutputs));
}

TEST(PlatoTest, generate_load_sroms_both_random_and_deterministic_loads)
{
    // SET INPUTS - CREATE FIRST RANDOM LOAD
    Plato::srom::Load tLoad1;
    tLoad1.mAppID = 1;
    tLoad1.mAppType = "nodeset";
    tLoad1.mLoadID = 1;
    tLoad1.mLoadType = "traction";
    tLoad1.mValues = {"1", "1", "1"};
    Plato::srom::Variable tRandVar1;
    tRandVar1.mType = "random rotation";
    tRandVar1.mSubType = "x";
    tRandVar1.mStatistics.mMean = "85";
    tRandVar1.mStatistics.mNumSamples = "2";
    tRandVar1.mStatistics.mLowerBound = "65";
    tRandVar1.mStatistics.mUpperBound = "135";
    tRandVar1.mStatistics.mDistribution = "beta";
    tRandVar1.mStatistics.mStandardDeviation = "15";
    // APPEND FIRST RANDOM VARIABLE FOR LOAD 1
    tLoad1.mRandomVars.push_back(tRandVar1);
    Plato::srom::Variable tRandVar2;
    tRandVar2.mType = "random rotation";
    tRandVar2.mSubType = "y";
    tRandVar2.mStatistics.mMean = "70";
    tRandVar2.mStatistics.mNumSamples = "2";
    tRandVar2.mStatistics.mLowerBound = "50";
    tRandVar2.mStatistics.mUpperBound = "120";
    tRandVar2.mStatistics.mDistribution = "beta";
    tRandVar2.mStatistics.mStandardDeviation = "15";
    // APPEND SECOND RANDOM VARIABLE FOR LOAD 1
    tLoad1.mRandomVars.push_back(tRandVar2);

    // APPEND FIRST RANDOM LOAD TO INPUT META DATA
    Plato::srom::InputMetaData tInputs;
    tInputs.mLoads.push_back(tLoad1);

    // SET INPUTS - CREATE SECOND RANDOM LOAD
    Plato::srom::Load tLoad2;
    tLoad2.mAppID = 2;
    tLoad2.mAppType = "nodeset";
    tLoad2.mLoadID = 2;
    tLoad2.mLoadType = "traction";
    tLoad2.mValues = {"1", "1", "1"};
    tRandVar1.mType = "random rotation";
    tRandVar1.mSubType = "x";
    tRandVar1.mStatistics.mMean = "85";
    tRandVar1.mStatistics.mNumSamples = "2";
    tRandVar1.mStatistics.mLowerBound = "65";
    tRandVar1.mStatistics.mUpperBound = "135";
    tRandVar1.mStatistics.mDistribution = "beta";
    tRandVar1.mStatistics.mStandardDeviation = "15";
    // APPEND FIRST RANDOM VARIABLE FOR LOAD 2
    tLoad2.mRandomVars.push_back(tRandVar1);
    tRandVar2.mType = "random rotation";
    tRandVar2.mSubType = "y";
    tRandVar2.mStatistics.mMean = "70";
    tRandVar2.mStatistics.mNumSamples = "2";
    tRandVar2.mStatistics.mLowerBound = "50";
    tRandVar2.mStatistics.mUpperBound = "120";
    tRandVar2.mStatistics.mDistribution = "beta";
    tRandVar2.mStatistics.mStandardDeviation = "10";
    // APPEND SECOND RANDOM VARIABLE FOR LOAD 2
    tLoad2.mRandomVars.push_back(tRandVar2);

    // APPEND SECOND RANDOM LOAD TO INPUT META DATA
    tInputs.mLoads.push_back(tLoad2);

    // SET INPUTS - CREATE DETERMINISTIC LOADS
    Plato::srom::Load tLoad3;
    tLoad3.mAppID = 3;
    tLoad3.mAppType = "sideset";
    tLoad3.mLoadID = 3;
    tLoad3.mLoadType = "pressure";
    tLoad3.mValues = {"1", "2", "3"};

    // SET INPUTS - APPEND FORST DETERMINISTIC LOAD
    tInputs.mLoads.push_back(tLoad3);

    // CALL FUNCTION TO BE TESTED
    Plato::srom::OutputMetaData tOutputs;
    ASSERT_TRUE(Plato::generate_load_sroms(tInputs, tOutputs));
    ASSERT_EQ(16u, tOutputs.mLoadCases.size());

    // SET GOLD VALUES
    std::vector<int> tGoldLoadCaseIDs = {1, 2, 3, 4, 5, 6 , 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    std::vector<std::vector<std::vector<double>>> tGoldLoadCases =
    { { {1.571521544770223, -0.6912988161476783, -0.228967205328253}, {1.45459857379489, -0.6912988161476783, -0.6373766044550824}, {1, 2, 3} },
      { {1.243876404316842, -0.6912988161476783, -0.987358819060539}, {1.45459857379489, -0.6912988161476783, -0.6373766044550824}, {1, 2, 3} },
      { {1.129580691573818, -1.2174657500169410, -0.491756656039655}, {1.45459857379489, -0.6912988161476783, -0.6373766044550824}, {1, 2, 3} },
      { {0.7297339277702746, -1.217465750016941, -0.992605431275013}, {1.45459857379489, -0.6912988161476783, -0.6373766044550824}, {1, 2, 3} },
      { {1.571521544770223, -0.6912988161476783, -0.228967205328253}, {1.49410148829542, -0.6912988161476783, -0.5382998137341604}, {1, 2, 3} },
      { {1.243876404316842, -0.6912988161476783, -0.987358819060539}, {1.49410148829542, -0.6912988161476783, -0.5382998137341604}, {1, 2, 3} },
      { {1.129580691573818, -1.2174657500169410, -0.491756656039655}, {1.49410148829542, -0.6912988161476783, -0.5382998137341604}, {1, 2, 3} },
      { {0.7297339277702746, -1.217465750016941, -0.992605431275013}, {1.49410148829542, -0.6912988161476783, -0.5382998137341604}, {1, 2, 3} },
      { {1.571521544770223, -0.6912988161476783, -0.228967205328253}, {0.95880529000861, -1.2174657500169410, -0.7736081458897784}, {1, 2, 3} },
      { {1.243876404316842, -0.6912988161476783, -0.987358819060539}, {0.95880529000861, -1.2174657500169410, -0.7736081458897784}, {1, 2, 3} },
      { {1.129580691573818, -1.2174657500169410, -0.491756656039655}, {0.95880529000861, -1.2174657500169410, -0.7736081458897784}, {1, 2, 3} },
      { {0.7297339277702746, -1.217465750016941, -0.992605431275013}, {0.95880529000861, -1.2174657500169410, -0.7736081458897784}, {1, 2, 3} },
      { {1.571521544770223, -0.6912988161476783, -0.228967205328253}, {1.008570893282811, -1.217465750016941, -0.7075039934575635}, {1, 2, 3} },
      { {1.243876404316842, -0.6912988161476783, -0.987358819060539}, {1.008570893282811, -1.217465750016941, -0.7075039934575635}, {1, 2, 3} },
      { {1.129580691573818, -1.2174657500169410, -0.491756656039655}, {1.008570893282811, -1.217465750016941, -0.7075039934575635}, {1, 2, 3} },
      { {0.7297339277702746, -1.217465750016941, -0.992605431275013}, {1.008570893282811, -1.217465750016941, -0.7075039934575635}, {1, 2, 3} } };

    std::vector<double> tGoldLoadCasesProbs =
            {0.1271150547979226, 0.08266435562779866, 0.08266435562779866, 0.05375756398187734, 0.0953682783064585, 0.0620190683634493, 0.0620190683634493, 0.04033170052950105,
             0.0826643556277987, 0.05375756398187733, 0.05375756398187733, 0.03495915093413995, 0.0620190683634493, 0.0403317005295011, 0.0403317005295011, 0.02622816031464305};

    std::vector<std::vector<std::string>> tGoldAppType =
            { {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"},
              {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"},
              {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"},
              {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"} };

    std::vector<std::vector<std::string>> tGoldLoadType =
            { {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"},
              {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"},
              {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"},
              {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"} };

    std::vector<std::vector<int>> tGoldAppIDs =
            { {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3},
              {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3} };

    std::vector<int> tGoldLoadIDs = {1, 2, 3, 4, 5, 6 , 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
                                     25, 26 ,27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41 ,42 ,43, 44, 45, 46, 47, 48};

    // TEST OUTPUT
    double tSum = 0;
    double tTolerance = 1e-6;
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tOutputs.mLoadCases.size(); tLoadCaseIndex++)
    {
        const Plato::srom::RandomLoadCase& tLoadCase = tOutputs.mLoadCases[tLoadCaseIndex];
        tSum += tLoadCase.mProbability;
        ASSERT_EQ(tGoldLoadCaseIDs[tLoadCaseIndex], tLoadCase.mLoadCaseID);
        ASSERT_NEAR(tGoldLoadCasesProbs[tLoadCaseIndex], tLoadCase.mProbability, tTolerance);
        const size_t tNumLoads = tLoadCase.mLoads.size();
        for(size_t tLoadIndex = 0; tLoadIndex < tNumLoads; tLoadIndex++)
        {
            size_t tIndex = tLoadCaseIndex * tNumLoads + tLoadIndex;
            ASSERT_EQ(tGoldLoadIDs[tIndex], tLoadCase.mLoads[tLoadIndex].mLoadID);
            ASSERT_EQ(tGoldAppIDs[tLoadCaseIndex][tLoadIndex], tLoadCase.mLoads[tLoadIndex].mAppID);
            ASSERT_STREQ(tGoldAppType[tLoadCaseIndex][tLoadIndex].c_str(), tLoadCase.mLoads[tLoadIndex].mAppType.c_str());
            ASSERT_STREQ(tGoldLoadType[tLoadCaseIndex][tLoadIndex].c_str(), tLoadCase.mLoads[tLoadIndex].mLoadType.c_str());
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][0], tLoadCase.mLoads[tLoadIndex].mLoadValues.mX, tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][1], tLoadCase.mLoads[tLoadIndex].mLoadValues.mY, tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][2], tLoadCase.mLoads[tLoadIndex].mLoadValues.mZ, tTolerance);
        }
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, generate_load_sroms_both_random_and_deterministic_loads_from_parsed_data)
{
    XMLGen::Load tL1;
    tL1.app_id = "1";
    tL1.app_type = "nodeset";
    tL1.load_id = 1;
    tL1.type = "traction";
    tL1.x = "1";
    tL1.y = "1";
    tL1.z = "1";
    XMLGen::Load tL2;
    tL2.app_id = "2";
    tL2.app_type = "nodeset";
    tL2.load_id = 2;
    tL2.type = "traction";
    tL2.x = "1";
    tL2.y = "1";
    tL2.z = "1";
    XMLGen::Load tL3;
    tL3.app_id = "3";
    tL3.app_type = "sideset";
    tL3.load_id = 3;
    tL3.type = "pressure";
    tL3.x = "1";
    tL3.y = "2";
    tL3.z = "3";
    XMLGen::LoadCase tLC1;
    tLC1.id = "1";
    tLC1.loads.push_back(tL1);
    XMLGen::LoadCase tLC2;
    tLC2.id = "2";
    tLC2.loads.push_back(tL2);
    XMLGen::LoadCase tLC3;
    tLC3.id = "3";
    tLC3.loads.push_back(tL3);
    XMLGen::Uncertainty tU1;
    tU1.variable_type = "load";
    tU1.type = "random rotation";
    tU1.id = "1";
    tU1.axis = "x";
    tU1.distribution = "beta";
    tU1.mean = "85";
    tU1.upper = "135";
    tU1.lower = "65";
    tU1.standard_deviation = "15";
    tU1.num_samples = "2";
    XMLGen::Uncertainty tU2;
    tU2.variable_type = "load";
    tU2.type = "random rotation";
    tU2.id = "1";
    tU2.axis = "y";
    tU2.distribution = "beta";
    tU2.mean = "70";
    tU2.upper = "120";
    tU2.lower = "50";
    tU2.standard_deviation = "15";
    tU2.num_samples = "2";
    XMLGen::Uncertainty tU3;
    tU3.variable_type = "load";
    tU3.type = "random rotation";
    tU3.id = "2";
    tU3.axis = "x";
    tU3.distribution = "beta";
    tU3.mean = "85";
    tU3.upper = "135";
    tU3.lower = "65";
    tU3.standard_deviation = "15";
    tU3.num_samples = "2";
    XMLGen::Uncertainty tU4;
    tU4.variable_type = "load";
    tU4.type = "random rotation";
    tU4.id = "2";
    tU4.axis = "y";
    tU4.distribution = "beta";
    tU4.mean = "70";
    tU4.upper = "120";
    tU4.lower = "50";
    tU4.standard_deviation = "10";
    tU4.num_samples = "2";

    std::vector<XMLGen::LoadCase> tLoadCases;
    std::vector<XMLGen::Uncertainty> tUncertainties;
    tLoadCases.push_back(tLC1);
    tLoadCases.push_back(tLC2);
    tLoadCases.push_back(tLC3);
    tUncertainties.push_back(tU1);
    tUncertainties.push_back(tU2);
    tUncertainties.push_back(tU3);
    tUncertainties.push_back(tU4);

    std::vector<Plato::srom::Load> tLoads;
    Plato::generate_srom_load_inputs(tLoadCases, tUncertainties, tLoads);

    Plato::srom::InputMetaData tInputs;
    tInputs.mLoads = tLoads;


/*

    // SET INPUTS - CREATE FIRST RANDOM LOAD
    Plato::srom::Load tLoad1;
    tLoad1.mAppID = 1;
    tLoad1.mAppType = "nodeset";
    tLoad1.mLoadID = 1;
    tLoad1.mLoadType = "traction";
    tLoad1.mValues = {"1", "1", "1"};
    Plato::srom::Variable tRandVar1;
    tRandVar1.mType = "random rotation";
    tRandVar1.mSubType = "x";
    tRandVar1.mStatistics.mMean = "85";
    tRandVar1.mStatistics.mNumSamples = "2";
    tRandVar1.mStatistics.mLowerBound = "65";
    tRandVar1.mStatistics.mUpperBound = "135";
    tRandVar1.mStatistics.mDistribution = "beta";
    tRandVar1.mStatistics.mStandardDeviation = "15";
    // APPEND FIRST RANDOM VARIABLE FOR LOAD 1
    tLoad1.mRandomVars.push_back(tRandVar1);
    Plato::srom::Variable tRandVar2;
    tRandVar2.mType = "random rotation";
    tRandVar2.mSubType = "y";
    tRandVar2.mStatistics.mMean = "70";
    tRandVar2.mStatistics.mNumSamples = "2";
    tRandVar2.mStatistics.mLowerBound = "50";
    tRandVar2.mStatistics.mUpperBound = "120";
    tRandVar2.mStatistics.mDistribution = "beta";
    tRandVar2.mStatistics.mStandardDeviation = "15";
    // APPEND SECOND RANDOM VARIABLE FOR LOAD 1
    tLoad1.mRandomVars.push_back(tRandVar2);

    // APPEND FIRST RANDOM LOAD TO INPUT META DATA
    Plato::srom::InputMetaData tInputs;
    tInputs.mLoads.push_back(tLoad1);

    // SET INPUTS - CREATE SECOND RANDOM LOAD
    Plato::srom::Load tLoad2;
    tLoad2.mAppID = 2;
    tLoad2.mAppType = "nodeset";
    tLoad2.mLoadID = 2;
    tLoad2.mLoadType = "traction";
    tLoad2.mValues = {"1", "1", "1"};
    tRandVar1.mType = "random rotation";
    tRandVar1.mSubType = "x";
    tRandVar1.mStatistics.mMean = "85";
    tRandVar1.mStatistics.mNumSamples = "2";
    tRandVar1.mStatistics.mLowerBound = "65";
    tRandVar1.mStatistics.mUpperBound = "135";
    tRandVar1.mStatistics.mDistribution = "beta";
    tRandVar1.mStatistics.mStandardDeviation = "15";
    // APPEND FIRST RANDOM VARIABLE FOR LOAD 2
    tLoad2.mRandomVars.push_back(tRandVar1);
    tRandVar2.mType = "random rotation";
    tRandVar2.mSubType = "y";
    tRandVar2.mStatistics.mMean = "70";
    tRandVar2.mStatistics.mNumSamples = "2";
    tRandVar2.mStatistics.mLowerBound = "50";
    tRandVar2.mStatistics.mUpperBound = "120";
    tRandVar2.mStatistics.mDistribution = "beta";
    tRandVar2.mStatistics.mStandardDeviation = "10";
    // APPEND SECOND RANDOM VARIABLE FOR LOAD 2
    tLoad2.mRandomVars.push_back(tRandVar2);

    // APPEND SECOND RANDOM LOAD TO INPUT META DATA
    tInputs.mLoads.push_back(tLoad2);

    // SET INPUTS - CREATE DETERMINISTIC LOADS
    Plato::srom::Load tLoad3;
    tLoad3.mAppID = 3;
    tLoad3.mAppType = "sideset";
    tLoad3.mLoadID = 3;
    tLoad3.mLoadType = "pressure";
    tLoad3.mValues = {"1", "2", "3"};

    // SET INPUTS - APPEND FORST DETERMINISTIC LOAD
    tInputs.mLoads.push_back(tLoad3);
    */

    // CALL FUNCTION TO BE TESTED
    Plato::srom::OutputMetaData tOutputs;
    ASSERT_TRUE(Plato::generate_load_sroms(tInputs, tOutputs));
    ASSERT_EQ(16u, tOutputs.mLoadCases.size());

    // SET GOLD VALUES
    std::vector<int> tGoldLoadCaseIDs = {1, 2, 3, 4, 5, 6 , 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    std::vector<std::vector<std::vector<double>>> tGoldLoadCases =
    { { {1.571521544770223, -0.6912988161476783, -0.228967205328253}, {1.45459857379489, -0.6912988161476783, -0.6373766044550824}, {1, 2, 3} },
      { {1.243876404316842, -0.6912988161476783, -0.987358819060539}, {1.45459857379489, -0.6912988161476783, -0.6373766044550824}, {1, 2, 3} },
      { {1.129580691573818, -1.2174657500169410, -0.491756656039655}, {1.45459857379489, -0.6912988161476783, -0.6373766044550824}, {1, 2, 3} },
      { {0.7297339277702746, -1.217465750016941, -0.992605431275013}, {1.45459857379489, -0.6912988161476783, -0.6373766044550824}, {1, 2, 3} },
      { {1.571521544770223, -0.6912988161476783, -0.228967205328253}, {1.49410148829542, -0.6912988161476783, -0.5382998137341604}, {1, 2, 3} },
      { {1.243876404316842, -0.6912988161476783, -0.987358819060539}, {1.49410148829542, -0.6912988161476783, -0.5382998137341604}, {1, 2, 3} },
      { {1.129580691573818, -1.2174657500169410, -0.491756656039655}, {1.49410148829542, -0.6912988161476783, -0.5382998137341604}, {1, 2, 3} },
      { {0.7297339277702746, -1.217465750016941, -0.992605431275013}, {1.49410148829542, -0.6912988161476783, -0.5382998137341604}, {1, 2, 3} },
      { {1.571521544770223, -0.6912988161476783, -0.228967205328253}, {0.95880529000861, -1.2174657500169410, -0.7736081458897784}, {1, 2, 3} },
      { {1.243876404316842, -0.6912988161476783, -0.987358819060539}, {0.95880529000861, -1.2174657500169410, -0.7736081458897784}, {1, 2, 3} },
      { {1.129580691573818, -1.2174657500169410, -0.491756656039655}, {0.95880529000861, -1.2174657500169410, -0.7736081458897784}, {1, 2, 3} },
      { {0.7297339277702746, -1.217465750016941, -0.992605431275013}, {0.95880529000861, -1.2174657500169410, -0.7736081458897784}, {1, 2, 3} },
      { {1.571521544770223, -0.6912988161476783, -0.228967205328253}, {1.008570893282811, -1.217465750016941, -0.7075039934575635}, {1, 2, 3} },
      { {1.243876404316842, -0.6912988161476783, -0.987358819060539}, {1.008570893282811, -1.217465750016941, -0.7075039934575635}, {1, 2, 3} },
      { {1.129580691573818, -1.2174657500169410, -0.491756656039655}, {1.008570893282811, -1.217465750016941, -0.7075039934575635}, {1, 2, 3} },
      { {0.7297339277702746, -1.217465750016941, -0.992605431275013}, {1.008570893282811, -1.217465750016941, -0.7075039934575635}, {1, 2, 3} } };

    std::vector<double> tGoldLoadCasesProbs =
            {0.1271150547979226, 0.08266435562779866, 0.08266435562779866, 0.05375756398187734, 0.0953682783064585, 0.0620190683634493, 0.0620190683634493, 0.04033170052950105,
             0.0826643556277987, 0.05375756398187733, 0.05375756398187733, 0.03495915093413995, 0.0620190683634493, 0.0403317005295011, 0.0403317005295011, 0.02622816031464305};

    std::vector<std::vector<std::string>> tGoldAppType =
            { {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"},
              {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"},
              {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"},
              {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"} };

    std::vector<std::vector<std::string>> tGoldLoadType =
            { {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"},
              {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"},
              {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"},
              {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"} };

    std::vector<std::vector<int>> tGoldAppIDs =
            { {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3},
              {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3}, {1, 2, 3} };

    std::vector<int> tGoldLoadIDs = {1, 2, 3, 4, 5, 6 , 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
                                     25, 26 ,27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41 ,42 ,43, 44, 45, 46, 47, 48};

    // TEST OUTPUT
    double tSum = 0;
    double tTolerance = 1e-6;
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tOutputs.mLoadCases.size(); tLoadCaseIndex++)
    {
        const Plato::srom::RandomLoadCase& tLoadCase = tOutputs.mLoadCases[tLoadCaseIndex];
        tSum += tLoadCase.mProbability;
        ASSERT_EQ(tGoldLoadCaseIDs[tLoadCaseIndex], tLoadCase.mLoadCaseID);
        ASSERT_NEAR(tGoldLoadCasesProbs[tLoadCaseIndex], tLoadCase.mProbability, tTolerance);
        const size_t tNumLoads = tLoadCase.mLoads.size();
        for(size_t tLoadIndex = 0; tLoadIndex < tNumLoads; tLoadIndex++)
        {
            size_t tIndex = tLoadCaseIndex * tNumLoads + tLoadIndex;
            ASSERT_EQ(tGoldLoadIDs[tIndex], tLoadCase.mLoads[tLoadIndex].mLoadID);
            ASSERT_EQ(tGoldAppIDs[tLoadCaseIndex][tLoadIndex], tLoadCase.mLoads[tLoadIndex].mAppID);
            ASSERT_STREQ(tGoldAppType[tLoadCaseIndex][tLoadIndex].c_str(), tLoadCase.mLoads[tLoadIndex].mAppType.c_str());
            ASSERT_STREQ(tGoldLoadType[tLoadCaseIndex][tLoadIndex].c_str(), tLoadCase.mLoads[tLoadIndex].mLoadType.c_str());
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][0], tLoadCase.mLoads[tLoadIndex].mLoadValues.mX, tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][1], tLoadCase.mLoads[tLoadIndex].mLoadValues.mY, tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][2], tLoadCase.mLoads[tLoadIndex].mLoadValues.mZ, tTolerance);
        }
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, generate_load_sroms_only_random_loads)
{
    // SET INPUTS - CREATE FIRST RANDOM LOAD
    Plato::srom::Load tLoad1;
    tLoad1.mAppID = 1;
    tLoad1.mAppType = "nodeset";
    tLoad1.mLoadID = 1;
    tLoad1.mLoadType = "traction";
    tLoad1.mValues = {"1", "1", "1"};
    Plato::srom::Variable tRandVar1;
    tRandVar1.mType = "random rotation";
    tRandVar1.mSubType = "x";
    tRandVar1.mStatistics.mMean = "85";
    tRandVar1.mStatistics.mNumSamples = "2";
    tRandVar1.mStatistics.mLowerBound = "65";
    tRandVar1.mStatistics.mUpperBound = "135";
    tRandVar1.mStatistics.mDistribution = "beta";
    tRandVar1.mStatistics.mStandardDeviation = "15";
    // APPEND FIRST RANDOM VARIABLE FOR LOAD 1
    tLoad1.mRandomVars.push_back(tRandVar1);

    // APPEND FIRST RANDOM LOAD TO INPUT META DATA
    Plato::srom::InputMetaData tInputs;
    tInputs.mLoads.push_back(tLoad1);

    // SET INPUTS - CREATE SECOND RANDOM LOAD
    Plato::srom::Load tLoad2;
    tLoad2.mAppID = 2;
    tLoad2.mAppType = "sideset";
    tLoad2.mLoadID = 2;
    tLoad2.mLoadType = "pressure";
    tLoad2.mValues = {"1", "1", "1"};
    tRandVar1.mType = "random rotation";
    tRandVar1.mSubType = "x";
    tRandVar1.mStatistics.mMean = "85";
    tRandVar1.mStatistics.mNumSamples = "2";
    tRandVar1.mStatistics.mLowerBound = "65";
    tRandVar1.mStatistics.mUpperBound = "135";
    tRandVar1.mStatistics.mDistribution = "beta";
    tRandVar1.mStatistics.mStandardDeviation = "10";
    // APPEND FIRST RANDOM VARIABLE FOR LOAD 2
    tLoad2.mRandomVars.push_back(tRandVar1);

    // APPEND SECOND RANDOM LOAD TO INPUT META DATA
    tInputs.mLoads.push_back(tLoad2);

    // CALL FUNCTION TO BE TESTED
    Plato::srom::OutputMetaData tOutputs;
    ASSERT_TRUE(Plato::generate_load_sroms(tInputs, tOutputs));
    ASSERT_EQ(4u, tOutputs.mLoadCases.size());

    // SET GOLD VALUES
    std::vector<int> tGoldLoadCaseIDs = {1, 2, 3, 4};
    std::vector<std::vector<std::vector<double>>> tGoldLoadCases =
    { { {1.0, -0.691298816147678, 1.2337365791743464}, {1.0, -0.99362231248073574, 1.006337269577369} },
      { {1.0, -1.217465750016941, 0.7195673335662809}, {1.0, -0.99362231248073574, 1.006337269577369} },
      { {1.0, -0.691298816147678, 1.2337365791743464}, {1.0, -0.9238313790941014, 1.0707640183537599} },
      { {1.0, -1.217465750016941, 0.7195673335662809}, {1.0, -0.9238313790941014, 1.0707640183537599} } };

    std::vector<double> tGoldLoadCasesProbs = {0.3462045113834939, 0.22514070339228703, 0.25974050237448448, 0.16891218190764895};

    std::vector<std::vector<std::string>> tGoldAppType =
            { {"nodeset", "sideset"}, {"nodeset", "sideset"}, {"nodeset", "sideset"}, {"nodeset", "sideset"} };

    std::vector<std::vector<std::string>> tGoldLoadType =
            { {"traction", "pressure"}, {"traction", "pressure"}, {"traction", "pressure"}, {"traction", "pressure"} };

    std::vector<std::vector<int>> tGoldAppIDs = { {1, 2}, {1, 2}, {1, 2}, {1, 2} };

    std::vector<int> tGoldLoadIDs = {1, 2, 3, 4, 5, 6 , 7, 8};

    // TEST OUTPUT
    double tSum = 0;
    double tTolerance = 1e-6;
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tOutputs.mLoadCases.size(); tLoadCaseIndex++)
    {
        const Plato::srom::RandomLoadCase& tLoadCase = tOutputs.mLoadCases[tLoadCaseIndex];
        tSum += tLoadCase.mProbability;
        ASSERT_EQ(tGoldLoadCaseIDs[tLoadCaseIndex], tLoadCase.mLoadCaseID);
        ASSERT_NEAR(tGoldLoadCasesProbs[tLoadCaseIndex], tLoadCase.mProbability, tTolerance);
        const size_t tNumLoads = tLoadCase.mLoads.size();
        for(size_t tLoadIndex = 0; tLoadIndex < tNumLoads; tLoadIndex++)
        {
            size_t tIndex = tLoadCaseIndex * tNumLoads + tLoadIndex;
            ASSERT_EQ(tGoldLoadIDs[tIndex], tLoadCase.mLoads[tLoadIndex].mLoadID);
            ASSERT_EQ(tGoldAppIDs[tLoadCaseIndex][tLoadIndex], tLoadCase.mLoads[tLoadIndex].mAppID);
            ASSERT_STREQ(tGoldAppType[tLoadCaseIndex][tLoadIndex].c_str(), tLoadCase.mLoads[tLoadIndex].mAppType.c_str());
            ASSERT_STREQ(tGoldLoadType[tLoadCaseIndex][tLoadIndex].c_str(), tLoadCase.mLoads[tLoadIndex].mLoadType.c_str());
            EXPECT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][0], tLoadCase.mLoads[tLoadIndex].mLoadValues.mX, tTolerance);
            EXPECT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][1], tLoadCase.mLoads[tLoadIndex].mLoadValues.mY, tTolerance);
            EXPECT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][2], tLoadCase.mLoads[tLoadIndex].mLoadValues.mZ, tTolerance);
        }
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_load_cases)
{
    std::map<int, std::vector<int> > tOriginalToNewLoadCaseMap;
    std::vector<XMLGen::LoadCase> tLoadCases;
    XMLGen::LoadCase tLC1;
    XMLGen::Load tL1;
    tLC1.id = "2";
    tLC1.loads.push_back(tL1);
    tLC1.loads.push_back(tL1);
    tLC1.loads.push_back(tL1);
    tLoadCases.push_back(tLC1);
    tLC1.id = "4";
    tLC1.loads.clear();
    tLC1.loads.push_back(tL1);
    tLC1.loads.push_back(tL1);
    tLC1.loads.push_back(tL1);
    tLoadCases.push_back(tLC1);
    tLC1.id = "6";
    tLC1.loads.clear();
    tLC1.loads.push_back(tL1);
    tLoadCases.push_back(tLC1);
    tLC1.id = "10";
    tLC1.loads.clear();
    tLC1.loads.push_back(tL1);
    tLC1.loads.push_back(tL1);
    tLoadCases.push_back(tLC1);
    std::vector<XMLGen::LoadCase> tNewLoadCases;
    Plato::expand_load_cases(tLoadCases, tNewLoadCases, tOriginalToNewLoadCaseMap);
    ASSERT_EQ(tNewLoadCases.size(), 9);
    ASSERT_STREQ(tNewLoadCases[0].id.c_str(), "2");
    ASSERT_STREQ(tNewLoadCases[1].id.c_str(), "1");
    ASSERT_STREQ(tNewLoadCases[2].id.c_str(), "3");
    ASSERT_STREQ(tNewLoadCases[3].id.c_str(), "4");
    ASSERT_STREQ(tNewLoadCases[4].id.c_str(), "5");
    ASSERT_STREQ(tNewLoadCases[5].id.c_str(), "7");
    ASSERT_STREQ(tNewLoadCases[6].id.c_str(), "6");
    ASSERT_STREQ(tNewLoadCases[7].id.c_str(), "10");
    ASSERT_STREQ(tNewLoadCases[8].id.c_str(), "8");
    ASSERT_EQ(tOriginalToNewLoadCaseMap[2][0], 0);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[2][1], 1);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[2][2], 2);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[4][0], 3);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[4][1], 4);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[4][2], 5);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[6][0], 6);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[10][0], 7);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[10][1], 8);
}

TEST(PlatoTest, initialize_load_id_counter)
{
    std::vector<XMLGen::LoadCase> tLoadCases;
    XMLGen::LoadCase tLC1;
    tLC1.id = "3";
    tLoadCases.push_back(tLC1);
    Plato::UniqueCounter tUniqueLoadIDCounter;
    Plato::initialize_load_id_counter(tLoadCases, tUniqueLoadIDCounter);
    int tID = tUniqueLoadIDCounter.assignNextUnique();
    ASSERT_EQ(tID, 1);
    tID = tUniqueLoadIDCounter.assignNextUnique();
    ASSERT_EQ(tID, 2);
    tID = tUniqueLoadIDCounter.assignNextUnique();
    ASSERT_EQ(tID, 4);
}

TEST(PlatoTest, expand_single_load_case)
{
    // Check case of single load case with single load
    std::map<int, std::vector<int> > tOriginalToNewLoadCaseMap;
    XMLGen::LoadCase tOldLoadCase;
    tOldLoadCase.id = "88";
    XMLGen::Load tLoad1;
    tLoad1.app_id = "34";
    tLoad1.app_type = "nodeset";
    tLoad1.type = "traction";
    tLoad1.x = "0.0";
    tLoad1.y = "1.0";
    tLoad1.z = "33";
    tLoad1.scale = "24.5";
    tLoad1.load_id = "89";
    tOldLoadCase.loads.push_back(tLoad1);
    Plato::UniqueCounter tUniqueLoadIDCounter;
    tUniqueLoadIDCounter.mark(0);
    tUniqueLoadIDCounter.mark(88);
    std::vector<XMLGen::LoadCase> tNewLoadCaseList;
    Plato::expand_single_load_case(tOldLoadCase,tNewLoadCaseList,tUniqueLoadIDCounter,
                                   tOriginalToNewLoadCaseMap);
    ASSERT_EQ(tNewLoadCaseList.size(), 1);
    ASSERT_STREQ(tNewLoadCaseList[0].id.c_str(), "88");
    ASSERT_EQ(tNewLoadCaseList[0].loads.size(), 1);
    ASSERT_STREQ(tNewLoadCaseList[0].loads[0].load_id.c_str(), "89");
    ASSERT_STREQ(tNewLoadCaseList[0].loads[0].y.c_str(), "1.0");
    ASSERT_STREQ(tNewLoadCaseList[0].loads[0].x.c_str(), "0.0");
    ASSERT_STREQ(tNewLoadCaseList[0].loads[0].z.c_str(), "33");
    ASSERT_STREQ(tNewLoadCaseList[0].loads[0].type.c_str(), "traction");
    ASSERT_STREQ(tNewLoadCaseList[0].loads[0].app_type.c_str(), "nodeset");
    ASSERT_STREQ(tNewLoadCaseList[0].loads[0].app_id.c_str(), "34");
    ASSERT_STREQ(tNewLoadCaseList[0].loads[0].scale.c_str(), "24.5");
    ASSERT_EQ(tOriginalToNewLoadCaseMap[88][0], 0);
    // Check the case where a load case has more than one load
    XMLGen::Load tLoad2;
    tLoad2.app_id = "21";
    tLoad2.app_type = "sideset";
    tLoad2.type = "pressure";
    tLoad2.x = "44";
    tLoad2.y = "55";
    tLoad2.z = "66";
    tLoad2.scale = "12";
    tLoad2.load_id = "101";
    tOldLoadCase.loads.push_back(tLoad2);
    std::vector<XMLGen::LoadCase> tNewLoadCaseList2;
    tOriginalToNewLoadCaseMap.clear();
    Plato::expand_single_load_case(tOldLoadCase,tNewLoadCaseList2,tUniqueLoadIDCounter,
                                   tOriginalToNewLoadCaseMap);
    ASSERT_EQ(tNewLoadCaseList2.size(), 2);
    ASSERT_STREQ(tNewLoadCaseList2[1].id.c_str(), "1");
    ASSERT_STREQ(tNewLoadCaseList2[0].id.c_str(), "88");
    ASSERT_STREQ(tNewLoadCaseList2[1].loads[0].y.c_str(), "55");
    ASSERT_STREQ(tNewLoadCaseList2[1].loads[0].x.c_str(), "44");
    ASSERT_STREQ(tNewLoadCaseList2[1].loads[0].z.c_str(), "66");
    ASSERT_EQ(tOriginalToNewLoadCaseMap[88][0], 0);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[88][1], 1);
    // Check case where load case with multiple loads is is split and the ids of the resulting
    // load cases have to "straddle" original load case id
    XMLGen::LoadCase tLC1;
    tLC1.id = "2";
    XMLGen::Load tL1, tL2, tL3;
    tLC1.loads.push_back(tL1);
    tLC1.loads.push_back(tL2);
    tLC1.loads.push_back(tL3);
    Plato::UniqueCounter tIDCounter;
    tIDCounter.mark(0);
    tIDCounter.mark(2);
    std::vector<XMLGen::LoadCase> tNewList;
    tOriginalToNewLoadCaseMap.clear();
    Plato::expand_single_load_case(tLC1,tNewList,tIDCounter,
                                   tOriginalToNewLoadCaseMap);
    ASSERT_EQ(tNewList.size(), 3);
    ASSERT_STREQ(tNewList[0].id.c_str(), "2");
    ASSERT_STREQ(tNewList[1].id.c_str(), "1");
    ASSERT_STREQ(tNewList[2].id.c_str(), "3");
    ASSERT_EQ(tOriginalToNewLoadCaseMap[2][0], 0);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[2][1], 1);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[2][2], 2);
}

}
