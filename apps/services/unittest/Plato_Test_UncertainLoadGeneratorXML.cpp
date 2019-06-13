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

enum VariableType
{
    VT_LOAD,
    VT_MATERIAL
};

class Variable
{

public:

    Variable(){}
    virtual ~Variable(){}
    virtual VariableType variableType() = 0;

    // Get/set functions for member data
    bool isRandom() { return  mIsRandom; }
    void isRandom(const bool &aNewValue) { mIsRandom = aNewValue; }
    std::string type() { return mType; }
    void type(const std::string &aNewType) { mType = aNewType; }
    std::string subType() { return mSubType; }
    void subType(const std::string &aNewSubType) { mSubType = aNewSubType; }
    std::string globalID() { return mGlobalID; }
    void globalID(const std::string &aNewGlobalID) { mGlobalID = aNewGlobalID; }
    std::string distribution() { return mDistribution; }
    void distribution(const std::string &aNewDistribution) { mDistribution = aNewDistribution; }
    std::string mean() { return mMean; }
    void mean(const std::string &aNewMean) { mMean = aNewMean; }
    std::string upperBound() { return mUpperBound; }
    void upperBound(const std::string &aNewUpperBound) { mUpperBound = aNewUpperBound; }
    std::string lowerBound() { return mLowerBound; }
    void lowerBound(const std::string &aNewLowerBound) { mLowerBound = aNewLowerBound; }
    std::string standardDeviation() { return mStandardDeviation; }
    void standardDeviation(const std::string &aNewStandardDeviation) { mStandardDeviation = aNewStandardDeviation; }
    std::string numSamples() { return mNumSamples; }
    void numSamples(const std::string &aNewNumSamples) { mNumSamples = aNewNumSamples; }

private:

    bool mIsRandom; /*!< specifies whether this variable has uncertainty */
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

class SROMLoad : public Variable
{

public:

    SROMLoad(){}
    virtual ~SROMLoad(){}
    VariableType variableType() override { return VT_LOAD; }
    void x(const double &aX) { mX = aX; }
    double x() { return mX; }
    void y(const double &aY) { mY = aY; }
    double y() { return mY; }
    void z(const double &aZ) { mZ = aZ; }
    double z() { return mZ; }
    void appType(const std::string &aAppType) { mAppType = aAppType; }
    std::string appType() { return mAppType; }
    void loadType(const std::string &aType) { mLoadType = aType; }
    std::string loadType() { return mLoadType; }
    void appID(const int &aAppID) { mAppID = aAppID; }
    int appID() { return mAppID; }

private:

    double mX;
    double mY;
    double mZ;
    std::string mLoadType;
    std::string mAppType;
    int mAppID;

};

class SROMMaterial : public Variable
{

public:

    SROMMaterial(){}
    virtual ~SROMMaterial(){}
    virtual VariableType variableType() override { return VT_MATERIAL; }

};

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
    Plato::Vector3D mRotations; /*!< vector of random rotations, e.g. /f$(\theta_x, \theta_y, \theta_z)/f$ */
};

struct RandomLoad
{
    int mLoadID;           /*!< random load vector global identifier */
    double mProbability;   /*!< probability associated with this random load */
    Plato::Vector3D mMagnitude; /*!< vector of random loads, e.g. /f$(f_x, f_y, f_z)/f$ */
};

struct RandomLoadCase
{
    int mLoadCaseID; /*!< random load case global identifier */
    double mProbability; /*!< probability associated with this random load case */
    std::vector<Plato::RandomLoad> mLoads; /*!< set of random loads associated with this random load case */
};

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

inline void initialize_load_id_counter(const std::vector<LoadCase> &aLoadCases,
                                       Plato::UniqueCounter &aUniqueCounter)
{
    aUniqueCounter.mark(0); // Mark 0 as true since we don't want to have a 0 ID
    for(size_t i=0; i<aLoadCases.size(); ++i)
        aUniqueCounter.mark(std::atoi(aLoadCases[i].id.c_str()));
}

inline void expand_single_load_case(const LoadCase &aOldLoadCase,
                                    std::vector<LoadCase> &aNewLoadCaseList,
                                    Plato::UniqueCounter &aUniqueLoadIDCounter,
                                    std::map<int, std::vector<int> > &tOriginalToNewLoadCaseMap)
{
    int tOriginalLoadCaseID = std::atoi(aOldLoadCase.id.c_str());
    for(size_t j=0; j<aOldLoadCase.loads.size(); ++j)
    {
        std::string tIDString = aOldLoadCase.id;
        int tCurLoadCaseID = std::atoi(tIDString.c_str());
        // If this is a multi-load load case we need to generate
        // a new load case with a new id.
        if(j>0)
        {
            tCurLoadCaseID = aUniqueLoadIDCounter.assignNextUnique();
            tIDString = std::to_string(tCurLoadCaseID);
        }
        tOriginalToNewLoadCaseMap[tOriginalLoadCaseID].push_back(tCurLoadCaseID);
        LoadCase tNewLoadCase = aOldLoadCase;
        tNewLoadCase.id = tIDString;
        tNewLoadCase.loads[0] = aOldLoadCase.loads[j];
        tNewLoadCase.loads.resize(1);
        aNewLoadCaseList.push_back(tNewLoadCase);
    }
}

inline void expand_load_cases(const InputData &aInputData,
                              std::vector<LoadCase> &aNewLoadCaseList,
                              std::map<int, std::vector<int> > &tOriginalToNewLoadCaseMap)
{
    UniqueCounter tUniqueLoadIDCounter;
    initialize_load_id_counter(aInputData.load_cases, tUniqueLoadIDCounter);

    for(size_t i=0; i<aInputData.load_cases.size(); ++i)
        expand_single_load_case(aInputData.load_cases[i], aNewLoadCaseList, tUniqueLoadIDCounter,
                                tOriginalToNewLoadCaseMap);
}

inline void set_load_parameters(const LoadCase &aLoadCase,
                                std::shared_ptr<Plato::SROMLoad> &aLoad)
{
    aLoad->x(std::atof(aLoadCase.loads[0].x.c_str()));
    aLoad->y(std::atof(aLoadCase.loads[0].y.c_str()));
    aLoad->z(std::atof(aLoadCase.loads[0].z.c_str()));
    aLoad->loadType(aLoadCase.loads[0].type);
    aLoad->appType(aLoadCase.loads[0].app_type);
    aLoad->appID(std::atoi(aLoadCase.loads[0].app_id.c_str()));
}

inline std::shared_ptr<Plato::SROMLoad> create_deterministic_load_variable(const LoadCase &aLoadCase)
{
    std::shared_ptr<Plato::SROMLoad> tNewLoad = std::make_shared<Plato::SROMLoad>();
    tNewLoad->isRandom(false);
    tNewLoad->globalID(aLoadCase.id);
    set_load_parameters(aLoadCase, tNewLoad);
    return tNewLoad;
}

inline std::shared_ptr<Plato::SROMLoad> create_random_load_variable(const Uncertainty &aUncertainty,
                                                             const LoadCase &aLoadCase)
{
    std::shared_ptr<Plato::SROMLoad> tNewLoad = std::make_shared<Plato::SROMLoad>();
    tNewLoad->isRandom(true);
    tNewLoad->type(aUncertainty.type);
    tNewLoad->subType(aUncertainty.axis);
    tNewLoad->globalID(aLoadCase.id);
    tNewLoad->distribution(aUncertainty.distribution);
    tNewLoad->mean(aUncertainty.mean);
    tNewLoad->upperBound(aUncertainty.upper);
    tNewLoad->lowerBound(aUncertainty.lower);
    tNewLoad->standardDeviation(aUncertainty.standard_deviation);
    tNewLoad->numSamples(aUncertainty.num_samples);
    set_load_parameters(aLoadCase, tNewLoad);
    return tNewLoad;
}

inline void create_random_loads_from_uncertainty(const Uncertainty &aUncertainty,
                                                 const std::vector<LoadCase> &aNewLoadCases,
                                                 std::map<int, std::vector<int> > &aOriginalToNewLoadCaseMap,
                                                 std::set<int> &aUncertainLoads,
                                                 std::vector<std::shared_ptr<Plato::Variable> > &aSROMVariables)
{
    const int tUncertaintyLoadID = std::atoi(aUncertainty.id.c_str());
    for(size_t j=0; j<aOriginalToNewLoadCaseMap[tUncertaintyLoadID].size(); ++j)
    {
        const int tCurLoadCaseID = aOriginalToNewLoadCaseMap[tUncertaintyLoadID][j];
        std::string tCurLoadCaseIDString = std::to_string(tCurLoadCaseID);
        aUncertainLoads.insert(tCurLoadCaseID);
        for(size_t k=0; k<aNewLoadCases.size(); ++k)
        {
            if(aNewLoadCases[k].id == tCurLoadCaseIDString)
            {
                std::shared_ptr<Plato::Variable> tNewLoad =
                        create_random_load_variable(aUncertainty, aNewLoadCases[k]);
                aSROMVariables.push_back(tNewLoad);
                k=aNewLoadCases.size();
            }
        }
    }
}

inline void create_random_load_variables(const std::vector<Uncertainty> &aUncertainties,
                                         const std::vector<LoadCase> &aNewLoadCases,
                                         std::map<int, std::vector<int> > &aOriginalToNewLoadCaseMap,
                                         std::set<int> &aUncertainLoads,
                                         std::vector<std::shared_ptr<Plato::Variable> > &aSROMVariables)
{
    for(size_t i=0; i<aUncertainties.size(); ++i)
    {
        create_random_loads_from_uncertainty(aUncertainties[i], aNewLoadCases,
                                             aOriginalToNewLoadCaseMap,
                                             aUncertainLoads, aSROMVariables);
    }
}

inline void create_deterministic_load_variables(const std::vector<LoadCase> &aNewLoadCases,
                                                std::set<int> &aUncertainLoads,
                                                std::vector<std::shared_ptr<Plato::Variable> > &aSROMVariables)
{
    for(size_t i=0; i<aNewLoadCases.size(); ++i)
    {
        const int tCurLoadCaseID = std::atoi(aNewLoadCases[i].id.c_str());
        if(aUncertainLoads.find(tCurLoadCaseID) == aUncertainLoads.end())
        {
            std::shared_ptr<Plato::SROMLoad> tNewLoad = create_deterministic_load_variable(aNewLoadCases[i]);
            aSROMVariables.push_back(tNewLoad);
        }
    }
}

inline bool generateSROMInput(const InputData &aInputData,
                              std::vector<std::shared_ptr<Plato::Variable> > &aSROMVariables)
{
    std::map<int, std::vector<int> > tOriginalToNewLoadCaseMap;

    std::vector<LoadCase> tNewLoadCases;
    expand_load_cases(aInputData, tNewLoadCases, tOriginalToNewLoadCaseMap);

    std::set<int> tUncertainLoads;
    create_random_load_variables(aInputData.uncertainties, tNewLoadCases,
                                 tOriginalToNewLoadCaseMap, tUncertainLoads,
                                 aSROMVariables);

    create_deterministic_load_variables(tNewLoadCases, tUncertainLoads, aSROMVariables);

    return true;
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

    if(aMySampleProbPairs.size() != aRotationAxisToSampleProbPairs.size())
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: DUPLICATE RANDOM ROTATIONS WERE PROVIDED FOR A SINGLE LOAD."
                 << " RANDOM ROTATIONS ARE EXPECTED TO BE UNIQUE FOR A SINGLE LOAD.\n";
        return (false);
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
                tMyRandomRotations.mRotations.mX = aMyXaxisSampleProbPairs.mSamples[tIndexI];
                tMyRandomRotations.mRotations.mY = aMyYaxisSampleProbPairs.mSamples[tIndexJ];
                tMyRandomRotations.mRotations.mZ = aMyZaxisSampleProbPairs.mSamples[tIndexK];
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
            tMyRandomRotations.mRotations.mX = aMyXaxisSampleProbPairs.mSamples[tIndexI];
            tMyRandomRotations.mRotations.mY = aMyYaxisSampleProbPairs.mSamples[tIndexJ];
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
            tMyRandomRotations.mRotations.mX = aMyXaxisSampleProbPairs.mSamples[tIndexI];
            tMyRandomRotations.mRotations.mZ = aMyZaxisSampleProbPairs.mSamples[tIndexK];
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
            tMyRandomRotations.mRotations.mY = aMyYaxisSampleProbPairs.mSamples[tIndexJ];
            tMyRandomRotations.mRotations.mZ = aMyZaxisSampleProbPairs.mSamples[tIndexK];
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
        tMyRandomRotations.mRotations.mX = aMyXaxisSampleProbPairs.mSamples[tIndexI];
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
        tMyRandomRotations.mRotations.mY = aMyYaxisSampleProbPairs.mSamples[tIndexJ];
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
        tMyRandomRotations.mRotations.mZ = aMyZaxisSampleProbPairs.mSamples[tIndexK];
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

    const bool tRotateAboutX = !aMyXaxisSampleProbPairs.mSamples.empty();
    const bool tRotateAboutY = !aMyYaxisSampleProbPairs.mSamples.empty();
    const bool tRotateAboutZ = !aMyZaxisSampleProbPairs.mSamples.empty();

    if(tRotateAboutX && tRotateAboutY && tRotateAboutZ)
    {
        Plato::set_random_rotations_about_xyz(aMyXaxisSampleProbPairs, aMyYaxisSampleProbPairs, aMyZaxisSampleProbPairs, aMyRandomRotations);
    }
    else if(tRotateAboutX && tRotateAboutY && !tRotateAboutZ)
    {
        Plato::set_random_rotations_about_xy(aMyXaxisSampleProbPairs, aMyYaxisSampleProbPairs, aMyRandomRotations);
    }
    else if(tRotateAboutX && !tRotateAboutY && tRotateAboutZ)
    {
        Plato::set_random_rotations_about_xz(aMyXaxisSampleProbPairs, aMyZaxisSampleProbPairs, aMyRandomRotations);
    }
    else if(!tRotateAboutX && tRotateAboutY && tRotateAboutZ)
    {
        Plato::set_random_rotations_about_yz(aMyYaxisSampleProbPairs, aMyZaxisSampleProbPairs, aMyRandomRotations);
    }
    else if(tRotateAboutX && !tRotateAboutY && !tRotateAboutZ)
    {
        Plato::set_random_rotations_about_x(aMyXaxisSampleProbPairs, aMyRandomRotations);
    }
    else if(!tRotateAboutX && tRotateAboutY && !tRotateAboutZ)
    {
        Plato::set_random_rotations_about_y(aMyYaxisSampleProbPairs, aMyRandomRotations);
    }
    else if(!tRotateAboutX && !tRotateAboutY && tRotateAboutZ)
    {
        Plato::set_random_rotations_about_z(aMyZaxisSampleProbPairs, aMyRandomRotations);
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
                                             const std::vector<Plato::RandomRotations> & aMyRandomRotations)
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
                                const std::vector<Plato::RandomRotations> & aMyRandomRotations,
                                std::vector<Plato::RandomLoad> & aMyRandomLoads)
{
    if(Plato::check_expand_random_loads_inputs(aMyOriginalLoad, aMyRandomRotations) == false)
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: EXPAND RANDOM LOADS INPUTS ARE NOT VALID.\n";
        return (false);
    }

    const size_t tNumRandomLoads = aMyRandomRotations.size();
    for(size_t tIndex = 0; tIndex < tNumRandomLoads; tIndex++)
    {
        Plato::RandomLoad tMyRandomLoad;
        tMyRandomLoad.mMagnitude = aMyOriginalLoad;
        tMyRandomLoad.mProbability = aMyRandomRotations[tIndex].mProbability;
        if(Plato::apply_rotation_matrix(aMyRandomRotations[tIndex].mRotations, tMyRandomLoad.mMagnitude) == false)
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

inline bool update_initial_random_load_case(const std::vector<Plato::RandomLoad> & aNewSetRandomLoads,
                                            std::vector<Plato::RandomLoadCase> & aOldRandomLoadCases)
{
    std::vector<Plato::RandomLoadCase> tNewSetRandomLoadCase;

    for(size_t tLoadIndex = 0; tLoadIndex < aNewSetRandomLoads.size(); tLoadIndex++)
    {
        Plato::RandomLoadCase tNewRandomLoadCase;
        const Plato::RandomLoad& tMyNewRandomLoad = aNewSetRandomLoads[tLoadIndex];
        tNewRandomLoadCase.mLoads.push_back(tMyNewRandomLoad);
        tNewRandomLoadCase.mProbability = tMyNewRandomLoad.mProbability;
        tNewSetRandomLoadCase.push_back(tNewRandomLoadCase);
    } // index over new random loads

    aOldRandomLoadCases = tNewSetRandomLoadCase;

    return (true);
}

inline bool update_random_load_cases(const std::vector<Plato::RandomLoad> & aNewSetRandomLoads,
                                     std::vector<Plato::RandomLoadCase> & aOldRandomLoadCases)
{
    std::vector<Plato::RandomLoadCase> tNewSetRandomLoadCase;

    for(size_t tNewLoadIndex = 0; tNewLoadIndex < aNewSetRandomLoads.size(); tNewLoadIndex++)
    {
        const Plato::RandomLoad& tMyNewRandomLoad = aNewSetRandomLoads[tNewLoadIndex];
        std::vector<Plato::RandomLoadCase> tTempRandomLoadCases = aOldRandomLoadCases;
        for(size_t tOldLoadCaseIndex = 0; tOldLoadCaseIndex < aOldRandomLoadCases.size(); tOldLoadCaseIndex++)
        {
            tTempRandomLoadCases[tOldLoadCaseIndex].mLoads.push_back(tMyNewRandomLoad);
            Plato::RandomLoadCase& tMyTempRandomLoadCase = tTempRandomLoadCases[tOldLoadCaseIndex];
            tMyTempRandomLoadCase.mProbability = tMyTempRandomLoadCase.mProbability * tMyNewRandomLoad.mProbability;
            tNewSetRandomLoadCase.push_back(tMyTempRandomLoadCase);
        } // index over old random load cases
    } // index over new random loads

    aOldRandomLoadCases = tNewSetRandomLoadCase;

    return (true);
}

inline bool expand_random_load_cases(const std::vector<Plato::RandomLoad> & aNewSetRandomLoads,
                                     std::vector<Plato::RandomLoadCase> & aOldRandomLoadCases)
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

inline bool set_random_load_case_id(std::vector<Plato::RandomLoadCase> & aRandomLoadCases)
{
    if(aRandomLoadCases.empty())
    {
        std::cout<< "\nFILE: " << __FILE__
                 << "\nFUNCTION: " << __PRETTY_FUNCTION__
                 << "\nLINE:" << __LINE__
                 << "\nMESSAGE: INPUT SET OF RANDOM LOAD CASES IS EMPTY.\n";
        return (false);
    } // if statement

    Plato::UniqueCounter tCounter;
    for(size_t tIndex = 0; tIndex < aRandomLoadCases.size(); tIndex++)
    {
        aRandomLoadCases[tIndex].mLoadCaseID = tCounter.assignNextUnique();
    } // for-loop

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

TEST(PlatoTest, post_process_random_load_error)
{
    // ERROR: ZERO INPUTS PROVIDED
    std::vector<Plato::SromRandomVariable> tSromRandomVariableSet;
    std::map<Plato::axis3D::axis3D, Plato::SampleProbabilityPairs> tRotationAxisToSampleProbPairs;
    ASSERT_FALSE(Plato::post_process_random_load(tSromRandomVariableSet, tRotationAxisToSampleProbPairs));

    // ERROR: INPUT RANDOM ROTATIONS ARE NOT UNIQUE
    Plato::SromRandomVariable tRandomLoadX;
    tRandomLoadX.mType = "random rotation";
    tRandomLoadX.mSubType = "x";
    tRandomLoadX.mSampleProbPair.mNumSamples = 4;
    tRandomLoadX.mSampleProbPair.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tRandomLoadX.mSampleProbPair.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    Plato::SromRandomVariable tRandomLoadY;
    tRandomLoadY.mType = "random rotation";
    tRandomLoadY.mSubType = "x"; // ERROR - OVERWRITES ORIGINAL INPUT
    tRandomLoadY.mSampleProbPair.mNumSamples = 3;
    tRandomLoadY.mSampleProbPair.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tRandomLoadY.mSampleProbPair.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    tSromRandomVariableSet.push_back(tRandomLoadX);
    tSromRandomVariableSet.push_back(tRandomLoadY);
    ASSERT_FALSE(Plato::post_process_random_load(tSromRandomVariableSet, tRotationAxisToSampleProbPairs));
}

TEST(PlatoTest, post_process_random_load_OneRandomRotation)
{
    Plato::SromRandomVariable tRandomLoadX;
    tRandomLoadX.mType = "random rotation";
    tRandomLoadX.mSubType = "x";
    tRandomLoadX.mSampleProbPair.mNumSamples = 4;
    tRandomLoadX.mSampleProbPair.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tRandomLoadX.mSampleProbPair.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    std::vector<Plato::SromRandomVariable> tSromRandomVariableSet;
    tSromRandomVariableSet.push_back(tRandomLoadX);

    std::map<Plato::axis3D::axis3D, Plato::SampleProbabilityPairs> tRotationAxisToSampleProbPairs;
    ASSERT_TRUE(Plato::post_process_random_load(tSromRandomVariableSet, tRotationAxisToSampleProbPairs));

    // TEST RESULTS
    ASSERT_EQ(1u, tRotationAxisToSampleProbPairs.size());
    ASSERT_FALSE(tRotationAxisToSampleProbPairs.find(Plato::axis3D::x) == tRotationAxisToSampleProbPairs.end());
    ASSERT_TRUE(tRotationAxisToSampleProbPairs.find(Plato::axis3D::y) == tRotationAxisToSampleProbPairs.end());
    ASSERT_TRUE(tRotationAxisToSampleProbPairs.find(Plato::axis3D::z) == tRotationAxisToSampleProbPairs.end());

    const Plato::SampleProbabilityPairs& tSampleProbabilityPairs = tRotationAxisToSampleProbPairs.find(Plato::axis3D::x)->second;
    ASSERT_EQ(4u, tSampleProbabilityPairs.mNumSamples);

    const double tTolerance = 1e-4;
    for(size_t tIndex = 0; tIndex < tSampleProbabilityPairs.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadX.mSampleProbPair.mSamples[tIndex], tSampleProbabilityPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadX.mSampleProbPair.mProbabilities[tIndex], tSampleProbabilityPairs.mProbabilities[tIndex], tTolerance);
    }
}

TEST(PlatoTest, post_process_random_load_TwoRandomRotations)
{
    Plato::SromRandomVariable tRandomLoadX;
    tRandomLoadX.mType = "random rotation";
    tRandomLoadX.mSubType = "x";
    tRandomLoadX.mSampleProbPair.mNumSamples = 4;
    tRandomLoadX.mSampleProbPair.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tRandomLoadX.mSampleProbPair.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    Plato::SromRandomVariable tRandomLoadY;
    tRandomLoadY.mType = "random rotation";
    tRandomLoadY.mSubType = "y";
    tRandomLoadY.mSampleProbPair.mNumSamples = 3;
    tRandomLoadY.mSampleProbPair.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tRandomLoadY.mSampleProbPair.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    std::vector<Plato::SromRandomVariable> tSromRandomVariableSet;
    tSromRandomVariableSet.push_back(tRandomLoadX);
    tSromRandomVariableSet.push_back(tRandomLoadY);

    std::map<Plato::axis3D::axis3D, Plato::SampleProbabilityPairs> tRotationAxisToSampleProbPairs;
    ASSERT_TRUE(Plato::post_process_random_load(tSromRandomVariableSet, tRotationAxisToSampleProbPairs));

    // TEST RESULTS
    ASSERT_EQ(2u, tRotationAxisToSampleProbPairs.size());
    ASSERT_FALSE(tRotationAxisToSampleProbPairs.find(Plato::axis3D::x) == tRotationAxisToSampleProbPairs.end());
    ASSERT_FALSE(tRotationAxisToSampleProbPairs.find(Plato::axis3D::y) == tRotationAxisToSampleProbPairs.end());
    ASSERT_TRUE(tRotationAxisToSampleProbPairs.find(Plato::axis3D::z) == tRotationAxisToSampleProbPairs.end());

    // TEST RESULTS FOR ROTATION X SAMPLE-PROBABILITY PAIRS
    const double tTolerance = 1e-4;

    const Plato::SampleProbabilityPairs& tSampleProbabilityPairsX = tRotationAxisToSampleProbPairs.find(Plato::axis3D::x)->second;
    ASSERT_EQ(tRandomLoadX.mSampleProbPair.mNumSamples, tSampleProbabilityPairsX.mNumSamples);
    for(size_t tIndex = 0; tIndex < tSampleProbabilityPairsX.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadX.mSampleProbPair.mSamples[tIndex], tSampleProbabilityPairsX.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadX.mSampleProbPair.mProbabilities[tIndex], tSampleProbabilityPairsX.mProbabilities[tIndex], tTolerance);
    }

    // TEST RESULTS FOR ROTATION Y SAMPLE-PROBABILITY PAIRS
    const Plato::SampleProbabilityPairs& tSampleProbabilityPairsY = tRotationAxisToSampleProbPairs.find(Plato::axis3D::y)->second;
    ASSERT_EQ(tRandomLoadY.mSampleProbPair.mNumSamples, tSampleProbabilityPairsY.mNumSamples);
    for(size_t tIndex = 0; tIndex < tSampleProbabilityPairsY.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadY.mSampleProbPair.mSamples[tIndex], tSampleProbabilityPairsY.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadY.mSampleProbPair.mProbabilities[tIndex], tSampleProbabilityPairsY.mProbabilities[tIndex], tTolerance);
    }
}

TEST(PlatoTest, post_process_random_load_ThreeRandomRotations)
{
    Plato::SromRandomVariable tRandomLoadX;
    tRandomLoadX.mType = "random rotation";
    tRandomLoadX.mSubType = "x";
    tRandomLoadX.mSampleProbPair.mNumSamples = 4;
    tRandomLoadX.mSampleProbPair.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tRandomLoadX.mSampleProbPair.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    Plato::SromRandomVariable tRandomLoadY;
    tRandomLoadY.mType = "random rotation";
    tRandomLoadY.mSubType = "y";
    tRandomLoadY.mSampleProbPair.mNumSamples = 3;
    tRandomLoadY.mSampleProbPair.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tRandomLoadY.mSampleProbPair.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    Plato::SromRandomVariable tRandomLoadZ;
    tRandomLoadZ.mType = "random rotation";
    tRandomLoadZ.mSubType = "z";
    tRandomLoadZ.mSampleProbPair.mNumSamples = 4;
    tRandomLoadZ.mSampleProbPair.mSamples = {10, 13.333333333333, 16.666666666667, 20.0};
    tRandomLoadZ.mSampleProbPair.mProbabilities = {0.25, 0.25, 0.25, 0.25};

    std::vector<Plato::SromRandomVariable> tSromRandomVariableSet;
    tSromRandomVariableSet.push_back(tRandomLoadX);
    tSromRandomVariableSet.push_back(tRandomLoadY);
    tSromRandomVariableSet.push_back(tRandomLoadZ);

    std::map<Plato::axis3D::axis3D, Plato::SampleProbabilityPairs> tRotationAxisToSampleProbPairs;
    ASSERT_TRUE(Plato::post_process_random_load(tSromRandomVariableSet, tRotationAxisToSampleProbPairs));

    // TEST RESULTS
    ASSERT_EQ(3u, tRotationAxisToSampleProbPairs.size());
    ASSERT_FALSE(tRotationAxisToSampleProbPairs.find(Plato::axis3D::x) == tRotationAxisToSampleProbPairs.end());
    ASSERT_FALSE(tRotationAxisToSampleProbPairs.find(Plato::axis3D::y) == tRotationAxisToSampleProbPairs.end());
    ASSERT_FALSE(tRotationAxisToSampleProbPairs.find(Plato::axis3D::z) == tRotationAxisToSampleProbPairs.end());

    // TEST RESULTS FOR ROTATION X SAMPLE-PROBABILITY PAIRS
    const double tTolerance = 1e-4;

    const Plato::SampleProbabilityPairs& tSampleProbabilityPairsX = tRotationAxisToSampleProbPairs.find(Plato::axis3D::x)->second;
    ASSERT_EQ(tRandomLoadX.mSampleProbPair.mNumSamples, tSampleProbabilityPairsX.mNumSamples);
    for(size_t tIndex = 0; tIndex < tSampleProbabilityPairsX.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadX.mSampleProbPair.mSamples[tIndex], tSampleProbabilityPairsX.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadX.mSampleProbPair.mProbabilities[tIndex], tSampleProbabilityPairsX.mProbabilities[tIndex], tTolerance);
    }

    // TEST RESULTS FOR ROTATION Y SAMPLE-PROBABILITY PAIRS
    const Plato::SampleProbabilityPairs& tSampleProbabilityPairsY = tRotationAxisToSampleProbPairs.find(Plato::axis3D::y)->second;
    ASSERT_EQ(tRandomLoadY.mSampleProbPair.mNumSamples, tSampleProbabilityPairsY.mNumSamples);
    for(size_t tIndex = 0; tIndex < tSampleProbabilityPairsY.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadY.mSampleProbPair.mSamples[tIndex], tSampleProbabilityPairsY.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadY.mSampleProbPair.mProbabilities[tIndex], tSampleProbabilityPairsY.mProbabilities[tIndex], tTolerance);
    }

    // TEST RESULTS FOR ROTATION Z SAMPLE-PROBABILITY PAIRS
    const Plato::SampleProbabilityPairs& tSampleProbabilityPairsZ = tRotationAxisToSampleProbPairs.find(Plato::axis3D::z)->second;
    ASSERT_EQ(tRandomLoadZ.mSampleProbPair.mNumSamples, tSampleProbabilityPairsZ.mNumSamples);
    for(size_t tIndex = 0; tIndex < tSampleProbabilityPairsZ.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadZ.mSampleProbPair.mSamples[tIndex], tSampleProbabilityPairsZ.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadZ.mSampleProbPair.mProbabilities[tIndex], tSampleProbabilityPairsZ.mProbabilities[tIndex], tTolerance);
    }
}

TEST(PlatoTest, expand_random_rotations_about_xyz)
{
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    tSampleProbPairSetY.mNumSamples = 3;
    tSampleProbPairSetY.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetY.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 4;
    tSampleProbPairSetZ.mProbabilities = {0.25, 0.25, 0.25, 0.25};
    tSampleProbPairSetZ.mSamples = {10, 13.333333333333, 16.666666666667, 20.0};

    std::vector<Plato::RandomRotations> tRandomRotationSet;
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
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    tSampleProbPairSetY.mNumSamples = 3;
    tSampleProbPairSetY.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetY.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // CALL FUNCTION
    std::vector<Plato::RandomRotations> tRandomRotationSet;
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
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // CALL FUNCTION
    std::vector<Plato::RandomRotations> tRandomRotationSet;
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
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // CALL FUNCTION
    std::vector<Plato::RandomRotations> tRandomRotationSet;
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
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // CALL FUNCTION
    std::vector<Plato::RandomRotations> tRandomRotationSet;
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
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // CALL FUNCTION
    std::vector<Plato::RandomRotations> tRandomRotationSet;
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
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 4;
    tSampleProbPairSetZ.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetZ.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // CALL FUNCTION
    std::vector<Plato::RandomRotations> tRandomRotationSet;
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
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    std::vector<Plato::RandomRotations> tRandomRotationSet;
    ASSERT_FALSE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));
}

TEST(PlatoTest, expand_random_loads_about_z)
{
    // SET INPUTS
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 4;
    tSampleProbPairSetZ.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetZ.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::RandomLoad> tMyRandomLoads;
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
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mMagnitude.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mMagnitude.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mMagnitude.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_y)
{
    // SET INPUTS
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::RandomLoad> tMyRandomLoads;
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
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mMagnitude.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mMagnitude.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mMagnitude.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_x)
{
    // SET INPUTS
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::RandomLoad> tMyRandomLoads;
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
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mMagnitude.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mMagnitude.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mMagnitude.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_yz)
{
    // SET INPUTS
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::RandomLoad> tMyRandomLoads;
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
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mMagnitude.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mMagnitude.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mMagnitude.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_xz)
{
    // SET INPUTS
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::RandomLoad> tMyRandomLoads;
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
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mMagnitude.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mMagnitude.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mMagnitude.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_xy)
{
    // SET INPUTS
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetY.mNumSamples = 3;
    tSampleProbPairSetY.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetY.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::RandomLoad> tMyRandomLoads;
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
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mMagnitude.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mMagnitude.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mMagnitude.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_xyz)
{
    // SET INPUTS
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
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
    std::vector<Plato::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::RandomLoad> tMyRandomLoads;
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
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mMagnitude.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mMagnitude.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mMagnitude.mZ, tTolerance);
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
    std::vector<Plato::RandomLoad> tMyRandomLoads;
    std::vector<Plato::RandomRotations> tMyRandomRotationsSet;
    ASSERT_FALSE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));
}

TEST(PlatoTest, expand_random_loads_error_2)
{
    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED DUE TO NaN X-COMPONENT
    Plato::Vector3D tMyOriginalLoad(std::numeric_limits<double>::quiet_NaN(), 1, 1);
    std::vector<Plato::RandomRotations> tMyRandomRotationsSet;
    Plato::RandomRotations tMyRotation;
    tMyRotation.mRotations.mX = 0;
    tMyRotation.mRotations.mY = 0;
    tMyRotation.mRotations.mZ = 62.92995363352;
    tMyRandomRotationsSet.push_back(tMyRotation);
    std::vector<Plato::RandomLoad> tMyRandomLoads;
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
    std::vector<Plato::RandomLoad> tMyRandomLoads;
    std::vector<Plato::RandomRotations> tMyRandomRotationsSet;
    Plato::RandomRotations tMyRotation;
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
    std::vector<Plato::RandomLoad> tNewSetRandomLoads;
    std::vector<Plato::RandomLoadCase> tOldRandomLoadCases;
    ASSERT_FALSE(Plato::expand_random_load_cases(tNewSetRandomLoads, tOldRandomLoadCases));
}

TEST(PlatoTest, expand_random_load_cases_one_load)
{
    // SET INPUTS
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::RandomLoad> tNewSetRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION - FUNCTION BEING TESTED
    std::vector<Plato::RandomLoadCase> tRandomLoadCases;
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
        const Plato::RandomLoadCase& tRandomLoadCase = tRandomLoadCases[tLoadCaseIndex];
        const Plato::RandomLoad& tRandomLoad = tRandomLoadCase.mLoads[0];

        ASSERT_NEAR(tGoldLoads[tLoadCaseIndex][0], tRandomLoad.mMagnitude.mX, tTolerance);
        ASSERT_NEAR(tGoldLoads[tLoadCaseIndex][1], tRandomLoad.mMagnitude.mY, tTolerance);
        ASSERT_NEAR(tGoldLoads[tLoadCaseIndex][2], tRandomLoad.mMagnitude.mZ, tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tLoadCaseIndex], tRandomLoad.mProbability, tTolerance);
        tSum += tRandomLoadCase.mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_load_cases_two_load)
{
    // *** SET FIRST LOAD CASE ***
    Plato::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    Plato::Vector3D tMyOriginalLoad(1, 1, 1);
    std::vector<Plato::RandomLoad> tNewSetRandomLoads;
    ASSERT_TRUE(Plato::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION - FUNCTION BEING TESTED
    std::vector<Plato::RandomLoadCase> tRandomLoadCases;
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
    tRandomLoadCases;
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
        const Plato::RandomLoadCase& tRandomLoadCase = tRandomLoadCases[tLoadCaseIndex];
        ASSERT_NEAR(tGoldProbabilities[tLoadCaseIndex], tRandomLoadCase.mProbability, tTolerance);
        for(size_t tLoadIndex = 0; tLoadIndex < tRandomLoadCase.mLoads.size(); tLoadIndex++)
        {
            const Plato::RandomLoad& tRandomLoad = tRandomLoadCase.mLoads[tLoadIndex];
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][0], tRandomLoad.mMagnitude.mX, tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][1], tRandomLoad.mMagnitude.mY, tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][2], tRandomLoad.mMagnitude.mZ, tTolerance);
        }
        tSum += tRandomLoadCase.mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);

    // SET RANDOM LOAD CASES IDS - FUNCTION BEING TESTED
    ASSERT_TRUE(Plato::set_random_load_case_id(tRandomLoadCases));
    std::vector<size_t> tGoldLoadCaseIDs =
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
     24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47};
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tRandomLoadCases.size(); tLoadCaseIndex++)
    {
        const Plato::RandomLoadCase& tRandomLoadCase = tRandomLoadCases[tLoadCaseIndex];
        ASSERT_EQ(tGoldLoadCaseIDs[tLoadCaseIndex], tRandomLoadCase.mLoadCaseID);
    }
}

TEST(PlatoTest, set_random_load_case_id_error)
{
    std::vector<Plato::RandomLoadCase> tRandomLoadCases;
    ASSERT_FALSE(Plato::set_random_load_case_id(tRandomLoadCases));
}

TEST(PlatoTest, expand_load_cases)
{
    std::map<int, std::vector<int> > tOriginalToNewLoadCaseMap;
    InputData tInputData;
    LoadCase tLC1;
    Load tL1;
    tLC1.id = "2";
    tLC1.loads.push_back(tL1);
    tLC1.loads.push_back(tL1);
    tLC1.loads.push_back(tL1);
    tInputData.load_cases.push_back(tLC1);
    tLC1.id = "4";
    tLC1.loads.clear();
    tLC1.loads.push_back(tL1);
    tLC1.loads.push_back(tL1);
    tLC1.loads.push_back(tL1);
    tInputData.load_cases.push_back(tLC1);
    tLC1.id = "6";
    tLC1.loads.clear();
    tLC1.loads.push_back(tL1);
    tInputData.load_cases.push_back(tLC1);
    tLC1.id = "10";
    tLC1.loads.clear();
    tLC1.loads.push_back(tL1);
    tLC1.loads.push_back(tL1);
    tInputData.load_cases.push_back(tLC1);
    std::vector<LoadCase> tNewLoadCases;
    Plato::expand_load_cases(tInputData, tNewLoadCases, tOriginalToNewLoadCaseMap);
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
    ASSERT_EQ(tOriginalToNewLoadCaseMap[2][0], 2);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[2][1], 1);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[2][2], 3);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[4][0], 4);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[4][1], 5);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[4][2], 7);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[6][0], 6);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[10][0], 10);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[10][1], 8);
}

TEST(PlatoTest, initialize_load_id_counter)
{
    std::vector<LoadCase> tLoadCases;
    LoadCase tLC1;
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
    LoadCase tOldLoadCase;
    tOldLoadCase.id = "88";
    Load tLoad1;
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
    std::vector<LoadCase> tNewLoadCaseList;
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
    ASSERT_EQ(tOriginalToNewLoadCaseMap[88][0], 88);
    // Check the case where a load case has more than one load
    Load tLoad2;
    tLoad2.app_id = "21";
    tLoad2.app_type = "sideset";
    tLoad2.type = "pressure";
    tLoad2.x = "44";
    tLoad2.y = "55";
    tLoad2.z = "66";
    tLoad2.scale = "12";
    tLoad2.load_id = "101";
    tOldLoadCase.loads.push_back(tLoad2);
    std::vector<LoadCase> tNewLoadCaseList2;
    tOriginalToNewLoadCaseMap.clear();
    Plato::expand_single_load_case(tOldLoadCase,tNewLoadCaseList2,tUniqueLoadIDCounter,
                                   tOriginalToNewLoadCaseMap);
    ASSERT_EQ(tNewLoadCaseList2.size(), 2);
    ASSERT_STREQ(tNewLoadCaseList2[1].id.c_str(), "1");
    ASSERT_STREQ(tNewLoadCaseList2[0].id.c_str(), "88");
    ASSERT_STREQ(tNewLoadCaseList2[1].loads[0].y.c_str(), "55");
    ASSERT_STREQ(tNewLoadCaseList2[1].loads[0].x.c_str(), "44");
    ASSERT_STREQ(tNewLoadCaseList2[1].loads[0].z.c_str(), "66");
    ASSERT_EQ(tOriginalToNewLoadCaseMap[88][0], 88);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[88][1], 1);
    // Check case where load case with multiple loads is is split and the ids of the resulting
    // load cases have to "straddle" original load case id
    LoadCase tLC1;
    tLC1.id = "2";
    Load tL1, tL2, tL3;
    tLC1.loads.push_back(tL1);
    tLC1.loads.push_back(tL2);
    tLC1.loads.push_back(tL3);
    Plato::UniqueCounter tIDCounter;
    tIDCounter.mark(0);
    tIDCounter.mark(2);
    std::vector<LoadCase> tNewList;
    tOriginalToNewLoadCaseMap.clear();
    Plato::expand_single_load_case(tLC1,tNewList,tIDCounter,
                                   tOriginalToNewLoadCaseMap);
    ASSERT_EQ(tNewList.size(), 3);
    ASSERT_STREQ(tNewList[0].id.c_str(), "2");
    ASSERT_STREQ(tNewList[1].id.c_str(), "1");
    ASSERT_STREQ(tNewList[2].id.c_str(), "3");
    ASSERT_EQ(tOriginalToNewLoadCaseMap[2][0], 2);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[2][1], 1);
    ASSERT_EQ(tOriginalToNewLoadCaseMap[2][2], 3);
}

TEST(PlatoTest, generateSROMInput)
{
    InputData tInputData;
    LoadCase tLC1;

    Load tL1;
    tL1.type = "pressure";
    tL1.app_type = "sideset";
    tL1.app_id = "55";
    tL1.x = "11.1";
    tL1.y = "22.2";
    tL1.z = "33.3";
    Load tL2;
    tL2.type = "force";
    tL2.app_type = "nodeset";
    tL2.app_id = "3";
    tL2.x = "21.1";
    tL2.y = "32.2";
    tL2.z = "43.3";
    Load tL3;
    tL3.type = "traction";
    tL3.app_type = "sideset";
    tL3.app_id = "4";
    tL3.x = "31.1";
    tL3.y = "42.2";
    tL3.z = "53.3";

    tLC1.id = "2";
    tLC1.loads.push_back(tL1);
    tLC1.loads.push_back(tL2);
    tLC1.loads.push_back(tL3);
    tInputData.load_cases.push_back(tLC1);
    tLC1.id = "4";
    tLC1.loads.clear();
    tLC1.loads.push_back(tL3);
    tLC1.loads.push_back(tL2);
    tLC1.loads.push_back(tL1);
    tInputData.load_cases.push_back(tLC1);
    tLC1.id = "6";
    tLC1.loads.clear();
    tLC1.loads.push_back(tL2);
    tInputData.load_cases.push_back(tLC1);
    tLC1.id = "10";
    tLC1.loads.clear();
    tLC1.loads.push_back(tL3);
    tLC1.loads.push_back(tL1);
    tInputData.load_cases.push_back(tLC1);
    Uncertainty tUncertainty1;
    tUncertainty1.type = "angle variation";
    tUncertainty1.id = "4";
    tUncertainty1.axis = "x";
    tUncertainty1.distribution = "beta";
    tUncertainty1.mean = "90.0";
    tUncertainty1.upper = "95.0";
    tUncertainty1.lower = "89.0";
    tUncertainty1.standard_deviation = "1.3";
    tUncertainty1.num_samples = "3";
    Uncertainty tUncertainty2;
    tUncertainty2.type = "angle variation";
    tUncertainty2.id = "10";
    tUncertainty2.axis = "y";
    tUncertainty2.distribution = "beta";
    tUncertainty2.mean = "10.0";
    tUncertainty2.upper = "15.0";
    tUncertainty2.lower = "5.0";
    tUncertainty2.standard_deviation = "2.3";
    tUncertainty2.num_samples = "2";
    tInputData.uncertainties.push_back(tUncertainty1);
    tInputData.uncertainties.push_back(tUncertainty2);

    std::vector<std::shared_ptr<Plato::Variable> > tSROMVariables;
    Plato::generateSROMInput(tInputData, tSROMVariables);

    const double tTolerance = 1e-6;
    ASSERT_EQ(tSROMVariables.size(), 9);
    ASSERT_STREQ(tSROMVariables[0]->globalID().c_str(), "4");
    ASSERT_TRUE(tSROMVariables[0]->isRandom());
    ASSERT_STREQ(tSROMVariables[0]->type().c_str(), "angle variation");
    ASSERT_STREQ(tSROMVariables[0]->subType().c_str(), "x");
    ASSERT_STREQ(tSROMVariables[0]->distribution().c_str(), "beta");
    ASSERT_STREQ(tSROMVariables[0]->mean().c_str(), "90.0");
    ASSERT_STREQ(tSROMVariables[0]->upperBound().c_str(), "95.0");
    ASSERT_STREQ(tSROMVariables[0]->lowerBound().c_str(), "89.0");
    ASSERT_STREQ(tSROMVariables[0]->standardDeviation().c_str(), "1.3");
    ASSERT_STREQ(tSROMVariables[0]->numSamples().c_str(), "3");
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[0]))->x(), 31.1, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[0]))->y(), 42.2, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[0]))->z(), 53.3, tTolerance);
    ASSERT_EQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[0]))->appID(), 4);
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[0]))->appType().c_str(), "sideset");
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[0]))->loadType().c_str(), "traction");

    ASSERT_STREQ(tSROMVariables[1]->globalID().c_str(), "5");
    ASSERT_TRUE(tSROMVariables[1]->isRandom());
    ASSERT_STREQ(tSROMVariables[1]->type().c_str(), "angle variation");
    ASSERT_STREQ(tSROMVariables[1]->subType().c_str(), "x");
    ASSERT_STREQ(tSROMVariables[1]->distribution().c_str(), "beta");
    ASSERT_STREQ(tSROMVariables[1]->mean().c_str(), "90.0");
    ASSERT_STREQ(tSROMVariables[1]->upperBound().c_str(), "95.0");
    ASSERT_STREQ(tSROMVariables[1]->lowerBound().c_str(), "89.0");
    ASSERT_STREQ(tSROMVariables[1]->standardDeviation().c_str(), "1.3");
    ASSERT_STREQ(tSROMVariables[1]->numSamples().c_str(), "3");
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[1]))->x(), 21.1, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[1]))->y(), 32.2, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[1]))->z(), 43.3, tTolerance);
    ASSERT_EQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[1]))->appID(), 3);
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[1]))->appType().c_str(), "nodeset");
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[1]))->loadType().c_str(), "force");

    ASSERT_STREQ(tSROMVariables[2]->globalID().c_str(), "7");
    ASSERT_TRUE(tSROMVariables[2]->isRandom());
    ASSERT_STREQ(tSROMVariables[2]->type().c_str(), "angle variation");
    ASSERT_STREQ(tSROMVariables[2]->subType().c_str(), "x");
    ASSERT_STREQ(tSROMVariables[2]->distribution().c_str(), "beta");
    ASSERT_STREQ(tSROMVariables[2]->mean().c_str(), "90.0");
    ASSERT_STREQ(tSROMVariables[2]->upperBound().c_str(), "95.0");
    ASSERT_STREQ(tSROMVariables[2]->lowerBound().c_str(), "89.0");
    ASSERT_STREQ(tSROMVariables[2]->standardDeviation().c_str(), "1.3");
    ASSERT_STREQ(tSROMVariables[2]->numSamples().c_str(), "3");
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[2]))->x(), 11.1, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[2]))->y(), 22.2, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[2]))->z(), 33.3, tTolerance);
    ASSERT_EQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[2]))->appID(), 55);
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[2]))->appType().c_str(), "sideset");
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[2]))->loadType().c_str(), "pressure");

    ASSERT_STREQ(tSROMVariables[3]->globalID().c_str(), "10");
    ASSERT_TRUE(tSROMVariables[3]->isRandom());
    ASSERT_STREQ(tSROMVariables[3]->type().c_str(), "angle variation");
    ASSERT_STREQ(tSROMVariables[3]->subType().c_str(), "y");
    ASSERT_STREQ(tSROMVariables[3]->distribution().c_str(), "beta");
    ASSERT_STREQ(tSROMVariables[3]->mean().c_str(), "10.0");
    ASSERT_STREQ(tSROMVariables[3]->upperBound().c_str(), "15.0");
    ASSERT_STREQ(tSROMVariables[3]->lowerBound().c_str(), "5.0");
    ASSERT_STREQ(tSROMVariables[3]->standardDeviation().c_str(), "2.3");
    ASSERT_STREQ(tSROMVariables[3]->numSamples().c_str(), "2");
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[3]))->x(), 31.1, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[3]))->y(), 42.2, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[3]))->z(), 53.3, tTolerance);
    ASSERT_EQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[3]))->appID(), 4);
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[3]))->appType().c_str(), "sideset");
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[3]))->loadType().c_str(), "traction");

    ASSERT_STREQ(tSROMVariables[4]->globalID().c_str(), "8");
    ASSERT_TRUE(tSROMVariables[4]->isRandom());
    ASSERT_STREQ(tSROMVariables[4]->type().c_str(), "angle variation");
    ASSERT_STREQ(tSROMVariables[4]->subType().c_str(), "y");
    ASSERT_STREQ(tSROMVariables[4]->distribution().c_str(), "beta");
    ASSERT_STREQ(tSROMVariables[4]->mean().c_str(), "10.0");
    ASSERT_STREQ(tSROMVariables[4]->upperBound().c_str(), "15.0");
    ASSERT_STREQ(tSROMVariables[4]->lowerBound().c_str(), "5.0");
    ASSERT_STREQ(tSROMVariables[4]->standardDeviation().c_str(), "2.3");
    ASSERT_STREQ(tSROMVariables[4]->numSamples().c_str(), "2");
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[4]))->x(), 11.1, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[4]))->y(), 22.2, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[4]))->z(), 33.3, tTolerance);
    ASSERT_EQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[4]))->appID(), 55);
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[4]))->appType().c_str(), "sideset");
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[4]))->loadType().c_str(), "pressure");

    ASSERT_STREQ(tSROMVariables[5]->globalID().c_str(), "2");
    ASSERT_FALSE(tSROMVariables[5]->isRandom());
    ASSERT_STREQ(tSROMVariables[5]->type().c_str(), "");
    ASSERT_STREQ(tSROMVariables[5]->subType().c_str(), "");
    ASSERT_STREQ(tSROMVariables[5]->distribution().c_str(), "");
    ASSERT_STREQ(tSROMVariables[5]->mean().c_str(), "");
    ASSERT_STREQ(tSROMVariables[5]->upperBound().c_str(), "");
    ASSERT_STREQ(tSROMVariables[5]->lowerBound().c_str(), "");
    ASSERT_STREQ(tSROMVariables[5]->standardDeviation().c_str(), "");
    ASSERT_STREQ(tSROMVariables[5]->numSamples().c_str(), "");
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[5]))->x(), 11.1, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[5]))->y(), 22.2, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[5]))->z(), 33.3, tTolerance);
    ASSERT_EQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[5]))->appID(), 55);
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[5]))->appType().c_str(), "sideset");
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[5]))->loadType().c_str(), "pressure");

    ASSERT_STREQ(tSROMVariables[6]->globalID().c_str(), "1");
    ASSERT_FALSE(tSROMVariables[6]->isRandom());
    ASSERT_STREQ(tSROMVariables[6]->type().c_str(), "");
    ASSERT_STREQ(tSROMVariables[6]->subType().c_str(), "");
    ASSERT_STREQ(tSROMVariables[6]->distribution().c_str(), "");
    ASSERT_STREQ(tSROMVariables[6]->mean().c_str(), "");
    ASSERT_STREQ(tSROMVariables[6]->upperBound().c_str(), "");
    ASSERT_STREQ(tSROMVariables[6]->lowerBound().c_str(), "");
    ASSERT_STREQ(tSROMVariables[6]->standardDeviation().c_str(), "");
    ASSERT_STREQ(tSROMVariables[6]->numSamples().c_str(), "");
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[6]))->x(), 21.1, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[6]))->y(), 32.2, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[6]))->z(), 43.3, tTolerance);
    ASSERT_EQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[6]))->appID(), 3);
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[6]))->appType().c_str(), "nodeset");
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[6]))->loadType().c_str(), "force");

    ASSERT_STREQ(tSROMVariables[7]->globalID().c_str(), "3");
    ASSERT_FALSE(tSROMVariables[7]->isRandom());
    ASSERT_STREQ(tSROMVariables[7]->type().c_str(), "");
    ASSERT_STREQ(tSROMVariables[7]->subType().c_str(), "");
    ASSERT_STREQ(tSROMVariables[7]->distribution().c_str(), "");
    ASSERT_STREQ(tSROMVariables[7]->mean().c_str(), "");
    ASSERT_STREQ(tSROMVariables[7]->upperBound().c_str(), "");
    ASSERT_STREQ(tSROMVariables[7]->lowerBound().c_str(), "");
    ASSERT_STREQ(tSROMVariables[7]->standardDeviation().c_str(), "");
    ASSERT_STREQ(tSROMVariables[7]->numSamples().c_str(), "");
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[7]))->x(), 31.1, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[7]))->y(), 42.2, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[7]))->z(), 53.3, tTolerance);
    ASSERT_EQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[7]))->appID(), 4);
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[7]))->appType().c_str(), "sideset");
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[7]))->loadType().c_str(), "traction");

    ASSERT_STREQ(tSROMVariables[8]->globalID().c_str(), "6");
    ASSERT_FALSE(tSROMVariables[8]->isRandom());
    ASSERT_STREQ(tSROMVariables[8]->type().c_str(), "");
    ASSERT_STREQ(tSROMVariables[8]->subType().c_str(), "");
    ASSERT_STREQ(tSROMVariables[8]->distribution().c_str(), "");
    ASSERT_STREQ(tSROMVariables[8]->mean().c_str(), "");
    ASSERT_STREQ(tSROMVariables[8]->upperBound().c_str(), "");
    ASSERT_STREQ(tSROMVariables[8]->lowerBound().c_str(), "");
    ASSERT_STREQ(tSROMVariables[8]->standardDeviation().c_str(), "");
    ASSERT_STREQ(tSROMVariables[8]->numSamples().c_str(), "");
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[8]))->x(), 21.1, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[8]))->y(), 32.2, tTolerance);
    ASSERT_NEAR((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[8]))->z(), 43.3, tTolerance);
    ASSERT_EQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[8]))->appID(), 3);
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[8]))->appType().c_str(), "nodeset");
    ASSERT_STREQ((std::static_pointer_cast<Plato::SROMLoad>(tSROMVariables[8]))->loadType().c_str(), "force");
}

}
