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
 * Plato_SROM_Metadata.hpp
 *
 *  Created on: June 18, 2019
 */

#include <map>
#include <cmath>
#include <locale>
#include <cctype>
#include <cstdlib>
#include <iostream>
#include <memory>

#include "../../base/src/optimize/Plato_SolveUncertaintyProblem.hpp"
#include "../../base/src/optimize/Plato_KelleySachsAugmentedLagrangianLightInterface.hpp"
#include "Plato_SROM_Utilities.hpp"

namespace Plato
{

bool variable_type_string_to_enum(const std::string& aStringVarType, Plato::VariableType::type_t& aEnumVarType)
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


bool check_vector3d_values(const Plato::Vector3D & aMyOriginalLoad)
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

bool initialize_load_id_counter(const std::vector<XMLGen::LoadCase> &aLoadCases,
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

bool expand_single_load_case(const XMLGen::LoadCase &aOldLoadCase,
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

bool expand_load_cases(const std::vector<XMLGen::LoadCase> &aInputLoadCases,
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

bool set_random_variable_statistics(const XMLGen::Uncertainty &aRandomVariable, Plato::srom::Statistics& aStatistics)
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

bool create_deterministic_load_variable(const XMLGen::LoadCase &aLoadCase, Plato::srom::Load& aLoad)
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

int get_or_create_random_load_variable(const XMLGen::LoadCase &aLoadCase,
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

void add_random_variable_to_random_load(Plato::srom::Load &aRandomLoad,
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

void create_random_loads_from_uncertainty(const XMLGen::Uncertainty& aRandomVariable,
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

void create_random_load_variables(const std::vector<XMLGen::Uncertainty> &aRandomVariables,
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

void create_deterministic_load_variables(const std::vector<XMLGen::LoadCase> &aNewLoadCases,
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

bool generate_srom_load_inputs(const std::vector<XMLGen::LoadCase> &aInputLoadCases,
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

bool apply_rotation_matrix(const Plato::Vector3D& aRotatioAnglesInDegrees, Plato::Vector3D& aVectorToRotate)
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

bool define_distribution(const Plato::srom::Variable & aMyRandomVar, Plato::SromInputs<double> & aInput)
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

bool check_input_mean(const Plato::srom::Variable & aMyRandomVar)
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

bool check_input_lower_bound(const Plato::srom::Variable & aMyRandomVar)
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

bool check_input_upper_bound(const Plato::srom::Variable & aMyRandomVar)
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

bool check_input_standard_deviation(const Plato::srom::Variable & aMyRandomVar)
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

bool check_input_number_samples(const Plato::srom::Variable & aMyRandomVar)
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

bool check_input_statistics(const Plato::srom::Variable & aMyRandomVar)
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

bool define_input_statistics(const Plato::srom::Variable & aMyRandomVar, Plato::SromInputs<double> & aInput)
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

bool post_process_sample_probability_pairs(const std::vector<Plato::SromOutputs<double>> aMySromSolution,
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

bool compute_uniform_random_variable_statistics(const Plato::SromInputs<double> & aSromInputs,
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

bool compute_random_variable_statistics(const Plato::SromInputs<double> & aSromInputs,
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

bool compute_sample_probability_pairs(const std::vector<Plato::srom::Variable> & aSetRandomVariables,
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

bool expand_load_sample_probability_pair(const std::vector<Plato::srom::RandomVariable> & aMySampleProbPairs,
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

bool compute_random_rotations_about_xyz(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
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

bool compute_random_rotations_about_xy(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
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

bool compute_random_rotations_about_xz(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
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

bool compute_random_rotations_about_yz(const Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
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

bool compute_random_rotations_about_x(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
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

bool compute_random_rotations_about_y(const Plato::srom::SampleProbabilityPairs& aMyYaxisSampleProbPairs,
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

bool compute_random_rotations_about_z(const Plato::srom::SampleProbabilityPairs& aMyZaxisSampleProbPairs,
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

bool expand_random_rotations(const Plato::srom::SampleProbabilityPairs& aMyXaxisSampleProbPairs,
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

bool check_expand_random_loads_inputs(const Plato::Vector3D & aMyOriginalLoad,
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

bool expand_random_loads(const Plato::Vector3D & aMyOriginalLoad,
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

bool update_initial_random_load_case(const std::vector<Plato::srom::RandomLoad> & aNewSetRandomLoads,
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

bool update_random_load_cases(const std::vector<Plato::srom::RandomLoad> & aNewSetRandomLoads,
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

bool expand_random_load_cases(const std::vector<Plato::srom::RandomLoad> & aNewSetRandomLoads,
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

bool expand_random_and_deterministic_loads(const std::vector<Plato::srom::Load>& aLoads,
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

bool check_load_parameters(const Plato::srom::Load& aLoad)
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

bool set_load_components(const std::vector<std::string> & aInput, Plato::Vector3D & aOutput)
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

bool set_random_load_parameters(const Plato::srom::Load & aOriginalLoad, std::vector<Plato::srom::RandomLoad> & aSetRandomLoads)
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

bool generate_set_random_rotations(const std::vector<Plato::srom::RandomVariable> & aMySampleProbPairs,
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

bool generate_set_random_loads(const Plato::srom::Load & aOriginalLoad,
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

void generate_load_case_identifiers(std::vector<Plato::srom::RandomLoadCase> & aSetLoadCases)
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

bool check_deterministic_loads(const std::vector<Plato::srom::Load>& aDeterministicLoads)
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

void append_deterministic_loads(const std::vector<Plato::srom::Load>& aDeterministicLoads,
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

bool generate_output_random_load_cases(const std::vector<Plato::srom::Load>& aDeterministicLoads,
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

bool generate_load_sroms(const Plato::srom::InputMetaData & aInput, Plato::srom::OutputMetaData & aOutput)
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

} // namespace Plato

