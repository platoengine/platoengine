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
 * Plato_SromLoadUtilsXML.hpp
 *
 *  Created on: May 3, 2020
 */

#pragma once

#include <sstream>
#include <iostream>

#include "Plato_RandomLoadMetadata.hpp"
#include "XMLGeneratorDataStruct.hpp"
#include "Plato_UniqueCounter.hpp"
#include "Plato_FreeFunctions.hpp"
#include "Plato_SromXML.hpp"
#include "Plato_Macros.hpp"

namespace Plato
{

namespace srom
{

/******************************************************************************//**
 * \brief transform string with variable type to an enum (Plato::srom::VariableType)
 * \param [in] aStringVarType string variable type
 * \param [out] aEnumVarType enum variable type
 * \return flag (true = no error, false = error, string variable type is not defined/supported)
**********************************************************************************/
inline bool variable_type_string_to_enum(const std::string& aStringVarType, Plato::srom::VariableType::type_t& aEnumVarType)
{
    if(aStringVarType == "material")
    {
        aEnumVarType = Plato::srom::VariableType::MATERIAL;
    }
    else if(aStringVarType == "load")
    {
        aEnumVarType = Plato::srom::VariableType::LOAD;
    }
    else
    {
        return (false);
    }

    return (true);
}
// function variable_type_string_to_enum

/******************************************************************************//**
 * \brief Initialize counter used to set load identifiers (IDs)
 * \param [in] aLoadCases array of load cases
 * \param [in] aUniqueCounter IDs counter
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool initialize_load_id_counter(const std::vector<XMLGen::LoadCase> &aLoadCases,
                                       Plato::UniqueCounter &aUniqueCounter)
{
    if(aLoadCases.empty() == true)
    {
        PRINTERR("INPUT SET OF LOAD CASES IS EMPTY.\n");
        return (false);
    }

    aUniqueCounter.mark(0); // Mark 0 as true since we don't want to have a 0 ID
    for(size_t tIndex = 0; tIndex < aLoadCases.size(); tIndex++)
    {
        aUniqueCounter.mark(std::atoi(aLoadCases[tIndex].id.c_str()));
    }

    return (true);
}
// function initialize_load_id_counter

/******************************************************************************//**
 * \brief Expand set of load cases into an array of single load cases
 * \param [in] aOldLoadCase old load case metadata
 * \param [out] aNewLoadCaseList expanded set of load cases - load case is reformatted
 *               to facilitate the creation of the stochastic reduced order models
 * \param [out] aUniqueLoadIDCounter load identifiers counter
 * \param [out] aOriginalToNewLoadCaseMap map between original load case IDs and new load case IDs
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool expand_single_load_case(const XMLGen::LoadCase &aOldLoadCase,
                                    std::vector<XMLGen::LoadCase> &aNewLoadCaseList,
                                    Plato::UniqueCounter &aUniqueLoadIDCounter,
                                    std::map<int, std::vector<int> > &aOriginalToNewLoadCaseMap)
{
    if(aOldLoadCase.loads.empty() == true)
    {
        std::ostringstream tMsg;
        tMsg << "LOAD CASE #" << aOldLoadCase.id << " HAS AN EMPTY LOADS SET.\n";
        PRINTERR(tMsg.str().c_str());
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
        aOriginalToNewLoadCaseMap[tOriginalLoadCaseID].push_back(aNewLoadCaseList.size());
        XMLGen::LoadCase tNewLoadCase = aOldLoadCase;
        tNewLoadCase.id = tIDString;
        tNewLoadCase.loads[0] = aOldLoadCase.loads[tLoadCaseIndex];
        tNewLoadCase.loads.resize(1);
        aNewLoadCaseList.push_back(tNewLoadCase);
    }

    return (true);
}
// function expand_single_load_case

/******************************************************************************//**
 * \brief Expand old set of load cases into an array of new load cases with one load
 * \param [in] aOldLoadCase old set of load cases
 * \param [in] aNewLoadCaseList new set of load cases re-formatted to create the
 *             stochastic reduced order models (srom)
 * \param [in] aOriginalToNewLoadCaseMap map between original load case IDs and new load case IDs
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool expand_load_cases(const std::vector<XMLGen::LoadCase> &aInputLoadCases,
                              std::vector<XMLGen::LoadCase> &aNewLoadCaseList,
                              std::map<int, std::vector<int> > &aOriginalToNewLoadCaseMap)
{
    Plato::UniqueCounter tUniqueLoadIDCounter;
    if(Plato::srom::initialize_load_id_counter(aInputLoadCases, tUniqueLoadIDCounter) == false)
    {
        PRINTERR("FAILED TO INITIALIZE ORIGINAL SET OF LOAD IDENTIFIERS.\n");
        return (false);
    }

    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < aInputLoadCases.size(); tLoadCaseIndex++)
    {
        const XMLGen::LoadCase& tOldLoadCase = aInputLoadCases[tLoadCaseIndex];
        Plato::srom::expand_single_load_case(tOldLoadCase, aNewLoadCaseList, tUniqueLoadIDCounter, aOriginalToNewLoadCaseMap);
    }

    return (true);
}
// function expand_load_cases

/******************************************************************************//**
 * \brief Create a deterministic load from the input deterministic load case
 * \param [in] aLoadCase deterministic load case metadata (input load case has only
 *            one load, multiple loads are not expected)
 * \param [out] aLoad deterministic load metadata
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool create_deterministic_load_variable(const XMLGen::LoadCase &aLoadCase, Plato::srom::Load& aLoad)
{
    if(aLoadCase.loads.empty() == true)
    {
        std::ostringstream tMsg;
        tMsg << "LOAD CASE #" << aLoadCase.id << " HAS AN EMPTY LOADS SET.\n";
        PRINTERR(tMsg.str().c_str());
        return (false);
    }

    for(size_t i=0; i<aLoadCase.loads[0].values.size(); ++i)
        aLoad.mValues.push_back(aLoadCase.loads[0].values[i]);

    aLoad.mLoadID = aLoadCase.id;
    aLoad.mAppType = aLoadCase.loads[0].app_type;
    aLoad.mLoadType = aLoadCase.loads[0].type;
    aLoad.mAppID = std::atoi(aLoadCase.loads[0].app_id.c_str());

    return (true);
}
// function create_deterministic_load_variable

/******************************************************************************//**
 * \brief Get or create a random load given a random load case
 * \param [in] aLoadCase random load case metadata (load case only has one load,
 *              multiple loads are not expected)
 * \param [out] aRandomLoad set of random loads
 * \return random load identifier
**********************************************************************************/
inline int get_or_create_random_load_variable(const XMLGen::LoadCase &aLoadCase,
                                              std::vector<Plato::srom::Load> &aRandomLoads)
{
    for(size_t tRandomLoadIndex = 0; tRandomLoadIndex < aRandomLoads.size(); ++tRandomLoadIndex)
    {
        if(aRandomLoads[tRandomLoadIndex].mLoadID == aLoadCase.id)
            return (tRandomLoadIndex);
    }

    Plato::srom::Load tNewLoad;
    tNewLoad.mLoadType = aLoadCase.loads[0].type;
    tNewLoad.mAppType = aLoadCase.loads[0].app_type;
    tNewLoad.mAppID = std::atoi(aLoadCase.loads[0].app_id.c_str());
    tNewLoad.mAppName = aLoadCase.loads[0].app_name;
    tNewLoad.mLoadID = aLoadCase.id;
    for(size_t i=0; i<aLoadCase.loads[0].values.size(); ++i)
        tNewLoad.mValues.push_back(aLoadCase.loads[0].values[i]);
    aRandomLoads.push_back(tNewLoad);

    return (aRandomLoads.size() - 1);
}
// function get_or_create_random_load_variable

/******************************************************************************//**
 * \brief Append a random variable, and corresponding statistics, to the random load.
 * \param [in] aRandomLoad random load metadata
 * \param [out] aRandomVariable random variable metadata
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline void add_random_variable_to_random_load(Plato::srom::Load &aRandomLoad,
                                               const XMLGen::Uncertainty &aRandomVariable)
{
    Plato::srom::RandomVariable tNewVariable;
    tNewVariable.tag(aRandomVariable.type);
    tNewVariable.attribute(aRandomVariable.axis);
    tNewVariable.mean(aRandomVariable.mean);
    tNewVariable.lower(aRandomVariable.lower);
    tNewVariable.upper(aRandomVariable.upper);
    tNewVariable.samples(aRandomVariable.num_samples);
    tNewVariable.distribution(aRandomVariable.distribution);
    tNewVariable.deviation(aRandomVariable.standard_deviation);
    aRandomLoad.mRandomVars.push_back(tNewVariable);
}
// function add_random_variable_to_random_load

/******************************************************************************//**
 * \brief Create a random load and append it to the set of random load cases
 * \param [in] aRandomVariable random variable metadata
 * \param [in] aNewLoadCases set of load cases (deterministic plus random load cases)
 * \param [in] aOriginalToNewLoadCaseMap map from original load case identifier to new/expanded load case identifier
 * \param [out] aRandomLoadIDs set of random load case identifiers
 * \param [out] aRandomLoads set of random load cases
**********************************************************************************/
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
        const int tIndexOfRandomLoad =
            Plato::srom::get_or_create_random_load_variable(aNewLoadCases[tIndexIntoNewLoadCaseList], aRandomLoads);
        Plato::srom::add_random_variable_to_random_load(aRandomLoads[tIndexOfRandomLoad], aRandomVariable);
    }
}
// function create_random_loads_from_uncertainty

/******************************************************************************//**
 * \brief Create set of random loads
 * \param [in] aRandomVariable random variable metadata
 * \param [in] aNewLoadCases set of load cases (deterministic plus random load cases)
 * \param [in] aOriginalToNewLoadCaseMap map from original load case identifier to new/expanded load case identifier
 * \param [out] aRandomLoadIDs set of random load case identifiers
 * \param [out] aRandomLoads set of random load cases
**********************************************************************************/
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
        Plato::srom::VariableType::type_t tVarType = Plato::srom::VariableType::UNDEFINED;
        Plato::srom::variable_type_string_to_enum(tRandomVariable.variable_type, tVarType);
        if(tVarType == Plato::srom::VariableType::LOAD)
        {
            Plato::srom::create_random_loads_from_uncertainty(tRandomVariable, aNewLoadCases, aOriginalToNewLoadCaseMap, aRandomLoadIDs, aLoad);
        }
    }
}
// function create_random_load_variables

/******************************************************************************//**
 * \brief Create set of deterministic loads
 * \param [in] aNewLoadCases set of load cases (deterministic plus random load cases)
 * \param [in] aRandomLoadIDs set of random load case identifiers
 * \param [out] aDeterministicLoads set of deterministic load cases
**********************************************************************************/
inline void create_deterministic_load_variables(const std::vector<XMLGen::LoadCase> &aNewLoadCases,
                                                const std::set<int> & aRandomLoadIDs,
                                                std::vector<Plato::srom::Load> &aDeterministicLoads)
{
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < aNewLoadCases.size(); tLoadCaseIndex++)
    {
        const int tCurLoadCaseID = std::atoi(aNewLoadCases[tLoadCaseIndex].id.c_str());
        if(aRandomLoadIDs.find(tCurLoadCaseID) == aRandomLoadIDs.end())
        {
            Plato::srom::Load tNewLoad;
            Plato::srom::create_deterministic_load_variable(aNewLoadCases[tLoadCaseIndex], tNewLoad);
            aDeterministicLoads.push_back(tNewLoad);
        }
    }
}
// function create_deterministic_load_variables

/******************************************************************************//**
 * \brief Generate array of deterministic and random loads in the format expected \n
 * by the stochastic reduced order model application programming interface.
 * \param [in] aInputLoadCases set of load cases created by the XML generator
 * \param [in] aUncertainties set of random variables created by the XML generator
 * \param [out] aLoads set of deterministic and random loads in the format expected
 *                    by the Stochastic Reduced Order Model (SROM) interface
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool generate_srom_load_inputs(const std::vector<XMLGen::LoadCase> &aInputLoadCases,
                                      const std::vector<XMLGen::Uncertainty> &aUncertainties,
                                      std::vector<Plato::srom::Load> &aLoads)
{
    std::vector<XMLGen::LoadCase> tNewSetLoadCases;
    std::map<int, std::vector<int> > tOriginalToNewLoadCaseMap;
    Plato::srom::expand_load_cases(aInputLoadCases, tNewSetLoadCases, tOriginalToNewLoadCaseMap);

    std::set<int> tRandomLoadIDs;
    Plato::srom::create_random_load_variables
        (aUncertainties, tNewSetLoadCases, tOriginalToNewLoadCaseMap, tRandomLoadIDs, aLoads);

    Plato::srom::create_deterministic_load_variables(tNewSetLoadCases, tRandomLoadIDs, aLoads);

    return (true);
}
// function generate_srom_load_inputs

/******************************************************************************//**
 * \fn preprocess_srom_problem_load_inputs
 * \brief Pre-process Stochastic Reduced Order Model (SROM) problem inputs.
 * \param [in/out] aInputMetadata Plato problem input metadata
 * \param [in/out] aLoadCases     set of load case metadata
 * \param [in/out] aUncertainties set of uncertainty metadata
**********************************************************************************/
inline void preprocess_srom_problem_load_inputs
(XMLGen::InputData& aInputMetadata,
 std::vector<XMLGen::LoadCase>& aLoadCases,
 std::vector<XMLGen::Uncertainty>& aUncertainties)
{
    XMLGen::Objective &tCurObj = aInputMetadata.objectives[0];
    for (size_t tLoadCaseID = 0; tLoadCaseID < tCurObj.load_case_ids.size(); ++tLoadCaseID)
    {
        const std::string &tCurLoadCaseID = tCurObj.load_case_ids[tLoadCaseID];
        for (size_t tLoadCaseIndex = 0; tLoadCaseIndex < aInputMetadata.load_cases.size(); ++tLoadCaseIndex)
        {
            if (aInputMetadata.load_cases[tLoadCaseIndex].id == tCurLoadCaseID)
            {
                aLoadCases.push_back(aInputMetadata.load_cases[tLoadCaseIndex]);
                tLoadCaseIndex = aInputMetadata.load_cases.size();
            }
        }
        for (size_t tUncertaintyIndex = 0; tUncertaintyIndex < aInputMetadata.uncertainties.size(); ++tUncertaintyIndex)
        {
            if (aInputMetadata.uncertainties[tUncertaintyIndex].id == tCurLoadCaseID)
            {
                aUncertainties.push_back(aInputMetadata.uncertainties[tUncertaintyIndex]);
                aInputMetadata.mObjectiveLoadCaseIndexToUncertaintyIndex[tLoadCaseID] = tUncertaintyIndex;
            }
        }
    }
}
// function preprocess_srom_problem_load_inputs

inline void postprocess_srom_problem_load_outputs
(const Plato::srom::OutputMetaData& aOutputs,
 XMLGen::InputData& aInputData,
 std::vector<XMLGen::LoadCase>& aNewLoadCases,
 std::vector<double>& aLoadCaseProbabilities)
{
    int tStartingLoadCaseID = aNewLoadCases.size() + 1;
    XMLGen::Objective &tCurObj = aInputData.objectives[0];
    tCurObj.load_case_ids.clear();
    tCurObj.load_case_weights.clear();
    for (size_t tLoadCaseIndex = 0; tLoadCaseIndex < aOutputs.mLoadCases.size(); ++tLoadCaseIndex)
    {
        XMLGen::LoadCase tNewLoadCase;
        auto tLoadCaseID = Plato::to_string(tStartingLoadCaseID);
        tNewLoadCase.id = tLoadCaseID;
        for (size_t aLoadIndex = 0; aLoadIndex < aOutputs.mLoadCases[tLoadCaseIndex].mLoads.size(); ++aLoadIndex)
        {
            XMLGen::Load tNewLoad;
            tNewLoad.type = aOutputs.mLoadCases[tLoadCaseIndex].mLoads[aLoadIndex].mLoadType;
            tNewLoad.app_type = aOutputs.mLoadCases[tLoadCaseIndex].mLoads[aLoadIndex].mAppType;
            tNewLoad.app_id = Plato::to_string(aOutputs.mLoadCases[tLoadCaseIndex].mLoads[aLoadIndex].mAppID);
            tNewLoad.app_name = aOutputs.mLoadCases[tLoadCaseIndex].mLoads[aLoadIndex].mAppName;
            for (size_t tDim = 0; tDim < aOutputs.mLoadCases[tLoadCaseIndex].mLoads[aLoadIndex].mLoadValues.size(); ++tDim)
            {
                tNewLoad.values.push_back(Plato::to_string(aOutputs.mLoadCases[tLoadCaseIndex].mLoads[aLoadIndex].mLoadValues[tDim]));
            }
            tNewLoad.load_id = Plato::to_string(aOutputs.mLoadCases[tLoadCaseIndex].mLoads[aLoadIndex].mLoadID);
            tNewLoadCase.loads.push_back(tNewLoad);
        }
        aNewLoadCases.push_back(tNewLoadCase);
        aLoadCaseProbabilities.push_back(aOutputs.mLoadCases[tLoadCaseIndex].probability());
        tCurObj.load_case_ids.push_back(tLoadCaseID);
        tCurObj.load_case_weights.push_back("1");
        tStartingLoadCaseID++;
    }
}
// function postprocess_srom_problem_load_outputs

}
// namespace srom

}
// namespace Plato

