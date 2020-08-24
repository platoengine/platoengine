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
 * Plato_SromXMLGenLoad.hpp
 *
 *  Created on: May 3, 2020
 */

#pragma once

#include <sstream>
#include <iostream>

#include "XMLGeneratorDataStruct.hpp"

#include "Plato_RandomLoadMetadata.hpp"
#include "Plato_SromXMLGenHelpers.hpp"
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
        aUniqueCounter.mark(std::stoi(aLoadCases[tIndex].id.c_str()));
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
        tMsg << "Expand Single Load Case: Loads container in load case with identification number '" << aOldLoadCase.id << " is empty.\n";
        PRINTERR(tMsg.str());
        return (false);
    }

    auto tOriginalLoadCaseID = std::stoi(aOldLoadCase.id);
    for(auto& tOldLoad : aOldLoadCase.loads)
    {
        XMLGen::LoadCase tNewLoadCase;
        tNewLoadCase.id = aOldLoadCase.id;
        // If this is a multi-load load case, create a new load case with a new identification number.
        auto tLoadCaseIndex = &tOldLoad - &aOldLoadCase.loads[0];
        if(tLoadCaseIndex > 0)
        {
            auto tNewLoadCaseID = aUniqueLoadIDCounter.assignNextUnique();
            tNewLoadCase.id = std::to_string(tNewLoadCaseID);
        }
        aOriginalToNewLoadCaseMap[tOriginalLoadCaseID].push_back(aNewLoadCaseList.size());

        tNewLoadCase.loads.push_back(tOldLoad);
        aNewLoadCaseList.push_back(tNewLoadCase);
    }

    return (true);
}
// function expand_single_load_case

/******************************************************************************//**
 * \brief Expand old set of load cases into an array of new load cases with one load
 *
 * \param [in] aOldLoadCase old set of load cases
 * \param [in] aNewLoadCaseList new set of load cases re-formatted to create the
 *             stochastic reduced order models (srom)
 * \param [in] aOriginalToNewLoadCaseMap map between original load case IDs and \n
 *   new load case IDs, i.e. map<old_load_case_id,new_load_case_id>
 *
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool expand_load_cases(const std::vector<XMLGen::LoadCase> &aOldLoadCases,
                              std::vector<XMLGen::LoadCase> &aNewLoadCaseList,
                              std::map<int, std::vector<int> > &aOriginalToNewLoadCaseMap)
{
    Plato::UniqueCounter tUniqueLoadIDCounter;
    if(Plato::srom::initialize_load_id_counter(aOldLoadCases, tUniqueLoadIDCounter) == false)
    {
        PRINTERR("Expand Load Cases: FAILED TO INITIALIZE ORIGINAL SET OF LOAD IDENTIFIERS.\n");
        return (false);
    }

    for(auto& tOldLoadCase : aOldLoadCases)
    {
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
        tMsg << "Create Deterministic Load Variable: Load container in Load Case with identification number '"
            << aLoadCase.id << "' is empty.\n";
        PRINTERR(tMsg.str().c_str());
        return (false);
    }

    aLoad.mLoadID = aLoadCase.id;
    for (auto &tLoad : aLoadCase.loads)
    {
        aLoad.mLoadType = tLoad.type;
        aLoad.mAppType = tLoad.app_type;

        aLoad.mAppName = tLoad.app_name;
        aLoad.mAppID = tLoad.app_id.empty() ? std::numeric_limits<int>::max() : std::stoi(tLoad.app_id);
        if(tLoad.app_id.empty() && tLoad.app_name.empty())
        {
            THROWERR(std::string("Create Deterministic Load Variable: Mesh set, e.g. sideset or nodeset, identification ")
                + "number and name are not defined. One of the two mesh-set identifiers, identification number or name, "
                + "must be define.")
        }

        for (auto &tValue : tLoad.values)
        {
            aLoad.mValues.push_back(tValue);
        }
    }

    return (true);
}
// function create_deterministic_load_variable

/******************************************************************************//**
 * \brief Get or create a random load given a random load case
 * \param [in] aLoadCase random load case metadata (load case only has one load,
 *              multiple loads are not expected)
 * \param [out] aRandomLoad set of random loads
 * \return random load identification number
**********************************************************************************/
inline int create_random_load
(const XMLGen::LoadCase &aLoadCase,
 std::vector<Plato::srom::Load> &aRandomLoads)
{
    if(aLoadCase.loads.size() > 1u)
    {
        THROWERR(std::string("Create Random Load: Expecting one load per load case. There are '") +
            std::to_string(aLoadCase.loads.size()) + "' in load case with identification number '" + aLoadCase.id + "'.")
    }

    for(auto& tRandomLoad : aRandomLoads)
    {
        if(tRandomLoad.mLoadID == aLoadCase.id)
        {
            auto tRandomLoadIndex = &tRandomLoad - &aRandomLoads[0];
            return (tRandomLoadIndex);
        }
    }

    for(auto& tLoad : aLoadCase.loads)
    {
        Plato::srom::Load tNewLoad;
        tNewLoad.mLoadID = aLoadCase.id;
        tNewLoad.mLoadType = tLoad.type;
        tNewLoad.mAppType = tLoad.app_type;

        tNewLoad.mAppName = tLoad.app_name;
        tNewLoad.mAppID = tLoad.app_id.empty() ? std::numeric_limits<int>::max() : std::stoi(tLoad.app_id);
        if(tLoad.app_id.empty() && tLoad.app_name.empty())
        {
            THROWERR(std::string("Create Random Load: Mesh set, e.g. sideset or nodeset, identification ")
                + "number and name are not defined. One of the two mesh-set identifiers, identification "
                + "number or name, must be define.")
        }

        for(auto& tValue : tLoad.values)
        {
            tNewLoad.mValues.push_back(tValue);
        }
        aRandomLoads.push_back(tNewLoad);
    }

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
    tNewVariable.tag(aRandomVariable.tag());
    tNewVariable.attribute(aRandomVariable.attribute());
    tNewVariable.mean(aRandomVariable.mean());
    tNewVariable.guess(aRandomVariable.guess());
    tNewVariable.lower(aRandomVariable.lower());
    tNewVariable.upper(aRandomVariable.upper());
    tNewVariable.samples(aRandomVariable.samples());
    tNewVariable.distribution(aRandomVariable.distribution());
    tNewVariable.deviation(aRandomVariable.std());
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
    const int tRandomVarLoadID = std::stoi(aRandomVariable.id().c_str());
    for(size_t tIndexJ = 0; tIndexJ < aOriginalToNewLoadCaseMap[tRandomVarLoadID].size(); tIndexJ++)
    {
        const int tIndexIntoNewLoadCaseList = aOriginalToNewLoadCaseMap[tRandomVarLoadID][tIndexJ];
        std::string tCurLoadCaseIDString = aNewLoadCases[tIndexIntoNewLoadCaseList].id;
        const int tCurLoadCaseID = std::stoi(tCurLoadCaseIDString);
        aRandomLoadIDs.insert(tCurLoadCaseID);
        const int tIndexOfRandomLoad = Plato::srom::create_random_load(aNewLoadCases[tIndexIntoNewLoadCaseList], aRandomLoads);
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
    for(auto& tRandomVar : aRandomVariables)
    {
        Plato::srom::VariableType::type_t tVarType = Plato::srom::VariableType::UNDEFINED;
        Plato::srom::variable_type_string_to_enum(tRandomVar.category(), tVarType);
        if(tVarType == Plato::srom::VariableType::LOAD)
        {
            Plato::srom::create_random_loads_from_uncertainty(tRandomVar, aNewLoadCases, aOriginalToNewLoadCaseMap, aRandomLoadIDs, aLoad);
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
    for(auto& tNewLoadCase : aNewLoadCases)
    {
        auto tCurLoadCaseID = std::stoi(tNewLoadCase.id);
        if(aRandomLoadIDs.find(tCurLoadCaseID) == aRandomLoadIDs.end())
        {
            Plato::srom::Load tNewLoad;
            Plato::srom::create_deterministic_load_variable(tNewLoadCase, tNewLoad);
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
 * \param [out] aLoads set of deterministic and random loads in the format expected \n
 *   by the Stochastic Reduced Order Model (SROM) interface
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline std::vector<Plato::srom::Load>
generate_srom_load_inputs
(const std::vector<XMLGen::LoadCase> &aInputLoadCases,
 const std::vector<XMLGen::Uncertainty> &aUncertainties)
{
    std::vector<XMLGen::LoadCase> tNewSetLoadCases;
    std::map<int, std::vector<int> > tOriginalToNewLoadCaseMap;
    Plato::srom::expand_load_cases(aInputLoadCases, tNewSetLoadCases, tOriginalToNewLoadCaseMap);

    std::set<int> tRandomLoadIDs;
    std::vector<Plato::srom::Load> tLoads;
    Plato::srom::create_random_load_variables
        (aUncertainties, tNewSetLoadCases, tOriginalToNewLoadCaseMap, tRandomLoadIDs, tLoads);

    Plato::srom::create_deterministic_load_variables(tNewSetLoadCases, tRandomLoadIDs, tLoads);

    return (tLoads);
}
// function generate_srom_load_inputs

/******************************************************************************//**
 * \fn preprocess_srom_problem_load_inputs
 * \brief Pre-process Stochastic Reduced Order Model (SROM) problem inputs.
 * \param [in/out] aInputMetadata Plato problem input metadata
 * \param [in/out] aLoadCases     set of load case metadata
 * \param [in/out] aUncertainties set of uncertainty metadata
**********************************************************************************/
inline std::vector<XMLGen::LoadCase>
preprocess_srom_problem_load_inputs
(const XMLGen::InputData& aInputMetadata)
{
    if(aInputMetadata.objectives.empty())
    {
        THROWERR("Pre-Process SROM Problem Load Inputs: Objective block is empty; hence, it is not defined.")
    }

    if(aInputMetadata.objectives.size() > 1u)
    {
        THROWERR(std::string("Pre-Process SROM Problem Load Inputs: Only one objective block is allowed to be defined") +
            "in stochastic use cases. Meaning, there cannot be more than one objective block defined in the Plato input file.")
    }

    std::vector<XMLGen::LoadCase> tLoadCases;
    for (auto& tObjective : aInputMetadata.objectives)
    {
        for (auto& tID : tObjective.load_case_ids)
        {
            for (auto& tLoadCase : aInputMetadata.load_cases)
            {
                if (tLoadCase.id == tID)
                {
                    tLoadCases.push_back(tLoadCase);
                    break;
                }
            }
        }
    }
    return tLoadCases;
}
// function preprocess_srom_problem_load_inputs

/******************************************************************************//**
 * \fn return_load_identification_numbers
 * \brief Return load identification numbers.
 * \param [in] aInputMetadata  Plato problem input metadata
 * \return load identification numbers
**********************************************************************************/
inline std::vector<std::string>
return_load_identification_numbers
(const XMLGen::InputData& aInputMetadata)
{
    std::vector<std::string> tOutput;
    for(auto& tLoadCase : aInputMetadata.load_cases)
    {
        for(auto& tLoad : tLoadCase.loads)
        {
            tOutput.push_back(tLoad.load_id);
        }
    }
    return tOutput;
}
// function return_load_identification_numbers

/******************************************************************************//**
 * \fn return_random_load_identification_numbers
 * \brief Return random load identification numbers.
 * \param [in] aInputMetadata  Plato problem input metadata
 * \return random load identification numbers
**********************************************************************************/
inline std::vector<std::string>
return_random_load_identification_numbers
(const XMLGen::InputData& aInputMetadata)
{
    std::vector<std::string> tOutput;
    for(auto& tUQCase : aInputMetadata.uncertainties)
    {
        if(tUQCase.category().compare("load") == 0)
        {
            tOutput.push_back(tUQCase.id());
        }
    }
    return tOutput;
}
// function return_random_load_identification_numbers

/******************************************************************************//**
 * \fn check_random_load_identification_numbers
 * \brief Throw an error if any load identification number in the uncertainty \n
 * blocks does not match any of the load identification numbers in the load blocks.
 * \param [in] aInputMetadata  Plato problem input metadata
**********************************************************************************/
inline void check_random_load_identification_numbers
(const XMLGen::InputData& aInputMetadata)
{
    auto tRandomLoadIDs = Plato::srom::return_random_load_identification_numbers(aInputMetadata);
    auto tDeterministicLoadIDs = Plato::srom::return_load_identification_numbers(aInputMetadata);
    for(auto& tID : tRandomLoadIDs)
    {
        auto tItr = std::find(tDeterministicLoadIDs.begin(), tDeterministicLoadIDs.end(), tID);
        if(tItr == tDeterministicLoadIDs.end())
        {
            THROWERR(std::string("Check Random Load Identification Numbers: Load ID '") + tID + "' is not a valid load id. "
                + "Any load id defined in the uncertainty blocks must match a load id defined in the load blocks.")
        }
    }
}
// function check_random_load_identification_numbers

/******************************************************************************//**
 * \fn preprocess_load_inputs
 * \brief Pre-process non-deterministic load inputs.
 * \param [in/out] aInputMetadata  Plato problem input metadata
 * \param [in/out] aSromInputs     Stochastic Reduced Order Model (SROM) problem metadata
**********************************************************************************/
inline void preprocess_load_inputs
(const XMLGen::InputData& aInputMetadata,
 Plato::srom::InputMetaData& aSromInputs)
{
    Plato::srom::check_random_load_identification_numbers(aInputMetadata);

    auto tCategoriesToUncertaintiesMap = Plato::srom::split_uncertainties_into_categories(aInputMetadata);
    auto tIterator = tCategoriesToUncertaintiesMap.find(Plato::srom::category::LOAD);
    if(tIterator == tCategoriesToUncertaintiesMap.end())
    {
        THROWERR(std::string("Pre-Process Non-Deterministic Load Inputs: Requested a stochastic use case; ")
            + "however, the objective has no associated non-deterministic loads, i.e. no uncertainty block "
            + "is associated with a load identification number.")
    }

    auto tActiveLoadCases = Plato::srom::preprocess_srom_problem_load_inputs(aInputMetadata);
    auto tLoads = Plato::srom::generate_srom_load_inputs(tActiveLoadCases, tIterator->second);
    aSromInputs.loads(tLoads);
}
// function preprocess_load_inputs

/******************************************************************************//**
 * \fn postprocess_load_outputs
 * \brief Post-process non-deterministic load outputs.
 * \param [in/out] aSromOutputs     SROM problem output metadata
 * \param [in/out] aXMLGenMetaData  Plato problem input metadata
**********************************************************************************/
inline void postprocess_load_outputs
(const Plato::srom::OutputMetaData& aSromOutputs,
 XMLGen::InputData& aXMLGenMetaData)
{
    if(aXMLGenMetaData.objectives.size() > 1u)
    {
        THROWERR("Post-Process SROM Problem Load Outputs: Only one objective function block is expected in a stochastic use case.")
    }

    auto tLoadCases = aSromOutputs.loads();
    for(auto& tLoadCase : tLoadCases)
    {
        XMLGen::LoadCase tNewLoadCase;
        auto tIndex = &tLoadCase - &tLoadCases[0];
        auto tLoadCaseID = tIndex + 1u;
        tNewLoadCase.id = std::to_string(tLoadCaseID);
        for (size_t tLoadIndex = 0; tLoadIndex < tLoadCase.numLoads(); ++tLoadIndex)
        {
            XMLGen::Load tNewLoad;
            tNewLoad.mIsRandom = tLoadCase.isRandom(tLoadIndex);
            tNewLoad.type = tLoadCase.loadType(tLoadIndex);
            tNewLoad.app_id = tLoadCase.applicationID(tLoadIndex);
            tNewLoad.app_type = tLoadCase.applicationType(tLoadIndex);
            tNewLoad.app_name = tLoadCase.applicationName(tLoadIndex);
            for (size_t tDim = 0; tDim < tLoadCase.numLoadValues(tLoadIndex); ++tDim)
            {
                tNewLoad.values.push_back(Plato::to_string(tLoadCase.loadValue(tLoadIndex, tDim)));
            }
            tNewLoad.load_id = tLoadCase.loadID(tLoadIndex);
            tNewLoadCase.loads.push_back(tNewLoad);
        }

        auto tNewLoadSet = std::make_pair(tLoadCase.probability(), tNewLoadCase);
        aXMLGenMetaData.mRandomMetaData.append(tNewLoadSet);
        aXMLGenMetaData.objectives[0].load_case_ids.push_back(tNewLoadCase.id);
        aXMLGenMetaData.objectives[0].load_case_weights.push_back("1");
    }
}
// function postprocess_load_outputs

/******************************************************************************//**
 * \fn check_output_load_set_types
 * \brief Check if random load cases all have the same mesh set, e.g. sideset, format.
 * \param [in] aMetaData  Random use case metadata
**********************************************************************************/
inline void check_output_load_set_types
(const XMLGen::RandomMetaData& aMetaData)
{
    for(size_t tSampleIndex = 0; tSampleIndex < aMetaData.numSamples(); tSampleIndex++)
    {
        auto tLoadUseCase = aMetaData.sample(tSampleIndex).loadcase();
        for(auto& tLoad : tLoadUseCase.loads)
        {
            if(tLoad.type != "traction")
            {
                auto tLoadIndex = &tLoad - &tLoadUseCase.loads[0];
                THROWERR(std::string("Check Output Load Set Application Name: Error with Sample '") + std::to_string(tSampleIndex)
                    + "'. Uncertainty workflow only supports load cases that all have the same format. Load '"
                    + std::to_string(tLoadIndex) + "' has its type set to '" + tLoad.type + "' and expected 'traction'.")
            }
        }
    }
}
// function check_output_load_set_types

/******************************************************************************//**
 * \fn check_output_load_set_application_name
 * \brief Check if random load cases all have the same mesh set, e.g. sideset, format.
 * \param [in] aMetaData  Random use case metadata
**********************************************************************************/
inline void check_output_load_set_application_name
(const XMLGen::RandomMetaData& aMetaData)
{
    for(size_t tIndex = 0; tIndex < aMetaData.numSamples() - 1; tIndex++)
    {
        auto tLoadUseCaseOne = aMetaData.sample(tIndex).loadcase();
        auto tLoadUseCaseTwo = aMetaData.sample(tIndex + 1u).loadcase();

        for(auto& tLoad : tLoadUseCaseOne.loads)
        {
            auto tLoadIndex = &tLoad - &tLoadUseCaseOne.loads[0];
            if(tLoad.app_name != tLoadUseCaseTwo.loads[tLoadIndex].app_name)
            {
                std::ostringstream tMsg;
                tMsg << "Check Output Load Set Application Name: Uncertainty workflow only supports load cases that all have the same mesh set format. "
                    << "Sample '" << std::to_string(tIndex) << "' has its application name set to '" << tLoad.app_name << "' and Sample '"
                    << std::to_string(tIndex + 1u) << "' has its application name set to '" << tLoadUseCaseTwo.loads[tLoadIndex].app_name << "'.";
                THROWERR(tMsg.str())
            }
        }
    }
}
// function check_output_load_set_application_name

/******************************************************************************//**
 * \fn check_output_load_set_size
 * \brief Check if random load cases all have the same number of random loads.
 * \param [in] aMetaData  Random use case metadata
**********************************************************************************/
inline void check_output_load_set_size
(const XMLGen::RandomMetaData& aMetaData)
{
    if(aMetaData.loadSamplesDrawn() == false) { return; }

    const auto tNumLoadsInLoadCase = aMetaData.loadcase().loads.size();
    for (size_t tIndex = 1; tIndex < aMetaData.numSamples(); tIndex++)
    {
        auto& tSample = aMetaData.sample(tIndex);
        if (tSample.loadcase().loads.size() != tNumLoadsInLoadCase)
        {
            THROWERR(std::string("Check Output Load Set Size: Size mismatch in Sample '") + std::to_string(tIndex)
               + "'. Uncertainty workflow only supports load cases that all have the same size.")
        }
    }
}
// function check_output_load_set_size

}
// namespace srom

}
// namespace Plato

