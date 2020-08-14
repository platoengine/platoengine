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
 * \brief Create a deterministic load from the input deterministic load
 * \param [in]  aInputLoad deterministic input load metadata
 * \param [out] aLoad deterministic Plato::srom::Load metadata
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool create_deterministic_load_variable(const XMLGen::Load &aInputLoad, Plato::srom::Load& aLoad)
{
    aLoad.mLoadID = aInputLoad.load_id;
    aLoad.mLoadType = aInputLoad.type;
    aLoad.mAppType = aInputLoad.app_type;
    aLoad.mAppName = aInputLoad.app_name;
    aLoad.mAppID = aInputLoad.app_id.empty() ? std::numeric_limits<int>::max() : std::stoi(aInputLoad.app_id);
    if(aInputLoad.app_id.empty() && aInputLoad.app_name.empty())
    {
        THROWERR(std::string("Create Deterministic Load Variable: Mesh set, e.g. sideset or nodeset, identification ")
            + "number and name are not defined. One of the two mesh-set identifiers, identification number or name, "
            + "must be define.")
    }

    for (auto &tValue : aInputLoad.values)
    {
        aLoad.mValues.push_back(tValue);
    }

    return (true);
}
// function create_deterministic_load_variable

/******************************************************************************//**
 * \brief Get or create a vector of Plato::srom::Loads given a random input load
 * \param [in] aInputLoad   random input load metadata 
 * \param [out] aRandomLoads set of random loads
 * \return random load identification number
**********************************************************************************/
inline int create_random_load
(const XMLGen::Load &aInputLoad,
 std::vector<Plato::srom::Load> &aRandomLoads)
{
    for(auto& tRandomLoad : aRandomLoads)
    {
        if(tRandomLoad.mLoadID == aInputLoad.load_id)
        {
            auto tRandomLoadIndex = &tRandomLoad - &aRandomLoads[0];
            return (tRandomLoadIndex);
        }
    }

    Plato::srom::Load tNewLoad;
    tNewLoad.mLoadID = aInputLoad.load_id;
    tNewLoad.mLoadType = aInputLoad.type;
    tNewLoad.mAppType = aInputLoad.app_type;

    tNewLoad.mAppName = aInputLoad.app_name;
    tNewLoad.mAppID = aInputLoad.app_id.empty() ? std::numeric_limits<int>::max() : std::stoi(aInputLoad.app_id);
    if(aInputLoad.app_id.empty() && aInputLoad.app_name.empty())
    {
        THROWERR(std::string("Create Random Load: Mesh set, e.g. sideset or nodeset, identification ")
            + "number and name are not defined. One of the two mesh-set identifiers, identification "
            + "number or name, must be define.")
    }

    for(auto& tValue : aInputLoad.values)
    {
        tNewLoad.mValues.push_back(tValue);
    }
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
    tNewVariable.tag(aRandomVariable.tag());
    tNewVariable.attribute(aRandomVariable.attribute());
    tNewVariable.mean(aRandomVariable.mean());
    tNewVariable.lower(aRandomVariable.lower());
    tNewVariable.upper(aRandomVariable.upper());
    tNewVariable.samples(aRandomVariable.samples());
    tNewVariable.distribution(aRandomVariable.distribution());
    tNewVariable.deviation(aRandomVariable.std());
    aRandomLoad.mRandomVars.push_back(tNewVariable);
}
// function add_random_variable_to_random_load

/******************************************************************************//**
 * \brief Create a random load and append it to the set of random loads
 * \param [in] aRandomVariable random variable metadata
 * \param [in] aInputLoads set of loads (deterministic plus random loads associated with uncertain usecase service)
 * \param [out] aRandomLoadIDs set of random load identifiers
 * \param [out] aRandomLoads set of random loads
**********************************************************************************/
inline void create_random_load_from_uncertainty(const XMLGen::Uncertainty& aRandomVariable,
                                                 const std::vector<XMLGen::Load> &aInputLoads,
                                                 std::set<int> &aRandomLoadIDs,
                                                 std::vector<Plato::srom::Load> &aRandomLoads)
{
    XMLGen::Load tInputLoad;
    for(auto tLoad: aInputLoads)
    {
        if(tLoad.load_id == aRandomVariable.id())
        {
            tInputLoad = tLoad;
            break;
        }
    }
    const int tIndexOfRandomLoad = Plato::srom::create_random_load(tInputLoad, aRandomLoads);
    Plato::srom::add_random_variable_to_random_load(aRandomLoads[tIndexOfRandomLoad], aRandomVariable);
}
// function create_random_load_from_uncertainty

/******************************************************************************//**
 * \brief Create set of random loads
 * \param [in] aRandomVariable random variable metadata
 * \param [in] aInputLoads set of loads (deterministic plus random load associated with uncertain usecase service)
 * \param [out] aRandomLoadIDs set of random load identifiers
 * \param [out] aRandomLoads set of random loads
**********************************************************************************/
inline void create_random_load_variables(const std::vector<XMLGen::Uncertainty> &aRandomVariables,
                                         const std::vector<XMLGen::Load> &aInputLoads,
                                         std::set<int> &aRandomLoadIDs,
                                         std::vector<Plato::srom::Load> &aLoad)
{
    for(auto& tRandomVar : aRandomVariables)
    {
        Plato::srom::VariableType::type_t tVarType = Plato::srom::VariableType::UNDEFINED;
        Plato::srom::variable_type_string_to_enum(tRandomVar.category(), tVarType);
        if(tVarType == Plato::srom::VariableType::LOAD)
        {
            Plato::srom::create_random_load_from_uncertainty(tRandomVar, aInputLoads, aRandomLoadIDs, aLoad);
        }
    }
}
// function create_random_load_variables

/******************************************************************************//**
 * \brief Create set of deterministic loads
 * \param [in] aInputLoads set of loads (deterministic plus random loads associated with uncertain usecase)
 * \param [in] aRandomLoadIDs set of random load case identifiers
 * \param [out] aDeterministicLoads set of deterministic load cases
**********************************************************************************/
inline void create_deterministic_load_variables(const std::vector<XMLGen::Load> &aInputLoads,
                                                const std::set<int> & aRandomLoadIDs,
                                                std::vector<Plato::srom::Load> &aDeterministicLoads)
{
    for(auto& tLoad : aInputLoads)
    {
        auto tCurLoadID = std::stoi(tLoad.load_id);
        if(aRandomLoadIDs.find(tCurLoadID) == aRandomLoadIDs.end())
        {
            Plato::srom::Load tNewLoad;
            Plato::srom::create_deterministic_load_variable(tLoad, tNewLoad);
            aDeterministicLoads.push_back(tNewLoad);
        }
    }
}
// function create_deterministic_load_variables

/******************************************************************************//**
 * \brief Generate array of deterministic and random loads in the format expected \n
 * by the stochastic reduced order model application programming interface.
 * \param [in] aInputLoads loads created by the XML generator
 * \param [in] aUncertainties set of random variables created by the XML generator
 * \return set of deterministic and random loads in the format expected \n
 *   by the Stochastic Reduced Order Model (SROM) interface
**********************************************************************************/
inline std::vector<Plato::srom::Load>
generate_srom_load_inputs
(const std::vector<XMLGen::Load> &aInputLoads,
 const std::vector<XMLGen::Uncertainty> &aUncertainties)
{
    std::set<int> tRandomLoadIDs;
    std::vector<Plato::srom::Load> tLoads;
    Plato::srom::create_random_load_variables(aUncertainties, aInputLoads, tRandomLoadIDs, tLoads);
    Plato::srom::create_deterministic_load_variables(aInputLoads, tRandomLoadIDs, tLoads);

    return (tLoads);
}
// function generate_srom_load_inputs

/******************************************************************************//**
 * \fn preprocess_srom_problem_load_inputs
 * \brief Pre-process Stochastic Reduced Order Model (SROM) problem inputs.
 * \param [in/out]    aInputMetadata Plato problem input metadata
 * \return Loads      The set of loads for the uncertain usecase service 
**********************************************************************************/
inline std::vector<XMLGen::Load>
preprocess_srom_problem_load_inputs
(const XMLGen::InputData& aInputMetadata)
{
    if(aInputMetadata.services().size() != 1u)
    {
        THROWERR(std::string("Pre-Process SROM Problem Load Inputs: Exactly one service block is expected ") +
            "in stochastic use cases. Meaning, there cannot be more than one service block defined in the Plato input file.")
    }

    std::vector<XMLGen::Load> tLoads;
    XMLGen::Service tService = aInputMetadata.services()[0];
    for (auto& tID : tService.LoadIDs)
    {
        for (auto& tLoad : aInputMetadata.loads)
        {
            if (tLoad.load_id == tID)
            {
                tLoads.push_back(tLoad);
                break;
            }
        }
    }
    return tLoads;
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
    for(auto& tLoad : aInputMetadata.loads)
    {
        tOutput.push_back(tLoad.load_id);
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
            + "however, the service has no associated non-deterministic loads, i.e. no uncertainty block "
            + "is associated with a load identification number.")
    }

    auto tActiveLoads = Plato::srom::preprocess_srom_problem_load_inputs(aInputMetadata);
    auto tLoads = Plato::srom::generate_srom_load_inputs(tActiveLoads, tIterator->second);
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
    if(aXMLGenMetaData.services().size() != 1u)
    {
        THROWERR("Post-Process SROM Problem Load Outputs: Exactly one service block is expected in a stochastic use case.")
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

