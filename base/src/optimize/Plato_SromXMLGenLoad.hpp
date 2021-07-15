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
#include "XMLGeneratorLoadMetadata.hpp"

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
 * \brief Create a deterministic load from the input deterministic load
 * \param [in] aLoad deterministic load metadata 
 * \param [out] aLoad deterministic load metadata
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline bool create_deterministic_load_variable(const XMLGen::Load &aLoad, Plato::srom::Load& aNewLoad)
{
    aNewLoad.mLoadID = aLoad.id();
    aNewLoad.mLoadType = aLoad.type();
    aNewLoad.mAppType = aLoad.location_type();
    aNewLoad.mAppName = aLoad.location_name();
    aNewLoad.mAppID = aLoad.location_id().empty() ? std::numeric_limits<int>::max() : std::stoi(aLoad.location_id());
    if(aLoad.location_id().empty() && aLoad.location_name().empty())
    {
        THROWERR(std::string("Create Deterministic Load Variable: Mesh set, e.g. sideset or nodeset, identification ")
            + "number and name are not defined. One of the two mesh-set identifiers, identification number or name, "
            + "must be define.")
    }

    for (auto &tValue : aLoad.load_values())
    {
        aNewLoad.mValues.push_back(tValue);
    }

    return (true);
}
// function create_deterministic_load_variable

/******************************************************************************//**
 * \brief Get or create a random load given a random load
 * \param [in] aLoad random load metadata
 * \param [out] aRandomLoad set of random loads
 * \return random load identification number
**********************************************************************************/
inline int create_random_load
(const XMLGen::Load &aLoad,
 std::vector<Plato::srom::Load> &aRandomLoads)
{
    for(auto& tRandomLoad : aRandomLoads)
    {
        if(tRandomLoad.mLoadID == aLoad.id())
        {
            auto tRandomLoadIndex = &tRandomLoad - &aRandomLoads[0];
            return (tRandomLoadIndex);
        }
    }

    Plato::srom::Load tNewLoad;
    tNewLoad.mLoadID = aLoad.id();
    tNewLoad.mLoadType = aLoad.type();
    tNewLoad.mAppType = aLoad.location_type();
    tNewLoad.mAppName = aLoad.location_name();
    tNewLoad.mAppID = aLoad.location_id().empty() ? std::numeric_limits<int>::max() : std::stoi(aLoad.location_id());
    if(aLoad.location_id().empty() && aLoad.location_name().empty())
    {
        THROWERR(std::string("Create Random Load: Mesh set, e.g. sideset or nodeset, identification ")
                + "number and name are not defined. One of the two mesh-set identifiers, identification "
                + "number or name, must be define.")
    }

    for(auto& tValue : aLoad.load_values())
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
    tNewVariable.seed(aRandomVariable.seed());
    tNewVariable.mean(aRandomVariable.mean());
    tNewVariable.guess(aRandomVariable.guess());
    tNewVariable.lower(aRandomVariable.lower());
    tNewVariable.upper(aRandomVariable.upper());
    tNewVariable.deviation(aRandomVariable.std());
    tNewVariable.filename(aRandomVariable.filename());
    tNewVariable.samples(aRandomVariable.samples());
    tNewVariable.attribute(aRandomVariable.attribute());
    tNewVariable.dimensions(aRandomVariable.dimensions());
    tNewVariable.distribution(aRandomVariable.distribution());
    tNewVariable.correlationFilename(aRandomVariable.correlationFilename());
    aRandomLoad.mRandomVars.push_back(tNewVariable);
}
// function add_random_variable_to_random_load

/******************************************************************************//**
 * \brief Create a random load and append it to the set of random load cases
 * \param [in] aRandomVariable random variable metadata
 * \param [in] aLoads set of loads (deterministic plus random loads)
 * \param [out] aRandomLoadIDs set of random load case identifiers
 * \param [out] aRandomLoads set of random load cases
**********************************************************************************/
inline void create_random_loads_from_uncertainty(const XMLGen::Uncertainty& aRandomVariable,
                                                 const std::vector<XMLGen::Load> &aLoads,
                                                 std::set<int> &aRandomLoadIDs,
                                                 std::vector<Plato::srom::Load> &aRandomLoads)
{
    if(aRandomVariable.id().empty())
    {
        THROWERR("Input random load identification number (id) is empty.")
    }

    for(size_t tIndexJ = 0; tIndexJ < aLoads.size(); tIndexJ++)
    {
        if(aRandomVariable.id() == aLoads[tIndexJ].id())
        {
            const auto tCurLoadID = std::stoi(aLoads[tIndexJ].id());
            aRandomLoadIDs.insert(tCurLoadID);
            const auto tIndexOfRandomLoad = Plato::srom::create_random_load(aLoads[tIndexJ], aRandomLoads);
            Plato::srom::add_random_variable_to_random_load(aRandomLoads[tIndexOfRandomLoad], aRandomVariable);
            break;
        }
    }
}
// function create_random_loads_from_uncertainty

/******************************************************************************//**
 * \brief Create set of random loads
 * \param [in] aRandomVariable random variable metadata
 * \param [in] aLoads set of load (deterministic plus random loads)
 * \param [out] aRandomLoadIDs set of random load case identifiers
 * \param [out] aRandomLoads set of random load cases
**********************************************************************************/
inline void create_random_load_variables(const std::vector<XMLGen::Uncertainty> &aRandomVariables,
                                         const std::vector<XMLGen::Load> &aLoads,
                                         std::set<int> &aRandomLoadIDs,
                                         std::vector<Plato::srom::Load> &aRandomLoads)
{
    for(auto& tRandomVar : aRandomVariables)
    {
        Plato::srom::VariableType::type_t tVarType = Plato::srom::VariableType::UNDEFINED;
        Plato::srom::variable_type_string_to_enum(tRandomVar.category(), tVarType);
        if(tVarType == Plato::srom::VariableType::LOAD)
        {
            Plato::srom::create_random_loads_from_uncertainty(tRandomVar, aLoads, aRandomLoadIDs, aRandomLoads);
        }
    }
}
// function create_random_load_variables

/******************************************************************************//**
 * \brief Create set of deterministic loads
 * \param [in] aLoads set of loads (deterministic plus random loads)
 * \param [in] aRandomLoadIDs set of random load case identifiers
 * \param [out] aDeterministicLoads set of deterministic load cases
**********************************************************************************/
inline void create_deterministic_load_variables(const std::vector<XMLGen::Load> &aLoads,
                                                const std::set<int> & aRandomLoadIDs,
                                                std::vector<Plato::srom::Load> &aDeterministicLoads)
{
    for(auto& tLoad : aLoads)
    {
        if(tLoad.id().empty())
        {
            THROWERR("Load identification number (id) is empty.")
        }
        auto tCurLoadID = std::stoi(tLoad.id());
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
 * \param [in] aInputLoads set of loads created by the XML generator
 * \param [in] aUncertainties set of random variables created by the XML generator
 * \param [out] aLoads set of deterministic and random loads in the format expected \n
 *   by the Stochastic Reduced Order Model (SROM) interface
 * \return error flag - function call was successful, true = no error, false = error
**********************************************************************************/
inline std::vector<Plato::srom::Load>
generate_srom_load_inputs
(const std::vector<XMLGen::Load> &aInputLoads,
 const std::vector<XMLGen::Uncertainty> &aUncertainties)
{
    std::set<int> tRandomLoadIDs;
    std::vector<Plato::srom::Load> tAllLoads;
    Plato::srom::create_random_load_variables
        (aUncertainties, aInputLoads, tRandomLoadIDs, tAllLoads);
    Plato::srom::create_deterministic_load_variables(aInputLoads, tRandomLoadIDs, tAllLoads);

    return (tAllLoads);
}
// function generate_srom_load_inputs

/******************************************************************************//**
 * \fn preprocess_srom_problem_load_inputs
 * \brief Pre-process Stochastic Reduced Order Model (SROM) problem inputs.
 * \param [in/out] aInputMetadata Plato problem input metadata
 * \param [in/out] aLoads         set of load metadata
 * \param [in/out] aUncertainties set of uncertainty metadata
**********************************************************************************/
inline std::vector<XMLGen::Load>
preprocess_srom_problem_load_inputs
(const XMLGen::InputData& aInputMetadata)
{
    if(aInputMetadata.objective.scenarioIDs.empty())
    {
        THROWERR("Pre-Process SROM Problem Load Inputs: Objective block is empty; hence, it is not defined.")
    }

    if(aInputMetadata.objective.scenarioIDs.size() > 1u)
    {
        THROWERR(std::string("Pre-Process SROM Problem Load Inputs: Only one objective block is allowed to be defined") +
            "in stochastic use cases. Meaning, there cannot be more than one objective block defined in the Plato input file.")
    }

    std::vector<XMLGen::Load> tLoads;
    for (auto &tScenarioID : aInputMetadata.objective.scenarioIDs)
    {
        auto &tScenario = aInputMetadata.scenario(tScenarioID);
        for(auto &tLoadID : tScenario.loadIDs())
        {
            for(auto &tLoad : aInputMetadata.loads)
            {
                if(tLoad.id() == tLoadID)
                {
                    tLoads.push_back(tLoad);
                    break;
                }
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
    for(auto& tScenarioID : aInputMetadata.objective.scenarioIDs)
    {
        auto &tScenario = aInputMetadata.scenario(tScenarioID);
        for(auto &tLoadID : tScenario.loadIDs())
        {
            tOutput.push_back(tLoadID);
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

    auto tActiveLoads = Plato::srom::preprocess_srom_problem_load_inputs(aInputMetadata);
    auto tLoads = Plato::srom::generate_srom_load_inputs(tActiveLoads, tIterator->second);
    aSromInputs.loads(tLoads);
}
// function preprocess_load_inputs

/******************************************************************************//**
 * \fn load_value_to_string
 * \brief Take a load value and convert it to a string that is acceptable in a Teuchos MathExor
 * \param [in] aLoadValue Numeric value of the load
 * \return  string representation of the load
**********************************************************************************/
inline std::string load_value_to_string(const double aLoadValue)
{
    if (aLoadValue == 0.0)
        return std::string("0.0");

    std::string tLoadString = Plato::to_string(aLoadValue);
    size_t tIndexOfE = tLoadString.find('e');
    if (tIndexOfE == std::string::npos)
        return tLoadString;
    
    size_t tNumCharsToErase = 0;
    for (size_t tStringIndex = tIndexOfE + 1; tStringIndex < tLoadString.size(); ++tStringIndex)
    {
        if ( (tLoadString[tStringIndex] == '0') || (tLoadString[tStringIndex] == '+') )
            ++tNumCharsToErase;
        else
            break;
    }
    if (tNumCharsToErase > 0)
        tLoadString.erase(tIndexOfE + 1, tNumCharsToErase);
    
    return tLoadString;
}

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
    if(aXMLGenMetaData.objective.scenarioIDs.size() > 1u)
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
            tNewLoad.is_random((tLoadCase.isRandom(tLoadIndex) ? "true" : "false"));
            tNewLoad.type(tLoadCase.loadType(tLoadIndex));
            tNewLoad.location_id(tLoadCase.applicationID(tLoadIndex));
            tNewLoad.location_type(tLoadCase.applicationType(tLoadIndex));
            tNewLoad.location_name(tLoadCase.applicationName(tLoadIndex));
            std::vector<std::string> tValues;
            for (size_t tDim = 0; tDim < tLoadCase.numLoadValues(tLoadIndex); ++tDim)
            {
                tValues.push_back(Plato::srom::load_value_to_string(tLoadCase.loadValue(tLoadIndex, tDim)));
            }
            tNewLoad.load_values(tValues);
            tNewLoad.id(tLoadCase.loadID(tLoadIndex));
            tNewLoadCase.loads.push_back(tNewLoad);
        }

        auto tNewLoadSet = std::make_pair(tLoadCase.probability(), tNewLoadCase);
        aXMLGenMetaData.mRandomMetaData.append(tNewLoadSet);
//        aXMLGenMetaData.objectives[0].load_case_ids.push_back(tNewLoadCase.id);
//        aXMLGenMetaData.objectives[0].load_case_weights.push_back("1");
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
            if(tLoad.type() != "traction")
            {
                auto tLoadIndex = &tLoad - &tLoadUseCase.loads[0];
                THROWERR(std::string("Check Output Load Set Application Name: Error with Sample '") + std::to_string(tSampleIndex)
                    + "'. Uncertainty workflow only supports load cases that all have the same format. Load '"
                    + std::to_string(tLoadIndex) + "' has its type set to '" + tLoad.type() + "' and expected 'traction'.")
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
            if(tLoad.location_name() != tLoadUseCaseTwo.loads[tLoadIndex].location_name())
            {
                std::ostringstream tMsg;
                tMsg << "Check Output Load Set Application Name: Uncertainty workflow only supports load cases that all have the same mesh set format. "
                    << "Sample '" << std::to_string(tIndex) << "' has its application name set to '" << tLoad.location_name() << "' and Sample '"
                    << std::to_string(tIndex + 1u) << "' has its application name set to '" << tLoadUseCaseTwo.loads[tLoadIndex].location_name() << "'.";
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

