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
 * \enum Non-deterministic variable's category.  Is the non-deterministic variable \n
 * a material property or a load?
**********************************************************************************/
enum struct category
{
    MATERIAL = 0, LOAD = 1, UNDEFINED = 2,
};
// enum struct category

/*!< map from non-deterministic variable's category to list of non-deterministic variables' metadata */
using UncertaintyCategories = std::map<Plato::srom::category, std::vector<XMLGen::Uncertainty>>;
/******************************************************************************//**
 * \fn split_uncertainty_categories
 * \brief Split input uncertainties into categories.
 * \param [in]  aMetadata Plato problem input metadata
 * \return map from non-deterministic variable category to list of non-deterministic variables' metadata
**********************************************************************************/
inline UncertaintyCategories split_uncertainties_into_categories(const XMLGen::InputData& aMetadata)
{
    if(aMetadata.uncertainties.empty())
    {
        THROWERR(std::string("Split Uncertainty Into Categories: Input list of uncertainties is empty. ")
            + "Requested a stochastic use case but no uncertainty block was defined.")
    }

    Plato::srom::UncertaintyCategories tCategories;
    for(auto& tUncertainty : aMetadata.uncertainties)
    {
        auto tCategory = Plato::srom::tolower(tUncertainty.variable_type);
        auto tIsLoad = tCategory == "load" ? true : false;
        auto tIsMaterial = tCategory == "material" ? true : false;
        if (tIsLoad && !tIsMaterial)
        {
            tCategories[Plato::srom::category::LOAD].push_back(tUncertainty);
        }
        else if (tIsMaterial && !tIsLoad)
        {
            tCategories[Plato::srom::category::MATERIAL].push_back(tUncertainty);
        }
        else if (!tIsMaterial && !tIsLoad)
        {
            THROWERR(std::string("Split Uncertainty Into Categories: Non-deterministic category '") + tCategory + "' is not supported.")
        }
    }
    return tCategories;
}
// function split_uncertainty_categories

/*!< map from material identification number to map between material tag and list of non-deterministic variables' metadata */
using RandomMatPropMap = std::map<std::string, std::map<std::string, XMLGen::Uncertainty>>;
/******************************************************************************//**
 * \fn build_material_id_to_random_material_map
 * \brief Build map from material identification number to material tag-uncertainty pair.
 * \param [in] aRandomMatProperties list of random material properties and its metadata
 * \return map from material identification number to map between material tag and \n
 * list of non-deterministic variables' metadata
**********************************************************************************/
inline Plato::srom::RandomMatPropMap build_material_id_to_random_material_map
(const std::vector<XMLGen::Uncertainty>& aRandomMatProperties)
{
    Plato::srom::RandomMatPropMap tMap;
    for(auto& tRandomMatProp : aRandomMatProperties)
    {
        auto tIsMaterial = tRandomMatProp.variable_type == "material" ? true : false;
        if(!tIsMaterial)
        {
            THROWERR(std::string("Build Material ID to Random Material Map: Random variable with tag '")
                + tRandomMatProp.type + "' is not a material.")
        }
        tMap[tRandomMatProp.id].insert( {tRandomMatProp.type, tRandomMatProp} );
    }
    return tMap;
}
// function build_material_id_to_random_material_map

/******************************************************************************//**
 * \fn append_material_properties
 * \brief Append material properties.
 * \param [in]  aMaterial      input material metadata
 * \param [in]  aRandomMatMap  map from material identification number to map between \n
 *   material tag and list of non-deterministic variables' metadata
 * \param [out] aSromMaterial  Stochastic Reduced Order Model (SROM) material metadata
**********************************************************************************/
inline void append_material_properties
(const XMLGen::Material& aMaterial,
 const Plato::srom::RandomMatPropMap& aRandomMatMap,
 Plato::srom::Material& aSromMaterial)
{
    if(aRandomMatMap.empty())
    {
        THROWERR(std::string("Append Random Material: Map from material identification number ")
            + "to map from material tag to list of non-deterministic variables' metadata is empty.")
    }

    auto tMatID = aMaterial.id();
    aSromMaterial.materialID(tMatID);
    aSromMaterial.category(aMaterial.category());
    auto tTags = aMaterial.tags();

    for (auto &tTag : tTags)
    {
        auto tRandMatMapItr = aRandomMatMap.find(tMatID);
        auto tIsMaterialRandom = tRandMatMapItr != aRandomMatMap.end() ? true : false;
        if(tIsMaterialRandom)
        {
            auto tIterator = tRandMatMapItr->second.find(tTag);
            auto tIsMaterialPropertyRandom = tIterator != tRandMatMapItr->second.end() ? true : false;
            if(tIsMaterialPropertyRandom)
            {
                Plato::srom::Statistics tStats;
                tStats.mFile = tIterator->second.file;
                tStats.mMean = tIterator->second.mean;
                tStats.mUpperBound = tIterator->second.upper;
                tStats.mLowerBound = tIterator->second.lower;
                tStats.mNumSamples = tIterator->second.num_samples;
                tStats.mDistribution = tIterator->second.distribution;
                tStats.mStandardDeviation = tIterator->second.standard_deviation;
                aSromMaterial.append(tTag, aMaterial.attribute(), tStats);
            }
            else
            {
                aSromMaterial.append(tTag, aMaterial.attribute(), aMaterial.property(tTag));
            }
        }
        else
        {
            aSromMaterial.append(tTag, aMaterial.attribute(), aMaterial.property(tTag));
        }
    }
}
// function append_material_properties

/******************************************************************************//**
 * \fn preprocess_nondeterministic_material_inputs
 * \brief Pre-process non-deterministic material inputs, i.e. prepare inputs for \n
 *   Stochastic Reducded Order Model (SROM) problem.
 * \param [in] aInputMetadata  Plato problem input metadata
 * \param [in] aSromInputs     SROM problem input metadata
**********************************************************************************/
inline void preprocess_nondeterministic_material_inputs
(XMLGen::InputData& aInputMetadata, Plato::srom::InputMetaData& aSromInputs)
{
    if(aInputMetadata.materials.empty())
    {
        THROWERR(std::string("Pre-Process Non-Deterministic Material Inputs: Input list of materials is empty. ")
            + "Plato problem has no material defined in input file.")
    }

    auto tCategoriesToUncertaintiesMap = Plato::srom::split_uncertainties_into_categories(aInputMetadata);
    auto tIterator = tCategoriesToUncertaintiesMap.find(Plato::srom::category::MATERIAL);
    if(tIterator == tCategoriesToUncertaintiesMap.end())
    {
        THROWERR(std::string("Pre-Process Non-Deterministic Material Inputs: Requested a stochastic use case; ")
            + "however, the objective has no associated non-deterministic materials, i.e. no uncertainty block "
            + "is associated with a material identification number.")
    }

    auto aRandomMatPropMap = Plato::srom::build_material_id_to_random_material_map(tIterator->second);
    for(auto& tMaterial : aInputMetadata.materials)
    {
        Plato::srom::Material tSromMaterial;
        Plato::srom::append_material_properties(tMaterial, aRandomMatPropMap, tSromMaterial);
        aSromInputs.append(tSromMaterial);
    }
}
// function preprocess_nondeterministic_material_inputs

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
        PRINTERR("FAILED TO INITIALIZE ORIGINAL SET OF LOAD IDENTIFIERS.\n");
        return (false);
    }

    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < aOldLoadCases.size(); tLoadCaseIndex++)
    {
        const XMLGen::LoadCase& tOldLoadCase = aOldLoadCases[tLoadCaseIndex];
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
 * \fn define_srom_problem_usecase
 * \brief Define use case for Stochastic Reduced Order Model (SROM) problem
 * \param [in]  aUncertaintySet    set of input uncertainty blocks
 * \param [out] aSromInputMetadata SROM input metadata
**********************************************************************************/
inline void define_srom_problem_usecase
(const std::vector<XMLGen::Uncertainty>& aUncertaintySet,
 Plato::srom::InputMetaData& aSromInputMetadata)
{
    std::vector<std::string> tCategory;
    std::vector<std::string> tSupportedCategories = {"load", "material"};
    for(auto& tUncertainty : aUncertaintySet)
    {
        auto tMyCategory = Plato::srom::tolower(tUncertainty.variable_type);
        auto tIsSupported = std::find(tSupportedCategories.begin(), tSupportedCategories.end(), tMyCategory) != tSupportedCategories.end();
        if (!tIsSupported)
        {
            THROWERR(std::string("Define SROM Problem Use Case: Uncertainty category '") + tMyCategory + "' is not supported.")
        }
        tCategory.push_back(tUncertainty.variable_type);
    }

    bool tFoundLoadUseCase = std::find(tCategory.begin(), tCategory.end(), "load") != tCategory.end();
    bool tFoundMaterialUseCase = std::find(tCategory.begin(), tCategory.end(), "material") != tCategory.end();
    if(tFoundLoadUseCase && !tFoundMaterialUseCase)
    {
        aSromInputMetadata.usecase(Plato::srom::usecase::LOAD);
    }
    else if(!tFoundLoadUseCase && tFoundMaterialUseCase)
    {
        aSromInputMetadata.usecase(Plato::srom::usecase::MATERIAL);
    }
    else if(tFoundLoadUseCase && tFoundMaterialUseCase)
    {
        aSromInputMetadata.usecase(Plato::srom::usecase::MATERIAL_PLUS_LOAD);
    }
    else
    {
        aSromInputMetadata.usecase(Plato::srom::usecase::UNDEFINED);
        THROWERR(std::string("Define SROM Problem Use Case: Identified an undefined SROM problem use-case."))
    }
}
// function define_srom_problem_usecase

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
    std::vector<XMLGen::LoadCase> tLoadCases;
    const XMLGen::Objective &tCurObj = aInputMetadata.objectives[0];
    for (size_t tLoadCaseID = 0; tLoadCaseID < tCurObj.load_case_ids.size(); ++tLoadCaseID)
    {
        const std::string &tCurLoadCaseID = tCurObj.load_case_ids[tLoadCaseID];
        for (size_t tLoadCaseIndex = 0; tLoadCaseIndex < aInputMetadata.load_cases.size(); ++tLoadCaseIndex)
        {
            if (aInputMetadata.load_cases[tLoadCaseIndex].id == tCurLoadCaseID)
            {
                tLoadCases.push_back(aInputMetadata.load_cases[tLoadCaseIndex]);
                tLoadCaseIndex = aInputMetadata.load_cases.size();
            }
        }
    }
    return tLoadCases;
}
// function preprocess_srom_problem_load_inputs

/******************************************************************************//**
 * \fn postprocess_srom_problem_load_outputs
 * \brief Post-process Stochastic Reduced Order Model (SROM) problem outputs.
 * \param [in/out] aOutputs               SROM problem output metadata
 * \param [in/out] aInputData             Plato problem metadata
 * \param [in/out] aNewLoadCases          load case metadata
 * \param [in/out] aLoadCaseProbabilities load case probabilities
**********************************************************************************/
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
    auto tLoadCases = aOutputs.loadCases();
    for(auto& tLoadCase : tLoadCases)
    {
        XMLGen::LoadCase tNewLoadCase;
        auto tLoadCaseID = Plato::to_string(tStartingLoadCaseID);
        tNewLoadCase.id = tLoadCaseID;
        auto tLoadCaseIndex = &tLoadCase - &tLoadCases[0];

        for (size_t tLoadIndex = 0; tLoadIndex < tLoadCases[tLoadCaseIndex].numLoads(); ++tLoadIndex)
        {
            XMLGen::Load tNewLoad;
            tNewLoad.type = tLoadCases[tLoadCaseIndex].loadType(tLoadIndex);
            tNewLoad.app_id = tLoadCases[tLoadCaseIndex].applicationID(tLoadIndex);
            tNewLoad.app_type = tLoadCases[tLoadCaseIndex].applicationType(tLoadIndex);
            tNewLoad.app_name = tLoadCases[tLoadCaseIndex].applicationName(tLoadIndex);
            for (size_t tDim = 0; tDim < tLoadCases[tLoadCaseIndex].numLoadValues(tLoadIndex); ++tDim)
            {
                tNewLoad.values.push_back(Plato::to_string(tLoadCases[tLoadCaseIndex].loadValue(tLoadIndex, tDim)));
            }
            tNewLoad.load_id = tLoadCases[tLoadCaseIndex].loadID(tLoadIndex);
            tNewLoadCase.loads.push_back(tNewLoad);
        }
        aNewLoadCases.push_back(tNewLoadCase);
        aLoadCaseProbabilities.push_back(tLoadCases[tLoadCaseIndex].probability());
        tCurObj.load_case_ids.push_back(tLoadCaseID);
        tCurObj.load_case_weights.push_back("1");
        tStartingLoadCaseID++;
    }
}
// function postprocess_srom_problem_load_outputs

/******************************************************************************//**
 * \fn preprocess_nondeterministic_load_inputs
 * \brief Pre-process non-deterministic load inputs.
 * \param [in/out] aInputMetadata  Plato problem input metadata
 * \param [in/out] aSromInputs     Stochastic Reduced Order Model (SROM) problem metadata
**********************************************************************************/
inline void preprocess_nondeterministic_load_inputs
(const XMLGen::InputData& aInputMetadata,
 Plato::srom::InputMetaData& aSromInputs)
{
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
// function preprocess_nondeterministic_load_inputs

/******************************************************************************//**
 * \fn preprocess_srom_problem_inputs
 * \brief Pre-process Stochastic Reduced Order Model (SROM) problem inputs.
 * \param [in/out] aInputMetadata  Plato problem input metadata
 * \param [in/out] aSromInputs     SROM problem metadata
**********************************************************************************/
inline void preprocess_srom_problem_inputs
(XMLGen::InputData& aInputMetadata,
 Plato::srom::InputMetaData& aSromInputs)
{
    switch(aSromInputs.usecase())
    {
        case Plato::srom::usecase::LOAD:
        {
            Plato::srom::preprocess_nondeterministic_load_inputs(aInputMetadata, aSromInputs);
            break;
        }
        case Plato::srom::usecase::MATERIAL:
        {
            Plato::srom::preprocess_nondeterministic_material_inputs(aInputMetadata, aSromInputs);
            break;
        }
        case Plato::srom::usecase::MATERIAL_PLUS_LOAD:
        {
            Plato::srom::preprocess_nondeterministic_load_inputs(aInputMetadata, aSromInputs);
            Plato::srom::preprocess_nondeterministic_material_inputs(aInputMetadata, aSromInputs);
            break;
        }
        default:
        case Plato::srom::usecase::UNDEFINED:
        {
            THROWERR("Pre-Process SROM Problem Inputs: Detected an undefined SROM problem use case.")
            break;
        }
    }
}
// function preprocess_srom_problem_inputs

/******************************************************************************//**
 * \fn post_process_nondeterministic_load_outputs
 * \brief Post-process non-deterministic load outputs.
 * \param [in/out] aSromOutputs    SROM problem output metadata
 * \param [in/out] aInputMetadata  Plato problem input metadata
**********************************************************************************/
inline void post_process_nondeterministic_load_outputs
(const Plato::srom::OutputMetaData& aSromOutputs,
 XMLGen::InputData& aInputMetadata)
{
    std::vector<double> tLoadCaseProbabilities;
    std::vector<XMLGen::LoadCase> tNewLoadCases;
    Plato::srom::postprocess_srom_problem_load_outputs(aSromOutputs, aInputMetadata, tNewLoadCases, tLoadCaseProbabilities);
    aInputMetadata.load_cases = tNewLoadCases;
    aInputMetadata.load_case_probabilities = tLoadCaseProbabilities;
}
// function post_process_nondeterministic_load_outputs

/******************************************************************************//**
 * \fn postprocess_srom_problem_outputs
 * \brief Post-process Stochastic Reduced Order Model (SROM) problem outputs.
 * \param [in/out] aSromOutputs    SROM problem output metadata
 * \param [in/out] aInputMetadata  Plato problem input metadata
**********************************************************************************/
inline void postprocess_srom_problem_outputs
(const Plato::srom::OutputMetaData& aSromOutputs,
 XMLGen::InputData& aInputMetadata)
{
    switch(aSromOutputs.usecase())
    {
        case Plato::srom::usecase::LOAD:
        {
            Plato::srom::post_process_nondeterministic_load_outputs(aSromOutputs, aInputMetadata);
            break;
        }
        case Plato::srom::usecase::MATERIAL:
        {
            break;
        }
        case Plato::srom::usecase::MATERIAL_PLUS_LOAD:
        {
            Plato::srom::post_process_nondeterministic_load_outputs(aSromOutputs, aInputMetadata);
            break;
        }
        default:
        case Plato::srom::usecase::UNDEFINED:
        {
            THROWERR("Post-Process SROM Problem Outputs: Detected an undefined SROM problem use case.")
            break;
        }
    }
}
// function postprocess_srom_problem_outputs

}
// namespace srom

}
// namespace Plato

