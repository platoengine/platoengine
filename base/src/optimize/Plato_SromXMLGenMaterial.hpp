/*
 * Plato_SromXMLGenMaterial.hpp
 *
 *  Created on: May 11, 2020
 */

#pragma once

#include <unordered_map>

#include "Plato_SromXMLGenHelpers.hpp"
#include "Plato_SromXML.hpp"

namespace Plato
{

namespace srom
{

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
                aSromMaterial.append(tTag, aMaterial.attribute(tTag), tStats);
            }
            else
            {
                aSromMaterial.append(tTag, aMaterial.attribute(tTag), aMaterial.property(tTag));
            }
        }
        else
        {
            aSromMaterial.append(tTag, aMaterial.attribute(tTag), aMaterial.property(tTag));
        }
    }
}
// function append_material_properties

/******************************************************************************//**
 * \fn build_block_id_to_material_id_map
 * \brief Build map between material identification number and block identification number.
 * \param [in] aInputMetaData Plato problem input metadata
 * \return map between material identification number and block identification number
**********************************************************************************/
inline std::unordered_map<std::string, std::string>
build_material_id_to_block_id_map(XMLGen::InputData& aInputMetaData)
{
    if(aInputMetaData.materials.empty())
    {
        THROWERR(std::string("Build Block ID To Material ID Map: Input material container is empty. ")
            + " Material block is not defined in the input file.")
    }

    if(aInputMetaData.blocks.empty())
    {
        THROWERR(std::string("Build Block ID To Material ID Map: Input block container is empty. ")
            + " Block block is not defined in the input file.")
    }

    std::unordered_map<std::string, std::string> tMap;
    for(auto& tBlock : aInputMetaData.blocks)
    {
        tMap.insert({tBlock.material_id, tBlock.block_id});
    }
    return tMap;
}
// function build_block_id_to_material_id_map

/******************************************************************************//**
 * \fn set_block_identification_number
 * \brief Set block identification number to material.
 * \param [in]  aMap      map between material identification number and block identification number
 * \param [out] aMaterial material metadata
**********************************************************************************/
inline void set_block_identification_number
(const std::unordered_map<std::string, std::string>& aMap,
 Plato::srom::Material& aMaterial)
{
    auto tMaterialID = aMaterial.materialID();
    if(tMaterialID.empty())
    {
        THROWERR("Set Block Identification Number: Material identification number is empty, i.e. is not defined.")
    }

    auto tIterator = aMap.find(tMaterialID);
    if(tIterator == aMap.end())
    {
        THROWERR(std::string("Set Block Identification Number: No Block owns material with identification number '") + tMaterialID + "'.")
    }

    if(tIterator->second.empty())
    {
        THROWERR(std::string("Set Block Identification Number: Material with identification number '")
            + tMaterialID + "' has an empty block identification number, i.e. block id is not defined.")
    }

    aMaterial.blockID(tIterator->second);
}
// function set_block_identification_number

/******************************************************************************//**
 * \fn preprocess_material_inputs
 * \brief Pre-process non-deterministic material inputs, i.e. prepare inputs for \n
 *   Stochastic Reducded Order Model (SROM) problem.
 * \param [in] aInputMetadata  Plato problem input metadata
 * \param [in] aSromInputs     SROM problem input metadata
**********************************************************************************/
inline void preprocess_material_inputs
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

    auto tMaterialIDtoBlockID = Plato::srom::build_material_id_to_block_id_map(aInputMetadata);
    auto tRandomMatPropMap = Plato::srom::build_material_id_to_random_material_map(tIterator->second);
    for(auto& tMaterial : aInputMetadata.materials)
    {
        Plato::srom::Material tSromMaterial;
        Plato::srom::append_material_properties(tMaterial, tRandomMatPropMap, tSromMaterial);
        Plato::srom::set_block_identification_number(tMaterialIDtoBlockID, tSromMaterial);
        aSromInputs.append(tSromMaterial);
    }
}
// function preprocess_material_inputs

/******************************************************************************//**
 * \fn postprocess_material_outputs
 * \brief Post-process non-deterministic material outputs, i.e. prepare outputs for \n
 *   XML Generator tool.
 * \param [in] aOutput         SROM problem output metadata
 * \param [in] aXMLGenMetaData Plato problem metadata
**********************************************************************************/
inline void postprocess_material_outputs
(const Plato::srom::OutputMetaData& aOutput,
 XMLGen::InputData& aXMLGenMetaData)
{
    auto tRandomMaterialCases = aOutput.materials();
    if(tRandomMaterialCases.empty())
    {
        THROWERR("Post-Process Material Outputs: Container of random material cases is empty.")
    }

    double tSum = 0;
    for(auto& tRandomMaterialCase : tRandomMaterialCases)
    {
        XMLGen::RandomUseCaseMetaData tNewRandomUseCaseMetaData;
        tNewRandomUseCaseMetaData.usecase(aOutput.usecase());
        auto tRandomMaterials = tRandomMaterialCase.materials();
        for(auto& tRandomMaterial : tRandomMaterials)
        {
            XMLGen::Material tNewMaterial;
            tNewMaterial.id(tRandomMaterial.materialID());
            tNewMaterial.category(tRandomMaterial.category());
            auto tTags = tRandomMaterial.tags();
            for(auto& tTag : tTags)
            {
                tNewMaterial.property(tTag, tRandomMaterial.value(tTag), tRandomMaterial.attribute(tTag));
            }
            tNewRandomUseCaseMetaData.append(tRandomMaterial.blockID(), tNewMaterial);
        }
        tSum += tRandomMaterialCase.probability();
        tNewRandomUseCaseMetaData.probability(Plato::srom::to_string(tRandomMaterialCase.probability()));
        aXMLGenMetaData.mRandomUseCaseMetaData.push_back(tNewRandomUseCaseMetaData);
    }

    constexpr double tTolerance = 1e-2;
    if(std::abs(1.0 - tSum) > tTolerance)
    {
        THROWERR(std::string("Post-Process Material Outputs: The sum of the random use case probabilities is '")
            + std::to_string(tSum) + "'. The sum of these probabilities must yield a value greater than '0.99', "
            + "but less or equal to 1.0.")
    }
}
// function postprocess_material_outputs

}
// namespace srom

}
// namespace Plato
