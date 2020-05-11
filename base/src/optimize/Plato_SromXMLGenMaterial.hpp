/*
 * Plato_SromXMLGenMaterial.hpp
 *
 *  Created on: May 11, 2020
 */

#pragma once

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

}
// namespace srom

}
// namespace Plato
