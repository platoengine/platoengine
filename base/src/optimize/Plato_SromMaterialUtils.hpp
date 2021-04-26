/*
 * Plato_SromMaterialUtils.hpp
 *
 *  Created on: Apr 28, 2020
 */

#pragma once

#include "Plato_SromSolve.hpp"
#include "Plato_UniqueCounter.hpp"
#include "Plato_RandomMaterialMetadata.hpp"

namespace Plato
{

namespace srom
{

/******************************************************************************//**
 * \fn assign_material_case_identification_number
 * \brief Assign an identification number to each random material case in the set.
 * \param [out] aRandomMaterialCases set of random material cases
**********************************************************************************/
inline void assign_material_case_identification_number
(std::vector<Plato::srom::RandomMaterialCase> & aRandomMaterialCases)
{
    if(aRandomMaterialCases.empty())
    {
        THROWERR("Assign Material Case Identification Number: Input container of random material cases is empty.")
    }

    Plato::UniqueCounter tCaseCounter;
    tCaseCounter.mark(0);
    for(auto& tRandomMaterialCase : aRandomMaterialCases)
    {
        auto tID = std::to_string(tCaseCounter.assignNextUnique());
        tRandomMaterialCase.caseID(tID);
    }
}
// function assign_material_case_identification_number

/******************************************************************************//**
 * \fn define_deterministic_material
 * \brief Define a deterministic material for a random material case.
 * \param [in]  aInMaterial  deterministic material metadata
 * \param [out] aOutMaterial deterministic material for a random material case
**********************************************************************************/
inline void define_deterministic_material
(const Plato::srom::Material& aInMaterial,
 Plato::srom::RandomMaterial& aOutMaterial)
{
    aOutMaterial.probability(1.0);
    aOutMaterial.blockID(aInMaterial.blockID());
    aOutMaterial.category(aInMaterial.category());
    aOutMaterial.name(aInMaterial.name());
    aOutMaterial.materialID(aInMaterial.materialID());
    auto tDeterministicVars = aInMaterial.deterministicVars();
    for (auto& tVariable : tDeterministicVars)
    {
        aOutMaterial.append(tVariable.tag(), tVariable.attribute(), tVariable.value());
    }
}
// function define_deterministic_material

/******************************************************************************//**
 * \fn append_deterministic_materials
 * \brief Append deterministic materials to set of random material cases.
 * \param [in]  tDeterministicMaterials  list of deterministic materials
 * \param [out] RandomMaterialCases      list of random material cases
**********************************************************************************/
inline void append_deterministic_materials
(const std::vector<Plato::srom::Material>& tDeterministicMaterials,
 std::vector<Plato::srom::RandomMaterialCase>& RandomMaterialCases)
{
    if(tDeterministicMaterials.empty())
    {
        return;
    }

    for (auto& tMaterialCase : RandomMaterialCases)
    {
        for (auto& tMaterial : tDeterministicMaterials)
        {
            if (tMaterial.isRandom())
            {
                std::ostringstream tMsg;
                tMsg << "Append Deterministic Materials: Material with identification number '" << tMaterial.materialID()
                    << "' in block with identification number '" << "' is a random material.";
                THROWERR(tMsg.str().c_str())
            }
            tMaterial.check();
            Plato::srom::RandomMaterial tOutMaterial;
            Plato::srom::define_deterministic_material(tMaterial, tOutMaterial);
            tOutMaterial.check();
            tMaterialCase.append(tMaterial.materialID(), tOutMaterial);
        }
    }
}
// function append_deterministic_materials

/******************************************************************************//**
 * \fn split_random_and_deterministic_materials
 * \brief Split materials into lists of random and deterministic materials.
 * \param [in]  aMaterials         list of materials
 * \param [out] aRandomMats        list of random materials
 * \param [out] aDeterministicMats list of deterministic materials
**********************************************************************************/
inline void split_random_and_deterministic_materials
(const std::vector<Plato::srom::Material>& aMaterials,
 std::vector<Plato::srom::Material>& aRandomMats,
 std::vector<Plato::srom::Material>& aDeterministicMats)
{
    if(aMaterials.empty())
    {
        THROWERR("Expand Random And Deterministic Materials: Input material container is empty.");
    }

    for(auto& tMaterial : aMaterials)
    {
        if(tMaterial.isDeterministic())
        {
            aDeterministicMats.push_back(tMaterial);
        }
        else
        {
            aRandomMats.push_back(tMaterial);
        }
    }
}
// function split_random_and_deterministic_materials

/******************************************************************************//**
 * \fn initialize_random_material_set
 * \brief Initialize random material set.
 * \param [in]  aMaterial          material metadata
 * \param [in]  aSromVariable      Stochastic Reduced Order Model (SROM) variable metadata
 * \param [out] aRandomMaterialSet list of random materials
**********************************************************************************/
inline void initialize_random_material_set
(const Plato::srom::Material& aMaterial,
 const Plato::srom::SromVariable& aSromVariable,
 std::vector<Plato::srom::RandomMaterial>& aRandomMaterialSet)
{
    for(auto& tSample : aSromVariable.mSampleProbPairs.mSamples)
    {
        Plato::srom::RandomMaterial tRandomMaterial;
        tRandomMaterial.blockID(aMaterial.blockID());
        tRandomMaterial.category(aMaterial.category());
        tRandomMaterial.name(aMaterial.name());
        tRandomMaterial.materialID(aMaterial.materialID());

        auto tIndex = &tSample - &aSromVariable.mSampleProbPairs.mSamples[0];
        tRandomMaterial.probability(aSromVariable.mSampleProbPairs.mProbabilities[tIndex]);

        auto tValue = Plato::srom::to_string(aSromVariable.mSampleProbPairs.mSamples[tIndex]);
        tRandomMaterial.append(aSromVariable.mTag, aSromVariable.mAttribute, tValue);

        aRandomMaterialSet.push_back(tRandomMaterial);
    }
}
// function initialize_random_material_set

/******************************************************************************//**
 * \fn update_random_material_set
 * \brief Update random material set.
 * \param [in]  aMaterial          material metadata
 * \param [in]  aSromVariable      Stochastic Reduced Order Model (SROM) variable metadata
 * \param [out] aRandomMaterialSet list of random materials
**********************************************************************************/
inline void update_random_material_set
(const Plato::srom::Material& aMaterial,
 const Plato::srom::SromVariable& aSromVariable,
 std::vector<Plato::srom::RandomMaterial>& aRandomMaterialSet)
{
    auto tOriginalRandomMaterialSet = aRandomMaterialSet;

    for(auto& tRandomMaterial : aRandomMaterialSet)
    {
        auto tUpdatedProbability = tRandomMaterial.probability()
            * aSromVariable.mSampleProbPairs.mProbabilities[0];
        tRandomMaterial.probability(tUpdatedProbability);
        auto tValue = Plato::srom::to_string(aSromVariable.mSampleProbPairs.mSamples[0]);
        tRandomMaterial.append(aSromVariable.mTag, aSromVariable.mAttribute, tValue);
    }

    auto tBeginItr = aSromVariable.mSampleProbPairs.mSamples.begin();
    for(auto tItr = std::next(tBeginItr, 1); tItr != aSromVariable.mSampleProbPairs.mSamples.end(); tItr++)
    {
        for(auto& tOriginalRandomMaterial : tOriginalRandomMaterialSet)
        {
            Plato::srom::RandomMaterial tNewRandomMaterial;
            tNewRandomMaterial.blockID(aMaterial.blockID());
            tNewRandomMaterial.category(aMaterial.category());
            tNewRandomMaterial.name(aMaterial.name());
            tNewRandomMaterial.materialID(aMaterial.materialID());

            auto tIndex = std::distance(tBeginItr, tItr);
            auto tUpdatedProbability = tOriginalRandomMaterial.probability()
                * aSromVariable.mSampleProbPairs.mProbabilities[tIndex];
            tNewRandomMaterial.probability(tUpdatedProbability);
            auto tValue = Plato::srom::to_string(aSromVariable.mSampleProbPairs.mSamples[tIndex]);
            tNewRandomMaterial.append(aSromVariable.mTag, aSromVariable.mAttribute, tValue);

            auto tOriginalTags = tOriginalRandomMaterial.tags();
            for(auto& tTag : tOriginalTags)
            {
                tValue = tOriginalRandomMaterial.value(tTag);
                auto tAttribute = tOriginalRandomMaterial.attribute(tTag);
                tNewRandomMaterial.append(tTag, tAttribute, tValue);
            }

            aRandomMaterialSet.push_back(tNewRandomMaterial);
        }
    }
}
// function update_random_material_set

/******************************************************************************//**
 * \fn append_random_material_properties
 * \brief Append random material properties to random material.
 * \param [in]  aMaterial          material metadata
 * \param [in]  aSromVariables     list of Stochastic Reduced Order Model (SROM) variables
 * \param [out] aRandomMaterialSet list of random materials
**********************************************************************************/
inline void append_random_material_properties
(const Plato::srom::Material& aMaterial,
 const std::vector<Plato::srom::SromVariable>& aSromVariables,
 std::vector<Plato::srom::RandomMaterial>& aRandomMaterialSet)
{
    for(auto& tSromVar : aSromVariables)
    {
        if(aRandomMaterialSet.empty())
        {
            Plato::srom::initialize_random_material_set(aMaterial, tSromVar, aRandomMaterialSet);
        }
        else
        {
            Plato::srom::update_random_material_set(aMaterial, tSromVar, aRandomMaterialSet);
        }
    }
}
// function append_random_material_properties

/******************************************************************************//**
 * \fn append_deterministic_material_properties
 * \brief Append deterministic material properties to random material.
 * \param [in]  aMaterial          material metadata
 * \param [out] aRandomMaterialSet list of random materials
**********************************************************************************/
inline void append_deterministic_material_properties
(const Plato::srom::Material &aMaterial,
 std::vector<Plato::srom::RandomMaterial>& aRandomMaterialSet)
{
    auto tDeterministicVars = aMaterial.deterministicVars();
    for(auto& tRandomMaterial : aRandomMaterialSet)
    {
        for(auto& tVar : tDeterministicVars)
        {
            tRandomMaterial.append(tVar.tag(), tVar.attribute(), tVar.value());
        }
    }
}
// function append_deterministic_material_properties

/******************************************************************************//**
 * \fn build_random_material_set
 * \brief Build set of random materials.
 * \param [in]  aMaterial          material metadata
 * \param [in]  aSromVariables     list of Stochastic Reduced Order Model (SROM) variables
 * \param [out] aRandomMaterials   list of random materials
**********************************************************************************/
inline void build_random_material_set
(const Plato::srom::Material& aMaterial,
 const std::vector<Plato::srom::SromVariable>& aSromVariables,
 std::vector<Plato::srom::RandomMaterial>& aRandomMaterials)
{
    if(aSromVariables.empty())
    {
        THROWERR("Build Random Material: Input set of SROM variables is empty.")
    }

    Plato::srom::append_random_material_properties(aMaterial, aSromVariables, aRandomMaterials);
    Plato::srom::append_deterministic_material_properties(aMaterial, aRandomMaterials);
}
// function build_random_material_set

/******************************************************************************//**
 * \fn initialize_random_material_cases
 * \brief Initialize list of random material cases.
 * \param [in]  aRandomMaterials       list of random materials
 * \param [out] aRandomMaterialCases   list of random material cases
**********************************************************************************/
inline void initialize_random_material_cases
(const std::vector<Plato::srom::RandomMaterial>& aRandomMaterials,
 std::vector<Plato::srom::RandomMaterialCase>& aRandomMaterialCases)
{
    if(aRandomMaterials.empty())
    {
        THROWERR("Initialize Random Material Cases: Input container of random materials is empty.");
    }

    for(auto& tRandomMaterial : aRandomMaterials)
    {
        tRandomMaterial.check();
        Plato::srom::RandomMaterialCase tRandomMaterialCase;
        auto tMaterialID = tRandomMaterial.materialID();
        tRandomMaterialCase.append(tMaterialID, tRandomMaterial);
        tRandomMaterialCase.probability(tRandomMaterial.probability());
        aRandomMaterialCases.push_back(tRandomMaterialCase);
    }
}
// function initialize_random_material_cases

/******************************************************************************//**
 * \fn update_random_material_cases
 * \brief Update list of random material cases.
 * \param [in]  aRandomMaterials       list of random materials
 * \param [out] aRandomMaterialCases   list of random material cases
**********************************************************************************/
inline void update_random_material_cases
(const std::vector<Plato::srom::RandomMaterial>& aRandomMaterials,
 std::vector<Plato::srom::RandomMaterialCase>& aRandomMaterialCases)
{
    if(aRandomMaterials.empty())
    {
        THROWERR("Update Random Material Cases: Input container of random materials is empty.");
    }

    auto tOriginalRandomMaterialCases = aRandomMaterialCases;

    for(auto& tRandomMaterialCase : aRandomMaterialCases)
    {
        aRandomMaterials[0].check();
        auto tUpdatedProbability = tRandomMaterialCase.probability()
            * aRandomMaterials[0].probability();
        tRandomMaterialCase.probability(tUpdatedProbability);
        auto tID = aRandomMaterials[0].materialID();
        tRandomMaterialCase.append(tID, aRandomMaterials[0]);
    }

    auto tBeginItr = aRandomMaterials.begin();
    for(auto tItr = std::next(tBeginItr, 1); tItr != aRandomMaterials.end(); tItr++)
    {
        tItr->check();
        for(auto& tOriginalRandomMaterialCase : tOriginalRandomMaterialCases)
        {
            Plato::srom::RandomMaterialCase tNewRandomMaterialCase;
            auto tUpdatedProbability =
                tOriginalRandomMaterialCase.probability() * tItr->probability();
            tNewRandomMaterialCase.probability(tUpdatedProbability);
            auto tID = tItr->materialID();
            tNewRandomMaterialCase.append(tID, tItr.operator*());

            auto tOriginalMaterials = tOriginalRandomMaterialCase.materials();
            for(auto& tMaterial : tOriginalMaterials)
            {
                tID = tMaterial.materialID();
                tNewRandomMaterialCase.append(tID, tMaterial);
            }

            aRandomMaterialCases.push_back(tNewRandomMaterialCase);
        }
    }
}
// function update_random_material_cases

/******************************************************************************//**
 * \fn append_random_material_set
 * \brief Append random material set to list of random material cases.
 * \param [in]  aRandomMaterials       list of random materials
 * \param [out] aRandomMaterialCases   list of random material cases
**********************************************************************************/
inline void append_random_material_set
(const std::vector<Plato::srom::RandomMaterial>& aRandomMaterials,
 std::vector<Plato::srom::RandomMaterialCase>& aRandomMaterialCases)
{
    if(aRandomMaterials.empty())
    {
        THROWERR("Append Random Material Cases: Input container of random materials is empty.");
    }

    if(aRandomMaterialCases.empty())
    {
        Plato::srom::initialize_random_material_cases(aRandomMaterials, aRandomMaterialCases);
    }
    else
    {
        Plato::srom::update_random_material_cases(aRandomMaterials, aRandomMaterialCases);
    }
}
// function append_random_material_set

/******************************************************************************//**
 * \fn check_material_set
 * \brief Check if list of materials is properly defined.
 * \param [in]  aRandomMaterials       list of random materials
 * \param [out] aRandomMaterialCases   list of random material cases
 * \note The following criteria must be met to consider the material defined:
 *       1. material and block identification numbers are defined;
 *       2. number of defined material properties must be at least one.
 *       3. material identification number is unique, i.e. duplicate material identification numbers are not permitted.
**********************************************************************************/
inline void check_material_set(const std::vector<Plato::srom::Material>& aMaterials)
{
    std::vector<int> tIDs;
    for(auto& tMaterial : aMaterials)
    {
        tMaterial.check();
        auto tInteger = std::stoi(tMaterial.materialID());
        tIDs.push_back(tInteger);
    }

    std::sort(tIDs.begin(), tIDs.end());
    const auto tDuplicate = std::adjacent_find(tIDs.begin(), tIDs.end());
    if (tDuplicate != tIDs.end())
    {
        std::ostringstream tMsg;
        tMsg << "Check Material Inputs: Material identification numbers are not unique.  Material identification number '"
            << *tDuplicate << "' is a duplicate ID.\n";
        THROWERR(tMsg.str().c_str())
    }
}
// function check_material_set

/******************************************************************************//**
 * \fn build_material_sroms
 * \brief Build material Stochastic Reduced Order Models (SROMs).
 * \param [in]  aInput    list of materials
 * \param [out] aOutput   list of random material cases
**********************************************************************************/
inline void build_material_sroms(const std::vector<Plato::srom::Material>& aInput,
                                 std::vector<Plato::srom::RandomMaterialCase>& aOutput)
{
    if(aInput.empty())
    {
        THROWERR("Build Material SROMS: Input container of materials is empty.")
    }
    Plato::srom::check_material_set(aInput);

    aOutput.clear();
    std::vector<Plato::srom::Material> tRandomMaterials, tDeterministicMaterials;
    Plato::srom::split_random_and_deterministic_materials(aInput, tRandomMaterials, tDeterministicMaterials);

    for(auto& tMaterial : tRandomMaterials)
    {
        tMaterial.check();
        if(tMaterial.isDeterministic())
        {
            THROWERR("Build Material SROMS: Detected a deterministic material while looping over random materials set.")
        }

        auto tRandomVariables = tMaterial.randomVars();
        Plato::srom::set_random_variables_id(tRandomVariables);

        std::vector<Plato::srom::SromVariable> tMySampleProbPairs;
        if(Plato::srom::compute_sample_probability_pairs(tRandomVariables, tMySampleProbPairs) == false)
        {
            std::ostringstream tMsg;
            tMsg << "Build Material SROMS: Failed to compute Sample-Probability pairs for material "
                << "with identification number '" << tMaterial.materialID() << "' and category '"
                << tMaterial.category() << "'.";
            PRINTERR(tMsg.str().c_str());
        }

        std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
        Plato::srom::build_random_material_set(tMaterial, tMySampleProbPairs, tRandomMaterialSet);
        Plato::srom::append_random_material_set(tRandomMaterialSet, aOutput);
    }

    Plato::srom::append_deterministic_materials(tDeterministicMaterials, aOutput);
    Plato::srom::assign_material_case_identification_number(aOutput);
}
// function build_material_sroms

}
// namespace srom

}
// namespace Plato
