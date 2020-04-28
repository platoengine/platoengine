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
 * Plato_Test_UncertainMaterial.cpp
 *
 *  Created on: Apr 22, 2020
 */

#include "gtest/gtest.h"

#include "Plato_SromSolve.hpp"
#include "Plato_SromXMLUtils.hpp"
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
    aOutMaterial.materialID(aInMaterial.materialID());
    auto tDeterministicVars = aInMaterial.deterministicVars();
    for (auto& tVariable : tDeterministicVars)
    {
        aOutMaterial.append(tVariable.mTag, tVariable.mAttribute, tVariable.mValue);
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
                    << "' in block with identification number '" << tMaterial.blockID() << "' is not defined.";
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
                auto tValue = tOriginalRandomMaterial.value(tTag);
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
            tRandomMaterial.append(tVar.mTag, tVar.mAttribute, tVar.mValue);
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
                auto tID = tMaterial.materialID();
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
            THROWERR("Generate Material SROMS: Detected a deterministic material while looping over random materials set.")
        }

        std::vector<Plato::srom::SromVariable> tMySampleProbPairs;
        if(Plato::srom::compute_sample_probability_pairs(tMaterial.randomVars(), tMySampleProbPairs) == false)
        {
            std::ostringstream tMsg;
            tMsg << "Generate Material SROMS: Failed to compute Sample-Probability pairs for material "
                << "with identification number '" << tMaterial.materialID() << "' in block with identification "
                << "number '" << tMaterial.blockID() << "'.";
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

namespace UncertainMaterial
{

TEST(PlatoTest, SROM_ComputeSampleProbabilityPairs_HomogeneousElasticModulus_Beta)
{
    // DEFINE INPUTS
    Plato::srom::RandomVariable tMyRandomVar;
    tMyRandomVar.mTag = "elastic modulus";
    tMyRandomVar.mAttribute = "homogeneous";
    tMyRandomVar.mStatistics.mDistribution = "beta";
    tMyRandomVar.mStatistics.mMean = "1e9";
    tMyRandomVar.mStatistics.mUpperBound = "1e10";
    tMyRandomVar.mStatistics.mLowerBound = "1e8";
    tMyRandomVar.mStatistics.mStandardDeviation = "2e8";
    tMyRandomVar.mStatistics.mNumSamples = "3";
    std::vector<Plato::srom::RandomVariable> tRandomVarsSet;
    tRandomVarsSet.push_back(tMyRandomVar);

    // SOLVE SROM PROBLEM
    std::vector<Plato::srom::SromVariable> tMySampleProbPairs;
    Plato::srom::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs);

    // TEST RESULTS
    ASSERT_EQ(1u, tMySampleProbPairs.size());
    ASSERT_STREQ("elastic modulus", tMySampleProbPairs[0].mTag.c_str());
    ASSERT_STREQ("homogeneous", tMySampleProbPairs[0].mAttribute.c_str());
    ASSERT_EQ(3, tMySampleProbPairs[0].mSampleProbPairs.mNumSamples);
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mSamples.size());
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mProbabilities.size());

    // TEST RESULTS
    double tSum = 0;
    double tTolerance = 1e-4;
    const Plato::srom::SampleProbabilityPairs& tSampleProbabilityPairs = tMySampleProbPairs[0].mSampleProbPairs;
    std::vector<double> tGoldSamples = {1312017818.6197019, 659073448.54796219, 656356599.33196139};
    std::vector<double> tGoldProbabilities = {0.43210087774761252, 0.31840063469163404, 0.24868340186995475};
    for(int tIndex = 0; tIndex < tSampleProbabilityPairs.mNumSamples; tIndex++)
    {
        tSum += tSampleProbabilityPairs.mProbabilities[tIndex];
        ASSERT_NEAR(tGoldSamples[tIndex], tSampleProbabilityPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tIndex], tSampleProbabilityPairs.mProbabilities[tIndex], tTolerance);
    }
    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);

    std::system("rm -f plato_cdf_output.txt");
    std::system("rm -f plato_srom_diagnostics.txt");
    std::system("rm -f plato_ksal_algorithm_diagnostics.txt");
}

TEST(PlatoTest, SROM_SplitRandomAndDeterministicMaterials_Error)
{
    std::vector<Plato::srom::Material> tMaterials;
    std::vector<Plato::srom::Material> tRandomMaterial, tDeterministicMaterial;
    EXPECT_THROW(Plato::srom::split_random_and_deterministic_materials(tMaterials, tRandomMaterial, tDeterministicMaterial), std::runtime_error);
}

TEST(PlatoTest, SROM_SplitRandomAndDeterministicMaterials)
{
    // DEFINE MATERIAL ONE - RANDOM
    Plato::srom::Material tMaterialOne;
    tMaterialOne.blockID("1");
    tMaterialOne.materialID("1");
    tMaterialOne.category("isotropic");
    tMaterialOne.append("poissons ratio", "homogeneous", "0.3");
    Plato::srom::Statistics tElasticModulusStats;
    tElasticModulusStats.mDistribution = "beta";
    tElasticModulusStats.mMean = "1e9";
    tElasticModulusStats.mUpperBound = "1e10";
    tElasticModulusStats.mLowerBound = "1e8";
    tElasticModulusStats.mStandardDeviation = "2e8";
    tElasticModulusStats.mNumSamples = "3";
    tMaterialOne.append("elastic modulus", "homogeneous", tElasticModulusStats);

    // DEFINE MATERIAL TWO - DETERMINISTIC
    Plato::srom::Material tMaterialTwo;
    tMaterialTwo.blockID("22");
    tMaterialTwo.materialID("10");
    tMaterialTwo.category("isotropic");
    tMaterialTwo.append("poissons ratio", "homogeneous", "0.35");
    tMaterialTwo.append("elastic modulus", "homogeneous", "2.5e8");

    // APPEND MATERIALS TO MATERIALS SET
    std::vector<Plato::srom::Material> tMaterials;
    tMaterials.push_back(tMaterialOne);
    tMaterials.push_back(tMaterialTwo);

    // SPLIT MATERIALS INTO RANDOM AND DETERMINIST MATERIAL SETS
    std::vector<Plato::srom::Material> tRandomMaterial, tDeterministicMaterial;
    Plato::srom::split_random_and_deterministic_materials(tMaterials, tRandomMaterial, tDeterministicMaterial);

    // 1) TEST RANDOM MATERIAL
    ASSERT_EQ(1u, tRandomMaterial.size());
    ASSERT_TRUE(tRandomMaterial[0].isRandom());
    ASSERT_FALSE(tRandomMaterial[0].isDeterministic());
    ASSERT_STREQ("1", tRandomMaterial[0].blockID().c_str());
    ASSERT_STREQ("1", tRandomMaterial[0].materialID().c_str());
    ASSERT_STREQ("isotropic", tRandomMaterial[0].category().c_str());

    auto tRandomVars = tRandomMaterial[0].randomVars();
    ASSERT_EQ(1u, tRandomVars.size());
    ASSERT_STREQ("elastic modulus", tRandomVars[0].mTag.c_str());
    ASSERT_STREQ("homogeneous", tRandomVars[0].mAttribute.c_str());
    ASSERT_STREQ("1e9", tRandomVars[0].mStatistics.mMean.c_str());
    ASSERT_STREQ("3", tRandomVars[0].mStatistics.mNumSamples.c_str());
    ASSERT_STREQ("1e8", tRandomVars[0].mStatistics.mLowerBound.c_str());
    ASSERT_STREQ("1e10", tRandomVars[0].mStatistics.mUpperBound.c_str());
    ASSERT_STREQ("beta", tRandomVars[0].mStatistics.mDistribution.c_str());
    ASSERT_STREQ("2e8", tRandomVars[0].mStatistics.mStandardDeviation.c_str());

    auto tDeterministicVars = tRandomMaterial[0].deterministicVars();
    ASSERT_EQ(1u, tDeterministicVars.size());
    ASSERT_STREQ("poissons ratio", tDeterministicVars[0].mTag.c_str());
    ASSERT_STREQ("homogeneous", tDeterministicVars[0].mAttribute.c_str());
    ASSERT_STREQ("0.3", tDeterministicVars[0].mValue.c_str());

    // 2) TEST DETERMINISTIC MATERIAL
    ASSERT_EQ(1u, tDeterministicMaterial.size());
    ASSERT_TRUE(tDeterministicMaterial[0].isDeterministic());
    ASSERT_FALSE(tDeterministicMaterial[0].isRandom());
    tRandomVars = tDeterministicMaterial[0].randomVars();
    ASSERT_EQ(0u, tRandomVars.size());

    tDeterministicVars = tDeterministicMaterial[0].deterministicVars();
    ASSERT_EQ(2u, tDeterministicVars.size());
    ASSERT_STREQ("poissons ratio", tDeterministicVars[0].mTag.c_str());
    ASSERT_STREQ("homogeneous", tDeterministicVars[0].mAttribute.c_str());
    ASSERT_STREQ("0.35", tDeterministicVars[0].mValue.c_str());
    ASSERT_STREQ("elastic modulus", tDeterministicVars[1].mTag.c_str());
    ASSERT_STREQ("homogeneous", tDeterministicVars[1].mAttribute.c_str());
    ASSERT_STREQ("2.5e8", tDeterministicVars[1].mValue.c_str());
}

TEST(PlatoTest, SROM_CheckInputSetOfMaterials_Error)
{
    // DEFINE MATERIAL ONE - RANDOM
    Plato::srom::Material tMaterialOne;
    tMaterialOne.blockID("1");
    tMaterialOne.materialID("1");
    tMaterialOne.category("isotropic");
    tMaterialOne.append("poissons ratio", "homogeneous", "0.3");
    Plato::srom::Statistics tElasticModulusStats;
    tElasticModulusStats.mDistribution = "beta";
    tElasticModulusStats.mMean = "1e9";
    tElasticModulusStats.mUpperBound = "1e10";
    tElasticModulusStats.mLowerBound = "1e8";
    tElasticModulusStats.mStandardDeviation = "2e8";
    tElasticModulusStats.mNumSamples = "3";
    tMaterialOne.append("elastic modulus", "homogeneous", tElasticModulusStats);

    // DEFINE MATERIAL TWO - DETERMINISTIC
    Plato::srom::Material tMaterialTwo;
    tMaterialTwo.blockID("22");
    tMaterialTwo.materialID("10");
    tMaterialTwo.category("isotropic");

    std::vector<Plato::srom::Material> tInput;
    tInput.push_back(tMaterialOne);
    tInput.push_back(tMaterialTwo);
    EXPECT_THROW(Plato::srom::check_material_set(tInput), std::runtime_error);
}

TEST(PlatoTest, SROM_InitializeRandomMaterialSet)
{
    // DEFINE MATERIAL ONE - RANDOM
    Plato::srom::Material tMaterial;
    tMaterial.blockID("1");
    tMaterial.materialID("2");
    tMaterial.category("isotropic");
    Plato::srom::Statistics tPoissonsRatioStats;
    tPoissonsRatioStats.mDistribution = "beta";
    tPoissonsRatioStats.mMean = "0.3";
    tPoissonsRatioStats.mUpperBound = "0.4";
    tPoissonsRatioStats.mLowerBound = "0.25";
    tPoissonsRatioStats.mStandardDeviation = "0.05";
    tPoissonsRatioStats.mNumSamples = "2";
    tMaterial.append("poissons ratio", "homogeneous", tPoissonsRatioStats);
    Plato::srom::Statistics tElasticModulusStats;
    tElasticModulusStats.mDistribution = "beta";
    tElasticModulusStats.mMean = "1e9";
    tElasticModulusStats.mUpperBound = "1e10";
    tElasticModulusStats.mLowerBound = "1e8";
    tElasticModulusStats.mStandardDeviation = "2e8";
    tElasticModulusStats.mNumSamples = "3";
    tMaterial.append("elastic modulus", "homogeneous", tElasticModulusStats);

    // INITIALIZE RANDOM MATERIAL SET
    Plato::srom::SromVariable tSromVariable;
    tSromVariable.mTag = "poissons ratio";
    tSromVariable.mAttribute = "homogeneous";
    tSromVariable.mSampleProbPairs.mNumSamples = 2;
    tSromVariable.mSampleProbPairs.mSamples = {0.32, 0.27};
    tSromVariable.mSampleProbPairs.mProbabilities = {0.75, 0.25};
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    EXPECT_NO_THROW(Plato::srom::initialize_random_material_set(tMaterial, tSromVariable, tRandomMaterialSet));

    const double tTolerance = 1e-4;
    ASSERT_EQ(2u, tRandomMaterialSet.size());
    const std::vector<double> tGoldProbs = {0.75, 0.25};
    const std::vector<std::string> tGoldSamples = {"0.3200000000000000", "0.2700000000000000"};
    for(auto& tRandomMaterial : tRandomMaterialSet)
    {
        ASSERT_STREQ("1", tRandomMaterial.blockID().c_str());
        ASSERT_STREQ("2", tRandomMaterial.materialID().c_str());
        ASSERT_STREQ("isotropic", tRandomMaterial.category().c_str());

        auto tIndex = &tRandomMaterial - &tRandomMaterialSet[0];
        ASSERT_NEAR(tGoldProbs[tIndex], tRandomMaterial.probability(), tTolerance);

        auto tTags = tRandomMaterial.tags();
        ASSERT_EQ(1u, tTags.size());
        for(auto& tTag : tTags)
        {
            ASSERT_STREQ("poissons ratio", tTag.c_str());
            ASSERT_STREQ("homogeneous", tRandomMaterial.attribute(tTag).c_str());
            ASSERT_STREQ(tGoldSamples[tIndex].c_str(), tRandomMaterial.value(tTag).c_str());
        }
    }
}

TEST(PlatoTest, SROM_UpdateRandomMaterialSet)
{
    // 1. DEFINE MATERIAL ONE - RANDOM
    Plato::srom::Material tMaterial;
    tMaterial.blockID("1");
    tMaterial.materialID("2");
    tMaterial.category("isotropic");
    Plato::srom::Statistics tPoissonsRatioStats;
    tPoissonsRatioStats.mDistribution = "beta";
    tPoissonsRatioStats.mMean = "0.3";
    tPoissonsRatioStats.mUpperBound = "0.4";
    tPoissonsRatioStats.mLowerBound = "0.25";
    tPoissonsRatioStats.mStandardDeviation = "0.05";
    tPoissonsRatioStats.mNumSamples = "2";
    tMaterial.append("poissons ratio", "homogeneous", tPoissonsRatioStats);
    Plato::srom::Statistics tElasticModulusStats;
    tElasticModulusStats.mDistribution = "beta";
    tElasticModulusStats.mMean = "1e9";
    tElasticModulusStats.mUpperBound = "1e10";
    tElasticModulusStats.mLowerBound = "1e8";
    tElasticModulusStats.mStandardDeviation = "2e8";
    tElasticModulusStats.mNumSamples = "3";
    tMaterial.append("elastic modulus", "homogeneous", tElasticModulusStats);

    // 2. INITIALIZE RANDOM MATERIAL SET
    Plato::srom::SromVariable tSromVariableOne;
    tSromVariableOne.mTag = "poissons ratio";
    tSromVariableOne.mAttribute = "homogeneous";
    tSromVariableOne.mSampleProbPairs.mNumSamples = 2;
    tSromVariableOne.mSampleProbPairs.mSamples = {0.32, 0.27};
    tSromVariableOne.mSampleProbPairs.mProbabilities = {0.75, 0.25};
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    EXPECT_NO_THROW(Plato::srom::initialize_random_material_set(tMaterial, tSromVariableOne, tRandomMaterialSet));

    // 3. UPDATE RANDOM MATERIAL SET
    Plato::srom::SromVariable tSromVariableTwo;
    tSromVariableTwo.mTag = "elastic modulus";
    tSromVariableTwo.mAttribute = "homogeneous";
    tSromVariableTwo.mSampleProbPairs.mNumSamples = 2;
    tSromVariableTwo.mSampleProbPairs.mSamples = {1e9, 2.5e9};
    tSromVariableTwo.mSampleProbPairs.mProbabilities = {0.45, 0.55};
    EXPECT_NO_THROW(Plato::srom::update_random_material_set(tMaterial, tSromVariableTwo, tRandomMaterialSet));
    ASSERT_EQ(4u, tRandomMaterialSet.size());

    const std::vector<double> tGoldProbs = {0.3375, 0.1125, 0.4125, 0.1375};
    const std::vector<std::string> tGoldTags = {"elastic modulus", "poissons ratio"};
    const std::vector<std::vector<std::string>> tGoldSamples =
        { {"1000000000.0000000000000000", "0.3200000000000000"},
          {"1000000000.0000000000000000", "0.2700000000000000"},
          {"2500000000.0000000000000000", "0.3200000000000000"},
          {"2500000000.0000000000000000", "0.2700000000000000"}};

    double tProbSum = 0;
    const double tTolerance = 1e-4;
    for(auto& tRandomMaterial : tRandomMaterialSet)
    {
        ASSERT_STREQ("1", tRandomMaterial.blockID().c_str());
        ASSERT_STREQ("2", tRandomMaterial.materialID().c_str());
        ASSERT_STREQ("isotropic", tRandomMaterial.category().c_str());

        auto tMatIndex = &tRandomMaterial - &tRandomMaterialSet[0];
        ASSERT_NEAR(tGoldProbs[tMatIndex], tRandomMaterial.probability(), tTolerance);
        tProbSum += tRandomMaterial.probability();

        auto tTags = tRandomMaterial.tags();
        ASSERT_EQ(2u, tTags.size());
        for(auto& tTag : tTags)
        {
            auto tTagIndex = &tTag - &tTags[0];
            ASSERT_STREQ(tGoldTags[tTagIndex].c_str(), tTag.c_str());
            ASSERT_STREQ("homogeneous", tRandomMaterial.attribute(tTag).c_str());
            ASSERT_STREQ(tGoldSamples[tMatIndex][tTagIndex].c_str(), tRandomMaterial.value(tTag).c_str());
        }
    }
    ASSERT_NEAR(1.0, tProbSum, tTolerance);
}

TEST(PlatoTest, SROM_AppendRandomMaterialProperties)
{
    // 1. DEFINE MATERIAL ONE - RANDOM
    Plato::srom::Material tMaterial;
    tMaterial.blockID("1");
    tMaterial.materialID("2");
    tMaterial.category("isotropic");
    Plato::srom::Statistics tPoissonsRatioStats;
    tPoissonsRatioStats.mDistribution = "beta";
    tPoissonsRatioStats.mMean = "0.3";
    tPoissonsRatioStats.mUpperBound = "0.4";
    tPoissonsRatioStats.mLowerBound = "0.25";
    tPoissonsRatioStats.mStandardDeviation = "0.05";
    tPoissonsRatioStats.mNumSamples = "2";
    tMaterial.append("poissons ratio", "homogeneous", tPoissonsRatioStats);
    Plato::srom::Statistics tElasticModulusStats;
    tElasticModulusStats.mDistribution = "beta";
    tElasticModulusStats.mMean = "1e9";
    tElasticModulusStats.mUpperBound = "1e10";
    tElasticModulusStats.mLowerBound = "1e8";
    tElasticModulusStats.mStandardDeviation = "2e8";
    tElasticModulusStats.mNumSamples = "3";
    tMaterial.append("elastic modulus", "homogeneous", tElasticModulusStats);

    // 2. INITIALIZE SROM VARIABLE SET
    Plato::srom::SromVariable tSromVariableOne;
    tSromVariableOne.mTag = "poissons ratio";
    tSromVariableOne.mAttribute = "homogeneous";
    tSromVariableOne.mSampleProbPairs.mNumSamples = 2;
    tSromVariableOne.mSampleProbPairs.mSamples = {0.32, 0.27};
    tSromVariableOne.mSampleProbPairs.mProbabilities = {0.75, 0.25};

    Plato::srom::SromVariable tSromVariableTwo;
    tSromVariableTwo.mTag = "elastic modulus";
    tSromVariableTwo.mAttribute = "homogeneous";
    tSromVariableTwo.mSampleProbPairs.mNumSamples = 2;
    tSromVariableTwo.mSampleProbPairs.mSamples = {1e9, 2.5e9};
    tSromVariableTwo.mSampleProbPairs.mProbabilities = {0.45, 0.55};

    std::vector<Plato::srom::SromVariable> tSromVariableSet;
    tSromVariableSet.push_back(tSromVariableOne);
    tSromVariableSet.push_back(tSromVariableTwo);

    // 3. APPEND RANDOM MATERIAL PROPERTIES TO RANDOM MATERIAL SET
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    EXPECT_NO_THROW(Plato::srom::append_random_material_properties(tMaterial, tSromVariableSet, tRandomMaterialSet));
    ASSERT_EQ(4u, tRandomMaterialSet.size());

    const std::vector<double> tGoldProbs = {0.3375, 0.1125, 0.4125, 0.1375};
    const std::vector<std::string> tGoldTags = {"elastic modulus", "poissons ratio"};
    const std::vector<std::vector<std::string>> tGoldSamples =
        { {"1000000000.0000000000000000", "0.3200000000000000"},
          {"1000000000.0000000000000000", "0.2700000000000000"},
          {"2500000000.0000000000000000", "0.3200000000000000"},
          {"2500000000.0000000000000000", "0.2700000000000000"}};

    double tProbSum = 0;
    const double tTolerance = 1e-4;
    for(auto& tRandomMaterial : tRandomMaterialSet)
    {
        ASSERT_STREQ("1", tRandomMaterial.blockID().c_str());
        ASSERT_STREQ("2", tRandomMaterial.materialID().c_str());
        ASSERT_STREQ("isotropic", tRandomMaterial.category().c_str());

        auto tMatIndex = &tRandomMaterial - &tRandomMaterialSet[0];
        ASSERT_NEAR(tGoldProbs[tMatIndex], tRandomMaterial.probability(), tTolerance);
        tProbSum += tRandomMaterial.probability();

        auto tTags = tRandomMaterial.tags();
        ASSERT_EQ(2u, tTags.size());
        for(auto& tTag : tTags)
        {
            auto tTagIndex = &tTag - &tTags[0];
            ASSERT_STREQ(tGoldTags[tTagIndex].c_str(), tTag.c_str());
            ASSERT_STREQ("homogeneous", tRandomMaterial.attribute(tTag).c_str());
            ASSERT_STREQ(tGoldSamples[tMatIndex][tTagIndex].c_str(), tRandomMaterial.value(tTag).c_str());
        }
    }
    ASSERT_NEAR(1.0, tProbSum, tTolerance);
}

TEST(PlatoTest, SROM_AppendDeterministicMaterialProperties)
{
    // DEFINE RANDOM MATERIAL
    Plato::srom::Material tMaterial;
    tMaterial.blockID("1");
    tMaterial.materialID("2");
    tMaterial.category("isotropic");
    Plato::srom::Statistics tPoissonsRatioStats;
    tPoissonsRatioStats.mDistribution = "beta";
    tPoissonsRatioStats.mMean = "0.3";
    tPoissonsRatioStats.mUpperBound = "0.4";
    tPoissonsRatioStats.mLowerBound = "0.25";
    tPoissonsRatioStats.mStandardDeviation = "0.05";
    tPoissonsRatioStats.mNumSamples = "2";
    tMaterial.append("poissons ratio", "homogeneous", tPoissonsRatioStats);
    tMaterial.append("elastic modulus", "homogeneous", "1e9");
    auto tTags = tMaterial.tags();
    ASSERT_EQ(2u, tTags.size());
    ASSERT_STREQ("poissons ratio", tTags[0].c_str());
    ASSERT_STREQ("elastic modulus", tTags[1].c_str());

    // INITIALIZE RANDOM MATERIAL SET
    Plato::srom::SromVariable tSromVariable;
    tSromVariable.mTag = "poissons ratio";
    tSromVariable.mAttribute = "homogeneous";
    tSromVariable.mSampleProbPairs.mNumSamples = 2;
    tSromVariable.mSampleProbPairs.mSamples = {0.32, 0.27};
    tSromVariable.mSampleProbPairs.mProbabilities = {0.75, 0.25};

    std::vector<Plato::srom::SromVariable> tSromVariableSet;
    tSromVariableSet.push_back(tSromVariable);

    // APPEND MATERIAL PROPERTIES
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    EXPECT_NO_THROW(Plato::srom::append_random_material_properties(tMaterial, tSromVariableSet, tRandomMaterialSet));
    ASSERT_EQ(2u, tRandomMaterialSet.size());
    EXPECT_NO_THROW(Plato::srom::append_deterministic_material_properties(tMaterial, tRandomMaterialSet));

    const double tTolerance = 1e-4;
    const std::vector<double> tGoldProbs = {0.75, 0.25};
    const std::vector<std::string> tGoldTags = {"elastic modulus", "poissons ratio"};
    const std::vector<std::vector<std::string>> tGoldSamples = { {"1e9", "0.3200000000000000"}, {"1e9", "0.2700000000000000"}};

    for(auto& tRandomMaterial : tRandomMaterialSet)
    {
        ASSERT_STREQ("1", tRandomMaterial.blockID().c_str());
        ASSERT_STREQ("2", tRandomMaterial.materialID().c_str());
        ASSERT_STREQ("isotropic", tRandomMaterial.category().c_str());

        auto tMatIndex = &tRandomMaterial - &tRandomMaterialSet[0];
        ASSERT_NEAR(tGoldProbs[tMatIndex], tRandomMaterial.probability(), tTolerance);

        auto tTags = tRandomMaterial.tags();
        ASSERT_EQ(2u, tTags.size());
        for(auto& tTag : tTags)
        {
            auto tTagIndex = &tTag - &tTags[0];
            ASSERT_STREQ(tGoldTags[tTagIndex].c_str(), tTag.c_str());
            ASSERT_STREQ("homogeneous", tRandomMaterial.attribute(tTag).c_str());
            ASSERT_STREQ(tGoldSamples[tMatIndex][tTagIndex].c_str(), tRandomMaterial.value(tTag).c_str());
        }
    }
}

TEST(PlatoTest, SROM_BuildRandomMaterialSet)
{
    // DEFINE RANDOM MATERIAL
    Plato::srom::Material tMaterial;
    tMaterial.blockID("1");
    tMaterial.materialID("2");
    tMaterial.category("isotropic");
    Plato::srom::Statistics tPoissonsRatioStats;
    tPoissonsRatioStats.mDistribution = "beta";
    tPoissonsRatioStats.mMean = "0.3";
    tPoissonsRatioStats.mUpperBound = "0.4";
    tPoissonsRatioStats.mLowerBound = "0.25";
    tPoissonsRatioStats.mStandardDeviation = "0.05";
    tPoissonsRatioStats.mNumSamples = "2";
    tMaterial.append("poissons ratio", "homogeneous", tPoissonsRatioStats);
    tMaterial.append("elastic modulus", "homogeneous", "1e9");
    auto tTags = tMaterial.tags();
    ASSERT_EQ(2u, tTags.size());
    ASSERT_STREQ("poissons ratio", tTags[0].c_str());
    ASSERT_STREQ("elastic modulus", tTags[1].c_str());

    // INITIALIZE RANDOM MATERIAL SET
    Plato::srom::SromVariable tSromVariable;
    tSromVariable.mTag = "poissons ratio";
    tSromVariable.mAttribute = "homogeneous";
    tSromVariable.mSampleProbPairs.mNumSamples = 2;
    tSromVariable.mSampleProbPairs.mSamples = {0.32, 0.27};
    tSromVariable.mSampleProbPairs.mProbabilities = {0.75, 0.25};

    std::vector<Plato::srom::SromVariable> tSromVariableSet;
    tSromVariableSet.push_back(tSromVariable);

    // BUILD RANDOM MATERIAL SET
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    EXPECT_NO_THROW(Plato::srom::build_random_material_set(tMaterial, tSromVariableSet, tRandomMaterialSet));

    const double tTolerance = 1e-4;
    const std::vector<double> tGoldProbs = {0.75, 0.25};
    const std::vector<std::string> tGoldTags = {"elastic modulus", "poissons ratio"};
    const std::vector<std::vector<std::string>> tGoldSamples = { {"1e9", "0.3200000000000000"}, {"1e9", "0.2700000000000000"}};

    for(auto& tRandomMaterial : tRandomMaterialSet)
    {
        ASSERT_STREQ("1", tRandomMaterial.blockID().c_str());
        ASSERT_STREQ("2", tRandomMaterial.materialID().c_str());
        ASSERT_STREQ("isotropic", tRandomMaterial.category().c_str());

        auto tMatIndex = &tRandomMaterial - &tRandomMaterialSet[0];
        ASSERT_NEAR(tGoldProbs[tMatIndex], tRandomMaterial.probability(), tTolerance);

        auto tTags = tRandomMaterial.tags();
        ASSERT_EQ(2u, tTags.size());
        for(auto& tTag : tTags)
        {
            auto tTagIndex = &tTag - &tTags[0];
            ASSERT_STREQ(tGoldTags[tTagIndex].c_str(), tTag.c_str());
            ASSERT_STREQ("homogeneous", tRandomMaterial.attribute(tTag).c_str());
            ASSERT_STREQ(tGoldSamples[tMatIndex][tTagIndex].c_str(), tRandomMaterial.value(tTag).c_str());
        }
    }
}

TEST(PlatoTest, SROM_InitializeRandomMaterialCases_Error)
{
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialCases;
    EXPECT_THROW(Plato::srom::initialize_random_material_cases(tRandomMaterialSet, tRandomMaterialCases), std::runtime_error);
}

TEST(PlatoTest, SROM_InitializeRandomMaterialCases)
{
    // DEFINE RANDOM MATERIAL
    Plato::srom::Material tMaterial;
    tMaterial.blockID("1");
    tMaterial.materialID("2");
    tMaterial.category("isotropic");
    Plato::srom::Statistics tPoissonsRatioStats;
    tPoissonsRatioStats.mDistribution = "beta";
    tPoissonsRatioStats.mMean = "0.3";
    tPoissonsRatioStats.mUpperBound = "0.4";
    tPoissonsRatioStats.mLowerBound = "0.25";
    tPoissonsRatioStats.mStandardDeviation = "0.05";
    tPoissonsRatioStats.mNumSamples = "2";
    tMaterial.append("poissons ratio", "homogeneous", tPoissonsRatioStats);
    tMaterial.append("elastic modulus", "homogeneous", "1e9");
    auto tTags = tMaterial.tags();
    ASSERT_EQ(2u, tTags.size());
    ASSERT_STREQ("poissons ratio", tTags[0].c_str());
    ASSERT_STREQ("elastic modulus", tTags[1].c_str());

    // INITIALIZE RANDOM MATERIAL SET
    Plato::srom::SromVariable tSromVariable;
    tSromVariable.mTag = "poissons ratio";
    tSromVariable.mAttribute = "homogeneous";
    tSromVariable.mSampleProbPairs.mNumSamples = 2;
    tSromVariable.mSampleProbPairs.mSamples = {0.32, 0.27};
    tSromVariable.mSampleProbPairs.mProbabilities = {0.75, 0.25};

    std::vector<Plato::srom::SromVariable> tSromVariableSet;
    tSromVariableSet.push_back(tSromVariable);

    // BUILD RANDOM MATERIAL SET
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    EXPECT_NO_THROW(Plato::srom::build_random_material_set(tMaterial, tSromVariableSet, tRandomMaterialSet));
    ASSERT_EQ(2u, tRandomMaterialSet.size());

    // BUILD INITIAL RANDOM MATERIAL CASES
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialCases;
    EXPECT_NO_THROW(Plato::srom::initialize_random_material_cases(tRandomMaterialSet, tRandomMaterialCases));
    ASSERT_EQ(2u, tRandomMaterialCases.size());


    const double tTolerance = 1e-4;
    const std::vector<double> tGoldProbs = {0.75, 0.25};
    const std::vector<std::vector<std::string>> tGoldTags =
        { {"elastic modulus", "poissons ratio"}, {"elastic modulus", "poissons ratio"} };
    const std::vector<std::vector<std::string>> tGoldSamples = { {"1e9", "0.3200000000000000"}, {"1e9", "0.2700000000000000"}};

    for(auto& tRandomMaterialCase : tRandomMaterialCases)
    {
        ASSERT_EQ(1u, tRandomMaterialCase.numMaterials());

        auto tCaseIndex = &tRandomMaterialCase - &tRandomMaterialCases[0];
        ASSERT_NEAR(tGoldProbs[tCaseIndex], tRandomMaterialCase.probability(), tTolerance);

        auto tMaterialIDs = tRandomMaterialCase.materialIDs();
        ASSERT_EQ(1u, tMaterialIDs.size());
        for(auto& tMatID : tMaterialIDs)
        {
            ASSERT_STREQ("2", tMatID.c_str());
            ASSERT_STREQ("1", tRandomMaterialCase.blockID(tMatID).c_str());
            ASSERT_STREQ("isotropic", tRandomMaterialCase.category(tMatID).c_str());

            auto tTags = tRandomMaterialCase.tags(tMatID);
            ASSERT_EQ(2u, tTags.size());
            for(auto& tTag : tTags)
            {
                auto tTagIndex = &tTag - &tTags[0];
                ASSERT_STREQ(tGoldTags[tCaseIndex][tTagIndex].c_str(), tTag.c_str());
                ASSERT_STREQ("homogeneous", tRandomMaterialCase.attribute(tMatID, tTag).c_str());
                ASSERT_STREQ(tGoldSamples[tCaseIndex][tTagIndex].c_str(), tRandomMaterialCase.value(tMatID, tTag).c_str());
            }
        }
    }
}

TEST(PlatoTest, SROM_UpdateRandomMaterialCases_Error)
{
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialCases;
    EXPECT_THROW(Plato::srom::update_random_material_cases(tRandomMaterialSet, tRandomMaterialCases), std::runtime_error);
}

TEST(PlatoTest, SROM_UpdateRandomMaterialCases)
{
    // 1.1 DEFINE RANDOM MATERIAL ONE
    Plato::srom::Material tMaterialOne;
    tMaterialOne.blockID("1");
    tMaterialOne.materialID("2");
    tMaterialOne.category("isotropic");
    Plato::srom::Statistics tPoissonsRatioStats;
    tPoissonsRatioStats.mDistribution = "beta";
    tPoissonsRatioStats.mMean = "0.3";
    tPoissonsRatioStats.mUpperBound = "0.4";
    tPoissonsRatioStats.mLowerBound = "0.25";
    tPoissonsRatioStats.mStandardDeviation = "0.05";
    tPoissonsRatioStats.mNumSamples = "2";
    tMaterialOne.append("poissons ratio", "homogeneous", tPoissonsRatioStats);
    tMaterialOne.append("elastic modulus", "homogeneous", "1e9");
    auto tTags = tMaterialOne.tags();
    ASSERT_EQ(2u, tTags.size());
    ASSERT_STREQ("poissons ratio", tTags[0].c_str());
    ASSERT_STREQ("elastic modulus", tTags[1].c_str());

    // 1.2 INITIALIZE RANDOM MATERIAL SET ONE
    Plato::srom::SromVariable tSromVariableOne;
    tSromVariableOne.mTag = "poissons ratio";
    tSromVariableOne.mAttribute = "homogeneous";
    tSromVariableOne.mSampleProbPairs.mNumSamples = 2;
    tSromVariableOne.mSampleProbPairs.mSamples = {0.32, 0.27};
    tSromVariableOne.mSampleProbPairs.mProbabilities = {0.75, 0.25};

    std::vector<Plato::srom::SromVariable> tSromVariableSetOne;
    tSromVariableSetOne.push_back(tSromVariableOne);

    // 1.3 BUILD RANDOM MATERIAL SET ONE
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSetOne;
    EXPECT_NO_THROW(Plato::srom::build_random_material_set(tMaterialOne, tSromVariableSetOne, tRandomMaterialSetOne));
    ASSERT_EQ(2u, tRandomMaterialSetOne.size());

    // 1.4 BUILD INITIAL RANDOM MATERIAL CASES
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialCases;
    EXPECT_NO_THROW(Plato::srom::initialize_random_material_cases(tRandomMaterialSetOne, tRandomMaterialCases));
    ASSERT_EQ(2u, tRandomMaterialCases.size());

    // 2.1 DEFINE RANDOM MATERIAL TWO
    Plato::srom::Material tMaterialTwo;
    tMaterialTwo.blockID("2");
    tMaterialTwo.materialID("3");
    tMaterialTwo.category("isotropic");
    Plato::srom::Statistics tElasticModulusStats;
    tElasticModulusStats.mDistribution = "beta";
    tElasticModulusStats.mMean = "10";
    tElasticModulusStats.mUpperBound = "20";
    tElasticModulusStats.mLowerBound = "5";
    tElasticModulusStats.mStandardDeviation = "2";
    tElasticModulusStats.mNumSamples = "2";
    tMaterialTwo.append("elastic modulus", "homogeneous", tElasticModulusStats);
    tMaterialTwo.append("poissons ratio", "homogeneous", "0.28");
    tTags = tMaterialTwo.tags();
    ASSERT_EQ(2u, tTags.size());
    ASSERT_STREQ("elastic modulus", tTags[0].c_str());
    ASSERT_STREQ("poissons ratio", tTags[1].c_str());

    // 2.2 INITIALIZE RANDOM MATERIAL SET TWO
    Plato::srom::SromVariable tSromVariableTwo;
    tSromVariableTwo.mTag = "elastic modulus";
    tSromVariableTwo.mAttribute = "homogeneous";
    tSromVariableTwo.mSampleProbPairs.mNumSamples = 2;
    tSromVariableTwo.mSampleProbPairs.mSamples = {8, 17};
    tSromVariableTwo.mSampleProbPairs.mProbabilities = {0.65, 0.35};

    std::vector<Plato::srom::SromVariable> tSromVariableSetTwo;
    tSromVariableSetTwo.push_back(tSromVariableTwo);

    // 2.3 BUILD RANDOM MATERIAL SET TWO
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSetTwo;
    EXPECT_NO_THROW(Plato::srom::build_random_material_set(tMaterialTwo, tSromVariableSetTwo, tRandomMaterialSetTwo));
    ASSERT_EQ(2u, tRandomMaterialSetTwo.size());

    // 2.4 UPDATE SET OF RANDOM MATERIAL CASES
    EXPECT_NO_THROW(Plato::srom::update_random_material_cases(tRandomMaterialSetTwo, tRandomMaterialCases));
    ASSERT_EQ(4u, tRandomMaterialCases.size());

    // TEST RESULTS
    const double tTolerance = 1e-4;
    const std::vector<double> tGoldProbs = { 0.4875, 0.1625, 0.2625, 0.0875 };
    const std::vector<std::vector<std::string>> tGoldMatIDs =
        { {"2", "3"}, {"2", "3"}, {"2", "3"}, {"2", "3"} };
    const std::vector<std::vector<std::string>> tGoldBlockIDs =
        { {"1", "2"}, {"1", "2"}, {"1", "2"}, {"1", "2"} };
    const std::vector<std::vector<std::string>> tGoldTags =
        { {"elastic modulus", "poissons ratio"},
          {"elastic modulus", "poissons ratio"},
          {"elastic modulus", "poissons ratio"},
          {"elastic modulus", "poissons ratio"} };
    const std::vector<std::vector<std::vector<std::string>>> tGoldSamples =
        {
          { {"1e9", "0.3200000000000000"}, {"8.0000000000000000", "0.28"} },
          { {"1e9", "0.2700000000000000"}, {"8.0000000000000000", "0.28"} },
          { {"1e9", "0.3200000000000000"}, {"17.0000000000000000", "0.28"} },
          { {"1e9", "0.2700000000000000"}, {"17.0000000000000000", "0.28"} },
        };

    for(auto& tRandomMaterialCase : tRandomMaterialCases)
    {
        ASSERT_EQ(2u, tRandomMaterialCase.numMaterials());

        auto tCaseIndex = &tRandomMaterialCase - &tRandomMaterialCases[0];
        ASSERT_NEAR(tGoldProbs[tCaseIndex], tRandomMaterialCase.probability(), tTolerance);

        auto tMaterialIDs = tRandomMaterialCase.materialIDs();
        ASSERT_EQ(2u, tMaterialIDs.size());
        for(auto& tMatID : tMaterialIDs)
        {
            auto tMatIdIndex = &tMatID - &tMaterialIDs[0];
            ASSERT_STREQ(tGoldMatIDs[tCaseIndex][tMatIdIndex].c_str(), tMatID.c_str());
            ASSERT_STREQ(tGoldBlockIDs[tCaseIndex][tMatIdIndex].c_str(), tRandomMaterialCase.blockID(tMatID).c_str());
            ASSERT_STREQ("isotropic", tRandomMaterialCase.category(tMatID).c_str());

            auto tTags = tRandomMaterialCase.tags(tMatID);
            ASSERT_EQ(2u, tTags.size());
            for(auto& tTag : tTags)
            {
                auto tTagIndex = &tTag - &tTags[0];
                ASSERT_STREQ(tGoldTags[tCaseIndex][tTagIndex].c_str(), tTag.c_str());
                ASSERT_STREQ("homogeneous", tRandomMaterialCase.attribute(tMatID, tTag).c_str());
                ASSERT_STREQ(tGoldSamples[tCaseIndex][tMatIdIndex][tTagIndex].c_str(), tRandomMaterialCase.value(tMatID, tTag).c_str());
            }
        }
    }
}

TEST(PlatoTest, SROM_AppendRandomMaterialSet_Error)
{
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialCases;
    EXPECT_THROW(Plato::srom::append_random_material_set(tRandomMaterialSet, tRandomMaterialCases), std::runtime_error);
}

TEST(PlatoTest, SROM_AppendRandomMaterialSet)
{
    // 1.1 DEFINE RANDOM MATERIAL ONE
    Plato::srom::Material tMaterialOne;
    tMaterialOne.blockID("1");
    tMaterialOne.materialID("2");
    tMaterialOne.category("isotropic");
    Plato::srom::Statistics tPoissonsRatioStats;
    tPoissonsRatioStats.mDistribution = "beta";
    tPoissonsRatioStats.mMean = "0.3";
    tPoissonsRatioStats.mUpperBound = "0.4";
    tPoissonsRatioStats.mLowerBound = "0.25";
    tPoissonsRatioStats.mStandardDeviation = "0.05";
    tPoissonsRatioStats.mNumSamples = "2";
    tMaterialOne.append("poissons ratio", "homogeneous", tPoissonsRatioStats);
    tMaterialOne.append("elastic modulus", "homogeneous", "1e9");
    auto tTags = tMaterialOne.tags();
    ASSERT_EQ(2u, tTags.size());
    ASSERT_STREQ("poissons ratio", tTags[0].c_str());
    ASSERT_STREQ("elastic modulus", tTags[1].c_str());

    // 1.2 INITIALIZE RANDOM MATERIAL SET ONE
    Plato::srom::SromVariable tSromVariableOne;
    tSromVariableOne.mTag = "poissons ratio";
    tSromVariableOne.mAttribute = "homogeneous";
    tSromVariableOne.mSampleProbPairs.mNumSamples = 2;
    tSromVariableOne.mSampleProbPairs.mSamples = {0.32, 0.27};
    tSromVariableOne.mSampleProbPairs.mProbabilities = {0.75, 0.25};

    std::vector<Plato::srom::SromVariable> tSromVariableSetOne;
    tSromVariableSetOne.push_back(tSromVariableOne);

    // 1.3 BUILD RANDOM MATERIAL SET ONE
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSetOne;
    EXPECT_NO_THROW(Plato::srom::build_random_material_set(tMaterialOne, tSromVariableSetOne, tRandomMaterialSetOne));
    ASSERT_EQ(2u, tRandomMaterialSetOne.size());

    // 1.4 APPEND RANDOM MATERIAL SET
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialCases;
    EXPECT_NO_THROW(Plato::srom::append_random_material_set(tRandomMaterialSetOne, tRandomMaterialCases));

    // 2.1 DEFINE RANDOM MATERIAL TWO
    Plato::srom::Material tMaterialTwo;
    tMaterialTwo.blockID("2");
    tMaterialTwo.materialID("3");
    tMaterialTwo.category("isotropic");
    Plato::srom::Statistics tElasticModulusStats;
    tElasticModulusStats.mDistribution = "beta";
    tElasticModulusStats.mMean = "10";
    tElasticModulusStats.mUpperBound = "20";
    tElasticModulusStats.mLowerBound = "5";
    tElasticModulusStats.mStandardDeviation = "2";
    tElasticModulusStats.mNumSamples = "2";
    tMaterialTwo.append("elastic modulus", "homogeneous", tElasticModulusStats);
    tMaterialTwo.append("poissons ratio", "homogeneous", "0.28");
    tTags = tMaterialTwo.tags();
    ASSERT_EQ(2u, tTags.size());
    ASSERT_STREQ("elastic modulus", tTags[0].c_str());
    ASSERT_STREQ("poissons ratio", tTags[1].c_str());

    // 2.2 INITIALIZE RANDOM MATERIAL SET TWO
    Plato::srom::SromVariable tSromVariableTwo;
    tSromVariableTwo.mTag = "elastic modulus";
    tSromVariableTwo.mAttribute = "homogeneous";
    tSromVariableTwo.mSampleProbPairs.mNumSamples = 2;
    tSromVariableTwo.mSampleProbPairs.mSamples = {8, 17};
    tSromVariableTwo.mSampleProbPairs.mProbabilities = {0.65, 0.35};

    std::vector<Plato::srom::SromVariable> tSromVariableSetTwo;
    tSromVariableSetTwo.push_back(tSromVariableTwo);

    // 2.3 BUILD RANDOM MATERIAL SET TWO
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSetTwo;
    EXPECT_NO_THROW(Plato::srom::build_random_material_set(tMaterialTwo, tSromVariableSetTwo, tRandomMaterialSetTwo));
    ASSERT_EQ(2u, tRandomMaterialSetTwo.size());

    // 2.4 APPEND RANDOM MATERIAL SET
    EXPECT_NO_THROW(Plato::srom::append_random_material_set(tRandomMaterialSetTwo, tRandomMaterialCases));

    // TEST RESULTS
    const double tTolerance = 1e-4;
    const std::vector<double> tGoldProbs = { 0.4875, 0.1625, 0.2625, 0.0875 };
    const std::vector<std::vector<std::string>> tGoldMatIDs =
        { {"2", "3"}, {"2", "3"}, {"2", "3"}, {"2", "3"} };
    const std::vector<std::vector<std::string>> tGoldBlockIDs =
        { {"1", "2"}, {"1", "2"}, {"1", "2"}, {"1", "2"} };
    const std::vector<std::vector<std::string>> tGoldTags =
        { {"elastic modulus", "poissons ratio"},
          {"elastic modulus", "poissons ratio"},
          {"elastic modulus", "poissons ratio"},
          {"elastic modulus", "poissons ratio"} };
    const std::vector<std::vector<std::vector<std::string>>> tGoldSamples =
        {
          { {"1e9", "0.3200000000000000"}, {"8.0000000000000000", "0.28"} },
          { {"1e9", "0.2700000000000000"}, {"8.0000000000000000", "0.28"} },
          { {"1e9", "0.3200000000000000"}, {"17.0000000000000000", "0.28"} },
          { {"1e9", "0.2700000000000000"}, {"17.0000000000000000", "0.28"} },
        };

    for(auto& tRandomMaterialCase : tRandomMaterialCases)
    {
        ASSERT_EQ(2u, tRandomMaterialCase.numMaterials());

        auto tCaseIndex = &tRandomMaterialCase - &tRandomMaterialCases[0];
        ASSERT_NEAR(tGoldProbs[tCaseIndex], tRandomMaterialCase.probability(), tTolerance);

        auto tMaterialIDs = tRandomMaterialCase.materialIDs();
        ASSERT_EQ(2u, tMaterialIDs.size());
        for(auto& tMatID : tMaterialIDs)
        {
            auto tMatIdIndex = &tMatID - &tMaterialIDs[0];
            ASSERT_STREQ(tGoldMatIDs[tCaseIndex][tMatIdIndex].c_str(), tMatID.c_str());
            ASSERT_STREQ(tGoldBlockIDs[tCaseIndex][tMatIdIndex].c_str(), tRandomMaterialCase.blockID(tMatID).c_str());
            ASSERT_STREQ("isotropic", tRandomMaterialCase.category(tMatID).c_str());

            auto tTags = tRandomMaterialCase.tags(tMatID);
            ASSERT_EQ(2u, tTags.size());
            for(auto& tTag : tTags)
            {
                auto tTagIndex = &tTag - &tTags[0];
                ASSERT_STREQ(tGoldTags[tCaseIndex][tTagIndex].c_str(), tTag.c_str());
                ASSERT_STREQ("homogeneous", tRandomMaterialCase.attribute(tMatID, tTag).c_str());
                ASSERT_STREQ(tGoldSamples[tCaseIndex][tMatIdIndex][tTagIndex].c_str(), tRandomMaterialCase.value(tMatID, tTag).c_str());
            }
        }
    }
}

TEST(PlatoTest, SROM_AppendDeterministicMaterials_Error)
{
    // 1.1 MATERIAL CASE 1: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial11;
    tRandomMaterial11.blockID("1");
    tRandomMaterial11.category("isotropic");
    tRandomMaterial11.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial11.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase1;
    tRandomMatCase1.append("2", tRandomMaterial11);

    // 1.2 MATERIAL CASE 1: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial12;
    tRandomMaterial12.blockID("2");
    tRandomMaterial12.category("isotropic");
    tRandomMaterial12.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial12.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase1.append("3", tRandomMaterial12);
    tRandomMatCase1.probability(0.4875);

    // 2.1 MATERIAL CASE 2: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial21;
    tRandomMaterial21.blockID("1");
    tRandomMaterial21.category("isotropic");
    tRandomMaterial21.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial21.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase2;
    tRandomMatCase2.append("2", tRandomMaterial21);

    // 2.2 MATERIAL CASE 2: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial22;
    tRandomMaterial22.blockID("2");
    tRandomMaterial22.category("isotropic");
    tRandomMaterial22.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial22.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase2.append("3", tRandomMaterial22);
    tRandomMatCase2.probability(0.1625);

    // 3.1 MATERIAL CASE 3: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial31;
    tRandomMaterial31.blockID("1");
    tRandomMaterial31.category("isotropic");
    tRandomMaterial31.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial31.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase3;
    tRandomMatCase3.append("2", tRandomMaterial31);

    // 3.2 MATERIAL CASE 3: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial32;
    tRandomMaterial32.blockID("2");
    tRandomMaterial32.category("isotropic");
    tRandomMaterial32.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial32.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase3.append("3", tRandomMaterial32);
    tRandomMatCase3.probability(0.2625);

    // 4.1 MATERIAL CASE 4: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial41;
    tRandomMaterial41.blockID("1");
    tRandomMaterial41.category("isotropic");
    tRandomMaterial41.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial41.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase4;
    tRandomMatCase4.append("2", tRandomMaterial41);

    // 4.2 MATERIAL CASE 4: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial42;
    tRandomMaterial42.blockID("2");
    tRandomMaterial42.category("isotropic");
    tRandomMaterial42.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial42.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase4.append("3", tRandomMaterial42);
    tRandomMatCase4.probability(0.0875);

    // 5. APPEND RANDOM MATERIAL CASES
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialCases;
    tRandomMaterialCases.push_back(tRandomMatCase1);
    tRandomMaterialCases.push_back(tRandomMatCase2);
    tRandomMaterialCases.push_back(tRandomMatCase3);
    tRandomMaterialCases.push_back(tRandomMatCase4);

    // 6. APPEND DETERMINISTIC MATERIAL.
    Plato::srom::Material tDeterministicMaterial;
    tDeterministicMaterial.blockID("3");
    tDeterministicMaterial.materialID("4");
    tDeterministicMaterial.category("isotropic");
    tDeterministicMaterial.append("poissons ratio", "homogeneous", "0.4");
    Plato::srom::Statistics tStats;
    tStats.mDistribution = "beta";
    tStats.mMean = "10";
    tStats.mUpperBound = "20";
    tStats.mLowerBound = "5";
    tStats.mStandardDeviation = "2";
    tStats.mNumSamples = "2";
    tDeterministicMaterial.append("elastic modulus", "homogeneous", tStats);

    std::vector<Plato::srom::Material> tDeterministicMaterialSet;
    tDeterministicMaterialSet.push_back(tDeterministicMaterial);
    EXPECT_THROW(Plato::srom::append_deterministic_materials(tDeterministicMaterialSet, tRandomMaterialCases), std::runtime_error);
}

TEST(PlatoTest, SROM_AppendDeterministicMaterials_Test1)
{
    // 1.1 MATERIAL CASE 1: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial11;
    tRandomMaterial11.blockID("1");
    tRandomMaterial11.category("isotropic");
    tRandomMaterial11.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial11.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase1;
    tRandomMatCase1.append("2", tRandomMaterial11);

    // 1.2 MATERIAL CASE 1: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial12;
    tRandomMaterial12.blockID("2");
    tRandomMaterial12.category("isotropic");
    tRandomMaterial12.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial12.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase1.append("3", tRandomMaterial12);
    tRandomMatCase1.probability(0.4875);

    // 2.1 MATERIAL CASE 2: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial21;
    tRandomMaterial21.blockID("1");
    tRandomMaterial21.category("isotropic");
    tRandomMaterial21.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial21.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase2;
    tRandomMatCase2.append("2", tRandomMaterial21);

    // 2.2 MATERIAL CASE 2: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial22;
    tRandomMaterial22.blockID("2");
    tRandomMaterial22.category("isotropic");
    tRandomMaterial22.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial22.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase2.append("3", tRandomMaterial22);
    tRandomMatCase2.probability(0.1625);

    // 3.1 MATERIAL CASE 3: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial31;
    tRandomMaterial31.blockID("1");
    tRandomMaterial31.category("isotropic");
    tRandomMaterial31.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial31.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase3;
    tRandomMatCase3.append("2", tRandomMaterial31);

    // 3.2 MATERIAL CASE 3: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial32;
    tRandomMaterial32.blockID("2");
    tRandomMaterial32.category("isotropic");
    tRandomMaterial32.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial32.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase3.append("3", tRandomMaterial32);
    tRandomMatCase3.probability(0.2625);

    // 4.1 MATERIAL CASE 4: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial41;
    tRandomMaterial41.blockID("1");
    tRandomMaterial41.category("isotropic");
    tRandomMaterial41.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial41.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase4;
    tRandomMatCase4.append("2", tRandomMaterial41);

    // 4.2 MATERIAL CASE 4: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial42;
    tRandomMaterial42.blockID("2");
    tRandomMaterial42.category("isotropic");
    tRandomMaterial42.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial42.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase4.append("3", tRandomMaterial42);
    tRandomMatCase4.probability(0.0875);

    // 5. APPEND RANDOM MATERIAL CASES
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialCases;
    tRandomMaterialCases.push_back(tRandomMatCase1);
    tRandomMaterialCases.push_back(tRandomMatCase2);
    tRandomMaterialCases.push_back(tRandomMatCase3);
    tRandomMaterialCases.push_back(tRandomMatCase4);

    // 6. TEST: NO DETERMINISTIC MATERIALS WILL BE APPENDED.
    std::vector<Plato::srom::Material> tDeterministicMaterials;
    EXPECT_NO_THROW(Plato::srom::append_deterministic_materials(tDeterministicMaterials, tRandomMaterialCases));

    const double tTolerance = 1e-4;
    const std::vector<double> tGoldProbs = { 0.4875, 0.1625, 0.2625, 0.0875 };
    const std::vector<std::vector<std::string>> tGoldMatIDs =
        { {"2", "3"}, {"2", "3"}, {"2", "3"}, {"2", "3"} };
    const std::vector<std::vector<std::string>> tGoldBlockIDs =
        { {"1", "2"}, {"1", "2"}, {"1", "2"}, {"1", "2"} };
    const std::vector<std::vector<std::string>> tGoldTags =
        { {"elastic modulus", "poissons ratio"},
          {"elastic modulus", "poissons ratio"},
          {"elastic modulus", "poissons ratio"},
          {"elastic modulus", "poissons ratio"} };
    const std::vector<std::vector<std::vector<std::string>>> tGoldSamples =
        {
          { {"1e9", "0.32"}, {"8.0", "0.28"} },
          { {"1e9", "0.27"}, {"8.0", "0.28"} },
          { {"1e9", "0.32"}, {"17.0", "0.28"} },
          { {"1e9", "0.27"}, {"17.0", "0.28"} },
        };

    for(auto& tRandomMaterialCase : tRandomMaterialCases)
    {
        ASSERT_EQ(2u, tRandomMaterialCase.numMaterials());

        auto tCaseIndex = &tRandomMaterialCase - &tRandomMaterialCases[0];
        ASSERT_NEAR(tGoldProbs[tCaseIndex], tRandomMaterialCase.probability(), tTolerance);

        auto tMaterialIDs = tRandomMaterialCase.materialIDs();
        ASSERT_EQ(2u, tMaterialIDs.size());
        for(auto& tMatID : tMaterialIDs)
        {
            auto tMatIdIndex = &tMatID - &tMaterialIDs[0];
            ASSERT_STREQ(tGoldMatIDs[tCaseIndex][tMatIdIndex].c_str(), tMatID.c_str());
            ASSERT_STREQ(tGoldBlockIDs[tCaseIndex][tMatIdIndex].c_str(), tRandomMaterialCase.blockID(tMatID).c_str());
            ASSERT_STREQ("isotropic", tRandomMaterialCase.category(tMatID).c_str());

            auto tTags = tRandomMaterialCase.tags(tMatID);
            ASSERT_EQ(2u, tTags.size());
            for(auto& tTag : tTags)
            {
                auto tTagIndex = &tTag - &tTags[0];
                ASSERT_STREQ(tGoldTags[tCaseIndex][tTagIndex].c_str(), tTag.c_str());
                ASSERT_STREQ("homogeneous", tRandomMaterialCase.attribute(tMatID, tTag).c_str());
                ASSERT_STREQ(tGoldSamples[tCaseIndex][tMatIdIndex][tTagIndex].c_str(), tRandomMaterialCase.value(tMatID, tTag).c_str());
            }
        }
    }
}

TEST(PlatoTest, SROM_AppendDeterministicMaterials_Test2)
{
    // 1.1 MATERIAL CASE 1: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial11;
    tRandomMaterial11.blockID("1");
    tRandomMaterial11.category("isotropic");
    tRandomMaterial11.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial11.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase1;
    tRandomMatCase1.append("2", tRandomMaterial11);

    // 1.2 MATERIAL CASE 1: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial12;
    tRandomMaterial12.blockID("2");
    tRandomMaterial12.category("isotropic");
    tRandomMaterial12.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial12.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase1.append("3", tRandomMaterial12);
    tRandomMatCase1.probability(0.4875);

    // 2.1 MATERIAL CASE 2: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial21;
    tRandomMaterial21.blockID("1");
    tRandomMaterial21.category("isotropic");
    tRandomMaterial21.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial21.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase2;
    tRandomMatCase2.append("2", tRandomMaterial21);

    // 2.2 MATERIAL CASE 2: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial22;
    tRandomMaterial22.blockID("2");
    tRandomMaterial22.category("isotropic");
    tRandomMaterial22.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial22.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase2.append("3", tRandomMaterial22);
    tRandomMatCase2.probability(0.1625);

    // 3.1 MATERIAL CASE 3: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial31;
    tRandomMaterial31.blockID("1");
    tRandomMaterial31.category("isotropic");
    tRandomMaterial31.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial31.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase3;
    tRandomMatCase3.append("2", tRandomMaterial31);

    // 3.2 MATERIAL CASE 3: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial32;
    tRandomMaterial32.blockID("2");
    tRandomMaterial32.category("isotropic");
    tRandomMaterial32.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial32.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase3.append("3", tRandomMaterial32);
    tRandomMatCase3.probability(0.2625);

    // 4.1 MATERIAL CASE 4: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial41;
    tRandomMaterial41.blockID("1");
    tRandomMaterial41.category("isotropic");
    tRandomMaterial41.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial41.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase4;
    tRandomMatCase4.append("2", tRandomMaterial41);

    // 4.2 MATERIAL CASE 4: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial42;
    tRandomMaterial42.blockID("2");
    tRandomMaterial42.category("isotropic");
    tRandomMaterial42.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial42.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase4.append("3", tRandomMaterial42);
    tRandomMatCase4.probability(0.0875);

    // 5. APPEND RANDOM MATERIAL CASES
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialCases;
    tRandomMaterialCases.push_back(tRandomMatCase1);
    tRandomMaterialCases.push_back(tRandomMatCase2);
    tRandomMaterialCases.push_back(tRandomMatCase3);
    tRandomMaterialCases.push_back(tRandomMatCase4);

    // 6. APPEND DETERMINISTIC MATERIAL.
    Plato::srom::Material tDeterministicMaterial;
    tDeterministicMaterial.blockID("3");
    tDeterministicMaterial.materialID("4");
    tDeterministicMaterial.category("isotropic");
    tDeterministicMaterial.append("poissons ratio", "homogeneous", "0.4");
    tDeterministicMaterial.append("elastic modulus", "homogeneous", "1.0");
    std::vector<Plato::srom::Material> tDeterministicMaterialSet;
    tDeterministicMaterialSet.push_back(tDeterministicMaterial);
    EXPECT_NO_THROW(Plato::srom::append_deterministic_materials(tDeterministicMaterialSet, tRandomMaterialCases));

    const double tTolerance = 1e-4;
    const std::vector<double> tGoldProbs = { 0.4875, 0.1625, 0.2625, 0.0875 };
    const std::vector<std::vector<std::string>> tGoldMatIDs =
        { {"2", "3", "4"}, {"2", "3", "4"}, {"2", "3", "4"}, {"2", "3", "4"} };
    const std::vector<std::vector<std::string>> tGoldBlockIDs =
        { {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"} };
    const std::vector<std::vector<std::string>> tGoldTags =
        { {"elastic modulus", "poissons ratio"},
          {"elastic modulus", "poissons ratio"},
          {"elastic modulus", "poissons ratio"},
          {"elastic modulus", "poissons ratio"} };
    const std::vector<std::vector<std::vector<std::string>>> tGoldSamples =
        {
          { {"1e9", "0.32"}, {"8.0", "0.28"}, {"1.0", "0.4"} },
          { {"1e9", "0.27"}, {"8.0", "0.28"}, {"1.0", "0.4"} },
          { {"1e9", "0.32"}, {"17.0", "0.28"}, {"1.0", "0.4"} },
          { {"1e9", "0.27"}, {"17.0", "0.28"}, {"1.0", "0.4"} },
        };

    for(auto& tRandomMaterialCase : tRandomMaterialCases)
    {
        ASSERT_EQ(3u, tRandomMaterialCase.numMaterials());

        auto tCaseIndex = &tRandomMaterialCase - &tRandomMaterialCases[0];
        ASSERT_NEAR(tGoldProbs[tCaseIndex], tRandomMaterialCase.probability(), tTolerance);

        auto tMaterialIDs = tRandomMaterialCase.materialIDs();
        ASSERT_EQ(3u, tMaterialIDs.size());
        for(auto& tMatID : tMaterialIDs)
        {
            auto tMatIdIndex = &tMatID - &tMaterialIDs[0];
            ASSERT_STREQ(tGoldMatIDs[tCaseIndex][tMatIdIndex].c_str(), tMatID.c_str());
            ASSERT_STREQ(tGoldBlockIDs[tCaseIndex][tMatIdIndex].c_str(), tRandomMaterialCase.blockID(tMatID).c_str());
            ASSERT_STREQ("isotropic", tRandomMaterialCase.category(tMatID).c_str());

            auto tTags = tRandomMaterialCase.tags(tMatID);
            ASSERT_EQ(2u, tTags.size());
            for(auto& tTag : tTags)
            {
                auto tTagIndex = &tTag - &tTags[0];
                ASSERT_STREQ(tGoldTags[tCaseIndex][tTagIndex].c_str(), tTag.c_str());
                ASSERT_STREQ("homogeneous", tRandomMaterialCase.attribute(tMatID, tTag).c_str());
                ASSERT_STREQ(tGoldSamples[tCaseIndex][tMatIdIndex][tTagIndex].c_str(), tRandomMaterialCase.value(tMatID, tTag).c_str());
            }
        }
    }
}

TEST(PlatoTest, SROM_AssignMaterialCaseIdentificationNumber_Error)
{
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialCases;
    EXPECT_THROW(Plato::srom::assign_material_case_identification_number(tRandomMaterialCases), std::runtime_error);
}

TEST(PlatoTest, SROM_AssignMaterialCaseIdentificationNumber)
{
    // 1.1 MATERIAL CASE 1: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial11;
    tRandomMaterial11.blockID("1");
    tRandomMaterial11.category("isotropic");
    tRandomMaterial11.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial11.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase1;
    tRandomMatCase1.append("2", tRandomMaterial11);

    // 1.2 MATERIAL CASE 1: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial12;
    tRandomMaterial12.blockID("2");
    tRandomMaterial12.category("isotropic");
    tRandomMaterial12.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial12.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase1.append("3", tRandomMaterial12);
    tRandomMatCase1.probability(0.4875);

    // 2.1 MATERIAL CASE 2: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial21;
    tRandomMaterial21.blockID("1");
    tRandomMaterial21.category("isotropic");
    tRandomMaterial21.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial21.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase2;
    tRandomMatCase2.append("2", tRandomMaterial21);

    // 2.2 MATERIAL CASE 2: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial22;
    tRandomMaterial22.blockID("2");
    tRandomMaterial22.category("isotropic");
    tRandomMaterial22.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial22.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase2.append("3", tRandomMaterial22);
    tRandomMatCase2.probability(0.1625);

    // 3.1 MATERIAL CASE 3: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial31;
    tRandomMaterial31.blockID("1");
    tRandomMaterial31.category("isotropic");
    tRandomMaterial31.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial31.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase3;
    tRandomMatCase3.append("2", tRandomMaterial31);

    // 3.2 MATERIAL CASE 3: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial32;
    tRandomMaterial32.blockID("2");
    tRandomMaterial32.category("isotropic");
    tRandomMaterial32.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial32.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase3.append("3", tRandomMaterial32);
    tRandomMatCase3.probability(0.2625);

    // 4.1 MATERIAL CASE 4: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial41;
    tRandomMaterial41.blockID("1");
    tRandomMaterial41.category("isotropic");
    tRandomMaterial41.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial41.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase4;
    tRandomMatCase4.append("2", tRandomMaterial41);

    // 4.2 MATERIAL CASE 4: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial42;
    tRandomMaterial42.blockID("2");
    tRandomMaterial42.category("isotropic");
    tRandomMaterial42.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial42.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase4.append("3", tRandomMaterial42);
    tRandomMatCase4.probability(0.0875);

    // 5. APPEND RANDOM MATERIAL CASES
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialCases;
    tRandomMaterialCases.push_back(tRandomMatCase1);
    tRandomMaterialCases.push_back(tRandomMatCase2);
    tRandomMaterialCases.push_back(tRandomMatCase3);
    tRandomMaterialCases.push_back(tRandomMatCase4);

    // 6. APPEND DETERMINISTIC MATERIAL.
    Plato::srom::Material tDeterministicMaterial;
    tDeterministicMaterial.blockID("3");
    tDeterministicMaterial.materialID("4");
    tDeterministicMaterial.category("isotropic");
    tDeterministicMaterial.append("poissons ratio", "homogeneous", "0.4");
    tDeterministicMaterial.append("elastic modulus", "homogeneous", "1.0");
    std::vector<Plato::srom::Material> tDeterministicMaterialSet;
    tDeterministicMaterialSet.push_back(tDeterministicMaterial);
    EXPECT_NO_THROW(Plato::srom::append_deterministic_materials(tDeterministicMaterialSet, tRandomMaterialCases));
    EXPECT_NO_THROW(Plato::srom::assign_material_case_identification_number(tRandomMaterialCases));

    // 7. TEST RESULTS
    std::vector<std::string> tGoldIDs = {"1", "2", "3", "4"};
    for(auto& tMaterialCase : tRandomMaterialCases)
    {
        auto tIndex = &tMaterialCase - &tRandomMaterialCases[0];
        ASSERT_STREQ(tGoldIDs[tIndex].c_str(), tMaterialCase.caseID().c_str());
    }
}

TEST(PlatoTest, SROM_BuildMaterialSroms_Error)
{
    std::vector<Plato::srom::Material> tMaterialSet;
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialCaseSet;
    EXPECT_THROW(Plato::srom::build_material_sroms(tMaterialSet, tRandomMaterialCaseSet), std::runtime_error);
}

TEST(PlatoTest, SROM_BuildMaterialSroms)
{
    // 1.1 BUILD MATERIAL ONE
    Plato::srom::Material tMaterial1;
    tMaterial1.blockID("1");
    tMaterial1.materialID("1");
    tMaterial1.category("isotropic");
    Plato::srom::Statistics tElasticModulusStats1;
    tElasticModulusStats1.mDistribution = "beta";
    tElasticModulusStats1.mMean = "10";
    tElasticModulusStats1.mUpperBound = "20";
    tElasticModulusStats1.mLowerBound = "1";
    tElasticModulusStats1.mStandardDeviation = "3";
    tElasticModulusStats1.mNumSamples = "2";
    tMaterial1.append("youngs modulus", "homogeneous", tElasticModulusStats1);
    Plato::srom::Statistics tPoissonsRatioStats1;
    tPoissonsRatioStats1.mDistribution = "beta";
    tPoissonsRatioStats1.mMean = "0.3";
    tPoissonsRatioStats1.mUpperBound = "0.4";
    tPoissonsRatioStats1.mLowerBound = "0.25";
    tPoissonsRatioStats1.mStandardDeviation = "0.05";
    tPoissonsRatioStats1.mNumSamples = "2";
    tMaterial1.append("poissons ratio", "homogeneous", tPoissonsRatioStats1);

    // 1.2 BUILD MATERIAL TWO
    Plato::srom::Material tMaterial2;
    tMaterial2.blockID("2");
    tMaterial2.materialID("2");
    tMaterial2.category("isotropic");
    tMaterial2.append("youngs modulus", "homogeneous", "3.0");
    tMaterial2.append("poissons ratio", "homogeneous", "0.25");

    // 1.3 BUILD MATERIAL THREE
    Plato::srom::Material tMaterial3;
    tMaterial3.blockID("3");
    tMaterial3.materialID("3");
    tMaterial3.category("isotropic");
    tMaterial3.append("youngs modulus", "homogeneous", "1.0");
    Plato::srom::Statistics tPoissonsRatioStats2;
    tPoissonsRatioStats2.mDistribution = "beta";
    tPoissonsRatioStats2.mMean = "0.25";
    tPoissonsRatioStats2.mUpperBound = "0.35";
    tPoissonsRatioStats2.mLowerBound = "0.15";
    tPoissonsRatioStats2.mStandardDeviation = "0.1";
    tPoissonsRatioStats2.mNumSamples = "2";
    tMaterial3.append("poissons ratio", "homogeneous", tPoissonsRatioStats2);

    // 2 BUILD MATERIAL SET
    std::vector<Plato::srom::Material> tMaterialSet;
    tMaterialSet.push_back(tMaterial1);
    tMaterialSet.push_back(tMaterial2);
    tMaterialSet.push_back(tMaterial3);

    // 3 BUILD MATERIAL SROMS
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialCaseSet;
    EXPECT_NO_THROW(Plato::srom::build_material_sroms(tMaterialSet, tRandomMaterialCaseSet));
    ASSERT_EQ(8u, tRandomMaterialCaseSet.size());

    // 4 TEST RESULTS
    const double tTolerance = 1e-4;
    std::vector<std::string> tGoldIDs = {"1", "2", "3", "4", "5", "6", "7", "8"};
    const std::vector<double> tGoldProbs = { 0.1482212, 0.1440202, 0.0924449, 0.0898248,
                                             0.1641386, 0.1594865, 0.1023726, 0.0994711 };
    const std::vector<std::vector<std::string>> tGoldMatIDs =
        { {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"},
          {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"} };
    const std::vector<std::vector<std::string>> tGoldBlockIDs =
        { {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"},
          {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"} };
    const std::vector<std::vector<std::string>> tGoldTags =
        { {"poissons ratio", "youngs modulus"},
          {"poissons ratio", "youngs modulus"},
          {"poissons ratio", "youngs modulus"},
          {"poissons ratio", "youngs modulus"},
          {"poissons ratio", "youngs modulus"},
          {"poissons ratio", "youngs modulus"},
          {"poissons ratio", "youngs modulus"},
          {"poissons ratio", "youngs modulus"}
        };
    const std::vector<std::vector<std::vector<double>>> tGoldSamples =
        {
          { {0.2575729129623087, 7.6968145268325285},  {0.25, 3.0}, {0.3448799880969564, 1.0} },
          { {0.2575729129623087, 12.8669604630312087}, {0.25, 3.0}, {0.3448799880969564, 1.0} },
          { {0.3659845788546486, 7.6968145268325285},  {0.25, 3.0}, {0.3448799880969564, 1.0} },
          { {0.3659845788546486, 12.8669604630312087}, {0.25, 3.0}, {0.3448799880969564, 1.0} },
          { {0.2575729129623087, 7.6968145268325285},  {0.25, 3.0}, {0.2121659959660376, 1.0} },
          { {0.2575729129623087, 12.8669604630312087}, {0.25, 3.0}, {0.2121659959660376, 1.0} },
          { {0.3659845788546486, 7.6968145268325285},  {0.25, 3.0}, {0.2121659959660376, 1.0} },
          { {0.3659845788546486, 12.8669604630312087}, {0.25, 3.0}, {0.2121659959660376, 1.0} }
        };

    double tProbSum = 0;
    for(auto& tRandomMatCase : tRandomMaterialCaseSet)
    {
        ASSERT_EQ(3u, tRandomMatCase.numMaterials());
        auto tCaseIndex = &tRandomMatCase - &tRandomMaterialCaseSet[0];
        ASSERT_STREQ(tGoldIDs[tCaseIndex].c_str(), tRandomMatCase.caseID().c_str());
        EXPECT_NEAR(tGoldProbs[tCaseIndex], tRandomMatCase.probability(), tTolerance);
        tProbSum += tRandomMatCase.probability();

        auto tMaterialIDs = tRandomMatCase.materialIDs();
        ASSERT_EQ(3u, tMaterialIDs.size());
        for(auto& tMatID : tMaterialIDs)
        {
            auto tMatIdIndex = &tMatID - &tMaterialIDs[0];
            ASSERT_STREQ(tGoldMatIDs[tCaseIndex][tMatIdIndex].c_str(), tMatID.c_str());
            ASSERT_STREQ(tGoldBlockIDs[tCaseIndex][tMatIdIndex].c_str(), tRandomMatCase.blockID(tMatID).c_str());
            ASSERT_STREQ("isotropic", tRandomMatCase.category(tMatID).c_str());

            auto tTags = tRandomMatCase.tags(tMatID);
            ASSERT_EQ(2u, tTags.size());
            for(auto& tTag : tTags)
            {
                auto tTagIndex = &tTag - &tTags[0];
                ASSERT_STREQ(tGoldTags[tCaseIndex][tTagIndex].c_str(), tTag.c_str());
                ASSERT_STREQ("homogeneous", tRandomMatCase.attribute(tMatID, tTag).c_str());
                ASSERT_NEAR(tGoldSamples[tCaseIndex][tMatIdIndex][tTagIndex], std::stod(tRandomMatCase.value(tMatID, tTag)), tTolerance);
            }
        }
    }
    EXPECT_NEAR(1.0, tProbSum, tTolerance);
}

TEST(PlatoTest, SROM_ToString)
{
    // 1.1 MATERIAL CASE 1: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial11;
    tRandomMaterial11.blockID("1");
    tRandomMaterial11.category("isotropic");
    tRandomMaterial11.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial11.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase1;
    tRandomMatCase1.append("2", tRandomMaterial11);

    // 1.2 MATERIAL CASE 1: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial12;
    tRandomMaterial12.blockID("2");
    tRandomMaterial12.category("isotropic");
    tRandomMaterial12.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial12.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase1.append("3", tRandomMaterial12);
    tRandomMatCase1.probability(0.4875);

    // 2.1 MATERIAL CASE 2: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial21;
    tRandomMaterial21.blockID("1");
    tRandomMaterial21.category("isotropic");
    tRandomMaterial21.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial21.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase2;
    tRandomMatCase2.append("2", tRandomMaterial21);

    // 2.2 MATERIAL CASE 2: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial22;
    tRandomMaterial22.blockID("2");
    tRandomMaterial22.category("isotropic");
    tRandomMaterial22.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial22.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase2.append("3", tRandomMaterial22);
    tRandomMatCase2.probability(0.1625);

    // 3.1 MATERIAL CASE 3: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial31;
    tRandomMaterial31.blockID("1");
    tRandomMaterial31.category("isotropic");
    tRandomMaterial31.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial31.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase3;
    tRandomMatCase3.append("2", tRandomMaterial31);

    // 3.2 MATERIAL CASE 3: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial32;
    tRandomMaterial32.blockID("2");
    tRandomMaterial32.category("isotropic");
    tRandomMaterial32.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial32.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase3.append("3", tRandomMaterial32);
    tRandomMatCase3.probability(0.2625);

    // 4.1 MATERIAL CASE 4: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial41;
    tRandomMaterial41.blockID("1");
    tRandomMaterial41.category("isotropic");
    tRandomMaterial41.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial41.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase4;
    tRandomMatCase4.append("2", tRandomMaterial41);

    // 4.2 MATERIAL CASE 4: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial42;
    tRandomMaterial42.blockID("2");
    tRandomMaterial42.category("isotropic");
    tRandomMaterial42.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial42.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase4.append("3", tRandomMaterial42);
    tRandomMatCase4.probability(0.0875);

    // 5. APPEND RANDOM MATERIAL CASES
    std::vector<Plato::srom::RandomMaterialCase> tRandomMaterialSet;
    tRandomMaterialSet.push_back(tRandomMatCase1);
    tRandomMaterialSet.push_back(tRandomMatCase2);
    tRandomMaterialSet.push_back(tRandomMatCase3);
    tRandomMaterialSet.push_back(tRandomMatCase4);

    // 6. TEST FUNCTION THAT CONVERTS DOUBLE PROBABILITY VALUES INTO STRING
    const std::vector<std::string> tGoldProbs =
        { "0.4875000000000000", "0.1625000000000000", "0.2625000000000000", "0.0875000000000000" };
    for(auto& tRandomMatCase : tRandomMaterialSet)
    {
        auto tCaseIndex = &tRandomMatCase - &tRandomMaterialSet[0];
        ASSERT_STREQ(tGoldProbs[tCaseIndex].c_str(), tRandomMatCase.probabilityToString().c_str());
    }
}

TEST(PlatoTest, SROM_CheckMaterialSet)
{
    // 1.1 BUILD MATERIAL ONE
    Plato::srom::Material tMaterial1;
    tMaterial1.blockID("1");
    tMaterial1.materialID("1");
    tMaterial1.category("isotropic");
    Plato::srom::Statistics tElasticModulusStats1;
    tElasticModulusStats1.mDistribution = "beta";
    tElasticModulusStats1.mMean = "10";
    tElasticModulusStats1.mUpperBound = "20";
    tElasticModulusStats1.mLowerBound = "1";
    tElasticModulusStats1.mStandardDeviation = "3";
    tElasticModulusStats1.mNumSamples = "2";
    tMaterial1.append("youngs modulus", "homogeneous", tElasticModulusStats1);
    Plato::srom::Statistics tPoissonsRatioStats1;
    tPoissonsRatioStats1.mDistribution = "beta";
    tPoissonsRatioStats1.mMean = "0.3";
    tPoissonsRatioStats1.mUpperBound = "0.4";
    tPoissonsRatioStats1.mLowerBound = "0.25";
    tPoissonsRatioStats1.mStandardDeviation = "0.05";
    tPoissonsRatioStats1.mNumSamples = "2";
    tMaterial1.append("poissons ratio", "homogeneous", tPoissonsRatioStats1);

    // 1.2 BUILD MATERIAL TWO
    Plato::srom::Material tMaterial2;
    tMaterial2.blockID("2");
    tMaterial2.materialID("2");
    tMaterial2.category("isotropic");
    tMaterial2.append("youngs modulus", "homogeneous", "3.0");
    tMaterial2.append("poissons ratio", "homogeneous", "0.25");

    // 1.3 BUILD MATERIAL THREE
    Plato::srom::Material tMaterial3;
    tMaterial3.blockID("3");
    tMaterial3.materialID("2");
    tMaterial3.category("isotropic");
    tMaterial3.append("youngs modulus", "homogeneous", "1.0");
    Plato::srom::Statistics tPoissonsRatioStats2;
    tPoissonsRatioStats2.mDistribution = "beta";
    tPoissonsRatioStats2.mMean = "0.25";
    tPoissonsRatioStats2.mUpperBound = "0.35";
    tPoissonsRatioStats2.mLowerBound = "0.15";
    tPoissonsRatioStats2.mStandardDeviation = "0.1";
    tPoissonsRatioStats2.mNumSamples = "2";
    tMaterial3.append("poissons ratio", "homogeneous", tPoissonsRatioStats2);

    // 2. BUILD MATERIAL SET
    std::vector<Plato::srom::Material> tMaterialSet;
    tMaterialSet.push_back(tMaterial1);
    tMaterialSet.push_back(tMaterial2);
    tMaterialSet.push_back(tMaterial3);

    // 3. CALL FUNCTION - EXPECTS ERROR DUE TO DUPLICATE MATERIAL IDENTIFICATION NUMBER
    EXPECT_THROW(Plato::srom::check_material_set(tMaterialSet), std::runtime_error);
}

TEST(PlatoTest, SROM_ToString_ErrorAndDefaultValues)
{
    EXPECT_THROW(Plato::srom::to_string(std::numeric_limits<double>::infinity()), std::runtime_error);
    EXPECT_THROW(Plato::srom::to_string(std::numeric_limits<double>::quiet_NaN()), std::runtime_error);
    auto tOutput = Plato::srom::to_string(1.0, std::numeric_limits<int>::quiet_NaN());
    ASSERT_STREQ("1", tOutput.c_str());
    tOutput = Plato::srom::to_string(1.0, std::numeric_limits<int>::infinity());
    ASSERT_STREQ("1", tOutput.c_str());
}

}
// namespace UncertainMaterial
