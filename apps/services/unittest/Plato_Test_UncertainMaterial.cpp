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

#include "Plato_SromMaterialUtils.hpp"
#include <Plato_UnitTestUtils.hpp>

namespace PlatoTestUncertainMaterial
{

TEST(PlatoTest, SROM_ToLower)
{
    auto tOutput = Plato::srom::tolower("TOLOWER");
    ASSERT_STREQ("tolower", tOutput.c_str());
}

TEST(PlatoTest, SROM_WriteAndReadData)
{
    std::string tFilename("test.csv");
    std::vector<Plato::srom::DataPairs> tGoldDataSet;
    tGoldDataSet.push_back( { "Samples", std::vector<double>{} } );
    tGoldDataSet[0].second =
        {0.0566042854783908, 1.68480606445034, 1.50875023647275, 4.1493585768893, 4.97033414937478,
         2.29710545085141, 0.139247472523297, -1.10469284708504, 0.820970613576648, 6.26911462024901};
    tGoldDataSet.push_back( { "Probabilities", std::vector<double>{} } );
    tGoldDataSet[1].second =
        {0.0199137673514845, 0.0609654445695039, 0.0734112647021622, 0.372421165435386, 0.12736459057163,
         0.24272587457853, 0.0138153553915423, 0.00559607595111409, 0.0638507181006206, 0.0199357433480253};
    Plato::srom::write_data(tFilename, tGoldDataSet);

    auto tInputDataSet = Plato::srom::read_sample_probability_pairs(tFilename);
    ASSERT_EQ(2u, tInputDataSet.size());
    ASSERT_EQ(10u, tInputDataSet[0].second.size());
    ASSERT_EQ(10u, tInputDataSet[1].second.size());
    ASSERT_STREQ("samples", tInputDataSet[0].first.c_str());
    ASSERT_STREQ("probabilities", tInputDataSet[1].first.c_str());

    const double tTolerance = 1e-8;
    for(auto& tSample : tInputDataSet[0].second)
    {
        auto tIndex = &tSample - &tInputDataSet[0].second[0];
        ASSERT_NEAR(tGoldDataSet[0].second[tIndex], tSample, tTolerance);
    }

    for(auto& tProb : tInputDataSet[1].second)
    {
        auto tIndex = &tProb - &tInputDataSet[1].second[0];
        ASSERT_NEAR(tGoldDataSet[1].second[tIndex], tProb, tTolerance);
    }

    Plato::system("rm -f test.csv");
}

TEST(PlatoTest, SROM_SetRandomVariablesId)
{
    std::vector<Plato::srom::RandomVariable> tRandomVariableSet;
    tRandomVariableSet.push_back(Plato::srom::RandomVariable());
    tRandomVariableSet.push_back(Plato::srom::RandomVariable());
    tRandomVariableSet.push_back(Plato::srom::RandomVariable());
    Plato::srom::set_random_variables_id(tRandomVariableSet);
    ASSERT_EQ(0, tRandomVariableSet[0].id());
    ASSERT_EQ(1, tRandomVariableSet[1].id());
    ASSERT_EQ(2, tRandomVariableSet[2].id());
}

TEST(PlatoTest, SROM_PostProcessSampleProbabilityPairs_Error)
{
    Plato::srom::RandomVariable tRandomVar;
    tRandomVar.id(0);
    tRandomVar.tag("traction force");
    tRandomVar.attribute("magnitude");
    std::vector<Plato::srom::DataPairs> tSampleProbPairs;
    EXPECT_THROW(Plato::srom::post_process_sample_probability_pairs(tRandomVar, tSampleProbPairs), std::runtime_error);
}

TEST(PlatoTest, SROM_PostProcessSampleProbabilityPairs)
{
    // DEFINE INPUTS
    Plato::srom::RandomVariable tRandomVar;
    tRandomVar.tag("traction force");
    tRandomVar.attribute("magnitude");

    std::vector<Plato::srom::DataPairs> tSampleProbPairs;
    tSampleProbPairs.push_back( { "Samples", std::vector<double>{} } );
    tSampleProbPairs[0].second =
        {0.0566042854783908, 1.68480606445034, 1.50875023647275, 4.1493585768893, 4.97033414937478,
         2.29710545085141, 0.139247472523297, -1.10469284708504, 0.820970613576648, 6.26911462024901};
    tSampleProbPairs.push_back( { "Probabilities", std::vector<double>{} } );
    tSampleProbPairs[1].second =
        {0.0199137673514845, 0.0609654445695039, 0.0734112647021622, 0.372421165435386, 0.12736459057163,
         0.24272587457853, 0.0138153553915423, 0.00559607595111409, 0.0638507181006206, 0.0199357433480253};

    // CALL FUNCTION
    auto tSromVariable = Plato::srom::post_process_sample_probability_pairs(tRandomVar, tSampleProbPairs);

    // TEST RESULTS
    ASSERT_EQ(10, tSromVariable.mSampleProbPairs.mNumSamples);
    ASSERT_STREQ("traction force", tSromVariable.mTag.c_str());
    ASSERT_STREQ("magnitude", tSromVariable.mAttribute.c_str());

    const double tTolerance = 1e-8;
    for(auto& tSample : tSromVariable.mSampleProbPairs.mSamples)
    {
        auto tIndex = &tSample - &tSromVariable.mSampleProbPairs.mSamples[0];
        ASSERT_NEAR(tSampleProbPairs[0].second[tIndex], tSample, tTolerance);
    }

    for(auto& tProb : tSromVariable.mSampleProbPairs.mProbabilities)
    {
        auto tIndex = &tProb - &tSromVariable.mSampleProbPairs.mProbabilities[0];
        ASSERT_NEAR(tSampleProbPairs[1].second[tIndex], tProb, tTolerance);
    }
}

TEST(PlatoTest, TestProcessToPostProcessSampleProbPairsFromInputFile)
{
    // WRITE SAMPLE-PROBABILITY PAIRS TO FILE
    int tPrecision = 64;
    std::string tFilename("test.csv");
    std::vector<Plato::srom::DataPairs> tGoldDataSet;
    tGoldDataSet.push_back( { "Samples", std::vector<double>{} } );
    tGoldDataSet[0].second =
        {-18.124227441680492489695097901858389377593994140625, 15.69452170045176586654633865691721439361572265625};
    tGoldDataSet.push_back( { "Probabilities", std::vector<double>{} } );
    tGoldDataSet[1].second =
        {0.361124680672662068392497758395620621740818023681640625, 0.638872868975587149265038533485494554042816162109375};
    Plato::srom::write_data(tFilename, tGoldDataSet, tPrecision);

    // READ SAMPLE-PROBABILITY PAIRS FROM FILE
    auto tSampleProbPairsFromFile = Plato::srom::read_sample_probability_pairs(tFilename);

    // POST PROCESS SAMPLE-PROBABILITY PAIRS
    Plato::srom::RandomVariable tRandomVar;
    tRandomVar.tag("angle variation");
    tRandomVar.attribute("X");
    auto tSromVariable = Plato::srom::post_process_sample_probability_pairs(tRandomVar, tSampleProbPairsFromFile);

    // TEST RESULTS
    ASSERT_EQ(2, tSromVariable.mSampleProbPairs.mNumSamples);
    ASSERT_STREQ("angle variation", tSromVariable.mTag.c_str());
    ASSERT_STREQ("X", tSromVariable.mAttribute.c_str());

    const double tTolerance = 1e-6;
    for(auto& tSample : tSromVariable.mSampleProbPairs.mSamples)
    {
        auto tIndex = &tSample - &tSromVariable.mSampleProbPairs.mSamples[0];
        ASSERT_NEAR(tSampleProbPairsFromFile[0].second[tIndex], tSample, tTolerance);
    }

    for(auto& tProb : tSromVariable.mSampleProbPairs.mProbabilities)
    {
        auto tIndex = &tProb - &tSromVariable.mSampleProbPairs.mProbabilities[0];
        ASSERT_NEAR(tSampleProbPairsFromFile[1].second[tIndex], tProb, tTolerance);
    }

    Plato::system("rm -f test.csv");
}

TEST(PlatoTest, SROM_SolveSromProblem)
{
    // DEFINE INPUTS
    Plato::srom::RandomVariable tMyRandomVar;
    tMyRandomVar.tag("elastic modulus");
    tMyRandomVar.attribute("homogeneous");
    tMyRandomVar.distribution("beta");
    tMyRandomVar.mean("1e9");
    tMyRandomVar.upper("1e10");
    tMyRandomVar.lower("1e8");
    tMyRandomVar.deviation("2e8");
    tMyRandomVar.samples("3");

    auto tSamplerProbPairs = Plato::srom::compute_stochastic_reduced_order_model(tMyRandomVar);

    double tSum = 0;
    double tProbTolerance = 1e-4;
    double tSampleTolerance = 1e-1;
    std::vector<double> tGoldSamples = { 1311661401.549571, 660172913.150617, 657472271.01541042 };
    std::vector<double> tGoldProbabilities = { 0.4323551639529441, 0.31860804837646761, 0.24887927116109365 };
    for (int tIndex = 0; tIndex < tSamplerProbPairs.mSampleProbPairs.mNumSamples; tIndex++)
    {
        tSum += tSamplerProbPairs.mSampleProbPairs.mProbabilities[tIndex];
        EXPECT_NEAR(tGoldSamples[tIndex], tSamplerProbPairs.mSampleProbPairs.mSamples[tIndex], tSampleTolerance);
        EXPECT_NEAR(tGoldProbabilities[tIndex], tSamplerProbPairs.mSampleProbPairs.mProbabilities[tIndex], tProbTolerance);
    }
    tProbTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tProbTolerance);

    Plato::system("rm -f plato_cdf_output.txt");
    Plato::system("rm -f plato_srom_diagnostics.txt");
    Plato::system("rm -f plato_ksal_algorithm_diagnostics.txt");
}

TEST(PlatoTest, SROM_ReadSampleProbabilityPairs)
{
    // DEFINE RANDOM VARIABLE
    Plato::srom::RandomVariable tMyRandomVar;
    tMyRandomVar.id(0);
    tMyRandomVar.tag("traction force");
    tMyRandomVar.attribute("magnitude");
    tMyRandomVar.filename("test.csv");
    std::vector<Plato::srom::RandomVariable> tRandomVarsSet;
    tRandomVarsSet.push_back(tMyRandomVar);

    // CREATE CSV FILE WITH SAMPLE-PROB PAIRS
    std::string tFilename("test.csv");
    std::vector<Plato::srom::DataPairs> tGoldDataSet;
    tGoldDataSet.push_back( { "Samples", std::vector<double>{} } );
    tGoldDataSet[0].second =
        {0.0566042854783908, 1.68480606445034, 1.50875023647275, 4.1493585768893, 4.97033414937478,
         2.29710545085141, 0.139247472523297, -1.10469284708504, 0.820970613576648, 6.26911462024901};
    tGoldDataSet.push_back( { "Probabilities", std::vector<double>{} } );
    tGoldDataSet[1].second =
        {0.0199137673514845, 0.0609654445695039, 0.0734112647021622, 0.372421165435386, 0.12736459057163,
         0.24272587457853, 0.0138153553915423, 0.00559607595111409, 0.0638507181006206, 0.0199357433480253};
    Plato::srom::write_data(tFilename, tGoldDataSet);

    // COMPUTE SAMPLE-PROB PAIRS
    std::vector<Plato::srom::SromVariable> tSampleProbPairSet;
    Plato::srom::compute_sample_probability_pairs(tRandomVarsSet, tSampleProbPairSet);

    const double tTolerance = 1e-8;
    for(auto& tSampleProbPair : tSampleProbPairSet)
    {
        ASSERT_STREQ("traction force", tSampleProbPair.mTag.c_str());
        ASSERT_STREQ("magnitude", tSampleProbPair.mAttribute.c_str());

        for(auto& tSample : tSampleProbPair.mSampleProbPairs.mSamples)
        {
            auto tIndex = &tSample - &tSampleProbPair.mSampleProbPairs.mSamples[0];
            ASSERT_NEAR(tGoldDataSet[0].second[tIndex], tSample, tTolerance);
        }

        for(auto& tProb : tSampleProbPair.mSampleProbPairs.mProbabilities)
        {
            auto tIndex = &tProb - &tSampleProbPair.mSampleProbPairs.mProbabilities[0];
            ASSERT_NEAR(tGoldDataSet[1].second[tIndex], tProb, tTolerance);
        }
    }

    Plato::system("rm -f test.csv");
}

TEST(PlatoTest, SROM_ComputeSampleProbabilityPairs_HomogeneousElasticModulus_Beta)
{
    // DEFINE INPUTS
    Plato::srom::RandomVariable tMyRandomVar;
    tMyRandomVar.tag("elastic modulus");
    tMyRandomVar.attribute("homogeneous");
    tMyRandomVar.distribution("beta");
    tMyRandomVar.mean("1e9");
    tMyRandomVar.upper("1e10");
    tMyRandomVar.lower("1e8");
    tMyRandomVar.deviation("2e8");
    tMyRandomVar.samples("3");
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
    double tProbTolerance = 1e-4;
    double tSampleTolerance = 1e-1;
    const Plato::srom::SampleProbabilityPairs &tSampleProbabilityPairs = tMySampleProbPairs[0].mSampleProbPairs;
    std::vector<double> tGoldSamples = { 1311661401.549571, 660172913.150617, 657472271.01541042 };
    std::vector<double> tGoldProbabilities = { 0.4323551639529441, 0.31860804837646761, 0.24887927116109365 };
    for (int tIndex = 0; tIndex < tSampleProbabilityPairs.mNumSamples; tIndex++)
    {
        tSum += tSampleProbabilityPairs.mProbabilities[tIndex];
        EXPECT_NEAR(tGoldSamples[tIndex], tSampleProbabilityPairs.mSamples[tIndex], tSampleTolerance);
        EXPECT_NEAR(tGoldProbabilities[tIndex], tSampleProbabilityPairs.mProbabilities[tIndex], tProbTolerance);
    }
    tProbTolerance = 1e-2;
    EXPECT_NEAR(1.0, tSum, tProbTolerance);

    Plato::system("rm -f plato_cdf_output.txt");
    Plato::system("rm -f plato_srom_diagnostics.txt");
    Plato::system("rm -f plato_ksal_algorithm_diagnostics.txt");
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
    ASSERT_STREQ("1", tRandomMaterial[0].materialID().c_str());
    ASSERT_STREQ("isotropic", tRandomMaterial[0].category().c_str());

    auto tRandomVars = tRandomMaterial[0].randomVars();
    ASSERT_EQ(1u, tRandomVars.size());
    ASSERT_STREQ("elastic modulus", tRandomVars[0].tag().c_str());
    ASSERT_STREQ("homogeneous", tRandomVars[0].attribute().c_str());
    ASSERT_STREQ("1e9", tRandomVars[0].mean().c_str());
    ASSERT_STREQ("3", tRandomVars[0].samples().c_str());
    ASSERT_STREQ("1e8", tRandomVars[0].lower().c_str());
    ASSERT_STREQ("1e10", tRandomVars[0].upper().c_str());
    ASSERT_STREQ("beta", tRandomVars[0].distribution().c_str());
    ASSERT_STREQ("2e8", tRandomVars[0].deviation().c_str());

    auto tDeterministicVars = tRandomMaterial[0].deterministicVars();
    ASSERT_EQ(1u, tDeterministicVars.size());
    ASSERT_STREQ("poissons ratio", tDeterministicVars[0].tag().c_str());
    ASSERT_STREQ("homogeneous", tDeterministicVars[0].attribute().c_str());
    ASSERT_STREQ("0.3", tDeterministicVars[0].value().c_str());

    // 2) TEST DETERMINISTIC MATERIAL
    ASSERT_EQ(1u, tDeterministicMaterial.size());
    ASSERT_TRUE(tDeterministicMaterial[0].isDeterministic());
    ASSERT_FALSE(tDeterministicMaterial[0].isRandom());
    tRandomVars = tDeterministicMaterial[0].randomVars();
    ASSERT_EQ(0u, tRandomVars.size());

    tDeterministicVars = tDeterministicMaterial[0].deterministicVars();
    ASSERT_EQ(2u, tDeterministicVars.size());
    ASSERT_STREQ("poissons ratio", tDeterministicVars[0].tag().c_str());
    ASSERT_STREQ("homogeneous", tDeterministicVars[0].attribute().c_str());
    ASSERT_STREQ("0.35", tDeterministicVars[0].value().c_str());
    ASSERT_STREQ("elastic modulus", tDeterministicVars[1].tag().c_str());
    ASSERT_STREQ("homogeneous", tDeterministicVars[1].attribute().c_str());
    ASSERT_STREQ("2.5e8", tDeterministicVars[1].value().c_str());
}

TEST(PlatoTest, SROM_CheckInputSetOfMaterials_Error)
{
    // DEFINE MATERIAL ONE - RANDOM
    Plato::srom::Material tMaterialOne;
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
    tSromVariable.mSampleProbPairs.mSamples = { 0.32, 0.27 };
    tSromVariable.mSampleProbPairs.mProbabilities = { 0.75, 0.25 };
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    EXPECT_NO_THROW(Plato::srom::initialize_random_material_set(tMaterial, tSromVariable, tRandomMaterialSet));

    const double tTolerance = 1e-4;
    ASSERT_EQ(2u, tRandomMaterialSet.size());
    const std::vector<double> tGoldProbs = { 0.75, 0.25 };
    const std::vector<std::string> tGoldSamples = { "0.3200000000000000", "0.2700000000000000" };
    for (auto &tRandomMaterial : tRandomMaterialSet)
    {
        ASSERT_STREQ("2", tRandomMaterial.materialID().c_str());
        ASSERT_STREQ("isotropic", tRandomMaterial.category().c_str());

        auto tIndex = &tRandomMaterial - &tRandomMaterialSet[0];
        ASSERT_NEAR(tGoldProbs[tIndex], tRandomMaterial.probability(), tTolerance);

        auto tTags = tRandomMaterial.tags();
        ASSERT_EQ(1u, tTags.size());
        for (auto &tTag : tTags)
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
    tSromVariableOne.mSampleProbPairs.mSamples = { 0.32, 0.27 };
    tSromVariableOne.mSampleProbPairs.mProbabilities = { 0.75, 0.25 };
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    EXPECT_NO_THROW(Plato::srom::initialize_random_material_set(tMaterial, tSromVariableOne, tRandomMaterialSet));

    // 3. UPDATE RANDOM MATERIAL SET
    Plato::srom::SromVariable tSromVariableTwo;
    tSromVariableTwo.mTag = "elastic modulus";
    tSromVariableTwo.mAttribute = "homogeneous";
    tSromVariableTwo.mSampleProbPairs.mNumSamples = 2;
    tSromVariableTwo.mSampleProbPairs.mSamples = { 1e9, 2.5e9 };
    tSromVariableTwo.mSampleProbPairs.mProbabilities = { 0.45, 0.55 };
    EXPECT_NO_THROW(Plato::srom::update_random_material_set(tMaterial, tSromVariableTwo, tRandomMaterialSet));
    ASSERT_EQ(4u, tRandomMaterialSet.size());

    const std::vector<double> tGoldProbs = { 0.3375, 0.1125, 0.4125, 0.1375 };
    const std::vector<std::string> tGoldTags = { "elastic modulus", "poissons ratio" };
    const std::vector<std::vector<std::string>> tGoldSamples = { { "1000000000.0000000000000000", "0.3200000000000000" }, { "1000000000.0000000000000000",
        "0.2700000000000000" }, { "2500000000.0000000000000000", "0.3200000000000000" }, { "2500000000.0000000000000000", "0.2700000000000000" } };

    double tProbSum = 0;
    const double tTolerance = 1e-4;
    for (auto &tRandomMaterial : tRandomMaterialSet)
    {
        ASSERT_STREQ("2", tRandomMaterial.materialID().c_str());
        ASSERT_STREQ("isotropic", tRandomMaterial.category().c_str());

        auto tMatIndex = &tRandomMaterial - &tRandomMaterialSet[0];
        ASSERT_NEAR(tGoldProbs[tMatIndex], tRandomMaterial.probability(), tTolerance);
        tProbSum += tRandomMaterial.probability();

        auto tTags = tRandomMaterial.tags();
        ASSERT_EQ(2u, tTags.size());
        for (auto &tTag : tTags)
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
    tSromVariableOne.mSampleProbPairs.mSamples = { 0.32, 0.27 };
    tSromVariableOne.mSampleProbPairs.mProbabilities = { 0.75, 0.25 };

    Plato::srom::SromVariable tSromVariableTwo;
    tSromVariableTwo.mTag = "elastic modulus";
    tSromVariableTwo.mAttribute = "homogeneous";
    tSromVariableTwo.mSampleProbPairs.mNumSamples = 2;
    tSromVariableTwo.mSampleProbPairs.mSamples = { 1e9, 2.5e9 };
    tSromVariableTwo.mSampleProbPairs.mProbabilities = { 0.45, 0.55 };

    std::vector<Plato::srom::SromVariable> tSromVariableSet;
    tSromVariableSet.push_back(tSromVariableOne);
    tSromVariableSet.push_back(tSromVariableTwo);

    // 3. APPEND RANDOM MATERIAL PROPERTIES TO RANDOM MATERIAL SET
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    EXPECT_NO_THROW(Plato::srom::append_random_material_properties(tMaterial, tSromVariableSet, tRandomMaterialSet));
    ASSERT_EQ(4u, tRandomMaterialSet.size());

    const std::vector<double> tGoldProbs = { 0.3375, 0.1125, 0.4125, 0.1375 };
    const std::vector<std::string> tGoldTags = { "elastic modulus", "poissons ratio" };
    const std::vector<std::vector<std::string>> tGoldSamples = { { "1000000000.0000000000000000", "0.3200000000000000" }, { "1000000000.0000000000000000",
        "0.2700000000000000" }, { "2500000000.0000000000000000", "0.3200000000000000" }, { "2500000000.0000000000000000", "0.2700000000000000" } };

    double tProbSum = 0;
    const double tTolerance = 1e-4;
    for (auto &tRandomMaterial : tRandomMaterialSet)
    {
        ASSERT_STREQ("2", tRandomMaterial.materialID().c_str());
        ASSERT_STREQ("isotropic", tRandomMaterial.category().c_str());

        auto tMatIndex = &tRandomMaterial - &tRandomMaterialSet[0];
        ASSERT_NEAR(tGoldProbs[tMatIndex], tRandomMaterial.probability(), tTolerance);
        tProbSum += tRandomMaterial.probability();

        auto tTags = tRandomMaterial.tags();
        ASSERT_EQ(2u, tTags.size());
        for (auto &tTag : tTags)
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
    tSromVariable.mSampleProbPairs.mSamples = { 0.32, 0.27 };
    tSromVariable.mSampleProbPairs.mProbabilities = { 0.75, 0.25 };

    std::vector<Plato::srom::SromVariable> tSromVariableSet;
    tSromVariableSet.push_back(tSromVariable);

    // APPEND MATERIAL PROPERTIES
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    EXPECT_NO_THROW(Plato::srom::append_random_material_properties(tMaterial, tSromVariableSet, tRandomMaterialSet));
    ASSERT_EQ(2u, tRandomMaterialSet.size());
    EXPECT_NO_THROW(Plato::srom::append_deterministic_material_properties(tMaterial, tRandomMaterialSet));

    const double tTolerance = 1e-4;
    const std::vector<double> tGoldProbs = { 0.75, 0.25 };
    const std::vector<std::string> tGoldTags = { "elastic modulus", "poissons ratio" };
    const std::vector<std::vector<std::string>> tGoldSamples = { { "1e9", "0.3200000000000000" }, { "1e9", "0.2700000000000000" } };

    for (auto &tRandomMaterial : tRandomMaterialSet)
    {
        ASSERT_STREQ("2", tRandomMaterial.materialID().c_str());
        ASSERT_STREQ("isotropic", tRandomMaterial.category().c_str());

        auto tMatIndex = &tRandomMaterial - &tRandomMaterialSet[0];
        ASSERT_NEAR(tGoldProbs[tMatIndex], tRandomMaterial.probability(), tTolerance);

        tTags = tRandomMaterial.tags();
        ASSERT_EQ(2u, tTags.size());
        for (auto &tTag : tTags)
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
    tSromVariable.mSampleProbPairs.mSamples = { 0.32, 0.27 };
    tSromVariable.mSampleProbPairs.mProbabilities = { 0.75, 0.25 };

    std::vector<Plato::srom::SromVariable> tSromVariableSet;
    tSromVariableSet.push_back(tSromVariable);

    // BUILD RANDOM MATERIAL SET
    std::vector<Plato::srom::RandomMaterial> tRandomMaterialSet;
    EXPECT_NO_THROW(Plato::srom::build_random_material_set(tMaterial, tSromVariableSet, tRandomMaterialSet));

    const double tTolerance = 1e-4;
    const std::vector<double> tGoldProbs = { 0.75, 0.25 };
    const std::vector<std::string> tGoldTags = { "elastic modulus", "poissons ratio" };
    const std::vector<std::vector<std::string>> tGoldSamples = { { "1e9", "0.3200000000000000" }, { "1e9", "0.2700000000000000" } };

    for (auto &tRandomMaterial : tRandomMaterialSet)
    {
        ASSERT_STREQ("2", tRandomMaterial.materialID().c_str());
        ASSERT_STREQ("isotropic", tRandomMaterial.category().c_str());

        auto tMatIndex = &tRandomMaterial - &tRandomMaterialSet[0];
        ASSERT_NEAR(tGoldProbs[tMatIndex], tRandomMaterial.probability(), tTolerance);

        tTags = tRandomMaterial.tags();
        ASSERT_EQ(2u, tTags.size());
        for (auto &tTag : tTags)
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
    tSromVariable.mSampleProbPairs.mSamples = { 0.32, 0.27 };
    tSromVariable.mSampleProbPairs.mProbabilities = { 0.75, 0.25 };

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
    const std::vector<double> tGoldProbs = { 0.75, 0.25 };
    const std::vector<std::vector<std::string>> tGoldTags = { { "elastic modulus", "poissons ratio" }, { "elastic modulus", "poissons ratio" } };
    const std::vector<std::vector<std::string>> tGoldSamples = { { "1e9", "0.3200000000000000" }, { "1e9", "0.2700000000000000" } };

    for (auto &tRandomMaterialCase : tRandomMaterialCases)
    {
        ASSERT_EQ(1u, tRandomMaterialCase.numMaterials());

        auto tCaseIndex = &tRandomMaterialCase - &tRandomMaterialCases[0];
        ASSERT_NEAR(tGoldProbs[tCaseIndex], tRandomMaterialCase.probability(), tTolerance);

        auto tMaterialIDs = tRandomMaterialCase.materialIDs();
        ASSERT_EQ(1u, tMaterialIDs.size());
        for (auto &tMatID : tMaterialIDs)
        {
            ASSERT_STREQ("2", tMatID.c_str());
            ASSERT_STREQ("isotropic", tRandomMaterialCase.category(tMatID).c_str());

            tTags = tRandomMaterialCase.tags(tMatID);
            ASSERT_EQ(2u, tTags.size());
            for (auto &tTag : tTags)
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
    tSromVariableOne.mSampleProbPairs.mSamples = { 0.32, 0.27 };
    tSromVariableOne.mSampleProbPairs.mProbabilities = { 0.75, 0.25 };

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
    tSromVariableTwo.mSampleProbPairs.mSamples = { 8, 17 };
    tSromVariableTwo.mSampleProbPairs.mProbabilities = { 0.65, 0.35 };

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
    const std::vector<std::vector<std::string>> tGoldMatIDs = { { "2", "3" }, { "2", "3" }, { "2", "3" }, { "2", "3" } };
    const std::vector<std::vector<std::string>> tGoldTags = { { "elastic modulus", "poissons ratio" }, { "elastic modulus", "poissons ratio" }, {
        "elastic modulus", "poissons ratio" }, { "elastic modulus", "poissons ratio" } };
    const std::vector<std::vector<std::vector<std::string>>> tGoldSamples = { { { "1e9", "0.3200000000000000" }, { "8.0000000000000000", "0.28" } }, { { "1e9",
        "0.2700000000000000" }, { "8.0000000000000000", "0.28" } }, { { "1e9", "0.3200000000000000" }, { "17.0000000000000000", "0.28" } }, { { "1e9",
        "0.2700000000000000" }, { "17.0000000000000000", "0.28" } }, };

    for (auto &tRandomMaterialCase : tRandomMaterialCases)
    {
        ASSERT_EQ(2u, tRandomMaterialCase.numMaterials());

        auto tCaseIndex = &tRandomMaterialCase - &tRandomMaterialCases[0];
        ASSERT_NEAR(tGoldProbs[tCaseIndex], tRandomMaterialCase.probability(), tTolerance);

        auto tMaterialIDs = tRandomMaterialCase.materialIDs();
        ASSERT_EQ(2u, tMaterialIDs.size());
        for (auto &tMatID : tMaterialIDs)
        {
            auto tMatIdIndex = &tMatID - &tMaterialIDs[0];
            ASSERT_STREQ(tGoldMatIDs[tCaseIndex][tMatIdIndex].c_str(), tMatID.c_str());
            ASSERT_STREQ("isotropic", tRandomMaterialCase.category(tMatID).c_str());

            tTags = tRandomMaterialCase.tags(tMatID);
            ASSERT_EQ(2u, tTags.size());
            for (auto &tTag : tTags)
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
    tSromVariableOne.mSampleProbPairs.mSamples = { 0.32, 0.27 };
    tSromVariableOne.mSampleProbPairs.mProbabilities = { 0.75, 0.25 };

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
    tSromVariableTwo.mSampleProbPairs.mSamples = { 8, 17 };
    tSromVariableTwo.mSampleProbPairs.mProbabilities = { 0.65, 0.35 };

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
    const std::vector<std::vector<std::string>> tGoldMatIDs = { { "2", "3" }, { "2", "3" }, { "2", "3" }, { "2", "3" } };
    const std::vector<std::vector<std::string>> tGoldTags = { { "elastic modulus", "poissons ratio" }, { "elastic modulus", "poissons ratio" }, {
        "elastic modulus", "poissons ratio" }, { "elastic modulus", "poissons ratio" } };
    const std::vector<std::vector<std::vector<std::string>>> tGoldSamples = { { { "1e9", "0.3200000000000000" }, { "8.0000000000000000", "0.28" } }, { { "1e9",
        "0.2700000000000000" }, { "8.0000000000000000", "0.28" } }, { { "1e9", "0.3200000000000000" }, { "17.0000000000000000", "0.28" } }, { { "1e9",
        "0.2700000000000000" }, { "17.0000000000000000", "0.28" } }, };

    for (auto &tRandomMaterialCase : tRandomMaterialCases)
    {
        ASSERT_EQ(2u, tRandomMaterialCase.numMaterials());

        auto tCaseIndex = &tRandomMaterialCase - &tRandomMaterialCases[0];
        ASSERT_NEAR(tGoldProbs[tCaseIndex], tRandomMaterialCase.probability(), tTolerance);

        auto tMaterialIDs = tRandomMaterialCase.materialIDs();
        ASSERT_EQ(2u, tMaterialIDs.size());
        for (auto &tMatID : tMaterialIDs)
        {
            auto tMatIdIndex = &tMatID - &tMaterialIDs[0];
            ASSERT_STREQ(tGoldMatIDs[tCaseIndex][tMatIdIndex].c_str(), tMatID.c_str());
            ASSERT_STREQ("isotropic", tRandomMaterialCase.category(tMatID).c_str());

            tTags = tRandomMaterialCase.tags(tMatID);
            ASSERT_EQ(2u, tTags.size());
            for (auto &tTag : tTags)
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
    tRandomMaterial11.category("isotropic");
    tRandomMaterial11.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial11.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase1;
    tRandomMatCase1.append("2", tRandomMaterial11);

    // 1.2 MATERIAL CASE 1: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial12;
    tRandomMaterial12.category("isotropic");
    tRandomMaterial12.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial12.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase1.append("3", tRandomMaterial12);
    tRandomMatCase1.probability(0.4875);

    // 2.1 MATERIAL CASE 2: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial21;
    tRandomMaterial21.category("isotropic");
    tRandomMaterial21.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial21.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase2;
    tRandomMatCase2.append("2", tRandomMaterial21);

    // 2.2 MATERIAL CASE 2: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial22;
    tRandomMaterial22.category("isotropic");
    tRandomMaterial22.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial22.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase2.append("3", tRandomMaterial22);
    tRandomMatCase2.probability(0.1625);

    // 3.1 MATERIAL CASE 3: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial31;
    tRandomMaterial31.category("isotropic");
    tRandomMaterial31.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial31.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase3;
    tRandomMatCase3.append("2", tRandomMaterial31);

    // 3.2 MATERIAL CASE 3: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial32;
    tRandomMaterial32.category("isotropic");
    tRandomMaterial32.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial32.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase3.append("3", tRandomMaterial32);
    tRandomMatCase3.probability(0.2625);

    // 4.1 MATERIAL CASE 4: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial41;
    tRandomMaterial41.category("isotropic");
    tRandomMaterial41.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial41.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase4;
    tRandomMatCase4.append("2", tRandomMaterial41);

    // 4.2 MATERIAL CASE 4: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial42;
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
    tDeterministicMaterial.blockID("12");
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
    tRandomMaterial11.blockID("20");
    tRandomMaterial11.category("isotropic");
    tRandomMaterial11.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial11.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase1;
    tRandomMatCase1.append("2", tRandomMaterial11);

    // 1.2 MATERIAL CASE 1: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial12;
    tRandomMaterial12.blockID("30");
    tRandomMaterial12.category("isotropic");
    tRandomMaterial12.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial12.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase1.append("3", tRandomMaterial12);
    tRandomMatCase1.probability(0.4875);

    // 2.1 MATERIAL CASE 2: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial21;
    tRandomMaterial21.blockID("20");
    tRandomMaterial21.category("isotropic");
    tRandomMaterial21.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial21.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase2;
    tRandomMatCase2.append("2", tRandomMaterial21);

    // 2.2 MATERIAL CASE 2: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial22;
    tRandomMaterial22.blockID("30");
    tRandomMaterial22.category("isotropic");
    tRandomMaterial22.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial22.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase2.append("3", tRandomMaterial22);
    tRandomMatCase2.probability(0.1625);

    // 3.1 MATERIAL CASE 3: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial31;
    tRandomMaterial31.blockID("20");
    tRandomMaterial31.category("isotropic");
    tRandomMaterial31.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial31.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase3;
    tRandomMatCase3.append("2", tRandomMaterial31);

    // 3.2 MATERIAL CASE 3: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial32;
    tRandomMaterial32.blockID("30");
    tRandomMaterial32.category("isotropic");
    tRandomMaterial32.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial32.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase3.append("3", tRandomMaterial32);
    tRandomMatCase3.probability(0.2625);

    // 4.1 MATERIAL CASE 4: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial41;
    tRandomMaterial41.blockID("20");
    tRandomMaterial41.category("isotropic");
    tRandomMaterial41.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial41.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase4;
    tRandomMatCase4.append("2", tRandomMaterial41);

    // 4.2 MATERIAL CASE 4: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial42;
    tRandomMaterial42.blockID("30");
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
    const std::vector<std::vector<std::string>> tGoldBlockIDs = { { "20", "30" }, { "20", "30" }, { "20", "30" }, { "20", "30" } };
    const std::vector<std::vector<std::string>> tGoldMatIDs = { { "2", "3" }, { "2", "3" }, { "2", "3" }, { "2", "3" } };
    const std::vector<std::vector<std::string>> tGoldTags = { { "elastic modulus", "poissons ratio" }, { "elastic modulus", "poissons ratio" }, {
        "elastic modulus", "poissons ratio" }, { "elastic modulus", "poissons ratio" } };
    const std::vector<std::vector<std::vector<std::string>>> tGoldSamples = { { { "1e9", "0.32" }, { "8.0", "0.28" } },
        { { "1e9", "0.27" }, { "8.0", "0.28" } }, { { "1e9", "0.32" }, { "17.0", "0.28" } }, { { "1e9", "0.27" }, { "17.0", "0.28" } }, };

    for (auto &tRandomMaterialCase : tRandomMaterialCases)
    {
        ASSERT_EQ(2u, tRandomMaterialCase.numMaterials());

        auto tCaseIndex = &tRandomMaterialCase - &tRandomMaterialCases[0];
        ASSERT_NEAR(tGoldProbs[tCaseIndex], tRandomMaterialCase.probability(), tTolerance);

        auto tBlockIDs = tRandomMaterialCase.blockIDs();
        ASSERT_EQ(2u, tBlockIDs.size());
        auto tMaterialIDs = tRandomMaterialCase.materialIDs();
        ASSERT_EQ(2u, tMaterialIDs.size());
        for (auto &tMatID : tMaterialIDs)
        {
            auto tMatIdIndex = &tMatID - &tMaterialIDs[0];
            ASSERT_STREQ(tGoldBlockIDs[tCaseIndex][tMatIdIndex].c_str(), tBlockIDs[tMatIdIndex].c_str());
            ASSERT_STREQ(tGoldMatIDs[tCaseIndex][tMatIdIndex].c_str(), tMatID.c_str());
            ASSERT_STREQ("isotropic", tRandomMaterialCase.category(tMatID).c_str());

            auto tTags = tRandomMaterialCase.tags(tMatID);
            ASSERT_EQ(2u, tTags.size());
            for (auto &tTag : tTags)
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
    tRandomMaterial11.category("isotropic");
    tRandomMaterial11.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial11.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase1;
    tRandomMatCase1.append("2", tRandomMaterial11);

    // 1.2 MATERIAL CASE 1: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial12;
    tRandomMaterial12.category("isotropic");
    tRandomMaterial12.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial12.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase1.append("3", tRandomMaterial12);
    tRandomMatCase1.probability(0.4875);

    // 2.1 MATERIAL CASE 2: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial21;
    tRandomMaterial21.category("isotropic");
    tRandomMaterial21.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial21.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase2;
    tRandomMatCase2.append("2", tRandomMaterial21);

    // 2.2 MATERIAL CASE 2: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial22;
    tRandomMaterial22.category("isotropic");
    tRandomMaterial22.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial22.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase2.append("3", tRandomMaterial22);
    tRandomMatCase2.probability(0.1625);

    // 3.1 MATERIAL CASE 3: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial31;
    tRandomMaterial31.category("isotropic");
    tRandomMaterial31.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial31.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase3;
    tRandomMatCase3.append("2", tRandomMaterial31);

    // 3.2 MATERIAL CASE 3: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial32;
    tRandomMaterial32.category("isotropic");
    tRandomMaterial32.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial32.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase3.append("3", tRandomMaterial32);
    tRandomMatCase3.probability(0.2625);

    // 4.1 MATERIAL CASE 4: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial41;
    tRandomMaterial41.category("isotropic");
    tRandomMaterial41.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial41.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase4;
    tRandomMatCase4.append("2", tRandomMaterial41);

    // 4.2 MATERIAL CASE 4: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial42;
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
    tDeterministicMaterial.materialID("4");
    tDeterministicMaterial.category("isotropic");
    tDeterministicMaterial.append("poissons ratio", "homogeneous", "0.4");
    tDeterministicMaterial.append("elastic modulus", "homogeneous", "1.0");
    std::vector<Plato::srom::Material> tDeterministicMaterialSet;
    tDeterministicMaterialSet.push_back(tDeterministicMaterial);
    EXPECT_NO_THROW(Plato::srom::append_deterministic_materials(tDeterministicMaterialSet, tRandomMaterialCases));

    const double tTolerance = 1e-4;
    const std::vector<double> tGoldProbs = { 0.4875, 0.1625, 0.2625, 0.0875 };
    const std::vector<std::vector<std::string>> tGoldMatIDs = { { "2", "3", "4" }, { "2", "3", "4" }, { "2", "3", "4" }, { "2", "3", "4" } };
    const std::vector<std::vector<std::string>> tGoldTags = { { "elastic modulus", "poissons ratio" }, { "elastic modulus", "poissons ratio" }, {
        "elastic modulus", "poissons ratio" }, { "elastic modulus", "poissons ratio" } };
    const std::vector<std::vector<std::vector<std::string>>> tGoldSamples = { { { "1e9", "0.32" }, { "8.0", "0.28" }, { "1.0", "0.4" } }, { { "1e9", "0.27" }, {
        "8.0", "0.28" }, { "1.0", "0.4" } }, { { "1e9", "0.32" }, { "17.0", "0.28" }, { "1.0", "0.4" } }, { { "1e9", "0.27" }, { "17.0", "0.28" }, { "1.0",
        "0.4" } }, };

    for (auto &tRandomMaterialCase : tRandomMaterialCases)
    {
        ASSERT_EQ(3u, tRandomMaterialCase.numMaterials());

        auto tCaseIndex = &tRandomMaterialCase - &tRandomMaterialCases[0];
        ASSERT_NEAR(tGoldProbs[tCaseIndex], tRandomMaterialCase.probability(), tTolerance);

        auto tMaterialIDs = tRandomMaterialCase.materialIDs();
        ASSERT_EQ(3u, tMaterialIDs.size());
        for (auto &tMatID : tMaterialIDs)
        {
            auto tMatIdIndex = &tMatID - &tMaterialIDs[0];
            ASSERT_STREQ(tGoldMatIDs[tCaseIndex][tMatIdIndex].c_str(), tMatID.c_str());
            ASSERT_STREQ("isotropic", tRandomMaterialCase.category(tMatID).c_str());

            auto tTags = tRandomMaterialCase.tags(tMatID);
            ASSERT_EQ(2u, tTags.size());
            for (auto &tTag : tTags)
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
    tRandomMaterial11.category("isotropic");
    tRandomMaterial11.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial11.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase1;
    tRandomMatCase1.append("2", tRandomMaterial11);

    // 1.2 MATERIAL CASE 1: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial12;
    tRandomMaterial12.category("isotropic");
    tRandomMaterial12.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial12.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase1.append("3", tRandomMaterial12);
    tRandomMatCase1.probability(0.4875);

    // 2.1 MATERIAL CASE 2: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial21;
    tRandomMaterial21.category("isotropic");
    tRandomMaterial21.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial21.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase2;
    tRandomMatCase2.append("2", tRandomMaterial21);

    // 2.2 MATERIAL CASE 2: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial22;
    tRandomMaterial22.category("isotropic");
    tRandomMaterial22.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial22.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase2.append("3", tRandomMaterial22);
    tRandomMatCase2.probability(0.1625);

    // 3.1 MATERIAL CASE 3: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial31;
    tRandomMaterial31.category("isotropic");
    tRandomMaterial31.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial31.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase3;
    tRandomMatCase3.append("2", tRandomMaterial31);

    // 3.2 MATERIAL CASE 3: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial32;
    tRandomMaterial32.category("isotropic");
    tRandomMaterial32.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial32.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase3.append("3", tRandomMaterial32);
    tRandomMatCase3.probability(0.2625);

    // 4.1 MATERIAL CASE 4: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial41;
    tRandomMaterial41.category("isotropic");
    tRandomMaterial41.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial41.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase4;
    tRandomMatCase4.append("2", tRandomMaterial41);

    // 4.2 MATERIAL CASE 4: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial42;
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
    tDeterministicMaterial.materialID("4");
    tDeterministicMaterial.category("isotropic");
    tDeterministicMaterial.append("poissons ratio", "homogeneous", "0.4");
    tDeterministicMaterial.append("elastic modulus", "homogeneous", "1.0");
    std::vector<Plato::srom::Material> tDeterministicMaterialSet;
    tDeterministicMaterialSet.push_back(tDeterministicMaterial);
    EXPECT_NO_THROW(Plato::srom::append_deterministic_materials(tDeterministicMaterialSet, tRandomMaterialCases));
    EXPECT_NO_THROW(Plato::srom::assign_material_case_identification_number(tRandomMaterialCases));

    // 7. TEST RESULTS
    std::vector<std::string> tGoldIDs = { "1", "2", "3", "4" };
    for (auto &tMaterialCase : tRandomMaterialCases)
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
    tMaterial2.materialID("2");
    tMaterial2.category("isotropic");
    tMaterial2.append("youngs modulus", "homogeneous", "3.0");
    tMaterial2.append("poissons ratio", "homogeneous", "0.25");

    // 1.3 BUILD MATERIAL THREE
    Plato::srom::Material tMaterial3;
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
    std::vector<std::string> tGoldIDs = { "1", "2", "3", "4", "5", "6", "7", "8" };
    const std::vector<double> tGoldProbs = { 0.1482212, 0.1440202, 0.0924449, 0.0898248, 0.1641386, 0.1594865, 0.1023726, 0.0994711 };
    const std::vector<std::vector<std::string>> tGoldMatIDs = { { "1", "2", "3" }, { "1", "2", "3" }, { "1", "2", "3" }, { "1", "2", "3" }, { "1", "2", "3" }, {
        "1", "2", "3" }, { "1", "2", "3" }, { "1", "2", "3" } };
    const std::vector<std::vector<std::string>> tGoldTags = { { "poissons ratio", "youngs modulus" }, { "poissons ratio", "youngs modulus" }, {
        "poissons ratio", "youngs modulus" }, { "poissons ratio", "youngs modulus" }, { "poissons ratio", "youngs modulus" }, { "poissons ratio",
        "youngs modulus" }, { "poissons ratio", "youngs modulus" }, { "poissons ratio", "youngs modulus" } };
    const std::vector<std::vector<std::vector<double>>> tGoldSamples = { { { 0.2575729129623087, 7.6969461773057324 }, { 0.25, 3.0 },
        { 0.3448799880969564, 1.0 } }, { { 0.2575729129623087, 12.867065205497958 }, { 0.25, 3.0 }, { 0.3448799880969564, 1.0 } }, { { 0.3659845788546486,
        7.6969461773057324 }, { 0.25, 3.0 }, { 0.3448799880969564, 1.0 } }, { { 0.3659845788546486, 12.867065205497958 }, { 0.25, 3.0 }, { 0.3448799880969564,
        1.0 } }, { { 0.2575729129623087, 7.6969461773057324 }, { 0.25, 3.0 }, { 0.2121659959660376, 1.0 } }, { { 0.2575729129623087, 12.867065205497958 }, {
        0.25, 3.0 }, { 0.2121659959660376, 1.0 } }, { { 0.3659845788546486, 7.6969461773057324 }, { 0.25, 3.0 }, { 0.2121659959660376, 1.0 } }, { {
        0.3659845788546486, 12.867065205497958 }, { 0.25, 3.0 }, { 0.2121659959660376, 1.0 } } };

    double tProbSum = 0;
    for (auto &tRandomMatCase : tRandomMaterialCaseSet)
    {
        ASSERT_EQ(3u, tRandomMatCase.numMaterials());
        auto tCaseIndex = &tRandomMatCase - &tRandomMaterialCaseSet[0];
        ASSERT_STREQ(tGoldIDs[tCaseIndex].c_str(), tRandomMatCase.caseID().c_str());
        EXPECT_NEAR(tGoldProbs[tCaseIndex], tRandomMatCase.probability(), tTolerance);
        tProbSum += tRandomMatCase.probability();

        auto tMaterialIDs = tRandomMatCase.materialIDs();
        ASSERT_EQ(3u, tMaterialIDs.size());
        for (auto &tMatID : tMaterialIDs)
        {
            auto tMatIdIndex = &tMatID - &tMaterialIDs[0];
            ASSERT_STREQ(tGoldMatIDs[tCaseIndex][tMatIdIndex].c_str(), tMatID.c_str());
            ASSERT_STREQ("isotropic", tRandomMatCase.category(tMatID).c_str());

            auto tTags = tRandomMatCase.tags(tMatID);
            ASSERT_EQ(2u, tTags.size());
            for (auto &tTag : tTags)
            {
                auto tTagIndex = &tTag - &tTags[0];
                ASSERT_STREQ(tGoldTags[tCaseIndex][tTagIndex].c_str(), tTag.c_str());
                ASSERT_STREQ("homogeneous", tRandomMatCase.attribute(tMatID, tTag).c_str());
                ASSERT_NEAR(tGoldSamples[tCaseIndex][tMatIdIndex][tTagIndex], std::stod(tRandomMatCase.value(tMatID, tTag)), tTolerance);
            }
        }
    }
    EXPECT_NEAR(1.0, tProbSum, tTolerance);

    Plato::system("rm -f plato_cdf_output.txt");
    Plato::system("rm -f plato_srom_diagnostics.txt");
    Plato::system("rm -f plato_ksal_algorithm_diagnostics.txt");
}

TEST(PlatoTest, SROM_ToString)
{
    // 1.1 MATERIAL CASE 1: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial11;
    tRandomMaterial11.category("isotropic");
    tRandomMaterial11.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial11.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase1;
    tRandomMatCase1.append("2", tRandomMaterial11);

    // 1.2 MATERIAL CASE 1: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial12;
    tRandomMaterial12.category("isotropic");
    tRandomMaterial12.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial12.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase1.append("3", tRandomMaterial12);
    tRandomMatCase1.probability(0.4875);

    // 2.1 MATERIAL CASE 2: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial21;
    tRandomMaterial21.category("isotropic");
    tRandomMaterial21.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial21.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase2;
    tRandomMatCase2.append("2", tRandomMaterial21);

    // 2.2 MATERIAL CASE 2: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial22;
    tRandomMaterial22.category("isotropic");
    tRandomMaterial22.append("elastic modulus", "homogeneous", "8.0");
    tRandomMaterial22.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase2.append("3", tRandomMaterial22);
    tRandomMatCase2.probability(0.1625);

    // 3.1 MATERIAL CASE 3: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial31;
    tRandomMaterial31.category("isotropic");
    tRandomMaterial31.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial31.append("poissons ratio", "homogeneous", "0.32");
    Plato::srom::RandomMaterialCase tRandomMatCase3;
    tRandomMatCase3.append("2", tRandomMaterial31);

    // 3.2 MATERIAL CASE 3: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial32;
    tRandomMaterial32.category("isotropic");
    tRandomMaterial32.append("elastic modulus", "homogeneous", "17.0");
    tRandomMaterial32.append("poissons ratio", "homogeneous", "0.28");
    tRandomMatCase3.append("3", tRandomMaterial32);
    tRandomMatCase3.probability(0.2625);

    // 4.1 MATERIAL CASE 4: CREATE RANDOM MATERIAL 1
    Plato::srom::RandomMaterial tRandomMaterial41;
    tRandomMaterial41.category("isotropic");
    tRandomMaterial41.append("elastic modulus", "homogeneous", "1e9");
    tRandomMaterial41.append("poissons ratio", "homogeneous", "0.27");
    Plato::srom::RandomMaterialCase tRandomMatCase4;
    tRandomMatCase4.append("2", tRandomMaterial41);

    // 4.2 MATERIAL CASE 4: CREATE RANDOM MATERIAL 2
    Plato::srom::RandomMaterial tRandomMaterial42;
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
    const std::vector<std::string> tGoldProbs = { "0.4875000000000000", "0.1625000000000000", "0.2625000000000000", "0.0875000000000000" };
    for (auto &tRandomMatCase : tRandomMaterialSet)
    {
        auto tCaseIndex = &tRandomMatCase - &tRandomMaterialSet[0];
        ASSERT_STREQ(tGoldProbs[tCaseIndex].c_str(), tRandomMatCase.probabilityToString().c_str());
    }
}

TEST(PlatoTest, SROM_CheckMaterialSet)
{
    // 1.1 BUILD MATERIAL ONE
    Plato::srom::Material tMaterial1;
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
    tMaterial2.materialID("2");
    tMaterial2.category("isotropic");
    tMaterial2.append("youngs modulus", "homogeneous", "3.0");
    tMaterial2.append("poissons ratio", "homogeneous", "0.25");

    // 1.3 BUILD MATERIAL THREE
    Plato::srom::Material tMaterial3;
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
