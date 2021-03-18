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
 * Plato_Test_UncertainLoadGeneratorXML.cpp
 *
 *  Created on: Jun 1, 2019
 */

#include "gtest/gtest.h"

#include "Plato_UnitTestUtils.hpp"

#include "Plato_SromXMLGenLoad.hpp"
#include "Plato_SromLoadUtils.hpp"
#include "Plato_SromSolve.hpp"
#include "Plato_SromXML.hpp"

namespace PlatoUncertainLoadGeneratorXMLTest
{

TEST(PlatoTest, check_input_mean)
{
    Plato::srom::RandomVariable tMyRandomVar;
    ASSERT_FALSE(Plato::srom::check_input_mean(tMyRandomVar));

    tMyRandomVar.mean("5");
    ASSERT_TRUE(Plato::srom::check_input_mean(tMyRandomVar));
}

TEST(PlatoTest, check_input_lower_bound)
{
    Plato::srom::RandomVariable tMyRandomVar;
    ASSERT_FALSE(Plato::srom::check_input_lower_bound(tMyRandomVar));

    tMyRandomVar.lower("5");
    ASSERT_TRUE(Plato::srom::check_input_lower_bound(tMyRandomVar));
}

TEST(PlatoTest, check_input_upper_bound)
{
    Plato::srom::RandomVariable tMyRandomVar;
    ASSERT_FALSE(Plato::srom::check_input_upper_bound(tMyRandomVar));

    tMyRandomVar.upper("5");
    ASSERT_TRUE(Plato::srom::check_input_upper_bound(tMyRandomVar));
}

TEST(PlatoTest, check_input_standard_deviation)
{
    Plato::srom::RandomVariable tMyRandomVar;
    ASSERT_FALSE(Plato::srom::check_input_standard_deviation(tMyRandomVar));

    tMyRandomVar.deviation("5");
    ASSERT_TRUE(Plato::srom::check_input_standard_deviation(tMyRandomVar));
}

TEST(PlatoTest, check_input_number_samples)
{
    Plato::srom::RandomVariable tMyRandomVar;
    ASSERT_FALSE(Plato::srom::check_input_number_samples(tMyRandomVar));

    tMyRandomVar.samples("-1");
    ASSERT_FALSE(Plato::srom::check_input_number_samples(tMyRandomVar));

    tMyRandomVar.samples("0");
    ASSERT_FALSE(Plato::srom::check_input_number_samples(tMyRandomVar));

    tMyRandomVar.samples("2");
    ASSERT_TRUE(Plato::srom::check_input_number_samples(tMyRandomVar));
}

TEST(PlatoTest, check_input_statistics_uniform)
{
    // DEFINE INPUTS
    Plato::srom::RandomVariable tMyRandomVar;
    tMyRandomVar.tag("random rotation");
    tMyRandomVar.attribute("x");
    tMyRandomVar.distribution("uniform");

    // TEST UNDEFINED UPPER BOUND
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST UNDEFINED LOWER BOUND
    tMyRandomVar.lower("");
    tMyRandomVar.upper("135");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST UNDEFINED NUMBER OF SAMPLES
    tMyRandomVar.samples("");
    tMyRandomVar.lower("65");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST NUMBER OF SAMPLES = 0
    tMyRandomVar.samples("0");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST NEGATIVE NUMBER OF SAMPLES
    tMyRandomVar.samples("-1");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST SUCCESS
    tMyRandomVar.samples("4");
    ASSERT_NO_THROW(Plato::srom::check_input_statistics(tMyRandomVar));
}

TEST(PlatoTest, check_input_statistics_normal)
{
    // DEFINE INPUTS
    Plato::srom::RandomVariable tMyRandomVar;
    tMyRandomVar.tag("random rotation");
    tMyRandomVar.attribute("x");
    tMyRandomVar.samples("4");
    tMyRandomVar.lower("65");
    tMyRandomVar.upper("95");
    tMyRandomVar.deviation("5");
    tMyRandomVar.distribution("normal");

    // TEST UNDEFINED MEAN
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST UNDEFINED UPPER BOUND
    tMyRandomVar.mean("80");
    tMyRandomVar.upper("");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST UNDEFINED LOWER BOUND
    tMyRandomVar.lower("");
    tMyRandomVar.upper("95");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST UNDEFINED NUMBER OF SAMPLES
    tMyRandomVar.samples("");
    tMyRandomVar.lower("65");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST NUMBER OF SAMPLES = 0
    tMyRandomVar.samples("0");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST NEGATIVE NUMBER OF SAMPLES
    tMyRandomVar.samples("-1");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST UNDEFINED STANDARD DEVIATION
    tMyRandomVar.samples("4");
    tMyRandomVar.deviation("");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST SUCCESS
    tMyRandomVar.deviation("5");
    ASSERT_NO_THROW(Plato::srom::check_input_statistics(tMyRandomVar));
}

TEST(PlatoTest, check_input_statistics_beta)
{
    // DEFINE INPUTS
    Plato::srom::RandomVariable tMyRandomVar;
    tMyRandomVar.tag("random rotation");
    tMyRandomVar.attribute("x");
    tMyRandomVar.samples("4");
    tMyRandomVar.lower("65");
    tMyRandomVar.upper("95");
    tMyRandomVar.deviation("5");
    tMyRandomVar.distribution("beta");

    // TEST UNDEFINED MEAN
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST UNDEFINED UPPER BOUND
    tMyRandomVar.mean("80");
    tMyRandomVar.upper("");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST UNDEFINED LOWER BOUND
    tMyRandomVar.lower("");
    tMyRandomVar.upper("95");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST UNDEFINED NUMBER OF SAMPLES
    tMyRandomVar.samples("");
    tMyRandomVar.lower("65");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST NUMBER OF SAMPLES = 0
    tMyRandomVar.samples("0");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST NEGATIVE NUMBER OF SAMPLES
    tMyRandomVar.samples("-1");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST UNDEFINED STANDARD DEVIATION
    tMyRandomVar.samples("4");
    tMyRandomVar.deviation("");
    ASSERT_THROW(Plato::srom::check_input_statistics(tMyRandomVar), std::runtime_error);

    // TEST SUCCESS
    tMyRandomVar.deviation("5");
    ASSERT_NO_THROW(Plato::srom::check_input_statistics(tMyRandomVar));
}

TEST(PlatoTest, define_input_statistics)
{
    // DEFINE INPUTS
    Plato::srom::RandomVariable tMyRandomVar;
    tMyRandomVar.tag("random rotation");
    tMyRandomVar.attribute("x");
    tMyRandomVar.samples("4");
    tMyRandomVar.lower("65");
    tMyRandomVar.upper("95");
    tMyRandomVar.deviation("5");
    tMyRandomVar.distribution("beta");

    // TEST UNDEFINED PARAMETER
    Plato::SromInputs<double> tSromInputs;
    ASSERT_THROW(Plato::srom::define_input_statistics(tMyRandomVar, tSromInputs), std::runtime_error);

    // TEST SUCCESS
    tMyRandomVar.mean("80");
    ASSERT_NO_THROW(Plato::srom::define_input_statistics(tMyRandomVar, tSromInputs));

    ASSERT_EQ(4u, tSromInputs.mNumSamples);
    ASSERT_EQ(Plato::DistributionName::beta, tSromInputs.mDistribution);

    const double tTolerance = 1e-4;
    ASSERT_NEAR(80.0, tSromInputs.mMean, tTolerance);
    ASSERT_NEAR(25.0, tSromInputs.mVariance, tTolerance);
    ASSERT_NEAR(65.0, tSromInputs.mLowerBound, tTolerance);
    ASSERT_NEAR(95.0, tSromInputs.mUpperBound, tTolerance);
}

TEST(PlatoTest, define_random_samples_initial_guess_method)
{
    Plato::srom::RandomVariable tMyRandomVar;
    Plato::SromInputs<double> tSromInputs;

    tMyRandomVar.guess("random");
    ASSERT_NO_THROW(Plato::srom::define_random_samples_initial_guess_method(tMyRandomVar, tSromInputs));
    ASSERT_EQ(Plato::SromInitialGuess::random, tSromInputs.mInitialGuess);

    tMyRandomVar.guess("uniform");
    ASSERT_NO_THROW(Plato::srom::define_random_samples_initial_guess_method(tMyRandomVar, tSromInputs));
    ASSERT_EQ(Plato::SromInitialGuess::uniform, tSromInputs.mInitialGuess);

    tMyRandomVar.guess("product");
    ASSERT_THROW(Plato::srom::define_random_samples_initial_guess_method(tMyRandomVar, tSromInputs), std::runtime_error);
}

TEST(PlatoTest, define_distribution)
{
    Plato::srom::RandomVariable tMyRandomVar;
    Plato::SromInputs<double> tSromInputs;

    tMyRandomVar.distribution("normal");
    ASSERT_NO_THROW(Plato::srom::define_distribution(tMyRandomVar, tSromInputs));
    ASSERT_EQ(Plato::DistributionName::normal, tSromInputs.mDistribution);

    tMyRandomVar.distribution("beta");
    ASSERT_NO_THROW(Plato::srom::define_distribution(tMyRandomVar, tSromInputs));
    ASSERT_EQ(Plato::DistributionName::beta, tSromInputs.mDistribution);

    tMyRandomVar.distribution("uniform");
    ASSERT_NO_THROW(Plato::srom::define_distribution(tMyRandomVar, tSromInputs));
    ASSERT_EQ(Plato::DistributionName::uniform, tSromInputs.mDistribution);

    tMyRandomVar.distribution("lognormal");
    ASSERT_THROW(Plato::srom::define_distribution(tMyRandomVar, tSromInputs), std::runtime_error);
}

TEST(PlatoTest, compute_uniform_random_variable_statistics)
{
    Plato::SromInputs<double> tSromInputs;
    tSromInputs.mNumSamples = 4;
    tSromInputs.mLowerBound = 10.0;
    tSromInputs.mUpperBound = 20.0;
    tSromInputs.mDistribution = Plato::DistributionName::uniform;
    Plato::SromOutputs<double> tSromOutputs;
    ASSERT_NO_THROW(Plato::srom::compute_uniform_random_variable_statistics(tSromInputs, tSromOutputs));

    // TEST RESULTS
    double tSum = 0;
    size_t tRandVecDim = 0;
    double tTolerance = 1e-4;
    std::vector<double> tGoldSamples = {10, 13.333333333333, 16.666666666667, 20.0};
    std::vector<double> tGoldProbabilities(tSromInputs.mNumSamples, 0.25);
    for(size_t tIndex = 0; tIndex < tSromInputs.mNumSamples; tIndex++)
    {
        tSum += tSromOutputs.mProbabilities[tIndex];
        ASSERT_NEAR(tGoldSamples[tIndex], tSromOutputs.mSamples[tRandVecDim][tIndex], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tIndex], tSromOutputs.mProbabilities[tIndex], tTolerance);
    }
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, compute_sample_probability_pairs_error)
{
    Plato::SromInputs<double> tSromInputs;
    tSromInputs.mNumSamples = 4;
    tSromInputs.mLowerBound = 10.0;
    tSromInputs.mUpperBound = 20.0;
    tSromInputs.mDistribution = Plato::DistributionName::undefined;
    ASSERT_THROW(Plato::srom::compute_sample_probability_pairs(tSromInputs), std::runtime_error);

    Plato::system("rm -f plato_cdf_output.txt");
    Plato::system("rm -f plato_srom_diagnostics.txt");
    Plato::system("rm -f plato_ksal_algorithm_diagnostics.txt");
}

TEST(PlatoTest, random_sample_initial_guess)
{
    size_t tNumVectors = 1;
    size_t tNumSamples = 10;
    Plato::StandardMultiVector<double> tLower(tNumVectors, tNumSamples, 0.0);
    Plato::StandardMultiVector<double> tUpper(tNumVectors, tNumSamples, 1.0);
    Plato::StandardMultiVector<double> tGuess(tNumVectors, tNumSamples, 0.0);
    Plato::random_sample_initial_guess(tLower, tUpper, tGuess);
    for (decltype(tNumVectors) tDim=0; tDim < tNumVectors; tDim++)
    {
        for (decltype(tNumSamples) tSample=0; tSample < tNumSamples; tSample++)
        {
            EXPECT_TRUE(tGuess(tDim, tSample) >= 0.0);
            EXPECT_TRUE(tGuess(tDim, tSample) <= 1.0);
        }
    }
}

TEST(PlatoTest, compute_sample_probability_pairs)
{
    Plato::SromInputs<double> tSromInputs;
    tSromInputs.mNumSamples = 4;
    tSromInputs.mLowerBound = 10.0;
    tSromInputs.mUpperBound = 20.0;
    tSromInputs.mDistribution = Plato::DistributionName::uniform;
    auto tSromOutputs = Plato::srom::compute_sample_probability_pairs(tSromInputs);

    // TEST RESULTS
    double tSum = 0;
    size_t tRandVecDim = 0;
    double tTolerance = 1e-4;
    std::vector<double> tGoldSamples = {10, 13.333333333333, 16.666666666667, 20.0};
    std::vector<double> tGoldProbabilities(tSromInputs.mNumSamples, 0.25);
    for(size_t tIndex = 0; tIndex < tSromInputs.mNumSamples; tIndex++)
    {
        tSum += tSromOutputs.mProbabilities[tIndex];
        ASSERT_NEAR(tGoldSamples[tIndex], tSromOutputs.mSamples[tRandVecDim][tIndex], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tIndex], tSromOutputs.mProbabilities[tIndex], tTolerance);
    }
    ASSERT_NEAR(1.0, tSum, tTolerance);

    Plato::system("rm -f plato_cdf_output.txt");
    Plato::system("rm -f plato_srom_diagnostics.txt");
    Plato::system("rm -f plato_ksal_algorithm_diagnostics.txt");
}

TEST(PlatoTest, compute_sample_probability_pairs_OneRandVar)
{
    // DEFINE INPUTS
    Plato::srom::RandomVariable tMyRandomVar;
    tMyRandomVar.tag("random rotation");
    tMyRandomVar.attribute("x");
    tMyRandomVar.distribution("beta");
    tMyRandomVar.mean("85");
    tMyRandomVar.upper("65");
    tMyRandomVar.lower("135");
    tMyRandomVar.deviation("15");
    tMyRandomVar.samples("3");
    tMyRandomVar.guess("uniform");
    std::vector<Plato::srom::RandomVariable> tRandomVarsSet;
    tRandomVarsSet.push_back(tMyRandomVar);

    // SOLVE SROM PROBLEM
    std::vector<Plato::srom::SromVariable> tMySampleProbPairs;
    Plato::srom::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs);

    // TEST RESULTS
    ASSERT_EQ(1u, tMySampleProbPairs.size());
    ASSERT_STREQ("x", tMySampleProbPairs[0].mAttribute.c_str());
    ASSERT_STREQ("random rotation", tMySampleProbPairs[0].mTag.c_str());
    ASSERT_EQ(3, tMySampleProbPairs[0].mSampleProbPairs.mNumSamples);
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mSamples.size());
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mProbabilities.size());

    // TEST RESULTS
    double tSum = 0;
    double tTolerance = 1e-4;
    const Plato::srom::SampleProbabilityPairs& tSampleProbabilityPairs = tMySampleProbPairs[0].mSampleProbPairs;
    std::vector<double> tGoldSamples = {102.6401761033302, 88.31771931721183, 76.83726384445947};
    std::vector<double> tGoldProbabilities = {0.18178392025984, 0.19071306890276, 0.62750198867198};
    for(int tIndex = 0; tIndex < tSampleProbabilityPairs.mNumSamples; tIndex++)
    {
        tSum += tSampleProbabilityPairs.mProbabilities[tIndex];
        ASSERT_NEAR(tGoldSamples[tIndex], tSampleProbabilityPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tIndex], tSampleProbabilityPairs.mProbabilities[tIndex], tTolerance);
    }
    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);

    Plato::system("rm -f plato_cdf_output.txt");
    Plato::system("rm -f plato_srom_diagnostics.txt");
    Plato::system("rm -f plato_ksal_algorithm_diagnostics.txt");
}

TEST(PlatoTest, compute_sample_probability_pairs_TwoRandVars)
{
    std::vector<Plato::srom::RandomVariable> tRandomVarsSet;

    // DEFINE INPUTS
    Plato::srom::RandomVariable tMyRandomVarOne;
    tMyRandomVarOne.tag("random rotation");
    tMyRandomVarOne.attribute("x");
    tMyRandomVarOne.distribution("beta");
    tMyRandomVarOne.mean("85");
    tMyRandomVarOne.lower("65");
    tMyRandomVarOne.upper("135");
    tMyRandomVarOne.deviation("15");
    tMyRandomVarOne.samples("3");
    tRandomVarsSet.push_back(tMyRandomVarOne);

    Plato::srom::RandomVariable tMyRandomVarTwo;
    tMyRandomVarTwo.tag("random rotation");
    tMyRandomVarTwo.attribute("y");
    tMyRandomVarTwo.samples("4");
    tMyRandomVarTwo.distribution("beta");
    tMyRandomVarTwo.mean("70");
    tMyRandomVarTwo.lower("50");
    tMyRandomVarTwo.upper("120");
    tMyRandomVarTwo.deviation("15");
    tRandomVarsSet.push_back(tMyRandomVarTwo);

    // SOLVE SROM PROBLEM
    std::vector<Plato::srom::SromVariable> tMySampleProbPairs;
    Plato::srom::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs);

    // TEST RESULTS - RANDOM VARIABLE ONE
    ASSERT_EQ(2u, tMySampleProbPairs.size());
    ASSERT_STREQ("x", tMySampleProbPairs[0].mAttribute.c_str());
    ASSERT_STREQ("random rotation", tMySampleProbPairs[0].mTag.c_str());
    ASSERT_EQ(3, tMySampleProbPairs[0].mSampleProbPairs.mNumSamples);
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mSamples.size());
    ASSERT_EQ(3u, tMySampleProbPairs[0].mSampleProbPairs.mProbabilities.size());

    double tSum = 0;
    double tTolerance = 1e-4;
    const Plato::srom::SampleProbabilityPairs& tSampleProbabilityPairsOne = tMySampleProbPairs[0].mSampleProbPairs;
    std::vector<double> tGoldSamplesOne = {79.56461506624, 95.1780010696, 104.3742043151};
    std::vector<double> tGoldProbabilitiesOne = {0.441549282785, 0.3256625620299, 0.2326524892665};
    for(int tIndex = 0; tIndex < tSampleProbabilityPairsOne.mNumSamples; tIndex++)
    {
        tSum += tSampleProbabilityPairsOne.mProbabilities[tIndex];
        ASSERT_NEAR(tGoldSamplesOne[tIndex], tSampleProbabilityPairsOne.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tGoldProbabilitiesOne[tIndex], tSampleProbabilityPairsOne.mProbabilities[tIndex], tTolerance);
    }
    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);

    // TEST RESULTS - RANDOM VARIABLE TWO
    ASSERT_STREQ("y", tMySampleProbPairs[1].mAttribute.c_str());
    ASSERT_STREQ("random rotation", tMySampleProbPairs[1].mTag.c_str());
    ASSERT_EQ(4, tMySampleProbPairs[1].mSampleProbPairs.mNumSamples);
    ASSERT_EQ(4u, tMySampleProbPairs[1].mSampleProbPairs.mSamples.size());
    ASSERT_EQ(4u, tMySampleProbPairs[1].mSampleProbPairs.mProbabilities.size());

    tSum = 0;
    tTolerance = 1e-4;
    const Plato::srom::SampleProbabilityPairs& tSampleProbabilityPairsTwo = tMySampleProbPairs[1].mSampleProbPairs;
    std::vector<double> tGoldSamplesTwo = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    std::vector<double> tGoldProbabilitiesTwo = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    for(int tIndex = 0; tIndex < tSampleProbabilityPairsTwo.mNumSamples; tIndex++)
    {
        tSum += tSampleProbabilityPairsTwo.mProbabilities[tIndex];
        ASSERT_NEAR(tGoldSamplesTwo[tIndex], tSampleProbabilityPairsTwo.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tGoldProbabilitiesTwo[tIndex], tSampleProbabilityPairsTwo.mProbabilities[tIndex], tTolerance);
    }
    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);

    Plato::system("rm -f plato_cdf_output.txt");
    Plato::system("rm -f plato_srom_diagnostics.txt");
    Plato::system("rm -f plato_ksal_algorithm_diagnostics.txt");
}

TEST(PlatoTest, compute_sample_probability_pairs_error_undefined_distribution)
{
    // DEFINE INPUTS
    Plato::srom::RandomVariable tMyRandomVar;
    tMyRandomVar.tag("random rotation");
    tMyRandomVar.attribute("x");
    tMyRandomVar.distribution("");
    tMyRandomVar.mean("85");
    tMyRandomVar.upper("135");
    tMyRandomVar.lower("65");
    tMyRandomVar.deviation("15");
    tMyRandomVar.samples("3");
    std::vector<Plato::srom::RandomVariable> tRandomVarsSet;
    tRandomVarsSet.push_back(tMyRandomVar);

    // SOLVE SROM PROBLEM
    std::vector<Plato::srom::SromVariable> tMySampleProbPairs;
    EXPECT_THROW(Plato::srom::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs), std::runtime_error);
}

TEST(PlatoTest, compute_sample_probability_pairs_error_empty_random_var_set)
{
    // SOLVE SROM PROBLEM
    std::vector<Plato::srom::RandomVariable> tRandomVarsSet;
    std::vector<Plato::srom::SromVariable> tMySampleProbPairs;
    ASSERT_FALSE(Plato::srom::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs));
}

TEST(PlatoTest, compute_sample_probability_pairs_error_undefined_input_statistics_uniform)
{
    // DEFINE INPUTS
    Plato::srom::RandomVariable tMyRandomVar;
    tMyRandomVar.tag("random rotation");
    tMyRandomVar.attribute("x");
    tMyRandomVar.distribution("uniform");
    tMyRandomVar.lower("65");
    tMyRandomVar.upper("");
    tMyRandomVar.samples("3");
    std::vector<Plato::srom::RandomVariable> tRandomVarsSet;
    tRandomVarsSet.push_back(tMyRandomVar);

    // TEST UNDEFINED UPPER BOUND
    std::vector<Plato::srom::SromVariable> tMySampleProbPairs;
    EXPECT_THROW(Plato::srom::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs), std::runtime_error);

    // TEST UNDEFINED LOWER BOUND
    tRandomVarsSet[0].lower("");
    tRandomVarsSet[0].upper("135");
    EXPECT_THROW(Plato::srom::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs), std::runtime_error);

    // TEST UNDEFINED NUMBER OF SAMPLES
    tRandomVarsSet[0].samples("");
    tRandomVarsSet[0].lower("65");
    EXPECT_THROW(Plato::srom::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs), std::runtime_error);

    // TEST NUMBER OF SAMPLES = 0
    tRandomVarsSet[0].samples("0");
    tRandomVarsSet[0].lower("65");
    EXPECT_THROW(Plato::srom::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs), std::runtime_error);

    // TEST NEGATIVE NUMBER OF SAMPLES
    tRandomVarsSet[0].samples("-1");
    tRandomVarsSet[0].lower("65");
    EXPECT_THROW(Plato::srom::compute_sample_probability_pairs(tRandomVarsSet, tMySampleProbPairs), std::runtime_error);
}

TEST(PlatoTest, post_process_random_load_error)
{
    // ERROR: ZERO INPUTS PROVIDED
    std::vector<Plato::srom::SromVariable> tSromRandomVariableSet;
    Plato::srom::SampleProbabilityPairs tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs;
    ASSERT_FALSE(Plato::srom::expand_load_sample_probability_pair(tSromRandomVariableSet, tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs));
}

TEST(PlatoTest, post_process_random_load_OneRandomRotation)
{
    Plato::srom::SromVariable tRandomLoadX;
    tRandomLoadX.mTag = "random rotation";
    tRandomLoadX.mAttribute = "x";
    tRandomLoadX.mSampleProbPairs.mNumSamples = 4;
    tRandomLoadX.mSampleProbPairs.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tRandomLoadX.mSampleProbPairs.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    std::vector<Plato::srom::SromVariable> tSromRandomVariableSet;
    tSromRandomVariableSet.push_back(tRandomLoadX);

    Plato::srom::SampleProbabilityPairs tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs;
    ASSERT_TRUE(Plato::srom::expand_load_sample_probability_pair(tSromRandomVariableSet, tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs));

    // TEST RESULTS
    ASSERT_TRUE(tMyYaxisSampleProbPairs.mSamples.empty());
    ASSERT_TRUE(tMyZaxisSampleProbPairs.mSamples.empty());
    ASSERT_FALSE(tMyXaxisSampleProbPairs.mSamples.empty());

    ASSERT_EQ(4, tMyXaxisSampleProbPairs.mNumSamples);

    const double tTolerance = 1e-4;
    for(int tIndex = 0; tIndex < tMyXaxisSampleProbPairs.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadX.mSampleProbPairs.mSamples[tIndex], tMyXaxisSampleProbPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadX.mSampleProbPairs.mProbabilities[tIndex], tMyXaxisSampleProbPairs.mProbabilities[tIndex], tTolerance);
    }
}

TEST(PlatoTest, post_process_random_load_TwoRandomRotations)
{
    Plato::srom::SromVariable tRandomLoadX;
    tRandomLoadX.mTag = "random rotation";
    tRandomLoadX.mAttribute = "x";
    tRandomLoadX.mSampleProbPairs.mNumSamples = 4;
    tRandomLoadX.mSampleProbPairs.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tRandomLoadX.mSampleProbPairs.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    Plato::srom::SromVariable tRandomLoadY;
    tRandomLoadY.mTag = "random rotation";
    tRandomLoadY.mAttribute = "y";
    tRandomLoadY.mSampleProbPairs.mNumSamples = 3;
    tRandomLoadY.mSampleProbPairs.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tRandomLoadY.mSampleProbPairs.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    std::vector<Plato::srom::SromVariable> tSromRandomVariableSet;
    tSromRandomVariableSet.push_back(tRandomLoadX);
    tSromRandomVariableSet.push_back(tRandomLoadY);

    Plato::srom::SampleProbabilityPairs tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs;
    ASSERT_TRUE(Plato::srom::expand_load_sample_probability_pair(tSromRandomVariableSet, tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs));

    // TEST RESULTS
    ASSERT_TRUE(tMyZaxisSampleProbPairs.mSamples.empty());
    ASSERT_FALSE(tMyYaxisSampleProbPairs.mSamples.empty());
    ASSERT_FALSE(tMyXaxisSampleProbPairs.mSamples.empty());

    // TEST RESULTS FOR ROTATION X SAMPLE-PROBABILITY PAIRS
    const double tTolerance = 1e-4;

    ASSERT_EQ(tRandomLoadX.mSampleProbPairs.mNumSamples, tMyXaxisSampleProbPairs.mNumSamples);
    for(int tIndex = 0; tIndex < tMyXaxisSampleProbPairs.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadX.mSampleProbPairs.mSamples[tIndex], tMyXaxisSampleProbPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadX.mSampleProbPairs.mProbabilities[tIndex], tMyXaxisSampleProbPairs.mProbabilities[tIndex], tTolerance);
    }

    // TEST RESULTS FOR ROTATION Y SAMPLE-PROBABILITY PAIRS
    ASSERT_EQ(tRandomLoadY.mSampleProbPairs.mNumSamples, tMyYaxisSampleProbPairs.mNumSamples);
    for(int tIndex = 0; tIndex < tMyYaxisSampleProbPairs.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadY.mSampleProbPairs.mSamples[tIndex], tMyYaxisSampleProbPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadY.mSampleProbPairs.mProbabilities[tIndex], tMyYaxisSampleProbPairs.mProbabilities[tIndex], tTolerance);
    }
}

TEST(PlatoTest, post_process_random_load_ThreeRandomRotations)
{
    Plato::srom::SromVariable tRandomLoadX;
    tRandomLoadX.mTag = "random rotation";
    tRandomLoadX.mAttribute = "x";
    tRandomLoadX.mSampleProbPairs.mNumSamples = 4;
    tRandomLoadX.mSampleProbPairs.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tRandomLoadX.mSampleProbPairs.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    Plato::srom::SromVariable tRandomLoadY;
    tRandomLoadY.mTag = "random rotation";
    tRandomLoadY.mAttribute = "y";
    tRandomLoadY.mSampleProbPairs.mNumSamples = 3;
    tRandomLoadY.mSampleProbPairs.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tRandomLoadY.mSampleProbPairs.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    Plato::srom::SromVariable tRandomLoadZ;
    tRandomLoadZ.mTag = "random rotation";
    tRandomLoadZ.mAttribute = "z";
    tRandomLoadZ.mSampleProbPairs.mNumSamples = 4;
    tRandomLoadZ.mSampleProbPairs.mSamples = {10, 13.333333333333, 16.666666666667, 20.0};
    tRandomLoadZ.mSampleProbPairs.mProbabilities = {0.25, 0.25, 0.25, 0.25};

    std::vector<Plato::srom::SromVariable> tSromRandomVariableSet;
    tSromRandomVariableSet.push_back(tRandomLoadX);
    tSromRandomVariableSet.push_back(tRandomLoadY);
    tSromRandomVariableSet.push_back(tRandomLoadZ);

    Plato::srom::SampleProbabilityPairs tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs;
    ASSERT_TRUE(Plato::srom::expand_load_sample_probability_pair(tSromRandomVariableSet, tMyXaxisSampleProbPairs, tMyYaxisSampleProbPairs, tMyZaxisSampleProbPairs));

    // TEST RESULTS
    ASSERT_FALSE(tMyZaxisSampleProbPairs.mSamples.empty());
    ASSERT_FALSE(tMyYaxisSampleProbPairs.mSamples.empty());
    ASSERT_FALSE(tMyXaxisSampleProbPairs.mSamples.empty());

    // TEST RESULTS FOR ROTATION X SAMPLE-PROBABILITY PAIRS
    const double tTolerance = 1e-4;

    ASSERT_EQ(tRandomLoadX.mSampleProbPairs.mNumSamples, tMyXaxisSampleProbPairs.mNumSamples);
    for(int tIndex = 0; tIndex < tMyXaxisSampleProbPairs.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadX.mSampleProbPairs.mSamples[tIndex], tMyXaxisSampleProbPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadX.mSampleProbPairs.mProbabilities[tIndex], tMyXaxisSampleProbPairs.mProbabilities[tIndex], tTolerance);
    }

    // TEST RESULTS FOR ROTATION Y SAMPLE-PROBABILITY PAIRS
    ASSERT_EQ(tRandomLoadY.mSampleProbPairs.mNumSamples, tMyYaxisSampleProbPairs.mNumSamples);
    for(int tIndex = 0; tIndex < tMyYaxisSampleProbPairs.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadY.mSampleProbPairs.mSamples[tIndex], tMyYaxisSampleProbPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadY.mSampleProbPairs.mProbabilities[tIndex], tMyYaxisSampleProbPairs.mProbabilities[tIndex], tTolerance);
    }

    // TEST RESULTS FOR ROTATION Z SAMPLE-PROBABILITY PAIRS
    ASSERT_EQ(tRandomLoadZ.mSampleProbPairs.mNumSamples, tMyZaxisSampleProbPairs.mNumSamples);
    for(int tIndex = 0; tIndex < tMyZaxisSampleProbPairs.mNumSamples; tIndex++)
    {
        ASSERT_NEAR(tRandomLoadZ.mSampleProbPairs.mSamples[tIndex], tMyZaxisSampleProbPairs.mSamples[tIndex], tTolerance);
        ASSERT_NEAR(tRandomLoadZ.mSampleProbPairs.mProbabilities[tIndex], tMyZaxisSampleProbPairs.mProbabilities[tIndex], tTolerance);
    }
}

TEST(PlatoTest, expand_random_rotations_about_xyz)
{
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    tSampleProbPairSetY.mNumSamples = 3;
    tSampleProbPairSetY.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetY.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 4;
    tSampleProbPairSetZ.mProbabilities = {0.25, 0.25, 0.25, 0.25};
    tSampleProbPairSetZ.mSamples = {10, 13.333333333333, 16.666666666667, 20.0};

    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST NUMBER OF ROTATION VECTORS
    const size_t tNumRotationVectors = tSampleProbPairSetX.mNumSamples * tSampleProbPairSetY.mNumSamples * tSampleProbPairSetZ.mNumSamples;
    ASSERT_EQ(tNumRotationVectors, tRandomRotationSet.size()); // expects 48 rotation vectors
    std::vector<std::vector<double>> tSetGoldRotations =
        { {62.92995363352, 79.56461506624, 10}, {62.92995363352, 79.56461506624, 13.333333333333}, {62.92995363352, 79.56461506624, 16.666666666667}, {62.92995363352, 79.56461506624, 20},
          {62.92995363352,  95.1780010696, 10}, {62.92995363352,  95.1780010696, 13.333333333333}, {62.92995363352,  95.1780010696, 16.666666666667}, {62.92995363352,  95.1780010696, 20},
          {62.92995363352, 104.3742043151, 10}, {62.92995363352, 104.3742043151, 13.333333333333}, {62.92995363352, 104.3742043151, 16.666666666667}, {62.92995363352, 104.3742043151, 20},
          {69.67128118964, 79.56461506624, 10}, {69.67128118964, 79.56461506624, 13.333333333333}, {69.67128118964, 79.56461506624, 16.666666666667}, {69.67128118964, 79.56461506624, 20},
          {69.67128118964,  95.1780010696, 10}, {69.67128118964,  95.1780010696, 13.333333333333}, {69.67128118964,  95.1780010696, 16.666666666667}, {69.67128118964,  95.1780010696, 20},
          {69.67128118964, 104.3742043151, 10}, {69.67128118964, 104.3742043151, 13.333333333333}, {69.67128118964, 104.3742043151, 16.666666666667}, {69.67128118964, 104.3742043151, 20},
          {66.03455388567, 79.56461506624, 10}, {66.03455388567, 79.56461506624, 13.333333333333}, {66.03455388567, 79.56461506624, 16.666666666667}, {66.03455388567, 79.56461506624, 20},
          {66.03455388567,  95.1780010696, 10}, {66.03455388567,  95.1780010696, 13.333333333333}, {66.03455388567,  95.1780010696, 16.666666666667}, {66.03455388567,  95.1780010696, 20},
          {66.03455388567, 104.3742043151, 10}, {66.03455388567, 104.3742043151, 13.333333333333}, {66.03455388567, 104.3742043151, 16.666666666667}, {66.03455388567, 104.3742043151, 20},
          { 96.2527627689, 79.56461506624, 10}, { 96.2527627689, 79.56461506624, 13.333333333333}, { 96.2527627689, 79.56461506624, 16.666666666667}, { 96.2527627689, 79.56461506624, 20},
          { 96.2527627689,  95.1780010696, 10}, { 96.2527627689,  95.1780010696, 13.333333333333}, { 96.2527627689,  95.1780010696, 16.666666666667}, { 96.2527627689,  95.1780010696, 20},
          { 96.2527627689, 104.3742043151, 10}, { 96.2527627689, 104.3742043151, 13.333333333333}, { 96.2527627689, 104.3742043151, 16.666666666667}, { 96.2527627689, 104.3742043151, 20} };

    std::vector<double> tGoldProbabilities = { 0.040214307576243, 0.040214307576243, 0.040214307576243, 0.040214307576243,
                                               0.029659870248084, 0.029659870248084, 0.029659870248084, 0.029659870248084,
                                               0.021188934342120, 0.021188934342120, 0.021188934342120, 0.021188934342120,
                                               0.021685307558066, 0.021685307558066, 0.021685307558066, 0.021685307558066,
                                               0.015993894890335, 0.015993894890335, 0.015993894890335, 0.015993894890335,
                                               0.011425997007791, 0.011425997007791, 0.011425997007791, 0.011425997007791,
                                               0.025396048040355, 0.025396048040355, 0.025396048040355, 0.025396048040355,
                                               0.018730733788290, 0.018730733788290, 0.018730733788290, 0.018730733788290,
                                               0.013381187614786, 0.013381187614786, 0.013381187614786, 0.013381187614786,
                                               0.023089690523626, 0.023089690523626, 0.023089690523626, 0.023089690523626,
                                               0.017029690830826, 0.017029690830826, 0.017029690830826, 0.017029690830826,
                                               0.012165966939936, 0.012165966939936, 0.012165966939936, 0.012165966939936};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations[0], tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations[1], tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_about_xy)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    tSampleProbPairSetY.mNumSamples = 3;
    tSampleProbPairSetY.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetY.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST OUTPUTS
    ASSERT_EQ(12u, tRandomRotationSet.size());
    std::vector<std::vector<double>> tSetGoldRotations =
        { {62.92995363352, 79.56461506624, 0}, {62.92995363352,  95.1780010696, 0}, {62.92995363352, 104.3742043151, 0},
          {69.67128118964, 79.56461506624, 0}, {69.67128118964,  95.1780010696, 0}, {69.67128118964, 104.3742043151, 0},
          {66.03455388567, 79.56461506624, 0}, {66.03455388567,  95.1780010696, 0}, {66.03455388567, 104.3742043151, 0},
          {96.25276276890, 79.56461506624, 0}, {96.25276276890,  95.1780010696, 0}, {96.25276276890, 104.3742043151, 0}};

    std::vector<double> tGoldProbabilities = { 0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                               0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                               0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                               0.0923587620945064, 0.0681187633233061, 0.0486638677597447 };

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations[0], tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations[1], tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_about_xz)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST OUTPUTS
    ASSERT_EQ(12u, tRandomRotationSet.size());
    std::vector<std::vector<double>> tSetGoldRotations =
        { {62.92995363352, 0, 79.56461506624}, {62.92995363352, 0, 95.1780010696}, {62.92995363352, 0, 104.3742043151},
          {69.67128118964, 0, 79.56461506624}, {69.67128118964, 0, 95.1780010696}, {69.67128118964, 0, 104.3742043151},
          {66.03455388567, 0, 79.56461506624}, {66.03455388567, 0, 95.1780010696}, {66.03455388567, 0, 104.3742043151},
          {96.25276276890, 0, 79.56461506624}, {96.25276276890, 0, 95.1780010696}, {96.25276276890, 0, 104.3742043151}};

    std::vector<double> tGoldProbabilities = { 0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                               0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                               0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                               0.0923587620945064, 0.0681187633233061, 0.0486638677597447 };

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations[0], tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations[1], tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_about_yz)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST OUTPUTS
    ASSERT_EQ(12u, tRandomRotationSet.size());
    std::vector<std::vector<double>> tSetGoldRotations =
        { {0, 62.92995363352, 79.56461506624}, {0, 62.92995363352, 95.1780010696}, {0, 62.92995363352, 104.3742043151},
          {0, 69.67128118964, 79.56461506624}, {0, 69.67128118964, 95.1780010696}, {0, 69.67128118964, 104.3742043151},
          {0, 66.03455388567, 79.56461506624}, {0, 66.03455388567, 95.1780010696}, {0, 66.03455388567, 104.3742043151},
          {0, 96.25276276890, 79.56461506624}, {0, 96.25276276890, 95.1780010696}, {0, 96.25276276890, 104.3742043151}};

    std::vector<double> tGoldProbabilities = { 0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                               0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                               0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                               0.0923587620945064, 0.0681187633233061, 0.0486638677597447 };

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations[0], tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations[1], tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_about_x)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST OUTPUTS
    ASSERT_EQ(4u, tRandomRotationSet.size());
    std::vector<std::vector<double>> tSetGoldRotations =
        { {62.92995363352, 0, 0}, {69.67128118964, 0, 0}, {66.03455388567, 0, 0}, {96.25276276890, 0, 0} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations[0], tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations[1], tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_about_y)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST OUTPUTS
    ASSERT_EQ(4u, tRandomRotationSet.size());
    std::vector<std::vector<double>> tSetGoldRotations =
        { {0, 62.92995363352, 0}, {0, 69.67128118964, 0}, {0, 66.03455388567, 0}, {0, 96.25276276890, 0} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations[0], tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations[1], tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_about_z)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 4;
    tSampleProbPairSetZ.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetZ.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));

    // TEST OUTPUTS
    ASSERT_EQ(4u, tRandomRotationSet.size());
    std::vector<std::vector<double>> tSetGoldRotations =
        { {0, 0, 62.92995363352}, {0, 0, 69.67128118964}, {0, 0, 66.03455388567}, {0, 0, 96.25276276890} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tSetGoldRotations.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        const std::vector<double>& tGoldRotations = tSetGoldRotations[tSampleIndex];
        ASSERT_NEAR(tGoldRotations[0], tRandomRotationSet[tSampleIndex].mRotations[0], tTolerance);
        ASSERT_NEAR(tGoldRotations[1], tRandomRotationSet[tSampleIndex].mRotations[1], tTolerance);
        ASSERT_NEAR(tGoldRotations[2], tRandomRotationSet[tSampleIndex].mRotations[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tRandomRotationSet[tSampleIndex].mProbability, tTolerance);
        tSum += tRandomRotationSet[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_rotations_error)
{
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    std::vector<Plato::srom::RandomRotations> tRandomRotationSet;
    ASSERT_FALSE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tRandomRotationSet));
}

TEST(PlatoTest, expand_random_loads_about_z)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetZ.mNumSamples = 4;
    tSampleProbPairSetZ.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetZ.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    std::vector<double> tMyOriginalLoad(3,1.0);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(4u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
        { {-0.43537138598505315, 1.345530288126378, 1}, {-0.5903092061838121, 1.2851206328958531, 1},
          {-0.50760495686136231, 1.319976214850015, 1}, {-1.1029658951090628, 0.8851362800305180, 1} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues[0], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues[1], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_y)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    std::vector<double> tMyOriginalLoad(3, 1.0);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(4u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
        { {1.34553028812638, 1, -0.435371385985053}, {1.28512063289585, 1, -0.590309206183812},
          {1.31997621485002, 1, -0.507604956861362}, {0.88513628003051, 1, -1.10296589510906} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues[0], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues[1], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_x)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    std::vector<double> tMyOriginalLoad(3, 1.0);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(4u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
        { {1, -0.435371385985053, 1.34553028812638}, {1, -0.590309206183812, 1.28512063289585},
          {1, -0.507604956861362, 1.31997621485002}, {1, -1.10296589510906, 0.885136280030518} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues[0], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues[1], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_yz)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    std::vector<double> tMyOriginalLoad(3, 1.0);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(12u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
    { {-0.739748540735107, 1.50440149320111, -0.435371385985053}, {-1.11735350148433, 1.24978914581067, -0.435371385985053}, {-1.30272801733653, 1.05515471335339, -0.435371385985053},
      {-0.750690333076780, 1.44499102589591, -0.590309206183812}, {-1.11190151802027, 1.18962601489664, -0.590309206183812}, {-1.28773109166515, 0.99663618068657, -0.590309206183812},
      {-0.744377061825103, 1.47927008946926, -0.507604956861362}, {-1.11504724126135, 1.22433935554046, -0.507604956861362}, {-1.29638412187704, 1.03040060962466, -0.507604956861362},
      {-0.823138118323529, 1.05162249518973, -1.102965895109060}, {-1.07580285019335, 0.79127394860574, -1.102965895109060}, {-1.18843346139817, 0.60917332677608, -1.102965895109060} };
    std::vector<double> tGoldProbabilities = {0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                              0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                              0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                              0.0923587620945064, 0.0681187633233061, 0.0486638677597447};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues[0], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues[1], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_xz)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    std::vector<double> tMyOriginalLoad(3, 1.0);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(12u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
    { {0.609296803530850, 0.904602481171501, 1.34553028812638}, {0.343344489354037, 1.03521147857081, 1.34553028812638}, {0.173488310131751, 1.07677762327334, 1.34553028812638},
      {0.761671920821811, 0.876539128582960, 1.28512063289585}, {0.497650027509830, 1.04919464782510, 1.28512063289585}, {0.323575806212112, 1.11524152385909, 1.28512063289585},
      {0.680335616523720, 0.891519063800385, 1.31997621485002}, {0.415283283979423, 1.04173057278621, 1.31997621485002}, {0.243460611096492, 1.09470988077881, 1.31997621485002},
      {1.265849164342930, 0.783683392006009, 0.88513628003052}, {1.008214628020530, 1.09546201651137, 0.88513628003052}, {0.820183791860380, 1.24251048822264, 0.88513628003052} };
    std::vector<double> tGoldProbabilities = {0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                              0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                              0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                              0.0923587620945064, 0.0681187633233061, 0.0486638677597447};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues[0], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues[1], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_xy)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetY.mNumSamples = 3;
    tSampleProbPairSetY.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetY.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    std::vector<double> tMyOriginalLoad(3, 1.0);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(12u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
    { {1.50440149320111, -0.435371385985053, -0.739748540735107}, {1.24978914581067, -0.435371385985053, -1.11735350148433}, {1.05515471335339, -0.435371385985053, -1.30272801733653},
      {1.44499102589591, -0.590309206183812, -0.750690333076780}, {1.18962601489664, -0.590309206183812, -1.11190151802027}, {0.99663618068657, -0.590309206183812, -1.28773109166515},
      {1.47927008946926, -0.507604956861362, -0.744377061825103}, {1.22433935554046, -0.507604956861362, -1.11504724126135}, {1.03040060962466, -0.507604956861362, -1.29638412187704},
      {1.05162249518973, -1.10296589510906,  -0.823138118323529}, {0.791273948605747, -1.10296589510906, -1.07580285019335}, {0.609173326776081,-1.10296589510906,  -1.18843346139817} };
    std::vector<double> tGoldProbabilities = {0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                              0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                              0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                              0.0923587620945064, 0.0681187633233061, 0.0486638677597447};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues[0], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues[1], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_about_xyz)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetY.mNumSamples = 3;
    tSampleProbPairSetY.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetY.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};
    tSampleProbPairSetZ.mNumSamples = 4;
    tSampleProbPairSetZ.mProbabilities = {0.25, 0.25, 0.25, 0.25};
    tSampleProbPairSetZ.mSamples = {10, 13.333333333333, 16.666666666667, 20.0};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION - FUNCTION BEING TESTED
    std::vector<double> tMyOriginalLoad(3, 1.0);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_TRUE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // TEST OUTPUTS
    ASSERT_EQ(48u, tMyRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
    { {1.55714770193223, -0.167520538583969, -0.739748540735107}, {1.56425370752726, -0.0766970336291921, -0.739748540735107}, {1.56606677372885, 0.0143859898095396, -0.739748540735107},
      {1.56258076569489, 0.105420335612506, -0.739748540735107}, {1.30640348820948, -0.211733508719819, -1.11735350148433}, {1.31650446891269, -0.135414681824415, -1.11735350148433},
      {1.32215081522277, -0.0586376545102008, -1.11735350148433}, {1.32332342169111, 0.0183377840655412, -1.11735350148433}, {1.11472599012243, -0.245531423227262, -1.30272801733653},
      {1.12711643277193, -0.180300470942008, -1.30272801733653}, {1.13569307019323, -0.114459438935836, -1.30272801733653}, {1.14042688176016, -0.0482311124196051, -1.30272801733653},
      {1.52554448324919, -0.330421024532408, -0.750690333076780}, {1.54217577737325, -0.241159481481218, -0.750690333076780}, {1.55358883683092, -0.151081931081903, -0.750690333076780},
      {1.55974504344169, -0.0604931672516919, -0.750690333076780}, {1.27405904056906, -0.374764693332327, -1.11190151802027}, {1.29369416329485, -0.300050705849966, -1.11190151802027},
      {1.30895183448128, -0.224321441757799, -1.11190151802027}, {1.31978042699877, -0.147833144914024, -1.11190151802027}, {1.08400115558644, -0.408277026351148, -1.28773109166515},
      {1.10590639504269, -0.344557224510759, -1.28773109166515}, {1.12406959732454, -0.279671550089624, -1.28773109166515}, {1.13842930390487, -0.213839555670947, -1.28773109166515},
      {1.54494132864207, -0.243020741670963, -0.744377061825103}, {1.55645793187998, -0.152779239808649, -0.744377061825103}, {1.56270797416156, -0.0620207813599240, -0.744377061825103},
      {1.56367030732212, 0.0289475357806807, -0.744377061825103}, {1.29388356538789, -0.287288999048993, -1.11504724126135}, {1.30839888887741, -0.211570312992648, -1.11504724126135},
      {1.31848700469088, -0.135135739968118, -1.11504724126135}, {1.32411377783769, -0.0582439103757674, -1.11504724126135}, {1.10289118480055, -0.320966108856280, -1.29638412187704},
      {1.11968778695736, -0.256295665752715, -1.29638412187704}, {1.13269572007193, -0.190757999244703, -1.29638412187704}, {1.14187096941614, -0.124574868050287, -1.29638412187704},
      {1.22717400421935, -0.903597034928209, -0.823138118323529}, {1.27763731503050, -0.830714469239849, -0.823138118323529}, {1.32377750559417, -0.755021028744024, -0.823138118323529},
      {1.36543845211078, -0.676772836083224, -0.823138118323529}, {0.970780737057986, -0.948806085600751, -1.07580285019335}, {1.02430649721405, -0.890754976006840, -1.07580285019335},
      {1.07436632842014, -0.829689833535944, -1.07580285019335}, {1.12079084405347, -0.765817283300505, -1.07580285019335}, {0.791446632851788, -0.980427526733487, -1.18843346139817},
      {0.847114421213299, -0.932750269472827, -1.18843346139817}, {0.899915842521188, -0.881916880555393, -1.18843346139817}, {0.949672233479613, -0.828099364078381, -1.18843346139817} };

    std::vector<double> tGoldProbabilities = { 0.040214307576243, 0.040214307576243, 0.040214307576243, 0.040214307576243,
                                               0.029659870248084, 0.029659870248084, 0.029659870248084, 0.029659870248084,
                                               0.021188934342120, 0.021188934342120, 0.021188934342120, 0.021188934342120,
                                               0.021685307558066, 0.021685307558066, 0.021685307558066, 0.021685307558066,
                                               0.015993894890335, 0.015993894890335, 0.015993894890335, 0.015993894890335,
                                               0.011425997007791, 0.011425997007791, 0.011425997007791, 0.011425997007791,
                                               0.025396048040355, 0.025396048040355, 0.025396048040355, 0.025396048040355,
                                               0.018730733788290, 0.018730733788290, 0.018730733788290, 0.018730733788290,
                                               0.013381187614786, 0.013381187614786, 0.013381187614786, 0.013381187614786,
                                               0.023089690523626, 0.023089690523626, 0.023089690523626, 0.023089690523626,
                                               0.017029690830826, 0.017029690830826, 0.017029690830826, 0.017029690830826,
                                               0.012165966939936, 0.012165966939936, 0.012165966939936, 0.012165966939936};

    double tSum = 0;
    double tTolerance = 1e-4;
    const size_t tTotalNumSamples = tMyRandomLoads.size();
    for(size_t tSampleIndex = 0; tSampleIndex < tTotalNumSamples; tSampleIndex++)
    {
        ASSERT_NEAR(tGoldLoads[tSampleIndex][0], tMyRandomLoads[tSampleIndex].mLoadValues[0], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][1], tMyRandomLoads[tSampleIndex].mLoadValues[1], tTolerance);
        ASSERT_NEAR(tGoldLoads[tSampleIndex][2], tMyRandomLoads[tSampleIndex].mLoadValues[2], tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tSampleIndex], tMyRandomLoads[tSampleIndex].mProbability, tTolerance);
        tSum += tMyRandomLoads[tSampleIndex].mProbability;
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_loads_error_1)
{
    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED SINCE SET OF RANDOM ROTATION IS EMPTY
    std::vector<double> tMyOriginalLoad(3, 1.0);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_FALSE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));
}

TEST(PlatoTest, expand_random_loads_error_2)
{
    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED DUE TO NaN X-COMPONENT
    std::vector<double> tMyOriginalLoad;
    tMyOriginalLoad.push_back(std::numeric_limits<double>::quiet_NaN());
    tMyOriginalLoad.push_back(1);
    tMyOriginalLoad.push_back(1);
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    Plato::srom::RandomRotations tMyRotation;
    tMyRotation.mRotations.resize(3, 0.0);
    tMyRotation.mRotations[0] = 0;
    tMyRotation.mRotations[1] = 0;
    tMyRotation.mRotations[2] = 62.92995363352;
    tMyRandomRotationsSet.push_back(tMyRotation);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    ASSERT_FALSE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED DUE TO NaN Y-COMPONENT
    tMyOriginalLoad[0] = 1; tMyOriginalLoad[1] = std::numeric_limits<double>::quiet_NaN();
    ASSERT_FALSE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED DUE TO NaN Z-COMPONENT
    tMyOriginalLoad[1] = 1; tMyOriginalLoad[2] = std::numeric_limits<double>::quiet_NaN();
    ASSERT_FALSE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));
}

TEST(PlatoTest, expand_random_loads_error_3)
{
    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED DUE TO NaN X-COMPONENT
    std::vector<double> tMyOriginalLoad(3, 1.0);
    std::vector<Plato::srom::RandomLoad> tMyRandomLoads;
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    Plato::srom::RandomRotations tMyRotation;
    tMyRotation.mRotations.resize(3, 0.0);
    tMyRotation.mRotations[0] = std::numeric_limits<double>::quiet_NaN(); tMyRotation.mRotations[1] = 0; tMyRotation.mRotations[2] = 62.92995363352;
    tMyRandomRotationsSet.push_back(tMyRotation);
    ASSERT_FALSE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED DUE TO NaN Y-COMPONENT
    tMyRandomRotationsSet[0].mRotations[0] = 0; tMyRandomRotationsSet[0].mRotations[1] = std::numeric_limits<double>::quiet_NaN();
    ASSERT_FALSE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));

    // CALL EXPAND RANDOM LOADS FUNCTION - FAILURE IS EXPECTED DUE TO NaN Z-COMPONENT
    tMyRandomRotationsSet[0].mRotations[1] = 0; tMyRandomRotationsSet[0].mRotations[2] = std::numeric_limits<double>::quiet_NaN();
    ASSERT_FALSE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tMyRandomLoads));
}

TEST(PlatoTest, expand_random_load_cases_error)
{
    // CALL EXPAND RANDOM LOAD CASES FUNCTION - FAILURE IS EXPECTED DUE TO EMPTY ARRAY OF NEW RANDOM LOADS
    std::vector<Plato::srom::RandomLoad> tNewSetRandomLoads;
    std::vector<Plato::srom::RandomLoadCase> tOldRandomLoadCases;
    ASSERT_FALSE(Plato::srom::expand_random_load_cases(tNewSetRandomLoads, tOldRandomLoadCases));
}

TEST(PlatoTest, expand_random_load_cases_one_load)
{
    // SET INPUTS
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    std::vector<double> tMyOriginalLoad(3, 1.0);
    std::vector<Plato::srom::RandomLoad> tNewSetRandomLoads;
    ASSERT_TRUE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION - FUNCTION BEING TESTED
    std::vector<Plato::srom::RandomLoadCase> tRandomLoadCases;
    ASSERT_TRUE(Plato::srom::expand_random_load_cases(tNewSetRandomLoads, tRandomLoadCases));

    std::vector<std::vector<double>> tGoldLoads =
    { {0.609296803530850, 0.904602481171501, 1.34553028812638}, {0.343344489354037, 1.03521147857081, 1.34553028812638}, {0.173488310131751, 1.07677762327334, 1.34553028812638},
      {0.761671920821811, 0.876539128582960, 1.28512063289585}, {0.497650027509830, 1.04919464782510, 1.28512063289585}, {0.323575806212112, 1.11524152385909, 1.28512063289585},
      {0.680335616523720, 0.891519063800385, 1.31997621485002}, {0.415283283979423, 1.04173057278621, 1.31997621485002}, {0.243460611096492, 1.09470988077881, 1.31997621485002},
      {1.265849164342930, 0.783683392006009, 0.88513628003052}, {1.008214628020530, 1.09546201651137, 0.88513628003052}, {0.820183791860380, 1.24251048822264, 0.88513628003052} };
    std::vector<double> tGoldProbabilities = {0.160857230304970, 0.118639480992335, 0.0847557373684797,
                                              0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
                                              0.101584192161421, 0.0749229351531628, 0.0535247504591439,
                                              0.0923587620945064, 0.0681187633233061, 0.0486638677597447};

    double tSum = 0;
    double tTolerance = 1e-4;
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tRandomLoadCases.size(); tLoadCaseIndex++)
    {
        const Plato::srom::RandomLoadCase& tRandomLoadCase = tRandomLoadCases[tLoadCaseIndex];
        ASSERT_NEAR(tGoldLoads[tLoadCaseIndex][0], tRandomLoadCase.loadValue(0,0), tTolerance);
        ASSERT_NEAR(tGoldLoads[tLoadCaseIndex][1], tRandomLoadCase.loadValue(0,1), tTolerance);
        ASSERT_NEAR(tGoldLoads[tLoadCaseIndex][2], tRandomLoadCase.loadValue(0,2), tTolerance);
        ASSERT_NEAR(tGoldProbabilities[tLoadCaseIndex], tRandomLoadCase.loadProbability(0), tTolerance);
        tSum += tRandomLoadCase.probability();
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_load_cases_two_load)
{
    // *** SET FIRST LOAD CASE ***
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    std::vector<double> tMyOriginalLoad(3, 1.0);
    std::vector<Plato::srom::RandomLoad> tNewSetRandomLoads;
    ASSERT_TRUE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION - FUNCTION BEING TESTED
    std::vector<Plato::srom::RandomLoadCase> tRandomLoadCases;
    ASSERT_TRUE(Plato::srom::expand_random_load_cases(tNewSetRandomLoads, tRandomLoadCases));

    // *** SET SECOND LOAD CASE ***
    tSampleProbPairSetX.mNumSamples = 0;
    tSampleProbPairSetX.mSamples.clear();
    tSampleProbPairSetX.mProbabilities.clear();
    tSampleProbPairSetZ.mNumSamples = 0;
    tSampleProbPairSetZ.mSamples.clear();
    tSampleProbPairSetZ.mProbabilities.clear();
    tSampleProbPairSetY.mNumSamples = 4;
    tSampleProbPairSetY.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetY.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    // COMPUTE RANDOM ROTATIONS SET
    tMyRandomRotationsSet.clear();
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    tNewSetRandomLoads.clear();
    ASSERT_TRUE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION - FUNCTION BEING TESTED
    ASSERT_TRUE(Plato::srom::expand_random_load_cases(tNewSetRandomLoads, tRandomLoadCases));

    std::vector<std::vector<std::vector<double>>> tGoldLoadCases =
      { { {0.609296803530850, 0.904602481171501, 1.34553028812638}, {1.34553028812638, 1, -0.435371385985053} }, { {0.343344489354037, 1.03521147857081, 1.34553028812638}, {1.34553028812638, 1, -0.435371385985053} },
        { {0.173488310131751, 1.07677762327334, 1.34553028812638}, {1.34553028812638, 1, -0.435371385985053} }, { {0.761671920821811, 0.876539128582960, 1.28512063289585}, {1.34553028812638, 1, -0.435371385985053} },
        { {0.497650027509830, 1.04919464782510, 1.28512063289585}, {1.34553028812638, 1, -0.435371385985053} }, { {0.323575806212112, 1.11524152385909, 1.28512063289585}, {1.34553028812638, 1, -0.435371385985053} },
        { {0.680335616523720, 0.891519063800385, 1.31997621485002}, {1.34553028812638, 1, -0.435371385985053} }, { {0.415283283979423, 1.04173057278621, 1.31997621485002}, {1.34553028812638, 1, -0.435371385985053} },
        { {0.243460611096492, 1.09470988077881, 1.31997621485002}, {1.34553028812638, 1, -0.435371385985053} }, { {1.265849164342930, 0.783683392006009, 0.88513628003052}, {1.34553028812638, 1, -0.435371385985053} },
        { {1.008214628020530, 1.09546201651137, 0.88513628003052}, {1.34553028812638, 1, -0.435371385985053} }, { {0.820183791860380, 1.24251048822264, 0.88513628003052}, {1.34553028812638, 1, -0.435371385985053} },

        { {0.609296803530850, 0.904602481171501, 1.34553028812638}, {1.28512063289585, 1, -0.590309206183812} }, { {0.343344489354037, 1.03521147857081, 1.34553028812638}, {1.28512063289585, 1, -0.590309206183812} },
        { {0.173488310131751, 1.07677762327334, 1.34553028812638}, {1.28512063289585, 1, -0.590309206183812} }, { {0.761671920821811, 0.876539128582960, 1.28512063289585}, {1.28512063289585, 1, -0.590309206183812} },
        { {0.497650027509830, 1.04919464782510, 1.28512063289585}, {1.28512063289585, 1, -0.590309206183812} }, { {0.323575806212112, 1.11524152385909, 1.28512063289585}, {1.28512063289585, 1, -0.590309206183812} },
        { {0.680335616523720, 0.891519063800385, 1.31997621485002}, {1.28512063289585, 1, -0.590309206183812} }, { {0.415283283979423, 1.04173057278621, 1.31997621485002}, {1.28512063289585, 1, -0.590309206183812} },
        { {0.243460611096492, 1.09470988077881, 1.31997621485002}, {1.28512063289585, 1, -0.590309206183812} }, { {1.265849164342930, 0.783683392006009, 0.88513628003052}, {1.28512063289585, 1, -0.590309206183812} },
        { {1.008214628020530, 1.09546201651137, 0.88513628003052}, {1.28512063289585, 1, -0.590309206183812} }, { {0.820183791860380, 1.24251048822264, 0.88513628003052}, {1.28512063289585, 1, -0.590309206183812} },

        { {0.609296803530850, 0.904602481171501, 1.34553028812638}, {1.31997621485002, 1, -0.507604956861362} }, { {0.343344489354037, 1.03521147857081, 1.34553028812638}, {1.31997621485002, 1, -0.507604956861362} },
        { {0.173488310131751, 1.07677762327334, 1.34553028812638}, {1.31997621485002, 1, -0.507604956861362} }, { {0.761671920821811, 0.876539128582960, 1.2851206328958}, {1.31997621485002, 1, -0.507604956861362} },
        { {0.497650027509830, 1.04919464782510, 1.28512063289585}, {1.31997621485002, 1, -0.507604956861362} }, { {0.323575806212112, 1.11524152385909, 1.28512063289585},  {1.31997621485002, 1, -0.507604956861362} },
        { {0.680335616523720, 0.891519063800385, 1.31997621485002}, {1.31997621485002, 1, -0.507604956861362} }, { {0.415283283979423, 1.04173057278621, 1.31997621485002}, {1.31997621485002, 1, -0.507604956861362} },
        { {0.243460611096492, 1.09470988077881, 1.31997621485002}, {1.31997621485002, 1, -0.507604956861362} }, { {1.265849164342930, 0.783683392006009, 0.8851362800305}, {1.31997621485002, 1, -0.507604956861362} },
        { {1.008214628020530, 1.09546201651137, 0.88513628003052}, {1.31997621485002, 1, -0.507604956861362} }, { {0.820183791860380, 1.24251048822264, 0.88513628003052},  {1.31997621485002, 1, -0.507604956861362} },

        { {0.609296803530850, 0.904602481171501, 1.34553028812638}, {0.88513628003051, 1, -1.10296589510906} }, { {0.343344489354037, 1.03521147857081, 1.34553028812638}, {0.88513628003051, 1, -1.10296589510906} },
        { {0.173488310131751, 1.07677762327334, 1.34553028812638}, {0.88513628003051, 1, -1.10296589510906} }, {{0.761671920821811, 0.876539128582960, 1.28512063289585}, {0.88513628003051, 1, -1.10296589510906} },
        { {0.497650027509830, 1.04919464782510, 1.28512063289585}, {0.88513628003051, 1, -1.10296589510906} }, { {0.323575806212112, 1.11524152385909, 1.28512063289585}, {0.88513628003051, 1, -1.10296589510906} },
        { {0.680335616523720, 0.891519063800385, 1.31997621485002}, {0.88513628003051, 1, -1.10296589510906} }, {  {0.415283283979423, 1.04173057278621, 1.31997621485002}, {0.88513628003051, 1, -1.10296589510906} },
        { {0.243460611096492, 1.09470988077881, 1.31997621485002}, {0.88513628003051, 1, -1.10296589510906} }, { {1.265849164342930, 0.783683392006009, 0.88513628003052}, {0.88513628003051, 1, -1.10296589510906} },
        { {1.008214628020530, 1.09546201651137, 0.88513628003052}, {0.88513628003051, 1, -1.10296589510906} }, { {0.820183791860380, 1.24251048822264, 0.88513628003052}, {0.88513628003051, 1, -1.10296589510906} } };

    std::vector<double> tGoldProbabilities = { 0.0586005901270716,  0.0432205850202652, 0.0308766737872556, 0.0315999925544024, 0.0233064233973704, 0.0166500483982540 ,
                                               0.0370073113714254,  0.0272945655330732, 0.0194991667913430, 0.0336464699279151, 0.0248157929979522, 0.0177283381243119,

                                               0.0315999925544023,  0.0233064233973704, 0.0166500483982540, 0.0170400934064482, 0.0125678394062182, 0.00897843186006081,
                                               0.0199559554137536,  0.0147184194860575, 0.0105148006886510, 0.0181436432107338, 0.0133817572871952, 0.00955989268223052,

                                               0.0370073113714254,  0.0272945655330731, 0.0194991667913430, 0.0199559554137537, 0.0147184194860575, 0.0105148006886510 ,
                                               0.0233707730924189,  0.0172370019353027, 0.0123140694550314, 0.0212483421493127, 0.0156716131426772, 0.0111957597635392,

                                               0.0336464699279150,  0.0248157929979521, 0.0177283381243119, 0.0181436432107338, 0.0133817572871952, 0.00955989268223053,
                                               0.0212483421493127,  0.0156716131426772, 0.0111957597635392, 0.0193186610604985, 0.0142483860833551, 0.0101790100454297 };

    double tSum = 0;
    double tTolerance = 1e-4;
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tRandomLoadCases.size(); tLoadCaseIndex++)
    {
        const Plato::srom::RandomLoadCase& tRandomLoadCase = tRandomLoadCases[tLoadCaseIndex];
        ASSERT_NEAR(tGoldProbabilities[tLoadCaseIndex], tRandomLoadCase.probability(), tTolerance);
        for(size_t tLoadIndex = 0; tLoadIndex < tRandomLoadCase.numLoads(); tLoadIndex++)
        {
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][0], tRandomLoadCase.loadValue(tLoadIndex,0), tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][1], tRandomLoadCase.loadValue(tLoadIndex,1), tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][2], tRandomLoadCase.loadValue(tLoadIndex,2), tTolerance);
        }
        tSum += tRandomLoadCase.probability();
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);
}

TEST(PlatoTest, expand_random_and_deterministic_loads_error)
{
    // FUNCTION FAILED DUE TO EMPTY SET OF LOADS
    std::vector<Plato::srom::Load> tLoads;
    std::vector<Plato::srom::Load> tRandomLoads;
    std::vector<Plato::srom::Load> tDeterministicLoads;
    ASSERT_FALSE(Plato::srom::expand_random_and_deterministic_loads(tLoads, tRandomLoads, tDeterministicLoads));
}

TEST(PlatoTest, expand_random_and_deterministic_loads_only_random_loads)
{
    // INPUTS
    std::vector<Plato::srom::Load> tLoads;
    Plato::srom::Load tLoad1;
    tLoad1.mAppID = 1;
    tLoad1.mAppType = "nodeset";
    tLoad1.mLoadID = 1;
    tLoad1.mLoadType = "traction";
    tLoad1.mValues = {"1", "2", "3"};
    Plato::srom::RandomVariable tRandVar1;
    tRandVar1.tag("random rotation");
    tRandVar1.attribute("x");
    tRandVar1.mean("60");
    tRandVar1.samples("2");
    tRandVar1.lower("40");
    tRandVar1.upper("100");
    tRandVar1.distribution("beta");
    tRandVar1.deviation("10");
    Plato::srom::RandomVariable tRandVar2;
    tRandVar2.tag("random rotation");
    tRandVar2.attribute("y");
    tRandVar2.mean("65");
    tRandVar2.samples("2");
    tRandVar2.lower("40");
    tRandVar2.upper("100");
    tRandVar2.distribution("beta");
    tRandVar2.deviation("10");
    tLoad1.mRandomVars.push_back(tRandVar2);
    tLoads.push_back(tLoad1);

    Plato::srom::Load tLoad2;
    tLoad2.mAppID = 2;
    tLoad2.mAppType = "nodeset";
    tLoad2.mLoadID = 2;
    tLoad2.mLoadType = "traction";
    tLoad2.mValues = {"1", "2", "3"};
    tRandVar1.tag("random rotation");
    tRandVar1.attribute("x");
    tRandVar1.mean("70");
    tRandVar1.samples("2");
    tRandVar1.lower("45");
    tRandVar1.upper("110");
    tRandVar1.distribution("beta");
    tRandVar1.deviation("15");

    tRandVar2.tag("random rotation");
    tRandVar2.attribute("z");
    tRandVar2.mean("75");
    tRandVar2.samples("2");
    tRandVar2.lower("50");
    tRandVar2.upper("120");
    tRandVar2.distribution("beta");
    tRandVar2.deviation("12");
    tLoad2.mRandomVars.push_back(tRandVar2);
    tLoads.push_back(tLoad2);

    // CALL FUNCTION TO BE TESTED
    std::vector<Plato::srom::Load> tRandomLoads;
    std::vector<Plato::srom::Load> tDeterministicLoads;
    ASSERT_TRUE(Plato::srom::expand_random_and_deterministic_loads(tLoads, tRandomLoads, tDeterministicLoads));

    // TEST OUTPUT
    ASSERT_FALSE(tRandomLoads.empty());
    ASSERT_TRUE(tDeterministicLoads.empty());

    ASSERT_EQ(2u, tRandomLoads.size());
    for(size_t tLoadIndex = 0; tLoadIndex < tRandomLoads.size(); tLoadIndex++)
    {
        ASSERT_EQ(tLoads[tLoadIndex].mAppID, tRandomLoads[tLoadIndex].mAppID);
        ASSERT_EQ(tLoads[tLoadIndex].mLoadID, tRandomLoads[tLoadIndex].mLoadID);
        ASSERT_STREQ(tLoads[tLoadIndex].mAppType.c_str(), tRandomLoads[tLoadIndex].mAppType.c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mLoadType.c_str(), tRandomLoads[tLoadIndex].mLoadType.c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mValues[0].c_str(), tRandomLoads[tLoadIndex].mValues[0].c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mValues[1].c_str(), tRandomLoads[tLoadIndex].mValues[1].c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mValues[2].c_str(), tRandomLoads[tLoadIndex].mValues[2].c_str());

        for(size_t tVarIndex = 0; tVarIndex < tRandomLoads[tLoadIndex].mRandomVars.size(); tVarIndex++)
        {
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].tag().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].tag().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].attribute().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].attribute().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mean().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mean().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].samples().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].samples().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].lower().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].lower().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].upper().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].upper().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].distribution().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].distribution().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].deviation().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].deviation().c_str());
        }
    }
}

TEST(PlatoTest, expand_random_and_deterministic_loads)
{
    // RANDOM INPUTS
    std::vector<Plato::srom::Load> tLoads;
    Plato::srom::Load tLoad1;
    tLoad1.mAppID = 1;
    tLoad1.mAppType = "nodeset";
    tLoad1.mLoadID = 1;
    tLoad1.mLoadType = "traction";
    tLoad1.mValues = {"1", "2", "3"};
    Plato::srom::RandomVariable tRandVar1;
    tRandVar1.tag("random rotation");
    tRandVar1.attribute("x");
    tRandVar1.mean("60");
    tRandVar1.samples("2");
    tRandVar1.lower("40");
    tRandVar1.upper("100");
    tRandVar1.distribution("beta");
    tRandVar1.deviation("10");
    Plato::srom::RandomVariable tRandVar2;
    tRandVar2.tag("random rotation");
    tRandVar2.attribute("y");
    tRandVar2.mean("65");
    tRandVar2.samples("2");
    tRandVar2.lower("40");
    tRandVar2.upper("100");
    tRandVar2.distribution("beta");
    tRandVar2.deviation("10");
    tLoad1.mRandomVars.push_back(tRandVar2);
    tLoads.push_back(tLoad1);

    Plato::srom::Load tLoad2;
    tLoad2.mAppID = 2;
    tLoad2.mAppType = "nodeset";
    tLoad2.mLoadID = 2;
    tLoad2.mLoadType = "traction";
    tLoad2.mValues = {"1", "2", "3"};
    tRandVar1.tag("random rotation");
    tRandVar1.attribute("x");
    tRandVar1.mean("70");
    tRandVar1.samples("2");
    tRandVar1.lower("45");
    tRandVar1.upper("110");
    tRandVar1.distribution("beta");
    tRandVar1.deviation("15");

    tRandVar2.tag("random rotation");
    tRandVar2.attribute("z");
    tRandVar2.mean("75");
    tRandVar2.samples("2");
    tRandVar2.lower("50");
    tRandVar2.upper("120");
    tRandVar2.distribution("beta");
    tRandVar2.deviation("12");
    tLoad2.mRandomVars.push_back(tRandVar2);
    tLoads.push_back(tLoad2);

    // DETERMINISTIC INPUTS
    Plato::srom::Load tLoad3;
    tLoad3.mAppID = 3;
    tLoad3.mAppType = "SIDESET";
    tLoad3.mLoadID = 3;
    tLoad3.mLoadType = "pressure";
    tLoad3.mValues = {"1", "2", "3"};
    tLoads.push_back(tLoad3);

    // CALL FUNCTION TO BE TESTED
    std::vector<Plato::srom::Load> tRandomLoads;
    std::vector<Plato::srom::Load> tDeterministicLoads;
    ASSERT_TRUE(Plato::srom::expand_random_and_deterministic_loads(tLoads, tRandomLoads, tDeterministicLoads));

    // TEST OUTPUT
    ASSERT_FALSE(tRandomLoads.empty());
    ASSERT_FALSE(tDeterministicLoads.empty());

    // TEST RANDOM LOADS
    ASSERT_EQ(2u, tRandomLoads.size());
    for(size_t tLoadIndex = 0; tLoadIndex < tRandomLoads.size(); tLoadIndex++)
    {
        ASSERT_EQ(tLoads[tLoadIndex].mAppID, tRandomLoads[tLoadIndex].mAppID);
        ASSERT_EQ(tLoads[tLoadIndex].mLoadID, tRandomLoads[tLoadIndex].mLoadID);
        ASSERT_STREQ(tLoads[tLoadIndex].mAppType.c_str(), tRandomLoads[tLoadIndex].mAppType.c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mLoadType.c_str(), tRandomLoads[tLoadIndex].mLoadType.c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mValues[0].c_str(), tRandomLoads[tLoadIndex].mValues[0].c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mValues[1].c_str(), tRandomLoads[tLoadIndex].mValues[1].c_str());
        ASSERT_STREQ(tLoads[tLoadIndex].mValues[2].c_str(), tRandomLoads[tLoadIndex].mValues[2].c_str());

        for(size_t tVarIndex = 0; tVarIndex < tRandomLoads[tLoadIndex].mRandomVars.size(); tVarIndex++)
        {
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].tag().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].tag().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].attribute().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].attribute().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].mean().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].mean().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].samples().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].samples().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].lower().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].lower().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].upper().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].upper().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].distribution().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].distribution().c_str());
            ASSERT_STREQ(tLoads[tLoadIndex].mRandomVars[tVarIndex].deviation().c_str(), tRandomLoads[tLoadIndex].mRandomVars[tVarIndex].deviation().c_str());
        }
    }

    // TEST DETERMINISTIC LOADS
    ASSERT_EQ(1u, tDeterministicLoads.size());
    ASSERT_TRUE(tDeterministicLoads[0].mRandomVars.empty());
    ASSERT_EQ(tLoads[2].mAppID, tDeterministicLoads[0].mAppID);
    ASSERT_EQ(tLoads[2].mLoadID, tDeterministicLoads[0].mLoadID);
    ASSERT_STREQ(tLoads[2].mAppType.c_str(), tDeterministicLoads[0].mAppType.c_str());
    ASSERT_STREQ(tLoads[2].mLoadType.c_str(), tDeterministicLoads[0].mLoadType.c_str());
    ASSERT_STREQ(tLoads[2].mValues[0].c_str(), tDeterministicLoads[0].mValues[0].c_str());
    ASSERT_STREQ(tLoads[2].mValues[1].c_str(), tDeterministicLoads[0].mValues[1].c_str());
    ASSERT_STREQ(tLoads[2].mValues[2].c_str(), tDeterministicLoads[0].mValues[2].c_str());
}

TEST(PlatoTest, set_load_components_errors)
{
    // TEST ERROR: EMPTY INPUT ARRAY
    std::vector<double> tFloatLoad(3, 0.0);
    std::vector<std::string> tStringLoad;
    ASSERT_FALSE(Plato::srom::set_load_components(tStringLoad, tFloatLoad));

    // TEST ERROR: INPUT ARRAY SIZE IS NOT EQUAL TO 3
    tStringLoad = {"1", "2"};
    ASSERT_FALSE(Plato::srom::set_load_components(tStringLoad, tFloatLoad));
}

TEST(PlatoTest, set_load_components)
{
    std::vector<double> tFloatLoad;
    std::vector<std::string> tStringLoad = {"1", "2", "3"};
    ASSERT_TRUE(Plato::srom::set_load_components(tStringLoad, tFloatLoad));

    // TEST OUTPUT
    ASSERT_EQ(1.0, tFloatLoad[0]);
    ASSERT_EQ(2.0, tFloatLoad[1]);
    ASSERT_EQ(3.0, tFloatLoad[2]);
}

TEST(PlatoTest, generate_set_random_rotations_errors)
{
    // TEST ERROR: EMPTY SET OF SAMPLE-PROBABILITY PAIRS
    std::vector<Plato::srom::SromVariable> tMySampleProbPairs;
    std::vector<Plato::srom::RandomRotations> tMySetRandomRotation;
    ASSERT_FALSE(Plato::srom::generate_set_random_rotations(tMySampleProbPairs, tMySetRandomRotation));
}

TEST(PlatoTest, generate_set_random_rotations)
{
    // SET INPUTS
    Plato::srom::SromVariable tRandVar;
    tRandVar.mAttribute = "x";
    tRandVar.mTag = "random rotation";
    tRandVar.mSampleProbPairs.mNumSamples = 2;
    tRandVar.mSampleProbPairs.mSamples = {1, 2};
    tRandVar.mSampleProbPairs.mProbabilities = {0.5, 0.5};
    std::vector<Plato::srom::SromVariable> tMySampleProbPairs;
    tMySampleProbPairs.push_back(tRandVar);

    // CALL FUNCTION
    std::vector<Plato::srom::RandomRotations> tMySetRandomRotation;
    ASSERT_TRUE(Plato::srom::generate_set_random_rotations(tMySampleProbPairs, tMySetRandomRotation));

    // TEST OUTPUT
    ASSERT_FALSE(tMySetRandomRotation.empty());
    ASSERT_EQ(0.5, tMySetRandomRotation[0].mProbability);
    ASSERT_EQ(1.0, tMySetRandomRotation[0].mRotations[0]);
    ASSERT_EQ(0.0, tMySetRandomRotation[0].mRotations[1]);
    ASSERT_EQ(0.0, tMySetRandomRotation[0].mRotations[2]);

    ASSERT_EQ(0.5, tMySetRandomRotation[1].mProbability);
    ASSERT_EQ(2.0, tMySetRandomRotation[1].mRotations[0]);
    ASSERT_EQ(0.0, tMySetRandomRotation[1].mRotations[1]);
    ASSERT_EQ(0.0, tMySetRandomRotation[1].mRotations[2]);
}

TEST(PlatoTest, check_load_parameters)
{
    // FAILED: APPLICATION ID IS NOT DEFINED
    Plato::srom::Load tLoad;
    ASSERT_FALSE(Plato::srom::check_load_parameters(tLoad));

    // FAILED: APPLICATION TYPE IS NOT DEFINED
    tLoad.mAppID = 1;
    ASSERT_FALSE(Plato::srom::check_load_parameters(tLoad));

    // FAILED: LOAD TYPE IS NOT DEFINED
    tLoad.mAppType = "sideset";
    ASSERT_FALSE(Plato::srom::check_load_parameters(tLoad));

    // FAILED: LOAD VALUES/COMPONENTS ARE NOT DEFINED
    tLoad.mLoadType = "traction";
    ASSERT_FALSE(Plato::srom::check_load_parameters(tLoad));

    // PASS
    tLoad.mValues = {"1", "2", "3"};
    ASSERT_TRUE(Plato::srom::check_load_parameters(tLoad));
}

TEST(PlatoTest, check_deterministic_loads)
{
    // FAILED: APPLICATION ID IS NOT DEFINED
    Plato::srom::Load tLoad;
    std::vector<Plato::srom::Load> tDeterministicLoads;
    tDeterministicLoads.push_back(tLoad);
    ASSERT_FALSE(Plato::srom::check_deterministic_loads(tDeterministicLoads));

    // FAILED: APPLICATION TYPE IS NOT DEFINED
    tDeterministicLoads[0].mAppID = 1;
    ASSERT_FALSE(Plato::srom::check_deterministic_loads(tDeterministicLoads));

    // FAILED: LOAD TYPE IS NOT DEFINED
    tDeterministicLoads[0].mAppType = "sideset";
    ASSERT_FALSE(Plato::srom::check_deterministic_loads(tDeterministicLoads));

    // FAILED: LOAD VALUES/COMPONENTS ARE NOT DEFINED
    tDeterministicLoads[0].mLoadType = "traction";
    ASSERT_FALSE(Plato::srom::check_deterministic_loads(tDeterministicLoads));

    // PASS
    tDeterministicLoads[0].mValues = {"1", "2", "3"};
    ASSERT_TRUE(Plato::srom::check_deterministic_loads(tDeterministicLoads));
}

TEST(PlatoTest, append_deterministic_loads)
{
    // SET INPUTS - CREATE SET OF RANDOM LOAD CASES
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    std::vector<double> tMyOriginalLoad(3, 1.0);
    std::vector<Plato::srom::RandomLoad> tNewSetRandomLoads;
    ASSERT_TRUE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION
    std::vector<Plato::srom::RandomLoadCase> tRandomLoadCases;
    ASSERT_TRUE(Plato::srom::expand_random_load_cases(tNewSetRandomLoads, tRandomLoadCases));
    ASSERT_EQ(12u, tRandomLoadCases.size());

    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tRandomLoadCases.size(); tLoadCaseIndex++)
    {
        ASSERT_EQ(1u, tRandomLoadCases[tLoadCaseIndex].numLoads());
    }

    // SET INPUTS - CREATE SET OF DETERMINISTIC LOADS
    Plato::srom::Load tLoadDet1;
    tLoadDet1.mAppID = 3;
    tLoadDet1.mAppType = "sideset";
    tLoadDet1.mLoadType = "pressure";
    tLoadDet1.mValues = {"1", "2", "3"};
    std::vector<Plato::srom::Load> tDeterministicLoads;
    tDeterministicLoads.push_back(tLoadDet1);

    // CALL FUNCTION TO BE TESTED
    Plato::srom::append_deterministic_loads(tDeterministicLoads, tRandomLoadCases);

    // TEST OUTPUT
    const double tTolerance = 1e-6;
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tRandomLoadCases.size(); tLoadCaseIndex++)
    {
        ASSERT_EQ(2u, tRandomLoadCases[tLoadCaseIndex].numLoads());
        // EXPECT VALUES FOR DETERMINISTIC LOAD
        ASSERT_STREQ("3", tRandomLoadCases[tLoadCaseIndex].applicationID(1).c_str());
        ASSERT_NEAR(1.0, tRandomLoadCases[tLoadCaseIndex].loadValue(1,0), tTolerance);
        ASSERT_NEAR(2.0, tRandomLoadCases[tLoadCaseIndex].loadValue(1,1), tTolerance);
        ASSERT_NEAR(3.0, tRandomLoadCases[tLoadCaseIndex].loadValue(1,2), tTolerance);
        ASSERT_NEAR(1.0, tRandomLoadCases[tLoadCaseIndex].loadProbability(1), tTolerance);
        ASSERT_STREQ(tLoadDet1.mAppType.c_str(), tRandomLoadCases[tLoadCaseIndex].applicationType(1).c_str());
        ASSERT_STREQ(tLoadDet1.mLoadType.c_str(), tRandomLoadCases[tLoadCaseIndex].loadType(1).c_str());
    }
}

TEST(PlatoTest, set_random_load_parameters_error)
{
    Plato::srom::Load tOriginalLoad;
    std::vector<Plato::srom::RandomLoad> tSetRandomLoads;
    ASSERT_FALSE(Plato::srom::set_random_load_parameters(tOriginalLoad, tSetRandomLoads));
}

TEST(PlatoTest, set_random_load_parameters)
{
    // SET INPUTS
    Plato::srom::Load tOriginalLoad;
    tOriginalLoad.mAppID = 1;
    tOriginalLoad.mAppType = "sideset";
    tOriginalLoad.mLoadType = "traction";
    tOriginalLoad.mValues = {"1", "2", "3"};

    // CALL FUNCTION
    Plato::srom::RandomLoad tRandVar1;
    tRandVar1.mProbability = 0.5;
    std::vector<Plato::srom::RandomLoad> tSetRandomLoads;
    tSetRandomLoads.push_back(tRandVar1);
    ASSERT_TRUE(Plato::srom::set_random_load_parameters(tOriginalLoad, tSetRandomLoads));

    // TEST OUTPUT
    ASSERT_EQ(tOriginalLoad.mAppID, tSetRandomLoads[0].mAppID);
    ASSERT_STREQ(tOriginalLoad.mAppType.c_str(), tSetRandomLoads[0].mAppType.c_str());
    ASSERT_STREQ(tOriginalLoad.mLoadType.c_str(), tSetRandomLoads[0].mLoadType.c_str());
}

TEST(PlatoTest, generate_set_random_loads_error)
{
    // ERROR: EMPTY ORIGINAL LOAD VALUES
    Plato::srom::Load tOriginalLoad;
    std::vector<Plato::srom::RandomLoad> tSetRandomLoads;
    std::vector<Plato::srom::RandomRotations> tSetRandomRotations;
    ASSERT_FALSE(Plato::srom::generate_set_random_loads(tOriginalLoad, tSetRandomRotations, tSetRandomLoads));

    // ERROR: EMPTY SET OF RANDOM ROTATIONS
    tOriginalLoad.mValues = {"1", "2", "3"};
    ASSERT_FALSE(Plato::srom::generate_set_random_loads(tOriginalLoad, tSetRandomRotations, tSetRandomLoads));

    // ERROR: LOAD PARAMETERS ARE NOT DEFINED
    Plato::srom::RandomRotations tRotations1;
    tRotations1.mRotations.resize(3, 0.0);
    tRotations1.mRotations[0] = 62.92995363352; tRotations1.mRotations[1] = 0; tRotations1.mRotations[2] = 0;
    tRotations1.mProbability = 0.3643018720139;
    tSetRandomRotations.push_back(tRotations1);
    Plato::srom::RandomRotations tRotations2;
    tRotations2.mRotations.resize(3, 0.0);
    tRotations2.mRotations[0] = 69.67128118964; tRotations1.mRotations[1] = 0; tRotations1.mRotations[2] = 0;
    tRotations2.mProbability = 0.1964474490484;
    tSetRandomRotations.push_back(tRotations2);
    Plato::srom::RandomRotations tRotations3;
    tRotations3.mRotations.resize(3, 0.0);
    tRotations3.mRotations[0] = 66.03455388567; tRotations1.mRotations[1] = 0; tRotations1.mRotations[2] = 0;
    tRotations3.mProbability = 0.2300630894941;
    tSetRandomRotations.push_back(tRotations3);
    Plato::srom::RandomRotations tRotations4;
    tRotations4.mRotations.resize(3, 0.0);
    tRotations4.mRotations[0] = 96.25276276890; tRotations1.mRotations[1] = 0; tRotations1.mRotations[2] = 0;
    tRotations4.mProbability = 0.2091697703866;
    tSetRandomRotations.push_back(tRotations4);
    ASSERT_FALSE(Plato::srom::generate_set_random_loads(tOriginalLoad, tSetRandomRotations, tSetRandomLoads));
}

TEST(PlatoTest, generate_set_random_loads)
{
    // SET INPUPTS
    Plato::srom::Load tOriginalLoad;
    tOriginalLoad.mAppID = 1;
    tOriginalLoad.mLoadID = 1;
    tOriginalLoad.mAppType = "nodeset";
    tOriginalLoad.mLoadType = "traction";
    tOriginalLoad.mValues = {"1", "1", "1"};
    std::vector<Plato::srom::RandomRotations> tSetRandomRotations;
    Plato::srom::RandomRotations tRotations1;
    tRotations1.mRotations.resize(3, 0.0);
    tRotations1.mRotations[0] = 62.92995363352; tRotations1.mRotations[1] = 0; tRotations1.mRotations[2] = 0;
    tRotations1.mProbability = 0.3643018720139;
    tSetRandomRotations.push_back(tRotations1);
    Plato::srom::RandomRotations tRotations2;
    tRotations2.mRotations.resize(3, 0.0);
    tRotations2.mRotations[0] = 69.67128118964; tRotations1.mRotations[1] = 0; tRotations1.mRotations[2] = 0;
    tRotations2.mProbability = 0.1964474490484;
    tSetRandomRotations.push_back(tRotations2);
    Plato::srom::RandomRotations tRotations3;
    tRotations3.mRotations.resize(3, 0.0);
    tRotations3.mRotations[0] = 66.03455388567; tRotations1.mRotations[1] = 0; tRotations1.mRotations[2] = 0;
    tRotations3.mProbability = 0.2300630894941;
    tSetRandomRotations.push_back(tRotations3);
    Plato::srom::RandomRotations tRotations4;
    tRotations4.mRotations.resize(3, 0.0);
    tRotations4.mRotations[0] = 96.25276276890; tRotations1.mRotations[1] = 0; tRotations1.mRotations[2] = 0;
    tRotations4.mProbability = 0.2091697703866;
    tSetRandomRotations.push_back(tRotations4);

    std::vector<Plato::srom::RandomLoad> tSetRandomLoads;
    ASSERT_TRUE(Plato::srom::generate_set_random_loads(tOriginalLoad, tSetRandomRotations, tSetRandomLoads));
    ASSERT_EQ(4u, tSetRandomLoads.size());

    std::vector<std::vector<double>> tGoldLoads =
        { {1, -0.435371385985053, 1.34553028812638}, {1, -0.590309206183812, 1.28512063289585},
          {1, -0.507604956861362, 1.31997621485002}, {1, -1.10296589510906, 0.885136280030518} };
    std::vector<double> tGoldProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};

    const double tTolerance = 1e-6;
    for(size_t tIndex =0; tIndex < tSetRandomLoads.size(); tIndex++)
    {
        ASSERT_NEAR(tGoldProbabilities[tIndex], tSetRandomLoads[tIndex].mProbability, tTolerance);
        ASSERT_NEAR(tGoldLoads[tIndex][0], tSetRandomLoads[tIndex].mLoadValues[0], tTolerance);
        ASSERT_NEAR(tGoldLoads[tIndex][1], tSetRandomLoads[tIndex].mLoadValues[1], tTolerance);
        ASSERT_NEAR(tGoldLoads[tIndex][2], tSetRandomLoads[tIndex].mLoadValues[2], tTolerance);
    }
}

TEST(PlatoTest, generate_output_random_load_cases_errors)
{
    Plato::srom::Load tLoadDet1;
    tLoadDet1.mAppID = 3;
    tLoadDet1.mAppType = "sideset";
    tLoadDet1.mLoadID = 3;
    tLoadDet1.mLoadType = "pressure";
    tLoadDet1.mValues = {"1", "2", "3"};
    std::vector<Plato::srom::Load> tDeterministicLoads;
    tDeterministicLoads.push_back(tLoadDet1);

    // ERROR 1: EMPTY SET OF RANDOM LOAD CASES
    std::vector<Plato::srom::RandomLoadCase> tSetRandomLoadCases;
    ASSERT_FALSE(Plato::srom::generate_output_random_load_cases(tDeterministicLoads, tSetRandomLoadCases));

    // ERROR 2: DETERMINISTIC LOAD PARAMETERS NOT DEFINED
    tDeterministicLoads[0].mAppType.clear(); // remove application type

    // SET INPUTS - CREATE SET OF RANDOM LOAD CASES
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    std::vector<double> tMyOriginalLoad(3, 1.0);
    std::vector<Plato::srom::RandomLoad> tNewSetRandomLoads;
    ASSERT_TRUE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION
    ASSERT_TRUE(Plato::srom::expand_random_load_cases(tNewSetRandomLoads, tSetRandomLoadCases));
    ASSERT_EQ(12u, tSetRandomLoadCases.size());

    ASSERT_FALSE(Plato::srom::generate_output_random_load_cases(tDeterministicLoads, tSetRandomLoadCases));
}

TEST(PlatoTest, generate_output_random_load_cases)
{
    // SET INPUTS - CREATE SET OF DETERMINISTIC LOADS
    Plato::srom::Load tLoadDet1;
    tLoadDet1.mAppID = 3;
    tLoadDet1.mAppType = "sideset";
    tLoadDet1.mLoadID = 3;
    tLoadDet1.mLoadType = "pressure";
    tLoadDet1.mValues = {"1", "2", "3"};
    std::vector<Plato::srom::Load> tDeterministicLoads;
    tDeterministicLoads.push_back(tLoadDet1);

    // SET INPUTS - CREATE SET OF RANDOM LOAD CASES
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetX;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetY;
    Plato::srom::SampleProbabilityPairs tSampleProbPairSetZ;
    tSampleProbPairSetX.mNumSamples = 4;
    tSampleProbPairSetX.mSamples = {62.92995363352, 69.67128118964, 66.03455388567, 96.2527627689};
    tSampleProbPairSetX.mProbabilities = {0.3643018720139, 0.1964474490484, 0.2300630894941, 0.2091697703866};
    tSampleProbPairSetZ.mNumSamples = 3;
    tSampleProbPairSetZ.mSamples = {79.56461506624, 95.1780010696, 104.3742043151};
    tSampleProbPairSetZ.mProbabilities = {0.441549282785, 0.3256625620299, 0.2326524892665};

    // COMPUTE RANDOM ROTATIONS SET
    std::vector<Plato::srom::RandomRotations> tMyRandomRotationsSet;
    ASSERT_TRUE(Plato::srom::expand_random_rotations(tSampleProbPairSetX, tSampleProbPairSetY, tSampleProbPairSetZ, tMyRandomRotationsSet));

    // CALL EXPAND RANDOM LOADS FUNCTION
    std::vector<double> tMyOriginalLoad(3, 1.0);
    std::vector<Plato::srom::RandomLoad> tNewSetRandomLoads;
    ASSERT_TRUE(Plato::srom::expand_random_loads(tMyOriginalLoad, tMyRandomRotationsSet, tNewSetRandomLoads));

    // CALL EXPAND RANDOM LOAD CASES FUNCTION
    std::vector<Plato::srom::RandomLoadCase> tSetRandomLoadCases;
    ASSERT_TRUE(Plato::srom::expand_random_load_cases(tNewSetRandomLoads, tSetRandomLoadCases));
    ASSERT_EQ(12u, tSetRandomLoadCases.size());

    // CALL FUNCTION TO BE TESTED
    ASSERT_TRUE(Plato::srom::generate_output_random_load_cases(tDeterministicLoads, tSetRandomLoadCases));

    // TEST OUTPUT
    std::vector<std::vector<std::vector<double>>> tGoldLoadCases =
    { { {0.609296803530850, 0.90460248117150, 1.34553028812638}, {1, 2, 3} }, { {0.343344489354037, 1.03521147857081, 1.34553028812638}, {1, 2, 3} },
      { {0.173488310131751, 1.07677762327334, 1.34553028812638}, {1, 2, 3} }, { {0.761671920821811, 0.87653912858296, 1.28512063289585}, {1, 2, 3} },
      { {0.497650027509830, 1.04919464782510, 1.28512063289585}, {1, 2, 3} }, { {0.323575806212112, 1.11524152385909, 1.28512063289585}, {1, 2, 3} },
      { {0.680335616523720, 0.89151906380038, 1.31997621485002}, {1, 2, 3} }, { {0.415283283979423, 1.04173057278621, 1.31997621485002}, {1, 2, 3} },
      { {0.243460611096492, 1.09470988077881, 1.31997621485002}, {1, 2, 3} }, { {1.265849164342930, 0.78368339200601, 0.88513628003052}, {1, 2, 3} },
      { {1.008214628020530, 1.09546201651137, 0.88513628003052}, {1, 2, 3} }, { {0.820183791860380, 1.24251048822264, 0.88513628003052}, {1, 2, 3} } };
    std::vector<double> tGoldLoadCasesProbs =
            {0.160857230304970, 0.118639480992335, 0.0847557373684797,  0.0867412302322639, 0.0639755795613402, 0.0457039880311642,
             0.101584192161421, 0.0749229351531628, 0.0535247504591439, 0.0923587620945064, 0.0681187633233061, 0.0486638677597447};
    std::vector<std::string> tGoldLoadIDs = {"1", "2"};
    std::vector<std::string> tGoldLoadCaseIDs = {"1", "2", "3", "4", "5", "6" , "7", "8", "9", "10", "11", "12"};

    const double tTolerance = 1e-6;
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tSetRandomLoadCases.size(); tLoadCaseIndex++)
    {
        const Plato::srom::RandomLoadCase& tLoadCase = tSetRandomLoadCases[tLoadCaseIndex];
        ASSERT_STREQ(tGoldLoadCaseIDs[tLoadCaseIndex].c_str(), tLoadCase.caseID().c_str());
        ASSERT_NEAR(tGoldLoadCasesProbs[tLoadCaseIndex], tLoadCase.probability(), tTolerance);
        const size_t tNumLoads = tLoadCase.numLoads();
        for(size_t tLoadIndex = 0; tLoadIndex < tNumLoads; tLoadIndex++)
        {
            ASSERT_STREQ(tGoldLoadIDs[tLoadIndex].c_str(), tLoadCase.loadID(tLoadIndex).c_str());
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][0], tLoadCase.loadValue(tLoadIndex,0), tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][1], tLoadCase.loadValue(tLoadIndex,1), tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][2], tLoadCase.loadValue(tLoadIndex,2), tTolerance);
        }
    }
}

TEST(PlatoTest, generate_load_sroms_error)
{
    std::vector<Plato::srom::Load> tInputs;
    std::vector<Plato::srom::RandomLoadCase> tOutputs;
    ASSERT_FALSE(Plato::srom::build_load_sroms(tInputs, tOutputs));
}

TEST(PlatoTest, generate_load_sroms_both_random_and_deterministic_loads)
{
    // SET INPUTS - CREATE FIRST RANDOM LOAD
    Plato::srom::Load tLoad1;
    tLoad1.mAppID = 1;
    tLoad1.mAppType = "nodeset";
    tLoad1.mLoadID = 1;
    tLoad1.mLoadType = "traction";
    tLoad1.mValues = {"1", "1", "1"};
    Plato::srom::RandomVariable tRandVar1;
    tRandVar1.tag("random rotation");
    tRandVar1.attribute("x");
    tRandVar1.mean("85");
    tRandVar1.samples("2");
    tRandVar1.lower("65");
    tRandVar1.upper("135");
    tRandVar1.distribution("beta");
    tRandVar1.deviation("15");
    // APPEND FIRST RANDOM VARIABLE FOR LOAD 1
    tLoad1.mRandomVars.push_back(tRandVar1);
    Plato::srom::RandomVariable tRandVar2;
    tRandVar2.tag("random rotation");
    tRandVar2.attribute("y");
    tRandVar2.mean("70");
    tRandVar2.samples("2");
    tRandVar2.lower("50");
    tRandVar2.upper("120");
    tRandVar2.distribution("beta");
    tRandVar2.deviation("15");
    // APPEND SECOND RANDOM VARIABLE FOR LOAD 1
    tLoad1.mRandomVars.push_back(tRandVar2);

    // APPEND FIRST RANDOM LOAD TO INPUT META DATA
    std::vector<Plato::srom::Load> tInputs;
    tInputs.push_back(tLoad1);

    // SET INPUTS - CREATE SECOND RANDOM LOAD
    Plato::srom::Load tLoad2;
    tLoad2.mAppID = 2;
    tLoad2.mAppType = "nodeset";
    tLoad2.mLoadID = 2;
    tLoad2.mLoadType = "traction";
    tLoad2.mValues = {"1", "1", "1"};
    tRandVar1.tag("random rotation");
    tRandVar1.attribute("x");
    tRandVar1.mean("85");
    tRandVar1.samples("2");
    tRandVar1.lower("65");
    tRandVar1.upper("135");
    tRandVar1.distribution("beta");
    tRandVar1.deviation("15");
    // APPEND FIRST RANDOM VARIABLE FOR LOAD 2
    tLoad2.mRandomVars.push_back(tRandVar1);
    tRandVar2.tag("random rotation");
    tRandVar2.attribute("y");
    tRandVar2.mean("70");
    tRandVar2.samples("2");
    tRandVar2.lower("50");
    tRandVar2.upper("120");
    tRandVar2.distribution("beta");
    tRandVar2.deviation("10");
    // APPEND SECOND RANDOM VARIABLE FOR LOAD 2
    tLoad2.mRandomVars.push_back(tRandVar2);

    // APPEND SECOND RANDOM LOAD TO INPUT META DATA
    tInputs.push_back(tLoad2);

    // SET INPUTS - CREATE DETERMINISTIC LOADS
    Plato::srom::Load tLoad3;
    tLoad3.mAppID = 3;
    tLoad3.mAppType = "sideset";
    tLoad3.mLoadID = 3;
    tLoad3.mLoadType = "pressure";
    tLoad3.mValues = {"1", "2", "3"};

    // SET INPUTS - APPEND FORST DETERMINISTIC LOAD
    tInputs.push_back(tLoad3);

    // CALL FUNCTION TO BE TESTED
    std::vector<Plato::srom::RandomLoadCase> tOutputs;
    ASSERT_TRUE(Plato::srom::build_load_sroms(tInputs, tOutputs));
    ASSERT_EQ(16u, tOutputs.size());

    // SET GOLD VALUES
    std::vector<std::string> tGoldLoadCaseIDs =
        {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16"};
    std::vector<std::vector<std::vector<double>>> tGoldLoadCases =
    { { {1.571521544770223, -0.6912988161476783, -0.228967205328253}, {1.45459857379489, -0.6912988161476783, -0.6373766044550824}, {1, 2, 3} },
      { {1.243876404316842, -0.6912988161476783, -0.987358819060539}, {1.45459857379489, -0.6912988161476783, -0.6373766044550824}, {1, 2, 3} },
      { {1.129580691573818, -1.2174657500169410, -0.491756656039655}, {1.45459857379489, -0.6912988161476783, -0.6373766044550824}, {1, 2, 3} },
      { {0.7297339277702746, -1.217465750016941, -0.992605431275013}, {1.45459857379489, -0.6912988161476783, -0.6373766044550824}, {1, 2, 3} },
      { {1.571521544770223, -0.6912988161476783, -0.228967205328253}, {1.49410148829542, -0.6912988161476783, -0.5382998137341604}, {1, 2, 3} },
      { {1.243876404316842, -0.6912988161476783, -0.987358819060539}, {1.49410148829542, -0.6912988161476783, -0.5382998137341604}, {1, 2, 3} },
      { {1.129580691573818, -1.2174657500169410, -0.491756656039655}, {1.49410148829542, -0.6912988161476783, -0.5382998137341604}, {1, 2, 3} },
      { {0.7297339277702746, -1.217465750016941, -0.992605431275013}, {1.49410148829542, -0.6912988161476783, -0.5382998137341604}, {1, 2, 3} },
      { {1.571521544770223, -0.6912988161476783, -0.228967205328253}, {0.95880529000861, -1.2174657500169410, -0.7736081458897784}, {1, 2, 3} },
      { {1.243876404316842, -0.6912988161476783, -0.987358819060539}, {0.95880529000861, -1.2174657500169410, -0.7736081458897784}, {1, 2, 3} },
      { {1.129580691573818, -1.2174657500169410, -0.491756656039655}, {0.95880529000861, -1.2174657500169410, -0.7736081458897784}, {1, 2, 3} },
      { {0.7297339277702746, -1.217465750016941, -0.992605431275013}, {0.95880529000861, -1.2174657500169410, -0.7736081458897784}, {1, 2, 3} },
      { {1.571521544770223, -0.6912988161476783, -0.228967205328253}, {1.008570893282811, -1.217465750016941, -0.7075039934575635}, {1, 2, 3} },
      { {1.243876404316842, -0.6912988161476783, -0.987358819060539}, {1.008570893282811, -1.217465750016941, -0.7075039934575635}, {1, 2, 3} },
      { {1.129580691573818, -1.2174657500169410, -0.491756656039655}, {1.008570893282811, -1.217465750016941, -0.7075039934575635}, {1, 2, 3} },
      { {0.7297339277702746, -1.217465750016941, -0.992605431275013}, {1.008570893282811, -1.217465750016941, -0.7075039934575635}, {1, 2, 3} } };

    std::vector<double> tGoldLoadCasesProbs =
            {0.1271150547979226, 0.08266435562779866, 0.08266435562779866, 0.05375756398187734, 0.0953682783064585, 0.0620190683634493, 0.0620190683634493, 0.04033170052950105,
             0.0826643556277987, 0.05375756398187733, 0.05375756398187733, 0.03495915093413995, 0.0620190683634493, 0.0403317005295011, 0.0403317005295011, 0.02622816031464305};

    std::vector<std::vector<std::string>> tGoldAppType =
            { {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"},
              {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"},
              {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"},
              {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"}, {"nodeset", "nodeset", "sideset"} };

    std::vector<std::vector<std::string>> tGoldLoadType =
            { {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"},
              {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"},
              {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"},
              {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"}, {"traction", "traction", "pressure"} };

    std::vector<std::vector<std::string>> tGoldAppIDs =
            { {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"},
              {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"}, {"1", "2", "3"} };

    std::vector<std::string> tGoldLoadIDs = {"1", "2", "3"};

    // TEST OUTPUT
    double tSum = 0;
    double tTolerance = 1e-6;
    for(auto& tLoadCase : tOutputs)
    {
        tSum += tLoadCase.probability();

        auto tLoadCaseIndex = &tLoadCase - &tOutputs[0];
        ASSERT_STREQ(tGoldLoadCaseIDs[tLoadCaseIndex].c_str(), tLoadCase.caseID().c_str());
        ASSERT_NEAR(tGoldLoadCasesProbs[tLoadCaseIndex], tLoadCase.probability(), tTolerance);
        for(auto& tLoad : tLoadCase.loads())
        {
            auto tLoadIndex = &tLoad - &tLoadCase.loads()[0];
            ASSERT_STREQ(tGoldLoadIDs[tLoadIndex].c_str(), tLoadCase.loadID(tLoadIndex).c_str());
            ASSERT_STREQ(tGoldAppIDs[tLoadCaseIndex][tLoadIndex].c_str(), tLoadCase.applicationID(tLoadIndex).c_str());
            ASSERT_STREQ(tGoldLoadType[tLoadCaseIndex][tLoadIndex].c_str(), tLoadCase.loadType(tLoadIndex).c_str());
            ASSERT_STREQ(tGoldAppType[tLoadCaseIndex][tLoadIndex].c_str(), tLoadCase.applicationType(tLoadIndex).c_str());
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][0], tLoadCase.loadValue(tLoadIndex,0), tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][1], tLoadCase.loadValue(tLoadIndex,1), tTolerance);
            ASSERT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][2], tLoadCase.loadValue(tLoadIndex,2), tTolerance);
        }
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);

    Plato::system("rm -f plato_cdf_output.txt");
    Plato::system("rm -f plato_srom_diagnostics.txt");
    Plato::system("rm -f plato_ksal_algorithm_diagnostics.txt");
}

TEST(PlatoTest, generate_load_sroms_only_random_loads)
{
    // SET INPUTS - CREATE FIRST RANDOM LOAD
    Plato::srom::Load tLoad1;
    tLoad1.mAppID = 1;
    tLoad1.mAppType = "nodeset";
    tLoad1.mLoadID = 1;
    tLoad1.mLoadType = "traction";
    tLoad1.mValues = {"1", "1", "1"};
    Plato::srom::RandomVariable tRandVar1;
    tRandVar1.tag("random rotation");
    tRandVar1.attribute("x");
    tRandVar1.mean("85");
    tRandVar1.samples("2");
    tRandVar1.lower("65");
    tRandVar1.upper("135");
    tRandVar1.distribution("beta");
    tRandVar1.deviation("15");
    // APPEND FIRST RANDOM VARIABLE FOR LOAD 1
    tLoad1.mRandomVars.push_back(tRandVar1);

    // APPEND FIRST RANDOM LOAD TO INPUT META DATA
    std::vector<Plato::srom::Load> tInputs;
    tInputs.push_back(tLoad1);

    // SET INPUTS - CREATE SECOND RANDOM LOAD
    Plato::srom::Load tLoad2;
    tLoad2.mAppID = 2;
    tLoad2.mAppType = "sideset";
    tLoad2.mLoadID = 2;
    tLoad2.mLoadType = "pressure";
    tLoad2.mValues = {"1", "1", "1"};
    tRandVar1.tag("random rotation");
    tRandVar1.attribute("x");
    tRandVar1.mean("85");
    tRandVar1.samples("2");
    tRandVar1.lower("65");
    tRandVar1.upper("135");
    tRandVar1.distribution("beta");
    tRandVar1.deviation("10");
    // APPEND FIRST RANDOM VARIABLE FOR LOAD 2
    tLoad2.mRandomVars.push_back(tRandVar1);

    // APPEND SECOND RANDOM LOAD TO INPUT META DATA
    tInputs.push_back(tLoad2);

    // CALL FUNCTION TO BE TESTED
    std::vector<Plato::srom::RandomLoadCase> tOutputs;
    ASSERT_TRUE(Plato::srom::build_load_sroms(tInputs, tOutputs));
    ASSERT_EQ(4u, tOutputs.size());

    // SET GOLD VALUES
    std::vector<std::string> tGoldLoadCaseIDs = {"1", "2", "3", "4"};
    std::vector<std::vector<std::vector<double>>> tGoldLoadCases =
    { { {1.0, -0.691298816147678, 1.2337365791743464}, {1.0, -0.99362231248073574, 1.006337269577369} },
      { {1.0, -1.217465750016941, 0.7195673335662809}, {1.0, -0.99362231248073574, 1.006337269577369} },
      { {1.0, -0.691298816147678, 1.2337365791743464}, {1.0, -0.9238313790941014, 1.0707640183537599} },
      { {1.0, -1.217465750016941, 0.7195673335662809}, {1.0, -0.9238313790941014, 1.0707640183537599} } };

    std::vector<double> tGoldLoadCasesProbs = {0.3462045113834939, 0.22514070339228703, 0.25974050237448448, 0.16891218190764895};

    std::vector<std::vector<std::string>> tGoldAppType =
            { {"nodeset", "sideset"}, {"nodeset", "sideset"}, {"nodeset", "sideset"}, {"nodeset", "sideset"} };

    std::vector<std::vector<std::string>> tGoldLoadType =
            { {"traction", "pressure"}, {"traction", "pressure"}, {"traction", "pressure"}, {"traction", "pressure"} };

    std::vector<std::vector<std::string>> tGoldAppIDs = { {"1", "2"}, {"1", "2"}, {"1", "2"}, {"1", "2"} };

    std::vector<std::string> tGoldLoadIDs = {"1", "2"};

    // TEST OUTPUT
    double tSum = 0;
    double tTolerance = 1e-6;
    for(size_t tLoadCaseIndex = 0; tLoadCaseIndex < tOutputs.size(); tLoadCaseIndex++)
    {
        const Plato::srom::RandomLoadCase& tLoadCase = tOutputs[tLoadCaseIndex];
        tSum += tLoadCase.probability();
        ASSERT_STREQ(tGoldLoadCaseIDs[tLoadCaseIndex].c_str(), tLoadCase.caseID().c_str());
        ASSERT_NEAR(tGoldLoadCasesProbs[tLoadCaseIndex], tLoadCase.probability(), tTolerance);
        const size_t tNumLoads = tLoadCase.numLoads();
        for(size_t tLoadIndex = 0; tLoadIndex < tNumLoads; tLoadIndex++)
        {
            ASSERT_STREQ(tGoldLoadIDs[tLoadIndex].c_str(), tLoadCase.loadID(tLoadIndex).c_str());
            ASSERT_STREQ(tGoldAppIDs[tLoadCaseIndex][tLoadIndex].c_str(), tLoadCase.applicationID(tLoadIndex).c_str());
            ASSERT_STREQ(tGoldLoadType[tLoadCaseIndex][tLoadIndex].c_str(), tLoadCase.loadType(tLoadIndex).c_str());
            ASSERT_STREQ(tGoldAppType[tLoadCaseIndex][tLoadIndex].c_str(), tLoadCase.applicationType(tLoadIndex).c_str());
            EXPECT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][0], tLoadCase.loadValue(tLoadIndex,0), tTolerance);
            EXPECT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][1], tLoadCase.loadValue(tLoadIndex,1), tTolerance);
            EXPECT_NEAR(tGoldLoadCases[tLoadCaseIndex][tLoadIndex][2], tLoadCase.loadValue(tLoadIndex,2), tTolerance);
        }
    }

    tTolerance = 1e-2;
    ASSERT_NEAR(1.0, tSum, tTolerance);

    Plato::system("rm -f plato_cdf_output.txt");
    Plato::system("rm -f plato_srom_diagnostics.txt");
    Plato::system("rm -f plato_ksal_algorithm_diagnostics.txt");
}


}
