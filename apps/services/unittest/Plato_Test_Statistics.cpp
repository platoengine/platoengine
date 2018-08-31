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
 * Plato_Test_Statistics.cpp
 *
 *  Created on: Nov 17, 2017
 */

#include "gtest/gtest.h"

#include <numeric>
#include <cmath>

#include "Plato_DataFactory.hpp"
#include "Plato_Diagnostics.hpp"
#include "Plato_SromObjective.hpp"
#include "Plato_Communication.hpp"
#include "Plato_SromConstraint.hpp"
#include "Plato_SromStatistics.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_BetaDistribution.hpp"
#include "Plato_NormalDistribution.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_UniformDistribution.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_AugmentedLagrangianStageMng.hpp"
#include "Plato_KelleySachsAugmentedLagrangian.hpp"
#include "Plato_StandardVectorReductionOperations.hpp"
#include "Plato_StatisticsUtils.hpp"
#include "Plato_SolveUncertaintyProblem.hpp"

#include "Plato_UnitTestUtils.hpp"

namespace PlatoTest
{

TEST(PlatoTest, Uniform)
{
    const double tLowerBound = 2;
    const double tUpperBound = 7;
    Plato::UniformDistribution<double> tDistribution(tLowerBound, tUpperBound);

    // ********** TEST STATISTICS FOR UNIFORM DISTRIBUTION **********
    const double tTolerance = 1e-5;
    EXPECT_NEAR(tLowerBound, tDistribution.lower(), tTolerance);
    EXPECT_NEAR(tUpperBound, tDistribution.upper(), tTolerance);
    double tGoldValue = 4.5;
    EXPECT_NEAR(tGoldValue, tDistribution.mean(), tTolerance);
    tGoldValue = (1. / 12.) * (tUpperBound - tLowerBound) * (tUpperBound - tLowerBound);
    EXPECT_NEAR(tGoldValue, tDistribution.variance(), tTolerance);
    tGoldValue = std::log(tUpperBound - tLowerBound);
    EXPECT_NEAR(tGoldValue, tDistribution.entropy(), tTolerance);
    // ********** TEST PDF **********
    std::vector<double> tSamples;
    tSamples.push_back(1);
    tSamples.push_back(2);
    tSamples.push_back(3);
    tSamples.push_back(4);
    tSamples.push_back(5);
    tSamples.push_back(6);
    tSamples.push_back(7);
    tSamples.push_back(8);
    std::vector<double> tGoldPDF;
    tGoldPDF.push_back(0);
    tGoldPDF.push_back(0.2);
    tGoldPDF.push_back(0.2);
    tGoldPDF.push_back(0.2);
    tGoldPDF.push_back(0.2);
    tGoldPDF.push_back(0.2);
    tGoldPDF.push_back(0.2);
    tGoldPDF.push_back(0);
    for(size_t tIndex = 0; tIndex < tGoldPDF.size(); tIndex++)
    {
        EXPECT_NEAR(tGoldPDF[tIndex], tDistribution.pdf(tSamples[tIndex]), tTolerance);
    }
    // ********** TEST CDF **********
    std::vector<double> tGoldCDF;
    tGoldCDF.push_back(0);
    tGoldCDF.push_back(0);
    tGoldCDF.push_back(.2);
    tGoldCDF.push_back(.4);
    tGoldCDF.push_back(.6);
    tGoldCDF.push_back(.8);
    tGoldCDF.push_back(1);
    tGoldCDF.push_back(1);
    for(size_t tIndex = 0; tIndex < tGoldCDF.size(); tIndex++)
    {
        EXPECT_NEAR(tGoldCDF[tIndex], tDistribution.cdf(tSamples[tIndex]), tTolerance);
    }
    // ********** TEST MOMENTS **********
    size_t tNumMoments = 8;
    for(size_t tOrder = 0; tOrder <= tNumMoments; tOrder++)
    {
        double tExponent = tOrder + 1u;
        double tGold = (std::pow(tUpperBound, tExponent) - std::pow(tLowerBound, tExponent))
                / (tExponent * (tUpperBound - tLowerBound));
        EXPECT_NEAR(tGold, tDistribution.moment(tOrder), tTolerance);
    }
}

TEST(PlatoTest, Normal)
{
    const double tMean = 0;
    const double tSigma = 1;

    double tEndRange = 1e3;
    double tBeginRange = -1e3;
    double tIncrement = 0.1;
    double tRange = ((tEndRange - tBeginRange) / tIncrement) + 1;
    std::vector<double> tNumbers(tRange);

    std::vector<double> tPDF(tRange);
    std::vector<double> tCDF(tRange);
    for(size_t tIndex = 0; tIndex < tRange; tIndex++)
    {
        tNumbers[tIndex] = tBeginRange + (tIncrement * tIndex);
        tPDF[tIndex] = Plato::normal_pdf<double>(tNumbers[tIndex], tMean, tSigma);
        tCDF[tIndex] = Plato::normal_cdf<double>(tNumbers[tIndex], tMean, tSigma);
    }

    const double tBaseValue = 0;
    const double tTolerance = 1e-3;
    double tValue = std::accumulate(tPDF.begin(), tPDF.end(), tBaseValue) / tRange;
    EXPECT_NEAR(tValue, tMean, tTolerance);
}

TEST(PlatoTest, SromCDF)
{
    double tSample = 0.276806509167094;
    Plato::StandardVector<double> tSamples(4, 0.);
    tSamples[0] = 0.276806509167094;
    tSamples[1] = 0.431107226622461;
    tSamples[2] = 0.004622102620248;
    tSamples[3] = 0.224162021074166;
    Plato::StandardVector<double> tSampleProbabilities(4, 0.);
    tSampleProbabilities[0] = 0.25;
    tSampleProbabilities[1] = 0.25;
    tSampleProbabilities[2] = 0.25;
    tSampleProbabilities[3] = 0.25;
    double tSigma = 1e-3;

    double tOutput = Plato::compute_srom_cdf<double>(tSample, tSigma, tSamples, tSampleProbabilities);

    double tGold = 0.625;
    double tTolerance = 1e-4;
    EXPECT_NEAR(tGold, tOutput, tTolerance);
}

TEST(PlatoTest, SromMoments)
{
    Plato::StandardVector<double> tSamples(4, 0.);
    tSamples[0] = 0.276806509167094;
    tSamples[1] = 0.431107226622461;
    tSamples[2] = 0.004622102620248;
    tSamples[3] = 0.224162021074166;
    Plato::StandardVector<double> tSampleProbabilities(4, 0.);
    tSampleProbabilities[0] = 0.25;
    tSampleProbabilities[1] = 0.25;
    tSampleProbabilities[2] = 0.25;
    tSampleProbabilities[3] = 0.25;

    Plato::StandardVector<double> tMoments(4, 0.);
    for(size_t tIndex = 0; tIndex < tMoments.size(); tIndex++)
    {
        double tOrder = tIndex + static_cast<size_t>(1);
        tMoments[tIndex] = Plato::compute_srom_moment<double>(tOrder, tSamples, tSampleProbabilities);
    }

    Plato::StandardVector<double> tGold(4, 0.);
    tGold[0] = 0.234174464870992;
    tGold[1] = 0.078186314972017;
    tGold[2] = 0.028149028892565;
    tGold[3] = 0.010734332952929;
    PlatoTest::checkVectorData(tMoments, tGold);
}

TEST(PlatoTest, factorial)
{
    size_t tGold = 1;
    size_t tValue = Plato::factorial<size_t>(0);
    EXPECT_EQ(tGold, tValue);

    tGold = 1;
    tValue = Plato::factorial<size_t>(1);
    EXPECT_EQ(tGold, tValue);

    tGold = 362880;
    tValue = Plato::factorial<size_t>(9);
    EXPECT_EQ(tGold, tValue);
}

TEST(PlatoTest, Beta)
{
    double tAlpha = 1;
    double tBeta = 3;
    double tValue = Plato::beta<double>(tAlpha, tBeta);

    double tTolerance = 1e-6;
    double tGold = 1. / 3.;
    EXPECT_NEAR(tGold, tValue, tTolerance);
}

TEST(PlatoTest, PochhammerSymbol)
{
    // TEST ONE: NON-FINITE NUMBER CASE
    double tOutput = Plato::pochhammer_symbol<double>(-2, 0);
    const double tTolerance = 1e-5;
    EXPECT_NEAR(0, tOutput, tTolerance);

    // TEST TWO: FINITE NUMBER CASE
    tOutput = Plato::pochhammer_symbol<double>(2.166666666666666, 4.333333333333333);
    double tGold = 265.98433449717857;
    EXPECT_NEAR(tGold, tOutput, tTolerance);
}

TEST(PlatoTest, BetaMoment)
{
    size_t tOrder = 3;
    double tAlpha = 2.166666666666666;
    double tBeta = 4.333333333333333;
    double tValue = Plato::beta_moment<double>(tOrder, tAlpha, tBeta);

    double tTolerance = 1e-6;
    double tGold = 0.068990559186638;
    EXPECT_NEAR(tGold, tValue, tTolerance);
}

TEST(PlatoTest, ComputeShapeParameters)
{
    const double tMean = 90;
    const double tMaxValue = 135;
    const double tMinValue = 67.5;
    const double tVariance = 135;
    double tAlphaShapeParameter = 0;
    double tBetaShapeParameter = 0;
    Plato::shape_parameters<double>(tMinValue, tMaxValue, tMean, tVariance, tAlphaShapeParameter, tBetaShapeParameter);

    const double tTolerance = 1e-6;
    const double tGoldAlpha = 2.166666666666666;
    const double tGoldBeta = 4.333333333333333;
    EXPECT_NEAR(tGoldBeta, tBetaShapeParameter, tTolerance);
    EXPECT_NEAR(tGoldAlpha, tAlphaShapeParameter, tTolerance);
}

TEST(PlatoTest, IncompleteBeta)
{
    double tSample = 1.;
    const double tAlpha = 2.166666666666666;
    const double tBeta = 4.333333333333333;

    double tOutput = Plato::incomplete_beta<double>(tSample, tAlpha, tBeta);

    const double tTolerance = 1e-5;
    const double tGold = Plato::beta<double>(tAlpha, tBeta);
    EXPECT_NEAR(tOutput, tGold, tTolerance);
}

TEST(PlatoTest, BetaPDF)
{
    const size_t tRange = 4;
    std::vector<double> tPDF(tRange);
    std::vector<double> tShapeParam(tRange);
    tShapeParam[0] = 0.5;
    tShapeParam[1] = 1;
    tShapeParam[2] = 2;
    tShapeParam[3] = 4;
    const double tSample = 0.5;
    for(size_t tIndex = 0; tIndex < tRange; tIndex++)
    {
        tPDF[tIndex] = Plato::beta_pdf<double>(tSample, tShapeParam[tIndex], tShapeParam[tIndex]);
    }

    const double tTolerance = 1e-6;
    std::vector<double> tGoldPDF(tRange, 0.);
    tGoldPDF[0] = 0.636619772367582;
    tGoldPDF[1] = 1.0;
    tGoldPDF[2] = 1.5;
    tGoldPDF[3] = 2.1875;
    for(size_t tIndex = 0; tIndex < tRange; tIndex++)
    {
        EXPECT_NEAR(tPDF[tIndex], tGoldPDF[tIndex], tTolerance);
    }
}

TEST(PlatoTest, BetaCDF)
{
    const double tBeta = 3;
    const double tSample = 0.5;
    const size_t tRange = 11;
    std::vector<double> tCDF(tRange, 0.);
    for(size_t tIndex = 0; tIndex < tRange; tIndex++)
    {
        tCDF[tIndex] = Plato::beta_cdf<double>(tSample, tIndex, tBeta);
    }

    const double tTolerance = 1e-6;
    std::vector<double> tGoldCDF(tRange, 0.);
    tGoldCDF[0] = 1;
    tGoldCDF[1] = 0.875;
    tGoldCDF[2] = 0.6875;
    tGoldCDF[3] = 0.5;
    tGoldCDF[4] = 0.34375;
    tGoldCDF[5] = 0.2265625;
    tGoldCDF[6] = 0.14453125;
    tGoldCDF[7] = 0.08984375;
    tGoldCDF[8] = 0.0546875;
    tGoldCDF[9] = 0.03271484375;
    tGoldCDF[10] = 0.019287109375;
    for(size_t tIndex = 0; tIndex < tRange; tIndex++)
    {
        EXPECT_NEAR(tCDF[tIndex], tGoldCDF[tIndex], tTolerance);
    }
}

TEST(PlatoTest, BetaDistribution)
{
    const double tMean = 90;
    const double tMax = 135;
    const double tMin = 67.5;
    const double tVariance = 135;
    Plato::BetaDistribution<double> tDistribution(tMin, tMax, tMean, tVariance);

    // TEST INPUTS
    const double tTolerance = 1e-5;
    EXPECT_NEAR(tMin, tDistribution.min(), tTolerance);
    EXPECT_NEAR(tMax, tDistribution.max(), tTolerance);
    EXPECT_NEAR(tMean, tDistribution.mean(), tTolerance);
    EXPECT_NEAR(tVariance, tDistribution.variance(), tTolerance);

    // TEST BETA PDF & CDF
    double tSample = 0.276806509167094;
    double tGoldPDF = 2.179085850493935;
    EXPECT_NEAR(tGoldPDF, tDistribution.pdf(tSample), tTolerance);
    double tGoldCDF = 0.417022004702574;
    EXPECT_NEAR(tGoldCDF, tDistribution.cdf(tSample), tTolerance);

    // TEST BETA MOMENTS
    const size_t tNumMoments = 4;
    Plato::StandardVector<double> tGoldMoments(tNumMoments);
    tGoldMoments[0] = 0.333333333333333;
    tGoldMoments[1] = 0.140740740740740;
    tGoldMoments[2] = 0.068990559186638;
    tGoldMoments[3] = 0.037521181312031;
    for(size_t tIndex = 0; tIndex < tNumMoments; tIndex++)
    {
        size_t tOrder = tIndex + 1;
        EXPECT_NEAR(tGoldMoments[tIndex], tDistribution.moment(tOrder), tTolerance);
    }
}

TEST(PlatoTest, SromObjectiveTestOne)
{
    // ********* ALLOCATE BETA DISTRIBUTION *********
    const double tMean = 90;
    const double tMax = 135;
    const double tMin = 67.5;
    const double tVariance = 135;
    std::shared_ptr<Plato::BetaDistribution<double>> tDistribution =
            std::make_shared<Plato::BetaDistribution<double>>(tMin, tMax, tMean, tVariance);

    // ********* SET TEST DATA: SAMPLES AND PROBABILITIES *********
    const size_t tNumVectors = 2;
    const size_t tNumControls = 4;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls);
    size_t tVectorIndex = 0;
    tControl(tVectorIndex, 0) = 0.276806509167094;
    tControl(tVectorIndex, 1) = 0.431107226622461;
    tControl(tVectorIndex, 2) = 0.004622102620248;
    tControl(tVectorIndex, 3) = 0.224162021074166;
    tVectorIndex = 1;
    tControl[tVectorIndex].fill(0.25);

    // ********* TEST OBJECTIVE FUNCTION *********
    const size_t tMaxNumMoments = 4;
    Plato::SromObjective<double> tObjective(tDistribution, tMaxNumMoments);
    tObjective.setCdfMisfitTermWeight(1);
    tObjective.setMomentMisfitTermWeight(1);
    double tValue = tObjective.value(tControl);
    double tTolerance = 1e-5;
    double tGold = 0.617109315688096;
    EXPECT_NEAR(tGold, tValue, tTolerance);

    // ********* TEST OBJECTIVE GRADIENT *********
    Plato::StandardMultiVector<double> tGradient(tNumVectors, tNumControls);
    tObjective.gradient(tControl, tGradient);
    Plato::StandardMultiVector<double> tGradientGold(tNumVectors, tNumControls);
    tVectorIndex = 0;
    tGradientGold(tVectorIndex, 0) = -2.010045017107233;
    tGradientGold(tVectorIndex, 1) = -3.878346258927178;
    tGradientGold(tVectorIndex, 2) = -0.237208262654126;
    tGradientGold(tVectorIndex, 3) = -1.271234346951175;
    tVectorIndex = 1;
    tGradientGold(tVectorIndex, 0) = -0.524038145360132;
    tGradientGold(tVectorIndex, 1) = -2.239056684273221;
    tGradientGold(tVectorIndex, 2) = 0.493570515676146;
    tGradientGold(tVectorIndex, 3) = -0.104442116117926;
    PlatoTest::checkMultiVectorData(tGradient, tGradientGold);
}

TEST(PlatoTest, SromObjectiveTestTwo)
{
    // ********* ALLOCATE BETA DISTRIBUTION *********
    const double tMean = 90;
    const double tMax = 135;
    const double tMin = 67.5;
    const double tVariance = 135;
    std::shared_ptr<Plato::BetaDistribution<double>> tDistribution =
            std::make_shared<Plato::BetaDistribution<double>>(tMin, tMax, tMean, tVariance);

    // ********* SET TEST DATA: SAMPLES AND PROBABILITIES *********
    const size_t tNumVectors = 3;
    const size_t tNumControls = 4;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls);
    size_t tVectorIndex = 0;
    tControl(tVectorIndex, 0) = 0.276806509167094;
    tControl(tVectorIndex, 1) = 0.004622102620248;
    tControl(tVectorIndex, 2) = 0.376806509167094;
    tControl(tVectorIndex, 3) = 0.104622102620248;
    tVectorIndex = 1;
    tControl(tVectorIndex, 0) = 0.431107226622461;
    tControl(tVectorIndex, 1) = 0.224162021074166;
    tControl(tVectorIndex, 2) = 0.531107226622461;
    tControl(tVectorIndex, 3) = 0.324162021074166;
    tVectorIndex = 2;
    tControl[tVectorIndex].fill(0.25);

    // ********* TEST OBJECTIVE FUNCTION *********
    const size_t tRandomVecDim = 2;
    const size_t tMaxNumMoments = 4;
    Plato::SromObjective<double> tObjective(tDistribution, tMaxNumMoments, tRandomVecDim);
    tObjective.setCdfMisfitTermWeight(1);
    tObjective.setMomentMisfitTermWeight(1);
    double tValue = tObjective.value(tControl);
    double tTolerance = 1e-5;
    double tGold = 1.032230626961365;
    EXPECT_NEAR(tGold, tValue, tTolerance);

    // ********* TEST OBJECTIVE GRADIENT *********
    Plato::StandardMultiVector<double> tGradient(tNumVectors, tNumControls);
    tObjective.gradient(tControl, tGradient);
    Plato::StandardMultiVector<double> tGradientGold(tNumVectors, tNumControls);
    tVectorIndex = 0;
    tGradientGold(tVectorIndex, 0) = -2.42724408126656;
    tGradientGold(tVectorIndex, 1) = -0.337450847795798;
    tGradientGold(tVectorIndex, 2) = -3.887791716578634;
    tGradientGold(tVectorIndex, 3) = -1.076413326527892;
    tVectorIndex = 1;
    tGradientGold(tVectorIndex, 0) = 0.096246202011561;
    tGradientGold(tVectorIndex, 1) = 0.520617569090164;
    tGradientGold(tVectorIndex, 2) = -0.321363712239195;
    tGradientGold(tVectorIndex, 3) = 0.384504837554259;
    tVectorIndex = 2;
    tGradientGold(tVectorIndex, 0) = -0.53206506489113;
    tGradientGold(tVectorIndex, 1) = 0.619653114279367;
    tGradientGold(tVectorIndex, 2) = -1.84853491196106;
    tGradientGold(tVectorIndex, 3) = 0.426963908092988;
    PlatoTest::checkMultiVectorData(tGradient, tGradientGold);
}

TEST(PlatoTest, SromConstraint)
{
    // ********* SET TEST DATA: SAMPLES AND PROBABILITIES *********
    const size_t tNumVectors = 2;
    const size_t tNumControls = 4;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls);
    size_t tVectorIndex = 0;
    tControl(tVectorIndex, 0) = 0.183183326166505;
    tControl(tVectorIndex, 1) = 0.341948604575779;
    tControl(tVectorIndex, 2) = 0.410656896223290;
    tControl(tVectorIndex, 3) = 0.064209040541960;
    tVectorIndex = 1;
    tControl(tVectorIndex, 0) = 0.434251989288042;
    tControl(tVectorIndex, 1) = 0.351721349341024;
    tControl(tVectorIndex, 2) = 0.001250000000000;
    tControl(tVectorIndex, 3) = 0.212776663693648;

    // ********* TEST CONSTRAINT EVALUATION *********
    std::shared_ptr<Plato::StandardVectorReductionOperations<double>> tReductions =
            std::make_shared<Plato::StandardVectorReductionOperations<double>>();
    Plato::SromConstraint<double> tConstraint(tReductions);
    double tValue = tConstraint.value(tControl);

    double tGoldValue = 0;
    double tTolerance = 1e-6;
    EXPECT_NEAR(tGoldValue, tValue, tTolerance);

    // ********* TEST CONSTRAINT GRADIENT *********
    Plato::StandardMultiVector<double> tGradient(tNumVectors, tNumControls);
    tConstraint.gradient(tControl, tGradient);
    Plato::StandardMultiVector<double> tGradientGold(tNumVectors, tNumControls);
    tVectorIndex = 0;
    tGradientGold(tVectorIndex, 0) = 0;
    tGradientGold(tVectorIndex, 1) = 0;
    tGradientGold(tVectorIndex, 2) = 0;
    tGradientGold(tVectorIndex, 3) = 0;
    tVectorIndex = 1;
    tGradientGold(tVectorIndex, 0) = 1;
    tGradientGold(tVectorIndex, 1) = 1;
    tGradientGold(tVectorIndex, 2) = 1;
    tGradientGold(tVectorIndex, 3) = 1;
    PlatoTest::checkMultiVectorData(tGradient, tGradientGold);
}

TEST(PlatoTest, NormalDistribution)
{
    double tMean = 1;
    double tStandardDeviation = 0.5;
    Plato::NormalDistribution<double> tDistribution(tMean, tStandardDeviation);

    double tGoldValue = 1;
    double tTolerance = 1e-6;
    EXPECT_NEAR(tGoldValue, tDistribution.mean(), tTolerance);
    tGoldValue = 0.5;
    EXPECT_NEAR(tGoldValue, tDistribution.sigma(), tTolerance);

    // ********* TEST GAUSSIAN PDF AND CDF *********
    double tSample = 0.2;
    tGoldValue = 0.221841669358911;
    EXPECT_NEAR(tGoldValue, tDistribution.pdf(tSample), tTolerance);
    tGoldValue = 0.054799291699558;
    EXPECT_NEAR(tGoldValue, tDistribution.cdf(tSample), tTolerance);

    // ********* TEST GAUSSIAN MOMENTS *********
    size_t tMaxNumMoments = 8;
    std::vector<double> tGoldMoments(tMaxNumMoments + 1u);
    tGoldMoments[0] = 1;
    tGoldMoments[1] = tMean;
    tGoldMoments[2] = std::pow(tMean, 2.) + std::pow(tStandardDeviation, 2.);
    tGoldMoments[3] = std::pow(tMean, 3.) + 3 * tMean * std::pow(tStandardDeviation, 2.);
    tGoldMoments[4] = std::pow(tMean, 4.) + 6 * std::pow(tMean, 2.) * std::pow(tStandardDeviation, 2.)
                      + 3 * std::pow(tStandardDeviation, 4.);
    tGoldMoments[5] = std::pow(tMean, 5.) + 10 * std::pow(tMean, 3.) * std::pow(tStandardDeviation, 2.)
                      + 15 * tMean * std::pow(tStandardDeviation, 4.);
    tGoldMoments[6] = std::pow(tMean, 6.) + 15 * std::pow(tMean, 4.) * std::pow(tStandardDeviation, 2.)
                      + 45 * std::pow(tMean, 2.) * std::pow(tStandardDeviation, 4.) + 15 * std::pow(tStandardDeviation, 6.);
    tGoldMoments[7] = std::pow(tMean, 7.) + 21 * std::pow(tMean, 5.) * std::pow(tStandardDeviation, 2.)
                      + 105 * std::pow(tMean, 3.) * std::pow(tStandardDeviation, 4.)
                      + 105 * tMean * std::pow(tStandardDeviation, 6.);
    tGoldMoments[8] = std::pow(tMean, 8.) + 28 * std::pow(tMean, 6.) * std::pow(tStandardDeviation, 2.)
                      + 210 * std::pow(tMean, 4.) * std::pow(tStandardDeviation, 4.)
                      + 420 * std::pow(tMean, 2.) * std::pow(tStandardDeviation, 6.)
                      + 105 * tMean * std::pow(tStandardDeviation, 8.);
    for(size_t tOrder = 0; tOrder <= tMaxNumMoments; tOrder++)
    {
        double tValue = tDistribution.moment(tOrder);
        EXPECT_NEAR(tGoldMoments[tOrder], tValue, tTolerance);
    }
}

TEST(PlatoTest, CheckSromObjectiveGradient)
{
    // ********* ALLOCATE BETA DISTRIBUTION *********
    const double tMean = 90;
    const double tMax = 135;
    const double tMin = 67.5;
    const double tVariance = 135;
    std::shared_ptr<Plato::BetaDistribution<double>> tDistribution =
            std::make_shared<Plato::BetaDistribution<double>>(tMin, tMax, tMean, tVariance);

    // ********* CHECK OBJECTIVE GRADIENT *********
    std::ostringstream tOutputMsg;
    Plato::Diagnostics<double> tDiagnostics;
    const size_t tMaxNumMoments = 4;
    Plato::SromObjective<double> tObjective(tDistribution, tMaxNumMoments);
    const size_t tNumVectors = 2;
    const size_t tNumControls = 4;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls);
    tDiagnostics.checkCriterionGradient(tObjective, tControl, tOutputMsg);
    EXPECT_TRUE(tDiagnostics.didGradientTestPassed());

    int tMyRank = -1;
    MPI_Comm_rank(MPI_COMM_WORLD, &tMyRank);
    if(tMyRank == static_cast<int>(0))
    {
        std::cout << tOutputMsg.str().c_str();
    }
}

TEST(PlatoTest, CheckSromConstraintGradient)
{
    // ********* CHECK CONSTRAINT GRADIENT *********
    std::shared_ptr<Plato::StandardVectorReductionOperations<double>> tReductions =
            std::make_shared<Plato::StandardVectorReductionOperations<double>>();
    Plato::SromConstraint<double> tConstraint(tReductions);

    const size_t tNumVectors = 2;
    const size_t tNumControls = 4;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls);

    std::ostringstream tOutputMsg;
    Plato::Diagnostics<double> tDiagnostics;
    tDiagnostics.checkCriterionGradient(tConstraint, tControl, tOutputMsg);
    EXPECT_TRUE(tDiagnostics.didGradientTestPassed());

    int tMyRank = -1;
    MPI_Comm_rank(MPI_COMM_WORLD, &tMyRank);
    if(tMyRank == static_cast<int>(0))
    {
        std::cout << tOutputMsg.str().c_str();
    }
}

TEST(PlatoTest, SromOptimizationProblem)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory =
            std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumSamples = 4;
    const size_t tNumControlVectors = 2;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumSamples, tNumControlVectors);

    // ********* ALLOCATE BETA DISTRIBUTION *********
    const double tMean = 90;
    const double tMax = 135;
    const double tMin = 67.5;
    const double tVariance = 135;
    std::shared_ptr<Plato::BetaDistribution<double>> tDistribution =
            std::make_shared<Plato::BetaDistribution<double>>(tMin, tMax, tMean, tVariance);

    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT CRITERIA *********
    const size_t tMaxNumMoments = 4;
    std::shared_ptr<Plato::SromObjective<double>> tSromObjective =
            std::make_shared<Plato::SromObjective<double>>(tDistribution, tMaxNumMoments);
    std::shared_ptr<Plato::ReductionOperations<double>> tReductions = tDataFactory->getControlReductionOperations().create();
    std::shared_ptr<Plato::SromConstraint<double>> tSromConstraint = std::make_shared<Plato::SromConstraint<double>>(tReductions);
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tSromConstraint);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<double>> tStageMng =
            std::make_shared<Plato::AugmentedLagrangianStageMng<double>>(tDataFactory, tSromObjective, tConstraintList);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<double>> tDataMng =
            std::make_shared<Plato::TrustRegionAlgorithmDataMng<double>>(tDataFactory);

    // ********* SET INTIAL GUESS FOR VECTOR OF SAMPLES *********
    Plato::StandardVector<double> tVector(tNumSamples);
    for(size_t tIndex = 0; tIndex < tNumSamples; tIndex++)
    {
        double tValue = (1. / static_cast<double>(tNumSamples + 1u));
        tVector[tIndex] = static_cast<double>(tIndex + 1u) * tValue;
    }
    size_t tVectorIndex = 0;
    tDataMng->setInitialGuess(tVectorIndex, tVector);
    // ********* SET INTIAL GUESS FOR VECTOR OF PROBABILITIES *********
    double tScalarValue = 1. / static_cast<double>(4);
    tVector.fill(tScalarValue);
    tVectorIndex = 1;
    tDataMng->setInitialGuess(tVectorIndex, tVector);
    // ********* SET UPPER AND LOWER BOUNDS *********
    tScalarValue = 0;
    tDataMng->setControlLowerBounds(tScalarValue);
    tScalarValue = 1;
    tDataMng->setControlUpperBounds(tScalarValue);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM *********
    Plato::KelleySachsAugmentedLagrangian<double> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    tAlgorithm.solve();

    size_t tGoldIterations = 38;
    EXPECT_EQ(tGoldIterations, tAlgorithm.getNumIterationsDone());

    // ********* TEST RESULTS *********
    size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tConstraintValues(tNumVectors, tNumDuals);
    tStageMng->getCurrentConstraintValues(tConstraintValues);
    double tTolerance = 1e-5;
    double tGoldValue = -0.00014696838926697708;
    EXPECT_NEAR(tGoldValue, tConstraintValues(0,0), tTolerance);

    Plato::StandardMultiVector<double> tGoldControl(tNumControlVectors, tNumSamples);
    // GOLD SAMPLES
    tGoldControl(0,0) = 0.16377870487690938;
    tGoldControl(0,1) = 0.409813078171542;
    tGoldControl(0,2) = 0.56692565516265081;
    tGoldControl(0,3) = 0.28805773602398665;
    // GOLD PROBABILITIES
    tGoldControl(1,0) = 0.24979311097918996;
    tGoldControl(1,1) = 0.24999286050154013;
    tGoldControl(1,2) = 0.25018122221660277;
    tGoldControl(1,3) = 0.24988583791340019;
    PlatoTest::checkMultiVectorData(tDataMng->getCurrentControl(), tGoldControl);
}

TEST(PlatoTest, solveUncertaintyProblem_checkSize)
{
    // POSE INPUT DISTRIBUTION
    Plato::UncertaintyInputStruct<double, size_t> tInput;
    tInput.mDistribution = Plato::DistrubtionName::type_t::beta;
    tInput.mMean = 1.0;
    tInput.mLowerBound = 0.0;
    tInput.mUpperBound = 2.0;
    tInput.mVariance = 0.5;
    tInput.mNumSamples = 5;

    // SOLVE
    Plato::AlgorithmParamStruct<double, size_t> tParam;
    std::vector<Plato::UncertaintyOutputStruct<double>> tOutput;
    Plato::solve_uncertainty(tInput, tParam, tOutput);

    // CHECK
    EXPECT_EQ(tOutput.size(), tInput.mNumSamples);
}

TEST(PlatoTest, solveUncertaintyProblem_beta)
{
    const double tTol = 1e-6;

    // POSE PROBLEM WITH KNOWN SOLUTION
    Plato::UncertaintyInputStruct<double, size_t> tInput;
    tInput.mDistribution = Plato::DistrubtionName::type_t::beta;
    tInput.mMean = 90.;
    tInput.mUpperBound = 135.;
    tInput.mLowerBound = 67.5;
    tInput.mVariance = 135.;
    tInput.mNumSamples = 4;
    tInput.mMaxNumDistributionMoments = 4;

    // SOLVE
    Plato::AlgorithmParamStruct<double, size_t> tParam;
    std::vector<Plato::UncertaintyOutputStruct<double>> tOutput;
    Plato::solve_uncertainty(tInput, tParam, tOutput);

    // CHECK
    ASSERT_EQ(tOutput.size(), tInput.mNumSamples);
    // GOLD SAMPLES
    EXPECT_NEAR(tOutput[0].mSampleValue, 78.555110973752846, tTol);
    EXPECT_NEAR(tOutput[1].mSampleValue, 95.162418927276619, tTol);
    EXPECT_NEAR(tOutput[2].mSampleValue, 105.76745339378641, tTol);
    EXPECT_NEAR(tOutput[3].mSampleValue, 86.943956034155335, tTol);
    // GOLD PROBABILITIES
    EXPECT_NEAR(tOutput[0].mSampleWeight, 0.2496939761249817, tTol);
    EXPECT_NEAR(tOutput[1].mSampleWeight, 0.24989448275152742, tTol);
    EXPECT_NEAR(tOutput[2].mSampleWeight, 0.25008291182335385, tTol);
    EXPECT_NEAR(tOutput[3].mSampleWeight, 0.24978709886031122, tTol);
    // expect total probability of unity
    const double tTotalProbability = tOutput[0].mSampleWeight + tOutput[1].mSampleWeight + tOutput[2].mSampleWeight
                                     + tOutput[3].mSampleWeight;
    EXPECT_NEAR(tTotalProbability, 0.99945846956017426, tTol);
}

TEST(PlatoTest, solveUncertaintyProblem_uniform)
{
    const double tTol = 1e-6;

    // POSE PROBLEM WITH KNOWN SOLUTION
    Plato::UncertaintyInputStruct<double, size_t> tInput;
    tInput.mDistribution = Plato::DistrubtionName::type_t::uniform;
    tInput.mMean = 0.;
    tInput.mUpperBound = 75.;
    tInput.mLowerBound = 25.;
    tInput.mVariance = 0.;
    tInput.mNumSamples = 5;
    tInput.mMaxNumDistributionMoments = 4;

    // SOLVE
    Plato::AlgorithmParamStruct<double, size_t> tParam;
    std::vector<Plato::UncertaintyOutputStruct<double>> tOutput;
    Plato::solve_uncertainty(tInput, tParam, tOutput);

    // CHECK
    ASSERT_EQ(tOutput.size(), tInput.mNumSamples);
    // GOLD SAMPLES
    EXPECT_NEAR(tOutput[0].mSampleValue, 33.36670189468537, tTol);
    EXPECT_NEAR(tOutput[1].mSampleValue, 41.700254108657575, tTol);
    EXPECT_NEAR(tOutput[2].mSampleValue, 50.033818262841237, tTol);
    EXPECT_NEAR(tOutput[3].mSampleValue, 58.367394569475238, tTol);
    EXPECT_NEAR(tOutput[4].mSampleValue, 66.700983245423743, tTol);
    // GOLD PROBABILITIES
    EXPECT_NEAR(tOutput[0].mSampleWeight, 0.199629021, tTol);
    EXPECT_NEAR(tOutput[1].mSampleWeight, 0.199696349, tTol);
    EXPECT_NEAR(tOutput[2].mSampleWeight, 0.199829875, tTol);
    EXPECT_NEAR(tOutput[3].mSampleWeight, 0.200029615, tTol);
    EXPECT_NEAR(tOutput[4].mSampleWeight, 0.200295595, tTol);
    // expect total probability of unity
    double tTotalProbability = 0;
    for(size_t tIndex = 0; tIndex < tInput.mNumSamples; tIndex++)
    {
        tTotalProbability += tOutput[tIndex].mSampleWeight;
    }
    EXPECT_NEAR(tTotalProbability, 0.99948045716825895, tTol);
}

TEST(PlatoTest, solveUncertaintyProblem_normal)
{
    const double tTol = 1e-6;

    // POSE PROBLEM WITH KNOWN SOLUTION
    Plato::UncertaintyInputStruct<double, size_t> tInput;
    tInput.mDistribution = Plato::DistrubtionName::type_t::normal;
    tInput.mMean = 90.;
    tInput.mUpperBound = 0.;
    tInput.mLowerBound = 0.;
    tInput.mVariance = 45. * 45.;
    tInput.mNumSamples = 6;
    tInput.mMaxNumDistributionMoments = 5;

    // SOLVE
    Plato::AlgorithmParamStruct<double, size_t> tParam;
    std::vector<Plato::UncertaintyOutputStruct<double>> tOutput;
    Plato::solve_uncertainty(tInput, tParam, tOutput);

    // CHECK
    ASSERT_EQ(tOutput.size(), tInput.mNumSamples);
    // GOLD SAMPLES
    EXPECT_NEAR(tOutput[0].mSampleValue, 0.,tTol);//0.14289452, tTol);
    EXPECT_NEAR(tOutput[1].mSampleValue, 0.,tTol);//0.28575572, tTol);
    EXPECT_NEAR(tOutput[2].mSampleValue, 0.,tTol);//0.42861694, tTol);
    EXPECT_NEAR(tOutput[3].mSampleValue, 0.,tTol);//0.57147825, tTol);
    EXPECT_NEAR(tOutput[4].mSampleValue, 0.,tTol);//0.71433961, tTol);
    EXPECT_NEAR(tOutput[5].mSampleValue, 0.,tTol);//0.85720098, tTol);
    // GOLD PROBABILITIES
    EXPECT_NEAR(tOutput[0].mSampleWeight, 0.13995699, tTol);
    EXPECT_NEAR(tOutput[1].mSampleWeight, 0.14407919, tTol);
    EXPECT_NEAR(tOutput[2].mSampleWeight, 0.15301207, tTol);
    EXPECT_NEAR(tOutput[3].mSampleWeight, 0.16684967, tTol);
    EXPECT_NEAR(tOutput[4].mSampleWeight, 0.18573693, tTol);
    EXPECT_NEAR(tOutput[5].mSampleWeight, 0.20986992, tTol);
    // expect total probability of unity
    const double tTotalProbability = tOutput[0].mSampleWeight + tOutput[1].mSampleWeight + tOutput[2].mSampleWeight
                                     + tOutput[3].mSampleWeight
                                     + tOutput[4].mSampleWeight
                                     + tOutput[5].mSampleWeight;
    EXPECT_NEAR(tTotalProbability, 0.99950480285612575, tTol);
}

} //namespace PlatoTest
