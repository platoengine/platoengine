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
 * Plato_Test_Srom.cpp
 *
 *  Created on: Apr 29, 2020
 */

#include "gtest/gtest.h"

#include "Plato_SromXML.hpp"
#include <Plato_UnitTestUtils.hpp>

namespace PlatoTestSrom
{

TEST(PlatoTest, SROM_BuildSrom_Material)
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
    Plato::srom::InputMetaData tInput;
    Plato::srom::OutputMetaData tOutput;
    tInput.materials(tMaterialSet);
    tInput.usecase(Plato::srom::usecase::MATERIAL);
    EXPECT_NO_THROW(Plato::srom::build_sroms(tInput, tOutput));
    ASSERT_EQ(8u, tOutput.materials().size());

    // 4 TEST RESULTS
    const double tTolerance = 1e-4;
    std::vector<std::string> tGoldIDs = { "1", "2", "3", "4", "5", "6", "7", "8" };
    const std::vector<double> tGoldProbs = { 0.1482212, 0.1440202, 0.0924449, 0.0898248, 0.1641386, 0.1594865, 0.1023726, 0.0994711 };
    const std::vector<std::vector<std::string>> tGoldMatIDs =
        { { "1", "2", "3" }, { "1", "2", "3" }, { "1", "2", "3" },  { "1", "2", "3" },
          { "1", "2", "3" }, {"1", "2", "3" }, { "1", "2", "3" }, { "1", "2", "3" } };
    const std::vector<std::vector<std::string>> tGoldBlockIDs =
        { { "1", "2", "3" }, { "1", "2", "3" }, { "1", "2", "3" },  { "1", "2", "3" },
          { "1", "2", "3" }, {"1", "2", "3" }, { "1", "2", "3" }, { "1", "2", "3" } };
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
    auto tMaterialCases = tOutput.materials();
    for (auto &tRandomMatCase : tMaterialCases)
    {
        ASSERT_EQ(3u, tRandomMatCase.numMaterials());
        auto tCaseIndex = &tRandomMatCase - &tMaterialCases[0];
        ASSERT_STREQ(tGoldIDs[tCaseIndex].c_str(), tRandomMatCase.caseID().c_str());
        EXPECT_NEAR(tGoldProbs[tCaseIndex], tRandomMatCase.probability(), tTolerance);
        tProbSum += tRandomMatCase.probability();

        auto tBlockIDs = tRandomMatCase.blockIDs();
        ASSERT_EQ(3u, tBlockIDs.size());
        auto tMaterialIDs = tRandomMatCase.materialIDs();
        ASSERT_EQ(3u, tMaterialIDs.size());
        for (auto &tMatID : tMaterialIDs)
        {
            auto tMatIdIndex = &tMatID - &tMaterialIDs[0];
            ASSERT_STREQ(tGoldBlockIDs[tCaseIndex][tMatIdIndex].c_str(), tBlockIDs[tMatIdIndex].c_str());
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

}
