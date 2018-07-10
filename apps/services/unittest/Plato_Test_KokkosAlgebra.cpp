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
 * Plato_Test_KokkosAlgebra.cpp
 *
 *  Created on: May 5, 2018
 */

#include "gtest/gtest.h"

#include "Plato_UnitTestUtils.hpp"

#include "Plato_KokkosVector.hpp"
#include "Plato_KokkosReductionOperations.hpp"
#include "Plato_DeviceOptimalityCriteriaUpdate.hpp"

namespace PlatoTest
{

TEST(PlatoTest, KokkosVector)
{
    double tBaseValue = 1;
    const size_t tSize = 10;
    std::string tName = "ControlOne";
    Plato::KokkosVector<double> tControlOne(tSize, tBaseValue, tName);

    // ******************** TEST SIZE FUNCTION ********************
    size_t tSizeGold = tSize;
    EXPECT_EQ(tSizeGold, tControlOne.size());

    // ******************** TEST FILL FUNCTION ********************
    double tScalarValue = 4;
    tControlOne.fill(tScalarValue);

    const double tTolerance = 1e-6;
    std::vector<double> tGoldData = {4, 4, 4, 4, 4, 4, 4, 4, 4, 4};
    auto tHostData = create_mirror(tControlOne.view());
    Kokkos::deep_copy(tHostData, tControlOne.view());
    for(size_t tIndex = 0; tIndex < tSize; tIndex++)
    {
        EXPECT_NEAR(tHostData(tIndex), tGoldData[tIndex], tTolerance);
    }

    // ******************** TEST SCALE FUNCTION ********************
    tScalarValue = 2;
    tControlOne.scale(tScalarValue);

    tGoldData = {8, 8, 8, 8, 8, 8, 8, 8, 8, 8};
    for(size_t tIndex = 0; tIndex < tSize; tIndex++)
    {
        EXPECT_NEAR(tControlOne.vector()[tIndex], tGoldData[tIndex], tTolerance);
    }

    // ******************** TEST MODULUS FUNCTION ********************
    tScalarValue = -1;
    tControlOne.scale(tScalarValue);

    tGoldData = {-8, -8, -8, -8, -8, -8, -8, -8, -8, -8};
    tHostData = create_mirror(tControlOne.view());
    Kokkos::deep_copy(tHostData, tControlOne.view());
    for(size_t tIndex = 0; tIndex < tSize; tIndex++)
    {
        EXPECT_NEAR(tHostData(tIndex), tGoldData[tIndex], tTolerance);
    }

    tControlOne.modulus();

    tGoldData = {8, 8, 8, 8, 8, 8, 8, 8, 8, 8};
    tHostData = create_mirror(tControlOne.view());
    Kokkos::deep_copy(tHostData, tControlOne.view());
    for(size_t tIndex = 0; tIndex < tSize; tIndex++)
    {
        EXPECT_NEAR(tHostData(tIndex), tGoldData[tIndex], tTolerance);
    }

    // ******************** TEST UPDATE FUNCTION ********************
    tScalarValue = 1;
    tControlOne.fill(tScalarValue);
    std::shared_ptr<Plato::Vector<double>> tControlTwo = tControlOne.create(); // TEST CREATE FUNCTION
    tControlTwo->fill(tScalarValue);

    const double tAlpha = 1.0;
    const double tBeta = 2.0;
    tControlOne.update(tAlpha, tControlTwo.operator*(), tBeta);

    tGoldData = {3, 3, 3, 3, 3, 3, 3, 3, 3, 3};
    for(size_t tIndex = 0; tIndex < tSize; tIndex++)
    {
        EXPECT_NEAR(tControlOne.vector()[tIndex], tGoldData[tIndex], tTolerance);
    }

    // ******************** TEST DOT FUNCTION ********************
    double tDotValue = tControlOne.dot(tControlTwo.operator*());
    double tGoldValue = 30;
    EXPECT_NEAR(tDotValue, tGoldValue, tTolerance);

    // ******************** TEST OPERATOR[] FUNCTION ********************
    tScalarValue = 11.0;
    tControlOne.fill(tScalarValue);

    tControlOne.host(); // FIRST, BRING DATA FROM DEVICE TO HOST
    tGoldData = {11, 11, 11, 11, 11, 11, 11, 11, 11, 11};
    for(size_t tIndex = 0; tIndex < tSize; tIndex++)
    {
        EXPECT_NEAR(tControlOne[tIndex], tGoldData[tIndex], tTolerance);
    }
}

TEST(PlatoTest, KokkosReductionOperations)
{
    std::vector<double> tHostData = {1, 2, 3, 4, 5, 23, 6, 7, 8, 9, 10};
    Plato::KokkosVector<double> tVector(tHostData, "KokkosTestVector");
    const size_t tSizeGold = 11;
    EXPECT_EQ(tSizeGold, tVector.size());

    // ******************** TEST SUM FUNCTION ********************
    Plato::KokkosReductionOperations<double> tReduction;
    double tSum = tReduction.sum(tVector);

    double tGoldValue = 78;
    const double tTolerance = 1e-6;
    EXPECT_NEAR(tSum, tGoldValue, tTolerance);

    // ******************** TEST MAX FUNCTION ********************
    tGoldValue = 23;
    double tMax = tReduction.max(tVector);
    EXPECT_NEAR(tMax, tGoldValue, tTolerance);

    // ******************** TEST MIN FUNCTION ********************
    tGoldValue = 1;
    double tMin = tReduction.min(tVector);
    EXPECT_NEAR(tMin, tGoldValue, tTolerance);

    // ******************** TEST CREATE FUNCTION ********************
    std::shared_ptr<Plato::ReductionOperations<double>> tReductionCopy = tReduction.create();
    assert(tReductionCopy != nullptr);

    tGoldValue = 78;
    tSum = tReductionCopy->sum(tVector);
    EXPECT_NEAR(tSum, tGoldValue, tTolerance);

    tGoldValue = 23;
    tMax = tReductionCopy->max(tVector);
    EXPECT_NEAR(tMax, tGoldValue, tTolerance);

    tGoldValue = 1;
    tMin = tReductionCopy->min(tVector);
    EXPECT_NEAR(tMin, tGoldValue, tTolerance);
}

TEST(PlatoTest, KokkosNonlinearProgrammingUpdate)
{
    // ********* Allocate Data for Unit Test *********
    const size_t tNumControls = 10;
    Plato::KokkosVector<double> tControl(tNumControls);

    double tValue = 0.25;
    Plato::KokkosVector<double> tPrevControl(tNumControls, tValue);

    tValue = 1e-3;
    Plato::KokkosVector<double> tLowerBounds(tNumControls, tValue);

    tValue = 1;
    Plato::KokkosVector<double> tUpperBounds(tNumControls, tValue);

    tValue = 1e-5;
    Plato::KokkosVector<double> tObjGradient(tNumControls, tValue);

    tValue = 5e-6;
    Plato::KokkosVector<double> tObjInequality(tNumControls, tValue);

    const double tTrialDual = 0.5;

    // ********* Allocate Update Functor *********
    double tMoveLimit = 1e-1;
    double tScaleFactor = 0.01;
    double tDampingPower = 0.5;
    Plato::DeviceOptimalityCriteriaUpdate<double> tUpdate(tMoveLimit, tScaleFactor, tDampingPower);

    // ********* Call Update Functor *********
    tUpdate.update(tTrialDual, tLowerBounds, tUpperBounds, tPrevControl, tObjGradient, tObjInequality, tControl);

    // ********* Test Results *********
    tValue = 0.15;
    Plato::KokkosVector<double> tControlGold(tNumControls, tValue);

    // ********* Bring Data from Device to Host *********
    tControl.host();
    tControlGold.host();
    PlatoTest::checkVectorData(tControl, tControlGold);
}

} // namespace PlatoTest
