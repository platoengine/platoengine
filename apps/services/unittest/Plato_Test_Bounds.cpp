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
 * Plato_BoundsTest.cpp
 *
 *  Created on: Oct 21, 2017
 */

#include "gtest/gtest.h"

#include "Plato_UnitTestUtils.hpp"

#include "Plato_HostBounds.hpp"
#include "Plato_ErrorChecks.hpp"
#include "Plato_DeviceBounds.hpp"
#include "Plato_KokkosVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_StandardMultiVector.hpp"

namespace PlatoTest
{

TEST(PlatoTest, DeviceProject)
{
    // ********* Allocate Input Data *********
    const size_t tNumVectors = 8;
    std::vector<double> tVectorGold = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::KokkosVector<double> tVector(tVectorGold);

    Plato::StandardMultiVector<double> tData(tNumVectors, tVector);
    for(size_t tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
    {
        tData[tVectorIndex].update(1., tVector, 0.);
    }

    // ********* Allocate Lower & Upper Bounds *********
    Plato::StandardMultiVector<double> tLowerBounds(tNumVectors, tVector);
    const double tLowerBoundValue = 2;
    Plato::fill(tLowerBoundValue, tLowerBounds);

    Plato::StandardMultiVector<double> tUpperBounds(tNumVectors, tVector);
    const double tUpperBoundValue = 7;
    Plato::fill(tUpperBoundValue, tUpperBounds);

    // ********* Call Project *********
    Plato::DeviceBounds<double> tBounds;
    tBounds.project(tLowerBounds, tUpperBounds, tData);

    // ********* Check Results *********
    std::vector<double> tVectorBoundsGold = { 2, 2, 3, 4, 5, 6, 7, 7, 7, 7 };
    Plato::KokkosVector<double> tBoundVector(tVectorBoundsGold);
    Plato::StandardMultiVector<double> tGoldData(tNumVectors, tBoundVector);
    for(size_t tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
    {
        tGoldData[tVectorIndex].update(1., tBoundVector, 0.);
    }
    PlatoTest::checkMultiVectorData(tData, tGoldData);
}


TEST(PlatoTest, DeviceComputeActiveAndInactiveSet)
{
    // ********* Allocate Input Data *********
    const size_t tNumVectors = 4;
    std::vector<double> tVectorGold = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::KokkosVector<double> tVector(tVectorGold);
    // Default for second template typename is OrdinalType = size_t
    Plato::StandardMultiVector<double> tControl(tNumVectors, tVector);
    for(size_t tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
    {
        tControl[tVectorIndex].update(1., tVector, 0.);
    }

    // ********* Allocate Lower & Upper Bounds *********
    Plato::StandardMultiVector<double> tLowerBounds(tNumVectors, tVector);
    const double tLowerBoundValue = 2;
    Plato::fill(tLowerBoundValue, tLowerBounds);

    Plato::StandardMultiVector<double> tUpperBounds(tNumVectors, tVector);
    const double tUpperBoundValue = 7;
    Plato::fill(tUpperBoundValue, tUpperBounds);

    // ********* Allocate Active & Inactive Sets *********
    Plato::StandardMultiVector<double> tActiveSet(tNumVectors, tVector);
    Plato::StandardMultiVector<double> tInactiveSet(tNumVectors, tVector);

    // ********* Compute Active & Inactive Sets *********
    Plato::DeviceBounds<double> tBounds;
    tBounds.project(tLowerBounds, tUpperBounds, tControl);
    tBounds.computeActiveAndInactiveSets(tControl, tLowerBounds, tUpperBounds, tActiveSet, tInactiveSet);

    std::vector<double> tActiveSetGoldData = { 1, 1, 0, 0, 0, 0, 1, 1, 1, 1 };
    std::vector<double> tInactiveSetGoldData = { 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 };
    Plato::KokkosVector<double> tActiveSetGoldVec(tActiveSetGoldData);
    Plato::KokkosVector<double> tInactiveSetGoldVec(tInactiveSetGoldData);
    Plato::StandardMultiVector<double> tActiveSetGoldMV(tNumVectors, tActiveSetGoldVec);
    Plato::StandardMultiVector<double> tInactiveSetGoldMV(tNumVectors, tInactiveSetGoldVec);
    for(size_t tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
    {
        tActiveSetGoldMV[tVectorIndex].update(1., tActiveSetGoldVec, 0.);
        tInactiveSetGoldMV[tVectorIndex].update(1., tInactiveSetGoldVec, 0.);
    }
    PlatoTest::checkMultiVectorData(tActiveSet, tActiveSetGoldMV);
    PlatoTest::checkMultiVectorData(tInactiveSet, tInactiveSetGoldMV);
}

TEST(PlatoTest, HostProject)
{
    // ********* Allocate Input Data *********
    const size_t tNumVectors = 8;
    std::vector<double> tVectorGold = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::StandardVector<double> tVector(tVectorGold);
    // Default for second template typename is OrdinalType = size_t
    Plato::StandardMultiVector<double> tData(tNumVectors, tVector);
    for(size_t tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
    {
        tData[tVectorIndex].update(1., tVector, 0.);
    }

    // ********* Allocate Lower & Upper Bounds *********
    const double tLowerBoundValue = 2;
    const size_t tNumElementsPerVector = tVectorGold.size();
    Plato::StandardMultiVector<double> tLowerBounds(tNumVectors, tNumElementsPerVector, tLowerBoundValue);
    const double tUpperBoundValue = 7;
    Plato::StandardMultiVector<double> tUpperBounds(tNumVectors, tNumElementsPerVector, tUpperBoundValue);

    Plato::HostBounds<double> tBounds;
    tBounds.project(tLowerBounds, tUpperBounds, tData);

    std::vector<double> tVectorBoundsGold = { 2, 2, 3, 4, 5, 6, 7, 7, 7, 7 };
    Plato::StandardVector<double> tPlatoBoundVector(tVectorBoundsGold);
    Plato::StandardMultiVector<double> tGoldData(tNumVectors, tPlatoBoundVector);
    for(size_t tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
    {
        tGoldData[tVectorIndex].update(1., tPlatoBoundVector, 0.);
    }
    PlatoTest::checkMultiVectorData(tData, tGoldData);
}

TEST(PlatoTest, CheckInitialGuessIsSet)
{
    bool tIsSet = true;
    ASSERT_NO_THROW(Plato::error::checkInitialGuessIsSet(tIsSet));

    tIsSet = false;
    ASSERT_THROW(Plato::error::checkInitialGuessIsSet(tIsSet), std::invalid_argument);
}

TEST(PlatoTest, CheckBounds)
{
    // ********* TEST LOWER BOUND GREATER THAN UPPER BOUND ASSERTION *********
    size_t tNumVectors = 1;
    size_t tNumElements = 5;
    double tLowerBoundValue = 2;
    Plato::StandardMultiVector<double> tLowerBoundOne(tNumVectors, tNumElements, tLowerBoundValue);
    double tUpperBoundValue = 7;
    Plato::StandardMultiVector<double> tUpperBoundOne(tNumVectors, tNumElements, tUpperBoundValue);
    ASSERT_NO_THROW(Plato::error::checkBounds(tLowerBoundOne, tUpperBoundOne));

    tUpperBoundValue = 1;
    Plato::fill(tUpperBoundValue, tUpperBoundOne);
    ASSERT_THROW(Plato::error::checkBounds(tLowerBoundOne, tUpperBoundOne), std::invalid_argument);

    // ********* TEST NUMBER OF VECTORS ASSERTION *********
    tNumVectors = 2;
    Plato::StandardMultiVector<double> tLowerBoundTwo(tNumVectors, tNumElements, tLowerBoundValue);
    ASSERT_THROW(Plato::error::checkBounds(tLowerBoundTwo, tUpperBoundOne), std::invalid_argument);

    // ********* TEST VECTOR LENGTH ASSERTION *********
    tNumVectors = 1;
    tNumElements = 4;
    Plato::StandardMultiVector<double> tLowerBoundThree(tNumVectors, tNumElements, tLowerBoundValue);
    ASSERT_THROW(Plato::error::checkBounds(tLowerBoundThree, tUpperBoundOne), std::invalid_argument);
}

TEST(PlatoTest, CheckInitialGuess)
{
    // ********* TEST LOWER BOUND GREATER THAN UPPER BOUND ASSERTION *********
    size_t tNumVectors = 1;
    size_t tNumElements = 5;
    double tControlValue = 3;
    Plato::StandardMultiVector<double> tControls(tNumVectors, tNumElements, tControlValue);
    double tLowerBoundValue = 2;
    Plato::StandardMultiVector<double> tLowerBoundOne(tNumVectors, tNumElements, tLowerBoundValue);
    double tUpperBoundValue = 7;
    Plato::StandardMultiVector<double> tUpperBoundOne(tNumVectors, tNumElements, tUpperBoundValue);
    ASSERT_NO_THROW(Plato::error::checkInitialGuess(tControls, tLowerBoundOne, tUpperBoundOne));

    tControlValue = 7;
    Plato::fill(tControlValue, tControls);
    ASSERT_NO_THROW(Plato::error::checkInitialGuess(tControls, tLowerBoundOne, tUpperBoundOne));

    tControlValue = 2;
    Plato::fill(tControlValue, tControls);
    ASSERT_NO_THROW(Plato::error::checkInitialGuess(tControls, tLowerBoundOne, tUpperBoundOne));

    // ********* TEST NUMBER OF VECTORS ASSERTION *********
    tControlValue = 3;
    Plato::fill(tControlValue, tControls);
    tNumVectors = 2;
    Plato::StandardMultiVector<double> tLowerBoundTwo(tNumVectors, tNumElements, tLowerBoundValue);
    ASSERT_THROW(Plato::error::checkInitialGuess(tControls, tLowerBoundTwo, tUpperBoundOne), std::invalid_argument);

    Plato::StandardMultiVector<double> tUpperBoundTwo(tNumVectors, tNumElements, tUpperBoundValue);
    ASSERT_THROW(Plato::error::checkInitialGuess(tControls, tLowerBoundTwo, tUpperBoundTwo), std::invalid_argument);

    // ********* TEST VECTOR LENGTH ASSERTION *********
    tNumVectors = 1;
    tNumElements = 4;
    Plato::StandardMultiVector<double> tLowerBoundThree(tNumVectors, tNumElements, tLowerBoundValue);
    ASSERT_THROW(Plato::error::checkInitialGuess(tControls, tLowerBoundThree, tUpperBoundOne), std::invalid_argument);
}

TEST(PlatoTest, HostComputeActiveAndInactiveSet)
{
    // ********* Allocate Input Data *********
    const size_t tNumVectors = 4;
    std::vector<double> tVectorGold = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::StandardVector<double> tVector(tVectorGold);
    // Default for second template typename is OrdinalType = size_t
    Plato::StandardMultiVector<double> tData(tNumVectors, tVector);
    for(size_t tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
    {
        tData[tVectorIndex].update(1., tVector, 0.);
    }

    // ********* Allocate Lower & Upper Bounds *********
    const double tLowerBoundValue = 2;
    const size_t tNumElementsPerVector = tVectorGold.size();
    Plato::StandardMultiVector<double> tLowerBounds(tNumVectors, tNumElementsPerVector, tLowerBoundValue);
    const double tUpperBoundValue = 7;
    Plato::StandardMultiVector<double> tUpperBounds(tNumVectors, tNumElementsPerVector, tUpperBoundValue);

    // ********* Allocate Active & Inactive Sets *********
    Plato::StandardMultiVector<double> tActiveSet(tNumVectors, tNumElementsPerVector, tUpperBoundValue);
    Plato::StandardMultiVector<double> tInactiveSet(tNumVectors, tNumElementsPerVector, tUpperBoundValue);

    // ********* Compute Active & Inactive Sets *********
    Plato::HostBounds<double> tBounds;
    tBounds.project(tLowerBounds, tUpperBounds, tData);
    tBounds.computeActiveAndInactiveSets(tData, tLowerBounds, tUpperBounds, tActiveSet, tInactiveSet);

    std::vector<double> tActiveSetGold = { 1, 1, 0, 0, 0, 0, 1, 1, 1, 1 };
    std::vector<double> tInactiveSetGold = { 0, 0, 1, 1, 1, 1, 0, 0, 0, 0 };
    Plato::StandardVector<double> tPlatoActiveSetVectorGold(tActiveSetGold);
    Plato::StandardVector<double> tPlatoInactiveSetVectorGold(tInactiveSetGold);
    Plato::StandardMultiVector<double> tActiveSetGoldData(tNumVectors, tPlatoActiveSetVectorGold);
    Plato::StandardMultiVector<double> tInactiveSetGoldData(tNumVectors, tPlatoInactiveSetVectorGold);
    for(size_t tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
    {
        tActiveSetGoldData[tVectorIndex].update(1., tPlatoActiveSetVectorGold, 0.);
        tInactiveSetGoldData[tVectorIndex].update(1., tPlatoInactiveSetVectorGold, 0.);
    }
    PlatoTest::checkMultiVectorData(tActiveSet, tActiveSetGoldData);
    PlatoTest::checkMultiVectorData(tInactiveSet, tInactiveSetGoldData);
}

} // namespace PlatoTest
