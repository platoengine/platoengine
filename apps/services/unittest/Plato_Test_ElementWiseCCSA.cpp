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
 * Plato_Test_ElementWiseCCSA.cpp
 *
 *  Created on: May 11, 2018
 */

#include "gtest/gtest.h"

#include "Plato_UnitTestUtils.hpp"

#include "Plato_KokkosVector.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_HostDualProbElementWise.hpp"
#include "Plato_HostPrimalProbElementWise.hpp"
#include "Plato_DeviceDualProbElementWise.hpp"
#include "Plato_DevicePrimalProbElementWise.hpp"

namespace PlatoTest
{

TEST(PlatoTest, HostPrimalProbUpdateSigmaCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1;
    const size_t tSize = 10;
    Plato::StandardVector<double> tCurrentControl(tSize, tBaseValue);

    tBaseValue = 2;
    Plato::StandardVector<double> tPreviousControl(tSize, tBaseValue);

    tBaseValue = 0.5;
    Plato::StandardVector<double> tAntepenultimateControls(tSize, tBaseValue);

    tBaseValue = 1.0;
    Plato::StandardVector<double> tUpperBounds(tSize, tBaseValue);

    tBaseValue = 0.0;
    Plato::StandardVector<double> tLowerBounds(tSize, tBaseValue);

    tBaseValue = 0.1;
    Plato::StandardVector<double> tPreviousSigma(tSize, tBaseValue);

    tBaseValue = 0.0;
    Plato::StandardVector<double> tCurrentSigma(tSize, tBaseValue);

    std::map<Plato::element_wise::constant_t, double> tConstants;
    tConstants[Plato::element_wise::ExpansionFactor] = 2;
    tConstants[Plato::element_wise::ContractionFactor] = 0.5;
    tConstants[Plato::element_wise::UpperBoundScaleFactor] = 10.0;
    tConstants[Plato::element_wise::LowerBoundScaleFactor] = 0.01;

    // ************ Allocate instance with element wise functions ************
    Plato::HostPrimalProbElementWise<double> tHostPrimalProbElementWise;
    tHostPrimalProbElementWise.updateSigmaCoeff(tConstants,
                                                tCurrentControl,
                                                tPreviousControl,
                                                tAntepenultimateControls,
                                                tUpperBounds,
                                                tLowerBounds,
                                                tPreviousSigma,
                                                tCurrentSigma);

    // ************ Test output = CurrentSigma from updateSigmaCoeff ************
    tBaseValue = 0.05;
    Plato::StandardVector<double> tGold(tSize, tBaseValue);
    PlatoTest::checkVectorData(tCurrentSigma, tGold);
}

TEST(PlatoTest, HostPrimalProbUpdateObjectiveGlobalizationCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1;
    const size_t tSize = 10;
    Plato::StandardVector<double> tDeltaControl(tSize, tBaseValue);

    tBaseValue = 2;
    Plato::StandardVector<double> tCurrentSigma(tSize, tBaseValue);

    tBaseValue = 0.05;
    Plato::StandardVector<double> tCurrentGradient(tSize, tBaseValue);

    tBaseValue = 0.0;
    Plato::StandardVector<double> tOutputOne(tSize, tBaseValue);
    Plato::StandardVector<double> tOutputTwo(tSize, tBaseValue);

    // ************ Allocate instance with element wise functions ************
    Plato::HostPrimalProbElementWise<double> tHostPrimalProbElementWise;
    tHostPrimalProbElementWise.updateObjectiveGlobalizationCoeff(tDeltaControl,
                                                                 tCurrentSigma,
                                                                 tCurrentGradient,
                                                                 tOutputOne,
                                                                 tOutputTwo);

    // ************ Test outputs = tOutputOne & tOutputTwo from updateObjectiveGlobalizationCoeff ************
    tBaseValue = 1.0 / 3.0;
    Plato::StandardVector<double> tGoldOne(tSize, tBaseValue);
    PlatoTest::checkVectorData(tOutputOne, tGoldOne);
    tBaseValue = 0.1;
    Plato::StandardVector<double> tGoldTwo(tSize, tBaseValue);
    PlatoTest::checkVectorData(tOutputTwo, tGoldTwo);
}

TEST(PlatoTest, HostPrimalProbUpdateConstraintGlobalizationCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1;
    const size_t tSize = 10;
    Plato::StandardVector<double> tDeltaControl(tSize, tBaseValue);

    tBaseValue = 2;
    Plato::StandardVector<double> tCurrentSigma(tSize, tBaseValue);

    tBaseValue = 0.05;
    Plato::StandardVector<double> tCurrentGradient(tSize, tBaseValue);

    tBaseValue = 0.0;
    Plato::StandardVector<double> tOutputOne(tSize, tBaseValue);
    Plato::StandardVector<double> tOutputTwo(tSize, tBaseValue);

    // ************ Allocate instance with element wise functions ************
    Plato::HostPrimalProbElementWise<double> tHostPrimalProbElementWise;
    tHostPrimalProbElementWise.updateConstraintGlobalizationCoeff(tDeltaControl,
                                                                  tCurrentSigma,
                                                                  tCurrentGradient,
                                                                  tOutputOne,
                                                                  tOutputTwo);

    // ************ Test outputs = tOutputOne & tOutputTwo from updateObjectiveGlobalizationCoeff ************
    tBaseValue = 1.0 / 3.0;
    Plato::StandardVector<double> tGoldOne(tSize, tBaseValue);
    PlatoTest::checkVectorData(tOutputOne, tGoldOne);
    tBaseValue = 0.1;
    Plato::StandardVector<double> tGoldTwo(tSize, tBaseValue);
    PlatoTest::checkVectorData(tOutputTwo, tGoldTwo);
}

TEST(PlatoTest, HostPrimalProbComputeKarushKuhnTuckerObjectiveConditions)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1;
    const size_t tSize = 10;
    Plato::StandardVector<double> tControl(tSize, tBaseValue);

    tBaseValue = 5e-1;
    Plato::StandardVector<double> tObjectiveGradient(tSize, tBaseValue);

    tBaseValue = 1e-1;
    Plato::StandardVector<double> tConstraintGradientTimesDual(tSize, tBaseValue);

    tBaseValue = 0.0;
    Plato::StandardVector<double> tOutputOne(tSize, tBaseValue);
    Plato::StandardVector<double> tOutputTwo(tSize, tBaseValue);

    // ************ Allocate instance with element wise functions ************
    Plato::HostPrimalProbElementWise<double> tHostPrimalProbElementWise;
    tHostPrimalProbElementWise.computeKarushKuhnTuckerObjectiveConditions(tControl,
                                                                          tObjectiveGradient,
                                                                          tConstraintGradientTimesDual,
                                                                          tOutputOne,
                                                                          tOutputTwo);

    // ************ Test outputs = tOutputOne & tOutputTwo from updateObjectiveGlobalizationCoeff ************
    tBaseValue = 1.44;
    Plato::StandardVector<double> tGoldOne(tSize, tBaseValue);
    PlatoTest::checkVectorData(tOutputOne, tGoldOne);
    tBaseValue = 0.0;
    Plato::StandardVector<double> tGoldTwo(tSize, tBaseValue);
    PlatoTest::checkVectorData(tOutputTwo, tGoldTwo);
}

TEST(PlatoTest, HostDualProbUpdateTrialControl)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1e-1;
    const size_t tSize = 10;
    Plato::StandardVector<double> tTermA(tSize, tBaseValue);

    tBaseValue = 5e-1;
    Plato::StandardVector<double> tTermB(tSize, tBaseValue);

    tBaseValue = 0.0;
    Plato::StandardVector<double> tLowerBounds(tSize, tBaseValue);

    tBaseValue = 1e3;
    Plato::StandardVector<double> tUpperBounds(tSize, tBaseValue);

    tBaseValue = 0.01;
    Plato::StandardVector<double> tLowerAsymptotes(tSize, tBaseValue);

    tBaseValue = 10.0;
    Plato::StandardVector<double> tUpperAsymptotes(tSize, tBaseValue);

    Plato::StandardVector<double> tOutput(tSize);

    // ************ Allocate instance with element wise functions ************
    Plato::HostDualProbElementWise<double> tHostPrimalProbElementWise;
    tHostPrimalProbElementWise.updateTrialControl(tTermA,
                                                  tTermB,
                                                  tLowerBounds,
                                                  tUpperBounds,
                                                  tLowerAsymptotes,
                                                  tUpperAsymptotes,
                                                  tOutput);

    // ************ Test output from updateTrialControl ************
    tBaseValue = 6.9129202261942755;
    Plato::StandardVector<double> tGold(tSize, tBaseValue);
    PlatoTest::checkVectorData(tOutput, tGold);
}

TEST(PlatoTest, HostDualProbUpdateGradientCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1.0;
    const size_t tSize = 10;
    Plato::StandardVector<double> tTrialControls(tSize, tBaseValue);

    tBaseValue = 0.01;
    Plato::StandardVector<double> tLowerAsymptotes(tSize, tBaseValue);

    tBaseValue = 10.0;
    Plato::StandardVector<double> tUpperAsymptotes(tSize, tBaseValue);

    tBaseValue = 1e-2;
    Plato::StandardVector<double> tPcoeff(tSize, tBaseValue);

    tBaseValue = -1e-2;
    Plato::StandardVector<double> tQCoeff(tSize, tBaseValue);

    Plato::StandardVector<double> tOutputOne(tSize);
    Plato::StandardVector<double> tOutputTwo(tSize);

    // ************ Allocate instance with element wise functions ************
    Plato::HostDualProbElementWise<double> tHostPrimalProbElementWise;
    tHostPrimalProbElementWise.updateGradientCoeff(tTrialControls,
                                                   tLowerAsymptotes,
                                                   tUpperAsymptotes,
                                                   tPcoeff,
                                                   tQCoeff,
                                                   tOutputOne,
                                                   tOutputTwo);

    // ************ Test output from updateGradientCoeff ************
    tBaseValue = 0.0011111111111111;
    Plato::StandardVector<double> tGoldOne(tSize, tBaseValue);
    PlatoTest::checkVectorData(tOutputOne, tGoldOne);
    tBaseValue = -0.0101010101010101;
    Plato::StandardVector<double> tGoldTwo(tSize, tBaseValue);
    PlatoTest::checkVectorData(tOutputTwo, tGoldTwo);
}

TEST(PlatoTest, HostDualProbUpdateObjectiveCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1.0;
    const size_t tSize = 10;
    Plato::StandardVector<double> tCurrentSigma(tSize, tBaseValue);

    tBaseValue = -5e-4;
    Plato::StandardVector<double> tCurrentGradient(tSize, tBaseValue);

    Plato::StandardVector<double> tPcoeff(tSize);
    Plato::StandardVector<double> tQcoeff(tSize);
    Plato::StandardVector<double> tRcoeff(tSize);

    // ************ Allocate instance with element wise functions ************
    const double tGlobalizationFactor = 0.5;
    Plato::HostDualProbElementWise<double> tHostPrimalProbElementWise;
    tHostPrimalProbElementWise.updateObjectiveCoeff(tGlobalizationFactor,
                                                    tCurrentSigma,
                                                    tCurrentGradient,
                                                    tPcoeff,
                                                    tQcoeff,
                                                    tRcoeff);

    // ************ Test outputs from updateObjectiveCoeff ************
    tBaseValue = 0.125;
    Plato::StandardVector<double> tGoldOne(tSize, tBaseValue);
    PlatoTest::checkVectorData(tPcoeff, tGoldOne);
    tBaseValue = 0.1255;
    Plato::StandardVector<double> tGoldTwo(tSize, tBaseValue);
    PlatoTest::checkVectorData(tQcoeff, tGoldTwo);
    tBaseValue = 0.2505;
    Plato::StandardVector<double> tGoldThree(tSize, tBaseValue);
    PlatoTest::checkVectorData(tRcoeff, tGoldThree);
}

TEST(PlatoTest, HostDualProbUpdateConstraintCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1.0;
    const size_t tSize = 10;
    Plato::StandardVector<double> tCurrentSigma(tSize, tBaseValue);

    tBaseValue = -5e-4;
    Plato::StandardVector<double> tCurrentGradient(tSize, tBaseValue);

    Plato::StandardVector<double> tPcoeff(tSize);
    Plato::StandardVector<double> tQcoeff(tSize);
    Plato::StandardVector<double> tRcoeff(tSize);

    // ************ Allocate instance with element wise functions ************
    const double tGlobalizationFactor = 0.5;
    Plato::HostDualProbElementWise<double> tHostPrimalProbElementWise;
    tHostPrimalProbElementWise.updateConstraintCoeff(tGlobalizationFactor,
                                                     tCurrentSigma,
                                                     tCurrentGradient,
                                                     tPcoeff,
                                                     tQcoeff,
                                                     tRcoeff);

    // ************ Test outputs from updateConstraintCoeff ************
    tBaseValue = 0.125;
    Plato::StandardVector<double> tGoldOne(tSize, tBaseValue);
    PlatoTest::checkVectorData(tPcoeff, tGoldOne);
    tBaseValue = 0.1255;
    Plato::StandardVector<double> tGoldTwo(tSize, tBaseValue);
    PlatoTest::checkVectorData(tQcoeff, tGoldTwo);
    tBaseValue = 0.2505;
    Plato::StandardVector<double> tGoldThree(tSize, tBaseValue);
    PlatoTest::checkVectorData(tRcoeff, tGoldThree);
}

TEST(PlatoTest, HostDualProbUpdateMovingAsymptotesCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 0.5;
    const size_t tSize = 10;
    Plato::StandardVector<double> tTrialControls(tSize, tBaseValue);

    tBaseValue = 1e-1;
    Plato::StandardVector<double> tLowerAsymptotes(tSize, tBaseValue);

    tBaseValue = 1e-1;
    Plato::StandardVector<double> tUpperAsymptotes(tSize, tBaseValue);

    tBaseValue = 5e-1;
    Plato::StandardVector<double> tObjectivePcoeff(tSize, tBaseValue);

    tBaseValue = -2.5e-1;
    Plato::StandardVector<double> tObjectiveQcoeff(tSize, tBaseValue);

    tBaseValue = 1e-2;
    Plato::StandardVector<double> tDualTimesConstraintPcoeff(tSize, tBaseValue);

    tBaseValue = -2.5e-2;
    Plato::StandardVector<double> tDualTimesConstraintQcoeff(tSize, tBaseValue);

    Plato::StandardVector<double> tOutputOne(tSize);
    Plato::StandardVector<double> tOutputTwo(tSize);

    // ************ Allocate instance with element wise functions ************
    Plato::HostDualProbElementWise<double> tHostPrimalProbElementWise;
    tHostPrimalProbElementWise.updateMovingAsymptotesCoeff(tTrialControls,
                                                           tLowerAsymptotes,
                                                           tUpperAsymptotes,
                                                           tObjectivePcoeff,
                                                           tObjectiveQcoeff,
                                                           tDualTimesConstraintPcoeff,
                                                           tDualTimesConstraintQcoeff,
                                                           tOutputOne,
                                                           tOutputTwo);

    // ************ Test outputs from updateMovingAsymptotesCoeff ************
    tBaseValue = -1.275;
    Plato::StandardVector<double> tGoldOne(tSize, tBaseValue);
    PlatoTest::checkVectorData(tOutputOne, tGoldOne);
    tBaseValue = -0.6875;
    Plato::StandardVector<double> tGoldTwo(tSize, tBaseValue);
    PlatoTest::checkVectorData(tOutputTwo, tGoldTwo);
}

TEST(PlatoTest, DevicePrimalProbUpdateSigmaCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1;
    const size_t tSize = 10;
    std::string tName = "CurrentControl";
    Plato::KokkosVector<double> tCurrentControl(tSize, tBaseValue, tName);

    tBaseValue = 2;
    tName = "PreviousControl";
    Plato::KokkosVector<double> tPreviousControl(tSize, tBaseValue, tName);

    tBaseValue = 0.5;
    tName = "AntepenultimateControl";
    Plato::KokkosVector<double> tAntepenultimateControls(tSize, tBaseValue, tName);

    tBaseValue = 1.0;
    tName = "UpperBounds";
    Plato::KokkosVector<double> tUpperBounds(tSize, tBaseValue, tName);

    tBaseValue = 0.0;
    tName = "LowerBounds";
    Plato::KokkosVector<double> tLowerBounds(tSize, tBaseValue, tName);

    tBaseValue = 0.1;
    tName = "PreviousSigma";
    Plato::KokkosVector<double> tPreviousSigma(tSize, tBaseValue, tName);

    tBaseValue = 0.0;
    tName = "CurrentSigma";
    Plato::KokkosVector<double> tCurrentSigma(tSize, tBaseValue, tName);

    std::map<Plato::element_wise::constant_t, double> tConstants;
    tConstants[Plato::element_wise::ExpansionFactor] = 2;
    tConstants[Plato::element_wise::ContractionFactor] = 0.5;
    tConstants[Plato::element_wise::UpperBoundScaleFactor] = 10.0;
    tConstants[Plato::element_wise::LowerBoundScaleFactor] = 0.01;

    // ************ Allocate instance with element wise functions ************
    Plato::DevicePrimalProbElementWise<double> tDevicePrimalProbElementWise;
    tDevicePrimalProbElementWise.updateSigmaCoeff(tConstants,
                                                  tCurrentControl,
                                                  tPreviousControl,
                                                  tAntepenultimateControls,
                                                  tUpperBounds,
                                                  tLowerBounds,
                                                  tPreviousSigma,
                                                  tCurrentSigma);

    // ************ Test output = CurrentSigma from updateSigmaCoeff ************
    tBaseValue = 0.05;
    Plato::KokkosVector<double> tGold(tSize, tBaseValue);
    tGold.host();
    tCurrentSigma.host();
    PlatoTest::checkVectorData(tCurrentSigma, tGold);
}

TEST(PlatoTest, DevicePrimalProbUpdateObjectiveGlobalizationCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1;
    const size_t tSize = 10;
    Plato::KokkosVector<double> tDeltaControl(tSize, tBaseValue);

    tBaseValue = 2;
    Plato::KokkosVector<double> tCurrentSigma(tSize, tBaseValue);

    tBaseValue = 0.05;
    Plato::KokkosVector<double> tCurrentGradient(tSize, tBaseValue);

    tBaseValue = 0.0;
    Plato::KokkosVector<double> tOutputOne(tSize, tBaseValue);
    Plato::KokkosVector<double> tOutputTwo(tSize, tBaseValue);

    // ************ Allocate instance with element wise functions ************
    Plato::DevicePrimalProbElementWise<double> tDevicePrimalProbElementWise;
    tDevicePrimalProbElementWise.updateObjectiveGlobalizationCoeff(tDeltaControl,
                                                                   tCurrentSigma,
                                                                   tCurrentGradient,
                                                                   tOutputOne,
                                                                   tOutputTwo);

    // ************ Test outputs = tOutputOne & tOutputTwo from updateObjectiveGlobalizationCoeff ************
    tBaseValue = 1.0 / 3.0;
    Plato::KokkosVector<double> tGoldOne(tSize, tBaseValue);
    tGoldOne.host();
    tOutputOne.host();
    PlatoTest::checkVectorData(tOutputOne, tGoldOne);
    tBaseValue = 0.1;
    Plato::KokkosVector<double> tGoldTwo(tSize, tBaseValue);
    tGoldTwo.host();
    tOutputTwo.host();
    PlatoTest::checkVectorData(tOutputTwo, tGoldTwo);
}

TEST(PlatoTest, DevicePrimalProbUpdateConstraintGlobalizationCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1;
    const size_t tSize = 10;
    Plato::KokkosVector<double> tDeltaControl(tSize, tBaseValue);

    tBaseValue = 2;
    Plato::KokkosVector<double> tCurrentSigma(tSize, tBaseValue);

    tBaseValue = 0.05;
    Plato::KokkosVector<double> tCurrentGradient(tSize, tBaseValue);

    tBaseValue = 0.0;
    Plato::KokkosVector<double> tOutputOne(tSize, tBaseValue);
    Plato::KokkosVector<double> tOutputTwo(tSize, tBaseValue);

    // ************ Allocate instance with element wise functions ************
    Plato::DevicePrimalProbElementWise<double> tDevicePrimalProbElementWise;
    tDevicePrimalProbElementWise.updateConstraintGlobalizationCoeff(tDeltaControl,
                                                                    tCurrentSigma,
                                                                    tCurrentGradient,
                                                                    tOutputOne,
                                                                    tOutputTwo);

    // ************ Test outputs = tOutputOne & tOutputTwo from updateObjectiveGlobalizationCoeff ************
    tBaseValue = 1.0 / 3.0;
    Plato::KokkosVector<double> tGoldOne(tSize, tBaseValue);
    tGoldOne.host();
    tOutputOne.host();
    PlatoTest::checkVectorData(tOutputOne, tGoldOne);
    tBaseValue = 0.1;
    Plato::KokkosVector<double> tGoldTwo(tSize, tBaseValue);
    tGoldTwo.host();
    tOutputTwo.host();
    PlatoTest::checkVectorData(tOutputTwo, tGoldTwo);
}

TEST(PlatoTest, DeviceDualProbUpdateTrialControl)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1e-1;
    const size_t tSize = 10;
    Plato::KokkosVector<double> tTermA(tSize, tBaseValue);

    tBaseValue = 5e-1;
    Plato::KokkosVector<double> tTermB(tSize, tBaseValue);

    tBaseValue = 0.0;
    Plato::KokkosVector<double> tLowerBounds(tSize, tBaseValue);

    tBaseValue = 1e3;
    Plato::KokkosVector<double> tUpperBounds(tSize, tBaseValue);

    tBaseValue = 0.01;
    Plato::KokkosVector<double> tLowerAsymptotes(tSize, tBaseValue);

    tBaseValue = 10.0;
    Plato::KokkosVector<double> tUpperAsymptotes(tSize, tBaseValue);

    Plato::KokkosVector<double> tOutput(tSize);

    // ************ Allocate instance with element wise functions ************
    Plato::DeviceDualProbElementWise<double> tDevicePrimalProbElementWise;
    tDevicePrimalProbElementWise.updateTrialControl(tTermA,
                                                    tTermB,
                                                    tLowerBounds,
                                                    tUpperBounds,
                                                    tLowerAsymptotes,
                                                    tUpperAsymptotes,
                                                    tOutput);

    // ************ Test output from updateTrialControl ************
    tBaseValue = 6.9129202261942755;
    Plato::KokkosVector<double> tGold(tSize, tBaseValue);
    tGold.host();
    tOutput.host();
    PlatoTest::checkVectorData(tOutput, tGold);
}

TEST(PlatoTest, DeviceDualProbUpdateGradientCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1.0;
    const size_t tSize = 10;
    Plato::KokkosVector<double> tTrialControls(tSize, tBaseValue);

    tBaseValue = 0.01;
    Plato::KokkosVector<double> tLowerAsymptotes(tSize, tBaseValue);

    tBaseValue = 10.0;
    Plato::KokkosVector<double> tUpperAsymptotes(tSize, tBaseValue);

    tBaseValue = 1e-2;
    Plato::KokkosVector<double> tPcoeff(tSize, tBaseValue);

    tBaseValue = -1e-2;
    Plato::KokkosVector<double> tQCoeff(tSize, tBaseValue);

    Plato::KokkosVector<double> tOutputOne(tSize);
    Plato::KokkosVector<double> tOutputTwo(tSize);

    // ************ Allocate instance with element wise functions ************
    Plato::DeviceDualProbElementWise<double> tDevicePrimalProbElementWise;
    tDevicePrimalProbElementWise.updateGradientCoeff(tTrialControls,
                                                     tLowerAsymptotes,
                                                     tUpperAsymptotes,
                                                     tPcoeff,
                                                     tQCoeff,
                                                     tOutputOne,
                                                     tOutputTwo);

    // ************ Test output from updateGradientCoeff ************
    tBaseValue = 0.0011111111111111;
    Plato::KokkosVector<double> tGoldOne(tSize, tBaseValue);
    tGoldOne.host();
    tOutputOne.host();
    PlatoTest::checkVectorData(tOutputOne, tGoldOne);
    tBaseValue = -0.0101010101010101;
    Plato::KokkosVector<double> tGoldTwo(tSize, tBaseValue);
    tGoldTwo.host();
    tOutputTwo.host();
    PlatoTest::checkVectorData(tOutputTwo, tGoldTwo);
}

TEST(PlatoTest, DeviceDualProbUpdateObjectiveCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1.0;
    const size_t tSize = 10;
    Plato::KokkosVector<double> tCurrentSigma(tSize, tBaseValue);

    tBaseValue = -5e-4;
    Plato::KokkosVector<double> tCurrentGradient(tSize, tBaseValue);

    Plato::KokkosVector<double> tPcoeff(tSize);
    Plato::KokkosVector<double> tQcoeff(tSize);
    Plato::KokkosVector<double> tRcoeff(tSize);

    // ************ Allocate instance with element wise functions ************
    const double tGlobalizationFactor = 0.5;
    Plato::DeviceDualProbElementWise<double> tDevicePrimalProbElementWise;
    tDevicePrimalProbElementWise.updateObjectiveCoeff(tGlobalizationFactor,
                                                      tCurrentSigma,
                                                      tCurrentGradient,
                                                      tPcoeff,
                                                      tQcoeff,
                                                      tRcoeff);

    // ************ Test outputs from updateObjectiveCoeff ************
    tBaseValue = 0.125;
    Plato::KokkosVector<double> tGoldOne(tSize, tBaseValue);
    tPcoeff.host();
    tGoldOne.host();
    PlatoTest::checkVectorData(tPcoeff, tGoldOne);
    tBaseValue = 0.1255;
    Plato::KokkosVector<double> tGoldTwo(tSize, tBaseValue);
    tQcoeff.host();
    tGoldTwo.host();
    PlatoTest::checkVectorData(tQcoeff, tGoldTwo);
    tBaseValue = 0.2505;
    Plato::KokkosVector<double> tGoldThree(tSize, tBaseValue);
    tRcoeff.host();
    tGoldThree.host();
    PlatoTest::checkVectorData(tRcoeff, tGoldThree);
}

TEST(PlatoTest, DeviceDualProbUpdateConstraintCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 1.0;
    const size_t tSize = 10;
    Plato::KokkosVector<double> tCurrentSigma(tSize, tBaseValue);

    tBaseValue = -5e-4;
    Plato::KokkosVector<double> tCurrentGradient(tSize, tBaseValue);

    Plato::KokkosVector<double> tPcoeff(tSize);
    Plato::KokkosVector<double> tQcoeff(tSize);
    Plato::KokkosVector<double> tRcoeff(tSize);

    // ************ Allocate instance with element wise functions ************
    const double tGlobalizationFactor = 0.5;
    Plato::DeviceDualProbElementWise<double> tDevicePrimalProbElementWise;
    tDevicePrimalProbElementWise.updateConstraintCoeff(tGlobalizationFactor,
                                                       tCurrentSigma,
                                                       tCurrentGradient,
                                                       tPcoeff,
                                                       tQcoeff,
                                                       tRcoeff);

    // ************ Test outputs from updateConstraintCoeff ************
    tBaseValue = 0.125;
    Plato::KokkosVector<double> tGoldOne(tSize, tBaseValue);
    tPcoeff.host();
    tGoldOne.host();
    PlatoTest::checkVectorData(tPcoeff, tGoldOne);
    tBaseValue = 0.1255;
    Plato::KokkosVector<double> tGoldTwo(tSize, tBaseValue);
    tQcoeff.host();
    tGoldTwo.host();
    PlatoTest::checkVectorData(tQcoeff, tGoldTwo);
    tBaseValue = 0.2505;
    Plato::KokkosVector<double> tGoldThree(tSize, tBaseValue);
    tRcoeff.host();
    tGoldThree.host();
    PlatoTest::checkVectorData(tRcoeff, tGoldThree);
}

TEST(PlatoTest, DeviceDualProbUpdateMovingAsymptotesCoeff)
{
    // ************ Allocate input arguments to test function ************
    double tBaseValue = 0.5;
    const size_t tSize = 10;
    Plato::KokkosVector<double> tTrialControls(tSize, tBaseValue);

    tBaseValue = 1e-1;
    Plato::KokkosVector<double> tLowerAsymptotes(tSize, tBaseValue);

    tBaseValue = 1e-1;
    Plato::KokkosVector<double> tUpperAsymptotes(tSize, tBaseValue);

    tBaseValue = 5e-1;
    Plato::KokkosVector<double> tObjectivePcoeff(tSize, tBaseValue);

    tBaseValue = -2.5e-1;
    Plato::KokkosVector<double> tObjectiveQcoeff(tSize, tBaseValue);

    tBaseValue = 1e-2;
    Plato::KokkosVector<double> tDualTimesConstraintPcoeff(tSize, tBaseValue);

    tBaseValue = -2.5e-2;
    Plato::KokkosVector<double> tDualTimesConstraintQcoeff(tSize, tBaseValue);

    Plato::KokkosVector<double> tOutputOne(tSize);
    Plato::KokkosVector<double> tOutputTwo(tSize);

    // ************ Allocate instance with element wise functions ************
    Plato::DeviceDualProbElementWise<double> tDevicePrimalProbElementWise;
    tDevicePrimalProbElementWise.updateMovingAsymptotesCoeff(tTrialControls,
                                                             tLowerAsymptotes,
                                                             tUpperAsymptotes,
                                                             tObjectivePcoeff,
                                                             tObjectiveQcoeff,
                                                             tDualTimesConstraintPcoeff,
                                                             tDualTimesConstraintQcoeff,
                                                             tOutputOne,
                                                             tOutputTwo);

    // ************ Test outputs from updateMovingAsymptotesCoeff ************
    tBaseValue = -1.275;
    Plato::KokkosVector<double> tGoldOne(tSize, tBaseValue);
    tGoldOne.host();
    tOutputOne.host();
    PlatoTest::checkVectorData(tOutputOne, tGoldOne);
    tBaseValue = -0.6875;
    Plato::KokkosVector<double> tGoldTwo(tSize, tBaseValue);
    tGoldTwo.host();
    tOutputTwo.host();
    PlatoTest::checkVectorData(tOutputTwo, tGoldTwo);
}

} // namespace PlatoTest
