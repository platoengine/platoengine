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
 * Plato_Test_AugmentedLagrangian.cpp
 *
 *  Created on: Oct 21, 2017
 */

#include "gtest/gtest.h"

#include "Plato_UnitTestUtils.hpp"

#include "Plato_Radius.hpp"
#include "Plato_Circle.hpp"
#include "Plato_Rosenbrock.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_AnalyticalHessian.hpp"
#include "Plato_LinearOperatorList.hpp"
#include "Plato_AnalyticalGradient.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_GradientOperatorList.hpp"
#include "Plato_IdentityPreconditioner.hpp"
#include "Plato_LinearCriterionHessian.hpp"
#include "Plato_CentralDifferenceHessian.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_AugmentedLagrangianStageMng.hpp"
#include "Plato_KelleySachsAugmentedLagrangian.hpp"
#include "Plato_StandardVectorReductionOperations.hpp"
#include "Plato_GoldsteinPrice.hpp"
#include "Plato_Himmelblau.hpp"
#include "Plato_ShiftedEllipse.hpp"

namespace PlatoTest
{

TEST(PlatoTest, LinearCriterionHessian)
{
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    Plato::StandardMultiVector<double> tOutput(tNumVectors, tNumControls);
    double tValue = 0.5;
    Plato::StandardMultiVector<double> tVector(tNumVectors, tNumControls, tValue);
    tValue = 2;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tValue);

    // ********* COMPUTE LINEAR CRITERION HESSIAN TIMES VECTOR *********
    Plato::LinearCriterionHessian<double> tHessian;
    tHessian.apply(tControl, tVector, tOutput);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    PlatoTest::checkMultiVectorData(tOutput, tGold);
}

TEST(PlatoTest, CentralDifferenceHessian)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumControls = 2;
    tDataFactory->allocateControl(tNumControls);

    // ********* ALLOCATE TRUE CRITERION *********
    std::shared_ptr<Plato::Rosenbrock<double>> tCriterion = std::make_shared<Plato::Rosenbrock<double>>();

    // ********* ALLOCATE HESSIAN APPROXIMATION AND DATA STRUCTURES FOR TEST *********
    double tValue = 0.5;
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tVector(tNumVectors, tNumControls, tValue);
    tValue = 2;
    Plato::StandardMultiVector<double> tOutput(tNumVectors, tNumControls);
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tValue);
    Plato::CentralDifferenceHessian<double> tHessian(tDataFactory, tCriterion);

    // ********* COMPUTE HESSIAN APPROXIMATION *********
    tHessian.apply(tControl, tVector, tOutput);

    // ********* COMPUTE TRUE HESSIAN *********
    Plato::StandardMultiVector<double> tTrueHessianTimesVector(tNumVectors, tNumControls);
    tCriterion->hessian(tControl, tVector, tTrueHessianTimesVector);
    PlatoTest::checkMultiVectorData(tOutput, tTrueHessianTimesVector);
}

TEST(PlatoTest, TrustRegionAlgorithmDataMng)
{
    // ********* Test Factories for Dual Data *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();

    // ********* Allocate Core Optimization Data Templates *********
    const size_t tNumDuals = 10;
    const size_t tNumDualVectors = 2;
    tDataFactory->allocateDual(tNumDuals, tNumDualVectors);
    const size_t tNumStates = 20;
    const size_t tNumStateVectors = 6;
    tDataFactory->allocateState(tNumStates, tNumStateVectors);
    const size_t tNumControls = 5;
    const size_t tNumControlVectors = 3;
    tDataFactory->allocateControl(tNumControls, tNumControlVectors);

    // ********* Allocate Trust Region Algorithm Data Manager *********
    Plato::TrustRegionAlgorithmDataMng<double> tDataMng(tDataFactory);

    // ********* Test Trust Region Algorithm Data Manager *********
    // TEST NUMBER OF VECTORS FUNCTIONS
    EXPECT_EQ(tNumDualVectors, tDataMng.getNumDualVectors());
    EXPECT_EQ(tNumControlVectors, tDataMng.getNumControlVectors());

    // TEST CURRENT OBJECTIVE FUNCTION VALUE INTERFACES
    double tGoldValue = std::numeric_limits<double>::max();
    const double tTolerance = 1e-6;
    EXPECT_NEAR(tGoldValue, tDataMng.getCurrentObjectiveFunctionValue(), tTolerance);
    tGoldValue = 0.123;
    tDataMng.setCurrentObjectiveFunctionValue(0.123);
    EXPECT_NEAR(tGoldValue, tDataMng.getCurrentObjectiveFunctionValue(), tTolerance);

    // TEST PREVIOUS OBJECTIVE FUNCTION VALUE INTERFACES
    tGoldValue = std::numeric_limits<double>::max();
    EXPECT_NEAR(tGoldValue, tDataMng.getPreviousObjectiveFunctionValue(), tTolerance);
    tGoldValue = 0.321;
    tDataMng.setPreviousObjectiveFunctionValue(0.321);
    EXPECT_NEAR(tGoldValue, tDataMng.getPreviousObjectiveFunctionValue(), tTolerance);

    // 1) TEST INITIAL GUESS INTERFACES
    EXPECT_FALSE(tDataMng.isInitialGuessSet());
    double tValue = 0.5;
    tDataMng.setInitialGuess(0.5);
    EXPECT_TRUE(tDataMng.isInitialGuessSet());
    Plato::StandardVector<double> tPlatoControlVector(tNumControls, tValue);
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        PlatoTest::checkVectorData(tDataMng.getCurrentControl(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    // 2) TEST INITIAL GUESS INTERFACES
    tValue = 0.3;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue + static_cast<double>(0.1);
        tPlatoControlVector.fill(tValue);
        tDataMng.setInitialGuess(tVectorIndex, tValue);
        PlatoTest::checkVectorData(tDataMng.getCurrentControl(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    // 3) TEST INITIAL GUESS INTERFACES
    tValue = 0;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue + static_cast<double>(0.1);
        tPlatoControlVector.fill(tValue);
        tDataMng.setInitialGuess(tVectorIndex, tPlatoControlVector);
        PlatoTest::checkVectorData(tDataMng.getCurrentControl(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    // 4) TEST INITIAL GUESS INTERFACES
    Plato::StandardMultiVector<double> tPlatoControlMultiVector(tNumControlVectors, tPlatoControlVector);
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tPlatoControlMultiVector[tVectorIndex].fill(tVectorIndex);
    }
    tDataMng.setInitialGuess(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControl(), tPlatoControlMultiVector, tTolerance);

    // TEST DUAL VECTOR INTERFACES
    Plato::StandardVector<double> tPlatoDualVector(tNumDuals);
    for(size_t tVectorIndex = 0; tVectorIndex < tNumDualVectors; tVectorIndex++)
    {
        tValue = tValue + static_cast<double>(tVectorIndex);
        tPlatoDualVector.fill(tValue);
        tDataMng.setDual(tVectorIndex, tPlatoDualVector);
        PlatoTest::checkVectorData(tDataMng.getDual(tVectorIndex), tPlatoDualVector, tTolerance);
    }

    tValue = 20;
    Plato::StandardMultiVector<double> tPlatoDualMultiVector(tNumDualVectors, tPlatoDualVector);
    for(size_t tVectorIndex = 0; tVectorIndex < tNumDualVectors; tVectorIndex++)
    {
        tValue = tValue + static_cast<double>(tVectorIndex);
        tPlatoDualMultiVector[tVectorIndex].fill(tVectorIndex);
    }
    tDataMng.setDual(tPlatoDualMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getDual(), tPlatoDualMultiVector, tTolerance);

    // TEST TRIAL STEP INTERFACES
    tValue = 3;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue + static_cast<double>(tVectorIndex);
        tPlatoControlVector.fill(tValue);
        tDataMng.setTrialStep(tVectorIndex, tPlatoControlVector);
        PlatoTest::checkVectorData(tDataMng.getTrialStep(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tPlatoControlMultiVector[tVectorIndex].fill(tVectorIndex);
    }
    tDataMng.setTrialStep(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getTrialStep(), tPlatoControlMultiVector, tTolerance);

    // TEST ACTIVE SET INTERFACES
    tValue = 33;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue + static_cast<double>(tVectorIndex);
        tPlatoControlVector.fill(tValue);
        tDataMng.setActiveSet(tVectorIndex, tPlatoControlVector);
        PlatoTest::checkVectorData(tDataMng.getActiveSet(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tPlatoControlMultiVector[tVectorIndex].fill(tVectorIndex);
    }
    tDataMng.setActiveSet(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getActiveSet(), tPlatoControlMultiVector, tTolerance);

    // TEST INACTIVE SET INTERFACES
    tValue = 23;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue + static_cast<double>(tVectorIndex);
        tPlatoControlVector.fill(tValue);
        tDataMng.setInactiveSet(tVectorIndex, tPlatoControlVector);
        PlatoTest::checkVectorData(tDataMng.getInactiveSet(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tPlatoControlMultiVector[tVectorIndex].fill(tVectorIndex);
    }
    tDataMng.setInactiveSet(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getInactiveSet(), tPlatoControlMultiVector, tTolerance);

    // TEST CURRENT CONTROL INTERFACES
    tValue = 30;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue + static_cast<double>(tVectorIndex);
        tPlatoControlVector.fill(tValue);
        tDataMng.setCurrentControl(tVectorIndex, tPlatoControlVector);
        PlatoTest::checkVectorData(tDataMng.getCurrentControl(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tPlatoControlMultiVector[tVectorIndex].fill(tVectorIndex);
    }
    tDataMng.setCurrentControl(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControl(), tPlatoControlMultiVector, tTolerance);

    // TEST PREVIOUS CONTROL INTERFACES
    tValue = 80;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue + static_cast<double>(tVectorIndex);
        tPlatoControlVector.fill(tValue);
        tDataMng.setPreviousControl(tVectorIndex, tPlatoControlVector);
        PlatoTest::checkVectorData(tDataMng.getPreviousControl(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tPlatoControlMultiVector[tVectorIndex].fill(tVectorIndex);
    }
    tDataMng.setPreviousControl(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousControl(), tPlatoControlMultiVector, tTolerance);

    // TEST CURRENT GRADIENT INTERFACES
    tValue = 7882;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue + static_cast<double>(tVectorIndex);
        tPlatoControlVector.fill(tValue);
        tDataMng.setCurrentGradient(tVectorIndex, tPlatoControlVector);
        PlatoTest::checkVectorData(tDataMng.getCurrentGradient(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tPlatoControlMultiVector[tVectorIndex].fill(tVectorIndex);
    }
    tDataMng.setCurrentGradient(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentGradient(), tPlatoControlMultiVector, tTolerance);

    // TEST PREVIOUS GRADIENT INTERFACES
    tValue = 101183;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue + static_cast<double>(tVectorIndex);
        tPlatoControlVector.fill(tValue);
        tDataMng.setPreviousGradient(tVectorIndex, tPlatoControlVector);
        PlatoTest::checkVectorData(tDataMng.getPreviousGradient(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tPlatoControlMultiVector[tVectorIndex].fill(tVectorIndex);
    }
    tDataMng.setPreviousGradient(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousGradient(), tPlatoControlMultiVector, tTolerance);

    // TEST CONTROL LOWER BOUND INTERFACES
    tValue = -std::numeric_limits<double>::max();
    Plato::fill(tValue, tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tPlatoControlMultiVector, tTolerance);
    tValue = 1e-3;
    tDataMng.setControlLowerBounds(tValue);
    Plato::fill(tValue, tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tPlatoControlMultiVector, tTolerance);

    tValue = 1e-4;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue + (static_cast<double>(tVectorIndex) * tValue);
        tDataMng.setControlLowerBounds(tVectorIndex, tValue);
        tPlatoControlVector.fill(tValue);
        PlatoTest::checkVectorData(tDataMng.getControlLowerBounds(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    tValue = 1;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue + static_cast<double>(tVectorIndex);
        tPlatoControlVector.fill(tValue);
        tDataMng.setControlLowerBounds(tVectorIndex, tPlatoControlVector);
        PlatoTest::checkVectorData(tDataMng.getControlLowerBounds(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    tValue = 1e-2;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue * static_cast<double>(tVectorIndex + 1);
        tPlatoControlMultiVector[tVectorIndex].fill(tValue);
    }
    tDataMng.setControlLowerBounds(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tPlatoControlMultiVector, tTolerance);

    // TEST CONTROL UPPER BOUND INTERFACES
    tValue = std::numeric_limits<double>::max();
    Plato::fill(tValue, tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tPlatoControlMultiVector, tTolerance);
    tValue = 1e-3;
    tDataMng.setControlUpperBounds(tValue);
    Plato::fill(tValue, tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tPlatoControlMultiVector, tTolerance);

    tValue = 1e-4;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue + (static_cast<double>(tVectorIndex) * tValue);
        tDataMng.setControlUpperBounds(tVectorIndex, tValue);
        tPlatoControlVector.fill(tValue);
        PlatoTest::checkVectorData(tDataMng.getControlUpperBounds(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    tValue = 1;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue + static_cast<double>(tVectorIndex);
        tPlatoControlVector.fill(tValue);
        tDataMng.setControlUpperBounds(tVectorIndex, tPlatoControlVector);
        PlatoTest::checkVectorData(tDataMng.getControlUpperBounds(tVectorIndex), tPlatoControlVector, tTolerance);
    }

    tValue = 1e-2;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        tValue = tValue * static_cast<double>(tVectorIndex + 1);
        tPlatoControlMultiVector[tVectorIndex].fill(tValue);
    }
    tDataMng.setControlUpperBounds(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tPlatoControlMultiVector, tTolerance);

    // TEST GRADIENT INEXACTNESS FLAG FUNCTIONS
    EXPECT_FALSE(tDataMng.isGradientInexactnessToleranceExceeded());
    tDataMng.setGradientInexactnessFlag(true);
    EXPECT_TRUE(tDataMng.isGradientInexactnessToleranceExceeded());
    tDataMng.setGradientInexactnessFlag(false);
    EXPECT_FALSE(tDataMng.isGradientInexactnessToleranceExceeded());

    // TEST OBJECTIVE INEXACTNESS FLAG FUNCTIONS
    EXPECT_FALSE(tDataMng.isObjectiveInexactnessToleranceExceeded());
    tDataMng.setObjectiveInexactnessFlag(true);
    EXPECT_TRUE(tDataMng.isObjectiveInexactnessToleranceExceeded());
    tDataMng.setObjectiveInexactnessFlag(false);
    EXPECT_FALSE(tDataMng.isObjectiveInexactnessToleranceExceeded());

    // TEST COMPUTE STAGNATION MEASURE FUNCTION
    tValue = 1e-2;
    for(size_t tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
    {
        double tCurrentValue = tValue + static_cast<double>(tVectorIndex);
        tPlatoControlVector.fill(tCurrentValue);
        tDataMng.setCurrentControl(tVectorIndex, tPlatoControlVector);
        double tPreviousValue = tValue * static_cast<double>(tVectorIndex * tVectorIndex);
        tPlatoControlVector.fill(tPreviousValue);
        tDataMng.setPreviousControl(tVectorIndex, tPlatoControlVector);
    }
    tValue = 1.97;
    tDataMng.computeControlStagnationMeasure();
    EXPECT_NEAR(tValue, tDataMng.getControlStagnationMeasure(), tTolerance);

    // TEST COMPUTE NORM OF PROJECTED VECTOR
    Plato::fill(1., tPlatoControlMultiVector);
    tDataMng.setInactiveSet(tPlatoControlMultiVector);
    Plato::fill(2., tPlatoControlMultiVector);
    tValue = 7.745966692414834;
    EXPECT_NEAR(tValue, tDataMng.computeProjectedVectorNorm(tPlatoControlMultiVector), tTolerance);

    Plato::fill(1., tPlatoControlMultiVector);
    size_t tVectorIndex = 1;
    size_t tElementIndex = 2;
    tPlatoControlMultiVector(tVectorIndex, tElementIndex) = 0;
    tDataMng.setInactiveSet(tPlatoControlMultiVector);
    Plato::fill(2., tPlatoControlMultiVector);
    tValue = 7.483314773547883;
    EXPECT_NEAR(tValue, tDataMng.computeProjectedVectorNorm(tPlatoControlMultiVector), tTolerance);

    // TEST COMPUTE PROJECTED GRADIENT NORM
    Plato::fill(1., tPlatoControlMultiVector);
    tDataMng.setInactiveSet(tPlatoControlMultiVector);
    Plato::fill(3., tPlatoControlMultiVector);
    tDataMng.setCurrentGradient(tPlatoControlMultiVector);
    tDataMng.computeNormProjectedGradient();
    tValue = 11.61895003862225;
    EXPECT_NEAR(tValue, tDataMng.getNormProjectedGradient(), tTolerance);

    Plato::fill(1., tPlatoControlMultiVector);
    tVectorIndex = 0;
    tElementIndex = 0;
    tPlatoControlMultiVector(tVectorIndex, tElementIndex) = 0;
    tDataMng.setInactiveSet(tPlatoControlMultiVector);
    tDataMng.computeNormProjectedGradient();
    tValue = 11.224972160321824;
    EXPECT_NEAR(tValue, tDataMng.getNormProjectedGradient(), tTolerance);

    // TEST COMPUTE STATIONARY MEASURE
    Plato::fill(1., tPlatoControlMultiVector);
    tDataMng.setInactiveSet(tPlatoControlMultiVector);
    tDataMng.setCurrentControl(tPlatoControlMultiVector);
    Plato::fill(-2., tPlatoControlMultiVector);
    tDataMng.setControlLowerBounds(tPlatoControlMultiVector);
    Plato::fill(12., tPlatoControlMultiVector);
    tDataMng.setControlUpperBounds(tPlatoControlMultiVector);
    Plato::fill(-1., tPlatoControlMultiVector);
    tDataMng.setCurrentGradient(tPlatoControlMultiVector);
    tDataMng.computeStationarityMeasure();
    tValue = 3.872983346207417;
    EXPECT_NEAR(tValue, tDataMng.getStationarityMeasure(), tTolerance);

    // TEST RESET STAGE FUNCTION
    tValue = 1;
    tDataMng.setCurrentObjectiveFunctionValue(tValue);
    EXPECT_NEAR(tValue, tDataMng.getCurrentObjectiveFunctionValue(), tTolerance);
    tValue = 2;
    tDataMng.setPreviousObjectiveFunctionValue(tValue);
    EXPECT_NEAR(tValue, tDataMng.getPreviousObjectiveFunctionValue(), tTolerance);
    Plato::fill(1., tPlatoControlMultiVector);
    tDataMng.setCurrentControl(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControl(), tPlatoControlMultiVector);
    Plato::fill(-2., tPlatoControlMultiVector);
    tDataMng.setPreviousControl(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousControl(), tPlatoControlMultiVector);
    Plato::fill(10., tPlatoControlMultiVector);
    tDataMng.setCurrentGradient(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentGradient(), tPlatoControlMultiVector);
    Plato::fill(-12., tPlatoControlMultiVector);
    tDataMng.setPreviousGradient(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousGradient(), tPlatoControlMultiVector);

    tDataMng.resetCurrentStageDataToPreviousStageData();

    tValue = 2;
    EXPECT_NEAR(tValue, tDataMng.getCurrentObjectiveFunctionValue(), tTolerance);
    Plato::fill(-2., tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControl(), tPlatoControlMultiVector);
    Plato::fill(-12., tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentGradient(), tPlatoControlMultiVector);

    // TEST STORE CURRENT STAGE DATA FUNCTION
    tValue = 1;
    tDataMng.setCurrentObjectiveFunctionValue(tValue);
    EXPECT_NEAR(tValue, tDataMng.getCurrentObjectiveFunctionValue(), tTolerance);
    tValue = 2;
    EXPECT_NEAR(tValue, tDataMng.getPreviousObjectiveFunctionValue(), tTolerance);
    Plato::fill(1., tPlatoControlMultiVector);
    tDataMng.setCurrentControl(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControl(), tPlatoControlMultiVector);
    Plato::fill(-2., tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousControl(), tPlatoControlMultiVector);
    Plato::fill(10., tPlatoControlMultiVector);
    tDataMng.setCurrentGradient(tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentGradient(), tPlatoControlMultiVector);
    Plato::fill(-12., tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousGradient(), tPlatoControlMultiVector);

    tDataMng.cacheCurrentStageData();

    tValue = 1;
    EXPECT_NEAR(tValue, tDataMng.getPreviousObjectiveFunctionValue(), tTolerance);
    Plato::fill(1., tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousControl(), tPlatoControlMultiVector);
    Plato::fill(10., tPlatoControlMultiVector);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousGradient(), tPlatoControlMultiVector);
}

TEST(PlatoTest, RosenbrockCriterion)
{
    double tValue = 2;
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tValue);

    Plato::Rosenbrock<double> tCriterion;
    // TEST OBJECTIVE FUNCTION EVALUATION
    double tObjectiveValue = tCriterion.value(tControl);
    const double tGoldValue = 401;
    const double tTolerance = 1e-6;
    EXPECT_NEAR(tGoldValue, tObjectiveValue, tTolerance);

    // TEST GRADIENT EVALUATION FUNCTION
    Plato::StandardMultiVector<double> tGradient(tNumVectors, tNumControls);
    tCriterion.gradient(tControl, tGradient);
    Plato::StandardMultiVector<double> tGoldVector(tNumVectors, tNumControls);
    const size_t tVectorIndex = 0;
    tGoldVector(tVectorIndex, 0) = 1602;
    tGoldVector(tVectorIndex, 1) = -400;
    PlatoTest::checkMultiVectorData(tGradient, tGoldVector);

    // TEST HESSIAN TIMES VECTOR FUNCTION
    tValue = 1;
    Plato::StandardMultiVector<double> tVector(tNumVectors, tNumControls, tValue);
    Plato::StandardMultiVector<double> tHessianTimesVector(tNumVectors, tNumControls);
    tCriterion.hessian(tControl, tVector, tHessianTimesVector);
    tGoldVector(tVectorIndex, 0) = 3202;
    tGoldVector(tVectorIndex, 1) = -600;
    PlatoTest::checkMultiVectorData(tHessianTimesVector, tGoldVector);
}

TEST(PlatoTest, CircleCriterion)
{
    const double tValue = 1;
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tValue);

    Plato::Circle<double> tCriterion;

    // TEST OBJECTIVE FUNCTION EVALUATION
    double tObjectiveValue = tCriterion.value(tControl);
    const double tGoldValue = 2;
    const double tTolerance = 1e-6;
    EXPECT_NEAR(tGoldValue, tObjectiveValue, tTolerance);

    // TEST GRADIENT EVALUATION FUNCTION
    Plato::StandardMultiVector<double> tGradient(tNumVectors, tNumControls);
    tCriterion.gradient(tControl, tGradient);
    Plato::StandardMultiVector<double> tGoldVector(tNumVectors, tNumControls);
    const size_t tVectorIndex = 0;
    tGoldVector(tVectorIndex, 1) = -4;
    PlatoTest::checkMultiVectorData(tGradient, tGoldVector);

    // TEST HESSIAN TIMES VECTOR FUNCTION
    Plato::StandardMultiVector<double> tVector(tNumVectors, tNumControls, tValue);
    tVector(tVectorIndex, 1) = -2.;
    Plato::StandardMultiVector<double> tHessianTimesVector(tNumVectors, tNumControls);
    tCriterion.hessian(tControl, tVector, tHessianTimesVector);
    tGoldVector(tVectorIndex, 0) = 2.;
    tGoldVector(tVectorIndex, 1) = -8.;
    PlatoTest::checkMultiVectorData(tHessianTimesVector, tGoldVector);
}

TEST(PlatoTest, RadiusCriterion)
{
    const double tValue = 0.5;
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tValue);

    Plato::Radius<double> tCriterion;

    // TEST OBJECTIVE FUNCTION EVALUATION
    double tObjectiveValue = tCriterion.value(tControl);
    const double tGoldValue = -0.5;
    const double tTolerance = 1e-6;
    EXPECT_NEAR(tGoldValue, tObjectiveValue, tTolerance);

    // TEST GRADIENT EVALUATION FUNCTION
    Plato::StandardMultiVector<double> tGradient(tNumVectors, tNumControls);
    tCriterion.gradient(tControl, tGradient);
    Plato::StandardMultiVector<double> tGoldVector(tNumVectors, tNumControls);
    Plato::fill(1., tGoldVector);
    PlatoTest::checkMultiVectorData(tGradient, tGoldVector);

    // TEST HESSIAN TIMES VECTOR FUNCTION
    Plato::StandardMultiVector<double> tVector(tNumVectors, tNumControls, tValue);
    const size_t tVectorIndex = 0;
    tVector(tVectorIndex, 1) = -2.;
    Plato::StandardMultiVector<double> tHessianTimesVector(tNumVectors, tNumControls);
    tCriterion.hessian(tControl, tVector, tHessianTimesVector);
    tGoldVector(tVectorIndex, 0) = 1.0;
    tGoldVector(tVectorIndex, 1) = -4.0;
    PlatoTest::checkMultiVectorData(tHessianTimesVector, tGoldVector);
}

TEST(PlatoTest, AnalyticalGradient)
{
    const double tValue = 1;
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    const size_t tVectorIndex = 0;
    Plato::StandardMultiVector<double> tOutput(tNumVectors, tNumControls);
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tValue);

    std::shared_ptr<Plato::Circle<double>> tCriterion = std::make_shared<Plato::Circle<double>>();
    Plato::AnalyticalGradient<double> tGradient(tCriterion);

    // TEST COMPUTE FUNCTION
    tGradient.compute(tControl, tOutput);

    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(tVectorIndex, 0) = 0.0;
    tGold(tVectorIndex, 1) = -4.0;
    PlatoTest::checkMultiVectorData(tOutput, tGold);
}

TEST(PlatoTest, AnalyticalHessian)
{
    const double tValue = 1;
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    const size_t tVectorIndex = 0;
    Plato::StandardMultiVector<double> tVector(tNumVectors, tNumControls, tValue);
    tVector(tVectorIndex, 1) = -2.;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tValue);
    Plato::StandardMultiVector<double> tHessianTimesVector(tNumVectors, tNumControls);

    std::shared_ptr<Plato::Circle<double>> tCriterion = std::make_shared<Plato::Circle<double>>();
    Plato::AnalyticalHessian<double> tHessian(tCriterion);

    // TEST APPLY VECTOR TO HESSIAN OPERATOR FUNCTION
    tHessian.apply(tControl, tVector, tHessianTimesVector);

    Plato::StandardMultiVector<double> tGoldVector(tNumVectors, tNumControls);
    tGoldVector(tVectorIndex, 0) = 2.;
    tGoldVector(tVectorIndex, 1) = -8.;
    PlatoTest::checkMultiVectorData(tHessianTimesVector, tGoldVector);
}

TEST(PlatoTest, Preconditioner)
{
    Plato::IdentityPreconditioner<double> tPreconditioner;

    const double tValue = 1;
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    const size_t tVectorIndex = 0;
    Plato::StandardMultiVector<double> tOutput(tNumVectors, tNumControls);
    Plato::StandardMultiVector<double> tVector(tNumVectors, tNumControls, tValue);
    tVector(tVectorIndex, 1) = -2.;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tValue);

    // TEST APPLY PRECONDITIONER AND APPLY INVERSE PRECONDITIONER FUNCTIONS
    tPreconditioner.applyInvPreconditioner(tControl, tVector, tOutput);
    PlatoTest::checkMultiVectorData(tOutput, tVector);
    Plato::fill(0., tOutput);
    tPreconditioner.applyPreconditioner(tControl, tVector, tOutput);
    PlatoTest::checkMultiVectorData(tOutput, tVector);
}

TEST(PlatoTest, CriterionList)
{
    Plato::CriterionList<double> tList;
    size_t tGoldInteger = 0;
    EXPECT_EQ(tGoldInteger, tList.size());

    std::shared_ptr<Plato::Circle<double>> tCircle = std::make_shared<Plato::Circle<double>>();
    std::shared_ptr<Plato::Radius<double>> tRadius = std::make_shared<Plato::Radius<double>>();
    tList.add(tCircle);
    tGoldInteger = 1;
    EXPECT_EQ(tGoldInteger, tList.size());
    tList.add(tRadius);
    tGoldInteger = 2;
    EXPECT_EQ(tGoldInteger, tList.size());

    // ** TEST FIRST CRITERION OBJECTIVE **
    const double tValue = 1;
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tValue);

    size_t tCriterionIndex = 0;
    double tOutput = tList[tCriterionIndex].value(tControl);

    double tGoldScalar = 2;
    double tTolerance = 1e-6;
    EXPECT_NEAR(tGoldScalar, tOutput, tTolerance);

    // TEST FIRST CRITERION GRADIENT
    Plato::StandardMultiVector<double> tGradient(tNumVectors, tNumControls);
    tList[tCriterionIndex].gradient(tControl, tGradient);
    Plato::StandardMultiVector<double> tGoldVector(tNumVectors, tNumControls);
    const size_t tVectorIndex = 0;
    tGoldVector(tVectorIndex, 1) = -4;
    PlatoTest::checkMultiVectorData(tGradient, tGoldVector);

    // TEST FIRST CRITERION HESSIAN TIMES VECTOR FUNCTION
    Plato::StandardMultiVector<double> tVector(tNumVectors, tNumControls, tValue);
    tVector(tVectorIndex, 1) = -2.;
    Plato::StandardMultiVector<double> tHessianTimesVector(tNumVectors, tNumControls);
    tList[tCriterionIndex].hessian(tControl, tVector, tHessianTimesVector);
    tGoldVector(tVectorIndex, 0) = 2.;
    tGoldVector(tVectorIndex, 1) = -8.;
    PlatoTest::checkMultiVectorData(tHessianTimesVector, tGoldVector);

    // ** TEST SECOND CRITERION OBJECTIVE **
    tCriterionIndex = 1;
    Plato::fill(0.5, tControl);
    tOutput = tList[tCriterionIndex].value(tControl);
    tGoldScalar = -0.5;
    EXPECT_NEAR(tGoldScalar, tOutput, tTolerance);

    // TEST SECOND CRITERION GRADIENT
    Plato::fill(0., tGradient);
    tList[tCriterionIndex].gradient(tControl, tGradient);
    Plato::fill(1., tGoldVector);
    PlatoTest::checkMultiVectorData(tGradient, tGoldVector);

    // TEST SECOND HESSIAN TIMES VECTOR FUNCTION
    Plato::fill(0.5, tVector);
    tVector(tVectorIndex, 1) = -2.;
    Plato::fill(0., tHessianTimesVector);
    tList[tCriterionIndex].hessian(tControl, tVector, tHessianTimesVector);
    tGoldVector(tVectorIndex, 0) = 1.0;
    tGoldVector(tVectorIndex, 1) = -4.0;
    PlatoTest::checkMultiVectorData(tHessianTimesVector, tGoldVector);

    // **** TEST CREATE FUNCTION ****
    std::shared_ptr<Plato::CriterionList<double>> tCopy = tList.create();
    // FIRST OBJECTIVE
    tCriterionIndex = 0;
    Plato::fill(1.0, tControl);
    tOutput = tCopy->operator [](tCriterionIndex).value(tControl);
    tGoldScalar = 2;
    EXPECT_NEAR(tGoldScalar, tOutput, tTolerance);
    // SECOND OBJECTIVE
    tCriterionIndex = 1;
    Plato::fill(0.5, tControl);
    tOutput = tCopy->operator [](tCriterionIndex).value(tControl);
    tGoldScalar = -0.5;
    EXPECT_NEAR(tGoldScalar, tOutput, tTolerance);
}

TEST(PlatoTest, GradientOperatorList)
{
    std::shared_ptr<Plato::Circle<double>> tCircle = std::make_shared<Plato::Circle<double>>();
    std::shared_ptr<Plato::AnalyticalGradient<double>> tCircleGradient =
            std::make_shared<Plato::AnalyticalGradient<double>>(tCircle);
    std::shared_ptr<Plato::Radius<double>> tRadius = std::make_shared<Plato::Radius<double>>();
    std::shared_ptr<Plato::AnalyticalGradient<double>> tRadiusGradient =
            std::make_shared<Plato::AnalyticalGradient<double>>(tRadius);
    Plato::GradientOperatorList<double> tList;

    // ********* TEST ADD FUNCTION *********
    tList.add(tCircleGradient);
    size_t tIntegerGold = 1;
    EXPECT_EQ(tIntegerGold, tList.size());

    tList.add(tRadiusGradient);
    tIntegerGold = 2;
    EXPECT_EQ(tIntegerGold, tList.size());

    // ********* ALLOCATE DATA STRUCTURES FOR TEST *********
    double tValue = 1;
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tValue);

    // ********* TEST OPERATOR[] - FIRST CRITERION *********
    size_t tVectorIndex = 0;
    Plato::StandardMultiVector<double> tOutput(tNumVectors, tNumControls);
    size_t tGradientOperatorIndex = 0;
    tList[tGradientOperatorIndex].compute(tControl, tOutput);

    Plato::StandardMultiVector<double> tGoldVector(tNumVectors, tNumControls);
    tGoldVector(tVectorIndex, 0) = 0.0;
    tGoldVector(tVectorIndex, 1) = -4.0;
    PlatoTest::checkMultiVectorData(tOutput, tGoldVector);

    // ********* TEST OPERATOR[] - SECOND CRITERION *********
    tValue = 0.5;
    Plato::fill(tValue, tControl);
    Plato::fill(0., tOutput);
    tGradientOperatorIndex = 1;
    tList[tGradientOperatorIndex].compute(tControl, tOutput);
    tGoldVector(tVectorIndex, 0) = 1.0;
    tGoldVector(tVectorIndex, 1) = 1.0;
    PlatoTest::checkMultiVectorData(tOutput, tGoldVector);

    // ********* TEST PTR - FIRST CRITERION *********
    tValue = 1.0;
    Plato::fill(tValue, tControl);
    Plato::fill(0., tOutput);
    tGradientOperatorIndex = 0;
    tList.ptr(tGradientOperatorIndex)->compute(tControl, tOutput);
    tGoldVector(tVectorIndex, 0) = 0.0;
    tGoldVector(tVectorIndex, 1) = -4.0;
    PlatoTest::checkMultiVectorData(tOutput, tGoldVector);

    // ********* TEST PTR - SECOND CRITERION *********
    tValue = 0.5;
    Plato::fill(tValue, tControl);
    Plato::fill(0., tOutput);
    tGradientOperatorIndex = 1;
    tList.ptr(tGradientOperatorIndex)->compute(tControl, tOutput);
    tGoldVector(tVectorIndex, 0) = 1.0;
    tGoldVector(tVectorIndex, 1) = 1.0;
    PlatoTest::checkMultiVectorData(tOutput, tGoldVector);

    // ********* TEST CREATE - FIRST CRITERION *********
    std::shared_ptr<Plato::GradientOperatorList<double>> tListCopy = tList.create();

    tValue = 1.0;
    Plato::fill(tValue, tControl);
    Plato::fill(0., tOutput);
    tGradientOperatorIndex = 0;
    tListCopy->ptr(tGradientOperatorIndex)->compute(tControl, tOutput);
    tGoldVector(tVectorIndex, 0) = 0.0;
    tGoldVector(tVectorIndex, 1) = -4.0;
    PlatoTest::checkMultiVectorData(tOutput, tGoldVector);

    // ********* TEST CREATE - SECOND CRITERION *********
    tValue = 0.5;
    Plato::fill(tValue, tControl);
    Plato::fill(0., tOutput);
    tGradientOperatorIndex = 1;
    tListCopy->ptr(tGradientOperatorIndex)->compute(tControl, tOutput);
    tGoldVector(tVectorIndex, 0) = 1.0;
    tGoldVector(tVectorIndex, 1) = 1.0;
    PlatoTest::checkMultiVectorData(tOutput, tGoldVector);
}

TEST(PlatoTest, LinearOperatorList)
{
    // ********* ALLOCATE RADIUS AND CIRCLE CRITERIA *********
    std::shared_ptr<Plato::Circle<double>> tCircle = std::make_shared<Plato::Circle<double>>();
    std::shared_ptr<Plato::AnalyticalHessian<double>> tCircleHessian =
            std::make_shared<Plato::AnalyticalHessian<double>>(tCircle);

    std::shared_ptr<Plato::Radius<double>> tRadius = std::make_shared<Plato::Radius<double>>();
    std::shared_ptr<Plato::AnalyticalHessian<double>> tRadiusHessian =
            std::make_shared<Plato::AnalyticalHessian<double>>(tRadius);

    // ********* TEST ADD FUNCTION *********
    Plato::LinearOperatorList<double> tList;
    tList.add(tCircleHessian);
    size_t tIntegerGold = 1;
    EXPECT_EQ(tIntegerGold, tList.size());

    tList.add(tRadiusHessian);
    tIntegerGold = 2;
    EXPECT_EQ(tIntegerGold, tList.size());

    // ********* ALLOCATE DATA STRUCTURES FOR TEST *********
    double tValue = 1;
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tValue);

    // ********* TEST OPERATOR[] - FIRST CRITERION *********
    size_t tVectorIndex = 0;
    Plato::StandardMultiVector<double> tVector(tNumVectors, tNumControls, tValue);
    tVector(tVectorIndex, 1) = -2.;
    Plato::StandardMultiVector<double> tOutput(tNumVectors, tNumControls);
    size_t tLinearOperatorIndex = 0;
    tList[tLinearOperatorIndex].apply(tControl, tVector, tOutput);

    Plato::StandardMultiVector<double> tGoldVector(tNumVectors, tNumControls);
    tGoldVector(tVectorIndex, 0) = 2.;
    tGoldVector(tVectorIndex, 1) = -8.;
    PlatoTest::checkMultiVectorData(tOutput, tGoldVector);

    // ********* TEST OPERATOR[] - SECOND CRITERION *********
    Plato::fill(0., tOutput);
    tValue = 0.5;
    Plato::fill(tValue, tVector);
    tVector(tVectorIndex, 1) = -2.;
    Plato::fill(tValue, tControl);
    tLinearOperatorIndex = 1;
    tList[tLinearOperatorIndex].apply(tControl, tVector, tOutput);
    tGoldVector(tVectorIndex, 0) = 1.0;
    tGoldVector(tVectorIndex, 1) = -4.0;
    PlatoTest::checkMultiVectorData(tOutput, tGoldVector);

    // ********* TEST PTR - FIRST CRITERION *********
    Plato::fill(0., tOutput);
    tValue = 1.0;
    Plato::fill(tValue, tVector);
    tVector(tVectorIndex, 1) = -2.;
    Plato::fill(tValue, tControl);
    tLinearOperatorIndex = 0;
    tList.ptr(tLinearOperatorIndex)->apply(tControl, tVector, tOutput);
    tGoldVector(tVectorIndex, 0) = 2.;
    tGoldVector(tVectorIndex, 1) = -8.;
    PlatoTest::checkMultiVectorData(tOutput, tGoldVector);

    // ********* TEST PTR - SECOND CRITERION *********
    Plato::fill(0., tOutput);
    tValue = 0.5;
    Plato::fill(tValue, tVector);
    tVector(tVectorIndex, 1) = -2.;
    Plato::fill(tValue, tControl);
    tLinearOperatorIndex = 1;
    tList.ptr(tLinearOperatorIndex)->apply(tControl, tVector, tOutput);
    tGoldVector(tVectorIndex, 0) = 1.0;
    tGoldVector(tVectorIndex, 1) = -4.0;
    PlatoTest::checkMultiVectorData(tOutput, tGoldVector);

    // ********* TEST CREATE - FIRST CRITERION *********
    std::shared_ptr<Plato::LinearOperatorList<double>> tListCopy = tList.create();

    Plato::fill(0., tOutput);
    tValue = 1.0;
    Plato::fill(tValue, tVector);
    tVector(tVectorIndex, 1) = -2.;
    Plato::fill(tValue, tControl);
    tLinearOperatorIndex = 0;
    tListCopy->ptr(tLinearOperatorIndex)->apply(tControl, tVector, tOutput);
    tGoldVector(tVectorIndex, 0) = 2.;
    tGoldVector(tVectorIndex, 1) = -8.;
    PlatoTest::checkMultiVectorData(tOutput, tGoldVector);

    // ********* TEST CREATE - SECOND CRITERION *********
    Plato::fill(0., tOutput);
    tValue = 0.5;
    Plato::fill(tValue, tVector);
    tVector(tVectorIndex, 1) = -2.;
    Plato::fill(tValue, tControl);
    tLinearOperatorIndex = 1;
    tListCopy->ptr(tLinearOperatorIndex)->apply(tControl, tVector, tOutput);
    tGoldVector(tVectorIndex, 0) = 1.0;
    tGoldVector(tVectorIndex, 1) = -4.0;
    PlatoTest::checkMultiVectorData(tOutput, tGoldVector);
}

TEST(PlatoTest, AugmentedLagrangianStageMng)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumControls = 2;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumControls);

    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT *********
    std::shared_ptr<Plato::Circle<double>> tCircle = std::make_shared<Plato::Circle<double>>();
    std::shared_ptr<Plato::Radius<double>> tRadius = std::make_shared<Plato::Radius<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraint = std::make_shared<Plato::CriterionList<double>>();
    tConstraint->add(tRadius);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    Plato::AugmentedLagrangianStageMng<double> tStageMng(tDataFactory, tCircle, tConstraint);

    // ********* TEST AUGMENTED LAGRANGIAN STAGE MANAGER FUNCTIONALITIES *********
    size_t tIntegerGold = 0;
    EXPECT_EQ(tIntegerGold, tStageMng.getNumObjectiveFunctionEvaluations());
    EXPECT_EQ(tIntegerGold, tStageMng.getNumObjectiveGradientEvaluations());
    EXPECT_EQ(tIntegerGold, tStageMng.getNumObjectiveHessianEvaluations());
    for(size_t tIndex = 0; tIndex < tConstraint->size(); tIndex++)
    {
        EXPECT_EQ(tIntegerGold, tStageMng.getNumConstraintEvaluations(tIndex));
        EXPECT_EQ(tIntegerGold, tStageMng.getNumConstraintHessianEvaluations(tIndex));
        EXPECT_EQ(tIntegerGold, tStageMng.getNumConstraintGradientEvaluations(tIndex));
    }

    double tScalarGold = 0.1;
    double tTolerance = 1e-6;
    EXPECT_NEAR(tScalarGold, tStageMng.getPenaltyParameter(), tTolerance);
    tScalarGold = std::numeric_limits<double>::max();
    EXPECT_NEAR(tScalarGold, tStageMng.getNormObjectiveFunctionGradient(), tTolerance);

    // ********* TEST AUGMENTED LAGRANGIAN STAGE MANAGER OBJECTIVE EVALUATION *********
    double tValue = 1;
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tValue);
    tValue = tStageMng.evaluateObjective(tControl);
    tScalarGold = 8;
    EXPECT_NEAR(tScalarGold, tValue, tTolerance);
    tIntegerGold = 1;
    const size_t tConstraintIndex = 0;;
    EXPECT_EQ(tIntegerGold, tStageMng.getNumObjectiveFunctionEvaluations());
    EXPECT_EQ(tIntegerGold, tStageMng.getNumConstraintEvaluations(tConstraintIndex));

    // ********* TEST AUGMENTED LAGRANGIAN STAGE MANAGER - UPDATE LAGRANGE MULTIPLIERS *********
    tScalarGold = 0.1;
    EXPECT_NEAR(tScalarGold, tStageMng.getPenaltyParameter(), tTolerance);
    Plato::StandardMultiVector<double> tLagrangeMultipliers(tNumVectors, tNumDuals);
    tStageMng.getLagrangeMultipliers(tLagrangeMultipliers);
    tScalarGold = 1;
    Plato::StandardMultiVector<double> tLagrangeMultipliersGold(tNumVectors, tNumDuals, tScalarGold);
    PlatoTest::checkMultiVectorData(tLagrangeMultipliers, tLagrangeMultipliersGold);

    // ********* TEST AUGMENTED LAGRANGIAN STAGE MANAGER - UPDATE LAGRANGE MULTIPLIERS & EVALUATE CONSTRAINT *********
    tValue = 0.5;
    Plato::fill(tValue, tControl);
    tStageMng.evaluateConstraint(tControl);
    tIntegerGold = 2;
    EXPECT_EQ(tIntegerGold, tStageMng.getNumConstraintEvaluations(tConstraintIndex));

    tStageMng.updateCurrentCriteriaValues();
    Plato::StandardMultiVector<double> tCurrentConstraintValues(tNumVectors, tNumDuals);
    tStageMng.getCurrentConstraintValues(tCurrentConstraintValues);
    tValue = -0.5;
    Plato::StandardMultiVector<double> tCurrentConstraintValuesGold(tNumVectors, tNumDuals, tValue);
    PlatoTest::checkMultiVectorData(tCurrentConstraintValues, tCurrentConstraintValuesGold);

    tScalarGold = 0.1;
    EXPECT_NEAR(tScalarGold, tStageMng.getPenaltyParameter(), tTolerance);
    tStageMng.getLagrangeMultipliers(tLagrangeMultipliers);
    tScalarGold = 0.1;
    EXPECT_NEAR(tScalarGold, tStageMng.getPenaltyParameter(), tTolerance);

    tValue = 1;
    Plato::fill(tValue, tLagrangeMultipliersGold);
    PlatoTest::checkMultiVectorData(tLagrangeMultipliers, tLagrangeMultipliersGold);

    // ********* TEST AUGMENTED LAGRANGIAN STAGE MANAGER - COMPUTE GRADIENT *********
    Plato::StandardMultiVector<double> tOutput(tNumVectors, tNumControls);
    tStageMng.computeGradient(tControl, tOutput);
    tScalarGold = 6.0827625302982193;
    EXPECT_NEAR(tScalarGold, tStageMng.getNormObjectiveFunctionGradient(), tTolerance);
    tIntegerGold = 1;
    EXPECT_EQ(tIntegerGold, tStageMng.getNumObjectiveGradientEvaluations());
    EXPECT_EQ(tIntegerGold, tStageMng.getNumConstraintGradientEvaluations(tConstraintIndex));
    Plato::StandardMultiVector<double> tGoldMultiVector(tNumVectors, tNumControls);
    const size_t tVectorIndex = 0;
    tGoldMultiVector(tVectorIndex, 0) = -5;
    tGoldMultiVector(tVectorIndex, 1) = -10;
    PlatoTest::checkMultiVectorData(tOutput, tGoldMultiVector);

    // ********* TEST AUGMENTED LAGRANGIAN STAGE MANAGER - APPLY VECTOR TO HESSIAN *********
    tValue = 1.0;
    Plato::StandardMultiVector<double> tVector(tNumVectors, tNumControls, tValue);
    tStageMng.applyVectorToHessian(tControl, tVector, tOutput);
    EXPECT_EQ(tIntegerGold, tStageMng.getNumObjectiveHessianEvaluations());
    EXPECT_EQ(tIntegerGold, tStageMng.getNumConstraintHessianEvaluations(tConstraintIndex));
    tIntegerGold = 1;
    EXPECT_EQ(tIntegerGold, tStageMng.getNumConstraintGradientEvaluations(tConstraintIndex));
    tGoldMultiVector(tVectorIndex, 0) = 14;
    tGoldMultiVector(tVectorIndex, 1) = 16;
    PlatoTest::checkMultiVectorData(tOutput, tGoldMultiVector);

    // ********* TEST AUGMENTED LAGRANGIAN STAGE MANAGER - APPLY PRECONDITIONER *********
    tStageMng.applyVectorToPreconditioner(tControl, tVector, tOutput);
    tGoldMultiVector(tVectorIndex, 0) = 1.0;
    tGoldMultiVector(tVectorIndex, 1) = 1.0;
    PlatoTest::checkMultiVectorData(tOutput, tGoldMultiVector);
    tStageMng.applyVectorToInvPreconditioner(tControl, tVector, tOutput);
    PlatoTest::checkMultiVectorData(tOutput, tGoldMultiVector);
}

TEST(PlatoTest, SteihaugTointSolver)
{
    // ********* ALLOCATE DATA FACTORY *********
    Plato::DataFactory<double> tDataFactory;
    const size_t tNumDuals = 1;
    const size_t tNumControls = 2;
    tDataFactory.allocateDual(tNumDuals);
    tDataFactory.allocateControl(tNumControls);

    // ********* ALLOCATE SOLVER *********
    Plato::ProjectedSteihaugTointPcg<double> tSolver(tDataFactory);

    // ********* TEST MAX NUM ITERATIONS FUNCTIONS *********
    size_t tIntegerGold = 200;
    EXPECT_EQ(tIntegerGold, tSolver.getMaxNumIterations());
    tIntegerGold = 300;
    tSolver.setMaxNumIterations(tIntegerGold);
    EXPECT_EQ(tIntegerGold, tSolver.getMaxNumIterations());

    // ********* TEST NUM ITERATIONS DONE FUNCTIONS *********
    tIntegerGold = 0;
    EXPECT_EQ(tIntegerGold, tSolver.getNumIterationsDone());
    tIntegerGold = 2;
    tSolver.setNumIterationsDone(tIntegerGold);
    EXPECT_EQ(tIntegerGold, tSolver.getNumIterationsDone());

    // ********* TEST SOLVER TOLERANCE FUNCTIONS *********
    double tScalarGold = 1e-8;
    EXPECT_EQ(tScalarGold, tSolver.getSolverTolerance());
    tScalarGold = 0.2;
    tSolver.setSolverTolerance(tScalarGold);
    EXPECT_EQ(tScalarGold, tSolver.getSolverTolerance());

    // ********* TEST SET TRUST REGION RADIUS FUNCTIONS *********
    tScalarGold = 0;
    EXPECT_EQ(tScalarGold, tSolver.getTrustRegionRadius());
    tScalarGold = 2;
    tSolver.setTrustRegionRadius(tScalarGold);
    EXPECT_EQ(tScalarGold, tSolver.getTrustRegionRadius());

    // ********* TEST RESIDUAL NORM FUNCTIONS *********
    tScalarGold = 0;
    EXPECT_EQ(tScalarGold, tSolver.getNormResidual());
    tScalarGold = 1e-2;
    tSolver.setNormResidual(tScalarGold);
    EXPECT_EQ(tScalarGold, tSolver.getNormResidual());

    // ********* TEST RELATIVE TOLERANCE FUNCTIONS *********
    tScalarGold = 1e-1;
    EXPECT_EQ(tScalarGold, tSolver.getRelativeTolerance());
    tScalarGold = 1e-3;
    tSolver.setRelativeTolerance(tScalarGold);
    EXPECT_EQ(tScalarGold, tSolver.getRelativeTolerance());

    // ********* TEST RELATIVE TOLERANCE EXPONENTIAL FUNCTIONS *********
    tScalarGold = 0.5;
    EXPECT_EQ(tScalarGold, tSolver.getRelativeToleranceExponential());
    tScalarGold = 0.75;
    tSolver.setRelativeToleranceExponential(tScalarGold);
    EXPECT_EQ(tScalarGold, tSolver.getRelativeToleranceExponential());

    // ********* TEST RELATIVE TOLERANCE EXPONENTIAL FUNCTIONS *********
    Plato::krylov_solver::stop_t tStopGold = Plato::krylov_solver::stop_t::MAX_ITERATIONS;
    EXPECT_EQ(tStopGold, tSolver.getStoppingCriterion());
    tStopGold = Plato::krylov_solver::stop_t::TRUST_REGION_RADIUS;
    tSolver.setStoppingCriterion(tStopGold);
    EXPECT_EQ(tStopGold, tSolver.getStoppingCriterion());

    // ********* TEST INVALID CURVATURE FUNCTION *********
    double tScalarValue = -1;
    EXPECT_TRUE(tSolver.invalidCurvatureDetected(tScalarValue));
    EXPECT_EQ(Plato::krylov_solver::stop_t::NEGATIVE_CURVATURE, tSolver.getStoppingCriterion());
    tScalarValue = 0;
    EXPECT_TRUE(tSolver.invalidCurvatureDetected(tScalarValue));
    EXPECT_EQ(Plato::krylov_solver::stop_t::ZERO_CURVATURE, tSolver.getStoppingCriterion());
    tScalarValue = std::numeric_limits<double>::infinity();
    EXPECT_TRUE(tSolver.invalidCurvatureDetected(tScalarValue));
    EXPECT_EQ(Plato::krylov_solver::stop_t::INF_CURVATURE, tSolver.getStoppingCriterion());
    tScalarValue = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(tSolver.invalidCurvatureDetected(tScalarValue));
    EXPECT_EQ(Plato::krylov_solver::stop_t::NaN_CURVATURE, tSolver.getStoppingCriterion());
    tScalarValue = 1;
    EXPECT_FALSE(tSolver.invalidCurvatureDetected(tScalarValue));

    // ********* TEST TOLERANCE SATISFIED FUNCTION *********
    tScalarValue = 5e-9;
    EXPECT_TRUE(tSolver.toleranceSatisfied(tScalarValue));
    EXPECT_EQ(Plato::krylov_solver::stop_t::TOLERANCE, tSolver.getStoppingCriterion());
    tScalarValue = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(tSolver.toleranceSatisfied(tScalarValue));
    EXPECT_EQ(Plato::krylov_solver::stop_t::NaN_NORM_RESIDUAL, tSolver.getStoppingCriterion());
    tScalarValue = std::numeric_limits<double>::infinity();
    EXPECT_TRUE(tSolver.toleranceSatisfied(tScalarValue));
    EXPECT_EQ(Plato::krylov_solver::stop_t::INF_NORM_RESIDUAL, tSolver.getStoppingCriterion());
    tScalarValue = 1;
    EXPECT_FALSE(tSolver.toleranceSatisfied(tScalarValue));

    // ********* TEST COMPUTE STEIHAUG TOINT STEP FUNCTION *********
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tNewtonStep(tNumVectors, tNumControls);
    tNewtonStep(0,0) = 0.345854922279793;
    tNewtonStep(0,1) = 1.498704663212435;
    Plato::StandardMultiVector<double> tConjugateDirection(tNumVectors, tNumControls);
    tConjugateDirection(0,0) = 1.5;
    tConjugateDirection(0,1) = 6.5;
    Plato::StandardMultiVector<double> tPrecTimesNewtonStep(tNumVectors, tNumControls);
    tPrecTimesNewtonStep(0,0) = 0.345854922279793;
    tPrecTimesNewtonStep(0,1) = 1.498704663212435;
    Plato::StandardMultiVector<double> tPrecTimesConjugateDirection(tNumVectors, tNumControls);
    tPrecTimesConjugateDirection(0,0) = 1.5;
    tPrecTimesConjugateDirection(0,1) = 6.5;

    tScalarValue = 0.833854004007896;
    tSolver.setTrustRegionRadius(tScalarValue);
    tScalarValue = tSolver.computeSteihaugTointStep(tNewtonStep, tConjugateDirection, tPrecTimesNewtonStep, tPrecTimesConjugateDirection);

    double tTolerance = 1e-6;
    tScalarGold = -0.105569948186529;
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);
}

TEST(PlatoTest, ProjectedSteihaugTointPcg)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumControls = 2;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumControls);

    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT *********
    std::shared_ptr<Plato::Circle<double>> tCircle = std::make_shared<Plato::Circle<double>>();
    std::shared_ptr<Plato::Radius<double>> tRadius = std::make_shared<Plato::Radius<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraint = std::make_shared<Plato::CriterionList<double>>();
    tConstraint->add(tRadius);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    Plato::AugmentedLagrangianStageMng<double> tStageMng(tDataFactory, tCircle, tConstraint);

    // ********* Allocate Trust Region Algorithm Data Manager *********
    Plato::TrustRegionAlgorithmDataMng<double> tDataMng(tDataFactory);

    // ********* SET INITIAL DATA MANAGER VALUES *********
    double tScalarValue = 0.5;
    tDataMng.setInitialGuess(tScalarValue);
    tScalarValue = tStageMng.evaluateObjective(tDataMng.getCurrentControl());
    tStageMng.updateCurrentCriteriaValues();
    tDataMng.setCurrentObjectiveFunctionValue(tScalarValue);
    const double tTolerance = 1e-6;
    double tScalarGoldValue = 5.5;
    EXPECT_NEAR(tScalarGoldValue, tDataMng.getCurrentObjectiveFunctionValue(), tTolerance);

    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tVector(tNumVectors, tNumControls);
    tStageMng.computeGradient(tDataMng.getCurrentControl(), tVector);
    tDataMng.setCurrentGradient(tVector);
    tDataMng.computeNormProjectedGradient();
    tScalarGoldValue = 11.180339887498949;
    EXPECT_NEAR(tScalarGoldValue, tDataMng.getNormProjectedGradient(), tTolerance);
    tVector(0, 0) = -5;
    tVector(0, 1) = -10;
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentGradient(), tVector);

    // ********* ALLOCATE SOLVER DATA STRUCTURE *********
    Plato::ProjectedSteihaugTointPcg<double> tSolver(*tDataFactory);

    // ********* CONVERGENCE: SOLVER TOLERANCE MET *********
    tScalarValue = tDataMng.getNormProjectedGradient();
    tSolver.setTrustRegionRadius(tScalarValue);
    tSolver.solve(tStageMng, tDataMng);
    size_t tIntegerGoldValue = 2;
    EXPECT_EQ(tIntegerGoldValue, tSolver.getNumIterationsDone());
    EXPECT_EQ(Plato::krylov_solver::stop_t::NEGATIVE_CURVATURE, tSolver.getStoppingCriterion());
    EXPECT_FALSE(tSolver.getNormResidual() < tSolver.getSolverTolerance());
    tVector(0, 0) = -6.9709542221544076;
    tVector(0, 1) = 8.7410409696229934;
    PlatoTest::checkMultiVectorData(tDataMng.getTrialStep(), tVector);

    // ********* CONVERGENCE: MAX NUMBER OF ITERATIONS *********
    tSolver.setMaxNumIterations(1);
    tSolver.setSolverTolerance(1e-15);
    tSolver.solve(tStageMng, tDataMng);
    EXPECT_EQ(Plato::krylov_solver::stop_t::MAX_ITERATIONS, tSolver.getStoppingCriterion());
    EXPECT_FALSE(tSolver.getNormResidual() < tSolver.getSolverTolerance());
    tVector(0, 0) = 0.36764705882352944;
    tVector(0, 1) =  0.73529411764705888;
    PlatoTest::checkMultiVectorData(tDataMng.getTrialStep(), tVector);

    // ********* CONVERGENCE: TRUST REGION RADIUS VIOLATED *********
    tSolver.setMaxNumIterations(10);
    tSolver.setTrustRegionRadius(0.3);
    tSolver.solve(tStageMng, tDataMng);
    tIntegerGoldValue = 1;
    EXPECT_EQ(tIntegerGoldValue, tSolver.getNumIterationsDone());
    EXPECT_EQ(Plato::krylov_solver::stop_t::TRUST_REGION_RADIUS, tSolver.getStoppingCriterion());
    EXPECT_FALSE(tSolver.getNormResidual() < tSolver.getSolverTolerance());
    tVector(0, 0) = 0.13416407864998731;
    tVector(0, 1) = 0.26832815729997461;
    PlatoTest::checkMultiVectorData(tDataMng.getTrialStep(), tVector);
}

TEST(PlatoTest, TrustRegionStepMng)
{
    // ********* ALLOCATE DATA FACTORY *********
    Plato::DataFactory<double> tDataFactory;
    const size_t tNumDuals = 1;
    const size_t tNumControls = 2;
    tDataFactory.allocateDual(tNumDuals);
    tDataFactory.allocateControl(tNumControls);

    Plato::KelleySachsStepMng<double> tStepMng(tDataFactory);

    // ********* TEST ACTUAL REDUCTION FUNCTIONS *********
    double tTolerance = 1e-6;
    double tScalarGoldValue = 0;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getActualReduction(), tTolerance);
    tScalarGoldValue = 0.45;
    tStepMng.setActualReduction(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getActualReduction(), tTolerance);

    // ********* TEST TRUST REGION RADIUS FUNCTIONS *********
    tScalarGoldValue = 1e3;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getTrustRegionRadius(), tTolerance);
    tScalarGoldValue = 1e2;
    tStepMng.setTrustRegionRadius(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getTrustRegionRadius(), tTolerance);

    // ********* TEST TRUST REGION CONTRACTION FUNCTIONS *********
    tScalarGoldValue = 0.75;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getTrustRegionContraction(), tTolerance);
    tScalarGoldValue = 0.2;
    tStepMng.setTrustRegionContraction(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getTrustRegionContraction(), tTolerance);

    // ********* TEST TRUST REGION EXPANSION FUNCTIONS *********
    tScalarGoldValue = 2;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getTrustRegionExpansion(), tTolerance);
    tScalarGoldValue = 8;
    tStepMng.setTrustRegionExpansion(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getTrustRegionExpansion(), tTolerance);

    // ********* TEST MIN TRUST REGION RADIUS FUNCTIONS *********
    tScalarGoldValue = 1e-8;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getMinTrustRegionRadius(), tTolerance);
    tScalarGoldValue = 1e-2;
    tStepMng.setMinTrustRegionRadius(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getMinTrustRegionRadius(), tTolerance);

    // ********* TEST MAX TRUST REGION RADIUS FUNCTIONS *********
    tScalarGoldValue = 1e3;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getMaxTrustRegionRadius(), tTolerance);
    tScalarGoldValue = 1e1;
    tStepMng.setMaxTrustRegionRadius(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getMaxTrustRegionRadius(), tTolerance);

    // ********* TEST GRADIENT INEXACTNESS FUNCTIONS *********
    tScalarGoldValue = 1;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getGradientInexactnessToleranceConstant(), tTolerance);
    tScalarGoldValue = 2;
    tStepMng.setGradientInexactnessToleranceConstant(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getGradientInexactnessToleranceConstant(), tTolerance);
    // TEST INEXACTNESS TOLERANCE: SELECT CURRENT TRUST REGION RADIUS
    tScalarGoldValue = std::numeric_limits<double>::max();
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getGradientInexactnessTolerance(), tTolerance);
    tScalarGoldValue = 1e3;
    tStepMng.updateGradientInexactnessTolerance(tScalarGoldValue);
    tScalarGoldValue = 200;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getGradientInexactnessTolerance(), tTolerance);
    // TEST INEXACTNESS TOLERANCE: SELECT USER INPUT
    tScalarGoldValue = 1e1;
    tStepMng.updateGradientInexactnessTolerance(tScalarGoldValue);
    tScalarGoldValue = 20;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getGradientInexactnessTolerance(), tTolerance);

    // ********* TEST OBJECTIVE INEXACTNESS FUNCTIONS *********
    tScalarGoldValue = 1;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getObjectiveInexactnessToleranceConstant(), tTolerance);
    tScalarGoldValue = 3;
    tStepMng.setObjectiveInexactnessToleranceConstant(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getObjectiveInexactnessToleranceConstant(), tTolerance);
    // TEST INEXACTNESS TOLERANCE
    tScalarGoldValue = std::numeric_limits<double>::max();
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getObjectiveInexactnessTolerance(), tTolerance);
    tScalarGoldValue = 100;
    tStepMng.updateObjectiveInexactnessTolerance(tScalarGoldValue);
    tScalarGoldValue = 30;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getObjectiveInexactnessTolerance(), tTolerance);

    // ********* TEST ACTUAL OVER PREDICTED REDUCTION BOUND FUNCTIONS *********
    tScalarGoldValue = 0.25;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getActualOverPredictedReductionMidBound(), tTolerance);
    tScalarGoldValue = 0.4;
    tStepMng.setActualOverPredictedReductionMidBound(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getActualOverPredictedReductionMidBound(), tTolerance);

    tScalarGoldValue = 0.1;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getActualOverPredictedReductionLowerBound(), tTolerance);
    tScalarGoldValue = 0.05;
    tStepMng.setActualOverPredictedReductionLowerBound(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getActualOverPredictedReductionLowerBound(), tTolerance);

    tScalarGoldValue = 0.75;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getActualOverPredictedReductionUpperBound(), tTolerance);
    tScalarGoldValue = 0.8;
    tStepMng.setActualOverPredictedReductionUpperBound(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getActualOverPredictedReductionUpperBound(), tTolerance);

    // ********* TEST PREDICTED REDUCTION FUNCTIONS *********
    tScalarGoldValue = 0;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getPredictedReduction(), tTolerance);
    tScalarGoldValue = 0.12;
    tStepMng.setPredictedReduction(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getPredictedReduction(), tTolerance);

    // ********* TEST MIN COSINE ANGLE TOLERANCE FUNCTIONS *********
    tScalarGoldValue = 1e-2;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getMinCosineAngleTolerance(), tTolerance);
    tScalarGoldValue = 0.1;
    tStepMng.setMinCosineAngleTolerance(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getMinCosineAngleTolerance(), tTolerance);

    // ********* TEST ACTUAL OVER PREDICTED REDUCTION FUNCTIONS *********
    tScalarGoldValue = 0;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getActualOverPredictedReduction(), tTolerance);
    tScalarGoldValue = 0.23;
    tStepMng.setActualOverPredictedReduction(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getActualOverPredictedReduction(), tTolerance);

    // ********* TEST NUMBER OF TRUST REGION SUBPROBLEM ITERATIONS FUNCTIONS *********
    size_t tIntegerGoldValue = 0;
    EXPECT_EQ(tIntegerGoldValue, tStepMng.getNumTrustRegionSubProblemItrDone());
    tStepMng.updateNumTrustRegionSubProblemItrDone();
    tIntegerGoldValue = 1;
    EXPECT_EQ(tIntegerGoldValue, tStepMng.getNumTrustRegionSubProblemItrDone());

    tIntegerGoldValue = 25;
    EXPECT_EQ(tIntegerGoldValue, tStepMng.getMaxNumTrustRegionSubProblemIterations());
    tIntegerGoldValue = 50;
    tStepMng.setMaxNumTrustRegionSubProblemIterations(tIntegerGoldValue);
    EXPECT_EQ(tIntegerGoldValue, tStepMng.getMaxNumTrustRegionSubProblemIterations());

    EXPECT_TRUE(tStepMng.isInitialTrustRegionRadiusSetToNormProjectedGradient());
    tStepMng.setInitialTrustRegionRadiusSetToNormProjectedGradient(false);
    EXPECT_FALSE(tStepMng.isInitialTrustRegionRadiusSetToNormProjectedGradient());
}

TEST(PlatoTest, KelleySachsStepMng)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumControls = 2;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumControls);

    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT *********
    std::shared_ptr<Plato::Circle<double>> tCircle = std::make_shared<Plato::Circle<double>>();
    std::shared_ptr<Plato::Radius<double>> tRadius = std::make_shared<Plato::Radius<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tRadius);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    Plato::AugmentedLagrangianStageMng<double> tStageMng(tDataFactory, tCircle, tConstraintList);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    Plato::TrustRegionAlgorithmDataMng<double> tDataMng(tDataFactory);

    // ********* SET INITIAL DATA MANAGER VALUES *********
    double tScalarValue = 0.5;
    tDataMng.setInitialGuess(tScalarValue);
    tScalarValue = tStageMng.evaluateObjective(tDataMng.getCurrentControl());
    tStageMng.updateCurrentCriteriaValues();
    tDataMng.setCurrentObjectiveFunctionValue(tScalarValue);
    const double tTolerance = 1e-6;
    double tScalarGoldValue = 5.5;
    EXPECT_NEAR(tScalarGoldValue, tDataMng.getCurrentObjectiveFunctionValue(), tTolerance);

    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tVector(tNumVectors, tNumControls);
    tStageMng.computeGradient(tDataMng.getCurrentControl(), tVector);
    tDataMng.setCurrentGradient(tVector);
    tDataMng.computeNormProjectedGradient();
    double tNormProjectedGradientGold = 11.180339887498949;
    EXPECT_NEAR(tNormProjectedGradientGold, tDataMng.getNormProjectedGradient(), tTolerance);
    tDataMng.computeStationarityMeasure();
    double tStationarityMeasureGold = 11.180339887498949;
    EXPECT_NEAR(tStationarityMeasureGold, tDataMng.getStationarityMeasure(), tTolerance);
    tVector(0, 0) = -5;
    tVector(0, 1) = -10;
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentGradient(), tVector);

    // ********* ALLOCATE SOLVER *********
    Plato::ProjectedSteihaugTointPcg<double> tSolver(*tDataFactory);

    // ********* ALLOCATE STEP MANAGER *********
    Plato::KelleySachsStepMng<double> tStepMng(*tDataFactory);
    tStepMng.setTrustRegionRadius(tNormProjectedGradientGold);

    // ********* TEST CONSTANT FUNCTIONS *********
    tScalarGoldValue = 0;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getEtaConstant(), tTolerance);
    tScalarGoldValue = 0.12;
    tStepMng.setEtaConstant(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getEtaConstant(), tTolerance);

    tScalarGoldValue = 0;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getEpsilonConstant(), tTolerance);
    tScalarGoldValue = 0.11;
    tStepMng.setEpsilonConstant(tScalarGoldValue);
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getEpsilonConstant(), tTolerance);

    // ********* TEST SUBPROBLEM SOLVE *********
    tScalarValue = 0.01;
    tStepMng.setEtaConstant(tScalarValue);
    EXPECT_TRUE(tStepMng.solveSubProblem(tDataMng, tStageMng, tSolver));

    // VERIFY CURRENT SUBPROBLEM SOLVE RESULTS
    size_t tIntegerGoldValue = 12;
    EXPECT_EQ(tIntegerGoldValue, tStepMng.getNumTrustRegionSubProblemItrDone());
    tScalarGoldValue = 0.55856115689474017;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getActualOverPredictedReduction(), tTolerance);
    tScalarGoldValue = 3.3980470658923601;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getMidPointObjectiveFunctionValue(), tTolerance);
    tScalarGoldValue = 0.472203176033682;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getTrustRegionRadius(), tTolerance);
    tScalarGoldValue = -2.1019529341076399;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getActualReduction(), tTolerance);
    tScalarGoldValue = -3.763156295710246;
    EXPECT_NEAR(tScalarGoldValue, tStepMng.getPredictedReduction(), tTolerance);
    tScalarGoldValue = 0.1118033988749895;
    EXPECT_NEAR(tScalarGoldValue, tSolver.getSolverTolerance(), tTolerance);
    const Plato::MultiVector<double> & tMidControl = tStepMng.getMidPointControls();
    Plato::StandardMultiVector<double> tVectorGold(tNumVectors, tNumControls);
    tVectorGold(0, 0) = 0.71117568016052246;
    tVectorGold(0, 1) = 0.92235136032104492;
    PlatoTest::checkMultiVectorData(tMidControl, tVectorGold);
    const Plato::MultiVector<double> & tTrialStep = tDataMng.getTrialStep();
    tVectorGold(0, 0) = 0.21117568016052246;
    tVectorGold(0, 1) = 0.42235136032104492;
    PlatoTest::checkMultiVectorData(tTrialStep, tVectorGold);
}

TEST(PlatoTest, KelleySachsAlgorithm)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumControls = 2;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumControls);

    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT *********
    std::shared_ptr<Plato::Circle<double>> tCircle = std::make_shared<Plato::Circle<double>>();
    std::shared_ptr<Plato::Radius<double>> tRadius = std::make_shared<Plato::Radius<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tRadius);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<double>> tDataMng =
            std::make_shared<Plato::TrustRegionAlgorithmDataMng<double>>(tDataFactory);
    double tScalarValue = 0.5;
    tDataMng->setInitialGuess(tScalarValue);
    tScalarValue = -100;
    tDataMng->setControlLowerBounds(tScalarValue);
    tScalarValue = 100;
    tDataMng->setControlUpperBounds(tScalarValue);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<double>> tStageMng =
            std::make_shared<Plato::AugmentedLagrangianStageMng<double>>(tDataFactory, tCircle, tConstraintList);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM *********
    Plato::KelleySachsAugmentedLagrangian<double> tAlgorithm(tDataFactory, tDataMng, tStageMng);

    // TEST MAXIMUM NUMBER OF UPDATES
    size_t tIntegerGold = 10;
    EXPECT_EQ(tIntegerGold, tAlgorithm.getMaxNumUpdates());
    tIntegerGold = 50;
    tAlgorithm.setMaxNumUpdates(tIntegerGold);
    EXPECT_EQ(tIntegerGold, tAlgorithm.getMaxNumUpdates());

    // TEST NUMBER ITERATIONS DONE
    tIntegerGold = 0;
    EXPECT_EQ(tIntegerGold, tAlgorithm.getNumIterationsDone());
    tIntegerGold = 3;
    tAlgorithm.setNumIterationsDone(tIntegerGold);
    EXPECT_EQ(tIntegerGold, tAlgorithm.getNumIterationsDone());

    // TEST NUMBER ITERATIONS DONE
    tIntegerGold = 100;
    EXPECT_EQ(tIntegerGold, tAlgorithm.getMaxNumIterations());
    tIntegerGold = 30;
    tAlgorithm.setMaxNumIterations(tIntegerGold);
    EXPECT_EQ(tIntegerGold, tAlgorithm.getMaxNumIterations());

    // TEST STOPPING CRITERIA
    Plato::algorithm::stop_t tGold = Plato::algorithm::stop_t::NOT_CONVERGED;
    EXPECT_EQ(tGold, tAlgorithm.getStoppingCriterion());
    tGold = Plato::algorithm::stop_t::NaN_NORM_TRIAL_STEP;
    tAlgorithm.setStoppingCriterion(tGold);
    EXPECT_EQ(tGold, tAlgorithm.getStoppingCriterion());

    // TEST GRADIENT TOLERANCE
    double tScalarGold = 1e-4;
    EXPECT_EQ(tScalarGold, tAlgorithm.getGradientTolerance());
    tScalarGold = 1e-6;
    tAlgorithm.setGradientTolerance(tScalarGold);
    EXPECT_EQ(tScalarGold, tAlgorithm.getGradientTolerance());

    // TEST TRIAL STEP TOLERANCE
    tScalarGold = 1e-4;
    EXPECT_EQ(tScalarGold, tAlgorithm.getStationarityTolerance());
    tScalarGold = 1e-3;
    tAlgorithm.setStationarityTolerance(tScalarGold);
    EXPECT_EQ(tScalarGold, tAlgorithm.getStationarityTolerance());

    // TEST OBJECTIVE TOLERANCE
    tScalarGold = 1e-8;
    EXPECT_EQ(tScalarGold, tAlgorithm.getObjectiveStagnationTolerance());
    tScalarGold = 1e-5;
    tAlgorithm.setObjectiveStagnationTolerance(tScalarGold);
    EXPECT_EQ(tScalarGold, tAlgorithm.getObjectiveStagnationTolerance());

    // TEST CONTROL STAGNATION TOLERANCE
    double tMyTolerance = 1e-4;
    tScalarGold = 1e-8;
    EXPECT_NEAR(tScalarGold, tAlgorithm.getControlStagnationTolerance(), tMyTolerance);
    tScalarGold = 1e-2;
    tAlgorithm.setControlStagnationTolerance(tScalarGold);
    EXPECT_EQ(tScalarGold, tAlgorithm.getControlStagnationTolerance());

    // TEST ACTUAL REDUCTION TOLERANCE
    tScalarGold = 1e-8;
    EXPECT_EQ(tScalarGold, tAlgorithm.getActualReductionTolerance());
    tScalarGold = 1e-9;
    tAlgorithm.setActualReductionTolerance(tScalarGold);
    EXPECT_EQ(tScalarGold, tAlgorithm.getActualReductionTolerance());

    // TEST UPDATE CONTROL
    const size_t tNumVectors = 1;
    Plato::KelleySachsStepMng<double> tStepMng(*tDataFactory);
    Plato::StandardMultiVector<double> tMidControls(tNumVectors, tNumControls);
    tMidControls(0,0) = 0.6875;
    tMidControls(0,1) = 1.3125;
    tStepMng.setMidPointControls(tMidControls);
    Plato::StandardMultiVector<double> tMidGradient(tNumVectors, tNumControls);
    tMidGradient(0,0) = 1.0185546875;
    tMidGradient(0,1) = 0.3876953125;
    tScalarValue = 1.757354736328125;
    tStepMng.setMidPointObjectiveFunctionValue(tScalarValue);
    tScalarValue = -3.117645263671875;
    tStepMng.setActualReduction(tScalarValue);
    EXPECT_TRUE(tAlgorithm.updateControl(tMidGradient, tStepMng, *tDataMng, *tStageMng));

    Plato::StandardMultiVector<double> tGoldVector(tNumVectors, tNumControls);
    tGoldVector(0,0) = -0.3310546875;
    tGoldVector(0,1) = 0.9248046875;
    const Plato::MultiVector<double> & tCurrentControl = tDataMng->getCurrentControl();
    PlatoTest::checkMultiVectorData(tCurrentControl, tGoldVector);
    const double tTolerance = 1e-6;
    tScalarGold = -3.117645263671875;
    EXPECT_NEAR(tScalarGold, tStepMng.getActualReduction(), tTolerance);
    tScalarGold = 4.0548311933707737;
    EXPECT_NEAR(tScalarGold, tDataMng->getCurrentObjectiveFunctionValue(), tTolerance);
}

TEST(PlatoTest, KelleySachsAugmentedLagrangianCircleRadius)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory =
            std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumControls = 2;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumControls);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<double>> tDataMng =
            std::make_shared<Plato::TrustRegionAlgorithmDataMng<double>>(tDataFactory);
    double tScalarValue = 0.5;
    tDataMng->setInitialGuess(tScalarValue);
    tScalarValue = 0;
    tDataMng->setControlLowerBounds(tScalarValue);
    tScalarValue = 10;
    tDataMng->setControlUpperBounds(tScalarValue);

    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT CRITERIA *********
    std::shared_ptr<Plato::Circle<double>> tCircle = std::make_shared<Plato::Circle<double>>();
    std::shared_ptr<Plato::Radius<double>> tRadius = std::make_shared<Plato::Radius<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tRadius);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<double>> tStageMng =
            std::make_shared<Plato::AugmentedLagrangianStageMng<double>>(tDataFactory, tCircle, tConstraintList);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM *********
    Plato::KelleySachsAugmentedLagrangian<double> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    tAlgorithm.setMaxNumTrustRegionSubProblemIterations(10);
    tAlgorithm.setMaxTrustRegionRadius(1e1);
    tAlgorithm.solve();

    // TEST NUMBER OF ITERATIONS AND STOPPING CRITERION
    size_t tIntegerGold = 38;
    EXPECT_EQ(tIntegerGold, tAlgorithm.getNumIterationsDone());
    Plato::algorithm::stop_t tGold = Plato::algorithm::stop_t::SMALL_PENALTY_PARAMETER;
    EXPECT_EQ(tGold, tAlgorithm.getStoppingCriterion());

    // TEST OBJECTIVE FUNCTION VALUE
    const double tTolerance = 1e-6;
    double tScalarGold = 2.6779179289265822;
    EXPECT_NEAR(tScalarGold, tDataMng->getCurrentObjectiveFunctionValue(), tTolerance);

    // TEST CURRENT CONSTRAINT VALUE
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tConstraintValues(tNumVectors, tNumDuals);
    tStageMng->getCurrentConstraintValues(tConstraintValues);
    Plato::StandardMultiVector<double> tGoldConstraintValues(tNumVectors, tNumDuals);
    tGoldConstraintValues(0,0) = 1.746176845440761e-04;
    PlatoTest::checkMultiVectorData(tConstraintValues, tGoldConstraintValues);

    // TEST LAGRANGE MULTIPLIERS
    Plato::StandardMultiVector<double> tLagrangeMulipliers(tNumVectors, tNumDuals);
    tStageMng->getLagrangeMultipliers(tLagrangeMulipliers);
    Plato::StandardMultiVector<double> tGoldtLagrangeMulipliers(tNumVectors, tNumDuals);
    tGoldtLagrangeMulipliers(0,0) = 1;
    PlatoTest::checkMultiVectorData(tLagrangeMulipliers, tGoldtLagrangeMulipliers);

    // TEST CONTROL SOLUTION
    Plato::StandardMultiVector<double> tGoldVector(tNumVectors, tNumControls);
    tGoldVector(0,0) = 0.312106477189628;
    tGoldVector(0,1) = 0.950139023817475;
    const Plato::MultiVector<double> & tCurrentControl = tDataMng->getCurrentControl();
    PlatoTest::checkMultiVectorData(tCurrentControl, tGoldVector);

    // TEST CURRENT AUGMENTED LAGRANGIAN GRADIENT
    tGoldVector(0,0) = 0.17551763569502099;
    tGoldVector(0,1) = 0.52314282334931494;
    const Plato::MultiVector<double> & tCurrentGradient = tDataMng->getCurrentGradient();
    PlatoTest::checkMultiVectorData(tCurrentGradient, tGoldVector);
}

TEST(PlatoTest, KelleySachsAugmentedLagrangianRosenbrockRadius)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory =
            std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumControls = 2;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumControls);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<double>> tDataMng =
            std::make_shared<Plato::TrustRegionAlgorithmDataMng<double>>(tDataFactory);
    tDataMng->setInitialGuess(.1);
    tDataMng->setControlLowerBounds(-1.5);
    tDataMng->setControlUpperBounds(1.5);

    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT CRITERIA *********
    std::shared_ptr<Plato::Rosenbrock<double>> tRosenbrock = std::make_shared<Plato::Rosenbrock<double>>();
    std::shared_ptr<Plato::Radius<double>> tRadius = std::make_shared<Plato::Radius<double>>();
    tRadius->setLimit(2.);
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tRadius);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<double>> tStageMng =
            std::make_shared<Plato::AugmentedLagrangianStageMng<double>>(tDataFactory, tRosenbrock, tConstraintList);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM *********
    Plato::KelleySachsAugmentedLagrangian<double> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    tAlgorithm.setMaxNumTrustRegionSubProblemIterations(20);
    tAlgorithm.setMaxTrustRegionRadius(1e1);
    tAlgorithm.solve();

    // TEST NUMBER OF ITERATIONS AND STOPPING CRITERION
    size_t tIntegerGold = 38;
    EXPECT_EQ(tIntegerGold, tAlgorithm.getNumIterationsDone());
    Plato::algorithm::stop_t tGold = Plato::algorithm::stop_t::SMALL_PENALTY_PARAMETER;
    EXPECT_EQ(tGold, tAlgorithm.getStoppingCriterion());

    // TEST OBJECTIVE FUNCTION VALUE
    double tScalarGold = 0.;
    double tTolerance = 1e-4;
    EXPECT_NEAR(tScalarGold, tDataMng->getCurrentObjectiveFunctionValue(), tTolerance);

    // TEST CURRENT CONSTRAINT VALUE
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tConstraintValues(tNumVectors, tNumDuals);
    tStageMng->getCurrentConstraintValues(tConstraintValues);
    Plato::StandardMultiVector<double> tGoldConstraintValues(tNumVectors, tNumDuals);
    tTolerance = 1e-3;
    tGoldConstraintValues(0,0) = 0;
    PlatoTest::checkMultiVectorData(tConstraintValues, tGoldConstraintValues, tTolerance);

    // TEST CONTROL SOLUTION
    Plato::StandardMultiVector<double> tGoldVector(tNumVectors, tNumControls);
    tTolerance = 1e-3;
    tGoldVector(0,0) = 1.;
    tGoldVector(0,1) = 1.;
    const Plato::MultiVector<double> & tCurrentControl = tDataMng->getCurrentControl();
    PlatoTest::checkMultiVectorData(tCurrentControl, tGoldVector, tTolerance);
}

TEST(PlatoTest, KelleySachsAugmentedLagrangianGoldsteinPriceShiftedEllipse)
{
    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory =
            std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumControls = 2;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumControls);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<double>> tDataMng =
            std::make_shared<Plato::TrustRegionAlgorithmDataMng<double>>(tDataFactory);
    tDataMng->setInitialGuess(-.4);
    tDataMng->setControlLowerBounds(-1.);
    tDataMng->setControlUpperBounds(2.);

    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT CRITERIA *********
    std::shared_ptr<Plato::GoldsteinPrice<double>> tGoldsteinPrice = std::make_shared<Plato::GoldsteinPrice<double>>();
    std::shared_ptr<Plato::ShiftedEllipse<double>> tShiftedEllipse = std::make_shared<Plato::ShiftedEllipse<double>>();
    tShiftedEllipse->specify(0., 1., .5, 1.5);
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tShiftedEllipse);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<double>> tStageMng =
            std::make_shared<Plato::AugmentedLagrangianStageMng<double>>(tDataFactory, tGoldsteinPrice, tConstraintList);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM *********
    Plato::KelleySachsAugmentedLagrangian<double> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    tAlgorithm.setMaxTrustRegionRadius(1e1);
    tAlgorithm.solve();

    // TEST NUMBER OF ITERATIONS AND STOPPING CRITERION
    size_t tIntegerGold = 6;
    EXPECT_EQ(tIntegerGold, tAlgorithm.getNumIterationsDone());
    Plato::algorithm::stop_t tGold = Plato::algorithm::stop_t::OBJECTIVE_STAGNATION;
    EXPECT_EQ(tGold, tAlgorithm.getStoppingCriterion());

    // TEST OBJECTIVE FUNCTION VALUE
    const double tTolerance = 1e-2;
    EXPECT_NEAR(3, tDataMng->getCurrentObjectiveFunctionValue(), tTolerance);

    // TEST CONTROL SOLUTION
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tGoldVector(tNumVectors, tNumControls);
    tGoldVector(0,0) = 0.;
    tGoldVector(0,1) = -1.;
    const Plato::MultiVector<double> & tCurrentControl = tDataMng->getCurrentControl();
    PlatoTest::checkMultiVectorData(tCurrentControl, tGoldVector, tTolerance);
}

TEST(PlatoTest, KelleySachsAugmentedLagrangianHimmelblauShiftedEllipseTestOne)
{
    // this test focuses more on minimizing the objective and less on having a tight tolerance on the constraint

    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory =
            std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumControls = 2;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumControls);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<double>> tDataMng =
            std::make_shared<Plato::TrustRegionAlgorithmDataMng<double>>(tDataFactory);
    tDataMng->setInitialGuess(-2.);
    tDataMng->setControlLowerBounds(-5.);
    tDataMng->setControlUpperBounds(1.);

    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT CRITERIA *********
    std::shared_ptr<Plato::Himmelblau<double>> tHimmelblau = std::make_shared<Plato::Himmelblau<double>>();
    std::shared_ptr<Plato::ShiftedEllipse<double>> tShiftedEllipse = std::make_shared<Plato::ShiftedEllipse<double>>();
    tShiftedEllipse->specify(-2., 2., -3., 3.);
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tShiftedEllipse);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<double>> tStageMng =
            std::make_shared<Plato::AugmentedLagrangianStageMng<double>>(tDataFactory, tHimmelblau, tConstraintList);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM *********
    Plato::KelleySachsAugmentedLagrangian<double> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    tAlgorithm.setPenaltyParameterScaleFactor(0.5);
    tAlgorithm.setMaxTrustRegionRadius(1e1);
    tAlgorithm.solve();

    // TEST NUMBER OF ITERATIONS AND STOPPING CRITERION
    size_t tIntegerGold = 23;
    EXPECT_EQ(tIntegerGold, tAlgorithm.getNumIterationsDone());
    Plato::algorithm::stop_t tGold = Plato::algorithm::stop_t::STATIONARITY_MEASURE;
    EXPECT_EQ(tGold, tAlgorithm.getStoppingCriterion());

    // TEST OBJECTIVE FUNCTION VALUE
    const double tTolerance = 1e-2;
    EXPECT_NEAR(-0.2035, tDataMng->getCurrentObjectiveFunctionValue(), tTolerance);

    // TEST CONTROL SOLUTION
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tGoldVector(tNumVectors, tNumControls);
    tGoldVector(0, 0) = -3.779310;
    tGoldVector(0, 1) = -3.283186;
    const Plato::MultiVector<double> & tCurrentControl = tDataMng->getCurrentControl();
    PlatoTest::checkMultiVectorData(tCurrentControl, tGoldVector, tTolerance);
}

TEST(PlatoTest, KelleySachsAugmentedLagrangianHimmelblauShiftedEllipseTestTwo)
{
    // this test focuses more on minimizing the objective and having a tight tolerance on the constraint

    // ********* ALLOCATE DATA FACTORY *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory =
            std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumControls = 2;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumControls);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<double>> tDataMng =
            std::make_shared<Plato::TrustRegionAlgorithmDataMng<double>>(tDataFactory);
    tDataMng->setInitialGuess(-2.);
    tDataMng->setControlLowerBounds(-5.);
    tDataMng->setControlUpperBounds(1.);

    // ********* ALLOCATE OBJECTIVE AND CONSTRAINT CRITERIA *********
    std::shared_ptr<Plato::Himmelblau<double>> tHimmelblau = std::make_shared<Plato::Himmelblau<double>>();
    std::shared_ptr<Plato::ShiftedEllipse<double>> tShiftedEllipse = std::make_shared<Plato::ShiftedEllipse<double>>();
    tShiftedEllipse->specify(-2., 2., -3., 3.);
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tShiftedEllipse);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<double>> tStageMng =
            std::make_shared<Plato::AugmentedLagrangianStageMng<double>>(tDataFactory, tHimmelblau, tConstraintList);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM *********
    Plato::KelleySachsAugmentedLagrangian<double> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    tAlgorithm.setMaxTrustRegionRadius(1e1);
    tAlgorithm.solve();

    // TEST NUMBER OF ITERATIONS AND STOPPING CRITERION
    size_t tIntegerGold = 38;
    EXPECT_EQ(tIntegerGold, tAlgorithm.getNumIterationsDone());
    Plato::algorithm::stop_t tGold = Plato::algorithm::stop_t::SMALL_PENALTY_PARAMETER;
    EXPECT_EQ(tGold, tAlgorithm.getStoppingCriterion());

    // TEST OBJECTIVE FUNCTION VALUE
    const double tTolerance = 1e-4;
    EXPECT_NEAR(2.3995, tDataMng->getCurrentObjectiveFunctionValue(), tTolerance);

    // TEST CONTROL SOLUTION
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tGoldVector(tNumVectors, tNumControls);
    tGoldVector(0, 0) = -3.9798;
    tGoldVector(0, 1) = -3.3886;
    const Plato::MultiVector<double> & tCurrentControl = tDataMng->getCurrentControl();
    PlatoTest::checkMultiVectorData(tCurrentControl, tGoldVector, tTolerance);
}

} // namespace PlatoTest
