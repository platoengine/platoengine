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
 * Plato_ConservativeConvexSeparableApproximationTest.cpp
 *
 *  Created on: Jun 14, 2017
 */

#include "gtest/gtest.h"

#include "Plato_UnitTestUtils.hpp"

#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_MultiVectorList.hpp"
#include "Plato_CcsaTestObjective.hpp"
#include "Plato_CcsaTestInequality.hpp"
#include "Plato_DualProblemStageMng.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_PrimalProblemStageMng.hpp"
#include "Plato_MethodMovingAsymptotes.hpp"
#include "Plato_StandardVectorReductionOperations.hpp"
#include "Plato_ConservativeConvexSeparableAppxDataMng.hpp"
#include "Plato_ConservativeConvexSeparableAppxAlgorithm.hpp"
#include "Plato_GloballyConvergentMethodMovingAsymptotes.hpp"

namespace PlatoTest
{

TEST(PlatoTest, CcsaTestObjective)
{
    // ********* Allocate Criterion *********
    Plato::CcsaTestObjective<double> tCriterion;

    double tScalarValue = 1;
    const size_t tNumVectors = 1;
    const size_t tNumControls = 5;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tScalarValue);

    tScalarValue = 0.312;
    const double tTolerance = 1e-6;
    EXPECT_NEAR(tScalarValue, tCriterion.value(tControl), tTolerance);

    Plato::StandardMultiVector<double> tGradient(tNumVectors, tNumControls);
    tCriterion.gradient(tControl, tGradient);
    tScalarValue = 0.0624;
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls, tScalarValue);
    PlatoTest::checkMultiVectorData(tGradient, tGold);
}

TEST(PlatoTest, CcsaTestInequality)
{
    // ********* Allocate Criterion *********
    Plato::CcsaTestInequality<double> tCriterion;

    double tScalarValue = 1;
    const size_t tNumVectors = 1;
    const size_t tNumControls = 5;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tScalarValue);

    tScalarValue = 124;
    const double tTolerance = 1e-6;
    EXPECT_NEAR(tScalarValue, tCriterion.value(tControl), tTolerance);

    Plato::StandardMultiVector<double> tGradient(tNumVectors, tNumControls);
    tCriterion.gradient(tControl, tGradient);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = -183;
    tGold(0,1) = -111;
    tGold(0,2) = -57;
    tGold(0,3) = -21;
    tGold(0,4) = -3;
    PlatoTest::checkMultiVectorData(tGradient, tGold);
}

TEST(PlatoTest, ConservativeConvexSeparableAppxDataMng)
{
    // ********* Allocate Data Factory *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumControls = 2;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumControls);

    Plato::ConservativeConvexSeparableAppxDataMng<double> tDataMng(tDataFactory);

    // ********* TEST INTEGER AND SCALAR PARAMETERS *********
    size_t tOrdinalValue = 1;
    EXPECT_EQ(tOrdinalValue, tDataMng.getNumDualVectors());
    EXPECT_EQ(tOrdinalValue, tDataMng.getNumConstraints());
    EXPECT_EQ(tOrdinalValue, tDataMng.getNumControlVectors());

    double tScalarValue = 0.5;
    const double tTolerance = 1e-6;
    EXPECT_NEAR(tScalarValue, tDataMng.getDualProblemBoundsScaleFactor(), tTolerance);
    tDataMng.setDualProblemBoundsScaleFactor(0.25);
    tScalarValue = 0.25;
    EXPECT_NEAR(tScalarValue, tDataMng.getDualProblemBoundsScaleFactor(), tTolerance);

    // ********* TEST CONSTRAINT GLOBALIZATION FACTORS *********
    tScalarValue = 1;
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tDualMultiVector(tNumVectors, tNumDuals, tScalarValue);
    PlatoTest::checkMultiVectorData(tDualMultiVector, tDataMng.getConstraintGlobalizationFactors());
    tScalarValue = 2;
    Plato::fill(tScalarValue, tDualMultiVector);
    tDataMng.setConstraintGlobalizationFactors(tDualMultiVector);
    tScalarValue = 0.5;
    Plato::StandardVector<double> tDualVector(tNumDuals, tScalarValue);
    const size_t tVectorIndex = 0;
    tDataMng.setConstraintGlobalizationFactors(tVectorIndex, tDualVector);
    PlatoTest::checkVectorData(tDualVector, tDataMng.getConstraintGlobalizationFactors(tVectorIndex));

    // ********* TEST OBJECTIVE FUNCTION *********
    tScalarValue = std::numeric_limits<double>::max();
    EXPECT_NEAR(tScalarValue, tDataMng.getCurrentObjectiveFunctionValue(), tTolerance);
    tScalarValue = 0.2;
    tDataMng.setCurrentObjectiveFunctionValue(tScalarValue);
    EXPECT_NEAR(tScalarValue, tDataMng.getCurrentObjectiveFunctionValue(), tTolerance);

    tScalarValue = std::numeric_limits<double>::max();
    EXPECT_NEAR(tScalarValue, tDataMng.getPreviousObjectiveFunctionValue(), tTolerance);
    tScalarValue = 0.25;
    tDataMng.setPreviousObjectiveFunctionValue(tScalarValue);
    EXPECT_NEAR(tScalarValue, tDataMng.getPreviousObjectiveFunctionValue(), tTolerance);

    // ********* TEST INITIAL GUESS *********
    EXPECT_FALSE(tDataMng.isInitialGuessSet());
    tScalarValue = 0;
    Plato::StandardMultiVector<double> tControlMultiVector(tNumVectors, tNumControls, tScalarValue);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getCurrentControl());

    tScalarValue = 2;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setInitialGuess(tControlMultiVector);
    EXPECT_TRUE(tDataMng.isInitialGuessSet());
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getCurrentControl());

    tScalarValue = 5;
    tDataMng.setInitialGuess(tScalarValue);
    Plato::fill(tScalarValue, tControlMultiVector);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getCurrentControl());

    tScalarValue = 3;
    Plato::StandardVector<double> tControlVector(tNumControls, tScalarValue);
    tDataMng.setInitialGuess(tVectorIndex, tControlVector);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getCurrentControl(tVectorIndex));

    tScalarValue = 33;
    tDataMng.setInitialGuess(tVectorIndex, tScalarValue);
    tControlVector.fill(tScalarValue);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getCurrentControl(tVectorIndex));

    // ********* TEST SET DUAL FUNCTIONS *********
    tScalarValue = 11;
    Plato::fill(tScalarValue, tDualMultiVector);
    tDataMng.setDual(tDualMultiVector);
    PlatoTest::checkMultiVectorData(tDualMultiVector, tDataMng.getDual());

    tScalarValue = 21;
    tDualVector.fill(tScalarValue);
    tDataMng.setDual(tVectorIndex, tDualVector);
    PlatoTest::checkVectorData(tDualVector, tDataMng.getDual(tVectorIndex));

    // ********* TEST TRIAL STEP FUNCTIONS *********
    tScalarValue = 12;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setTrialStep(tControlMultiVector);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getTrialStep());

    tScalarValue = 22;
    tControlVector.fill(tScalarValue);
    tDataMng.setTrialStep(tVectorIndex, tControlVector);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getTrialStep(tVectorIndex));

    // ********* TEST ACTIVE SET FUNCTIONS *********
    tScalarValue = 10;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setActiveSet(tControlMultiVector);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getActiveSet());

    tScalarValue = 20;
    tControlVector.fill(tScalarValue);
    tDataMng.setActiveSet(tVectorIndex, tControlVector);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getActiveSet(tVectorIndex));

    // ********* TEST INACTIVE SET FUNCTIONS *********
    tScalarValue = 11;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setInactiveSet(tControlMultiVector);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getInactiveSet());

    tScalarValue = 21;
    tControlVector.fill(tScalarValue);
    tDataMng.setInactiveSet(tVectorIndex, tControlVector);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getInactiveSet(tVectorIndex));

    // ********* TEST CURRENT CONTROL FUNCTIONS *********
    tScalarValue = 1;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setCurrentControl(tControlMultiVector);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getCurrentControl());

    tScalarValue = 2;
    tControlVector.fill(tScalarValue);
    tDataMng.setCurrentControl(tVectorIndex, tControlVector);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getCurrentControl(tVectorIndex));

    // ********* TEST PREVIOUS CONTROL FUNCTIONS *********
    tScalarValue = 2;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setPreviousControl(tControlMultiVector);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getPreviousControl());

    tScalarValue = 3;
    tControlVector.fill(tScalarValue);
    tDataMng.setPreviousControl(tVectorIndex, tControlVector);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getPreviousControl(tVectorIndex));

    // ********* TEST CURRENT GRADIENT FUNCTIONS *********
    tScalarValue = 3;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setCurrentObjectiveGradient(tControlMultiVector);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getCurrentObjectiveGradient());

    tScalarValue = 4;
    tControlVector.fill(tScalarValue);
    tDataMng.setCurrentObjectiveGradient(tVectorIndex, tControlVector);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getCurrentObjectiveGradient(tVectorIndex));

    // ********* TEST CURRENT SIGMA FUNCTIONS *********
    tScalarValue = 5;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setCurrentSigma(tControlMultiVector);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getCurrentSigma());

    tScalarValue = 6;
    tControlVector.fill(tScalarValue);
    tDataMng.setCurrentSigma(tVectorIndex, tControlVector);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getCurrentSigma(tVectorIndex));

    // ********* TEST CONTROL LOWER BOUNDS FUNCTIONS *********
    tScalarValue = 6;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setControlLowerBounds(tControlMultiVector);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getControlLowerBounds());

    tScalarValue = 9;
    tDataMng.setControlLowerBounds(tScalarValue);
    Plato::fill(tScalarValue, tControlMultiVector);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getControlLowerBounds());

    tScalarValue = 7;
    tControlVector.fill(tScalarValue);
    tDataMng.setControlLowerBounds(tVectorIndex, tControlVector);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getControlLowerBounds(tVectorIndex));

    tScalarValue = 8;
    tControlVector.fill(tScalarValue);
    tDataMng.setControlLowerBounds(tVectorIndex, tScalarValue);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getControlLowerBounds(tVectorIndex));

    // ********* TEST CONTROL UPPER BOUNDS FUNCTIONS *********
    tScalarValue = 61;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setControlUpperBounds(tControlMultiVector);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getControlUpperBounds());

    tScalarValue = 91;
    tDataMng.setControlUpperBounds(tScalarValue);
    Plato::fill(tScalarValue, tControlMultiVector);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getControlUpperBounds());

    tScalarValue = 71;
    tControlVector.fill(tScalarValue);
    tDataMng.setControlUpperBounds(tVectorIndex, tControlVector);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getControlUpperBounds(tVectorIndex));

    tScalarValue = 81;
    tControlVector.fill(tScalarValue);
    tDataMng.setControlUpperBounds(tVectorIndex, tScalarValue);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getControlUpperBounds(tVectorIndex));

    // ********* TEST CONSTRAINT VALUES FUNCTIONS *********
    tScalarValue = 61;
    Plato::fill(tScalarValue, tDualMultiVector);
    tDataMng.setCurrentConstraintValues(tDualMultiVector);
    PlatoTest::checkMultiVectorData(tDualMultiVector, tDataMng.getCurrentConstraintValues());

    tScalarValue = 91;
    tDualVector.fill(tScalarValue);
    const size_t tConstraintIndex = 0;
    tDataMng.setCurrentConstraintValues(tConstraintIndex, tDualVector);
    PlatoTest::checkVectorData(tDualVector, tDataMng.getCurrentConstraintValues(tConstraintIndex));

    // ********* TEST CONSTRAINT GRADIENT FUNCTIONS *********
    tScalarValue = 1;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setCurrentConstraintGradients(tConstraintIndex, tControlMultiVector);
    PlatoTest::checkMultiVectorData(tControlMultiVector, tDataMng.getCurrentConstraintGradients(tConstraintIndex));
    const Plato::MultiVectorList<double> & tCurrentConstraintGradientList = tDataMng.getCurrentConstraintGradients();
    PlatoTest::checkMultiVectorData(tControlMultiVector, tCurrentConstraintGradientList[tConstraintIndex]);

    tScalarValue = 9;
    tControlVector.fill(tScalarValue);
    tDataMng.setCurrentConstraintGradients(tConstraintIndex, tVectorIndex, tControlVector);
    PlatoTest::checkVectorData(tControlVector, tDataMng.getCurrentConstraintGradients(tConstraintIndex, tVectorIndex));

    // ********* TEST COMPUTE STAGNATION MEASURE *********
    tScalarValue = 1;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setCurrentControl(tControlMultiVector);
    tScalarValue = 4;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setPreviousControl(tControlMultiVector);
    tDataMng.computeStagnationMeasure();
    tScalarValue = 3;
    EXPECT_NEAR(tScalarValue, tDataMng.getStagnationMeasure(), tTolerance);

    // ********* TEST COMPUTE INACTIVE VECTOR NORM *********
    tScalarValue = 1;
    tControlVector.fill(tScalarValue);
    tControlVector[0] = 0;
    tDataMng.setInactiveSet(tVectorIndex, tControlVector);
    tScalarValue = 4;
    Plato::fill(tScalarValue, tControlMultiVector);
    EXPECT_NEAR(tScalarValue, tDataMng.computeInactiveVectorNorm(tControlMultiVector), tTolerance);

    tScalarValue = 8;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.setCurrentObjectiveGradient(tControlMultiVector);
    tDataMng.computeNormInactiveGradient();
    tScalarValue = 4;
    EXPECT_NEAR(tScalarValue, tDataMng.getNormInactiveGradient(), tTolerance);

    // ********* TEST OBJECTIVE STAGNATION FUNCTION *********
    tScalarValue = 0.5;
    tDataMng.setCurrentObjectiveFunctionValue(tScalarValue);
    tScalarValue = 1.25;
    tDataMng.setPreviousObjectiveFunctionValue(tScalarValue);
    tScalarValue = 0.75;
    tDataMng.computeObjectiveStagnationMeasure();
    EXPECT_NEAR(tScalarValue, tDataMng.getObjectiveStagnationMeasure(), tTolerance);

    // ********* TEST FEASIBILITY MEASURE *********
    tScalarValue = 0.5;
    tDualVector.fill(tScalarValue);
    tDataMng.setCurrentConstraintValues(tVectorIndex, tDualVector);
    tDataMng.computeFeasibilityMeasure();
    EXPECT_NEAR(tScalarValue, tDataMng.getFeasibilityMeasure(), tTolerance);
}

TEST(PlatoTest, PrimalProblemStageMng)
{
    // ********* Allocate Data Factory *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumControls = 5;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumControls);

    // ********* Allocate Criteria *********
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    std::shared_ptr<Plato::CcsaTestInequality<double>> tConstraint = std::make_shared<Plato::CcsaTestInequality<double>>();
    tConstraintList->add(tConstraint);
    std::shared_ptr<Plato::CcsaTestObjective<double>> tObjective = std::make_shared<Plato::CcsaTestObjective<double>>();

    // ********* Allocate Primal Stage Manager *********
    Plato::PrimalProblemStageMng<double> tStageMng(tDataFactory, tObjective, tConstraintList);

    // ********* TEST OBJECTIVE EVALUATION *********
    size_t tOrdinalValue = 0;
    EXPECT_EQ(tOrdinalValue, tStageMng.getNumObjectiveFunctionEvaluations());
    double tScalarValue = 1;
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls, tScalarValue);
    tScalarValue = 0.312;
    const double tTolerance = 1e-6;
    EXPECT_NEAR(tScalarValue, tStageMng.evaluateObjective(tControl), tTolerance);
    tOrdinalValue = 1;
    EXPECT_EQ(tOrdinalValue, tStageMng.getNumObjectiveFunctionEvaluations());

    // ********* TEST CONSTRAINT EVALUATION *********
    tOrdinalValue = 0;
    const size_t tConstraintIndex = 0;
    EXPECT_EQ(tOrdinalValue, tStageMng.getNumConstraintEvaluations(tConstraintIndex));
    Plato::StandardMultiVector<double> tConstraints(tNumVectors, tNumDuals);
    tStageMng.evaluateConstraints(tControl, tConstraints);
    tScalarValue = 124;
    Plato::StandardMultiVector<double> tDualMultiVector(tNumVectors, tNumDuals, tScalarValue);
    PlatoTest::checkMultiVectorData(tConstraints, tDualMultiVector);
    tOrdinalValue = 1;
    EXPECT_EQ(tOrdinalValue, tStageMng.getNumConstraintEvaluations(tConstraintIndex));

    // ********* TEST OBJECTIVE GRADIENT *********
    tOrdinalValue = 0;
    EXPECT_EQ(tOrdinalValue, tStageMng.getNumObjectiveGradientEvaluations());
    Plato::StandardMultiVector<double> tObjectiveGradient(tNumVectors, tNumControls);
    tStageMng.computeGradient(tControl, tObjectiveGradient);
    tScalarValue = 0.0624;
    Plato::StandardMultiVector<double> tControlMultiVector(tNumVectors, tNumControls, tScalarValue);
    PlatoTest::checkMultiVectorData(tObjectiveGradient, tControlMultiVector);
    tOrdinalValue = 1;
    EXPECT_EQ(tOrdinalValue, tStageMng.getNumObjectiveGradientEvaluations());

    // ********* TEST CONSTRAINT GRADIENT *********
    tOrdinalValue = 0;
    EXPECT_EQ(tOrdinalValue, tStageMng.getNumConstraintGradientEvaluations(tConstraintIndex));
    Plato::MultiVectorList<double> tConstraintGradients;
    tConstraintGradients.add(tControlMultiVector);
    tStageMng.computeConstraintGradients(tControl, tConstraintGradients);
    tControlMultiVector(0, 0) = -183;
    tControlMultiVector(0, 1) = -111;
    tControlMultiVector(0, 2) = -57;
    tControlMultiVector(0, 3) = -21;
    tControlMultiVector(0, 4) = -3;
    PlatoTest::checkMultiVectorData(tConstraintGradients[tConstraintIndex], tControlMultiVector);
    tOrdinalValue = 1;
    EXPECT_EQ(tOrdinalValue, tStageMng.getNumConstraintGradientEvaluations(tConstraintIndex));
}

TEST(PlatoTest, DualProblemStageMng)
{
    // ********* Allocate Data Factory *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumControls = 5;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumControls);

    // ********* Allocate Data Manager and Test Data *********
    Plato::ConservativeConvexSeparableAppxDataMng<double> tDataMng(tDataFactory);
    double tScalarValue = 1e-2;
    tDataMng.setControlLowerBounds(tScalarValue);
    tScalarValue = 1;
    tDataMng.setControlUpperBounds(tScalarValue);
    tScalarValue = 0.1;
    tDataMng.setCurrentSigma(tScalarValue);
    tScalarValue = 1;
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tControlMultiVector(tNumVectors, tNumControls, tScalarValue);
    tDataMng.setCurrentControl(tControlMultiVector);
    tDataMng.setCurrentObjectiveGradient(tControlMultiVector);
    tControlMultiVector(0, 0) = 2;
    const size_t tConstraintIndex = 0;
    tDataMng.setCurrentConstraintGradients(tConstraintIndex, tControlMultiVector);
    tScalarValue = 0.1;
    tDataMng.setCurrentObjectiveFunctionValue(tScalarValue);
    tScalarValue = 0.2;
    Plato::StandardMultiVector<double> tDualMultiVector(tNumVectors, tNumDuals, tScalarValue);
    tDataMng.setCurrentConstraintValues(tDualMultiVector);

    // ********* Allocate Dual Problem Data Manager *********
    Plato::DualProblemStageMng<double> tDualStageMng(tDataFactory.operator*());

    // ********* TEST UPDATE FUNCTION *********
    tDualStageMng.update(tDataMng);
    tScalarValue = 0.9;
    Plato::fill(tScalarValue, tControlMultiVector);
    PlatoTest::checkMultiVectorData(tDualStageMng.getLowerAsymptotes(), tControlMultiVector);
    tScalarValue = 1.1;
    Plato::fill(tScalarValue, tControlMultiVector);
    PlatoTest::checkMultiVectorData(tDualStageMng.getUpperAsymptotes(), tControlMultiVector);
    tScalarValue = 0.95;
    Plato::fill(tScalarValue, tControlMultiVector);
    PlatoTest::checkMultiVectorData(tDualStageMng.getTrialControlLowerBounds(), tControlMultiVector);
    tScalarValue = 1.05;
    Plato::fill(tScalarValue, tControlMultiVector);
    PlatoTest::checkMultiVectorData(tDualStageMng.getTrialControlUpperBounds(), tControlMultiVector);

    // ********* TEST UPDATE OBJECTIVE COEFFICIENTS FUNCTION *********
    tScalarValue = 0.5;
    tDataMng.setDualObjectiveGlobalizationFactor(tScalarValue);
    tDualStageMng.updateObjectiveCoefficients(tDataMng);
    tScalarValue = 0.0225;
    Plato::fill(tScalarValue, tControlMultiVector);
    PlatoTest::checkMultiVectorData(tDualStageMng.getObjectiveCoefficientsP(), tControlMultiVector);
    tScalarValue = 0.0125;
    Plato::fill(tScalarValue, tControlMultiVector);
    PlatoTest::checkMultiVectorData(tDualStageMng.getObjectiveCoefficientsQ(), tControlMultiVector);
    tScalarValue = -1.65;
    const double tTolerance = 1e-6;
    EXPECT_NEAR(tScalarValue, tDualStageMng.getObjectiveCoefficientsR(), tTolerance);

    // ********* TEST UPDATE CONSTRAINT COEFFICIENTS FUNCTION *********
    tScalarValue = 0.5;
    Plato::fill(tScalarValue, tDualMultiVector);
    tDataMng.setConstraintGlobalizationFactors(tDualMultiVector);
    tDualStageMng.updateConstraintCoefficients(tDataMng);
    tScalarValue = 0.0225;
    Plato::fill(tScalarValue, tControlMultiVector);
    tControlMultiVector(0,0) = 0.0325;
    PlatoTest::checkMultiVectorData(tDualStageMng.getConstraintCoefficientsP(tConstraintIndex), tControlMultiVector);
    tScalarValue = 0.0125;
    Plato::fill(tScalarValue, tControlMultiVector);
    PlatoTest::checkMultiVectorData(tDualStageMng.getConstraintCoefficientsQ(tConstraintIndex), tControlMultiVector);
    tScalarValue = -1.65;
    EXPECT_NEAR(tScalarValue, tDualStageMng.getObjectiveCoefficientsR(), tTolerance);

    // ********* TEST EVALUATE OBJECTIVE FUNCTION *********
    tScalarValue = 1;
    Plato::fill(tScalarValue, tDualMultiVector);
    double tObjectiveValue = tDualStageMng.evaluateObjective(tDualMultiVector);
    tScalarValue = -0.21245071085465561;
    EXPECT_NEAR(tScalarValue, tObjectiveValue, tTolerance);

    // ********* TEST TRIAL CONTROL CALCULATION *********
    tScalarValue = 0;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDualStageMng.getTrialControl(tControlMultiVector);
    tScalarValue = 0.985410196624969;
    Plato::StandardMultiVector<double> tControlGold(tNumVectors, tNumControls, tScalarValue);
    tControlGold(0,0) = 0.980539949569856;
    PlatoTest::checkMultiVectorData(tControlMultiVector, tControlGold);

    // ********* TEST OBJECTIVE GRADIENT FUNCTION *********
    tScalarValue = 1;
    Plato::fill(tScalarValue, tDualMultiVector);
    Plato::StandardMultiVector<double> tDualGradient(tNumVectors, tNumDuals);
    tDualStageMng.computeGradient(tDualMultiVector, tDualGradient);
    tScalarValue = -0.14808035198890879;
    Plato::StandardMultiVector<double> tDualGold(tNumVectors, tNumDuals, tScalarValue);
    PlatoTest::checkMultiVectorData(tDualGradient, tDualGold);
}

TEST(PlatoTest, ComputeKarushKuhnTuckerConditionsInexactness)
{
    // ********* Allocate Data Factory *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumControls = 5;
    tDataFactory->allocateDual(tNumDuals);
    tDataFactory->allocateControl(tNumControls);

    // ********* Allocate Data Manager and Test Data *********
    Plato::ConservativeConvexSeparableAppxDataMng<double> tDataMng(tDataFactory);
    double tScalarValue = 1;
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tControlMultiVector(tNumVectors, tNumControls, tScalarValue);
    tDataMng.setCurrentObjectiveGradient(tControlMultiVector);
    const size_t tConstraintIndex = 0;
    tControlMultiVector(0,0) = 2;
    tDataMng.setCurrentConstraintGradients(tConstraintIndex, tControlMultiVector);
    tScalarValue = 0.1;
    tDataMng.setCurrentObjectiveFunctionValue(tScalarValue);
    tScalarValue = 1;
    Plato::StandardMultiVector<double> tDualMultiVector(tNumVectors, tNumDuals, tScalarValue);
    tDataMng.setCurrentConstraintValues(tDualMultiVector);

    // ********* TEST KARUSH-KUHN-TUCKER MEASURE *********
    tScalarValue = 0.1;
    Plato::fill(tScalarValue, tDualMultiVector);
    tScalarValue = 1;
    Plato::fill(tScalarValue, tControlMultiVector);
    tDataMng.computeKarushKuhnTuckerConditionsInexactness(tControlMultiVector, tDualMultiVector);
    double tGold = 1.02215458713;
    const double tTolerance = 1e-6;
    tScalarValue = tDataMng.getKarushKuhnTuckerConditionsInexactness();
    EXPECT_NEAR(tScalarValue, tGold, tTolerance);
}

} // PlatoTest
