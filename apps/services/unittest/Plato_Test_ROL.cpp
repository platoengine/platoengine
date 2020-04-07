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
 * Plato_Test_ROL.cpp
 *
 *  Created on: Nov 6, 2017
 */

#include <gtest/gtest.h>

#include "Teuchos_XMLParameterListHelpers.hpp"

#include "ROL_Bounds.hpp"
#include "ROL_OptimizationSolver.hpp"
#include "ROL_OptimizationProblem.hpp"

#include "Plato_Parser.hpp"
#include "Plato_Interface.hpp"
#include "Plato_OptimizerInterface.hpp"
#include "Plato_OptimizerEngineStageData.hpp"
#include "Plato_StructuralTopologyOptimization.hpp"

#include "Plato_ProxyVolumeROL.hpp"
#include "Plato_SerialVectorROL.hpp"
#include "Plato_ProxyComplianceROL.hpp"
#include "Plato_DistributedVectorROL.hpp"
#include "Plato_SerialEpetraVectorROL.hpp"
#include "Plato_StructuralTopologyOptimizationProxyGoldResults.hpp"

namespace PlatoTest
{
#ifndef BUILD_IN_SIERRA
TEST(PlatoTest, DistributedVectorROL)
{
    double tScalarValue = 1;
    int tNumElements = 10;
    std::vector<double> tDataOne(tNumElements, tScalarValue);
    Plato::DistributedVectorROL<double> tVectorOne(MPI_COMM_WORLD, tDataOne);

    // *********** TEST DIMENSION FUNCTION ***********
    int tCommSize = 0;
    MPI_Comm_size(MPI_COMM_WORLD, &tCommSize);
    int tOrdinalGold = tCommSize * tNumElements;
    EXPECT_EQ(tVectorOne.dimension(), tOrdinalGold);

    // *********** TEST SCALE FUNCTION ***********
    tScalarValue = 2;
    tVectorOne.scale(tScalarValue);
    std::vector<double> tDataTwo(tNumElements, tScalarValue);
    Plato::DistributedVectorROL<double> tVectorTwo(MPI_COMM_WORLD, tDataTwo);
    const double tTolerance = 1e-6;
    for(int tElemIndex = 0; tElemIndex < tNumElements; tElemIndex++)
    {
        EXPECT_NEAR(tVectorOne.vector()[tElemIndex], tVectorTwo.vector()[tElemIndex], tTolerance);
    }

    // *********** TEST PLUS FUNCTION ***********
    tVectorOne.plus(tVectorTwo);
    tScalarValue = 4;
    std::fill(tVectorTwo.vector().begin(), tVectorTwo.vector().end(), tScalarValue); // Set gold values
    for(int tElemIndex = 0; tElemIndex < tNumElements; tElemIndex++)
    {
        EXPECT_NEAR(tVectorOne.vector()[tElemIndex], tVectorTwo.vector()[tElemIndex], tTolerance);
    }

    // *********** TEST DOT FUNCTION ***********
    tScalarValue = 1;
    std::fill(tVectorOne.vector().begin(), tVectorOne.vector().end(), tScalarValue);
    std::fill(tVectorTwo.vector().begin(), tVectorTwo.vector().end(), tScalarValue);
    tScalarValue = tVectorOne.dot(tVectorTwo);
    double tScalarGold = static_cast<double>(tVectorOne.dimension());
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);

    // *********** TEST NORM FUNCTION ***********
    tScalarValue = tVectorOne.norm();
    tScalarGold = std::sqrt(static_cast<double>(tNumElements * tCommSize));
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);

    // *********** TEST CLONE FUNCTION ***********
    Teuchos::RCP<ROL::Vector<double>> tCopy = tVectorOne.clone();
    tOrdinalGold = tVectorOne.dimension();
    EXPECT_EQ(tCopy->dimension(), tOrdinalGold);

    // *********** TEST REDUCE SUM FUNCTION ***********
    ROL::Elementwise::ReductionSum<double> tReduceSum;
    tScalarValue = tVectorOne.reduce(tReduceSum);
    tScalarGold = static_cast<double>(tVectorOne.dimension());
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);

    // *********** TEST REDUCE MIN FUNCTION ***********
    ROL::Elementwise::ReductionMin<double> tReduceMin;
    size_t tIndex = 4;
    tVectorOne.vector()[tIndex] = 44;
    tScalarValue = tVectorOne.reduce(tReduceMin);
    tScalarGold = 1;
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);

    // *********** TEST REDUCE MAX FUNCTION ***********
    ROL::Elementwise::ReductionMax<double> tReduceMax;
    tScalarValue = tVectorOne.reduce(tReduceMax);
    tScalarGold = 44;
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);
}

TEST(PlatoTest, SerialVectorROL)
{
    double tScalarValue = 1;
    int tNumElements = 10;
    std::vector<double> tDataOne(tNumElements, tScalarValue);
    Plato::SerialVectorROL<double> tVectorOne(tDataOne);

    // *********** TEST DIMENSION FUNCTION ***********
    int tOrdinalGold = tNumElements;
    EXPECT_EQ(tVectorOne.dimension(), tOrdinalGold);

    // *********** TEST SCALE FUNCTION ***********
    tScalarValue = 2;
    tVectorOne.scale(tScalarValue);
    std::vector<double> tDataTwo(tNumElements, tScalarValue);
    Plato::SerialVectorROL<double> tVectorTwo(tDataTwo);
    const double tTolerance = 1e-6;
    for(int tElemIndex = 0; tElemIndex < tNumElements; tElemIndex++)
    {
        EXPECT_NEAR(tVectorOne.vector()[tElemIndex], tVectorTwo.vector()[tElemIndex], tTolerance);
    }

    // *********** TEST PLUS FUNCTION ***********
    tVectorOne.plus(tVectorTwo);
    tScalarValue = 4;
    std::fill(tVectorTwo.vector().data(), tVectorTwo.vector().data() + tNumElements, tScalarValue); // Set gold values
    for(int tElemIndex = 0; tElemIndex < tNumElements; tElemIndex++)
    {
        EXPECT_NEAR(tVectorOne.vector()[tElemIndex], tVectorTwo.vector()[tElemIndex], tTolerance);
    }

    // *********** TEST DOT FUNCTION ***********
    tScalarValue = 1;
    std::fill(tVectorOne.vector().data(), tVectorOne.vector().data() + tNumElements, tScalarValue);
    std::fill(tVectorTwo.vector().data(), tVectorTwo.vector().data() + tNumElements, tScalarValue);
    tScalarValue = tVectorOne.dot(tVectorTwo);
    double tScalarGold = static_cast<double>(tVectorOne.dimension());
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);

    // *********** TEST NORM FUNCTION ***********
    tScalarValue = tVectorOne.norm();
    tScalarGold = std::sqrt(static_cast<double>(tNumElements));
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);

    // *********** TEST CLONE FUNCTION ***********
    Teuchos::RCP<ROL::Vector<double>> tCopy = tVectorOne.clone();
    tOrdinalGold = tVectorOne.dimension();
    EXPECT_EQ(tCopy->dimension(), tOrdinalGold);

    // *********** TEST REDUCE SUM FUNCTION ***********
    ROL::Elementwise::ReductionSum<double> tReduceSum;
    tScalarValue = tVectorOne.reduce(tReduceSum);
    tScalarGold = static_cast<double>(tVectorOne.dimension());
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);

    // *********** TEST REDUCE MIN FUNCTION ***********
    ROL::Elementwise::ReductionMin<double> tReduceMin;
    size_t tIndex = 4;
    tVectorOne.vector()[tIndex] = 44;
    tScalarValue = tVectorOne.reduce(tReduceMin);
    tScalarGold = 1;
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);

    // *********** TEST REDUCE MAX FUNCTION ***********
    ROL::Elementwise::ReductionMax<double> tReduceMax;
    tScalarValue = tVectorOne.reduce(tReduceMax);
    tScalarGold = 44;
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);
}

TEST(PlatoTest, SerialEpetraVectorROL)
{
    double tScalarValue = 1;
    int tNumElements = 10;
    std::vector<double> tDataOne(tNumElements, tScalarValue);
    Plato::SerialEpetraVectorROL<double> tVectorOne(tDataOne);

    // *********** TEST DIMENSION FUNCTION ***********
    int tOrdinalGold = tNumElements;
    EXPECT_EQ(tVectorOne.dimension(), tOrdinalGold);

    // *********** TEST SCALE FUNCTION ***********
    tScalarValue = 2;
    tVectorOne.scale(tScalarValue);
    std::vector<double> tDataTwo(tNumElements, tScalarValue);
    Plato::SerialEpetraVectorROL<double> tVectorTwo(tDataTwo);
    const double tTolerance = 1e-6;
    for(int tElemIndex = 0; tElemIndex < tNumElements; tElemIndex++)
    {
        EXPECT_NEAR(tVectorOne.vector()[tElemIndex], tVectorTwo.vector()[tElemIndex], tTolerance);
    }

    // *********** TEST PLUS FUNCTION ***********
    tVectorOne.plus(tVectorTwo);
    tScalarValue = 4;
    std::fill(tVectorTwo.vector().A(), tVectorTwo.vector().A() + tNumElements, tScalarValue); // Set gold values
    for(int tElemIndex = 0; tElemIndex < tNumElements; tElemIndex++)
    {
        EXPECT_NEAR(tVectorOne.vector()[tElemIndex], tVectorTwo.vector()[tElemIndex], tTolerance);
    }

    // *********** TEST DOT FUNCTION ***********
    tScalarValue = 1;
    std::fill(tVectorOne.vector().A(), tVectorOne.vector().A() + tNumElements, tScalarValue);
    std::fill(tVectorTwo.vector().A(), tVectorTwo.vector().A() + tNumElements, tScalarValue);
    tScalarValue = tVectorOne.dot(tVectorTwo);
    double tScalarGold = static_cast<double>(tVectorOne.dimension());
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);

    // *********** TEST NORM FUNCTION ***********
    tScalarValue = tVectorOne.norm();
    tScalarGold = std::sqrt(static_cast<double>(tNumElements));
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);

    // *********** TEST CLONE FUNCTION ***********
    Teuchos::RCP<ROL::Vector<double>> tCopy = tVectorOne.clone();
    tOrdinalGold = tVectorOne.dimension();
    EXPECT_EQ(tCopy->dimension(), tOrdinalGold);

    // *********** TEST REDUCE SUM FUNCTION ***********
    ROL::Elementwise::ReductionSum<double> tReduceSum;
    tScalarValue = tVectorOne.reduce(tReduceSum);
    tScalarGold = static_cast<double>(tVectorOne.dimension());
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);

    // *********** TEST REDUCE MIN FUNCTION ***********
    ROL::Elementwise::ReductionMin<double> tReduceMin;
    size_t tIndex = 4;
    tVectorOne.vector()[tIndex] = 44;
    tScalarValue = tVectorOne.reduce(tReduceMin);
    tScalarGold = 1;
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);

    // *********** TEST REDUCE MAX FUNCTION ***********
    ROL::Elementwise::ReductionMax<double> tReduceMax;
    tScalarValue = tVectorOne.reduce(tReduceMax);
    tScalarGold = 44;
    EXPECT_NEAR(tScalarGold, tScalarValue, tTolerance);
}

TEST(PlatoTest, ProxyComplianceROL)
{
    // ************** ALLOCATE SIMPLE STRUCTURAL TOPOLOGY OPTIMIZATION SOLVER **************
    const double tPoissonRatio = 0.3;
    const double tElasticModulus = 1;
    const int tNumElementsXdirection = 30;
    const int tNumElementsYdirection = 10;
    std::shared_ptr<Plato::StructuralTopologyOptimization> tPDE =
            std::make_shared<Plato::StructuralTopologyOptimization>(tPoissonRatio, tElasticModulus, tNumElementsXdirection, tNumElementsYdirection);
    // ************** DISABLE CACHING FOR ROL: CACHING IS NOT CLEARLY UNDERSTOOD YET FOR ROL **************
    tPDE->disableCacheState();

    // ************** SET FORCE VECTOR **************
    const int tGlobalNumDofs = tPDE->getGlobalNumDofs();
    Epetra_SerialDenseVector tForce(tGlobalNumDofs);
    const int tDOFsIndex = 1;
    tForce[tDOFsIndex] = -1;
    tPDE->setForceVector(tForce);

    // ************** SET FIXED DEGREES OF FREEDOM (DOFs) VECTOR **************
    std::vector<double> tData = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
    Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tData.data(), tData.size());
    tPDE->setFixedDOFs(tFixedDOFs);

    // ************** ALLOCATE COMPLIANCE CRITERION **************
    Plato::ProxyComplianceROL<double> tObjective(tPDE);

    // ************** TEST COMPLIANCE CRITERION EVALUATION **************
    const double tBaseValue = 1;
    const int tNumControls = tPDE->getNumDesignVariables();
    Plato::SerialEpetraVectorROL<double> tControl(tNumControls, tBaseValue);
    double tDummyTolerance = 1e-8;
    double tObjectiveValue = tObjective.value(tControl, tDummyTolerance);

    const double tTolerance = 1e-6;
    double tObjectiveGold = 123.0693511744326;
    EXPECT_NEAR(tObjectiveGold, tObjectiveValue, tTolerance);

    // ************** TEST COMPLIANCE CRITERION GRADIENT **************
    Plato::SerialEpetraVectorROL<double> tGradient(tNumControls, tBaseValue);
    tObjective.gradient(tGradient, tControl, tDummyTolerance);

    std::vector<double> tGoldGradient = TopoProxy::getGoldFilteredGradient();
    for(size_t tIndex = 0; tIndex < tGoldGradient.size(); tIndex++)
    {
        EXPECT_NEAR(tGradient.vector()[tIndex], tGoldGradient[tIndex], tTolerance);
    }
}

TEST(PlatoTest, ProxyVolumeROL)
{
    // ************** ALLOCATE SIMPLE STRUCTURAL TOPOLOGY OPTIMIZATION SOLVER **************
    const double tPoissonRatio = 0.3;
    const double tElasticModulus = 1;
    const int tNumElementsXdirection = 30;
    const int tNumElementsYdirection = 10;
    std::shared_ptr<Plato::StructuralTopologyOptimization> tPDE =
            std::make_shared<Plato::StructuralTopologyOptimization>(tPoissonRatio, tElasticModulus, tNumElementsXdirection, tNumElementsYdirection);

    // ************** SET FORCE VECTOR **************
    const int tGlobalNumDofs = tPDE->getGlobalNumDofs();
    Epetra_SerialDenseVector tForce(tGlobalNumDofs);
    const int tDOFsIndex = 1;
    tForce[tDOFsIndex] = -1;
    tPDE->setForceVector(tForce);

    // ************** SET FIXED DEGREES OF FREEDOM (DOFs) VECTOR **************
    std::vector<double> tData = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
    Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tData.data(), tData.size());
    tPDE->setFixedDOFs(tFixedDOFs);

    // ************** ALLOCATE VOLUME CRITERION **************
    Plato::ProxyVolumeROL<double> tConstraint(tPDE);

    // ************** TEST VOLUME CRITERION EVALUATION **************
    double tBaseValue = 1;
    const int tNumControls = tPDE->getNumDesignVariables();
    Plato::SerialEpetraVectorROL<double> tControl(tNumControls, tBaseValue);
    const int tNumConstraints = 1;
    Plato::SerialEpetraVectorROL<double> tConstraintValues(tNumConstraints);

    double tDummyTolerance = 1e-8;
    tConstraint.value(tConstraintValues, tControl, tDummyTolerance);
    const double tTolerance = 1e-4;
    double tConstraintGold = 0.6;
    EXPECT_NEAR(tConstraintGold, tConstraintValues.vector()[0], tTolerance);

    // ************** TEST VOLUME CRITERION JACOBIAN TIMES DIRECTION **************
    tBaseValue = 1;
    Plato::SerialEpetraVectorROL<double> tDirection(tNumControls, tBaseValue);
    Plato::SerialEpetraVectorROL<double> tJacobianTimesDirection(tNumConstraints);
    tConstraint.applyJacobian(tJacobianTimesDirection, tDirection, tControl, tDummyTolerance);
    std::vector<double> tGoldOne(tNumConstraints, 1);
    for(int tIndex = 0; tIndex < tJacobianTimesDirection.dimension(); tIndex++)
    {
        EXPECT_NEAR(tJacobianTimesDirection.vector()[tIndex], tGoldOne[tIndex], tTolerance);
    }

    // ************** TEST VOLUME CRITERION ADJOINT JACOBIAN TIMES DUAL **************
    tBaseValue = 0;
    Plato::SerialEpetraVectorROL<double> tDualDirection(tNumConstraints, 1);
    Plato::SerialEpetraVectorROL<double> tAdjointJacobianTimesDirection(tNumControls);
    tConstraint.applyAdjointJacobian(tAdjointJacobianTimesDirection, tDualDirection, tControl, tDummyTolerance);
    std::vector<double> tGoldTwo(tNumControls, 1. / 300.);
    for(int tIndex = 0; tIndex < tAdjointJacobianTimesDirection.dimension(); tIndex++)
    {
        EXPECT_NEAR(tAdjointJacobianTimesDirection.vector()[tIndex], tGoldTwo[tIndex], tTolerance);
    }
}

#endif
} // PlatoTest
