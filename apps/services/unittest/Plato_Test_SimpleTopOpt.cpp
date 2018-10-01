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
 * Plato_Test_SimpleTopOpt.cpp
 *
 *  Created on: Nov 8, 2017
 */

#include <gtest/gtest.h>

#include <vector>

#include "Plato_SharedField.hpp"
#include "Plato_SharedValue.hpp"
#include "Plato_Communication.hpp"
#include "Plato_StructuralTopologyOptimizationProxyApp.hpp"

#include "Plato_Rosenbrock.hpp"
#include "Plato_ProxyVolume.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_ProxyCompliance.hpp"
#include "Plato_EpetraSerialDenseVector.hpp"
#include "Plato_EpetraSerialDenseMultiVector.hpp"
#include "Plato_StructuralTopologyOptimization.hpp"

#include "Plato_OptimalityCriteriaLightInterface.hpp"

#include "Plato_PrimalProblemStageMng.hpp"
#include "Plato_MethodMovingAsymptotes.hpp"
#include "Plato_ConservativeConvexSeparableAppxDataMng.hpp"
#include "Plato_ConservativeConvexSeparableAppxAlgorithm.hpp"
#include "Plato_GloballyConvergentMethodMovingAsymptotes.hpp"

#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_AugmentedLagrangianStageMng.hpp"
#include "Plato_KelleySachsAugmentedLagrangian.hpp"
#include "Plato_MLKelleySachsAugmentedLagrangian.hpp"

#include "Plato_Diagnostics.hpp"
#include "Plato_StructuralTopologyOptimizationProxyGoldResults.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Output data structure for the Method of Moving Asymptotes (MMA) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmOutputsMMA
{
    OrdinalType mNumOuterIter; /*!< number of outer iterations */
    OrdinalType mNumObjFuncEval; /*!< number of objective function evaluations */
    OrdinalType mNumObjGradEval; /*!< number of objective gradient evaluations */

    ScalarType mKKTMeasure; /*!< Karush-Kuhn-Tucker (KKT) conditions inexactness */
    ScalarType mObjFuncValue; /*!< objective function value */
    ScalarType mNormObjFuncGrad; /*!< norm of the objective function gradient */
    ScalarType mStationarityMeasure; /*!< norm of the descent direction */
    ScalarType mControlStagnationMeasure; /*!< norm of the difference between two subsequent control fields */
    ScalarType mObjectiveStagnationMeasure; /*!< measures stagnation in two subsequent objective function evaluations */

    std::string mStopCriterion; /*!< stopping criterion */

    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mConstraints; /*!< constraint values */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mSolution; /*!< optimal solution */
};
// struct AlgorithmOutputsMMA

/******************************************************************************//**
 * @brief Input data structure for the Method of Moving Asymptotes (MMA) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmInputsMMA
{
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    AlgorithmInputsMMA() :
            mPrintDiagnostics(false),
            mMaxNumOuterIter(500),
            mMovingAsymptoteExpansionFactor(1.2),
            mMovingAsymptoteContractionFactor(0.4),
            mInitialMovingAsymptoteScaleFactor(0.5),
            mMovingAsymptoteUpperBoundScaleFactor(10),
            mMovingAsymptoteLowerBoundScaleFactor(0.01),
            mStationarityTolerance(1e-4),
            mControlStagnationTolerance(1e-8),
            mObjectiveStagnationTolerance(1e-6),
            mKarushKuhnTuckerConditionsTolerance(1e-5),
            mCommWrapper(),
            mMemorySpace(Plato::MemorySpace::HOST),
            mDual(nullptr),
            mLowerBounds(nullptr),
            mUpperBounds(nullptr),
            mInitialGuess(nullptr),
            mReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>())
    {
        mCommWrapper.useDefaultComm();
    }

    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    virtual ~AlgorithmInputsMMA()
    {
    }

    bool mPrintDiagnostics; /*!< flag to enable problem statistics output (default=false) */

    OrdinalType mMaxNumOuterIter; /*!< maximum number of outer iterations */

    ScalarType mMovingAsymptoteExpansionFactor; /*!< moving asymptotes expansion factor */
    ScalarType mMovingAsymptoteContractionFactor; /*!< moving asymptotes' contraction factor */
    ScalarType mInitialMovingAsymptoteScaleFactor; /*!< initial moving asymptotes' scale factor */
    ScalarType mMovingAsymptoteUpperBoundScaleFactor; /*!< scale factor for upper bound on moving asymptotes */
    ScalarType mMovingAsymptoteLowerBoundScaleFactor; /*!< scale factor for lower bound on moving asymptotes */

    ScalarType mStationarityTolerance; /*!< stationarity tolerance */
    ScalarType mControlStagnationTolerance; /*!< control stagnation tolerance */
    ScalarType mObjectiveStagnationTolerance; /*!< objective function stagnation tolerance */
    ScalarType mKarushKuhnTuckerConditionsTolerance; /*!< Karush-Kuhn-Tucker (KKT) inexactness tolerance */

    Plato::CommWrapper mCommWrapper; /*!< distributed memory communication wrapper */
    Plato::MemorySpace::type_t mMemorySpace; /*!< memory space: HOST (default) OR DEVICE */

    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mDual; /*!< Lagrange multipliers */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mLowerBounds; /*!< lower bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mUpperBounds; /*!< upper bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mInitialGuess; /*!< initial guess */
    /*!< operations which require communication across processors, e.g. max, min, global sum */
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mReductionOperations;
};
// struct AlgorithmInputsMMA

/******************************************************************************//**
 * @brief Set Method of Moving Asymptotes (MMA) algorithm inputs
 * @param [in] aInputs Method of Moving Asymptotes algorithm inputs
 * @param [in,out] aAlgorithm Method of Moving Asymptotes algorithm interface
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_mma_algorithm_inputs(const Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> & aInputs,
                                     Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> & aAlgorithm)
{
    if(aInputs.mPrintDiagnostics == true)
    {
        aAlgorithm.enableDiagnostics();
    }

    aAlgorithm.setMovingAsymptoteExpansionFactor(aInputs.mMovingAsymptoteExpansionFactor);
    aAlgorithm.setMovingAsymptoteContractionFactor(aInputs.mMovingAsymptoteContractionFactor);
    aAlgorithm.setInitialMovingAsymptoteScaleFactor(aInputs.mInitialMovingAsymptoteScaleFactor);
    aAlgorithm.setMovingAsymptoteUpperBoundScaleFactor(aInputs.mMovingAsymptoteUpperBoundScaleFactor);
    aAlgorithm.setMovingAsymptoteLowerBoundScaleFactor(aInputs.mMovingAsymptoteLowerBoundScaleFactor);

    aAlgorithm.setMaxNumIterations(aInputs.mMaxNumOuterIter);
    aAlgorithm.setStationarityTolerance(aInputs.mStationarityTolerance);
    aAlgorithm.setControlStagnationTolerance(aInputs.mControlStagnationTolerance);
    aAlgorithm.setObjectiveStagnationTolerance(aInputs.mObjectiveStagnationTolerance);
    aAlgorithm.setKarushKuhnTuckerConditionsTolerance(aInputs.mKarushKuhnTuckerConditionsTolerance);
}
// function set_mma_algorithm_inputs

/******************************************************************************//**
 * @brief Set Method of Moving Asymptotes (MMA) algorithm outputs
 * @param [in] aAlgorithm Method of Moving Asymptotes algorithm interface
 * @param [in,out] aOutputs Method of Moving Asymptotes algorithm outputs
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_mma_algorithm_outputs(const Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> & aAlgorithm,
                                      Plato::AlgorithmOutputsMMA<ScalarType, OrdinalType> & aOutputs)
{
    aOutputs.mNumOuterIter = aAlgorithm.getNumIterationsDone();
    aOutputs.mNumObjFuncEval = aAlgorithm.getDataMng().getNumObjectiveFunctionEvaluations();
    aOutputs.mNumObjGradEval = aAlgorithm.getDataMng().getNumObjectiveGradientEvaluations();

    aOutputs.mKKTMeasure = aAlgorithm.getDataMng().getKarushKuhnTuckerConditionsInexactness();
    aOutputs.mObjFuncValue = aAlgorithm.getDataMng().getCurrentObjectiveFunctionValue();
    aOutputs.mNormObjFuncGrad = aAlgorithm.getDataMng().getNormInactiveGradient();
    aOutputs.mStationarityMeasure = aAlgorithm.getDataMng().getStationarityMeasure();
    aOutputs.mControlStagnationMeasure = aAlgorithm.getDataMng().getControlStagnationMeasure();
    aOutputs.mObjectiveStagnationMeasure = aAlgorithm.getDataMng().getObjectiveStagnationMeasure();

    Plato::get_ccsa_stop_criterion(aAlgorithm.getStoppingCriterion(), aOutputs.mStopCriterion);

    const Plato::MultiVector<ScalarType, OrdinalType> & tSolution = aAlgorithm.getDataMng().getCurrentControl();
    aOutputs.mSolution = tSolution.create();
    Plato::update(static_cast<ScalarType>(1), tSolution, static_cast<ScalarType>(0), *aOutputs.mSolution);

    const OrdinalType tDUAL_VECTOR_INDEX = 0;
    const Plato::MultiVector<ScalarType, OrdinalType> & tConstraintValues = aAlgorithm.getDataMng().getCurrentConstraintValues();
    aOutputs.mConstraints = tConstraintValues[tDUAL_VECTOR_INDEX].create();
    aOutputs.mConstraints->update(static_cast<ScalarType>(1), tConstraintValues[tDUAL_VECTOR_INDEX], static_cast<ScalarType>(0));
}
// function set_mma_algorithm_outputs

/******************************************************************************//**
 * @brief Method of Moving Asymptotes (MMA) algorithm interface
 * @param [in] aObjective user-defined objective function
 * @param [in] aConstraints user-defined list of constraints
 * @param [in] aInputs Optimality Criteria algorithm inputs
 * @param [in,out] aOutputs Optimality Criteria algorithm outputs
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void solve_mma(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aObjective,
                      const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aConstraints,
                      const Plato::AlgorithmInputsMMA<ScalarType, OrdinalType> & aInputs,
                      Plato::AlgorithmOutputsMMA<ScalarType, OrdinalType> & aOutputs)
{
    // ********* ALLOCATE DATA STRUCTURES *********
    std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> tDataFactory;
    tDataFactory = std::make_shared<Plato::DataFactory<ScalarType, OrdinalType>>();
    tDataFactory->setCommWrapper(aInputs.mCommWrapper);
    tDataFactory->allocateDual(*aInputs.mDual);
    tDataFactory->allocateControl(*aInputs.mInitialGuess);
    tDataFactory->allocateControlReductionOperations(*aInputs.mReductionOperations);

    // ********* ALLOCATE OPTIMALITY CRITERIA ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType>> tDataMng;
    tDataMng = std::make_shared<Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType>>(tDataFactory);
    tDataMng->setInitialGuess(*aInputs.mInitialGuess);
    tDataMng->setControlLowerBounds(*aInputs.mLowerBounds);
    tDataMng->setControlUpperBounds(*aInputs.mUpperBounds);

    // ********* ALLOCATE STAGE MANAGER MANAGER *********
    std::shared_ptr<Plato::PrimalProblemStageMng<ScalarType, OrdinalType>> tStageMng;
    tStageMng = std::make_shared<Plato::PrimalProblemStageMng<ScalarType, OrdinalType>>(tDataFactory, aObjective, aConstraints);

    // ********* ALLOCATE OPTIMALITY CRITERIA ALGORITHM AND SOLVE OPTIMIZATION PROBLEM *********
    std::shared_ptr<Plato::MethodMovingAsymptotes<ScalarType, OrdinalType>> tSubProblem;
    tSubProblem = std::make_shared<Plato::MethodMovingAsymptotes<ScalarType, OrdinalType>>(tDataFactory);
    Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> tAlgorithm(tStageMng, tDataMng, tSubProblem);
    Plato::set_mma_algorithm_inputs(aInputs, tAlgorithm);
    tAlgorithm.solve();
    Plato::set_mma_algorithm_outputs(tAlgorithm, aOutputs);
}
// function solve_mma

} // namespace Plato


namespace PlatoTest
{

TEST(PlatoTest, StructuralTopologyOptimization_ElementStiffnessMatrix)
{
    const double tPoissonRatio = 0.3;
    const double tElasticModulus = 1;
    const int tNumElementsXdirection = 1;
    const int tNumElementsYdirection = 1;
    Plato::StructuralTopologyOptimization tPDE(tPoissonRatio, tElasticModulus, tNumElementsXdirection, tNumElementsYdirection);
    const Epetra_SerialDenseMatrix & tElementStiffnessMatrix = tPDE.getElementStiffnessMatrix();

    const int tNumDOFs = 8;
    const int tStride = tNumDOFs;
    std::vector<double> tGoldData = TopoProxy::getGoldElemStiffnessMatrix();
    Epetra_SerialDenseMatrix tGoldElementStiffnessMatrix(Epetra_DataAccess::Copy, tGoldData.data(), tStride, tNumDOFs, tNumDOFs);

    const double tTolerance = 1e-6;
    for(int tRowIndex = 0; tRowIndex < tNumDOFs; tRowIndex++)
    {
        for(int tColumnIndex = 0; tColumnIndex < tNumDOFs; tColumnIndex++)
        {
            EXPECT_NEAR(tGoldElementStiffnessMatrix(tRowIndex, tColumnIndex), tElementStiffnessMatrix(tRowIndex, tColumnIndex), tTolerance);
        }
    }
}

TEST(PlatoTest, StructuralTopologyOptimization_Solve)
{
    // *********** RESULTS WERE GENERATED WITH THE MATLAB 99 LINE TOPOLOGY OPTIMIZATION CODE (top.m) ***********
    const double tPoissonRatio = 0.3;
    const double tElasticModulus = 1;
    const int tNumElementsXdirection = 30;
    const int tNumElementsYdirection = 10;
    Plato::StructuralTopologyOptimization tPDE(tPoissonRatio, tElasticModulus, tNumElementsXdirection, tNumElementsYdirection);

    // ************** TEST SET FORCE VECTOR **************
    const int tTotalNumDOFs = tPDE.getGlobalNumDofs();
    Epetra_SerialDenseVector tForce(tTotalNumDOFs);
    const int tDOFsIndex = 1;
    tForce[tDOFsIndex] = -1;
    tPDE.setForceVector(tForce);
    const double tTolerance = 1e-6;
    const Epetra_SerialDenseVector & tSolverForce = tPDE.getForceVector();
    for(int tIndex = 0; tIndex < tForce.Length(); tIndex++)
    {
        EXPECT_NEAR(tForce[tIndex], tSolverForce[tIndex], tTolerance);
    }

    // ************** TEST SET FIXED DEGREES OF FREEDOM (DOFs) VECTOR **************
    std::vector<double> tData = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
    Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tData.data(), tData.size());
    tPDE.setFixedDOFs(tFixedDOFs);
    const Epetra_SerialDenseVector & tSolverFixedDOFs = tPDE.getFixedDOFs();
    EXPECT_EQ(tFixedDOFs.Length(), tSolverFixedDOFs.Length());
    for(int tIndex = 0; tIndex < tFixedDOFs.Length(); tIndex++)
    {
        EXPECT_NEAR(tFixedDOFs[tIndex], tSolverFixedDOFs[tIndex], tTolerance);
    }

    // ************** TEST SOLVE **************
    const int tNumDesignVariables = tPDE.getNumDesignVariables();
    std::vector<double> tControlData(tNumDesignVariables, 1);
    Epetra_SerialDenseVector tControl(Epetra_DataAccess::Copy, tControlData.data(), tNumDesignVariables);
    tPDE.solve(tControl);
    tPDE.cacheState();

    double tStiffMatrixNormOne = 5.109890109890110;
    EXPECT_NEAR(tStiffMatrixNormOne, tPDE.getStiffnessMatrixOneNorm(), tTolerance);
    std::vector<double> tGoldSolutionData = TopoProxy::getGoldStateData();
    const Epetra_SerialDenseVector & tSolution = tPDE.getDisplacements();
    for(int tIndex = 0; tIndex < tSolution.Length(); tIndex++)
    {
        EXPECT_NEAR(tSolution[tIndex], tGoldSolutionData[tIndex], tTolerance);
    }
}

TEST(PlatoTest, StructuralTopologyOptimization_ComplianceObjective_Gradient_Hessian)
{
    // *********** RESULTS WERE GENERATED WITH THE MATLAB 99 LINE TOPOLOGY OPTIMIZATION CODE (top.m) ***********
    const double tPoissonRatio = 0.3;
    const double tElasticModulus = 1;
    const int tNumElementsXdirection = 30;
    const int tNumElementsYdirection = 10;
    Plato::StructuralTopologyOptimization tPDE(tPoissonRatio, tElasticModulus, tNumElementsXdirection, tNumElementsYdirection);

    // ************** SET FORCE VECTOR **************
    const int tTotalNumDOFs = tPDE.getGlobalNumDofs();
    Epetra_SerialDenseVector tForce(tTotalNumDOFs);
    const int tDOFsIndex = 1;
    tForce[tDOFsIndex] = -1;
    tPDE.setForceVector(tForce);

    // ************** SET FIXED DEGREES OF FREEDOM (DOFs) VECTOR **************
    std::vector<double> tData = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
    Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tData.data(), tData.size());
    tPDE.setFixedDOFs(tFixedDOFs);

    // ************** SOLVE PDE **************
    const int tNumDesignVariables = tPDE.getNumDesignVariables();
    std::vector<double> tControlData(tNumDesignVariables, 1);
    Epetra_SerialDenseVector tControl(Epetra_DataAccess::Copy, tControlData.data(), tNumDesignVariables);
    tPDE.solve(tControl);
    tPDE.cacheState();

    // ************** TEST COMPUTE OBJECTIVE **************
    const double tTolerance = 1e-6;
    double tObjectiveGold = 123.069351174;
    EXPECT_NEAR(tObjectiveGold, tPDE.computeCompliance(tControl), tTolerance);

    // ************** TEST COMPUTE SENSITIVITY **************
    Epetra_SerialDenseVector tGradient(tNumDesignVariables);
    tPDE.computeComplianceGradient(tControl, tGradient);
    std::vector<double> tGoldGradientData = TopoProxy::getGoldGradientData();
    Epetra_SerialDenseVector tGoldGradient(Epetra_DataAccess::Copy, tGoldGradientData.data(), tNumDesignVariables);
    for(int tIndex = 0; tIndex < tGradient.Length(); tIndex++)
    {
        EXPECT_NEAR(tGradient[tIndex], tGoldGradient[tIndex], tTolerance);
    }

    // ************** TEST COMPUTE HESSIAN (GAUSS NEWTON APPROXIMATION) TIMES VECTOR **************
    std::vector<double> tVectorData(tNumDesignVariables, 1);
    Epetra_SerialDenseVector tVector(Epetra_DataAccess::Copy, tVectorData.data(), tNumDesignVariables);
    Epetra_SerialDenseVector tHessianTimesVector(tNumDesignVariables);
    tPDE.computeComplianceHessianTimesVector(tControl, tVector, tHessianTimesVector);
    std::vector<double> tGoldHessianTimesVector = TopoProxy::getGoldHessianTimesVectorData();
    for(int tIndex = 0; tIndex < tGradient.Length(); tIndex++)
    {
        EXPECT_NEAR(tHessianTimesVector[tIndex], tGoldHessianTimesVector[tIndex], tTolerance);
    }
}

TEST(PlatoTest, StructuralTopologyOptimization_ApplySensitivityFilter)
{
    // *********** RESULTS WERE GENERATED WITH THE MATLAB 99 LINE TOPOLOGY OPTIMIZATION CODE (top.m) ***********
    const double tPoissonRatio = 0.3;
    const double tElasticModulus = 1;
    const int tNumElementsXdirection = 30;
    const int tNumElementsYdirection = 10;
    Plato::StructuralTopologyOptimization tPDE(tPoissonRatio, tElasticModulus, tNumElementsXdirection, tNumElementsYdirection);

    // ************** SET FORCE VECTOR **************
    const int tGlobalNumDofs = tPDE.getGlobalNumDofs();
    Epetra_SerialDenseVector tForce(tGlobalNumDofs);
    const int tDOFsIndex = 1;
    tForce[tDOFsIndex] = -1;
    tPDE.setForceVector(tForce);

    // ************** SET FIXED DEGREES OF FREEDOM (DOFs) VECTOR **************
    std::vector<double> tData = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
    Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tData.data(), tData.size());
    tPDE.setFixedDOFs(tFixedDOFs);

    // ************** SOLVE PDE **************
    const int tNumDesignVariables = tPDE.getNumDesignVariables();
    std::vector<double> tControlData(tNumDesignVariables, 1);
    Epetra_SerialDenseVector tControl(Epetra_DataAccess::Copy, tControlData.data(), tNumDesignVariables);
    tPDE.solve(tControl);
    tPDE.cacheState();

    // ************** COMPUTE SENSITIVITY **************
    Epetra_SerialDenseVector tUnfilteredGradient(tNumDesignVariables);
    tPDE.computeComplianceGradient(tControl, tUnfilteredGradient);

    // ************** TEST APPLY FILTER TO GRADIENT **************
    Epetra_SerialDenseVector tFilteredGradient(tNumDesignVariables);
    tPDE.applySensitivityFilter(tControl, tUnfilteredGradient, tFilteredGradient);

    const double tTolerance = 1e-6;
    std::vector<double> tGoldData = TopoProxy::getGoldFilteredGradient();
    Epetra_SerialDenseVector tGoldFilteredGradient(Epetra_DataAccess::Copy, tGoldData.data(), tNumDesignVariables);
    for(int tIndex = 0; tIndex < tGoldFilteredGradient.Length(); tIndex++)
    {
        EXPECT_NEAR(tFilteredGradient[tIndex], tGoldFilteredGradient[tIndex], tTolerance);
    }
}

TEST(PlatoTest, StructuralTopologyOptimizationProxyApp)
{
    Plato::StructuralTopologyOptimizationProxyApp tApplication;
    tApplication.initialize();

    // ******** SET APPLICATION COMM ********
    Plato::CommunicationData tPlatoComm;
    tPlatoComm.mLocalComm = MPI_COMM_WORLD;
    tPlatoComm.mInterComm = MPI_COMM_WORLD;
    tPlatoComm.mLocalCommName = "Plato";
    auto layout = Plato::data::layout_t::SCALAR_FIELD;
    tApplication.exportDataMap(layout, tPlatoComm.mMyOwnedGlobalIDs[layout]);

    Plato::CommunicationData tApplicationComm;
    tApplicationComm.mLocalComm = MPI_COMM_WORLD;
    tApplicationComm.mInterComm = MPI_COMM_WORLD;
    tApplicationComm.mLocalCommName = "Application";
    layout = Plato::data::layout_t::VECTOR_FIELD;
    tApplication.exportDataMap(layout, tApplicationComm.mMyOwnedGlobalIDs[layout]);

    // ******** TEST PDE SOLVE ********
    std::string tArgumentName("Topology");
    int tNumDesignVariables = tApplication.getNumDesignVariables();
    Plato::SharedField tSharedControl(tArgumentName, Plato::communication::broadcast_t::SENDER_AND_RECEIVER, 
                                      tPlatoComm, Plato::data::layout_t::SCALAR_FIELD);
    std::vector<double> tControl(tNumDesignVariables, 1);
    tSharedControl.setData(tControl);
    tSharedControl.transmitData();
    tApplication.importData(tArgumentName, tSharedControl);

    std::string tOperation("Solve");
    tApplication.compute(tOperation);
    Plato::SharedField tSharedState(tArgumentName,
                                    Plato::communication::broadcast_t::SENDER_AND_RECEIVER,
                                    tApplicationComm,
                                    Plato::data::layout_t::VECTOR_FIELD);
    tArgumentName = "StateSolution";
    tApplication.exportData(tArgumentName, tSharedState);
    tSharedState.transmitData();

    const double tTolerance = 1e-6;
    const int tNumStates = tApplication.getGlobalNumDofs();
    std::vector<double> tState(tNumStates, 0);
    tSharedState.getData(tState);
    std::vector<double> tGoldStateData = TopoProxy::getGoldStateData();
    for(int tIndex = 0; tIndex < tNumStates; tIndex++)
    {
        EXPECT_NEAR(tState[tIndex], tGoldStateData[tIndex], tTolerance);
    }

    // ******** TEST OBJECTIVE EVALUATION ********
    tOperation = "Objective";
    tApplication.compute(tOperation);
    tArgumentName = "InternalEnergy";
    std::string tProviderName("Application");
    Plato::SharedValue tSharedObjective(tArgumentName, tProviderName, tApplicationComm);
    tApplication.exportData(tArgumentName, tSharedObjective);
    tSharedObjective.transmitData();

    std::vector<double> tObjectiveData(1);
    tSharedObjective.getData(tObjectiveData);
    double tObjectiveGold = 123.069351174;
    EXPECT_NEAR(tObjectiveGold, tObjectiveData[0], tTolerance);

    // ******** TEST CONSTRAINT EVALUATION ********
    tOperation = "Constraint";
    tApplication.compute(tOperation);
    tArgumentName = "Volume";
    Plato::SharedValue tSharedConstraint(tArgumentName, tProviderName, tApplicationComm);
    tApplication.exportData(tArgumentName, tSharedConstraint);
    tSharedConstraint.transmitData();

    std::vector<double> tConstraintData(1);
    tSharedConstraint.getData(tConstraintData);
    double tConstraintGold = 0.6;
    EXPECT_NEAR(tConstraintGold, tConstraintData[0], tTolerance);

    // ******** TEST OBJECTIVE GRADIENT CALCULATION ********
    tOperation = "ObjectiveGradient";
    tApplication.compute(tOperation);
    tArgumentName = "InternalEnergyGradient";
    Plato::SharedField tSharedGradient(tArgumentName, Plato::communication::broadcast_t::SENDER_AND_RECEIVER, 
                                       tPlatoComm, Plato::data::layout_t::SCALAR_FIELD);
    tApplication.exportData(tArgumentName, tSharedGradient);
    tSharedGradient.transmitData();

    std::vector<double> tGradient(tNumDesignVariables, 0);
    tSharedGradient.getData(tGradient);
    std::vector<double> tGoldGradientData = TopoProxy::getGoldGradientData();
    for(int tIndex = 0; tIndex < tNumDesignVariables; tIndex++)
    {
        EXPECT_NEAR(tGradient[tIndex], tGoldGradientData[tIndex], tTolerance);
    }

    // ******** TEST FILTERED OBJECTIVE GRADIENT CALCULATION ********
    tOperation = "FilteredObjectiveGradient";
    tApplication.compute(tOperation);
    tArgumentName = "FilteredInternalEnergyGradient";
    tApplication.exportData(tArgumentName, tSharedGradient);
    tSharedGradient.transmitData();

    tSharedGradient.getData(tGradient);
    std::vector<double> tGoldFilteredGradientData = TopoProxy::getGoldFilteredGradient();
    for(int tIndex = 0; tIndex < tNumDesignVariables; tIndex++)
    {
        EXPECT_NEAR(tGradient[tIndex], tGoldFilteredGradientData[tIndex], tTolerance);
    }

    // ******** TEST CONSTRAINT GRADIENT CALCULATION ********
    tOperation = "ConstraintGradient";
    tApplication.compute(tOperation);
    tArgumentName = "VolumeGradient";
    tApplication.exportData(tArgumentName, tSharedGradient);
    tSharedGradient.transmitData();

    tSharedGradient.getData(tGradient);
    std::vector<double> tGoldConstraintGradientData(tNumDesignVariables, 1.0 / 300.0);
    for(int tIndex = 0; tIndex < tNumDesignVariables; tIndex++)
    {
        EXPECT_NEAR(tGradient[tIndex], tGoldConstraintGradientData[tIndex], tTolerance);
    }
}

TEST(PlatoTest, EpetraVectorSize)
{
    const size_t tNumElements = 10;
    Plato::EpetraSerialDenseVector<double> tVector(tNumElements);
    const size_t tGold = 10;
    EXPECT_EQ(tVector.size(), tGold);
}

TEST(PlatoTest, EpetraVectorScale)
{
    const double tBaseValue = 1;
    const int tNumElements = 10;
    Plato::EpetraSerialDenseVector<double, int> tVector(tNumElements, tBaseValue);

    double tScaleValue = 2;
    tVector.scale(tScaleValue);

    double tGold = 2;
    double tTolerance = 1e-6;
    for(int tIndex = 0; tIndex < tVector.size(); tIndex++)
    {
        EXPECT_NEAR(tVector[tIndex], tGold, tTolerance);
    }
}

TEST(PlatoTest, EpetraVectorEntryWiseProduct)
{
    std::vector<double> tTemplateVector =
        { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::EpetraSerialDenseVector<double> tVector1(tTemplateVector);
    Plato::EpetraSerialDenseVector<double> tVector2(tTemplateVector);

    tVector1.entryWiseProduct(tVector2);

    double tTolerance = 1e-6;
    std::vector<double> tGold = { 1, 4, 9, 16, 25, 36, 49, 64, 81, 100 };
    for(size_t tIndex = 0; tIndex < tVector1.size(); tIndex++)
    {
        EXPECT_NEAR(tVector1[tIndex], tGold[tIndex], tTolerance);
    }
}

TEST(PlatoTest, EpetraVectorModulus)
{
    std::vector<double> tTemplateVector = { -1, 2, -3, 4, 5, -6, -7, 8, -9, -10 };
    Plato::EpetraSerialDenseVector<double> tVector(tTemplateVector);

    tVector.modulus();

    double tTolerance = 1e-6;
    std::vector<double> tGold = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    for(size_t tIndex = 0; tIndex < tVector.size(); tIndex++)
    {
        EXPECT_NEAR(tVector[tIndex], tGold[tIndex], tTolerance);
    }
}

TEST(PlatoTest, EpetraVectorDot)
{
    std::vector<double> tTemplateVector1 = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::EpetraSerialDenseVector<double> tVector1(tTemplateVector1);
    std::vector<double> tTemplateVector2 = { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 };
    Plato::EpetraSerialDenseVector<double> tVector2(tTemplateVector2);

    double tDot = tVector1.dot(tVector2);

    double tGold = 110;
    double tTolerance = 1e-6;
    EXPECT_NEAR(tDot, tGold, tTolerance);
}

TEST(PlatoTest, EpetraVectorFill)
{
    std::vector<double> tTemplateVector = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::EpetraSerialDenseVector<double> tVector(tTemplateVector);

    double tFillValue = 3;
    tVector.fill(tFillValue);

    double tGold = 3.;
    double tTolerance = 1e-6;
    for(size_t tIndex = 0; tIndex < tVector.size(); tIndex++)
    {
        EXPECT_NEAR(tVector[tIndex], tGold, tTolerance);
    }
}

TEST(PlatoTest, EpetraVectorCreate)
{
    std::vector<double> tTemplateVector = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::EpetraSerialDenseVector<double> tVector(tTemplateVector);

    // TEST ONE: CREATE COPY OF BASE CONTAINER WITH THE SAME NUMBER OF ELEMENTS AS THE BASE VECTOR AND FILL IT WITH ZEROS
    std::shared_ptr<Plato::Vector<double>> tCopy1 = tVector.create();

    size_t tGoldSize1 = 10;
    EXPECT_EQ(tCopy1->size(), tGoldSize1);
    EXPECT_TRUE(tCopy1->size() == tVector.size());

    double tGoldDot1 = 0;
    double tTolerance = 1e-6;
    EXPECT_NEAR(tCopy1->dot(tVector), tGoldDot1, tTolerance);
}

TEST(PlatoTest, EpetraVectorData)
{
    std::vector<double> tData = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    Plato::EpetraSerialDenseVector<double> tVector(tData);

    // TEST DATA FUNCTION
    size_t tGoldSize1 = 10;
    EXPECT_EQ(tVector.size(), tGoldSize1);
    const double tTolerance = 1e-6;
    for(size_t tIndex = 0; tIndex < tVector.size(); tIndex++)
    {
        EXPECT_NEAR(tData[tIndex], tVector.data()[tIndex], tTolerance);
    }
}

TEST(PlatoTest, ProxyCompliance)
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

    // ************** ALLOCATE COMPLIANCE CRITERION **************
    Plato::ProxyCompliance<double> tObjective(tPDE);

    // ************** TEST COMPLIANCE CRITERION EVALUATION **************
    const double tBaseValue = 1;
    const size_t tNumVectors = 1;
    const size_t tNumControls = tPDE->getNumDesignVariables();
    Plato::EpetraSerialDenseMultiVector<double> tControl(tNumVectors, tNumControls, tBaseValue);
    double tObjectiveValue = tObjective.value(tControl);
    const double tTolerance = 1e-6;
    double tObjectiveGold = 1.;
    EXPECT_NEAR(tObjectiveGold, tObjectiveValue, tTolerance);

    // ************** TEST COMPLIANCE CRITERION GRADIENT **************
    Plato::EpetraSerialDenseMultiVector<double> tGradient(tNumVectors, tNumControls);
    tObjective.cacheData();
    tObjective.gradient(tControl, tGradient);

    const size_t tVectorIndex = 0;
    std::vector<double> tGoldGradient = TopoProxy::getGoldNormalizedFilteredGradient();
    for(size_t tIndex = 0; tIndex < tGoldGradient.size(); tIndex++)
    {
        EXPECT_NEAR(tGradient(tVectorIndex, tIndex), tGoldGradient[tIndex], tTolerance);
    }
}

TEST(PlatoTest, ProxyVolume)
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
    Plato::ProxyVolume<double> tConstraint(tPDE);

    // ************** TEST VOLUME CRITERION EVALUATION **************
    const double tBaseValue = 1;
    const size_t tNumVectors = 1;
    const size_t tNumControls = tPDE->getNumDesignVariables();
    Plato::EpetraSerialDenseMultiVector<double> tControl(tNumVectors, tNumControls, tBaseValue);
    double tObjectiveValue = tConstraint.value(tControl);
    const double tTolerance = 1e-6;
    double tConstraintGold = 0.6;
    EXPECT_NEAR(tConstraintGold, tObjectiveValue, tTolerance);

    // ************** TEST VOLUME CRITERION GRADIENT **************
    Plato::EpetraSerialDenseMultiVector<double> tGradient(tNumVectors, tNumControls);
    tConstraint.cacheData();
    tConstraint.gradient(tControl, tGradient);

    const size_t tVectorIndex = 0;
    std::vector<double> tGoldGradient(tNumControls, 1. / 300.0);
    for(size_t tIndex = 0; tIndex < tGoldGradient.size(); tIndex++)
    {
        EXPECT_NEAR(tGradient(tVectorIndex, tIndex), tGoldGradient[tIndex], tTolerance);
    }
}

TEST(PlatoTest, SolveStrucTopoOptimalityCriteriaLightInterface)
{
    // ************** ALLOCATE SIMPLE STRUCTURAL TOPOLOGY OPTIMIZATION SOLVER **************
    const double tPoissonRatio = 0.3;
    const double tElasticModulus = 1;
    const int tNumElementsXdirection = 30;
    const int tNumElementsYdirection = 10;
    std::shared_ptr<Plato::StructuralTopologyOptimization> tPDE;
    tPDE = std::make_shared<Plato::StructuralTopologyOptimization>(tPoissonRatio, tElasticModulus, tNumElementsXdirection, tNumElementsYdirection);

    // ************** SET FORCE VECTOR **************
    const int tGlobalNumDofs = tPDE->getGlobalNumDofs();
    Epetra_SerialDenseVector tForce(tGlobalNumDofs);
    const int tDOFsIndex = 1;
    tForce[tDOFsIndex] = -1;
    tPDE->setForceVector(tForce);

    // ************** SET FIXED DEGREES OF FREEDOM (DOFs) VECTOR **************
    std::vector<double> tDofs = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
    Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tDofs.data(), tDofs.size());
    tPDE->setFixedDOFs(tFixedDOFs);

    // ************** ALLOCATE VOLUME AND COMPLIANCE CRITERION **************
    std::shared_ptr<Plato::ProxyVolume<double>> tVolume = std::make_shared<Plato::ProxyVolume<double>>(tPDE);
    std::shared_ptr<Plato::ProxyCompliance<double>> tCompliance = std::make_shared<Plato::ProxyCompliance<double>>(tPDE);
    std::shared_ptr<Plato::CriterionList<double>> tConstraints = std::make_shared<Plato::CriterionList<double>>();
    tConstraints->add(tVolume);

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumDuals = 1;
    const size_t tNumVectors = 1;
    const size_t tNumControls = tPDE->getNumDesignVariables();
    Plato::AlgorithmInputsOC<double> tInputs;
    tInputs.mDual = std::make_shared<Plato::EpetraSerialDenseMultiVector<double>>(tNumVectors, tNumDuals);
    const double tValue = tPDE->getVolumeFraction();
    tInputs.mInitialGuess = std::make_shared<Plato::EpetraSerialDenseMultiVector<double>>(tNumVectors, tNumControls, tValue);
    tInputs.mUpperBounds = std::make_shared<Plato::EpetraSerialDenseMultiVector<double>>(tNumVectors, tNumControls, 1.0 /* base value */);
    tInputs.mLowerBounds = std::make_shared<Plato::EpetraSerialDenseMultiVector<double>>(tNumVectors, tNumControls, 1e-3 /* base value */);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::AlgorithmOutputsOC<double> tOutputs;
    Plato::solve_optimality_criteria<double, size_t>(tCompliance, tConstraints, tInputs, tOutputs);

    const double tTolerance = 1e-6;
    EXPECT_EQ(49, tOutputs.mNumOuterIter);
    EXPECT_EQ(50, tOutputs.mNumObjFuncEval);
    EXPECT_EQ(50, tOutputs.mNumObjGradEval);
    EXPECT_NEAR(0.17786129647595, tOutputs.mObjFuncValue, tTolerance);
    EXPECT_NEAR(0.066488674895463576, tOutputs.mNormObjFuncGrad, tTolerance);
    EXPECT_NEAR(0.065024946645311221, tOutputs.mControlStagnationMeasure, tTolerance);
    EXPECT_NEAR(1.4078545110540741e-06, tOutputs.mObjectiveStagnationMeasure, tTolerance);
    EXPECT_NEAR(5.8759036524747e-6, (*tOutputs.mConstraints)[0], tTolerance);
    EXPECT_STREQ("\n\n****** Optimization stopping due to objective stagnation. ******\n\n", tOutputs.mStopCriterion.c_str());
    std::vector<double> tGoldControl = TopoProxy::get_gold_control_optimality_criteria_test();
    for(size_t tIndex = 0; tIndex < tGoldControl.size(); tIndex++)
    {
        EXPECT_NEAR(tGoldControl[tIndex], (*tOutputs.mSolution)(0 /* vector index */, tIndex), tTolerance);
    }
}

TEST(PlatoTest, SolveStrucTopoWithGCMMA)
{
    // ************** ALLOCATE SIMPLE STRUCTURAL TOPOLOGY OPTIMIZATION SOLVER **************
     const double tPoissonRatio = 0.3;
     const double tElasticModulus = 1;
     const int tNumElementsXdirection = 30;
     const int tNumElementsYdirection = 10;
     std::shared_ptr<Plato::StructuralTopologyOptimization> tPDE =
             std::make_shared<Plato::StructuralTopologyOptimization>(tPoissonRatio, tElasticModulus, tNumElementsXdirection, tNumElementsYdirection);
     tPDE->setFilterRadius(1.2);

     // ************** SET FORCE VECTOR **************
     const int tGlobalNumDofs = tPDE->getGlobalNumDofs();
     Epetra_SerialDenseVector tForce(tGlobalNumDofs);
     const int tDOFsIndex = 1;
     tForce[tDOFsIndex] = -1;
     tPDE->setForceVector(tForce);

     // ************** SET FIXED DEGREES OF FREEDOM (DOFs) VECTOR **************
     std::vector<double> tDofs = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
     Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tDofs.data(), tDofs.size());
     tPDE->setFixedDOFs(tFixedDOFs);

     // ************** ALLOCATE VOLUME AND COMPLIANCE CRITERION **************
     std::shared_ptr<Plato::ProxyVolume<double>> tVolume = std::make_shared<Plato::ProxyVolume<double>>(tPDE);
     std::shared_ptr<Plato::ProxyCompliance<double>> tCompliance = std::make_shared<Plato::ProxyCompliance<double>>(tPDE);
     tCompliance->disableFilter();
     std::shared_ptr<Plato::CriterionList<double>> tConstraints = std::make_shared<Plato::CriterionList<double>>();
     tConstraints->add(tVolume);

     // ********* ALLOCATE CORE DATA STRUCTURES *********
     std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
     const size_t tNumDuals = 1;
     const size_t tNumVectors = 1;
     Plato::EpetraSerialDenseMultiVector<double> tDual(tNumVectors, tNumDuals);
     tDataFactory->allocateDual(tDual);
     const size_t tNumControls = tPDE->getNumDesignVariables();
     Plato::EpetraSerialDenseMultiVector<double> tControl(tNumVectors, tNumControls);
     tDataFactory->allocateControl(tControl);

     // ********* ALLOCATE PRIMAL PROBLEM STAGE MANAGER *********
     std::shared_ptr<Plato::PrimalProblemStageMng<double>> tStageMng =
             std::make_shared<Plato::PrimalProblemStageMng<double>>(tDataFactory, tCompliance, tConstraints);

     // ********* ALLOCATE CCSA ALGORITHM DATA MANAGER *********
     std::shared_ptr<Plato::ConservativeConvexSeparableAppxDataMng<double>> tDataMng =
             std::make_shared<Plato::ConservativeConvexSeparableAppxDataMng<double>>(tDataFactory);

     // ********* SET BOUNDS AND INITIAL GUESS *********
     double tValue = tPDE->getVolumeFraction();
     tDataMng->setInitialGuess(tValue);
     tValue = 1e-3;
     tDataMng->setControlLowerBounds(tValue);
     tValue = 1;
     tDataMng->setControlUpperBounds(tValue);

     // ********* ALLOCATE GCMMA SUBPROBLEM MANAGER *********
     std::shared_ptr<Plato::GloballyConvergentMethodMovingAsymptotes<double>> tSubProblem =
             std::make_shared<Plato::GloballyConvergentMethodMovingAsymptotes<double>>(tDataFactory);

     // ********* ALLOCATE CCSA ALGORITHM AND SOLVE STRUCTURAL TOPOLOGY OPTIMIZATION PROBLEM *********
     Plato::ConservativeConvexSeparableAppxAlgorithm<double> tAlgorithm(tStageMng, tDataMng, tSubProblem);
     tAlgorithm.solve();

     // ********* TEST OUTPUT DATA *********
     const double tTolerance = 1e-6;
     const double tCurrentObjective = tDataMng->getCurrentObjectiveFunctionValue();
     const double tGoldObjective = 0.161095582197499;
     EXPECT_NEAR(tCurrentObjective, tGoldObjective, tTolerance);

     const size_t tVectorIndex = 0;
     std::vector<double> tGoldControl = TopoProxy::getGoldControlGcmmaTest();
     const Plato::Vector<double> & tCurrentControl = tDataMng->getCurrentControl(tVectorIndex);
     for(size_t tIndex = 0; tIndex < tGoldControl.size(); tIndex++)
     {
         EXPECT_NEAR(tCurrentControl[tIndex], tGoldControl[tIndex], tTolerance);
     }
}

TEST(PlatoTest, SolveStrucTopoMethodMovingAsymptotesLightInterface)
{
    // ************** ALLOCATE SIMPLE STRUCTURAL TOPOLOGY OPTIMIZATION SOLVER **************
    const double tPoissonRatio = 0.3;
    const double tElasticModulus = 1;
    const int tNumElementsXdirection = 30;
    const int tNumElementsYdirection = 10;
    std::shared_ptr<Plato::StructuralTopologyOptimization> tPDE;
    tPDE = std::make_shared<Plato::StructuralTopologyOptimization>(tPoissonRatio, tElasticModulus, tNumElementsXdirection, tNumElementsYdirection);

    // ************** SET FORCE VECTOR **************
    const int tGlobalNumDofs = tPDE->getGlobalNumDofs();
    Epetra_SerialDenseVector tForce(tGlobalNumDofs);
    const int tDOFsIndex = 1;
    tForce[tDOFsIndex] = -1;
    tPDE->setForceVector(tForce);

    // ************** SET FIXED DEGREES OF FREEDOM (DOFs) VECTOR **************
    std::vector<double> tDofs = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
    Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tDofs.data(), tDofs.size());
    tPDE->setFixedDOFs(tFixedDOFs);

    // ************** ALLOCATE VOLUME AND COMPLIANCE CRITERION **************
    std::shared_ptr<Plato::ProxyVolume<double>> tVolume = std::make_shared<Plato::ProxyVolume<double>>(tPDE);
    std::shared_ptr<Plato::ProxyCompliance<double>> tCompliance = std::make_shared<Plato::ProxyCompliance<double>>(tPDE);
    tCompliance->disableFilter();
    std::shared_ptr<Plato::CriterionList<double>> tConstraints = std::make_shared<Plato::CriterionList<double>>();
    tConstraints->add(tVolume);

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumDuals = 1;
    const size_t tNumVectors = 1;
    const size_t tNumControls = tPDE->getNumDesignVariables();
    Plato::AlgorithmInputsMMA<double> tInputs;
    tInputs.mDual = std::make_shared<Plato::EpetraSerialDenseMultiVector<double>>(tNumVectors, tNumDuals);
    const double tValue = tPDE->getVolumeFraction();
    tInputs.mInitialGuess = std::make_shared<Plato::EpetraSerialDenseMultiVector<double>>(tNumVectors, tNumControls, tValue);
    tInputs.mUpperBounds = std::make_shared<Plato::EpetraSerialDenseMultiVector<double>>(tNumVectors, tNumControls, 1.0 /* base value */);
    tInputs.mLowerBounds = std::make_shared<Plato::EpetraSerialDenseMultiVector<double>>(tNumVectors, tNumControls, 1e-3 /* base value */);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::AlgorithmOutputsMMA<double> tOutputs;
    Plato::solve_mma<double, size_t>(tCompliance, tConstraints, tInputs, tOutputs);

    const double tTolerance = 1e-6;
    EXPECT_EQ(95, tOutputs.mNumOuterIter);
    EXPECT_EQ(96, tOutputs.mNumObjFuncEval);
    EXPECT_EQ(96, tOutputs.mNumObjGradEval);
    EXPECT_NEAR(0.16688506430875721, tOutputs.mObjFuncValue, tTolerance);
    EXPECT_NEAR(0.00013414822686334704, tOutputs.mKKTMeasure, tTolerance);
    EXPECT_NEAR(2.9065999369154898e-5, tOutputs.mNormObjFuncGrad, tTolerance);
    EXPECT_NEAR(-0.078339302681999956, (*tOutputs.mConstraints)[0], tTolerance);
    EXPECT_NEAR(0.0087197998107464668, tOutputs.mStationarityMeasure, tTolerance);
    EXPECT_NEAR(0.00046357698689347693, tOutputs.mControlStagnationMeasure, tTolerance);
    EXPECT_NEAR(8.0091931212167289e-7, tOutputs.mObjectiveStagnationMeasure, tTolerance);
    EXPECT_STREQ("\n\n****** Optimization stopping due to objective stagnation. ******\n\n", tOutputs.mStopCriterion.c_str());
    std::vector<double> tGoldControl = TopoProxy::get_gold_control_mma_test();
    for(size_t tIndex = 0; tIndex < tGoldControl.size(); tIndex++)
    {
        EXPECT_NEAR(tGoldControl[tIndex], (*tOutputs.mSolution)(0 /* vector index */, tIndex), tTolerance);
    }
}

TEST(PlatoTest, SolveStrucTopoWithTrustRegionAugmentedLagrangian)
{
    // ************** ALLOCATE SIMPLE STRUCTURAL TOPOLOGY OPTIMIZATION SOLVER **************
    const double tPoissonRatio = 0.3;
    const double tElasticModulus = 1;
    const int tNumElementsXdirection = 30;
    const int tNumElementsYdirection = 10;
    std::shared_ptr<Plato::StructuralTopologyOptimization> tPDE =
            std::make_shared <Plato::StructuralTopologyOptimization>(tPoissonRatio, tElasticModulus, tNumElementsXdirection, tNumElementsYdirection);
    tPDE->setFilterRadius(1.2);

    // ************** SET FORCE VECTOR **************
    const int tGlobalNumDofs = tPDE->getGlobalNumDofs();
    Epetra_SerialDenseVector tForce(tGlobalNumDofs);
    const int tDOFsIndex = 1;
    tForce[tDOFsIndex] = -1;
    tPDE->setForceVector(tForce);

    // ************** SET FIXED DEGREES OF FREEDOM (DOFs) VECTOR **************
    std::vector<double> tDofs = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
    Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tDofs.data(), tDofs.size());
    tPDE->setFixedDOFs(tFixedDOFs);

    // ************** ALLOCATE VOLUME AND COMPLIANCE CRITERION **************
    std::shared_ptr<Plato::ProxyVolume<double>> tVolume = std::make_shared<Plato::ProxyVolume<double>>(tPDE);
    std::shared_ptr<Plato::ProxyCompliance<double>> tCompliance = std::make_shared<Plato::ProxyCompliance<double>>(tPDE);
    std::shared_ptr<Plato::CriterionList<double>> tConstraints = std::make_shared<Plato::CriterionList<double>>();
    tConstraints->add(tVolume);

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumVectors = 1;
    Plato::EpetraSerialDenseMultiVector<double> tDual(tNumVectors, tNumDuals);
    tDataFactory->allocateDual(tDual);
    const size_t tNumControls = tPDE->getNumDesignVariables();
    Plato::EpetraSerialDenseMultiVector<double> tControl(tNumVectors, tNumControls);
    tDataFactory->allocateControl(tControl);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<double>> tDataMng =
            std::make_shared<Plato::TrustRegionAlgorithmDataMng<double>>(tDataFactory);

    // ********* SET BOUNDS AND INITIAL GUESS *********
    double tValue = tPDE->getVolumeFraction();
    tDataMng->setInitialGuess(tValue);
    tValue = 1e-2;
    tDataMng->setControlLowerBounds(tValue);
    tValue = 1;
    tDataMng->setControlUpperBounds(tValue);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<double>> tStageMng =
            std::make_shared<Plato::AugmentedLagrangianStageMng<double>>(tDataFactory, tCompliance, tConstraints);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM *********
    Plato::KelleySachsAugmentedLagrangian<double> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    tAlgorithm.solve();

    // ********* TEST OUTPUT DATA *********
    const double tTolerance = 1e-4;
    const double tCurrentObjective = tDataMng->getCurrentObjectiveFunctionValue();
    const double tGoldObjective = 0.1565636381251273;
    EXPECT_NEAR(tCurrentObjective, tGoldObjective, tTolerance);

    const size_t tVectorIndex = 0;
    std::vector<double> tGoldControl = TopoProxy::getGoldControlTrustRegionTest();
    const Plato::Vector<double> & tCurrentControl = tDataMng->getCurrentControl(tVectorIndex);
    for(size_t tIndex = 0; tIndex < tGoldControl.size(); tIndex++)
    {
        EXPECT_NEAR(tCurrentControl[tIndex], tGoldControl[tIndex], tTolerance);
    }
}

TEST(DISABLED_PlatoTest, DISABLED_SolveStrucTopoWithTrustRegionAugmentedLagrangianML)
{
    // ************** ALLOCATE SIMPLE STRUCTURAL TOPOLOGY OPTIMIZATION SOLVER **************
    const double tPoissonRatio = 0.3;
    const double tElasticModulus = 1;
    const int tNumElementsXdirection = 30;
    const int tNumElementsYdirection = 10;
    std::shared_ptr<Plato::StructuralTopologyOptimization> tPDE =
            std::make_shared <Plato::StructuralTopologyOptimization>(tPoissonRatio, tElasticModulus, tNumElementsXdirection, tNumElementsYdirection);
    tPDE->setFilterRadius(1.2);

    // ************** SET FORCE VECTOR **************
    const int tGlobalNumDofs = tPDE->getGlobalNumDofs();
    Epetra_SerialDenseVector tForce(tGlobalNumDofs);
    const int tDOFsIndex = 1;
    tForce[tDOFsIndex] = -1;
    tPDE->setForceVector(tForce);

    // ************** SET FIXED DEGREES OF FREEDOM (DOFs) VECTOR **************
    std::vector<double> tDofs = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
    Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tDofs.data(), tDofs.size());
    tPDE->setFixedDOFs(tFixedDOFs);

    // ************** ALLOCATE VOLUME AND COMPLIANCE CRITERION **************
    std::shared_ptr<Plato::ProxyVolume<double>> tVolume = std::make_shared<Plato::ProxyVolume<double>>(tPDE);
    std::shared_ptr<Plato::ProxyCompliance<double>> tCompliance = std::make_shared<Plato::ProxyCompliance<double>>(tPDE);
    std::shared_ptr<Plato::CriterionList<double>> tConstraints = std::make_shared<Plato::CriterionList<double>>();
    tConstraints->add(tVolume);

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    const size_t tNumDuals = 1;
    const size_t tNumVectors = 1;
    Plato::EpetraSerialDenseMultiVector<double> tDual(tNumVectors, tNumDuals);
    tDataFactory->allocateDual(tDual);
    const size_t tNumControls = tPDE->getNumDesignVariables();
    Plato::EpetraSerialDenseMultiVector<double> tControl(tNumVectors, tNumControls);
    tDataFactory->allocateControl(tControl);

    // ********* ALLOCATE TRUST REGION ALGORITHM DATA MANAGER *********
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<double>> tDataMng =
            std::make_shared<Plato::TrustRegionAlgorithmDataMng<double>>(tDataFactory);

    // ********* SET BOUNDS AND INITIAL GUESS *********
    double tValue = tPDE->getVolumeFraction();
    tDataMng->setInitialGuess(tValue);
    tValue = 1e-2;
    tDataMng->setControlLowerBounds(tValue);
    tValue = 1;
    tDataMng->setControlUpperBounds(tValue);

    // ********* AUGMENTED LAGRANGIAN STAGE MANAGER *********
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<double>> tStageMng =
            std::make_shared<Plato::AugmentedLagrangianStageMng<double>>(tDataFactory, tCompliance, tConstraints);

    // ********* ALLOCATE KELLEY-SACHS ALGORITHM *********
    Plato::MLKelleySachsAugmentedLagrangian<double> tAlgorithm(tDataFactory, tDataMng, tStageMng);
    tAlgorithm.solve();

    // ********* TEST OUTPUT DATA *********
    const double tTolerance = 1e-6;
    const double tCurrentObjective = tDataMng->getCurrentObjectiveFunctionValue();
    const double tGoldObjective = 0.15512930075;
    EXPECT_NEAR(tCurrentObjective, tGoldObjective, tTolerance);

    const size_t tVectorIndex = 0;
    std::vector<double> tGoldControl = TopoProxy::getGoldControlTrustRegionTest();
    const Plato::Vector<double> & tCurrentControl = tDataMng->getCurrentControl(tVectorIndex);
    for(size_t tIndex = 0; tIndex < tGoldControl.size(); tIndex++)
    {
        EXPECT_NEAR(tCurrentControl[tIndex], tGoldControl[tIndex], tTolerance);
    }
}

TEST(PlatoTest, CheckSimpleTopoProxyCriteria)
{
    // ************** ALLOCATE SIMPLE STRUCTURAL TOPOLOGY OPTIMIZATION SOLVER **************
    const double tPoissonRatio = 0.3;
    const double tElasticModulus = 1;
    const int tNumElementsXdirection = 30;
    const int tNumElementsYdirection = 10;
    std::shared_ptr<Plato::StructuralTopologyOptimization> tPDE =
            std::make_shared <Plato::StructuralTopologyOptimization>(tPoissonRatio, tElasticModulus, tNumElementsXdirection, tNumElementsYdirection);
    tPDE->setFilterRadius(1.2);

    // ************** SET FORCE VECTOR **************
    const int tGlobalNumDofs = tPDE->getGlobalNumDofs();
    Epetra_SerialDenseVector tForce(tGlobalNumDofs);
    const int tDOFsIndex = 1;
    tForce[tDOFsIndex] = -1;
    tPDE->setForceVector(tForce);

    // ************** SET FIXED DEGREES OF FREEDOM (DOFs) VECTOR **************
    std::vector<double> tDofs = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 681};
    Epetra_SerialDenseVector tFixedDOFs(Epetra_DataAccess::Copy, tDofs.data(), tDofs.size());
    tPDE->setFixedDOFs(tFixedDOFs);

    // ************** ALLOCATE VOLUME AND COMPLIANCE CRITERION **************
    Plato::ProxyVolume<double> tVolume(tPDE);
    Plato::ProxyCompliance<double> tCompliance(tPDE);

    // ********* ALLOCATE CORE DATA STRUCTURES *********
    const size_t tNumVectors = 1;
    const size_t tNumControls = tPDE->getNumDesignVariables();
    Plato::EpetraSerialDenseMultiVector<double> tControl(tNumVectors, tNumControls);

    // ********* CHECK VOLUME CRITERION *********
    std::ostringstream tOutputMsg;
    Plato::Diagnostics<double> tDiagnostics;
    tDiagnostics.checkCriterionGradient(tVolume, tControl, tOutputMsg);
    EXPECT_TRUE(tDiagnostics.didGradientTestPassed());

    // ********* CHECK COMPLIANCE CRITERION *********
    tCompliance.disableFilter();
    tDiagnostics.checkCriterionGradient(tCompliance, tControl, tOutputMsg);
    EXPECT_TRUE(tDiagnostics.didGradientTestPassed());

    int tMyRank = -1;
    MPI_Comm_rank(MPI_COMM_WORLD, &tMyRank);
    if(tMyRank == static_cast<int>(0))
    {
        std::cout << tOutputMsg.str().c_str();
    }
}

TEST(PlatoTest, CheckRosenbrockCriterion)
{
    // ********* ALLOCATE ROSENBROCK *********
    Plato::Rosenbrock<double> tCriterion;
    const size_t tNumVectors = 1;
    const size_t tNumControls = 2;
    Plato::EpetraSerialDenseMultiVector<double> tControl(tNumVectors, tNumControls);

    // ********* CHECK ROSENBROCK CRITERION *********
    std::ostringstream tOutputMsg;
    Plato::Diagnostics<double> tDiagnostics;
    tDiagnostics.checkCriterionGradient(tCriterion, tControl, tOutputMsg);
    EXPECT_TRUE(tDiagnostics.didGradientTestPassed());
    tDiagnostics.checkCriterionHessian(tCriterion, tControl, tOutputMsg);
    EXPECT_TRUE(tDiagnostics.didHessianTestPassed());

    int tMyRank = -1;
    MPI_Comm_rank(MPI_COMM_WORLD, &tMyRank);
    if(tMyRank == static_cast<int>(0))
    {
        std::cout << tOutputMsg.str().c_str();
    }
}

} // namespace PlatoTest
