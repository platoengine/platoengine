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
 * Plato_GloballyConvergentMethodMovingAsymptotesLightInterface.hpp
 *
 *  Created on: Oct 3, 2018
 */

#pragma once

#include "Plato_DataFactory.hpp"
#include "Plato_PrimalProblemStageMng.hpp"
#include "Plato_ConservativeConvexSeparableAppxDataMng.hpp"
#include "Plato_GloballyConvergentMethodMovingAsymptotes.hpp"
#include "Plato_ConservativeConvexSeparableAppxAlgorithm.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Output data structure for the Globally Convergent Method of Moving Asymptotes (GCMMA) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmOutputsGCMMA
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
// struct AlgorithmOutputsGCMMA

/******************************************************************************//**
 * @brief Input data structure for the Globally Convergent Method of Moving Asymptotes (GCMMA) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmInputsGCMMA
{
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    AlgorithmInputsGCMMA() :
            mPrintDiagnostics(false),
            mMaxNumOuterIter(500),
            mMaxNumInnerIter(5),
            mMovingAsymptoteExpansionFactor(1.2),
            mMovingAsymptoteContractionFactor(0.4),
            mInitialMovingAsymptoteScaleFactor(0.5),
            mMovingAsymptoteUpperBoundScaleFactor(10),
            mMovingAsymptoteLowerBoundScaleFactor(0.01),
            mOuterStationarityTolerance(1e-4),
            mOuterControlStagnationTolerance(1e-8),
            mOuterObjectiveStagnationTolerance(1e-6),
            mOuterKarushKuhnTuckerConditionsTolerance(1e-5),
            mInnerControlStagnationTolerance(1e-8),
            mInnerObjectiveStagnationTolerance(1e-6),
            mInnerKarushKuhnTuckerConditionsTolerance(5e-4),
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
    virtual ~AlgorithmInputsGCMMA()
    {
    }

    bool mPrintDiagnostics; /*!< flag to enable problem statistics output (default=false) */

    OrdinalType mMaxNumOuterIter; /*!< maximum number of outer iterations */
    OrdinalType mMaxNumInnerIter; /*!< maximum number of inner iterations */

    ScalarType mMovingAsymptoteExpansionFactor; /*!< moving asymptotes expansion factor */
    ScalarType mMovingAsymptoteContractionFactor; /*!< moving asymptotes' contraction factor */
    ScalarType mInitialMovingAsymptoteScaleFactor; /*!< initial moving asymptotes' scale factor */
    ScalarType mMovingAsymptoteUpperBoundScaleFactor; /*!< scale factor for upper bound on moving asymptotes */
    ScalarType mMovingAsymptoteLowerBoundScaleFactor; /*!< scale factor for lower bound on moving asymptotes */

    ScalarType mOuterStationarityTolerance; /*!< outer stationarity tolerance */
    ScalarType mOuterControlStagnationTolerance; /*!< outer control stagnation tolerance */
    ScalarType mOuterObjectiveStagnationTolerance; /*!< outer objective function stagnation tolerance */
    ScalarType mOuterKarushKuhnTuckerConditionsTolerance; /*!< outer Karush-Kuhn-Tucker (KKT) inexactness tolerance */

    ScalarType mInnerControlStagnationTolerance; /*!< inner control stagnation tolerance */
    ScalarType mInnerObjectiveStagnationTolerance; /*!< inner objective function stagnation tolerance */
    ScalarType mInnerKarushKuhnTuckerConditionsTolerance; /*!< inner Karush-Kuhn-Tucker (KKT) inexactness tolerance */

    Plato::CommWrapper mCommWrapper; /*!< distributed memory communication wrapper */
    Plato::MemorySpace::type_t mMemorySpace; /*!< memory space: HOST (default) OR DEVICE */

    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mDual; /*!< Lagrange multipliers */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mLowerBounds; /*!< lower bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mUpperBounds; /*!< upper bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mInitialGuess; /*!< initial guess */

    /*!< operations which require communication across processors, e.g. max, min, global sum */
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mReductionOperations;
};
// struct AlgorithmInputsGCMMA

/******************************************************************************//**
 * @brief Set Globally Convergent Method of Moving Asymptotes (GCMMA) algorithm inputs
 * @param [in] aInputs Globally Convergent Method of Moving Asymptotes input parameter structure
 * @param [in,out] aSubProblem Globally Convergent Method of Moving Asymptotes subproblem interface
 * @param [in,out] aAlgorithm main algorithm interface
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_gcmma_algorithm_inputs(const Plato::AlgorithmInputsGCMMA<ScalarType, OrdinalType> & aInputs,
                                       Plato::GloballyConvergentMethodMovingAsymptotes<ScalarType, OrdinalType> & aSubProblem,
                                       Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> & aAlgorithm)
{
    if(aInputs.mPrintDiagnostics == true)
    {
        aAlgorithm.enableDiagnostics();
    }

    aSubProblem.setMaxNumIterations(aInputs.mMaxNumInnerIter);
    aSubProblem.setControlStagnationTolerance(aInputs.mInnerControlStagnationTolerance);
    aSubProblem.setObjectiveStagnationTolerance(aInputs.mInnerObjectiveStagnationTolerance);
    aSubProblem.setKarushKuhnTuckerConditionsTolerance(aInputs.mInnerKarushKuhnTuckerConditionsTolerance);

    aAlgorithm.setMovingAsymptoteExpansionFactor(aInputs.mMovingAsymptoteExpansionFactor);
    aAlgorithm.setMovingAsymptoteContractionFactor(aInputs.mMovingAsymptoteContractionFactor);
    aAlgorithm.setInitialMovingAsymptoteScaleFactor(aInputs.mInitialMovingAsymptoteScaleFactor);
    aAlgorithm.setMovingAsymptoteUpperBoundScaleFactor(aInputs.mMovingAsymptoteUpperBoundScaleFactor);
    aAlgorithm.setMovingAsymptoteLowerBoundScaleFactor(aInputs.mMovingAsymptoteLowerBoundScaleFactor);

    aAlgorithm.setMaxNumIterations(aInputs.mMaxNumOuterIter);
    aAlgorithm.setStationarityTolerance(aInputs.mOuterStationarityTolerance);
    aAlgorithm.setControlStagnationTolerance(aInputs.mOuterControlStagnationTolerance);
    aAlgorithm.setObjectiveStagnationTolerance(aInputs.mOuterObjectiveStagnationTolerance);
    aAlgorithm.setKarushKuhnTuckerConditionsTolerance(aInputs.mOuterKarushKuhnTuckerConditionsTolerance);
}
// function set_gcmma_algorithm_inputs

/******************************************************************************//**
 * @brief Set Globally Convergent Method of Moving Asymptotes (GCMMA) output structure
 * @param [in] aAlgorithm Main interface to Globally Convergent Method of Moving Asymptotes algorithm
 * @param [in,out] aOutputs Globally Convergent Method of Moving Asymptotes algorithm output structure
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void set_gcmma_algorithm_outputs(const Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> & aAlgorithm,
                                        Plato::AlgorithmOutputsGCMMA<ScalarType, OrdinalType> & aOutputs)
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
// function set_gcmma_algorithm_outputs

/******************************************************************************//**
 * @brief Main interface to Globally Convergent Method of Moving Asymptotes (GCMMA) algorithm
 * @param [in] aObjective user-defined objective function
 * @param [in] aConstraints user-defined list of constraints
 * @param [in] aInputs Globally Convergent Method of Moving Asymptotes input structure
 * @param [in,out] aOutputs Globally Convergent Method of Moving Asymptotes output structure
 **********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
inline void solve_gcmma(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aObjective,
                        const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aConstraints,
                        const Plato::AlgorithmInputsGCMMA<ScalarType, OrdinalType> & aInputs,
                        Plato::AlgorithmOutputsGCMMA<ScalarType, OrdinalType> & aOutputs)
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
    std::shared_ptr<Plato::GloballyConvergentMethodMovingAsymptotes<ScalarType, OrdinalType>> tSubProblem;
    tSubProblem = std::make_shared<Plato::GloballyConvergentMethodMovingAsymptotes<ScalarType, OrdinalType>>(tDataFactory);
    Plato::ConservativeConvexSeparableAppxAlgorithm<ScalarType, OrdinalType> tAlgorithm(tStageMng, tDataMng, tSubProblem);
    Plato::set_gcmma_algorithm_inputs(aInputs, *tSubProblem, tAlgorithm);
    tAlgorithm.solve();
    Plato::set_gcmma_algorithm_outputs(tAlgorithm, aOutputs);
}
// function solve_gcmma

} // namespace Plato
