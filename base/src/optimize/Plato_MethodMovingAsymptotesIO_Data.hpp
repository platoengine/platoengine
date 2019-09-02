/*
 * Plato_MethodMovingAsymptotesIO_Data.hpp
 *
 *  Created on: Sep 1, 2019
 */

#pragma once

#include <vector>
#include <string>
#include <memory>

#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_CommWrapper.hpp"
#include "Plato_StandardVectorReductionOperations.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Diagnostic data for the Method of Moving Asymptotes (MMA) algorithm.
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct OutputDataMMA
{
    OrdinalType mNumIter;  /*!< number of outer iterations */
    OrdinalType mObjFuncCount;  /*!< number of objective function evaluations */

    ScalarType mObjFuncValue;  /*!< objective function value */
    ScalarType mNormObjFuncGrad;  /*!< norm of the objective function gradient */
    ScalarType mControlStagnationMeasure;  /*!< norm of the difference between two subsequent control fields */
    ScalarType mObjectiveStagnationMeasure;  /*!< measures stagnation in two subsequent objective function evaluations */
    std::vector<ScalarType> mConstraints;  /*!< residual value for each constraint */
};
// struct OutputDataMMA

/******************************************************************************//**
 * @brief Output data structure for the Method of Moving Asymptotes (MMA) algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct AlgorithmOutputsMMA
{
    OrdinalType mNumSolverIter; /*!< number of outer iterations */
    OrdinalType mNumObjFuncEval; /*!< number of objective function evaluations */

    ScalarType mObjFuncValue; /*!< objective function value */
    ScalarType mNormObjFuncGrad; /*!< norm of the objective function gradient */
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
            mOutputStageName(),
            mUpdateFrequency(0),
            mMaxNumSolverIter(500),
            mNumControlVectors(1),
            mMaxNumSubProblemIter(100),
            mMaxNumTrustRegionIter(50),
            mMoveLimit(0.5),
            mAsymptoteExpansion(1.2),
            mAsymptoteContraction(0.7),
            mInitialAugLagPenalty(1.0),
            mInitialAymptoteScaling(0.5),
            mSubProblemBoundsScaling(0.1),
            mOptimalityTolerance(1e-6),
            mFeasibilityTolerance(1e-4),
            mControlStagnationTolerance(1e-6),
            mObjectiveStagnationTolerance(1e-8),
            mConstraintNormalizationMultipliers(),
            mCommWrapper(),
            mMemorySpace(Plato::MemorySpace::HOST),
            mLowerBounds(nullptr),
            mUpperBounds(nullptr),
            mInitialGuess(nullptr),
            mConstraintNormalizationParams(nullptr),
            mDualReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>()),
            mControlReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>())
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
    std::string mOutputStageName; /*!< output stage name */

    OrdinalType mUpdateFrequency; /*!< continuation frequency (default = disabled = 0) */
    OrdinalType mMaxNumSolverIter; /*!< maximum number of outer iterations */
    OrdinalType mNumControlVectors; /*!< number of control vectors (default = 1) */
    OrdinalType mMaxNumSubProblemIter; /*!< maximum number of trust region sub problem iterations */
    OrdinalType mMaxNumTrustRegionIter; /*!< maximum number of trust region iterations */

    ScalarType mMoveLimit; /*!< move limit */
    ScalarType mAsymptoteExpansion; /*!< moving asymptotes expansion factor */
    ScalarType mAsymptoteContraction; /*!< moving asymptotes' contraction factor */
    ScalarType mInitialAugLagPenalty; /*!< initial penalty on augmented Lagrangian function */
    ScalarType mInitialAymptoteScaling; /*!< initial moving asymptotes' scale factor */
    ScalarType mSubProblemBoundsScaling; /*!< scaling on subproblem upper and lower bounds */

    ScalarType mOptimalityTolerance; /*!< optimality tolerance */
    ScalarType mFeasibilityTolerance; /*!< feasibility tolerance */
    ScalarType mControlStagnationTolerance; /*!< control stagnation tolerance */
    ScalarType mObjectiveStagnationTolerance; /*!< objective function stagnation tolerance */

    std::vector<ScalarType> mConstraintNormalizationMultipliers; /*!< constraint normalization multipliers */

    Plato::CommWrapper mCommWrapper; /*!< distributed memory communication wrapper */
    Plato::MemorySpace::type_t mMemorySpace; /*!< memory space: HOST (default) OR DEVICE */

    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mLowerBounds; /*!< lower bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mUpperBounds; /*!< upper bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType,OrdinalType>> mInitialGuess; /*!< initial guess */
    std::shared_ptr<Plato::Vector<ScalarType,OrdinalType>> mConstraintNormalizationParams; /*!< constraint normalization parameters */

    /*!< operations which require communication across processors, e.g. max, min, global sum */
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mDualReductionOperations;
    std::shared_ptr<Plato::ReductionOperations<ScalarType,OrdinalType>> mControlReductionOperations;
};
// struct AlgorithmInputsMMA

}
// namespace Plato
