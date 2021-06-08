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
            mConstraintNormalizationMultipliers(),
            mCommWrapper(),
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

    bool mPrintMMADiagnostics = false; /*!< flag to enable problem statistics output (default=false) */
    bool mPrintAugLagSubProbDiagnostics = false; /*!< output augmented Lagrangian subproblem diagnostics to text file (default=false) */
    std::string mOutputStageName = ""; /*!< output stage name */

    OrdinalType mUpdateFrequency = 0; /*!< continuation frequency (default = disabled = 0) */
    OrdinalType mMaxNumSolverIter = 500; /*!< maximum number of outer iterations */
    OrdinalType mNumControlVectors = 1; /*!< number of control vectors (default = 1) */
    OrdinalType mMaxNumSubProblemIter = 50; /*!< maximum number of trust region sub problem iterations */
    OrdinalType mMaxNumTrustRegionIter = 50; /*!< maximum number of trust region iterations */

    ScalarType mMoveLimit = 0.5; /*!< move limit */
    ScalarType mAsymptoteExpansion = 1.2; /*!< moving asymptotes expansion factor */
    ScalarType mAsymptoteContraction = 0.7; /*!< moving asymptotes' contraction factor */
    ScalarType mInitialAugLagPenalty = 0.0015; /*!< initial penalty value used in augmented Lagrangian function, \f$ \mu = 1/\alpha \f$, where \f$ \alpha \f$ is the initial penalty value */
    ScalarType mInitialAymptoteScaling = 0.5; /*!< initial moving asymptotes' scale factor */
    ScalarType mSubProblemBoundsScaling = 0.1; /*!< scaling on subproblem upper and lower bounds */
    ScalarType mAugLagSubProbPenaltyMultiplier = 1.025; /*!< augmented Lagrangian subproblem penalty multiplier, \f$ \alpha_{i+1} = \beta*\alpha_{i}, where \f$ \beta \f$ is the multiplier */

    ScalarType mOptimalityTolerance = 1e-6; /*!< optimality tolerance */
    ScalarType mFeasibilityTolerance = 1e-4; /*!< feasibility tolerance */
    ScalarType mControlStagnationTolerance = 1e-6; /*!< control stagnation tolerance */
    ScalarType mObjectiveStagnationTolerance = 1e-8; /*!< objective function stagnation tolerance */
    ScalarType mAugLagSubProbFeasibilityTolerance = 1e-8; /*!< augmented Lagrangian algorithm feasibility tolerance */

    std::vector<ScalarType> mConstraintNormalizationMultipliers; /*!< constraint normalization multipliers */

    Plato::CommWrapper mCommWrapper; /*!< distributed memory communication wrapper */
    Plato::MemorySpace::type_t mMemorySpace = Plato::MemorySpace::HOST; /*!< memory space: HOST (default) OR DEVICE */

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
