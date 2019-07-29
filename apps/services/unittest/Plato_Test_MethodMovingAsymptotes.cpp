/*
 * Plato_Test_MethodMovingAsymptotes.cpp
 *
 *  Created on: Jul 21, 2019
 */

#include "gtest/gtest.h"

#include "Plato_UnitTestUtils.hpp"

#include "Plato_DataFactory.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_MultiVectorList.hpp"
#include "Plato_AugmentedLagrangian.hpp"

namespace Plato
{

namespace experimental
{

/******************************************************************************//**
 * Criterion interface for the approximation function used to solve the Method of
 * Moving Asymptotes (MMA) subproblem.
 *
 * The approximation function for the MMA subproblem is defined as
 * /f$\sum_{j=1}^{N}\left( \frac{p_{ij}^{k}}{u_j^k - x_j} + \frac{q_{ij}^{k}}{x_j - l_j}
 * + r_{i} \right)/f$
 *
 * where
 *
 * /f$r_i = f_i(\mathbf{x}^k) - \sum_{j=1}^{N}\left( \frac{p_{ij}^{k}}{u_j^k - x_j^k}
 * + \frac{q_{ij}^{k}}{x_j^k - l_j} \right)/f$
 *
 * /f$p_{ij} = (x_j^k - l_j)^2\left( \alpha_{1}\left(\frac{\partial{f}_i}{\partial{x}_j}
 * (\mathbf{x}^k)\right)^{+} + \alpha_{2}\left(\frac{\partial{f}_i}{\partial{x}_j}
 * (\mathbf{x}^k)\right)^{-} + \frac{\epsilon}{x_j^{\max} - x_j^{\min}}\right)/f$
 *
 * /f$q_{ij} = (x_j^k - l_j)^2\left( \alpha_{2}\left(\frac{\partial{f}_i}{\partial{x}_j}
 * (\mathbf{x}^k)\right)^{+} + \alpha_{1}\left(\frac{\partial{f}_i}{\partial{x}_j}
 * (\mathbf{x}^k)\right)^{-} + \frac{\epsilon}{x_j^{\max} - x_j^{\min}}\right)/f$
 *
 * /f$\frac{\partial{f}_i}{\partial{x}_j}(\mathbf{x}^k)\right)^{+} = \max\left(
 * \frac{\partial{f}_i}{\partial{x}_j}, 0 \right)/f$
 *
 * /f$\frac{\partial{f}_i}{\partial{x}_j}(\mathbf{x}^k)\right)^{-} = \max\left(
 * -\frac{\partial{f}_i}{\partial{x}_j}, 0 \right)/f$
 *
 * Nomenclature:
 *
 * /f$x_j/f$: trial control j-th value
 * /f$x_j^{\max}/f$: j-th value for control upper bound
 * /f$x_j^{\min}/f$: j-th value for control lower bound
 * /f$u_j/f$: upper asymptote j-th value
 * /f$l_j/f$: lower asymptote j-th value
 * /f$\epsilon/f$: positive coefficient
 * /f$\alpha_{1}, \alpha_{2}/f$: positive coefficients
 * /f$\mathbf{x}^k/f$: current controls
 * /f$f_i(\mathbf{x}^k)/f$: current criterion value
 * /f$\frac{\partial{f}_i}{\partial{x}_j}(\mathbf{x}^k)/f$: current criterion gradient
 *
***********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesCriterion : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    explicit MethodMovingAsymptotesCriterion(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mInitialAymptoteScale(0.5),
        mCurrentNormalizedCriterionValue(0)
    {
    }

    ~MethodMovingAsymptotesCriterion()
    {
    }

    /******************************************************************************//**
     * Safely cache application specific data after a new trial control is accepted.
     ***********************************************************************************/
    void cacheData()
    {
        return;
    }

    /******************************************************************************//**
     * Evaluate approximation function.
     * @param [in] aControl: control, i.e. design, variables
     * @return approximation function value
     ***********************************************************************************/
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> &aControl)
    {
        const ScalarType tOutput = this->evaluateApproximationFunction(aControl);
        return (tOutput);
    }

    /******************************************************************************//**
     * Compute approximation function gradient.
     * @param [in] aControl: control, i.e. design, variables
     * @param [in/out] aOutput: approximation function gradient
     ***********************************************************************************/
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> &aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> &aOutput)
    {
        this->computeApproximationFunctionGradient(aControl, aOutput);
    }

    /******************************************************************************//**
     * Apply vector to approximation function Hessian.
     * @param [in] aControl: control, i.e. design, variables
     * @param [in] aVector: descent direction
     * @param [in/out] aOutput: approximation function gradient
     ***********************************************************************************/
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> &aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> &aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> &aOutput)
    {
        this->computeApproximationFunctionHessTimesVec(aControl, aVector, aOutput);
    }

private:
    void updateAsymptotes(const Plato::StateData<ScalarType, OrdinalType> &aData)
    {
        const OrdinalType tCurrentOptimizationIteration = aData.getCurrentOptimizationIteration();
        if (tCurrentOptimizationIteration >= static_cast<OrdinalType>(2))
        {
            this->updateCurrentAsymptotes();
        }
        else
        {
            this->updateInitialAsymptotes(aData);
        }
    }

    void updateInitialAsymptotes(const Plato::StateData<ScalarType, OrdinalType> &aData)
    {
        // TODO: I NEED TO SUBTRACT LOWER BOUNDS FROM UPPER BOUNDS
        Plato::update(static_cast<ScalarType>(1), aData.getCurrentControl(), static_cast<ScalarType>(0), *mLowerAsymptotes);
        Plato::update(-mInitialAymptoteScale, *mUpperMinusLowerBounds, static_cast<ScalarType>(1), *mLowerAsymptotes);

        Plato::update(static_cast<ScalarType>(1), aData.getCurrentControl(), static_cast<ScalarType>(0), *mUpperAsymptotes);
        Plato::update(mInitialAymptoteScale, *mUpperMinusLowerBounds, static_cast<ScalarType>(1), *mUpperAsymptotes);
    }

    ScalarType evaluateApproximationFunction(const Plato::MultiVector<ScalarType, OrdinalType> &aControls)
    {
        mControlWork1->fill(static_cast<ScalarType>(0));
        mControlWork2->fill(static_cast<ScalarType>(0));

        const OrdinalType tNumVectors = aControls.getNumVectors();
        for(OrdinalType tVecIndex = 0; tVecIndex < tNumVectors; tVecIndex++)
        {
            const OrdinalType tNumControls = aControls[tVecIndex].size();
            for(OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                ScalarType tValueOne = (*mAppxFunctionP)(tVecIndex, tControlIndex)
                    / ((*mUpperAsymptotes)(tVecIndex, tControlIndex) - aControls(tVecIndex, tControlIndex));
                ScalarType tValueTwo = (*mAppxFunctionQ)(tVecIndex, tControlIndex)
                    / (aControls(tVecIndex, tControlIndex) - (*mLowerAsymptotes)(tVecIndex, tControlIndex));
                (*mControlWork1)[tControlIndex] += tValueOne + tValueTwo;

                tValueOne = (*mAppxFunctionP)(tVecIndex, tControlIndex)
                    / ((*mUpperAsymptotes)(tVecIndex, tControlIndex) - (*mCurrentControls)(tVecIndex, tControlIndex));
                tValueTwo = (*mAppxFunctionQ)(tVecIndex, tControlIndex)
                    / ((*mCurrentControls)(tVecIndex, tControlIndex) - (*mLowerAsymptotes)(tVecIndex, tControlIndex));
                (*mControlWork2)[tControlIndex] += tValueOne + tValueTwo;
            }
        }

        const ScalarType tTermOne = mReductionOperations->sum(*mControlWork1);
        const ScalarType tTermTwo = mReductionOperations->sum(*mControlWork2);
        const ScalarType tOutput = tTermOne + mCurrentNormalizedCriterionValue - tTermTwo;

        return (tOutput);
    }

    void computeApproximationFunctionGradient(const Plato::MultiVector<ScalarType, OrdinalType> &aControls,
                                              Plato::MultiVector<ScalarType, OrdinalType> &aGradient)
    {
        const OrdinalType tNumVectors = aControls.getNumVectors();
        for (OrdinalType tVecIndex = 0; tVecIndex < tNumVectors; tVecIndex++)
        {
            const OrdinalType tNumControls = aControls[tVecIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                ScalarType tDenominatorValue = (*mUpperAsymptotes)(tVecIndex, tControlIndex) - aControls(tVecIndex, tControlIndex);
                ScalarType tDenominator = tDenominatorValue * tDenominatorValue
                ScalarType tValueOne = (*mAppxFunctionP)(tVecIndex, tControlIndex) / tDenominator;
                ScalarType tValueTwo = (*mAppxFunctionQ)(tVecIndex, tControlIndex) / tDenominator;
                aGradient(tVecIndex, tControlIndex) = tValueOne + tValueTwo;
            }
        }
    }

    void computeApproximationFunctionHessTimesVec(const Plato::MultiVector<ScalarType, OrdinalType> &aControls,
                                                  const Plato::MultiVector<ScalarType, OrdinalType> &aVector,
                                                  Plato::MultiVector<ScalarType, OrdinalType> &aHessTimesVec)
    {
        const OrdinalType tNumVectors = aControls.getNumVectors();
        for (OrdinalType tVecIndex = 0; tVecIndex < tNumVectors; tVecIndex++)
        {
            const OrdinalType tNumControls = aControls[tVecIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                ScalarType tDenominatorValue = (*mUpperAsymptotes)(tVecIndex, tControlIndex) - aControls(tVecIndex, tControlIndex);
                ScalarType tDenominator = tDenominatorValue * tDenominatorValue * tDenominatorValue;
                ScalarType tValueOne = (*mAppxFunctionP)(tVecIndex, tControlIndex) / tDenominator;
                ScalarType tValueTwo = (*mAppxFunctionQ)(tVecIndex, tControlIndex) / tDenominator;
                aHessTimesVec(tVecIndex, tControlIndex) = static_cast<ScalarType>(2) * (tValueOne + tValueTwo) * aVector(tVecIndex, tControlIndex);
            }
        }
    }

private:
    ScalarType mInitialAymptoteScale;
    ScalarType mCurrentNormalizedCriterionValue;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWork1;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWork2;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionP;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionQ;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControls;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerAsymptotes;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperAsymptotes;

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mReductionOperations;
};

template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotes
{
public:
    MethodMovingAsymptotes(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> &aObjective,
                           const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> &aConstraints,
                           const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
            mMoveLimit(0.5),
            mApproxFunctionEpsilon(1e-5),
            mAsymptoteExpansion(1.2),
            mAsymptoteContraction(0.7),
            mInitialAymptoteScale(0.5),
            mSubProblemBoundsScaling(0.1),
            mApproxFunctionScalingOne(1.001),
            mApproxFunctionScalingTwo(0.001),
            mCurrentObjectiveValue(std::numeric_limits<ScalarType>::max()),
            mPreviousObjectiveValue(std::numeric_limits<ScalarType>::max()),
            mObjective(aObjective),
            mConstraints(aConstraints)
    {
    }

    void solve()
    {
        // initialize static quantities
        Plato::update(static_cast<ScalarType>(1), *mControlUpperBounds, static_cast<ScalarType>(0), *mUpperMinusLowerBounds);
        Plato::update(static_cast<ScalarType>(-1), *mControlLowerBounds, static_cast<ScalarType>(1), *mUpperMinusLowerBounds);

        // Compute initial objective function values and gradients
        mCurrentObjectiveValue = mObjective->value(*mCurrentControls);
        mObjective->gradient(*mCurrentControls, *mCurrentObjectiveGradient);

        // Compute initial constraint values and gradients
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            (*mCurrentConstraintValues)[tConstraintIndex] = (*mConstraints)[tConstraintIndex].value(*mCurrentControls)
                    - (*mConstraintLimits)[tConstraintIndex];
            (*mConstraints)[tConstraintIndex].gradient(*mCurrentControls, (*mCurrentConstraintGradients)[tConstraintIndex]);
        }

        // update asymptotes
        this->updateAsymptotes();

        // update sub-problem bounds
        this->updateSubProblemBounds();

        // Compute P and Q coefficients for the objective approximation function
        this->computeApproximationFunctionCoefficients(mCurrentObjectiveValue,
                                                       *mCurrentObjectiveGradient,
                                                       *mObjectivePappxFunctions,
                                                       *mObjectiveQappxFunctions);

        // Compute P and Q coefficients for the constraint approximation functions
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            this->computeApproximationFunctionCoefficients((*mConstraintLimits)[tConstraintIndex],
                                                           (*mCurrentConstraintGradients)[tConstraintIndex],
                                                           (*mConstraintPapproxFunction)[tConstraintIndex],
                                                           (*mObjectiveQappxFunctions)[tConstraintIndex]);
        }
    }

    void updateAsymptotes()
    {
        if(mIterationCount >= static_cast<OrdinalType>(2))
        {
            this->updateCurrentAsymptotes();
        }
        else
        {
            this->updateInitialAsymptotes();
        }
    }

    void updateCurrentAsymptotes()
    {
        const OrdinalType tNumVectors = mCurrentControls->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = (*mCurrentControls)[tVectorIndex].size();
            for(OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                const ScalarType tMeasure = ( (*mCurrentControls)(tVectorIndex, tControlIndex) * (*mPreviousControls)(tVectorIndex, tControlIndex) )
                        * ( (*mPreviousControls)(tVectorIndex, tControlIndex) * (*mAntepenultimateControls)(tVectorIndex, tControlIndex) )
                ScalarType tGammaValue = tMeasure > static_cast<ScalarType>(0) ? mAsymptoteExpansion : mAsymptoteContraction;
                tGammaValue = std::abs(tGammaValue) <= std::numeric_limits<ScalarType>::min() ? static_cast<ScalarType>(1) : tGammaValue;

                (*mLowerAsymptotes)(tVectorIndex, tControlIndex) = (*mCurrentControls)(tVectorIndex, tControlIndex)
                        - ( tGammaValue * ( (*mPreviousControls)(tVectorIndex, tControlIndex) - (*mLowerAsymptotes)(tVectorIndex, tControlIndex) ) );
                (*mUpperAsymptotes)(tVectorIndex, tControlIndex) = (*mCurrentControls)(tVectorIndex, tControlIndex)
                        + ( tGammaValue * ( (*mUpperAsymptotes)(tVectorIndex, tControlIndex) - (*mPreviousControls)(tVectorIndex, tControlIndex) ) );
            }
        }
    }

    void updateInitialAsymptotes()
    {
        Plato::update(static_cast<ScalarType>(1), *mCurrentControls, static_cast<ScalarType>(0), *mLowerAsymptotes);
        Plato::update(-mInitialAymptoteScale, *mUpperMinusLowerBounds, static_cast<ScalarType>(1), *mLowerAsymptotes);

        Plato::update(static_cast<ScalarType>(1), *mCurrentControls, static_cast<ScalarType>(0), *mUpperAsymptotes);
        Plato::update(mInitialAymptoteScale, *mUpperMinusLowerBounds, static_cast<ScalarType>(1), *mUpperAsymptotes);
    }

    void updateSubProblemBounds()
    {
        std::vector<ScalarType> tCriteria(3/* length */);
        const OrdinalType tNumVectors = mCurrentControls->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = (*mCurrentControls)[tVectorIndex].size();
            for(OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                tCriteria[0] = (*mControlLowerBounds)(tVectorIndex, tControlIndex);
                tCriteria[1] = (*mLowerAsymptotes)(tVectorIndex, tControlIndex)
                        + (mSubProblemBoundsScaling * ((*mCurrentControls)(tVectorIndex, tControlIndex)
                                - (*mLowerAsymptotes)(tVectorIndex, tControlIndex)));
                tCriteria[2] = (*mCurrentControls)(tVectorIndex, tControlIndex)
                        - (mMoveLimit * (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                (*mSubProblemLowerBounds)(tVectorIndex, tControlIndex) = *std::max_element(tCriteria.begin(),tCriteria.end());

                tCriteria[0] = (*mControlUpperBounds)(tVectorIndex, tControlIndex);
                tCriteria[1] = (*mUpperAsymptotes)(tVectorIndex, tControlIndex)
                        - (mSubProblemBoundsScaling * ((*mUpperAsymptotes)(tVectorIndex, tControlIndex)
                                - (*mCurrentControls)(tVectorIndex, tControlIndex)));
                tCriteria[2] = (*mCurrentControls)(tVectorIndex, tControlIndex)
                        + (mMoveLimit * (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                (*mSubProblemUpperBounds)(tVectorIndex, tControlIndex) = *std::min_element(tCriteria.begin(),tCriteria.end());
            }
        }
    }

    void computeApproximationFunctionCoefficients(const ScalarType& aNormalization,
                                                  const Plato::MultiVector<ScalarType, OrdinalType>& aCriterionGrad,
                                                  Plato::MultiVector<ScalarType, OrdinalType>& aPappxFunction,
                                                  Plato::MultiVector<ScalarType, OrdinalType>& aQappxFunction)
    {
        const OrdinalType tAbsNormalization = std::abs(aNormalization);
        const OrdinalType tNumVectors = aCriterionGrad.getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = aCriterionGrad[tVectorIndex].size();
            for(OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                const ScalarType tNormalizedGradValue = aCriterionGrad(tVectorIndex, tControlIndex) / tAbsNormalization;
                const ScalarType tGradValuePlus = std::max(tNormalizedGradValue, static_cast<ScalarType>(0));
                const ScalarType tGradValueMinus = std::max(-tNormalizedGradValue, static_cast<ScalarType>(0));

                aPappxFunction(tVectorIndex, tControlIndex) = (mApproxFunctionScalingOne * tGradValuePlus)
                        + (mApproxFunctionScalingTwo * tGradValueMinus)
                        + (mApproxFunctionEpsilon / (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                const ScalarType tUpperAsymmMinusCurrentControlSquared = (*mUpperAsymptotes)(tVectorIndex, tControlIndex)
                        - (*mCurrentControls)(tVectorIndex, tControlIndex);
                tUpperAsymmMinusCurrentControlSquared *= tUpperAsymmMinusCurrentControlSquared;
                aPappxFunction(tVectorIndex, tControlIndex) *= tUpperAsymmMinusCurrentControlSquared;

                aQappxFunction(tVectorIndex, tControlIndex) = (mApproxFunctionScalingTwo * tGradValuePlus)
                        + (mApproxFunctionScalingOne * tGradValueMinus)
                        + (mApproxFunctionEpsilon / (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                const ScalarType tCurrentControlMinusLowerAsymmSquared = (*mCurrentControls)(tVectorIndex, tControlIndex)
                        - (*mLowerAsymptotes)(tVectorIndex, tControlIndex);
                tCurrentControlMinusLowerAsymmSquared *= tCurrentControlMinusLowerAsymmSquared;
                aQappxFunction(tVectorIndex, tControlIndex) *= tCurrentControlMinusLowerAsymmSquared;
            }
        }
    }

private:
    OrdinalType mIterationCount;
    OrdinalType mMaxNumIterations;

    ScalarType mMoveLimit;
    ScalarType mAsymptoteExpansion;
    ScalarType mAsymptoteContraction;
    ScalarType mInitialAymptoteScale;
    ScalarType mApproxFunctionEpsilon;
    ScalarType mSubProblemBoundsScaling;
    ScalarType mApproxFunctionScalingOne;
    ScalarType mApproxFunctionScalingTwo;

    ScalarType mCurrentObjectiveValue;
    ScalarType mPreviousObjectiveValue;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWork1;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWork2;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mConstraintLimits;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentConstraintValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mPreviousConstraintValues;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentObjectiveGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousObjectiveGradient;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mCurrentConstraintGradients;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mPreviousConstraintGradients;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControls;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousControls;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAntepenultimateControls;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerAsymptotes;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperAsymptotes;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAsymptotesScaling;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperAsymptotesMinusCurrentControls;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControlsMinusLowerAsymptotes;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlLowerBounds;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlUpperBounds;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperMinusLowerBounds;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mSubProblemLowerBounds;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mSubProblemUpperBounds;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mObjectivePappxFunctions;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mObjectiveQappxFunctions;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mConstraintPapproxFunction;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mConstraintQapproxFunction;

    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mObjective;
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> mConstraints;
    std::shared_ptr<Plato::AugmentedLagrangian<ScalarType, OrdinalType>> mOptimizer;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mReductionOperations;
};
// class MethodMovingAsymptotes

}
// namespace experimental

}
// namespace Plato
