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
};
// class MethodMovingAsymptotes

}
// namespace experimental

}
// namespace Plato
