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

template<typename ScalarType, typename OrdinalType = size_t>
struct ApproximationFunctionData
{
    ScalarType mCurrentNormalizedCriterionValue;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionP;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionQ;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControls;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerAsymptotes;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperAsymptotes;
};

template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesDataMng
{
public:
    explicit MethodMovingAsymptotesDataMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mCurrentObjectiveValue(std::numeric_limits<ScalarType>::max()),
        mPreviousObjectiveValue(std::numeric_limits<ScalarType>::max())
    {
    }

    ~MethodMovingAsymptotesDataMng()
    {
    }

    OrdinalType getNumConstraints() const
    {
        const OrdinalType tNumConstraints = mCurrentConstraintValues->size();
        return tNumConstraints;
    }

    ScalarType getCurrentObjectiveValue() const
    {
        return mCurrentObjectiveValue;
    }

    void setCurrentObjectiveValue(const ScalarType &aValue) const
    {
        mCurrentObjectiveValue = aValue;
    }

    ScalarType getPreviousObjectiveValue() const
    {
        return mPreviousObjectiveValue;
    }

    void setPreviousObjectiveValue(const ScalarType &aValue) const
    {
        mPreviousObjectiveValue = aValue;
    }

    ScalarType getConstraintLimit(const OrdinalType &aIndex) const
    {
        return ((*mConstraintLimits)[aIndex]);
    }

    void setConstraintLimit(const OrdinalType &aIndex, const ScalarType &aValue) const
    {
        (*mConstraintLimits)[aIndex] = aValue;
    }

    ScalarType getCurrentConstraintValue(const OrdinalType &aIndex) const
    {
        return ((*mCurrentConstraintValues)[aIndex]);
    }

    void setCurrentConstraintValue(const OrdinalType &aIndex, const ScalarType &aValue) const
    {
        (*mCurrentConstraintValues)[aIndex] = aValue;
    }

    ScalarType getPreviousConstraintValues(const OrdinalType &aIndex) const
    {
        return ((*mPreviousConstraintValues)[aIndex]);
    }

    void setPreviousConstraintValues(const OrdinalType &aIndex, const ScalarType &aValue) const
    {
        (*mPreviousConstraintValues)[aIndex] = aValue;
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getLowerAsymptotes()
    {
        return (*mLowerAsymptotes);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getLowerAsymptotes() const
    {
        return (*mLowerAsymptotes);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getUpperAsymptotes()
    {
        return (*mUpperAsymptotes);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getUpperAsymptotes() const
    {
        return (*mUpperAsymptotes);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getObjFuncAppxFunctionP()
    {
        return (*mObjFuncAppxFunctionP);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getObjFuncAppxFunctionP() const
    {
        return (*mObjFuncAppxFunctionP);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getObjFuncAppxFunctionQ()
    {
        return (*mObjFuncAppxFunctionQ);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getObjFuncAppxFunctionQ() const
    {
        return (*mObjFuncAppxFunctionQ);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getConstraintAppxFunctionP(const OrdinalType & aIndex)
    {
        return (*mConstrAppxFunctionP)[aIndex];
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getConstraintAppxFunctionP(const OrdinalType & aIndex) const
    {
        return (*mConstrAppxFunctionP)[aIndex];
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getConstraintAppxFunctionQ(const OrdinalType & aIndex)
    {
        return (*mConstrAppxFunctionQ)[aIndex];
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getConstraintAppxFunctionQ(const OrdinalType & aIndex) const
    {
        return (*mConstrAppxFunctionQ)[aIndex];
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getCurrentControls()
    {
        return (*mCurrentControls);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getCurrentControls() const
    {
        return (*mCurrentControls);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getPreviousControls()
    {
        return (*mPreviousControls);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getPreviousControls() const
    {
        return (*mPreviousControls);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getAntepenultimateControls()
    {
        return (*mAntepenultimateControls);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getAntepenultimateControls() const
    {
        return (*mAntepenultimateControls);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getControlLowerBounds()
    {
        return (*mControlLowerBounds);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getControlLowerBounds() const
    {
        return (*mControlLowerBounds);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getControlUpperBounds()
    {
        return (*mControlUpperBounds);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getControlUpperBounds() const
    {
        return (*mControlUpperBounds);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getSubProblemControlLowerBounds()
    {
        return (*mSubProblemControlLowerBounds);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getSubProblemControlLowerBounds() const
    {
        return (*mSubProblemControlLowerBounds);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getSubProblemControlUpperBounds()
    {
        return (*mSubProblemControlUpperBounds);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getSubProblemControlUpperBounds() const
    {
        return (*mSubProblemControlUpperBounds);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getCurrentObjectiveGradient()
    {
        return (*mCurrentObjectiveGradient);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getCurrentObjectiveGradient() const
    {
        return (*mCurrentObjectiveGradient);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getPreviousObjectiveGradient()
    {
        return (*mPreviousObjectiveGradient);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getPreviousObjectiveGradient() const
    {
        return (*mPreviousObjectiveGradient);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getCurrentConstraintGradient(const OrdinalType &aIndex)
    {
        return (*mCurrentConstraintGradients)[aIndex];
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getCurrentConstraintGradient(const OrdinalType &aIndex) const
    {
        return (*mCurrentConstraintGradients)[aIndex];
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getPreviousConstraintGradient(const OrdinalType &aIndex)
    {
        return (*mPreviousConstraintGradients)[aIndex];
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getPreviousConstraintGradient(const OrdinalType &aIndex) const
    {
        return (*mPreviousConstraintGradients)[aIndex];
    }

    void cacheState()
    {
        Plato::update(static_cast<ScalarType>(1), *mPreviousControls, static_cast<ScalarType>(1), *mAntepenultimateControls);
        Plato::update(static_cast<ScalarType>(1), *mCurrentControls, static_cast<ScalarType>(1), *mPreviousControls);

        mPreviousObjectiveValue = mCurrentObjectiveValue;
        Plato::update(static_cast<ScalarType>(1), *mCurrentObjectiveGradient, static_cast<ScalarType>(1), *mPreviousObjectiveGradient);

        mPreviousConstraintValues->update(static_cast<ScalarType>(1), *mCurrentConstraintValues, static_cast<ScalarType>(0));
        const OrdinalType tNumConstraints = mCurrentConstraintValues->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            Plato::update(static_cast<ScalarType>(1),
                          (*mCurrentConstraintGradients)[tIndex],
                          static_cast<ScalarType>(1),
                          (*mPreviousConstraintGradients)[tIndex]);
        }
    }

private:
    ScalarType mCurrentObjectiveValue;
    ScalarType mPreviousObjectiveValue;
    ScalarType mCurrentNormalizedObjectiveValue;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mConstraintLimits;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentConstraintValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mPreviousConstraintValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentNormalizedConstraintValue;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentObjectiveGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousObjectiveGradient;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mCurrentConstraintGradients;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mPreviousConstraintGradients;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControls;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousControls;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAntepenultimateControls;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerAsymptotes;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperAsymptotes;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlLowerBounds;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlUpperBounds;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mSubProblemControlLowerBounds;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mSubProblemControlUpperBounds;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mObjFuncAppxFunctionP;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mObjFuncAppxFunctionQ;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mConstrAppxFunctionP;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mConstrAppxFunctionQ;
};

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

    void update(const Plato::ApproximationFunctionData<ScalarType, OrdinalType> &aData)
    {

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
     * @param [in/out] aOutput: gradient of approximation function
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
     * @param [in/out] aOutput: application of vector to the Hessian of the approximation function
     ***********************************************************************************/
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> &aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> &aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> &aOutput)
    {
        this->computeApproximationFunctionHessTimesVec(aControl, aVector, aOutput);
    }

private:
    ScalarType evaluateApproximationFunction(const Plato::MultiVector<ScalarType, OrdinalType> &aControls)
    {
        mControlWork1->fill(static_cast<ScalarType>(0));
        mControlWork2->fill(static_cast<ScalarType>(0));

        const OrdinalType tNumVectors = aControls.getNumVectors();
        for (OrdinalType tVecIndex = 0; tVecIndex < tNumVectors; tVecIndex++)
        {
            const OrdinalType tNumControls = aControls[tVecIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
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
        mApproxFuncEpsilon(1e-5),
        mAsymptoteExpansion(1.2),
        mAsymptoteContraction(0.7),
        mInitialAymptoteScaling(0.5),
        mSubProblemBoundsScaling(0.1),
        mApproxFuncScalingOne(1.001),
        mApproxFuncScalingTwo(0.001),
        mObjective(aObjective),
        mConstraints(aConstraints)
    {
    }

    void solve()
    {
        this->initialize();

        mDataMng->cacheState();
        this->evaluateConstraints();
        this->evaluateObjective();
        this->updateSubProblem();
        this->solveSubProblem();
    }

private:
    void initialize()
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlUpperBounds = mDataMng->getControlUpperBounds();
        Plato::update(static_cast<ScalarType>(1), tControlUpperBounds, static_cast<ScalarType>(0), *mUpperMinusLowerBounds);
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlLowerBounds = mDataMng->getControlLowerBounds();
        Plato::update(static_cast<ScalarType>(-1), tControlLowerBounds, static_cast<ScalarType>(1), *mUpperMinusLowerBounds);
    }

    void evaluateObjective()
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = mDataMng->getCurrentControls();
        ScalarType tObjFuncValue = mObjective->value(tCurrentControls);
        mDataMng->setCurrentObjectiveValue(tObjFuncValue);
        Plato::MultiVector<ScalarType, OrdinalType> &tCurrentObjGrad = mDataMng->getCurrentObjectiveGradient();
        mObjective->gradient(tCurrentControls, tCurrentObjGrad);
    }

    void evaluateConstraints()
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = mDataMng->getCurrentControls();
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const ScalarType tValue = (*mConstraints)[tIndex].value(tCurrentControls);
            mDataMng->setCurrentConstraintValue(tIndex, tValue);
            Plato::MultiVector<ScalarType, OrdinalType> &tCurrentConstraintGrad = mDataMng->getCurrentConstraintGradient(tIndex);
            (*mConstraints)[tIndex].gradient(tCurrentControls, tCurrentConstraintGrad);
        }
    }

    void updateSubProblem()
    {
        this->updateAsymptotes();
        this->updateSubProblemBounds();
        this->updateObjectiveApproximationFunctionData();
        this->updateObjectiveApproximationFunction();
        this->updateConstraintApproximationFunctionsData();
        this->updateConstraintApproximationFunctions();
    }

    void updateObjectiveApproximationFunction()
    {
        mMMAData.mCurrentNormalizedCriterionValue = 1;
        Plato::update(static_cast<ScalarType>(1), mDataMng->getCurrentControls(), static_cast<ScalarType>(0), mMMAData.mCurrentControls);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getLowerAsymptotes(), static_cast<ScalarType>(0), mMMAData.mLowerAsymptotes);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getUpperAsymptotes(), static_cast<ScalarType>(0), mMMAData.mUpperAsymptotes);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getObjFuncAppxFunctionP(), static_cast<ScalarType>(0), mMMAData.mAppxFunctionP);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getObjFuncAppxFunctionQ(), static_cast<ScalarType>(0), mMMAData.mAppxFunctionQ);

        mObjAppxFunc->update(mMMAData);
    }

    void updateConstraintApproximationFunctions()
    {
        Plato::update(static_cast<ScalarType>(1), mDataMng->getCurrentControls(), static_cast<ScalarType>(0), mMMAData.mCurrentControls);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getLowerAsymptotes(), static_cast<ScalarType>(0), mMMAData.mLowerAsymptotes);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getUpperAsymptotes(), static_cast<ScalarType>(0), mMMAData.mUpperAsymptotes);

        const OrdinalType tNumConstraints = mDataMng->getNumConstraints();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const ScalarType tConstraintValue = mDataMng->getCurrentConstraintValue(tIndex);
            const ScalarType tConstraintLimit = mDataMng->getCurrentConstraintLimit(tIndex);
            mMMAData.mCurrentNormalizedCriterionValue = tConstraintValue / tConstraintLimit;
            Plato::update(static_cast<ScalarType>(1), mDataMng->getConstraintAppxFunctionP(), static_cast<ScalarType>(0), mMMAData.mAppxFunctionP);
            Plato::update(static_cast<ScalarType>(1), mDataMng->getConstraintAppxFunctionQ(), static_cast<ScalarType>(0), mMMAData.mAppxFunctionQ);
            mConstrAppxFuncs[tIndex]->update(mMMAData);
        }
    }

    void updateObjectiveApproximationFunctionData()
    {
        const ScalarType tObjFuncValue = mDataMng->getCurrentObjectiveValue();
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentObjGrad = mDataMng->getCurrentObjectiveGradient();
        Plato::MultiVector<ScalarType, OrdinalType> &tAppxFunctionP = mDataMng->getObjFuncAppxFunctionP();
        Plato::MultiVector<ScalarType, OrdinalType> &tAppxFunctionQ = mDataMng->getObjFuncAppxFunctionQ();
        this->computeApproxFuncCoefficients(tObjFuncValue, tCurrentObjGrad, tAppxFunctionP, tAppxFunctionQ);
    }

    void updateConstraintApproximationFunctionsData()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const ScalarType tLimitValue = mDataMng->getConstraintLimit(tIndex);
            const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentConstraintGrad = mDataMng->getCurrentConstraintGradient(tIndex);
            Plato::MultiVector<ScalarType, OrdinalType> &tMyAppxFunctionP = mDataMng->getConstraintAppxFunctionP(tIndex);
            Plato::MultiVector<ScalarType, OrdinalType> &tMyAppxFunctionQ = mDataMng->getConstraintAppxFunctionQ(tIndex);
            this->computeApproxFuncCoefficients(tLimitValue, tCurrentConstraintGrad, tMyAppxFunctionP, tMyAppxFunctionQ);
        }
    }

    void solveSubProblem()
    {
        mSubProblemSolver->setInitialGuess(mDataMng->getCurrentControls());
        mSubProblemSolver->setControlLowerBounds(mDataMng->getSubProblemControlLowerBounds());
        mSubProblemSolver->setControlUpperBounds(mDataMng->getSubProblemControlUpperBounds());
        mSubProblemSolver->solve();
        mSubProblemSolver->getSolution(mDataMng->getCurrentControls());
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
        Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = mDataMng->getLowerAsymptotes();
        Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = mDataMng->getUpperAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = mDataMng->getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tPreviousControls = mDataMng->getPreviousControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tAntepenultimateControls = mDataMng->getAntepenultimateControls();

        const OrdinalType tNumVectors = tCurrentControls.getNumVectors();
        for (OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = tCurrentControls[tVectorIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                const ScalarType tMeasure = (tCurrentControls(tVectorIndex, tControlIndex) * tPreviousControls(tVectorIndex, tControlIndex))
                    * (tPreviousControls(tVectorIndex, tControlIndex) * tAntepenultimateControls(tVectorIndex, tControlIndex))
                ScalarType tGammaValue = tMeasure > static_cast<ScalarType>(0) ? mAsymptoteExpansion : mAsymptoteContraction;
                tGammaValue = std::abs(tGammaValue) <= std::numeric_limits<ScalarType>::min() ? static_cast<ScalarType>(1) : tGammaValue;

                tLowerAsymptotes(tVectorIndex, tControlIndex) = tCurrentControls(tVectorIndex, tControlIndex)
                    - (tGammaValue * (tPreviousControls(tVectorIndex, tControlIndex) - tLowerAsymptotes(tVectorIndex, tControlIndex)));
                tUpperAsymptotes(tVectorIndex, tControlIndex) = tCurrentControls(tVectorIndex, tControlIndex)
                    + (tGammaValue * (tUpperAsymptotes(tVectorIndex, tControlIndex) - tPreviousControls(tVectorIndex, tControlIndex)));
            }
        }
    }

    void updateInitialAsymptotes()
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = mDataMng->getCurrentControls();
        Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = mDataMng->getLowerAsymptotes();
        Plato::update(static_cast<ScalarType>(1), tCurrentControls, static_cast<ScalarType>(0), tLowerAsymptotes);
        Plato::update(-mInitialAymptoteScaling, *mUpperMinusLowerBounds, static_cast<ScalarType>(1), tLowerAsymptotes);

        Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = mDataMng->getUpperAsymptotes();
        Plato::update(static_cast<ScalarType>(1), tCurrentControls, static_cast<ScalarType>(0), tUpperAsymptotes);
        Plato::update(mInitialAymptoteScaling, *mUpperMinusLowerBounds, static_cast<ScalarType>(1), tUpperAsymptotes);
    }

    void updateSubProblemBounds()
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = mDataMng->getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = mDataMng->getLowerAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = mDataMng->getUpperAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlLowerBounds = mDataMng->getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlUpperBounds = mDataMng->getControlUpperBounds();

        Plato::MultiVector<ScalarType, OrdinalType> &tSubProblemLowerBounds = mDataMng->getSubProblemControlLowerBounds()
        Plato::MultiVector<ScalarType, OrdinalType> &tSubProblemUpperBounds = mDataMng->getSubProblemControlUpperBounds()

        std::vector<ScalarType> tCriteria(3/* length */);
        const OrdinalType tNumVectors = tCurrentControls.getNumVectors();
        for (OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = tCurrentControls[tVectorIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                tCriteria[0] = tControlLowerBounds(tVectorIndex, tControlIndex);
                tCriteria[1] = tLowerAsymptotes(tVectorIndex, tControlIndex)
                    + (mSubProblemBoundsScaling * (tCurrentControls(tVectorIndex, tControlIndex) - tLowerAsymptotes(tVectorIndex, tControlIndex)));
                tCriteria[2] = tCurrentControls(tVectorIndex, tControlIndex) - (mMoveLimit * (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                tSubProblemLowerBounds(tVectorIndex, tControlIndex) = *std::max_element(tCriteria.begin(), tCriteria.end());

                tCriteria[0] = tControlUpperBounds(tVectorIndex, tControlIndex);
                tCriteria[1] = tUpperAsymptotes(tVectorIndex, tControlIndex)
                    - (mSubProblemBoundsScaling * (tUpperAsymptotes(tVectorIndex, tControlIndex) - tCurrentControls(tVectorIndex, tControlIndex)));
                tCriteria[2] = tCurrentControls(tVectorIndex, tControlIndex) + (mMoveLimit * (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                tSubProblemUpperBounds(tVectorIndex, tControlIndex) = *std::min_element(tCriteria.begin(), tCriteria.end());
            }
        }
    }

    void computeApproxFuncCoefficients(const ScalarType &aNormalization,
                                       const Plato::MultiVector<ScalarType, OrdinalType> &aCriterionGrad,
                                       Plato::MultiVector<ScalarType, OrdinalType> &aAppxFunctionP,
                                       Plato::MultiVector<ScalarType, OrdinalType> &aAppxFunctionQ)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = mDataMng->getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = mDataMng->getLowerAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = mDataMng->getUpperAsymptotes();

        const OrdinalType tAbsNormalization = std::abs(aNormalization);
        const OrdinalType tNumVectors = aCriterionGrad.getNumVectors();
        for (OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = aCriterionGrad[tVectorIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                const ScalarType tNormalizedGradValue = aCriterionGrad(tVectorIndex, tControlIndex) / tAbsNormalization;
                const ScalarType tGradValuePlus = std::max(tNormalizedGradValue, static_cast<ScalarType>(0));
                const ScalarType tGradValueMinus = std::max(-tNormalizedGradValue, static_cast<ScalarType>(0));

                aAppxFunctionP(tVectorIndex, tControlIndex) = (mApproxFuncScalingOne * tGradValuePlus) + (mApproxFuncScalingTwo * tGradValueMinus)
                    + (mApproxFuncEpsilon / (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                const ScalarType tUpperAsymmMinusCurrentControlSquared = tUpperAsymptotes(tVectorIndex, tControlIndex)
                    - tCurrentControls(tVectorIndex, tControlIndex);
                tUpperAsymmMinusCurrentControlSquared *= tUpperAsymmMinusCurrentControlSquared;
                aAppxFunctionP(tVectorIndex, tControlIndex) *= tUpperAsymmMinusCurrentControlSquared;

                aAppxFunctionQ(tVectorIndex, tControlIndex) = (mApproxFuncScalingTwo * tGradValuePlus) + (mApproxFuncScalingOne * tGradValueMinus)
                    + (mApproxFuncEpsilon / (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                const ScalarType tCurrentControlMinusLowerAsymmSquared = tCurrentControls(tVectorIndex, tControlIndex)
                    - tLowerAsymptotes(tVectorIndex, tControlIndex);
                tCurrentControlMinusLowerAsymmSquared *= tCurrentControlMinusLowerAsymmSquared;
                aAppxFunctionQ(tVectorIndex, tControlIndex) *= tCurrentControlMinusLowerAsymmSquared;
            }
        }
    }

private:
    OrdinalType mIterationCount;
    OrdinalType mMaxNumIterations;

    ScalarType mMoveLimit;
    ScalarType mApproxFuncEpsilon;
    ScalarType mAsymptoteExpansion;
    ScalarType mAsymptoteContraction;
    ScalarType mApproxFuncScalingOne;
    ScalarType mApproxFuncScalingTwo;
    ScalarType mInitialAymptoteScaling;
    ScalarType mSubProblemBoundsScaling;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperMinusLowerBounds;

    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mObjective;
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> mConstraints;

    Plato::ApproximationFunctionData<ScalarType, OrdinalType> mMMAData;
    std::shared_ptr<Plato::AugmentedLagrangian<ScalarType, OrdinalType>> mSubProblemSolver;
    std::shared_ptr<Plato::MethodMovingAsymptotesDataMng<ScalarType, OrdinalType>> mDataMng;
    std::shared_ptr<Plato::MethodMovingAsymptotesCriterion<ScalarType, OrdinalType>> mObjAppxFunc;
    std::vector<std::shared_ptr<Plato::MethodMovingAsymptotesCriterion<ScalarType, OrdinalType>>> mConstrAppxFuncs;
};
// class MethodMovingAsymptotes

}
// namespace Plato
