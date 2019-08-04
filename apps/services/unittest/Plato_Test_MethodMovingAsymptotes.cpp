/*
 * Plato_Test_MethodMovingAsymptotesNew.cpp
 *
 *  Created on: Jul 21, 2019
 */

#include <gtest/gtest.h>

#include <stddef.h>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "Plato_Radius.hpp"
#include "Plato_Circle.hpp"
#include "Plato_Rosenbrock.hpp"
#include "Plato_Himmelblau.hpp"
#include "Plato_GoldsteinPrice.hpp"
#include "Plato_ShiftedEllipse.hpp"
#include "Plato_CcsaTestObjective.hpp"
#include "Plato_CcsaTestInequality.hpp"

#include "Plato_AugmentedLagrangian.hpp"
#include "Plato_CommWrapper.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_Diagnostics.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_MultiVectorList.hpp"
#include "Plato_ReductionOperations.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_Types.hpp"
#include "Plato_Vector.hpp"
#include "Plato_Macros.hpp"

#include "Plato_UnitTestUtils.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
struct ApproximationFunctionData
{
    ApproximationFunctionData(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mCurrentNormalizedCriterionValue(0),
        mAppxFunctionP(aDataFactory->control().create()),
        mAppxFunctionQ(aDataFactory->control().create()),
        mCurrentControls(aDataFactory->control().create()),
        mLowerAsymptotes(aDataFactory->control().create()),
        mUpperAsymptotes(aDataFactory->control().create())
    {
    }

    ScalarType mCurrentNormalizedCriterionValue;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionP;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionQ;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControls;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerAsymptotes;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperAsymptotes;
};

template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesNewDataMng
{
public:
    explicit MethodMovingAsymptotesNewDataMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mCurrentObjectiveValue(std::numeric_limits<ScalarType>::max()),
        mPreviousObjectiveValue(std::numeric_limits<ScalarType>::max()),
        mControlStagnationMeasure(std::numeric_limits<ScalarType>::max()),
        mCurrentNormalizedObjectiveValue(std::numeric_limits<ScalarType>::max()),
        mControlWork(aDataFactory->control(0 /* vector index */).create()),
        mConstraintNormalization(aDataFactory->dual(0 /* vector index */).create()),
        mCurrentConstraintValues(aDataFactory->dual(0 /* vector index */).create()),
        mPreviousConstraintValues(aDataFactory->dual(0 /* vector index */).create()),
        mCurrentNormalizedConstraintValue(aDataFactory->dual(0 /* vector index */).create()),
        mCurrentObjectiveGradient(aDataFactory->control().create()),
        mPreviousObjectiveGradient(aDataFactory->control().create()),
        mCurrentConstraintGradients(std::make_shared<MultiVectorList<ScalarType, OrdinalType>>()),
        mPreviousConstraintGradients(std::make_shared<MultiVectorList<ScalarType, OrdinalType>>()),
        mCurrentControls(aDataFactory->control().create()),
        mPreviousControls(aDataFactory->control().create()),
        mAntepenultimateControls(aDataFactory->control().create()),
        mLowerAsymptotes(aDataFactory->control().create()),
        mUpperAsymptotes(aDataFactory->control().create()),
        mControlLowerBounds(aDataFactory->control().create()),
        mControlUpperBounds(aDataFactory->control().create()),
        mSubProblemControlLowerBounds(aDataFactory->control().create()),
        mSubProblemControlUpperBounds(aDataFactory->control().create()),
        mObjFuncAppxFunctionP(aDataFactory->control().create()),
        mObjFuncAppxFunctionQ(aDataFactory->control().create()),
        mConstrAppxFunctionP(std::make_shared<MultiVectorList<ScalarType, OrdinalType>>()),
        mConstrAppxFunctionQ(std::make_shared<MultiVectorList<ScalarType, OrdinalType>>()),
        mComm(aDataFactory->getCommWrapper().create()),
        mControlReductionOps(aDataFactory->getControlReductionOperations().create())
    {
        this->initialize(*aDataFactory);
    }

    ~MethodMovingAsymptotesNewDataMng()
    {
    }

    /******************************************************************************//**
     * @brief Return a const reference to the distributed memory communication wrapper
     * @return const reference to the distributed memory communication wrapper
    **********************************************************************************/
    const Plato::CommWrapper& getCommWrapper() const
    {
        return (mComm.operator*());
    }

    OrdinalType getNumConstraints() const
    {
        const OrdinalType tNumConstraints = mCurrentConstraintValues->size();
        return tNumConstraints;
    }

    ScalarType getControlStagnationMeasure() const
    {
        return mControlStagnationMeasure;
    }

    ScalarType getCurrentObjectiveValue() const
    {
        return mCurrentObjectiveValue;
    }

    void setCurrentObjectiveValue(const ScalarType &aValue)
    {
        mCurrentObjectiveValue = aValue;
    }

    ScalarType getPreviousObjectiveValue() const
    {
        return mPreviousObjectiveValue;
    }

    void setPreviousObjectiveValue(const ScalarType &aValue)
    {
        mPreviousObjectiveValue = aValue;
    }

    ScalarType getConstraintNormalization(const OrdinalType &aIndex) const
    {
        return ((*mConstraintNormalization)[aIndex]);
    }

    void setConstraintNormalization(const OrdinalType &aIndex, const ScalarType &aValue)
    {
        (*mConstraintNormalization)[aIndex] = aValue;
    }

    ScalarType getCurrentConstraintValue(const OrdinalType &aIndex) const
    {
        return ((*mCurrentConstraintValues)[aIndex]);
    }

    void setCurrentConstraintValue(const OrdinalType &aIndex, const ScalarType &aValue)
    {
        (*mCurrentConstraintValues)[aIndex] = aValue;
    }

    ScalarType getPreviousConstraintValues(const OrdinalType &aIndex) const
    {
        return ((*mPreviousConstraintValues)[aIndex]);
    }

    void setPreviousConstraintValues(const OrdinalType &aIndex, const ScalarType &aValue)
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

    void setLowerAsymptotes(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mLowerAsymptotes);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getUpperAsymptotes()
    {
        return (*mUpperAsymptotes);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getUpperAsymptotes() const
    {
        return (*mUpperAsymptotes);
    }

    void setUpperAsymptotes(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mUpperAsymptotes);
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

    void setCurrentControls(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentControls);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getPreviousControls()
    {
        return (*mPreviousControls);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getPreviousControls() const
    {
        return (*mPreviousControls);
    }

    void setPreviousControls(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mPreviousControls);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getAntepenultimateControls()
    {
        return (*mAntepenultimateControls);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getAntepenultimateControls() const
    {
        return (*mAntepenultimateControls);
    }

    void setAntepenultimateControls(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mAntepenultimateControls);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getControlLowerBounds()
    {
        return (*mControlLowerBounds);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getControlLowerBounds() const
    {
        return (*mControlLowerBounds);
    }

    void setControlLowerBounds(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mControlLowerBounds);
    }

    Plato::MultiVector<ScalarType, OrdinalType>& getControlUpperBounds()
    {
        return (*mControlUpperBounds);
    }

    const Plato::MultiVector<ScalarType, OrdinalType>& getControlUpperBounds() const
    {
        return (*mControlUpperBounds);
    }

    void setControlUpperBounds(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mControlUpperBounds);
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

    void setCurrentObjectiveGradient(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentObjectiveGradient);
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

    void setCurrentConstraintGradient(const OrdinalType &aIndex, const Plato::MultiVector<ScalarType, OrdinalType>& aGradient)
    {
        if(aIndex >= mCurrentConstraintGradients->size())
        {
            std::string tMsg = std::string("THE INPUT CONSTRAINT GRADIENT INDEX IS GREATER THAN THE NUMBER OF CONSTRAINTS. ")
                    + "THE NUMBER OF CONSTRAINTS IS SET TO " + std::to_string(aIndex) + " AND THE INPUT INDEX IS "
                    + std::to_string(aIndex) + "\n";
            THROWERR(tMsg)
        }
        Plato::update(static_cast<ScalarType>(1), aGradient, static_cast<ScalarType>(0), (*mCurrentConstraintGradients)[aIndex]);
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
        Plato::update(static_cast<ScalarType>(1), *mPreviousControls, static_cast<ScalarType>(0), *mAntepenultimateControls);
        Plato::update(static_cast<ScalarType>(1), *mCurrentControls, static_cast<ScalarType>(0), *mPreviousControls);

        mPreviousObjectiveValue = mCurrentObjectiveValue;
        Plato::update(static_cast<ScalarType>(1), *mCurrentObjectiveGradient, static_cast<ScalarType>(0), *mPreviousObjectiveGradient);

        mPreviousConstraintValues->update(static_cast<ScalarType>(1), *mCurrentConstraintValues, static_cast<ScalarType>(0));
        const OrdinalType tNumConstraints = mCurrentConstraintValues->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            Plato::update(static_cast<ScalarType>(1),
                          (*mCurrentConstraintGradients)[tIndex],
                          static_cast<ScalarType>(0),
                          (*mPreviousConstraintGradients)[tIndex]);
        }
    }

    void computeControlStagnationMeasure()
    {
        OrdinalType tNumVectors = mCurrentControls->getNumVectors();
        std::vector<ScalarType> tStorage(tNumVectors, std::numeric_limits<ScalarType>::min());
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            mControlWork->update(static_cast<ScalarType>(1), (*mCurrentControls)[tIndex], static_cast<ScalarType>(0));
            mControlWork->update(static_cast<ScalarType>(-1), (*mPreviousControls)[tIndex], static_cast<ScalarType>(1));
            mControlWork->modulus();
            tStorage[tIndex] = mControlReductionOps->max(*mControlWork);
        }
        mControlStagnationMeasure = *std::max_element(tStorage.begin(), tStorage.end());
    }

private:
    void initialize(const Plato::DataFactory<ScalarType, OrdinalType> &aDataFactory)
    {
        const OrdinalType tNumConstraints = this->getNumConstraints();
        if(tNumConstraints < static_cast<OrdinalType>(0))
        {
            THROWERR(std::string("INVALID NUMBER OF CONSTRAINTS ") + std::to_string(tNumConstraints) + ". THE NUMBER OF CONSTRAINTS SHOULD BE ZERO OR A POSITIVE NUMBER.\n")
        }

        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            mConstrAppxFunctionP->add(aDataFactory.control().create());
            mConstrAppxFunctionQ->add(aDataFactory.control().create());
            mCurrentConstraintGradients->add(aDataFactory.control().create());
            mPreviousConstraintGradients->add(aDataFactory.control().create());
        }
    }

private:
    ScalarType mCurrentObjectiveValue;
    ScalarType mPreviousObjectiveValue;
    ScalarType mControlStagnationMeasure;
    ScalarType mCurrentNormalizedObjectiveValue;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWork;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mConstraintNormalization;
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

    std::shared_ptr<Plato::CommWrapper> mComm;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductionOps;
};

/******************************************************************************//**
 * Criterion interface to the approximation function used to solve the Method of
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
class MethodMovingAsymptotesNewCriterion : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * Constructor
     * @param [in] aDataFactory constant reference to the core data factory shared pointer.
    ***********************************************************************************/
    explicit MethodMovingAsymptotesNewCriterion(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mCurrentNormalizedCriterionValue(0),
        mControlWork1(aDataFactory->control(0 /* vector index */).create()),
        mControlWork2(aDataFactory->control(0 /* vector index */).create()),
        mAppxFunctionP(aDataFactory->control().create()),
        mAppxFunctionQ(aDataFactory->control().create()),
        mCurrentControls(aDataFactory->control().create()),
        mLowerAsymptotes(aDataFactory->control().create()),
        mUpperAsymptotes(aDataFactory->control().create()),
        mControlReductionOps(aDataFactory->getControlReductionOperations().create())
    {
    }

    /******************************************************************************//**
     * Destructor
    ***********************************************************************************/
    ~MethodMovingAsymptotesNewCriterion()
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
        mCurrentNormalizedCriterionValue = aData.mCurrentNormalizedCriterionValue;
        Plato::update(static_cast<ScalarType>(1), *aData.mAppxFunctionP, static_cast<ScalarType>(0), *mAppxFunctionP);
        Plato::update(static_cast<ScalarType>(1), *aData.mAppxFunctionQ, static_cast<ScalarType>(0), *mAppxFunctionQ);
        Plato::update(static_cast<ScalarType>(1), *aData.mCurrentControls, static_cast<ScalarType>(0), *mCurrentControls);
        Plato::update(static_cast<ScalarType>(1), *aData.mLowerAsymptotes, static_cast<ScalarType>(0), *mLowerAsymptotes);
        Plato::update(static_cast<ScalarType>(1), *aData.mUpperAsymptotes, static_cast<ScalarType>(0), *mUpperAsymptotes);
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
                ScalarType tDenominator = (*mUpperAsymptotes)(tVecIndex, tControlIndex) - aControls(tVecIndex, tControlIndex);
                tDenominator = std::abs(tDenominator) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominator : std::numeric_limits<ScalarType>::epsilon();
                ScalarType tValueOne = (*mAppxFunctionP)(tVecIndex, tControlIndex) / tDenominator;
                tDenominator = aControls(tVecIndex, tControlIndex) - (*mLowerAsymptotes)(tVecIndex, tControlIndex);
                tDenominator = std::abs(tDenominator) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominator : std::numeric_limits<ScalarType>::epsilon();
                ScalarType tValueTwo = (*mAppxFunctionQ)(tVecIndex, tControlIndex) / tDenominator;
                (*mControlWork1)[tControlIndex] += tValueOne + tValueTwo;

                tDenominator = (*mUpperAsymptotes)(tVecIndex, tControlIndex) - (*mCurrentControls)(tVecIndex, tControlIndex);
                tDenominator = std::abs(tDenominator) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominator : std::numeric_limits<ScalarType>::epsilon();
                tValueOne = (*mAppxFunctionP)(tVecIndex, tControlIndex) / tDenominator;
                tDenominator = (*mCurrentControls)(tVecIndex, tControlIndex) - (*mLowerAsymptotes)(tVecIndex, tControlIndex);
                tDenominator = std::abs(tDenominator) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominator : std::numeric_limits<ScalarType>::epsilon();
                tValueTwo = (*mAppxFunctionQ)(tVecIndex, tControlIndex) / tDenominator;
                (*mControlWork2)[tControlIndex] += tValueOne + tValueTwo;
            }
        }

        const ScalarType tTermOne = mControlReductionOps->sum(*mControlWork1);
        const ScalarType tTermTwo = mControlReductionOps->sum(*mControlWork2);
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
                tDenominatorValue = std::abs(tDenominatorValue) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominatorValue : std::numeric_limits<ScalarType>::epsilon();
                ScalarType tDenominator = tDenominatorValue * tDenominatorValue;
                ScalarType tValueOne = (*mAppxFunctionP)(tVecIndex, tControlIndex) / tDenominator;

                tDenominatorValue = aControls(tVecIndex, tControlIndex) - (*mLowerAsymptotes)(tVecIndex, tControlIndex);
                tDenominatorValue = std::abs(tDenominatorValue) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominatorValue : std::numeric_limits<ScalarType>::epsilon();
                tDenominator = tDenominatorValue * tDenominatorValue;
                ScalarType tValueTwo = (*mAppxFunctionQ)(tVecIndex, tControlIndex) / tDenominator;
                aGradient(tVecIndex, tControlIndex) = tValueOne - tValueTwo;
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
                tDenominatorValue = std::abs(tDenominatorValue) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominatorValue : std::numeric_limits<ScalarType>::epsilon();
                ScalarType tDenominator = tDenominatorValue * tDenominatorValue * tDenominatorValue;
                ScalarType tValueOne = (*mAppxFunctionP)(tVecIndex, tControlIndex) / tDenominator;

                tDenominatorValue = aControls(tVecIndex, tControlIndex) - (*mLowerAsymptotes)(tVecIndex, tControlIndex);
                tDenominatorValue = std::abs(tDenominatorValue) > std::numeric_limits<ScalarType>::epsilon()
                        ? tDenominatorValue : std::numeric_limits<ScalarType>::epsilon();
                tDenominator = tDenominatorValue * tDenominatorValue * tDenominatorValue;
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

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductionOps;
};

template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesOperations
{
public:
    MethodMovingAsymptotesOperations(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mMoveLimit(0.5),
        mApproxFuncEpsilon(1e-5),
        mAsymptoteExpansion(1.2),
        mAsymptoteContraction(0.7),
        mApproxFuncScalingOne(1.001),
        mApproxFuncScalingTwo(0.001),
        mInitialAymptoteScaling(0.5),
        mSubProblemBoundsScaling(0.1),
        mUpperMinusLowerBounds(aDataFactory->control().create()),
        mCurrentAsymptotesMultipliers(aDataFactory->control().create())
    {
    }

    ~MethodMovingAsymptotesOperations()
    {
    }

    void setMoveLimit(const ScalarType& aInput)
    {
        mMoveLimit = aInput;
    }

    void setAsymptoteExpansion(const ScalarType& aInput)
    {
        mAsymptoteExpansion = aInput;
    }

    void setAsymptoteContraction(const ScalarType& aInput)
    {
        mAsymptoteContraction = aInput;
    }

    void setInitialAymptoteScaling(const ScalarType& aInput)
    {
        mInitialAymptoteScaling = aInput;
    }

    void getUpperMinusLowerBounds(Plato::MultiVector<ScalarType, OrdinalType> &aData)
    {
        Plato::update(static_cast<ScalarType>(1), *mUpperMinusLowerBounds, static_cast<ScalarType>(0), aData);
    }

    void getCurrentAsymptotesMultipliers(Plato::MultiVector<ScalarType, OrdinalType> &aData)
    {
        Plato::update(static_cast<ScalarType>(1), *mCurrentAsymptotesMultipliers, static_cast<ScalarType>(0), aData);
    }

    void initialize(const Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlUpperBounds = aDataMng.getControlUpperBounds();
        Plato::update(static_cast<ScalarType>(1), tControlUpperBounds, static_cast<ScalarType>(0), *mUpperMinusLowerBounds);
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlLowerBounds = aDataMng.getControlLowerBounds();
        Plato::update(static_cast<ScalarType>(-1), tControlLowerBounds, static_cast<ScalarType>(1), *mUpperMinusLowerBounds);
    }

    void updateInitialAsymptotes(Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        Plato::update(static_cast<ScalarType>(1), tCurrentControls, static_cast<ScalarType>(0), tLowerAsymptotes);
        Plato::update(-mInitialAymptoteScaling, *mUpperMinusLowerBounds, static_cast<ScalarType>(1), tLowerAsymptotes);

        Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();
        Plato::update(static_cast<ScalarType>(1), tCurrentControls, static_cast<ScalarType>(0), tUpperAsymptotes);
        Plato::update(mInitialAymptoteScaling, *mUpperMinusLowerBounds, static_cast<ScalarType>(1), tUpperAsymptotes);
    }

    void updateCurrentAsymptotesMultipliers(Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tPreviousControls = aDataMng.getPreviousControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tAntepenultimateControls = aDataMng.getAntepenultimateControls();

        const OrdinalType tNumVectors = tCurrentControls.getNumVectors();
        for (OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = tCurrentControls[tVectorIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                const ScalarType tMeasure = (tCurrentControls(tVectorIndex, tControlIndex) - tPreviousControls(tVectorIndex, tControlIndex))
                    * (tPreviousControls(tVectorIndex, tControlIndex) - tAntepenultimateControls(tVectorIndex, tControlIndex));
                ScalarType tGammaValue = tMeasure > static_cast<ScalarType>(0) ? mAsymptoteExpansion : mAsymptoteContraction;
                (*mCurrentAsymptotesMultipliers)(tVectorIndex, tControlIndex) =
                        std::abs(tGammaValue) <= std::numeric_limits<ScalarType>::min() ? static_cast<ScalarType>(1) : tGammaValue;
            }
        }
    }

    void updateCurrentAsymptotes(Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tPreviousControls = aDataMng.getPreviousControls();

        const OrdinalType tNumVectors = tCurrentControls.getNumVectors();
        for (OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumControls = tCurrentControls[tVectorIndex].size();
            for (OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
            {
                tLowerAsymptotes(tVectorIndex, tControlIndex) = tCurrentControls(tVectorIndex, tControlIndex)
                    - ((*mCurrentAsymptotesMultipliers)(tVectorIndex, tControlIndex)
                            * (tPreviousControls(tVectorIndex, tControlIndex) - tLowerAsymptotes(tVectorIndex, tControlIndex)));
                tUpperAsymptotes(tVectorIndex, tControlIndex) = tCurrentControls(tVectorIndex, tControlIndex)
                    + ((*mCurrentAsymptotesMultipliers)(tVectorIndex, tControlIndex)
                            * (tUpperAsymptotes(tVectorIndex, tControlIndex) - tPreviousControls(tVectorIndex, tControlIndex)));
            }
        }
    }

    void updateObjectiveApproximationFunctionData(Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        ScalarType tNormalizationValue = std::abs(aDataMng.getCurrentObjectiveValue());
        if(tNormalizationValue <= std::numeric_limits<ScalarType>::epsilon())
        {
            tNormalizationValue = 1.0;
        }

        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentObjGrad = aDataMng.getCurrentObjectiveGradient();

        Plato::MultiVector<ScalarType, OrdinalType> &tAppxFunctionP = aDataMng.getObjFuncAppxFunctionP();
        Plato::MultiVector<ScalarType, OrdinalType> &tAppxFunctionQ = aDataMng.getObjFuncAppxFunctionQ();
        this->computeApproxFuncCoefficients(tNormalizationValue,
                                            tCurrentControls,
                                            tLowerAsymptotes,
                                            tUpperAsymptotes,
                                            tCurrentObjGrad,
                                            tAppxFunctionP,
                                            tAppxFunctionQ);
    }

    void updateConstraintApproximationFunctionsData(Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();

        const OrdinalType tNumConstraints = aDataMng.getNumConstraints();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const ScalarType tConstraintNormalization = aDataMng.getConstraintNormalization(tIndex);
            const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentConstraintGrad = aDataMng.getCurrentConstraintGradient(tIndex);
            Plato::MultiVector<ScalarType, OrdinalType> &tMyAppxFunctionP = aDataMng.getConstraintAppxFunctionP(tIndex);
            Plato::MultiVector<ScalarType, OrdinalType> &tMyAppxFunctionQ = aDataMng.getConstraintAppxFunctionQ(tIndex);
            this->computeApproxFuncCoefficients(tConstraintNormalization,
                                                tCurrentControls,
                                                tLowerAsymptotes,
                                                tUpperAsymptotes,
                                                tCurrentConstraintGrad,
                                                tMyAppxFunctionP,
                                                tMyAppxFunctionQ);
        }
    }

    void updateSubProblemBounds(Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>& aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> &tCurrentControls = aDataMng.getCurrentControls();
        const Plato::MultiVector<ScalarType, OrdinalType> &tLowerAsymptotes = aDataMng.getLowerAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tUpperAsymptotes = aDataMng.getUpperAsymptotes();
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlLowerBounds = aDataMng.getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> &tControlUpperBounds = aDataMng.getControlUpperBounds();

        Plato::MultiVector<ScalarType, OrdinalType> &tSubProblemLowerBounds = aDataMng.getSubProblemControlLowerBounds();
        Plato::MultiVector<ScalarType, OrdinalType> &tSubProblemUpperBounds = aDataMng.getSubProblemControlUpperBounds();

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

private:
    void computeApproxFuncCoefficients(const ScalarType &aNormalization,
                                       const Plato::MultiVector<ScalarType, OrdinalType> &aCurrentControls,
                                       const Plato::MultiVector<ScalarType, OrdinalType> &aLowerAsymptotes,
                                       const Plato::MultiVector<ScalarType, OrdinalType> &aUpperAsymptotes,
                                       const Plato::MultiVector<ScalarType, OrdinalType> &aCriterionGrad,
                                       Plato::MultiVector<ScalarType, OrdinalType> &aAppxFunctionP,
                                       Plato::MultiVector<ScalarType, OrdinalType> &aAppxFunctionQ)
    {
        const ScalarType tAbsNormalization = std::abs(aNormalization);
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
                ScalarType tUpperAsymmMinusCurrentControlSquared = aUpperAsymptotes(tVectorIndex, tControlIndex)
                    - aCurrentControls(tVectorIndex, tControlIndex);
                tUpperAsymmMinusCurrentControlSquared *= tUpperAsymmMinusCurrentControlSquared;
                aAppxFunctionP(tVectorIndex, tControlIndex) *= tUpperAsymmMinusCurrentControlSquared;

                aAppxFunctionQ(tVectorIndex, tControlIndex) = (mApproxFuncScalingTwo * tGradValuePlus) + (mApproxFuncScalingOne * tGradValueMinus)
                    + (mApproxFuncEpsilon / (*mUpperMinusLowerBounds)(tVectorIndex, tControlIndex));
                ScalarType tCurrentControlMinusLowerAsymmSquared = aCurrentControls(tVectorIndex, tControlIndex)
                    - aLowerAsymptotes(tVectorIndex, tControlIndex);
                tCurrentControlMinusLowerAsymmSquared *= tCurrentControlMinusLowerAsymmSquared;
                aAppxFunctionQ(tVectorIndex, tControlIndex) *= tCurrentControlMinusLowerAsymmSquared;
            }
        }
    }

private:
    ScalarType mMoveLimit;
    ScalarType mApproxFuncEpsilon;
    ScalarType mAsymptoteExpansion;
    ScalarType mAsymptoteContraction;
    ScalarType mApproxFuncScalingOne;
    ScalarType mApproxFuncScalingTwo;
    ScalarType mInitialAymptoteScaling;
    ScalarType mSubProblemBoundsScaling;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperMinusLowerBounds;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentAsymptotesMultipliers;
};

template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesNew
{
public:
    MethodMovingAsymptotesNew(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> &aObjective,
                           const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> &aConstraints,
                           const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mIterationCount(0),
        mMaxNumIterations(500),
        mProblemUpdateFrequency(0),
        mNumTrustRegionIterations(50),
        mInitialAugLagPenalty(1),
        mAugLagPenaltyReduction(1.1),
        mControlStagnationTol(1e-6),
        mFeasibilityTolerance(1e-4),
        mStoppingCriterion(Plato::algorithm::stop_t::NOT_CONVERGED),
        mObjective(aObjective),
        mConstraints(aConstraints),
        mMMAData(std::make_shared<Plato::ApproximationFunctionData<ScalarType, OrdinalType>>(aDataFactory)),
        mDataMng(std::make_shared<Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>>(aDataFactory)),
        mOperations(std::make_shared<Plato::MethodMovingAsymptotesOperations<ScalarType, OrdinalType>>(aDataFactory)),
        mConstrAppxFuncList(std::make_shared<Plato::CriterionList<ScalarType, OrdinalType>>())
    {
        this->initialize(aDataFactory);
    }

    void setMaxNumIterations(const OrdinalType& aInput)
    {
        mMaxNumIterations = aInput;
    }

    void setProblemUpdateFrequency(const OrdinalType& aInput)
    {
        mProblemUpdateFrequency = aInput;
    }

    void setNumTrustRegionIterations(const OrdinalType& aInput)
    {
        mNumTrustRegionIterations = aInput;
    }

    void setMoveLimit(const ScalarType& aInput)
    {
        mOperations->setMoveLimit(aInput);
    }

    void setAsymptoteExpansion(const ScalarType& aInput)
    {
        mOperations->setAsymptoteExpansion(aInput);
    }

    void setAsymptoteContraction(const ScalarType& aInput)
    {
        mOperations->setAsymptoteContraction(aInput);
    }

    void setInitialAymptoteScaling(const ScalarType& aInput)
    {
        mOperations->setInitialAymptoteScaling(aInput);
    }


    void setInitialAugLagPenalty(const ScalarType& aInput)
    {
        mInitialAugLagPenalty = aInput;
    }

    void setAugLagPenaltyReduction(const ScalarType& aInput)
    {
        mAugLagPenaltyReduction = aInput;
    }

    void setFeasibilityTolerance(const ScalarType& aInput)
    {
        mFeasibilityTolerance = aInput;
    }

    void setControlLowerBounds(const Plato::MultiVector<ScalarType, OrdinalType> &aInput)
    {
        mDataMng->setControlLowerBounds(aInput);
    }

    void setControlUpperBounds(const Plato::MultiVector<ScalarType, OrdinalType> &aInput)
    {
        mDataMng->setControlUpperBounds(aInput);
    }

    void setInitialGuess(const Plato::MultiVector<ScalarType, OrdinalType> &aInput)
    {
        mDataMng->setCurrentControls(aInput);
    }

    void setConstraintNormalization(const OrdinalType & aIndex, const ScalarType & aValue)
    {
        mDataMng->setConstraintNormalization(aIndex, aValue);
    }

    void getSolution(Plato::MultiVector<ScalarType, OrdinalType>& aInput) const
    {
        mSubProblemSolver->getSolution(aInput);
    }

    OrdinalType getNumIterations() const
    {
        return mIterationCount;
    }

    ScalarType getOptimalObjectiveValue() const
    {
        return (mDataMng->getCurrentObjectiveValue());
    }

    ScalarType getOptimalConstraintValue(const OrdinalType& aIndex) const
    {
        return (mDataMng->getCurrentConstraintValue(aIndex));
    }

    void solve()
    {
        mOperations->initialize(*mDataMng);
        while(true)
        {
            this->updateState();
            this->updateSubProblem();
            this->solveSubProblem();

            mIterationCount++;
            bool tStop = this->checkStoppingCriteria();
            if(tStop == true)
            {
                this->updateState();
                break;
            }
        }
    }

private:
    void initialize(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory)
    {
        this->initializeApproximationFunctions(aDataFactory);
        this->initializeSubProblemSolver(aDataFactory);
    }

    void initializeSubProblemSolver(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory)
    {
        mSubProblemSolver = std::make_shared<Plato::AugmentedLagrangian<ScalarType, OrdinalType>>(mObjAppxFunc, mConstrAppxFuncList, aDataFactory);
        mSubProblemSolver->disablePostSmoothing();
        mSubProblemSolver->setPenaltyParameter(mInitialAugLagPenalty);
        mSubProblemSolver->setFeasibilityTolerance(mFeasibilityTolerance);
        mSubProblemSolver->setControlStagnationTolerance(mControlStagnationTol);
        mSubProblemSolver->setPenaltyParameterScaleFactor(mAugLagPenaltyReduction);
        mSubProblemSolver->setMaxNumTrustRegionSubProblemIterations(mNumTrustRegionIterations);
    }

    void initializeApproximationFunctions(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory)
    {
        mObjAppxFunc = std::make_shared<Plato::MethodMovingAsymptotesNewCriterion<ScalarType, OrdinalType>>(aDataFactory);
        const OrdinalType tNumConstraints = mDataMng->getNumConstraints();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            mConstrAppxFuncs.push_back(std::make_shared<Plato::MethodMovingAsymptotesNewCriterion<ScalarType, OrdinalType>>(aDataFactory));
            mConstrAppxFuncList->add(mConstrAppxFuncs[tIndex]);
        }
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
        mOperations->updateSubProblemBounds(*mDataMng);
        mOperations->updateObjectiveApproximationFunctionData(*mDataMng);
        mOperations->updateConstraintApproximationFunctionsData(*mDataMng);
        this->updateObjectiveApproximationFunction();
        this->updateConstraintApproximationFunctions();
    }

    void updateState()
    {
        this->evaluateObjective();
        this->evaluateConstraints();
        this->performContinuation();
        mDataMng->cacheState();
        this->printDiagnostics();
    }

    void printDiagnostics()
    {
        const Plato::CommWrapper &tComm = mDataMng->getCommWrapper();
        if (tComm.myProcID() == 0)
        {
            std::cout << "iteration = " << mIterationCount << ", objective = " << mDataMng->getCurrentObjectiveValue() << ", constraint = "
                << mDataMng->getCurrentConstraintValue(0) << "\n";
        }
    }

    void performContinuation()
    {
        const bool tIsContinuationEnabled = mProblemUpdateFrequency > static_cast<OrdinalType>(0);
        bool tPerformContinuation = tIsContinuationEnabled ? (mIterationCount % mProblemUpdateFrequency) == static_cast<OrdinalType>(0) : false;
        if (tPerformContinuation)
        {
            mObjective->updateProblem(mDataMng->getCurrentControls());
            const OrdinalType tNumConstraints = mDataMng->getNumConstraints();
            for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
            {
                (*mConstraints)[tIndex].updateProblem(mDataMng->getCurrentControls());
            }
        }
    }

    void updateAsymptotes()
    {
        if(mIterationCount >= static_cast<OrdinalType>(2))
        {
            mOperations->updateCurrentAsymptotesMultipliers(*mDataMng);
            mOperations->updateCurrentAsymptotes(*mDataMng);
        }
        else
        {
            mOperations->updateInitialAsymptotes(*mDataMng);
        }
    }

    void updateObjectiveApproximationFunction()
    {
        mMMAData->mCurrentNormalizedCriterionValue = 1;
        Plato::update(static_cast<ScalarType>(1), mDataMng->getCurrentControls(), static_cast<ScalarType>(0), *mMMAData->mCurrentControls);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getLowerAsymptotes(), static_cast<ScalarType>(0), *mMMAData->mLowerAsymptotes);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getUpperAsymptotes(), static_cast<ScalarType>(0), *mMMAData->mUpperAsymptotes);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getObjFuncAppxFunctionP(), static_cast<ScalarType>(0), *mMMAData->mAppxFunctionP);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getObjFuncAppxFunctionQ(), static_cast<ScalarType>(0), *mMMAData->mAppxFunctionQ);

        mObjAppxFunc->update(*mMMAData);
    }

    void updateConstraintApproximationFunctions()
    {
        Plato::update(static_cast<ScalarType>(1), mDataMng->getCurrentControls(), static_cast<ScalarType>(0), *mMMAData->mCurrentControls);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getLowerAsymptotes(), static_cast<ScalarType>(0), *mMMAData->mLowerAsymptotes);
        Plato::update(static_cast<ScalarType>(1), mDataMng->getUpperAsymptotes(), static_cast<ScalarType>(0), *mMMAData->mUpperAsymptotes);

        const OrdinalType tNumConstraints = mDataMng->getNumConstraints();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const ScalarType tConstraintValue = mDataMng->getCurrentConstraintValue(tIndex);
            const ScalarType tConstraintNormalization = mDataMng->getConstraintNormalization(tIndex);
            mMMAData->mCurrentNormalizedCriterionValue = tConstraintValue / tConstraintNormalization;
            Plato::update(static_cast<ScalarType>(1), mDataMng->getConstraintAppxFunctionP(tIndex), static_cast<ScalarType>(0), *mMMAData->mAppxFunctionP);
            Plato::update(static_cast<ScalarType>(1), mDataMng->getConstraintAppxFunctionQ(tIndex), static_cast<ScalarType>(0), *mMMAData->mAppxFunctionQ);
            mConstrAppxFuncs[tIndex]->update(*mMMAData);
        }
    }

    void solveSubProblem()
    {
        mSubProblemSolver->resetParameters();
        mSubProblemSolver->setInitialGuess(mDataMng->getCurrentControls());
        mSubProblemSolver->setControlLowerBounds(mDataMng->getSubProblemControlLowerBounds());
        mSubProblemSolver->setControlUpperBounds(mDataMng->getSubProblemControlUpperBounds());
        mSubProblemSolver->solve();
        mSubProblemSolver->getSolution(mDataMng->getCurrentControls());
    }

    /******************************************************************************//**
     * @brief Check stopping criteria
     * @return stopping criterion met, yes or no
    **********************************************************************************/
    bool checkStoppingCriteria()
    {
        bool tStop = false;
        mDataMng->computeControlStagnationMeasure();
        const ScalarType tControlStagnation = mDataMng->getControlStagnationMeasure();

        if(mIterationCount == mMaxNumIterations)
        {
            mStoppingCriterion = Plato::algorithm::MAX_NUMBER_ITERATIONS;
            tStop = true;
        }
        else if(std::abs(tControlStagnation) <= mControlStagnationTol)
        {
            mStoppingCriterion = Plato::algorithm::CONTROL_STAGNATION;
            tStop = true;
        }

        return (tStop);
    }

private:
    OrdinalType mIterationCount; /*!< number of optimization iterations */
    OrdinalType mMaxNumIterations; /*!< maximum number of optimization iterations */
    OrdinalType mProblemUpdateFrequency; /*!< problem update, i.e. continuation, frequency */
    OrdinalType mNumTrustRegionIterations; /*!< maximum number of trust region subproblem iterations */

    ScalarType mInitialAugLagPenalty; /*!< initial augmented Lagragian penalty parameter */
    ScalarType mAugLagPenaltyReduction; /*!< augmented Lagragian penalty reduction parameter */
    ScalarType mControlStagnationTol; /*!< control stagnation tolerance - secondary stopping tolerance */
    ScalarType mFeasibilityTolerance; /*!< feasibility tolerance */
    Plato::algorithm::stop_t mStoppingCriterion; /*!< stopping criterion */

    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mObjective; /*!< objective criterion interface */
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> mConstraints; /*!< constraint criteria interface */

    std::shared_ptr<Plato::ApproximationFunctionData<ScalarType, OrdinalType>> mMMAData; /*!< structure with approximation function's data */
    std::shared_ptr<Plato::AugmentedLagrangian<ScalarType, OrdinalType>> mSubProblemSolver; /*!< MMA subproblem solver */
    std::shared_ptr<Plato::MethodMovingAsymptotesNewDataMng<ScalarType, OrdinalType>> mDataMng; /*!< MMA data manager */
    std::shared_ptr<Plato::MethodMovingAsymptotesOperations<ScalarType, OrdinalType>> mOperations; /*!< interface to MMA core operations */

    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> mConstrAppxFuncList; /*!< list of constraint criteria */
    std::shared_ptr<Plato::MethodMovingAsymptotesNewCriterion<ScalarType, OrdinalType>> mObjAppxFunc; /*!< objective criterion approximation function */
    std::vector<std::shared_ptr<Plato::MethodMovingAsymptotesNewCriterion<ScalarType, OrdinalType>>> mConstrAppxFuncs; /*!< list of constraint criteria approximation function */
};
// class MethodMovingAsymptotesNew

}
// namespace Plato

namespace PlatoTest
{

TEST(PlatoTest, MethodMovingAsymptotesNewCriterion)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    Plato::ApproximationFunctionData<double> tAppxFuncCoreData(tDataFactory);
    Plato::fill(0.1, *tAppxFuncCoreData.mAppxFunctionP);
    Plato::fill(0.11, *tAppxFuncCoreData.mAppxFunctionQ);
    Plato::fill(1.0, *tAppxFuncCoreData.mCurrentControls);
    Plato::fill(5.0, *tAppxFuncCoreData.mUpperAsymptotes);
    Plato::fill(-5.0, *tAppxFuncCoreData.mLowerAsymptotes);
    tAppxFuncCoreData.mCurrentNormalizedCriterionValue = 1;

    Plato::MethodMovingAsymptotesNewCriterion<double> tAppxFunction(tDataFactory);
    tAppxFunction.update(tAppxFuncCoreData);

    // TEST GRADIENT
    std::ostringstream tGradDiagnosticMsg;
    Plato::Diagnostics<double> tDiagnostics;
    ASSERT_NO_THROW(tDiagnostics.checkCriterionGradient(tAppxFunction, *tAppxFuncCoreData.mCurrentControls, tGradDiagnosticMsg));
    ASSERT_TRUE(tDiagnostics.didGradientTestPassed());
    Plato::CommWrapper tComm(MPI_COMM_WORLD);
    if(tComm.myProcID() == static_cast<int>(0))
    {
        std::cout << tGradDiagnosticMsg.str().c_str();
    }

    // TEST HESSIAN
    std::ostringstream tHessDiagnosticMsg;
    ASSERT_NO_THROW(tDiagnostics.checkCriterionHessian(tAppxFunction, *tAppxFuncCoreData.mCurrentControls, tHessDiagnosticMsg));
    ASSERT_TRUE(tDiagnostics.didHessianTestPassed());
    if(tComm.myProcID() == static_cast<int>(0))
    {
        std::cout << tHessDiagnosticMsg.str().c_str();
    }
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_initialize)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CALL FUNCTION
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);

    // TEST OUTPUT
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.75);
    tOperations.getUpperMinusLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tData, tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_updateInitialAsymptotes)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CALL FUNCTION
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateInitialAsymptotes(tDataMng);

    // TEST OUTPUT
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.625);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tGold);
    Plato::fill(1.375, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_updateCurrentAsymptotesMultipliers)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.75, tData);
    tDataMng.setPreviousControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousControls(), tData);
    Plato::fill(0.65, tData);
    tDataMng.setAntepenultimateControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getAntepenultimateControls(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateCurrentAsymptotesMultipliers(tDataMng);

    // TEST OUTPUT
    tOperations.getCurrentAsymptotesMultipliers(tData);
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 1.2);
    PlatoTest::checkMultiVectorData(tData, tGold);

    // CASE 2: CONTRACTION PARAMETER IS CHOSEN
    Plato::fill(0.55, tData);
    tDataMng.setPreviousControls(tData);
    tOperations.updateCurrentAsymptotesMultipliers(tDataMng);

    // TEST OUTPUT
    tOperations.getCurrentAsymptotesMultipliers(tData);
    Plato::fill(0.7, tGold);
    PlatoTest::checkMultiVectorData(tData, tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_updateCurrentAsymptotes)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(0.625, tData);
    tDataMng.setLowerAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tData);
    Plato::fill(1.375, tData);
    tDataMng.setUpperAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tData);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.75, tData);
    tDataMng.setPreviousControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getPreviousControls(), tData);
    Plato::fill(0.65, tData);
    tDataMng.setAntepenultimateControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getAntepenultimateControls(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateCurrentAsymptotesMultipliers(tDataMng);
    tOperations.updateCurrentAsymptotes(tDataMng);

    // TEST OUTPUT
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.85);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tGold);
    Plato::fill(1.75, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_updateSubProblemBounds)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // SET DATA
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(0.625, tData);
    tDataMng.setLowerAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tData);
    Plato::fill(1.375, tData);
    tDataMng.setUpperAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tData);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateSubProblemBounds(tDataMng);

    // TEST OUTPUT
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.6625);
    PlatoTest::checkMultiVectorData(tDataMng.getSubProblemControlLowerBounds(), tGold);
    Plato::fill(1., tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getSubProblemControlUpperBounds(), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_updateObjectiveApproximationFunctionData)
{
    const size_t tNumControls = 5;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateControl(tNumControls);

    // CREATE MEANINGFUL CRITERION
    Plato::CcsaTestObjective<double> tCriterion;
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(1.0, tData);
    double tObjFuncValue = tCriterion.value(tData);
    Plato::StandardMultiVector<double> tGradient(1 /* number of vectors */, tNumControls);
    tCriterion.gradient(tData, tGradient);

    // SET DATA
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    tDataMng.setCurrentObjectiveValue(tObjFuncValue);
    const double tTolerance = 1e-6;
    ASSERT_NEAR(0.312, tDataMng.getCurrentObjectiveValue(), tTolerance);
    tDataMng.setCurrentObjectiveGradient(tGradient);
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.0624 /* initial value */);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentObjectiveGradient(), tGold);

    Plato::fill(0.625, tData);
    tDataMng.setLowerAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tData);
    Plato::fill(1.375, tData);
    tDataMng.setUpperAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tData);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateObjectiveApproximationFunctionData(tDataMng);

    // TEST OUTPUT
    Plato::fill(0.028155, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getObjFuncAppxFunctionP(), tGold);
    Plato::fill(3e-5, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getObjFuncAppxFunctionQ(), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotesNewOperations_updateConstraintApproximationFunctionsData)
{
    const size_t tNumControls = 5;
    const size_t tNumConstraints = 2;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateDual(tNumConstraints);
    tDataFactory->allocateControl(tNumControls);

    // CREATE MEANINGFUL CRITERION
    Plato::CcsaTestObjective<double> tCriterion;
    Plato::MethodMovingAsymptotesNewDataMng<double> tDataMng(tDataFactory);
    ASSERT_EQ(2u, tDataMng.getNumConstraints());
    Plato::StandardMultiVector<double> tData(1 /* number of vectors */, tNumControls);
    Plato::fill(1.0, tData);

    // SET DATA FOR CONSTRAINT 1
    size_t tContraintIndex = 0;
    double tConstraintValue = tCriterion.value(tData);
    tDataMng.setCurrentConstraintValue(tContraintIndex, tConstraintValue);
    tDataMng.setConstraintNormalization(tContraintIndex, 1);
    const double tTolerance = 1e-6;
    ASSERT_NEAR(0.312, tDataMng.getCurrentConstraintValue(tContraintIndex), tTolerance);

    Plato::StandardMultiVector<double> tConstraintGradient(1 /* number of vectors */, tNumControls);
    tCriterion.gradient(tData, tConstraintGradient);
    tDataMng.setCurrentConstraintGradient(tContraintIndex, tConstraintGradient);
    Plato::StandardMultiVector<double> tGold(1 /* number of vectors */, tNumControls, 0.0624 /* initial value */);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentConstraintGradient(tContraintIndex), tGold);

    // SET DATA FOR CONSTRAINT 2
    tContraintIndex = 1;
    tCriterion.setWeightConstant(0.624);
    tConstraintValue = tCriterion.value(tData);
    tDataMng.setConstraintNormalization(tContraintIndex, 1);
    tDataMng.setCurrentConstraintValue(tContraintIndex, tConstraintValue);
    ASSERT_NEAR(3.12, tDataMng.getCurrentConstraintValue(tContraintIndex), tTolerance);

    tCriterion.gradient(tData, tConstraintGradient);
    tDataMng.setCurrentConstraintGradient(tContraintIndex, tConstraintGradient);
    Plato::fill(0.624, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentConstraintGradient(tContraintIndex), tGold);

    // SET OTHER DATA
    Plato::fill(0.625, tData);
    tDataMng.setLowerAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getLowerAsymptotes(), tData);
    Plato::fill(1.375, tData);
    tDataMng.setUpperAsymptotes(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getUpperAsymptotes(), tData);
    Plato::fill(1.0, tData);
    tDataMng.setCurrentControls(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getCurrentControls(), tData);
    Plato::fill(0.25, tData);
    tDataMng.setControlLowerBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlLowerBounds(), tData);
    Plato::fill(1., tData);
    tDataMng.setControlUpperBounds(tData);
    PlatoTest::checkMultiVectorData(tDataMng.getControlUpperBounds(), tData);

    // CASE 1: EXPANSION PARAMETER IS CHOSEN
    Plato::MethodMovingAsymptotesOperations<double> tOperations(tDataFactory);
    tOperations.initialize(tDataMng);
    tOperations.updateConstraintApproximationFunctionsData(tDataMng);

    // TEST OUTPUT FOR FIRST CONSTRAINT
    tContraintIndex = 0;
    Plato::fill(0.00878565, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getConstraintAppxFunctionP(tContraintIndex), tGold);
    Plato::fill(0.00001065, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getConstraintAppxFunctionQ(tContraintIndex), tGold);

    // TEST OUTPUT FOR SECOND CONSTRAINT
    tContraintIndex = 1;
    Plato::fill(0.087839625, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getConstraintAppxFunctionP(tContraintIndex), tGold);
    Plato::fill(0.000089625, tGold);
    PlatoTest::checkMultiVectorData(tDataMng.getConstraintAppxFunctionQ(tContraintIndex), tGold);
}

TEST(PlatoTest, MethodMovingAsymptotes_5Bars)
{
    // ********* SET DATA FACTORY *********
    const size_t tNumControls = 5;
    const size_t tNumConstraints = 1;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateDual(tNumConstraints);
    tDataFactory->allocateControl(tNumControls);

    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::CcsaTestObjective<double>> tObjective = std::make_shared<Plato::CcsaTestObjective<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    std::shared_ptr<Plato::CcsaTestInequality<double>> tConstraint = std::make_shared<Plato::CcsaTestInequality<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::MethodMovingAsymptotesNew<double> tAlgorithm(tObjective, tConstraintList, tDataFactory);
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tData(tNumVectors, tNumControls, 5.0 /* values */);
    tAlgorithm.setInitialGuess(tData);
    Plato::fill(1.0, tData);
    tAlgorithm.setControlLowerBounds(tData);
    Plato::fill(10.0, tData);
    tAlgorithm.setControlUpperBounds(tData);
    tAlgorithm.setConstraintNormalization(0, 1 /* upper bound */);
    tAlgorithm.solve();

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(27u, tAlgorithm.getNumIterations());
    ASSERT_NEAR(1.34, tAlgorithm.getOptimalObjectiveValue(), tTolerance);
    ASSERT_TRUE(std::abs(tAlgorithm.getOptimalConstraintValue(0)) < 1e-4);
    tAlgorithm.getSolution(tData);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 6.00792; tGold(0,1) = 5.3093769; tGold(0,2) = 4.4977; tGold(0,3) = 3.5053559; tGold(0,4) = 2.15340156;
    PlatoTest::checkMultiVectorData(tGold, tData);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tAlgorithm.getNumIterations() << "\n" << std::flush;
    const double tBestObjFuncValue = tAlgorithm.getOptimalObjectiveValue();
    std::cout << "BEST OBJECTIVE VALUE = " << tBestObjFuncValue << "\n" << std::flush;
    const double tBestConstraint = tAlgorithm.getOptimalConstraintValue(0);
    std::cout << "BEST CONSTRAINT VALUE = " << tBestConstraint << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(tData);
}

TEST(PlatoTest, MethodMovingAsymptotes_RosenbrockRadius)
{
    // ********* SET DATA FACTORY *********
    const size_t tNumControls = 2;
    const size_t tNumConstraints = 1;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateDual(tNumConstraints);
    tDataFactory->allocateControl(tNumControls);

    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::Rosenbrock<double>> tObjective = std::make_shared<Plato::Rosenbrock<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    std::shared_ptr<Plato::Radius<double>> tConstraint = std::make_shared<Plato::Radius<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::MethodMovingAsymptotesNew<double> tAlgorithm(tObjective, tConstraintList, tDataFactory);
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tData(tNumVectors, tNumControls, 1 /* values */);
    tAlgorithm.setInitialGuess(tData);
    Plato::fill(0.0, tData);
    tAlgorithm.setControlLowerBounds(tData);
    Plato::fill(2.0, tData);
    tAlgorithm.setControlUpperBounds(tData);
    tAlgorithm.setConstraintNormalization(0, 2 /* upper bound - used for normalization of constraint appx function */);
    tAlgorithm.setMoveLimit(0.05);
    tAlgorithm.solve();

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(35u, tAlgorithm.getNumIterations());
    ASSERT_NEAR(0.0456733, tAlgorithm.getOptimalObjectiveValue(), tTolerance);
    ASSERT_TRUE(std::abs(tAlgorithm.getOptimalConstraintValue(0)) < 1e-4);
    tAlgorithm.getSolution(tData);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 0.786486; tGold(0,1) = 0.617638;
    PlatoTest::checkMultiVectorData(tGold, tData);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tAlgorithm.getNumIterations() << "\n" << std::flush;
    const double tBestObjFuncValue = tAlgorithm.getOptimalObjectiveValue();
    std::cout << "BEST OBJECTIVE VALUE = " << tBestObjFuncValue << "\n" << std::flush;
    const double tBestConstraint = tAlgorithm.getOptimalConstraintValue(0);
    std::cout << "BEST CONSTRAINT VALUE = " << tBestConstraint << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(tData);
}

TEST(PlatoTest, MethodMovingAsymptotes_HimmelblauShiftedEllipse)
{
    // ********* SET DATA FACTORY *********
    const size_t tNumControls = 2;
    const size_t tNumConstraints = 1;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateDual(tNumConstraints);
    tDataFactory->allocateControl(tNumControls);

    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::Himmelblau<double>> tObjective = std::make_shared<Plato::Himmelblau<double>>();
    std::shared_ptr<Plato::ShiftedEllipse<double>> tConstraint = std::make_shared<Plato::ShiftedEllipse<double>>();
    tConstraint->specify(-2., 2., -3., 3.);
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::MethodMovingAsymptotesNew<double> tAlgorithm(tObjective, tConstraintList, tDataFactory);
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tData(tNumVectors, tNumControls, -2 /* values */);
    tAlgorithm.setInitialGuess(tData);
    Plato::fill(-5.0, tData);
    tAlgorithm.setControlLowerBounds(tData);
    Plato::fill(-1.0, tData);
    tAlgorithm.setControlUpperBounds(tData);
    tAlgorithm.setConstraintNormalization(0, 0.5);
    tAlgorithm.solve();

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(26u, tAlgorithm.getNumIterations());
    ASSERT_NEAR(11.804, tAlgorithm.getOptimalObjectiveValue(), tTolerance);
    ASSERT_TRUE(std::abs(tAlgorithm.getOptimalConstraintValue(0)) < 1e-4);
    tAlgorithm.getSolution(tData);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = -3.99832678; tGold(0,1) = -2.878447094;
    PlatoTest::checkMultiVectorData(tGold, tData);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tAlgorithm.getNumIterations() << "\n" << std::flush;
    const double tBestObjFuncValue = tAlgorithm.getOptimalObjectiveValue();
    std::cout << "BEST OBJECTIVE VALUE = " << tBestObjFuncValue << "\n" << std::flush;
    const double tBestConstraint = tAlgorithm.getOptimalConstraintValue(0);
    std::cout << "BEST CONSTRAINT VALUE = " << tBestConstraint << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(tData);
}

TEST(PlatoTest, MethodMovingAsymptotes_GoldsteinPriceShiftedEllipse)
{
    // ********* SET DATA FACTORY *********
    const size_t tNumControls = 2;
    const size_t tNumConstraints = 1;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateDual(tNumConstraints);
    tDataFactory->allocateControl(tNumControls);

    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::GoldsteinPrice<double>> tObjective = std::make_shared<Plato::GoldsteinPrice<double>>();
    std::shared_ptr<Plato::ShiftedEllipse<double>> tConstraint = std::make_shared<Plato::ShiftedEllipse<double>>();
    tConstraint->specify(0., 1., .5, 1.5);
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::MethodMovingAsymptotesNew<double> tAlgorithm(tObjective, tConstraintList, tDataFactory);
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tData(tNumVectors, tNumControls, -0.4 /* values */);
    tAlgorithm.setInitialGuess(tData);
    Plato::fill(-3.0, tData);
    tAlgorithm.setControlLowerBounds(tData);
    Plato::fill(0.0, tData);
    tAlgorithm.setControlUpperBounds(tData);
    tAlgorithm.setConstraintNormalization(0, 0.5);
    tAlgorithm.solve();

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(18u, tAlgorithm.getNumIterations());
    ASSERT_NEAR(3, tAlgorithm.getOptimalObjectiveValue(), tTolerance);
    ASSERT_TRUE(std::abs(tAlgorithm.getOptimalConstraintValue(0)) < 5e-4);
    tAlgorithm.getSolution(tData);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 0.0; tGold(0,1) = -1.0;
    PlatoTest::checkMultiVectorData(tGold, tData, tTolerance);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tAlgorithm.getNumIterations() << "\n" << std::flush;
    const double tBestObjFuncValue = tAlgorithm.getOptimalObjectiveValue();
    std::cout << "BEST OBJECTIVE VALUE = " << tBestObjFuncValue << "\n" << std::flush;
    const double tBestConstraint = tAlgorithm.getOptimalConstraintValue(0);
    std::cout << "BEST CONSTRAINT VALUE = " << tBestConstraint << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(tData);
}

TEST(PlatoTest, MethodMovingAsymptotes_CircleRadius)
{
    // ********* SET DATA FACTORY *********
    const size_t tNumControls = 2;
    const size_t tNumConstraints = 1;
    std::shared_ptr<Plato::DataFactory<double>> tDataFactory = std::make_shared<Plato::DataFactory<double>>();
    tDataFactory->allocateDual(tNumConstraints);
    tDataFactory->allocateControl(tNumControls);

    // ********* SET OBJECTIVE AND COSNTRAINT *********
    std::shared_ptr<Plato::Circle<double>> tObjective = std::make_shared<Plato::Circle<double>>();
    std::shared_ptr<Plato::Radius<double>> tConstraint = std::make_shared<Plato::Radius<double>>();
    std::shared_ptr<Plato::CriterionList<double>> tConstraintList = std::make_shared<Plato::CriterionList<double>>();
    tConstraintList->add(tConstraint);

    // ********* SOLVE OPTIMIZATION PROBLEM *********
    Plato::MethodMovingAsymptotesNew<double> tAlgorithm(tObjective, tConstraintList, tDataFactory);
    const size_t tNumVectors = 1;
    Plato::StandardMultiVector<double> tData(tNumVectors, tNumControls, 0.5 /* values */);
    tAlgorithm.setInitialGuess(tData);
    Plato::fill(0.0, tData);
    tAlgorithm.setControlLowerBounds(tData);
    Plato::fill(2.0, tData);
    tAlgorithm.setControlUpperBounds(tData);
    tAlgorithm.setConstraintNormalization(0, 0.5);
    tAlgorithm.solve();

    // ********* TEST SOLUTION *********
    const double tTolerance = 1e-4;
    ASSERT_EQ(18u, tAlgorithm.getNumIterations());
    ASSERT_NEAR(2.67798, tAlgorithm.getOptimalObjectiveValue(), tTolerance);
    ASSERT_TRUE(std::abs(tAlgorithm.getOptimalConstraintValue(0)) < 1e-4);
    tAlgorithm.getSolution(tData);
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(0,0) = 0.3129487804; tGold(0,1) = 0.9497764126;
    PlatoTest::checkMultiVectorData(tGold, tData, tTolerance);

    // ********* PRINT SOLUTION *********
    std::cout << "NUMBER OF ITERATIONS = " << tAlgorithm.getNumIterations() << "\n" << std::flush;
    const double tBestObjFuncValue = tAlgorithm.getOptimalObjectiveValue();
    std::cout << "BEST OBJECTIVE VALUE = " << tBestObjFuncValue << "\n" << std::flush;
    const double tBestConstraint = tAlgorithm.getOptimalConstraintValue(0);
    std::cout << "BEST CONSTRAINT VALUE = " << tBestConstraint << "\n" << std::flush;
    std::cout << "SOLUTION\n" << std::flush;
    PlatoTest::printMultiVector(tData);
}

}
