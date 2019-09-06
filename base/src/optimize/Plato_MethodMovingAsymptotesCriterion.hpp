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
 * Plato_MethodMovingAsymptotesCriterion.hpp
 *
 *  Created on: Sep 1, 2019
 */

#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <algorithm>

#include "Plato_Criterion.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Metadata associated with approximation functions, see Svanberg, Krister.
 * "MMA and GCMMA-two methods for nonlinear optimization." vol 1 (2007): 1-15.
**********************************************************************************/
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
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionP; /*!< approximation function */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionQ; /*!< approximation function */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControls; /*!< current optimization variables */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerAsymptotes; /*!< current lower asymptotes */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperAsymptotes; /*!< current upper asymptotes */
};
// struct ApproximationFunctionData

/******************************************************************************//**
 * Criterion interface to the approximation function used to solve the Method of
 * Moving Asymptotes (MMA) subproblem.
 *
 * The approximation function for the MMA subproblem is defined as
 * /f$\sum_{j=1}^{N}\left( \alpha\frac{p_{ij}^{k}}{u_j^k - x_j} + \beta \left(
 *    \frac{q_{ij}^{k}}{x_j - l_j} + r_{i} \right) \right)/f$
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
 * /f$\alpha/f$: positive constant {0}\leq\alpha\leq{1}
 * /f$\beta/f$: positive constant {0}\leq\\beta\leq{1}
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
    /******************************************************************************//**
     * Constructor
     * @param [in] aDataFactory constant reference to the core data factory shared pointer.
    ***********************************************************************************/
    explicit MethodMovingAsymptotesCriterion(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mObjFuncAppxFuncMultiplier(1),
        mConstraintAppxFuncMultiplier(1),
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
    ~MethodMovingAsymptotesCriterion()
    {
    }

    /******************************************************************************//**
     * Set objective approximation function multiplier.
    ***********************************************************************************/
    void setObjFuncAppxFuncMultiplier(const ScalarType& aInput)
    {
        mObjFuncAppxFuncMultiplier = aInput;
    }

    /******************************************************************************//**
     * Set constraint approximation functions multiplier.
    ***********************************************************************************/
    void setConstraintAppxFuncMultiplier(const ScalarType& aInput)
    {
        mConstraintAppxFuncMultiplier = aInput;
    }

    /******************************************************************************//**
     * Safely cache application specific data after a new trial control is accepted.
    ***********************************************************************************/
    void cacheData()
    {
        return;
    }

    /******************************************************************************//**
     * Update approximation functions for the next subproblem iteration
     * @param [in] aData struct with approximation function's data
    ***********************************************************************************/
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
    /******************************************************************************//**
     * Evaluate approximation function
     * @param [in] aControls optimization variables
    ***********************************************************************************/
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
        const ScalarType tOutput = (mObjFuncAppxFuncMultiplier * tTermOne)
            + (mConstraintAppxFuncMultiplier * (mCurrentNormalizedCriterionValue - tTermTwo));

        return (tOutput);
    }

    /******************************************************************************//**
     * Compute approximation function gradient
     * @param [in] aControls optimization variables
     * @param [out] aGradient approximation function gradient
    ***********************************************************************************/
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
                aGradient(tVecIndex, tControlIndex) = (mObjFuncAppxFuncMultiplier * tValueOne) - tValueTwo;
            }
        }
    }

    /******************************************************************************//**
     * Compute approximation function gradient
     * @param [in] aControls optimization variables
     * @param [in] aVector descent direction
     * @param [out] aHessTimesVec vector apply to approximation function's Hessian
    ***********************************************************************************/
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
                tDenominatorValue =
                    std::abs(tDenominatorValue) > std::numeric_limits<ScalarType>::epsilon() ? tDenominatorValue : std::numeric_limits<ScalarType>::epsilon();
                ScalarType tDenominator = tDenominatorValue * tDenominatorValue * tDenominatorValue;
                ScalarType tValueOne = (*mAppxFunctionP)(tVecIndex, tControlIndex) / tDenominator;

                tDenominatorValue = aControls(tVecIndex, tControlIndex) - (*mLowerAsymptotes)(tVecIndex, tControlIndex);
                tDenominatorValue =
                    std::abs(tDenominatorValue) > std::numeric_limits<ScalarType>::epsilon() ? tDenominatorValue : std::numeric_limits<ScalarType>::epsilon();
                tDenominator = tDenominatorValue * tDenominatorValue * tDenominatorValue;
                ScalarType tValueTwo = (*mAppxFunctionQ)(tVecIndex, tControlIndex) / tDenominator;
                aHessTimesVec(tVecIndex, tControlIndex) = static_cast<ScalarType>(2) * ((mObjFuncAppxFuncMultiplier * tValueOne) + tValueTwo)
                    * aVector(tVecIndex, tControlIndex);
            }
        }
    }

private:
    ScalarType mObjFuncAppxFuncMultiplier; /*!< positive constant {0}\leq\alpha\leq{1} */
    ScalarType mConstraintAppxFuncMultiplier; /*!< positive constant {0}\leq\beta\leq{1} */
    ScalarType mCurrentNormalizedCriterionValue; /*!< current normalized objective function value */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWork1; /*!< optimization variables work vector one */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWork2; /*!< optimization variables work vector two */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionP; /*!< approximation function one (P) */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAppxFunctionQ; /*!< approximation function two (Q) */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControls; /*!< current optimization variables */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerAsymptotes; /*!< current lower asymptotes */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperAsymptotes; /*!< current upper asymptotes */

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductionOps; /*!< reduction operation interface for controls */

private:
    MethodMovingAsymptotesCriterion(const Plato::MethodMovingAsymptotesCriterion<ScalarType, OrdinalType> & aRhs);
    Plato::MethodMovingAsymptotesCriterion<ScalarType, OrdinalType> & operator=(const Plato::MethodMovingAsymptotesCriterion<ScalarType, OrdinalType> & aRhs);
};
// class MethodMovingAsymptotesCriterion

}
// namespace Plato
