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
 * Plato_MethodMovingAsymptotesDataMng.hpp
 *
 *  Created on: Sep 1, 2019
 */

#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <algorithm>

#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_MultiVectorList.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Data manager for Method of Moving Asymptotes (MMA) optimization algorithm
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class MethodMovingAsymptotesDataMng
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aDataFactory factory used to create internal data
    **********************************************************************************/
    explicit MethodMovingAsymptotesDataMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> &aDataFactory) :
        mFeasibilityMeasure(std::numeric_limits<ScalarType>::max()),
        mNormObjectiveGradient(std::numeric_limits<ScalarType>::max()),
        mCurrentObjectiveValue(std::numeric_limits<ScalarType>::max()),
        mPreviousObjectiveValue(std::numeric_limits<ScalarType>::max()),
        mControlStagnationMeasure(std::numeric_limits<ScalarType>::max()),
        mObjectiveStagnationMeasure(std::numeric_limits<ScalarType>::max()),
        mCurrentNormalizedObjectiveValue(std::numeric_limits<ScalarType>::max()),
        mControlWork(aDataFactory->control(0 /* vector index */).create()),
        mConstraintWork(aDataFactory->dual(0 /* vector index */).create()),
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
        mDualReductionOps(aDataFactory->getDualReductionOperations().create()),
        mControlReductionOps(aDataFactory->getControlReductionOperations().create())
    {
        this->initialize(*aDataFactory);
    }
    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~MethodMovingAsymptotesDataMng()
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

    /******************************************************************************//**
     * @brief Return number of constraints
     * @return number of constraints
    **********************************************************************************/
    OrdinalType getNumConstraints() const
    {
        const OrdinalType tNumConstraints = mCurrentConstraintValues->size();
        return tNumConstraints;
    }

    /******************************************************************************//**
     * @brief Return feasibility measure
     * @return feasibility measure, maximum constraint residual
    **********************************************************************************/
    ScalarType getFeasibilityMeasure() const
    {
        return mFeasibilityMeasure;
    }

    /******************************************************************************//**
     * @brief Return norm of the objective function
     * @return norm of the objective function
    **********************************************************************************/
    ScalarType getNormObjectiveGradient() const
    {
        return mNormObjectiveGradient;
    }

    /******************************************************************************//**
     * @brief Return control stagnation measure
     * @return control stagnation measure
    **********************************************************************************/
    ScalarType getControlStagnationMeasure() const
    {
        return mControlStagnationMeasure;
    }

    /******************************************************************************//**
     * @brief Return objective stagnation measure
     * @return objective stagnation measure
    **********************************************************************************/
    ScalarType getObjectiveStagnationMeasure() const
    {
        return mObjectiveStagnationMeasure;
    }

    /******************************************************************************//**
     * @brief Return current objective function value
     * @return current objective function value
    **********************************************************************************/
    ScalarType getCurrentObjectiveValue() const
    {
        return mCurrentObjectiveValue;
    }

    /******************************************************************************//**
     * @brief Set current objective function value
     * @param [in] aValue current objective function value
    **********************************************************************************/
    void setCurrentObjectiveValue(const ScalarType &aValue)
    {
        mCurrentObjectiveValue = aValue;
    }

    /******************************************************************************//**
     * @brief Return previous objective function value
     * @return previous objective function value
    **********************************************************************************/
    ScalarType getPreviousObjectiveValue() const
    {
        return mPreviousObjectiveValue;
    }

    /******************************************************************************//**
     * @brief Set previous objective function value
     * @param [in] aValue previous objective function value
    **********************************************************************************/
    void setPreviousObjectiveValue(const ScalarType &aValue)
    {
        mPreviousObjectiveValue = aValue;
    }

    /******************************************************************************//**
     * @brief Return constraint normalization value
     * @param [in] aIndex constraint index
     * @return constraint normalization value
    **********************************************************************************/
    ScalarType getConstraintNormalization(const OrdinalType &aIndex) const
    {
        return ((*mConstraintNormalization)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Set constraint normalization parameters
     * @param [in] aInput reference to 1D container of constraint normalization parameters
    **********************************************************************************/
    void setConstraintNormalizationParams(const Plato::Vector<ScalarType, OrdinalType>& aInput)
    {
        mConstraintNormalization->update(static_cast<ScalarType>(1.0), aInput, static_cast<ScalarType>(0.0));
    }

    /******************************************************************************//**
     * @brief Set constraint normalization value
     * @param [in] aIndex constraint index
     * @param [in] aValue constraint normalization value
    **********************************************************************************/
    void setConstraintNormalization(const OrdinalType &aIndex, const ScalarType &aValue)
    {
        (*mConstraintNormalization)[aIndex] = aValue;
    }

    /******************************************************************************//**
     * @brief Return current constraint value
     * @param [in] aIndex constraint index
     * @return current constraint value
    **********************************************************************************/
    ScalarType getCurrentConstraintValue(const OrdinalType &aIndex) const
    {
        return ((*mCurrentConstraintValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Set current constraint value
     * @param [in] aIndex constraint index
     * @param [in] aValue current constraint value
    **********************************************************************************/
    void setCurrentConstraintValue(const OrdinalType &aIndex, const ScalarType &aValue)
    {
        (*mCurrentConstraintValues)[aIndex] = aValue;
    }

    /******************************************************************************//**
     * @brief Return previous constraint value
     * @param [in] aIndex constraint index
     * @return previous constraint value
    **********************************************************************************/
    ScalarType getPreviousConstraintValues(const OrdinalType &aIndex) const
    {
        return ((*mPreviousConstraintValues)[aIndex]);
    }

    /******************************************************************************//**
     * @brief Set previous constraint value
     * @param [in] aIndex constraint index
     * @param [in] aValue previous constraint value
    **********************************************************************************/
    void setPreviousConstraintValues(const OrdinalType &aIndex, const ScalarType &aValue)
    {
        (*mPreviousConstraintValues)[aIndex] = aValue;
    }

    /******************************************************************************//**
     * @brief Gather container of constraint values
     * @param [in] aInput reference to 1D container of constraint values
    **********************************************************************************/
    void getCurrentConstraintValues(Plato::Vector<ScalarType, OrdinalType>& aInput) const
    {
        aInput.update(static_cast<ScalarType>(1.0), *mCurrentConstraintValues, static_cast<ScalarType>(0.0));
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of lower asymptotes
     * @return reference to 2D container of lower asymptotes
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getLowerAsymptotes()
    {
        return (*mLowerAsymptotes);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of lower asymptotes
     * @return const reference to 2D container of lower asymptotes
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getLowerAsymptotes() const
    {
        return (*mLowerAsymptotes);
    }

    /******************************************************************************//**
     * @brief Set const reference to 2D container of lower asymptotes
     * @param [in] aInput const reference to 2D container of lower asymptotes
    **********************************************************************************/
    void setLowerAsymptotes(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mLowerAsymptotes);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of upper asymptotes
     * @return reference to 2D container of upper asymptotes
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getUpperAsymptotes()
    {
        return (*mUpperAsymptotes);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of upper asymptotes
     * @return const reference to 2D container of upper asymptotes
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getUpperAsymptotes() const
    {
        return (*mUpperAsymptotes);
    }

    /******************************************************************************//**
     * @brief Set const reference to 2D container of upper asymptotes
     * @param [in] aInput const reference to 2D container of upper asymptotes
    **********************************************************************************/
    void setUpperAsymptotes(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mUpperAsymptotes);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of objective approximation function one (P)
     * @return reference to 2D container of objective approximation function one (P)
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getObjFuncAppxFunctionP()
    {
        return (*mObjFuncAppxFunctionP);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of objective approximation function one (P)
     * @return const reference to 2D container of objective approximation function one (P)
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getObjFuncAppxFunctionP() const
    {
        return (*mObjFuncAppxFunctionP);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of objective approximation function two (Q)
     * @return reference to 2D container of objective approximation function two (Q)
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getObjFuncAppxFunctionQ()
    {
        return (*mObjFuncAppxFunctionQ);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of objective approximation function two (Q)
     * @return const reference to 2D container of objective approximation function two (Q)
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getObjFuncAppxFunctionQ() const
    {
        return (*mObjFuncAppxFunctionQ);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of constraint approximation function one (P)
     * @param [in] aIndex constraint index
     * @return reference to 2D container of constraint approximation function one (P)
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getConstraintAppxFunctionP(const OrdinalType & aIndex)
    {
        return (*mConstrAppxFunctionP)[aIndex];
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of constraint approximation function one (P)
     * @param [in] aIndex constraint index
     * @return const reference to 2D container of constraint approximation function one (P)
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getConstraintAppxFunctionP(const OrdinalType & aIndex) const
    {
        return (*mConstrAppxFunctionP)[aIndex];
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of constraint approximation function two (Q)
     * @param [in] aIndex constraint index
     * @return reference to 2D container of constraint approximation function two (Q)
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getConstraintAppxFunctionQ(const OrdinalType & aIndex)
    {
        return (*mConstrAppxFunctionQ)[aIndex];
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of constraint approximation function two (Q)
     * @param [in] aIndex constraint index
     * @return const reference to 2D container of constraint approximation function two (Q)
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getConstraintAppxFunctionQ(const OrdinalType & aIndex) const
    {
        return (*mConstrAppxFunctionQ)[aIndex];
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of current optimization variables
     * @return reference to 2D container of current optimization variables
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getCurrentControls()
    {
        return (*mCurrentControls);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of current optimization variables
     * @return const reference to 2D container of current optimization variables
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getCurrentControls() const
    {
        return (*mCurrentControls);
    }

    /******************************************************************************//**
     * @brief Set 2D container of current optimization variables
     * @return const reference to 2D container of current optimization variables
    **********************************************************************************/
    void setCurrentControls(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentControls);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of previous optimization variables
     * @return reference to 2D container of previous optimization variables
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getPreviousControls()
    {
        return (*mPreviousControls);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of previous optimization variables
     * @return const reference to 2D container of previous optimization variables
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getPreviousControls() const
    {
        return (*mPreviousControls);
    }

    /******************************************************************************//**
     * @brief Set 2D container of previous optimization variables
     * @return const reference to 2D container of previous optimization variables
    **********************************************************************************/
    void setPreviousControls(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mPreviousControls);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of antepenultimate optimization variables
     * @return reference to 2D container of antepenultimate optimization variables
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getAntepenultimateControls()
    {
        return (*mAntepenultimateControls);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of antepenultimate optimization variables
     * @return const reference to 2D container of antepenultimate optimization variables
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getAntepenultimateControls() const
    {
        return (*mAntepenultimateControls);
    }

    /******************************************************************************//**
     * @brief Set 2D container of antepenultimate optimization variables
     * @return const reference to 2D container of antepenultimate optimization variables
    **********************************************************************************/
    void setAntepenultimateControls(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mAntepenultimateControls);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of lower bounds on optimization variables
     * @return reference to 2D container of lower bounds on optimization variables
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getControlLowerBounds()
    {
        return (*mControlLowerBounds);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of lower bounds on optimization variables
     * @return const reference to 2D container of lower bounds on optimization variables
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getControlLowerBounds() const
    {
        return (*mControlLowerBounds);
    }

    /******************************************************************************//**
     * @brief Set 2D container of lower bounds on optimization variables
     * @return const reference to 2D container of lower bounds on optimization variables
    **********************************************************************************/
    void setControlLowerBounds(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mControlLowerBounds);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of upper bounds on optimization variables
     * @return reference to 2D container of upper bounds on optimization variables
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getControlUpperBounds()
    {
        return (*mControlUpperBounds);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of upper bounds on optimization variables
     * @return const reference to 2D container of upper bounds on optimization variables
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getControlUpperBounds() const
    {
        return (*mControlUpperBounds);
    }

    /******************************************************************************//**
     * @brief Set 2D container of upper bounds on optimization variables
     * @return const reference to 2D container of upper bounds on optimization variables
    **********************************************************************************/
    void setControlUpperBounds(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mControlUpperBounds);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of lower bounds on optimization variables for MMA subproblem
     * @return reference to 2D container of lower bounds on optimization variables for MMA subproblem
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getSubProblemControlLowerBounds()
    {
        return (*mSubProblemControlLowerBounds);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of lower bounds on optimization variables for MMA subproblem
     * @return const reference to 2D container of lower bounds on optimization variables for MMA subproblem
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getSubProblemControlLowerBounds() const
    {
        return (*mSubProblemControlLowerBounds);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of upper bounds on optimization variables for MMA subproblem
     * @return reference to 2D container of upper bounds on optimization variables for MMA subproblem
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getSubProblemControlUpperBounds()
    {
        return (*mSubProblemControlUpperBounds);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of upper bounds on optimization variables for MMA subproblem
     * @return const reference to 2D container of upper bounds on optimization variables for MMA subproblem
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getSubProblemControlUpperBounds() const
    {
        return (*mSubProblemControlUpperBounds);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of current objective function first-order sensitivities
     * @return reference to 2D container of current objective function first-order sensitivities
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getCurrentObjectiveGradient()
    {
        return (*mCurrentObjectiveGradient);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of current objective function first-order sensitivities
     * @return const reference to 2D container of current objective function first-order sensitivities
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getCurrentObjectiveGradient() const
    {
        return (*mCurrentObjectiveGradient);
    }

    /******************************************************************************//**
     * @brief Set 2D container of current objective function first-order sensitivities
     * @return reference to 2D container ofcurrent  objective function first-order sensitivities
    **********************************************************************************/
    void setCurrentObjectiveGradient(const Plato::MultiVector<ScalarType, OrdinalType>& aInput)
    {
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentObjectiveGradient);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of previous objective function first-order sensitivities
     * @return reference to 2D container of previous objective function first-order sensitivities
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getPreviousObjectiveGradient()
    {
        return (*mPreviousObjectiveGradient);
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of previous objective function first-order sensitivities
     * @return const reference to 2D container of previous objective function first-order sensitivities
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getPreviousObjectiveGradient() const
    {
        return (*mPreviousObjectiveGradient);
    }

    /******************************************************************************//**
     * @brief Return reference to 2D container of current constraint first-order sensitivities
     * @param [in] aIndex constraint index
     * @return reference to 2D container of current constraint first-order sensitivities
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getCurrentConstraintGradient(const OrdinalType &aIndex)
    {
        return (*mCurrentConstraintGradients)[aIndex];
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of current constraint first-order sensitivities
     * @param [in] aIndex constraint index
     * @return const reference to 2D container of current constraint first-order sensitivities
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getCurrentConstraintGradient(const OrdinalType &aIndex) const
    {
        return (*mCurrentConstraintGradients)[aIndex];
    }

    /******************************************************************************//**
     * @brief Set 2D container of current constraint first-order sensitivities
     * @param [in] aIndex constraint index
     * @param const reference to 2D container of current constraint first-order sensitivities
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Return reference to 2D container of previous constraint first-order sensitivities
     * @param [in] aIndex constraint index
     * @return reference to 2D container of previous constraint first-order sensitivities
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType>& getPreviousConstraintGradient(const OrdinalType &aIndex)
    {
        return (*mPreviousConstraintGradients)[aIndex];
    }

    /******************************************************************************//**
     * @brief Return const reference to 2D container of previous constraint first-order sensitivities
     * @param [in] aIndex constraint index
     * @return const reference to 2D container of previous constraint first-order sensitivities
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType>& getPreviousConstraintGradient(const OrdinalType &aIndex) const
    {
        return (*mPreviousConstraintGradients)[aIndex];
    }

    /******************************************************************************//**
     * @brief Cache current state, e.g. controls and objective and constraints values and gradients
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Compute stopping measures
    **********************************************************************************/
    void computeStoppingMeasures()
    {
        this->computeFeasibilityMeasure();
        this->computeNormObjectiveGradient();
        this->computeControlStagnationMeasure();
        this->computeObjectiveStagnationMeasure();
    }

    /******************************************************************************//**
     * @brief Compute feasibility tolerance
    **********************************************************************************/
    void computeFeasibilityMeasure()
    {
        mConstraintWork->update(static_cast<ScalarType>(1), *mCurrentConstraintValues, static_cast<ScalarType>(0));
        mConstraintWork->modulus();
        mFeasibilityMeasure = mDualReductionOps->max(*mConstraintWork);
    }

    /******************************************************************************//**
     * @brief Compute norm of the objective function gradient
    **********************************************************************************/
    void computeNormObjectiveGradient()
    {
        mNormObjectiveGradient = Plato::norm(*mCurrentObjectiveGradient);
    }

    /******************************************************************************//**
     * @brief Compute control stagnation measure
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Compute objective stagnation measure
    **********************************************************************************/
    void computeObjectiveStagnationMeasure()
    {
        mObjectiveStagnationMeasure = mCurrentObjectiveValue - mPreviousObjectiveValue;
        mObjectiveStagnationMeasure = std::abs(mObjectiveStagnationMeasure);
    }

private:
    /******************************************************************************//**
     * @brief Initialize internal metadata
    **********************************************************************************/
    void initialize(const Plato::DataFactory<ScalarType, OrdinalType> &aDataFactory)
    {
        const OrdinalType tNumConstraints = this->getNumConstraints();
        if(tNumConstraints <= static_cast<OrdinalType>(0))
        {
            THROWERR(std::string("INVALID NUMBER OF CONSTRAINTS ") + std::to_string(tNumConstraints) + ". THE NUMBER OF CONSTRAINTS SHOULD BE A POSITIVE NUMBER.\n")
        }

        mConstraintNormalization->fill(1.0);
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            mConstrAppxFunctionP->add(aDataFactory.control().create());
            mConstrAppxFunctionQ->add(aDataFactory.control().create());
            mCurrentConstraintGradients->add(aDataFactory.control().create());
            mPreviousConstraintGradients->add(aDataFactory.control().create());
        }
    }

private:
    ScalarType mFeasibilityMeasure; /*!< current feasibility measure */
    ScalarType mNormObjectiveGradient; /*!< current norm of the objective function gradient */
    ScalarType mCurrentObjectiveValue; /*!< current objective value */
    ScalarType mPreviousObjectiveValue; /*!< previous objective value */
    ScalarType mControlStagnationMeasure; /*!< current control stagnation measure */
    ScalarType mObjectiveStagnationMeasure; /*!< current objective stagnation measure */
    ScalarType mCurrentNormalizedObjectiveValue; /*!< current normalize objective value */

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWork; /*!< 1D container of optimization variables - work */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mConstraintWork; /*!< 1D container of constraint values - work */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mConstraintNormalization; /*!< 1D container of constraint normalization factors */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentConstraintValues; /*!< 1D container of current constraint values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mPreviousConstraintValues; /*!< 1D container of previous constraint values */
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentNormalizedConstraintValue; /*!< 1D container of current normalized constraint values */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentObjectiveGradient; /*!< 2D container of current objective first-order sensitivities */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousObjectiveGradient; /*!< 2D container of previous objective first-order sensitivities */
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mCurrentConstraintGradients; /*!< 2D container of current constraint first-order sensitivities */
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mPreviousConstraintGradients; /*!< 2D container of previous constraint first-order sensitivities */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControls; /*!< 2D container of current optimization variables */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousControls; /*!< 2D container of previous optimization variables */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mAntepenultimateControls; /*!< 2D container of antepenultimate optimization variables */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerAsymptotes; /*!< 2D container of current lower asymptotes */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperAsymptotes; /*!< 2D container of current upper asymptotes */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlLowerBounds; /*!< 2D container of lower bounds on optimization variables */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlUpperBounds; /*!< 2D container of upper bounds on optimization variables */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mSubProblemControlLowerBounds; /*!< 2D container of lower bounds on optimization variables for MMA subproblem */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mSubProblemControlUpperBounds; /*!< 2D container of upper bounds on optimization variables for MMA subproblem */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mObjFuncAppxFunctionP; /*!< 2D container of objective approximation function one (P) */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mObjFuncAppxFunctionQ; /*!< 2D container of objective approximation function two (Q) */
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mConstrAppxFunctionP; /*!< 2D container of constraint approximation function one (P) */
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mConstrAppxFunctionQ; /*!< 2D container of constraint approximation function two (Q) */

    std::shared_ptr<Plato::CommWrapper> mComm; /*!< wrapper to MPI communicator */
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mDualReductionOps; /*!< reduction operation interface for dual variables */
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductionOps; /*!< reduction operation interface for control variables */

private:
    MethodMovingAsymptotesDataMng(const Plato::MethodMovingAsymptotesDataMng<ScalarType, OrdinalType> & aRhs);
    Plato::MethodMovingAsymptotesDataMng<ScalarType, OrdinalType> & operator=(const Plato::MethodMovingAsymptotesDataMng<ScalarType, OrdinalType> & aRhs);
};
// class MethodMovingAsymptotesDataMng

}
// namespace Plato
