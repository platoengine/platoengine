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
// *************************************************************************
//@HEADER
*/

/*
 * Plato_StateData.hpp
 *
 *  Created on: Oct 21, 2017
 */

#ifndef PLATO_STATEDATA_HPP_
#define PLATO_STATEDATA_HPP_

#include <limits>
#include <memory>
#include <cassert>

#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class StateData
{
public:
    /******************************************************************************//**
     * @brief Default constructor
     * @param [in] aDataFactory linear algebra factory
    **********************************************************************************/
    explicit StateData(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory) :
            mCurrentOuterIteration(0),
            mCurrentCriterionValue(std::numeric_limits<ScalarType>::max()),
            mCurrentNormalizationConstant(std::numeric_limits<ScalarType>::max()),
            mCurrentControl(aDataFactory.control().create()),
            mPreviousControl(aDataFactory.control().create()),
            mCurrentTrialStep(aDataFactory.control().create()),
            mCurrentCriterionGradient(aDataFactory.control().create()),
            mPreviousCriterionGradient(aDataFactory.control().create()),
            mCurrentControlLowerBounds(aDataFactory.control().create()),
            mCurrentControlUpperBounds(aDataFactory.control().create())
    {
    }

    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    ~StateData()
    {
    }

    /******************************************************************************//**
     * @brief Return current criterion value
     * @return current criterion value
    **********************************************************************************/
    ScalarType getCurrentCriterionValue() const
    {
        return (mCurrentCriterionValue);
    }

    /******************************************************************************//**
     * @brief Set current criterion value
     * @param [in] aInput current criterion value
    **********************************************************************************/
    void setCurrentCriterionValue(const ScalarType & aInput)
    {
        mCurrentCriterionValue = aInput;
    }

    /******************************************************************************//**
     * @brief Return current normalization constant
     * @return current normalization constant
    **********************************************************************************/
    ScalarType getCurrentNormalizationConstant() const
    {
        return (mCurrentNormalizationConstant);
    }

    /******************************************************************************//**
     * @brief Set current normalization constant
     * @param [in] aInput current normalization constant
    **********************************************************************************/
    void setCurrentNormalizationConstant(const ScalarType & aInput)
    {
        mCurrentNormalizationConstant = aInput;
    }

    /******************************************************************************//**
     * @brief Return current optimization iteration
     * @return current optimization iteration
    **********************************************************************************/
    OrdinalType getCurrentOptimizationIteration() const
    {
        return (mCurrentOuterIteration);
    }

    /******************************************************************************//**
     * @brief Set current outer optimization iteration
     * @param [in] aInput current outer optimization iteration
    **********************************************************************************/
    void setCurrentOptimizationIteration(const OrdinalType & aInput)
    {
        mCurrentOuterIteration = aInput;
    }

    /******************************************************************************//**
     * @brief Return constant reference to 2D container of current controls
     * @return constant reference to 2D container of current controls
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentControl() const
    {
        assert(mCurrentControl.get() != nullptr);
        return (*mCurrentControl);
    }

    /******************************************************************************//**
     * @brief Set current control multi-vector
     * @param [in] aInput current control multi-vector
    **********************************************************************************/
    void setCurrentControl(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentControl.get() != nullptr);
        assert(aInput.getNumVectors() == mCurrentControl->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), mCurrentControl.operator*());
    }

    /******************************************************************************//**
     * @brief Return constant reference to 2D container of previous controls
     * @return constant reference to 2D container of previous controls
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getPreviousControl() const
    {
        assert(mPreviousControl.get() != nullptr);
        return (*mPreviousControl);
    }

    /******************************************************************************//**
     * @brief Set 2D container of previous controls
     * @param [in] aInput previous controls
    **********************************************************************************/
    void setPreviousControl(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mPreviousControl.get() != nullptr);
        assert(aInput.getNumVectors() == mPreviousControl->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mPreviousControl);
    }

    /******************************************************************************//**
     * @brief Return constant reference to 2D container of current descent direction
     * @return constant reference to 2D container of current descent direction
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentTrialStep() const
    {
        assert(mCurrentTrialStep.get() != nullptr);
        return (*mCurrentTrialStep);
    }

    /******************************************************************************//**
     * @brief Set current descent direction
     * @param [in] aInput current descent direction
    **********************************************************************************/
    void setCurrentTrialStep(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentTrialStep.get() != nullptr);
        assert(aInput.getNumVectors() == mCurrentTrialStep->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), mCurrentTrialStep.operator*());
    }

    /******************************************************************************//**
     * @brief Return constant reference to 2D container of current criterion gradient
     * @return constant reference to 2D container of current criterion gradient
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentCriterionGradient() const
    {
        assert(mCurrentCriterionGradient.get() != nullptr);
        return (*mCurrentCriterionGradient);
    }

    /******************************************************************************//**
     * @brief Set current criterion gradient
     * @param [in] aInput current criterion gradient
    **********************************************************************************/
    void setCurrentCriterionGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentCriterionGradient.get() != nullptr);
        assert(aInput.getNumVectors() == mCurrentCriterionGradient->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), mCurrentCriterionGradient.operator*());
    }

    /******************************************************************************//**
     * @brief Return constant reference to 2D container of previous criterion gradient
     * @return constant reference to 2D container of previous criterion gradient
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getPreviousCriterionGradient() const
    {
        assert(mPreviousCriterionGradient.get() != nullptr);
        return (*mPreviousCriterionGradient);
    }

    /******************************************************************************//**
     * @brief Set previous criterion gradient
     * @param [in] aInput previous criterion gradient
    **********************************************************************************/
    void setPreviousCriterionGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mPreviousCriterionGradient.get() != nullptr);
        assert(aInput.getNumVectors() == mPreviousCriterionGradient->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), mPreviousCriterionGradient.operator*());
    }

    /******************************************************************************//**
     * @brief Return constant reference to 2D container of current control lower bounds
     * @return constant reference to 2D container of current control lower bounds
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentControlLowerBounds() const
    {
        assert(mCurrentControlLowerBounds.get() != nullptr);
        return (*mCurrentControlLowerBounds);
    }

    /******************************************************************************//**
     * @brief Set current control lower bounds
     * @param [in] aInput current control lower bounds
    **********************************************************************************/
    void setCurrentControlLowerBounds(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentControlLowerBounds.get() != nullptr);
        assert(aInput.getNumVectors() == mCurrentControlLowerBounds->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentControlLowerBounds);
    }

    /******************************************************************************//**
     * @brief Return constant reference to 2D container of current control upper bounds
     * @return constant reference to 2D container of current control upper bounds
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentControlUpperBounds() const
    {
        assert(mCurrentControlUpperBounds.get() != nullptr);
        return (*mCurrentControlUpperBounds);
    }

    /******************************************************************************//**
     * @brief Set current control upper bounds
     * @param [in] aInput current control upper bounds
    **********************************************************************************/
    void setCurrentControlUpperBounds(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentControlUpperBounds.get() != nullptr);
        assert(aInput.getNumVectors() == mCurrentControlUpperBounds->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentControlUpperBounds);
    }

private:
    OrdinalType mCurrentOuterIteration; /*!< current outer optimization iteration */
    ScalarType mCurrentCriterionValue; /*!< current criterion value */
    ScalarType mCurrentNormalizationConstant; /*!< current normalization constant */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControl; /*!< current controls */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousControl; /*!< previous controls */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentTrialStep; /*!< current trial step */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentCriterionGradient; /*!< current criterion gradient */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousCriterionGradient; /*!< previous criterion gradient */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControlLowerBounds; /*!< current control lower bounds */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControlUpperBounds; /*!< current control upper bounds */

private:
    StateData(const Plato::StateData<ScalarType, OrdinalType>&);
    Plato::StateData<ScalarType, OrdinalType> & operator=(const Plato::StateData<ScalarType, OrdinalType>&);
};
// class StateData

}
// namespace Plato

#endif /* PLATO_STATEDATA_HPP_ */
