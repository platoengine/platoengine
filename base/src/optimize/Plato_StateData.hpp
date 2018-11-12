/*
 * Plato_StateData.hpp
 *
 *  Created on: Oct 21, 2017
 */

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
            mCurrentCriteriaValue(std::numeric_limits<ScalarType>::max()),
            mCurrentControl(aDataFactory.control().create()),
            mCurrentTrialStep(aDataFactory.control().create()),
            mCurrentCriteriaGradient(aDataFactory.control().create())
    {
    }

    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    ~StateData()
    {
    }

    /******************************************************************************//**
     * @brief Return current criteria value
     * @return current criteria value
    **********************************************************************************/
    ScalarType getCurrentCriteriaValue() const
    {
        return (mCurrentCriteriaValue);
    }

    /******************************************************************************//**
     * @brief Set current criteria value
     * @param [in] aInput current criteria value
    **********************************************************************************/
    void setCurrentCriteriaValue(const ScalarType & aInput)
    {
        mCurrentCriteriaValue = aInput;
    }

    /******************************************************************************//**
     * @brief Return constant reference to current control multi-vector
     * @return current control multi-vector
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentControl() const
    {
        assert(mCurrentControl.get() != nullptr);
        return (mCurrentControl.operator*());
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
     * @brief Return constant reference to current descent direction
     * @return current descent direction
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentTrialStep() const
    {
        assert(mCurrentTrialStep.get() != nullptr);
        return (mCurrentTrialStep.operator*());
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
     * @brief Return constant reference to current criteria gradient
     * @return current criteria gradient
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentCriteriaGradient() const
    {
        assert(mCurrentCriteriaGradient.get() != nullptr);
        return (mCurrentCriteriaGradient);
    }

    /******************************************************************************//**
     * @brief Set current criteria gradient
     * @param [in] aInput current criteria gradient
    **********************************************************************************/
    void setCurrentCriteriaGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentCriteriaGradient.get() != nullptr);
        assert(aInput.getNumVectors() == mCurrentCriteriaGradient->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), mCurrentCriteriaGradient.operator*());
    }

private:
    ScalarType mCurrentCriteriaValue;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentTrialStep;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentCriteriaGradient;

private:
    StateData(const Plato::StateData<ScalarType, OrdinalType>&);
    Plato::StateData<ScalarType, OrdinalType> & operator=(const Plato::StateData<ScalarType, OrdinalType>&);
};
// class StateData

} // namespace Plato

#endif /* PLATO_STATEDATA_HPP_ */
