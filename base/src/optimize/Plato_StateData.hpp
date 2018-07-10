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
    explicit StateData(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory) :
            mCurrentObjectiveFunctionValue(std::numeric_limits<ScalarType>::max()),
            mCurrentControl(aDataFactory.control().create()),
            mCurrentTrialStep(aDataFactory.control().create()),
            mCurrentObjectiveGradient(aDataFactory.control().create()),
            mCurrentConstraintGradient(aDataFactory.control().create())
    {
    }
    ~StateData()
    {
    }

    ScalarType getCurrentObjectiveFunctionValue() const
    {
        return (mCurrentObjectiveFunctionValue);
    }
    void setCurrentObjectiveFunctionValue(const ScalarType & aInput)
    {
        mCurrentObjectiveFunctionValue = aInput;
    }
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentControl() const
    {
        assert(mCurrentControl.get() != nullptr);
        return (mCurrentControl.operator*());
    }
    void setCurrentControl(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentControl.get() != nullptr);
        assert(aInput.getNumVectors() == mCurrentControl->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), mCurrentControl.operator*());
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentTrialStep() const
    {
        assert(mCurrentTrialStep.get() != nullptr);
        return (mCurrentTrialStep.operator*());
    }
    void setCurrentTrialStep(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentTrialStep.get() != nullptr);
        assert(aInput.getNumVectors() == mCurrentTrialStep->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), mCurrentTrialStep.operator*());
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentObjectiveGradient() const
    {
        assert(mCurrentObjectiveGradient.get() != nullptr);
        return (mCurrentObjectiveGradient);
    }
    void setCurrentObjectiveGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentObjectiveGradient.get() != nullptr);
        assert(aInput.getNumVectors() == mCurrentObjectiveGradient->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), mCurrentObjectiveGradient.operator*());
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentConstraintGradient() const
    {
        assert(mCurrentConstraintGradient.get() != nullptr);
        return (mCurrentConstraintGradient);
    }
    void setCurrentConstraintGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentConstraintGradient.get() != nullptr);
        assert(aInput.getNumVectors() == mCurrentConstraintGradient->getNumVectors());

        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), mCurrentConstraintGradient.operator*());
    }

private:
    ScalarType mCurrentObjectiveFunctionValue;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentTrialStep;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentObjectiveGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentConstraintGradient;

private:
    StateData(const Plato::StateData<ScalarType, OrdinalType>&);
    Plato::StateData<ScalarType, OrdinalType> & operator=(const Plato::StateData<ScalarType, OrdinalType>&);
};

} // namespace Plato

#endif /* PLATO_STATEDATA_HPP_ */
