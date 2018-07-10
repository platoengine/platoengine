/*
 * Plato_StateData.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
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
