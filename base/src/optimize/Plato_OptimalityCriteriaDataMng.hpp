/*
 * Plato_OptimalityCriteriaDataMng.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_OPTIMALITYCRITERIADATAMNG_HPP_
#define PLATO_OPTIMALITYCRITERIADATAMNG_HPP_

#include <cmath>
#include <limits>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_ReductionOperations.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class OptimalityCriteriaDataMng
{
public:
    explicit OptimalityCriteriaDataMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory) :
            mInitialGuessSet(false),
            mMaxInequalityValue(std::numeric_limits<ScalarType>::max()),
            mNormObjectiveGradient(std::numeric_limits<ScalarType>::max()),
            mCurrentObjectiveValue(std::numeric_limits<ScalarType>::max()),
            mPreviousObjectiveValue(std::numeric_limits<ScalarType>::max()),
            mControlStagnationMeasure(std::numeric_limits<ScalarType>::max()),
            mObjectiveStagnationMeasure(std::numeric_limits<ScalarType>::max()),
            mMemorySpace(aFactory->getMemorySpace()),
            mCurrentDual(),
            mDualWorkVector(),
            mControlWorkVector(),
            mCurrentConstraintValues(),
            mPreviousConstraintValues(),
            mCurrentControl(aFactory->control().create()),
            mPreviousControl(aFactory->control().create()),
            mObjectiveGradient(aFactory->control().create()),
            mInequalityGradient(aFactory->control().create()),
            mControlLowerBounds(aFactory->control().create()),
            mControlUpperBounds(aFactory->control().create()),
            mDualReductionOperations(),
            mControlReductionOperations()
    {
        this->initialize(aFactory);
    }
    ~OptimalityCriteriaDataMng()
    {
    }

    OrdinalType getNumConstraints() const
    {
        OrdinalType tNumVectors = mCurrentDual->size();
        return (tNumVectors);
    }
    OrdinalType getNumControlVectors() const
    {
        OrdinalType tNumVectors = mCurrentControl->getNumVectors();
        return (tNumVectors);
    }

    ScalarType getMaxInequalityValue() const
    {
        return (mMaxInequalityValue);
    }
    ScalarType getNormObjectiveGradient() const
    {
        return (mNormObjectiveGradient);
    }
    ScalarType getControlStagnationMeasure() const
    {
        return (mControlStagnationMeasure);
    }
    ScalarType getObjectiveStagnationMeasure() const
    {
        return (mObjectiveStagnationMeasure);
    }

    Plato::MemorySpace::type_t getMemorySpace() const
    {
        return (mMemorySpace);
    }

    void computeMaxInequalityValue()
    {
        mDualWorkVector->update(1., *mCurrentConstraintValues, 0.);
        mDualWorkVector->modulus();
        mMaxInequalityValue = mDualReductionOperations->max(*mDualWorkVector);
    }
    void computeNormObjectiveGradient()
    {
        ScalarType tCummulativeDotProduct = 0.;
        OrdinalType tNumVectors = mObjectiveGradient->getNumVectors();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyGradient = (*mObjectiveGradient)[tIndex];
            tCummulativeDotProduct += tMyGradient.dot(tMyGradient);
        }
        mNormObjectiveGradient = std::sqrt(tCummulativeDotProduct);
    }
    void computeControlStagnationMeasure()
    {
        OrdinalType tNumVectors = mCurrentControl->getNumVectors();
        std::vector<ScalarType> storage(tNumVectors, std::numeric_limits<ScalarType>::min());
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tCurrentControl = mCurrentControl->operator[](tIndex);
            mControlWorkVector->update(1., tCurrentControl, 0.);
            Plato::Vector<ScalarType, OrdinalType> & tPreviousControl = mPreviousControl->operator[](tIndex);
            mControlWorkVector->update(-1., tPreviousControl, 1.);
            mControlWorkVector->modulus();
            storage[tIndex] = mControlReductionOperations->max(*mControlWorkVector);
        }
        mControlStagnationMeasure = *std::max_element(storage.begin(), storage.end());
    }
    void computeObjectiveStagnationMeasure()
    {
        mObjectiveStagnationMeasure = std::abs(mCurrentObjectiveValue - mPreviousObjectiveValue);
    }

    ScalarType getCurrentObjectiveValue() const
    {
        return (mCurrentObjectiveValue);
    }
    void setCurrentObjectiveValue(const ScalarType & aInput)
    {
        mCurrentObjectiveValue = aInput;
    }
    ScalarType getPreviousObjectiveValue() const
    {
        return (mPreviousObjectiveValue);
    }
    void setPreviousObjectiveValue(const ScalarType & aInput)
    {
        mPreviousObjectiveValue = aInput;
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentDual() const
    {
        assert(mCurrentDual.get() != nullptr);
        assert(mCurrentDual->size() > static_cast<OrdinalType>(0));
        return (mCurrentDual.operator *());
    }
    void setCurrentDual(const OrdinalType & aIndex, const ScalarType & aValue)
    {
        assert(mCurrentDual.get() != nullptr);
        assert(aIndex >= static_cast<OrdinalType>(0));
        assert(aIndex < mCurrentDual->size());
        mCurrentDual->operator [](aIndex) = aValue;
    }

    const Plato::Vector<ScalarType, OrdinalType> & getCurrentConstraintValues() const
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        assert(mCurrentConstraintValues->size() > static_cast<OrdinalType>(0));
        return (mCurrentConstraintValues.operator *());
    }
    ScalarType getCurrentConstraintValues(const OrdinalType & aIndex) const
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        assert(aIndex >= static_cast<OrdinalType>(0));
        assert(aIndex < mCurrentConstraintValues->size());
        return(mCurrentConstraintValues->operator [](aIndex));
    }
    void setCurrentConstraintValue(const OrdinalType & aIndex, const ScalarType & aValue)
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        assert(aIndex >= static_cast<OrdinalType>(0));
        assert(aIndex < mCurrentConstraintValues->size());
        mCurrentConstraintValues->operator [](aIndex) = aValue;
    }

    const Plato::Vector<ScalarType, OrdinalType> & getPreviousConstraintValues() const
    {
        assert(mPreviousConstraintValues.get() != nullptr);
        assert(mPreviousConstraintValues->size() > static_cast<OrdinalType>(0));
        return (mPreviousConstraintValues.operator *());
    }
    ScalarType getPreviousConstraintValues(const OrdinalType & aIndex) const
    {
        assert(mPreviousConstraintValues.get() != nullptr);
        assert(aIndex >= static_cast<OrdinalType>(0));
        assert(aIndex < mPreviousConstraintValues->size());
        return(mPreviousConstraintValues->operator [](aIndex));
    }
    void setPreviousConstraintValue(const OrdinalType & aIndex, const ScalarType & aValue)
    {
        assert(mPreviousConstraintValues.get() != nullptr);
        assert(aIndex >= static_cast<OrdinalType>(0));
        assert(aIndex < mPreviousConstraintValues->size());
        mPreviousConstraintValues->operator [](aIndex) = aValue;
    }

    void setInitialGuess(const ScalarType & aValue)
    {
        assert(mCurrentControl.get() != nullptr);
        assert(mCurrentControl->getNumVectors() > static_cast<OrdinalType>(0));
        OrdinalType tNumVectors = mCurrentControl->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            mCurrentControl->operator [](tVectorIndex).fill(aValue);
        }
        mInitialGuessSet = true;
    }
    void setInitialGuess(const OrdinalType & aVectorIndex, const ScalarType & aValue)
    {
        assert(mCurrentControl.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mCurrentControl->getNumVectors());

        mCurrentControl->operator [](aVectorIndex).fill(aValue);
        mInitialGuessSet = true;
    }
    void setInitialGuess(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInitialGuess)
    {
        assert(mCurrentControl.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mCurrentControl->getNumVectors());

        mCurrentControl->operator [](aVectorIndex).update(1., aInitialGuess, 0.);
        mInitialGuessSet = true;
    }
    void setInitialGuess(const Plato::MultiVector<ScalarType, OrdinalType> & aInitialGuess)
    {
        assert(aInitialGuess.getNumVectors() == mCurrentControl->getNumVectors());

        const OrdinalType tNumVectors = aInitialGuess.getNumVectors();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tInputInitialGuess = aInitialGuess[tIndex];
            Plato::Vector<ScalarType, OrdinalType> & tMyControl = mCurrentControl->operator [](tIndex);
            assert(tInputInitialGuess.size() == tMyControl.size());
            tMyControl.update(1., tInputInitialGuess, 0.);
        }
        mInitialGuessSet = true;
    }
    bool isInitialGuessSet() const
    {
        return (mInitialGuessSet);
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentControl() const
    {
        assert(mCurrentControl.get() != nullptr);

        return (mCurrentControl.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentControl(const OrdinalType & aVectorIndex) const
    {
        assert(mCurrentControl.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mCurrentControl->getNumVectors());

        return (mCurrentControl->operator [](aVectorIndex));
    }
    void setCurrentControl(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() == mCurrentControl->getNumVectors());

        const OrdinalType tNumVectors = aControl.getNumVectors();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tInputControl = aControl[tIndex];
            Plato::Vector<ScalarType, OrdinalType> & tMyControl = mCurrentControl->operator [](tIndex);
            assert(tInputControl.size() == tMyControl.size());
            tMyControl.update(1., tInputControl, 0.);
        }
    }
    void setCurrentControl(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aControl)
    {
        assert(mCurrentControl.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mCurrentControl->getNumVectors());

        mCurrentControl->operator [](aVectorIndex).update(1., aControl, 0.);
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getPreviousControl() const
    {
        assert(mPreviousControl.get() != nullptr);
        return (mPreviousControl.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getPreviousControl(const OrdinalType & aVectorIndex) const
    {
        assert(mPreviousControl.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mPreviousControl->getNumVectors());

        return (mPreviousControl->operator [](aVectorIndex));
    }
    void setPreviousControl(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() == mPreviousControl->getNumVectors());

        const OrdinalType tNumVectors = aControl.getNumVectors();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tInputControl = aControl[tIndex];
            Plato::Vector<ScalarType, OrdinalType> & tMyControl = mPreviousControl->operator [](tIndex);
            assert(tInputControl.size() == tMyControl.size());
            tMyControl.update(1., tInputControl, 0.);
        }
    }
    void setPreviousControl(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aControl)
    {
        assert(mPreviousControl.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mPreviousControl->getNumVectors());

        mPreviousControl->operator [](aVectorIndex).update(1., aControl, 0.);
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getObjectiveGradient() const
    {
        assert(mObjectiveGradient.get() != nullptr);

        return (mObjectiveGradient.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getObjectiveGradient(const OrdinalType & aVectorIndex) const
    {
        assert(mObjectiveGradient.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mObjectiveGradient->getNumVectors());

        return (mObjectiveGradient->operator [](aVectorIndex));
    }
    void setObjectiveGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aGradient)
    {
        assert(aGradient.getNumVectors() == mObjectiveGradient->getNumVectors());

        const OrdinalType tNumVectors = aGradient.getNumVectors();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tInputGradient = aGradient[tIndex];
            Plato::Vector<ScalarType, OrdinalType> & tMyObjectiveGradient = mObjectiveGradient->operator [](tIndex);
            assert(tInputGradient.size() == tMyObjectiveGradient.size());
            tMyObjectiveGradient.update(1., tInputGradient, 0.);
        }
    }
    void setObjectiveGradient(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aGradient)
    {
        assert(mObjectiveGradient.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mObjectiveGradient->getNumVectors());

        mObjectiveGradient->operator [](aVectorIndex).update(1., aGradient, 0.);
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getInequalityGradient() const
    {
        assert(mInequalityGradient.get() != nullptr);

        return (mInequalityGradient.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getInequalityGradient(const OrdinalType & aVectorIndex) const
    {
        assert(mInequalityGradient.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mInequalityGradient->getNumVectors());

        return (mInequalityGradient->operator [](aVectorIndex));
    }
    void setInequalityGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aGradient)
    {
        assert(aGradient.getNumVectors() == mInequalityGradient->getNumVectors());

        const OrdinalType tNumVectors = aGradient.getNumVectors();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tInputGradient = aGradient[tIndex];
            Plato::Vector<ScalarType, OrdinalType> & tMyInequalityGradient = mInequalityGradient->operator [](tIndex);
            assert(tInputGradient.size() == tMyInequalityGradient.size());
            tMyInequalityGradient.update(1., tInputGradient, 0.);
        }
    }
    void setInequalityGradient(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aGradient)
    {
        assert(mInequalityGradient.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mInequalityGradient->getNumVectors());

        mInequalityGradient->operator [](aVectorIndex).update(1., aGradient, 0.);
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getControlLowerBounds() const
    {
        assert(mControlLowerBounds.get() != nullptr);

        return (mControlLowerBounds.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getControlLowerBounds(const OrdinalType & aVectorIndex) const
    {
        assert(mControlLowerBounds.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mControlLowerBounds->getNumVectors());

        return (mControlLowerBounds->operator [](aVectorIndex));
    }
    void setControlLowerBounds(const ScalarType & aValue)
    {
        assert(mControlLowerBounds.get() != nullptr);
        assert(mControlLowerBounds->getNumVectors() > static_cast<OrdinalType>(0));

        OrdinalType tNumVectors = mControlLowerBounds->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            mControlLowerBounds->operator [](tVectorIndex).fill(aValue);
        }
    }
    void setControlLowerBounds(const OrdinalType & aVectorIndex, const ScalarType & aValue)
    {
        assert(mControlLowerBounds.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mControlLowerBounds->getNumVectors());

        mControlLowerBounds->operator [](aVectorIndex).fill(aValue);
    }
    void setControlLowerBounds(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aLowerBound)
    {
        assert(mControlLowerBounds.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mControlLowerBounds->getNumVectors());

        mControlLowerBounds->operator [](aVectorIndex).update(1., aLowerBound, 0.);
    }
    void setControlLowerBounds(const Plato::MultiVector<ScalarType, OrdinalType> & aLowerBound)
    {
        assert(aLowerBound.getNumVectors() == mControlLowerBounds->getNumVectors());

        const OrdinalType tNumVectors = aLowerBound.getNumVectors();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tInputLowerBound = aLowerBound[tIndex];
            Plato::Vector<ScalarType, OrdinalType> & tMyLowerBound = mControlLowerBounds->operator [](tIndex);
            assert(tInputLowerBound.size() == tMyLowerBound.size());
            tMyLowerBound.update(1., tInputLowerBound, 0.);
        }
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getControlUpperBounds() const
    {
        assert(mControlUpperBounds.get() != nullptr);

        return (mControlUpperBounds.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getControlUpperBounds(const OrdinalType & aVectorIndex) const
    {
        assert(mControlUpperBounds.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mControlUpperBounds->getNumVectors());

        return (mControlUpperBounds->operator [](aVectorIndex));
    }
    void setControlUpperBounds(const ScalarType & aValue)
    {
        assert(mControlUpperBounds.get() != nullptr);
        assert(mControlUpperBounds->getNumVectors() > static_cast<OrdinalType>(0));

        OrdinalType tNumVectors = mControlUpperBounds->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            mControlUpperBounds->operator [](tVectorIndex).fill(aValue);
        }
    }
    void setControlUpperBounds(const OrdinalType & aVectorIndex, const ScalarType & aValue)
    {
        assert(mControlUpperBounds.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mControlUpperBounds->getNumVectors());

        mControlUpperBounds->operator [](aVectorIndex).fill(aValue);
    }
    void setControlUpperBounds(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aUpperBound)
    {
        assert(mControlUpperBounds.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mControlUpperBounds->getNumVectors());

        mControlUpperBounds->operator [](aVectorIndex).update(1., aUpperBound, 0.);
    }
    void setControlUpperBounds(const Plato::MultiVector<ScalarType, OrdinalType> & aUpperBound)
    {
        assert(aUpperBound.getNumVectors() == mControlUpperBounds->getNumVectors());

        const OrdinalType tNumVectors = aUpperBound.getNumVectors();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tInputUpperBound = aUpperBound[tIndex];
            Plato::Vector<ScalarType, OrdinalType> & tMyUpperBound = mControlUpperBounds->operator [](tIndex);
            assert(tInputUpperBound.size() == tMyUpperBound.size());
            tMyUpperBound.update(1., tInputUpperBound, 0.);
        }
    }

private:
    void initialize(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory)
    {
        assert(aFactory->dual().getNumVectors() > static_cast<OrdinalType>(0));
        assert(aFactory->control().getNumVectors() > static_cast<OrdinalType>(0));

        const OrdinalType tVECTOR_INDEX = 0;
        mCurrentDual = aFactory->dual(tVECTOR_INDEX).create();
        mDualWorkVector = aFactory->dual(tVECTOR_INDEX).create();
        mControlWorkVector = aFactory->control(tVECTOR_INDEX).create();
        mCurrentConstraintValues = aFactory->dual(tVECTOR_INDEX).create();
        mPreviousConstraintValues = aFactory->dual(tVECTOR_INDEX).create();

        mDualReductionOperations = aFactory->getDualReductionOperations().create();
        mControlReductionOperations = aFactory->getControlReductionOperations().create();

        ScalarType tScalarValue = 1;
        Plato::fill(tScalarValue, mControlUpperBounds.operator*());
        // LOWER BOUNDS ARE SET TO ZERO BY DEFAULT
    }

private:
    bool mInitialGuessSet;

    ScalarType mMaxInequalityValue;
    ScalarType mNormObjectiveGradient;
    ScalarType mCurrentObjectiveValue;
    ScalarType mPreviousObjectiveValue;
    ScalarType mControlStagnationMeasure;
    ScalarType mObjectiveStagnationMeasure;

    Plato::MemorySpace::type_t mMemorySpace;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentDual;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mDualWorkVector;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWorkVector;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mCurrentConstraintValues;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mPreviousConstraintValues;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mObjectiveGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mInequalityGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlLowerBounds;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlUpperBounds;

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mDualReductionOperations;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductionOperations;

private:
    OptimalityCriteriaDataMng(const Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType>&);
    Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType> & operator=(const Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType>&);
};

}

#endif /* PLATO_OPTIMALITYCRITERIADATAMNG_HPP_ */
