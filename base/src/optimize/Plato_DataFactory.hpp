/*
 * Plato_DataFactory.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_DATAFACTORY_HPP_
#define PLATO_DATAFACTORY_HPP_

#include <cassert>

#include "Plato_StandardMultiVector.hpp"
#include "Plato_StandardVectorReductionOperations.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class DataFactory
{
public:
    /********************************************************************************/
    DataFactory(Plato::MemorySpace::type_t aMemorySpace = Plato::MemorySpace::HOST) :
            mNumDuals(1),
            mNumStates(1),
            mNumControls(0),
            mNumLowerBounds(0),
            mNumUpperBounds(0),
            mMemorySpace(aMemorySpace),
            mDual(),
            mState(),
            mControl(),
            mLowerBoundVector(),
            mUpperBoundVector(),
            mDualReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>()),
            mStateReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>()),
            mControlReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>())
    /********************************************************************************/
    {
        this->allocateDual(mNumDuals);
        this->allocateState(mNumStates);
    }

    /********************************************************************************/
    ~DataFactory()
    /********************************************************************************/
    {
    }

    /********************************************************************************/
    OrdinalType getNumControls() const
    /********************************************************************************/
    {
        assert(mNumControls > static_cast<ScalarType>(0));
        return (mNumControls);
    }

    /********************************************************************************/
    OrdinalType getNumLowerBounds() const
    /********************************************************************************/
    {
        assert(mNumLowerBounds > static_cast<ScalarType>(0));
        return (mNumLowerBounds);
    }

    /********************************************************************************/
    OrdinalType getNumUpperBounds() const
    /********************************************************************************/
    {
        assert(mNumUpperBounds > static_cast<ScalarType>(0));
        return (mNumUpperBounds);
    }

    /********************************************************************************/
    Plato::MemorySpace::type_t getMemorySpace() const
    /********************************************************************************/
    {
        return (mMemorySpace);
    }

    /********************************************************************************/
    void allocateDual(const OrdinalType & aNumElements, OrdinalType aNumVectors = 1)
    /********************************************************************************/
    {
        assert(aNumElements > static_cast<OrdinalType>(0));
        mNumDuals = aNumElements;
        Plato::StandardVector<ScalarType, OrdinalType> tVector(aNumElements);
        mDual = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(aNumVectors, tVector);
    }

    /********************************************************************************/
    void allocateDual(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    /********************************************************************************/
    {
        assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
        const OrdinalType tVectorIndex = 0;
        mNumDuals = aInput[tVectorIndex].size();
        assert(mNumDuals > static_cast<OrdinalType>(0));
        mDual = aInput.create();
    }

    /********************************************************************************/
    void allocateDual(const Plato::Vector<ScalarType, OrdinalType> & aInput, OrdinalType aNumVectors = 1)
    /********************************************************************************/
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mNumDuals = aInput.size();
        mDual = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(aNumVectors, aInput);
    }

    /********************************************************************************/
    void allocateDualReductionOperations(const Plato::ReductionOperations<ScalarType, OrdinalType> & aInput)
    /********************************************************************************/
    {
        mDualReductionOperations = aInput.create();
    }

    /********************************************************************************/
    void allocateState(const OrdinalType & aNumElements, OrdinalType aNumVectors = 1)
    /********************************************************************************/
    {
        assert(aNumElements > static_cast<OrdinalType>(0));
        mNumStates = aNumElements;
        Plato::StandardVector<ScalarType, OrdinalType> tVector(aNumElements);
        mState = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(aNumVectors, tVector);
    }

    /********************************************************************************/
    void allocateState(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    /********************************************************************************/
    {
        assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
        const OrdinalType tVectorIndex = 0;
        mNumStates = aInput[tVectorIndex].size();
        assert(mNumStates > static_cast<OrdinalType>(0));
        mState = aInput.create();
    }

    /********************************************************************************/
    void allocateState(const Plato::Vector<ScalarType, OrdinalType> & aInput, OrdinalType aNumVectors = 1)
    /********************************************************************************/
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mNumStates = aInput.size();
        mState = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(aNumVectors, aInput);
    }

    /********************************************************************************/
    void allocateStateReductionOperations(const Plato::ReductionOperations<ScalarType, OrdinalType> & aInput)
    /********************************************************************************/
    {
        mStateReductionOperations = aInput.create();
    }

    /********************************************************************************/
    void allocateControl(const OrdinalType & aNumElements, OrdinalType aNumVectors = 1)
    /********************************************************************************/
    {
        assert(aNumElements > static_cast<OrdinalType>(0));
        mNumControls = aNumElements;
        Plato::StandardVector<ScalarType, OrdinalType> tVector(aNumElements);
        mControl = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(aNumVectors, tVector);
    }

    /********************************************************************************/
    void allocateControl(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    /********************************************************************************/
    {
        assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
        const OrdinalType tVectorIndex = 0;
        mNumControls = aInput[tVectorIndex].size();
        assert(mNumControls > static_cast<OrdinalType>(0));
        mControl = aInput.create();
    }

    /********************************************************************************/
    void allocateControl(const Plato::Vector<ScalarType, OrdinalType> & aInput, OrdinalType aNumVectors = 1)
    /********************************************************************************/
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mNumControls = aInput.size();
        mControl = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(aNumVectors, aInput);
    }

    /********************************************************************************/
    void allocateControlReductionOperations(const Plato::ReductionOperations<ScalarType, OrdinalType> & aInput)
    /********************************************************************************/
    {
        mControlReductionOperations = aInput.create();
    }

    /********************************************************************************/
    void allocateLowerBoundVector(const Plato::Vector<ScalarType, OrdinalType> & aInput, OrdinalType aNumVectors = 1)
    /********************************************************************************/
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mNumLowerBounds = aInput.size();
        mLowerBoundVector = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(aNumVectors, aInput);
    }

    /********************************************************************************/
    void allocateUpperBoundVector(const Plato::Vector<ScalarType, OrdinalType> & aInput, OrdinalType aNumVectors = 1)
    /********************************************************************************/
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mNumUpperBounds = aInput.size();
        mUpperBoundVector = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(aNumVectors, aInput);
    }

    /********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & dual() const
    /********************************************************************************/
    {
        assert(mDual.get() != nullptr);
        return (mDual.operator*());
    }

    /********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & dual(const OrdinalType & aVectorIndex) const
    /********************************************************************************/
    {
        assert(mDual.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mDual->getNumVectors());

        return (mDual.operator *().operator [](aVectorIndex));
    }

    /********************************************************************************/
    const Plato::ReductionOperations<ScalarType, OrdinalType> & getDualReductionOperations() const
    /********************************************************************************/
    {
        assert(mDualReductionOperations.get() != nullptr);
        return (mDualReductionOperations.operator *());
    }

    /********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & state() const
    /********************************************************************************/
    {
        assert(mState.get() != nullptr);
        return (mState.operator *());
    }

    /********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & state(const OrdinalType & aVectorIndex) const
    /********************************************************************************/
    {
        assert(mState.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mState->getNumVectors());

        return (mState.operator *().operator [](aVectorIndex));
    }

    /********************************************************************************/
    const Plato::ReductionOperations<ScalarType, OrdinalType> & getStateReductionOperations() const
    /********************************************************************************/
    {
        assert(mStateReductionOperations.get() != nullptr);
        return (mStateReductionOperations.operator *());
    }

    /********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & control() const
    /********************************************************************************/
    {
        assert(mControl.get() != nullptr);
        return (mControl.operator *());
    }

    /********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & control(const OrdinalType & aVectorIndex) const
    /********************************************************************************/
    {
        assert(mControl.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mControl->getNumVectors());

        return (mControl.operator *().operator [](aVectorIndex));
    }

    /********************************************************************************/
    const Plato::ReductionOperations<ScalarType, OrdinalType> & getControlReductionOperations() const
    /********************************************************************************/
    {
        assert(mControlReductionOperations.get() != nullptr);
        return (mControlReductionOperations.operator *());
    }

private:
    OrdinalType mNumDuals;
    OrdinalType mNumStates;
    OrdinalType mNumControls;
    OrdinalType mNumLowerBounds;
    OrdinalType mNumUpperBounds;

    Plato::MemorySpace::type_t mMemorySpace;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDual;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mState;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerBoundVector;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperBoundVector;

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mDualReductionOperations;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mStateReductionOperations;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductionOperations;

private:
    DataFactory(const Plato::DataFactory<ScalarType, OrdinalType>&);
    Plato::DataFactory<ScalarType, OrdinalType> & operator=(const Plato::DataFactory<ScalarType, OrdinalType>&);
};

}

#endif /* PLATO_DATAFACTORY_HPP_ */
