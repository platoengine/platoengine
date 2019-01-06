/*
 * Plato_DataFactory.hpp
 *
 *  Created on: Oct 17, 2017
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
//
// *************************************************************************
//@HEADER
*/

#ifndef PLATO_DATAFACTORY_HPP_
#define PLATO_DATAFACTORY_HPP_

#include <cassert>

#include "Plato_CommWrapper.hpp"
#include "Plato_StandardMultiVector.hpp"
#include "Plato_StandardVectorReductionOperations.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class DataFactory
{
public:
    /******************************************************************************//**
     * @brief Default constructor
     * @param [in] aMemorySpace memory space (HOST OR DEVICE). Default = HOST
    **********************************************************************************/
    DataFactory(Plato::MemorySpace::type_t aMemorySpace = Plato::MemorySpace::HOST) :
            mNumDuals(1),
            mNumStates(1),
            mNumControls(0),
            mNumObjFuncValues(0),
            mMemorySpace(aMemorySpace),
            mCommWrapper(std::make_shared<Plato::CommWrapper>()),
            mObjFuncValues(),
            mDual(),
            mState(),
            mControl(),
            mLowerBoundVector(),
            mUpperBoundVector(),
            mDualReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>()),
            mStateReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>()),
            mControlReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>()),
            mObjFuncReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType, OrdinalType>>())
    {
        mCommWrapper->useDefaultComm();
        this->allocateDual(mNumDuals);
        this->allocateState(mNumStates);
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~DataFactory()
    {
    }

    /******************************************************************************//**
     * @brief Return number of controls
     * @return number of controls
    **********************************************************************************/
    OrdinalType getNumControls() const
    {
        assert(mNumControls > static_cast<ScalarType>(0));
        return (mNumControls);
    }

    /******************************************************************************//**
     * @brief Return number of control upper bound entries
     * @return number of control upper bound entries
    **********************************************************************************/
    OrdinalType getNumCriterionValues() const
    {
        assert(mNumObjFuncValues > static_cast<ScalarType>(0));
        return (mNumObjFuncValues);
    }

    /******************************************************************************//**
     * @brief Return memory space flag
     * @return memory space flag
    **********************************************************************************/
    Plato::MemorySpace::type_t getMemorySpace() const
    {
        return (mMemorySpace);
    }

    /******************************************************************************//**
     * @brief Set distributed memory communication wrapper
     * @param [in] aInput Plato communication wrapper
    **********************************************************************************/
    void setCommWrapper(const Plato::CommWrapper & aInput)
    {
        mCommWrapper.reset();
        mCommWrapper = aInput.create();
    }

    /******************************************************************************//**
     * @brief Return a const reference to the distributed memory communication wrapper
     * @return const reference to the distributed memory communication wrapper
    **********************************************************************************/
    const Plato::CommWrapper& getCommWrapper() const
    {
        return (mCommWrapper.operator*());
    }

    /******************************************************************************//**
     * @brief Allocate a Plato vector of objective function values.
     * @param [in] aNumElements number of elements
    **********************************************************************************/
    void allocateObjFuncValues(const OrdinalType & aNumElements)
    {
        assert(aNumElements > static_cast<OrdinalType>(0));

        if(mObjFuncValues.use_count() != 0)
        {
            mObjFuncValues.reset();
        }
        mNumObjFuncValues = aNumElements;
        mObjFuncValues = std::make_shared<Plato::StandardVector<ScalarType, OrdinalType>>(aNumElements);
    }

    /******************************************************************************//**
     * @brief Allocate objective function values vector
     * @param [in] aInput template vector of objective function values
    **********************************************************************************/
    void allocateObjFuncValues(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));

        if(mObjFuncValues.use_count() != 0)
        {
            mObjFuncValues.reset();
        }
        mNumObjFuncValues = aInput.size();
        mObjFuncValues = aInput.create();
    }

    /******************************************************************************//**
     * @brief Allocates template for the objective function values reduction operations.
     * @param [in] aInput reduction operations wrapper.
    **********************************************************************************/
    void allocateObjFuncReductionOperations(const Plato::ReductionOperations<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.get() != nullptr);

        if(mObjFuncReductionOperations.use_count() != 0)
        {
            mObjFuncReductionOperations.reset();
        }
        mObjFuncReductionOperations = aInput.create();
    }

    /******************************************************************************//**
     * @brief Allocate a Plato multi-vector of dual variables
     * @param [in] aNumElements number of elements in a given vector
     * @param [in] aNumVectors number of vectors (default = 1)
    **********************************************************************************/
    void allocateDual(const OrdinalType & aNumElements, OrdinalType aNumVectors = 1)
    {
        if(mDual.use_count() != 0)
        {
            mDual.reset();
        }

        assert(aNumElements > static_cast<OrdinalType>(0));
        mNumDuals = aNumElements;
        Plato::StandardVector<ScalarType, OrdinalType> tVector(aNumElements);
        mDual = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(aNumVectors, tVector);
    }

    /******************************************************************************//**
     * @brief Allocate dual multi-vector
     * @param [in] aInput Plato multi-vector
    **********************************************************************************/
    void allocateDual(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() > static_cast<OrdinalType>(0));
        const OrdinalType tVectorIndex = 0;
        mNumDuals = aInput[tVectorIndex].size();
        assert(mNumDuals > static_cast<OrdinalType>(0));
        mDual = aInput.create();
    }

    /******************************************************************************//**
     * @brief Allocates dual multi-vector.
     * @param [in] aInput Plato standard multi-vector
    **********************************************************************************/
    void allocateDual(const Plato::Vector<ScalarType, OrdinalType> & aInput, OrdinalType aNumVectors = 1)
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mNumDuals = aInput.size();
        mDual = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(aNumVectors, aInput);
    }

    /******************************************************************************//**
     * @brief Allocates template for dual reduction operations.
     * @param [in] aInput reduction operations wrapper.
    **********************************************************************************/
    void allocateDualReductionOperations(const Plato::ReductionOperations<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.get() != nullptr);

        if(mDualReductionOperations.use_count() != 0)
        {
            mDualReductionOperations.reset();
        }
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

    /******************************************************************************//**
     * @brief Allocates template for state reduction operations.
     * @param [in] aInput reduction operations wrapper.
    **********************************************************************************/
    void allocateStateReductionOperations(const Plato::ReductionOperations<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.get() != nullptr);

        if(mStateReductionOperations.use_count() != 0)
        {
            mStateReductionOperations.reset();
        }
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

    /******************************************************************************//**
     * @brief Allocates template for control reduction operations.
     * @param [in] aInput reduction operations wrapper.
    **********************************************************************************/
    void allocateControlReductionOperations(const Plato::ReductionOperations<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.get() != nullptr);

        if(mControlReductionOperations.use_count() != 0)
        {
            mControlReductionOperations.reset();
        }
        mControlReductionOperations = aInput.create();
    }

    /********************************************************************************/
    void allocateLowerBoundVector(const Plato::Vector<ScalarType, OrdinalType> & aInput, OrdinalType aNumVectors = 1)
    /********************************************************************************/
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mLowerBoundVector = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(aNumVectors, aInput);
    }

    /********************************************************************************/
    void allocateUpperBoundVector(const Plato::Vector<ScalarType, OrdinalType> & aInput, OrdinalType aNumVectors = 1)
    /********************************************************************************/
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mUpperBoundVector = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(aNumVectors, aInput);
    }

    /******************************************************************************//**
     * @brief Returns const reference to dual multi-vector template.
     * @return const reference to multi-vector of dual values.
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & dual() const
    {
        assert(mDual.get() != nullptr);
        return (mDual.operator*());
    }

    /******************************************************************************//**
     * @brief Returns const reference to dual vector template.
     * @param [in] aVectorIndex Position of an element in the multi-vector.
     * @return const reference to dual vector.
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & dual(const OrdinalType & aVectorIndex) const
    {
        assert(mDual.get() != nullptr);
        assert(aVectorIndex < mDual->getNumVectors());

        return (mDual.operator *().operator [](aVectorIndex));
    }

    /******************************************************************************//**
     * @brief Returns const reference to reduction operations wrapper for duals.
     * @return const reference to reduction operations wrapper.
    **********************************************************************************/
    const Plato::ReductionOperations<ScalarType, OrdinalType> & getDualReductionOperations() const
    {
        assert(mDualReductionOperations.get() != nullptr);
        return (mDualReductionOperations.operator *());
    }

    /******************************************************************************//**
     * @brief Returns const reference to state multi-vector template
     * @return const reference to multi-vector of dual values
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & state() const
    {
        assert(mState.get() != nullptr);
        return (mState.operator *());
    }

    /******************************************************************************//**
     * @brief Returns const reference to state vector template.
     * @param [in] aVectorIndex Position of an element in the multi-vector.
     * @return const reference to state vector.
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & state(const OrdinalType & aVectorIndex) const
    {
        assert(mState.get() != nullptr);
        assert(aVectorIndex < mState->getNumVectors());

        return (mState.operator *().operator [](aVectorIndex));
    }

    /******************************************************************************//**
     * @brief Returns const reference to reduction operations wrapper for states.
     * @return const reference to reduction operations wrapper.
    **********************************************************************************/
    const Plato::ReductionOperations<ScalarType, OrdinalType> & getStateReductionOperations() const
    {
        assert(mStateReductionOperations.get() != nullptr);
        return (mStateReductionOperations.operator *());
    }

    /******************************************************************************//**
     * @brief Returns const reference to control multi-vector template
     * @return const reference to multi-vector of dual values
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & control() const
    {
        assert(mControl.get() != nullptr);
        return (mControl.operator *());
    }

    /******************************************************************************//**
     * @brief Returns const reference to control vector template.
     * @param [in] aVectorIndex Position of an element in the multi-vector.
     * @return const reference to state vector.
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & control(const OrdinalType & aVectorIndex) const
    {
        assert(mControl.get() != nullptr);
        assert(aVectorIndex < mControl->getNumVectors());

        return (mControl.operator *().operator [](aVectorIndex));
    }

    /******************************************************************************//**
     * @brief Returns const reference to reduction operations wrapper for controls.
     * @return const reference to reduction operations wrapper.
    **********************************************************************************/
    const Plato::ReductionOperations<ScalarType, OrdinalType> & getControlReductionOperations() const
    {
        assert(mControlReductionOperations.get() != nullptr);
        return (mControlReductionOperations.operator *());
    }

    /******************************************************************************//**
     * @brief Returns const reference to vector template of objective function values.
     * @return const reference to vector of objective function values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & objective() const
    {
        assert(mObjFuncValues.get() != nullptr);
        return (mObjFuncValues.operator*());
    }

    /******************************************************************************//**
     * @brief Returns const reference to reduction operations wrapper for objective function values.
     * @return const reference to reduction operations wrapper.
    **********************************************************************************/
    const Plato::ReductionOperations<ScalarType, OrdinalType> & getObjFuncReductionOperations() const
    {
        assert(mObjFuncReductionOperations.get() != nullptr);
        return (mObjFuncReductionOperations.operator *());
    }

private:
    OrdinalType mNumDuals;
    OrdinalType mNumStates;
    OrdinalType mNumControls;
    OrdinalType mNumObjFuncValues;

    Plato::MemorySpace::type_t mMemorySpace;

    std::shared_ptr<Plato::CommWrapper> mCommWrapper;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mObjFuncValues;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDual;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mState;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerBoundVector;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperBoundVector;

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mDualReductionOperations;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mStateReductionOperations;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductionOperations;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mObjFuncReductionOperations;

private:
    DataFactory(const Plato::DataFactory<ScalarType, OrdinalType>&);
    Plato::DataFactory<ScalarType, OrdinalType> & operator=(const Plato::DataFactory<ScalarType, OrdinalType>&);
};

}

#endif /* PLATO_DATAFACTORY_HPP_ */
