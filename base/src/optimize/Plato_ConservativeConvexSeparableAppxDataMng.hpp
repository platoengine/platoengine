/*
 * Plato_ConservativeConvexSeparableAppxDataMng.hpp
 *
 *  Created on: Nov 4, 2017
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

#ifndef PLATO_CONSERVATIVECONVEXSEPARABLEAPPXDATAMNG_HPP_
#define PLATO_CONSERVATIVECONVEXSEPARABLEAPPXDATAMNG_HPP_

#include <cmath>
#include <limits>
#include <vector>
#include <memory>
#include <numeric>
#include <cassert>
#include <algorithm>

#include "Plato_HostBounds.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DeviceBounds.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_StandardVector.hpp"
#include "Plato_MultiVectorList.hpp"
#include "Plato_ReductionOperations.hpp"
#include "Plato_HostPrimalProbElementWise.hpp"
#include "Plato_DevicePrimalProbElementWise.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class ConservativeConvexSeparableAppxDataMng
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param aDataFactory vector and multi-vector factory
    **********************************************************************************/
    explicit ConservativeConvexSeparableAppxDataMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory) :
            mIsInitialGuessSet(false),
            mNumObjFuncEval(0),
            mNumObjGradEval(0),
            mControlStagnationMeasure(std::numeric_limits<ScalarType>::max()),
            mFeasibilityMeasure(std::numeric_limits<ScalarType>::max()),
            mStationarityMeasure(std::numeric_limits<ScalarType>::max()),
            mNormInactiveGradient(std::numeric_limits<ScalarType>::max()),
            mObjectiveStagnationMeasure(std::numeric_limits<ScalarType>::max()),
            mDualProblemBoundsScaleFactor(0.5),
            mCurrentObjectiveFunctionValue(std::numeric_limits<ScalarType>::max()),
            mPreviousObjectiveFunctionValue(std::numeric_limits<ScalarType>::max()),
            mDualObjectiveGlobalizationFactor(1),
            mKarushKuhnTuckerConditionsInexactness(std::numeric_limits<ScalarType>::max()),
            mMemorySpace(aDataFactory->getMemorySpace()),
            mDualWorkVectorOne(),
            mDualWorkVectorTwo(),
            mControlWorkVectorOne(),
            mControlWorkVectorTwo(),
            mDual(aDataFactory->dual().create()),
            mTrialStep(aDataFactory->control().create()),
            mActiveSet(aDataFactory->control().create()),
            mInactiveSet(aDataFactory->control().create()),
            mCurrentSigma(aDataFactory->control().create()),
            mCurrentControl(aDataFactory->control().create()),
            mPreviousControl(aDataFactory->control().create()),
            mControlLowerBounds(aDataFactory->control().create()),
            mControlUpperBounds(aDataFactory->control().create()),
            mControlWorkMultiVector(aDataFactory->control().create()),
            mCurrentConstraintValues(aDataFactory->dual().create()),
            mCurrentObjectiveGradient(aDataFactory->control().create()),
            mConstraintGlobalizationFactors(aDataFactory->dual().create()),
            mCommWrapper(aDataFactory->getCommWrapper().create()),
            mBounds(nullptr),
            mDualReductions(aDataFactory->getDualReductionOperations().create()),
            mControlReductions(aDataFactory->getControlReductionOperations().create()),
            mElementWiseFunctions(nullptr),
            mCurrentConstraintGradients(std::make_shared<Plato::MultiVectorList<ScalarType, OrdinalType>>())
    {
        this->initialize();
        this->initializeBounds();
        this->initializeElementWiseFunctions();
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    ~ConservativeConvexSeparableAppxDataMng()
    {
    }

    /******************************************************************************//**
     * @brief Check if initial guess is set
     * @return flag (true/false) specifying if initial control guess has been set
    **********************************************************************************/
    bool isInitialGuessSet() const
    {
        return (mIsInitialGuessSet);
    }

    /******************************************************************************//**
     * @brief Return memory space (e.g. HOST or DEVICE)
     * @return memory space type
    **********************************************************************************/
    Plato::MemorySpace::type_t getMemorySpace() const
    {
        return (mMemorySpace);
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
     * @brief Get number of objective function evaluations
     * @return number of objective function evaluations
    **********************************************************************************/
    OrdinalType getNumObjectiveFunctionEvaluations() const
    {
        return (mNumObjFuncEval);
    }

    /******************************************************************************//**
     * @brief Set number of objective function evaluations
     * @param [in] aInput number of objective function evaluations
    **********************************************************************************/
    void setNumObjectiveFunctionEvaluations(const OrdinalType& aInput)
    {
        mNumObjFuncEval = aInput;
    }

    /******************************************************************************//**
     * @brief Get number of objective gradient evaluations
     * @return number of objective gradient evaluations
    **********************************************************************************/
    OrdinalType getNumObjectiveGradientEvaluations() const
    {
        return (mNumObjGradEval);
    }

    /******************************************************************************//**
     * @brief Set number of objective gradient evaluations
     * @param [in] aInput number of objective gradient evaluations
    **********************************************************************************/
    void setNumObjectiveGradientEvaluations(const OrdinalType& aInput)
    {
        mNumObjGradEval = aInput;
    }

    // NOTE: NUMBER OF CONTROL VECTORS
    OrdinalType getNumControlVectors() const
    {
        return (mCurrentControl->getNumVectors());
    }
    // NOTE: NUMBER OF DUAL VECTORS
    OrdinalType getNumDualVectors() const
    {
        return (mDual->getNumVectors());
    }
    // NOTE :GET NUMBER OF CONSTRAINTS
    OrdinalType getNumConstraints() const
    {
        OrdinalType tNumConstraints = mCurrentConstraintGradients->size();
        return (tNumConstraints);
    }

    // NOTE: DUAL PROBLEM PARAMETERS
    ScalarType getDualProblemBoundsScaleFactor() const
    {
        return (mDualProblemBoundsScaleFactor);
    }
    void setDualProblemBoundsScaleFactor(const ScalarType & aInput)
    {
        mDualProblemBoundsScaleFactor = aInput;
    }
    ScalarType getDualObjectiveGlobalizationFactor() const
    {
        return (mDualObjectiveGlobalizationFactor);
    }
    void setDualObjectiveGlobalizationFactor(const ScalarType & aInput)
    {
        mDualObjectiveGlobalizationFactor = aInput;
    }

    // NOTE: CONSTRAINT GLOBALIZATION FACTORS FUNCTIONS
    const Plato::MultiVector<ScalarType, OrdinalType> & getConstraintGlobalizationFactors() const
    {
        assert(mConstraintGlobalizationFactors.get() != nullptr);
        return (mConstraintGlobalizationFactors.operator*());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getConstraintGlobalizationFactors(const OrdinalType & aVectorIndex) const
    {
        assert(mConstraintGlobalizationFactors.get() != nullptr);
        assert(aVectorIndex < mConstraintGlobalizationFactors->getNumVectors());
        return (mConstraintGlobalizationFactors->operator [](aVectorIndex));
    }
    void setConstraintGlobalizationFactors(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mConstraintGlobalizationFactors->getNumVectors());
        Plato::update(1., aInput, 0., *mConstraintGlobalizationFactors);
    }
    void setConstraintGlobalizationFactors(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mConstraintGlobalizationFactors.get() != nullptr);
        assert(aVectorIndex < mConstraintGlobalizationFactors->getNumVectors());
        mConstraintGlobalizationFactors->operator [](aVectorIndex).update(1., aInput, 0.);
    }

    // NOTE: OBJECTIVE FUNCTION VALUE
    ScalarType getCurrentObjectiveFunctionValue() const
    {
        return (mCurrentObjectiveFunctionValue);
    }
    void setCurrentObjectiveFunctionValue(const ScalarType & aInput)
    {
        mCurrentObjectiveFunctionValue = aInput;
    }
    ScalarType getPreviousObjectiveFunctionValue() const
    {
        return (mPreviousObjectiveFunctionValue);
    }
    void setPreviousObjectiveFunctionValue(const ScalarType & aInput)
    {
        mPreviousObjectiveFunctionValue = aInput;
    }

    // NOTE: SET INITIAL GUESS
    void setInitialGuess(const ScalarType & aValue)
    {
        assert(mCurrentControl.get() != nullptr);
        assert(mCurrentControl->getNumVectors() > static_cast<OrdinalType>(0));

        OrdinalType tNumVectors = mCurrentControl->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            mCurrentControl->operator [](tVectorIndex).fill(aValue);
        }
        mIsInitialGuessSet = true;
    }
    void setInitialGuess(const OrdinalType & aVectorIndex, const ScalarType & aValue)
    {
        assert(mCurrentControl.get() != nullptr);
        assert(aVectorIndex < mCurrentControl->getNumVectors());

        mCurrentControl->operator [](aVectorIndex).fill(aValue);
        mIsInitialGuessSet = true;
    }
    void setInitialGuess(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentControl.get() != nullptr);
        assert(aVectorIndex < mCurrentControl->getNumVectors());

        mCurrentControl->operator [](aVectorIndex).update(1., aInput, 0.);
        mIsInitialGuessSet = true;
    }
    void setInitialGuess(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mCurrentControl->getNumVectors());
        Plato::update(1., aInput, 0., *mCurrentControl);
        mIsInitialGuessSet = true;
    }

    // NOTE: DUAL VECTOR
    const Plato::MultiVector<ScalarType, OrdinalType> & getDual() const
    {
        assert(mDual.get() != nullptr);
        return (mDual.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getDual(const OrdinalType & aVectorIndex) const
    {
        assert(mDual.get() != nullptr);
        assert(aVectorIndex < mDual->getNumVectors());
        return (mDual->operator [](aVectorIndex));
    }
    void setDual(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mDual->getNumVectors());
        Plato::update(1., aInput, 0., *mDual);
    }
    void setDual(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mDual.get() != nullptr);
        assert(aVectorIndex < mDual->getNumVectors());
        mDual->operator [](aVectorIndex).update(1., aInput, 0.);
    }

    // NOTE: TRIAL STEP FUNCTIONS
    const Plato::MultiVector<ScalarType, OrdinalType> & getTrialStep() const
    {
        assert(mTrialStep.get() != nullptr);

        return (mTrialStep.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getTrialStep(const OrdinalType & aVectorIndex) const
    {
        assert(mTrialStep.get() != nullptr);
        assert(aVectorIndex < mTrialStep->getNumVectors());

        return (mTrialStep->operator [](aVectorIndex));
    }
    void setTrialStep(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mTrialStep->getNumVectors());
        Plato::update(1., aInput, 0., *mTrialStep);
    }
    void setTrialStep(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mTrialStep.get() != nullptr);
        assert(aVectorIndex < mTrialStep->getNumVectors());

        mTrialStep->operator [](aVectorIndex).update(1., aInput, 0.);
    }

    // NOTE: ACTIVE SET FUNCTIONS
    const Plato::MultiVector<ScalarType, OrdinalType> & getActiveSet() const
    {
        assert(mActiveSet.get() != nullptr);

        return (mActiveSet.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getActiveSet(const OrdinalType & aVectorIndex) const
    {
        assert(mActiveSet.get() != nullptr);
        assert(aVectorIndex < mActiveSet->getNumVectors());

        return (mActiveSet->operator [](aVectorIndex));
    }
    void setActiveSet(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mActiveSet->getNumVectors());
        Plato::update(1., aInput, 0., *mActiveSet);
    }
    void setActiveSet(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mActiveSet.get() != nullptr);
        assert(aVectorIndex < mActiveSet->getNumVectors());

        mActiveSet->operator [](aVectorIndex).update(1., aInput, 0.);
    }

    // NOTE: INACTIVE SET FUNCTIONS
    const Plato::MultiVector<ScalarType, OrdinalType> & getInactiveSet() const
    {
        assert(mInactiveSet.get() != nullptr);

        return (mInactiveSet.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getInactiveSet(const OrdinalType & aVectorIndex) const
    {
        assert(mInactiveSet.get() != nullptr);
        assert(aVectorIndex < mInactiveSet->getNumVectors());

        return (mInactiveSet->operator [](aVectorIndex));
    }
    void setInactiveSet(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mInactiveSet->getNumVectors());
        Plato::update(1., aInput, 0., *mInactiveSet);
    }
    void setInactiveSet(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mInactiveSet.get() != nullptr);
        assert(aVectorIndex < mInactiveSet->getNumVectors());

        mInactiveSet->operator [](aVectorIndex).update(1., aInput, 0.);
    }

    // NOTE: CURRENT CONTROL
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentControl() const
    {
        assert(mCurrentControl.get() != nullptr);

        return (mCurrentControl.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentControl(const OrdinalType & aVectorIndex) const
    {
        assert(mCurrentControl.get() != nullptr);
        assert(aVectorIndex < mCurrentControl->getNumVectors());

        return (mCurrentControl->operator [](aVectorIndex));
    }
    void setCurrentControl(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mCurrentControl->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentControl);
    }
    void setCurrentControl(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentControl.get() != nullptr);
        assert(aVectorIndex < mCurrentControl->getNumVectors());

        mCurrentControl->operator [](aVectorIndex).update(1., aInput, 0.);
    }

    // NOTE: PREVIOUS CONTROL
    const Plato::MultiVector<ScalarType, OrdinalType> & getPreviousControl() const
    {
        assert(mPreviousControl.get() != nullptr);

        return (mPreviousControl.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getPreviousControl(const OrdinalType & aVectorIndex) const
    {
        assert(mPreviousControl.get() != nullptr);
        assert(aVectorIndex < mPreviousControl->getNumVectors());

        return (mPreviousControl->operator [](aVectorIndex));
    }
    void setPreviousControl(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mPreviousControl->getNumVectors());
        Plato::update(1., aInput, 0., *mPreviousControl);
    }
    void setPreviousControl(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mPreviousControl.get() != nullptr);
        assert(aVectorIndex < mPreviousControl->getNumVectors());

        mPreviousControl->operator [](aVectorIndex).update(1., aInput, 0.);
    }

    // NOTE: CURRENT OBJECTIVE GRADIENT
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentObjectiveGradient() const
    {
        assert(mCurrentObjectiveGradient.get() != nullptr);

        return (mCurrentObjectiveGradient.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentObjectiveGradient(const OrdinalType & aVectorIndex) const
    {
        assert(mCurrentObjectiveGradient.get() != nullptr);
        assert(aVectorIndex < mCurrentObjectiveGradient->getNumVectors());

        return (mCurrentObjectiveGradient->operator [](aVectorIndex));
    }
    void setCurrentObjectiveGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mCurrentObjectiveGradient->getNumVectors());
        Plato::update(1., aInput, 0., *mCurrentObjectiveGradient);
    }
    void setCurrentObjectiveGradient(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentObjectiveGradient.get() != nullptr);
        assert(aVectorIndex < mCurrentObjectiveGradient->getNumVectors());

        mCurrentObjectiveGradient->operator [](aVectorIndex).update(1., aInput, 0.);
    }

    // NOTE: CURRENT SIGMA VECTOR
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentSigma() const
    {
        assert(mCurrentSigma.get() != nullptr);

        return (mCurrentSigma.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentSigma(const OrdinalType & aVectorIndex) const
    {
        assert(mCurrentSigma.get() != nullptr);
        assert(aVectorIndex < mCurrentSigma->getNumVectors());

        return (mCurrentSigma->operator [](aVectorIndex));
    }
    void setCurrentSigma(const ScalarType & aInput)
    {
        assert(mCurrentSigma.get() != nullptr);
        Plato::fill(aInput, mCurrentSigma.operator*());
    }
    void setCurrentSigma(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mCurrentSigma->getNumVectors());
        Plato::update(1., aInput, 0., *mCurrentSigma);
    }
    void setCurrentSigma(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentSigma.get() != nullptr);
        assert(aVectorIndex < mCurrentSigma->getNumVectors());

        mCurrentSigma->operator [](aVectorIndex).update(1., aInput, 0.);
    }

    // NOTE: SET CONTROL LOWER BOUNDS
    const Plato::MultiVector<ScalarType, OrdinalType> & getControlLowerBounds() const
    {
        assert(mControlLowerBounds.get() != nullptr);

        return (mControlLowerBounds.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getControlLowerBounds(const OrdinalType & aVectorIndex) const
    {
        assert(mControlLowerBounds.get() != nullptr);
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
        assert(aVectorIndex < mControlLowerBounds->getNumVectors());

        mControlLowerBounds->operator [](aVectorIndex).fill(aValue);
    }
    void setControlLowerBounds(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mControlLowerBounds.get() != nullptr);
        assert(aVectorIndex < mControlLowerBounds->getNumVectors());

        mControlLowerBounds->operator [](aVectorIndex).update(1., aInput, 0.);
    }
    void setControlLowerBounds(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mControlLowerBounds->getNumVectors());
        Plato::update(1., aInput, 0., *mControlLowerBounds);
    }

    // NOTE: SET CONTROL UPPER BOUNDS
    const Plato::MultiVector<ScalarType, OrdinalType> & getControlUpperBounds() const
    {
        assert(mControlUpperBounds.get() != nullptr);

        return (mControlUpperBounds.operator *());
    }
    const Plato::Vector<ScalarType, OrdinalType> & getControlUpperBounds(const OrdinalType & aVectorIndex) const
    {
        assert(mControlUpperBounds.get() != nullptr);
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
        assert(aVectorIndex < mControlUpperBounds->getNumVectors());

        mControlUpperBounds->operator [](aVectorIndex).fill(aValue);
    }
    void setControlUpperBounds(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mControlUpperBounds.get() != nullptr);
        assert(aVectorIndex < mControlUpperBounds->getNumVectors());

        mControlUpperBounds->operator [](aVectorIndex).update(1., aInput, 0.);
    }
    void setControlUpperBounds(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mControlUpperBounds->getNumVectors());
        Plato::update(1., aInput, 0., *mControlUpperBounds);
    }

    /******************************************************************************//**
     * @brief Return multi-vector of constraint values
     * @return multi-vector of constraint values
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentConstraintValues() const
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        return (mCurrentConstraintValues.operator *());
    }

    /******************************************************************************//**
     * @brief Return vector of constraint values
     * @param [in] aVectorIndex dual vector index (default = 1)
     * @return vector of constraint values
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentConstraintValues(const OrdinalType & aVectorIndex) const
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        assert(aVectorIndex < mCurrentConstraintValues->getNumVectors());
        return (mCurrentConstraintValues->operator [](aVectorIndex));
    }

    /******************************************************************************//**
     * @brief Return constraint value
     * @param [in] aVectorIndex dual vector index
     * @param [in] aConstraintIndex constraint index
     * @return constraint value
    **********************************************************************************/
    ScalarType getCurrentConstraintValue(const OrdinalType & aVectorIndex, const OrdinalType & aConstraintIndex) const
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        assert(aVectorIndex < mCurrentConstraintValues->getNumVectors());
        return ((*mCurrentConstraintValues)[aVectorIndex][aConstraintIndex]);
    }

    /******************************************************************************//**
     * @brief Set current constraint values
     * @param [in] aInput constraint values
    **********************************************************************************/
    void setCurrentConstraintValues(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.getNumVectors() == mCurrentConstraintValues->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mCurrentConstraintValues);
    }

    /******************************************************************************//**
     * @brief Set current constraint values
     * @param [in] aVectorIndex dual vector index
     * @param [in] aInput constraint values
    **********************************************************************************/
    void setCurrentConstraintValues(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentConstraintValues.get() != nullptr);
        assert(aVectorIndex < mCurrentConstraintValues->getNumVectors());
        mCurrentConstraintValues->operator [](aVectorIndex).update(1., aInput, 0.);
    }

    /******************************************************************************//**
     * @brief Return list with constraint gradients
     * @return constraint gradients
    **********************************************************************************/
    const Plato::MultiVectorList<ScalarType, OrdinalType> & getCurrentConstraintGradients() const
    {
        assert(mCurrentConstraintGradients.get() != nullptr);
        return (mCurrentConstraintGradients.operator*());
    }
    const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentConstraintGradients(const OrdinalType & aConstraintIndex) const
    {
        assert(mCurrentConstraintGradients.get() != nullptr);
        assert(aConstraintIndex < mCurrentConstraintGradients->size());
        assert(mCurrentConstraintGradients->ptr(aConstraintIndex).get() != nullptr);

        return (mCurrentConstraintGradients->operator[](aConstraintIndex));
    }
    const Plato::Vector<ScalarType, OrdinalType> & getCurrentConstraintGradients(const OrdinalType & aConstraintIndex,
                                                                                 const OrdinalType & aVectorIndex) const
    {
        assert(mCurrentConstraintGradients.get() != nullptr);
        assert(aConstraintIndex < mCurrentConstraintGradients->size());
        assert(mCurrentConstraintGradients->ptr(aConstraintIndex).get() != nullptr);
        assert(aVectorIndex < mCurrentConstraintGradients->operator[](aConstraintIndex).getNumVectors());

        return (mCurrentConstraintGradients->operator()(aConstraintIndex, aVectorIndex));
    }
    void setCurrentConstraintGradients(const Plato::MultiVectorList<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));

        const OrdinalType tNumConstraints = aInput.size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            Plato::update(static_cast<ScalarType>(1),
                          aInput[tConstraintIndex],
                          static_cast<ScalarType>(0),
                          mCurrentConstraintGradients->operator[](tConstraintIndex));
        }
    }
    void setCurrentConstraintGradients(const OrdinalType & aConstraintIndex,
                                       const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentConstraintGradients.get() != nullptr);
        assert(aConstraintIndex < mCurrentConstraintGradients->size());
        assert(mCurrentConstraintGradients->ptr(aConstraintIndex).get() != nullptr);
        assert(aInput.getNumVectors() == mCurrentConstraintGradients->operator[](aConstraintIndex).getNumVectors());

        Plato::update(static_cast<ScalarType>(1),
                      aInput,
                      static_cast<ScalarType>(0),
                      mCurrentConstraintGradients->operator[](aConstraintIndex));
    }
    void setCurrentConstraintGradients(const OrdinalType & aConstraintIndex,
                                       const OrdinalType & aVectorIndex,
                                       const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mCurrentConstraintGradients.get() != nullptr);
        assert(aConstraintIndex < mCurrentConstraintGradients->size());
        assert(aVectorIndex < mCurrentConstraintGradients->operator[](aConstraintIndex).getNumVectors());
        assert(aInput.size() == mCurrentConstraintGradients->operator()(aConstraintIndex, aVectorIndex).size());

        const ScalarType tAlpha = 1;
        const ScalarType tBeta = 0;
        mCurrentConstraintGradients->operator()(aConstraintIndex, aVectorIndex).update(tAlpha, aInput, tBeta);
    }

    // NOTE: Control STAGNATION MEASURE CRITERION
    void computeControlStagnationMeasure()
    {
        OrdinalType tNumVectors = mCurrentControl->getNumVectors();
        std::vector<ScalarType> storage(tNumVectors, std::numeric_limits<ScalarType>::min());
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyCurrentControl = mCurrentControl->operator[](tIndex);
            mControlWorkVectorOne->update(1., tMyCurrentControl, 0.);
            const Plato::Vector<ScalarType, OrdinalType> & tMyPreviousControl = mPreviousControl->operator[](tIndex);
            mControlWorkVectorOne->update(-1., tMyPreviousControl, 1.);
            mControlWorkVectorOne->modulus();
            storage[tIndex] = mControlReductions->max(*mControlWorkVectorOne);
        }
        mControlStagnationMeasure = *std::max_element(storage.begin(), storage.end());
    }
    ScalarType getControlStagnationMeasure() const
    {
        return (mControlStagnationMeasure);
    }

    // NOTE: NORM OF CURRENT PROJECTED GRADIENT
    ScalarType computeInactiveVectorNorm(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        ScalarType tCummulativeDotProduct = 0.;
        OrdinalType tNumVectors = aInput.getNumVectors();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyInactiveSet = (*mInactiveSet)[tIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tMyInputVector = aInput[tIndex];

            mControlWorkVectorOne->update(1., tMyInputVector, 0.);
            mControlWorkVectorOne->entryWiseProduct(tMyInactiveSet);
            tCummulativeDotProduct += mControlWorkVectorOne->dot(*mControlWorkVectorOne);
        }
        ScalarType tOutput = std::sqrt(tCummulativeDotProduct);
        return(tOutput);
    }
    void computeNormInactiveGradient()
    {
        ScalarType tCummulativeDotProduct = 0.;
        OrdinalType tNumVectors = mCurrentObjectiveGradient->getNumVectors();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyInactiveSet = (*mInactiveSet)[tIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tMyGradient = (*mCurrentObjectiveGradient)[tIndex];

            mControlWorkVectorOne->update(1., tMyGradient, 0.);
            mControlWorkVectorOne->entryWiseProduct(tMyInactiveSet);
            tCummulativeDotProduct += mControlWorkVectorOne->dot(*mControlWorkVectorOne);
        }
        mNormInactiveGradient = (static_cast<ScalarType>(1) / mGlobalNumControls) * std::sqrt(tCummulativeDotProduct);
    }
    ScalarType getNormInactiveGradient() const
    {
        return (mNormInactiveGradient);
    }

    // NOTE: FEASIBILITY MEASURE CALCULATION
    void computeObjectiveStagnationMeasure()
    {
        mObjectiveStagnationMeasure = mPreviousObjectiveFunctionValue - mCurrentObjectiveFunctionValue;
        mObjectiveStagnationMeasure = std::abs(mObjectiveStagnationMeasure);
    }
    ScalarType getObjectiveStagnationMeasure() const
    {
        return (mObjectiveStagnationMeasure);
    }

    // NOTE: FEASIBILITY MEASURE CALCULATION
    void computeFeasibilityMeasure()
    {
        const OrdinalType tNumVectors = mCurrentConstraintValues->getNumVectors();
        std::vector<ScalarType> tStorage(tNumVectors, static_cast<ScalarType>(0));
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyConstraintValues = mCurrentConstraintValues->operator[](tVectorIndex);
            mDualWorkVectorOne->update(static_cast<ScalarType>(1), tMyConstraintValues, static_cast<ScalarType>(0));
            mDualWorkVectorOne->modulus();
            tStorage[tVectorIndex] = mDualReductions->max(mDualWorkVectorOne.operator*());
        }
        const ScalarType tInitialValue = 0;
        mFeasibilityMeasure = std::accumulate(tStorage.begin(), tStorage.end(), tInitialValue);
    }
    ScalarType getFeasibilityMeasure() const
    {
        return (mFeasibilityMeasure);
    }

    // NOTE: STATIONARITY MEASURE CALCULATION
    void computeStationarityMeasure()
    {
        assert(mInactiveSet.get() != nullptr);
        assert(mCurrentControl.get() != nullptr);
        assert(mControlLowerBounds.get() != nullptr);
        assert(mControlUpperBounds.get() != nullptr);
        assert(mCurrentObjectiveGradient.get() != nullptr);

        Plato::update(static_cast<ScalarType>(1), *mCurrentControl, static_cast<ScalarType>(0), *mControlWorkMultiVector);
        Plato::update(static_cast<ScalarType>(-1), *mCurrentObjectiveGradient, static_cast<ScalarType>(1), *mControlWorkMultiVector);
        mBounds->project(*mControlLowerBounds, *mControlUpperBounds, *mControlWorkMultiVector);
        Plato::update(static_cast<ScalarType>(1), *mCurrentControl, static_cast<ScalarType>(-1), *mControlWorkMultiVector);
        Plato::update(static_cast<ScalarType>(1), *mControlWorkMultiVector, static_cast<ScalarType>(0), *mTrialStep);

        Plato::entryWiseProduct(*mInactiveSet, *mControlWorkMultiVector);
        mStationarityMeasure = Plato::norm(*mControlWorkMultiVector);
    }
    ScalarType getStationarityMeasure() const
    {
        return (mStationarityMeasure);
    }

    /*! Check inexactness in the Karush-Kuhn-Tucker (KKT) conditions (i.e. KKT residual) and compute
     * the norm of the KKT residual, where r(x,\lambda) = \{C1, C2, C3, C4\} denotes the residual vector
     * and C# denotes the corresponding Condition. The KKT conditions are given by:
     *
     * Condition 1: \left(1 + x_j\right)\left(\frac{\partial{f}_0}{\partial{x}_j} + \sum_{i=1}^{N_c}
     *              \lambda_i\frac{\partial{f}_i}{\partial{x}_j}\right)^{+} = 0,\quad{j}=1,\dots,n_x
     * Condition 2: \left(1 - x_j\right)\left(\frac{\partial{f}_0}{\partial{x}_j} + \sum_{i=1}^{N_c}
     *              \lambda_i\frac{\partial{f}_i}{\partial{x}_j}\right)^{-} = 0,\quad{j}=1,\dots,n_x
     * Condition 3: f_i(x)^{+} = 0,\quad{i}=1,\dots,N_c
     * Condition 4: \lambda_{i}f_i(x)^{-} = 0,\quad{i}=1,\dots,N_c.
     *
     * The nomenclature is given as follows: x denotes the control vector, \lambda denotes the dual
     * vector, N_c is the number of constraints, n_x is the number of controls, f_0 is the objective
     * function and f_i is the i-th constraint. Finally, a^{+} = max{0, a} and a^{-} = max{0, −a}.
     **/
    void computeKarushKuhnTuckerConditionsInexactness(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                                      const Plato::MultiVector<ScalarType, OrdinalType> & aDual)
    {
        const OrdinalType tVectorIndex = 0;
        assert(aDual.getNumVectors() == mDual->getNumVectors());
        assert(aDual.getNumVectors() == static_cast<OrdinalType>(1));
        assert(aDual[tVectorIndex].size() == mDual->operator[](tVectorIndex).size());
        assert(aControl.getNumVectors() == mCurrentControl->getNumVectors());
        assert(aControl[tVectorIndex].size() == mCurrentControl->operator[](tVectorIndex).size());

        Plato::fill(static_cast<ScalarType>(0), mControlWorkMultiVector.operator*());
        const Plato::Vector<ScalarType, OrdinalType> & tDual = aDual[tVectorIndex];
        const OrdinalType tNumConstraints = tDual.size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::MultiVector<ScalarType, OrdinalType> & tConstraintGradients =
                    mCurrentConstraintGradients->operator[](tConstraintIndex);
            Plato::update(tDual[tConstraintIndex],
                          tConstraintGradients,
                          static_cast<ScalarType>(1),
                          mControlWorkMultiVector.operator*());
        }

        ScalarType tConditioneOne = std::numeric_limits<ScalarType>::max();
        ScalarType tConditioneTwo = std::numeric_limits<ScalarType>::max();
        this->computeConditionsOneAndTwo(aControl, aDual, tConditioneOne, tConditioneTwo);

        ScalarType tConditioneThree = std::numeric_limits<ScalarType>::max();
        ScalarType tConditioneFour = std::numeric_limits<ScalarType>::max();
        this->computeConditionsThreeAndFour(aControl, aDual, tConditioneThree, tConditioneFour);

        ScalarType tSum = tConditioneOne + tConditioneTwo + tConditioneThree + tConditioneFour;
        mKarushKuhnTuckerConditionsInexactness = (static_cast<ScalarType>(1) / mGlobalNumControls) * std::sqrt(tSum);
    }
    ScalarType getKarushKuhnTuckerConditionsInexactness() const
    {
        return (mKarushKuhnTuckerConditionsInexactness);
    }

    const Plato::BoundsBase<ScalarType, OrdinalType> & bounds() const
    {
        return (*mBounds);
    }

    const Plato::PrimalProbElementWiseFunctions<ScalarType, OrdinalType> & elementWiseFunctions() const
    {
        return (*mElementWiseFunctions);
    }

private:
    void initializeBounds()
    {
        switch(mMemorySpace)
        {
            case Plato::MemorySpace::DEVICE:
            {
                mBounds = std::make_shared<Plato::DeviceBounds<ScalarType, OrdinalType>>();
                break;
            }
            default:
            case Plato::MemorySpace::HOST:
            {
                mBounds = std::make_shared<Plato::HostBounds<ScalarType, OrdinalType>>();
                break;
            }
        }
    }

    void initializeElementWiseFunctions()
    {
        switch(mMemorySpace)
        {
            case Plato::MemorySpace::DEVICE:
            {
                mElementWiseFunctions = std::make_shared<Plato::DevicePrimalProbElementWise<ScalarType, OrdinalType>>();
                break;
            }
            default:
            case Plato::MemorySpace::HOST:
            {
                mElementWiseFunctions = std::make_shared<Plato::HostPrimalProbElementWise<ScalarType, OrdinalType>>();
                break;
            }
        }
    }

    void initialize()
    {
        const OrdinalType tControlVectorIndex = 0;
        mControlWorkVectorOne = mCurrentControl->operator[](tControlVectorIndex).create();
        mControlWorkVectorTwo = mCurrentControl->operator[](tControlVectorIndex).create();
        Plato::fill(static_cast<ScalarType>(0), *mActiveSet);
        Plato::fill(static_cast<ScalarType>(1), *mInactiveSet);

        assert(mDual->getNumVectors() == static_cast<OrdinalType>(1));
        const OrdinalType tDualVectorIndex = 0;
        mDualWorkVectorOne = mDual->operator[](tDualVectorIndex).create();
        mDualWorkVectorTwo = mDual->operator[](tDualVectorIndex).create();

        const OrdinalType tNumConstraints = mDual->operator[](tDualVectorIndex).size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            mCurrentConstraintGradients->add(mCurrentControl.operator*());
        }

        ScalarType tScalarValue = std::numeric_limits<ScalarType>::max();
        Plato::fill(tScalarValue, *mControlUpperBounds);
        tScalarValue = -std::numeric_limits<ScalarType>::max();
        Plato::fill(tScalarValue, *mControlLowerBounds);

        tScalarValue = 1;
        Plato::fill(tScalarValue, mConstraintGlobalizationFactors.operator*());

        const OrdinalType tNumElements = 1;
        const OrdinalType tElementIndex = 0;
        Plato::StandardVector<ScalarType> tVector(tNumElements);
        const OrdinalType tNumVectors = mCurrentControl->getNumVectors();
        std::vector<ScalarType> tWorkVector(tNumVectors);
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            tVector[tElementIndex] = mCurrentControl->operator[](tIndex).size();
            tWorkVector[tIndex] = mControlReductions->sum(tVector);
        }
        const ScalarType tBaseValue = 0;
        mGlobalNumControls = std::accumulate(tWorkVector.begin(), tWorkVector.end(), tBaseValue);
    }
    /*!
     * Compute the following Karush-Kuhn-Tucker (KKT) conditions:
     *
     * Condition 3: f_i(x)^{+} = 0,\quad{i}=1,\dots,N_c
     * Condition 4: \lambda_{i}f_i(x)^{-} = 0,\quad{i}=1,\dots,N_c.
     *
     * where the nomenclature is given as follows: \lambda denotes the dual vector, N_c is the
     * number of constraints, n_x is the number of controls and f_i is the i-th constraint.
     * Finally, a^{+} = max{0, a} and a^{-} = max{0, −a}.
     **/
    void computeConditionsOneAndTwo(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                    const Plato::MultiVector<ScalarType, OrdinalType> & aDual,
                                    ScalarType & aConditionOne,
                                    ScalarType & aConditionTwo)
    {
        const OrdinalType tNumControlVectors = aControl.getNumVectors();
        std::vector<ScalarType> tStorageOne(tNumControlVectors, static_cast<ScalarType>(0));
        std::vector<ScalarType> tStorageTwo(tNumControlVectors, static_cast<ScalarType>(0));
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumControlVectors; tVectorIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tWorkOne = mControlWorkVectorOne.operator*();
            Plato::Vector<ScalarType, OrdinalType> & tWorkTwo = mControlWorkVectorTwo.operator*();
            const Plato::Vector<ScalarType, OrdinalType> & tControl = aControl[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tObjectiveGradient =
                    mCurrentObjectiveGradient->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tConstraintGradientTimesDual =
                    mControlWorkMultiVector->operator[](tVectorIndex);

            mElementWiseFunctions->computeKarushKuhnTuckerObjectiveConditions(tControl, 
                                                                              tObjectiveGradient, 
                                                                              tConstraintGradientTimesDual, 
                                                                              tWorkOne, 
                                                                              tWorkTwo);

            tStorageOne[tVectorIndex] = mControlReductions->sum(tWorkOne);
            tStorageTwo[tVectorIndex] = mControlReductions->sum(tWorkTwo);
        }

        const ScalarType tInitialValue = 0;
        aConditionOne = std::accumulate(tStorageOne.begin(), tStorageOne.end(), tInitialValue);
        aConditionTwo = std::accumulate(tStorageTwo.begin(), tStorageTwo.end(), tInitialValue);
    }
    /*!
     * Compute the following Karush-Kuhn-Tucker (KKT) conditions:
     *
     * Condition 1: \left(1 + x_j\right)\left(\frac{\partial{f}_0}{\partial{x}_j} + \sum_{i=1}^{N_c}
     *              \lambda_i\frac{\partial{f}_i}{\partial{x}_j}\right)^{+} = 0,\quad{j}=1,\dots,n_x
     * Condition 2: \left(1 - x_j\right)\left(\frac{\partial{f}_0}{\partial{x}_j} + \sum_{i=1}^{N_c}
     *              \lambda_i\frac{\partial{f}_i}{\partial{x}_j}\right)^{-} = 0,\quad{j}=1,\dots,n_x,
     *
     * where the nomenclature is given as follows: x denotes the control vector, \lambda denotes
     * the dual vector, N_c is the number of constraints, n_x is the number of controls, f_0 is
     * the objective function and f_i is the i-th constraint. Finally, a^{+} = max{0, a} and a^{-}
     * = max{0, −a}.
     **/
    void computeConditionsThreeAndFour(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                       const Plato::MultiVector<ScalarType, OrdinalType> & aDual,
                                       ScalarType & aConditionThree,
                                       ScalarType & aConditionFour)
    {
        const OrdinalType tNumDualVectors = aDual.getNumVectors();
        std::vector<ScalarType> tStorageOne(tNumDualVectors, static_cast<ScalarType>(0));
        std::vector<ScalarType> tStorageTwo(tNumDualVectors, static_cast<ScalarType>(0));
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumDualVectors; tVectorIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tWorkOne = mDualWorkVectorOne.operator*();
            Plato::Vector<ScalarType, OrdinalType> & tWorkTwo = mDualWorkVectorTwo.operator*();
            const Plato::Vector<ScalarType, OrdinalType> & tDual = aDual[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tConstraintValues =
                    mCurrentConstraintValues->operator[](tVectorIndex);

            const OrdinalType tNumConstraints = tDual.size();
            for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
            {
                tWorkOne[tConstraintIndex] = std::max(static_cast<ScalarType>(0), tConstraintValues[tConstraintIndex]);
                tWorkOne[tConstraintIndex] = tWorkOne[tConstraintIndex] * tWorkOne[tConstraintIndex];

                tWorkTwo[tConstraintIndex] = tDual[tConstraintIndex]
                        * std::max(static_cast<ScalarType>(0), -tConstraintValues[tConstraintIndex]);
                tWorkTwo[tConstraintIndex] = tWorkTwo[tConstraintIndex] * tWorkTwo[tConstraintIndex];
            }

            tStorageOne[tVectorIndex] = mDualReductions->sum(tWorkOne);
            tStorageTwo[tVectorIndex] = mDualReductions->sum(tWorkTwo);
        }

        const ScalarType tInitialValue = 0;
        aConditionThree = std::accumulate(tStorageOne.begin(), tStorageOne.end(), tInitialValue);
        aConditionFour = std::accumulate(tStorageTwo.begin(), tStorageTwo.end(), tInitialValue);
    }

private:
    bool mIsInitialGuessSet;

    OrdinalType mNumObjFuncEval;
    OrdinalType mNumObjGradEval;

    ScalarType mGlobalNumControls;
    ScalarType mControlStagnationMeasure;
    ScalarType mFeasibilityMeasure;
    ScalarType mStationarityMeasure;
    ScalarType mNormInactiveGradient;
    ScalarType mObjectiveStagnationMeasure;
    ScalarType mDualProblemBoundsScaleFactor;
    ScalarType mCurrentObjectiveFunctionValue;
    ScalarType mPreviousObjectiveFunctionValue;
    ScalarType mDualObjectiveGlobalizationFactor;
    ScalarType mKarushKuhnTuckerConditionsInexactness;

    Plato::MemorySpace::type_t mMemorySpace;

    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mDualWorkVectorOne;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mDualWorkVectorTwo;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWorkVectorOne;
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> mControlWorkVectorTwo;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDual;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialStep;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mActiveSet;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mInactiveSet;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentSigma;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlLowerBounds;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlUpperBounds;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlWorkMultiVector;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentConstraintValues;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentObjectiveGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mConstraintGlobalizationFactors;

    std::shared_ptr<Plato::CommWrapper> mCommWrapper;
    std::shared_ptr<Plato::BoundsBase<ScalarType, OrdinalType>> mBounds;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mDualReductions;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mControlReductions;
    std::shared_ptr<Plato::PrimalProbElementWiseFunctions<ScalarType, OrdinalType>> mElementWiseFunctions;

    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mCurrentConstraintGradients;

private:
    ConservativeConvexSeparableAppxDataMng(const Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aRhs);
    Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & operator=(const Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_CONSERVATIVECONVEXSEPARABLEAPPXDATAMNG_HPP_ */
