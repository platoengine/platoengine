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
 * Plato_HessianLBFGS.hpp
 *
 *  Created on: Nov 13, 2018
 */

#pragma once

#include <cmath>
#include <cassert>
#include <algorithm>

#include "Plato_StateData.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_LinearOperator.hpp"
#include "Plato_MultiVectorList.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class HessianLBFGS : public Plato::LinearOperator<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Default constructor
     * @param [in] aDataFactory linear algebra factory
     * @param [in] aStorage memory size (default = 8)
    **********************************************************************************/
    explicit HessianLBFGS(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory, OrdinalType aStorage = 8) :
            mMaxStorage(aStorage),
            mMemoryLength(0),
            mLowerBoundCurvature(1e-7),
            mUpperBoundCurvature(1e7),
            mNewDeltaControl(aDataFactory.control().create()),
            mNewDeltaGradient(aDataFactory.control().create()),
            mOldHessTimesVector(aDataFactory.control().create()),
            mNewHessTimesVector(aDataFactory.control().create()),
            mDeltaControl(),
            mDeltaGradient(),
            mOldHessTimesDeltaControl(),
            mNewHessTimesDeltaControl()
    {
        this->initialize();
    }

    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aMultiVec multi-vector template
     * @param [in] aStorage memory size (default = 8)
    **********************************************************************************/
    explicit HessianLBFGS(const Plato::MultiVector<ScalarType, OrdinalType> & aMultiVec, OrdinalType aStorage = 8) :
            mMaxStorage(aStorage),
            mMemoryLength(0),
            mLowerBoundCurvature(1e-7),
            mUpperBoundCurvature(1e7),
            mNewDeltaControl(aMultiVec.create()),
            mNewDeltaGradient(aMultiVec.create()),
            mOldHessTimesVector(aMultiVec.create()),
            mNewHessTimesVector(aMultiVec.create()),
            mDeltaControl(),
            mDeltaGradient(),
            mOldHessTimesDeltaControl(),
            mNewHessTimesDeltaControl()
    {
        this->initialize();
    }

    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    virtual ~HessianLBFGS()
    {
    }

    /******************************************************************************//**
     * @brief Set maximum number of secant vectors stored for approximation
     * @param [in] aInput memory size
    **********************************************************************************/
    void setMaxStorage(const OrdinalType & aInput)
    {
        mMaxStorage = aInput;
        this->reset();
        this->initialize();
    }

    /******************************************************************************//**
     * @brief Set lower and upper bounds on the curvature
     * @param [in] aLower lower bound
     * @param [in] aUpper upper bound
    **********************************************************************************/
    void setCurvatureBounds(const ScalarType & aLower, const ScalarType & aUpper)
    {
        mLowerBoundCurvature = aLower;
        mUpperBoundCurvature = aUpper;
    }

    /******************************************************************************//**
     * @brief Return control's  secant information, i.e. ,  /f$ \Delta{x}_i /f$,
     * @param [in] aIndex location of vector on the list (requirement: index < memory size)
     * @return /f$ \Delta{x}_{i} = x_i - x_{i-1} /f$, where /f$ i /f$ denotes aIndex
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getDeltaControl(const OrdinalType & aIndex)
    {
        assert(aIndex < mMaxStorage);
        return ( (*mDeltaControl)[aIndex] );
    }

    /******************************************************************************//**
     * @brief Return gradient's secant information, i.e. ,  /f$ \Delta{g}_i /f$,
     * @param [in] aIndex location of vector on the list (requirement: index < memory size)
     * @return /f$ \Delta{g}_{i} = g_i - g_{i-1} /f$, where /f$ i /f$ denotes aIndex
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & getDeltaGradient(const OrdinalType & tIndex)
    {
        assert(tIndex < mMaxStorage);
        return ( (*mDeltaGradient)[tIndex] );
    }

    /******************************************************************************//**
     * @brief Update memory, i.e. secant information
     * @param [in] aStateData data structure with information on current and previous state
    **********************************************************************************/
    void update(const Plato::StateData<ScalarType, OrdinalType> & aStateData)
    {
        const ScalarType tCurrentCriterionValue = aStateData.getCurrentCriterionValue();
        const ScalarType tNormCurrentControl = Plato::norm(aStateData.getCurrentControl());
        const ScalarType tNormCurrentCriterionGrad = Plato::norm(aStateData.getCurrentCriterionGradient());
        if(std::isfinite(tCurrentCriterionValue) && std::isfinite(tNormCurrentControl) && std::isfinite(tNormCurrentCriterionGrad))
        {
            this->computeNewSecantInformation(aStateData);
            this->updateMemory();
        }
    }

    /******************************************************************************//**
     * @brief Apply vector to Hessian operator
     * @param [in] aControl design variables
     * @param [in] aVector descent direction
     * @param [in,out] aOutput application of vector to Hessian operator
    **********************************************************************************/
    void apply(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
               const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
               Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        if(mMemoryLength == static_cast<OrdinalType>(0))
        {
            Plato::update(static_cast<ScalarType>(1), aVector, static_cast<ScalarType>(0), aOutput);
            return;
        }

        this->computeInitialApproximation(aVector);

        for(OrdinalType tIndex_I = 1; tIndex_I < mMemoryLength; tIndex_I++)
        {
            for(OrdinalType tIndex_J = tIndex_I; tIndex_J < mMemoryLength; tIndex_J++)
            {
                this->bfgs(tIndex_I, (*mDeltaControl)[tIndex_J], (*mOldHessTimesDeltaControl)[tIndex_J], (*mNewHessTimesDeltaControl)[tIndex_J]);
            }
            this->bfgs(tIndex_I, aVector, *mOldHessTimesVector, *mNewHessTimesVector);
            this->advance(tIndex_I);
        }

        Plato::update(static_cast<ScalarType>(1), *mOldHessTimesVector, static_cast<ScalarType>(0), aOutput);
    }

private:
    /******************************************************************************//**
     * @brief Reset shared pointers memory and set to null
    **********************************************************************************/
    void reset()
    {
        mDeltaControl.reset();
        mDeltaGradient.reset();
        mOldHessTimesDeltaControl.reset();
        mNewHessTimesDeltaControl.reset();
    }

    /******************************************************************************//**
     * @brief Allocate member data
    **********************************************************************************/
    void initialize()
    {
        const OrdinalType tVECTOR_INDEX = 0;
        assert(mMaxStorage > static_cast<OrdinalType>(0));
        assert(mNewHessTimesVector->getNumVectors() > static_cast<OrdinalType>(0));
        assert((*mNewHessTimesVector)[tVECTOR_INDEX].size() > static_cast<OrdinalType>(0));

        mDeltaControl = std::make_shared<Plato::MultiVectorList<ScalarType, OrdinalType>>(mMaxStorage, *mNewHessTimesVector);
        mDeltaGradient = std::make_shared<Plato::MultiVectorList<ScalarType, OrdinalType>>(mMaxStorage, *mNewHessTimesVector);
        mOldHessTimesDeltaControl = std::make_shared<Plato::MultiVectorList<ScalarType, OrdinalType>>(mMaxStorage, *mNewHessTimesVector);
        mNewHessTimesDeltaControl = std::make_shared<Plato::MultiVectorList<ScalarType, OrdinalType>>(mMaxStorage, *mNewHessTimesVector);
    }

    /******************************************************************************//**
     * @brief Store current Hessian approximation information for the next L-BFGS iteration
     * @param [in] aOuterIndex outer loop index
    **********************************************************************************/
    void advance(const OrdinalType & aOuterIndex)
    {
        for(OrdinalType tIndex = 0; tIndex < mMemoryLength; tIndex++)
        {
            Plato::fill(static_cast<ScalarType>(0), (*mOldHessTimesDeltaControl)[tIndex]);
        }

        Plato::update(static_cast<ScalarType>(1), *mNewHessTimesVector, static_cast<ScalarType>(0), *mOldHessTimesVector);
        for(OrdinalType tIndex = aOuterIndex; tIndex < mMemoryLength; tIndex++)
        {
            Plato::update(static_cast<ScalarType>(1), (*mNewHessTimesDeltaControl)[tIndex], static_cast<ScalarType>(0), (*mOldHessTimesDeltaControl)[tIndex]);
        }
    }

    /******************************************************************************//**
     * @brief Compute Hessian approximation for the first L-BFGS iteration
     * @param [in] aOuterIndex outer loop index
    **********************************************************************************/
    void computeInitialApproximation(const Plato::MultiVector<ScalarType, OrdinalType> & aVector)
    {
        const OrdinalType tLatestSecantInfoIndex = mMemoryLength - static_cast<OrdinalType>(1);
        const ScalarType tDeltaControlDotDeltaGrad =
                Plato::dot( (*mDeltaControl)[tLatestSecantInfoIndex], (*mDeltaGradient)[tLatestSecantInfoIndex] );
        const ScalarType tDeltaGradDotDeltaGrad =
                Plato::dot( (*mDeltaGradient)[tLatestSecantInfoIndex], (*mDeltaGradient)[tLatestSecantInfoIndex] );
        const ScalarType tGamma0 = tDeltaGradDotDeltaGrad / tDeltaControlDotDeltaGrad;

        for(OrdinalType tIndex = 0; tIndex < mMemoryLength; tIndex++)
        {
            Plato::update(tGamma0, (*mDeltaControl)[tIndex], static_cast<ScalarType>(0), (*mOldHessTimesDeltaControl)[tIndex]);
        }
        Plato::update(tGamma0, aVector, static_cast<ScalarType>(0), *mOldHessTimesVector);
    }

    /******************************************************************************//**
     * @brief Compute matrix-free limited memory Broyden–Fletcher–Goldfarb–Shanno (L-BFGS) approximation
     * @param [in] aOuterIndex outer loop index
     * @param [in] aVector vector to apply
     * @param [in] aOuterIndex Hessian times a vector
     * @param [in,out] aOutput result
    **********************************************************************************/
    void bfgs(const OrdinalType & aOuterIndex,
              const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
              const Plato::MultiVector<ScalarType, OrdinalType> & aHessTimesVector,
              Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> & tMyDeltaGrad = (*mDeltaGradient)[aOuterIndex];
        const Plato::MultiVector<ScalarType, OrdinalType> & tMyDeltaControl = (*mDeltaControl)[aOuterIndex];
        const Plato::MultiVector<ScalarType, OrdinalType> & tMyOldHessTimesDeltaControl = (*mOldHessTimesDeltaControl)[aOuterIndex];

        Plato::update(static_cast<ScalarType>(1), aHessTimesVector, static_cast<ScalarType>(0), aOutput);

        const ScalarType tHessTimesDeltaControlTimesVector = Plato::dot(tMyOldHessTimesDeltaControl, aVector);
        const ScalarType tDeltaControlTimesHessTimesDeltaControl = Plato::dot(tMyDeltaControl, tMyOldHessTimesDeltaControl);
        const ScalarType tAlpha = tHessTimesDeltaControlTimesVector / tDeltaControlTimesHessTimesDeltaControl;
        Plato::update(-tAlpha, tMyOldHessTimesDeltaControl, static_cast<ScalarType>(1), aOutput);

        const ScalarType tDeltaGradDotVector = Plato::dot(tMyDeltaGrad, aVector);
        const ScalarType tDeltaGradDotDeltaControl = Plato::dot(tMyDeltaGrad, tMyDeltaControl);
        const ScalarType tBeta = tDeltaGradDotVector / tDeltaGradDotDeltaControl;
        Plato::update(tBeta, tMyDeltaGrad, static_cast<ScalarType>(1), aOutput);
    }

    /******************************************************************************//**
     * @brief Update memory, i.e. secant information, storage
    **********************************************************************************/
    void updateMemory()
    {
        const ScalarType tCurvatureCondition = Plato::dot(*mNewDeltaControl, *mNewDeltaGradient);
        bool tIsCurvatureConditionAboveLowerBound = tCurvatureCondition > mLowerBoundCurvature ? true : false;
        bool tIsCurvatureConditionBelowUpperBound = tCurvatureCondition < mUpperBoundCurvature ? true : false;
        if(tIsCurvatureConditionAboveLowerBound && tIsCurvatureConditionBelowUpperBound)
        {
            const OrdinalType tLength = mMaxStorage - static_cast<OrdinalType>(1);
            if(mMemoryLength == mMaxStorage)
            {
                for(OrdinalType tBaseIndex = 0; tBaseIndex < tLength; tBaseIndex++)
                {
                    const OrdinalType tNextIndex = tBaseIndex + static_cast<OrdinalType>(1);
                    Plato::update(static_cast<ScalarType>(1), (*mDeltaControl)[tNextIndex], static_cast<ScalarType>(0), (*mDeltaControl)[tBaseIndex]);
                    Plato::update(static_cast<ScalarType>(1), (*mDeltaGradient)[tNextIndex], static_cast<ScalarType>(0), (*mDeltaGradient)[tBaseIndex]);
                }
            }

            const OrdinalType tIndex = std::min(mMemoryLength, tLength);
            Plato::MultiVector<ScalarType, OrdinalType> & tMyDeltaControl = (*mDeltaControl)[tIndex];
            Plato::update(static_cast<ScalarType>(1), *mNewDeltaControl, static_cast<ScalarType>(0), tMyDeltaControl);
            Plato::MultiVector<ScalarType, OrdinalType> & tMyDeltaGradient = (*mDeltaGradient)[tIndex];
            Plato::update(static_cast<ScalarType>(1), *mNewDeltaGradient, static_cast<ScalarType>(0), tMyDeltaGradient);

            mMemoryLength++;
            mMemoryLength = std::min(mMemoryLength, mMaxStorage);
        }
    }

    /******************************************************************************//**
     * @brief Compute new secant information, i.e. /f$ \Delta{x}_{i} = x_i - x_{i-1} /f$ and  /f$ \Delta{g}_{i} = g_i - g_{i-1} /f$
     * @param [in] aStateData data structure with information on current and previous state
    **********************************************************************************/
    void computeNewSecantInformation(const Plato::StateData<ScalarType, OrdinalType> & aStateData)
    {
        Plato::update(static_cast<ScalarType>(1), aStateData.getCurrentControl(), static_cast<ScalarType>(0), *mNewDeltaControl);
        Plato::update(static_cast<ScalarType>(-1), aStateData.getPreviousControl(), static_cast<ScalarType>(1), *mNewDeltaControl);
        Plato::update(static_cast<ScalarType>(1), aStateData.getCurrentCriterionGradient(), static_cast<ScalarType>(0), *mNewDeltaGradient);
        Plato::update(static_cast<ScalarType>(-1), aStateData.getPreviousCriterionGradient(), static_cast<ScalarType>(1), *mNewDeltaGradient);
    }

private:
    OrdinalType mMaxStorage; /*!< maximum number of secant vectors stored, (default = 8) */
    OrdinalType mMemoryLength; /*!< current number of secant vectors stored */

    ScalarType mLowerBoundCurvature; /*!< lower bound on curvature condition, default = 1e-7 */
    ScalarType mUpperBoundCurvature; /*!< upper bound on curvature condition, default = 1e7 */

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mNewDeltaControl; /*!< new control secant information,  /f$ \Delta{x}_{i} = x_i - x_{i-1} /f$, i = memory index */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mNewDeltaGradient; /*!< new gradient secant information,  /f$ \Delta{g}_{i} = g_i - g_{i-1} /f$ */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mOldHessTimesVector; /*!< old L-BFGS Hessian times vector approximation,  /f$ \nabla^2{f}_{j-1}(x_k)v /f$, j = outer recursive loop index */
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mNewHessTimesVector; /*!< new L-BFGS Hessian times vector approximation,  /f$ \nabla^2{f}_{j}(x_k)v /f$, j = outer recursive loop index */

    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mDeltaControl; /*!< list of secant control vectors,  /f$ \Delta{x}_{i} = x_i - x_{i-1} /f$, i = memory index */
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mDeltaGradient; /*!< list of secant gradient vectors,  /f$ \Delta{g}_{i} = g_i - g_{i-1} /f$, i = memory index */
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mOldHessTimesDeltaControl; /*!< list of old Hessian approximation times secant control vectors,  /f$ \nabla^2f_{j-1}(x_k)\Delta{x} /f$, j = outer l-bfgs index & k = optimization iteration */
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mNewHessTimesDeltaControl; /*!< list of new Hessian approximation times secant control vectors,  /f$ \nabla^2f_{j}(x_k)\Delta{x} /f$, j = outer l-bfgs index & k = optimization iteration */
};
// class HessianLBFGS

}
// namespace Plato
