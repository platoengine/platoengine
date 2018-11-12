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
 * Plato_Test_Hessian.cpp
 *
 *  Created on: Nov 11, 2018
 */

#include "gtest/gtest.h"

#include "Plato_UnitTestUtils.hpp"

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

    virtual ~HessianLBFGS()
    {
    }

    void setMaxStorage(const OrdinalType & aInput)
    {
        mMaxStorage = aInput;
        this->reset();
        this->initialize();
    }

    void setCurvatureBounds(const ScalarType & aLower, const ScalarType & aUpper)
    {
        mLowerBoundCurvature = aLower;
        mUpperBoundCurvature = aUpper;
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getDeltaControl(const OrdinalType & tIndex)
    {
        assert(tIndex < mMaxStorage);
        return ( (*mDeltaControl)[tIndex] );
    }

    const Plato::MultiVector<ScalarType, OrdinalType> & getDeltaGradient(const OrdinalType & tIndex)
    {
        assert(tIndex < mMaxStorage);
        return ( (*mDeltaGradient)[tIndex] );
    }

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
    }

private:
    void reset()
    {
        mDeltaControl.reset();
        mDeltaGradient.reset();
        mOldHessTimesDeltaControl.reset();
        mNewHessTimesDeltaControl.reset();
    }

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

    void advance(const OrdinalType & aOuterIndex)
    {
        Plato::update(static_cast<ScalarType>(1), *mNewHessTimesVector, static_cast<ScalarType>(0), *mOldHessTimesVector);
        for(OrdinalType tIndex = aOuterIndex; tIndex < mMemoryLength; tIndex++)
        {
            Plato::update(static_cast<ScalarType>(1), (*mNewHessTimesDeltaControl)[tIndex], static_cast<ScalarType>(0), (*mOldHessTimesDeltaControl)[tIndex]);
        }
    }

    void computeInitialApproximation(const Plato::MultiVector<ScalarType, OrdinalType> & aVector)
    {
        const ScalarType tDeltaControlDotDeltaGrad =
                Plato::dot( (*mDeltaControl)[mMemoryLength], (*mDeltaGradient)[mMemoryLength] );
        const ScalarType tDeltaGradDotDeltaGrad =
                Plato::dot( (*mDeltaGradient)[mMemoryLength], (*mDeltaGradient)[mMemoryLength] );
        const ScalarType tGamma0 = tDeltaGradDotDeltaGrad / tDeltaControlDotDeltaGrad;

        for(OrdinalType tIndex = 0; tIndex < mMemoryLength; tIndex++)
        {
            Plato::update(tGamma0, (*mDeltaControl)[tIndex], static_cast<ScalarType>(0), (*mOldHessTimesDeltaControl)[tIndex]);
        }
        Plato::update(tGamma0, aVector, static_cast<ScalarType>(0), *mOldHessTimesVector);
    }

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

    void updateMemory()
    {
        const ScalarType tCurvatureCondition = Plato::dot(*mNewDeltaControl, *mNewDeltaGradient);
        bool tIsCurvatureConditionAboveLowerBound = tCurvatureCondition > mLowerBoundCurvature ? true : false;
        bool tIsCurvatureConditionBelowUpperBound = tCurvatureCondition < mUpperBoundCurvature ? true : false;
        if(tIsCurvatureConditionAboveLowerBound && tIsCurvatureConditionBelowUpperBound)
        {
            if(mMemoryLength == mMaxStorage)
            {
                const OrdinalType tLength = mMaxStorage - static_cast<OrdinalType>(1);
                for(OrdinalType tBaseIndex = 0; tBaseIndex < tLength; tBaseIndex++)
                {
                    const OrdinalType tNextIndex = tBaseIndex + static_cast<OrdinalType>(1);
                    Plato::update(static_cast<ScalarType>(1), (*mDeltaControl)[tNextIndex], static_cast<ScalarType>(0), (*mDeltaControl)[tBaseIndex]);
                    Plato::update(static_cast<ScalarType>(1), (*mDeltaGradient)[tNextIndex], static_cast<ScalarType>(0), (*mDeltaGradient)[tBaseIndex]);
                }
            }

            Plato::MultiVector<ScalarType, OrdinalType> & tMyDeltaControl = (*mDeltaControl)[mMemoryLength];
            Plato::update(static_cast<ScalarType>(1), *mNewDeltaControl, static_cast<ScalarType>(0), tMyDeltaControl);
            Plato::MultiVector<ScalarType, OrdinalType> & tMyDeltaGradient = (*mDeltaGradient)[mMemoryLength];
            Plato::update(static_cast<ScalarType>(1), *mNewDeltaGradient, static_cast<ScalarType>(0), tMyDeltaGradient);

            mMemoryLength++;
            mMemoryLength = std::min(mMemoryLength, mMaxStorage);
        }
    }

    void computeNewSecantInformation(const Plato::StateData<ScalarType, OrdinalType> & aStateData)
    {
        Plato::update(static_cast<ScalarType>(1), aStateData.getCurrentControl(), static_cast<ScalarType>(0), *mNewDeltaControl);
        Plato::update(static_cast<ScalarType>(-1), aStateData.getPreviousControl(), static_cast<ScalarType>(1), *mNewDeltaControl);
        Plato::update(static_cast<ScalarType>(1), aStateData.getCurrentCriterionGradient(), static_cast<ScalarType>(0), *mNewDeltaGradient);
        Plato::update(static_cast<ScalarType>(-1), aStateData.getPreviousCriterionGradient(), static_cast<ScalarType>(1), *mNewDeltaGradient);
    }

private:
    OrdinalType mMaxStorage;
    OrdinalType mMemoryLength;

    ScalarType mLowerBoundCurvature;
    ScalarType mUpperBoundCurvature;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mNewDeltaControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mNewDeltaGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mOldHessTimesVector;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mNewHessTimesVector;

    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mDeltaControl;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mDeltaGradient;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mOldHessTimesDeltaControl;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mNewHessTimesDeltaControl;
};
// class HessianLBFGS

}
// namespace Plato

namespace PlatoTest
{

TEST(PlatoTest, HessianLBFGS)
{
    const size_t tNumVectors = 1;
    const size_t tNumControls = 10;
    Plato::DataFactory<double> tDataFactory;
    Plato::StandardMultiVector<double> tControl(tNumVectors, tNumControls);
    tDataFactory.allocateControl(tControl);

    const size_t tMaxMemoryLength = 4;
    Plato::HessianLBFGS<double> tHessian(tDataFactory, tMaxMemoryLength);

    // **** FIRST CALL TO APPLY ****
    const size_t tVECTOR_INDEX = 0;
    Plato::StateData<double> tStateData(tDataFactory);
    tStateData.setCurrentCriterionValue(1);
    tControl(tVECTOR_INDEX, 0) = 0.552783345944550;
    tControl(tVECTOR_INDEX, 1) = 1.03909065350496;
    tControl(tVECTOR_INDEX, 2) = -1.11763868326521;
    tControl(tVECTOR_INDEX, 3) = 1.26065870912090;
    tStateData.setCurrentControl(tControl);
    Plato::StandardMultiVector<double> tGrad(tNumVectors, tNumControls);
    tGrad(tVECTOR_INDEX, 0) = -0.485176625320520;
    tGrad(tVECTOR_INDEX, 1) = 2.00321866981568;
    tGrad(tVECTOR_INDEX, 2) = -0.753023677610612;
    tGrad(tVECTOR_INDEX, 3) = 0.00503015828035112;
    tStateData.setCurrentCriterionGradient(tGrad);
    Plato::StandardMultiVector<double> tOutput(tNumVectors, tNumControls);
    tHessian.apply(tControl, tGrad, tOutput);
    PlatoTest::checkMultiVectorData(tOutput, tGrad);

    // **** FIRST CALL TO UPDATE ****
    tStateData.setPreviousControl(tControl);
    tStateData.setPreviousCriterionGradient(tGrad);
    tStateData.setCurrentCriterionValue(1.817160461668281);
    tControl(tVECTOR_INDEX, 0) = 1.03795997126507;
    tControl(tVECTOR_INDEX, 1) = -0.964128016310720;
    tControl(tVECTOR_INDEX, 2) = -0.364615005654596;
    tControl(tVECTOR_INDEX, 3) = 1.25562855084055;
    tStateData.setCurrentControl(tControl);
    tGrad(tVECTOR_INDEX, 0) = 2.52394128404335;
    tGrad(tVECTOR_INDEX, 1) = -2.08147201454278;
    tGrad(tVECTOR_INDEX, 2) = -0.0939313592226049;
    tGrad(tVECTOR_INDEX, 3) = 0.489260463064621;
    tStateData.setCurrentCriterionGradient(tGrad);
    tHessian.update(tStateData);
    // TEST SECANT INFORMATION
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(tVECTOR_INDEX, 0) = 0.485176625320520;
    tGold(tVECTOR_INDEX, 1) = -2.00321866981568;
    tGold(tVECTOR_INDEX, 2) = 0.753023677610612;
    tGold(tVECTOR_INDEX, 3) = -0.00503015828035114;
    size_t tMemoryIndex = 0;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaControl(tMemoryIndex), tGold);
    tGold(tVECTOR_INDEX, 0) = 3.00911790936387;
    tGold(tVECTOR_INDEX, 1) = -4.08469068435845;
    tGold(tVECTOR_INDEX, 2) = 0.659092318388008;
    tGold(tVECTOR_INDEX, 3) = 0.484230304784270;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaGradient(tMemoryIndex), tGold);
}

} // namespace PlatoTest
