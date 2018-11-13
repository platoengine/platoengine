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

        Plato::update(static_cast<ScalarType>(1), *mOldHessTimesVector, static_cast<ScalarType>(0), aOutput);
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
    tControl(tVECTOR_INDEX, 0) = 0.443421912904091;
    tControl(tVECTOR_INDEX, 1) = 0.391894209432449;
    tControl(tVECTOR_INDEX, 2) = -1.25067890682641;
    tControl(tVECTOR_INDEX, 3) = -0.947960922331432;
    tStateData.setCurrentControl(tControl);
    Plato::StandardMultiVector<double> tGrad(tNumVectors, tNumControls);
    tGrad(tVECTOR_INDEX, 0) = -0.0461472880012221;
    tGrad(tVECTOR_INDEX, 1) = 0.389850514573980;
    tGrad(tVECTOR_INDEX, 2) = -1.79838890265604;
    tGrad(tVECTOR_INDEX, 3) = -0.542323904007486;
    tStateData.setCurrentCriterionGradient(tGrad);
    Plato::StandardMultiVector<double> tOutput(tNumVectors, tNumControls);
    tHessian.apply(tControl, tGrad, tOutput);
    // TEST OUTPUT (I.E. HESSIAN TIMES VECTOR) INFORMATION
    PlatoTest::checkMultiVectorData(tOutput, tGrad);

    // **** FIRST CALL TO UPDATE ****
    tStateData.setPreviousControl(tControl);
    tStateData.setPreviousCriterionGradient(tGrad);
    tStateData.setCurrentCriterionValue(0.109592611823559);
    tControl(tVECTOR_INDEX, 0) = 0.489569200905313;
    tControl(tVECTOR_INDEX, 1) = 0.00204369485846889;
    tControl(tVECTOR_INDEX, 2) = 0.547709995829628;
    tControl(tVECTOR_INDEX, 3) = -0.405637018323946;
    tStateData.setCurrentControl(tControl);
    tGrad(tVECTOR_INDEX, 0) = 0.0584394870944449;
    tGrad(tVECTOR_INDEX, 1) = -0.0646818067395529;
    tGrad(tVECTOR_INDEX, 2) = 0.335430363132848;
    tGrad(tVECTOR_INDEX, 3) = -0.152329694588052;
    tStateData.setCurrentCriterionGradient(tGrad);
    tHessian.update(tStateData);
    // TEST SECANT INFORMATION
    Plato::StandardMultiVector<double> tGold(tNumVectors, tNumControls);
    tGold(tVECTOR_INDEX, 0) = 0.0461472880012221;
    tGold(tVECTOR_INDEX, 1) = -0.389850514573980;
    tGold(tVECTOR_INDEX, 2) = 1.79838890265604;
    tGold(tVECTOR_INDEX, 3) = 0.542323904007486;
    size_t tMemoryIndex = 0;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaControl(tMemoryIndex), tGold);
    tGold(tVECTOR_INDEX, 0) = 0.104586775095667;
    tGold(tVECTOR_INDEX, 1) = -0.454532321313533;
    tGold(tVECTOR_INDEX, 2) = 2.13381926578888;
    tGold(tVECTOR_INDEX, 3) = 0.389994209419434;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaGradient(tMemoryIndex), tGold);

    // **** SECOND CALL TO APPLY ****
    tHessian.apply(tControl, tGrad, tOutput);
    tGold(tVECTOR_INDEX, 0) = 0.0679955736380994;
    tGold(tVECTOR_INDEX, 1) = -0.0752586439729834;
    tGold(tVECTOR_INDEX, 2) = 0.390280289763564;
    tGold(tVECTOR_INDEX, 3) = -0.177238806851467;
    PlatoTest::checkMultiVectorData(tOutput, tGold);

    // **** SECOND CALL TO UPDATE ****
    tStateData.setPreviousControl(tControl);
    tStateData.setPreviousCriterionGradient(tGrad);
    tStateData.setCurrentCriterionValue(0.032636727941169);
    tControl(tVECTOR_INDEX, 0) = 0.445516336185158;
    tControl(tVECTOR_INDEX, 1) = 0.0532848840480212;
    tControl(tVECTOR_INDEX, 2) = 0.283617316968124;
    tControl(tVECTOR_INDEX, 3) = -0.295663841011898;
    tStateData.setCurrentControl(tControl);
    tGrad(tVECTOR_INDEX, 0) = 0.0317281548766766;
    tGrad(tVECTOR_INDEX, 1) = -0.0493312706795408;
    tGrad(tVECTOR_INDEX, 2) = 0.131097137331731;
    tGrad(tVECTOR_INDEX, 3) = -0.0811928988069817;
    tStateData.setCurrentCriterionGradient(tGrad);
    tHessian.update(tStateData);
    // TEST SECANT INFORMATION
    tMemoryIndex = 1;
    tGold(tVECTOR_INDEX, 0) = -0.0440528647201551;
    tGold(tVECTOR_INDEX, 1) = 0.0512411891895523;
    tGold(tVECTOR_INDEX, 2) = -0.264092678861504;
    tGold(tVECTOR_INDEX, 3) = 0.109973177312048;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaControl(tMemoryIndex), tGold);
    tGold(tVECTOR_INDEX, 0) = -0.0267113322177683;
    tGold(tVECTOR_INDEX, 1) = 0.0153505360600121;
    tGold(tVECTOR_INDEX, 2) = -0.204333225801117;
    tGold(tVECTOR_INDEX, 3) = 0.0711367957810707;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaGradient(tMemoryIndex), tGold);

    // **** THIRD CALL TO APPLY ****
    tHessian.apply(tControl, tGrad, tOutput);
    tGold(tVECTOR_INDEX, 0) = 0.0199527987952355;
    tGold(tVECTOR_INDEX, 1) = -0.0240929686636601;
    tGold(tVECTOR_INDEX, 2) = 0.0990200656830435;
    tGold(tVECTOR_INDEX, 3) = -0.0536863786797123;
    PlatoTest::checkMultiVectorData(tOutput, tGold);

    // **** THIRD CALL TO UPDATE ****
    tStateData.setPreviousControl(tControl);
    tStateData.setPreviousCriterionGradient(tGrad);
    tStateData.setCurrentCriterionValue(0.007566542431534);
    tControl(tVECTOR_INDEX, 0) = 0.395769290027611;
    tControl(tVECTOR_INDEX, 1) = 0.143632124453243;
    tControl(tVECTOR_INDEX, 2) = 0.106991652387717;
    tControl(tVECTOR_INDEX, 3) = -0.173783680572607;
    tStateData.setCurrentControl(tControl);
    tGrad(tVECTOR_INDEX, 0) = 0.000937283094272392;
    tGrad(tVECTOR_INDEX, 1) = -0.0134649180922684;
    tGrad(tVECTOR_INDEX, 2) = 0.0328085583912217;
    tGrad(tVECTOR_INDEX, 3) = -0.0399875786912396;
    tStateData.setCurrentCriterionGradient(tGrad);
    tHessian.update(tStateData);
    // TEST SECANT INFORMATION
    tMemoryIndex = 2;
    tGold(tVECTOR_INDEX, 0) = -0.0497470461575475;
    tGold(tVECTOR_INDEX, 1) = 0.0903472404052214;
    tGold(tVECTOR_INDEX, 2) = -0.176625664580407;
    tGold(tVECTOR_INDEX, 3) = 0.121880160439291;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaControl(tMemoryIndex), tGold);
    tGold(tVECTOR_INDEX, 0) = -0.0307908717824042;
    tGold(tVECTOR_INDEX, 1) = 0.0358663525872724;
    tGold(tVECTOR_INDEX, 2) = -0.0982885789405098;
    tGold(tVECTOR_INDEX, 3) = 0.0412053201157421;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaGradient(tMemoryIndex), tGold);

    // **** FOURTH CALL TO APPLY ****
    tHessian.apply(tControl, tGrad, tOutput);
    tGold(tVECTOR_INDEX, 0) = 0.00135191001659628;
    tGold(tVECTOR_INDEX, 1) = -0.00441309826626570;
    tGold(tVECTOR_INDEX, 2) = 0.0172320163658772;
    tGold(tVECTOR_INDEX, 3) = -0.0153808765593237;
    PlatoTest::checkMultiVectorData(tOutput, tGold);

    // **** FOURTH CALL TO UPDATE ****
    tStateData.setPreviousControl(tControl);
    tStateData.setPreviousCriterionGradient(tGrad);
    tStateData.setCurrentCriterionValue(0.003500230915017);
    tControl(tVECTOR_INDEX, 0) = 0.397985674896240;
    tControl(tVECTOR_INDEX, 1) = 0.181777625237522;
    tControl(tVECTOR_INDEX, 2) = 0.0396623133358196;
    tControl(tVECTOR_INDEX, 3) = -0.0763673623062437;
    tStateData.setCurrentControl(tControl);
    tGrad(tVECTOR_INDEX, 0) = -0.00649579917408771;
    tGrad(tVECTOR_INDEX, 1) = 0.00295503140822104;
    tGrad(tVECTOR_INDEX, 2) = 0.000667403100264777;
    tGrad(tVECTOR_INDEX, 3) = -0.0168257587172167;
    tStateData.setCurrentCriterionGradient(tGrad);
    tHessian.update(tStateData);
    // TEST SECANT INFORMATION
    tMemoryIndex = 3;
    tGold(tVECTOR_INDEX, 0) = 0.00221638486862907;
    tGold(tVECTOR_INDEX, 1) = 0.0381455007842798;
    tGold(tVECTOR_INDEX, 2) = -0.0673293390518972;
    tGold(tVECTOR_INDEX, 3) = 0.0974163182663638;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaControl(tMemoryIndex), tGold);
    tGold(tVECTOR_INDEX, 0) = -0.00743308226836010;
    tGold(tVECTOR_INDEX, 1) = 0.0164199495004894;
    tGold(tVECTOR_INDEX, 2) = -0.0321411552909569;
    tGold(tVECTOR_INDEX, 3) = 0.0231618199740229;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaGradient(tMemoryIndex), tGold);

    // **** FIFTH CALL TO APPLY ****
    tHessian.apply(tControl, tGrad, tOutput);
    tGold(tVECTOR_INDEX, 0) = -0.00229239685067636;
    tGold(tVECTOR_INDEX, 1) = 0.00155358914580580;
    tGold(tVECTOR_INDEX, 2) = -0.00101909682362042;
    tGold(tVECTOR_INDEX, 3) = -0.00448752067926337;
    PlatoTest::checkMultiVectorData(tOutput, tGold);

    // **** FIFTH CALL TO UPDATE ****
    tStateData.setPreviousControl(tControl);
    tStateData.setPreviousCriterionGradient(tGrad);
    tStateData.setCurrentCriterionValue(0.00280643470452400);
    tControl(tVECTOR_INDEX, 0) = 0.419815226767597;
    tControl(tVECTOR_INDEX, 1) = 0.177849585899931;
    tControl(tVECTOR_INDEX, 2) = 0.0271026675517160;
    tControl(tVECTOR_INDEX, 3) = -0.0155434351483244;
    tStateData.setCurrentControl(tControl);
    tGrad(tVECTOR_INDEX, 0) = -0.00199003369352253;
    tGrad(tVECTOR_INDEX, 1) = -0.00141609836718944;
    tGrad(tVECTOR_INDEX, 2) = -0.00396913089385007;
    tGrad(tVECTOR_INDEX, 3) = -0.00351408655752225;
    tStateData.setCurrentCriterionGradient(tGrad);
    tHessian.update(tStateData);
    // TEST SECANT INFORMATION
    tMemoryIndex = 3;
    tGold(tVECTOR_INDEX, 0) = 0.0218295518713569;
    tGold(tVECTOR_INDEX, 1) = -0.00392803933759181;
    tGold(tVECTOR_INDEX, 2) = -0.0125596457841036;
    tGold(tVECTOR_INDEX, 3) = 0.0608239271579193;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaControl(tMemoryIndex), tGold);
    tGold(tVECTOR_INDEX, 0) = 0.00450576548056518;
    tGold(tVECTOR_INDEX, 1) = -0.00437112977541048;
    tGold(tVECTOR_INDEX, 2) = -0.00463653399411485;
    tGold(tVECTOR_INDEX, 3) = 0.0133116721596944;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaGradient(tMemoryIndex), tGold);

    // **** SIXTH CALL TO APPLY ****
    tHessian.apply(tControl, tGrad, tOutput);
    tGold(tVECTOR_INDEX, 0) = -0.000711575885200426;
    tGold(tVECTOR_INDEX, 1) = 3.68497108923860e-05;
    tGold(tVECTOR_INDEX, 2) = -0.00162407767089955;
    tGold(tVECTOR_INDEX, 3) = -0.000589763690047226;
    PlatoTest::checkMultiVectorData(tOutput, tGold);

    // **** SIXTH CALL TO UPDATE ****
    tStateData.setPreviousControl(tControl);
    tStateData.setPreviousCriterionGradient(tGrad);
    tStateData.setCurrentCriterionValue(0.00271758701509064);
    tControl(tVECTOR_INDEX, 0) = 0.427527806619517;
    tControl(tVECTOR_INDEX, 1) = 0.187260750753182;
    tControl(tVECTOR_INDEX, 2) = 0.0364711844401739;
    tControl(tVECTOR_INDEX, 3) = 0.00335670836543723;
    tStateData.setCurrentControl(tControl);
    tGrad(tVECTOR_INDEX, 0) = -0.00260747328651977;
    tGrad(tVECTOR_INDEX, 1) = -0.00131180969810058;
    tGrad(tVECTOR_INDEX, 2) = -0.00232532199722960;
    tGrad(tVECTOR_INDEX, 3) = 0.000437493273595805;
    tStateData.setCurrentCriterionGradient(tGrad);
    tHessian.update(tStateData);
    // TEST SECANT INFORMATION
    tMemoryIndex = 3;
    tGold(tVECTOR_INDEX, 0) = 0.00771257985191987;
    tGold(tVECTOR_INDEX, 1) = 0.00941116485325130;
    tGold(tVECTOR_INDEX, 2) = 0.00936851688845784;
    tGold(tVECTOR_INDEX, 3) = 0.0189001435137616;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaControl(tMemoryIndex), tGold);
    tGold(tVECTOR_INDEX, 0) = -0.000617439592997240;
    tGold(tVECTOR_INDEX, 1) = 0.000104288669088859;
    tGold(tVECTOR_INDEX, 2) = 0.00164380889662047;
    tGold(tVECTOR_INDEX, 3) = 0.00395157983111805;
    PlatoTest::checkMultiVectorData(tHessian.getDeltaGradient(tMemoryIndex), tGold);
}

} // namespace PlatoTest
