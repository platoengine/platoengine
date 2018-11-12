/*
 * Plato_Test_Hessian.cpp
 *
 *  Created on: Nov 11, 2018
 */

#include <cassert>

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class HessianLBFGS : public Plato::LinearOperator<ScalarType, OrdinalType>
{
public:
    explicit HessianLBFGS(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory, OrdinalType aStorage = 8) :
            mMaxStorage(aStorage),
            mMemoryLength(0),
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

    void update(const Plato::StateData<ScalarType, OrdinalType> & aStateData)
    {
    }

    void apply(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
               const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
               Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        if(mMemoryLength == static_cast<OrdinalType>(0))
        {
            Plato::update(static_cast<ScalarType>(1), aVector, static_cast<ScalarType>(0), aOutput);
        }

        this->computeInitialApproximation(aVector);

        for(OrdinalType tIndex_I = 1; tIndex_I < mMemoryLength; tIndex_I++)
        {
            Plato::fill(static_cast<ScalarType>(0), *mNewHessTimesDeltaControl);
            for(OrdinalType tIndex_J = tIndex_I; tIndex_J < mMemoryLength; tIndex_J++)
            {
                this->bfgs(tIndex_I, (*mDeltaControl)[tIndex_J], (*mOldHessTimesDeltaControl)[tIndex_J], (*mNewHessTimesDeltaControl)[tIndex_J]);
            }
            this->bfgs(tIndex_I, aVector, *mOldHessTimesVector, *mNewHessTimesVector);
            this->advance();
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

    void advance()
    {
        Plato::update(static_cast<ScalarType>(1), *mNewHessTimesVector, static_cast<ScalarType>(0), *mOldHessTimesVector);
        for(OrdinalType tIndex = tIndex_I; tIndex < mMemoryLength; tIndex++)
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

        Plato::fill(static_cast<ScalarType>(0), *mOldHessTimesDeltaControl);
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

private:
    OrdinalType mMaxStorage;
    OrdinalType mMemoryLength;

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
