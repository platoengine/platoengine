#pragma once

#include <cmath>
#include <memory>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class ShiftedEllipse : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    ShiftedEllipse() :
            mXCenter(0.),
            mXRadius(1.),
            mYCenter(0.),
            mYRadius(1.)
    {
    }
    virtual ~ShiftedEllipse()
    {
    }
    void specify(ScalarType aXCenter, ScalarType aXRadius, ScalarType aYCenter, ScalarType aYRadius)
    {
        mXCenter = aXCenter;
        mXRadius = aXRadius;
        mYCenter = aYCenter;
        mYRadius = aYRadius;
    }

    void cacheData()
    {
        return;
    }

    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));

        const OrdinalType tVectorIndex = 0;
        ScalarType tOutput = ((aControl(tVectorIndex, 0) - mXCenter) * (aControl(tVectorIndex, 0) - mXCenter) / (mXRadius * mXRadius))
                             + ((aControl(tVectorIndex, 1) - mYCenter) * (aControl(tVectorIndex, 1) - mYCenter) / (mYRadius * mYRadius))
                             - 1.;
        return (tOutput);
    }
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aOutput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aControl.getNumVectors() == aOutput.getNumVectors());

        const OrdinalType tVectorIndex = 0;
        aOutput(tVectorIndex, 0) = (2. / (mXRadius * mXRadius)) * (aControl(tVectorIndex, 0) - mXCenter);
        aOutput(tVectorIndex, 1) = (2. / (mYRadius * mYRadius)) * (aControl(tVectorIndex, 1) - mYCenter);

    }
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aOutput.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aVector.getNumVectors() > static_cast<OrdinalType>(0));
        assert(aVector.getNumVectors() == aOutput.getNumVectors());

        const OrdinalType tVectorIndex = 0;
        aOutput(tVectorIndex, 0) = (2. / (mXRadius * mXRadius)) * aVector(tVectorIndex, 0);
        aOutput(tVectorIndex, 1) = (2. / (mYRadius * mYRadius)) * aVector(tVectorIndex, 1);
    }

private:
    ScalarType mXCenter;
    ScalarType mXRadius;
    ScalarType mYCenter;
    ScalarType mYRadius;

private:
    ShiftedEllipse(const Plato::ShiftedEllipse<ScalarType, OrdinalType> & aRhs);
    Plato::ShiftedEllipse<ScalarType, OrdinalType> & operator=(const Plato::ShiftedEllipse<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato
