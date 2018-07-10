/*
 * Plato_CcsaTestInequality.hpp
 *
 *  Created on: Nov 4, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_CCSATESTINEQUALITY_HPP_
#define PLATO_CCSATESTINEQUALITY_HPP_

#include <cmath>
#include <vector>
#include <memory>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class CcsaTestInequality : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    CcsaTestInequality() :
            mConstant(1)
    {
    }
    virtual ~CcsaTestInequality()
    {
    }

    void cacheData()
    {
        return;
    }
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));

        const OrdinalType tVectorIndex = 0;
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];

        ScalarType tTermOne = static_cast<ScalarType>(61) / std::pow(tMyControl[0], static_cast<ScalarType>(3));
        ScalarType tTermTwo = static_cast<ScalarType>(37) / std::pow(tMyControl[1], static_cast<ScalarType>(3));
        ScalarType tTermThree = static_cast<ScalarType>(19) / std::pow(tMyControl[2], static_cast<ScalarType>(3));
        ScalarType tTermFour = static_cast<ScalarType>(7) / std::pow(tMyControl[3], static_cast<ScalarType>(3));
        ScalarType tTermFive = static_cast<ScalarType>(1) / std::pow(tMyControl[4], static_cast<ScalarType>(3));
        ScalarType tResidual = tTermOne + tTermTwo + tTermThree + tTermFour + tTermFive;
        tResidual = tResidual - mConstant;

        return (tResidual);
    }
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aControl.getNumVectors() > static_cast<OrdinalType>(0));

        const OrdinalType tVectorIndex = 0;
        Plato::Vector<ScalarType, OrdinalType> & tMyGradient = aOutput[tVectorIndex];
        const Plato::Vector<ScalarType, OrdinalType> & tMyControl = aControl[tVectorIndex];

        ScalarType tScaleFactor = -3;
        tMyGradient[0] = tScaleFactor
                * (static_cast<ScalarType>(61) / std::pow(tMyControl[0], static_cast<ScalarType>(4)));
        tMyGradient[1] = tScaleFactor
                * (static_cast<ScalarType>(37) / std::pow(tMyControl[1], static_cast<ScalarType>(4)));
        tMyGradient[2] = tScaleFactor
                * (static_cast<ScalarType>(19) / std::pow(tMyControl[2], static_cast<ScalarType>(4)));
        tMyGradient[3] = tScaleFactor
                * (static_cast<ScalarType>(7) / std::pow(tMyControl[3], static_cast<ScalarType>(4)));
        tMyGradient[4] = tScaleFactor
                * (static_cast<ScalarType>(1) / std::pow(tMyControl[4], static_cast<ScalarType>(4)));
    }
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        const ScalarType tScalarValue = 0;
        const OrdinalType tVectorIndex = 0;
        aOutput[tVectorIndex].fill(tScalarValue);
    }

private:
    ScalarType mConstant;

private:
    CcsaTestInequality(const Plato::CcsaTestInequality<ScalarType, OrdinalType> & aRhs);
    Plato::CcsaTestInequality<ScalarType, OrdinalType> & operator=(const Plato::CcsaTestInequality<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_CCSATESTINEQUALITY_HPP_ */
