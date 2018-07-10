/*
 * Plato_OptimalityCriteriaTestInequalityOne.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_OPTIMALITYCRITERIATESTINEQUALITYONE_HPP_
#define PLATO_OPTIMALITYCRITERIATESTINEQUALITYONE_HPP_


#include <cmath>
#include <memory>
#include <cassert>

#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class OptimalityCriteriaTestInequalityOne : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    explicit OptimalityCriteriaTestInequalityOne() :
            mBound(1.)
    {
    }
    virtual ~OptimalityCriteriaTestInequalityOne()
    {
    }

    void cacheData()
    {
        return;
    }
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        const OrdinalType tVectorIndex = 0;
        assert(aControl[tVectorIndex].size() == static_cast<OrdinalType>(5));

        const ScalarType tPower = 3.;
        ScalarType tFirstTerm = static_cast<ScalarType>(61.) / std::pow(aControl(tVectorIndex,0), tPower);
        ScalarType tSecondTerm = static_cast<ScalarType>(37.) / std::pow(aControl(tVectorIndex,1), tPower);
        ScalarType tThirdTerm = static_cast<ScalarType>(19.) / std::pow(aControl(tVectorIndex,2), tPower);
        ScalarType tFourthTerm = static_cast<ScalarType>(7.) / std::pow(aControl(tVectorIndex,3), tPower);
        ScalarType tFifthTerm = static_cast<ScalarType>(1.) / std::pow(aControl(tVectorIndex,4), tPower);

        ScalarType tValue = tFirstTerm + tSecondTerm + tThirdTerm + tFourthTerm + tFifthTerm;
        ScalarType tOutput = tValue - mBound;

        return (tOutput);
    }

    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aGradient)
    {
        const OrdinalType tVectorIndex = 0;
        assert(aControl[tVectorIndex].size() == static_cast<OrdinalType>(5));

        const ScalarType tPower = 4;
        const ScalarType tScaleFactor = -3.;
        aGradient(tVectorIndex,0) = tScaleFactor * (static_cast<ScalarType>(61.) / std::pow(aControl(tVectorIndex,0), tPower));
        aGradient(tVectorIndex,1) = tScaleFactor * (static_cast<ScalarType>(37.) / std::pow(aControl(tVectorIndex,1), tPower));
        aGradient(tVectorIndex,2) = tScaleFactor * (static_cast<ScalarType>(19.) / std::pow(aControl(tVectorIndex,2), tPower));
        aGradient(tVectorIndex,3) = tScaleFactor * (static_cast<ScalarType>(7.) / std::pow(aControl(tVectorIndex,3), tPower));
        aGradient(tVectorIndex,4) = tScaleFactor * (static_cast<ScalarType>(1.) / std::pow(aControl(tVectorIndex,4), tPower));
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
    ScalarType mBound;

private:
    OptimalityCriteriaTestInequalityOne(const Plato::OptimalityCriteriaTestInequalityOne<ScalarType, OrdinalType>&);
    Plato::OptimalityCriteriaTestInequalityOne<ScalarType, OrdinalType> & operator=(const Plato::OptimalityCriteriaTestInequalityOne<ScalarType, OrdinalType>&);
};

}

#endif /* PLATO_OPTIMALITYCRITERIATESTINEQUALITYONE_HPP_ */
