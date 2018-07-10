/*
 * Plato_OptimalityCriteriaTestInequalityTwo.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_OPTIMALITYCRITERIATESTINEQUALITYTWO_HPP_
#define PLATO_OPTIMALITYCRITERIATESTINEQUALITYTWO_HPP_

#include <cmath>
#include <memory>
#include <cassert>

#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class OptimalityCriteriaTestInequalityTwo : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    explicit OptimalityCriteriaTestInequalityTwo()
    {
    }
    virtual ~OptimalityCriteriaTestInequalityTwo()
    {
    }

    void cacheData()
    {
        return;
    }
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        const OrdinalType tVectorIndex = 0;
        assert(aControl[tVectorIndex].size() == static_cast<OrdinalType>(2));

        ScalarType tDenominator = aControl(tVectorIndex, 1) + (static_cast<ScalarType>(0.25) * aControl(tVectorIndex, 0));
        ScalarType tOutput = static_cast<ScalarType>(1) - (static_cast<ScalarType>(1.5) / tDenominator);

        return (tOutput);
    }

    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aGradient)
    {
        const OrdinalType tVectorIndex = 0;
        assert(aControl[tVectorIndex].size() == static_cast<OrdinalType>(2));

        ScalarType tPower = 2;
        ScalarType tDenominator = aControl(tVectorIndex, 1) + (static_cast<ScalarType>(0.25) * aControl(tVectorIndex, 0));
        tDenominator = std::pow(tDenominator, tPower);
        ScalarType tFirstElement = static_cast<ScalarType>(0.375) / tDenominator;
        aGradient(tVectorIndex, 0) = tFirstElement;
        ScalarType tSecondElement = static_cast<ScalarType>(1.5) / tDenominator;
        aGradient(tVectorIndex, 1) = tSecondElement;
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
    OptimalityCriteriaTestInequalityTwo(const Plato::OptimalityCriteriaTestInequalityTwo<ScalarType, OrdinalType>&);
    Plato::OptimalityCriteriaTestInequalityTwo<ScalarType, OrdinalType> & operator=(const Plato::OptimalityCriteriaTestInequalityTwo<ScalarType, OrdinalType>&);
};

}

#endif /* PLATO_OPTIMALITYCRITERIATESTINEQUALITYTWO_HPP_ */
