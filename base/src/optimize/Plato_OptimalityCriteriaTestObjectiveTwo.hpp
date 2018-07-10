/*
 * Plato_OptimalityCriteriaTestObjectiveTwo.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_OPTIMALITYCRITERIATESTOBJECTIVETWO_HPP_
#define PLATO_OPTIMALITYCRITERIATESTOBJECTIVETWO_HPP_

#include <memory>
#include <cassert>

#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class OptimalityCriteriaTestObjectiveTwo : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    OptimalityCriteriaTestObjectiveTwo()
    {
    }
    virtual ~OptimalityCriteriaTestObjectiveTwo()
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
        ScalarType tOutput = aControl(tVectorIndex, 0) + (static_cast<ScalarType>(2) * aControl(tVectorIndex, 1));
        return (tOutput);
    }

    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aGradient)
    {
        const OrdinalType tVectorIndex = 0;
        assert(aControl[tVectorIndex].size() == static_cast<OrdinalType>(2));
        aGradient(tVectorIndex, 0) = static_cast<ScalarType>(1);
        aGradient(tVectorIndex, 1) = static_cast<ScalarType>(2);
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
    OptimalityCriteriaTestObjectiveTwo(const Plato::OptimalityCriteriaTestObjectiveTwo<ScalarType, OrdinalType>&);
    Plato::OptimalityCriteriaTestObjectiveTwo<ScalarType, OrdinalType> & operator=(const Plato::OptimalityCriteriaTestObjectiveTwo<
            ScalarType, OrdinalType>&);
};

}

#endif /* PLATO_OPTIMALITYCRITERIATESTOBJECTIVETWO_HPP_ */
