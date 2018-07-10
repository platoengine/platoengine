/*
 * Plato_OptimalityCriteriaTestObjectiveOne.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_OPTIMALITYCRITERIATESTOBJECTIVEONE_HPP_
#define PLATO_OPTIMALITYCRITERIATESTOBJECTIVEONE_HPP_

#include <memory>

#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_StandardVectorReductionOperations.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class OptimalityCriteriaTestObjectiveOne : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    OptimalityCriteriaTestObjectiveOne() :
            mConstant(0.0624),
            mReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType,OrdinalType>>())
    {

    }
    explicit OptimalityCriteriaTestObjectiveOne(const Plato::ReductionOperations<ScalarType, OrdinalType> & aInterface) :
            mConstant(0.0624),
            mReductionOperations(aInterface.create())
    {
    }
    virtual ~OptimalityCriteriaTestObjectiveOne()
    {
    }

    void cacheData()
    {
        return;
    }

    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        const OrdinalType tVectorIndex = 0;
        ScalarType tSum = mReductionOperations->sum(aControl[tVectorIndex]);
        ScalarType tOutput = mConstant * tSum;
        return (tOutput);
    }

    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aGradient)
    {
        const OrdinalType tVectorIndex = 0;
        aGradient[tVectorIndex].fill(mConstant);
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
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mReductionOperations;

private:
    OptimalityCriteriaTestObjectiveOne(const Plato::OptimalityCriteriaTestObjectiveOne<ScalarType, OrdinalType>&);
    Plato::OptimalityCriteriaTestObjectiveOne<ScalarType, OrdinalType> & operator=(const Plato::OptimalityCriteriaTestObjectiveOne<ScalarType, OrdinalType>&);
};

}

#endif /* PLATO_OPTIMALITYCRITERIATESTOBJECTIVEONE_HPP_ */
