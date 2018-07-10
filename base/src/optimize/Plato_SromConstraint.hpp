/*
 * Plato_SromConstraint.hpp
 *
 *  Created on: Jan 31, 2018
 */

#ifndef PLATO_SROMCONSTRAINT_HPP_
#define PLATO_SROMCONSTRAINT_HPP_

#include <memory>

#include "Plato_Vector.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_ReductionOperations.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class SromConstraint : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    SromConstraint(const std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> & aReductionOperations) :
            mReductionOperations(aReductionOperations)
    {
    }
    virtual ~SromConstraint()
    {
    }

    void cacheData()
    {
        return;
    }
    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        const OrdinalType tVectorIndex = aControl.getNumVectors() - static_cast<OrdinalType>(1);
        const Plato::Vector<ScalarType, OrdinalType> & tProbabilities = aControl[tVectorIndex];
        ScalarType tSum = mReductionOperations->sum(tProbabilities);
        ScalarType tOutput = tSum - static_cast<ScalarType>(1);
        return (tOutput);
    }
    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        const OrdinalType tNumDimensions = aControl.getNumVectors() - static_cast<OrdinalType>(1);
        for(OrdinalType tIndex = 0; tIndex < tNumDimensions; tIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tMySamplesGradient = aOutput[tIndex];
            tMySamplesGradient.fill(static_cast<ScalarType>(0));
        }
        Plato::Vector<ScalarType, OrdinalType> & tMyProbabilityGradient = aOutput[tNumDimensions];
        tMyProbabilityGradient.fill(static_cast<ScalarType>(1));
    }
    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        Plato::fill(static_cast<ScalarType>(0), aOutput);
    }

private:
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mReductionOperations;

private:
    SromConstraint(const Plato::SromConstraint<ScalarType, OrdinalType> & aRhs);
    Plato::SromConstraint<ScalarType, OrdinalType> & operator=(const Plato::SromConstraint<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* BASE_SRC_OPTIMIZE_PLATO_SROMCONSTRAINT_HPP_ */
