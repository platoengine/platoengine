/*
 * Plato_LinearCriterionHessian.hpp
 *
 *  Created on: Feb 5, 2018
 */

#ifndef BASE_SRC_OPTIMIZE_PLATO_LINEARCRITERIONHESSIAN_HPP_
#define BASE_SRC_OPTIMIZE_PLATO_LINEARCRITERIONHESSIAN_HPP_

#include "Plato_StateData.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_LinearOperator.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class LinearCriterionHessian : public Plato::LinearOperator<ScalarType, OrdinalType>
{
public:
    LinearCriterionHessian()
    {
    }
    virtual ~LinearCriterionHessian()
    {
    }

    void update(const Plato::StateData<ScalarType, OrdinalType> & aStateData)
    {
        return;
    }
    void apply(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
               const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
               Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        Plato::fill(static_cast<ScalarType>(0), aOutput);
    }

private:
    LinearCriterionHessian(const Plato::LinearCriterionHessian<ScalarType, OrdinalType> & aRhs);
    Plato::LinearCriterionHessian<ScalarType, OrdinalType> & operator=(const Plato::LinearCriterionHessian<ScalarType, OrdinalType> & aRhs);
};

} // namespace

#endif /* BASE_SRC_OPTIMIZE_PLATO_LINEARCRITERIONHESSIAN_HPP_ */
