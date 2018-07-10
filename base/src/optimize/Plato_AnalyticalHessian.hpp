/*
 * Plato_AnalyticalHessian.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_ANALYTICALHESSIAN_HPP_
#define PLATO_ANALYTICALHESSIAN_HPP_

#include <memory>

#include "Plato_Criterion.hpp"
#include "Plato_StateData.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_LinearOperator.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class AnalyticalHessian : public Plato::LinearOperator<ScalarType, OrdinalType>
{
public:
    explicit AnalyticalHessian(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aCriterion) :
            mCriterion(aCriterion)
    {
    }
    virtual ~AnalyticalHessian()
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
        mCriterion->hessian(aControl, aVector, aOutput);
    }

private:
    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mCriterion;

private:
    AnalyticalHessian(const Plato::AnalyticalHessian<ScalarType, OrdinalType> & aRhs);
    Plato::AnalyticalHessian<ScalarType, OrdinalType> & operator=(const Plato::AnalyticalHessian<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_ANALYTICALHESSIAN_HPP_ */
