/*
 * Plato_AnalyticalGradient.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_ANALYTICALGRADIENT_HPP_
#define PLATO_ANALYTICALGRADIENT_HPP_

#include <memory>

#include "Plato_StateData.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_GradientOperator.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class AnalyticalGradient : public Plato::GradientOperator<ScalarType, OrdinalType>
{
public:
    explicit AnalyticalGradient(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aCriterion) :
            mCriterion(aCriterion)
    {
    }
    virtual ~AnalyticalGradient()
    {
    }

    void update(const Plato::StateData<ScalarType, OrdinalType> & aStateData)
    {
        return;
    }
    void compute(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        Plato::fill(static_cast<ScalarType>(0), aOutput);
        mCriterion->gradient(aControl, aOutput);
    }

private:
    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mCriterion;

private:
    AnalyticalGradient(const Plato::AnalyticalGradient<ScalarType, OrdinalType> & aRhs);
    Plato::AnalyticalGradient<ScalarType, OrdinalType> & operator=(const Plato::AnalyticalGradient<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_ANALYTICALGRADIENT_HPP_ */
