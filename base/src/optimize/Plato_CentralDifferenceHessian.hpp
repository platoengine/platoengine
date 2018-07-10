/*
 * Plato_CentralDifferenceHessian.hpp
 *
 *  Created on: Feb 1, 2018
 */

#ifndef PLATO_CENTRALDIFFERENCEHESSIAN_HPP_
#define PLATO_CENTRALDIFFERENCEHESSIAN_HPP_

#include "Plato_StateData.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_LinearOperator.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class CentralDifferenceHessian : public Plato::LinearOperator<ScalarType, OrdinalType>
{
public:
    explicit CentralDifferenceHessian(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                      const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aCriterion) :
            mEpsilon(1e-6),
            mCriterion(aCriterion),
            mGradient(aFactory->control().create()),
            mTrialControl(aFactory->control().create())
    {
    }
    virtual ~CentralDifferenceHessian()
    {
    }

    void setEpsilon(const ScalarType & aInput)
    {
        mEpsilon = aInput;
    }

    void update(const Plato::StateData<ScalarType, OrdinalType> & aStateData)
    {
        return;
    }

    void apply(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
               const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
               Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        // Compute \hat{x} = x_{current} + \epsilon d, where d denotes the direction vector and \epsilon denotes the perturbation
        Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *mTrialControl);
        Plato::update(mEpsilon, aVector, static_cast<ScalarType>(1), *mTrialControl);
        mCriterion->gradient(*mTrialControl, aOutput);

        // Compute \hat{x} = x_{current} - \epsilon d, where d denotes the direction vector and \epsilon denotes the perturbation
        Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *mTrialControl);
        Plato::update(-mEpsilon, aVector, static_cast<ScalarType>(1), *mTrialControl);
        mCriterion->gradient(*mTrialControl, *mGradient);

        // Compute central difference approximation
        Plato::update(static_cast<ScalarType>(-1), *mGradient, static_cast<ScalarType>(1), aOutput);
        const ScalarType tConstant = static_cast<ScalarType>(0.5) / mEpsilon;
        Plato::scale(tConstant, aOutput);
    }

private:
    ScalarType mEpsilon;
    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mCriterion;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialControl;

private:
    CentralDifferenceHessian(const Plato::CentralDifferenceHessian<ScalarType, OrdinalType> & aRhs);
    Plato::CentralDifferenceHessian<ScalarType, OrdinalType> & operator=(const Plato::CentralDifferenceHessian<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_CENTRALDIFFERENCEHESSIAN_HPP_ */
