/*
 * Plato_IdentityHessian.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_IDENTITYHESSIAN_HPP_
#define PLATO_IDENTITYHESSIAN_HPP_

#include <memory>

#include "Plato_StateData.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_LinearOperator.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class IdentityHessian : public Plato::LinearOperator<ScalarType, OrdinalType>
{
public:
    IdentityHessian()
    {
    }
    virtual ~IdentityHessian()
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
        Plato::update(static_cast<ScalarType>(1), aVector, static_cast<ScalarType>(0), aOutput);
    }

private:
    IdentityHessian(const Plato::IdentityHessian<ScalarType, OrdinalType> & aRhs);
    Plato::IdentityHessian<ScalarType, OrdinalType> & operator=(const Plato::IdentityHessian<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_IDENTITYHESSIAN_HPP_ */
