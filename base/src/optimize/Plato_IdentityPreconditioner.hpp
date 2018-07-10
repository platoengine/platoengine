/*
 * Plato_IdentityPreconditioner.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_IDENTITYPRECONDITIONER_HPP_
#define PLATO_IDENTITYPRECONDITIONER_HPP_

#include "Plato_StateData.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_Preconditioner.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class IdentityPreconditioner : public Preconditioner<ScalarType, OrdinalType>
{
public:
    IdentityPreconditioner()
    {
    }
    virtual ~IdentityPreconditioner()
    {
    }
    void update(const Plato::StateData<ScalarType, OrdinalType> & aStateData)
    {
        return;
    }
    void applyPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                             const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                             Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aVector.getNumVectors() == aOutput.getNumVectors());
        Plato::update(1., aVector, 0., aOutput);
    }
    void applyInvPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(aVector.getNumVectors() == aOutput.getNumVectors());
        Plato::update(1., aVector, 0., aOutput);
    }

private:
    IdentityPreconditioner(const Plato::IdentityPreconditioner<ScalarType, OrdinalType> & aRhs);
    Plato::IdentityPreconditioner<ScalarType, OrdinalType> & operator=(const Plato::IdentityPreconditioner<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_IDENTITYPRECONDITIONER_HPP_ */
