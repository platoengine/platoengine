/*
 * Plato_Preconditioner.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_PRECONDITIONER_HPP_
#define PLATO_PRECONDITIONER_HPP_

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class StateData;
template<typename ScalarType, typename OrdinalType>
class MultiVector;

template<typename ScalarType, typename OrdinalType = size_t>
class Preconditioner
{
public:
    virtual ~Preconditioner()
    {
    }

    virtual void update(const Plato::StateData<ScalarType, OrdinalType> & aStateData) = 0;
    virtual void applyPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                     const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                     Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
    virtual void applyInvPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                        const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                        Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
};

} // namespace Plato

#endif /* PLATO_PRECONDITIONER_HPP_ */
