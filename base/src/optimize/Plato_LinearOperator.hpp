/*
 * Plato_LinearOperator.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_LINEAROPERATOR_HPP_
#define PLATO_LINEAROPERATOR_HPP_

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class StateData;
template<typename ScalarType, typename OrdinalType>
class MultiVector;

template<typename ScalarType, typename OrdinalType = size_t>
class LinearOperator
{
public:
    virtual ~LinearOperator()
    {
    }

    virtual void update(const Plato::StateData<ScalarType, OrdinalType> & aStateData) = 0;
    virtual void apply(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                       const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                       Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
};

} // namespace Plato

#endif /* PLATO_LINEAROPERATOR_HPP_ */
