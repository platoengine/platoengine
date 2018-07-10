/*
 * Plato_GradientOperator.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_GRADIENTOPERATOR_HPP_
#define PLATO_GRADIENTOPERATOR_HPP_

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class StateData;
template<typename ScalarType, typename OrdinalType>
class MultiVector;

template<typename ScalarType, typename OrdinalType = size_t>
class GradientOperator
{
public:
    virtual ~GradientOperator()
    {
    }

    virtual void update(const Plato::StateData<ScalarType, OrdinalType> & aStateData) = 0;
    virtual void compute(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                         Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
};

} // namespace Plato

#endif /* PLATO_GRADIENTOPERATOR_HPP_ */
