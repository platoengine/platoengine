/*
 * Plato_OptimizerInterface.hpp
 *
 *  Created on: Oct 30, 2017
 */

#ifndef PLATO_OPTIMIZERINTERFACE_HPP_
#define PLATO_OPTIMIZERINTERFACE_HPP_

namespace Plato
{

struct optimizer
{

    enum algorithm_t
    {
        OPTIMALITY_CRITERIA = 1,
        METHOD_OF_MOVING_ASYMPTOTES = 2,
        GLOBALLY_CONVERGENT_METHOD_OF_MOVING_ASYMPTOTES = 3,
        KELLEY_SACHS_UNCONSTRAINED = 4,
        KELLEY_SACHS_BOUND_CONSTRAINED = 5,
        KELLEY_SACHS_AUGMENTED_LAGRANGIAN = 6,
        DERIVATIVE_CHECKER = 7,
        RAPID_OPTIMIZATION_LIBRARY = 8,
        STOCHASTIC_REDUCED_ORDER_MODEL = 9,
    };

};

template<typename ScalarType, typename OrdinalType = size_t>
class OptimizerInterface
{
public:
    virtual ~OptimizerInterface()
    {
    }

    virtual void optimize() = 0;
    virtual void finalize() = 0;
    virtual void initialize() = 0;
};

} // namespace Plato

#endif /* PLATO_OPTIMIZERINTERFACE_HPP_ */
