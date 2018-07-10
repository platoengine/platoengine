/*
 * Plato_StateManager.hpp
 *
 *  Created on: Oct 22, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_STATEMANAGER_HPP_
#define PLATO_STATEMANAGER_HPP_

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class MultiVector;

template<typename ScalarType, typename OrdinalType = size_t>
class StateManager
{
public:
    virtual ~StateManager()
    {
    }

    virtual ScalarType evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl) = 0;
    virtual void computeGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
    virtual void applyVectorToHessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                      const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                      Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;

    virtual ScalarType getCurrentObjectiveValue() const = 0;
    virtual void setCurrentObjectiveValue(const ScalarType & aInput) = 0;
    virtual const Plato::MultiVector<ScalarType, OrdinalType> & getTrialStep() const = 0;
    virtual void setTrialStep(const Plato::MultiVector<ScalarType, OrdinalType> & aInput) = 0;
    virtual const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentControl() const = 0;
    virtual void setCurrentControl(const Plato::MultiVector<ScalarType, OrdinalType> & aInput) = 0;
    virtual const Plato::MultiVector<ScalarType, OrdinalType> & getCurrentGradient() const = 0;
    virtual void setCurrentGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aInput) = 0;
    virtual const Plato::MultiVector<ScalarType, OrdinalType> & getControlLowerBounds() const = 0;
    virtual void setControlLowerBounds(const Plato::MultiVector<ScalarType, OrdinalType> & aInput) = 0;
    virtual const Plato::MultiVector<ScalarType, OrdinalType> & getControlUpperBounds() const = 0;
    virtual void setControlUpperBounds(const Plato::MultiVector<ScalarType, OrdinalType> & aInput) = 0;
};

} // namespace Plato

#endif /* PLATO_STATEMANAGER_HPP_ */
