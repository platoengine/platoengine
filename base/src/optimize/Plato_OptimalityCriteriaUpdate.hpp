/*
 * Plato_OptimalityCriteriaUpdate.hpp
 *
 *  Created on: May 6, 2018
 */

#ifndef PLATO_OPTIMALITYCRITERIAUPDATE_HPP_
#define PLATO_OPTIMALITYCRITERIAUPDATE_HPP_

#include <memory>

#include "Plato_Vector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class OptimalityCriteriaUpdate
{
public:
    virtual ~OptimalityCriteriaUpdate(){}

    virtual void reset(const ScalarType & aMoveLimit,
                       const ScalarType & aScaleFactor,
                       const ScalarType & aDampingPower) = 0;

    virtual void update(const ScalarType & aTrialDual,
                        const Plato::Vector<ScalarType, OrdinalType> & aLowerBounds,
                        const Plato::Vector<ScalarType, OrdinalType> & aUpperBounds,
                        const Plato::Vector<ScalarType, OrdinalType> & aPreviousControls,
                        const Plato::Vector<ScalarType, OrdinalType> & aObjectiveGradient,
                        const Plato::Vector<ScalarType, OrdinalType> & aInequalityGradient,
                        Plato::Vector<ScalarType, OrdinalType> & aTrialControls) = 0;

    virtual std::shared_ptr<Plato::OptimalityCriteriaUpdate<ScalarType, OrdinalType>> create() const = 0;
};
// class OptimalityCriteriaUpdate

} // namespace Plato

#endif /* PLATO_OPTIMALITYCRITERIAUPDATE_HPP_ */
