/*
 * Plato_BoundsBase.hpp
 *
 *  Created on: May 6, 2018
 */

#ifndef PLATO_BOUNDSBASE_HPP_
#define PLATO_BOUNDSBASE_HPP_

#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class BoundsBase
{
public:
    virtual ~BoundsBase()
    {
    }

    virtual void project(const Plato::MultiVector<ScalarType, OrdinalType> & aLowerBound,
                         const Plato::MultiVector<ScalarType, OrdinalType> & aUpperBound,
                         Plato::MultiVector<ScalarType, OrdinalType> & aInput) const = 0;

    virtual void computeActiveAndInactiveSets(const Plato::MultiVector<ScalarType, OrdinalType> & aInput,
                                              const Plato::MultiVector<ScalarType, OrdinalType> & aLowerBound,
                                              const Plato::MultiVector<ScalarType, OrdinalType> & aUpperBound,
                                              Plato::MultiVector<ScalarType, OrdinalType> & aActiveSet,
                                              Plato::MultiVector<ScalarType, OrdinalType> & aInactiveSet) const = 0;
};
// class BoundsBase

} // namespace Plato



#endif /* PLATO_BOUNDSBASE_HPP_ */
