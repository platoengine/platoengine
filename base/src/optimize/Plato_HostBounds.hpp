/*
 * Plato_HostBounds.hpp
 *
 *  Created on: May 6, 2018
 */

#ifndef PLATO_HOSTBOUNDS_HPP_
#define PLATO_HOSTBOUNDS_HPP_

#include <cmath>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_BoundsBase.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class HostBounds : public Plato::BoundsBase<ScalarType, OrdinalType>
{
public:
    HostBounds()
    {
    }
    virtual ~HostBounds()
    {
    }

    void project(const Plato::MultiVector<ScalarType, OrdinalType> & aLowerBound,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aUpperBound,
                 Plato::MultiVector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.getNumVectors() == aUpperBound.getNumVectors());
        assert(aLowerBound.getNumVectors() == aUpperBound.getNumVectors());

        OrdinalType tNumVectors = aInput.getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tVector = aInput[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tLowerBound = aLowerBound[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tUpperBound = aUpperBound[tVectorIndex];

            assert(tVector.size() == tLowerBound.size());
            assert(tUpperBound.size() == tLowerBound.size());

            OrdinalType tNumElements = tVector.size();
            for(OrdinalType tIndex = 0; tIndex < tNumElements; tIndex++)
            {
                tVector[tIndex] = std::max(tVector[tIndex], tLowerBound[tIndex]);
                tVector[tIndex] = std::min(tVector[tIndex], tUpperBound[tIndex]);
            }
        }
    }

    void computeActiveAndInactiveSets(const Plato::MultiVector<ScalarType, OrdinalType> & aInput,
                                      const Plato::MultiVector<ScalarType, OrdinalType> & aLowerBound,
                                      const Plato::MultiVector<ScalarType, OrdinalType> & aUpperBound,
                                      Plato::MultiVector<ScalarType, OrdinalType> & aActiveSet,
                                      Plato::MultiVector<ScalarType, OrdinalType> & aInactiveSet) const
    {
        assert(aInput.getNumVectors() == aLowerBound.getNumVectors());
        assert(aInput.getNumVectors() == aInactiveSet.getNumVectors());
        assert(aActiveSet.getNumVectors() == aInactiveSet.getNumVectors());
        assert(aLowerBound.getNumVectors() == aUpperBound.getNumVectors());

        OrdinalType tNumVectors = aInput.getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tActiveSet = aActiveSet[tVectorIndex];
            Plato::Vector<ScalarType, OrdinalType> & tInactiveSet = aInactiveSet[tVectorIndex];

            const Plato::Vector<ScalarType, OrdinalType> & tVector = aInput[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tLowerBound = aLowerBound[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tUpperBound = aUpperBound[tVectorIndex];

            assert(tVector.size() == tLowerBound.size());
            assert(tVector.size() == tInactiveSet.size());
            assert(tActiveSet.size() == tInactiveSet.size());
            assert(tUpperBound.size() == tLowerBound.size());

            tActiveSet.fill(0.);
            tInactiveSet.fill(0.);

            OrdinalType tNumElements = tVector.size();
            for(OrdinalType tIndex = 0; tIndex < tNumElements; tIndex++)
            {
                tActiveSet[tIndex] = static_cast<OrdinalType>((tVector[tIndex] >= tUpperBound[tIndex])
                        || (tVector[tIndex] <= tLowerBound[tIndex]));
                tInactiveSet[tIndex] = static_cast<OrdinalType>((tVector[tIndex] < tUpperBound[tIndex])
                        && (tVector[tIndex] > tLowerBound[tIndex]));
            }
        }
    }

private:
    HostBounds(const Plato::HostBounds<ScalarType, OrdinalType>&);
    Plato::HostBounds<ScalarType, OrdinalType> & operator=(const Plato::HostBounds<ScalarType, OrdinalType>&);
};
// class HostBounds

} // namespace Plato

#endif /* PLATO_HOSTBOUNDS_HPP_ */
