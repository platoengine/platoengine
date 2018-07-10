/*
 * Plato_DeviceBounds.hpp
 *
 *  Created on: May 6, 2018
 */

#ifndef PLATO_DEVICEBOUNDS_HPP_
#define PLATO_DEVICEBOUNDS_HPP_

#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_BoundsBase.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_KokkosTypes.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class DeviceBounds : public Plato::BoundsBase<ScalarType, OrdinalType>
{
public:
    DeviceBounds()
    {
    }
    virtual ~DeviceBounds()
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
            Plato::Vector<ScalarType, OrdinalType> & tInput = aInput[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tLowerBound = aLowerBound[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tUpperBound = aUpperBound[tVectorIndex];

            assert(tInput.size() == tLowerBound.size());
            assert(tUpperBound.size() == tLowerBound.size());

            ScalarType* tInputData = tInput.data();
            const ScalarType* tUpperBoundData = tUpperBound.data();
            const ScalarType* tLowerBoundData = tLowerBound.data();

            OrdinalType tNumElements = tInput.size();
            Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tNumElements), KOKKOS_LAMBDA(const OrdinalType & aIndex)
            {
                tInputData[aIndex] = fmax(tInputData[aIndex], tLowerBoundData[aIndex]);
                tInputData[aIndex] = fmin(tInputData[aIndex], tUpperBoundData[aIndex]);
            }, "DeviceBounds::project");
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
            tActiveSet.fill(0.);
            Plato::Vector<ScalarType, OrdinalType> & tInactiveSet = aInactiveSet[tVectorIndex];
            tInactiveSet.fill(0.);

            const Plato::Vector<ScalarType, OrdinalType> & tVector = aInput[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tLowerBound = aLowerBound[tVectorIndex];
            const Plato::Vector<ScalarType, OrdinalType> & tUpperBound = aUpperBound[tVectorIndex];

            assert(tVector.size() == tLowerBound.size());
            assert(tVector.size() == tInactiveSet.size());
            assert(tActiveSet.size() == tInactiveSet.size());
            assert(tUpperBound.size() == tLowerBound.size());

            ScalarType* tActiveSetData = tActiveSet.data();
            ScalarType* tInactiveSetData = tInactiveSet.data();

            const ScalarType* tVectorData = tVector.data();
            const ScalarType* tLowerBoundData = tLowerBound.data();
            const ScalarType* tUpperBoundData = tUpperBound.data();

            OrdinalType tNumElements = tVector.size();
            Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tNumElements), KOKKOS_LAMBDA(const OrdinalType & aIndex)
            {
                tActiveSetData[aIndex] = static_cast<OrdinalType>((tVectorData[aIndex] >= tUpperBoundData[aIndex])
                        || (tVectorData[aIndex] <= tLowerBoundData[aIndex]));

                tInactiveSetData[aIndex] = static_cast<OrdinalType>((tVectorData[aIndex] < tUpperBoundData[aIndex])
                        && (tVectorData[aIndex] > tLowerBoundData[aIndex]));
            }, "DeviceBounds::computeActiveAndInactiveSets");
        }
    }

private:
    DeviceBounds(const Plato::DeviceBounds<ScalarType, OrdinalType>&);
    Plato::DeviceBounds<ScalarType, OrdinalType> & operator=(const Plato::DeviceBounds<ScalarType, OrdinalType>&);
};
// class DeviceBounds

} // namespace Plato

#endif /* PLATO_DEVICEBOUNDS_HPP_ */
