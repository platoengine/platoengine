/*
 * Plato_KokkosReductionOperations.hpp
 *
 *  Created on: May 5, 2018
 */

#ifndef PLATO_KOKKOSREDUCTIONOPERATIONS_HPP_
#define PLATO_KOKKOSREDUCTIONOPERATIONS_HPP_

#include <memory>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_KokkosTypes.hpp"
#include "Plato_ReductionOperations.hpp"

namespace Plato
{

/******************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class KokkosReductionOperations : public Plato::ReductionOperations<ScalarType, OrdinalType>
/******************************************************************************/
{
public:
    KokkosReductionOperations()
    {
    }
    virtual ~KokkosReductionOperations()
    {
    }

    //! Returns the maximum element in range
    ScalarType max(const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        const OrdinalType tSize = aInput.size();
        const ScalarType* tInputData = aInput.data();

        ScalarType tOutput;
#ifdef NEW_TRILINOS_INTEGRATION
        Kokkos::Max<ScalarType> tMaxReducer(tOutput);
#else
        Kokkos::Experimental::Max<ScalarType> tMaxReducer(tOutput);
#endif
        Kokkos::parallel_reduce("KokkosReductionOperations::max",
                                Kokkos::RangePolicy<>(0, tSize),
                                KOKKOS_LAMBDA(const OrdinalType & aIndex, ScalarType & aValue){
            tMaxReducer.join(aValue, tInputData[aIndex]);
        }, tMaxReducer);

        return (tOutput);
    }
    //! Returns the minimum element in range
    ScalarType min(const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        const OrdinalType tSize = aInput.size();
        const ScalarType* tInputData = aInput.data();

        ScalarType tOutput;
#ifdef NEW_TRILINOS_INTEGRATION
        Kokkos::Min<ScalarType> tMinReducer(tOutput);
#else
        Kokkos::Experimental::Min<ScalarType> tMinReducer(tOutput);
#endif
        Kokkos::parallel_reduce("KokkosReductionOperations::min",
                                Kokkos::RangePolicy<>(0, tSize),
                                KOKKOS_LAMBDA(const OrdinalType & aIndex, ScalarType & aValue){
            tMinReducer.join(aValue, tInputData[aIndex]);
        }, tMinReducer);

        return (tOutput);
    }
    //! Returns the sum of all the elements in container.
    ScalarType sum(const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));

        ScalarType tOutput = 0.;
        const OrdinalType tSize = aInput.size();
        const ScalarType* tInputData = aInput.data();
        Kokkos::parallel_reduce("KokkosReductionOperations::sum",
                                Kokkos::RangePolicy<>(0, tSize),
                                KOKKOS_LAMBDA(const OrdinalType & aIndex, ScalarType & aSum){
            aSum += tInputData[aIndex];
        }, tOutput);

        return (tOutput);
    }
    //! Creates object of type Plato::ReductionOperations
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> create() const
    {
        std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> tCopy =
                std::make_shared<KokkosReductionOperations<ScalarType, OrdinalType>>();
        return (tCopy);
    }

private:
    KokkosReductionOperations(const Plato::KokkosReductionOperations<ScalarType, OrdinalType> & aRhs);
    Plato::KokkosReductionOperations<ScalarType, OrdinalType> & operator=(const Plato::KokkosReductionOperations<ScalarType,OrdinalType> & aRhs);
};
// class KokkosReductionOperations

} // namespace Plato

#endif /* PLATO_KOKKOSREDUCTIONOPERATIONS_HPP_ */
