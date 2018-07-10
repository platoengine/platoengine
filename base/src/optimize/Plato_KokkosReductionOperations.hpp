/*
 * Plato_KokkosReductionOperations.hpp
 *
 *  Created on: May 5, 2018
 */

/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
//
// *************************************************************************
//@HEADER
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
