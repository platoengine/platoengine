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

/******************************************************************************//**
 * @brief Operations used to reduce the elements of an array into a single result.
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class KokkosReductionOperations : public Plato::ReductionOperations<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
    **********************************************************************************/
    KokkosReductionOperations()
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~KokkosReductionOperations()
    {
    }

    /******************************************************************************//**
     * @brief Returns the global maximum element in range.
     * @param [in] aInput array of elements
     * @return global maximum value
    **********************************************************************************/
    ScalarType max(const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        const OrdinalType tSize = aInput.size();
        const ScalarType* tInputData = aInput.data();

        ScalarType tOutput;
        Kokkos::Max<ScalarType> tMaxReducer(tOutput);
        Kokkos::parallel_reduce("KokkosReductionOperations::max",
                                Kokkos::RangePolicy<>(0, tSize),
                                KOKKOS_LAMBDA(const OrdinalType & aIndex, ScalarType & aValue){
            tMaxReducer.join(aValue, tInputData[aIndex]);
        }, tMaxReducer);

        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Returns the global minimum element in range.
     * @param [in] aInput array of elements
     * @return global minimum value
    **********************************************************************************/
    ScalarType min(const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        const OrdinalType tSize = aInput.size();
        const ScalarType* tInputData = aInput.data();

        ScalarType tOutput;
        Kokkos::Min<ScalarType> tMinReducer(tOutput);
        Kokkos::parallel_reduce("KokkosReductionOperations::min",
                                Kokkos::RangePolicy<>(0, tSize),
                                KOKKOS_LAMBDA(const OrdinalType & aIndex, ScalarType & aValue){
            tMinReducer.join(aValue, tInputData[aIndex]);
        }, tMinReducer);

        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Returns the global sum of all the elements in the container.
     * @param [in] aInput array of elements
     * @return global sum
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Computes minimum value and also the index attached to the minimum value.
     * @param [in] aInput array of elements
     * @param [out] aOutput struct with minimum value and the index attached to the minimum
    **********************************************************************************/
    void minloc(const Plato::Vector<ScalarType, OrdinalType> & aInput,
                Plato::ReductionOutputs<ScalarType, OrdinalType> & aOutput) const
    {
        // TODO: FINISH: KOKKOS CODE BELOW IS NOT BUILDING
        aOutput.mOutputIndex = 0;
        aOutput.mOutputValue = aInput[0];
        const OrdinalType tMyNumElements = aInput.size();
        for(OrdinalType tIndex = 0; tIndex < tMyNumElements; tIndex++)
        {
            if(aInput[tIndex] < aOutput.mOutputValue)
            {
                aOutput.mOutputValue = aInput[tIndex];
                aOutput.mOutputIndex = tIndex;
            }
        }
        aOutput.mOutputRank = 0;

/*        typedef Kokkos::MinLoc<ScalarType, OrdinalType>::value_type minloc_type;
        minloc_type tGlobalMinLoc;
        Kokkos::MinLoc<ScalarType, OrdinalType> tMinLocReducer(tGlobalMinLoc);

        const OrdinalType tSize = aInput.size();
        const ScalarType* tInputData = aInput.data();
        Kokkos::parallel_reduce( "MinLocReduce",
            Kokkos::RangePolicy<>(0, tSize),
            KOKKOS_LAMBDA (const OrdinalType & aIndex, Kokkos::Experimental::MinLoc<ScalarType, OrdinalType>& aLocalMinLoc)
        {
            if( tInputData[aIndex] < aLocalMinLoc.val )
            {
                aLocalMinLoc.loc = aIndex;
                aLocalMinLoc.val = tInputData[aIndex];
            }
        }, tMinLocReducer);

        aOutput.mOutputRank = 0;
        aOutput.mOutputValue = tMinLocReducer.val;
        aOutput.mOutputIndex = tMinLocReducer.loc;*/
    }

    /******************************************************************************//**
     * @brief Returns a copy of a ReductionOperations instance
     * @return copy of this instance
    **********************************************************************************/
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
