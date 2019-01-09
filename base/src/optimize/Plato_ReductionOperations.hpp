/*
 * Plato_ReductionOperations.hpp
 *
 *  Created on: Oct 6, 2017
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

#ifndef PLATO_REDUCTIONOPERATIONS_HPP_
#define PLATO_REDUCTIONOPERATIONS_HPP_

#include <memory>

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class Vector;

/******************************************************************************//**
 * @brief Output struct specialized for min_loc reduction operations.
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
struct ReductionOutputs
{
    ScalarType mOutputValue; /*!< output value, e.g. global minimum */
    OrdinalType mOutputRank; /*!< rank ID that owns output value  */
    OrdinalType mOutputIndex; /*!< local element index associated with the rank ID that owns the output value  */
};
// struct ReductionOutputs

/******************************************************************************//**
 * @brief Interface to common parallel programming operations used to reduce the
 *        elements of an array into a single result.
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class ReductionOperations
{
public:
    virtual ~ReductionOperations()
    {
    }

    /******************************************************************************//**
     * @brief Returns the global maximum element in range.
     * @param [in] aInput array of elements
     * @return global maximum
    **********************************************************************************/
    virtual ScalarType max(const Plato::Vector<ScalarType, OrdinalType> & aInput) const = 0;

    /******************************************************************************//**
     * @brief Returns the global minimum element in range.
     * @param [in] aInput array of elements
     * @return global minimum
    **********************************************************************************/
    virtual ScalarType min(const Plato::Vector<ScalarType, OrdinalType> & aInput) const = 0;

    /******************************************************************************//**
     * @brief Returns the global sum of all the elements in the container.
     * @param [in] aInput array of elements
     * @return global sum
    **********************************************************************************/
    virtual ScalarType sum(const Plato::Vector<ScalarType, OrdinalType> & aInput) const = 0;

    /******************************************************************************//**
     * @brief Computes a global minimum and also the index attached to the minimum value.
     * @param [in] aInput array of elements
     * @param [out] aOutput struct with global minimum and the rank and index attached to the minimum
    **********************************************************************************/
    virtual void minloc(const Plato::Vector<ScalarType, OrdinalType> & aInput,
                        Plato::ReductionOutputs<ScalarType, OrdinalType> & aOutput) const = 0;

    /******************************************************************************//**
     * @brief Returns a copy of a ReductionOperations instance
     * @return copy of this instance
    **********************************************************************************/
    //! Creates object of type Plato::ReductionOperations
    virtual std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> create() const = 0;
};
// class ReductionOperations

} // namespace Plato

#endif /* PLATO_REDUCTIONOPERATIONS_HPP_ */
