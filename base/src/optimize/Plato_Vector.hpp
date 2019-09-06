/*
 * Plato_Vector.hpp
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
// *************************************************************************
//@HEADER
*/

#ifndef PLATO_VECTOR_HPP_
#define PLATO_VECTOR_HPP_

#include <memory>

namespace Plato
{

/******************************************************************************//**
 * @brief Supported memory space
**********************************************************************************/
struct MemorySpace
{
    enum type_t
    {
        HOST = 1, DEVICE = 2
    };
};

template<typename ScalarType, typename OrdinalType = size_t>
class Vector
{
public:
    virtual ~Vector()
    {
    }

    //! Scales a Vector by a ScalarType constant.
    virtual void scale(const ScalarType & aInput) = 0;
    //! Entry-Wise product of two vectors.
    virtual void entryWiseProduct(const Plato::Vector<ScalarType, OrdinalType> & aInput) = 0;
    //! Update vector values with scaled values of A, this = beta*this + alpha*A.
    virtual void update(const ScalarType & aAlpha,
                        const Plato::Vector<ScalarType, OrdinalType> & aInputVector,
                        const ScalarType & aBeta) = 0;
    //! Computes the absolute value of each element in the container.
    virtual void modulus() = 0;
    //! Returns the inner product of two vectors.
    virtual ScalarType dot(const Plato::Vector<ScalarType, OrdinalType> & aInputVector) const = 0;
    //! Assigns new contents to the Vector, replacing its current contents, and not modifying its size.
    virtual void fill(const ScalarType & aValue) = 0;
    //! Returns the number of local elements in the Vector.
    virtual OrdinalType size() const = 0;
    //! Creates an object of type Plato::Vector
    virtual std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> create() const = 0;
    //! Operator overloads the square bracket operator.
    virtual ScalarType & operator [](const OrdinalType & aIndex) = 0;
    //! Operator overloads the square bracket operator.
    virtual const ScalarType & operator [](const OrdinalType & aIndex) const = 0;
    //! Returns a direct pointer to the memory array used internally by the vector to store its owned elements.
    virtual ScalarType* data() = 0;
    //! Returns a direct const pointer to the memory array used internally by the vector to store its owned elements.
    virtual const ScalarType* data() const = 0;
};

} // namespace Plato

#endif /* PLATO_VECTOR_HPP_ */
