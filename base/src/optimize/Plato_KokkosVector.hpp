/*
 * Plato_KokkosVector.hpp
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

#ifndef PLATO_KOKKOSVECTOR_HPP_
#define PLATO_KOKKOSVECTOR_HPP_

#include <memory>
#include <vector>
#include <string>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_KokkosTypes.hpp"

namespace Plato
{

/******************************************************************************/
template<typename ScalarType = double, typename OrdinalType = size_t>
class KokkosVector : public Plato::Vector<ScalarType, OrdinalType>
/******************************************************************************/
{
public:
    KokkosVector(const OrdinalType & aSize, ScalarType const & aValue = 0.0, std::string const & aName = "") :
        mName(aName),
        mHostData(std::vector<ScalarType>(aSize, aValue)),
        mView()
    {
        this->initialize();
    }
    KokkosVector(const std::vector<ScalarType> & aBase, std::string const & aName = "") :
        mName(aName),
        mHostData(aBase),
        mView()
    {
        this->initialize();
    }
    virtual ~KokkosVector()
    {
    }

    //! Scales a Vector by a ScalarType constant.
    void scale(const ScalarType & aInput)
    {
        ScalarType* tMyData = this->data();
        const OrdinalType tSize = this->size();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tSize),
                             KOKKOS_LAMBDA(const OrdinalType & aIndex){
            tMyData[aIndex] = tMyData[aIndex] * aInput;
        }, "KokkosVector::scale(aInput)");
    }
    //! Entry-Wise product of two vectors.
    void entryWiseProduct(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(this->size() == aInput.size());

        ScalarType* tMyData = this->data();
        const OrdinalType tSize = this->size();
        const ScalarType* tInputData = aInput.data();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tSize),
                             KOKKOS_LAMBDA(const OrdinalType & aIndex){
            tMyData[aIndex] = tMyData[aIndex] * tInputData[aIndex];
        }, "KokkosVector::entryWiseProduct(aInput)");
    }
    //! Update vector values with scaled values of A, this = beta*this + alpha*A.
    void update(const ScalarType & aAlpha,
                const Plato::Vector<ScalarType, OrdinalType> & aInputVector,
                const ScalarType & aBeta)
    {
        assert(this->size() == aInputVector.size());

        ScalarType* tMyData = this->data();
        const OrdinalType tSize = this->size();
        const ScalarType* tInputData = aInputVector.data();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tSize),
                             KOKKOS_LAMBDA(const OrdinalType & aIndex){
            tMyData[aIndex] = aBeta * tMyData[aIndex] + aAlpha * tInputData[aIndex];
        }, "KokkosVector::update(aAlpha, aInputVec, aBeta)");
    }
    //! Computes the absolute value of each element in the container.
    void modulus()
    {
        ScalarType* tMyData = this->data();
        const OrdinalType tSize = this->size();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tSize),
                             KOKKOS_LAMBDA(const OrdinalType & aIndex){
            tMyData[aIndex] = std::abs(tMyData[aIndex]);
        }, "KokkosVector::modulus()");
    }
    //! Returns the inner product of two vectors.
    ScalarType dot(const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        assert(this->size() == aInput.size());

        ScalarType tOutput = 0.;
        const OrdinalType tSize = this->size();
        const ScalarType* tMyData = this->data();
        const ScalarType* tInputData = aInput.data();
        Kokkos::parallel_reduce("KokkosVector::dot(aInput)",
                                Kokkos::RangePolicy<>(0, tSize),
                                KOKKOS_LAMBDA(const OrdinalType & aIndex, ScalarType & aSum){
            aSum += tMyData[aIndex] * tInputData[aIndex];
        }, tOutput);

        return (tOutput);
    }
    //! Assigns new contents to vector, replacing its current contents, and not modifying its size.
    void fill(const ScalarType & aInput)
    {
        OrdinalType tSize = this->size();
        ScalarType* tMyData = mView.data();
        Kokkos::parallel_for(Kokkos::RangePolicy<>(0, tSize),
                             KOKKOS_LAMBDA(const OrdinalType & aIndex){
            tMyData[aIndex] = aInput;
        }, "KokkosVector::fill(aInput)");
    }
    //! Returns the number of local elements in the Vector.
    OrdinalType size() const
    {
        return (mView.size());
    }
    //! Creates an object of type Plato::Vector
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> create() const
    {
        const ScalarType tBaseValue = 0;
        const OrdinalType tSize = this->size();
        std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> tOutput =
                std::make_shared<Plato::KokkosVector<ScalarType, OrdinalType>>(tSize, tBaseValue);
        return (tOutput);
    }
    /*! Operator overloads the non-const square bracket operator. Data should be copy
        from device into host first by calling this->host(). */
    ScalarType & operator [](const OrdinalType & aIndex)
    {
        assert(aIndex < this->size());
        return (mHostData[aIndex]);
    }
    /*! Operator overloads the const square bracket operator. Data should be copy
        from device into host first by calling this->host(). */
    const ScalarType & operator [](const OrdinalType & aIndex) const
    {
        assert(aIndex < this->size());
        return (mHostData[aIndex]);
    }
    //! Returns a direct pointer to the memory array used internally by the vector to store its owned elements.
    ScalarType* data()
    {
        return (mView.data());
    }
    //! Returns a direct const pointer to the memory array used internally by the vector to store its owned elements.
    const ScalarType* data() const
    {
        return (mView.data());
    }
    //! Returns non-const reference to a Kokkos::View
    Plato::ScalarVectorT<ScalarType> & view()
    {
        return (mView);
    }
    //! Returns const reference to a Kokkos::View
    const Plato::ScalarVectorT<ScalarType> & view() const
    {
        return (mView);
    }
    //! Returns a const reference to the underlying host array used to store its owned elements.
    const std::vector<ScalarType> & vector()
    {
        this->host();
        return (mHostData);
    }
    //! Copy device array into an array on the host.
    void host()
    {
        auto tHostData = Kokkos::create_mirror(mView);
        Kokkos::deep_copy(tHostData, mView);

        assert(this->size() == static_cast<OrdinalType>(mHostData.size()));
        const OrdinalType tSize = this->size();
        for(OrdinalType tIndex = 0; tIndex < tSize; tIndex++)
        {
            mHostData[tIndex] = tHostData(tIndex);
        }
    }

private:
    void initialize()
    {
        const OrdinalType tSize = mHostData.size();
        mView = Plato::ScalarVectorT<ScalarType>(mName, tSize);
        auto tHostView = Kokkos::create_mirror(mView);

        const ScalarType* tMyHostData = mHostData.data();
        for(OrdinalType tIndex = 0; tIndex < tSize; tIndex++)
        {
            tHostView(tIndex) = tMyHostData[tIndex];
        }
        Kokkos::deep_copy(mView, tHostView);
    }

private:
    std::string mName;
    std::vector<ScalarType> mHostData;
    Plato::ScalarVectorT<ScalarType> mView;

private:
    KokkosVector(const Plato::KokkosVector<ScalarType, OrdinalType> & aRhs);
    Plato::KokkosVector<ScalarType, OrdinalType> & operator=(const Plato::KokkosVector<ScalarType, OrdinalType> & aRhs);
};
// class KokkosVector

} // namespace Plato

#endif /* PLATO_KOKKOSVECTOR_HPP_ */
