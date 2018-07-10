/*
 * Plato_EpetraSerialDenseMultiVector.hpp
 *
 *  Created on: Nov 27, 2017
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

#ifndef PLATO_EPETRASERIALDENSEMULTIVECTOR_HPP_
#define PLATO_EPETRASERIALDENSEMULTIVECTOR_HPP_

#include <vector>
#include <memory>

#include "Plato_MultiVector.hpp"
#include "Plato_EpetraSerialDenseVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class EpetraSerialDenseMultiVector : public Plato::MultiVector<ScalarType, OrdinalType>
{
public:
    explicit EpetraSerialDenseMultiVector(const OrdinalType & aNumVectors, const OrdinalType & aVectorLength, ScalarType aBaseValue = 0) :
        mData(std::vector<std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>>>(aNumVectors))
    {
        this->initialize(aVectorLength, aBaseValue);
    }
    virtual ~EpetraSerialDenseMultiVector()
    {
    }

    //! Creates a copy of type MultiVector
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> create() const
    {
        const OrdinalType tVectorIndex = 0;
        const OrdinalType tNumVectors = this->getNumVectors();
        const OrdinalType tVectorLength = mData[tVectorIndex]->size();
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tOutput;
        tOutput = std::make_shared<Plato::EpetraSerialDenseMultiVector<ScalarType, OrdinalType>>(tNumVectors, tVectorLength);
        return (tOutput);
    }
    //! Number of vectors
    OrdinalType getNumVectors() const
    {
        OrdinalType tNumVectors = mData.size();
        return (tNumVectors);
    }
    //! Operator overloads the square bracket operator
    virtual Plato::Vector<ScalarType, OrdinalType> & operator [](const OrdinalType & aVectorIndex)
    {
        assert(mData.empty() == false);
        assert(aVectorIndex < this->getNumVectors());

        return (mData[aVectorIndex].operator *());
    }
    //! Operator overloads the square bracket operator
    virtual const Plato::Vector<ScalarType, OrdinalType> & operator [](const OrdinalType & aVectorIndex) const
    {
        assert(mData.empty() == false);
        assert(mData[aVectorIndex].get() != nullptr);
        assert(aVectorIndex < this->getNumVectors());

        return (mData[aVectorIndex].operator *());
    }
    //! Operator overloads the square bracket operator
    virtual ScalarType & operator ()(const OrdinalType & aVectorIndex, const OrdinalType & aElementIndex)
    {
        assert(aVectorIndex < this->getNumVectors());
        assert(aElementIndex < mData[aVectorIndex]->size());

        return (mData[aVectorIndex].operator *().operator [](aElementIndex));
    }
    //! Operator overloads the square bracket operator
    virtual const ScalarType & operator ()(const OrdinalType & aVectorIndex, const OrdinalType & aElementIndex) const
    {
        assert(aVectorIndex < this->getNumVectors());
        assert(aElementIndex < mData[aVectorIndex]->size());

        return (mData[aVectorIndex].operator *().operator [](aElementIndex));
    }

private:
    void initialize(const OrdinalType & aVectorLength, const ScalarType & aBaseValue)
    {
        Plato::EpetraSerialDenseVector<ScalarType,OrdinalType> tVector(aVectorLength);

        OrdinalType tNumVectors = mData.size();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            mData[tIndex] = tVector.create();
            mData[tIndex]->fill(aBaseValue);
        }
    }

private:
    std::vector<std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>>> mData;

private:
    EpetraSerialDenseMultiVector(const Plato::EpetraSerialDenseMultiVector<ScalarType, OrdinalType>&);
    Plato::EpetraSerialDenseMultiVector<ScalarType, OrdinalType> & operator=(const Plato::EpetraSerialDenseMultiVector<ScalarType, OrdinalType>&);
};

} // namespace Plato

#endif /* PLATO_EPETRASERIALDENSEMULTIVECTOR_HPP_ */
