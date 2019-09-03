/*
 * Plato_StandardMultiVector.hpp
 *
 *  Created on: Oct 17, 2017
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

#ifndef PLATO_STANDARDMULTIVECTOR_HPP_
#define PLATO_STANDARDMULTIVECTOR_HPP_

#include <vector>
#include <cassert>

#include "Plato_MultiVector.hpp"
#include "Plato_StandardVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class StandardMultiVector : public Plato::MultiVector<ScalarType, OrdinalType>
{
public:
    StandardMultiVector() :
        mData()
    {
    }
    explicit StandardMultiVector(const OrdinalType & aNumVectors, const std::vector<ScalarType> & aVectorTemplate) :
        mData(std::vector<std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>>>(aNumVectors))
    {
        this->initialize(aVectorTemplate);
    }
    explicit StandardMultiVector(const OrdinalType & aNumVectors, const Plato::Vector<ScalarType, OrdinalType> & aVectorTemplate) :
        mData(std::vector<std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>>>(aNumVectors))
    {
        this->initialize(aVectorTemplate);
    }
    explicit StandardMultiVector(const std::vector<std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>>>& aMultiVectorTemplate) :
        mData(std::vector<std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>>>(aMultiVectorTemplate.size()))
    {
        this->initialize(aMultiVectorTemplate);
    }
    explicit StandardMultiVector(const OrdinalType & aNumVectors, const OrdinalType & aVectorLength, ScalarType aBaseValue = 0) :
        mData(std::vector<std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>>>(aNumVectors))
    {
        this->initialize(aVectorLength, aBaseValue);
    }
    virtual ~StandardMultiVector()
    {
    }

    /******************************************************************************//**
     * @brief Copy data from input vector to Plato vector
     * @param [in] aIndex vector index
     * @param [in] aVec input vector
     **********************************************************************************/
    void setData(const OrdinalType & aIndex, const std::vector<ScalarType> & aVec)
    {
        assert(static_cast<OrdinalType>(mData[aIndex]->size()) == static_cast<OrdinalType>(aVec.size()));
        const OrdinalType tLength = mData[aIndex]->size();
        for(OrdinalType tIndex = 0; tIndex < tLength; tIndex++)
        {
            mData[aIndex]->operator[](tIndex) = aVec[tIndex];
        }
    }

    /******************************************************************************//**
     * @brief Copy data from Plato vector to std::vector
     * @param [in] aIndex vector index
     * @param [in,out] aVec input vector
     **********************************************************************************/
    void getData(const OrdinalType & aIndex, std::vector<ScalarType> & aVec)
    {
        assert(static_cast<OrdinalType>(mData[aIndex]->size()) == static_cast<OrdinalType>(aVec.size()));
        const OrdinalType tLength = mData[aIndex]->size();
        for(OrdinalType tIndex = 0; tIndex < tLength; tIndex++)
        {
            aVec[tIndex] = mData[aIndex]->operator[](tIndex);
        }
    }

    //! Adds vector to list
    void add(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(aInput.size() > static_cast<OrdinalType>(0));
        mData.push_back(aInput.create());
    }
    //! Adds vector to list
    void add(const std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> & aInput)
    {
        assert(aInput.get() != nullptr);
        assert(aInput->size() > static_cast<OrdinalType>(0));
        mData.push_back(aInput->create());
    }
    //! Creates a copy of type MultiVector
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> create() const
    {
        const OrdinalType tVectorIndex = 0;
        const OrdinalType tNumVectors = this->getNumVectors();
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tOutput;
        const Plato::Vector<ScalarType, OrdinalType> & tVectorTemplate = *mData[tVectorIndex];
        tOutput = std::make_shared<Plato::StandardMultiVector<ScalarType, OrdinalType>>(tNumVectors, tVectorTemplate);
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
        Plato::StandardVector<ScalarType,OrdinalType> tVector(aVectorLength);

        OrdinalType tNumVectors = mData.size();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            mData[tIndex] = tVector.create();
            mData[tIndex]->fill(aBaseValue);
        }
    }
    void initialize(const std::vector<ScalarType> & aVectorTemplate)
    {
        Plato::StandardVector<ScalarType,OrdinalType> tVector(aVectorTemplate);

        OrdinalType tNumVectors = mData.size();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            mData[tIndex] = tVector.create();
        }
    }
    void initialize(const Plato::Vector<ScalarType, OrdinalType> & aVectorTemplate)
    {
        OrdinalType tNumVectors = mData.size();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            mData[tIndex] = aVectorTemplate.create();
        }
    }
    void initialize(const std::vector<std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>>> & aMultiVectorTemplate)
    {
        assert(static_cast<OrdinalType>(mData.size()) > static_cast<OrdinalType>(0));
        assert(static_cast<OrdinalType>(aMultiVectorTemplate.size()) > static_cast<OrdinalType>(0));
        OrdinalType tNumVectors = aMultiVectorTemplate.size();
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; tIndex++)
        {
            assert(aMultiVectorTemplate[tIndex]->size() > static_cast<OrdinalType>(0));
            mData[tIndex] = aMultiVectorTemplate[tIndex]->create();
            mData[tIndex]->update(static_cast<ScalarType>(1.), *aMultiVectorTemplate[tIndex], static_cast<ScalarType>(0.));
        }
    }

private:
    std::vector<std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>>> mData;

private:
    StandardMultiVector(const Plato::StandardMultiVector<ScalarType, OrdinalType>&);
    Plato::StandardMultiVector<ScalarType, OrdinalType> & operator=(const Plato::StandardMultiVector<ScalarType, OrdinalType>&);
};

}

#endif /* PLATO_STANDARDMULTIVECTOR_HPP_ */
