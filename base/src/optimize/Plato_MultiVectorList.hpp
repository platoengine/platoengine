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

/*
 * Plato_MultiVectorList.hpp
 *
 *  Created on: Oct 21, 2017
 */

#ifndef PLATO_MULTIVECTORLIST_HPP_
#define PLATO_MULTIVECTORLIST_HPP_

#include <memory>
#include <vector>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Interface to a list of 2D containersS
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class MultiVectorList
{
public:
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    MultiVectorList() :
            mList()
    {
    }

    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aLength maximum number of elements stored in list
     * @param [in] aMultiVec multi-vector template
    **********************************************************************************/
    MultiVectorList(const OrdinalType & aLength, const Plato::MultiVector<ScalarType, OrdinalType> & aMultiVec) :
            mList()
    {
        mList.resize(aLength);
        this->initialize(aMultiVec);
    }

    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    ~MultiVectorList()
    {
    }

    /******************************************************************************//**
     * @brief Return size of list
     * @return size
    **********************************************************************************/
    OrdinalType size() const
    {
        return (mList.size());
    }

    /******************************************************************************//**
     * @brief Appends the given element value to the end of the container
     * @param [in] aInput multi-vector
    **********************************************************************************/
    void add(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        mList.push_back(aInput.create());
    }

    /******************************************************************************//**
     * @brief Appends the given element value to the end of the container
     * @param [in] aInput multi-vector shared pointer
    **********************************************************************************/
    void add(const std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> & aInput)
    {
        mList.push_back(aInput);
    }

    /******************************************************************************//**
     * @brief Returns a reference to the multi-vector at specified location position
     * @param [in] aInput specified position
     * @return reference to a multi-vector
    **********************************************************************************/
    Plato::MultiVector<ScalarType, OrdinalType> & operator [](const OrdinalType & aInput)
    {
        assert(aInput < static_cast<OrdinalType>(mList.size()));
        assert(mList[aInput].get() != nullptr);
        return (mList[aInput].operator*());
    }

    /******************************************************************************//**
     * @brief Returns a constant reference to the multi-vector at specified location position
     * @param [in] aInput specified position
     * @return constant reference to a multi-vector
    **********************************************************************************/
    const Plato::MultiVector<ScalarType, OrdinalType> & operator [](const OrdinalType & aInput) const
    {
        assert(aInput < static_cast<OrdinalType>(mList.size()));
        assert(mList[aInput].get() != nullptr);
        return (mList[aInput].operator*());
    }

    /******************************************************************************//**
     * @brief Returns a reference to the vector at specified location position
     * @param [in] aElemIndex index to element (i.e. multi-vector) in list
     * @param [in] aVectorIndex index to element (i.e. vector) in multi-vector
     * @return reference to a vector
    **********************************************************************************/
    Plato::Vector<ScalarType, OrdinalType> & operator ()(const OrdinalType & aElemIndex,
                                                         const OrdinalType & aVectorIndex)
    {
        assert(aElemIndex < static_cast<OrdinalType>(mList.size()));
        assert(mList[aElemIndex].get() != nullptr);
        return (mList[aElemIndex]->operator[](aVectorIndex));
    }

    /******************************************************************************//**
     * @brief Returns a constant reference to the vector at specified location
     * @param [in] aElemIndex index to element (i.e. multi-vector) in list
     * @param [in] aVectorIndex index to element (i.e. vector) in multi-vector
     * @return constant reference to a vector
    **********************************************************************************/
    const Plato::Vector<ScalarType, OrdinalType> & operator ()(const OrdinalType & aElemIndex,
                                                               const OrdinalType & aVectorIndex) const
    {
        assert(aElemIndex < static_cast<OrdinalType>(mList.size()));
        assert(mList[aElemIndex].get() != nullptr);
        return (mList[aElemIndex]->operator[](aVectorIndex));
    }

    /******************************************************************************//**
     * @brief Return a shared pointer to a multi-vector list with dimensions equivalent to this.
     * @return [in] aInput shared pointer to a multi-vector list
    **********************************************************************************/
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> create() const
    {
        assert(this->size() > static_cast<OrdinalType>(0));
        std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> tOutput =
                std::make_shared<Plato::MultiVectorList<ScalarType, OrdinalType>>();
        const OrdinalType tListSize = this->size();
        for(OrdinalType tIndex = 0; tIndex < tListSize; tIndex++)
        {
            assert(mList[tIndex].get() != nullptr);
            const std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> & tMultiVector = mList[tIndex];
            tOutput->add(tMultiVector);
        }
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Return a constant shared pointer to the multi-vector at specified location
     * @param [in] aIndex specified location
     * @return constant shared pointer to the multi-vector at specified location
    **********************************************************************************/
    const std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> & ptr(const OrdinalType & aIndex) const
    {
        assert(aIndex < static_cast<OrdinalType>(mList.size()));
        assert(mList[aIndex].get() != nullptr);
        return (mList[aIndex]);
    }

private:
    /******************************************************************************//**
     * @brief Allocate multi-vector list
     * @param [in] aInput multi-vector template
    **********************************************************************************/
    void initialize(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(static_cast<OrdinalType>(mList.size()) > static_cast<OrdinalType>(0));

        const OrdinalType tSize = mList.size();
        for(OrdinalType tIndex = 0; tIndex < tSize; tIndex++)
        {
            mList[tIndex] = aInput.create();
        }
    }

private:
    std::vector<std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>>> mList; /*!< multi-vector list */

private:
    MultiVectorList(const Plato::MultiVectorList<ScalarType, OrdinalType>&);
    Plato::MultiVectorList<ScalarType, OrdinalType> & operator=(const Plato::MultiVectorList<ScalarType, OrdinalType>&);
};
// class MultiVectorList

}
 // namespace Plato

#endif /* PLATO_MULTIVECTORLIST_HPP_ */
