/*
 * Plato_MultiVectorList.hpp
 *
 *  Created on: Oct 21, 2017
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

#ifndef PLATO_MULTIVECTORLIST_HPP_
#define PLATO_MULTIVECTORLIST_HPP_

#include <memory>
#include <vector>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class MultiVectorList
{
public:
    MultiVectorList() :
            mList()
    {
    }
    ~MultiVectorList()
    {
    }

    OrdinalType size() const
    {
        return (mList.size());
    }
    void add(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        mList.push_back(aInput.create());
    }
    void add(const std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> & aInput)
    {
        mList.push_back(aInput);
    }
    Plato::MultiVector<ScalarType, OrdinalType> & operator [](const OrdinalType & aInput)
    {
        assert(aInput < mList.size());
        assert(mList[aInput].get() != nullptr);
        return (mList[aInput].operator*());
    }
    const Plato::MultiVector<ScalarType, OrdinalType> & operator [](const OrdinalType & aInput) const
    {
        assert(aInput < mList.size());
        assert(mList[aInput].get() != nullptr);
        return (mList[aInput].operator*());
    }
    Plato::Vector<ScalarType, OrdinalType> & operator ()(const OrdinalType & aListIndex,
                                                         const OrdinalType & aVectorIndex)
    {
        assert(aListIndex < mList.size());
        assert(mList[aListIndex].get() != nullptr);
        return (mList[aListIndex]->operator[](aVectorIndex));
    }
    const Plato::Vector<ScalarType, OrdinalType> & operator ()(const OrdinalType & aListIndex,
                                                               const OrdinalType & aVectorIndex) const
    {
        assert(aListIndex < mList.size());
        assert(mList[aListIndex].get() != nullptr);
        return (mList[aListIndex]->operator[](aVectorIndex));
    }
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> create() const
    {
        assert(this->size() > static_cast<OrdinalType>(0));
        std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> tOutput = std::make_shared<
                Plato::MultiVectorList<ScalarType, OrdinalType>>();
        const OrdinalType tListSize = this->size();
        for(OrdinalType tIndex = 0; tIndex < tListSize; tIndex++)
        {
            assert(mList[tIndex].get() != nullptr);
            const std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> & tMultiVector = mList[tIndex];
            tOutput->add(tMultiVector);
        }
        return (tOutput);
    }
    const std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> & ptr(const OrdinalType & aInput) const
    {
        assert(aInput < mList.size());
        assert(mList[aInput].get() != nullptr);
        return (mList[aInput]);
    }

private:
    std::vector<std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>>>mList;

private:
    MultiVectorList(const Plato::MultiVectorList<ScalarType, OrdinalType>&);
    Plato::MultiVectorList<ScalarType, OrdinalType> & operator=(const Plato::MultiVectorList<ScalarType, OrdinalType>&);
};

}
 // namespace Plato

#endif /* PLATO_MULTIVECTORLIST_HPP_ */
