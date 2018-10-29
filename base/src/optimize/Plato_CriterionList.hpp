/*
 * Plato_CriterionList.hpp
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
//
// *************************************************************************
//@HEADER
*/

#ifndef PLATO_CRITERIONLIST_HPP_
#define PLATO_CRITERIONLIST_HPP_

#include <vector>
#include <memory>
#include <cassert>

#include "Plato_Criterion.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class CriterionList
{
public:
    CriterionList() :
            mWeights(),
            mCriterionList()
    {
    }
    ~CriterionList()
    {
    }

    OrdinalType size() const
    {
        return (mCriterionList.size());
    }
    void add(const std::vector<std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>>> & aCriteria)
    {
        assert(aCriteria.empty() == false);
        const OrdinalType tNumCriteria = aCriteria.size();
        for(OrdinalType tIndex = 0; tIndex < tNumCriteria; tIndex++)
        {
            assert(aCriteria[tIndex] != nullptr);
            mCriterionList.push_back(aCriteria[tIndex]);
            mWeights.push_back(static_cast<ScalarType>(1));
        }
    }
    void add(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aCriterion, ScalarType aMyWeight = 1)
    {
        assert(aCriterion != nullptr);
        mCriterionList.push_back(aCriterion);
        mWeights.push_back(aMyWeight);
    }
    void add(const std::vector<std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>>> & aCriteria, const std::vector<ScalarType> & aWeights)
    {
        assert(aCriteria.empty() == false);
        const OrdinalType tNumCriteria = aCriteria.size();
        for(OrdinalType tIndex = 0; tIndex < tNumCriteria; tIndex++)
        {
            assert(aCriteria[tIndex] != nullptr);
            mCriterionList.push_back(aCriteria[tIndex]);
        }
        assert(aWeights.empty() == false);
        assert(aWeights.size() == aCriteria.size());
        mWeights = aWeights;
    }
    ScalarType weight(const OrdinalType & aIndex) const
    {
        assert(mWeights.empty() == false);
        assert(aIndex < mWeights.size());
        return (mWeights[aIndex]);
    }
    Plato::Criterion<ScalarType, OrdinalType> & operator [](const OrdinalType & aIndex)
    {
        assert(aIndex < mCriterionList.size());
        assert(mCriterionList[aIndex].get() != nullptr);
        return (mCriterionList[aIndex].operator*());
    }
    const Plato::Criterion<ScalarType, OrdinalType> & operator [](const OrdinalType & aIndex) const
    {
        assert(aIndex < mCriterionList.size());
        assert(mCriterionList[aIndex].get() != nullptr);
        return (mCriterionList[aIndex].operator*());
    }
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> create() const
    {
        assert(this->size() > static_cast<OrdinalType>(0));
        std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> tOutput =
                std::make_shared<Plato::CriterionList<ScalarType, OrdinalType>>();
        const OrdinalType tNumCriterion = this->size();
        for(OrdinalType tIndex = 0; tIndex < tNumCriterion; tIndex++)
        {
            assert(mCriterionList[tIndex].get() != nullptr);
            const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & tCriterion = mCriterionList[tIndex];
            ScalarType tMyWeight = mWeights[tIndex];
            tOutput->add(tCriterion, tMyWeight);
        }
        return (tOutput);
    }
    const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & ptr(const OrdinalType & aIndex) const
    {
        assert(aIndex < mCriterionList.size());
        assert(mCriterionList[aIndex].get() != nullptr);
        return(mCriterionList[aIndex]);
    }

private:
    std::vector<ScalarType> mWeights;
    std::vector<std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>>> mCriterionList;

private:
    CriterionList(const Plato::CriterionList<ScalarType, OrdinalType>&);
    Plato::CriterionList<ScalarType, OrdinalType> & operator=(const Plato::CriterionList<ScalarType, OrdinalType>&);
};

}

#endif /* PLATO_CRITERIONLIST_HPP_ */
