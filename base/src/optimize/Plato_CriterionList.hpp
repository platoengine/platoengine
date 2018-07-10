/*
 * Plato_CriterionList.hpp
 *
 *  Created on: Oct 17, 2017
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
        assert(aIndex >= static_cast<OrdinalType>(0));
        return (mWeights[aIndex]);
    }
    Plato::Criterion<ScalarType, OrdinalType> & operator [](const OrdinalType & aIndex)
    {
        assert(aIndex < mCriterionList.size());
        assert(mCriterionList[aIndex].get() != nullptr);
        assert(aIndex >= static_cast<OrdinalType>(0));
        return (mCriterionList[aIndex].operator*());
    }
    const Plato::Criterion<ScalarType, OrdinalType> & operator [](const OrdinalType & aIndex) const
    {
        assert(aIndex < mCriterionList.size());
        assert(mCriterionList[aIndex].get() != nullptr);
        assert(aIndex >= static_cast<OrdinalType>(0));
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
