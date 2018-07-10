/*
 * Plato_LinearOperatorList.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_LINEAROPERATORLIST_HPP_
#define PLATO_LINEAROPERATORLIST_HPP_

#include <vector>
#include <memory>
#include <cassert>

#include "Plato_CriterionList.hpp"
#include "Plato_LinearOperator.hpp"
#include "Plato_IdentityHessian.hpp"
#include "Plato_AnalyticalHessian.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class LinearOperatorList
{
public:
    LinearOperatorList() :
            mList()
    {
    }
    explicit LinearOperatorList(const OrdinalType & aNumCriterion) :
            mList()
    {
        for(OrdinalType tIndex = 0; tIndex < aNumCriterion; tIndex++)
        {
            mList.push_back(std::make_shared<Plato::IdentityHessian<ScalarType, OrdinalType>>());
        }
    }
    explicit LinearOperatorList(const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aInput) :
            mList()
    {
        const OrdinalType tNumCriterion = aInput->size();
        for(OrdinalType tIndex = 0; tIndex < tNumCriterion; tIndex++)
        {
            mList.push_back(std::make_shared<Plato::AnalyticalHessian<ScalarType, OrdinalType>>(aInput->ptr(tIndex)));
        }
    }
    ~LinearOperatorList()
    {
    }

    OrdinalType size() const
    {
        return (mList.size());
    }
    void add(const std::shared_ptr<Plato::LinearOperator<ScalarType, OrdinalType>> & aInput)
    {
        mList.push_back(aInput);
    }
    Plato::LinearOperator<ScalarType, OrdinalType> & operator [](const OrdinalType & aIndex)
    {
        assert(aIndex < mList.size());
        assert(mList[aIndex].get() != nullptr);
        return (mList[aIndex].operator*());
    }
    const Plato::LinearOperator<ScalarType, OrdinalType> & operator [](const OrdinalType & aIndex) const
    {
        assert(aIndex < mList.size());
        assert(mList[aIndex].get() != nullptr);
        return (mList[aIndex].operator*());
    }
    std::shared_ptr<Plato::LinearOperatorList<ScalarType, OrdinalType>> create() const
    {
        assert(this->size() > static_cast<OrdinalType>(0));

        std::shared_ptr<Plato::LinearOperatorList<ScalarType, OrdinalType>> tOutput =
                std::make_shared<Plato::LinearOperatorList<ScalarType, OrdinalType>>();
        const OrdinalType tNumLinearOperators = this->size();
        for(OrdinalType tIndex = 0; tIndex < tNumLinearOperators; tIndex++)
        {
            assert(mList[tIndex].get() != nullptr);

            const std::shared_ptr<Plato::LinearOperator<ScalarType, OrdinalType>> & tLinearOperator = mList[tIndex];
            tOutput->add(tLinearOperator);
        }
        return (tOutput);
    }
    const std::shared_ptr<Plato::LinearOperator<ScalarType, OrdinalType>> & ptr(const OrdinalType & aIndex) const
    {
        assert(aIndex < mList.size());
        assert(mList[aIndex].get() != nullptr);
        return(mList[aIndex]);
    }

private:
    std::vector<std::shared_ptr<Plato::LinearOperator<ScalarType, OrdinalType>>> mList;

private:
    LinearOperatorList(const Plato::LinearOperatorList<ScalarType, OrdinalType>&);
    Plato::LinearOperatorList<ScalarType, OrdinalType> & operator=(const Plato::LinearOperatorList<ScalarType, OrdinalType>&);
};

}

#endif /* PLATO_LINEAROPERATORLIST_HPP_ */
