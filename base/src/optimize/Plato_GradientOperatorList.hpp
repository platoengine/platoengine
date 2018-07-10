/*
 * Plato_GradientOperatorList.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_GRADIENTOPERATORLIST_HPP_
#define PLATO_GRADIENTOPERATORLIST_HPP_

#include <vector>
#include <memory>
#include <cassert>

#include "Plato_CriterionList.hpp"
#include "Plato_AnalyticalGradient.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class GradientOperatorList
{
public:
    GradientOperatorList() :
            mList()
    {
    }
    explicit GradientOperatorList(const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aInput) :
            mList()
    {
        const OrdinalType tNumCriterion = aInput->size();
        for(OrdinalType tIndex = 0; tIndex < tNumCriterion; tIndex++)
        {
            mList.push_back(std::make_shared<Plato::AnalyticalGradient<ScalarType, OrdinalType>>(aInput->ptr(tIndex)));
        }
    }
    ~GradientOperatorList()
    {
    }

    OrdinalType size() const
    {
        return (mList.size());
    }
    void add(const std::shared_ptr<Plato::GradientOperator<ScalarType, OrdinalType>> & aInput)
    {
        mList.push_back(aInput);
    }
    Plato::GradientOperator<ScalarType, OrdinalType> & operator [](const OrdinalType & aIndex)
    {
        assert(aIndex < mList.size());
        assert(mList[aIndex].get() != nullptr);
        return (mList[aIndex].operator*());
    }
    const Plato::GradientOperator<ScalarType, OrdinalType> & operator [](const OrdinalType & aIndex) const
    {
        assert(aIndex < mList.size());
        assert(mList[aIndex].get() != nullptr);
        return (mList[aIndex].operator*());
    }
    std::shared_ptr<Plato::GradientOperatorList<ScalarType, OrdinalType>> create() const
    {
        assert(this->size() > static_cast<OrdinalType>(0));

        std::shared_ptr<Plato::GradientOperatorList<ScalarType, OrdinalType>> tOutput =
                std::make_shared<Plato::GradientOperatorList<ScalarType, OrdinalType>>();
        const OrdinalType tNumGradientOperators = this->size();
        for(OrdinalType tIndex = 0; tIndex < tNumGradientOperators; tIndex++)
        {
            assert(mList[tIndex].get() != nullptr);

            const std::shared_ptr<Plato::GradientOperator<ScalarType, OrdinalType>> & tGradientOperator = mList[tIndex];
            tOutput->add(tGradientOperator);
        }
        return (tOutput);
    }
    const std::shared_ptr<Plato::GradientOperator<ScalarType, OrdinalType>> & ptr(const OrdinalType & aIndex) const
    {
        assert(aIndex < mList.size());
        assert(mList[aIndex].get() != nullptr);
        return(mList[aIndex]);
    }

private:
    std::vector<std::shared_ptr<Plato::GradientOperator<ScalarType, OrdinalType>>> mList;

private:
    GradientOperatorList(const Plato::GradientOperatorList<ScalarType, OrdinalType>&);
    Plato::GradientOperatorList<ScalarType, OrdinalType> & operator=(const Plato::GradientOperatorList<ScalarType, OrdinalType>&);
};

} //namespace Plato

#endif /* PLATO_GRADIENTOPERATORLIST_HPP_ */
