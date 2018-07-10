/*
 * Plato_OptimalityCriteriaStageMng.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_OPTIMALITYCRITERIASTAGEMNG_HPP_
#define PLATO_OPTIMALITYCRITERIASTAGEMNG_HPP_

#include <memory>

#include "Plato_Criterion.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_OptimalityCriteriaDataMng.hpp"
#include "Plato_OptimalityCriteriaStageMngBase.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class OptimalityCriteriaStageMng : public Plato::OptimalityCriteriaStageMngBase<ScalarType, OrdinalType>
{
public:
    explicit OptimalityCriteriaStageMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory) :
            mWorkMyGradient(aDataFactory->control().create()),
            mWorkTotalGradient(aDataFactory->control().create()),
            mObjective(),
            mConstraintList()
    {
    }
    explicit OptimalityCriteriaStageMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory,
                                        const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aObjective,
                                        const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aInequality) :
            mWorkMyGradient(aDataFactory->control().create()),
            mWorkTotalGradient(aDataFactory->control().create()),
            mObjective(aObjective),
            mConstraintList(aInequality)
    {
    }
    virtual ~OptimalityCriteriaStageMng()
    {
    }

    //! Directive to cache any criterion specific data once the trial control is accepted.
    void cacheData()
    {
        // Communicate to performer that criteria specific data can be cached since trial control was accepted
        mObjective->cacheData();

        assert(mConstraintList.get() != nullptr);
        const OrdinalType tNumConstraints = mConstraintList->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            (*mConstraintList)[tConstraintIndex].cacheData();
        }
    }
    //! Directive to update optimization specific data once the trial control is accepted.
    void update(Plato::OptimalityCriteriaDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        // Evaluate objective function
        assert(mObjective.get() != nullptr);
        const Plato::MultiVector<ScalarType, OrdinalType> & tControl = aDataMng.getCurrentControl();
        ScalarType tObjectiveValue = mObjective->value(tControl);
        aDataMng.setCurrentObjectiveValue(tObjectiveValue);

        // Evaluate inequality constraints
        assert(mConstraintList.get() != nullptr);
        const OrdinalType tNumConstraints = mConstraintList->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            ScalarType tConstraintValue = (*mConstraintList)[tConstraintIndex].value(tControl);
            aDataMng.setCurrentConstraintValue(tConstraintIndex, tConstraintValue);
        }

        // Cache criteria specific data
        this->cacheData();

        // Compute objective gradient
        Plato::fill(static_cast<ScalarType>(0), mWorkMyGradient.operator*());
        mObjective->gradient(tControl, mWorkMyGradient.operator*());
        aDataMng.setObjectiveGradient(mWorkMyGradient.operator*());

        // Compute constraint gradients
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            Plato::fill(static_cast<ScalarType>(0), mWorkMyGradient.operator*());
            (*mConstraintList)[tConstraintIndex].gradient(tControl, mWorkMyGradient.operator*());
            Plato::update(static_cast<ScalarType>(1), *mWorkMyGradient, static_cast<ScalarType>(0), *mWorkTotalGradient);
        }
        aDataMng.setInequalityGradient(mWorkTotalGradient.operator*());
    }

private:
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mWorkMyGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mWorkTotalGradient;

    std::shared_ptr<Plato::Criterion<ScalarType,OrdinalType>> mObjective;
    std::shared_ptr<Plato::CriterionList<ScalarType,OrdinalType>> mConstraintList;

private:
    OptimalityCriteriaStageMng(const Plato::OptimalityCriteriaStageMng<ScalarType, OrdinalType>&);
    Plato::OptimalityCriteriaStageMng<ScalarType, OrdinalType> & operator=(const Plato::OptimalityCriteriaStageMng<ScalarType, OrdinalType>&);
};

}

#endif /* PLATO_OPTIMALITYCRITERIASTAGEMNG_HPP_ */
