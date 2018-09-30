/*
 * Plato_OptimalityCriteriaStageMng.hpp
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
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aDataFactory data factory used to allocate Plato vectors and multi-vectors
    **********************************************************************************/
    explicit OptimalityCriteriaStageMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory) :
            mNumObjFuncEval(0),
            mNumObjGradEval(0),
            mWorkMyGradient(aDataFactory->control().create()),
            mWorkTotalGradient(aDataFactory->control().create()),
            mObjective(),
            mConstraintList()
    {
    }

    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aDataFactory data factory used to allocate Plato vectors and multi-vectors
     * @param [in] aObjective interface to objective function
     * @param [in] aInequality interface to inequality constraints
    **********************************************************************************/
    explicit OptimalityCriteriaStageMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory,
                                        const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aObjective,
                                        const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aInequality) :
            mNumObjFuncEval(0),
            mNumObjGradEval(0),
            mWorkMyGradient(aDataFactory->control().create()),
            mWorkTotalGradient(aDataFactory->control().create()),
            mObjective(aObjective),
            mConstraintList(aInequality)
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
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
        mNumObjFuncEval++;

        aDataMng.setCurrentObjectiveValue(tObjectiveValue);
        aDataMng.setNumObjectiveFunctionEvaluations(mNumObjFuncEval);

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
        mNumObjGradEval++;

        aDataMng.setObjectiveGradient(mWorkMyGradient.operator*());
        aDataMng.setNumObjectiveGradientEvaluations(mNumObjGradEval);

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
    OrdinalType mNumObjFuncEval;
    OrdinalType mNumObjGradEval;

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
