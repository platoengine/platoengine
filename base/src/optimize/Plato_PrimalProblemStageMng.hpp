/*
 * Plato_PrimalProblemStageMng.hpp
 *
 *  Created on: Nov 4, 2017
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

#ifndef PLATO_PRIMALPROBLEMSTAGEMNG_HPP_
#define PLATO_PRIMALPROBLEMSTAGEMNG_HPP_

#include <memory>
#include <vector>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_StateData.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_MultiVectorList.hpp"
#include "Plato_AnalyticalGradient.hpp"
#include "Plato_GradientOperatorList.hpp"
#include "Plato_ConservativeConvexSeparableAppxDataMng.hpp"
#include "Plato_ConservativeConvexSeparableAppxStageMng.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class PrimalProblemStageMng : public Plato::ConservativeConvexSeparableAppxStageMng<ScalarType, OrdinalType>
{
public:
    PrimalProblemStageMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory,
                          const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aObjective,
                          const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aConstraints) :
            mNumObjectiveFunctionEvaluations(0),
            mNumObjectiveGradientEvaluations(0),
            mNumConstraintEvaluations(std::vector<OrdinalType>(aConstraints->size())),
            mNumConstraintGradientEvaluations(std::vector<OrdinalType>(aConstraints->size())),
            mObjective(aObjective),
            mConstraints(aConstraints),
            mObjectiveGradient(),
            mConstraintGradients(),
            mStateData(std::make_shared<Plato::StateData<ScalarType, OrdinalType>>(aDataFactory.operator*()))
    {
        mObjectiveGradient = std::make_shared<Plato::AnalyticalGradient<ScalarType, OrdinalType>>(mObjective);
        mConstraintGradients = std::make_shared<Plato::GradientOperatorList<ScalarType, OrdinalType>>(mConstraints);
    }
    virtual ~PrimalProblemStageMng()
    {
    }

    OrdinalType getNumObjectiveFunctionEvaluations() const
    {
        return (mNumObjectiveFunctionEvaluations);
    }
    OrdinalType getNumObjectiveGradientEvaluations() const
    {
        return (mNumObjectiveGradientEvaluations);
    }
    OrdinalType getNumConstraintEvaluations(const OrdinalType & aIndex) const
    {
        assert(mNumConstraintEvaluations.empty() == false);
        assert(aIndex >= static_cast<OrdinalType>(0));
        assert(aIndex < mNumConstraintEvaluations.size());
        return (mNumConstraintEvaluations[aIndex]);
    }
    OrdinalType getNumConstraintGradientEvaluations(const OrdinalType & aIndex) const
    {
        assert(mNumConstraintGradientEvaluations.empty() == false);
        assert(aIndex >= static_cast<OrdinalType>(0));
        assert(aIndex < mNumConstraintGradientEvaluations.size());
        return (mNumConstraintGradientEvaluations[aIndex]);
    }

    void setObjectiveGradient(const Plato::GradientOperator<ScalarType, OrdinalType> & aInput)
    {
        mObjectiveGradient = aInput.create();
    }
    void setConstraintGradients(const Plato::GradientOperatorList<ScalarType, OrdinalType> & aInput)
    {
        mConstraintGradients = aInput.create();
    }

    //! Directive to cache any criterion specific data once the trial control is accepted.
    void cacheData()
    {
        // Communicate user that criteria specific data can be cached since trial control was accepted
        mObjective->cacheData();
        const OrdinalType tNumConstraints = mConstraints->size();
        assert(tNumConstraints == mConstraints->size());
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            (*mConstraints)[tConstraintIndex].cacheData();
        }
    }
    void update(const Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mStateData->setCurrentTrialStep(aDataMng.getTrialStep());
        mStateData->setCurrentControl(aDataMng.getCurrentControl());
        mStateData->setCurrentObjectiveGradient(aDataMng.getCurrentObjectiveGradient());
        mStateData->setCurrentObjectiveFunctionValue(aDataMng.getCurrentObjectiveFunctionValue());

        mObjectiveGradient->update(mStateData.operator*());

        const OrdinalType tNumConstraints = mConstraintGradients->size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintGradient =
                    aDataMng.getCurrentConstraintGradients(tConstraintIndex);
            mStateData->setCurrentConstraintGradient(tMyConstraintGradient);
            mConstraintGradients->operator[](tConstraintIndex).update(mStateData.operator*());
        }
    }
    ScalarType evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(mObjective.get() != nullptr);

        ScalarType tObjectiveFunctionValue = mObjective->value(aControl);
        mNumObjectiveFunctionEvaluations++;

        return (tObjectiveFunctionValue);
    }
    void computeGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                         Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mObjectiveGradient.get() != nullptr);
        Plato::fill(static_cast<ScalarType>(0), aOutput);
        mObjectiveGradient->compute(aControl, aOutput);
        mNumObjectiveGradientEvaluations++;
    }
    void evaluateConstraints(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                             Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mConstraints.get() != nullptr);

        Plato::fill(static_cast<ScalarType>(0), aOutput);
        const OrdinalType tNumVectors = aOutput.getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tMyOutput = aOutput[tVectorIndex];
            const OrdinalType tNumConstraints = tMyOutput.size();
            assert(tNumConstraints == mConstraints->size());

            for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
            {
                assert(mConstraints->ptr(tConstraintIndex).get() != nullptr);

                tMyOutput[tConstraintIndex] = mConstraints->operator[](tConstraintIndex).value(aControl);
                mNumConstraintEvaluations[tConstraintIndex] =
                        mNumConstraintEvaluations[tConstraintIndex] + static_cast<OrdinalType>(1);
            }
        }
    }
    void computeConstraintGradients(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                    Plato::MultiVectorList<ScalarType, OrdinalType> & aOutput)
    {
        assert(mConstraintGradients.get() != nullptr);
        assert(mConstraintGradients->size() == aOutput.size());

        const OrdinalType tNumConstraints = aOutput.size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            assert(mConstraints->ptr(tIndex).get() != nullptr);

            Plato::MultiVector<ScalarType, OrdinalType> & tMyOutput = aOutput[tIndex];
            Plato::fill(static_cast<ScalarType>(0), tMyOutput);
            mConstraints->operator[](tIndex).gradient(aControl, tMyOutput);
            mNumConstraintGradientEvaluations[tIndex] =
                    mNumConstraintGradientEvaluations[tIndex] + static_cast<OrdinalType>(1);
        }
    }

private:
    OrdinalType mNumObjectiveFunctionEvaluations;
    OrdinalType mNumObjectiveGradientEvaluations;

    std::vector<OrdinalType> mNumConstraintEvaluations;
    std::vector<OrdinalType> mNumConstraintGradientEvaluations;

    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mObjective;
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> mConstraints;
    std::shared_ptr<Plato::GradientOperator<ScalarType, OrdinalType>> mObjectiveGradient;
    std::shared_ptr<Plato::GradientOperatorList<ScalarType, OrdinalType>> mConstraintGradients;

    std::shared_ptr<Plato::StateData<ScalarType, OrdinalType>> mStateData;

private:
    PrimalProblemStageMng(const Plato::PrimalProblemStageMng<ScalarType, OrdinalType> & aRhs);
    Plato::PrimalProblemStageMng<ScalarType, OrdinalType> & operator=(const Plato::PrimalProblemStageMng<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_PRIMALPROBLEMSTAGEMNG_HPP_ */
