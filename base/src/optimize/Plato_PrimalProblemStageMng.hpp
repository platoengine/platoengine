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
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aDataFactory vector and multi-vector factory
     * @param [in] aObjective objective function interface
     * @param [in] aConstraints inequality constraint interface
    **********************************************************************************/
    PrimalProblemStageMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory,
                          const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aObjective,
                          const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aConstraints) :
            mNumObjFunEval(0),
            mNumObjGradEval(0),
            mNumConstraintEval(std::vector<OrdinalType>(aConstraints->size())),
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

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~PrimalProblemStageMng()
    {
    }

    /******************************************************************************//**
     * @brief Return number of objective function evaluations
     * @return number of objective function evaluations
    **********************************************************************************/
    OrdinalType getNumObjectiveFunctionEvaluations() const
    {
        return (mNumObjFunEval);
    }

    /******************************************************************************//**
     * @brief Return number of objective gradient evaluations
     * @return number of objective gradient evaluations
    **********************************************************************************/
    OrdinalType getNumObjectiveGradientEvaluations() const
    {
        return (mNumObjGradEval);
    }

    /******************************************************************************//**
     * @brief Return number of constraint evaluations
     * @param [in] aIndex constraint index
     * @return number of constraint evaluations
    **********************************************************************************/
    OrdinalType getNumConstraintEvaluations(const OrdinalType & aIndex) const
    {
        assert(mNumConstraintEval.empty() == false);
        assert(aIndex < mNumConstraintEval.size());
        return (mNumConstraintEval[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return number of constraint gradient evaluations
     * @param [in] aIndex constraint index
     * @return number of constraint gradient evaluations
    **********************************************************************************/
    OrdinalType getNumConstraintGradientEvaluations(const OrdinalType & aIndex) const
    {
        assert(mNumConstraintGradientEvaluations.empty() == false);
        assert(aIndex < mNumConstraintGradientEvaluations.size());
        return (mNumConstraintGradientEvaluations[aIndex]);
    }

    /******************************************************************************//**
     * @brief Set method used to compute objective function gradient (e.g. analytical, finite difference).
     * @param [in] aInput method used to compute the objective gradient
    **********************************************************************************/
    void setObjectiveGradient(const Plato::GradientOperator<ScalarType, OrdinalType> & aInput)
    {
        mObjectiveGradient = aInput.create();
    }

    /******************************************************************************//**
     * @brief Set method used to compute constraint gradients (e.g. analytical, finite difference).
     * @param [in] aInput method used to compute the constraint gradients
    **********************************************************************************/
    void setConstraintGradients(const Plato::GradientOperatorList<ScalarType, OrdinalType> & aInput)
    {
        mConstraintGradients = aInput.create();
    }

    /******************************************************************************//**
     * @brief Cache any criteria specific data once the trial control is accepted.
    **********************************************************************************/
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

    /******************************************************************************//**
     * @brief Update primal data, e.g. objective and constraint values plus gradients.
     * @param aDataMng CCSA algorithm data manager
    **********************************************************************************/
    void update(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        aDataMng.setNumObjectiveFunctionEvaluations(mNumObjFunEval);
        aDataMng.setNumObjectiveGradientEvaluations(mNumObjGradEval);

        mStateData->setCurrentTrialStep(aDataMng.getTrialStep());
        mStateData->setCurrentControl(aDataMng.getCurrentControl());
        mStateData->setCurrentObjectiveGradient(aDataMng.getCurrentObjectiveGradient());
        mStateData->setCurrentObjectiveFunctionValue(aDataMng.getCurrentObjectiveFunctionValue());

        mObjectiveGradient->update(mStateData.operator*());

        const OrdinalType tNumConstraints = aDataMng.getNumConstraints();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintGradient =
                    aDataMng.getCurrentConstraintGradients(tConstraintIndex);
            mStateData->setCurrentConstraintGradient(tMyConstraintGradient);
            mConstraintGradients->operator[](tConstraintIndex).update(mStateData.operator*());
        }
    }

    /******************************************************************************//**
     * @brief Evaluate objective function
     * @param [in] aControl current controls (i.e. optimization variables)
     * @return objective function value
    **********************************************************************************/
    ScalarType evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        assert(mObjective.get() != nullptr);

        ScalarType tObjectiveFunctionValue = mObjective->value(aControl);
        mNumObjFunEval++;

        return (tObjectiveFunctionValue);
    }

    /******************************************************************************//**
     * @brief Compute objective function gradient
     * @param [in] aControl current controls (i.e. optimization variables)
     * @param [in,out] aOutput gradient
    **********************************************************************************/
    void computeGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                         Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mObjectiveGradient.get() != nullptr);
        Plato::fill(static_cast<ScalarType>(0), aOutput);
        mObjectiveGradient->compute(aControl, aOutput);
        mNumObjGradEval++;
    }

    /******************************************************************************//**
     * @brief Evaluate each constraint defined for the optimization problem
     * @param [in] aControl current controls (i.e. optimization variables)
     * @param [in,out] aOutput constraint values
    **********************************************************************************/
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
                mNumConstraintEval[tConstraintIndex] =
                        mNumConstraintEval[tConstraintIndex] + static_cast<OrdinalType>(1);
            }
        }
    }

    /******************************************************************************//**
     * @brief Evaluate gradient for each constraint defined for the optimization problem
     * @param [in] aControl current controls (i.e. optimization variables)
     * @param [in,out] aOutput constraint gradients
    **********************************************************************************/
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
    OrdinalType mNumObjFunEval; /*!< number of objective function evaluations */
    OrdinalType mNumObjGradEval; /*!< number of objective gradient evaluations */

    std::vector<OrdinalType> mNumConstraintEval; /*!< number of constraint evaluations */
    std::vector<OrdinalType> mNumConstraintGradientEvaluations; /*!< number of constraint gradient evaluations */

    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mObjective; /*!< interface to objective function */
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> mConstraints; /*!< interface to constraints */
    std::shared_ptr<Plato::GradientOperator<ScalarType, OrdinalType>> mObjectiveGradient; /*!< interface to objective gradient computation approach */
    std::shared_ptr<Plato::GradientOperatorList<ScalarType, OrdinalType>> mConstraintGradients; /*!< interface to constraint gradient computation approach */
    std::shared_ptr<Plato::StateData<ScalarType, OrdinalType>> mStateData; /*!< primal data structure, contains current optimization state data */

private:
    PrimalProblemStageMng(const Plato::PrimalProblemStageMng<ScalarType, OrdinalType> & aRhs);
    Plato::PrimalProblemStageMng<ScalarType, OrdinalType> & operator=(const Plato::PrimalProblemStageMng<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_PRIMALPROBLEMSTAGEMNG_HPP_ */
