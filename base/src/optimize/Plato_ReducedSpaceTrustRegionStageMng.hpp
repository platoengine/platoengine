/*
 * Plato_ReducedSpaceTrustRegionStageMng.hpp
 *
 *  Created on: Dec 20, 2017
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

#ifndef PLATO_REDUCEDSPACETRUSTREGIONSTAGEMNG_HPP_
#define PLATO_REDUCEDSPACETRUSTREGIONSTAGEMNG_HPP_

#include <memory>
#include <cassert>

#include "Plato_StateData.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_LinearOperatorList.hpp"
#include "Plato_TrustRegionStageMng.hpp"
#include "Plato_GradientOperatorList.hpp"
#include "Plato_IdentityPreconditioner.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class ReducedSpaceTrustRegionStageMng : public Plato::TrustRegionStageMng<ScalarType, OrdinalType>
{
public:
    ReducedSpaceTrustRegionStageMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                    const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aObjectives) :
            mNumObjFuncEval(0),
            mNumObjGradEval(0),
            mNumObjectiveHessianEvaluations(0),
            mNumPreconditionerEvaluations(0),
            mNumInversePreconditionerEvaluations(0),
            mWorkVec(aFactory->control().create()),
            mStateData(std::make_shared<Plato::StateData<ScalarType, OrdinalType>>(*aFactory)),
            mObjectives(aObjectives),
            mPreconditioner(std::make_shared<Plato::IdentityPreconditioner<ScalarType, OrdinalType>>()),
            mObjectiveHessians(std::make_shared<Plato::LinearOperatorList<ScalarType, OrdinalType>>(aObjectives)),
            mObjectivesGradient(std::make_shared<Plato::GradientOperatorList<ScalarType, OrdinalType>>(aObjectives))
    {
    }
    virtual ~ReducedSpaceTrustRegionStageMng()
    {
    }

    OrdinalType getNumObjectiveFunctionEvaluations() const
    {
        return (mNumObjFuncEval);
    }
    OrdinalType getNumObjectiveGradientEvaluations() const
    {
        return (mNumObjGradEval);
    }
    OrdinalType getNumObjectiveHessianEvaluations() const
    {
        return (mNumObjectiveHessianEvaluations);
    }
    OrdinalType getNumPreconditionerEvaluations() const
    {
        return (mNumPreconditionerEvaluations);
    }
    OrdinalType getNumInversePreconditionerEvaluations() const
    {
        return (mNumInversePreconditionerEvaluations);
    }

    void setObjectiveGradient(const std::shared_ptr<Plato::GradientOperatorList<ScalarType, OrdinalType>> & aInput)
    {
        mObjectivesGradient = aInput;
    }
    void setObjectiveHessian(const std::shared_ptr<Plato::LinearOperatorList<ScalarType, OrdinalType>> & aInput)
    {
        mObjectiveHessians = aInput;
    }
    void setIdentityObjectiveHessian()
    {
        mObjectiveHessians.reset();
        const OrdinalType tNumObjectives = mObjectives->size();
        mObjectiveHessians = std::make_shared<Plato::LinearOperatorList<ScalarType, OrdinalType>>(tNumObjectives);
    }
    void setPreconditioner(const std::shared_ptr<Plato::Preconditioner<ScalarType, OrdinalType>> & aInput)
    {
        mPreconditioner = aInput;
    }

    //! Directive to cache any criterion specific data once the trial control is accepted.
    void cacheData()
    {
        // Communicate user that criteria specific data can be cached since trial control was accepted
        assert(mObjectives.get() != nullptr);
        const OrdinalType tNumObjectives = mObjectives->size();
        for(OrdinalType tObjectiveIndex = 0; tObjectiveIndex < tNumObjectives; tObjectiveIndex++)
        {
            (*mObjectives)[tObjectiveIndex].cacheData();
        }
    }

    //! Directive to update problem for each criterion.
    void updateProblem()
    {
        // Communicate user that criteria specific data can be cached since trial control was accepted
        assert(mObjectives.get() != nullptr);
        const OrdinalType tNumObjectives = mObjectives->size();
        for(OrdinalType tObjectiveIndex = 0; tObjectiveIndex < tNumObjectives; tObjectiveIndex++)
        {
            (*mObjectives)[tObjectiveIndex].updateProblem();
        }
    }

    /*! Directive that updates current optimization state data and notifies any active gradient and Hessian
     *  approximation methods that they need to update any method specific data since a trial control was accepted. */
    void updateOptimizationData(Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        assert(mStateData.get() != nullptr);
        assert(mObjectives.get() != nullptr);
        assert(mPreconditioner.get() != nullptr);

        mStateData->setCurrentTrialStep(aDataMng.getTrialStep());
        mStateData->setCurrentControl(aDataMng.getCurrentControl());
        mStateData->setCurrentObjectiveGradient(aDataMng.getCurrentGradient());
        mStateData->setCurrentObjectiveFunctionValue(aDataMng.getCurrentObjectiveFunctionValue());
        mPreconditioner->update(mStateData.operator*());

        const OrdinalType tNumObjectives = mObjectives->size();
        for(OrdinalType tObjectiveIndex = 0; tObjectiveIndex < tNumObjectives; tObjectiveIndex++)
        {
            mObjectivesGradient->operator[](tObjectiveIndex).update(mStateData.operator*());
            mObjectiveHessians->operator[](tObjectiveIndex).update(mStateData.operator*());
        }

        aDataMng.setNumObjectiveFunctionEvaluations(mNumObjFuncEval);
        aDataMng.setNumObjectiveGradientEvaluations(mNumObjGradEval);
    }

    /*! Evaluate objective function f_i(\mathbf{z})\ \forall\ i=1,\dots,n, where i\in\mathcal{N}
     *  and $n$ denotes the number of objective functions. */
    ScalarType evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                  ScalarType aTolerance = std::numeric_limits<ScalarType>::max())
    {
        assert(mObjectives.get() != nullptr);

        ScalarType tOutput = 0;
        const OrdinalType tNumObjectives = mObjectives->size();
        for(OrdinalType tIndex = 0; tIndex < tNumObjectives; tIndex++)
        {
            ScalarType tMyWeight = mObjectives->weight(tIndex);
            ScalarType tMyValue = tMyWeight * mObjectives->operator[](tIndex).value(aControl);
            tOutput += tMyValue;
        }
        mNumObjFuncEval++;
        return (tOutput);
    }
    /*! Compute reduced gradient, \nabla_{\mathbf{z}}{F}=\sum_{i=1}^{n}\nabla_{\mathbf{z}}f_i(\mathbf{z})\
     *  \forall\ i=1,\dots,n, where i\in\mathcal{N} and $n$ denotes the number of objective functions. */
    void computeGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                         Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mObjectives.get() != nullptr);
        assert(mObjectivesGradient.get() != nullptr);

        Plato::fill(static_cast<ScalarType>(0), aOutput);
        const OrdinalType tNumObjectives = mObjectivesGradient->size();
        for(OrdinalType tIndex = 0; tIndex < tNumObjectives; tIndex++)
        {
            Plato::fill(static_cast<ScalarType>(0), *mWorkVec);
            (*mObjectivesGradient)[tIndex].compute(aControl, *mWorkVec);
            ScalarType tMyWeight = mObjectives->weight(tIndex);
            Plato::update(tMyWeight, *mWorkVec, static_cast<ScalarType>(1), aOutput);
        }
        mNumObjGradEval++;
    }
    /*! Compute the application of a vector to the reduced Hessian. The reduced Hessian is defined as
     *  \nabla_{\mathbf{z}}^{2}{F}=\sum_{i=1}^{n}\nabla_{\mathbf{z}}^{2}f_i(\mathbf{z})\ \forall\ i=1,
     *  \dots,n, where i\in\mathcal{N} and $n$ denotes the number of objective functions. */
    void applyVectorToHessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                              const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                              Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mObjectives.get() != nullptr);
        assert(mObjectiveHessians.get() != nullptr);

        Plato::fill(static_cast<ScalarType>(0), aOutput);
        const OrdinalType tNumObjectives = mObjectiveHessians->size();
        for(OrdinalType tIndex = 0; tIndex < tNumObjectives; tIndex++)
        {
            Plato::fill(static_cast<ScalarType>(0), *mWorkVec);
            (*mObjectiveHessians)[tIndex].apply(aControl, aVector, *mWorkVec);
            ScalarType tMyWeight = mObjectives->weight(tIndex);
            Plato::update(tMyWeight, *mWorkVec, static_cast<ScalarType>(1), aOutput);
        }
        mNumObjectiveHessianEvaluations++;
    }
    /*! Compute the application of a vector to the preconditioner operator. */
    void applyVectorToPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                     const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                     Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mPreconditioner.get() != nullptr);
        mPreconditioner->applyPreconditioner(aControl, aVector, aOutput);
        mNumPreconditionerEvaluations++;
    }
    /*! Compute the application of a vector to the inverse preconditioner operator. */
    void applyVectorToInvPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                        const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                        Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mPreconditioner.get() != nullptr);
        mPreconditioner->applyInvPreconditioner(aControl, aVector, aOutput);
        mNumInversePreconditionerEvaluations++;
    }

private:
    OrdinalType mNumObjFuncEval;
    OrdinalType mNumObjGradEval;
    OrdinalType mNumObjectiveHessianEvaluations;
    OrdinalType mNumPreconditionerEvaluations;
    OrdinalType mNumInversePreconditionerEvaluations;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mWorkVec;

    std::shared_ptr<Plato::StateData<ScalarType, OrdinalType>> mStateData;
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> mObjectives;
    std::shared_ptr<Plato::Preconditioner<ScalarType, OrdinalType>> mPreconditioner;
    std::shared_ptr<Plato::LinearOperatorList<ScalarType, OrdinalType>> mObjectiveHessians;
    std::shared_ptr<Plato::GradientOperatorList<ScalarType, OrdinalType>> mObjectivesGradient;

private:
    ReducedSpaceTrustRegionStageMng(const Plato::ReducedSpaceTrustRegionStageMng<ScalarType, OrdinalType>&);
    Plato::ReducedSpaceTrustRegionStageMng<ScalarType, OrdinalType> & operator=(const Plato::ReducedSpaceTrustRegionStageMng<ScalarType, OrdinalType>&);
}; // class ReducedSpaceTrustRegionStageMng

} // namespace Plato

#endif /* PLATO_REDUCEDSPACETRUSTREGIONSTAGEMNG_HPP_ */
