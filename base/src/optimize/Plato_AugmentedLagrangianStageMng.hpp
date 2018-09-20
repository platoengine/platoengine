/*
 * Plato_AugmentedLagrangianStageMng.hpp
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

#ifndef PLATO_AUGMENTEDLAGRANGIANSTAGEMNG_HPP_
#define PLATO_AUGMENTEDLAGRANGIANSTAGEMNG_HPP_

#include <limits>
#include <vector>
#include <memory>

#include "Plato_Vector.hpp"
#include "Plato_StateData.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_Preconditioner.hpp"
#include "Plato_MultiVectorList.hpp"
#include "Plato_IdentityHessian.hpp"
#include "Plato_AnalyticalHessian.hpp"
#include "Plato_AnalyticalGradient.hpp"
#include "Plato_LinearOperatorList.hpp"
#include "Plato_ReductionOperations.hpp"
#include "Plato_TrustRegionStageMng.hpp"
#include "Plato_GradientOperatorList.hpp"
#include "Plato_IdentityPreconditioner.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class AugmentedLagrangianStageMng : public Plato::TrustRegionStageMng<ScalarType, OrdinalType>
{
public:
    /****************************************************************************************************************/
    AugmentedLagrangianStageMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aObjective,
                                const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aConstraints) :
            mNumObjectiveFunctionEvaluations(0),
            mNumObjectiveGradientEvaluations(0),
            mNumObjectiveHessianEvaluations(0),
            mTrialObjectiveValue(std::numeric_limits<ScalarType>::max()),
            mCurrentObjectiveValue(std::numeric_limits<ScalarType>::max()),
            mPreviousObjectiveValue(std::numeric_limits<ScalarType>::max()),
            mObjectiveStagnationMeasure(std::numeric_limits<ScalarType>::max()),
            mMinPenaltyValue(1e-10),
            mNormObjectiveGradient(std::numeric_limits<ScalarType>::max()),
            mFeasibilityStagnationMeasure(std::numeric_limits<ScalarType>::max()),
            mPenaltyParameter(0.1),
            mPenaltyParameterScaleFactor(1.2),
            mDynamicFeasibilityTolerance(1e-2),
            mInitialDynamicFeasibilityTolerance(1e-2),
            mNumConstraintEvaluations(std::vector<OrdinalType>(aConstraints->size())),
            mNumConstraintGradientEvaluations(std::vector<OrdinalType>(aConstraints->size())),
            mNumConstraintHessianEvaluations(std::vector<OrdinalType>(aConstraints->size())),
            mStateData(std::make_shared<Plato::StateData<ScalarType, OrdinalType>>(aFactory.operator*())),
            mDualWorkVec(aFactory->dual().create()),
            mControlWorkVec(aFactory->control().create()),
            mObjectiveGradient(aFactory->control().create()),
            mLagrangeMultipliers(aFactory->dual().create()),
            mTrialConstraintValues(aFactory->dual().create()),
            mCurrentConstraintValues(aFactory->dual().create()),
            mPreviousConstraintValues(aFactory->dual().create()),
            mCostraintGradients(std::make_shared<Plato::MultiVectorList<ScalarType, OrdinalType>>()),
            mObjective(aObjective),
            mConstraints(aConstraints),
            mPreconditioner(std::make_shared<Plato::IdentityPreconditioner<ScalarType, OrdinalType>>()),
            mObjectiveHessianOperator(std::make_shared<Plato::AnalyticalHessian<ScalarType, OrdinalType>>(aObjective)),
            mConstraintHessianOperators(std::make_shared<Plato::LinearOperatorList<ScalarType, OrdinalType>>(aConstraints)),
            mDualReductionOperations(aFactory->getDualReductionOperations().create()),
            mObjectiveGradientOperator(std::make_shared<Plato::AnalyticalGradient<ScalarType, OrdinalType>>(aObjective)),
            mConstraintGradientOperator(std::make_shared<Plato::GradientOperatorList<ScalarType, OrdinalType>>(aConstraints)),
            mVerbose(false)
    /****************************************************************************************************************/
    {
        this->initialize();
    }

    /****************************************************************************************************************/
    virtual ~AugmentedLagrangianStageMng()
    /****************************************************************************************************************/
    {
    }

    /****************************************************************************************************************/
    OrdinalType getNumObjectiveFunctionEvaluations() const
    /****************************************************************************************************************/
    {
        return (mNumObjectiveFunctionEvaluations);
    }

    /****************************************************************************************************************/
    OrdinalType getNumObjectiveGradientEvaluations() const
    /****************************************************************************************************************/
    {
        return (mNumObjectiveGradientEvaluations);
    }

    /****************************************************************************************************************/
    OrdinalType getNumObjectiveHessianEvaluations() const
    /****************************************************************************************************************/
    {
        return (mNumObjectiveHessianEvaluations);
    }

    /****************************************************************************************************************/
    OrdinalType getNumConstraintEvaluations(const OrdinalType & aIndex) const
    /****************************************************************************************************************/
    {
        assert(mNumConstraintEvaluations.empty() == false);
        assert(aIndex >= static_cast<OrdinalType>(0));
        assert(aIndex < mNumConstraintEvaluations.size());
        return (mNumConstraintEvaluations[aIndex]);
    }

    /****************************************************************************************************************/
    OrdinalType getNumConstraintGradientEvaluations(const OrdinalType & aIndex) const
    /****************************************************************************************************************/
    {
        assert(mNumConstraintGradientEvaluations.empty() == false);
        assert(aIndex >= static_cast<OrdinalType>(0));
        assert(aIndex < mNumConstraintGradientEvaluations.size());
        return (mNumConstraintGradientEvaluations[aIndex]);
    }

    /****************************************************************************************************************/
    OrdinalType getNumConstraintHessianEvaluations(const OrdinalType & aIndex) const
    /****************************************************************************************************************/
    {
        assert(mNumConstraintHessianEvaluations.empty() == false);
        assert(aIndex >= static_cast<OrdinalType>(0));
        assert(aIndex < mNumConstraintHessianEvaluations.size());
        return (mNumConstraintHessianEvaluations[aIndex]);
    }

    /****************************************************************************************************************/
    ScalarType getPenaltyParameter() const
    /****************************************************************************************************************/
    {
        return (mPenaltyParameter);
    }

    /****************************************************************************************************************/
    void setPenaltyParameter(const ScalarType & aInput)
    /****************************************************************************************************************/
    {
        mPenaltyParameter = aInput;
    }

    /****************************************************************************************************************/
    ScalarType getPenaltyParameterScaleFactor() const
    /****************************************************************************************************************/
    {
        return (mPenaltyParameterScaleFactor);
    }

    /****************************************************************************************************************/
    void setPenaltyParameterScaleFactor(const ScalarType & aInput)
    /****************************************************************************************************************/
    {
        mPenaltyParameterScaleFactor = aInput;
    }

    /****************************************************************************************************************/
    ScalarType getDynamicFeasibilityTolerance() const
    /****************************************************************************************************************/
    {
        return (mDynamicFeasibilityTolerance);
    }

    /****************************************************************************************************************/
    void setDynamicFeasibilityTolerance(const ScalarType & aInput)
    /****************************************************************************************************************/
    {
        mDynamicFeasibilityTolerance = aInput;
    }

    /****************************************************************************************************************/
    ScalarType getInitialDynamicFeasibilityTolerance() const
    /****************************************************************************************************************/
    {
        return (mInitialDynamicFeasibilityTolerance);
    }

    /****************************************************************************************************************/
    void setInitialDynamicFeasibilityTolerance(const ScalarType & aInput)
    /****************************************************************************************************************/
    {
        mInitialDynamicFeasibilityTolerance = aInput;
        mDynamicFeasibilityTolerance = mInitialDynamicFeasibilityTolerance;
    }

    /****************************************************************************************************************/
    ScalarType getNormObjectiveFunctionGradient() const
    /****************************************************************************************************************/
    {
        return (mNormObjectiveGradient);
    }

    /****************************************************************************************************************/
    void getLagrangeMultipliers(Plato::MultiVector<ScalarType, OrdinalType> & aInput) const
    /****************************************************************************************************************/
    {
        assert(mLagrangeMultipliers.get() != nullptr);
        Plato::update(1., *mLagrangeMultipliers, 0., aInput);
    }

    /****************************************************************************************************************/
    void setLagrangeMultipliers(const ScalarType & aValue)
    /****************************************************************************************************************/
    {
        assert(mLagrangeMultipliers.get() != nullptr);
        assert(aValue >= static_cast<ScalarType>(0));
        assert(mLagrangeMultipliers->getNumVectors() > static_cast<OrdinalType>(0));
        if(aValue < static_cast<ScalarType>(0))
        {
            std::cout << "\n\n**** ERROR IN: " << __FILE__ << ", FUNCTION:" << __PRETTY_FUNCTION__ << ", LINE: " << __LINE__
                      << ", MESSAGE: NEGATIVE INPUT LAGRANGE MULTIPLIER. ABORT. ****\n\n";
            std::abort();
        }
        OrdinalType tNumVectors = mLagrangeMultipliers->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            mLagrangeMultipliers->operator [](tVectorIndex).fill(aValue);
        }
    }

    /****************************************************************************************************************/
    void setLagrangeMultipliers(const OrdinalType & aVectorIndex, const OrdinalType & aConstraintIndex, const ScalarType & aValue)
    /****************************************************************************************************************/
    {
        assert(mLagrangeMultipliers.get() != nullptr);
        assert(aValue >= static_cast<ScalarType>(0));
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aConstraintIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mLagrangeMultipliers->getNumVectors());
        assert(mLagrangeMultipliers->operator[](aVectorIndex).size() >= static_cast<OrdinalType>(0));
        assert(aConstraintIndex < mLagrangeMultipliers->operator[](aVectorIndex).size());
        mLagrangeMultipliers->operator ()(aVectorIndex, aConstraintIndex).fill(aValue);
        if(aValue < static_cast<ScalarType>(0))
        {
            std::cout << "\n\n**** ERROR IN: " << __FILE__ << ", FUNCTION:" << __PRETTY_FUNCTION__ << ", LINE: " << __LINE__
                      << ", MESSAGE: NEGATIVE INPUT LAGRANGE MULTIPLIER. ABORT. ****\n\n";
            std::abort();
        }
    }

    /****************************************************************************************************************/
    void setLagrangeMultipliers(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    /****************************************************************************************************************/
    {
        assert(mLagrangeMultipliers.get() != nullptr);
        assert(aVectorIndex >= static_cast<OrdinalType>(0));
        assert(aVectorIndex < mLagrangeMultipliers->getNumVectors());
        assert(mLagrangeMultipliers->operator[](aVectorIndex).size() == aInput.size());
        for(OrdinalType tIndex = 0; tIndex < aInput.size(); tIndex++)
        {
            if(aInput[tIndex] < static_cast<ScalarType>(0))
            {
                std::cout << "\n\n**** ERROR IN: " << __FILE__ << ", FUNCTION:" << __PRETTY_FUNCTION__ << ", LINE: " << __LINE__
                << ", MESSAGE: NEGATIVE INPUT LAGRANGE MULTIPLIER WITH INDEX " << tIndex << " WAS DETECTED. ABORT. ****\n\n";
                std::abort();
            }
        }
        mLagrangeMultipliers->operator [](aVectorIndex).update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    /****************************************************************************************************************/
    void getCurrentConstraintValues(Plato::MultiVector<ScalarType, OrdinalType> & aInput) const
    /****************************************************************************************************************/
    {
        Plato::update(1., *mCurrentConstraintValues, 0., aInput);
    }

    /****************************************************************************************************************/
    void setObjectiveGradient(const std::shared_ptr<Plato::GradientOperator<ScalarType, OrdinalType>> & aInput)
    /****************************************************************************************************************/
    {
        assert(aInput.get() != nullptr);
        mObjectiveGradientOperator = aInput;
    }

    /****************************************************************************************************************/
    void setConstraintGradients(const std::shared_ptr<Plato::GradientOperatorList<ScalarType, OrdinalType>> & aInput)
    /****************************************************************************************************************/
    {
        assert(aInput.get() != nullptr);
        mConstraintGradientOperator = aInput;
    }

    /****************************************************************************************************************/
    void setObjectiveHessian(const std::shared_ptr<Plato::LinearOperator<ScalarType, OrdinalType>> & aInput)
    /****************************************************************************************************************/
    {
        assert(aInput.get() != nullptr);
        mObjectiveHessianOperator = aInput;
    }

    /****************************************************************************************************************/
    void setConstraintHessians(const std::shared_ptr<Plato::LinearOperatorList<ScalarType, OrdinalType>> & aInput)
    /****************************************************************************************************************/
    {
        assert(aInput.get() != nullptr);
        mConstraintHessianOperators = aInput;
    }

    /****************************************************************************************************************/
    void setPreconditioner(const std::shared_ptr<Plato::Preconditioner<ScalarType, OrdinalType>> & aInput)
    /****************************************************************************************************************/
    {
        assert(aInput.get() != nullptr);
        mPreconditioner = aInput;
    }

    /****************************************************************************************************************/
    void setIdentityObjectiveHessian()
    /****************************************************************************************************************/
    {
        mObjectiveHessianOperator.reset();
        mObjectiveHessianOperator = std::make_shared<Plato::IdentityHessian<ScalarType, OrdinalType>>();
    }

    /****************************************************************************************************************/
    //! Directive to cache any criterion specific data once the trial control is accepted.
    /****************************************************************************************************************/
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

    /****************************************************************************************************************/
    void storePreviousState()
    /****************************************************************************************************************/
    {
        mPreviousObjectiveValue = mCurrentObjectiveValue;
    }

    /****************************************************************************************************************/
    void updateOptimizationData(Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    /****************************************************************************************************************/
    {
        // compute objective and constraint stagnation measures
        mObjectiveStagnationMeasure = std::abs(mCurrentObjectiveValue - mPreviousObjectiveValue);
        this->computeFeasibiltiyStagnation();

        mStateData->setCurrentTrialStep(aDataMng.getTrialStep());
        mStateData->setCurrentControl(aDataMng.getCurrentControl());
        mStateData->setCurrentObjectiveGradient(aDataMng.getCurrentGradient());
        mStateData->setCurrentObjectiveFunctionValue(aDataMng.getCurrentObjectiveFunctionValue());

        mObjectiveGradientOperator->update(mStateData.operator*());
        mObjectiveHessianOperator->update(mStateData.operator*());
        mPreconditioner->update(mStateData.operator*());

        const OrdinalType tNumConstraints = mConstraints->size();
        assert(tNumConstraints == mCostraintGradients->size());
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            const Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintGradient =
                    mCostraintGradients->operator[](tConstraintIndex);
            mStateData->setCurrentConstraintGradient(tMyConstraintGradient);
            mConstraintGradientOperator->operator[](tConstraintIndex).update(mStateData.operator*());
            mConstraintHessianOperators->operator[](tConstraintIndex).update(mStateData.operator*());
        }
    }

    /****************************************************************************************************************/
    ScalarType evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                  ScalarType aTolerance = std::numeric_limits<ScalarType>::max())
    /****************************************************************************************************************/
    {
        // Evaluate objective function, f(\mathbf{z})
        mTrialObjectiveValue = mObjective->value(aControl);
        this->increaseObjectiveFunctionEvaluationCounter();

        // Evaluate inequality constraints, h(\mathbf{u}(\mathbf{z}),\mathbf{z})
        this->evaluateConstraint(aControl);

        // Evaluate Lagrangian functional, \ell(\mathbf{u}(\mathbf{z}),\mathbf{z},\mu) =
        //   f(\mathbf{u}(\mathbf{z}),\mathbf{z}) + \mu^{T}h(\mathbf{u}(\mathbf{z}),\mathbf{z})
        ScalarType tLagrangeMultipliersDotInequalityValue =
                Plato::dot(*mLagrangeMultipliers, *mTrialConstraintValues);
        ScalarType tLagrangianValue = mTrialObjectiveValue + tLagrangeMultipliersDotInequalityValue;

        // Evaluate augmented Lagrangian functional, \mathcal{L}(\mathbf{z}),\mathbf{z},\mu) =
        //   \ell(\mathbf{u}(\mathbf{z}),\mathbf{z},\mu) +
        //   \frac{1}{2\beta}(h(\mathbf{u}(\mathbf{z}),\mathbf{z})^{T}h(\mathbf{u}(\mathbf{z}),\mathbf{z})),
        //   where \beta\in\mathbb{R} denotes a penalty parameter
        ScalarType tInequalityValueDotInequalityValue = Plato::dot(*mTrialConstraintValues, *mTrialConstraintValues);
        ScalarType tAugmentedLagrangianValue = tLagrangianValue
                + ((static_cast<ScalarType>(0.5) / mPenaltyParameter) * tInequalityValueDotInequalityValue);

        return (tAugmentedLagrangianValue);
    }

    /****************************************************************************************************************/
    void computeGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                         Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    /****************************************************************************************************************/
    {
        assert(mObjectiveGradientOperator.get() != nullptr);

        // Compute objective function gradient: \frac{\partial f}{\partial\mathbf{z}}
        Plato::fill(static_cast<ScalarType>(0), *mObjectiveGradient);
        mObjectiveGradientOperator->compute(aControl, *mObjectiveGradient);
        mNormObjectiveGradient = Plato::norm(*mObjectiveGradient);
        this->increaseObjectiveGradientEvaluationCounter();

        // Compute inequality constraint gradient: \frac{\partial h_i}{\partial\mathbf{z}}
        const ScalarType tOneOverPenalty = static_cast<ScalarType>(1.) / mPenaltyParameter;
        const OrdinalType tNumConstraintVectors = mCurrentConstraintValues->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumConstraintVectors; tVectorIndex++)
        {
            const OrdinalType tNumConstraints = (*mCurrentConstraintValues)[tVectorIndex].size();
            for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
            {
                assert(mConstraintGradientOperator.get() != nullptr);
                assert(mConstraintGradientOperator->ptr(tConstraintIndex).get() != nullptr);

                // Add contribution from: \lambda_i\frac{\partial h_i}{\partial\mathbf{z}} to Lagrangian gradient
                Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintGradient = mCostraintGradients->operator[](tVectorIndex);
                Plato::fill(static_cast<ScalarType>(0), tMyConstraintGradient);
                mConstraintGradientOperator->operator[](tConstraintIndex).compute(aControl, tMyConstraintGradient);
                this->increaseConstraintGradientEvaluationCounter(tConstraintIndex);
                const ScalarType tLagrangeMultiplier = mLagrangeMultipliers->operator()(tVectorIndex, tConstraintIndex);
                Plato::update(tLagrangeMultiplier, tMyConstraintGradient, static_cast<ScalarType>(0), aOutput);

                // Add contribution from \mu*h_i(\mathbf{u}(\mathbf{z}),\mathbf{z})\frac{\partial h_i}{\partial\mathbf{z}}.
                ScalarType tAlpha = tOneOverPenalty * mCurrentConstraintValues->operator()(tVectorIndex, tConstraintIndex);
                Plato::update(tAlpha, tMyConstraintGradient, static_cast<ScalarType>(1), aOutput);
            }
        }
        // Compute Augmented Lagrangian gradient
        Plato::update(static_cast<ScalarType>(1), *mObjectiveGradient, static_cast<ScalarType>(1), aOutput);
    }

    /****************************************************************************************************************/
    /*! Reduced space interface: Assemble the reduced space gradient operator. \n
        In: \n
            aControl = design variable vector, unchanged on exist. \n
            aVector = trial descent direction, unchanged on exist. \n
        Out: \n
            aOutput = application of the trial descent direction to the Hessian operator.
    */
    void applyVectorToHessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                              const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                              Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    /****************************************************************************************************************/
    {
        assert(mObjectiveHessianOperator.get() != nullptr);
        Plato::fill(static_cast<ScalarType>(0), aOutput);
        mObjectiveHessianOperator->apply(aControl, aVector, aOutput);
        this->increaseObjectiveHessianEvaluationCounter();

        // Apply vector to inequality constraint Hessian operator and add contribution to total Hessian
        const ScalarType tOneOverPenalty = static_cast<ScalarType>(1.) / mPenaltyParameter;
        const OrdinalType tNumConstraintVectors = mCurrentConstraintValues->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumConstraintVectors; tVectorIndex++)
        {
            const OrdinalType tNumConstraints = (*mCurrentConstraintValues)[tVectorIndex].size();
            const Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintGradient = (*mCostraintGradients)[tVectorIndex];
            for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
            {
                assert(mConstraintHessianOperators.get() != nullptr);
                assert(mConstraintHessianOperators->ptr(tConstraintIndex).get() != nullptr);
                // Add contribution from: \lambda_i\frac{\partial^2 h_i}{\partial\mathbf{z}^2}
                Plato::fill(static_cast<ScalarType>(0), *mControlWorkVec);
                (*mConstraintHessianOperators)[tConstraintIndex].apply(aControl, aVector, *mControlWorkVec);
                this->increaseConstraintHessianEvaluationCounter(tConstraintIndex);
                Plato::update((*mLagrangeMultipliers)(tVectorIndex, tConstraintIndex),
                              *mControlWorkVec,
                              static_cast<ScalarType>(1),
                              aOutput);

                // Add contribution from: \mu\frac{\partial^2 h_i}{\partial\mathbf{z}^2}\h_i(\mathbf{z})
                ScalarType tAlpha = tOneOverPenalty * (*mCurrentConstraintValues)(tVectorIndex, tConstraintIndex);
                Plato::update(tAlpha, *mControlWorkVec, static_cast<ScalarType>(1), aOutput);

                // Compute Jacobian, i.e. \frac{\partial h_i}{\partial\mathbf{z}}
                ScalarType tJacobianDotTrialDirection = Plato::dot(tMyConstraintGradient, aVector);
                ScalarType tBeta = tOneOverPenalty * tJacobianDotTrialDirection;
                // Add contribution from: \mu\left(\frac{\partial h_i}{\partial\mathbf{z}}^{T}
                //                        \frac{\partial h_i}{\partial\mathbf{z}}\right)
                Plato::update(tBeta, tMyConstraintGradient, static_cast<ScalarType>(1), aOutput);
            }
        }
    }

    /****************************************************************************************************************/
    void applyVectorToPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                     const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                     Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    /****************************************************************************************************************/
    {
        assert(mPreconditioner.get() != nullptr);
        mPreconditioner->applyPreconditioner(aControl, aVector, aOutput);
    }
    void applyVectorToInvPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                        const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                        Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mPreconditioner.get() != nullptr);
        mPreconditioner->applyInvPreconditioner(aControl, aVector, aOutput);
    }

    void evaluateConstraint(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        // Evaluate inequality constraints, h(\mathbf{u}(\mathbf{z}),\mathbf{z})
        const OrdinalType tNumConstraintVectors = mTrialConstraintValues->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumConstraintVectors; tVectorIndex++)
        {
            const OrdinalType tNumConstraints = (*mTrialConstraintValues)[tVectorIndex].size();
            for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
            {
                (*mTrialConstraintValues)(tVectorIndex, tConstraintIndex) = (*mConstraints)[tConstraintIndex].value(aControl);
                this->increaseConstraintEvaluationCounter(tConstraintIndex);
            }
        }
    }

    /****************************************************************************************************************/
    void updateLagrangeMultipliers()
    /****************************************************************************************************************/
    {
        const ScalarType tPenalty = static_cast<ScalarType>(1) / mPenaltyParameter;
        const OrdinalType tNumConstraintVectors = mCurrentConstraintValues->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumConstraintVectors; tVectorIndex++)
        {
            const OrdinalType tNumConstraints = (*mCurrentConstraintValues)[tVectorIndex].size();
            Plato::Vector<ScalarType, OrdinalType> & tMyLambda = (*mLagrangeMultipliers)[tVectorIndex];
            for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
            {
                ScalarType tBeta = tPenalty * (*mCurrentConstraintValues)(tVectorIndex, tConstraintIndex);
                tMyLambda[tConstraintIndex] = tMyLambda[tConstraintIndex] + tBeta;
            }
        }
        mDynamicFeasibilityTolerance = static_cast<ScalarType>(1) / (static_cast<ScalarType>(1) + tPenalty);
    }

    /****************************************************************************************************************/
    void updatePenaltyParameter()
    /****************************************************************************************************************/
    {
        mPenaltyParameter = mPenaltyParameter / mPenaltyParameterScaleFactor;
        const ScalarType tPenalty = static_cast<ScalarType>(1) / mPenaltyParameter;
        mDynamicFeasibilityTolerance = mInitialDynamicFeasibilityTolerance / (static_cast<ScalarType>(1) + tPenalty);
    }

    /****************************************************************************************************************/
    void cacheCurrentCriteriaValues()
    /****************************************************************************************************************/
    {
        mPreviousObjectiveValue = mCurrentObjectiveValue;
        Plato::update(static_cast<ScalarType>(1), *mCurrentConstraintValues, static_cast<ScalarType>(0), *mPreviousConstraintValues);
    }

    /****************************************************************************************************************/
    void updateCurrentCriteriaValues()
    /****************************************************************************************************************/
    {
        mCurrentObjectiveValue = mTrialObjectiveValue;
        Plato::update(static_cast<ScalarType>(1), *mTrialConstraintValues, static_cast<ScalarType>(0), *mCurrentConstraintValues);
    }

    /****************************************************************************************************************/
    ScalarType getStagnationMeasure() const
    /****************************************************************************************************************/
    {
        const ScalarType tOutput = mObjectiveStagnationMeasure + mFeasibilityStagnationMeasure;
        return (tOutput);
    }

    /****************************************************************************************************************/
    void computeFeasibiltiyStagnation()
    /****************************************************************************************************************/
    {
        Plato::update(static_cast<ScalarType>(1), *mCurrentConstraintValues, static_cast<ScalarType>(0), *mDualWorkVec);
        Plato::update(static_cast<ScalarType>(-1), *mPreviousConstraintValues, static_cast<ScalarType>(1), *mDualWorkVec);
        const OrdinalType tNumVectors = mDualWorkVec->getNumVectors();
        std::vector<ScalarType> tSum(tNumVectors);
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            Plato::Vector<ScalarType, OrdinalType> & tMyVector = (*mDualWorkVec)[tVectorIndex];
            tMyVector.modulus();
            tSum[tVectorIndex] = mDualReductionOperations->sum(tMyVector);
        }
        const ScalarType tBaseValue = 0;
        mFeasibilityStagnationMeasure = std::accumulate(tSum.begin(), tSum.end(), tBaseValue);
    }

private:
    /****************************************************************************************************************/
    void initialize()
    /****************************************************************************************************************/
    {
        const OrdinalType tVECTOR_INDEX = 0;
        const OrdinalType tNumConstraints = mCurrentConstraintValues->operator[](tVECTOR_INDEX).size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            mCostraintGradients->add(mControlWorkVec.operator*());
        }

        const ScalarType tInitialLagrangeMultipliers = 1;
        Plato::fill(tInitialLagrangeMultipliers, *mLagrangeMultipliers);
    }

    /****************************************************************************************************************/
    void increaseObjectiveFunctionEvaluationCounter()
    /****************************************************************************************************************/
    {
        mNumObjectiveFunctionEvaluations++;
    }

    /****************************************************************************************************************/
    void increaseObjectiveGradientEvaluationCounter()
    /****************************************************************************************************************/
    {
        mNumObjectiveGradientEvaluations++;
    }

    /****************************************************************************************************************/
    void increaseObjectiveHessianEvaluationCounter()
    /****************************************************************************************************************/
    {
        mNumObjectiveHessianEvaluations++;
    }

    /****************************************************************************************************************/
    void increaseConstraintEvaluationCounter(const OrdinalType & aIndex)
    /****************************************************************************************************************/
    {
        mNumConstraintEvaluations[aIndex] = mNumConstraintEvaluations[aIndex] + static_cast<OrdinalType>(1);
    }

    /****************************************************************************************************************/
    void increaseConstraintGradientEvaluationCounter(const OrdinalType & aIndex)
    /****************************************************************************************************************/
    {
        mNumConstraintGradientEvaluations[aIndex] = mNumConstraintGradientEvaluations[aIndex] + static_cast<OrdinalType>(1);
    }

    /****************************************************************************************************************/
    void increaseConstraintHessianEvaluationCounter(const OrdinalType & aIndex)
    /****************************************************************************************************************/
    {
        mNumConstraintHessianEvaluations[aIndex] = mNumConstraintHessianEvaluations[aIndex] + static_cast<OrdinalType>(1);
    }

private:
    OrdinalType mNumObjectiveFunctionEvaluations;
    OrdinalType mNumObjectiveGradientEvaluations;
    OrdinalType mNumObjectiveHessianEvaluations;

    ScalarType mTrialObjectiveValue;
    ScalarType mCurrentObjectiveValue;
    ScalarType mPreviousObjectiveValue;
    ScalarType mObjectiveStagnationMeasure;

    ScalarType mMinPenaltyValue;
    ScalarType mNormObjectiveGradient;
    ScalarType mFeasibilityStagnationMeasure;
    ScalarType mPenaltyParameter;
    ScalarType mPenaltyParameterScaleFactor;
    ScalarType mDynamicFeasibilityTolerance;
    ScalarType mInitialDynamicFeasibilityTolerance;

    std::vector<OrdinalType> mNumConstraintEvaluations;
    std::vector<OrdinalType> mNumConstraintGradientEvaluations;
    std::vector<OrdinalType> mNumConstraintHessianEvaluations;

    std::shared_ptr<Plato::StateData<ScalarType, OrdinalType>> mStateData;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDualWorkVec;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlWorkVec;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mObjectiveGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLagrangeMultipliers;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialConstraintValues;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentConstraintValues;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousConstraintValues;

    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mCostraintGradients;

    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mObjective;
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> mConstraints;

    std::shared_ptr<Plato::Preconditioner<ScalarType, OrdinalType>> mPreconditioner;
    std::shared_ptr<Plato::LinearOperator<ScalarType, OrdinalType>> mObjectiveHessianOperator;
    std::shared_ptr<Plato::LinearOperatorList<ScalarType, OrdinalType>> mConstraintHessianOperators;

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mDualReductionOperations;

    std::shared_ptr<Plato::GradientOperator<ScalarType, OrdinalType>> mObjectiveGradientOperator;
    std::shared_ptr<Plato::GradientOperatorList<ScalarType, OrdinalType>> mConstraintGradientOperator;

    bool mVerbose;

private:
    AugmentedLagrangianStageMng(const Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>&);
    Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType> & operator=(const Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>&);
};

} // namespace Plato

#endif /* PLATO_AUGMENTEDLAGRANGIANSTAGEMNG_HPP_ */
