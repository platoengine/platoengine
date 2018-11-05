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

/*
 * Plato_AugmentedLagrangianStageMng.hpp
 *
 *  Created on: Oct 21, 2017
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
    /******************************************************************************//**
     * @brief Default constructor
    **********************************************************************************/
    AugmentedLagrangianStageMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aObjective,
                                const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aConstraints) :
            mNumObjFuncEval(0),
            mNumObjGradEval(0),
            mNumObjHessEval(0),
            mTrialObjFuncValue(std::numeric_limits<ScalarType>::max()),
            mCurrentObjFuncValue(std::numeric_limits<ScalarType>::max()),
            mTrialAugLagFuncValue(std::numeric_limits<ScalarType>::max()),
            mCurrentAugLagFuncValue(std::numeric_limits<ScalarType>::max()),
            mPreviousObjectiveValue(std::numeric_limits<ScalarType>::max()),
            mPreviousAugLagFuncValue(std::numeric_limits<ScalarType>::max()),
            mObjectiveStagnationMeasure(std::numeric_limits<ScalarType>::max()),
            mMinPenaltyValue(1e-10),
            mNormConstraints(std::numeric_limits<ScalarType>::max()),
            mNormObjFuncGrad(std::numeric_limits<ScalarType>::max()),
            mNormAugLagFuncGrad(std::numeric_limits<ScalarType>::max()),
            mFeasibilityStagnationMeasure(std::numeric_limits<ScalarType>::max()),
            mPenaltyParameter(0.05),
            mPenaltyParameterScaleFactor(2),
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
            mConstraintGradientOperator(std::make_shared<Plato::GradientOperatorList<ScalarType, OrdinalType>>(aConstraints))
    /****************************************************************************************************************/
    {
        this->initialize();
    }

    /******************************************************************************//**
     * @brief Default destructor
    **********************************************************************************/
    virtual ~AugmentedLagrangianStageMng()
    {
    }

    /******************************************************************************//**
     * @brief Return number of constraints
     * @return number of constraints
    **********************************************************************************/
    OrdinalType getNumConstraints() const
    {
        const OrdinalType tVECTOR_INDEX = 0;
        const OrdinalType tNumConstraints = mCurrentConstraintValues->operator[](tVECTOR_INDEX).size();
        return (tNumConstraints);
    }

    /******************************************************************************//**
     * @brief Return number of objective function evaluations
     * @return number of objective function evaluations
    **********************************************************************************/
    OrdinalType getNumObjectiveFunctionEvaluations() const
    {
        return (mNumObjFuncEval);
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
     * @brief Return number of objective Hessian evaluations
     * @return number of objective Hessian evaluations
    **********************************************************************************/
    OrdinalType getNumObjectiveHessianEvaluations() const
    {
        return (mNumObjHessEval);
    }

    /******************************************************************************//**
     * @brief Return number of constraint evaluations
     * @param [in] aIndex constraint index
     * @return number of constraint evaluations
    **********************************************************************************/
    OrdinalType getNumConstraintEvaluations(const OrdinalType & aIndex) const
    {
        assert(mNumConstraintEvaluations.empty() == false);
        assert(aIndex < mNumConstraintEvaluations.size());
        return (mNumConstraintEvaluations[aIndex]);
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
     * @brief Return number of constraint Hessian evaluations
     * @param [in] aIndex constraint index
     * @return number of constraint Hessian evaluations
    **********************************************************************************/
    OrdinalType getNumConstraintHessianEvaluations(const OrdinalType & aIndex) const
    {
        assert(mNumConstraintHessianEvaluations.empty() == false);
        assert(aIndex < mNumConstraintHessianEvaluations.size());
        return (mNumConstraintHessianEvaluations[aIndex]);
    }

    /******************************************************************************//**
     * @brief Return penalty parameter value
     * @return penalty parameter
    **********************************************************************************/
    ScalarType getPenaltyParameter() const
    {
        return (mPenaltyParameter);
    }

    /******************************************************************************//**
     * @brief Return current objective function value
     * @return current objective function value
    **********************************************************************************/
    ScalarType getCurrentObjectiveFunctionValue() const
    {
        return (mCurrentObjFuncValue);
    }

    /******************************************************************************//**
     * @brief Return current augmented Lagrangian function value
     * @return current augmented Lagrangian function value
    **********************************************************************************/
    ScalarType getCurrentAugmenteLagrangianFunctionValue() const
    {
        return (mCurrentAugLagFuncValue);
    }

    /******************************************************************************//**
     * @brief Set penalty parameter
     * @param [in] penalty parameter
    **********************************************************************************/
    void setPenaltyParameter(const ScalarType & aInput)
    {
        mPenaltyParameter = aInput;
    }

    /******************************************************************************//**
     * @brief Return penalty parameter scale
     * @return penalty parameter scale
    **********************************************************************************/
    ScalarType getPenaltyParameterScaleFactor() const
    {
        return (mPenaltyParameterScaleFactor);
    }

    /****************************************************************************************************************/
    void setPenaltyParameterScaleFactor(const ScalarType & aInput)
    /****************************************************************************************************************/
    {
        mPenaltyParameterScaleFactor = aInput;
    }

    /******************************************************************************//**
     * @brief Return dynamic feasibility tolerance
     * @return dynamic feasibility tolerance
    **********************************************************************************/
    ScalarType getDynamicFeasibilityTolerance() const
    {
        return (mDynamicFeasibilityTolerance);
    }

    /****************************************************************************************************************/
    void setDynamicFeasibilityTolerance(const ScalarType & aInput)
    /****************************************************************************************************************/
    {
        mDynamicFeasibilityTolerance = aInput;
    }

    /******************************************************************************//**
     * @brief Return initial dynamic feasibility tolerance
     * @return initial dynamic feasibility tolerance
    **********************************************************************************/
    ScalarType getInitialDynamicFeasibilityTolerance() const
    {
        return (mInitialDynamicFeasibilityTolerance);
    }

    void setInitialDynamicFeasibilityTolerance(const ScalarType & aInput)
    {
        mInitialDynamicFeasibilityTolerance = aInput;
        mDynamicFeasibilityTolerance = mInitialDynamicFeasibilityTolerance;
    }

    /******************************************************************************//**
     * @brief Return norm of objective function gradient
     * @return norm of objective function gradient
    **********************************************************************************/
    ScalarType getNormObjectiveFunctionGradient() const
    {
        return (mNormObjFuncGrad);
    }

    /******************************************************************************//**
     * @brief Return norm of augmented Lagrangian gradient
     * @return norm of augmented Lagrangian gradient
    **********************************************************************************/
    ScalarType getNormAugmentedLagrangianGradient() const
    {
        return (mNormAugLagFuncGrad);
    }

    /******************************************************************************//**
     * @brief Get Lagrange multipliers
     * @param [out] aInput Lagrange multipliers
    **********************************************************************************/
    void getLagrangeMultipliers(Plato::MultiVector<ScalarType, OrdinalType> & aInput) const
    {
        assert(mLagrangeMultipliers.get() != nullptr);
        Plato::update(1., *mLagrangeMultipliers, 0., aInput);
    }

    /****************************************************************************************************************/
    void setLagrangeMultipliers(const ScalarType & aValue)
    /****************************************************************************************************************/
    {
        assert(mLagrangeMultipliers.get() != nullptr);
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
        assert(aVectorIndex < mLagrangeMultipliers->getNumVectors());
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
        Plato::update(static_cast<ScalarType>(1.), *mCurrentConstraintValues, static_cast<ScalarType>(0.), aInput);
    }

    /****************************************************************************************************************/
    void getCurrentConstraintValues(const OrdinalType & aIndex, Plato::Vector<ScalarType, OrdinalType> & aInput) const
    /****************************************************************************************************************/
    {
        assert(aIndex < mCurrentConstraintValues->getNumVectors());
        aInput.update(static_cast<ScalarType>(1.), (*mCurrentConstraintValues)[aIndex], static_cast<ScalarType>(0.));
    }

    /******************************************************************************//**
     * @brief Return constraint value
     * @param [in] aVecIndex vector index
     * @param [in] aConstraintIndex constraint index
     * @return constraint value
    **********************************************************************************/
    ScalarType getCurrentConstraintValues(const OrdinalType & aVecIndex, const OrdinalType & aConstraintIndex) const
    {
        assert(aVecIndex < mCurrentConstraintValues->getNumVectors());
        assert(aConstraintIndex < (*mCurrentConstraintValues)[aVecIndex].size());
        return ((*mCurrentConstraintValues)(aVecIndex, aConstraintIndex));
    }

    /******************************************************************************//**
     * @brief Return norm of constraint vector
     * @return norm of constraint vector
    **********************************************************************************/
    ScalarType getNormConstraintVector() const
    {
        return (mNormConstraints);
    }

    /******************************************************************************//**
     * @brief Compute norm of constraint vector
     * @return norm of constraint vector
    **********************************************************************************/
    void computeNormConstraintVector()
    {
        mNormConstraints = Plato::norm(*mCurrentConstraintValues);
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

    //! Directive to update problem for each criterion (e.g. application based continuation).
    void updateProblem()
    {
        // Communicate user that criteria specific data can be cached since trial control was accepted
        assert(mObjective.get() != nullptr);
        mObjective->updateProblem();

        const OrdinalType tNumConstraints = mConstraints->size();
        assert(tNumConstraints == mConstraints->size());
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            (*mConstraints)[tConstraintIndex].updateProblem();
        }
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

        this->cacheTrialCriteriaValues();
    }

    /****************************************************************************************************************/
    void updateOptimizationData(Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    /****************************************************************************************************************/
    {
        // compute objective and constraint stagnation measures
        mObjectiveStagnationMeasure = std::abs(mCurrentObjFuncValue - mPreviousObjectiveValue);
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

        aDataMng.setNumObjectiveFunctionEvaluations(mNumObjFuncEval);
        aDataMng.setNumObjectiveGradientEvaluations(mNumObjGradEval);
    }

    /******************************************************************************//**
     * @brief Evaluate augmented Lagrangian function
     * @param [in] aControl optimization variables
     **********************************************************************************/
    ScalarType evaluateAugmentedLagrangian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        // Evaluate objective function, f(\mathbf{z})
        mCurrentObjFuncValue = mObjective->value(aControl);
        mNumObjFuncEval++;

        // Evaluate inequality constraints, h(\mathbf{u}(\mathbf{z}),\mathbf{z})
        this->evaluateConstraint(aControl, *mCurrentConstraintValues);

        // Evaluate augmented Lagrangian
        mCurrentAugLagFuncValue = this->evaluateAugmentedLagrangianCriterion(mCurrentObjFuncValue);

        return (mCurrentAugLagFuncValue);
    }

    /******************************************************************************//**
     * @brief Compute/Evaluate objective function
     * @param [in] aControl optimization variables
     * @param [in] aTolerance objective inexactness tolerance
    **********************************************************************************/
    ScalarType evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                 ScalarType aTolerance = std::numeric_limits<ScalarType>::max())
    {
        // Evaluate objective function, f(\mathbf{z})
        mTrialObjFuncValue = mObjective->value(aControl);
        mNumObjFuncEval++;

        // Evaluate inequality constraints, h(\mathbf{u}(\mathbf{z}),\mathbf{z})
        this->evaluateConstraint(aControl, *mTrialConstraintValues);

        // Evaluate augmented Lagrangian
        mTrialAugLagFuncValue = this->evaluateAugmentedLagrangianCriterion(mTrialObjFuncValue);

        return (mTrialAugLagFuncValue);
    }

    /******************************************************************************//**
     * @brief Compute gradient
     * @param [in] aControl optimization variables
     * @param [out] aOutput augmented Lagrangian gradient
    **********************************************************************************/
    void computeGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                         Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mObjectiveGradientOperator.get() != nullptr);

        // Compute objective function gradient: \frac{\partial f}{\partial\mathbf{z}}
        Plato::fill(static_cast<ScalarType>(0), aOutput);
        mObjectiveGradientOperator->compute(aControl, aOutput);
        mNormObjFuncGrad = Plato::norm(aOutput);
        mNumObjGradEval++;

        this->computeAugmentedLagrangianGradient(aControl, aOutput);
        mNormAugLagFuncGrad = Plato::norm(aOutput);
    }

    /******************************************************************************//**
     * @brief Compute Hessian times vector
     * @param [in] aControl optimization variables
     * @param [in] aVector descent direction
     * @param [out] aOutput Hessian times vector
    **********************************************************************************/
    void applyVectorToHessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                              const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                              Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mObjectiveHessianOperator.get() != nullptr);
        Plato::fill(static_cast<ScalarType>(0), aOutput);
        mObjectiveHessianOperator->apply(aControl, aVector, aOutput);
        mNumObjHessEval++;

        this->computeAugmentedLagrangianHessian(aControl, aVector, aOutput);
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

    /******************************************************************************//**
     * @brief Evaluate inequality constraints
    **********************************************************************************/
    void evaluateConstraint(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                            Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        // Evaluate inequality constraints, h(\mathbf{u}(\mathbf{z}),\mathbf{z})
        const OrdinalType tNumVectors = aOutput.getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const OrdinalType tNumConstraints = aOutput[tVectorIndex].size();
            Plato::Vector<ScalarType, OrdinalType> & tMyConstraint  = aOutput[tVectorIndex];
            for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
            {
                tMyConstraint[tConstraintIndex] = (*mConstraints)[tConstraintIndex].value(aControl);
                mNumConstraintEvaluations[tConstraintIndex] = mNumConstraintEvaluations[tConstraintIndex] + static_cast<OrdinalType>(1);
            }
        }
    }

    /******************************************************************************//**
     * @brief Update Lagrange multipliers
     * \f$ \lambda_{k+1} = \lambda^{k}_{i} + \frac{1}{\mu^k}c_{i}(x_{k})\f$, where
     * \f$\lambda\f$ denotes the k-th iteration Lagrange multipliers associated with
     * the i-th inequality constraint \f$c_{i}(x_{k})\f$, \f$\mu\f$ denotes the penalty
     * parameter and \f$x_{k}\f$ is the current set of optimization variables.
    **********************************************************************************/
    void updateLagrangeMultipliers()
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
                tMyLambda[tConstraintIndex] = std::max(static_cast<ScalarType>(0), tMyLambda[tConstraintIndex]);
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
        mDynamicFeasibilityTolerance = mInitialDynamicFeasibilityTolerance / std::sqrt(tPenalty);
    }

    /****************************************************************************************************************/
    void cacheCurrentCriteriaValues()
    /****************************************************************************************************************/
    {
        mPreviousObjectiveValue = mCurrentObjFuncValue;
        mPreviousAugLagFuncValue = mCurrentAugLagFuncValue;
        Plato::update(static_cast<ScalarType>(1), *mCurrentConstraintValues, static_cast<ScalarType>(0), *mPreviousConstraintValues);
    }

    /****************************************************************************************************************/
    void cacheTrialCriteriaValues()
    /****************************************************************************************************************/
    {
        mCurrentObjFuncValue = mTrialObjFuncValue;
        mCurrentAugLagFuncValue = mTrialAugLagFuncValue;
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
    /******************************************************************************//**
     * @brief Initialize Lagrange multipliers and constraint gradients
    **********************************************************************************/
    void initialize()
    {
        const OrdinalType tVECTOR_INDEX = 0;
        const OrdinalType tNumConstraints = mCurrentConstraintValues->operator[](tVECTOR_INDEX).size();
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            mCostraintGradients->add(mControlWorkVec.operator*());
        }

        const ScalarType tInitialLagrangeMultipliers = 0.5;
        Plato::fill(tInitialLagrangeMultipliers, *mLagrangeMultipliers);
    }

    /******************************************************************************//**
     * @brief Evaluate augmented Lagrangian criterion
     * @param [in] aObjFuncValue objective function value
     * @return augmented Lagrangian evaluation
    **********************************************************************************/
    ScalarType evaluateAugmentedLagrangianCriterion(const ScalarType & aObjFuncValue)
    {
        ScalarType tOutput = aObjFuncValue;

        const OrdinalType tCONSTRAINT_VEC_INDEX = 0;
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
                // Evaluate Lagrangian functional, \ell(\mathbf{u}(\mathbf{z}),\mathbf{z},\mu) =
                //   f(\mathbf{u}(\mathbf{z}),\mathbf{z}) + \mu^{T}h(\mathbf{u}(\mathbf{z}),\mathbf{z})
                const ScalarType tMyConstraintValue = (*mTrialConstraintValues)(tCONSTRAINT_VEC_INDEX, tIndex);
                const ScalarType tLagrangeMultipliersDotInequalityValue =
                        (*mLagrangeMultipliers)(tCONSTRAINT_VEC_INDEX, tIndex) * tMyConstraintValue;

                // Evaluate augmented Lagrangian functional, \mathcal{L}(\mathbf{z}),\mathbf{z},\mu) =
                //   \ell(\mathbf{u}(\mathbf{z}),\mathbf{z},\mu) +
                //   \frac{1}{2\beta}(h(\mathbf{u}(\mathbf{z}),\mathbf{z})^{T}h(\mathbf{u}(\mathbf{z}),\mathbf{z})),
                //   where \beta\in\mathbb{R} denotes a penalty parameter
                const ScalarType tInequalityValueDotInequalityValue = tMyConstraintValue * tMyConstraintValue;
                const ScalarType tMyConstraintContribution = tLagrangeMultipliersDotInequalityValue
                        + ((static_cast<ScalarType>(0.5) / mPenaltyParameter) * tInequalityValueDotInequalityValue);

                tOutput += tMyConstraintContribution;
        }

        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Compute augmented Lagrangian gradient
     * @param [in] aControl optimization variables
     * @param [out] aOutput augmented Lagrangian gradient
    **********************************************************************************/
    void computeAugmentedLagrangianGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                            Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mConstraintGradientOperator.get() != nullptr);

        // Compute inequality constraint gradient: \frac{\partial h_i}{\partial\mathbf{z}}
        const OrdinalType tCONSTRAINT_VEC_INDEX = 0;
        const ScalarType tOneOverPenalty = static_cast<ScalarType>(1.) / mPenaltyParameter;

        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            assert(mConstraintGradientOperator->ptr(tIndex).get() != nullptr);

            const ScalarType tMyConstraintValue = (*mCurrentConstraintValues)(tCONSTRAINT_VEC_INDEX, tIndex);

            // Add contribution from: \lambda_i\frac{\partial h_i}{\partial\mathbf{z}} to Lagrangian gradient
            Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintGradient =
                    mCostraintGradients->operator[](tCONSTRAINT_VEC_INDEX);
            Plato::fill(static_cast<ScalarType>(0), tMyConstraintGradient);
            mConstraintGradientOperator->operator[](tIndex).compute(aControl, tMyConstraintGradient);
            mNumConstraintGradientEvaluations[tIndex] = mNumConstraintGradientEvaluations[tIndex] + static_cast<OrdinalType>(1);

            const ScalarType tLagrangeMultiplier = mLagrangeMultipliers->operator()(tCONSTRAINT_VEC_INDEX, tIndex);
            Plato::update(tLagrangeMultiplier, tMyConstraintGradient, static_cast<ScalarType>(0), *mObjectiveGradient);

            // Add contribution from: \mu*h_i(\mathbf{u}(\mathbf{z}),\mathbf{z})\frac{\partial h_i}{\partial\mathbf{z}}.
            const ScalarType tAlpha = tOneOverPenalty * tMyConstraintValue;
            Plato::update(tAlpha, tMyConstraintGradient, static_cast<ScalarType>(1), *mObjectiveGradient);

            // Compute Augmented Lagrangian gradient
            Plato::update(static_cast<ScalarType>(1), *mObjectiveGradient, static_cast<ScalarType>(1), aOutput);
        }
    }

    /******************************************************************************//**
     * @brief Compute augmented Lagrangian Hessian times vector
     * @param [in] aControl optimization variables
     * @param [in] aVector descent direction
     * @param [out] aOutput augmented Lagrangian Hessian
    **********************************************************************************/
    void computeAugmentedLagrangianHessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                           const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                           Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mConstraintHessianOperators.get() != nullptr);

        // Apply vector to inequality constraint Hessian operator and add contribution to total Hessian
        const OrdinalType tCONSTRAINT_VEC_INDEX = 0;
        const ScalarType tOneOverPenalty = static_cast<ScalarType>(1.) / mPenaltyParameter;

        const OrdinalType tNumConstraints = mConstraints->size();
        const Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintGradients = (*mCostraintGradients)[tCONSTRAINT_VEC_INDEX];
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            assert(mConstraintHessianOperators->ptr(tIndex).get() != nullptr);

            const ScalarType tMyConstraintValue = (*mCurrentConstraintValues)(tCONSTRAINT_VEC_INDEX, tIndex);
            // Add contribution from: \lambda_i\frac{\partial^2 h_i}{\partial\mathbf{z}^2}
            Plato::fill(static_cast<ScalarType>(0), *mControlWorkVec);
            (*mConstraintHessianOperators)[tIndex].apply(aControl, aVector, *mControlWorkVec);
            mNumConstraintHessianEvaluations[tIndex] = mNumConstraintHessianEvaluations[tIndex] + static_cast<OrdinalType>(1);

            Plato::update((*mLagrangeMultipliers)(tCONSTRAINT_VEC_INDEX, tIndex),
                          *mControlWorkVec,
                          static_cast<ScalarType>(1),
                          aOutput);

            // Add contribution from: \mu\frac{\partial^2 h_i}{\partial\mathbf{z}^2}\h_i(\mathbf{z})
            ScalarType tAlpha = tOneOverPenalty * tMyConstraintValue;
            Plato::update(tAlpha, *mControlWorkVec, static_cast<ScalarType>(1), aOutput);

            // Compute Jacobian, i.e. \frac{\partial h_i}{\partial\mathbf{z}}
            ScalarType tJacobianDotTrialDirection = Plato::dot(tMyConstraintGradients, aVector);
            ScalarType tBeta = tOneOverPenalty * tJacobianDotTrialDirection;
            // Add contribution from: \mu\left(\frac{\partial h_i}{\partial\mathbf{z}}^{T}
            //                        \frac{\partial h_i}{\partial\mathbf{z}}\right)
            Plato::update(tBeta, tMyConstraintGradients, static_cast<ScalarType>(1), aOutput);
        }
    }

private:
    OrdinalType mNumObjFuncEval;
    OrdinalType mNumObjGradEval;
    OrdinalType mNumObjHessEval;

    ScalarType mTrialObjFuncValue;
    ScalarType mCurrentObjFuncValue;
    ScalarType mTrialAugLagFuncValue;
    ScalarType mCurrentAugLagFuncValue;
    ScalarType mPreviousObjectiveValue;
    ScalarType mPreviousAugLagFuncValue;
    ScalarType mObjectiveStagnationMeasure;

    ScalarType mMinPenaltyValue;
    ScalarType mNormConstraints;
    ScalarType mNormObjFuncGrad;
    ScalarType mNormAugLagFuncGrad;
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

private:
    AugmentedLagrangianStageMng(const Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>&);
    Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType> & operator=(const Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>&);
};

} // namespace Plato

#endif /* PLATO_AUGMENTEDLAGRANGIANSTAGEMNG_HPP_ */
