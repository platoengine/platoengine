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

#include "Plato_Macros.hpp"
#include "Plato_Vector.hpp"
#include "Plato_StateData.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_HessianLBFGS.hpp"
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
     * @param [in] aFactory optimization data factory
     * @param [in] aObjective objective function interface
     * @param [in] aConstraints list of constraint interface
     **********************************************************************************/
    AugmentedLagrangianStageMng(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aObjective,
                                const std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> & aConstraints) :
            mIsMeanNormEnabled(false),
            mNumObjFuncEval(0),
            mNumObjGradEval(0),
            mNumObjHessEval(0),
            mTrialObjFuncValue(std::numeric_limits<ScalarType>::max()),
            mCurrentObjFuncValue(std::numeric_limits<ScalarType>::max()),
            mPreviousObjFuncValue(std::numeric_limits<ScalarType>::max()),
            mTrialAugLagFuncValue(std::numeric_limits<ScalarType>::max()),
            mCurrentAugLagFuncValue(std::numeric_limits<ScalarType>::max()),
            mPreviousAugLagFuncValue(std::numeric_limits<ScalarType>::max()),
            mObjectiveStagnationMeasure(std::numeric_limits<ScalarType>::max()),
            mNormConstraints(std::numeric_limits<ScalarType>::max()),
            mNormObjFuncGrad(std::numeric_limits<ScalarType>::max()),
            mNormAugLagFuncGrad(std::numeric_limits<ScalarType>::max()),
            mPenaltyParameter(0.05),
            mInitialPenaltyParameter(mPenaltyParameter),
            mPenaltyParameterLowerBound(1e-5),
            mPenaltyParameterScaleFactor(2),
            mDefaultInitialLagrangeMultiplierValue(0.5),
            mNumConstraintEvaluations(std::vector<OrdinalType>(aConstraints->size())),
            mNumConstraintGradientEvaluations(std::vector<OrdinalType>(aConstraints->size())),
            mNumConstraintHessianEvaluations(std::vector<OrdinalType>(aConstraints->size())),
            mStateData(std::make_shared<Plato::StateData<ScalarType, OrdinalType>>(aFactory.operator*())),
            mDualWorkMultiVec(aFactory->dual().create()),
            mControlWorkVec(aFactory->control().create()),
            mLagrangeMultipliers(aFactory->dual().create()),
            mTrialConstraintValues(aFactory->dual().create()),
            mCurrentConstraintValues(aFactory->dual().create()),
            mPreviousConstraintValues(aFactory->dual().create()),
            mCurrentObjFuncGrad(aFactory->control().create()),
            mPreviousObjFuncGrad(aFactory->control().create()),
            mCurrentCostraintGrad(std::make_shared<Plato::MultiVectorList<ScalarType, OrdinalType>>()),
            mPreviousCostraintGrad(std::make_shared<Plato::MultiVectorList<ScalarType, OrdinalType>>()),
            mObjective(aObjective),
            mConstraints(aConstraints),
            mPreconditioner(std::make_shared<Plato::IdentityPreconditioner<ScalarType, OrdinalType>>()),
            mObjFuncHessian(std::make_shared<Plato::AnalyticalHessian<ScalarType, OrdinalType>>(mObjective)),
            mConstraintHessians(std::make_shared<Plato::LinearOperatorList<ScalarType, OrdinalType>>(mConstraints)),
            mDualReductionOperations(aFactory->getDualReductionOperations().create()),
            mObjectiveGradOperator(std::make_shared<Plato::AnalyticalGradient<ScalarType, OrdinalType>>(mObjective)),
            mConstraintGradientOperator(std::make_shared<Plato::GradientOperatorList<ScalarType, OrdinalType>>(mConstraints))
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
     * @brief Enable mean norm, i.e. /f$ \mu = \frac{1}{N} \sum_{i=1}^{N} x_i * x_i /f$
    **********************************************************************************/
    void enableMeanNorm()
    {
        mIsMeanNormEnabled = true;
    }

    /******************************************************************************//**
     * @brief Reset penalty and lagrange multipliers to initial default values.
    **********************************************************************************/
    void resetParameters()
    {
        mPenaltyParameter = mInitialPenaltyParameter;
        Plato::fill(mDefaultInitialLagrangeMultiplierValue, *mLagrangeMultipliers);
    }

    /******************************************************************************//**
     * @brief Return number of constraints
     * @return number of constraints
    **********************************************************************************/
    OrdinalType getNumConstraints() const
    {
        const OrdinalType tNumConstraints = mConstraints->size();
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
        mInitialPenaltyParameter = mPenaltyParameter;
    }

    /******************************************************************************//**
     * @brief Return penalty parameter scale
     * @return penalty parameter scale
    **********************************************************************************/
    ScalarType getPenaltyParameterScaleFactor() const
    {
        return (mPenaltyParameterScaleFactor);
    }

    /******************************************************************************//**
     * @brief Set scale factor used to update penalty parameter
     * @param [in] aInput scale factor
    **********************************************************************************/
    void setPenaltyParameterScaleFactor(const ScalarType & aInput)
    {
        mPenaltyParameterScaleFactor = aInput;
    }

    /******************************************************************************//**
     * @brief Return lower bound on penalty parameter
     * @return lower bound on penalty parameter
    **********************************************************************************/
    ScalarType getPenaltyParameterLowerBound() const
    {
        return (mPenaltyParameterLowerBound);
    }

    /******************************************************************************//**
     * @brief Set lower bound on penalty parameter
     * @param [in] aInput lower bound on penalty parameter
    **********************************************************************************/
    void setPenaltyParameterLowerBound(const ScalarType & aInput)
    {
        mPenaltyParameterLowerBound = aInput;
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

    /******************************************************************************//**
     * @brief Set all the Lagrange multipliers to input scalar value
     * @param [in] aInput Lagrange multipliers
    **********************************************************************************/
    void setLagrangeMultipliers(const ScalarType & aInput)
    {
        assert(mLagrangeMultipliers.get() != nullptr);
        assert(mLagrangeMultipliers->getNumVectors() > static_cast<OrdinalType>(0));
        if(aInput < static_cast<ScalarType>(0))
        {
            THROWERR("NEGATIVE LAGRANGE MULTIPLIERS DETECTED.\n")
        }
        OrdinalType tNumVectors = mLagrangeMultipliers->getNumVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            mLagrangeMultipliers->operator [](tVectorIndex).fill(aInput);
        }
    }

    /******************************************************************************//**
     * @brief Set Lagrange multipliers to input scalar value
     * @param [in] aVectorIndex dual vector index
     * @param [in] aConstraintIndex constraint index
     * @param [in] aValue scalar value
    **********************************************************************************/
    void setLagrangeMultipliers(const OrdinalType & aVectorIndex, const OrdinalType & aConstraintIndex, const ScalarType & aValue)
    {
        assert(mLagrangeMultipliers.get() != nullptr);
        assert(aVectorIndex < mLagrangeMultipliers->getNumVectors());
        assert(aConstraintIndex < mLagrangeMultipliers->operator[](aVectorIndex).size());
        mLagrangeMultipliers->operator ()(aVectorIndex, aConstraintIndex).fill(aValue);
        if(aValue < static_cast<ScalarType>(0))
        {
            THROWERR("NEGATIVE LAGRANGE MULTIPLIERS DETECTED.\n")
        }
    }

    /******************************************************************************//**
     * @brief Set vector of Lagrange multipliers
     * @param [in] aVectorIndex dual vector index
     * @param [in] aInput vector of Lagrange multipliers
    **********************************************************************************/
    void setLagrangeMultipliers(const OrdinalType & aVectorIndex, const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        assert(mLagrangeMultipliers.get() != nullptr);
        assert(aVectorIndex < mLagrangeMultipliers->getNumVectors());
        assert(mLagrangeMultipliers->operator[](aVectorIndex).size() == aInput.size());
        for(OrdinalType tIndex = 0; tIndex < aInput.size(); tIndex++)
        {
            if(aInput[tIndex] < static_cast<ScalarType>(0))
            {
                std::string tError = "LAGRANGE MULTIPLIER WITH INDEX = " + std::to_string(tIndex) + " IS NEGATIVE. "
                        + "LAGRANGE MULTIPLIERS HOULD BE POSITIVE.\n";
                THROWERR(tError)
            }
        }
        mLagrangeMultipliers->operator [](aVectorIndex).update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0));
    }

    /******************************************************************************//**
     * @brief Return all the constraint values
     * @param [out] aInput multi-vector of constraint values
    **********************************************************************************/
    void getCurrentConstraintValues(Plato::MultiVector<ScalarType, OrdinalType> & aInput) const
    {
        Plato::update(static_cast<ScalarType>(1.), *mCurrentConstraintValues, static_cast<ScalarType>(0.), aInput);
    }

    /******************************************************************************//**
     * @brief Return all the constraint values
     * @param [out] aInput vector of constraint values
    **********************************************************************************/
    void getCurrentConstraintValues(const OrdinalType & aIndex, Plato::Vector<ScalarType, OrdinalType> & aInput) const
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
     * @brief Compute l1-norm of constraint vector
     * @return l1-norm of constraint vector
    **********************************************************************************/
    void computeNormConstraintVector()
    {
        Plato::update(static_cast<ScalarType>(1), *mCurrentConstraintValues, static_cast<ScalarType>(0), *mDualWorkMultiVec);

        const OrdinalType tNumVectors = mDualWorkMultiVec->getNumVectors();
        std::vector<ScalarType> tContainer(tNumVectors);
        for(OrdinalType tIndex = 0; tIndex < tNumVectors; ++tIndex)
        {
            (*mDualWorkMultiVec)[tIndex].modulus();
            tContainer[tIndex] = mDualReductionOperations->max( (*mDualWorkMultiVec)[tIndex] );
        }
        mNormConstraints = *std::max_element(tContainer.begin(), tContainer.end());
    }

    /******************************************************************************//**
     * @brief Set numerical method used to compute the objective gradient
     * @param [in] aInput interface to numerical method
    **********************************************************************************/
    void setObjectiveGradient(const std::shared_ptr<Plato::GradientOperator<ScalarType, OrdinalType>> & aInput)
    {
        assert(aInput.get() != nullptr);
        mObjectiveGradOperator = aInput;
    }

    /******************************************************************************//**
     * @brief Set numerical method used to compute the constraint gradients
     * @param [in] aInput interface to numerical method
    **********************************************************************************/
    void setConstraintGradients(const std::shared_ptr<Plato::GradientOperatorList<ScalarType, OrdinalType>> & aInput)
    {
        assert(aInput.get() != nullptr);
        mConstraintGradientOperator = aInput;
    }

    /******************************************************************************//**
     * @brief Set numerical method used to compute the application of a vector to the objective Hessian
     * @param [in] aInput interface to numerical method.
    **********************************************************************************/
    void setObjectiveHessian(const std::shared_ptr<Plato::LinearOperator<ScalarType, OrdinalType>> & aInput)
    {
        assert(aInput.get() != nullptr);
        mObjFuncHessian = aInput;
    }

    /******************************************************************************//**
     * @brief Set objective function Hessian to LBFGS.
     * @param [in] aMaxMemory memory size (default = 8)
    **********************************************************************************/
    void setObjectiveHessianLBFGS(OrdinalType aMaxMemory = 8)
    {
        mObjFuncHessian.reset();
        mObjFuncHessian = std::make_shared<Plato::HessianLBFGS<ScalarType, OrdinalType>>(*mControlWorkVec, aMaxMemory);
    }

    /******************************************************************************//**
     * @brief Set numerical method used to compute the application of a vector to the constraint function Hessians
     * @param [in] aInput interface to numerical method.
    **********************************************************************************/
    void setConstraintHessians(const std::shared_ptr<Plato::LinearOperatorList<ScalarType, OrdinalType>> & aInput)
    {
        assert(aInput.get() != nullptr);
        mConstraintHessians = aInput;
    }

    /******************************************************************************//**
     * @brief Set constraint Hessians to LBFGS.
     * @param [in] aMaxMemory memory size (default = 8)
    **********************************************************************************/
    void setConstraintHessiansLBFGS(OrdinalType aMaxMemory = 8)
    {
        mConstraintHessians.reset();
        mConstraintHessians = std::make_shared<Plato::LinearOperatorList<ScalarType, OrdinalType>>();
        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            mConstraintHessians->add(std::make_shared<Plato::HessianLBFGS<ScalarType, OrdinalType>>(*mControlWorkVec, aMaxMemory));
        }
    }

    /******************************************************************************//**
     * @brief Set numerical method used to compute the application of a vector to the preconditioner
     * @param [in] aInput interface to numerical method
    **********************************************************************************/
    void setPreconditioner(const std::shared_ptr<Plato::Preconditioner<ScalarType, OrdinalType>> & aInput)
    {
        assert(aInput.get() != nullptr);
        mPreconditioner = aInput;
    }

    /*******************************************************************************//**
     * @brief Directive to update app-specific parameters (e.g. continuation).
     * @param [in] aControl optimization variables
    ***********************************************************************************/
    void updateProblem(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        // Communicate user that criteria specific data can be cached since trial control was accepted
        assert(mObjective.get() != nullptr);
        mObjective->updateProblem(aControl);

        const OrdinalType tNumConstraints = mConstraints->size();
        assert(tNumConstraints == mConstraints->size());
        for(OrdinalType tConstraintIndex = 0; tConstraintIndex < tNumConstraints; tConstraintIndex++)
        {
            (*mConstraints)[tConstraintIndex].updateProblem(aControl);
        }
    }

    /******************************************************************************//**
     * @brief Cache criteria specific data once the trial control is accepted.
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

        this->cacheTrialCriteriaValues();
    }

    /******************************************************************************//**
     * @brief Update state data
     * @param [out] aDataMng optimization algorithm data manager
    **********************************************************************************/
    void updateOptimizationData(Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        // compute objective and constraint stagnation measures
        mObjectiveStagnationMeasure = std::abs(mCurrentObjFuncValue - mPreviousObjFuncValue);

        mStateData->setCurrentTrialStep(aDataMng.getTrialStep());
        mStateData->setCurrentControl(aDataMng.getCurrentControl());
        mStateData->setPreviousControl(aDataMng.getPreviousControl());

        this->updatePreconditionerStateData(aDataMng);
        this->updateConstraintsStateData();
        this->updateObjFuncStateData();
        this->cacheCriteriaGradients();

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
        mCurrentAugLagFuncValue = this->evaluateAugmentedLagrangianFunc(mCurrentObjFuncValue);

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
        mTrialAugLagFuncValue = this->evaluateAugmentedLagrangianFunc(mTrialObjFuncValue);

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
        assert(mObjectiveGradOperator.get() != nullptr);

        // Compute objective function gradient: \frac{\partial f}{\partial\mathbf{z}}
        Plato::fill(static_cast<ScalarType>(0), aOutput);
        mObjectiveGradOperator->compute(aControl, aOutput);
        Plato::update(static_cast<ScalarType>(1), aOutput, static_cast<ScalarType>(0), *mCurrentObjFuncGrad);
        mNormObjFuncGrad =
                mIsMeanNormEnabled == true ? Plato::norm_mean(*mCurrentObjFuncGrad) : Plato::norm(*mCurrentObjFuncGrad);
        mNumObjGradEval++;

        this->computeAugmentedLagrangianGradient(aControl, aOutput);
        mNormAugLagFuncGrad = mIsMeanNormEnabled == true ? Plato::norm_mean(aOutput) : Plato::norm(aOutput);
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
        assert(mObjFuncHessian.get() != nullptr);
        Plato::fill(static_cast<ScalarType>(0), aOutput);
        mObjFuncHessian->apply(aControl, aVector, aOutput);
        mNumObjHessEval++;

        this->computeAugmentedLagrangianHessian(aControl, aVector, aOutput);
    }

    /******************************************************************************//**
     * @brief Apply vector to preconditioner operator
     * @param [in] aControl vector of optimization variables
     * @param [in] aVector descent direction
     * @param [out] aOutput application of vector to preconditioner operator
    **********************************************************************************/
    void applyVectorToPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                     const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                     Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        assert(mPreconditioner.get() != nullptr);
        mPreconditioner->applyPreconditioner(aControl, aVector, aOutput);
    }

    /******************************************************************************//**
     * @brief Apply vector to inverse preconditioner operator
     * @param [in] aControl vector of optimization variables
     * @param [in] aVector descent direction
     * @param [out] aOutput application of vector to inverse preconditioner operator
    **********************************************************************************/
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
    }

    /******************************************************************************//**
     * @brief Update augmented Lagrangian penalty parameter
    **********************************************************************************/
    void updatePenaltyParameter()
    {
        mPenaltyParameter = mPenaltyParameter / mPenaltyParameterScaleFactor;
        mPenaltyParameter = std::max(mPenaltyParameter, mPenaltyParameterLowerBound);
    }

    /******************************************************************************//**
     * @brief Cache trial objective and constraint criteria values
    **********************************************************************************/
    void cacheTrialCriteriaValues()
    {
        mPreviousObjFuncValue = mCurrentObjFuncValue;
        mCurrentObjFuncValue = mTrialObjFuncValue;

        mPreviousAugLagFuncValue = mCurrentAugLagFuncValue;
        mCurrentAugLagFuncValue = mTrialAugLagFuncValue;
        Plato::update(static_cast<ScalarType>(1), *mCurrentConstraintValues, static_cast<ScalarType>(0), *mPreviousConstraintValues);
        Plato::update(static_cast<ScalarType>(1), *mTrialConstraintValues, static_cast<ScalarType>(0), *mCurrentConstraintValues);
    }

private:
    /******************************************************************************//**
     * @brief Initialize Lagrange multipliers and constraint gradients
    **********************************************************************************/
    void initialize()
    {
        const OrdinalType tVECTOR_INDEX = 0;
        const OrdinalType tNumConstraints = mCurrentConstraintValues->operator[](tVECTOR_INDEX).size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            mCurrentCostraintGrad->add(*mControlWorkVec);
            mPreviousCostraintGrad->add(*mControlWorkVec);
        }

        Plato::fill(mDefaultInitialLagrangeMultiplierValue, *mLagrangeMultipliers);
    }

    /******************************************************************************//**
     * @brief Evaluate augmented Lagrangian criterion
     * @param [in] aObjFuncValue objective function value
     * @return augmented Lagrangian evaluation
    **********************************************************************************/
    ScalarType evaluateAugmentedLagrangianFunc(const ScalarType & aObjFuncValue)
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
        const OrdinalType tDUAL_VEC_INDEX = 0;
        const ScalarType tOneOverPenalty = static_cast<ScalarType>(1.) / mPenaltyParameter;

        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            assert(mConstraintGradientOperator->ptr(tIndex).get() != nullptr);

            const ScalarType tMyConstraintValue = (*mCurrentConstraintValues)(tDUAL_VEC_INDEX, tIndex);

            // Add contribution from: \lambda_i\frac{\partial h_i}{\partial\mathbf{z}} to Lagrangian gradient
            Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintGradient =
                    mCurrentCostraintGrad->operator[](tDUAL_VEC_INDEX);
            Plato::fill(static_cast<ScalarType>(0), tMyConstraintGradient);
            mConstraintGradientOperator->operator[](tIndex).compute(aControl, tMyConstraintGradient);
            mNumConstraintGradientEvaluations[tIndex] = mNumConstraintGradientEvaluations[tIndex] + static_cast<OrdinalType>(1);

            const ScalarType tLagrangeMultiplier = mLagrangeMultipliers->operator()(tDUAL_VEC_INDEX, tIndex);
            Plato::update(tLagrangeMultiplier, tMyConstraintGradient, static_cast<ScalarType>(0), *mControlWorkVec);

            // Add contribution from: \mu*h_i(\mathbf{u}(\mathbf{z}),\mathbf{z})\frac{\partial h_i}{\partial\mathbf{z}}.
            const ScalarType tAlpha = tOneOverPenalty * tMyConstraintValue;
            Plato::update(tAlpha, tMyConstraintGradient, static_cast<ScalarType>(1), *mControlWorkVec);

            // Compute Augmented Lagrangian gradient
            Plato::update(static_cast<ScalarType>(1), *mControlWorkVec, static_cast<ScalarType>(1), aOutput);
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
        assert(mConstraintHessians.get() != nullptr);

        // Apply vector to inequality constraint Hessian operator and add contribution to total Hessian
        const OrdinalType tDUAL_VEC_INDEX = 0;
        const ScalarType tOneOverPenalty = static_cast<ScalarType>(1.) / mPenaltyParameter;

        const OrdinalType tNumConstraints = mConstraints->size();
        const Plato::MultiVector<ScalarType, OrdinalType> & tMyConstraintGrad = (*mCurrentCostraintGrad)[tDUAL_VEC_INDEX];
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            assert(mConstraintHessians->ptr(tIndex).get() != nullptr);

            const ScalarType tMyConstraintValue = (*mCurrentConstraintValues)(tDUAL_VEC_INDEX, tIndex);
            // Add contribution from: \lambda_i\frac{\partial^2 h_i}{\partial\mathbf{z}^2}
            Plato::fill(static_cast<ScalarType>(0), *mControlWorkVec);
            (*mConstraintHessians)[tIndex].apply(aControl, aVector, *mControlWorkVec);
            mNumConstraintHessianEvaluations[tIndex] = mNumConstraintHessianEvaluations[tIndex] + static_cast<OrdinalType>(1);

            Plato::update((*mLagrangeMultipliers)(tDUAL_VEC_INDEX, tIndex),
                          *mControlWorkVec,
                          static_cast<ScalarType>(1),
                          aOutput);

            // Add contribution from: \mu\frac{\partial^2 h_i}{\partial\mathbf{z}^2}\h_i(\mathbf{z})
            ScalarType tAlpha = tOneOverPenalty * tMyConstraintValue;
            Plato::update(tAlpha, *mControlWorkVec, static_cast<ScalarType>(1), aOutput);

            // Compute Jacobian, i.e. \frac{\partial h_i}{\partial\mathbf{z}}
            ScalarType tJacobianDotTrialDirection = Plato::dot(tMyConstraintGrad, aVector);
            ScalarType tBeta = tOneOverPenalty * tJacobianDotTrialDirection;
            // Add contribution from: \mu\left(\frac{\partial h_i}{\partial\mathbf{z}}^{T}
            //                        \frac{\partial h_i}{\partial\mathbf{z}}\right)
            Plato::update(tBeta, tMyConstraintGrad, static_cast<ScalarType>(1), aOutput);
        }
    }

    /******************************************************************************//**
     * @brief Update the state data used to compute the application of a vector to the preconditioner.
    **********************************************************************************/
    void updatePreconditionerStateData(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mStateData->setCurrentCriterionGradient(aDataMng.getCurrentGradient());
        mStateData->setPreviousCriterionGradient(aDataMng.getPreviousGradient());
        mStateData->setCurrentCriterionValue(aDataMng.getCurrentObjectiveFunctionValue());
        mPreconditioner->update(*mStateData);
    }

    /******************************************************************************//**
     * @brief Update the state data needed to approximate the objective's sensitivities.
     *   Operation is null if analytical sensitivities are provided or calculations are disabled.
    **********************************************************************************/
    void updateObjFuncStateData()
    {
        mStateData->setCurrentCriterionValue(mCurrentObjFuncValue);
        mStateData->setCurrentCriterionGradient(*mCurrentObjFuncGrad);
        mStateData->setPreviousCriterionGradient(*mPreviousObjFuncGrad);
        mObjectiveGradOperator->update(*mStateData);
        mObjFuncHessian->update(*mStateData);
    }

    /******************************************************************************//**
     * @brief Update the state data needed to approximate the constraints' sensitivities.
     *   Operation is null if analytical sensitivities are provided or disabled by the user.
    **********************************************************************************/
    void updateConstraintsStateData()
    {
        const OrdinalType tNumConstraints = mConstraints->size();
        assert(tNumConstraints == mCurrentCostraintGrad->size());

        const OrdinalType tVECTOR_INDEX = 0;
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            const ScalarType tMyCurrentValue = (*mCurrentConstraintValues)(tVECTOR_INDEX, tIndex);
            mStateData->setCurrentCriterionValue(tMyCurrentValue);
            const Plato::MultiVector<ScalarType, OrdinalType> & tMyCurrentGrad = (*mCurrentCostraintGrad)[tIndex];
            mStateData->setCurrentCriterionGradient(tMyCurrentGrad);
            const Plato::MultiVector<ScalarType, OrdinalType> & tMyPreviousGrad = (*mPreviousCostraintGrad)[tIndex];
            mStateData->setPreviousCriterionGradient(tMyPreviousGrad);
            (*mConstraintGradientOperator)[tIndex].update(*mStateData);
            (*mConstraintHessians)[tIndex].update(*mStateData);
        }
    }

    /******************************************************************************//**
     * @brief Cache criteria gradients
    **********************************************************************************/
    void cacheCriteriaGradients()
    {
        Plato::update(static_cast<ScalarType>(1), *mCurrentObjFuncGrad, static_cast<ScalarType>(0), *mPreviousObjFuncGrad);

        const OrdinalType tNumConstraints = mConstraints->size();
        for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
        {
            Plato::update(static_cast<ScalarType>(1), (*mCurrentCostraintGrad)[tIndex], static_cast<ScalarType>(0), (*mPreviousCostraintGrad)[tIndex]);
        }
    }

private:
    bool mIsMeanNormEnabled;

    OrdinalType mNumObjFuncEval;
    OrdinalType mNumObjGradEval;
    OrdinalType mNumObjHessEval;

    ScalarType mTrialObjFuncValue;
    ScalarType mCurrentObjFuncValue;
    ScalarType mPreviousObjFuncValue;
    ScalarType mTrialAugLagFuncValue;
    ScalarType mCurrentAugLagFuncValue;
    ScalarType mPreviousAugLagFuncValue;
    ScalarType mObjectiveStagnationMeasure;

    ScalarType mNormConstraints;
    ScalarType mNormObjFuncGrad;
    ScalarType mNormAugLagFuncGrad;
    ScalarType mPenaltyParameter;
    ScalarType mInitialPenaltyParameter;
    ScalarType mPenaltyParameterLowerBound;
    ScalarType mPenaltyParameterScaleFactor;
    ScalarType mDefaultInitialLagrangeMultiplierValue;

    std::vector<OrdinalType> mNumConstraintEvaluations;
    std::vector<OrdinalType> mNumConstraintGradientEvaluations;
    std::vector<OrdinalType> mNumConstraintHessianEvaluations;

    std::shared_ptr<Plato::StateData<ScalarType, OrdinalType>> mStateData;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDualWorkMultiVec;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mControlWorkVec;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLagrangeMultipliers;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialConstraintValues;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentConstraintValues;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousConstraintValues;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentObjFuncGrad;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mPreviousObjFuncGrad;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mCurrentCostraintGrad;
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> mPreviousCostraintGrad;

    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mObjective;
    std::shared_ptr<Plato::CriterionList<ScalarType, OrdinalType>> mConstraints;

    std::shared_ptr<Plato::Preconditioner<ScalarType, OrdinalType>> mPreconditioner;
    std::shared_ptr<Plato::LinearOperator<ScalarType, OrdinalType>> mObjFuncHessian;
    std::shared_ptr<Plato::LinearOperatorList<ScalarType, OrdinalType>> mConstraintHessians;

    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mDualReductionOperations;

    std::shared_ptr<Plato::GradientOperator<ScalarType, OrdinalType>> mObjectiveGradOperator;
    std::shared_ptr<Plato::GradientOperatorList<ScalarType, OrdinalType>> mConstraintGradientOperator;

private:
    AugmentedLagrangianStageMng(const Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>&);
    Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType> & operator=(const Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>&);
};

} // namespace Plato

#endif /* PLATO_AUGMENTEDLAGRANGIANSTAGEMNG_HPP_ */
