/*
 * Plato_KelleySachsStepMng.hpp
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

#ifndef PLATO_KELLEYSACHSSTEPMNG_HPP_
#define PLATO_KELLEYSACHSSTEPMNG_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_TrustRegionStepMng.hpp"
#include "Plato_SteihaugTointSolver.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class KelleySachsStepMng : public Plato::TrustRegionStepMng<ScalarType, OrdinalType>
{
public:
    explicit KelleySachsStepMng(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory) :
            Plato::TrustRegionStepMng<ScalarType, OrdinalType>(),
            mEta(0),
            mEpsilon(0),
            mNormInactiveGradient(0),
            mMidPointObjectiveFunction(0),
            mRadiusFlag(false),
            mActiveSet(aDataFactory.control().create()),
            mInactiveSet(aDataFactory.control().create()),
            mMidControls(aDataFactory.control().create()),
            mLowerBoundLimit(aDataFactory.control().create()),
            mUpperBoundLimit(aDataFactory.control().create()),
            mInactiveGradient(aDataFactory.control().create()),
            mHessianTimesVector(aDataFactory.control().create()),
            mWorkMultiVectorOne(aDataFactory.control().create()),
            mWorkMultiVectorTwo(aDataFactory.control().create()),
            mProjectedTrialStep(aDataFactory.control().create()),
            mProjectedCauchyStep(aDataFactory.control().create()),
            mActiveProjectedTrialStep(aDataFactory.control().create()),
            mInactiveProjectedTrialStep(aDataFactory.control().create())
    {
        // NOTE: INITIALIZE WORK VECTOR
    }
    virtual ~KelleySachsStepMng()
    {
    }

    //! Returns adaptive constants eta, which ensures superlinear convergence
    ScalarType getEtaConstant() const
    {
        return (mEta);
    }
    //! Sets adaptive constants eta, which ensures superlinear convergence
    void setEtaConstant(const ScalarType & aInput)
    {
        mEta = aInput;
    }
    //! Returns adaptive constants epsilon, which ensures superlinear convergence
    ScalarType getEpsilonConstant() const
    {
        return (mEpsilon);
    }
    //! Sets adaptive constants epsilon, which ensures superlinear convergence
    void setEpsilonConstant(const ScalarType &  aInput)
    {
        mEpsilon = aInput;
    }
    //! Sets objective function value computed with the control values at the mid-point
    void setMidPointObjectiveFunctionValue(const ScalarType & aInput)
    {
        mMidPointObjectiveFunction = aInput;
    }
    //! Returns objective function value computed with the control values at the mid-point
    ScalarType getMidPointObjectiveFunctionValue() const
    {
        return (mMidPointObjectiveFunction);
    }
    //! Sets control values at the mid-point
    void setMidPointControls(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        assert(mMidControls.get() != nullptr);
        assert(aInput.getNumVectors() == mMidControls->getNumVectors());
        Plato::update(static_cast<ScalarType>(1), aInput, static_cast<ScalarType>(0), *mMidControls);
    }
    //! Returns control values at the mid-point
    const Plato::MultiVector<ScalarType, OrdinalType> & getMidPointControls() const
    {
        return (mMidControls.operator*());
    }

    bool solveSubProblem(Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng,
                         Plato::TrustRegionStageMng<ScalarType, OrdinalType> & aStageMng,
                         Plato::SteihaugTointSolver<ScalarType, OrdinalType> & aSolver)
    {
        mRadiusFlag = false;
        bool tTrialControlAccepted = true;
        this->setNumTrustRegionSubProblemItrDone(1);
        ScalarType  tMIN_TRUST_REGION_RADIUS = 1e-4;
        if(this->getTrustRegionRadius() < tMIN_TRUST_REGION_RADIUS)
        {
            this->setTrustRegionRadius(tMIN_TRUST_REGION_RADIUS);
        }

        OrdinalType tMaxNumSubProblemItr = this->getMaxNumTrustRegionSubProblemIterations();
        while(this->getNumTrustRegionSubProblemItrDone() <= tMaxNumSubProblemItr)
        {
            // Compute active and inactive sets
            this->computeActiveAndInactiveSet(aDataMng);
            // Set solver tolerance
            this->setSolverTolerance(aDataMng, aSolver);
            // Compute descent direction
            ScalarType tTrustRegionRadius = this->getTrustRegionRadius();
            aSolver.setTrustRegionRadius(tTrustRegionRadius);
            aSolver.solve(aStageMng, aDataMng);
            // Compute projected mid control
            this->computeProjectedMidControl(aDataMng);
            // Compute projected trial step
            this->computeProjectedTrialStep(aDataMng);
            // Apply projected trial step to Hessian operator
            this->applyProjectedTrialStepToHessian(aDataMng, aStageMng);
            // Compute predicted reduction based on mid trial control
            ScalarType tPredictedReduction = this->computePredictedReduction(aDataMng, aStageMng);

            if(aDataMng.isObjectiveInexactnessToleranceExceeded() == true)
            {
                tTrialControlAccepted = false;
                break;
            }

            // Update objective function inexactness tolerance (bound)
            this->updateObjectiveInexactnessTolerance(tPredictedReduction);
            // Evaluate current mid objective function
            ScalarType tTolerance = this->getObjectiveInexactnessTolerance();
            mMidPointObjectiveFunction = aStageMng.evaluateObjective(*mMidControls, tTolerance);
            // Compute actual reduction based on mid trial control
            ScalarType tCurrentObjectiveFunctionValue = aDataMng.getCurrentObjectiveFunctionValue();
            ScalarType tActualReduction = mMidPointObjectiveFunction - tCurrentObjectiveFunctionValue;
            this->setActualReduction(tActualReduction);
            // Compute actual over predicted reduction ratio
            ScalarType tActualOverPredReduction = tActualReduction /
                    (tPredictedReduction + std::numeric_limits<ScalarType>::epsilon());
            this->setActualOverPredictedReduction(tActualOverPredReduction);
            // Update trust region radius: io_->printTrustRegionSubProblemDiagnostics(aDataMng, aSolver, this);
            if(this->updateTrustRegionRadius(aDataMng) == true)
            {
                break;
            }
            this->updateNumTrustRegionSubProblemItrDone();
        }
        return (tTrialControlAccepted);
    }

private:
    void setSolverTolerance(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng,
                            Plato::SteihaugTointSolver<ScalarType, OrdinalType> & aSolver)
    {
        ScalarType tCummulativeDotProduct = 0;
        const OrdinalType tNumVectors = aDataMng.getNumControlVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyInactiveSet = aDataMng.getInactiveSet(tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyCurrentGradient = aDataMng.getCurrentGradient(tVectorIndex);
            Plato::Vector<ScalarType, OrdinalType> & tMyInactiveGradient = (*mInactiveGradient)[tVectorIndex];

            tMyInactiveGradient.update(static_cast<ScalarType>(1), tMyCurrentGradient, static_cast<ScalarType>(0));
            tMyInactiveGradient.entryWiseProduct(tMyInactiveSet);
            tCummulativeDotProduct += tMyInactiveGradient.dot(tMyInactiveGradient);
        }
        mNormInactiveGradient = std::sqrt(tCummulativeDotProduct);
        ScalarType tSolverStoppingTolerance = this->getEtaConstant() * mNormInactiveGradient;
        aSolver.setSolverTolerance(tSolverStoppingTolerance);
    }
    void computeProjectedMidControl(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = aDataMng.getCurrentControl();
        Plato::update(static_cast<ScalarType>(1), tCurrentControl, static_cast<ScalarType>(0), *mMidControls);
        const Plato::MultiVector<ScalarType, OrdinalType> & tTrialStep = aDataMng.getTrialStep();
        Plato::update(static_cast<ScalarType>(1), tTrialStep, static_cast<ScalarType>(1), *mMidControls);
        const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = aDataMng.getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = aDataMng.getControlUpperBounds();
        aDataMng.bounds().project(tLowerBounds, tUpperBounds, *mMidControls);
    }
    void computeProjectedTrialStep(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = aDataMng.getCurrentControl();
        Plato::update(static_cast<ScalarType>(1), *mMidControls, static_cast<ScalarType>(0), *mProjectedTrialStep);
        Plato::update(static_cast<ScalarType>(-1), tCurrentControl, static_cast<ScalarType>(1), *mProjectedTrialStep);
    }
    ScalarType computePredictedReduction(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng,
                                         Plato::TrustRegionStageMng<ScalarType, OrdinalType> & aStageMng)
    {
        const bool tHaveHessian = aStageMng.getHaveHessian();
        if(!tHaveHessian)
        {
            // predicted reduction = gradient'*step
            // without Hessian information, best prediction is linear model (not quadratic).

            ScalarType tGradientBasedPredictedReduction = Plato::dot(*mInactiveGradient, *mProjectedTrialStep);
            this->setPredictedReduction(tGradientBasedPredictedReduction);
            return tGradientBasedPredictedReduction;
        }

        const OrdinalType tNumVectors = aDataMng.getNumControlVectors();
        ScalarType tProjTrialStepDotInactiveGradient = 0;
        ScalarType tProjTrialStepDotHessTimesVector = 0;
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyInactiveGradient = mInactiveGradient->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyHessianTimesVector = mHessianTimesVector->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyProjectedTrialStep = mProjectedTrialStep->operator[](tVectorIndex);

            tProjTrialStepDotHessTimesVector += tMyProjectedTrialStep.dot(tMyHessianTimesVector);
            tProjTrialStepDotInactiveGradient += tMyProjectedTrialStep.dot(tMyInactiveGradient);
        }

        // predicted reduction = gradient'*step + (1/2)*step'*Hessian*step
        ScalarType tPredictedReduction = tProjTrialStepDotInactiveGradient
                + (static_cast<ScalarType>(0.5) * tProjTrialStepDotHessTimesVector);

        this->setPredictedReduction(tPredictedReduction);
        return (tPredictedReduction);
    }
    bool updateTrustRegionRadius(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const ScalarType tActualReduction = this->getActualReduction();
        const ScalarType tMaxTrustRegionRadius = this->getMaxTrustRegionRadius();
        const ScalarType tTrustRegionExpansion = this->getTrustRegionExpansion();
        const ScalarType tTrustRegionContraction = this->getTrustRegionContraction();
        const ScalarType tActualOverPredReduction = this->getActualOverPredictedReduction();
        const ScalarType tActualOverPredMidBound = this->getActualOverPredictedReductionMidBound();
        const ScalarType tActualOverPredLowerBound = this->getActualOverPredictedReductionLowerBound();
        const ScalarType tActualOverPredUpperBound = this->getActualOverPredictedReductionUpperBound();
        const ScalarType tSufficientDecreaseCondition = this->computeSufficientDecreaseCondition(aDataMng);
        // Set termination criteria for trust region subproblem
        bool tIsRatioBelowMidBound = tActualOverPredReduction < tActualOverPredMidBound;
        bool tIsRatioAboveMidBound = tActualOverPredReduction >= tActualOverPredMidBound;
        bool tIsRatioBelowLowerBound = tActualOverPredReduction < tActualOverPredLowerBound;
        bool tIsRatioAboveLowerBound = tActualOverPredReduction >= tActualOverPredLowerBound;
        bool tIsRatioBelowUpperBound = tActualOverPredReduction < tActualOverPredUpperBound;
        bool tIsRatioAboveUpperBound = tActualOverPredReduction >= tActualOverPredUpperBound;
        bool tIsRatioAboveMidBoundAndBelowUpperBound = tIsRatioAboveMidBound && tIsRatioBelowUpperBound;
        bool tIsRadiusFlagTrueAndRatioAboveUpperBound = (mRadiusFlag == true) && tIsRatioAboveUpperBound;
        bool tIsActualReductionAboveSufficientDecreaseCondition = tActualReduction >= tSufficientDecreaseCondition;
        // Check termination criteria for trust region subproblem
        bool tStopTrustRegionSubProblem = false;
        ScalarType tCurrentTrustRegionRadius = this->getTrustRegionRadius();
        bool tIsTurstRegionRadiusEqualMaxRadius = tCurrentTrustRegionRadius == tMaxTrustRegionRadius;
        if(tIsRatioBelowLowerBound || tIsActualReductionAboveSufficientDecreaseCondition)
        {
            tCurrentTrustRegionRadius = tTrustRegionContraction * tCurrentTrustRegionRadius;
            mRadiusFlag = true;
        }
        else if(tIsRatioAboveLowerBound && tIsRatioBelowMidBound)
        {
            tCurrentTrustRegionRadius = tTrustRegionContraction * tCurrentTrustRegionRadius;
            tStopTrustRegionSubProblem = true;
        }
        else if(tIsRatioAboveMidBound && tIsTurstRegionRadiusEqualMaxRadius)
        {
            tStopTrustRegionSubProblem = true;
        }
        else if(tIsRatioAboveMidBoundAndBelowUpperBound || tIsRadiusFlagTrueAndRatioAboveUpperBound)
        {
//            tCurrentTrustRegionRadius = tTrustRegionExpansion * tCurrentTrustRegionRadius;
            tStopTrustRegionSubProblem = true;
        }
        else if(tIsRatioAboveUpperBound && (mRadiusFlag == false))
        {
            tCurrentTrustRegionRadius = tTrustRegionExpansion * tCurrentTrustRegionRadius;
            tCurrentTrustRegionRadius = std::min(tMaxTrustRegionRadius, tCurrentTrustRegionRadius);
//            tStopTrustRegionSubProblem = true;
        }
        // Check if trust region radius is below allowable tolerance
        const ScalarType tMinTrustRegionRadius = this->getMinTrustRegionRadius();
        if(tCurrentTrustRegionRadius < tMinTrustRegionRadius)
        {
            tStopTrustRegionSubProblem = true;
        }
        this->setTrustRegionRadius(tCurrentTrustRegionRadius);

        return (tStopTrustRegionSubProblem);
    }

    void applyProjectedTrialStepToHessian(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng,
                                          Plato::TrustRegionStageMng<ScalarType, OrdinalType> & aStageMng)
    {
        // Compute active projected trial step
        Plato::fill(static_cast<ScalarType>(0), *mHessianTimesVector);
        Plato::update(static_cast<ScalarType>(1),
                      *mProjectedTrialStep,
                      static_cast<ScalarType>(0),
                      *mActiveProjectedTrialStep);
        const Plato::MultiVector<ScalarType, OrdinalType> & tActiveSet = aDataMng.getActiveSet();
        Plato::entryWiseProduct(tActiveSet, *mActiveProjectedTrialStep);

        // Compute inactive projected trial step
        Plato::update(static_cast<ScalarType>(1),
                      *mProjectedTrialStep,
                      static_cast<ScalarType>(0),
                      *mInactiveProjectedTrialStep);
        const Plato::MultiVector<ScalarType, OrdinalType> & tInactiveSet = aDataMng.getInactiveSet();
        Plato::entryWiseProduct(tInactiveSet, *mInactiveProjectedTrialStep);

        // Apply inactive projected trial step to Hessian
        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = aDataMng.getCurrentControl();
        aStageMng.applyVectorToHessian(tCurrentControl, *mInactiveProjectedTrialStep, *mHessianTimesVector);

        // Compute Hessian times projected trial step, i.e. ( ActiveSet + (InactiveSet' * Hess * InactiveSet) ) * Vector
        Plato::entryWiseProduct(tInactiveSet, *mHessianTimesVector);
        Plato::update(static_cast<ScalarType>(1),
                      *mActiveProjectedTrialStep,
                      static_cast<ScalarType>(1),
                      *mHessianTimesVector);
    }

    ScalarType computeSufficientDecreaseCondition(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        ScalarType tConditionOne = this->getTrustRegionRadius()
                / (mNormInactiveGradient + std::numeric_limits<ScalarType>::epsilon());
        ScalarType tLambda = std::min(tConditionOne, static_cast<ScalarType>(1.));

        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = aDataMng.getCurrentControl();
        Plato::update(static_cast<ScalarType>(1), tCurrentControl, static_cast<ScalarType>(0), *mWorkMultiVectorOne);
        Plato::update(-tLambda, *mInactiveGradient, static_cast<ScalarType>(1), *mWorkMultiVectorOne);

        const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = aDataMng.getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = aDataMng.getControlUpperBounds();
        aDataMng.bounds().project(tLowerBounds, tUpperBounds, *mWorkMultiVectorOne);

        Plato::update(static_cast<ScalarType>(1), tCurrentControl, static_cast<ScalarType>(0), *mProjectedCauchyStep);
        Plato::update(static_cast<ScalarType>(-1), *mWorkMultiVectorOne, static_cast<ScalarType>(1), *mProjectedCauchyStep);

        const ScalarType tSLOPE_CONSTANT = 1e-4;
        ScalarType tNormProjectedCauchyStep = Plato::norm(*mProjectedCauchyStep);
        ScalarType tCurrentStationarityMeasure = aDataMng.getStationarityMeasure();
        ScalarType tLowerBound = -tCurrentStationarityMeasure * tSLOPE_CONSTANT * tNormProjectedCauchyStep;

        return (tLowerBound);
    }

    void computeActiveAndInactiveSet(Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> & tMyLowerBound = aDataMng.getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> & tMyUpperBound = aDataMng.getControlUpperBounds();
        // Compute current lower bound limit
        Plato::fill(mEpsilon, *mWorkMultiVectorOne);
        Plato::update(static_cast<ScalarType>(1), tMyLowerBound, static_cast<ScalarType>(0), *mLowerBoundLimit);
        Plato::update(static_cast<ScalarType>(-1), *mWorkMultiVectorOne, static_cast<ScalarType>(1), *mLowerBoundLimit);
        // Compute current upper bound limit
        Plato::update(static_cast<ScalarType>(1), tMyUpperBound, static_cast<ScalarType>(0), *mUpperBoundLimit);
        Plato::update(static_cast<ScalarType>(1), *mWorkMultiVectorOne, static_cast<ScalarType>(1), *mUpperBoundLimit);

        // Compute active and inactive sets
        const Plato::MultiVector<ScalarType, OrdinalType> & tMyCurrentControl = aDataMng.getCurrentControl();
        Plato::update(static_cast<ScalarType>(1), tMyCurrentControl, static_cast<ScalarType>(0), *mWorkMultiVectorOne);

        const Plato::MultiVector<ScalarType, OrdinalType> & tMyGradient = aDataMng.getCurrentGradient();
        Plato::update(static_cast<ScalarType>(-1), tMyGradient, static_cast<ScalarType>(1), *mWorkMultiVectorOne);
        aDataMng.bounds().computeActiveAndInactiveSets(*mWorkMultiVectorOne,
                                                       *mLowerBoundLimit,
                                                       *mUpperBoundLimit,
                                                       *mActiveSet,
                                                       *mInactiveSet);
        /* Check if all the controls are inside the active set, if true, activate
         *  inactive set and let the projection handle the box constraints. */
        const ScalarType tNormInactiveSet = Plato::norm(*mInactiveSet);
        if(tNormInactiveSet <= static_cast<ScalarType>(0))
        {
            Plato::fill(static_cast<ScalarType>(0), *mActiveSet);
            Plato::fill(static_cast<ScalarType>(1), *mInactiveSet);
        }
        aDataMng.setActiveSet(*mActiveSet);
        aDataMng.setInactiveSet(*mInactiveSet);
    }

private:
    ScalarType mEta;
    ScalarType mEpsilon;
    ScalarType mNormInactiveGradient;
    ScalarType mMidPointObjectiveFunction;

    bool mRadiusFlag;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mActiveSet;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mInactiveSet;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mMidControls;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mLowerBoundLimit;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mUpperBoundLimit;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mInactiveGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mHessianTimesVector;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mWorkMultiVectorOne;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mWorkMultiVectorTwo;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mProjectedTrialStep;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mProjectedCauchyStep;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mActiveProjectedTrialStep;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mInactiveProjectedTrialStep;

private:
    KelleySachsStepMng(const Plato::KelleySachsStepMng<ScalarType, OrdinalType> & aRhs);
    Plato::KelleySachsStepMng<ScalarType, OrdinalType> & operator=(const Plato::KelleySachsStepMng<ScalarType, OrdinalType> & aRhs);
};

}

#endif /* PLATO_KELLEYSACHSSTEPMNG_HPP_ */
