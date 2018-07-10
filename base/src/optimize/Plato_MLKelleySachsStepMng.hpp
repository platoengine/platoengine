/*
 * Plato_MLKelleySachsStepMng.hpp
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

#ifndef PLATO_MLKELLEYSACHSSTEPMNG_HPP_
#define PLATO_MLKELLEYSACHSSTEPMNG_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <cassert>
#include <iostream>
#include <stdlib.h>

#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_TrustRegionStepMng.hpp"
#include "Plato_SteihaugTointSolver.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_MLTrustRegionStepPrediction.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class MLKelleySachsStepMng : public Plato::TrustRegionStepMng<ScalarType, OrdinalType>
{
public:
    explicit MLKelleySachsStepMng(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory,
                                  const std::shared_ptr<Plato::MLTrustRegionStepPrediction<ScalarType, OrdinalType>> & aPrediction) :
            Plato::TrustRegionStepMng<ScalarType, OrdinalType>(),
            mEta(0),
            mEpsilon(0),
            mNormInactiveGradient(0),
            mMidPointObjectiveFunction(0),
            mTrustRegionRadiusFlag(false),
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
            mInactiveProjectedTrialStep(aDataFactory.control().create()),
            mPrediction(aPrediction)
    {
        // NOTE: INITIALIZE WORK VECTOR
    }
    virtual ~MLKelleySachsStepMng()
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
        mTrustRegionRadiusFlag = false;
        bool tTrialControlAccepted = true;
        this->setNumTrustRegionSubProblemItrDone(1);
        ScalarType  tMinTrustRegionRadius = this->getMinTrustRegionRadius();
        if(this->getTrustRegionRadius() < tMinTrustRegionRadius)
        {
            this->setTrustRegionRadius(tMinTrustRegionRadius);
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
            // Compute projected trial step
            this->computeProjectedTrialStep(aDataMng);
            // Apply projected trial step to Hessian operator
            this->applyProjectedTrialStepToHessian(aDataMng, aStageMng);
            // Compute predicted reduction based on mid trial control
            ScalarType tPredictedReduction = this->computePredictedReduction(aDataMng);

            if(aDataMng.isObjectiveInexactnessToleranceExceeded() == true)
            {
                tTrialControlAccepted = false;
                break;
            }

            // Update objective function inexactness tolerance (bound)
            this->updateObjectiveInexactnessTolerance(tPredictedReduction);
            // Evaluate current mid objective function
            ScalarType tTolerance = this->getObjectiveInexactnessTolerance();

            bool trust_region_update_radius_return = false;
            ml_trust_region::step_prediction step_prediction = mPrediction->make_prediction();
            if(step_prediction == ml_trust_region::step_prediction::no_prediction
               || step_prediction == ml_trust_region::step_prediction::accept_step)
            {
                // verify accept or no prediction

                mMidPointObjectiveFunction = aStageMng.evaluateObjective(*mMidControls, tTolerance);
                // Compute actual reduction based on mid trial control
                ScalarType tCurrentObjectiveFunctionValue = aDataMng.getCurrentObjectiveFunctionValue();
                ScalarType tActualReduction = mMidPointObjectiveFunction - tCurrentObjectiveFunctionValue;
                this->setActualReduction(tActualReduction);
                // Compute actual over predicted reduction ratio
                ScalarType tActualOverPredReduction = tActualReduction
                                                      / (tPredictedReduction + std::numeric_limits<ScalarType>::epsilon());
                this->setActualOverPredictedReduction(tActualOverPredReduction);
                // Update trust region radius: io_->printTrustRegionSubProblemDiagnostics(aDataMng, aSolver, this);
                trust_region_update_radius_return = this->updateTrustRegionRadius(aDataMng);
            }
            else if(step_prediction == ml_trust_region::step_prediction::reject_step)
            {
                // forcibly reject
                trust_region_update_radius_return = forceContractionUpdate();
            }
            else
            {
                std::cout << "\n\n**** ERROR IN: " << __FILE__ << ", FUNCTION:" << __PRETTY_FUNCTION__ << ", LINE: " << __LINE__
                          << ", MESSAGE: COULD NOT MATCH STEP_PREDICTION (" << step_prediction << "). ABORT. ****\n\n";
                std::abort();
            }

            // if successfully accepting step, end sub problem
            if(trust_region_update_radius_return == true)
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
    void computeProjectedTrialStep(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        // Project trial control
        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = aDataMng.getCurrentControl();
        Plato::update(static_cast<ScalarType>(1), tCurrentControl, static_cast<ScalarType>(0), *mMidControls);
        const Plato::MultiVector<ScalarType, OrdinalType> & tTrialStep = aDataMng.getTrialStep();
        Plato::update(static_cast<ScalarType>(1), tTrialStep, static_cast<ScalarType>(1), *mMidControls);
        const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = aDataMng.getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = aDataMng.getControlUpperBounds();
        aDataMng.bounds().project(tLowerBounds, tUpperBounds, *mMidControls);

        // Compute projected trial step
        Plato::update(static_cast<ScalarType>(1), *mMidControls, static_cast<ScalarType>(0), *mProjectedTrialStep);
        Plato::update(static_cast<ScalarType>(-1), tCurrentControl, static_cast<ScalarType>(1), *mProjectedTrialStep);
    }
    ScalarType computePredictedReduction(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        ScalarType tProjTrialStepDotInactiveGradient = 0;
        ScalarType tProjTrialStepDotHessTimesVector = 0;
        const OrdinalType tNumVectors = aDataMng.getNumControlVectors();
        for(OrdinalType tVectorIndex = 0; tVectorIndex < tNumVectors; tVectorIndex++)
        {
            const Plato::Vector<ScalarType, OrdinalType> & tMyInactiveGradient = mInactiveGradient->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyHessianTimesVector = mHessianTimesVector->operator[](tVectorIndex);
            const Plato::Vector<ScalarType, OrdinalType> & tMyProjectedTrialStep = mProjectedTrialStep->operator[](tVectorIndex);

            tProjTrialStepDotHessTimesVector += tMyProjectedTrialStep.dot(tMyHessianTimesVector);
            tProjTrialStepDotInactiveGradient += tMyProjectedTrialStep.dot(tMyInactiveGradient);
        }

        ScalarType tPredictedReduction = tProjTrialStepDotInactiveGradient
                + (static_cast<ScalarType>(0.5) * tProjTrialStepDotHessTimesVector);
        this->setPredictedReduction(tPredictedReduction);

        return (tPredictedReduction);
    }
    bool updateTrustRegionRadius(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        ScalarType tActualReduction = this->getActualReduction();
        ScalarType tCurrentTrustRegionRadius = this->getTrustRegionRadius();
        ScalarType tActualOverPredReduction = this->getActualOverPredictedReduction();
        ScalarType tActualOverPredMidBound = this->getActualOverPredictedReductionMidBound();
        ScalarType tActualOverPredLowerBound = this->getActualOverPredictedReductionLowerBound();
        ScalarType tActualOverPredUpperBound = this->getActualOverPredictedReductionUpperBound();

        bool tStopTrustRegionSubProblem = false;
        ScalarType tActualReductionLowerBound = this->computeActualReductionLowerBound(aDataMng);
        if(tActualReduction >= tActualReductionLowerBound)
        {
            tCurrentTrustRegionRadius = this->getTrustRegionContraction()
                    * tCurrentTrustRegionRadius;
            mTrustRegionRadiusFlag = true;
            mPrediction->store_prediction(ml_trust_region::step_prediction::reject_step);
        }
        else if(tActualOverPredReduction < tActualOverPredLowerBound)
        {
            tCurrentTrustRegionRadius = this->getTrustRegionContraction()
                    * tCurrentTrustRegionRadius;
            mTrustRegionRadiusFlag = true;
            mPrediction->store_prediction(ml_trust_region::step_prediction::reject_step);
        }
        else if(tActualOverPredReduction >= tActualOverPredLowerBound && tActualOverPredReduction < tActualOverPredMidBound)
        {
            tStopTrustRegionSubProblem = true;
            mPrediction->store_prediction(ml_trust_region::step_prediction::accept_step);
        }
        else if(tActualOverPredReduction >= tActualOverPredMidBound && tActualOverPredReduction < tActualOverPredUpperBound)
        {
            tCurrentTrustRegionRadius = this->getTrustRegionExpansion()
                    * tCurrentTrustRegionRadius;
            tStopTrustRegionSubProblem = true;
            mPrediction->store_prediction(ml_trust_region::step_prediction::accept_step);
        }
        else if(tActualOverPredReduction > tActualOverPredUpperBound && mTrustRegionRadiusFlag == true)
        {
            tCurrentTrustRegionRadius =
                    static_cast<ScalarType>(2) * this->getTrustRegionExpansion() * tCurrentTrustRegionRadius;
            tStopTrustRegionSubProblem = true;
            mPrediction->store_prediction(ml_trust_region::step_prediction::accept_step);
        }
        else
        {
            ScalarType tMaxTrustRegionRadius = this->getMaxTrustRegionRadius();
            tCurrentTrustRegionRadius = this->getTrustRegionExpansion() * tCurrentTrustRegionRadius;
            tCurrentTrustRegionRadius = std::min(tMaxTrustRegionRadius, tCurrentTrustRegionRadius);
            tStopTrustRegionSubProblem = true;
            mPrediction->store_prediction(ml_trust_region::step_prediction::accept_step);
        }
        this->setTrustRegionRadius(tCurrentTrustRegionRadius);

        return (tStopTrustRegionSubProblem);
    }
    bool forceContractionUpdate()
    {
        // when a prediction suggests contraction, forcibly take the contraction
        ScalarType tCurrentTrustRegionRadius = this->getTrustRegionRadius();

        tCurrentTrustRegionRadius = this->getTrustRegionContraction() * tCurrentTrustRegionRadius;
        mTrustRegionRadiusFlag = true;
        this->setTrustRegionRadius(tCurrentTrustRegionRadius);

        bool tStopTrustRegionSubProblem = false;
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

    ScalarType computeActualReductionLowerBound(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
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

    ScalarType computeLambdaScaleFactor(const Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        const Plato::MultiVector<ScalarType, OrdinalType> & tGradient = aDataMng.getCurrentGradient();
        Plato::update(static_cast<ScalarType>(1), tGradient, static_cast<ScalarType>(0), *mWorkMultiVectorTwo);
        const Plato::MultiVector<ScalarType, OrdinalType> & tInactiveSet = aDataMng.getInactiveSet();
        Plato::entryWiseProduct(tInactiveSet, *mWorkMultiVectorTwo);
        ScalarType tNormCurrentProjectedGradient = Plato::norm(*mWorkMultiVectorTwo);

        ScalarType tCondition = 0;
        const ScalarType tCurrentTrustRegionRadius = this->getTrustRegionRadius();
        if(tNormCurrentProjectedGradient > 0)
        {
            tCondition = tCurrentTrustRegionRadius / tNormCurrentProjectedGradient;
        }
        else
        {
            ScalarType tNormProjectedGradient = aDataMng.getNormProjectedGradient();
            tCondition = tCurrentTrustRegionRadius / tNormProjectedGradient;
        }
        ScalarType tLambda = std::min(tCondition, static_cast<ScalarType>(1.));

        return (tLambda);
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

        ScalarType tLambda = this->computeLambdaScaleFactor(aDataMng);
        const Plato::MultiVector<ScalarType, OrdinalType> & tMyGradient = aDataMng.getCurrentGradient();
        Plato::update(-tLambda, tMyGradient, static_cast<ScalarType>(1), *mWorkMultiVectorOne);
        aDataMng.bounds().computeActiveAndInactiveSets(*mWorkMultiVectorOne,
                                                       *mLowerBoundLimit,
                                                       *mUpperBoundLimit,
                                                       *mActiveSet,
                                                       *mInactiveSet);
        aDataMng.setActiveSet(*mActiveSet);
        aDataMng.setInactiveSet(*mInactiveSet);
    }

private:
    ScalarType mEta;
    ScalarType mEpsilon;
    ScalarType mNormInactiveGradient;
    ScalarType mMidPointObjectiveFunction;

    bool mTrustRegionRadiusFlag;

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

    std::shared_ptr<Plato::MLTrustRegionStepPrediction<ScalarType, OrdinalType>> mPrediction;

private:
    MLKelleySachsStepMng(const Plato::MLKelleySachsStepMng<ScalarType, OrdinalType> & aRhs);
    Plato::MLKelleySachsStepMng<ScalarType, OrdinalType> & operator=(const Plato::MLKelleySachsStepMng<ScalarType, OrdinalType> & aRhs);
};

}

#endif /* PLATO_MLKELLEYSACHSSTEPMNG_HPP_ */
