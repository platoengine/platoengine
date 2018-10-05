/*
 * Plato_KelleySachsAugmentedLagrangian.hpp
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

#ifndef PLATO_KELLEYSACHSAUGMENTEDLAGRANGIAN_HPP_
#define PLATO_KELLEYSACHSAUGMENTEDLAGRANGIAN_HPP_

#include <cmath>
#include <memory>
#include <cassert>
#include <iostream>
#include <algorithm>

#include "Plato_Types.hpp"
#include "Plato_Vector.hpp"
#include "Plato_ErrorChecks.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_KelleySachsStepMng.hpp"
#include "Plato_KelleySachsAlgorithm.hpp"
#include "Plato_ProjectedSteihaugTointPcg.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_AugmentedLagrangianStageMng.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class KelleySachsAugmentedLagrangian : public Plato::KelleySachsAlgorithm<ScalarType, OrdinalType>
{
public:
    KelleySachsAugmentedLagrangian(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory,
                                   const std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> & aDataMng,
                                   const std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>> & aStageMng) :
            Plato::KelleySachsAlgorithm<ScalarType, OrdinalType>(*aDataFactory),
            mGammaConstant(1),
            mMinPenaltyParameter(1e-4),
            mOptimalityTolerance(1e-4),
            mFeasibilityTolerance(1e-4),
            mGradient(aDataFactory->control().create()),
            mCurrentConstraintValues(aDataFactory->dual().create()),
            mStepMng(std::make_shared<Plato::KelleySachsStepMng<ScalarType, OrdinalType>>(*aDataFactory)),
            mSolver(std::make_shared<Plato::ProjectedSteihaugTointPcg<ScalarType, OrdinalType>>(*aDataFactory)),
            mDataMng(aDataMng),
            mStageMng(aStageMng),
            mVerbose(false)
    {
        this->initialize();
    }
    virtual ~KelleySachsAugmentedLagrangian()
    {
    }

    void setMinPenaltyParameter(const ScalarType & aInput)
    {
        mMinPenaltyParameter = aInput;
    }

    void setOptimalityTolerance(const ScalarType & aInput)
    {
        mOptimalityTolerance = aInput;
    }

    void setFeasibilityTolerance(const ScalarType & aInput)
    {
        mFeasibilityTolerance = aInput;
    }

    void setMinTrustRegionRadius(const ScalarType & aInput)
    {
        mStepMng->setMinTrustRegionRadius(aInput);
    }

    void setMaxTrustRegionRadius(const ScalarType & aInput)
    {
        mStepMng->setMaxTrustRegionRadius(aInput);
    }

    void setTrustRegionExpansion(const ScalarType & aInput)
    {
        mStepMng->setTrustRegionExpansion(aInput);
    }

    void setTrustRegionContraction(const ScalarType & aInput)
    {
        mStepMng->setTrustRegionContraction(aInput);
    }

    void setPenaltyParameterScaleFactor(const ScalarType & aInput)
    {
        mStageMng->setPenaltyParameterScaleFactor(aInput);
    }

    void setNormAugmentedLagrangianGradientRelativeToleranceConstant(const ScalarType & aInput)
    {
        mGammaConstant = aInput;
    }

    /******************************************************************************//**
     * @brief Set maximum number of trust region subproblem iterations
     * @param [in] aInput maximum number of trust region subproblem iterations
    **********************************************************************************/
    void setMaxNumTrustRegionSubProblemIterations(const OrdinalType & aInput)
    {
        mStepMng->setMaxNumTrustRegionSubProblemIterations(aInput);
    }

    /******************************************************************************//**
     * @brief Return reference to data manager
     * @return trust region algorithm's data manager
    **********************************************************************************/
    const Plato::TrustRegionAlgorithmDataMng<ScalarType,OrdinalType> & getDataMng() const
    {
        return (*mDataMng);
    }


    /******************************************************************************//**
     * @brief Return reference to trust region step manager
     * @return trust region step manager
    **********************************************************************************/
    const Plato::KelleySachsStepMng<ScalarType,OrdinalType> & getStepMng() const
    {
        return (*mStepMng);
    }

    /******************************************************************************//**
     * @brief Return reference to stage manager
     * @return trust region algorithm's stage manager
    **********************************************************************************/
    const Plato::AugmentedLagrangianStageMng<ScalarType,OrdinalType> & getStageMng() const
    {
        return (*mStageMng);
    }

    /******************************************************************************//**
     * @brief Solve constrained optimization problem
    **********************************************************************************/
    void solve()
    {
        this->checkInitialGuess();

        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = mDataMng->getCurrentControl();
        ScalarType tTolerance = mStepMng->getObjectiveInexactnessTolerance();
        ScalarType tCurrentObjectiveFunctionValue = mStageMng->evaluateObjective(tCurrentControl, tTolerance);
        mDataMng->setCurrentObjectiveFunctionValue(tCurrentObjectiveFunctionValue);
        mStageMng->updateCurrentCriteriaValues();

        mStageMng->cacheData();
        mStageMng->computeGradient(tCurrentControl, *mGradient);
        mDataMng->setCurrentGradient(*mGradient);
        mDataMng->computeNormProjectedGradient();

        if(mStepMng->isInitialTrustRegionRadiusSetToNormProjectedGradient() == true)
        {
            ScalarType tTrustRegionRadius = mDataMng->getNormProjectedGradient();
            const ScalarType tMaxTrustRegionRadius = mStepMng->getMaxTrustRegionRadius();
            if(tTrustRegionRadius > tMaxTrustRegionRadius)
            {
                tTrustRegionRadius = tMaxTrustRegionRadius;
            }
            mStepMng->setTrustRegionRadius(tTrustRegionRadius);
        }
        mDataMng->computeStationarityMeasure();

        OrdinalType tIteration = 0;
        while(1)
        {
            tIteration++;
            this->setNumIterationsDone(tIteration);
            mDataMng->cacheCurrentStageData();
            mStageMng->cacheCurrentCriteriaValues();
            // Compute adaptive constants to ensure superlinear convergence
            ScalarType tStationarityMeasure = mDataMng->getStationarityMeasure();
            ScalarType tValue = std::pow(tStationarityMeasure, static_cast<ScalarType>(0.75));
            ScalarType tEpsilon = std::min(static_cast<ScalarType>(1e-3), tValue);
            mStepMng->setEpsilonConstant(tEpsilon);
            tValue = std::pow(tStationarityMeasure, static_cast<ScalarType>(0.95));
            ScalarType tEta = static_cast<ScalarType>(0.1) * std::min(static_cast<ScalarType>(1e-1), tValue);
            mStepMng->setEtaConstant(tEta);
            // Solve trust region subproblem
            mStepMng->solveSubProblem(*mDataMng, *mStageMng, *mSolver);
            // Update mid objective, control, and gradient information if necessary
            this->update();
            // Update stage manager data
            mStageMng->updateOptimizationData(mDataMng.operator*());
            if(this->checkPenaltyParameter() == true)
            {
                break;
            }
        }
    }

private:
    void initialize()
    {
        // Check Bounds
        try
        {
            const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getControlLowerBounds();
            const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getControlUpperBounds();
            Plato::error::checkBounds(tLowerBounds, tUpperBounds);
        }
        catch(const std::invalid_argument & tErrorMsg)
        {
            std::cout << tErrorMsg.what() << std::flush;
            std::abort();
        }
    }
    void checkInitialGuess()
    {
        try
        {
            bool tIsInitialGuessSet = mDataMng->isInitialGuessSet();
            Plato::error::checkInitialGuessIsSet(tIsInitialGuessSet);
        }
        catch(const std::invalid_argument & tErrorMsg)
        {
            std::cout << tErrorMsg.what() << std::flush;
            std::abort();
        }

        try
        {
            const Plato::MultiVector<ScalarType, OrdinalType> & tControl = mDataMng->getCurrentControl();
            const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getControlLowerBounds();
            const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getControlUpperBounds();
            Plato::error::checkInitialGuess(tControl, tLowerBounds, tUpperBounds);
        }
        catch(const std::invalid_argument & tErrorMsg)
        {
            std::cout << tErrorMsg.what() << std::flush;
            std::abort();
        }

        const Plato::MultiVector<ScalarType, OrdinalType> & tControl = mDataMng->getCurrentControl();
        const Plato::MultiVector<ScalarType, OrdinalType> & tLowerBounds = mDataMng->getControlLowerBounds();
        const Plato::MultiVector<ScalarType, OrdinalType> & tUpperBounds = mDataMng->getControlUpperBounds();
        std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> tWorkMultiVector = tControl.create();
        Plato::update(static_cast<ScalarType>(1), tControl, static_cast<ScalarType>(0), *tWorkMultiVector);
        mDataMng->bounds().project(tLowerBounds, tUpperBounds, *tWorkMultiVector);
        mDataMng->setCurrentControl(*tWorkMultiVector);
    }
    void update()
    {
        // Update objectives and inequalities values
        mStageMng->updateCurrentCriteriaValues();
        // Cache user specific data since trial control was accepted
        mStageMng->cacheData();
        // Compute gradient at new midpoint
        const Plato::MultiVector<ScalarType, OrdinalType> & tMidControl = mStepMng->getMidPointControls();
        mStageMng->computeGradient(tMidControl, *mGradient);

        if(this->updateControl(*mGradient, *mStepMng, *mDataMng, *mStageMng) == true)
        {
            // Update new gradient and inequality constraint values since control
            // was successfully updated; else, keep mid gradient and thus mid control.
            mStageMng->updateCurrentCriteriaValues();
            mStageMng->cacheData();
            const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = mDataMng->getCurrentControl();
            mStageMng->computeGradient(tCurrentControl, *mGradient);
            mDataMng->setCurrentGradient(*mGradient);
        }
        else
        {
            // Keep current objective function, control, and gradient values at mid point
            const ScalarType tMidObjectiveFunctionValue = mStepMng->getMidPointObjectiveFunctionValue();
            mDataMng->setCurrentObjectiveFunctionValue(tMidObjectiveFunctionValue);
            mDataMng->setCurrentControl(tMidControl);
            mDataMng->setCurrentGradient(*mGradient);
        }

        // Compute norm of projected gradient
        mDataMng->computeNormProjectedGradient();
        // Compute stationarity measure
        mDataMng->computeStationarityMeasure();
        // Compute control stagnation measure
        mDataMng->computeControlStagnationMeasure();
        // Compute objective stagnation measure
        mDataMng->computeObjectiveStagnationMeasure();
        // compute gradient inexactness bound
        ScalarType tNormProjectedGradient = mDataMng->getNormProjectedGradient();
        mStepMng->updateGradientInexactnessTolerance(tNormProjectedGradient);
    }

    bool checkPenaltyParameter()
    {
        bool tStop = false;
        mStageMng->getCurrentConstraintValues(*mCurrentConstraintValues);
        const ScalarType tNormConstraintValues = Plato::norm(*mCurrentConstraintValues);
        const ScalarType tDynamicFeasibilityTolerance = mStageMng->getDynamicFeasibilityTolerance();
        const ScalarType tMyFeasibilityTolerance = std::max(mFeasibilityTolerance, tDynamicFeasibilityTolerance);
        if(tNormConstraintValues <= tMyFeasibilityTolerance)
        {

            if(tNormConstraintValues <= mFeasibilityTolerance)
            {
                tStop = this->checkStoppingCriteria();
                return (tStop);
            }
            else
            {
                mStageMng->updateLagrangeMultipliers();
            }
        }
        else
        {
            mStageMng->updatePenaltyParameter();
            const ScalarType tCurrentPenaltyParameter = mStageMng->getPenaltyParameter();
            if(tCurrentPenaltyParameter < mMinPenaltyParameter)
            {
                this->setStoppingCriterion(Plato::algorithm::stop_t::SMALL_PENALTY_PARAMETER);
                tStop = true;
                return (tStop);
            }
        }

        tStop = this->checkStoppingCriteria();
        return (tStop);
    }

    bool checkStoppingCriteria()
    {
        bool tStop = false;
        if(this->checkNaN() == true)
        {
            // Stop optimization algorithm: NaN number detected
            tStop = true;
            mDataMng->resetCurrentStageDataToPreviousStageData();
        }
        else
        {
            // Get termination criteria
            const OrdinalType tNumIterations = this->getNumIterationsDone();
            const ScalarType tActualReduction = mStepMng->getActualReduction();
            const ScalarType tStagnationMeasure = mStageMng->getStagnationMeasure();
            const ScalarType tStationarityMeasure = mDataMng->getStationarityMeasure();
            const ScalarType tCurrentTrustRegionRadius = mStepMng->getTrustRegionRadius();
            const ScalarType tControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
            // Get termination criteria tolerances
            const OrdinalType tMaxNumIterations = this->getMaxNumIterations();
            const ScalarType tStationarityTolerance = this->getStationarityTolerance();
            const ScalarType tMinTrustRegionRadius = mStepMng->getMinTrustRegionRadius();
            const ScalarType tActualReductionTolerance = this->getActualReductionTolerance();
            const ScalarType tControlStagnationTolerance = this->getControlStagnationTolerance();
            const ScalarType tObjectiveStagnationTolerance = this->getObjectiveStagnationTolerance();
            if(tStationarityMeasure <= tStationarityTolerance)
            {
                this->setStoppingCriterion(Plato::algorithm::stop_t::STATIONARITY_MEASURE);
                tStop = true;
            }
            else if( (tNumIterations > static_cast<OrdinalType>(1)) && (std::abs(tActualReduction) < tActualReductionTolerance) )
            {
                this->setStoppingCriterion(Plato::algorithm::stop_t::ACTUAL_REDUCTION_TOLERANCE);
                tStop = true;
            }
            else if( (tNumIterations > static_cast<OrdinalType>(1)) && (tStagnationMeasure < tObjectiveStagnationTolerance) )
            {
                this->setStoppingCriterion(Plato::algorithm::stop_t::OBJECTIVE_STAGNATION);
                tStop = true;
            }
            else if( tCurrentTrustRegionRadius < tMinTrustRegionRadius )
            {
                tStop = true;
                this->setStoppingCriterion(Plato::algorithm::stop_t::SMALL_TRUST_REGION_RADIUS);
            }
            else if( tControlStagnationMeasure < tControlStagnationTolerance )
            {
                tStop = true;
                this->setStoppingCriterion(Plato::algorithm::stop_t::CONTROL_STAGNATION);
            }
            else if( tNumIterations >= tMaxNumIterations )
            {
                tStop = true;
                this->setStoppingCriterion(Plato::algorithm::stop_t::MAX_NUMBER_ITERATIONS);
            }
        }

        return (tStop);
    }

    bool checkNaN()
    {
        const ScalarType tActualReduction = mStepMng->getActualReduction();
        const ScalarType tStagnationMeasure = mStageMng->getStagnationMeasure();
        const ScalarType tStationarityMeasure = mDataMng->getStationarityMeasure();
        const ScalarType tControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
        const ScalarType tNormProjectedAugmentedLagrangianGradient = mDataMng->getNormProjectedGradient();

        bool tNaN_Detected = false;
        if(std::isfinite(tStationarityMeasure) == false)
        {
            tNaN_Detected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_STATIONARITY_MEASURE);
        }
        else if(std::isfinite(tNormProjectedAugmentedLagrangianGradient) == false)
        {
            tNaN_Detected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_NORM_GRADIENT);
        }
        else if(std::isfinite(tActualReduction) == false)
        {
            tNaN_Detected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_ACTUAL_REDUCTION);
        }
        else if(std::isfinite(tStagnationMeasure) == false)
        {
            tNaN_Detected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_OBJECTIVE_STAGNATION);
        }
        else if(std::isfinite(tControlStagnationMeasure) == false)
        {
            tNaN_Detected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_CONTROL_STAGNATION);
        }

        return (tNaN_Detected);
    }

private:
    ScalarType mGammaConstant;
    ScalarType mMinPenaltyParameter;
    ScalarType mOptimalityTolerance;
    ScalarType mFeasibilityTolerance;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mCurrentConstraintValues;

    std::shared_ptr<Plato::KelleySachsStepMng<ScalarType, OrdinalType>> mStepMng;
    std::shared_ptr<Plato::ProjectedSteihaugTointPcg<ScalarType,OrdinalType>> mSolver;
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType,OrdinalType>> mDataMng;
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType,OrdinalType>> mStageMng;

    bool mVerbose;

private:
    KelleySachsAugmentedLagrangian(const Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> & aRhs);
    Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> & operator=(const Plato::KelleySachsAugmentedLagrangian<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_KELLEYSACHSAUGMENTEDLAGRANGIAN_HPP_ */
