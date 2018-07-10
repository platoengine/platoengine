/*
 * Plato_MLKelleySachsAugmentedLagrangian.hpp
 *
 *  Created on: Oct 21, 2017
 */

#ifndef PLATO_MLKELLEYSACHSAUGMENTEDLAGRANGIAN_HPP_
#define PLATO_MLKELLEYSACHSAUGMENTEDLAGRANGIAN_HPP_

#include <cmath>
#include <memory>
#include <cassert>
#include <algorithm>

#include "Plato_Types.hpp"
#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_MLKelleySachsStepMng.hpp"
#include "Plato_MLKelleySachsAlgorithm.hpp"
#include "Plato_ProjectedSteihaugTointPcg.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_AugmentedLagrangianStageMng.hpp"
#include "Plato_MLTrustRegionStepPrediction.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class MLKelleySachsAugmentedLagrangian : public Plato::MLKelleySachsAlgorithm<ScalarType, OrdinalType>
{
public:
    MLKelleySachsAugmentedLagrangian(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory,
                                   const std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> & aDataMng,
                                   const std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType, OrdinalType>> & aStageMng) :
            Plato::MLKelleySachsAlgorithm<ScalarType, OrdinalType>(*aDataFactory),
            mGammaConstant(1),
            mOptimalityTolerance(1e-5),
            mFeasibilityTolerance(1e-4),
            mGradient(aDataFactory->control().create()),
            mPrediction(std::make_shared<Plato::MLTrustRegionStepPrediction<ScalarType, OrdinalType>>(aDataMng)),
            mStepMng(std::make_shared<Plato::MLKelleySachsStepMng<ScalarType, OrdinalType>>(*aDataFactory, mPrediction)),
            mSolver(std::make_shared<Plato::ProjectedSteihaugTointPcg<ScalarType, OrdinalType>>(*aDataFactory)),
            mDataMng(aDataMng),
            mStageMng(aStageMng)
    {
    }
    virtual ~MLKelleySachsAugmentedLagrangian()
    {
    }

    void setOptimalityTolerance(const ScalarType & aInput)
    {
        mOptimalityTolerance = aInput;
    }
    void setFeasibilityTolerance(const ScalarType & aInput)
    {
        mFeasibilityTolerance = aInput;
    }
    void setNormAugmentedLagrangianGradientRelativeToleranceConstant(const ScalarType & aInput)
    {
        mGammaConstant = aInput;
    }
    void setMaxNumTrustRegionSubProblemIterations(const OrdinalType & aInput)
    {
        mStepMng->setMaxNumTrustRegionSubProblemIterations(aInput);
    }

    void solve()
    {
        assert(mDataMng->isInitialGuessSet() == true);

        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = mDataMng->getCurrentControl();
        ScalarType tTolerance = mStepMng->getObjectiveInexactnessTolerance();
        ScalarType tCurrentObjectiveFunctionValue = mStageMng->evaluateObjective(tCurrentControl, tTolerance);
        mDataMng->setCurrentObjectiveFunctionValue(tCurrentObjectiveFunctionValue);
        mStageMng->updateCurrentCriteriaValues();

        mStageMng->cacheData();
        mStageMng->computeGradient(tCurrentControl, *mGradient);
        mDataMng->setCurrentGradient(*mGradient);
        mDataMng->computeNormProjectedGradient();
        mDataMng->cacheCurrentStageData();

        if(mStepMng->isInitialTrustRegionRadiusSetToNormProjectedGradient() == true)
        {
            ScalarType tNormProjectedGradient = mDataMng->getNormProjectedGradient();
            mStepMng->setTrustRegionRadius(tNormProjectedGradient);
        }
        mDataMng->computeStationarityMeasure();

        OrdinalType tIteration = 0;
        while(1)
        {
            tIteration++;
            this->setNumIterationsDone(tIteration);
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
            if(this->checkStoppingCriteria() == true)
            {
                break;
            }
        }
    }

private:
    void update()
    {
        // Store current objective function, control, and gradient values
        mDataMng->cacheCurrentStageData();
        // Cache state data since trial control was accepted
        mStageMng->cacheData();
        // Update inequality constraint values at mid point
        mStageMng->updateCurrentCriteriaValues();
        // Compute gradient at new midpoint
        const Plato::MultiVector<ScalarType, OrdinalType> & tMidControl = mStepMng->getMidPointControls();
        mStageMng->computeGradient(tMidControl, *mGradient);

        if(this->updateControl(*mGradient, *mStepMng, *mDataMng, *mStageMng) == true)
        {
            // Update new gradient and inequality constraint values since control
            // was successfully updated; else, keep mid gradient and thus mid control.
            mStageMng->cacheData();
            mStageMng->updateCurrentCriteriaValues();
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

    bool checkStoppingCriteria()
    {
        bool tStop = false;
        ScalarType tCurrentLagrangeMultipliersPenalty = mStageMng->getPenaltyParameter();
        ScalarType tTolerance = mGammaConstant * tCurrentLagrangeMultipliersPenalty;
        ScalarType tNormAugmentedLagrangianGradient = mDataMng->getNormProjectedGradient();
        if(tNormAugmentedLagrangianGradient <= tTolerance)
        {
            if(this->checkPrimaryStoppingCriteria() == true)
            {
                tStop = true;
            }
            else
            {
                // Update Lagrange multipliers and stop if penalty is below defined threshold/tolerance
                mStageMng->updateLagrangeMultipliers();
            }
        }
        else
        {
            if(this->checkPrimaryStoppingCriteria() == true)
            {
                tStop = true;
            }
        }

        return (tStop);
    }

    bool checkPrimaryStoppingCriteria()
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
            const ScalarType tObjectiveStagnationMeasure = mDataMng->getObjectiveStagnationMeasure();
            const ScalarType tControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
            const ScalarType tStationarityMeasure = mDataMng->getStationarityMeasure();

            if( tStationarityMeasure <= this->getTrialStepTolerance() )
            {
                tStop = true;
                this->setStoppingCriterion(Plato::algorithm::stop_t::NORM_STEP);
            }
            else if( tObjectiveStagnationMeasure < this->getObjectiveStagnationTolerance() )
            {
                tStop = true;
                this->setStoppingCriterion(Plato::algorithm::stop_t::OBJECTIVE_STAGNATION);
            }
            else if( tControlStagnationMeasure < this->getControlStagnationTolerance() )
            {
                tStop = true;
                this->setStoppingCriterion(Plato::algorithm::stop_t::CONTROL_STAGNATION);
            }
            else if( this->getNumIterationsDone() >= this->getMaxNumIterations() )
            {
                tStop = true;
                this->setStoppingCriterion(Plato::algorithm::stop_t::MAX_NUMBER_ITERATIONS);
            }
        }

        return (tStop);
    }

    bool checkNaN()
    {
        const ScalarType tStationarityMeasure = mDataMng->getStationarityMeasure();
        const ScalarType tOptimalityMeasure = mDataMng->getNormProjectedGradient();
        const ScalarType tControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
        const ScalarType tNormProjectedAugmentedLagrangianGradient = mDataMng->getNormProjectedGradient();

        bool tNaN_ValueDetected = false;
        if(std::isfinite(tStationarityMeasure) == false)
        {
            tNaN_ValueDetected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_NORM_TRIAL_STEP);
        }
        else if(std::isfinite(tNormProjectedAugmentedLagrangianGradient) == false)
        {
            tNaN_ValueDetected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_NORM_GRADIENT);
        }
        else if(std::isfinite(tOptimalityMeasure) == false)
        {
            tNaN_ValueDetected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_OBJECTIVE_GRADIENT);
        }
        else if(std::isfinite(tControlStagnationMeasure) == false)
        {
            tNaN_ValueDetected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_CONTROL_STAGNATION);
        }

        return (tNaN_ValueDetected);
    }

private:
    ScalarType mGammaConstant;
    ScalarType mOptimalityTolerance;
    ScalarType mFeasibilityTolerance;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mGradient;

    std::shared_ptr<Plato::MLTrustRegionStepPrediction<ScalarType,OrdinalType>> mPrediction;
    std::shared_ptr<Plato::MLKelleySachsStepMng<ScalarType, OrdinalType>> mStepMng;
    std::shared_ptr<Plato::ProjectedSteihaugTointPcg<ScalarType,OrdinalType>> mSolver;
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType,OrdinalType>> mDataMng;
    std::shared_ptr<Plato::AugmentedLagrangianStageMng<ScalarType,OrdinalType>> mStageMng;

private:
    MLKelleySachsAugmentedLagrangian(const Plato::MLKelleySachsAugmentedLagrangian<ScalarType, OrdinalType> & aRhs);
    Plato::MLKelleySachsAugmentedLagrangian<ScalarType, OrdinalType> & operator=(const Plato::MLKelleySachsAugmentedLagrangian<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_MLKELLEYSACHSAUGMENTEDLAGRANGIAN_HPP_ */
