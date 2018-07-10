/*
 * Plato_KelleySachsBoundConstrained.hpp
 *
 *  Created on: Dec 20, 2017
 */

#ifndef PLATO_KELLEYSACHSBOUNDCONSTRAINED_HPP_
#define PLATO_KELLEYSACHSBOUNDCONSTRAINED_HPP_

#include <cmath>
#include <memory>
#include <cassert>
#include <iostream>
#include <algorithm>

#include "Plato_Types.hpp"
#include "Plato_ErrorChecks.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_KelleySachsStepMng.hpp"
#include "Plato_KelleySachsAlgorithm.hpp"
#include "Plato_ProjectedSteihaugTointPcg.hpp"
#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_ReducedSpaceTrustRegionStageMng.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class KelleySachsBoundConstrained : public Plato::KelleySachsAlgorithm<ScalarType, OrdinalType>
{
public:
    KelleySachsBoundConstrained(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aDataFactory,
                                const std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> & aDataMng,
                                const std::shared_ptr<Plato::ReducedSpaceTrustRegionStageMng<ScalarType, OrdinalType>> & aStageMng) :
            Plato::KelleySachsAlgorithm<ScalarType, OrdinalType>(*aDataFactory),
            mOptimalityTolerance(1e-5),
            mGradient(aDataFactory->control().create()),
            mStepMng(std::make_shared<Plato::KelleySachsStepMng<ScalarType, OrdinalType>>(*aDataFactory)),
            mSolver(std::make_shared<Plato::ProjectedSteihaugTointPcg<ScalarType, OrdinalType>>(*aDataFactory)),
            mDataMng(aDataMng),
            mStageMng(aStageMng)
    {
        this->initialize();
    }
    virtual ~KelleySachsBoundConstrained()
    {
    }

    /*! Set optimality tolerance, i.e. \Vert\nabla_{\mathbf{z}}F\Vert < \epsilon, where \nabla_{\mathbf{z}}
     *  F = \sum_{i=1}^{n}\nabla_{\mathbf{z}}f_{i}(\mathbf{z})\ \forall\ i=1,\dots,n. Here, \mathbf{z}
     *  denotes the set of controls, f_i(\mathbf{z}) is an objective function, $n\in\mathcal{N}$ is the
     *  number of objective functions and \epsilon > 0 denotes a user-defined tolerance. */
    void setOptimalityTolerance(const ScalarType & aInput)
    {
        mOptimalityTolerance = aInput;
    }
    void setMaxNumTrustRegionSubProblemIterations(const OrdinalType & aInput)
    {
        mStepMng->setMaxNumTrustRegionSubProblemIterations(aInput);
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

    void solve()
    {
        this->checkInitialGuess();

        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = mDataMng->getCurrentControl();
        ScalarType tTolerance = mStepMng->getObjectiveInexactnessTolerance();
        ScalarType tCurrentObjectiveFunctionValue = mStageMng->evaluateObjective(tCurrentControl, tTolerance);
        mDataMng->setCurrentObjectiveFunctionValue(tCurrentObjectiveFunctionValue);

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
        // Cache state data since trial control was accepted
        mStageMng->cacheData();
        // Compute gradient at new midpoint
        const Plato::MultiVector<ScalarType, OrdinalType> & tMidControl = mStepMng->getMidPointControls();
        mStageMng->computeGradient(tMidControl, *mGradient);

        if(this->updateControl(*mGradient, *mStepMng, *mDataMng, *mStageMng) == true)
        {
            // Update new gradient and inequality constraint values since control
            // was successfully updated; else, keep mid gradient and thus mid control.
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

        // Compute stationarity measure
        mDataMng->computeStationarityMeasure();
        // Compute control stagnation measure
        mDataMng->computeControlStagnationMeasure();
        // Compute objective stagnation measure
        mDataMng->computeObjectiveStagnationMeasure();
        // compute gradient inexactness bound for next trust region sub-problem solve
        ScalarType tNormProjectedGradient = mDataMng->getNormProjectedGradient();
        mStepMng->updateGradientInexactnessTolerance(tNormProjectedGradient);
    }
    /*! Check stopping criteria. Return true if a stopping criteria is met; otherwise, return false and
     *  solve the trust region sub-problem to compute a new trial control. Repeat until one stopping
     *  criteria is satisfied. */
    bool checkStoppingCriteria()
    {
        bool tStop = false;
        if(this->checkNaN() == true)
        {
            // Stop optimization algorithm: NaN criterion measure detected
            tStop = true;
            mDataMng->resetCurrentStageDataToPreviousStageData();
        }
        else
        {
            // Get termination criteria
            const OrdinalType tNumIterations = this->getNumIterationsDone();
            const ScalarType tActualReduction = mStepMng->getActualReduction();
            const ScalarType tStationarityMeasure = mDataMng->getStationarityMeasure();
            const ScalarType tCurrentTrustRegionRadius = mStepMng->getTrustRegionRadius();
            const ScalarType tStagnationMeasure = mDataMng->getObjectiveStagnationMeasure();
            const ScalarType tControlStagnationMeasure = mDataMng->getControlStagnationMeasure();
            // Get termination criteria tolerances
            const OrdinalType tMaxNumIterations = this->getMaxNumIterations();
            const ScalarType tStagnationTolerance = this->getStagnationTolerance();
            const ScalarType tStationarityTolerance = this->getStationarityTolerance();
            const ScalarType tMinTrustRegionRadius = mStepMng->getMinTrustRegionRadius();
            const ScalarType tActualReductionTolerance = this->getActualReductionTolerance();
            const ScalarType tControlStagnationTolerance = this->getControlStagnationTolerance();
            if( tStationarityMeasure <= tStationarityTolerance )
            {
                tStop = true;
                this->setStoppingCriterion(Plato::algorithm::stop_t::STATIONARITY_MEASURE);
            }
            else if( (tNumIterations > static_cast<OrdinalType>(1)) && (std::abs(tActualReduction) < tActualReductionTolerance) )
            {
                this->setStoppingCriterion(Plato::algorithm::stop_t::ACTUAL_REDUCTION_TOLERANCE);
                tStop = true;
            }
            else if( tStagnationMeasure < tStagnationTolerance )
            {
                tStop = true;
                this->setStoppingCriterion(Plato::algorithm::stop_t::OBJECTIVE_STAGNATION);
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
    //! Check for any NaN stopping criteria measures.
    bool checkNaN()
    {
        const ScalarType tStationarityMeasure = mDataMng->getStationarityMeasure();
        const ScalarType tOptimalityMeasure = mDataMng->getNormProjectedGradient();
        const ScalarType tControlStagnationMeasure = mDataMng->getControlStagnationMeasure();

        bool tNaN_ValueDetected = false;
        if(std::isfinite(tStationarityMeasure) == false)
        {
            tNaN_ValueDetected = true;
            this->setStoppingCriterion(Plato::algorithm::stop_t::NaN_STATIONARITY_MEASURE);
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
    ScalarType mOptimalityTolerance;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mGradient;
    std::shared_ptr<Plato::KelleySachsStepMng<ScalarType, OrdinalType>> mStepMng;
    std::shared_ptr<Plato::ProjectedSteihaugTointPcg<ScalarType,OrdinalType>> mSolver;
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType,OrdinalType>> mDataMng;
    std::shared_ptr<Plato::ReducedSpaceTrustRegionStageMng<ScalarType,OrdinalType>> mStageMng;

private:
    KelleySachsBoundConstrained(const Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType>&);
    Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType> & operator=(const Plato::KelleySachsBoundConstrained<ScalarType, OrdinalType>&);
}; // class KelleySachsBoundConstrained

} // namespace Plato

#endif /* PLATO_KELLEYSACHSBOUNDCONSTRAINED_HPP_ */
