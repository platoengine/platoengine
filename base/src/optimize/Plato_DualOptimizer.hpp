/*
 * Plato_DualOptimizer.hpp
 *
 *  Created on: Dec 1, 2017
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

#ifndef PLATO_DUALOPTIMIZER_HPP_
#define PLATO_DUALOPTIMIZER_HPP_

#include <cmath>
#include <vector>
#include <memory>
#include <limits>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_HostBounds.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_DualProblemStageMng.hpp"
#include "Plato_ConservativeConvexSeparableAppxDataMng.hpp"

namespace Plato
{

enum gcmma_nlcg
{
    FLETCHER_REEVES = 1,
    POLAK_RIBIERE = 2,
    HESTENES_STIEFEL = 3,
    DAI_YUAN = 4,
};

template<typename ScalarType, typename OrdinalType>
class DualOptimizer
{
public:
    DualOptimizer(const Plato::DataFactory<ScalarType, OrdinalType> & aFactory) :
            mType(Plato::gcmma_nlcg::POLAK_RIBIERE),
            mLineSearchStepLowerBound(1e-3),
            mLineSearchStepUpperBound(0.5),
            mLineSearchIterationCount(0),
            mMaxNumLineSearchIterations(5),
            mNumIterationsDone(0),
            mMaxNumIterations(10),
            mGradientTolerance(1e-8),
            mTrialStepTolerance(1e-8),
            mObjectiveStagnationTolerance(1e-8),
            mNewObjectiveFunctionValue(std::numeric_limits<ScalarType>::max()),
            mOldObjectiveFunctionValue(std::numeric_limits<ScalarType>::max()),
            mNewDual(aFactory.dual().create()),
            mOldDual(aFactory.dual().create()),
            mNewGradient(aFactory.dual().create()),
            mOldGradient(aFactory.dual().create()),
            mNewTrialStep(aFactory.dual().create()),
            mOldTrialStep(aFactory.dual().create()),
            mNewSteepestDescent(aFactory.dual().create()),
            mOldSteepestDescent(aFactory.dual().create()),
            mTrialDual(aFactory.dual().create()),
            mProjectedStep(aFactory.dual().create()),
            mDualLowerBound(aFactory.dual().create()),
            mDualUpperBound(aFactory.dual().create()),
            mBounds(std::make_shared<Plato::HostBounds<ScalarType, OrdinalType>>()),
            mDualProbStageMng(std::make_shared<Plato::DualProblemStageMng<ScalarType, OrdinalType>>(aFactory))

    {
        Plato::fill(static_cast<ScalarType>(0), *mDualLowerBound);
        Plato::fill(std::numeric_limits<ScalarType>::max(), *mDualUpperBound);
    }
    ~DualOptimizer()
    {
    }

    void setDaiYuanMethod()
    {
        mType = Plato::gcmma_nlcg::DAI_YUAN;
    }
    void setPolakRibiereMethod()
    {
        mType = Plato::gcmma_nlcg::POLAK_RIBIERE;
    }
    void setFletcherReevesMethod()
    {
        mType = Plato::gcmma_nlcg::FLETCHER_REEVES;
    }
    void setHestenesStiefelMethod()
    {
        mType = Plato::gcmma_nlcg::HESTENES_STIEFEL;
    }

    OrdinalType getIterationCount() const
    {
        return (mNumIterationsDone);
    }

    void updateIterationCount()
    {
        mNumIterationsDone++;
    }

    OrdinalType getMaxNumIterations() const
    {
        return (mMaxNumIterations);
    }

    void setMaxNumIterations(const OrdinalType & input_)
    {
        mMaxNumIterations = input_;
    }

    ScalarType getGradientTolerance() const
    {
        return (mGradientTolerance);
    }

    void setGradientTolerance(const ScalarType &  tolerance_)
    {
        mGradientTolerance = tolerance_;
    }

    ScalarType getObjectiveStagnationTolerance() const
    {
        return (mObjectiveStagnationTolerance);
    }

    void setObjectiveStagnationTolerance(const ScalarType &  tolerance_)
    {
        mObjectiveStagnationTolerance = tolerance_;
    }

    ScalarType getTrialStepTolerance() const
    {
        return (mTrialStepTolerance);
    }

    void setTrialStepTolerance(const ScalarType & tolerance_)
    {
        mTrialStepTolerance = tolerance_;
    }

    ScalarType getLineSearchStepLowerBound() const
    {
        return (mLineSearchStepLowerBound);
    }

    void setLineSearchStepLowerBound(const ScalarType & aInput)
    {
        mLineSearchStepLowerBound = aInput;
    }

    ScalarType getLineSearchStepUpperBound() const
    {
        return (mLineSearchStepUpperBound);
    }

    void setLineSearchStepUpperBound(const ScalarType & aInput)
    {
        mLineSearchStepUpperBound = aInput;
    }

    OrdinalType getLineSearchIterationCount() const
    {
        return (mLineSearchIterationCount);
    }

    void updateLineSearchIterationCount()
    {
        mLineSearchIterationCount++;
    }

    OrdinalType getMaxNumLineSearchIterations() const
    {
        return (mMaxNumLineSearchIterations);
    }

    void setMaxNumLineSearchIterations(const OrdinalType & aInput)
    {
        mMaxNumLineSearchIterations = aInput;
    }

    void update(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mDualProbStageMng->update(aDataMng);
    }
    void updateObjectiveCoefficients(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mDualProbStageMng->updateObjectiveCoefficients(aDataMng);
    }
    void updateConstraintCoefficients(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mDualProbStageMng->updateConstraintCoefficients(aDataMng);
    }
    void initializeAuxiliaryVariables(Plato::ConservativeConvexSeparableAppxDataMng<ScalarType, OrdinalType> & aDataMng)
    {
        mDualProbStageMng->initializeAuxiliaryVariables(aDataMng);
    }

    void solve(Plato::MultiVector<ScalarType,OrdinalType> & aDual,
               Plato::MultiVector<ScalarType,OrdinalType> & aTrialControl)
    {
        mNumIterationsDone = 0;
        Plato::update(static_cast<ScalarType>(1), aDual, static_cast<ScalarType>(0), *mNewDual);

        mNewObjectiveFunctionValue = mDualProbStageMng->evaluateObjective(*mNewDual);
        mDualProbStageMng->computeGradient(*mNewDual, *mNewGradient);
        Plato::update(static_cast<ScalarType>(-1), *mNewGradient, static_cast<ScalarType>(0), *mNewSteepestDescent);

        Plato::update(static_cast<ScalarType>(1), *mNewDual, static_cast<ScalarType>(0), *mTrialDual);
        Plato::update(static_cast<ScalarType>(1), *mNewSteepestDescent, static_cast<ScalarType>(1), *mTrialDual);
        this->computeProjectedGradient(*mTrialDual, *mDualLowerBound, *mDualUpperBound, *mNewSteepestDescent);
        Plato::update(static_cast<ScalarType>(1), *mNewSteepestDescent, static_cast<ScalarType>(0), *mNewTrialStep);

        this->storeCurrentState();
        this->quadraticLineSearch();

        if(this->stoppingCriteriaSatisfied() == true)
        {
            Plato::update(static_cast<ScalarType>(1), *mNewDual, static_cast<ScalarType>(0), aDual);
            mDualProbStageMng->getTrialControl(aTrialControl);
            return;
        }

        while(1)
        {
            mDualProbStageMng->computeGradient(*mNewDual, *mNewGradient);
            Plato::update(static_cast<ScalarType>(-1), *mNewGradient, static_cast<ScalarType>(0), *mNewSteepestDescent);

            Plato::update(static_cast<ScalarType>(1), *mNewDual, static_cast<ScalarType>(0), *mTrialDual);
            Plato::update(static_cast<ScalarType>(1), *mNewSteepestDescent, static_cast<ScalarType>(1), *mTrialDual);
            this->computeProjectedGradient(*mTrialDual, *mDualLowerBound, *mDualUpperBound, *mNewSteepestDescent);

            ScalarType tScaleParameter = this->computeScaling();
            if(std::isfinite(tScaleParameter) == false)
            {
                break;
            }

            Plato::update(static_cast<ScalarType>(1), *mNewSteepestDescent, static_cast<ScalarType>(0), *mNewTrialStep);
            Plato::update(tScaleParameter, *mOldTrialStep, static_cast<ScalarType>(1), *mNewTrialStep);

            this->storeCurrentState();
            this->quadraticLineSearch();

            mNumIterationsDone++;
            if(this->stoppingCriteriaSatisfied() == true)
            {
                break;
            }
        }

        Plato::update(static_cast<ScalarType>(1), *mNewDual, static_cast<ScalarType>(0), aDual);
        mDualProbStageMng->getTrialControl(aTrialControl);
    }

    void reset()
    {
        const ScalarType tValue = 0;
        mNewObjectiveFunctionValue = tValue;
        mOldObjectiveFunctionValue = tValue;

        Plato::fill(tValue, *mNewDual);
        Plato::fill(tValue, *mOldDual);
        Plato::fill(tValue, *mNewGradient);
        Plato::fill(tValue, *mOldGradient);
        Plato::fill(tValue, *mNewTrialStep);
        Plato::fill(tValue, *mOldTrialStep);
        Plato::fill(tValue, *mNewSteepestDescent);
        Plato::fill(tValue, *mOldSteepestDescent);
    }

private:
    void quadraticLineSearch()
    {
        ScalarType tAlpha = 1e-4;
        std::vector<ScalarType> tStepValues(2);
        // objective_function_values[0] = current value;
        // objective_function_values[1] = old trial value;
        // objective_function_values[2] = new trial value
        std::vector<ScalarType> tObjectiveValues(3);
        tObjectiveValues[0] = mNewObjectiveFunctionValue;

        Plato::dot(*mNewTrialStep, *mNewTrialStep);
        ScalarType tNormTrialStep = Plato::norm(*mNewTrialStep);
        ScalarType tValue = static_cast<ScalarType>(100.) / (static_cast<ScalarType>(1.) + tNormTrialStep);
        tStepValues[1] = std::min(static_cast<ScalarType>(1.), tValue);

        Plato::update(static_cast<ScalarType>(1), *mNewDual, static_cast<ScalarType>(0), *mTrialDual);
        Plato::update(tStepValues[1], *mNewTrialStep, static_cast<ScalarType>(1), *mTrialDual);
        mBounds->project(*mDualLowerBound, *mDualUpperBound, *mTrialDual);
        this->computeProjectedStep(*mTrialDual, *mNewDual, *mProjectedStep);

        tObjectiveValues[2] = mDualProbStageMng->evaluateObjective(*mTrialDual);

        ScalarType tInitialProjectedStepDotGradient = Plato::dot(*mProjectedStep, *mNewGradient);
        ScalarType tTargetObjectiveValue = tObjectiveValues[0] - tAlpha * tStepValues[1] * tInitialProjectedStepDotGradient;

        mLineSearchIterationCount = 0;
        OrdinalType tMaxNumIterations = this->getMaxNumLineSearchIterations();
        while(tObjectiveValues[2] > tTargetObjectiveValue)
        {
            tStepValues[0] = tStepValues[1];
            ScalarType tLambda =
                    this->quadraticInterpolation(tStepValues, tObjectiveValues, tInitialProjectedStepDotGradient);
            tStepValues[1] = tLambda;

            Plato::update(static_cast<ScalarType>(1), *mNewDual, static_cast<ScalarType>(0), *mTrialDual);
            Plato::update(tStepValues[1], *mNewTrialStep, static_cast<ScalarType>(0), *mTrialDual);
            mBounds->project(*mDualLowerBound, *mDualUpperBound, *mTrialDual);
            this->computeProjectedStep(*mTrialDual, *mNewDual, *mProjectedStep);

            tObjectiveValues[1] = tObjectiveValues[2];
            tObjectiveValues[2] = mDualProbStageMng->evaluateObjective(*mTrialDual);
            if(this->getLineSearchIterationCount() >= tMaxNumIterations)
            {
                return;
            }
            tTargetObjectiveValue = tObjectiveValues[0] - tAlpha * tStepValues[1] * tInitialProjectedStepDotGradient;
            this->updateLineSearchIterationCount();
        }
        mNewObjectiveFunctionValue = tObjectiveValues[2];
        Plato::update(static_cast<ScalarType>(1), *mTrialDual, static_cast<ScalarType>(0), *mNewDual);
    }

    ScalarType quadraticInterpolation(const std::vector<ScalarType> aStepValues,
                                      const std::vector<ScalarType> aObjectiveValues,
                                      const ScalarType & aInitialProjectedStepDotGradient)
    {
        // Recall:
        // objective_function_values[0] = current value;
        // objective_function_values[1] = old trial value;
        // objective_function_values[2] = new trial value
        ScalarType tStepLowerBound = aStepValues[1] * this->getLineSearchStepLowerBound();
        ScalarType tStepUpperBound = aStepValues[1] * this->getLineSearchStepUpperBound();
        // Quadratic interpolation model
        ScalarType tValue = -aInitialProjectedStepDotGradient
                / (static_cast<ScalarType>(2) * aStepValues[1]
                   * (aObjectiveValues[2] - aObjectiveValues[0] - aInitialProjectedStepDotGradient));
        if(tValue < tStepLowerBound)
        {
            tValue = tStepLowerBound;
        }
        if(tValue > tStepUpperBound)
        {
            tValue = tStepUpperBound;
        }
        return (tValue);
    }

    ScalarType computeScaling()
    {
        ScalarType tOutput = 0;
        switch(mType)
        {
            case Plato::gcmma_nlcg::FLETCHER_REEVES:
            {
                tOutput = this->fletcherReeves(*mNewSteepestDescent, *mOldSteepestDescent);
                break;
            }
            case Plato::gcmma_nlcg::POLAK_RIBIERE:
            {
                tOutput = this->polakRibiere(*mNewSteepestDescent, *mOldSteepestDescent);
                break;
            }
            case Plato::gcmma_nlcg::HESTENES_STIEFEL:
            {
                tOutput = this->hestenesStiefel(*mNewSteepestDescent, *mOldSteepestDescent, *mOldTrialStep);
                break;
            }
            case Plato::gcmma_nlcg::DAI_YUAN:
            {
                tOutput = this->daiYuan(*mNewSteepestDescent, *mOldSteepestDescent, *mOldTrialStep);
                break;
            }
        }

        return (tOutput);
    }

    void storeCurrentState()
    {
        mOldObjectiveFunctionValue = mNewObjectiveFunctionValue;
        Plato::update(static_cast<ScalarType>(1), *mNewDual, static_cast<ScalarType>(0), *mOldDual);
        Plato::update(static_cast<ScalarType>(1), *mNewGradient, static_cast<ScalarType>(0), *mOldGradient);
        Plato::update(static_cast<ScalarType>(1), *mNewTrialStep, static_cast<ScalarType>(0), *mOldTrialStep);
        Plato::update(static_cast<ScalarType>(1), *mNewSteepestDescent, static_cast<ScalarType>(0), *mOldSteepestDescent);
    }

    bool stoppingCriteriaSatisfied()
    {
        bool tIsCriteriaSatisfied = false;
        const OrdinalType tMaxNumIterations = this->getMaxNumIterations();
        const ScalarType tNormProjectedStep = Plato::norm(*mNewTrialStep);
        const ScalarType tObjectiveStagnation = std::abs(mNewObjectiveFunctionValue - mOldObjectiveFunctionValue);
        if(tNormProjectedStep < this->getTrialStepTolerance())
        {
            tIsCriteriaSatisfied = true;
        }
        else if(tObjectiveStagnation < this->getObjectiveStagnationTolerance())
        {
            tIsCriteriaSatisfied = true;
        }
        else if(this->getIterationCount() >= tMaxNumIterations)
        {
            tIsCriteriaSatisfied = true;
        }
        return (tIsCriteriaSatisfied);
    }

    void computeProjectedStep(const Plato::MultiVector<ScalarType,OrdinalType> & aTrialControls,
                              const Plato::MultiVector<ScalarType,OrdinalType> & aCurrentControls,
                              Plato::MultiVector<ScalarType,OrdinalType> & aProjectedStep)
    {
        assert(aTrialControls.getNumVectors() == aCurrentControls.getNumVectors());
        assert(aProjectedStep.getNumVectors() == aCurrentControls.getNumVectors());
        assert(aTrialControls[0].size() == aCurrentControls[0].size());
        assert(aProjectedStep[0].size() == aCurrentControls[0].size());

        Plato::update(static_cast<ScalarType>(1), aTrialControls, static_cast<ScalarType>(0), aProjectedStep);
        Plato::update(static_cast<ScalarType>(-1), aCurrentControls, static_cast<ScalarType>(1), aProjectedStep);
    }

    void computeProjectedGradient(const Plato::MultiVector<ScalarType,OrdinalType>  & aTrialValues,
                                  const Plato::MultiVector<ScalarType,OrdinalType>  & aLowerBounds,
                                  const Plato::MultiVector<ScalarType,OrdinalType>  & aUpperBounds,
                                  Plato::MultiVector<ScalarType,OrdinalType>  & aGradient)
    {
        assert(aLowerBounds.getNumVectors() == aTrialValues.getNumVectors());
        assert(aUpperBounds.getNumVectors() == aTrialValues.getNumVectors());
        assert(aGradient.getNumVectors() == aTrialValues.getNumVectors());
        assert(aLowerBounds[0].size() == aTrialValues[0].size());
        assert(aUpperBounds[0].size() == aTrialValues[0].size());
        assert(aGradient[0].size() == aTrialValues[0].size());

        OrdinalType tNumElements = aGradient[0].size();
        for(OrdinalType tIndex = 0; tIndex < tNumElements; ++tIndex)
        {
            aGradient(0,tIndex) = aTrialValues(0,tIndex) < aLowerBounds(0,tIndex) ? static_cast<ScalarType>(0.) : aGradient(0,tIndex);
            aGradient(0,tIndex) = aTrialValues(0,tIndex) > aUpperBounds(0,tIndex) ? static_cast<ScalarType>(0.) : aGradient(0,tIndex);
        }
    }

    ScalarType fletcherReeves(const Plato::MultiVector<ScalarType, OrdinalType> & aNewSteepestDescent,
                              const Plato::MultiVector<ScalarType, OrdinalType> & aOldSteepestDescent)
    {
        assert(aNewSteepestDescent.getNumVectors() == aOldSteepestDescent.getNumVectors());
        assert(aNewSteepestDescent[0].size() == aOldSteepestDescent[0].size());

        ScalarType tNewSteepestDescentDotNewSteepestDescent = Plato::dot(aNewSteepestDescent, aNewSteepestDescent);
        ScalarType tOldSteepestDescentDotNewSteepestDescent = Plato::dot(aOldSteepestDescent, aOldSteepestDescent);
        ScalarType tScale = (tNewSteepestDescentDotNewSteepestDescent) / (tOldSteepestDescentDotNewSteepestDescent);

        return (tScale);
    }

    ScalarType polakRibiere(const Plato::MultiVector<ScalarType,OrdinalType> & aNewSteepestDescent,
                        const Plato::MultiVector<ScalarType,OrdinalType> & aOldSteepestDescent)
    {
        assert(aNewSteepestDescent.getNumVectors() == aOldSteepestDescent.getNumVectors());
        assert(aNewSteepestDescent[0].size() == aOldSteepestDescent[0].size());

        ScalarType tNewSteepestDescentDotNewSteepestDescent = Plato::dot(aNewSteepestDescent, aNewSteepestDescent);
        Plato::dot(aNewSteepestDescent, aOldSteepestDescent);
        ScalarType tNewSteepestDescentDotOldSteepestDescent = Plato::dot(aNewSteepestDescent, aOldSteepestDescent);
        ScalarType tNumerator = tNewSteepestDescentDotNewSteepestDescent - tNewSteepestDescentDotOldSteepestDescent;
        ScalarType tOldSteepestDescentDotNewSteepestDescent = Plato::dot(aOldSteepestDescent, aOldSteepestDescent);
        ScalarType tScale = tNumerator / tOldSteepestDescentDotNewSteepestDescent;

        return (tScale);
    }

    ScalarType hestenesStiefel(const Plato::MultiVector<ScalarType, OrdinalType> & aNewSteepestDescent,
                               const Plato::MultiVector<ScalarType, OrdinalType> & aOldSteepestDescent,
                               const Plato::MultiVector<ScalarType, OrdinalType> & aOldTrialStep)
    {
        assert(aNewSteepestDescent.getNumVectors() == aOldSteepestDescent.getNumVectors());
        assert(aOldTrialStep.getNumVectors() == aOldSteepestDescent.getNumVectors());
        assert(aNewSteepestDescent[0].size() == aOldSteepestDescent[0].size());
        assert(aOldTrialStep[0].size() == aOldSteepestDescent[0].size());

        ScalarType tNewSteepestDescentDotNewSteepestDescent = Plato::dot(aNewSteepestDescent, aNewSteepestDescent);
        ScalarType tNewSteepestDescentDotOldSteepestDescent =  Plato::dot(aNewSteepestDescent, aOldSteepestDescent);
        ScalarType tNumerator = tNewSteepestDescentDotNewSteepestDescent - tNewSteepestDescentDotOldSteepestDescent;
        ScalarType tOldTrialStepDotNewSteepestDescent = Plato::dot(aOldTrialStep, aNewSteepestDescent);
        ScalarType tOldTrialStepDotOldSteepestDescent = Plato::dot(aOldTrialStep, aOldSteepestDescent);
        ScalarType tDenominator = tOldTrialStepDotNewSteepestDescent - tOldTrialStepDotOldSteepestDescent;
        ScalarType tScale = tNumerator / tDenominator;

        return (tScale);
    }

    ScalarType daiYuan(const Plato::MultiVector<ScalarType, OrdinalType> & aNewSteepestDescent,
                       const Plato::MultiVector<ScalarType, OrdinalType> & aOldSteepestDescent,
                       const Plato::MultiVector<ScalarType, OrdinalType> & aOldTrialStep)
    {
        assert(aNewSteepestDescent.getNumVectors() == aOldSteepestDescent.getNumVectors());
        assert(aOldTrialStep.getNumVectors() == aOldSteepestDescent.getNumVectors());
        assert(aNewSteepestDescent[0].size() == aOldSteepestDescent[0].size());
        assert(aOldTrialStep[0].size() == aOldSteepestDescent[0].size());

        ScalarType tOldTrialStepDotNewSteepestDescent = Plato::dot(aOldTrialStep, aNewSteepestDescent);
        ScalarType tOldTrialStepDotOldSteepestDescent = Plato::dot(aOldTrialStep, aOldSteepestDescent);
        ScalarType tDenominator = tOldTrialStepDotNewSteepestDescent - tOldTrialStepDotOldSteepestDescent;
        ScalarType tNewSteepestDescentDotNewSteepestDescent = Plato::dot(aNewSteepestDescent, aNewSteepestDescent);
        ScalarType tScale = tNewSteepestDescentDotNewSteepestDescent / tDenominator;

        return (tScale);
    }

private:
    Plato::gcmma_nlcg mType;

    ScalarType mLineSearchStepLowerBound;
    ScalarType mLineSearchStepUpperBound;

    OrdinalType mLineSearchIterationCount;
    OrdinalType mMaxNumLineSearchIterations;
    OrdinalType mNumIterationsDone;
    OrdinalType mMaxNumIterations;

    ScalarType mGradientTolerance;
    ScalarType mTrialStepTolerance;
    ScalarType mObjectiveStagnationTolerance;
    ScalarType mNewObjectiveFunctionValue;
    ScalarType mOldObjectiveFunctionValue;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mNewDual;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mOldDual;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mNewGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mOldGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mNewTrialStep;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mOldTrialStep;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mNewSteepestDescent;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mOldSteepestDescent;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialDual;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mProjectedStep;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDualLowerBound;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mDualUpperBound;

    std::shared_ptr<Plato::BoundsBase<ScalarType, OrdinalType>> mBounds;
    std::shared_ptr<Plato::DualProblemStageMng<ScalarType, OrdinalType>> mDualProbStageMng;

private:
    DualOptimizer<ScalarType, OrdinalType>(const Plato::DualOptimizer<ScalarType, OrdinalType> & aRhs);
    Plato::DualOptimizer<ScalarType, OrdinalType> & operator=(const Plato::DualOptimizer<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_DUALOPTIMIZER_HPP_ */
