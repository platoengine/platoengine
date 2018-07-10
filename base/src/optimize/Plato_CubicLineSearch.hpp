/*
 * Plato_CubicLineSearch.hpp
 *
 *  Created on: Oct 22, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_CUBICLINESEARCH_HPP_
#define PLATO_CUBICLINESEARCH_HPP_

#include <cmath>
#include <vector>
#include <memory>

#include "Plato_LineSearch.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_StateManager.hpp"
#include "Plato_LinearAlgebra.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class CubicLineSearch : public Plato::LineSearch<ScalarType, OrdinalType>
{
public:
    explicit CubicLineSearch(const Plato::DataFactory<ScalarType, OrdinalType> & aDataFactory) :
            mMaxNumIterations(50),
            mNumIterationsDone(0),
            mContractionFactor(0.5),
            mArmijoRuleConstant(1e-4),
            mStagnationTolerance(1e-8),
            mInitialGradientDotTrialStep(0),
            mStepValues(3, static_cast<ScalarType>(0)),
            mTrialControl(aDataFactory.control().create()),
            mProjectedTrialStep(aDataFactory.control().create())
    {
    }
    virtual ~CubicLineSearch()
    {
    }

    OrdinalType getNumIterationsDone() const
    {
        return (mNumIterationsDone);
    }
    ScalarType getStepValue() const
    {
        return (mStepValues[2]);
    }

    void setMaxNumIterations(const OrdinalType & aInput)
    {
        mMaxNumIterations = aInput;
    }
    void setStagnationTolerance(const OrdinalType & aInput)
    {
        mStagnationTolerance = aInput;
    }
    void setContractionFactor(const ScalarType & aInput)
    {
        mContractionFactor = aInput;
    }

    void step(Plato::StateManager<ScalarType, OrdinalType> & aStateMng)
    {
        ScalarType tInitialStep = 1;
        Plato::update(static_cast<ScalarType>(1),
                      aStateMng.getCurrentControl(),
                      static_cast<ScalarType>(0),
                      mTrialControl.operator*());
        Plato::update(tInitialStep, aStateMng.getTrialStep(), static_cast<ScalarType>(1), *mTrialControl);

        ScalarType tTrialObjectiveValue = aStateMng.evaluateObjective(mTrialControl.operator*());
        // tTrialObjectiveValue[0] = current, tTrialObjectiveValue[1] = old, tTrialObjectiveValue[2] = trial
        const OrdinalType tSize = 3;
        std::vector<ScalarType> tObjectiveFunctionValues(tSize, 0.);
        tObjectiveFunctionValues[0] = aStateMng.getCurrentObjectiveValue();
        tObjectiveFunctionValues[2] = tTrialObjectiveValue;
        // step[0] = old, step[1] = current, step[2] = new
        mStepValues[2] = tInitialStep;
        mStepValues[1] = mStepValues[2];

        mNumIterationsDone = 1;
        mInitialGradientDotTrialStep = Plato::dot(aStateMng.getCurrentGradient(), aStateMng.getTrialStep());
        while(mNumIterationsDone <= mMaxNumIterations)
        {
            ScalarType tSufficientDecreaseCondition = tObjectiveFunctionValues[0]
                    + (mArmijoRuleConstant * mStepValues[1] * mInitialGradientDotTrialStep);
            bool tSufficientDecreaseConditionSatisfied =
                    tObjectiveFunctionValues[2] < tSufficientDecreaseCondition ? true : false;
            bool tStepIsLessThanTolerance = mStepValues[2] < mStagnationTolerance ? true : false;
            if(tSufficientDecreaseConditionSatisfied || tStepIsLessThanTolerance)
            {
                break;
            }
            mStepValues[0] = mStepValues[1];
            mStepValues[1] = mStepValues[2];
            if(mNumIterationsDone == static_cast<OrdinalType>(1))
            {
                // first backtrack: do a quadratic fit
                ScalarType tDenominator = static_cast<ScalarType>(2)
                        * (tObjectiveFunctionValues[2] - tObjectiveFunctionValues[0] - mInitialGradientDotTrialStep);
                mStepValues[2] = -mInitialGradientDotTrialStep / tDenominator;
            }
            else
            {
                this->interpolate(tObjectiveFunctionValues);
            }
            this->checkCurrentStepValue();
            Plato::update(static_cast<ScalarType>(1),
                          aStateMng.getCurrentControl(),
                          static_cast<ScalarType>(0),
                          mTrialControl.operator*());
            Plato::update(mStepValues[2], aStateMng.getTrialStep(), static_cast<ScalarType>(1), *mTrialControl);

            tTrialObjectiveValue = aStateMng.evaluateObjective(mTrialControl.operator*());
            tObjectiveFunctionValues[1] = tObjectiveFunctionValues[2];
            tObjectiveFunctionValues[2] = tTrialObjectiveValue;
            mNumIterationsDone++;
        }

        aStateMng.setCurrentObjectiveValue(tTrialObjectiveValue);
        aStateMng.setCurrentControl(mTrialControl.operator*());
    }

private:
    void checkCurrentStepValue()
    {
        const ScalarType tGamma = 0.1;
        if(mStepValues[2] > mContractionFactor * mStepValues[1])
        {
            mStepValues[2] = mContractionFactor * mStepValues[1];
        }
        else if(mStepValues[2] < tGamma * mStepValues[1])
        {
            mStepValues[2] = tGamma * mStepValues[1];
        }
        if(std::isfinite(mStepValues[2]) == false)
        {
            mStepValues[2] = tGamma * mStepValues[1];
        }
    }
    void interpolate(const std::vector<ScalarType> & aObjectiveFunctionValues)
    {
        ScalarType tPointOne = aObjectiveFunctionValues[2] - aObjectiveFunctionValues[0]
                - mStepValues[1] * mInitialGradientDotTrialStep;
        ScalarType tPointTwo = aObjectiveFunctionValues[1] - aObjectiveFunctionValues[0]
                - mStepValues[0] * mInitialGradientDotTrialStep;
        ScalarType tPointThree = static_cast<ScalarType>(1.) / (mStepValues[1] - mStepValues[0]);

        // find cubic unique minimum
        ScalarType tPointA = tPointThree
                * ((tPointOne / (mStepValues[1] * mStepValues[1])) - (tPointTwo / (mStepValues[0] * mStepValues[0])));
        ScalarType tPointB = tPointThree
                * ((tPointTwo * mStepValues[1] / (mStepValues[0] * mStepValues[0]))
                        - (tPointOne * mStepValues[0] / (mStepValues[1] * mStepValues[1])));
        ScalarType tPointC = tPointB * tPointB - static_cast<ScalarType>(3) * tPointA * mInitialGradientDotTrialStep;

        // cubic equation has unique minimum
        ScalarType tValueOne = (-tPointB + std::sqrt(tPointC)) / (static_cast<ScalarType>(3) * tPointA);
        // cubic equation is a quadratic
        ScalarType tValueTwo = -mInitialGradientDotTrialStep / (static_cast<ScalarType>(2) * tPointB);
        mStepValues[2] = tPointA != 0 ? mStepValues[2] = tValueOne : mStepValues[2] = tValueTwo;
    }

private:
    OrdinalType mMaxNumIterations;
    OrdinalType mNumIterationsDone;

    ScalarType mContractionFactor;
    ScalarType mArmijoRuleConstant;
    ScalarType mStagnationTolerance;
    ScalarType mInitialGradientDotTrialStep;

    std::vector<ScalarType> mStepValues;

    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialControl;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mProjectedTrialStep;

private:
    CubicLineSearch(const Plato::CubicLineSearch<ScalarType, OrdinalType> & aRhs);
    Plato::CubicLineSearch<ScalarType, OrdinalType> & operator=(const Plato::CubicLineSearch<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_CUBICLINESEARCH_HPP_ */
