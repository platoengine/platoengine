#pragma once

#include "Plato_TrustRegionAlgorithmDataMng.hpp"
#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"

#include <memory>
#include <iostream>
#include <vector>

namespace Plato
{

namespace ml_trust_region
{
enum step_prediction
{
    no_prediction = 0,
    reject_step = 1,
    accept_step = 2,
};
}

template<typename ScalarType, typename OrdinalType = size_t>
class MLTrustRegionStepPrediction
{
public:
    MLTrustRegionStepPrediction(const std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> & aDataMng) :
            mVerbose(false),
            mDataMng(aDataMng)
    {
    }
    ~MLTrustRegionStepPrediction()
    {
    }

    ml_trust_region::step_prediction make_prediction()
    {
        std::vector<double> prediction_input;
        make_prediction_input(prediction_input);

        if(mVerbose)
        {
            std::cout << "making no prediction    , from vector:";
            print_vector(prediction_input);
        }
        return ml_trust_region::step_prediction::no_prediction;
    }
    void store_prediction(const ml_trust_region::step_prediction& accurate_prediction)
    {
        std::vector<double> prediction_input;
        make_prediction_input(prediction_input);

        if(mVerbose)
        {
            std::cout << "accurate prediction is " << accurate_prediction << ", from vector:";
            print_vector(prediction_input);
        }
    }

private:

    void make_prediction_input(std::vector<double>& prediction_input)
    {
        prediction_input.clear();
        prediction_input.push_back(mDataMng->getNumControlVectors());
        prediction_input.push_back(mDataMng->getNumDualVectors());
        prediction_input.push_back(mDataMng->getCurrentObjectiveFunctionValue());
        prediction_input.push_back(mDataMng->getPreviousObjectiveFunctionValue());
        const Plato::MultiVector<ScalarType, OrdinalType> & tDual = mDataMng->getDual();
        const Plato::MultiVector<ScalarType, OrdinalType> & tTrialStep = mDataMng->getTrialStep();
        const Plato::MultiVector<ScalarType, OrdinalType> & tActiveSet = mDataMng->getActiveSet();
        const Plato::MultiVector<ScalarType, OrdinalType> & tInactiveSet = mDataMng->getInactiveSet();
        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentControl = mDataMng->getCurrentControl();
        const Plato::MultiVector<ScalarType, OrdinalType> & tPreviousControl = mDataMng->getPreviousControl();
        const Plato::MultiVector<ScalarType, OrdinalType> & tCurrentGraident = mDataMng->getCurrentGradient();
        const Plato::MultiVector<ScalarType, OrdinalType> & tPreviousGradient = mDataMng->getPreviousGradient();
        prediction_input.push_back(norm(tDual));
        prediction_input.push_back(norm(tTrialStep));
        prediction_input.push_back(norm(tActiveSet));
        prediction_input.push_back(norm(tInactiveSet));
        prediction_input.push_back(norm(tCurrentControl));
        prediction_input.push_back(norm(tPreviousControl));
        prediction_input.push_back(norm(tCurrentGraident));
        prediction_input.push_back(norm(tPreviousGradient));
        prediction_input.push_back(dot(tTrialStep, tActiveSet));
        prediction_input.push_back(dot(tTrialStep, tInactiveSet));
        prediction_input.push_back(dot(tTrialStep, tCurrentGraident));
        prediction_input.push_back(dot(tTrialStep, tPreviousGradient));
        prediction_input.push_back(dot(tCurrentControl, tPreviousControl));
        prediction_input.push_back(dot(tCurrentControl, tCurrentGraident));
        prediction_input.push_back(dot(tCurrentControl, tPreviousGradient));
        prediction_input.push_back(dot(tPreviousControl, tCurrentGraident));
        prediction_input.push_back(dot(tPreviousControl, tPreviousGradient));
        prediction_input.push_back(mDataMng->isGradientInexactnessToleranceExceeded());
        prediction_input.push_back(mDataMng->isObjectiveInexactnessToleranceExceeded());
        prediction_input.push_back(mDataMng->getObjectiveStagnationMeasure());
        prediction_input.push_back(mDataMng->getControlStagnationMeasure());
        prediction_input.push_back(mDataMng->getNormProjectedGradient());
        prediction_input.push_back(mDataMng->getStationarityMeasure());
    }

    void print_vector(const std::vector<double>& input)
    {
        const size_t length = input.size();
        for(size_t i = 0u; i < length; i++)
        {
            std::cout << input[i] << ",";
        }
        std::cout << std::endl;
    }

    bool mVerbose;
    std::shared_ptr<Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType>> mDataMng;

};

}
