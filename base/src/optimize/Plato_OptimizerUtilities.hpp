/*
 * Plato_OptimizerUtilities.hpp
 *
 *  Created on: Mar 11, 2018
 */

#ifndef PLATO_OPTIMIZERUTILITIES_HPP_
#define PLATO_OPTIMIZERUTILITIES_HPP_

#include <vector>
#include <string>
#include <memory>
#include <cassert>
#include <algorithm>

#include "Plato_Parser.hpp"
#include "Plato_Interface.hpp"
#include "Plato_OptimizerEngineStageData.hpp"

namespace Plato
{

/********************************************************************************/
template<typename ScalarType, typename OrdinalType>
inline void initialize(Plato::Interface* aInterface, Plato::OptimizerEngineStageData & aInputData)
/********************************************************************************/
{
    auto tInputData = aInterface->getInputData();
    auto tOptimizationNode = tInputData.get<Plato::InputData>("Optimizer");
    Plato::Parse::parseOptimizerStages(tOptimizationNode, aInputData);

    const OrdinalType tNumConstraints = aInputData.getNumConstraints();
    for(OrdinalType tIndex = 0; tIndex < tNumConstraints; tIndex++)
    {
        std::string tReferenceValueName = aInputData.getConstraintReferenceValueName(tIndex);
        std::string tConstraintValueName = aInputData.getConstraintValueName(tIndex);
        if(tReferenceValueName.empty() == false)
        {
            ScalarType tReferenceValue = 0;
            Teuchos::ParameterList tArguments;
            tArguments.set(tReferenceValueName, &tReferenceValue);
            aInterface->compute(tReferenceValueName, tArguments);
            aInputData.addConstraintReferenceValue(tConstraintValueName, tReferenceValue);
        }

        // Do some error checking and make sure we have valid constraint values set.  If the user
        // specified an absolute value rather than a normalized one we need to calculate
        // the normalized one.
        bool tAbsoluteWasSet = aInputData.constraintAbsoluteTargetValueWasSet(tConstraintValueName);
        bool tNormalizedWasSet = aInputData.constraintNormalizedTargetValueWasSet(tConstraintValueName);
        bool tReferenceWasSet = aInputData.constraintReferenceValueWasSet(tConstraintValueName);
        if(tReferenceWasSet)
        {
            double tReferenceTargetValue = aInputData.getConstraintReferenceValue(tConstraintValueName);
            if(tAbsoluteWasSet && tNormalizedWasSet)
            {
                // If both normalized and absolute target values were set make sure they
                // are consistent.
                double tAbsoluteTargetValue = aInputData.getConstraintAbsoluteTargetValue(tConstraintValueName);
                double tNormalizedTargetValue = aInputData.getConstraintNormalizedTargetValue(tConstraintValueName);
                double tCalculatedNormalizedValue = tAbsoluteTargetValue / tReferenceTargetValue;
                if(fabs(tCalculatedNormalizedValue - tNormalizedTargetValue) > 0.0001)
                {
                    std::cout << "\n\nERROR: Both normalized and absolute constraint values were specified but they are not consistent.\n";
                    std::cout << "Reference value: " << tReferenceTargetValue << ", Absolute target: " << tAbsoluteTargetValue;
                    std::cout << ", Normalized target: " << tNormalizedTargetValue << "\n";
                    std::cout << "The following should be true: Normalized target = (Absolute target)/(Reference value)\n\n";
                }
            }
            else if(tAbsoluteWasSet && !tNormalizedWasSet)
            {
                // If just an absolute value was set we need to calculate the
                // normalized value.
                double tAbsoluteTargetValue = aInputData.getConstraintAbsoluteTargetValue(tConstraintValueName);
                double tNormalizedTargetValue = tAbsoluteTargetValue / tReferenceTargetValue;
                aInputData.addConstraintNormalizedTargetValue(tConstraintValueName, tNormalizedTargetValue);
            }
            else if(!tAbsoluteWasSet && tNormalizedWasSet)
            {
                // If just normalized was set do nothing--all is good.
            }
            else // (!tAbsoluteWasSet && !tNormalizedWasSet)
            {
                // If neither was set we have a problem.
                std::cout << "\n\nERROR: A constraint target value was not specified.\n\n";
            }
        }
        else
        {
            std::cout << "\n\nERROR: A constraint reference value was not specified or calculated by a stage.\n\n";
        }
    }
}

/********************************************************************************/
template<typename ScalarType>
inline void getUpperBoundsInputData(const Plato::OptimizerEngineStageData & aInputData,
                                    Plato::Interface* aInterface,
                                    std::vector<ScalarType> & aOutput)
/********************************************************************************/
{
    std::string tUpperBoundsStageName = aInputData.getSetUpperBoundsStageName();
    if(tUpperBoundsStageName.empty() == false)
    {
        std::string tUpperBoundVectorName = aInputData.getUpperBoundVectorName();
        std::string tUpperBoundValueName = aInputData.getUpperBoundValueName();
        std::vector<ScalarType> tUpperBoundValues = aInputData.getUpperBoundValues();

        // ********* COMPUTE UPPER BOUNDS THROUGH PLATO ENGINE *********
        ScalarType tValue = 0;
        std::fill(aOutput.begin(), aOutput.end(), tValue);
        Teuchos::ParameterList tUpperBoundsStageArgs;
        tUpperBoundsStageArgs.set(tUpperBoundVectorName, aOutput.data());
        tUpperBoundsStageArgs.set(tUpperBoundValueName, tUpperBoundValues.data());
        aInterface->compute(tUpperBoundsStageName, tUpperBoundsStageArgs);
    }
    else
    {
        std::vector<ScalarType> tUpperBounds = aInputData.getUpperBoundValues();
        assert(tUpperBounds.size() == static_cast<size_t>(1));
        const ScalarType tValue = tUpperBounds[0];
        std::fill(aOutput.begin(), aOutput.end(), tValue);
    }
}

/********************************************************************************/
template<typename ScalarType>
inline void getLowerBoundsInputData(const Plato::OptimizerEngineStageData & aInputData,
                                    Plato::Interface* aInterface,
                                    std::vector<ScalarType> & aOutput)
/********************************************************************************/
{
    std::string tSetLowerBoundsStageName = aInputData.getSetLowerBoundsStageName();
    if(tSetLowerBoundsStageName.empty() == false)
    {
        std::string tLowerBoundVectorName = aInputData.getLowerBoundVectorName();
        std::string tLowerBoundValueName = aInputData.getLowerBoundValueName();
        std::vector<ScalarType> tLowerBoundValues = aInputData.getLowerBoundValues();

        // ********* COMPUTE LOWER BOUNDS THROUGH PLATO ENGINE *********
        Teuchos::ParameterList tLowerBoundsStageArgs;
        tLowerBoundsStageArgs.set(tLowerBoundVectorName, aOutput.data());
        tLowerBoundsStageArgs.set(tLowerBoundValueName, tLowerBoundValues.data());
        aInterface->compute(tSetLowerBoundsStageName, tLowerBoundsStageArgs);
    }
    else
    {
        std::vector<ScalarType> tLowerBounds = aInputData.getLowerBoundValues();
        assert(tLowerBounds.size() == static_cast<size_t>(1));
        const ScalarType tValue = tLowerBounds[0];
        std::fill(aOutput.begin(), aOutput.end(), tValue);
    }
}

/********************************************************************************/
template<typename ScalarType>
inline void getInitialGuessInputData(const std::string & aControlName,
                                     const Plato::OptimizerEngineStageData & aInputData,
                                     Plato::Interface* aInterface,
                                     std::vector<ScalarType> & aOutput)
/********************************************************************************/
{
    std::string tInitializationStageName = aInputData.getInitializationStageName();
    if(tInitializationStageName.empty() == false)
    {
        Teuchos::ParameterList tArguments;
        tArguments.set(aControlName, aOutput.data());
        aInterface->compute(tInitializationStageName, tArguments);
    }
    else
    {
        // Use user-defined values to compute initial guess. Hence, a stage was not defined by the user.
        std::vector<ScalarType> tInitialGuess = aInputData.getInitialGuess();
        if(tInitialGuess.size() <= static_cast<size_t>(1))
        {
            assert(tInitialGuess.size() == static_cast<size_t>(1));
            const ScalarType tValue = tInitialGuess[0];
            std::fill(aOutput.begin(), aOutput.end(), tValue);
        }
        else
        {
            assert(aOutput.size() == tInitialGuess.size());
            std::copy(tInitialGuess.begin(), tInitialGuess.end(), aOutput.begin());
        }
    }
}

} //namespace Plato

#endif /* PLATO_OPTIMIZERUTILITIES_HPP_ */
