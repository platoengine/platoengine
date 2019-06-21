/*
 * Plato_Test_MMA.cpp
 *
 *  Created on: Jun 21, 2019
 */

#include "gtest/gtest.h"

#include <memory>

#include "Plato_MultiVector.hpp"
#include "Plato_CriterionList.hpp"
#include "Plato_MultiVectorList.hpp"
#include "Plato_KelleySachsBoundLightInterface.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
void solve(Plato::MultiVector<ScalarType, OrdinalType> & aControls,
           Plato::MultiVector<ScalarType, OrdinalType> & aDuals,
           Plato::MultiVector<ScalarType, OrdinalType> & aLowerBounds,
           Plato::MultiVector<ScalarType, OrdinalType> & aUpperBounds,
           Plato::CriterionList<ScalarType, OrdinalType> & aObjectives,
           Plato::CriterionList<ScalarType, OrdinalType> & aConstraints)
{
    // Compute initial objective function values and gradients
    const OrdinalType tNumObjectives = aObjectives.size();
    std::vector<ScalarType> tCurrentObjFuncVals(tNumObjectives);
    Plato::MultiVectorList<ScalarType, OrdinalType> tCurrentObjFuncGrads(tNumObjectives, aControls);
    for(OrdinalType tIndex = 0; tIndex < tNumObjectives; tIndex++)
    {
        tCurrentObjFuncVals[tIndex] = aObjectives[tIndex].value(aControls);
        Plato::MultiVector<ScalarType, OrdinalType>& tMyGradient = tCurrentObjFuncGrads[tIndex];
        aObjectives[tIndex].gradient(aControls, tMyGradient);
    }

    // Compute initial constraint values and gradients
    const OrdinalType tNumConstraints = aConstraints.size();
    std::vector<ScalarType> tCurrentConstraintVals(tNumConstraints);
    Plato::MultiVectorList<ScalarType, OrdinalType> tCurrentConstraintGrads(tNumConstraints, aControls);
    for(OrdinalType tIndex = 0; tIndex < tNumObjectives; tIndex++)
    {
        tCurrentConstraintVals[tIndex] = aConstraints[tIndex].value(aControls);
        Plato::MultiVector<ScalarType, OrdinalType>& tMyGradient = tCurrentConstraintGrads[tIndex];
        aConstraints[tIndex].gradient(aControls, tMyGradient);
    }

}

}
// namespace Plato

