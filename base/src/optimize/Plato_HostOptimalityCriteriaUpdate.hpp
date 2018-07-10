/*
 * Plato_HostOptimalityCriteriaUpdate.hpp
 *
 *  Created on: May 6, 2018
 */

#ifndef PLATO_HOSTOPTIMALITYCRITERIAUPDATE_HPP_
#define PLATO_HOSTOPTIMALITYCRITERIAUPDATE_HPP_

#include <cmath>
#include <algorithm>

#include "Plato_Vector.hpp"
#include "Plato_OptimalityCriteriaUpdate.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class HostOptimalityCriteriaUpdate : public OptimalityCriteriaUpdate<ScalarType, OrdinalType>
{
public:
    /********************************************************************************/
    explicit HostOptimalityCriteriaUpdate(const ScalarType & aMoveLimit,
                                          const ScalarType & aScaleFactor,
                                          const ScalarType & aDampingPower) :
            mMoveLimit(aMoveLimit),
            mScaleFactor(aScaleFactor),
            mDampingPower(aDampingPower)
    /********************************************************************************/
    {
    }

    /********************************************************************************/
    virtual ~HostOptimalityCriteriaUpdate()
    /********************************************************************************/
    {
    }

    /********************************************************************************/
    void reset(const ScalarType & aMoveLimit, const ScalarType & aScaleFactor, const ScalarType & aDampingPower)
    /********************************************************************************/
    {
        mMoveLimit = aMoveLimit;
        mScaleFactor = aScaleFactor;
        mDampingPower = aDampingPower;
    }

    /********************************************************************************/
    void update(const ScalarType & aTrialDual,
                const Plato::Vector<ScalarType, OrdinalType> & aLowerBounds,
                const Plato::Vector<ScalarType, OrdinalType> & aUpperBounds,
                const Plato::Vector<ScalarType, OrdinalType> & aPreviousControls,
                const Plato::Vector<ScalarType, OrdinalType> & aObjectiveGradient,
                const Plato::Vector<ScalarType, OrdinalType> & aInequalityGradient,
                Plato::Vector<ScalarType, OrdinalType> & aTrialControls)
    /********************************************************************************/
    {
        OrdinalType tNumControls = aPreviousControls.size();
        for(OrdinalType tControlIndex = 0; tControlIndex < tNumControls; tControlIndex++)
        {
            if(aInequalityGradient[tControlIndex] == static_cast<ScalarType>(0))
            {
                aTrialControls[tControlIndex] = aPreviousControls[tControlIndex];
            }
            else
            {
                ScalarType tMyDesignVariableOffset = ( mScaleFactor * ( aUpperBounds[tControlIndex]
                        - aLowerBounds[tControlIndex] ) ) - aLowerBounds[tControlIndex];
                ScalarType tMyValue = -aObjectiveGradient[tControlIndex]
                        / (aTrialDual * aInequalityGradient[tControlIndex]);
                ScalarType tFabsValue = std::abs(tMyValue);
                ScalarType tSignValue = copysign(1.0, tMyValue);
                ScalarType tMyTrialControlValue = ((aPreviousControls[tControlIndex] + tMyDesignVariableOffset)
                        * tSignValue * std::pow(tFabsValue, mDampingPower)) - tMyDesignVariableOffset;

                ScalarType tMyControlValue = aPreviousControls[tControlIndex] + mMoveLimit;
                tMyTrialControlValue = std::min(tMyControlValue, tMyTrialControlValue);
                tMyControlValue = aPreviousControls[tControlIndex] - mMoveLimit;
                tMyTrialControlValue = std::max(tMyControlValue, tMyTrialControlValue);
                tMyTrialControlValue = std::min(aUpperBounds[tControlIndex], tMyTrialControlValue);
                tMyTrialControlValue = std::max(aLowerBounds[tControlIndex], tMyTrialControlValue);
                aTrialControls[tControlIndex] = tMyTrialControlValue;
            }
        }
    }

    /********************************************************************************/
    std::shared_ptr<Plato::OptimalityCriteriaUpdate<ScalarType, OrdinalType>> create() const
        /********************************************************************************/
    {
        std::shared_ptr<Plato::OptimalityCriteriaUpdate<ScalarType, OrdinalType>> tCopy =
                std::make_shared<HostOptimalityCriteriaUpdate<ScalarType, OrdinalType>>(mMoveLimit, mScaleFactor, mDampingPower);
        return (tCopy);
    }

private:
    ScalarType mMoveLimit;
    ScalarType mScaleFactor;
    ScalarType mDampingPower;

private:
    HostOptimalityCriteriaUpdate(const Plato::HostOptimalityCriteriaUpdate<ScalarType, OrdinalType>&);
    Plato::HostOptimalityCriteriaUpdate<ScalarType, OrdinalType> & operator=(const Plato::HostOptimalityCriteriaUpdate<ScalarType, OrdinalType>&);
};
// class HostOptimalityCriteriaUpdate

} // namespace Plato

#endif /* PLATO_HOSTOPTIMALITYCRITERIAUPDATE_HPP_ */
