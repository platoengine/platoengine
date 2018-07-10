/*
 * Plato_StatisticsUtils.hpp
 *
 *  Created on: Jan 31, 2018
 */

#ifndef PLATO_STATISTICSUTILS_HPP_
#define PLATO_STATISTICSUTILS_HPP_

#include <cmath>

#define _MATH_DEFINES_DEFINED

namespace Plato
{

template<typename OrdinalType>
inline OrdinalType factorial(const OrdinalType & aInput)
{
    OrdinalType tOutput = 1;
    for(OrdinalType tIndex = 1; tIndex <= aInput; tIndex++)
    {
        tOutput *= tIndex;
    }
    return (tOutput);
}

template<typename ScalarType>
inline ScalarType pochhammer_symbol(const ScalarType & aX, const ScalarType & aN)
{
    const ScalarType tCoefficientOne = std::tgamma(aX + aN);
    const ScalarType tCoefficientTwo = std::tgamma(aX);
    ScalarType tOutput = tCoefficientOne / tCoefficientTwo;
    tOutput = std::isfinite(tOutput) ? tOutput : static_cast<ScalarType>(0);
    return (tOutput);
}

template<typename ScalarType>
inline void shape_parameters(const ScalarType & aMinValue,
                             const ScalarType & aMaxValue,
                             const ScalarType & aMean,
                             const ScalarType & aSigma,
                             ScalarType & aAlpha,
                             ScalarType & aBeta)
{
    // Scale mean/variance to lie in [0,1] for the standard beta distribution:
    ScalarType tMeanStd = (aMean - aMinValue) / (aMaxValue - aMinValue);
    ScalarType tVarianceStd = (static_cast<ScalarType>(1) / (aMaxValue - aMinValue))
            * (static_cast<ScalarType>(1) / (aMaxValue - aMinValue)) * aSigma;
    // Compute shape parameters for Beta distributions based on standard mean/variance:
    aAlpha = tMeanStd
            * (tMeanStd * (static_cast<ScalarType>(1) - tMeanStd) / tVarianceStd - static_cast<ScalarType>(1));
    aBeta = (tMeanStd * (static_cast<ScalarType>(1) - tMeanStd) / tVarianceStd - static_cast<ScalarType>(1)) - aAlpha;
}

} // namespace Plato

#endif /* PLATO_STATISTICSUTILS_HPP_ */
