/*
 * Plato_BetaDistribution.hpp
 *
 *  Created on: Jan 31, 2018
 */

#ifndef PLATO_BETADISTRIBUTION_HPP_
#define PLATO_BETADISTRIBUTION_HPP_

#include <cmath>
#include <cassert>

#include "Plato_Distribution.hpp"
#include "Plato_StatisticsUtils.hpp"

#define _MATH_DEFINES_DEFINED

namespace Plato
{

template<typename ScalarType>
inline ScalarType beta(const ScalarType & aAlpha, const ScalarType & aBeta)
{
    ScalarType tCoefficientOne = std::lgamma(aAlpha);
    tCoefficientOne = std::isfinite(tCoefficientOne) == true ? tCoefficientOne : static_cast<ScalarType>(0);

    ScalarType tCoefficientTwo = std::lgamma(aBeta);
    tCoefficientTwo = std::isfinite(tCoefficientTwo) == true ? tCoefficientTwo : static_cast<ScalarType>(0);

    ScalarType tCoefficientThree = std::lgamma(aAlpha + aBeta);
    tCoefficientThree = std::isfinite(tCoefficientThree) == true ? tCoefficientThree : static_cast<ScalarType>(0);

    ScalarType tExponent = tCoefficientOne + tCoefficientTwo - tCoefficientThree;
    ScalarType tOutput = std::exp(tExponent);

    return (tOutput);
}

template<typename ScalarType, typename OrdinalType = size_t>
inline ScalarType incomplete_beta(const ScalarType & aValue, const ScalarType & aAlpha, const ScalarType & aBeta)
{
    const OrdinalType tNUM_TERMS = 21;
    const ScalarType tConstantTwo = static_cast<ScalarType>(1) - aBeta;

    ScalarType tSum = 0;
    for(OrdinalType tIndex = 0; tIndex <= tNUM_TERMS; tIndex++)
    {
        ScalarType tFactorial = Plato::factorial<OrdinalType>(tIndex);
        ScalarType tDenominator = tFactorial * (aAlpha + tIndex);
        ScalarType tNumerator = Plato::pochhammer_symbol<ScalarType>(tConstantTwo, tIndex);
        ScalarType tConstant = tNumerator / tDenominator;
        tConstant = std::isfinite(tConstant) ? tConstant : 0;
        tSum = tSum + (tConstant * std::pow(aValue, tIndex));
    }
    const ScalarType tConstant = std::pow(aValue, aAlpha);
    tSum = tConstant * tSum;

    return (tSum);
}

template<typename ScalarType>
inline ScalarType beta_pdf(const ScalarType & aSample, const ScalarType & aAlpha, const ScalarType & aBeta)
{
    const ScalarType tEPSILON = 1e-14;
    const ScalarType tAlpha = aAlpha + tEPSILON;
    const ScalarType tBeta = aBeta + tEPSILON;

    const ScalarType tNumerator = std::pow(aSample, tAlpha - static_cast<ScalarType>(1))
            * std::pow(static_cast<ScalarType>(1) - aSample, tBeta - static_cast<ScalarType>(1));
    const ScalarType tDenominator = Plato::beta<ScalarType>(tAlpha, tBeta);
    const ScalarType tOutput = tNumerator / tDenominator;

    return (tOutput);
}

template<typename ScalarType, typename OrdinalType = size_t>
inline ScalarType beta_cdf(const ScalarType & aSample, const ScalarType & aAlpha, const ScalarType & aBeta)
{
    if(aSample < static_cast<ScalarType>(0))
    {
        const ScalarType tOutput = 0;
        return (tOutput);
    }
    else if(aSample > static_cast<ScalarType>(1))
    {
        const ScalarType tOutput = 1;
        return (tOutput);
    }

    const ScalarType tEPSILON = 1e-14;
    const ScalarType tAlpha = aAlpha + tEPSILON;
    const ScalarType tBeta = aBeta + tEPSILON;
    const ScalarType tNumerator = Plato::incomplete_beta<ScalarType, OrdinalType>(aSample, tAlpha, tBeta);
    const ScalarType tDenominator = Plato::beta<ScalarType>(tAlpha, tBeta);
    const ScalarType tOutput = tNumerator / tDenominator;

    return (tOutput);
}

template<typename ScalarType, typename OrdinalType = size_t>
inline ScalarType beta_moment(const OrdinalType & aOrder, const ScalarType & aAlpha, const ScalarType & aBeta)
{
    ScalarType tDenominator = Plato::beta<ScalarType>(aAlpha, aBeta);
    ScalarType tMyAlpha = aAlpha + static_cast<ScalarType>(aOrder);
    ScalarType tNumerator = Plato::beta<ScalarType>(tMyAlpha, aBeta);
    ScalarType tOutput = tNumerator / tDenominator;
    return (tOutput);
}

template<typename ScalarType, typename OrdinalType = size_t>
inline ScalarType shift_beta_moment(const OrdinalType & aOrder,
                                    const ScalarType & aShift,
                                    const ScalarType & aAlpha,
                                    const ScalarType & aBeta)
{
    assert(aOrder >= static_cast<OrdinalType>(0));
    ScalarType tOutput = 0;
    for(OrdinalType tIndex = 1; tIndex <= aOrder; tIndex++)
    {
        ScalarType tNumerator = Plato::factorial<OrdinalType>(aOrder);
        ScalarType tDenominator = Plato::factorial<OrdinalType>(tIndex) * Plato::factorial<OrdinalType>(aOrder - tIndex);
        ScalarType tCoefficient = tNumerator / tDenominator;
        ScalarType tShiftParameter = tCoefficient * std::pow(aShift, static_cast<ScalarType>(tIndex));
        OrdinalType tMyOrder = aOrder - tIndex;
        ScalarType tMoment = Plato::beta_moment<ScalarType, OrdinalType>(tMyOrder, aAlpha, aBeta);
        tOutput = tOutput + tShiftParameter * tMoment;
    }
    return (tOutput);
}

template<typename ScalarType, typename OrdinalType = size_t>
class BetaDistribution : public Plato::Distribution<ScalarType, OrdinalType>
{
public:
    explicit BetaDistribution(const ScalarType & aMin,
                              const ScalarType & aMax,
                              const ScalarType & aMean,
                              const ScalarType & aVariance) :
            mMin(aMin),
            mMax(aMax),
            mMean(aMean),
            mVariance(aVariance),
            mBeta(0),
            mAlpha(0)
    {
        Plato::shape_parameters<ScalarType>(aMin, aMax, aMean, aVariance, mAlpha, mBeta);
    }
    virtual ~BetaDistribution()
    {
    }

    ScalarType min() const
    {
        return (mMin);
    }
    ScalarType max() const
    {
        return (mMax);
    }
    ScalarType mean() const
    {
        return (mMean);
    }
    ScalarType variance() const
    {
        return (mVariance);
    }
    ScalarType beta() const
    {
        return (mBeta);
    }
    ScalarType alpha() const
    {
        return (mAlpha);
    }

    ScalarType pdf(const ScalarType & aInput)
    {
        ScalarType tOutput = Plato::beta_pdf<ScalarType>(aInput, mAlpha, mBeta);
        return (tOutput);
    }
    ScalarType cdf(const ScalarType & aInput)
    {
        ScalarType tOutput = Plato::beta_cdf<ScalarType, OrdinalType>(aInput, mAlpha, mBeta);
        return (tOutput);
    }
    ScalarType moment(const OrdinalType & aInput)
    {
        ScalarType tOutput = Plato::beta_moment<ScalarType, OrdinalType>(aInput, mAlpha, mBeta);
        return (tOutput);
    }

private:
    ScalarType mMin;
    ScalarType mMax;
    ScalarType mMean;
    ScalarType mVariance;

    ScalarType mBeta;
    ScalarType mAlpha;

private:
    BetaDistribution(const Plato::BetaDistribution<ScalarType, OrdinalType> & aRhs);
    Plato::BetaDistribution<ScalarType, OrdinalType> & operator=(const Plato::BetaDistribution<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_BETADISTRIBUTION_HPP_ */
