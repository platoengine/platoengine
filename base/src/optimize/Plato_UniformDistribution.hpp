/*
 * Plato_UniformDistribution.hpp
 *
 *  Created on: Feb 1, 2018
 */

#ifndef PLATO_UNIFORMDISTRIBUTION_HPP_
#define PLATO_UNIFORMDISTRIBUTION_HPP_

#include <cmath>
#include <cassert>

#include "Plato_Distribution.hpp"

namespace Plato
{

template<typename ScalarType>
inline ScalarType uniform_entropy(const ScalarType & aLowerBound, const ScalarType & aUpperBound)
{
    const ScalarType tValue = aUpperBound - aLowerBound;
    const ScalarType tOutput = std::log(tValue);
    return (tOutput);
}

template<typename ScalarType>
inline ScalarType uniform_pdf(const ScalarType & aSample, const ScalarType & aLowerBound, const ScalarType & aUpperBound)
{
    // f(x) = \frac{1}{ub-lb},\ \forall\ x\in\[lb,ub\]; else, f(x)=0
    ScalarType tOutput =
            aSample >= aLowerBound && aSample <= aUpperBound ? static_cast<ScalarType>(1) / (aUpperBound - aLowerBound) : static_cast<ScalarType>(0);
    return (tOutput);
}

template<typename ScalarType>
inline ScalarType uniform_cdf(const ScalarType & aSample, const ScalarType & aLowerBound, const ScalarType & aUpperBound)
{
    ScalarType tOutput = 1;
    if(aSample < aLowerBound)
    {
        // F(x) = 0,\ \forall\ x<lb
        tOutput = static_cast<ScalarType>(0);
    }
    else if(aSample >= aLowerBound && aSample < aUpperBound)
    {
        // F(x) = \frac{x - lb}{ub - lb},\ \forall\ x\in\[lb,ub)
        ScalarType tNumerator = aSample - aLowerBound;
        ScalarType tDenominator = aUpperBound - aLowerBound;
        tOutput = tNumerator / tDenominator;
    }
    // else, F(x)=1,\ \forall\ x>ub
    return (tOutput);
}

template<typename ScalarType>
inline ScalarType uniform_moment(const ScalarType & aOrder, const ScalarType & aLowerBound, const ScalarType & aUpperBound)
{
    // m_n(x) = \frac{b^{n+1} - a^{n+1}}{n*(b - a)}, where n denotes the order, a denotes lower bound and b denotes upper bound.
    assert(aOrder >= static_cast<ScalarType>(0));
    const ScalarType tExponent = aOrder + static_cast<ScalarType>(1);
    const ScalarType tNumerator = std::pow(aUpperBound, tExponent) - std::pow(aLowerBound, tExponent);
    const ScalarType tDenominator = (aOrder + static_cast<ScalarType>(1)) * (aUpperBound - aLowerBound);
    const ScalarType tOutput = tNumerator / tDenominator;
    return (tOutput);
}

template<typename ScalarType>
inline ScalarType uniform_variance(const ScalarType & aLowerBound, const ScalarType & aUpperBound)
{
    // Var(X) = \frac{(b-a)^2}{12}, where a denotes lower bound and b denotes upper bound.
    const ScalarType tNumerator = (aUpperBound - aLowerBound) * (aUpperBound - aLowerBound);
    const ScalarType tOutput = tNumerator / static_cast<ScalarType>(12);
    return (tOutput);
}

template<typename ScalarType, typename OrdinalType = size_t>
class UniformDistribution : public Plato::Distribution<ScalarType, OrdinalType>
{
public:
    explicit UniformDistribution(const ScalarType & aLowerBound, const ScalarType & aUpperBound) :
            mLowerBound(aLowerBound),
            mUpperBound(aUpperBound)
    {
    }
    virtual ~UniformDistribution()
    {
    }

    ScalarType lower() const
    {
        return (mLowerBound);
    }

    ScalarType upper() const
    {
        return (mUpperBound);
    }

    ScalarType mean() const
    {
        const ScalarType tOrder = 1;
        const ScalarType tOutput = Plato::uniform_moment<ScalarType>(tOrder, mLowerBound, mUpperBound);
        return (tOutput);
    }

    ScalarType variance() const
    {
        const ScalarType tOutput = Plato::uniform_variance<ScalarType>(mLowerBound, mUpperBound);
        return (tOutput);
    }

    ScalarType entropy() const
    {
        const ScalarType tOuput = Plato::uniform_entropy<ScalarType>(mLowerBound, mUpperBound);
        return (tOuput);
    }

    ScalarType pdf(const ScalarType & aInput)
    {
        ScalarType tOutput = Plato::uniform_pdf<ScalarType>(aInput, mLowerBound, mUpperBound);
        return (tOutput);
    }

    ScalarType cdf(const ScalarType & aInput)
    {
        ScalarType tOutput = Plato::uniform_cdf<ScalarType>(aInput, mLowerBound, mUpperBound);
        return (tOutput);
    }

    ScalarType moment(const OrdinalType & aInput)
    {
        ScalarType tOutput = Plato::uniform_moment<ScalarType>(aInput, mLowerBound, mUpperBound);
        return (tOutput);
    }

private:
    ScalarType mLowerBound;
    ScalarType mUpperBound;

private:
    UniformDistribution(const Plato::UniformDistribution<ScalarType, OrdinalType> & aRhs);
    Plato::UniformDistribution<ScalarType, OrdinalType> & operator=(const Plato::UniformDistribution<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_UNIFORMDISTRIBUTION_HPP_ */
