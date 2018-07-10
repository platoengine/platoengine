/*
 * Plato_SromStatistics.hpp
 *
 *  Created on: Jan 31, 2018
 */

#ifndef PLATO_SROMSTATISTICS_HPP_
#define PLATO_SROMSTATISTICS_HPP_

#include <cmath>
#include <cassert>

#include "Plato_Vector.hpp"

#define _MATH_DEFINES_DEFINED

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
inline ScalarType compute_srom_cdf(const ScalarType & aX,
                                   const ScalarType & aSigma,
                                   const Plato::Vector<ScalarType, OrdinalType> & aSamples,
                                   const Plato::Vector<ScalarType, OrdinalType> & aSamplesProbability)
{
    ScalarType tSum = 0;
    OrdinalType tNumSamples = aSamples.size();
    for(OrdinalType tIndexJ = 0; tIndexJ < tNumSamples; tIndexJ++)
    {
        ScalarType tValue = (aX - aSamples[tIndexJ]) / (aSigma * std::sqrt(static_cast<ScalarType>(2)));
        tSum = tSum + aSamplesProbability[tIndexJ] *
                (static_cast<ScalarType>(0.5) * (static_cast<ScalarType>(1) + std::erf(tValue)));
    }
    return (tSum);
}

template<typename ScalarType, typename OrdinalType = size_t>
inline ScalarType compute_srom_moment(const ScalarType & aOrder,
                                      const Plato::Vector<ScalarType, OrdinalType> & aSamples,
                                      const Plato::Vector<ScalarType, OrdinalType> & aSamplesProbability)
{
    assert(aOrder >= static_cast<OrdinalType>(0));
    assert(aSamples.size() == aSamplesProbability.size());

    ScalarType tOutput = 0;
    OrdinalType tNumSamples = aSamples.size();
    for(OrdinalType tSampleIndex = 0; tSampleIndex < tNumSamples; tSampleIndex++)
    {
        tOutput = tOutput + (aSamplesProbability[tSampleIndex] * std::pow(aSamples[tSampleIndex], aOrder));
    }
    return (tOutput);
}

} // namespace Plato

#endif /* PLATO_SROMSTATISTICS_HPP_ */
