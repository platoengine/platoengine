/*
 * Plato_DistributionFactory.hpp
 *
 *  Created on: Aug 28, 2018
 */

#ifndef PLATO_DISTRIBUTIONFACTORY_HPP_
#define PLATO_DISTRIBUTIONFACTORY_HPP_

#include "Plato_BetaDistribution.hpp"
#include "Plato_UniformDistribution.hpp"
#include "Plato_NormalDistribution.hpp"

#include <cmath>
#include <stdlib.h>

namespace Plato
{

struct DistrubtionName
{
    enum type_t
    {
        beta = 0,
        normal = 1,
        uniform = 2,
    };
};
// struct DistrubtionName

template<typename ScalarType, typename OrdinalType>
struct UncertaintyInputStruct
{
    DistrubtionName::type_t mDistribution;
    ScalarType mMean;
    ScalarType mLowerBound;
    ScalarType mUpperBound;
    ScalarType mVariance;
    OrdinalType mNumSamples;
    OrdinalType mMaxNumDistributionMoments; // if zero, then use default

    UncertaintyInputStruct() :   // default Constructor
            mDistribution(DistrubtionName::type_t::beta),
            mMean(0.),
            mLowerBound(0.),
            mUpperBound(0.),
            mVariance(0.),
            mNumSamples(0),
            mMaxNumDistributionMoments(0)
    {
    }
};
// struct UncertaintyInputStruct

// factory constructor
template<typename ScalarType, typename OrdinalType>
std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>>
build_distrubtion(const Plato::UncertaintyInputStruct<ScalarType, OrdinalType>& aInput)
{
    // grab values from input struct
    const ScalarType tMean = aInput.mMean;
    const ScalarType tMax = aInput.mUpperBound;
    const ScalarType tMin = aInput.mLowerBound;
    const ScalarType tVariance = aInput.mVariance;
    const ScalarType tStandardDeviation = std::sqrt(tVariance);

    // make
    std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> tDistribution;
    switch(aInput.mDistribution)
    {
        case DistrubtionName::type_t::beta:
        {
            tDistribution = std::make_shared < Plato::BetaDistribution<ScalarType, OrdinalType> > (tMin, tMax, tMean, tVariance);
            break;
        }
        case DistrubtionName::type_t::normal:
        {
            tDistribution = std::make_shared < Plato::NormalDistribution<ScalarType, OrdinalType> > (tMean, tStandardDeviation);
            break;
        }
        case DistrubtionName::type_t::uniform:
        {
            tDistribution = std::make_shared < Plato::UniformDistribution<ScalarType, OrdinalType> > (tMin, tMax);
            break;
        }
        default:
        {
            std::abort();
            break;
        }
    }

    return tDistribution;
}
// function build_distrubtion

} // namespace Plato

#endif /* PLATO_DISTRIBUTIONFACTORY_HPP_ */
