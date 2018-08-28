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

struct DistrubtionName {
enum type_t {
    beta,
    normal,
    uniform,
};
};

template<typename ScalarType, typename OrdinalType>
struct UncertaintyInputStruct
{
    DistrubtionName::type_t distribution;
    ScalarType mean;
    ScalarType lower_bound;
    ScalarType upper_bound;
    ScalarType variance;
    OrdinalType num_samples;
    OrdinalType max_num_distribution_moments; // if zero, then use default

    UncertaintyInputStruct() :   // default Constructor
            distribution(DistrubtionName::type_t::beta),
            mean(1.),
            lower_bound(0.),
            upper_bound(1.),
            variance(1.),
            num_samples(1),
            max_num_distribution_moments(0)
    {
    }
};

// factory constructor
template<typename ScalarType, typename OrdinalType>
std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> build_distrubtion(const Plato::UncertaintyInputStruct<ScalarType,
        OrdinalType>& aInput)

{
    // grab values from input struct
    const ScalarType tMean = aInput.mean;
    const ScalarType tMax = aInput.upper_bound;
    const ScalarType tMin = aInput.lower_bound;
    const ScalarType tVariance = aInput.variance;
    const ScalarType tStandardDeviation = sqrt(tVariance);

    // make
    std::shared_ptr<Plato::Distribution<ScalarType, OrdinalType>> tDistribution;
    switch(aInput.distribution)
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


}

#endif /* PLATO_DISTRIBUTIONFACTORY_HPP_ */
