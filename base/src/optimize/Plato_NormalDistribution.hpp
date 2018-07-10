/*
 * Plato_NormalDistribution.hpp
 *
 *  Created on: Jan 31, 2018
 */

#ifndef PLATO_NORMALDISTRIBUTION_HPP_
#define PLATO_NORMALDISTRIBUTION_HPP_

#include <cmath>

#include "Plato_Distribution.hpp"

#define _MATH_DEFINES_DEFINED

namespace Plato
{

template<typename ScalarType>
inline ScalarType normal_pdf(const ScalarType & aValue, const ScalarType & aMean, const ScalarType & aSigma)
{
    ScalarType tConstant = static_cast<ScalarType>(1)
            / std::sqrt(static_cast<ScalarType>(2) * static_cast<ScalarType>(M_PI) * aSigma * aSigma);
    ScalarType tExponential = std::exp(static_cast<ScalarType>(-1) * (aValue - aMean) * (aValue - aMean)
            / (static_cast<ScalarType>(2) * aSigma * aSigma));
    ScalarType tOutput = tConstant * tExponential;
    return (tOutput);
}

template<typename ScalarType>
inline ScalarType normal_cdf(const ScalarType & aValue, const ScalarType & aMean, const ScalarType & aSigma)
{
    ScalarType tOutput = static_cast<ScalarType>(0.5) * (static_cast<ScalarType>(1)
            + std::erf((aValue - aMean) / (aSigma * std::sqrt(static_cast<ScalarType>(2)))));
    return (tOutput);
}

template<typename ScalarType, typename OrdinalType = size_t>
inline ScalarType normal_moment(const OrdinalType & aOrder, const ScalarType & aMean, const ScalarType & aSigma)
{
    ScalarType tOutput = 0;
    switch(aOrder)
    {
        case static_cast<OrdinalType>(0):
        {
            tOutput = 1;
            break;
        }
        case static_cast<OrdinalType>(1):
        {
            tOutput = aMean;
            break;
        }
        case static_cast<OrdinalType>(2):
        {
            tOutput = aMean * aMean + aSigma * aSigma;
            break;
        }
        case static_cast<OrdinalType>(3):
        {
            tOutput = aMean * aMean * aMean + static_cast<ScalarType>(3) * aMean * aSigma * aSigma;
            break;
        }
        case static_cast<OrdinalType>(4):
        {
            tOutput = aMean * aMean * aMean * aMean + static_cast<ScalarType>(6) * aMean * aMean * aSigma * aSigma
                    + static_cast<ScalarType>(3) * aSigma * aSigma * aSigma * aSigma;
            break;
        }
        case static_cast<OrdinalType>(5):
        {
            tOutput = aMean * aMean * aMean * aMean * aMean
                    + static_cast<ScalarType>(10) * aMean * aMean * aMean * aSigma * aSigma
                    + static_cast<ScalarType>(15) * aMean * aSigma * aSigma * aSigma * aSigma;
            break;
        }
        case static_cast<OrdinalType>(6):
        {
            tOutput = aMean * aMean * aMean * aMean * aMean * aMean
                    + static_cast<ScalarType>(15) * aMean * aMean * aMean * aMean * aSigma * aSigma
                    + static_cast<ScalarType>(45) * aMean * aMean * aSigma * aSigma * aSigma * aSigma
                    + static_cast<ScalarType>(15) * aSigma * aSigma * aSigma * aSigma * aSigma * aSigma;
            break;
        }
        case static_cast<OrdinalType>(7):
        {
            tOutput = aMean * aMean * aMean * aMean * aMean * aMean * aMean
                    + static_cast<ScalarType>(21) * aMean * aMean * aMean * aMean * aMean * aSigma * aSigma
                    + static_cast<ScalarType>(105) * aMean * aMean * aMean * aSigma * aSigma * aSigma * aSigma
                    + static_cast<ScalarType>(105) * aMean * aSigma * aSigma * aSigma * aSigma * aSigma * aSigma;
            break;
        }
        case static_cast<OrdinalType>(8):
        {
            tOutput = aMean * aMean * aMean * aMean * aMean * aMean * aMean * aMean
                    + static_cast<ScalarType>(28) * aMean * aMean * aMean * aMean * aMean * aMean * aSigma * aSigma
                    + static_cast<ScalarType>(210) * aMean * aMean * aMean * aMean * aSigma * aSigma * aSigma * aSigma
                    + static_cast<ScalarType>(420) * aMean * aMean * aSigma * aSigma * aSigma * aSigma * aSigma * aSigma
                    + static_cast<ScalarType>(105) * aSigma * aSigma * aSigma * aSigma * aSigma * aSigma * aSigma * aSigma;
            break;
        }
        default:
        {
            std::cout << "\n\n********** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __func__ << ", LINE: " << __LINE__
            << ", MSG: Moment of Order = " << aOrder << " is NOT Defined **********\n\n";
            std::abort();
            break;
        }
    }
    return (tOutput);
}

template<typename ScalarType, typename OrdinalType = size_t>
class NormalDistribution : public Plato::Distribution<ScalarType, OrdinalType>
{
public:
    explicit NormalDistribution(const ScalarType & aMean, const ScalarType & aStandardDeviation) :
            mMean(aMean),
            mStandardDeviation(aStandardDeviation)
    {
    }

    ScalarType mean() const
    {
        return (mMean);
    }

    ScalarType sigma() const
    {
        return (mStandardDeviation);
    }

    ScalarType pdf(const ScalarType & aInput)
    {
        ScalarType tOutput = Plato::normal_pdf<ScalarType>(aInput, mMean, mStandardDeviation);
        return (tOutput);
    }

    ScalarType cdf(const ScalarType & aInput)
    {
        ScalarType tOutput = Plato::normal_cdf<ScalarType>(aInput, mMean, mStandardDeviation);
        return (tOutput);
    }

    ScalarType moment(const OrdinalType & aInput)
    {
        ScalarType tOutput = Plato::normal_moment<ScalarType, OrdinalType>(aInput, mMean, mStandardDeviation);
        return (tOutput);
    }

private:
    ScalarType mMean;
    ScalarType mStandardDeviation;

private:
    NormalDistribution(const Plato::NormalDistribution<ScalarType, OrdinalType> & aRhs);
    Plato::NormalDistribution<ScalarType, OrdinalType> & operator=(const Plato::NormalDistribution<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_NORMALDISTRIBUTION_HPP_ */
