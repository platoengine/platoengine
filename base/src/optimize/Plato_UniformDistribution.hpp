/*
 * Plato_UniformDistribution.hpp
 *
 *  Created on: Feb 1, 2018
 */

/*
//@HEADER
// *************************************************************************
//   Plato Engine v.1.0: Copyright 2018, National Technology & Engineering
//                    Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Sandia Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact the Plato team (plato3D-help@sandia.gov)
// *************************************************************************
//@HEADER
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
