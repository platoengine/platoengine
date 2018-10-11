/*
 * Plato_TrustRegionStageMng.hpp
 *
 *  Created on: Oct 21, 2017
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

#ifndef PLATO_TRUSTREGIONSTAGEMNG_HPP_
#define PLATO_TRUSTREGIONSTAGEMNG_HPP_

#include <limits>

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class MultiVector;
template<typename ScalarType, typename OrdinalType>
class TrustRegionAlgorithmDataMng;

template<typename ScalarType, typename OrdinalType = size_t>
class TrustRegionStageMng
{
public:
    TrustRegionStageMng() :
        mHaveHessian(true)
    {
    }

    virtual ~TrustRegionStageMng()
    {
    }

    void setHaveHessian(const bool& aInput)
    {
        mHaveHessian = aInput;
    }
    bool getHaveHessian() const
    {
        return mHaveHessian;
    }

    virtual void cacheData() = 0;
    virtual void updateOptimizationData(Plato::TrustRegionAlgorithmDataMng<ScalarType, OrdinalType> & aDataMng) = 0;
    virtual ScalarType evaluateObjective(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                          ScalarType aTolerance = std::numeric_limits<ScalarType>::max()) = 0;
    virtual void computeGradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
    virtual void applyVectorToHessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                      const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                      Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
    virtual void applyVectorToPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                             const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                             Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;
    virtual void applyVectorToInvPreconditioner(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                                                const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                                                Plato::MultiVector<ScalarType, OrdinalType> & aOutput) = 0;

private:
    bool mHaveHessian;
};

} // namespace Plato

#endif /* PLATO_TRUSTREGIONSTAGEMNG_HPP_ */
