/*
 * Plato_CentralDifferenceHessian.hpp
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
//
// *************************************************************************
//@HEADER
*/

#ifndef PLATO_CENTRALDIFFERENCEHESSIAN_HPP_
#define PLATO_CENTRALDIFFERENCEHESSIAN_HPP_

#include "Plato_StateData.hpp"
#include "Plato_Criterion.hpp"
#include "Plato_DataFactory.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_LinearOperator.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class CentralDifferenceHessian : public Plato::LinearOperator<ScalarType, OrdinalType>
{
public:
    explicit CentralDifferenceHessian(const std::shared_ptr<Plato::DataFactory<ScalarType, OrdinalType>> & aFactory,
                                      const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aCriterion) :
            mEpsilon(1e-6),
            mCriterion(aCriterion),
            mGradient(aFactory->control().create()),
            mTrialControl(aFactory->control().create())
    {
    }
    virtual ~CentralDifferenceHessian()
    {
    }

    void setEpsilon(const ScalarType & aInput)
    {
        mEpsilon = aInput;
    }

    void update(const Plato::StateData<ScalarType, OrdinalType> & aStateData)
    {
        return;
    }

    void apply(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
               const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
               Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        // Compute \hat{x} = x_{current} + \epsilon d, where d denotes the direction vector and \epsilon denotes the perturbation
        Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *mTrialControl);
        Plato::update(mEpsilon, aVector, static_cast<ScalarType>(1), *mTrialControl);
        mCriterion->gradient(*mTrialControl, aOutput);

        // Compute \hat{x} = x_{current} - \epsilon d, where d denotes the direction vector and \epsilon denotes the perturbation
        Plato::update(static_cast<ScalarType>(1), aControl, static_cast<ScalarType>(0), *mTrialControl);
        Plato::update(-mEpsilon, aVector, static_cast<ScalarType>(1), *mTrialControl);
        mCriterion->gradient(*mTrialControl, *mGradient);

        // Compute central difference approximation
        Plato::update(static_cast<ScalarType>(-1), *mGradient, static_cast<ScalarType>(1), aOutput);
        const ScalarType tConstant = static_cast<ScalarType>(0.5) / mEpsilon;
        Plato::scale(tConstant, aOutput);
    }

private:
    ScalarType mEpsilon;
    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mCriterion;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mGradient;
    std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> mTrialControl;

private:
    CentralDifferenceHessian(const Plato::CentralDifferenceHessian<ScalarType, OrdinalType> & aRhs);
    Plato::CentralDifferenceHessian<ScalarType, OrdinalType> & operator=(const Plato::CentralDifferenceHessian<ScalarType, OrdinalType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_CENTRALDIFFERENCEHESSIAN_HPP_ */
