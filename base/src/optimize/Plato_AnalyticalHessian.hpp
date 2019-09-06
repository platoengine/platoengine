/*
 * Plato_AnalyticalHessian.hpp
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
//
// *************************************************************************
//@HEADER
*/

#ifndef PLATO_ANALYTICALHESSIAN_HPP_
#define PLATO_ANALYTICALHESSIAN_HPP_

#include <memory>

#include "Plato_Criterion.hpp"
#include "Plato_StateData.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_LinearAlgebra.hpp"
#include "Plato_LinearOperator.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Analytical Hessian interface
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class AnalyticalHessian : public Plato::LinearOperator<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aCriterion problem criterion
    **********************************************************************************/
    explicit AnalyticalHessian(const std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> & aCriterion) :
            mCriterion(aCriterion)
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~AnalyticalHessian()
    {
    }

    /******************************************************************************//**
     * @brief Update state-dependent data needed to compute the Hessian
     * @param [in] aStateData state data, e.g. controls, gradient, etc.
    **********************************************************************************/
    void update(const Plato::StateData<ScalarType, OrdinalType> & aStateData)
    {
        return;
    }

    /******************************************************************************//**
     * @brief Apply vector to analytical Hessian
     * @param [in] aControl design variables
     * @param [in] aOutput application of the input vector to analytical Hessian
    **********************************************************************************/
    void apply(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
               const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
               Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        Plato::fill(static_cast<ScalarType>(0), aOutput);
        mCriterion->hessian(aControl, aVector, aOutput);
    }

private:
    std::shared_ptr<Plato::Criterion<ScalarType, OrdinalType>> mCriterion;  /*!< criterion interface */

private:
    AnalyticalHessian(const Plato::AnalyticalHessian<ScalarType, OrdinalType> & aRhs);
    Plato::AnalyticalHessian<ScalarType, OrdinalType> & operator=(const Plato::AnalyticalHessian<ScalarType, OrdinalType> & aRhs);
};
// class AnalyticalHessian

}
// namespace Plato

#endif /* PLATO_ANALYTICALHESSIAN_HPP_ */
