/*
 * Plato_OptimalityCriteriaTestObjectiveOne.hpp
 *
 *  Created on: Oct 17, 2017
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

#ifndef PLATO_OPTIMALITYCRITERIATESTOBJECTIVEONE_HPP_
#define PLATO_OPTIMALITYCRITERIATESTOBJECTIVEONE_HPP_

#include <memory>

#include "Plato_Criterion.hpp"
#include "Plato_MultiVector.hpp"
#include "Plato_StandardVectorReductionOperations.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class OptimalityCriteriaTestObjectiveOne : public Plato::Criterion<ScalarType, OrdinalType>
{
public:
    OptimalityCriteriaTestObjectiveOne() :
            mConstant(0.0624),
            mReductionOperations(std::make_shared<Plato::StandardVectorReductionOperations<ScalarType,OrdinalType>>())
    {

    }
    explicit OptimalityCriteriaTestObjectiveOne(const Plato::ReductionOperations<ScalarType, OrdinalType> & aInterface) :
            mConstant(0.0624),
            mReductionOperations(aInterface.create())
    {
    }
    virtual ~OptimalityCriteriaTestObjectiveOne()
    {
    }

    void cacheData()
    {
        return;
    }

    ScalarType value(const Plato::MultiVector<ScalarType, OrdinalType> & aControl)
    {
        const OrdinalType tVectorIndex = 0;
        ScalarType tSum = mReductionOperations->sum(aControl[tVectorIndex]);
        ScalarType tOutput = mConstant * tSum;
        return (tOutput);
    }

    void gradient(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                  Plato::MultiVector<ScalarType, OrdinalType> & aGradient)
    {
        const OrdinalType tVectorIndex = 0;
        aGradient[tVectorIndex].fill(mConstant);
    }

    void hessian(const Plato::MultiVector<ScalarType, OrdinalType> & aControl,
                 const Plato::MultiVector<ScalarType, OrdinalType> & aVector,
                 Plato::MultiVector<ScalarType, OrdinalType> & aOutput)
    {
        const ScalarType tScalarValue = 0;
        const OrdinalType tVectorIndex = 0;
        aOutput[tVectorIndex].fill(tScalarValue);
    }

private:
    ScalarType mConstant;
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> mReductionOperations;

private:
    OptimalityCriteriaTestObjectiveOne(const Plato::OptimalityCriteriaTestObjectiveOne<ScalarType, OrdinalType>&);
    Plato::OptimalityCriteriaTestObjectiveOne<ScalarType, OrdinalType> & operator=(const Plato::OptimalityCriteriaTestObjectiveOne<ScalarType, OrdinalType>&);
};

}

#endif /* PLATO_OPTIMALITYCRITERIATESTOBJECTIVEONE_HPP_ */
