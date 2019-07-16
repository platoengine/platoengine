/*
 * Plato_SerialVectorROL.hpp
 *
 *  Created on: Feb 27, 2018
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

#ifndef PLATO_SERIALVECTORROL_HPP_
#define PLATO_SERIALVECTORROL_HPP_

#include <vector>
#include <numeric>
#include <cassert>
#include <iostream>
#include <algorithm>

#include "Teuchos_RCP.hpp"
#include "Epetra_SerialDenseVector.h"

#include "ROL_Vector.hpp"
#include "ROL_Elementwise_Reduce.hpp"

namespace Plato
{

template<typename ScalarType>
class SerialVectorROL : public ROL::Vector<ScalarType>
{
public:
    explicit SerialVectorROL(const std::vector<ScalarType> & aInput) :
            mData(aInput)
    {
    }
    explicit SerialVectorROL(const int & aLength, ScalarType aValue = 0) :
            mData(std::vector<ScalarType>(aLength, aValue))
    {
    }
    virtual ~SerialVectorROL()
    {
    }

    void plus(const ROL::Vector<ScalarType> & aInput) override
    {
        const Plato::SerialVectorROL<ScalarType> & tInput = dynamic_cast<const Plato::SerialVectorROL<ScalarType>&>(aInput);
        assert(tInput.mData.size() == mData.size());
        for(size_t tIndex = 0; tIndex < mData.size(); tIndex++)
        {
            mData[tIndex] = mData[tIndex] + tInput.mData[tIndex];
        }
    }

    void scale(const ScalarType aInput) override
    {
        assert(mData.size() > static_cast<size_t>(0));
        for(size_t tIndex = 0; tIndex < mData.size(); tIndex++)
        {
            mData[tIndex] = mData[tIndex] * aInput;
        }
    }

    ScalarType dot(const ROL::Vector<ScalarType> & aInput) const override
    {
        const Plato::SerialVectorROL<ScalarType> & tInput = dynamic_cast<const Plato::SerialVectorROL<ScalarType>&>(aInput);
        assert(tInput.mData.size() == mData.size());
        ScalarType tBaseValue = 0;
        ScalarType tOutput = std::inner_product(mData.begin(), mData.end(), tInput.mData.begin(), tBaseValue);

        return (tOutput);
    }

    ScalarType norm() const override
    {
        assert(mData.size() > static_cast<size_t>(0));
        ScalarType tBaseValue = 0;
        ScalarType tOutput = std::inner_product(mData.begin(), mData.end(), mData.begin(), tBaseValue);
        tOutput = std::sqrt(tOutput);

        return (tOutput);
    }

    Teuchos::RCP<ROL::Vector<ScalarType>> clone() const override
    {
        const size_t tLength = mData.size();
        return Teuchos::rcp(new Plato::SerialVectorROL<ScalarType>(tLength));
    }

    int dimension() const override
    {
        assert(mData.size() > static_cast<size_t>(0));
        return (mData.size());
    }

    ScalarType reduce(const ROL::Elementwise::ReductionOp<ScalarType> & aReductionOperations) const override
    {
        ScalarType tOutput = 0;
#ifdef BUILD_IN_SIERRA // Handle different versions of Teuchos
        ROL::Elementwise::EReductionType tReductionType = aReductionOperations.reductionType();
        switch(tReductionType)
        {
            case ROL::Elementwise::EReductionType::REDUCE_SUM:
            {
                tOutput = this->sum();
                break;
            }
            case ROL::Elementwise::EReductionType::REDUCE_MAX:
            {
                tOutput = this->max();
                break;
            }
            case ROL::Elementwise::EReductionType::REDUCE_MIN:
            {
                tOutput = this->min();
                break;
            }
            default:
            case ROL::Elementwise::EReductionType::REDUCE_AND:
            {
                THROWERR("LOGICAL REDUCE AND OPERATION IS NOT IMPLEMENTED.\n")
                break;
            }
        }
#else
        Teuchos::EReductionType tReductionType = aReductionOperations.reductionType();
        switch(tReductionType)
        {
            case Teuchos::EReductionType::REDUCE_SUM:
            {
                tOutput = this->sum();
                break;
            }
            case Teuchos::EReductionType::REDUCE_MAX:
            {
                tOutput = this->max();
                break;
            }
            case Teuchos::EReductionType::REDUCE_MIN:
            {
                tOutput = this->min();
                break;
            }
            default:
            case Teuchos::EReductionType::REDUCE_AND:
            {
                THROWERR("LOGICAL REDUCE AND OPERATION IS NOT IMPLEMENTED.\n")
                break;
            }
        }
#endif

        return (tOutput);
    }

    void applyUnary(const ROL::Elementwise::UnaryFunction<double> & aFunction)
    {
        size_t tLength = mData.size();
        for(size_t tIndex = 0; tIndex < tLength; tIndex++)
        {
            mData[tIndex] = aFunction.apply(mData[tIndex]);
        }
    }

    void applyBinary(const ROL::Elementwise::BinaryFunction<ScalarType> & aFunction, const ROL::Vector<ScalarType> & aInput)
    {
        assert(this->dimension() == aInput.dimension());
        const Plato::SerialVectorROL<ScalarType>& tInput = dynamic_cast<const Plato::SerialVectorROL<ScalarType>&>(aInput);
        assert(tInput.mData.size() == mData.size());
        size_t tLength = mData.size();
        for(size_t tIndex = 0; tIndex < tLength; tIndex++)
        {
            mData[tIndex] = aFunction.apply(mData[tIndex], tInput.mData[tIndex]);
        }
    }

    std::vector<ScalarType> & vector()
    {
        return (mData);
    }

    const std::vector<ScalarType> & vector() const
    {
        return (mData);
    }

private:
    ScalarType min() const
    {
        size_t tLength = mData.size();
        ScalarType tMinValue = *std::min_element(mData.data(), mData.data() + tLength);
        return (tMinValue);
    }
    ScalarType max() const
    {
        size_t tLength = mData.size();
        ScalarType tMaxValue = *std::max_element(mData.data(), mData.data() + tLength);
        return (tMaxValue);
    }
    ScalarType sum() const
    {
        ScalarType tBaseValue = 0;
        size_t tLength = mData.size();
        ScalarType tSumValue = std::accumulate(mData.data(), mData.data() + tLength, tBaseValue);
        return (tSumValue);
    }

private:
    std::vector<ScalarType> mData;

private:
    SerialVectorROL(const Plato::SerialVectorROL<ScalarType> & aRhs);
    Plato::SerialVectorROL<ScalarType> & operator=(const Plato::SerialVectorROL<ScalarType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_SERIALVECTORROL_HPP_ */
