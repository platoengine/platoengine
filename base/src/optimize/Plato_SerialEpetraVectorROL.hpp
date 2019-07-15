/*
 * Plato_SerialEpetraVectorROL.hpp
 *
 *  Created on: Feb 8, 2018
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

#ifndef PLATO_SERIALEPETRAVECTORROL_HPP_
#define PLATO_SERIALEPETRAVECTORROL_HPP_

#include <vector>
#include <numeric>
#include <cassert>
#include <iostream>
#include <algorithm>

#include "Teuchos_RCP.hpp"
#include "Plato_Macros.hpp"
#include "Epetra_SerialDenseVector.h"

#include "ROL_Vector.hpp"
#include "ROL_Elementwise_Reduce.hpp"

namespace Plato
{

template<typename ScalarType>
class SerialEpetraVectorROL : public ROL::Vector<ScalarType>
{
public:
    explicit SerialEpetraVectorROL(const std::vector<ScalarType> & aInput) :
            mLength(aInput.size()),
            mData(Epetra_SerialDenseVector(aInput.size()))
    {
        const size_t tLength = aInput.size();
        for(size_t tIndex = 0; tIndex < tLength; tIndex++)
        {
            mData[tIndex] = aInput[tIndex];
        }
    }
    explicit SerialEpetraVectorROL(const int & aLength, ScalarType aValue = 0) :
            mLength(aLength),
            mData(Epetra_SerialDenseVector(aLength))
    {
        assert(aLength > static_cast<int>(0));
        for(int tIndex = 0; tIndex < aLength; tIndex++)
        {
            mData[tIndex] = aValue;
        }
    }
    virtual ~SerialEpetraVectorROL()
    {
    }

    void plus(const ROL::Vector<ScalarType> & aInput) override
    {
        assert(aInput.dimension() == mData.Length());

        const Plato::SerialEpetraVectorROL<ScalarType> & tInput = dynamic_cast<const Plato::SerialEpetraVectorROL<ScalarType>&>(aInput);
        int tDimension = tInput.dimension();
        for(int tIndex = 0; tIndex < tDimension; tIndex++)
        {
            mData[tIndex] = mData[tIndex] + tInput.mData[tIndex];
        }
    }

    void scale(const ScalarType aInput) override
    {
        int tDimension = mData.Length();
        for(int tIndex = 0; tIndex < tDimension; tIndex++)
        {
            mData[tIndex] = mData[tIndex] * aInput;
        }
    }

    ScalarType dot(const ROL::Vector<ScalarType> & aInput) const override
    {
        assert(aInput.dimension() == mData.Length());

        const Plato::SerialEpetraVectorROL<ScalarType>& tInput = dynamic_cast<const Plato::SerialEpetraVectorROL<ScalarType>&>(aInput);
        const Epetra_SerialDenseVector & tInputVector = tInput.vector();
        const int tLength = tInputVector.Length();
        ScalarType tOutput = tInputVector.DOT(tLength, tInputVector.A(), mData.A());

        return (tOutput);
    }

    ScalarType norm() const override
    {
        assert(mData.Length() > static_cast<int>(0));
        ScalarType tOutput = mData.Norm2();

        return (tOutput);
    }

    Teuchos::RCP<ROL::Vector<ScalarType>> clone() const override
    {
        const int tLength = mData.Length();
        return Teuchos::rcp(new Plato::SerialEpetraVectorROL<ScalarType>(tLength));
    }

    int dimension() const override
    {
        assert(mData.Length() > static_cast<int>(0));
        return (mData.Length());
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
        int tLength = mData.Length();
        for(int tIndex = 0; tIndex < tLength; tIndex++)
        {
            mData[tIndex] = aFunction.apply(mData[tIndex]);
        }
    }

    void applyBinary(const ROL::Elementwise::BinaryFunction<ScalarType> & aFunction, const ROL::Vector<ScalarType> & aInput)
    {
        assert(this->dimension() == aInput.dimension());
        const Plato::SerialEpetraVectorROL<ScalarType>& tInput = dynamic_cast<const Plato::SerialEpetraVectorROL<ScalarType>&>(aInput);
        int tLength = mData.Length();
        for(int tIndex = 0; tIndex < tLength; tIndex++)
        {
            mData[tIndex] = aFunction.apply(mData[tIndex], tInput.mData[tIndex]);
        }
    }

    Epetra_SerialDenseVector & vector()
    {
        return (mData);
    }

    const Epetra_SerialDenseVector & vector() const
    {
        return (mData);
    }

private:
    ScalarType min() const
    {
        ScalarType tMinValue = *std::min_element(mData.A(), mData.A() + mLength);
        return (tMinValue);
    }
    ScalarType max() const
    {
        ScalarType tMaxValue = *std::max_element(mData.A(), mData.A() + mLength);
        return (tMaxValue);
    }
    ScalarType sum() const
    {
        ScalarType tBaseValue = 0;
        ScalarType tSumValue = std::accumulate(mData.A(), mData.A() + mLength, tBaseValue);
        return (tSumValue);
    }

private:
    int mLength;
    Epetra_SerialDenseVector mData;

private:
    SerialEpetraVectorROL(const Plato::SerialEpetraVectorROL<ScalarType> & aRhs);
    Plato::SerialEpetraVectorROL<ScalarType> & operator=(const Plato::SerialEpetraVectorROL<ScalarType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_SERIALEPETRAVECTORROL_HPP_ */
