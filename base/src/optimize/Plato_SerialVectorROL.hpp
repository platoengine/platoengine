/*
 * Plato_SerialVectorROL.hpp
 *
 *  Created on: Feb 27, 2018
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
                std::ostringstream tErrorMsg;
                tErrorMsg << "****** ERROR IN " << __FILE__ << ", FUNCTION: " << __FUNCTION__ << ", LINE: " << __LINE__
                          << ", LOGICAL REDUCE AND IS NOT IMPLEMENTED ******\n" << std::flush;
                std::runtime_error(tErrorMsg.str().c_str());
                break;
            }
        }

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
