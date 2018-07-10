/*
 * Plato_DistributedVectorROL.hpp
 *
 *  Created on: Feb 8, 2018
 */

#ifndef PLATO_DISTRIBUTEDVECTORROL_HPP_
#define PLATO_DISTRIBUTEDVECTORROL_HPP_

#include <mpi.h>

#include <cmath>
#include <vector>
#include <numeric>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <stdexcept>

#include "Teuchos_RCP.hpp"

#include "ROL_Vector.hpp"
#include "ROL_Elementwise_Reduce.hpp"

namespace Plato
{

template<typename ScalarType>
class DistributedVectorROL : public ROL::Vector<ScalarType>
{
public:
    /******************************************************************************/
    explicit DistributedVectorROL(const MPI_Comm & aComm, const std::vector<ScalarType> & aInput) :
            mComm(aComm),
            mGlobalDimension(0),
            mData(aInput)
    /******************************************************************************/
    {
        this->initialize();
    }
    /******************************************************************************/
    explicit DistributedVectorROL(const MPI_Comm & aComm, const size_t & aLocalNumElements, ScalarType aValue = 0) :
            mComm(aComm),
            mGlobalDimension(0),
            mData(std::vector<ScalarType>(aLocalNumElements, aValue))
    /******************************************************************************/
    {
        this->initialize();
    }

    /******************************************************************************/
    virtual ~DistributedVectorROL()
    /******************************************************************************/
    {
    }

    /******************************************************************************/
    void plus(const ROL::Vector<ScalarType> & aInput) override
    /******************************************************************************/
    {
        const Plato::DistributedVectorROL<ScalarType> & tInput =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aInput);
        assert(tInput.mData.size() == mData.size());
        assert(tInput.dimension() == mGlobalDimension);

        size_t tDimension = tInput.mData.size();
        for(size_t tIndex = 0; tIndex < tDimension; tIndex++)
        {
            mData[tIndex] = mData[tIndex] + tInput.mData[tIndex];
        }
    }

    /******************************************************************************/
    void scale(const ScalarType aInput) override
    /******************************************************************************/
    {
        size_t tDimension = mData.size();
        for(size_t tIndex = 0; tIndex < tDimension; tIndex++)
        {
            mData[tIndex] = mData[tIndex] * aInput;
        }
    }

    /******************************************************************************/
    ScalarType dot(const ROL::Vector<ScalarType> & aInput) const override
    /******************************************************************************/
    {
        const Plato::DistributedVectorROL<ScalarType>& tInput =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aInput);
        assert(tInput.mData.size() == mData.size());
        assert(tInput.dimension() == mGlobalDimension);

        ScalarType tBaseValue = 0;
        ScalarType tLocalInnerProduct = std::inner_product(mData.begin(), mData.end(), tInput.mData.begin(), tBaseValue);

        ScalarType tGlobalInnerProduct = 0;
        MPI_Allreduce(&tLocalInnerProduct, &tGlobalInnerProduct, 1, MPI_DOUBLE, MPI_SUM, mComm);

        return (tGlobalInnerProduct);
    }

    /******************************************************************************/
    ScalarType norm() const override
    /******************************************************************************/
    {
        ScalarType tBaseValue = 0;
        ScalarType tLocalInnerProduct = std::inner_product(mData.begin(), mData.end(), mData.begin(), tBaseValue);

        ScalarType tGlobalInnerProduct = 0;
        MPI_Allreduce(&tLocalInnerProduct, &tGlobalInnerProduct, 1, MPI_DOUBLE, MPI_SUM, mComm);

        ScalarType tOutput = std::sqrt(tGlobalInnerProduct);
        return (tOutput);
    }

    /******************************************************************************/
    Teuchos::RCP<ROL::Vector<ScalarType>> clone() const override
    /******************************************************************************/
    {
        return Teuchos::rcp(new Plato::DistributedVectorROL<ScalarType>(mComm, std::vector<ScalarType>(mData.size())));
    }

    /******************************************************************************/
    int dimension() const override
    /******************************************************************************/
    {
        assert(mGlobalDimension > static_cast<int>(0));
        return (mGlobalDimension);
    }

    /******************************************************************************/
    ScalarType reduce(const ROL::Elementwise::ReductionOp<ScalarType> & aReductionOperations) const override
    /******************************************************************************/
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

    /******************************************************************************/
    void applyUnary(const ROL::Elementwise::UnaryFunction<double> & aFunction)
    /******************************************************************************/
    {
        size_t tLength = mData.size();
        for(size_t tIndex = 0; tIndex < tLength; tIndex++)
        {
            mData[tIndex] = aFunction.apply(mData[tIndex]);
        }
    }

    /******************************************************************************/
    void applyBinary(const ROL::Elementwise::BinaryFunction<ScalarType> & aFunction, const ROL::Vector<ScalarType> & aInput)
    /******************************************************************************/
    {
        assert(this->dimension() == aInput.dimension());
        const Plato::DistributedVectorROL<ScalarType>& tInput =
                dynamic_cast<const Plato::DistributedVectorROL<ScalarType>&>(aInput);
        size_t tLength = mData.size();
        for(size_t tIndex = 0; tIndex < tLength; tIndex++)
        {
            mData[tIndex] = aFunction.apply(mData[tIndex], tInput.mData[tIndex]);
        }
    }

    /******************************************************************************/
    std::vector<ScalarType> & vector()
    /******************************************************************************/
    {
        return (mData);
    }

    /******************************************************************************/
    const std::vector<ScalarType> & vector() const
    /******************************************************************************/
    {
        return (mData);
    }

    /******************************************************************************/
    void set(const std::vector<ScalarType> & aInput)
    /******************************************************************************/
    {
        mData = aInput;
    }

    /******************************************************************************/
    void fill(const ScalarType & aInput)
    /******************************************************************************/
    {
        std::fill(mData.begin(), mData.end(), aInput);
    }

private:
    /******************************************************************************/
    void initialize()
    /******************************************************************************/
    {
        int tLocalValue = mData.size();
        MPI_Allreduce(&tLocalValue, &mGlobalDimension, 1, MPI_INT, MPI_SUM, mComm);
    }

    /******************************************************************************/
    ScalarType min() const
    /******************************************************************************/
    {
        ScalarType tGlobalMinValue = 0;
        ScalarType tLocalMinValue = *std::min_element(mData.begin(), mData.end());
        MPI_Allreduce(&tLocalMinValue, &tGlobalMinValue, 1, MPI_DOUBLE, MPI_MIN, mComm);
        return (tGlobalMinValue);
    }

    /******************************************************************************/
    ScalarType max() const
    /******************************************************************************/
    {
        ScalarType tGlobalMaxValue = 0;
        ScalarType tLocalMaxValue = *std::max_element(mData.begin(), mData.end());
        MPI_Allreduce(&tLocalMaxValue, &tGlobalMaxValue, 1, MPI_DOUBLE, MPI_MAX, mComm);
        return (tGlobalMaxValue);
    }

    /******************************************************************************/
    ScalarType sum() const
    /******************************************************************************/
    {
        ScalarType tBaseValue = 0;
        ScalarType tLocalSumValue = std::accumulate(mData.begin(), mData.end(), tBaseValue);
        ScalarType tGlobalSumValue = 0;
        MPI_Allreduce(&tLocalSumValue, &tGlobalSumValue, 1, MPI_DOUBLE, MPI_SUM, mComm);
        return (tGlobalSumValue);
    }

private:
    MPI_Comm mComm;
    int mGlobalDimension;
    std::vector<ScalarType> mData;

private:
    DistributedVectorROL(const Plato::DistributedVectorROL<ScalarType> & aRhs);
    Plato::DistributedVectorROL<ScalarType> & operator=(const Plato::DistributedVectorROL<ScalarType> & aRhs);
};

} // namespace Plato

#endif /* PLATO_DISTRIBUTEDVECTORROL_HPP_ */
