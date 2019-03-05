/*
 * Plato_DistributedReductionOperations.hpp
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

#ifndef PLATO_DISTRIBUTEDREDUCTIONOPERATIONS_HPP_
#define PLATO_DISTRIBUTEDREDUCTIONOPERATIONS_HPP_

#include <mpi.h>

#include <vector>
#include <cassert>
#include <algorithm>

#include "Plato_Vector.hpp"
#include "Plato_ReductionOperations.hpp"

namespace Plato
{

/******************************************************************************//**
 * @brief Common parallel programming operations used to reduce the elements
 *        of an array into a single result.
**********************************************************************************/
template<typename ScalarType, typename OrdinalType = size_t>
class DistributedReductionOperations : public Plato::ReductionOperations<ScalarType, OrdinalType>
{
public:
    /******************************************************************************//**
     * @brief Constructor
     * @param [in] MPI communicator (default = MPI_COMM_WORLD)
    **********************************************************************************/
    DistributedReductionOperations(MPI_Comm aComm = MPI_COMM_WORLD) :
            mComm(aComm)
    {
    }

    /******************************************************************************//**
     * @brief Destructor
    **********************************************************************************/
    virtual ~DistributedReductionOperations()
    {
    }

    /******************************************************************************//**
     * @brief Returns the global maximum element in range.
     * @param [in] aInput array of elements
     * @return global maximum value
    **********************************************************************************/
    ScalarType max(const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.size() > 0);

        const ScalarType tValue = 0;
        const OrdinalType tSize = aInput.size();
        std::vector<ScalarType> tCopy(tSize, tValue);
        for(OrdinalType tIndex = 0; tIndex < tSize; tIndex++)
        {
            tCopy[tIndex] = aInput[tIndex];
        }
        ScalarType aLocalMaxValue = *std::max_element(tCopy.begin(), tCopy.end());

        ScalarType aGlobalMaxValue = 0;
        MPI_Allreduce(&aLocalMaxValue, &aGlobalMaxValue, 1, MPI_DOUBLE, MPI_MAX, mComm);

        return (aGlobalMaxValue);
    }

    /******************************************************************************//**
     * @brief Returns the global minimum element in range.
     * @param [in] aInput array of elements
     * @return global minimum value
    **********************************************************************************/
    ScalarType min(const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.size() > 0);

        const ScalarType tValue = 0;
        const OrdinalType tSize = aInput.size();
        std::vector<ScalarType> tCopy(tSize, tValue);
        for(OrdinalType tIndex = 0; tIndex < tSize; tIndex++)
        {
            tCopy[tIndex] = aInput[tIndex];
        }
        ScalarType aLocalMinValue = *std::min_element(tCopy.begin(), tCopy.end());

        ScalarType aGlobalMinValue = 0;
        MPI_Allreduce(&aLocalMinValue, &aGlobalMinValue, 1, MPI_DOUBLE, MPI_MIN, mComm);

        return (aGlobalMinValue);
    }

    /******************************************************************************//**
     * @brief Returns the global sum of all the elements in the container.
     * @param [in] aInput array of elements
     * @return global sum
    **********************************************************************************/
    ScalarType sum(const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        assert(aInput.size() > 0);

        const ScalarType tValue = 0;
        const OrdinalType tSize = aInput.size();
        std::vector<ScalarType> tCopy(tSize, tValue);
        for(OrdinalType tIndex = 0; tIndex < tSize; tIndex++)
        {
            tCopy[tIndex] = aInput[tIndex];
        }

        ScalarType tBaseValue = 0;
        ScalarType tLocalSum = std::accumulate(tCopy.begin(), tCopy.end(), tBaseValue);

        ScalarType tGlobalSum = 0;
        MPI_Allreduce(&tLocalSum, &tGlobalSum, 1, MPI_DOUBLE, MPI_SUM, mComm);

        return (tGlobalSum);
    }

    /******************************************************************************//**
     * @brief Computes a global minimum and also the index attached to the minimum value.
     * @param [in] aInput array of elements
     * @param [out] aOutput struct with global minimum and the rank and index attached to the minimum
    **********************************************************************************/
    void minloc(const Plato::Vector<ScalarType, OrdinalType> & aInput,
                Plato::ReductionOutputs<ScalarType, OrdinalType> & aOutput) const
    {
        struct
        {
            ScalarType mValue;
            int mIndex;
        } tInput, tOutput;

        /* local minloc */
        tInput.mIndex = 0;
        tInput.mValue = aInput[0];
        const OrdinalType tMyNumElements = aInput.size(); /* local num elements */
        for(OrdinalType tIndex = 0; tIndex < tMyNumElements; tIndex++)
        {
            if(aInput[tIndex] < tInput.mValue)
            {
                tInput.mValue = aInput[tIndex];
                tInput.mIndex = tIndex;
            }
        }

        /* global minloc */
        int tMyRank = 0;
        MPI_Comm_rank(mComm, &tMyRank);
        tInput.mIndex = tMyRank * tMyNumElements + tInput.mIndex;
        MPI_Allreduce(&tInput, &tOutput, 1, MPI_DOUBLE_INT, MPI_MINLOC, mComm);

        aOutput.mOutputValue = tOutput.mValue;
        aOutput.mOutputRank = tOutput.mIndex / tMyNumElements;
        aOutput.mOutputIndex = tOutput.mIndex % tMyNumElements;
    }

    /******************************************************************************//**
     * @brief Returns a copy of a ReductionOperations instance
     * @return copy of this instance
    **********************************************************************************/
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> create() const
    {
        std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> tCopy =
                std::make_shared<DistributedReductionOperations<ScalarType, OrdinalType>>(mComm);
        return (tCopy);
    }

    /******************************************************************************//**
     * @brief Return total number of ranks (i.e. processes)
     * @return total number of ranks
    **********************************************************************************/
    OrdinalType getNumRanks() const
    {
        int tNumRanks = 0;
        MPI_Comm_size(mComm, &tNumRanks);
        assert(tNumRanks > static_cast<int>(0));
        return (tNumRanks);
    }

private:
    MPI_Comm mComm; /*!< MPI communicator */

private:
    DistributedReductionOperations(const Plato::DistributedReductionOperations<ScalarType, OrdinalType> &);
    Plato::DistributedReductionOperations<ScalarType, OrdinalType> & operator=(const Plato::DistributedReductionOperations<ScalarType, OrdinalType> &);
};
// class DistributedReductionOperations

} // namespace Plato

#endif /* PLATO_DISTRIBUTEDREDUCTIONOPERATIONS_HPP_ */
