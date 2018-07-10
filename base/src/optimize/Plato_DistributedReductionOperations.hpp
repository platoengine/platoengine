/*
 * Plato_DistributedReductionOperations.hpp
 *
 *  Created on: Oct 17, 2017
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

template<typename ScalarType, typename OrdinalType = size_t>
class DistributedReductionOperations : public Plato::ReductionOperations<ScalarType, OrdinalType>
{
public:
    DistributedReductionOperations(MPI_Comm aComm = MPI_COMM_WORLD) :
            mComm(aComm)
    {
    }
    virtual ~DistributedReductionOperations()
    {
    }

    //! Returns the maximum element in range
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
    //! Returns the minimum element in range
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
    //! Returns the sum of all the elements in container.
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
    //! Creates an instance of type Plato::ReductionOperations
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> create() const
    {
        std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> tCopy =
                std::make_shared<DistributedReductionOperations<ScalarType, OrdinalType>>(mComm);
        return (tCopy);
    }
    //! Return number of ranks (i.e. processes)
    OrdinalType getNumRanks() const
    {
        int tNumRanks = 0;
        MPI_Comm_size(mComm, &tNumRanks);
        assert(tNumRanks > static_cast<int>(0));
        return (tNumRanks);
    }

private:
    MPI_Comm mComm;

private:
    DistributedReductionOperations(const Plato::DistributedReductionOperations<ScalarType, OrdinalType> &);
    Plato::DistributedReductionOperations<ScalarType, OrdinalType> & operator=(const Plato::DistributedReductionOperations<ScalarType, OrdinalType> &);
};

}

#endif /* PLATO_DISTRIBUTEDREDUCTIONOPERATIONS_HPP_ */
