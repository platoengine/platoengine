/*
 * Plato_StandardVectorReductionOperations.hpp
 *
 *  Created on: Oct 17, 2017
 */

#ifndef PLATO_STANDARDVECTORREDUCTIONOPERATIONS_HPP_
#define PLATO_STANDARDVECTORREDUCTIONOPERATIONS_HPP_

#include <vector>
#include <cassert>
#include <algorithm>

#include "Plato_Vector.hpp"
#include "Plato_ReductionOperations.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class StandardVectorReductionOperations : public Plato::ReductionOperations<ScalarType, OrdinalType>
{
public:
    StandardVectorReductionOperations()
    {
    }
    virtual ~StandardVectorReductionOperations()
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

        ScalarType aMaxValue = *std::max_element(tCopy.begin(), tCopy.end());
        return (aMaxValue);
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

        ScalarType aMinValue = *std::min_element(tCopy.begin(), tCopy.end());
        return (aMinValue);
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
        ScalarType tSum = std::accumulate(tCopy.begin(), tCopy.end(), tBaseValue);
        return (tSum);
    }
    //! Creates an instance of type Plato::ReductionOperations
    std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> create() const
    {
        std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> tCopy =
                std::make_shared<StandardVectorReductionOperations<ScalarType, OrdinalType>>();
        return (tCopy);
    }

private:
    StandardVectorReductionOperations(const Plato::StandardVectorReductionOperations<ScalarType, OrdinalType> &);
    Plato::StandardVectorReductionOperations<ScalarType, OrdinalType> & operator=(const Plato::StandardVectorReductionOperations<ScalarType, OrdinalType> &);
};

}

#endif /* PLATO_STANDARDVECTORREDUCTIONOPERATIONS_HPP_ */
