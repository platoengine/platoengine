/*
 * Plato_ReductionOperations.hpp
 *
 *  Created on: Oct 6, 2017
 */

#ifndef PLATO_REDUCTIONOPERATIONS_HPP_
#define PLATO_REDUCTIONOPERATIONS_HPP_

#include <memory>

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class Vector;

template<typename ScalarType, typename OrdinalType = size_t>
class ReductionOperations
{
public:
    virtual ~ReductionOperations()
    {
    }

    //! Returns the maximum element in range
    virtual ScalarType max(const Plato::Vector<ScalarType, OrdinalType> & aInput) const = 0;
    //! Returns the minimum element in range
    virtual ScalarType min(const Plato::Vector<ScalarType, OrdinalType> & aInput) const = 0;
    //! Returns the sum of all the elements in container.
    virtual ScalarType sum(const Plato::Vector<ScalarType, OrdinalType> & aInput) const = 0;
    //! Creates object of type Plato::ReductionOperations
    virtual std::shared_ptr<Plato::ReductionOperations<ScalarType, OrdinalType>> create() const = 0;
};

}

#endif /* PLATO_REDUCTIONOPERATIONS_HPP_ */
