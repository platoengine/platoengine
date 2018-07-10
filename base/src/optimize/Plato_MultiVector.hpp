/*
 * Plato_MultiVector.hpp
 *
 *  Created on: Oct 6, 2017
 */

#ifndef PLATO_MULTIVECTOR_HPP_
#define PLATO_MULTIVECTOR_HPP_

#include <memory>

namespace Plato
{

template<typename ScalarType, typename OrdinalType>
class Vector;

template<typename ScalarType, typename OrdinalType = size_t>
class MultiVector
{
public:
    virtual ~MultiVector()
    {
    }

    //! Returns number of vectors
    virtual OrdinalType getNumVectors() const = 0;
    //! Creates a copy of type MultiVector
    virtual std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> create() const = 0;
    //! Operator overloads the square bracket operator
    virtual Plato::Vector<ScalarType, OrdinalType> & operator [](const OrdinalType & aVectorIndex) = 0;
    //! Operator overloads the square bracket operator
    virtual const Plato::Vector<ScalarType, OrdinalType> & operator [](const OrdinalType & aVectorIndex) const = 0;
    //! Operator overloads the square bracket operator
    virtual ScalarType & operator ()(const OrdinalType & aVectorIndex, const OrdinalType & aElementIndex) = 0;
    //! Operator overloads the square bracket operator
    virtual const ScalarType & operator ()(const OrdinalType & aVectorIndex, const OrdinalType & aElementIndex) const = 0;
};

}

#endif /* PLATO_MULTIVECTOR_HPP_ */
