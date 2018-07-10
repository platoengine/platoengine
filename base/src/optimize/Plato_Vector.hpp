/*
 * Plato_Vector.hpp
 *
 *  Created on: Oct 6, 2017
 */

#ifndef PLATO_VECTOR_HPP_
#define PLATO_VECTOR_HPP_

#include <memory>

namespace Plato
{

struct MemorySpace
{
    enum type_t
    {
        HOST = 1, DEVICE = 2
    };
};

template<typename ScalarType, typename OrdinalType = size_t>
class Vector
{
public:
    virtual ~Vector()
    {
    }

    //! Scales a Vector by a ScalarType constant.
    virtual void scale(const ScalarType & aInput) = 0;
    //! Entry-Wise product of two vectors.
    virtual void entryWiseProduct(const Plato::Vector<ScalarType, OrdinalType> & aInput) = 0;
    //! Update vector values with scaled values of A, this = beta*this + alpha*A.
    virtual void update(const ScalarType & aAlpha,
                        const Plato::Vector<ScalarType, OrdinalType> & aInputVector,
                        const ScalarType & aBeta) = 0;
    //! Computes the absolute value of each element in the container.
    virtual void modulus() = 0;
    //! Returns the inner product of two vectors.
    virtual ScalarType dot(const Plato::Vector<ScalarType, OrdinalType> & aInputVector) const = 0;
    //! Assigns new contents to the Vector, replacing its current contents, and not modifying its size.
    virtual void fill(const ScalarType & aValue) = 0;
    //! Returns the number of local elements in the Vector.
    virtual OrdinalType size() const = 0;
    //! Creates an object of type Plato::Vector
    virtual std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> create() const = 0;
    //! Operator overloads the square bracket operator.
    virtual ScalarType & operator [](const OrdinalType & aIndex) = 0;
    //! Operator overloads the square bracket operator.
    virtual const ScalarType & operator [](const OrdinalType & aIndex) const = 0;
    //! Returns a direct pointer to the memory array used internally by the vector to store its owned elements.
    virtual ScalarType* data() = 0;
    //! Returns a direct const pointer to the memory array used internally by the vector to store its owned elements.
    virtual const ScalarType* data() const = 0;
};

} // namespace Plato

#endif /* PLATO_VECTOR_HPP_ */
