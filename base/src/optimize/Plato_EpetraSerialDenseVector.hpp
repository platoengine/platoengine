/*
 * Plato_EpetraSerialDenseVector.hpp
 *
 *  Created on: Nov 27, 2017
 */

#ifndef PLATO_EPETRASERIALDENSEVECTOR_HPP_
#define PLATO_EPETRASERIALDENSEVECTOR_HPP_

#include <memory>

#include "Plato_Vector.hpp"
#include "Epetra_SerialDenseVector.h"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class EpetraSerialDenseVector : public Plato::Vector<ScalarType, OrdinalType>
{
public:
    explicit EpetraSerialDenseVector(const std::vector<ScalarType> & aInput) :
            mData(Epetra_SerialDenseVector(aInput.size()))
    {
        const OrdinalType tLength = aInput.size();
        for(OrdinalType tIndex = 0; tIndex < tLength; tIndex++)
        {
            mData[tIndex] = aInput[tIndex];
        }
    }
    explicit EpetraSerialDenseVector(const OrdinalType & aLength, ScalarType aValue = 0) :
            mData(Epetra_SerialDenseVector(aLength))
    {
        this->fill(aValue);
    }
    virtual ~EpetraSerialDenseVector()
    {
    }

    //! Scales a Vector by a ScalarType constant.
    void scale(const ScalarType & aInput)
    {
        const OrdinalType tLength = mData.Length();
        mData.SCAL(static_cast<int>(tLength), aInput, mData.A());
    }
    //! Entry-Wise product of two vectors.
    void entryWiseProduct(const Plato::Vector<ScalarType, OrdinalType> & aInput)
    {
        const OrdinalType tMyLength = mData.Length();
        assert(aInput.size() == tMyLength);

        for(OrdinalType tIndex = 0; tIndex < tMyLength; tIndex++)
        {
            mData[tIndex] = aInput[tIndex] * mData[tIndex];
        }
    }
    //! Update vector values with scaled values of A, this = beta*this + alpha*A.
    void update(const ScalarType & aAlpha,
                        const Plato::Vector<ScalarType, OrdinalType> & aInputVector,
                        const ScalarType & aBeta)
    {
        const OrdinalType tMyLength = mData.Length();
        assert(aInputVector.size() == tMyLength);
        for(OrdinalType tIndex = 0; tIndex < tMyLength; tIndex++)
        {
            mData[tIndex] = aBeta * mData[tIndex] + aAlpha * aInputVector[tIndex];
        }
    }
    //! Computes the absolute value of each element in the container.
    void modulus()
    {
        const OrdinalType tMyLength = this->size();
        for(OrdinalType tIndex = 0; tIndex < tMyLength; tIndex++)
        {
            mData[tIndex] = std::abs(mData[tIndex]);
        }
    }
    //! Returns the inner product of two vectors.
    ScalarType dot(const Plato::Vector<ScalarType, OrdinalType> & aInput) const
    {
        const OrdinalType tMyLength = mData.Length();
        assert(aInput.size() == tMyLength);

        const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType> & tInputVector =
                dynamic_cast<const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType>&>(aInput);

        ScalarType tOutput = mData.DOT(tMyLength, mData.A(), tInputVector.mData.A());
        return (tOutput);
    }
    //! Assigns new contents to the Vector, replacing its current contents, and not modifying its size.
    void fill(const ScalarType & aValue)
    {
        const OrdinalType tMyLength = this->size();
        for(OrdinalType tIndex = 0; tIndex < tMyLength; tIndex++)
        {
            mData[tIndex] = aValue;
        }
    }
    //! Returns the number of local elements in the Vector.
    OrdinalType size() const
    {
        const OrdinalType tMyLength = mData.Length();
        return (tMyLength);
    }
    //! Creates an object of type Plato::Vector
    std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> create() const
    {
        const ScalarType tBaseValue = 0;
        const OrdinalType tMyLength = this->size();
        std::shared_ptr<Plato::Vector<ScalarType, OrdinalType>> tOutput =
                std::make_shared<Plato::EpetraSerialDenseVector<ScalarType, OrdinalType>>(tMyLength, tBaseValue);
        return (tOutput);
    }
    //! Operator overloads the square bracket operator
    ScalarType & operator [](const OrdinalType & aIndex)
    {
        assert(aIndex < this->size());
        assert(aIndex >= static_cast<OrdinalType>(0));
        return (mData[aIndex]);
    }
    //! Operator overloads the square bracket operator
    const ScalarType & operator [](const OrdinalType & aIndex) const
    {
        assert(aIndex < this->size());
        assert(aIndex >= static_cast<OrdinalType>(0));
        return (mData[aIndex]);
    }
    //! Returns a direct pointer to the memory array used internally by the vector to store its owned elements.
    ScalarType* data()
    {
        return (mData.A());
    }
    //! Returns a direct const pointer to the memory array used internally by the vector to store its owned elements.
    const ScalarType* data() const
    {
        return (mData.A());
    }
    //! Returns reference to Epetra_SerialDenseVector
    Epetra_SerialDenseVector & vector()
    {
        return (mData);
    }
    //! Returns const reference to Epetra_SerialDenseVector
    const Epetra_SerialDenseVector & vector() const
    {
        return (mData);
    }

private:
    Epetra_SerialDenseVector mData;

private:
    EpetraSerialDenseVector(const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType>&);
    Plato::EpetraSerialDenseVector<ScalarType, OrdinalType> & operator=(const Plato::EpetraSerialDenseVector<ScalarType, OrdinalType>&);
};

} // namespace Plato

#endif /* PLATO_EPETRASERIALDENSEVECTOR_HPP_ */
