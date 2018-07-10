/*
 * Plato_MultiVectorList.hpp
 *
 *  Created on: Oct 21, 2017
 *      Author: Miguel A. Aguilo Valentin
 */

#ifndef PLATO_MULTIVECTORLIST_HPP_
#define PLATO_MULTIVECTORLIST_HPP_

#include <memory>
#include <vector>
#include <cassert>

#include "Plato_Vector.hpp"
#include "Plato_MultiVector.hpp"

namespace Plato
{

template<typename ScalarType, typename OrdinalType = size_t>
class MultiVectorList
{
public:
    MultiVectorList() :
            mList()
    {
    }
    ~MultiVectorList()
    {
    }

    OrdinalType size() const
    {
        return (mList.size());
    }
    void add(const Plato::MultiVector<ScalarType, OrdinalType> & aInput)
    {
        mList.push_back(aInput.create());
    }
    void add(const std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> & aInput)
    {
        mList.push_back(aInput);
    }
    Plato::MultiVector<ScalarType, OrdinalType> & operator [](const OrdinalType & aInput)
    {
        assert(aInput < mList.size());
        assert(mList[aInput].get() != nullptr);
        return (mList[aInput].operator*());
    }
    const Plato::MultiVector<ScalarType, OrdinalType> & operator [](const OrdinalType & aInput) const
    {
        assert(aInput < mList.size());
        assert(mList[aInput].get() != nullptr);
        return (mList[aInput].operator*());
    }
    Plato::Vector<ScalarType, OrdinalType> & operator ()(const OrdinalType & aListIndex,
                                                         const OrdinalType & aVectorIndex)
    {
        assert(aListIndex < mList.size());
        assert(mList[aListIndex].get() != nullptr);
        return (mList[aListIndex]->operator[](aVectorIndex));
    }
    const Plato::Vector<ScalarType, OrdinalType> & operator ()(const OrdinalType & aListIndex,
                                                               const OrdinalType & aVectorIndex) const
    {
        assert(aListIndex < mList.size());
        assert(mList[aListIndex].get() != nullptr);
        return (mList[aListIndex]->operator[](aVectorIndex));
    }
    std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> create() const
    {
        assert(this->size() > static_cast<OrdinalType>(0));
        std::shared_ptr<Plato::MultiVectorList<ScalarType, OrdinalType>> tOutput = std::make_shared<
                Plato::MultiVectorList<ScalarType, OrdinalType>>();
        const OrdinalType tListSize = this->size();
        for(OrdinalType tIndex = 0; tIndex < tListSize; tIndex++)
        {
            assert(mList[tIndex].get() != nullptr);
            const std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> & tMultiVector = mList[tIndex];
            tOutput->add(tMultiVector);
        }
        return (tOutput);
    }
    const std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>> & ptr(const OrdinalType & aInput) const
    {
        assert(aInput < mList.size());
        assert(mList[aInput].get() != nullptr);
        return (mList[aInput]);
    }

private:
    std::vector<std::shared_ptr<Plato::MultiVector<ScalarType, OrdinalType>>>mList;

private:
    MultiVectorList(const Plato::MultiVectorList<ScalarType, OrdinalType>&);
    Plato::MultiVectorList<ScalarType, OrdinalType> & operator=(const Plato::MultiVectorList<ScalarType, OrdinalType>&);
};

}
 // namespace Plato

#endif /* PLATO_MULTIVECTORLIST_HPP_ */
