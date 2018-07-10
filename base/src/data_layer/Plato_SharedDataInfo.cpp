/*
 * Plato_SharedDataInfo.cpp
 *
 *  Created on: Oct 3, 2017
 */

#include <cassert>

#include "Plato_SharedDataInfo.hpp"

namespace Plato
{

/******************************************************************************/
SharedDataInfo::SharedDataInfo() :
        mSharedDataSize(),
        mBroadcast(),
        mSharedDataIdentifiers(),
        mSharedDataMap()
/******************************************************************************/
{
}

/******************************************************************************/
SharedDataInfo::~SharedDataInfo()
/******************************************************************************/
{
}

/******************************************************************************/
const std::string & SharedDataInfo::getProviderName(const int & aIndex) const
/******************************************************************************/
{
    assert(mSharedDataMap.empty() == false);
    assert(aIndex >= static_cast<int>(0));
    assert(aIndex < static_cast<int>(mSharedDataMap.size()));
    return (mSharedDataMap[aIndex].first);
}

/******************************************************************************/
const std::vector<std::string> & SharedDataInfo::getReceiverNames(const int & aIndex) const
/******************************************************************************/
{
    assert(mSharedDataMap.empty() == false);
    assert(aIndex >= static_cast<int>(0));
    assert(aIndex < static_cast<int>(mSharedDataMap.size()));
    return (mSharedDataMap[aIndex].second);
}

/******************************************************************************/
void SharedDataInfo::setSharedDataMap(const std::string & aProviderName, const std::vector<std::string> & aReceiverNames)
/******************************************************************************/
{
    mSharedDataMap.push_back(std::make_pair(aProviderName, aReceiverNames));
}

/******************************************************************************/
int SharedDataInfo::getSharedDataSize(const std::string & aName) const
/******************************************************************************/
{
    assert(mSharedDataSize.empty() == false);
    std::map<std::string, int>::const_iterator tIterator;
    tIterator = mSharedDataSize.find(aName);
    assert(tIterator != mSharedDataSize.end());
    return (tIterator->second);
}

/******************************************************************************/
void SharedDataInfo::setSharedDataSize(const std::string & aName, const int & aSize)
/******************************************************************************/
{
    mSharedDataSize[aName] = aSize;
}

/******************************************************************************/
bool SharedDataInfo::isNameDefined(const std::string & aName) const
/******************************************************************************/
{
    auto tIterator = std::find_if(mSharedDataIdentifiers.begin(),
                                  mSharedDataIdentifiers.end(),
                                  [&aName](const std::pair<std::string, std::string> & aElement)
                                  {return (aElement.first == aName);});
    bool tFoundLayout = tIterator != mSharedDataIdentifiers.end() ? true : false;
    return (tFoundLayout);
}

/******************************************************************************/
bool SharedDataInfo::isLayoutDefined(const std::string & aLayout) const
/******************************************************************************/
{
    auto tIterator = std::find_if(mSharedDataIdentifiers.begin(),
                                  mSharedDataIdentifiers.end(),
                                  [&aLayout](const std::pair<std::string, std::string> & aElement)
                                  {return (aElement.second == aLayout);});
    bool tFoundLayout = tIterator != mSharedDataIdentifiers.end() ? true : false;
    return (tFoundLayout);
}

/******************************************************************************/
const std::string & SharedDataInfo::getSharedDataName(const int & aIndex) const
/******************************************************************************/
{
    assert(mSharedDataIdentifiers.empty() == false);
    assert(aIndex >= static_cast<int>(0));
    assert(aIndex < static_cast<int>(mSharedDataMap.size()));
    return (mSharedDataIdentifiers[aIndex].first);
}

/******************************************************************************/
const std::string & SharedDataInfo::getSharedDataLayout(const int & aIndex) const
/******************************************************************************/
{
    assert(mSharedDataIdentifiers.empty() == false);
    assert(aIndex >= static_cast<int>(0));
    assert(aIndex < static_cast<int>(mSharedDataMap.size()));
    return (mSharedDataIdentifiers[aIndex].second);
}

/******************************************************************************/
void SharedDataInfo::setSharedDataIdentifiers(const std::string & aName, const std::string & aLayout)
/******************************************************************************/
{
    mSharedDataIdentifiers.push_back(std::make_pair(aName, aLayout));
}

/******************************************************************************/
Plato::communication::broadcast_t SharedDataInfo::getMyBroadcast(const int & aIndex) const
/******************************************************************************/
{
    assert(mBroadcast.empty() == false);
    assert(aIndex >= static_cast<int>(0));
    assert(aIndex < static_cast<int>(mSharedDataMap.size()));
    return (mBroadcast[aIndex]);
}

/******************************************************************************/
void SharedDataInfo::setMyBroadcast(const Plato::communication::broadcast_t & aInput)
/******************************************************************************/
{
    mBroadcast.push_back(aInput);
}

/******************************************************************************/
int SharedDataInfo::getNumSharedData() const
/******************************************************************************/
{
    assert(mBroadcast.empty() == false);
    assert(mSharedDataMap.empty() == false);
    assert(mSharedDataIdentifiers.empty() == false);
    assert(mBroadcast.size() == mSharedDataIdentifiers.size());
    assert(mSharedDataIdentifiers.size() == mSharedDataMap.size());
    return (mBroadcast.size());
}

}
