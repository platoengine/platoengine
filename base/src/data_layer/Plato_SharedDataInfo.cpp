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
const std::vector<std::string> & SharedDataInfo::getProviderNames(const int & aIndex) const
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
void SharedDataInfo::setSharedDataMap(
  const std::vector<std::string> & aProviderNames, 
  const std::vector<std::string> & aReceiverNames)
/******************************************************************************/
{
    mSharedDataMap.push_back(std::make_pair(aProviderNames, aReceiverNames));
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
bool SharedDataInfo::getSharedDataDynamic(const std::string & aName) const
/******************************************************************************/
{
    assert(mSharedDataDynamic.empty() == false);
    std::map<std::string, bool>::const_iterator tIterator;
    tIterator = mSharedDataDynamic.find(aName);
    assert(tIterator != mSharedDataDynamic.end());
    return (tIterator->second);
}

/******************************************************************************/
void SharedDataInfo::setSharedDataDynamic(const std::string & aName, const bool & aIsDynamic)
/******************************************************************************/
{
    mSharedDataDynamic[aName] = aIsDynamic;
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
