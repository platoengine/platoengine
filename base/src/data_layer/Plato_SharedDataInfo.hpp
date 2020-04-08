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
 * Plato_SharedDataInfo.hpp
 *
 *  Created on: Oct 3, 2017
 */

#ifndef PLATO_SHAREDDATAINFO_HPP_
#define PLATO_SHAREDDATAINFO_HPP_

#include <map>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>

namespace Plato
{

struct communication
{
    enum broadcast_t
    {
        SENDER = 1, RECEIVER = 2, SENDER_AND_RECEIVER = 3, UNDEFINED = 4,
    };
};

class SharedDataInfo
{
public:
    SharedDataInfo();
    ~SharedDataInfo();

    int getNumSharedData() const;
    int getSharedDataSize(const std::string & aName) const;
    void setSharedDataSize(const std::string & aName, const int & aSize);

    bool getSharedDataDynamic(const std::string & aName) const;
    void setSharedDataDynamic(const std::string & aName, const bool & aDynamic);

    const std::vector<std::string> & getProviderNames(const int & aIndex) const;
    const std::vector<std::string> & getReceiverNames(const int & aIndex) const;
    void setSharedDataMap(
           const std::vector<std::string> & aProviderNames, 
           const std::vector<std::string> & aReceiverNames);

    bool isNameDefined(const std::string & aName) const;
    bool isLayoutDefined(const std::string & aLayout) const;
    const std::string & getSharedDataName(const int & aIndex) const;
    const std::string & getSharedDataLayout(const int & aIndex) const;
    void setSharedDataIdentifiers(const std::string & aName, const std::string & aLayout);

    Plato::communication::broadcast_t getMyBroadcast(const int & aIndex) const;
    void setMyBroadcast(const Plato::communication::broadcast_t & aInput);

private:
    std::map<std::string, int> mSharedDataSize;
    std::map<std::string, bool> mSharedDataDynamic;
    std::vector<Plato::communication::broadcast_t> mBroadcast;
    std::vector<std::pair<std::string, std::string>> mSharedDataIdentifiers;
    std::vector<std::pair<std::vector<std::string>, std::vector<std::string>>> mSharedDataMap;

private:
    SharedDataInfo(const SharedDataInfo& aRhs);
    SharedDataInfo& operator=(const SharedDataInfo& aRhs);
};

inline Plato::communication::broadcast_t getBroadcastType(const std::string & aLocalCommName,
                                                          const std::vector<std::string> & aProviderNames,
                                                          const std::vector<std::string> & aReceiverNames)
{
    const bool tProvider = (std::count(aProviderNames.begin(), aProviderNames.end(), aLocalCommName) > 0);
    const bool tReceiver = (std::count(aReceiverNames.begin(), aReceiverNames.end(), aLocalCommName) > 0);
    Plato::communication::broadcast_t tMyBroadcast = Plato::communication::broadcast_t::UNDEFINED;

    if(tProvider && tReceiver)
    {
        tMyBroadcast = Plato::communication::broadcast_t::SENDER_AND_RECEIVER;
    }
    else if(tProvider)
    {
        tMyBroadcast = Plato::communication::broadcast_t::SENDER;
    }
    else if(tReceiver)
    {
        tMyBroadcast = Plato::communication::broadcast_t::RECEIVER;
    }

    return (tMyBroadcast);
}

}

#endif /* PLATO_SHAREDDATAINFO_HPP_ */
