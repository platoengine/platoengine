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

    const std::string & getProviderName(const int & aIndex) const;
    const std::vector<std::string> & getReceiverNames(const int & aIndex) const;
    void setSharedDataMap(const std::string & aProviderName, const std::vector<std::string> & aReceiverNames);

    bool isNameDefined(const std::string & aName) const;
    bool isLayoutDefined(const std::string & aLayout) const;
    const std::string & getSharedDataName(const int & aIndex) const;
    const std::string & getSharedDataLayout(const int & aIndex) const;
    void setSharedDataIdentifiers(const std::string & aName, const std::string & aLayout);

    Plato::communication::broadcast_t getMyBroadcast(const int & aIndex) const;
    void setMyBroadcast(const Plato::communication::broadcast_t & aInput);

private:
    std::map<std::string, int> mSharedDataSize;
    std::vector<Plato::communication::broadcast_t> mBroadcast;
    std::vector<std::pair<std::string, std::string>> mSharedDataIdentifiers;
    std::vector<std::pair<std::string, std::vector<std::string>>> mSharedDataMap;

private:
    SharedDataInfo(const SharedDataInfo& aRhs);
    SharedDataInfo& operator=(const SharedDataInfo& aRhs);
};

inline Plato::communication::broadcast_t getBroadcastType(const std::string & aLocalCommName,
                                                          const std::string & aProviderName,
                                                          const std::vector<std::string> & aReceiverNames)
{
    const bool tProvider = (aLocalCommName == aProviderName);
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
