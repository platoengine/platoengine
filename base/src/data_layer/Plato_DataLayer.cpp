/*
 * Plato_Interface.cpp
 *
 *  Created on: March 20, 2017
 *
 */

// TODO:
// 4.  Implement shared pointers
// 5.  Performer stdout.  Redirect to file? to stringstream then communicate?
// 6.  Ndof SharedField
#include <cstdlib>
#include <stdlib.h>

#include "Plato_Interface.hpp"
#include "Plato_SharedField.hpp"
#include "Plato_SharedValue.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_Communication.hpp"
#include "Plato_SharedDataInfo.hpp"

namespace Plato
{

/******************************************************************************/
DataLayer::DataLayer(const Plato::SharedDataInfo & aSharedDataInfo, const Plato::CommunicationData & aCommData) :
        mSharedData(),
        mSharedDataMap()
/******************************************************************************/
{
    // create the shared fields
    //
    const int tNumSharedData = aSharedDataInfo.getNumSharedData();
    for(int tIndex = 0; tIndex < tNumSharedData; tIndex++)
    {
        std::string tMyName = aSharedDataInfo.getSharedDataName(tIndex);
        std::string tMyLayout = aSharedDataInfo.getSharedDataLayout(tIndex);

        SharedData* tNewData = nullptr;
        if(tMyLayout == "NODAL FIELD" )
        {
            const Plato::communication::broadcast_t tBroadcastType = aSharedDataInfo.getMyBroadcast(tIndex);
            tNewData = new Plato::SharedField(tMyName, tBroadcastType, aCommData,  Plato::data::layout_t::SCALAR_FIELD);
        }
        else 
        if(tMyLayout == "ELEMENT FIELD")
        {
            const Plato::communication::broadcast_t tBroadcastType = aSharedDataInfo.getMyBroadcast(tIndex);
            tNewData = new Plato::SharedField(tMyName, tBroadcastType, aCommData, Plato::data::layout_t::ELEMENT_FIELD);
        }
        else 
        if(tMyLayout == "GLOBAL")
        {
            const int tSize = aSharedDataInfo.getSharedDataSize(tMyName);
            assert(tSize > static_cast<int>(0));
            std::string tMyProviderName = aSharedDataInfo.getProviderName(tIndex);
            tNewData = new Plato::SharedValue(tMyName, tMyProviderName, aCommData, tSize);
        }
        else
        {
            // TODO: TRHOW
            std::stringstream ss;
            ss << "Plato::DataLayer: Given unknown SharedData Layout ('" << tMyLayout << "').";
            throw ParsingException(ss.str());
        }

        mSharedData.push_back(tNewData);
        mSharedDataMap[tMyName] = tNewData;
    }
}

/******************************************************************************/
DataLayer::~DataLayer()
/******************************************************************************/
{
    const size_t tNumSharedData = mSharedData.size();
    for(size_t tSharedDataIndex = 0u; tSharedDataIndex < tNumSharedData; tSharedDataIndex++)
    {
        delete mSharedData[tSharedDataIndex];
    }
    mSharedData.clear();
    mSharedDataMap.clear();
}

/******************************************************************************/
SharedData* DataLayer::getSharedData(const std::string & aName) const
/******************************************************************************/
{
    auto tIterator = mSharedDataMap.find(aName);
    if(tIterator != mSharedDataMap.end())
    {
        return tIterator->second;
    }
    else
    {
        std::stringstream ss;
        ss << "Plato::DataLayer: request for SharedData ('" << aName << "') that doesn't exist.";
        throw ParsingException(ss.str());
    }
}

/******************************************************************************/
const std::vector<SharedData*> & DataLayer::getSharedData() const
/******************************************************************************/
{
    return mSharedData;
}

} /* namespace Plato */

