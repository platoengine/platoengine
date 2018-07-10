/*
 * Plato_SharedValue.cpp
 *
 *  Created on: April 23, 2017
 *
 */

#include "Plato_Parser.hpp"
#include "Plato_SharedData.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_SharedValue.hpp"
#include "Plato_Communication.hpp"

namespace Plato
{

/******************************************************************************/
void SharedValue::transmitData()
/******************************************************************************/
{
    int tMyProcID = -1;
    MPI_Comm_rank(mMyComm, &tMyProcID);

    int tGlobalProcID = -1;
    bool tIsProvider = mLocalCommName == mProviderName;
    if(tMyProcID == 0 && tIsProvider)
    {
        MPI_Comm_rank(mInterComm, &tGlobalProcID);
    }

    int tSenderProcID = -1;
    MPI_Allreduce(&tGlobalProcID, &tSenderProcID, 1, MPI_INT, MPI_MAX, mInterComm);
    MPI_Bcast(mData.data(), mData.size(), MPI_DOUBLE, tSenderProcID, mInterComm);
}

/******************************************************************************/
void SharedValue::setData(const std::vector<double> & aData)
/******************************************************************************/
{
    for(int tIndex = 0; tIndex < mNumData; tIndex++)
    {
        mData[tIndex] = aData[tIndex];
    }
}

/******************************************************************************/
void SharedValue::getData(std::vector<double> & aData) const
/******************************************************************************/
{
    for(int tIndex = 0; tIndex < mNumData; tIndex++)
    {
        aData[tIndex] = mData[tIndex];
    }
}

/*****************************************************************************/
int SharedValue::size() const
/*****************************************************************************/
{
    return mNumData;
}

/******************************************************************************/
std::string SharedValue::myName() const
{
    return mMyName;
}
/******************************************************************************/

/******************************************************************************/
Plato::data::layout_t SharedValue::myLayout() const
{
    return mMyLayout;
}

/*****************************************************************************/
SharedValue::SharedValue(const std::string & aMyName,
                         const std::string & aProviderName,
                         const Plato::CommunicationData & aCommData,
                         int aSize) :
        mMyName(aMyName),
        mProviderName(aProviderName),
        mLocalCommName(aCommData.mLocalCommName),
        mMyComm(aCommData.mLocalComm),
        mInterComm(aCommData.mInterComm),
        mNumData(aSize),
        mData(std::vector<double>(aSize)),
        mMyLayout(Plato::data::layout_t::SCALAR)
/*****************************************************************************/
{
}

/*****************************************************************************/
SharedValue::~SharedValue()
/*****************************************************************************/
{
}

} // End namespace Plato
