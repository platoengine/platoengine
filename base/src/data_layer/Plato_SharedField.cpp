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
 * Plato_SharedField.cpp
 *
 *  Created on: April 23, 2017
 *
 */

#include "Plato_SharedField.hpp"

namespace Plato
{

/******************************************************************************/
int SharedField::size() const
/******************************************************************************/
{
    assert(mMyBroadcast !=  Plato::communication::broadcast_t::UNDEFINED);

    int tMyLength = 0;
    switch(mMyBroadcast)
    {
        case  Plato::communication::broadcast_t::SENDER:
        {
            assert(mSendDataVector.get() != nullptr);
            tMyLength = mSendDataVector->MyLength();
            break;
        }
        case Plato::communication::broadcast_t::RECEIVER:
        {
            assert(mRecvDataVector.get() != nullptr);
            tMyLength = mRecvDataVector->MyLength();
            break;
        }
        case Plato::communication::broadcast_t::SENDER_AND_RECEIVER:
        {
            assert(mSendDataVector.get() != nullptr);
            assert(mRecvDataVector.get() != nullptr);
            tMyLength = mRecvDataVector->MyLength();
            break;
        }
        default:
        case  Plato::communication::broadcast_t::UNDEFINED:
        {
            // TODO: THROW
            break;
        }
    }
    return (tMyLength);
}

/******************************************************************************/
std::string SharedField::myName() const
/******************************************************************************/
{
    return mMyName;
}

/******************************************************************************/
Plato::data::layout_t SharedField::myLayout() const
/******************************************************************************/
{
    return mMyLayout;
}

/******************************************************************************/
Plato::communication::broadcast_t SharedField::myBroadcast() const
/******************************************************************************/
{
    return mMyBroadcast;
}

/******************************************************************************/
void SharedField::setData(const std::vector<double> & aData)
/******************************************************************************/
{
    assert(aData.empty() == false);
    assert(mSendDataVector.get() != nullptr);
    assert(mMyBroadcast != Plato::communication::broadcast_t::UNDEFINED);

    int tMyLength = mSendDataVector->MyLength();
    double* tMyDataView = nullptr;
    mSendDataVector->ExtractView(&tMyDataView);

    for(int tIndex = 0; tIndex < tMyLength; tIndex++)
    {
        tMyDataView[tIndex] = aData[tIndex];
    }
}

/******************************************************************************/
void SharedField::getData(std::vector<double>& aData) const
/******************************************************************************/
{
    assert(aData.empty() == false);
    assert(mRecvDataVector.get() != nullptr);
    assert(mMyBroadcast != Plato::communication::broadcast_t::UNDEFINED);

    int tMyLength = mRecvDataVector->MyLength();
    double* tMyDataView = nullptr;
    mRecvDataVector->ExtractView(&tMyDataView);

    for(int tIndex = 0; tIndex < tMyLength; tIndex++)
    {
        aData[tIndex] = tMyDataView[tIndex];
    }
}

/******************************************************************************/
void SharedField::setData(const double & aDataVal, const int & aGlobalIndex)
/******************************************************************************/
{
    assert(mSendDataVector.get() != nullptr);
    assert(mGlobalIDsProvided.get() != nullptr);
    assert(mMyBroadcast != Plato::communication::broadcast_t::UNDEFINED);

    int tLocalID = mGlobalIDsProvided->LID(aGlobalIndex);
    if(tLocalID >= 0)
    {
        (*mSendDataVector)[tLocalID] = aDataVal;
    }
}

/******************************************************************************/
void SharedField::getData(double & aDataVal, const int & aGlobalIndex) const
/******************************************************************************/
{
    assert(mRecvDataVector.get() != nullptr);
    assert(mGlobalIDsReceived.get() != nullptr);
    assert(mMyBroadcast != Plato::communication::broadcast_t::UNDEFINED);

    int tLocalID = mGlobalIDsReceived->LID(aGlobalIndex);
    if(tLocalID >= 0)
    {
        aDataVal = (*mRecvDataVector)[tLocalID];
    }
}

/******************************************************************************/
void SharedField::transmitData()
/******************************************************************************/
{
    assert(mNodeImporter.get() != nullptr);
    assert(mRecvDataVector.get() != nullptr);
    assert(mSendDataVector.get() != nullptr);

    mRecvDataVector->PutScalar(0.0);
    mRecvDataVector->Import(*mSendDataVector, *mNodeImporter, Insert);
}

/******************************************************************************/
void SharedField::initialize(const Plato::CommunicationData & aCommData)
/******************************************************************************/
{
    std::vector<int> tMySendGlobalIDs;
    std::vector<int> tMyRecvGlobalIDs;

    const auto& ownedGIDs = aCommData.mMyOwnedGlobalIDs.at(mMyLayout);
    auto begin = ownedGIDs.begin();
    auto end   = ownedGIDs.end();
    auto size  = ownedGIDs.size();

    switch(mMyBroadcast)
    {
        case Plato::communication::broadcast_t::SENDER_AND_RECEIVER:
        {
            tMySendGlobalIDs.resize(size);
            tMyRecvGlobalIDs.resize(size);
            std::copy(begin, end, tMySendGlobalIDs.begin());
            std::copy(begin, end, tMyRecvGlobalIDs.begin());
            break;
        }
        case Plato::communication::broadcast_t::SENDER:
        {
            tMySendGlobalIDs.resize(size);
            std::copy(begin, end, tMySendGlobalIDs.begin());
            break;
        }
        case Plato::communication::broadcast_t::RECEIVER:
        {
            tMyRecvGlobalIDs.resize(size);
            std::copy(begin, end, tMyRecvGlobalIDs.begin());
            break;
        }
        default:
        case Plato::communication::broadcast_t::UNDEFINED:
        {
            // TODO: THROW
            break;
        }
    }

    mGlobalIDsProvided = std::make_shared<Epetra_Map>(-1, tMySendGlobalIDs.size(), tMySendGlobalIDs.data(), 0, *mEpetraComm);
    mGlobalIDsReceived = std::make_shared<Epetra_Map>(-1, tMyRecvGlobalIDs.size(), tMyRecvGlobalIDs.data(), 0, *mEpetraComm);

    mNodeImporter = std::make_shared<Epetra_Import>(*mGlobalIDsReceived, *mGlobalIDsProvided);

    mSendDataVector = std::make_shared<Epetra_Vector>(*mGlobalIDsProvided);
    mSendDataVector->PutScalar(0.0);
    mRecvDataVector = std::make_shared<Epetra_Vector>(*mGlobalIDsReceived);
    mRecvDataVector->PutScalar(0.0);
}

/*****************************************************************************/
SharedField::SharedField(const std::string & aMyName,
                         const Plato::communication::broadcast_t & aMyBroadcast,
                         const Plato::CommunicationData & aCommData,
                         Plato::data::layout_t aMyLayout) :
        SharedData(),
        mMyName(aMyName),
        mMyLayout(aMyLayout),
        mMyBroadcast(aMyBroadcast),
        mEpetraComm(std::make_shared<Epetra_MpiComm>(aCommData.mInterComm)),
        mGlobalIDsProvided(nullptr),
        mGlobalIDsReceived(nullptr),
        mNodeImporter(nullptr),
        mSendDataVector(nullptr),
        mRecvDataVector(nullptr)
/*****************************************************************************/
{
#ifdef DEBUG_LOCATION
    _print_entering_location(__AXSIS_FUNCTION_NAMER__);
#endif //DEBUG_LOCATION

    this->initialize(aCommData);
}

/*****************************************************************************/
SharedField::~SharedField()
/*****************************************************************************/
{
}

} // End namespace Plato
