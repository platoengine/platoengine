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

    if( mProviderNames.size() == 1 )
    { // single provider
        int tGlobalProcID = -1;
        bool tIsProvider = mLocalCommName == mProviderNames[0];
        if(tMyProcID == 0 && tIsProvider)
        {
            MPI_Comm_rank(mInterComm, &tGlobalProcID);
        }
        // determine provider procID
        int tSenderProcID = -1;
        MPI_Allreduce(&tGlobalProcID, &tSenderProcID, 1, MPI_INT, MPI_MAX, mInterComm);

        if( mIsDynamic )
        {
            int tNumData = mData.size();
            MPI_Bcast(&tNumData, 1, MPI_INT, tSenderProcID, mInterComm);
            mData.resize(tNumData);
            mNumData = tNumData;
        }
        auto tRecv(mData);
        MPI_Bcast(tRecv.data(), tRecv.size(), MPI_DOUBLE, tSenderProcID, mInterComm);
        if( !tIsProvider ){
            mData = tRecv;
        }
    }
    else 
    { // multiple provider
    
        // reduce data from all rank zeros of all provider local comms.
        //

        // determine if local rank is a provider
        bool tIsaProvider = 
         ( std::find( mProviderNames.begin(), 
                      mProviderNames.end(), 
                      mLocalCommName ) != mProviderNames.end() );
        int tProviderColor = (tMyProcID == 0 && tIsaProvider) ? 1 : 0;
    
        // create temporary comm for reducing from multiple providers
        int tGlobalProcID = -1;
        MPI_Comm_rank(mInterComm, &tGlobalProcID);
        MPI_Comm tReductionComm;
        MPI_Comm_split(mInterComm, tProviderColor, tGlobalProcID, &tReductionComm);

        // reduce data to rank zero of tReductionComm
        std::vector<double> tRecv(mData.size(), 0.0);
        MPI_Reduce(mData.data(), tRecv.data(), mData.size(), MPI_DOUBLE, MPI_SUM, /*rank_of_root=*/0, tReductionComm);


        // broadcast the result to all ranks in mIntercomm
        //

        // get rank in mReductionComm
        int tRankInReductionComm;
        MPI_Comm_rank(tReductionComm, &tRankInReductionComm);

        // if the local rank is not zero (i.e., the sender), then flip the global rank to -1
        if( tRankInReductionComm ) tGlobalProcID = -1;

        // global max now returns the rank of the sender
        int tSenderProcID=-1;
        MPI_Allreduce(&tGlobalProcID, &tSenderProcID, 1, MPI_INT, MPI_MAX, mInterComm);

        // broadcast the reduced data to all ranks 
        MPI_Bcast(tRecv.data(), tRecv.size(), MPI_DOUBLE, tSenderProcID, mInterComm);
        if( !tIsaProvider ){
            mData = tRecv;
        }
    }
}
  

/******************************************************************************/
void SharedValue::setData(const std::vector<double> & aData)
/******************************************************************************/
{
    if( mIsDynamic )
    {
        mNumData = aData.size();
        mData.resize(mNumData);
    }
    for(int tIndex = 0; tIndex < mNumData; tIndex++)
    {
        mData[tIndex] = aData[tIndex];
    }
}

/******************************************************************************/
void SharedValue::getData(std::vector<double> & aData) const
/******************************************************************************/
{
    if( mIsDynamic )
    {
        aData.resize(mNumData);
    }
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
                         const std::vector<std::string> & aProviderNames,
                         const Plato::CommunicationData & aCommData,
                         int aSize, bool aIsDynamic) :
        mMyName(aMyName),
        mProviderNames(aProviderNames),
        mLocalCommName(aCommData.mLocalCommName),
        mMyComm(aCommData.mLocalComm),
        mInterComm(aCommData.mInterComm),
        mNumData(aSize),
        mIsDynamic(aIsDynamic),
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
