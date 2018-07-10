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
