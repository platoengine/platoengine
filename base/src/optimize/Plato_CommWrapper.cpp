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
// *************************************************************************
//@HEADER
*/

/*
 * Plato_CommWrapper.cpp
 *
 *  Created on: Sep 19, 2018
 */

#include <sstream>
#include <stdexcept>

#include "Plato_CommWrapper.hpp"

namespace Plato
{

CommWrapper::CommWrapper() :
        mComm()
{
}

CommWrapper::CommWrapper(const MPI_Comm & aInput) :
        mComm(aInput)
{
}

CommWrapper::~CommWrapper()
{
}

bool CommWrapper::isCommInitialized() const
{
    int tIsInitialized = 0;
    MPI_Initialized(&tIsInitialized);
    bool tOutput = tIsInitialized == static_cast<int>(1) ? true : false;
    return (tOutput);
}

void CommWrapper::useDefaultComm()
{
    mComm = MPI_COMM_WORLD;
}

void CommWrapper::setComm(const MPI_Comm & aInput)
{
    mComm = aInput;
}

int CommWrapper::size() const
{
    int tWorldSize = 0;
    try
    {
        if(mComm == MPI_COMM_NULL)
        {
            throw std::invalid_argument("\n\n ******** MESSAGE: NULL MPI COMMUNICATOR. ABORT! ******** \n\n");
        }
        MPI_Comm_size(mComm, &tWorldSize);
    }
    catch(const std::invalid_argument & tError)
    {
        std::ostringstream tMessage;
        tMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__
        << ", LINE: " << __LINE__ << " ******** \n\n";
        tMessage << tError.what();
        std::cout << tMessage.str().c_str() << std::flush;
        throw tError;
    }
    return (tWorldSize);
}

int CommWrapper::myProcID() const
{
    int tMyProcID = 0;
    try
    {
        if(mComm == MPI_COMM_NULL)
        {
            throw std::invalid_argument("\n\n ******** MESSAGE: NULL MPI COMMUNICATOR. ABORT! ******** \n\n");
        }
        MPI_Comm_rank(mComm, &tMyProcID);
    }
    catch(const std::invalid_argument & tError)
    {
        std::ostringstream tMessage;
        tMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__
        << ", LINE: " << __LINE__ << " ******** \n\n";
        tMessage << tError.what();
        std::cout << tMessage.str().c_str() << std::flush;
        throw tError;
    }
    return (tMyProcID);
}

} // namespace Plato
