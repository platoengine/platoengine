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
 * Plato_CommWrapper.hpp
 *
 *  Created on: Sep 19, 2018
 */

#pragma once

#include <mpi.h>

#include <memory>
#include <sstream>
#include <stdexcept>

namespace Plato
{

/******************************************************************************//**
 * @brief Simple wrapper class for distributed memory applications.
 **********************************************************************************/
class CommWrapper
{
public:
    /******************************************************************************//**
     * @brief Default constructor
     **********************************************************************************/
    CommWrapper() :
            mComm(),
            mIsCommSet(false)
    {
    }

    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aInput MPI communicator
     **********************************************************************************/
    explicit CommWrapper(const MPI_Comm & aInput) :
            mComm(aInput),
            mIsCommSet(true)
    {
    }

    /******************************************************************************//**
     * @brief Destructor
     **********************************************************************************/
    ~CommWrapper()
    {
    }

    /******************************************************************************//**
     * @brief Indicates if MPI_Init has been called
     * @return true/false flag
     **********************************************************************************/
    bool isCommInitialized() const
    {
        return (mIsCommSet);
    }

    /******************************************************************************//**
     * @brief Set communicator to MPI_COMM_WORLD
     **********************************************************************************/
    void useDefaultComm()
    {
        mComm = MPI_COMM_WORLD;
        mIsCommSet = true;
    }

    /******************************************************************************//**
     * @brief Set MPI communicator
     * @param [in] aInput MPI communicator
     **********************************************************************************/
    void setComm(const MPI_Comm & aInput)
    {
        mComm = aInput;
        mIsCommSet = true;
    }

    /******************************************************************************//**
     * @brief Return size of MPI communicator
     * @return size of MPI communicator
     **********************************************************************************/
    int size() const
    {
        int tWorldSize = 0;
        try
        {
            if( mIsCommSet == false )
            {
                throw std::invalid_argument("\n\n ******** MESSAGE: NULL MPI COMMUNICATOR. ABORT! ******** \n\n");
            }
            MPI_Comm_size(mComm, &tWorldSize);
        }
        catch(const std::invalid_argument & tError)
        {
            std::ostringstream tMessage;
            tMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__ << ", LINE: "
                     << __LINE__ << " ******** \n\n";
            tMessage << tError.what();
            std::cout << tMessage.str().c_str() << std::flush;
            throw tError;
        }
        return (tWorldSize);
    }

    /******************************************************************************//**
     * @brief Return the rank of the calling process in the communicator
     * @return my processor rank
    **********************************************************************************/
    int myProcID() const
    {
        int tMyProcID = 0;
        try
        {
            if( mIsCommSet == false )
            {
                throw std::invalid_argument("\n\n ******** MESSAGE: NULL MPI COMMUNICATOR. ABORT! ******** \n\n");
            }
            MPI_Comm_rank(mComm, &tMyProcID);
        }
        catch(const std::invalid_argument & tError)
        {
            std::ostringstream tMessage;
            tMessage << "\n\n ******** ERROR IN FILE: " << __FILE__ << ", FUNCTION: " << __PRETTY_FUNCTION__ << ", LINE: "
                     << __LINE__ << " ******** \n\n";
            tMessage << tError.what();
            std::cout << tMessage.str().c_str() << std::flush;
            throw tError;
        }
        return (tMyProcID);
    }

    /******************************************************************************//**
     * @brief Create a copy of the distributed memory communication wrapper
     * @return my processor rank
    **********************************************************************************/
    std::shared_ptr<Plato::CommWrapper> create() const
    {
        return (std::make_shared<Plato::CommWrapper>(mComm));
    }

private:
    MPI_Comm mComm; /*!< MPI communicator */
    bool mIsCommSet; /*!< specifies if MPI communicator is set */
};
// class CommWrapper

}// namespace Plato
