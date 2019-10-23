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
#include <vector>
#include <sstream>
#include <stdexcept>

#include "Plato_Macros.hpp"
#include "Plato_Vector.hpp"

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
     * @brief Blocks until all processes in the communicator have reached this routine.
     * @brief @param [in] aRootProc root/source processor that owns the data
     * @brief @param [in] aInput input buffer own by root/source processor
     * @brief @param [out] aOutput output buffer own by all processors
     **********************************************************************************/
    template<typename ScalarType, typename OrdinalType>
    void broadcast(const OrdinalType & aRootProc,
                   const Plato::Vector<ScalarType, OrdinalType> & aInput,
                   Plato::Vector<ScalarType, OrdinalType> & aOutput) const
    {
        assert(aInput.size() == aOutput.size());

        const int tRootProc = aRootProc;
        const int tCount = aInput.size();
        std::vector<double> tBuffer(tCount);

        for(int tIndex = 0; tIndex < tCount; tIndex++)
        {
            tBuffer[tIndex] = aInput[tIndex];
        }

        MPI_Bcast(tBuffer.data(), tCount, MPI_DOUBLE, tRootProc, mComm);

        for(int tIndex= 0; tIndex < tCount; tIndex++)
        {
            aOutput[tIndex] = tBuffer[tIndex];
        }
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
                throw std::invalid_argument("NULL MPI COMMUNICATOR DETECTED.\n");
            }
            MPI_Comm_size(mComm, &tWorldSize);
        }
        catch(const std::invalid_argument & tError)
        {
            PRINTERR(tError.what());
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
                throw std::invalid_argument("NULL MPI COMMUNICATOR DETECTED.\n");
            }
            MPI_Comm_rank(mComm, &tMyProcID);
        }
        catch(const std::invalid_argument & tError)
        {
            PRINTERR(tError.what());
            throw tError;
        }
        return (tMyProcID);
    }

    /******************************************************************************//**
     * @brief Create a copy of the distributed memory communication wrapper
     * @return shared pointer
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
