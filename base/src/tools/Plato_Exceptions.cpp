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
 * Plato_Exceptions.cpp
 *
 *  Created on: April 23, 2017
 *
 */

#include "Plato_Exceptions.hpp"
#include "Plato_Performer.hpp"

namespace Plato
{

/******************************************************************************/
void ExceptionHandler::Catch()
/******************************************************************************/
{
    try
    {
        throw;
    }
    catch(const ParsingException & tParsingException)
    {
        this->registerException(tParsingException);
    }
    catch(const LogicException & tLogicException)
    {
        this->registerException(tLogicException);
    }
    catch(const TerminateSignal & tTerminateSignal)
    {
        this->registerException(tTerminateSignal);
    }
    catch(std::exception const& any_std_exception)
    {
        this->registerException(any_std_exception);
    }
    catch(...)
    {
        this->registerException();
    }
}

/******************************************************************************/
void ExceptionHandler::registerException()
/******************************************************************************/
{
    this->setError();

    if(mMyPID == 0)
    {
        mErrorStream << " -- Fatal Error --------------------------------------------------------------" << std::endl;
        mErrorStream << "  Exception of unknown type on Performer '" << mMyCommName << "': " << std::endl;
        mErrorStream << " -----------------------------------------------------------------------------" << std::endl;
    }
}

/******************************************************************************/
void ExceptionHandler::setError()
/******************************************************************************/
{
    mErrorStatus = 1;
}

/******************************************************************************/
void ExceptionHandler::printAcout()
/******************************************************************************/
{
    std::ostringstream tBuffer;
    tBuffer << mErrorStream.str().c_str();

    int tData = 0;
    const int tRootPID = 0;
    if(mGlobalCommSize > 1)
    {
        if(mGlobalPID != tRootPID)
        {
            int tTag = 0;
            MPI_Status tStatus;
            int tSource = mGlobalPID - 1;
            MPI_Recv(&tData, 1, MPI_INT, tSource, tTag, mGlobalComm, &tStatus );
        }

        std::cout << tBuffer.str();
        if(tBuffer.str().size())
        {
            std::cout << std::endl;
        }
        std::cout.flush();

        if(mGlobalPID < (mGlobalCommSize - 1))
        {
            int tTag = 0;
            int tDestination = mGlobalPID + 1;
            MPI_Send(&tData, 1, MPI_INT, tDestination, tTag, mGlobalComm);
        }
        MPI_Barrier(mGlobalComm);
    }
    else
    {
        std::cout << tBuffer.str() << std::endl;
    }
}

/******************************************************************************/
void ExceptionHandler::registerException(const std::exception & any_std_exception)
/******************************************************************************/
{
    this->setError();

    if(mMyPID == 0)
    {
        mErrorStream << " -- Fatal Error --------------------------------------------------------------" << std::endl;
        mErrorStream << "  std::exception on Performer '" << mMyCommName << "': " << std::endl;
        mErrorStream << "  " << any_std_exception.what() << std::endl;
        mErrorStream << " -----------------------------------------------------------------------------" << std::endl;
    }
}

/******************************************************************************/
void ExceptionHandler::registerException(const Plato::ParsingException & aParsingException)
/******************************************************************************/
{
    this->setError();

    if(mMyPID == 0)
    {
        mErrorStream << " -- Fatal Error --------------------------------------------------------------" << std::endl;
        mErrorStream << "  Parsing exception on Performer '" << mMyCommName << "': " << std::endl;
        mErrorStream << "  " << aParsingException.message();
        mErrorStream << " -----------------------------------------------------------------------------" << std::endl;
    }
}

/******************************************************************************/
void ExceptionHandler::registerException(const Plato::LogicException & aLogicException)
/******************************************************************************/
{
    this->setError();
    if(mMyPID == 0)
    {
        mErrorStream << " -- Fatal Error --------------------------------------------------------------" << std::endl;
        mErrorStream << "  Logic exception on Performer '" << mMyCommName << "': " << std::endl;
        mErrorStream << "  " << aLogicException.message();
        mErrorStream << " -----------------------------------------------------------------------------" << std::endl;
    }
}

/******************************************************************************/
void ExceptionHandler::registerException(const Plato::TerminateSignal & aTerminateSignal)
/******************************************************************************/
{
    this->setError();
    if(mMyPID == 0)
    {
        mErrorStream << " -----------------------------------------------------------------------------" << std::endl;
        mErrorStream << "  Terminate signal received on Performer '" << mMyCommName << "': " << std::endl;
        mErrorStream << "  " << aTerminateSignal.message();
        mErrorStream << " -----------------------------------------------------------------------------" << std::endl;
    }
}

/******************************************************************************/
void ExceptionHandler::handleExceptions()
/******************************************************************************/
{
    int tOutput = 0;
    MPI_Allreduce(&mErrorStatus, &tOutput, 1, MPI_INT, MPI_SUM, mGlobalComm);
    if(tOutput > 0)
    {
        this->printAcout();
        throw 1;
    }
}

std::string ParsingException::message() const
{
    std::stringstream errorStream;
    errorStream << "  Error message: " << mMessage << std::endl;
    if(mContext.size())
    {
        errorStream << "  Context: " << std::endl;
        errorStream << mContext;
    }
    return errorStream.str();
}

std::string LogicException::message() const
{
    std::stringstream errorStream;
    errorStream << "  Error message: " << mMessage << std::endl;
    return errorStream.str();
}

std::string TerminateSignal::message() const
{
    std::stringstream errorStream;
    errorStream << "  Message: " << mMessage << std::endl;
    return errorStream.str();
}

/******************************************************************************/
ExceptionHandler::ExceptionHandler(const std::string & aLocalCommName, const MPI_Comm & aLocalComm, const MPI_Comm & aGlobalComm) :
        mMyPID(-1),
        mGlobalPID(-1),
        mErrorStatus(0),
        mGlobalCommSize(-1),
        mMyComm(aLocalComm),
        mGlobalComm(aGlobalComm),
        mMyCommName(aLocalCommName),
        mErrorStream()
/******************************************************************************/
{
    MPI_Comm_rank(mMyComm, &mMyPID);
    MPI_Comm_rank(mGlobalComm, &mGlobalPID);
    MPI_Comm_size(mGlobalComm, &mGlobalCommSize);
}

/******************************************************************************/
ParsingException::ParsingException(const std::string & aMessage, const pugi::xml_node& aContextNode) :
        mMessage(aMessage),
        mContext()
/******************************************************************************/
{
    std::stringstream tMessage;
    aContextNode.print(tMessage);
    mContext = tMessage.str();
}

/******************************************************************************/
ParsingException::ParsingException(const std::string & aMessage) :
        mMessage(aMessage),
        mContext("")
{
}
/******************************************************************************/

/******************************************************************************/
LogicException::LogicException(const std::string & aMessage) :
        mMessage(aMessage)
/******************************************************************************/
{
}

/******************************************************************************/
TerminateSignal::TerminateSignal(const std::string & aMessage) :
        mMessage(aMessage)
/******************************************************************************/
{
}
} // End namespace Plato
