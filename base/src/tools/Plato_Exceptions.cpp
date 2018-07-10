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
        mErrorStream << "  " << any_std_exception.what();
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

} // End namespace Plato
