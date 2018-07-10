/*
 * Plato_Exception.hpp
 *
 *  Created on: April 23, 2017
 *
 */

#ifndef SRC_EXCEPTION_HPP_
#define SRC_EXCEPTION_HPP_

#include <string>
#include <sstream>
#include <iostream>

#include "mpi.h"
#include "Plato_Parser.hpp"

namespace Plato
{

class Performer;

class ParsingException
{
public:
    explicit ParsingException(const std::string & aMessage);
    ParsingException(const std::string & aMessage, const pugi::xml_node& aContextNode);
    std::string message() const;

private:
    std::string mMessage;
    std::string mContext;
};

class LogicException
{
public:
    explicit LogicException(const std::string & aMessage);
    std::string message() const;

private:
    std::string mMessage;
};

/******************************************************************************/
//!  Exception handler class.
/*!
 The ExceptionHandler class is used to record Exceptions and manage a
 graceful exit of MPMD processes.
 */
/******************************************************************************/
class ExceptionHandler
{
public:
    ExceptionHandler(const std::string & aLocalCommName, const MPI_Comm & aLocalComm, const MPI_Comm & aGlobalComm);

    void Catch();
    void handleExceptions();

    void registerException(const Plato::ParsingException & aParsingException);
    void registerException(const Plato::LogicException & aLogicException);
    void registerException(const std::exception& any_std_exception);
    void registerException();

private:
    void setError();
    void printAcout();

private:
    int mMyPID;
    int mGlobalPID;
    int mErrorStatus;
    int mGlobalCommSize;

    MPI_Comm mMyComm;
    MPI_Comm mGlobalComm;
    std::string mMyCommName;
    std::stringstream mErrorStream;

};

} // end namespace Plato

#endif
