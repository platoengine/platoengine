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

void CommWrapper::setDefaultComm()
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
        if(mComm == nullptr)
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
        if(mComm == nullptr)
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
