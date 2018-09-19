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
            mComm()
    {
    }

    /******************************************************************************//**
     * @brief Constructor
     * @param [in] aInput MPI communicator
     **********************************************************************************/
    explicit CommWrapper(const MPI_Comm & aInput) :
            mComm(aInput)
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
        int tIsInitialized = 0;
        MPI_Initialized(&tIsInitialized);
        bool tOutput = tIsInitialized == static_cast<int>(1) ? true : false;
        return (tOutput);
    }

    /******************************************************************************//**
     * @brief Set communicator to MPI_COMM_WORLD
     **********************************************************************************/
    void useDefaultComm()
    {
        mComm = MPI_COMM_WORLD;
    }

    /******************************************************************************//**
     * @brief Set MPI communicator
     * @param [in] aInput MPI communicator
     **********************************************************************************/
    void setComm(const MPI_Comm & aInput)
    {
        mComm = aInput;
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
            if(mComm == nullptr)
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
            if(mComm == nullptr)
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
};
// class CommWrapper

}// namespace Plato
