/*
 * Plato_Communication.hpp
 *
 *  Created on: Oct 3, 2017
 */

#ifndef PLATO_COMMUNICATION_HPP_
#define PLATO_COMMUNICATION_HPP_

#include <vector>
#include <string>
#include <cassert>
#include <map>

#include "mpi.h"

#include "Plato_SharedData.hpp"

namespace Plato
{

struct CommunicationData
{
public:
    CommunicationData() :
            mLocalComm(),
            mInterComm(),
            mLocalCommName(),
            mMyOwnedGlobalIDs()
    {
    }
    ~CommunicationData()
    {
    }

    MPI_Comm mLocalComm;
    MPI_Comm mInterComm;
    std::string mLocalCommName;
    std::map<data::layout_t,std::vector<int>> mMyOwnedGlobalIDs;
};

}

#endif /* PLATO_COMMUNICATION_HPP_ */
