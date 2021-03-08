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
 * Plato_Interface.cpp
 *
 *  Created on: March 20, 2017
 *
 */

#include <limits>
#include <vector>
#include <string>
#include <cstdlib>
#include <stdlib.h>
#include <fstream>

#include "Plato_Interface.hpp"
#include "Plato_InputData.hpp"
#include "Plato_SharedField.hpp"
#include "Plato_SharedValue.hpp"
#include "Plato_Application.hpp"
#include "Plato_Performer.hpp"
#include "Plato_Operation.hpp"
#include "Plato_Stage.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_Parser.hpp"
#include "Plato_SharedDataInfo.hpp"
#include "Plato_StageInputDataMng.hpp"

namespace Plato
{

/******************************************************************************/
Interface::Interface(MPI_Comm aGlobalComm) :
        mDataLayer(nullptr),
        mPerformer(nullptr),
        mStages(),
        mExceptionHandler(nullptr),
        mConsole(nullptr),
        mLocalCommID(-1),
        mPerformerID(-1),
        mLocalPerformerName(),
        mInputData("Input Data"),
        mLocalComm(),
        mGlobalComm(aGlobalComm),
        mIsDone(false)
/******************************************************************************/
{
    // get the local program's communicator id from the environment.
    // This value (PLATO_PERFORMER_ID) is specified as an argument to mpirun.
    const char* tPerfChar = getenv("PLATO_PERFORMER_ID");
    if(tPerfChar)
    {
        mPerformerID = atoi(tPerfChar);
    }
    else
    {
        // throw
    }

    const char* input_char = getenv("PLATO_INTERFACE_FILE");
    Plato::Parser* parser = new Plato::PugiParser();
    mInputData = parser->parseFile(input_char);
    delete parser;

    this->createPerformers();

    mConsole = new Console(mLocalPerformerName, mPerformerID, Plato::Get::InputData(mInputData,"Console"), mLocalComm);
}

/******************************************************************************/
Interface::Interface(const int & aCommID, const std::string & aXML_String, MPI_Comm aGlobalComm) :
        mDataLayer(nullptr),
        mPerformer(nullptr),
        mStages(),
        mExceptionHandler(nullptr),
        mConsole(nullptr),
        mLocalCommID(-1),
        mPerformerID(-1),
        mLocalPerformerName(),
        mInputData("Input Data"),
        mLocalComm(),
        mGlobalComm(aGlobalComm),
        mIsDone(false)
/******************************************************************************/
{
    // get the local program's communicator id.
    // This value is specified as an argument to mpirun.
    // if(comm_id is valid) TODO: not sure what valid is here
    mLocalCommID = aCommID;
    //else
    // throw

    const char* input_char = getenv("PLATO_INTERFACE_FILE");
    Plato::Parser* parser = new Plato::PugiParser();
    mInputData = parser->parseFile(input_char);

    this->createPerformers();

    mConsole = new Console(mLocalPerformerName, mPerformerID, Plato::Get::InputData(mInputData,"Console"), mLocalComm);
}

/******************************************************************************/
void Interface::getLocalComm(MPI_Comm& aLocalComm)
/******************************************************************************/
{
    aLocalComm = mLocalComm;
}

/******************************************************************************/
Plato::InputData Interface::getInputData() const
/******************************************************************************/
{
    return mInputData;
}

/******************************************************************************/
int Interface::getStageIndex(std::string aStageName) const
/******************************************************************************/
{
    for(size_t tIndex = 0; tIndex < mStages.size(); ++tIndex)
    {
        if(mStages[tIndex]->getName() == aStageName)
        {
            return tIndex;
        }
    }
    return -1;
}

/******************************************************************************/
void Interface::broadcastStageIndex(int & aStageIndex)
/******************************************************************************/
{
    MPI_Bcast(&aStageIndex, 1, MPI_INT, 0, mGlobalComm);

    if(aStageIndex == INVALID_STAGE)
    {
        std::stringstream tMsg;
        tMsg << "\n\n ********** PLATO ERROR: Interface::broadcastStageIndex: Invalid stage requested.\n\n";
        Plato::ParsingException tParsingException(tMsg.str());
        registerException(tParsingException);
    } else
    if(aStageIndex == TERMINATE_STAGE)
    {
        std::stringstream tMsg;
        tMsg << "\n\n INFO: Interface::broadcastStageIndex: Terminate stage requested.  Exiting.\n\n";
        Plato::TerminateSignal tTerminateSignal(tMsg.str());
        registerException(tTerminateSignal);
    }
    handleExceptions();
}

/******************************************************************************/
Plato::Stage*
Interface::getStage(std::string aStageName)
/******************************************************************************/
{
    // check for control file
    std::ifstream tControlFile;
    tControlFile.open("plato.control");
    if(tControlFile)
    {
        Plato::Parser* parser = new Plato::PugiParser();
        auto tControlData = parser->parseFile("plato.control");
        delete parser;

        auto tTerminate = Plato::Get::Bool(tControlData, "Terminate", false);

        if(tTerminate) aStageName = "Terminate";
    }

    // broadcast the index of the next stage
    int tStageIndex;
    if(aStageName == "Terminate")
        tStageIndex = TERMINATE_STAGE;
    else
        tStageIndex = getStageIndex(aStageName);

    broadcastStageIndex(tStageIndex);

    if(tStageIndex >= 0)
    {
        return mStages[tStageIndex];
    }
    else
    {
        mIsDone = true;
        return nullptr;
    }
}

/******************************************************************************/
Plato::Stage*
Interface::getStage()
/******************************************************************************/
{

    // broadcast the index of the next stage
    int tStageIndex;

    broadcastStageIndex(tStageIndex);

    if(tStageIndex >= 0)
    {
        return mStages[tStageIndex];
    }
    else
    {
        mIsDone = true;
        return nullptr;
    }
}

/******************************************************************************/
void Interface::perform()
/******************************************************************************/
{
    mExceptionHandler->handleExceptions();

    while(this->isDone() == false)
    {

        // performers 'hang' here until a new stage is established
        Plato::Stage* tStage = this->getStage();

        // 'Terminate' stage is nullptr
        if(!tStage)
        {
            continue;
        }

        this->perform(tStage);

    }

      mPerformer->finalize();
}

/******************************************************************************/
void Interface::perform(Plato::Stage* aStage)
/******************************************************************************/
{
    Console::Status("Stage: (" + mPerformer->myName() + ") " + aStage->getName());

    // transmits input data
    //
    aStage->begin();

    // any local operations?
    //
    Plato::Operation* tOperation = aStage->getNextOperation();
    while(tOperation)
    {

        tOperation->sendInput();

        // copy data from Plato::SharedData buffers to hostedCode data containers
        //
        std::vector<std::string> tOperationInputDataNames = tOperation->getInputDataNames();
        for(std::string tName : tOperationInputDataNames)
        {
            try
            {
                tOperation->importData(tName, mDataLayer->getSharedData(tName));
            }
            catch(...)
            {
                mExceptionHandler->Catch();
            }
            mExceptionHandler->handleExceptions();
        }

        try
        {
            tOperation->compute();
        }
        catch(...)
        {
            mExceptionHandler->Catch();
        }
        mExceptionHandler->handleExceptions();

        // copy data from hostedCode data containers to Plato::SharedData buffers
        //
        std::vector<std::string> tOperationOutputDataNames = tOperation->getOutputDataNames();
        for(std::string tName : tOperationOutputDataNames)
        {
            try
            {
                tOperation->exportData(tName, mDataLayer->getSharedData(tName));
            }
            catch(...)
            {
                mExceptionHandler->Catch();
            }
            mExceptionHandler->handleExceptions();
        }

        tOperation->sendOutput();

        tOperation = aStage->getNextOperation();
    }

    // transmits output data
    //
    aStage->end();
}

void Interface::finalize()
{
    this->getStage("Terminate");
}

/******************************************************************************/
void Interface::compute(const std::vector<std::string> & aStageNames, Teuchos::ParameterList& aArguments)
/******************************************************************************/
{
    for(const std::string & tStageName : aStageNames)
    {
        this->compute(tStageName, aArguments);
    }
}

/******************************************************************************/
void Interface::compute(const std::string & aStageName, Teuchos::ParameterList& aArguments)
/******************************************************************************/
{
    // find requested stage
    Plato::Stage* tStage = getStage(aStageName);

    // Unpack input arguments into Plato::SharedData
    //
    std::vector<std::string> tStageInputDataNames = tStage->getInputDataNames();
    for(std::string tName : tStageInputDataNames)
    {
        exportData(aArguments.get<double*>(tName), mDataLayer->getSharedData(tName));
    }

    this->perform(tStage);

    // Unpack output arguments from Plato::SharedData
    //
    std::vector<std::string> tStageOutputDataNames = tStage->getOutputDataNames();
    for(std::string tName : tStageOutputDataNames)
    {
        importData(aArguments.get<double*>(tName), mDataLayer->getSharedData(tName));
    }
}

/******************************************************************************/
void Interface::exportData(double* aFrom, Plato::SharedData* aTo)
/******************************************************************************/
{
    int tMyLength = aTo->size();
    std::vector<double> tExportData(tMyLength);
    std::copy(aFrom, aFrom + tMyLength, tExportData.begin());
    aTo->setData(tExportData);
}

/******************************************************************************/
void Interface::importData(double* aTo, Plato::SharedData* aFrom)
/******************************************************************************/
{
    int tMyLength = aFrom->size();
    std::vector<double> tImportData(tMyLength);
    aFrom->getData(tImportData);
    std::copy(tImportData.begin(), tImportData.end(), aTo);
}

/******************************************************************************/
void Interface::registerApplication(Plato::Application* aApplication)
/******************************************************************************/
{
    if(aApplication == nullptr)
    {
        registerException(Plato::ParsingException("Failed to create Performer"));
    }
    mExceptionHandler->handleExceptions();
    
    try
    {
        aApplication->initialize();
    }
    catch(...)
    {
        mExceptionHandler->Catch();
    }
    mExceptionHandler->handleExceptions();

    if(mPerformer)
    {
        mPerformer->setApplication(aApplication);
    }

    try
    {
        this->createSharedData(aApplication);
    }
    catch(...)
    {
        mExceptionHandler->Catch();
    }
    mExceptionHandler->handleExceptions();

    try
    {
        this->createStages();
    }
    catch(...)
    {
        mExceptionHandler->Catch();
    }
    mExceptionHandler->handleExceptions();
}

/******************************************************************************/
void Interface::createStages()
/******************************************************************************/
{
    auto tStages = mInputData.getByName<Plato::InputData>("Stage");
    for(auto tStageNode=tStages.begin(); tStageNode!=tStages.end(); ++tStageNode)
    {
        Plato::StageInputDataMng tStageInputDataMng;
        Plato::Parse::parseStageData(*tStageNode, tStageInputDataMng);
        Plato::Stage* tNewStage = new Plato::Stage(tStageInputDataMng, mPerformer, mDataLayer->getSharedData());
        mStages.push_back(tNewStage);
    }
}

/******************************************************************************/
void Interface::createPerformers()
/******************************************************************************/
{
    int tMyRank, tNumGlobalRanks;
    MPI_Comm_rank(mGlobalComm, &tMyRank);
    MPI_Comm_size(mGlobalComm, &tNumGlobalRanks);

    std::vector<int> tPerfIDs;
    std::map<int,int> tPerfCommSize;
    for( auto tNode : mInputData.getByName<Plato::InputData>("Performer") )
    {
        // is a PerformerID specified?  If not, error out.
        //
        int tLocalPerformerID = Plato::Get::Int(tNode, "PerformerID", std::numeric_limits<int>::min());

        if( tLocalPerformerID == std::numeric_limits<int>::min() )
        {
            if( tMyRank == 0 )
            {
                std::cout << " -- Fatal Error --------------------------------------------------------------" << std::endl;
                std::cout << "  Each Performer definition must include a 'PerformerID'." << std::endl;
                std::cout << " -----------------------------------------------------------------------------" << std::endl;
            }
            throw 1;
        } 
       
        // is the PerformerID already used? If so, error out.
        //
        if( std::count( tPerfIDs.begin(), tPerfIDs.end(), tLocalPerformerID ) )
        {
            if( tMyRank == 0 )
            {
                std::cout << " -- Fatal Error --------------------------------------------------------------" << std::endl;
                std::cout << "  Duplicate PerformerID's.  Each performer must have a unique PerformerID." << std::endl;
                std::cout << " -----------------------------------------------------------------------------" << std::endl;
            }
            throw 1;
        } 
        else 
        {
            tPerfIDs.push_back(tLocalPerformerID);
        }


        // Are any PerformerIDs specified in the interface definition that weren't 
        // defined on the mpi command line?
        //
        int tMyPerformerSpec = (tLocalPerformerID == mPerformerID) ? 1 : 0;
        int tNumRanksThisID = 0;
        MPI_Allreduce(&tMyPerformerSpec, &tNumRanksThisID, 1, MPI_INT, MPI_SUM, mGlobalComm);
        if( tNumRanksThisID == 0 ){
            if( tMyRank == 0 )
            {
                std::cout << " -- Fatal Error --------------------------------------------------------------" << std::endl;
                std::cout << "  A Performer spec is defined for which no PerformerID is given on the mpi command line." << std::endl;
                std::cout << " -----------------------------------------------------------------------------" << std::endl;
            }
            throw 1;
        }
        else 
        {
           tPerfCommSize[tLocalPerformerID] = tNumRanksThisID;
        }
    }

    // Is there a Performer spec for my local Performer ID?  
    //
    int tErrorNoSpec = ( std::count( tPerfIDs.begin(), tPerfIDs.end(), mPerformerID ) == 0 ) ? 1 : 0;
    int tErrorNoSpecGlobal = 0;
    MPI_Allreduce(&tErrorNoSpec, &tErrorNoSpecGlobal, 1, MPI_INT, MPI_MAX, mGlobalComm);
    if( tErrorNoSpecGlobal ){
        if( tMyRank == 0 )
        {
            std::cout << " -- Fatal Error --------------------------------------------------------------" << std::endl;
            std::cout << "  A Performer spec must be provided for each PerformerID defined on the mpi command line." << std::endl;
            std::cout << " -----------------------------------------------------------------------------" << std::endl;
        }
        throw 1;
    }

    // If the Performer spec has N names defined then the allocated ranks on that PerformerID are
    // broken into N local comms.  To avoid any semantics of how ranks are assigned, manually color 
    // the local comms before splitting.
    //
    std::vector<int> tPerformerIDs(tNumGlobalRanks);
    MPI_Allgather(&mPerformerID, 1, MPI_INT, tPerformerIDs.data(), 1, MPI_INT, mGlobalComm);

    int tCommIndex = 0;
    for( auto tNode : mInputData.getByName<Plato::InputData>("Performer") )
    {
        int tLocalPerformerID = Plato::Get::Int(tNode, "PerformerID");
        int tNumRanksThisID = tPerfCommSize[tLocalPerformerID];
        auto tPerformerNames = tNode.getByName<std::string>("Name");
        int tNumCommsThisID = tPerformerNames.size();
        int tLocalPerformerCommSize = tNumRanksThisID / tNumCommsThisID;

        // Does the number of Comms partition the ranks for this PerformerID without a remainder?
        //
        int tErrorUneven = ( tNumCommsThisID * tLocalPerformerCommSize == tNumRanksThisID ) ? 0 : 1;
        if( tErrorUneven ){
            if( tMyRank == 0 )
            {
                    std::cout << " -- Fatal Error --------------------------------------------------------------" << std::endl;
                    std::cout << "  Error creating performer with ID=" << tLocalPerformerID << "." << std::endl;
                    std::cout << "  Attempted to spread N=" << tNumRanksThisID 
                              << " processes over M=" << tNumCommsThisID << " comms, but N % M != 0." << std::endl;
                    std::cout << "  Change N to a whole number multiple of M and try again." << std::endl;
                    std::cout << " -----------------------------------------------------------------------------" << std::endl;
            }
            throw 1;
        }

        // loop through the PerformerIDs and assign to local Comms
        int tRankCount = 0;
        for( int i=0; i<tNumGlobalRanks; i++ )
        {
            if( tPerformerIDs[i] == tLocalPerformerID )
            {
                if( i == tMyRank )
                {
                     int tNameIndex = tRankCount/tLocalPerformerCommSize;
                     mLocalPerformerName = tPerformerNames[tNameIndex];
                     mLocalCommID = tCommIndex;
                }
                tRankCount++;
                if( tRankCount % tLocalPerformerCommSize == 0 )
                {
                    tCommIndex++;
                }
            }
        }
    }

    // Did any rank not find their local comm id?
    //
    int tErrorNoComm = ( mLocalCommID == -1 ) ? 1 : 0;
    int tErrorNoCommGlobal = 0;
    MPI_Allreduce(&tErrorNoComm, &tErrorNoCommGlobal, 1, MPI_INT, MPI_MAX, mGlobalComm);
    if( tErrorNoCommGlobal ){
        if( tMyRank == 0 )
        {
                std::cout << " -- Fatal Error --------------------------------------------------------------" << std::endl;
                std::cout << "  Not all ranks were assigned to a local comm. " << std::endl;
                std::cout << " -----------------------------------------------------------------------------" << std::endl;
        }
        throw 1;
    }

    MPI_Comm_split(mGlobalComm, mLocalCommID, tMyRank, &mLocalComm);

    mPerformer = std::make_shared<Plato::Performer>(mLocalPerformerName, mLocalCommID);

    mExceptionHandler = new Plato::ExceptionHandler(mLocalPerformerName, mLocalComm, mGlobalComm);
}

/******************************************************************************/
void Interface::createSharedData(Plato::Application* aApplication)
/******************************************************************************/
{
    Plato::CommunicationData tCommunicationData;
    tCommunicationData.mLocalComm = mLocalComm;
    tCommunicationData.mInterComm = mGlobalComm;
    tCommunicationData.mLocalCommName = mLocalPerformerName;

    Plato::SharedDataInfo tSharedDataInfo;
    for( auto tNode : mInputData.getByName<Plato::InputData>("SharedData") )
    {
        std::vector<std::string> tMyProviderNames = tNode.getByName<std::string>("OwnerName");

        std::vector<std::string> tMyReceiverNames = tNode.getByName<std::string>("UserName");
        tSharedDataInfo.setSharedDataMap(tMyProviderNames, tMyReceiverNames);

        Plato::communication::broadcast_t tMyBroadcast =
                Plato::getBroadcastType(tCommunicationData.mLocalCommName, tMyProviderNames, tMyReceiverNames);
        tSharedDataInfo.setMyBroadcast(tMyBroadcast);

        int tMySize = 1;
        if( tNode.size<std::string>("Size") )
        {
            tMySize = Plato::Get::Int(tNode, "Size");
        }
        std::string tMyName = Plato::Get::String(tNode, "Name");
        std::string tMyLayout = Plato::Get::String(tNode, "Layout");
        bool tIsDynamic = Plato::Get::Bool(tNode, "Dynamic", false);
        Parse::toUppercase(tMyLayout);
        tSharedDataInfo.setSharedDataSize(tMyName, tMySize);
        tSharedDataInfo.setSharedDataDynamic(tMyName, tIsDynamic);
        tSharedDataInfo.setSharedDataIdentifiers(tMyName, tMyLayout);
    }

    this->exportGraph(tSharedDataInfo, aApplication, tCommunicationData);
    mDataLayer = new Plato::DataLayer(tSharedDataInfo, tCommunicationData);
}

/******************************************************************************/
void Interface::exportGraph(const Plato::SharedDataInfo & aSharedDataInfo,
                            Plato::Application* aApplication,
                            Plato::CommunicationData & aCommunicationData)
/******************************************************************************/
{
    if(aSharedDataInfo.isLayoutDefined("NODAL FIELD") == true)
    {
        auto tLayout = Plato::data::layout_t::SCALAR_FIELD;
        this->exportOwnedGlobalIDs(tLayout, aApplication, aCommunicationData);
    }

    if(aSharedDataInfo.isLayoutDefined("ELEMENT FIELD") == true)
    {
        auto tLayout = Plato::data::layout_t::ELEMENT_FIELD;
        this->exportOwnedGlobalIDs(tLayout, aApplication, aCommunicationData);
    }
}

/******************************************************************************/
void Interface::exportOwnedGlobalIDs(const Plato::data::layout_t & aLayout,
                                     Plato::Application* aApplication,
                                     Plato::CommunicationData & aCommunicationData)
/******************************************************************************/
{
    try
    {
        aApplication->exportDataMap(aLayout, aCommunicationData.mMyOwnedGlobalIDs[aLayout]);
    }
    catch(...)
    {
        mExceptionHandler->Catch();
    }
}

/******************************************************************************/
int Interface::size(const std::string & aName) const
/******************************************************************************/
{
    int tLength = 0;
    Plato::SharedData* tSharedData = mDataLayer->getSharedData(aName);
    if(tSharedData)
    {
        tLength = tSharedData->size();
    }
    else
    {
        // TODO: throw?  return zereo?
    }
    return tLength;
}

/******************************************************************************/
void Interface::Catch()
/******************************************************************************/
{
    mExceptionHandler->Catch();
}

/******************************************************************************/
void Interface::registerException(Plato::ParsingException aParsingException)
/******************************************************************************/
{
    mExceptionHandler->registerException(aParsingException);
}

/******************************************************************************/
void Interface::registerException(Plato::LogicException aLogicException)
/******************************************************************************/
{
    mExceptionHandler->registerException(aLogicException);
}

/******************************************************************************/
void Interface::registerException(Plato::TerminateSignal aTerminateSignal)
/******************************************************************************/
{
    mExceptionHandler->registerException(aTerminateSignal);
}

/******************************************************************************/
void Interface::handleExceptions()
/******************************************************************************/
{
    mExceptionHandler->handleExceptions();
}

/******************************************************************************/
void Interface::registerException()
/******************************************************************************/
{
    mExceptionHandler->registerException();
}

/******************************************************************************/
bool Interface::isDone()
/******************************************************************************/
{
    return mIsDone;
}

/******************************************************************************/
Interface::~Interface()
/******************************************************************************/
{
    if(mDataLayer)
    {
        delete mDataLayer;
        mDataLayer = nullptr;
    }
    if(mExceptionHandler)
    {
        delete mExceptionHandler;
        mExceptionHandler = nullptr;
    }
    if(mConsole)
    {
        delete mConsole;
        mConsole = nullptr;
    }
    const size_t tNumStages = mStages.size();
    for(size_t tStageIndex = 0u; tStageIndex < tNumStages; tStageIndex++)
    {
        delete mStages[tStageIndex];
    }
    mStages.clear();
}

} /* namespace Plato */

