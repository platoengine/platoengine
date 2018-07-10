/*
 * Plato_Interface.cpp
 *
 *  Created on: March 20, 2017
 *
 */

#include <vector>
#include <string>
#include <cstdlib>
#include <stdlib.h>

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
        mPerformers(),
        mStages(),
        mExceptionHandler(nullptr),
        mLocalCommID(-1),
        mLocalPerformerName(),
        mInputData("Input Data"),
        mLocalComm(),
        mGlobalComm(aGlobalComm),
        mIsDone(false)
/******************************************************************************/
{
    // get the local program's communicator id from the environment.
    // This value (PLATO_COMM_ID) is specified as an argument to mpirun.
    const char* tCommChar = getenv("PLATO_COMM_ID");
    if(tCommChar)
    {
        mLocalCommID = atoi(tCommChar);
    }
    else
    {
        // throw
    }

    const char* input_char = getenv("PLATO_INTERFACE_FILE");
    Plato::Parser* parser = new Plato::PugiParser();
    mInputData = parser->parseFile(input_char);
    delete parser;

    this->createLocalComm();
    this->createPerformers();
}

/******************************************************************************/
Interface::Interface(const int & aCommID, const std::string & aXML_String, MPI_Comm aGlobalComm) :
        mDataLayer(nullptr),
        mPerformers(),
        mStages(),
        mExceptionHandler(nullptr),
        mLocalCommID(-1),
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

    this->createLocalComm();
    this->createPerformers();
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
    }
    handleExceptions();
}

/******************************************************************************/
Plato::Stage*
Interface::getStage(std::string aStageName)
/******************************************************************************/
{
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

    for(auto& tMyPerformer : mPerformers)
    {
        tMyPerformer->finalize();
    }
}

/******************************************************************************/
void Interface::perform(Plato::Stage* aStage)
/******************************************************************************/
{
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
void Interface::registerPerformer(Plato::Application* aApplication)
/******************************************************************************/
{
    Plato::Performer* tPerformer = nullptr;
    try
    {
        aApplication->initialize();
        tPerformer = this->getPerformer(mLocalCommID);
    }
    catch(...)
    {
        mExceptionHandler->Catch();
    }
    mExceptionHandler->handleExceptions();

    if(tPerformer)
    {
        tPerformer->setApplication(aApplication);
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
        Plato::Stage* tNewStage = new Plato::Stage(tStageInputDataMng, mPerformers, mDataLayer->getSharedData());
        mStages.push_back(tNewStage);
    }
}

/******************************************************************************/
void Interface::createPerformers()
/******************************************************************************/
{
    for( auto tNode : mInputData.getByName<Plato::InputData>("Performer") )
    {
        std::string tPerformerName = Plato::Get::String(tNode, "Name");
        int tLocalCommID = Plato::Get::Int(tNode, "CommID");

        // Only performers that are on this localComm are created
        if(tLocalCommID != mLocalCommID)
        {
            continue;
        }

        mLocalPerformerName = tPerformerName;

        mPerformers.push_back(new Plato::Performer(tPerformerName, tLocalCommID));
    }

    mExceptionHandler = new Plato::ExceptionHandler(mLocalPerformerName, mLocalComm, mGlobalComm);
}

/******************************************************************************/
Plato::Performer* Interface::getPerformer(int aPerformerCommID)
/******************************************************************************/
{
    for(Plato::Performer* tPerformer : mPerformers)
    {
        if(tPerformer->myCommID() == aPerformerCommID)
        {
            return tPerformer;
        }
    }
    std::stringstream tMessage;
    tMessage << "Plato::Interface: Performer not defined for CommID " << aPerformerCommID;
    throw Plato::ParsingException(tMessage.str());
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
        std::string tMyProviderName = Plato::Get::String(tNode, "OwnerName");

        std::vector<std::string> tMyReceiverNames = tNode.getByName<std::string>("UserName");
        tSharedDataInfo.setSharedDataMap(tMyProviderName, tMyReceiverNames);

        Plato::communication::broadcast_t tMyBroadcast =
                Plato::getBroadcastType(tCommunicationData.mLocalCommName, tMyProviderName, tMyReceiverNames);
        tSharedDataInfo.setMyBroadcast(tMyBroadcast);

        int tMySize = 1;
        if( tNode.size<std::string>("Size") )
        {
            tMySize = Plato::Get::Int(tNode, "Size");
        }
        std::string tMyName = Plato::Get::String(tNode, "Name");
        std::string tMyLayout = Plato::Get::String(tNode, "Layout");
        Parse::toUppercase(tMyLayout);
        tSharedDataInfo.setSharedDataSize(tMyName, tMySize);
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
void Interface::createLocalComm()
/******************************************************************************/
{
    int tMyRank;
    MPI_Comm_rank(mGlobalComm, &tMyRank);
    MPI_Comm_split(mGlobalComm, mLocalCommID, tMyRank, &mLocalComm);
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
    const size_t tNumStages = mStages.size();
    for(size_t tStageIndex = 0u; tStageIndex < tNumStages; tStageIndex++)
    {
        delete mStages[tStageIndex];
    }
    mStages.clear();
    const size_t tNumPerformers = mPerformers.size();
    for(size_t tPerformerIndex = 0u; tPerformerIndex < tNumPerformers; tPerformerIndex++)
    {
        delete mPerformers[tPerformerIndex];
    }
    mPerformers.clear();
}

} /* namespace Plato */

