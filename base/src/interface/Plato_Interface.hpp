/*
 * Plato_Interface.hpp
 *
 *  Created on: March 20, 2017
 *
 */

#ifndef SRC_INTERFACE_HPP_
#define SRC_INTERFACE_HPP_

#include <memory>

#include <Teuchos_ParameterList.hpp>

#include <mpi.h>
#include <vector>
#include <string>

#include "Plato_Parser.hpp"
#include "Plato_DataLayer.hpp"
#include "Plato_Exceptions.hpp"
#include "Plato_SharedData.hpp"

namespace Plato
{

class Stage;
class Performer;
class SharedData;
class Application;
struct aSharedDataInfo;
struct CommunicationData;

enum stage_index_t
{
    INVALID_STAGE = -2,
    TERMINATE_STAGE = -1
};

/**
 * This class provides an interface between PlatoEngine and the 
 * hosted codes. 
 */

/******************************************************************************/
//!  Interface between optimizer and Plato
/*!
 This should be separated into two virtual bases.
 */
/******************************************************************************/
class Interface
{
public:
    explicit Interface(MPI_Comm aGlobalComm = MPI_COMM_WORLD);
    Interface(const int & aCommID, const std::string & a_XML_String, MPI_Comm aGlobalComm = MPI_COMM_WORLD);
    virtual ~Interface();

    // optimizer interface
    void compute(const std::string & stageName, Teuchos::ParameterList & aArguments);
    void compute(const std::vector<std::string> & stageNames, Teuchos::ParameterList & aArguments);

    void registerPerformer(Plato::Application* aPerformer);
    void perform();

    // data motion
    int size(const std::string & aName) const;
    void exportData(double* aFrom, Plato::SharedData* aTo);
    void importData(double* aTo, Plato::SharedData* aFrom);

    // local communicator functionality
    void getLocalComm(MPI_Comm& aLocalComm);

    // accessors
    Plato::Stage* getStage();
    Plato::Stage* getStage(std::string aStageName);
    Plato::Performer* getPerformer(int aPerformerCommID);
    std::string getLocalPerformerName(){return mLocalPerformerName;}
    int getStageIndex(std::string aStageName) const;
    Plato::InputData getInputData() const;

    // error handling
    void Catch();
    void handleExceptions();
    void registerException();
    void registerException(Plato::ParsingException aParsingException);
    void registerException(Plato::LogicException aLogicException);

    // control
    bool isDone();

private:
    void perform(Plato::Stage* aStage);
    void broadcastStageIndex(int & aStageIndex);

    void createStages();
    void createLocalComm();
    void createPerformers();
    void createSharedData(Plato::Application* aApplication);

    void exportGraph(const Plato::SharedDataInfo & aSharedDataInfo,
                     Plato::Application* aApplication,
                     Plato::CommunicationData & aCommunicationData);
    void exportOwnedGlobalIDs(const Plato::data::layout_t & aLayout,
                              Plato::Application* aApplication,
                              Plato::CommunicationData & aCommunicationData);

private:
    Plato::DataLayer* mDataLayer = nullptr;

    std::vector<Plato::Performer*> mPerformers;
    std::vector<Plato::Stage*> mStages;

    Plato::ExceptionHandler* mExceptionHandler;

    int mLocalCommID;
    std::string mLocalPerformerName;
    Plato::InputData mInputData;

    MPI_Comm mLocalComm;
    MPI_Comm mGlobalComm;
    bool mIsDone;
};

} /* namespace Plato */

#endif /* SRC_INTERFACE_HPP_ */
