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

    // TODO: deprecate this function.  'registerPerformer' is misleading.  
    void registerPerformer(Plato::Application* aApplication){ this->registerApplication(aApplication); }
    void registerApplication(Plato::Application* aApplication);
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

    std::shared_ptr<Plato::Performer> mPerformer;
    std::vector<Plato::Stage*> mStages;

    Plato::ExceptionHandler* mExceptionHandler;

    int mLocalCommID;
    int mPerformerID;
    std::string mLocalPerformerName;
    Plato::InputData mInputData;

    MPI_Comm mLocalComm;
    MPI_Comm mGlobalComm;
    bool mIsDone;
};

} /* namespace Plato */

#endif /* SRC_INTERFACE_HPP_ */
