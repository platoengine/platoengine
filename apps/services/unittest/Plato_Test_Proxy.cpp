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
 * Plato_Test_Proxy.cpp
 *
 *  Created on: Sep 27, 2017
 */

#ifdef STK_ENABLED

#include <gtest/gtest.h>

#include <memory>
#include <numeric>

#include <mpi.h>

#include "Epetra_MpiComm.h"
#include "Epetra_Map.h"
#include "Epetra_Vector.h"
#include "Epetra_Import.h"
#include "Epetra_Export.h"

#include "Plato_Performer.hpp"
#include "Plato_SharedValue.hpp"
#include "Plato_SharedField.hpp"
#include "Plato_Application.hpp"
#include "Plato_StkMeshUtils.hpp"
#include "Plato_Communication.hpp"
#include "Plato_PlatoProxyApp.hpp"
#include "Plato_SharedDataInfo.hpp"
#include "Plato_PhysicsProxyApp.hpp"

namespace PlatoTest
{

struct color
{
    enum application_t
    {
        PLATO = 0,
        PHYSICS = 1,
        UNDEFINED = 2,
    };
};

TEST(PlatoTest, ProxyApp)
{
    int tWorldCommSize = -1;
    MPI_Comm_size(MPI_COMM_WORLD, &tWorldCommSize);

    const int tPlatoCommSize = 2;
    const int tPhysicsCommSize = tWorldCommSize - tPlatoCommSize;

    const int tMinWorldCommSize = 3;
    if(tWorldCommSize > tMinWorldCommSize)
    {
        MPI_Comm tAppComm;
        int tWorldCommRank = -1;
        MPI_Comm_rank(MPI_COMM_WORLD, &tWorldCommRank);
        PlatoTest::color::application_t tColor = PlatoTest::color::UNDEFINED;
        if(tWorldCommRank < tPlatoCommSize)
        {
            tColor = PlatoTest::color::PLATO;
        }
        else
        {
            tColor = PlatoTest::color::PHYSICS;
        }
        // Split the communicator based on the color and use the original rank for ordering
        MPI_Comm_split(MPI_COMM_WORLD, tColor, tWorldCommRank, &tAppComm);

        // ************ TEST INITIALIZE ************
        std::shared_ptr<Plato::PlatoProxyApp> tPlatoApp;
        std::shared_ptr<Plato::PhysicsProxyApp> tPhysicsApp;
        std::string tMeshSpecs = "generated:1x1x" + std::to_string(tPhysicsCommSize);
        if(tColor == PlatoTest::color::PHYSICS)
        {
            int tPhysicsAppCommSize = -1;
            MPI_Comm_size(tAppComm, &tPhysicsAppCommSize);
            EXPECT_EQ(tPhysicsCommSize, tPhysicsAppCommSize);

            tPhysicsApp = std::make_shared<Plato::PhysicsProxyApp>(tMeshSpecs, tAppComm);
            tPhysicsApp->initialize();

            int tIntegerGold = tPhysicsAppCommSize;
            EXPECT_EQ(tIntegerGold, tPhysicsApp->getGlobalNumElements());
            tIntegerGold = 4 * (tPhysicsAppCommSize + 1);
            EXPECT_EQ(tIntegerGold, tPhysicsApp->getGlobalNumNodes());
        }
        else
        {
            int tPlatoAppCommSize = -1;
            MPI_Comm_size(tAppComm, &tPlatoAppCommSize);
            EXPECT_EQ(tPlatoCommSize, tPlatoAppCommSize);

            tPlatoApp = std::make_shared<Plato::PlatoProxyApp>(tMeshSpecs, tAppComm);
            tPlatoApp->initialize();
        }

        // ************ TEST FIELD DATA IMPORT/EXPORT WITH DESIGN VARIABLES ************
        Plato::CommunicationData com_data;
        com_data.mLocalComm = tAppComm;
        com_data.mInterComm = MPI_COMM_WORLD;
        if(tColor == PlatoTest::color::PHYSICS)
        {
            com_data.mLocalCommName = "PhyscisComm";
            auto layout = Plato::data::layout_t::SCALAR_FIELD;
            tPhysicsApp->exportDataMap(layout, com_data.mMyOwnedGlobalIDs[layout]);
        }
        else
        {
            com_data.mLocalCommName = "PlatoComm";
            auto layout = Plato::data::layout_t::SCALAR_FIELD;
            tPlatoApp->exportDataMap(layout, com_data.mMyOwnedGlobalIDs[layout]);
        }


        std::string tVariableName("DesignVariables");
        std::shared_ptr<Plato::SharedField> tDesignVariables;
        if(tColor == PlatoTest::color::PHYSICS)
        {
            tDesignVariables = std::make_shared<Plato::SharedField>(tVariableName, Plato::communication::broadcast_t::RECEIVER, 
                                                                    com_data, Plato::data::layout_t::SCALAR_FIELD);
        }
        else
        {
            tDesignVariables = std::make_shared<Plato::SharedField>(tVariableName, Plato::communication::broadcast_t::SENDER, 
                                                                    com_data, Plato::data::layout_t::SCALAR_FIELD);
        }
        EXPECT_EQ(Plato::data::layout_t::SCALAR_FIELD, tDesignVariables->myLayout());


        // ************ TEST COMPUTE DESIGN VARIABLES AND EXPORT FROM PLATO APP AND IMPORT DATA TO PHYSICS APP ************
        std::string tArgumentName("DesignVariables");
        if(tColor == PlatoTest::color::PLATO)
        {
            EXPECT_EQ(Plato::communication::broadcast_t::SENDER, tDesignVariables->myBroadcast());
            std::string tOperationName("UpdateDesignVariables");
            tPlatoApp->compute(tOperationName);
            tPlatoApp->exportData(tArgumentName, *tDesignVariables);
            tDesignVariables->transmitData();
        }
        else
        {
            EXPECT_EQ(Plato::communication::broadcast_t::RECEIVER, tDesignVariables->myBroadcast());
            tDesignVariables->transmitData();
            tPhysicsApp->importData(tArgumentName, *tDesignVariables);
        }

        // ************ TEST SUM OF DATA FROM SENDER AND RECEIVER ************
        int tMySize = tDesignVariables->size();
        std::vector<double> tMyData(tMySize, 0);
        tDesignVariables->getData(tMyData);

        double tInitialValue = 0;
        double tMySum = std::accumulate(tMyData.begin(), tMyData.end(), tInitialValue);
        EXPECT_TRUE(tMySum > static_cast<double>(0));

        double tSum = 0;
        MPI_Allreduce(&tMySum, &tSum, 1, MPI_DOUBLE, MPI_SUM, tAppComm);
        EXPECT_TRUE(tSum > static_cast<double>(0));

        double tMax = 0;
        const double tTolerance = 1e-6;
        double tMyMax = *std::max_element(tMyData.begin(), tMyData.end());
        MPI_Allreduce(&tMyMax, &tMax, 1, MPI_DOUBLE, MPI_MAX, tAppComm);
        double tScalarGold = tPlatoCommSize;
        EXPECT_NEAR(tScalarGold, tMax, tTolerance);

        // ************ TEST COMPUTE OBJECTIVE FUNCTION AND EXPORT FROM PHYSICS APP AND IMPORT TO PLATO APP ************
        tVariableName.clear();
        tVariableName.assign("ObjectiveValue");
        std::shared_ptr<Plato::SharedValue> tObjectiveValue;

        if(tColor == PlatoTest::color::PHYSICS)
        {
            std::vector<std::string> tOwnerNames = {"PhyscisComm"};
            tObjectiveValue = std::make_shared<Plato::SharedValue>(tVariableName, tOwnerNames, com_data);
        }
        else
        {
            std::vector<std::string> tOwnerNames = {"PlatoComm"};
            tObjectiveValue = std::make_shared<Plato::SharedValue>(tVariableName, tOwnerNames, com_data);
        }
        EXPECT_EQ(Plato::data::layout_t::SCALAR, tObjectiveValue->myLayout());

        int tIntegerGold = 1;
        EXPECT_EQ(tIntegerGold, tObjectiveValue->size());

        tArgumentName.clear();
        tArgumentName.assign("ObjectiveValue");
        if(tColor == PlatoTest::color::PHYSICS)
        {
            std::string tOperationName("ObjectiveFunctionEvaluation");
            tPhysicsApp->compute(tOperationName);
            tPhysicsApp->exportData(tArgumentName, *tObjectiveValue);
            tObjectiveValue->transmitData();
        }
        else
        {
            tObjectiveValue->transmitData();
            tPlatoApp->importData(tArgumentName, *tObjectiveValue);
        }

        tMySize = 1;
        std::vector<double> tObjectiveData(tMySize, 0);
        tObjectiveValue->getData(tObjectiveData);
        tScalarGold = tPhysicsCommSize;
        EXPECT_NEAR(tScalarGold, tObjectiveData[0], tTolerance);

        // ************ TEST FIELD DATA IMPORT/EXPORT WITH OBJECTIVE GRADIENT ************
        tVariableName.clear();
        tVariableName.assign("ObjectiveGradient");
        std::shared_ptr<Plato::SharedField> tObjectiveGradient;
        if(tColor == PlatoTest::color::PHYSICS)
        {
            tObjectiveGradient = std::make_shared<Plato::SharedField>(tVariableName, Plato::communication::broadcast_t::SENDER, 
                                                                      com_data, Plato::data::layout_t::SCALAR_FIELD);
        }
        else
        {
            tObjectiveGradient = std::make_shared<Plato::SharedField>(tVariableName, Plato::communication::broadcast_t::RECEIVER, 
                                                                      com_data, Plato::data::layout_t::SCALAR_FIELD);
        }
        EXPECT_EQ(Plato::data::layout_t::SCALAR_FIELD, tObjectiveGradient->myLayout());

        // ************ TEST COMPUTE OBJECTIVE GRADIENT AND EXPORT DATA FROM PHYSICS APP AND IMPORT DATA TO PLATO APP ************
        tArgumentName.clear();
        tArgumentName.assign("ObjectiveGradient");
        if(tColor == PlatoTest::color::PHYSICS)
        {
            EXPECT_EQ(Plato::communication::broadcast_t::SENDER, tObjectiveGradient->myBroadcast());
            std::string tOperationName("ComputeObjectiveGradient");
            tPhysicsApp->compute(tOperationName);
            tPhysicsApp->exportData(tArgumentName, *tObjectiveGradient);
            tObjectiveGradient->transmitData();
        }
        else
        {
            EXPECT_EQ(Plato::communication::broadcast_t::RECEIVER, tObjectiveGradient->myBroadcast());
            tObjectiveGradient->transmitData();
            tPlatoApp->importData(tArgumentName, *tObjectiveGradient);
        }

        // ************ TEST GRADIENT DATA FROM RECEIVED BY PLATO FROM PHYSICS (SENDER) ************
        tMySize = tObjectiveGradient->size();
        std::fill(tMyData.begin(), tMyData.end(), 0.);
        tObjectiveGradient->getData(tMyData);

        tMySum = std::accumulate(tMyData.begin(), tMyData.end(), tInitialValue);
        EXPECT_TRUE(tMySum > static_cast<double>(0));

        tSum = 0;
        MPI_Allreduce(&tMySum, &tSum, 1, MPI_DOUBLE, MPI_SUM, tAppComm);
        EXPECT_TRUE(tSum > static_cast<double>(0));

        tMax = 0;
        tMyMax = *std::max_element(tMyData.begin(), tMyData.end());
        MPI_Allreduce(&tMyMax, &tMax, 1, MPI_DOUBLE, MPI_MAX, tAppComm);
        tScalarGold = tPhysicsCommSize;
        EXPECT_NEAR(tScalarGold, tMax, tTolerance);
    }
}

TEST(PlatoTest, ProxyPerformer)
{
    int tWorldCommSize = -1;
    MPI_Comm_size(MPI_COMM_WORLD, &tWorldCommSize);

    const int tPlatoCommSize = 2;
    const int tPhysicsCommSize = tWorldCommSize - tPlatoCommSize;

    const int tMinWorldCommSize = 3;
    if(tWorldCommSize > tMinWorldCommSize)
    {
        MPI_Comm tAppComm;
        int tWorldCommRank = -1;
        MPI_Comm_rank(MPI_COMM_WORLD, &tWorldCommRank);
        PlatoTest::color::application_t tColor = PlatoTest::color::UNDEFINED;
        if(tWorldCommRank < tPlatoCommSize)
        {
            tColor = PlatoTest::color::PLATO;
        }
        else
        {
            tColor = PlatoTest::color::PHYSICS;
        }
        // Split the communicator based on the color and use the original rank for ordering
        MPI_Comm_split(MPI_COMM_WORLD, tColor, tWorldCommRank, &tAppComm);

        // ************ REGISTER PERFORMER ************
        std::shared_ptr<Plato::Performer> tPerformer;
        std::shared_ptr<Plato::PlatoProxyApp> tPlatoApp;
        std::shared_ptr<Plato::PhysicsProxyApp> tPhysicsApp;
        std::string tMeshSpecs = "generated:1x1x" + std::to_string(tPhysicsCommSize);
        if(tColor == PlatoTest::color::PHYSICS)
        {
            int tPhysicsAppCommSize = -1;
            MPI_Comm_size(tAppComm, &tPhysicsAppCommSize);
            EXPECT_EQ(tPhysicsCommSize, tPhysicsAppCommSize);

            tPhysicsApp = std::make_shared<Plato::PhysicsProxyApp>(tMeshSpecs, tAppComm);
            tPhysicsApp->initialize();

            std::string tName("PhysicsApp");
            tPerformer = std::make_shared<Plato::Performer>(tName, tColor);
            tPerformer->setApplication(tPhysicsApp.get());
        }
        else
        {
            int tPlatoAppCommSize = -1;
            MPI_Comm_size(tAppComm, &tPlatoAppCommSize);
            EXPECT_EQ(tPlatoCommSize, tPlatoAppCommSize);

            tPlatoApp = std::make_shared<Plato::PlatoProxyApp>(tMeshSpecs, tAppComm);
            tPlatoApp->initialize();

            std::string tName("PlatoApp");
            tPerformer = std::make_shared<Plato::Performer>(tName, tColor);
            tPerformer->setApplication(tPlatoApp.get());
            int OrdinalGold = 0;
            EXPECT_EQ(OrdinalGold, tPerformer->myCommID());
            EXPECT_STREQ(tName.c_str(), tPerformer->myName().c_str());
        }

        // ************ INITIALIZE COMMUNICATION DATA INSTANCE ************
        Plato::CommunicationData com_data;
        com_data.mLocalComm = tAppComm;
        com_data.mInterComm = MPI_COMM_WORLD;
        if(tColor == PlatoTest::color::PHYSICS)
        {
            com_data.mLocalCommName = "PhyscisComm";
            auto layout = Plato::data::layout_t::SCALAR_FIELD;
            tPhysicsApp->exportDataMap(layout, com_data.mMyOwnedGlobalIDs[layout]);
        }
        else
        {
            com_data.mLocalCommName = "PlatoComm";
            auto layout = Plato::data::layout_t::SCALAR_FIELD;
            tPlatoApp->exportDataMap(layout, com_data.mMyOwnedGlobalIDs[layout]);
        }

        // ************ TEST COMPUTE DESIGN VARIABLES AND EXPORT FROM PLATO APP AND IMPORT DATA TO PHYSICS APP ************
        std::string tVariableName("DesignVariables");
        std::shared_ptr<Plato::SharedField> tDesignVariables;
        if(tColor == PlatoTest::color::PHYSICS)
        {
            tDesignVariables = std::make_shared<Plato::SharedField>(tVariableName, Plato::communication::broadcast_t::RECEIVER, 
                                                                    com_data, Plato::data::layout_t::SCALAR_FIELD);
        }
        else
        {
            tDesignVariables = std::make_shared<Plato::SharedField>(tVariableName, Plato::communication::broadcast_t::SENDER, 
                                                                    com_data, Plato::data::layout_t::SCALAR_FIELD);
        }
        EXPECT_EQ(Plato::data::layout_t::SCALAR_FIELD, tDesignVariables->myLayout());

        std::string tArgumentName("DesignVariables");
        if(tColor == PlatoTest::color::PLATO)
        {
            EXPECT_EQ(Plato::communication::broadcast_t::SENDER, tDesignVariables->myBroadcast());
            std::string tOperationName("UpdateDesignVariables");
            tPerformer->compute(tOperationName);
            tPerformer->exportData(tArgumentName, *tDesignVariables);
            tDesignVariables->transmitData();
        }
        else
        {
            EXPECT_EQ(Plato::communication::broadcast_t::RECEIVER, tDesignVariables->myBroadcast());
            tDesignVariables->transmitData();
            tPerformer->importData(tArgumentName, *tDesignVariables);
        }

        // ************ TEST SUM OF DATA FROM SENDER AND RECEIVER ************
        int tMySize = tDesignVariables->size();
        std::vector<double> tMyData(tMySize, 0);
        tDesignVariables->getData(tMyData);

        double tInitialValue = 0;
        double tMySum = std::accumulate(tMyData.begin(), tMyData.end(), tInitialValue);
        EXPECT_TRUE(tMySum > static_cast<double>(0));

        double tSum = 0;
        MPI_Allreduce(&tMySum, &tSum, 1, MPI_DOUBLE, MPI_SUM, tAppComm);
        EXPECT_TRUE(tSum > static_cast<double>(0));

        double tMax = 0;
        const double tTolerance = 1e-6;
        double tMyMax = *std::max_element(tMyData.begin(), tMyData.end());
        MPI_Allreduce(&tMyMax, &tMax, 1, MPI_DOUBLE, MPI_MAX, tAppComm);
        double tScalarGold = tPlatoCommSize;
        EXPECT_NEAR(tScalarGold, tMax, tTolerance);
    }
}

}

#endif // STK_ENABLED
