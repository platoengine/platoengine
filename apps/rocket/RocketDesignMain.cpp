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
 * RocketDesignMain.cpp
 *
 *  Created on: Sep 27, 2018
 */

#include "Plato_Interface.hpp"
#include "Plato_SimpleRocket.hpp"
#include "Plato_RocketDesignApp.hpp"

#ifndef NDEBUG
#include <fenv.h>
#endif

namespace Plato
{

class RocketDesignApp : public Plato::Application
{
public:
    /******************************************************************************//**
     * @brief Constructor
     **********************************************************************************/
    RocketDesignApp() :
            mNumDesigVariables(2),
            mRocketSim(),
            mDataMap()
    {
    }

    /******************************************************************************//**
     * @brief Constructor
     **********************************************************************************/
    RocketDesignApp(int aArgc, char **aArgv) :
            mNumDesigVariables(2),
            mRocketSim(),
            mDataMap()
    {
    }

    /******************************************************************************//**
     * @brief Destructor
     **********************************************************************************/
    virtual ~RocketDesignApp()
    {
    }

    /******************************************************************************//**
     * @brief Deallocate memory
     **********************************************************************************/
    void finalize()
    {
    }

    /******************************************************************************//**
     * @brief Allocate memory
     **********************************************************************************/
    void initialize()
    {
        this->makeParameterMap();
    }

    /******************************************************************************//**
     * @brief Perform an operation, e.g. evaluate objective function
     * @param [in] aOperationName name of operation
     **********************************************************************************/
    void compute(const std::string & aOperationName)
    {
        if(aOperationName.compare("Solve") == static_cast<int>(0))
        {
            mRocketSim.solve();
        }
        else if(aOperationName.compare("Objective") == static_cast<int>(0))
        {
            this->evaluateObjective();
        }
        else if(aOperationName.compare("ObjectiveGradient") == static_cast<int>(0))
        {
            this->computeObjectiveGradient();
        }
        else
        {
            std::ostringstream tErrorMsg;
            tErrorMsg << "\n\n*********\n ERROR IN " << __FILE__ << "\n FUNCTION: " << __PRETTY_FUNCTION__ << "\n LINE: "
                      << __LINE__ << "\n MESSAGE: OPERATION NAME = '" << aOperationName.c_str()
                      << "' IS NOT DEFINE. ABORT!\n *********\n\n";
            std::perror(tErrorMsg.str().c_str());
            std::abort();
        }
    }

    /******************************************************************************//**
     * @brief Export data from user's application
     * @param [in] aArgumentName name of export data (e.g. objective gradient)
     * @param [out] aExportData container used to store output data
     **********************************************************************************/
    void exportData(const std::string & aArgumentName, Plato::SharedData & aExportData)
    {
    }

    /******************************************************************************//**
     * @brief Import data from Plato to user's application
     * @param [in] aArgumentName name of import data (e.g. design variables)
     * @param [in] aImportData container with import data
     **********************************************************************************/
    void importData(const std::string & aArgumentName, const Plato::SharedData & aImportData)
    {
    }

    /******************************************************************************//**
     * @brief Export distributed memory graph
     * @param [in] aDataLayout data layout (options: SCALAR, SCALAR_FIELD, VECTOR_FIELD, TENSOR_FIELD, ELEMENT_FIELD, SCALAR_PARAMETER)
     * @param [out] aMyOwnedGlobalIDs my processor's global IDs
     **********************************************************************************/
    void exportDataMap(const Plato::data::layout_t & aDataLayout, std::vector<int> & aMyOwnedGlobalIDs)
    {
    }

private:
    void makeParameterMap()
    {
        const int tLength = 1;
        std::string tName = "ThrustMisfitObjective";
        mDataMap[tName] = std::vector<double>(tLength);

        tName = "DesignVariables";
        mDataMap[tName] = std::vector<double>(mNumDesigVariables);

        tName = "ThrustMisfitObjectiveGradient";
        mDataMap[tName] = std::vector<double>(mNumDesigVariables);
    }

    void evaluateObjective()
    {
        std::string tArgumentName("DesignVariables");
        auto tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        Plato::StandardMultiVector<double> tControl(1 /* NUM CONTROL VECTORS */, mNumDesigVariables);
        std::vector<double> & tControlVector = tIterator->second;
        tControl.copy(0 /* VECTOR INDEX */, tControlVector);
        double tObjectiveValue = mObjective.value(tControl);

        tArgumentName = "ThrustMisfitObjective";
        tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        const int tIndex = 0;
        tIterator->second.operator[](tIndex) = tObjectiveValue;
    }

    void computeObjectiveGradient()
    {
        std::string tArgumentName("DesignVariables");
        auto tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        std::vector<double> & tControlVector = tIterator->second;
        Plato::StandardMultiVector<double> tControl(1 /* NUM CONTROL VECTORS */, tControlVector);

        tArgumentName = "ThrustMisfitObjectiveGradient";
        tIterator = mDataMap.find(tArgumentName);
        assert(tIterator != mDataMap.end());
        Plato::StandardMultiVector<double> tGradient(1 /* NUM CONTROL VECTORS */, mNumDesigVariables);
        mObjective.gradient(tControl, tGradient);
        std::vector<double> & tGradientVector = tIterator->second;
        assert(tGradientVector.size() == mNumDesigVariables);
        tGradient.copy(0 /* VECTOR INDEX */, tGradientVector);
    }

private:
    size_t mNumDesigVariables; /*!< import/export parameter map */
    Plato::SimpleRocket<double> mRocketSim; /*!< rocket simulation interface */
    Plato::SimpleRocketObjective<double> mObjective; /*!< rocket design problem objective */
    std::map<std::string, std::vector<double>> mDataMap; /*!< import/export parameter map */

private:
    RocketDesignApp(const Plato::RocketDesignApp & aRhs);
    Plato::RocketDesignApp & operator=(const Plato::RocketDesignApp & aRhs);
};

}

/******************************************************************************/
int main(int aArgc, char **aArgv)
/******************************************************************************/
{
#ifndef NDEBUG
    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif

    MPI_Init(&aArgc, &aArgv);

    /************************* CREATE PLATO INTERFACE *************************/
    Plato::Interface* tPlatoInterface = nullptr;
    try
    {
        tPlatoInterface = new Plato::Interface();
    }
    catch(...)
    {
        MPI_Finalize();
        exit(0);
    }
    /************************* CREATE PLATO INTERFACE *************************/

    /*************************** SET PLATO INTERFACE **************************/
    MPI_Comm tLocalComm;
    tPlatoInterface->getLocalComm(tLocalComm);
    /*************************** SET PLATO INTERFACE **************************/

    /************************ CREATE LOCAL APPLICATION ************************/
    Plato::StructuralTopologyOptimizationProxyApp* tMyApp = nullptr;
    try
    {
        tMyApp = new Plato::StructuralTopologyOptimizationProxyApp(aArgc, aArgv);
    }
    catch(...)
    {
        MPI_Finalize();
        exit(0);
    }
    /************************ CREATE LOCAL APPLICATION ************************/

    /************************** REGISTER APPLICATION **************************/
    try
    {
        tPlatoInterface->registerPerformer(tMyApp);
    }
    catch(...)
    {
        MPI_Finalize();
        exit(0);
    }
    /************************** REGISTER APPLICATION **************************/

    /******************************** PERFORM *********************************/
    try
    {
        tPlatoInterface->perform();
    }
    catch(...)
    {
    }
    /******************************** PERFORM *********************************/

    delete tMyApp;

    MPI_Finalize();
}
// main

