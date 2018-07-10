/*
 * Plato_Main.cpp
 *
 *  Created on: Nov 21, 2016
 *
 */

#include "communicator.hpp"

#include "PlatoApp.hpp"
#include "Plato_Interface.hpp"
#include "Plato_OptimizerInterface.hpp"
#include "Plato_OptimizerFactory.hpp"

#ifndef NDEBUG
#include <fenv.h>
#endif

/******************************************************************************/
int main(int aArgc, char *aArgv[])
/******************************************************************************/
{
#ifndef NDEBUG
    feclearexcept(FE_ALL_EXCEPT);
    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif

    MPI_Init(&aArgc, (char***) &aArgv);

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

    MPI_Comm tLocalComm;
    tPlatoInterface->getLocalComm(tLocalComm);
    WorldComm.init(tLocalComm);

    // Create Plato services application and register it with the Plato interface
    PlatoApp *tPlatoApp = nullptr;
    if(aArgc > static_cast<int>(1))
    {
        tPlatoApp = new PlatoApp(aArgc, aArgv, tLocalComm);
    }
    else
    {
        tPlatoApp = new PlatoApp(tLocalComm);
    }

    Plato::OptimizerInterface<double>* tOptimizer = nullptr;
    try
    {
        tPlatoInterface->registerPerformer(tPlatoApp);
    }
    catch(...)
    {
        MPI_Finalize();
        exit(0);
    }

    try
    {
        Plato::OptimizerFactory<double> tOptimizerFactory;
        tOptimizer = tOptimizerFactory.create(tPlatoInterface, tLocalComm);
        if(tOptimizer)
        {
            tOptimizer->optimize();
        }
        else
        {
            tPlatoInterface->handleExceptions();
        }
    }
    catch(...)
    {
        MPI_Finalize();
        exit(0);
    }

    if(tPlatoApp)
    {
        delete tPlatoApp;
    }
    if(tPlatoInterface)
    {
        delete tPlatoInterface;
    }
    if(tOptimizer)
    {
        delete tOptimizer;
    }

    MPI_Finalize();
    exit(0);
}


