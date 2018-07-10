/*
 * StructuralTopologyOptimizationProxyMain.cpp
 *
 *  Created on: Mar 1, 2018
 */

#include "Plato_Interface.hpp"
#include "Plato_StructuralTopologyOptimizationProxyApp.hpp"

#ifndef NDEBUG
#include <fenv.h>
#endif

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
