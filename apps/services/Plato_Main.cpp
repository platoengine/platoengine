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
 * Plato_Main.cpp
 *
 *  Created on: Nov 21, 2016
 *
 */

#include "communicator.hpp"

#ifdef GEOMETRY
#include <Kokkos_Core.hpp>
#endif

#include "PlatoApp.hpp"
#include "Plato_Interface.hpp"
#include "Plato_OptimizerInterface.hpp"
#include "Plato_OptimizerFactory.hpp"
#include "Plato_Exceptions.hpp"
#include <iostream>
#include <sstream>

#ifndef NDEBUG
#include <fenv.h>
#endif

void safeExit(){
#if defined(GEOMETRY) || defined(AMFILTER_ENABLED)
    Kokkos::finalize();
#endif
    MPI_Finalize();
    exit(0);
}

void writeSplashScreen();

/******************************************************************************/
int main(int aArgc, char *aArgv[])
/******************************************************************************/
{
#ifndef NDEBUG
    feclearexcept(FE_ALL_EXCEPT);
    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif

    MPI_Init(&aArgc, (char***) &aArgv);

#if defined(GEOMETRY) || defined(AMFILTER_ENABLED)
    Kokkos::initialize(aArgc, aArgv);
#endif

    Plato::Interface* tPlatoInterface = nullptr;
    try
    {
        tPlatoInterface = new Plato::Interface();
    }
    catch(Plato::ParsingException& e)
    {
        std::cout << e.message() << std::endl;
        safeExit();
    }
    catch(...)
    {
        safeExit();
    }

    MPI_Comm tLocalComm;
    tPlatoInterface->getLocalComm(tLocalComm);
    WorldComm.init(tLocalComm);

    // Create Plato services application and register it with the Plato interface
    PlatoApp *tPlatoApp = nullptr;
    try
    {
        if(aArgc > static_cast<int>(1))
        {
            tPlatoApp = new PlatoApp(aArgc, aArgv, tLocalComm);
        }
        else
        {
            tPlatoApp = new PlatoApp(tLocalComm);
        }
    }
    catch(...)
    {
        tPlatoApp = nullptr;
        tPlatoInterface->Catch();
    }

    try
    {
        tPlatoInterface->registerPerformer(tPlatoApp);
    }
    catch(...)
    {
        safeExit();
    }

    writeSplashScreen();

    Plato::OptimizerInterface<double>* tOptimizer = nullptr;
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
        safeExit();
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

    safeExit();
}


void writeSplashScreen()
{

  std::stringstream splash;
  splash << "################################################################################" << std::endl;
  splash << "#                                                                              #" << std::endl;
  splash << "#                    @@@@@                                                     #" << std::endl;
  splash << "#                    @@@@@                                                     #" << std::endl;
  splash << "#                    @@@@@                                                     #" << std::endl;
  splash << "#                    @@@@@                       ,*****                        #" << std::endl;
  splash << "#                    @@@@@                       %@@@@@                        #" << std::endl;
  splash << "#                    @@@@@                       %@@@@@                        #" << std::endl;
  splash << "#                    @@@@@                       %@@@@@                        #" << std::endl;
  splash << "#  @@@@@@@@@@@@@@@   @@@@@    @@@@@@@@@@@@@@   @@@@@@@@@@@   @@@@@@@@@@@@@@@,  #" << std::endl;
  splash << "#  @@@@@@@@@@@@@@@   @@@@@   ,@@@@@@@@@@@@@@   @@@@@@@@@@@   @@@@@@@@@@@@@@@@  #" << std::endl;
  splash << "#  @@@@@     @@@@@   @@@@@   ,@@@@     @@@@@     %@@@@@      @@@@@.    %@@@@@  #" << std::endl;
  splash << "#  @@@@@     @@@@@   @@@@@             @@@@@     %@@@@@      @@@@@.    %@@@@@  #" << std::endl;
  splash << "#  @@@@@     @@@@@   @@@@@    *@@@@@@@@@@@@@     %@@@@@      @@@@@.    %@@@@@  #" << std::endl;
  splash << "#  @@@@@     @@@@@   @@@@@   @@@@@@@@@@@@@@@     %@@@@@      @@@@@.    %@@@@@  #" << std::endl;
  splash << "#  @@@@@     @@@@@   @@@@@   @@@@@     @@@@@     %@@@@@      @@@@@.    %@@@@@  #" << std::endl;
  splash << "#  @@@@@     @@@@@   @@@@@   @@@@@     @@@@@     %@@@@@      @@@@@.    %@@@@@  #" << std::endl;
  splash << "#  @@@@@     @@@@@   @@@@@   @@@@@     @@@@@     %@@@@@      @@@@@.    %@@@@@  #" << std::endl;
  splash << "#  @@@@@@@@@@@@@@@   @@@@@   @@@@@@@@@@@@@@@     %@@@@@@@@   @@@@@@@@@@@@@@@@  #" << std::endl;
  splash << "#  @@@@@@@@@@@@@@#   @@@@@   &@@@@@@@@@@@@@@      @@@@@@@@   @@@@@@@@@@@@@@@*  #" << std::endl;
  splash << "#  @@@@@                                                                       #" << std::endl;
  splash << "#  @@@@@                                                                       #" << std::endl;
  splash << "#  @@@@@   OPTIMIZATION BASED DESIGN                                           #" << std::endl;
  splash << "#  @@@@@                                                                       #" << std::endl;
  splash << "#  @@@@@   Questions? contact Plato3d-help@sandia.gov                          #" << std::endl;
  splash << "#                                                                              #" << std::endl;
  splash << "################################################################################" << std::endl;
  Plato::Console::Alert(splash.str());
}
