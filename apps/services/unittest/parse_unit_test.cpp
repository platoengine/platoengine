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
 * parse_unit_test.cpp
 *
 *  Created on: Aug 1, 2017
 */

#include <gtest/gtest.h>                // for AssertHelper, TEST, etc

#include "PlatoApp.hpp"
#include "Plato_Interface.hpp"
#include "ParseUnitTestStrings.hpp"

namespace ParseUnitTest
{

TEST(DISABLED_ParseTest, test1)
{
    const std::string interface_xml_string = 
        InterfaceHeader +
        InterfacePerf0Block +
        InterfacePerf1Block +
        InterfacePerf2Block +
        InterfaceInternalEnergy1GradientSharedDataBlock +
        InterfaceInternalEnergy2GradientSharedDataBlock +
        InterfaceInternalEnergyGradientSharedDataBlock +
        InterfaceVolumeGradientSharedDataBlock +
        InterfaceOptimizationDOFsSharedDataBlock +
        InterfaceTopologySharedDataBlock +
        InterfaceVolumeSharedDataBlock +
        InterfaceDesignVolumeSharedDataBlock +
        InterfaceInternalEnergy1SharedDataBlock +
        InterfaceInternalEnergy2SharedDataBlock +
        InterfaceInternalEnergySharedDataBlock +
        InterfaceOutputToFileStageBlock +
        InterfaceInitializeOptimizationDOFsStageBlock +
        InterfaceFilterControlStageBlock +
        InterfaceDesignVolumeStageBlock +
        InterfaceVolumeStageBlock +
        InterfaceInternalEnergyStageBlock +
        InterfaceFilterBlock +
        InterfaceOptimizerBlock +
        InterfaceMeshBlock +
        InterfaceOutputBlock; 
    const std::string plato_main_xml_string = 
        PlatoMainMeshBlock +
        PlatoMainOutputBlock;
    const std::string plato_app_xml_string = 
        PlatoAppHeader +
        PlatoAppPlatoMainOutputOperationBlock +
        PlatoAppFilterControlOperationBlock +
        PlatoAppFilterObjectiveGradientOperationBlock +
        PlatoAppFilterConstraintGradientOperationBlock +
        PlatoAppInitializeFieldOperationBlock +
        PlatoAppDesignVolumeOperationBlock +
        PlatoAppComputeVolumeOperationBlock +
        PlatoAppAggregatorOperationBlock;
        

    Plato::Interface *pi = new Plato::Interface(0, interface_xml_string);
    MPI_Comm tLocalComm;
    pi->getLocalComm(tLocalComm);
    PlatoApp* pa = new PlatoApp(plato_main_xml_string, plato_app_xml_string, tLocalComm);
    pi->registerPerformer(pa);

    // delete memory
    delete pa;
    delete pi;
}

}
