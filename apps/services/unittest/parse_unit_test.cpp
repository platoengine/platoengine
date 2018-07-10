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
