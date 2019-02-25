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
 * Plato_Test_XTK_Interface.cpp
 *
 *  Created on: Jan 22, 2019
 *      Author: ktdoble
 */
#include <gtest/gtest.h>
#include <typedefs.hpp>
#include "Plato_XTK_Application.hpp"
#include "Plato_XTK_SharedData.hpp"


namespace PlatoTestXTK
{
  TEST(PlatoTestXTK, XTK)
  {
      std::string tMeshFileName   = "generated:2x2x2";
      std::string tTopoFieldName  = "topo_0";
      std::string tOutputMeshFile = "./plato_xtk_test.exo";

      // Initialize the application
      Plato::XTKApplication tXTKInterface(MPI_COMM_WORLD,
                                          tMeshFileName,
                                          tOutputMeshFile);

      // Level set field data
      std::vector<double> tLevelSetVals = { -1.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

      // Import the level set Field
      int tNumNodes = 27;
      Plato::XTKSharedData tTopoValsIn(tNumNodes,
                                       tTopoFieldName,
                                       Plato::data::SCALAR_FIELD);

      tTopoValsIn.setData(tLevelSetVals);
      tXTKInterface.importData(tTopoFieldName, tTopoValsIn );

      // Decompose
      tXTKInterface.compute("xtk_decompose");

      // Compute sensitivity
      tXTKInterface.compute("xtk_compute_sensitivity");

      // Output to mtk mesh
      tXTKInterface.compute("xtk_output_mesh");

      // Export to exodus file
      tXTKInterface.compute("xtk_export_mesh");

      // Export the level set field
      int tNumNodesAfter = 51;
      Plato::XTKSharedData tTopoValOut( tNumNodesAfter,
                                        tTopoFieldName,
                                        Plato::data::SCALAR_FIELD);

      tXTKInterface.exportData("topo_0", tTopoValOut);

      // TODO: verify the data
      std::vector<double> tData(tNumNodesAfter);
      tTopoValOut.getData(tData);


      // Export the number of design variables a node depends on
      Plato::XTKSharedData tNumDesVarsdxdp( tNumNodesAfter,
                                            "NumDesVar",
                                            Plato::data::SCALAR_FIELD);

      tXTKInterface.exportData("NumDesVar", tNumDesVarsdxdp);


      // Export the first design variables id depends on
      Plato::XTKSharedData tDesVarsdxdp0( tNumNodesAfter,
                                          "DesVar0",
                                          Plato::data::SCALAR_FIELD);

      tXTKInterface.exportData("DesVar0", tDesVarsdxdp0);


      // Export the second design variables id depends on
      Plato::XTKSharedData tDesVarsdxdp1( tNumNodesAfter,
                                          "DesVar1",
                                          Plato::data::SCALAR_FIELD);

      tXTKInterface.exportData("DesVar1", tDesVarsdxdp1);


      // Export the nodes change in x position relative to design var 0
      Plato::XTKSharedData tdxdp0x( tNumNodesAfter,
                                    "dx1dp0",
                                    Plato::data::SCALAR_FIELD);

      tXTKInterface.exportData("dx1dp0", tdxdp0x);


      // Export the nodes change in y position relative to design var 0
      Plato::XTKSharedData tdxdp0y( tNumNodesAfter,
                                    "dx2dp0",
                                    Plato::data::SCALAR_FIELD);

      tXTKInterface.exportData("dx2dp0", tdxdp0y);

      // Export the nodes change in z position relative to design var 0
      Plato::XTKSharedData tdxdp0z( tNumNodesAfter,
                                    "dx3dp0",
                                    Plato::data::SCALAR_FIELD);

      tXTKInterface.exportData("dx3dp0", tdxdp0z);

      // Export the nodes change in x position relative to design var 1
      Plato::XTKSharedData tdxdp1x( tNumNodesAfter,
                                    "dx1dp1",
                                    Plato::data::SCALAR_FIELD);

      tXTKInterface.exportData("dx1dp1", tdxdp1x);


      // Export the nodes change in y position relative to design var 1
      Plato::XTKSharedData tdxdp1y( tNumNodesAfter,
                                    "dx2dp1",
                                    Plato::data::SCALAR_FIELD);

      tXTKInterface.exportData("dx2dp1", tdxdp1y);

      // Export the nodes change in z position relative to design var 1
      Plato::XTKSharedData tdxdp1z( tNumNodesAfter,
                                    "dx3dp1",
                                    Plato::data::SCALAR_FIELD);

      tXTKInterface.exportData("dx3dp1", tdxdp1z);



//      // Export node map
//      //TODO: ADD gold node map
//      std::vector<int> tNodeMap;
//      tXTKInterface.exportDataMap(Plato::data::SCALAR_FIELD,tNodeMap);





  }
}

