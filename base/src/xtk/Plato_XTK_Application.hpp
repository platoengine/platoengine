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

#ifndef BASE_SRC_XTK_PLATO_XTK_APPLICATION_HPP_
#define BASE_SRC_XTK_PLATO_XTK_APPLICATION_HPP_

// Includes from XTK
#include <cl_XTK_Model.hpp>
#include <cl_MGE_Geometry_Engine.hpp>
#include <cl_MTK_Mesh.hpp>
#include <cl_XTK_Model.hpp>
#include <cl_XTK_Enums.hpp>
#include <cl_Sphere.hpp>
#include <cl_MGE_Geometry_Engine.hpp>
#include <cl_Discrete_Level_Set.hpp>

#include "cl_Logger.hpp"
#include "cl_Communication_Manager.hpp"

#include "Plato_SharedData.hpp"

// create communicator and logger
//moris::Comm_Manager gMorisComm;
//moris::Logger       gLogger;


namespace Plato
{


/******************************************************************************/
class XTKApplication
/******************************************************************************/
{
public:
    /*!
     * @param[in] aCommunicator   - MPI communicator to use in XTK
     * @param[in] aInputMeshFile  - Mesh file to use for background mesh
     * @param[in] aFieldName      - Node field on background mesh to refine against
     * @param[in] aOutputMeshFile - Name of output mesh file
     */
    XTKApplication(MPI_Comm    aCommunicator,
                   std::string aInputMeshFile,
                   std::string aOutputMeshFile = std::string());

    /*!
     * Deconstructor that deletes meshes created via factor
     */
    ~XTKApplication();


    //TODO: Parse input parameters
    void
    initialize(){};


    /*!
     * @brief Perform XTK operations
     * @param aOperationName - Operation name
     *  Valid operations:
     *   - "xtk_decompose"       - perform xtk decomposition
     *   - "xtk_output_mesh"     - output to an mtk mesh
     *   - "xtk_export_mesh"     - export to an exodus file
     *   - "compute_sensitivity" - compute sensitivity
     *
     *
     */
    void compute(const std::string & aOperationName);


    /*!
     * @brief Export Data from XTK.
     * Valid: aArgumentName:
     *  - "topo_0" - export the topology field
     *  - "dx1dp0" - Shape sensitivity in x direction to design variable 0
     *  - "dx2dp0" - Shape sensitivity in y direction to design variable 0
     *  - "dx3dp0" - Shape sensitivity in z direction to design variable 0
     *  - "dx1dp1" - Shape sensitivity in x direction to design variable 1
     *  - "dx2dp1" - Shape sensitivity in y direction to design variable 1
     *  - "dx3dp1" - Shape sensitivity in z direction to design variable 1
     *  - "NumDesVar" - Number of design variables a  node depends on (should be 0 or 2)
     *  - "DesVar0"   - Design variable 0
     *  - "DesVar1"   - Design variable 1
     *
     * @param aArgumentName - Data name to export
     * @param aExportData   - Exported data
     */
    void
    exportData(const std::string & aArgumentName,
               Plato::SharedData & aExportData);

    /*!
     * Import data (specifically import the design field)
     * Valid argument names:
     * - topo_0 - import the topology field (NOTE!!! this must be called prior to any XTK related computations)
     * @param aArgumentName
     * @param aImportData
     */
    void
    importData(const std::string       & aArgumentName,
               const Plato::SharedData & aImportData);

    /*!
     *
     * @param aDataLayout
     * @param aMyOwnedGlobalIDs
     */
    void
    exportDataMap(const Plato::data::layout_t & aDataLayout,
                  std::vector<int>            & aMyOwnedGlobalIDs);





private:
    xtk::Model              mXTKModel;
    std::string             mOutputMeshFile;
    moris::mtk::Mesh*       mXTKOutputMesh;
    xtk::Discrete_Level_Set mGeometry; /*Keeps the geometry in scope for the geometry engine*/
    moris::mtk::Mesh*       mMTKBackgroundMesh;


    void
    get_dxdp_data(moris::uint aSpatialComp,
                  moris::uint aDesVarInd,
                  std::vector<double> & adxdp);

    void
    get_dxdp_des_var_ids( moris::uint aDesVarInd,
                         std::vector<double> & adxdpIds);

};
}

#endif /* BASE_SRC_XTK_PLATO_XTK_APPLICATION_HPP_ */
