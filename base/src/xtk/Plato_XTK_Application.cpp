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
 * XTK_Interface.cpp
 *
 *  Created on: Jan 22, 2019
 *      Author: ktdoble
 */

#include "Plato_XTK_Application.hpp"
#include "Plato_SharedField.hpp"

#include "cl_MTK_Mesh_Data_Input.hpp"
#include "cl_MTK_Scalar_Field_Info.hpp"
#include "cl_MGE_Geometry_Object.hpp"
#include "cl_XTK_Background_Mesh.hpp"

// MORIS Globals to control communication and logging
#include "cl_Logger.hpp" // MRS/IOS/src
#include "cl_Communication_Manager.hpp" // COM/src
moris::Comm_Manager gMorisComm;
moris::Logger       gLogger;

namespace Plato
{
//-------------------------------------------------------------------------------------
XTKApplication::XTKApplication( MPI_Comm    aCommunicator,
                                std::string aInputMeshFile,
                                std::string aOutputMeshFile)
        {

    gMorisComm = moris::Comm_Manager(aCommunicator);

    moris::sint aSeverityLevel = 1;
    gLogger.initialize(aSeverityLevel);


    // Create Mesh --------------------------------------------------------------------
    // Declare scalar node field
    moris::mtk::Scalar_Field_Info<moris::DDRMat> tTopoField;
    std::string tFieldName1 = "topo_0";
    tTopoField.set_field_name(tFieldName1);
    tTopoField.set_field_entity_rank(EntityRank::NODE);

    // Initialize field information container
    moris::mtk::MtkFieldsInfo tFieldsInfo;

    // Place the node field into the field info container
    add_field_for_mesh_input(&tTopoField,tFieldsInfo);

    // Declare some supplementary fields
    moris::mtk::MtkMeshData tSupplemMeshData;
    tSupplemMeshData.FieldsInfo = &tFieldsInfo;

    mMTKBackgroundMesh = moris::mtk::create_mesh( MeshType::STK, aInputMeshFile, &tSupplemMeshData );

    // TODO: replace with discrete field
    // Create geometry field
    mGeometry = xtk::Discrete_Level_Set( mMTKBackgroundMesh, {tFieldName1});


    // Create phase table
    xtk::Phase_Table tPhaseTable (1,  Phase_Table_Structure::EXP_BASE_2);

    // Create geometry engine
    xtk::Geometry_Engine tGeometryEngine(mGeometry,tPhaseTable);

    // Setup XTK Model ----------------------------------------------------------------
    size_t tModelDimension = 3;
    mXTKModel = xtk::Model(tModelDimension,mMTKBackgroundMesh,tGeometryEngine,false);
    mXTKModel.mVerbose = true;


    mOutputMeshFile = aOutputMeshFile;
}

//-------------------------------------------------------------------------------------

XTKApplication::~XTKApplication()
{
    delete mMTKBackgroundMesh;

    if(mXTKOutputMesh != nullptr)
    {
        delete mXTKOutputMesh;
    }
}



//-------------------------------------------------------------------------------------
void
XTKApplication::compute(const std::string & aOperationName)
{
    if(aOperationName.compare("xtk_decompose") == 0)
    {
        moris::Cell<enum Subdivision_Method> tDecompositionMethods = {Subdivision_Method::NC_REGULAR_SUBDIVISION_HEX8,
                                                                      Subdivision_Method::C_HIERARCHY_TET4};
        mXTKModel.decompose(tDecompositionMethods);
    }

    else if(aOperationName.compare("xtk_compute_sensitivity") == 0)
    {
        mXTKModel.compute_sensitivity();
    }

    else if(aOperationName.compare("xtk_output_mesh")== 0)
    {
        mXTKOutputMesh = mXTKModel.get_output_mesh();
    }
    else if(aOperationName.compare("xtk_export_mesh")== 0)
    {
        std::cout<<mOutputMeshFile<<std::endl;
        mXTKOutputMesh->create_output_mesh(mOutputMeshFile);
    }
    else
    {
        std::cerr<<"Invalid XTK operation specified in compute function: "<< aOperationName<<std::endl;
        throw;
    }
}

//-------------------------------------------------------------------------------------

void
XTKApplication::importData(const std::string       & aArgumentName,
                           const Plato::SharedData & aImportData)
{
    if(aArgumentName.compare("topo_0") == 0)
    {
        // Get data into a moris matrix
        std::vector<double> tLevelSetData(aImportData.size());
        aImportData.getData(tLevelSetData);
        moris::Matrix<moris::DDRMat> tLevelSetMatrix(tLevelSetData.data(),(moris::size_t) 1, (moris::size_t) tLevelSetData.size());

        // Change field in mesh
        mMTKBackgroundMesh->add_mesh_field_real_scalar_data_loc_inds("topo_0", moris::EntityRank::NODE, tLevelSetMatrix);

        // Tell the model that the geometry is now ready and link the geometry with the background mesh
        mXTKModel.link_background_mesh_to_geometry_objects();
    }

    else
    {
        std::cerr<<"Invalid aArgumentName specified in importData: "<< aArgumentName<<std::endl;
    }

}

//-------------------------------------------------------------------------------------
void
XTKApplication::exportData(const std::string & aArgumentName,
                           Plato::SharedData & aExportData)
{
    if(aArgumentName.compare("topo_0") == 0)
    {
        moris::Cell< moris::Matrix < moris::DDRMat > > tLSVData = mXTKModel.assemble_geometry_data_as_mesh_field();
        int tMyLength = (int) tLSVData(0).numel();
        std::vector<double> tExportData(tLSVData(0).data(), tLSVData(0).data()+tMyLength);
        aExportData.setData(tExportData);
    }


    else if (aArgumentName.compare("DesVar0") == 0)
    {
        // Allocate data
        std::vector<double> tdxdpId0(mXTKModel.get_background_mesh().get_num_entities(moris::EntityRank::NODE));

        this->get_dxdp_des_var_ids(0,tdxdpId0);

        aExportData.setData(tdxdpId0);
    }

    else if (aArgumentName.compare("DesVar1") == 0)
     {
         // Allocate data
         std::vector<double> tdxdpId1(mXTKModel.get_background_mesh().get_num_entities(moris::EntityRank::NODE));

         this->get_dxdp_des_var_ids(1,tdxdpId1);

         aExportData.setData(tdxdpId1);
     }

    else if (aArgumentName.compare("dx1dp0") == 0)
    {
        // allocate data
        std::vector<double> tdxdp0x(mXTKModel.get_background_mesh().get_num_entities(moris::EntityRank::NODE));

        // collect data from xtk
        this->get_dxdp_data(0,0,tdxdp0x);

        // set data in shared data
        aExportData.setData(tdxdp0x);
    }

    else if (aArgumentName.compare("dx2dp0") == 0)
    {

        // Allocate data
        std::vector<double> tdxdp0y(mXTKModel.get_background_mesh().get_num_entities(moris::EntityRank::NODE));

        // collect data from xtk
        this->get_dxdp_data(0,1,tdxdp0y);

        aExportData.setData(tdxdp0y);
    }

    else if (aArgumentName.compare("dx3dp0") == 0)
    {

        // Allocate data
        std::vector<double> tdxdp0z(mXTKModel.get_background_mesh().get_num_entities(moris::EntityRank::NODE));

        // collect data from xtk
        this->get_dxdp_data(0,2,tdxdp0z);

        aExportData.setData(tdxdp0z);
    }

    else if (aArgumentName.compare("dx1dp1") == 0)
    {
        // allocate data
        std::vector<double> tdxdp1x(mXTKModel.get_background_mesh().get_num_entities(moris::EntityRank::NODE));

        // collect data from xtk
        this->get_dxdp_data(1,0,tdxdp1x);

        // set data in shared data
        aExportData.setData(tdxdp1x);
    }

    else if (aArgumentName.compare("dx2dp1") == 0)
    {

        // Allocate data
        std::vector<double> tdxdp0y(mXTKModel.get_background_mesh().get_num_entities(moris::EntityRank::NODE));

        // collect data from xtk
        this->get_dxdp_data(1,1,tdxdp0y);

        // set data in shared data
        aExportData.setData(tdxdp0y);
    }

    else if (aArgumentName.compare("dx3dp1") == 0)
    {

        // Allocate data
        std::vector<double> tdxdp1z(mXTKModel.get_background_mesh().get_num_entities(moris::EntityRank::NODE));

        // collect data from xtk
        this->get_dxdp_data(1,2,tdxdp1z);

        // set data in shared data
        aExportData.setData(tdxdp1z);
    }

    else if (aArgumentName.compare("NumDesVar") == 0)
        {
            // get a reference to the background grid
            xtk::Background_Mesh const & tBackgroundMesh = mXTKModel.get_background_mesh();

            // Get number of nodes (in background and new nodes create
            moris::uint tNumNodes                        = tBackgroundMesh.get_num_entities(moris::EntityRank::NODE);

            // XTK model for its geometry engine
            xtk::Geometry_Engine & tGeoEng               = mXTKModel.get_geom_engine();

            // Allocate data for number of dxdp indices a node depends on
            std::vector<double> tDxDpNumInds(tNumNodes);

            //Iterate through all nodes
            for(moris::moris_index iNode = 0; iNode<(moris::moris_index)tNumNodes; iNode++)
            {
                // If the node is on the interface it has shape sensitivity
                if(tBackgroundMesh.is_interface_node(iNode,0))
                {
                // Get the geometry object associated with this node
                xtk::Geometry_Object const & tNodeGeoObj = tGeoEng.get_geometry_object(iNode);

                // get the node adv indices
                moris::Matrix< moris::IndexMat > const & tNodeADVIndices = tNodeGeoObj.get_node_adv_indices();

                // Add size to data
                tDxDpNumInds[iNode] = tNodeADVIndices.numel();
                }

                // non-interface nodes do not have shape sensitivity
                else
                {
                    tDxDpNumInds[iNode] = 0;
                }
            }

            // place the data into the SharedData
            aExportData.setData(tDxDpNumInds);
        }

    else
    {
        std::cerr<<"Invalid XTK operation specified in compute function: "<< aArgumentName<<std::endl;
    }
}
//-------------------------------------------------------------------------------------
void
XTKApplication::get_dxdp_data(moris::uint aSpatialComp,
                              moris::uint aDesVarInd,
                              std::vector<double> & adxdp)
{
    // get a reference to the background grid
    xtk::Background_Mesh const & tBackgroundMesh = mXTKModel.get_background_mesh();

    // Get number of nodes (in background and new nodes create
    moris::uint tNumNodes                        = tBackgroundMesh.get_num_entities(moris::EntityRank::NODE);

    // XTK model for its geometry engine
    xtk::Geometry_Engine & tGeoEng               = mXTKModel.get_geom_engine();

    for(moris::moris_index iNode = 0; iNode<(moris::moris_index)tNumNodes; iNode++)
    {
        if(tBackgroundMesh.is_interface_node(iNode,0))
        {
            // Get the geometry object associated with this node
            xtk::Geometry_Object const & tNodeGeoObj = tGeoEng.get_geometry_object(iNode);

            // the sensitivity matrix
            moris::Matrix< moris::DDRMat > const & tdxdp = tNodeGeoObj.get_sensitivity_dx_dp();

            // Add size to data (first converting local node index to global id)
            adxdp[iNode] = tdxdp(aDesVarInd,aSpatialComp);
        }

        else
        {
            adxdp[iNode] = 0;
        }
    }
}

//-------------------------------------------------------------------------------------

void
XTKApplication::get_dxdp_des_var_ids( moris::uint aDesVarInd,
                                      std::vector<double> & adxdpIds)
{
    // get a reference to the background grid
    xtk::Background_Mesh const & tBackgroundMesh = mXTKModel.get_background_mesh();

    // Get number of nodes (in background and new nodes create
    moris::uint tNumNodes                        = tBackgroundMesh.get_num_entities(moris::EntityRank::NODE);

    // XTK model for its geometry engine
    xtk::Geometry_Engine & tGeoEng               = mXTKModel.get_geom_engine();


    for(moris::moris_index iNode = 0; iNode<(moris::moris_index)tNumNodes; iNode++)
    {
        if(tBackgroundMesh.is_interface_node(iNode,0))
        {
            // Get the geometry object associated with this node
            xtk::Geometry_Object const & tNodeGeoObj = tGeoEng.get_geometry_object(iNode);

            // get the node adv indices
            moris::Matrix< moris::IndexMat > const & tNodeADVIndices = tNodeGeoObj.get_node_adv_indices();

            MORIS_ASSERT(tNodeADVIndices.numel() == 2," only nodes which depend on two advs are supported currently");

            // Add size to data (first converting local node index to global id)
            adxdpIds[iNode] = (double) mMTKBackgroundMesh->get_glb_entity_id_from_entity_loc_index(tNodeADVIndices(1),EntityRank::NODE);
        }

        else
        {
            adxdpIds[iNode] = 0;
        }
    }
}

}


