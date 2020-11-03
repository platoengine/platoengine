/*
 * MeshPrune.cpp
 *
 *  Created on: Jul 28, 2017
 *      Author: tzirkle
 */
#include "MeshPrune.hpp"

#include "PruneMeshAPISTK.hpp"

    MeshPrune::MeshPrune()
    {

    }

    void MeshPrune::prune_mesh(MeshManager &tMeshManager,int argc, char *argv[],int tLastPrune)
    {
        tMeshManager.define_iso_fields();

        int tNumberOfBufferLayers;

        prune::PerceptPrune pruner;

        /*
        if (!tLastPrune)
        {
            tNumberOfBufferLayers = 0;
        }
        if (tLastPrune)
        {
        */
            tNumberOfBufferLayers = tMeshManager.get_buffer_layers();
//        }

//        PruneMeshAPISTK tMesh = (PruneMeshAPISTK(tMeshManager.get_communicator(),&tMeshManager.get_output_bulk_data(),&tMeshManager.get_output_meta_data(),tMeshManager.get_transfer_field_name()));
//        tMesh.define_iso_fields();


       if(pruner.import(argc,argv,
                        "LSD","",1e-5,0.0,0,1,0,0,tNumberOfBufferLayers)
               )
               {
                   pruner.run_percept_mesh_stand_alone(tMeshManager);
//                   pruner.run_stand_alone(tMesh);
               }
    }



