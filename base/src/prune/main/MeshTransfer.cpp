/*
 * MeshTransfer.cpp
 *
 *  Created on: Jul 28, 2017
 *      Author: tzirkle
 */

#include "MeshTransfer.hpp"

    MeshTransfer::MeshTransfer()
    {

    }


    void MeshTransfer::transfer_mesh(MeshManager& aMeshManager)
    {

        if(stk::parallel_machine_rank(aMeshManager.get_u_communicator()) == 0)
            std::cout<<"Beginning Transfer"<<std::endl;
//#ifdef BUILD_IN_SIERRA
        std::shared_ptr<STKMeshTransfer> transfer =
              buildSTKMeshTransfer<STKMeshTransfer>(aMeshManager.get_input_bulk_data(),
                                aMeshManager.get_input_coordinate_field(),
                                aMeshManager.get_input_transfer_field(),
                                aMeshManager.get_output_bulk_data(),
                                aMeshManager.get_output_coordinate_field(),
                                aMeshManager.get_output_transfer_field(),
                                "transfer_coords",
                                SRC_FIELD,
                                0.0);
/*
#else
        boost::shared_ptr<STKMeshTransfer> transfer =
              buildSTKMeshTransfer<STKMeshTransfer>(aMeshManager.get_input_bulk_data(),
                                aMeshManager.get_input_coordinate_field(),
                                aMeshManager.get_input_transfer_field(),
                                aMeshManager.get_output_bulk_data(),
                                aMeshManager.get_output_coordinate_field(),
                                aMeshManager.get_output_transfer_field(),
                                "transfer_coords",
                                SRC_FIELD,
                                0.0);
#endif
*/

         initializeSTKMeshTransfer(&*transfer);
         if(stk::parallel_machine_rank(aMeshManager.get_u_communicator()) == 0)
             std::cout<<"Applying Transfer"<<std::endl;
         transfer->apply();
    }


