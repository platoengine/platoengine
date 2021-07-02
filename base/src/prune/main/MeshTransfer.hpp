/*
 * MeshTransfer.hpp
 *
 *  Created on: Jul 28, 2017
 *      Author: tzirkle
 */

#ifndef ISO_PRUNE_MAIN_MESHTRANSFER_HPP_
#define ISO_PRUNE_MAIN_MESHTRANSFER_HPP_

#include "MeshManager.hpp"
#include <STKMeshTransferSetup.hpp>
#include <PerceptMesh.hpp>

class MeshTransfer
{
public:
    MeshTransfer();

    void transfer_mesh(MeshManager& aMeshManager);
};



#endif /* ISO_PRUNE_MAIN_MESHTRANSFER_HPP_ */
