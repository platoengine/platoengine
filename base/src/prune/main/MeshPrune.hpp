/*
 * MeshPrune.hpp
 *
 *  Created on: Jul 28, 2017
 *      Author: tzirkle
 */

#ifndef ISO_PRUNE_MAIN_MESHPRUNE_HPP_
#define ISO_PRUNE_MAIN_MESHPRUNE_HPP_

#include "MeshManager.hpp"
#include <PerceptMesh.hpp>
#include "PerceptPrune.hpp"

class MeshPrune
{
public:
    MeshPrune();

    void prune_mesh(MeshManager &tMeshManager,int argc, char *argv[],int tLastPrune);

};



#endif /* ISO_PRUNE_MAIN_MESHPRUNE_HPP_ */
