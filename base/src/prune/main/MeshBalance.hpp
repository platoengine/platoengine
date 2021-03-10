/*
 * MeshBalance.hpp
 *
 *  Created on: Jul 28, 2017
 *      Author: tzirkle
 */

#ifndef ISO_PRUNE_MAIN_MESHBALANCE_HPP_
#define ISO_PRUNE_MAIN_MESHBALANCE_HPP_



#include "MeshManager.hpp"

class MeshBalance
{
public:
    MeshBalance();

    void balance_mesh(MeshManager &tMeshManager);
};



#endif /* ISO_PRUNE_MAIN_MESHBALANCE_HPP_ */
