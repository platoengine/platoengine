/*
 * MeshBalance.cpp
 *
 *  Created on: Jul 28, 2017
 *      Author: tzirkle
 */

#include "MeshBalance.hpp"
#include <ZoltanPartition.hpp>
#include <Rebalance.hpp>
/*
#include <PerceptMesh.hpp>
#include <adapt/UniformRefinerPattern.hpp>
#include <adapt/UniformRefiner.hpp>
#include <adapt/RefinerUtil.hpp>
#include <adapt/UniformRefinerPattern_def.hpp>
#include <stk_rebalance/Rebalance.hpp>
#include <stk_rebalance/Partition.hpp>
*/


    MeshBalance::MeshBalance()
    {

    }

    void MeshBalance::balance_mesh(MeshManager &tMeshManager)
    {
        int spatial_dimension = 3;

              typedef stk::mesh::Field<double, stk::mesh::Cartesian> VectorField;
              const stk::mesh::EntityRank rank_to_rebalance    = stk::topology::ELEMENT_RANK;

              Teuchos::ParameterList emptyList;
              stk::rebalance::Zoltan zoltan_partition(tMeshManager.get_output_bulk_data(), tMeshManager.get_u_communicator(), spatial_dimension, emptyList);

              stk::mesh::MetaData & fem_meta = tMeshManager.get_output_meta_data();

              VectorField & coord_field = fem_meta.declare_field< VectorField >( stk::topology::NODE_RANK,  "coordinates" ) ;
              stk::mesh::Selector selector(fem_meta.universal_part());

              stk::rebalance::Rebalance rb;
              rb.rebalance(tMeshManager.get_output_bulk_data(), selector, &coord_field, NULL, zoltan_partition,rank_to_rebalance);
    }



