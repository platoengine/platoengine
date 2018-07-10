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

// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Implementation_MeshMonolithic.hpp"
#include "PSL_Implementation_MeshModular.hpp"
#include "PSL_Interface_MeshMonolithic.hpp"
#include "PSL_Interface_MeshModular.hpp"
#include "PSL_Abstract_OptimizationMesh.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_Point.hpp"
#include "PSL_Random.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <cstddef>
#include <algorithm>
#include <mpi.h>

namespace PlatoSubproblemLibrary
{

namespace MeshAbstractionTest
{

void test_AbstractInterface_NeighboredPoints(AbstractInterface::OptimizationMesh* interface,
                                             int xlen, int ylen, int zlen,
                                             double xdist, double ydist, double zdist,
                                             bool test_locations);

PSL_TEST(MeshAbstraction,mesh_interface)
{
    set_rand_seed();
    AbstractAuthority authority;

    // build
    const int xlen = 3;
    const int ylen = 4;
    const int zlen = 5;
    const double xdist = 3.0;
    const double ydist = 8.0;
    const double zdist = 15.0;
    example::MeshMonolithic mono_mesh;
    mono_mesh.build_from_structured_grid(xlen, ylen, zlen, xdist, ydist, zdist);
    example::Interface_MeshMonolithic mono_interface;
    mono_interface.set_mesh(&mono_mesh);

    // test
    const bool mono_test_locations = true;
    test_AbstractInterface_NeighboredPoints(&mono_interface, xlen, ylen, zlen, xdist, ydist, zdist, mono_test_locations);

    // check neighbors
    std::vector<size_t> mono_neighbors = mono_interface.get_point_neighbors(0);
    ASSERT_EQ(mono_neighbors.size(), 7u);
    EXPECT_EQ(mono_neighbors[0], 1u);
    EXPECT_EQ(mono_neighbors[1], 5u);
    EXPECT_EQ(mono_neighbors[2], 6u);
    EXPECT_EQ(mono_neighbors[3], 20u);
    EXPECT_EQ(mono_neighbors[4], 21u);
    EXPECT_EQ(mono_neighbors[5], 25u);
    EXPECT_EQ(mono_neighbors[6], 26u);

    // build for modular
    example::ElementBlock modular_block;
    const int rank = authority.mpi_wrapper->get_rank();
    const int num_processors = authority.mpi_wrapper->get_size();
    modular_block.build_from_structured_grid(xlen, ylen, zlen, xdist, ydist, zdist, rank, num_processors);
    example::Interface_MeshModular modular_interface;
    modular_interface.set_mesh(&modular_block);

    // test
    const bool modular_test_locations = (rank == 0);
    test_AbstractInterface_NeighboredPoints(&modular_interface, xlen, ylen, zlen, xdist, ydist, zdist, modular_test_locations);

    // check neighbors
    std::vector<size_t> modular_neighbors = modular_interface.get_point_neighbors(0);
    ASSERT_EQ(modular_neighbors.size(), 7u);
    EXPECT_EQ(modular_neighbors[0], 1u);
    EXPECT_EQ(modular_neighbors[1], 5u);
    EXPECT_EQ(modular_neighbors[2], 6u);
    EXPECT_EQ(modular_neighbors[3], 20u);
    EXPECT_EQ(modular_neighbors[4], 21u);
    EXPECT_EQ(modular_neighbors[5], 25u);
    EXPECT_EQ(modular_neighbors[6], 26u);
}

void test_AbstractInterface_NeighboredPoints(AbstractInterface::OptimizationMesh* interface,
                                             int xlen, int ylen, int zlen,
                                             double xdist, double ydist, double zdist,
                                             bool test_locations)
{
    // sizes
    const size_t expected_num_points = xlen * ylen * zlen;
    EXPECT_EQ(interface->get_num_points(), expected_num_points);

    if(test_locations)
    {
        // check first point location
        Point first_point = interface->get_point(0);
        ASSERT_EQ(first_point.dimension(), 3u);
        EXPECT_EQ(first_point(0), 0.0);
        EXPECT_EQ(first_point(1), 0.0);
        EXPECT_EQ(first_point(2), 0.0);

        // check last point location
        Point last_point = interface->get_point(xlen * ylen * zlen - 1);
        ASSERT_EQ(last_point.dimension(), 3u);
        EXPECT_EQ(last_point(0), xdist);
        EXPECT_EQ(last_point(1), ydist);
        EXPECT_EQ(last_point(2), zdist);
    }

    // check num blocks
    ASSERT_EQ(interface->get_num_blocks(), 1u);

    // check num elements
    const size_t expected_num_elements = (xlen - 1) * (ylen - 1) * (zlen - 1);
    EXPECT_EQ(interface->get_num_elements(0), expected_num_elements);

    // check nodes from element
    std::vector<size_t> nodes_from_element = interface->get_nodes_from_element(0u,0u);
    ASSERT_EQ(nodes_from_element.size(), 8u);
    std::sort(nodes_from_element.begin(), nodes_from_element.end());
    EXPECT_EQ(nodes_from_element[0], 0u);
    EXPECT_EQ(nodes_from_element[1], 1u);
    EXPECT_EQ(nodes_from_element[2], 5u);
    EXPECT_EQ(nodes_from_element[3], 6u);
    EXPECT_EQ(nodes_from_element[4], 20u);
    EXPECT_EQ(nodes_from_element[5], 21u);
    EXPECT_EQ(nodes_from_element[6], 25u);
    EXPECT_EQ(nodes_from_element[7], 26u);

    // check optimizable
    EXPECT_EQ(interface->is_block_optimizable(0), true);
}

bool does_a_node_exceed_tolerance(AbstractInterface::MpiWrapper* mpi_wrapper,
                                  example::ElementBlock* modular_block,
                                  double spatial_expect)
{
    int local_result = 0;

    std::vector<size_t> local_to_global;
    modular_block->get_global_ids(local_to_global);
    const size_t local_num_nodes = local_to_global.size();
    for(size_t local_index = 0u; local_index < local_num_nodes; local_index++)
    {
        Point* local_point = modular_block->get_node(local_index)->get_point();

        const size_t global_index = local_to_global[local_index];
        Point* global_point = modular_block->get_global_point(global_index);

        // found a distance that exceeds the tolerance?
        local_result |= (local_point->distance(global_point) > spatial_expect);
    }

    // make global
    int global_result = 0;
    mpi_wrapper->all_reduce_max(local_result, global_result);
    return global_result;
}


PSL_TEST(MeshAbstraction,randomSubsetOfStructuredGrid)
{
    set_rand_seed();
    AbstractAuthority authority;

    // build
    const double xdist = 3.0;
    const double ydist = 8.0;
    const double zdist = 15.0;
    const size_t approx_points_per_processor = 16u;

    // build for modular
    example::ElementBlock modular_block;
    modular_block.build_random_subset_of_structured_grid(approx_points_per_processor,
                                                         xdist, ydist, zdist,
                                                         &authority);
    example::Interface_MeshModular modular_interface;
    modular_interface.set_mesh(&modular_block);

    // ensure all global nodes still exist
    const int global_num_nodes = modular_block.get_global_num_nodes();
    std::vector<int> locally_exists(global_num_nodes, 0);
    std::vector<size_t> local_to_global;
    modular_block.get_global_ids(local_to_global);
    const size_t num_locals = local_to_global.size();
    for(size_t li = 0u; li < num_locals; li++)
    {
        locally_exists[local_to_global[li]] = 1;
    }
    std::vector<int> globally_exists(global_num_nodes, 0);
    authority.mpi_wrapper->all_reduce_sum(locally_exists, globally_exists);
    for(int gi = 0u; gi < global_num_nodes; gi++)
    {
        EXPECT_GT(globally_exists[gi],0);
    }

    // ensure all elements still exist
    int local_num_elements = modular_block.get_num_elements();
    const int global_num_elements = modular_block.get_global_num_elements();
    int compute_global_num_elements = 0;
    authority.mpi_wrapper->all_reduce_sum(local_num_elements, compute_global_num_elements);
    EXPECT_EQ(compute_global_num_elements, global_num_elements);

    // specify perturb tolerances
    const double spatial_tolerance = 1e-2;
    const double spatial_perturb = spatial_tolerance / 10.0;
    const double spatial_expect = spatial_perturb / 100.0;

    EXPECT_EQ(does_a_node_exceed_tolerance(authority.mpi_wrapper,&modular_block, spatial_expect), false);

    // perturb
    modular_block.random_perturb_local_nodal_locations(spatial_perturb);

    EXPECT_EQ(does_a_node_exceed_tolerance(authority.mpi_wrapper,&modular_block, spatial_expect), true);
}

}

}

