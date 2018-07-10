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

#include "PSL_Abstract_ParallelExchanger.hpp"
#include "PSL_Interface_ParallelExchanger_localAndNonlocal.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Abstract_ParallelVector.hpp"
#include "PSL_Interface_ParallelVector.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_Abstract_Mesh.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_Implementation_MeshModular.hpp"
#include "PSL_Point.hpp"
#include "PSL_Interface_MeshModular.hpp"
#include "PSL_Interface_ParallelExchanger_global.hpp"
#include "PSL_Interface_ParallelExchanger_ownershipFlag.hpp"
#include "PSL_Interface_ParallelExchanger_ownershipAndGlobal.hpp"
#include "PSL_Random.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>
#include <set>
#include <utility>

namespace PlatoSubproblemLibrary
{

namespace TestingParallelExchangerInterface
{
void test_length_change_from_exchange(AbstractInterface::ParallelVector* parallel_vector,
                                      AbstractInterface::ParallelExchanger* parallel_exchanger);
void test_matching_shared_node_data(AbstractInterface::MpiWrapper* mpi_wrapper,
                                    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data);
void test_rigorous_shared_node_data(AbstractInterface::MpiWrapper* mpi_wrapper,
                                    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data,
                                    AbstractInterface::Mesh* mesh);
void test_rigorous_global_ids(AbstractInterface::MpiWrapper* mpi_wrapper,
                              const std::vector<size_t>& global_ids,
                              AbstractInterface::Mesh* mesh);
void test_globalbased_exchanger(AbstractInterface::MpiWrapper* mpi_wrapper,
                                example::ElementBlock* modular_block,
                                const std::vector<size_t>& global_ids,
                                AbstractInterface::ParallelExchanger* parallel_exchanger);
}

PSL_TEST(ParallelExchangerInterface,test_parallel_exchanger_interface)
{
    set_rand_seed();
    AbstractAuthority authority;

    example::Interface_ParallelExchanger_localAndNonlocal parallel_exchanger(&authority);
    example::Interface_ParallelExchanger_global parallel_exchanger_global(&authority);

    // get mesh
    const int xlen = 3;
    const int ylen = 4;
    const int zlen = 5;
    const double xdist = 3.0;
    const double ydist = 8.0;
    const double zdist = 15.0;
    example::ElementBlock modular_block;
    const int rank = authority.mpi_wrapper->get_rank();
    const int num_processors = authority.mpi_wrapper->get_size();
    modular_block.build_from_structured_grid(xlen, ylen, zlen, xdist, ydist, zdist, rank, num_processors);
    const size_t local_num_nodes = modular_block.get_num_nodes();

    // build mesh interface
    example::Interface_MeshModular mesh_interface;
    mesh_interface.set_mesh(&modular_block);

    // transfer and build shared node data
    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data;
    modular_block.get_shared_node_data(shared_node_data);
    parallel_exchanger.put_shared_pairs(shared_node_data);
    parallel_exchanger.put_num_local_locations(local_num_nodes);
    parallel_exchanger.build();

    // test shared node data
    TestingParallelExchangerInterface::test_matching_shared_node_data(authority.mpi_wrapper, shared_node_data);
    TestingParallelExchangerInterface::test_rigorous_shared_node_data(authority.mpi_wrapper, shared_node_data, &mesh_interface);

    // build parallel exchanger from global identifiers
    std::vector<size_t> global_ids;
    modular_block.get_global_ids(global_ids);
    parallel_exchanger_global.put_globals(global_ids);
    parallel_exchanger_global.build();

    // test global ids
    TestingParallelExchangerInterface::test_rigorous_global_ids(authority.mpi_wrapper, global_ids, &mesh_interface);

    // get parallel vector
    example::Interface_ParallelVector parallel_vector;

    // test
    parallel_vector.m_data.assign(local_num_nodes, 0.);
    TestingParallelExchangerInterface::test_length_change_from_exchange(&parallel_vector, &parallel_exchanger);
    parallel_vector.m_data.assign(local_num_nodes, 0.);
    TestingParallelExchangerInterface::test_length_change_from_exchange(&parallel_vector, &parallel_exchanger_global);
}

PSL_TEST(ParallelExchangerInterface,test_parallel_exchanger_globals)
{
    set_rand_seed();
    AbstractAuthority authority;

    example::Interface_ParallelExchanger_global parallel_exchanger_global(&authority);

    // build for modular
    const double xdist = 3.0;
    const double ydist = 8.0;
    const double zdist = 15.0;
    const size_t approx_points_per_processor = 8u;
    example::ElementBlock modular_block;
    modular_block.build_random_subset_of_structured_grid(approx_points_per_processor,
                                                         xdist,
                                                         ydist,
                                                         zdist,
                                                         &authority);
    const size_t local_num_nodes = modular_block.get_num_nodes();

    // build mesh interface
    example::Interface_MeshModular mesh_interface;
    mesh_interface.set_mesh(&modular_block);

    // build parallel exchanger from global identifiers
    std::vector<size_t> global_ids;
    modular_block.get_global_ids(global_ids);
    ASSERT_EQ(global_ids.size(), local_num_nodes);
    if(authority.mpi_wrapper->get_size() % 2u)
    {
        for(size_t l = 0u; l < local_num_nodes; l++)
        {
            global_ids[l]++;
        }
    }
    parallel_exchanger_global.put_globals(global_ids);
    parallel_exchanger_global.build();

    TestingParallelExchangerInterface::test_globalbased_exchanger(authority.mpi_wrapper,
                                                                  &modular_block,
                                                                  global_ids,
                                                                  &parallel_exchanger_global);

    std::vector<size_t> contraction_indexes = parallel_exchanger_global.get_local_contracted_indexes();
    const size_t contraction_indexes_size = contraction_indexes.size();

    // state ownership of contracted indexes
    std::vector<bool> locally_owned(local_num_nodes, false);
    for(size_t c = 0u; c < contraction_indexes_size; c++)
    {
        const size_t local_index = contraction_indexes[c];
        locally_owned[local_index] = true;
    }

    // build exchanger
    example::Interface_ParallelExchanger_ownershipFlag parallel_exchanger_flagged(&authority);
    parallel_exchanger_flagged.put_is_owned(locally_owned);
    parallel_exchanger_flagged.build();

    // if owned, set to global id; otherwise, -1
    example::Interface_ParallelVector parallel_vector;
    parallel_vector.m_data.assign(local_num_nodes, -1.);
    for(size_t l = 0u; l < local_num_nodes; l++)
    {
        const size_t g = global_ids[l];
        if(locally_owned[l])
        {
            parallel_vector.set_value(l, g);
        }
    }

    // contract
    std::vector<double> flagged_contracted_vector = parallel_exchanger_flagged.get_contraction_to_local_indexes(&parallel_vector);

    // contracted should be >= 0
    const size_t flagged_num_contracted = flagged_contracted_vector.size();
    for(size_t c = 0u; c < flagged_num_contracted; c++)
    {
        EXPECT_GE(flagged_contracted_vector[c], 0.);
    }

    // expand
    parallel_exchanger_flagged.get_expansion_to_parallel_vector(flagged_contracted_vector, &parallel_vector);

    for(size_t l = 0u; l < local_num_nodes; l++)
    {
        const size_t g = global_ids[l];
        if(locally_owned[l])
        {
            EXPECT_FLOAT_EQ(parallel_vector.get_value(l), g);
        }
        else
        {
            EXPECT_FLOAT_EQ(parallel_vector.get_value(l), -1.);
        }
    }
}

namespace TestingParallelExchangerInterface
{

void test_length_change_from_exchange(AbstractInterface::ParallelVector* parallel_vector,
                                      AbstractInterface::ParallelExchanger* parallel_exchanger)
{
    // get rank , size
    AbstractInterface::MpiWrapper* mpi_wrapper = parallel_exchanger->get_mpi();
    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    // initialize parallel vector with size
    const size_t initial_parallel_size = parallel_vector->get_length();
    for(size_t parallel_index = 0u; parallel_index < initial_parallel_size; parallel_index++)
    {
        parallel_vector->set_value(parallel_index, size);
    }

    // contract
    std::vector<double> local_vector = parallel_exchanger->get_contraction_to_local_indexes(parallel_vector);
    const size_t num_local_contraction = local_vector.size();

    // fill local with rank
    local_vector.assign(num_local_contraction, rank);

    // expand
    parallel_exchanger->get_expansion_to_parallel_vector(local_vector, parallel_vector);
    const size_t num_parallel_expansion = parallel_vector->get_length();

    // get set of values
    std::set<size_t> parallel_vector_values;
    size_t max_value = parallel_vector->get_value(0);
    for(size_t k = 0u; k < num_parallel_expansion; k++)
    {
        parallel_vector_values.insert(size_t(parallel_vector->get_value(k)));
        max_value = std::max(size_t(parallel_vector->get_value(k)), max_value);
    }
    EXPECT_GT(size, max_value);

    // compare to expectations
    if(num_local_contraction == num_parallel_expansion)
    {
        // all values in parallel vector should be the local rank
        ASSERT_EQ(parallel_vector_values.size(), 1u);
        EXPECT_EQ(*parallel_vector_values.begin(), rank);
    }
    else
    {
        // some values in parallel vector should be different from local rank
        EXPECT_GT(parallel_vector_values.size(), 1u);
    }

    EXPECT_EQ(initial_parallel_size, parallel_vector->get_length());
}

void test_matching_shared_node_data(AbstractInterface::MpiWrapper* mpi_wrapper,
                                    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data)
{
    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    for(size_t proc = 0u; proc < size; proc++)
    {
        // you can not share with yourself
        if(proc == rank)
        {
            continue;
        }

        // compare number of shared datas
        if(proc < rank)
        {
            // send first
            const int num_share = shared_node_data[proc].size();
            mpi_wrapper->send(proc, num_share);
            int recv_num_share = 0u;
            mpi_wrapper->receive(proc, recv_num_share);
            EXPECT_EQ(num_share, recv_num_share);

            // compare explicit local data
            std::vector<std::pair<size_t, size_t> > data(shared_node_data[proc].begin(), shared_node_data[proc].end());
            std::sort(data.begin(), data.end());
            std::vector<int> data_firsts(num_share);
            std::vector<int> data_seconds(num_share);
            for(int k = 0; k < num_share; k++)
            {
                data_firsts[k] = data[k].first;
                data_seconds[k] = data[k].second;
            }
            mpi_wrapper->send(proc, data_firsts);
            mpi_wrapper->send(proc, data_seconds);
        }
        else
        {
            // recv first
            int recv_num_share = 0u;
            mpi_wrapper->receive(proc, recv_num_share);
            const int num_share = shared_node_data[proc].size();
            mpi_wrapper->send(proc, num_share);
            EXPECT_EQ(num_share, recv_num_share);

            // compare explicit local data
            std::vector<std::pair<int, int> > data(num_share);
            for(int k = 0; k < num_share; k++)
            {
                data[k] = std::make_pair(shared_node_data[proc][k].second, shared_node_data[proc][k].first);
            }
            std::sort(data.begin(), data.end());
            std::vector<int> data_firsts(num_share);
            std::vector<int> data_seconds(num_share);
            mpi_wrapper->receive(proc, data_firsts);
            mpi_wrapper->receive(proc, data_seconds);
            for(int k = 0; k < num_share; k++)
            {
                EXPECT_EQ(data[k].first, data_firsts[k]);
                EXPECT_EQ(data[k].second, data_seconds[k]);
            }
        }
    }
}

void test_rigorous_shared_node_data(AbstractInterface::MpiWrapper* mpi_wrapper,
                                    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data,
                                    AbstractInterface::Mesh* mesh)
{
    // for distance comparisons
    const double epsilon = 1e-6;

    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    // build local point cloud
    const size_t local_num_nodes = mesh->get_num_points();
    std::vector<bool> select_all(local_num_nodes, true);
    PointCloud* local_point_cloud = mesh->build_point_cloud(select_all);

    for(size_t proc = 0u; proc < size; proc++)
    {
        // you can not share with yourself
        if(proc == rank)
        {
            continue;
        }

        // allocate
        std::vector<std::pair<size_t,size_t> > local_and_nonlocal_pairs;
        PointCloud* nonlocal_point_cloud;

        // exchange data
        if(proc < rank)
        {
            // send first
            mpi_wrapper->send_point_cloud(proc, local_point_cloud);
            nonlocal_point_cloud = mpi_wrapper->receive_point_cloud(proc);
        }
        else
        {
            // recv first
            nonlocal_point_cloud = mpi_wrapper->receive_point_cloud(proc);
            mpi_wrapper->send_point_cloud(proc, local_point_cloud);
        }

        const size_t nonlocal_num_nodes = nonlocal_point_cloud->get_num_points();

        // for each local node
        for(size_t local_index = 0u; local_index < local_num_nodes; local_index++)
        {
            Point* local_point = local_point_cloud->get_point(local_index);

            // for each nonlocal node
            for(size_t nonlocal_index = 0; nonlocal_index < nonlocal_num_nodes; nonlocal_index++)
            {
                Point* nonlocal_point = nonlocal_point_cloud->get_point(nonlocal_index);

                // if distance less than threshold, assume same point
                const double distance = local_point->distance(nonlocal_point);
                if(distance < epsilon)
                {
                    local_and_nonlocal_pairs.push_back(std::make_pair(local_index, nonlocal_index));
                }
            }
        }

        // check agreement of results
        std::sort(shared_node_data[proc].begin(),shared_node_data[proc].end());
        const size_t num_shared_expected = shared_node_data[proc].size();
        const size_t num_shared_found = local_and_nonlocal_pairs.size();
        EXPECT_EQ(num_shared_expected, num_shared_found);
        if(num_shared_expected == num_shared_found)
        {
            for(size_t pairing_index = 0u; pairing_index < num_shared_expected; pairing_index++)
            {
                EXPECT_EQ(shared_node_data[proc][pairing_index].first, local_and_nonlocal_pairs[pairing_index].first);
                EXPECT_EQ(shared_node_data[proc][pairing_index].second, local_and_nonlocal_pairs[pairing_index].second);
            }
        }

        delete nonlocal_point_cloud;
    }

    delete local_point_cloud;
}

void test_rigorous_global_ids(AbstractInterface::MpiWrapper* mpi_wrapper,
                              const std::vector<size_t>& global_ids,
                              AbstractInterface::Mesh* mesh)
{
    // for distance comparisons
    const double epsilon = 1e-6;

    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    // build local point cloud
    const size_t local_num_nodes = mesh->get_num_points();
    std::vector<bool> select_all(local_num_nodes, true);
    PointCloud* local_point_cloud = mesh->build_point_cloud(select_all);
    std::vector<int> int_global_ids_from_local(global_ids.begin(), global_ids.end());

    for(size_t proc = 0u; proc < size; proc++)
    {
        // you can not share with yourself
        if(proc == rank)
        {
            continue;
        }

        // allocate
        PointCloud* nonlocal_point_cloud;

        // exchange data
        std::vector<int> int_global_ids_from_nonlocal;
        if(proc < rank)
        {
            // send first
            mpi_wrapper->send_point_cloud(proc, local_point_cloud);
            mpi_wrapper->send(proc, int_global_ids_from_local);
            nonlocal_point_cloud = mpi_wrapper->receive_point_cloud(proc);
            int_global_ids_from_nonlocal.resize(nonlocal_point_cloud->get_num_points());
            mpi_wrapper->receive(proc, int_global_ids_from_nonlocal);
        }
        else
        {
            // recv first
            nonlocal_point_cloud = mpi_wrapper->receive_point_cloud(proc);
            int_global_ids_from_nonlocal.resize(nonlocal_point_cloud->get_num_points());
            mpi_wrapper->receive(proc, int_global_ids_from_nonlocal);
            mpi_wrapper->send_point_cloud(proc, local_point_cloud);
            mpi_wrapper->send(proc, int_global_ids_from_local);
        }

        const size_t nonlocal_num_nodes = nonlocal_point_cloud->get_num_points();

        // for each local node
        for(size_t local_index = 0u; local_index < local_num_nodes; local_index++)
        {
            Point* local_point = local_point_cloud->get_point(local_index);

            // for each nonlocal node
            for(size_t nonlocal_index = 0; nonlocal_index < nonlocal_num_nodes; nonlocal_index++)
            {
                Point* nonlocal_point = nonlocal_point_cloud->get_point(nonlocal_index);

                // if distance less than threshold, same point
                const double distance = local_point->distance(nonlocal_point);
                if(distance < epsilon)
                {
                    // check global matching
                    EXPECT_EQ(int_global_ids_from_nonlocal[nonlocal_index], int_global_ids_from_local[local_index]);
                }
                else
                {
                    // check global different
                    EXPECT_NE(int_global_ids_from_nonlocal[nonlocal_index], int_global_ids_from_local[local_index]);
                }
            }
        }

        delete nonlocal_point_cloud;
    }

    delete local_point_cloud;
}

void test_globalbased_exchanger(AbstractInterface::MpiWrapper* mpi_wrapper,
                                example::ElementBlock* modular_block,
                                const std::vector<size_t>& global_ids,
                                AbstractInterface::ParallelExchanger* parallel_exchanger)
{
    const size_t local_num_nodes = global_ids.size();

    // get parallel vector
    example::Interface_ParallelVector parallel_vector;
    parallel_vector.m_data.assign(local_num_nodes, 0.);
    std::vector<size_t> contraction_indexes = parallel_exchanger->get_local_contracted_indexes();
    std::vector<double> contraction_data = parallel_exchanger->get_contraction_to_local_indexes(&parallel_vector);

    // same lengths
    const size_t contraction_indexes_size = contraction_indexes.size();
    const size_t contraction_data_size = contraction_data.size();
    ASSERT_EQ(contraction_indexes_size, contraction_data_size);

    // for contraction, put data as globals
    for(size_t c = 0u; c < contraction_indexes_size; c++)
    {
        const size_t local_index = contraction_indexes[c];
        const size_t global_index = global_ids[local_index];
        contraction_data[c] = global_index;
    }

    // check contraction size sums to global num nodes
    int local_unique_num_nodes = contraction_indexes_size;
    int global_unique_num_nodes = 0;
    mpi_wrapper->all_reduce_sum(local_unique_num_nodes, global_unique_num_nodes);
    int global_num_nodes = modular_block->get_global_num_nodes();
    EXPECT_EQ(global_unique_num_nodes, global_num_nodes);

    // expand back
    parallel_exchanger->get_expansion_to_parallel_vector(contraction_data, &parallel_vector);
    EXPECT_EQ(parallel_vector.get_length(), local_num_nodes);

    // expect to be global value
    for(size_t l = 0u; l < local_num_nodes; l++)
    {
        const size_t g = global_ids[l];
        const double pv_value = parallel_vector.get_value(l);
        EXPECT_FLOAT_EQ(g, pv_value);
    }
}

}

}
