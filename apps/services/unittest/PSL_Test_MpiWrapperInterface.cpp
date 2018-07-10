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

// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Implementation_MpiWrapper.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_Random.hpp"

#include <mpi.h>
#include <iostream>

namespace PlatoSubproblemLibrary
{

#define MpiWrapperInterfaceTest_AllocateUtilities \
    MPI_Comm comm = MPI_COMM_WORLD; \
    example::Interface_BasicGlobalUtilities utilities; \
    example::Interface_MpiWrapper mpi_interface(&utilities, &comm); \
    AbstractInterface::MpiWrapper* mpi_wrapper = &mpi_interface; \
    if (mpi_wrapper->get_size() <= 1u) { \
        std::cout << "test not run; must be run in parallel" << std::endl; \
        return; \
    }

namespace MpiWrapperInterface
{

PSL_TEST(MpiWrapperInterface,send_and_recv_int)
{
    set_rand_seed();
    MpiWrapperInterfaceTest_AllocateUtilities

    const double initial_time = mpi_wrapper->get_time();

    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    // send up, receive down
    const size_t send_rank = (int(rank) + 1) % size;
    const size_t recv_rank = (int(size + rank) - 1) % size;

    const int some_constant = 42;
    const size_t expected_data_size = 3u;

    std::vector<int> send_data = {some_constant, int(rank), int(some_constant * rank)};
    std::vector<int> receive_data(expected_data_size);

    if(rank % 2u == 0u)
    {
        mpi_wrapper->send(send_rank, send_data);
        mpi_wrapper->receive(recv_rank, receive_data);
    }
    else
    {
        mpi_wrapper->receive(recv_rank, receive_data);
        mpi_wrapper->send(send_rank, send_data);
    }

    // check
    const size_t receive_data_size = receive_data.size();
    ASSERT_EQ(receive_data_size, expected_data_size);
    EXPECT_EQ(receive_data[0], some_constant);
    EXPECT_EQ(receive_data[1], int(recv_rank));
    EXPECT_EQ(receive_data[2], int(recv_rank*some_constant));

    // expect time to have passed
    const double final_time = mpi_wrapper->get_time();
    EXPECT_GT(final_time, initial_time);
}

PSL_TEST(MpiWrapperInterface,send_and_recv_float)
{
    set_rand_seed();
    MpiWrapperInterfaceTest_AllocateUtilities

    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    // send up, receive down
    const size_t send_rank = (int(rank) + 1) % size;
    const size_t recv_rank = (int(size + rank) - 1) % size;

    const float some_constant = 27.16;
    const size_t expected_data_size = 3u;

    std::vector<float> send_data = {some_constant, float(rank), float(some_constant * rank)};
    std::vector<float> receive_data(expected_data_size);

    if(rank % 2u == 0u)
    {
        mpi_wrapper->send(send_rank, send_data);
        mpi_wrapper->receive(recv_rank, receive_data);
    }
    else
    {
        mpi_wrapper->receive(recv_rank, receive_data);
        mpi_wrapper->send(send_rank, send_data);
    }

    // check
    const size_t receive_data_size = receive_data.size();
    ASSERT_EQ(receive_data_size, expected_data_size);
    EXPECT_FLOAT_EQ(receive_data[0], some_constant);
    EXPECT_FLOAT_EQ(receive_data[1], float(recv_rank));
    EXPECT_FLOAT_EQ(receive_data[2], float(recv_rank*some_constant));
}

PSL_TEST(MpiWrapperInterface,send_and_recv_double)
{
    set_rand_seed();
    MpiWrapperInterfaceTest_AllocateUtilities

    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    // send up, receive down
    const size_t send_rank = (int(rank) + 1) % size;
    const size_t recv_rank = (int(size + rank) - 1) % size;

    const double some_constant = 13.42;
    const size_t expected_data_size = 3u;

    std::vector<double> send_data = {some_constant, double(rank), double(some_constant * rank)};
    std::vector<double> receive_data(expected_data_size);

    if(rank % 2u == 0u)
    {
        mpi_wrapper->send(send_rank, send_data);
        mpi_wrapper->receive(recv_rank, receive_data);
    }
    else
    {
        mpi_wrapper->receive(recv_rank, receive_data);
        mpi_wrapper->send(send_rank, send_data);
    }

    // check
    const size_t receive_data_size = receive_data.size();
    ASSERT_EQ(receive_data_size, expected_data_size);
    EXPECT_FLOAT_EQ(receive_data[0], some_constant);
    EXPECT_FLOAT_EQ(receive_data[1], double(recv_rank));
    EXPECT_FLOAT_EQ(receive_data[2], double(recv_rank*some_constant));
}

PSL_TEST(MpiWrapperInterface,allgather_int)
{
    set_rand_seed();
    MpiWrapperInterfaceTest_AllocateUtilities

    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    const int some_constant = 42;
    const size_t expected_data_size_per_rank = 3u;

    std::vector<int> local_data = {some_constant, int(rank), int(some_constant * rank)};
    std::vector<int> global_data(expected_data_size_per_rank * size);

    mpi_wrapper->all_gather(local_data, global_data);

    // check
    const size_t global_data_size = global_data.size();
    ASSERT_EQ(global_data_size, expected_data_size_per_rank*size);
    for(size_t rank_index = 0u; rank_index < size; rank_index++)
    {
        EXPECT_EQ(global_data[expected_data_size_per_rank*rank_index+0u], some_constant);
        EXPECT_EQ(global_data[expected_data_size_per_rank*rank_index+1u], int(rank_index));
        EXPECT_EQ(global_data[expected_data_size_per_rank*rank_index+2u], int(rank_index*some_constant));
    }
}

PSL_TEST(MpiWrapperInterface,allgather_float)
{
    set_rand_seed();
    MpiWrapperInterfaceTest_AllocateUtilities

    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    const float some_constant = 42;
    const size_t expected_data_size_per_rank = 3u;

    std::vector<float> local_data = {some_constant, float(rank), float(some_constant * rank)};
    std::vector<float> global_data(expected_data_size_per_rank * size);

    mpi_wrapper->all_gather(local_data, global_data);

    // check
    const size_t global_data_size = global_data.size();
    ASSERT_EQ(global_data_size, expected_data_size_per_rank*size);
    for(size_t rank_index = 0u; rank_index < size; rank_index++)
    {
        EXPECT_EQ(global_data[expected_data_size_per_rank*rank_index+0u], some_constant);
        EXPECT_FLOAT_EQ(global_data[expected_data_size_per_rank*rank_index+1u], float(rank_index));
        EXPECT_FLOAT_EQ(global_data[expected_data_size_per_rank*rank_index+2u], float(rank_index*some_constant));
    }
}

PSL_TEST(MpiWrapperInterface,allgather_double)
{
    set_rand_seed();
    MpiWrapperInterfaceTest_AllocateUtilities

    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    const double some_constant = 42;
    const size_t expected_data_size_per_rank = 3u;

    std::vector<double> local_data = {some_constant, double(rank), double(some_constant * rank)};
    std::vector<double> global_data(expected_data_size_per_rank * size);

    mpi_wrapper->all_gather(local_data, global_data);

    // check
    const size_t global_data_size = global_data.size();
    ASSERT_EQ(global_data_size, expected_data_size_per_rank*size);
    for(size_t rank_index = 0u; rank_index < size; rank_index++)
    {
        EXPECT_EQ(global_data[expected_data_size_per_rank*rank_index+0u], some_constant);
        EXPECT_FLOAT_EQ(global_data[expected_data_size_per_rank*rank_index+1u], double(rank_index));
        EXPECT_FLOAT_EQ(global_data[expected_data_size_per_rank*rank_index+2u], double(rank_index*some_constant));
    }
}

PSL_TEST(MpiWrapperInterface,allreducesum_int)
{
    set_rand_seed();
    MpiWrapperInterfaceTest_AllocateUtilities

    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    const int some_constant = 42;
    const size_t expected_data_size = 3u;

    std::vector<int> local_contribution = {some_constant, int(rank), int(some_constant * rank)};
    std::vector<int> global_result(expected_data_size);

    mpi_wrapper->all_reduce_sum(local_contribution, global_result);

    // check
    const size_t global_data_size = global_result.size();
    ASSERT_EQ(global_data_size, expected_data_size);
    EXPECT_EQ(global_result[0], int(some_constant*size));
    EXPECT_EQ(global_result[1], int(size*(size-1)/2));
    EXPECT_EQ(global_result[2], int(some_constant*size*(size-1)/2));
}

PSL_TEST(MpiWrapperInterface,allreducesum_float)
{
    set_rand_seed();
    MpiWrapperInterfaceTest_AllocateUtilities

    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    const float some_constant = 42.;
    const size_t expected_data_size = 3u;

    std::vector<float> local_contribution = {some_constant, float(rank), float(some_constant * rank)};
    std::vector<float> global_result(expected_data_size);

    mpi_wrapper->all_reduce_sum(local_contribution, global_result);

    // check
    const size_t global_data_size = global_result.size();
    ASSERT_EQ(global_data_size, expected_data_size);
    EXPECT_FLOAT_EQ(global_result[0], some_constant*size);
    EXPECT_FLOAT_EQ(global_result[1], size*(size-1)/2);
    EXPECT_FLOAT_EQ(global_result[2], some_constant*size*(size-1)/2);
}

PSL_TEST(MpiWrapperInterface,allreducesum_double)
{
    set_rand_seed();
    MpiWrapperInterfaceTest_AllocateUtilities

    const size_t rank = mpi_wrapper->get_rank();
    const size_t size = mpi_wrapper->get_size();

    const double some_constant = 47.;
    const size_t expected_data_size = 3u;

    std::vector<double> local_contribution = {some_constant, double(rank), double(some_constant * rank)};
    std::vector<double> global_result(expected_data_size);

    mpi_wrapper->all_reduce_sum(local_contribution, global_result);

    // check
    const size_t global_data_size = global_result.size();
    ASSERT_EQ(global_data_size, expected_data_size);
    EXPECT_FLOAT_EQ(global_result[0], some_constant*size);
    EXPECT_FLOAT_EQ(global_result[1], size*(size-1)/2);
    EXPECT_FLOAT_EQ(global_result[2], some_constant*size*(size-1)/2);
}

}

}
