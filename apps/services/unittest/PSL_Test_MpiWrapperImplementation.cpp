// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Implementation_MpiWrapper.hpp"
#include "PSL_Random.hpp"

#include <mpi.h>
#include <iostream>

namespace PlatoSubproblemLibrary
{

#define MpiWrapperImplementationTest_AllocateUtilities \
    MPI_Comm comm = MPI_COMM_WORLD; \
    if (example::get_size(comm) <= 1u) { \
        std::cout << "test not run; must be run in parallel" << std::endl; \
        return; \
    }

namespace MpiWrapperImplementation
{

PSL_TEST(MpiWrapperImplementation,send_and_recv_int)
{
    set_rand_seed();
    MpiWrapperImplementationTest_AllocateUtilities

    const size_t rank = example::get_rank(comm);
    const size_t size = example::get_size(comm);

    // send up, receive down
    const size_t send_rank = (int(rank) + 1) % size;
    const size_t recv_rank = (int(size + rank) - 1) % size;

    const int some_constant = 42;
    const size_t expected_data_size = 3u;

    std::vector<int> send_data = {some_constant, int(rank), int(some_constant * rank)};
    std::vector<int> receive_data(expected_data_size);

    if(rank % 2u == 0u)
    {
        example::send(comm, send_rank, send_data);
        example::receive(comm, recv_rank, receive_data);
    }
    else
    {
        example::receive(comm, recv_rank, receive_data);
        example::send(comm, send_rank, send_data);
    }

    // check
    const size_t receive_data_size = receive_data.size();
    ASSERT_EQ(receive_data_size, expected_data_size);
    EXPECT_EQ(receive_data[0], some_constant);
    EXPECT_EQ(receive_data[1], int(recv_rank));
    EXPECT_EQ(receive_data[2], int(recv_rank*some_constant));
}

PSL_TEST(MpiWrapperImplementation,send_and_recv_float)
{
    set_rand_seed();
    MpiWrapperImplementationTest_AllocateUtilities

    const size_t rank = example::get_rank(comm);
    const size_t size = example::get_size(comm);

    // send up, receive down
    const size_t send_rank = (int(rank) + 1) % size;
    const size_t recv_rank = (int(size + rank) - 1) % size;

    const float some_constant = 27.16;
    const size_t expected_data_size = 3u;

    std::vector<float> send_data = {some_constant, float(rank), float(some_constant * rank)};
    std::vector<float> receive_data(expected_data_size);

    if(rank % 2u == 0u)
    {
        example::send(comm, send_rank, send_data);
        example::receive(comm, recv_rank, receive_data);
    }
    else
    {
        example::receive(comm, recv_rank, receive_data);
        example::send(comm, send_rank, send_data);
    }

    // check
    const size_t receive_data_size = receive_data.size();
    ASSERT_EQ(receive_data_size, expected_data_size);
    EXPECT_FLOAT_EQ(receive_data[0], some_constant);
    EXPECT_FLOAT_EQ(receive_data[1], float(recv_rank));
    EXPECT_FLOAT_EQ(receive_data[2], float(recv_rank*some_constant));
}

PSL_TEST(MpiWrapperImplementation,send_and_recv_double)
{
    set_rand_seed();
    MpiWrapperImplementationTest_AllocateUtilities

    const size_t rank = example::get_rank(comm);
    const size_t size = example::get_size(comm);

    // send up, receive down
    const size_t send_rank = (int(rank) + 1) % size;
    const size_t recv_rank = (int(size + rank) - 1) % size;

    const double some_constant = 13.42;
    const size_t expected_data_size = 3u;

    std::vector<double> send_data = {some_constant, double(rank), double(some_constant * rank)};
    std::vector<double> receive_data(expected_data_size);

    if(rank % 2u == 0u)
    {
        example::send(comm, send_rank, send_data);
        example::receive(comm, recv_rank, receive_data);
    }
    else
    {
        example::receive(comm, recv_rank, receive_data);
        example::send(comm, send_rank, send_data);
    }

    // check
    const size_t receive_data_size = receive_data.size();
    ASSERT_EQ(receive_data_size, expected_data_size);
    EXPECT_FLOAT_EQ(receive_data[0], some_constant);
    EXPECT_FLOAT_EQ(receive_data[1], double(recv_rank));
    EXPECT_FLOAT_EQ(receive_data[2], double(recv_rank*some_constant));
}

PSL_TEST(MpiWrapperImplementation,allgather_int)
{
    set_rand_seed();
    MpiWrapperImplementationTest_AllocateUtilities

    const size_t rank = example::get_rank(comm);
    const size_t size = example::get_size(comm);

    const int some_constant = 42;
    const size_t expected_data_size_per_rank = 3u;

    std::vector<int> local_data = {some_constant, int(rank), int(some_constant * rank)};
    std::vector<int> global_data(expected_data_size_per_rank * size);

    example::all_gather(comm, local_data, global_data);

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

PSL_TEST(MpiWrapperImplementation,allgather_float)
{
    set_rand_seed();
    MpiWrapperImplementationTest_AllocateUtilities

    const size_t rank = example::get_rank(comm);
    const size_t size = example::get_size(comm);

    const float some_constant = 42.;
    const size_t expected_data_size_per_rank = 3u;

    std::vector<float> local_data = {some_constant, float(rank), float(some_constant * rank)};
    std::vector<float> global_data(expected_data_size_per_rank * size);

    example::all_gather(comm, local_data, global_data);

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

PSL_TEST(MpiWrapperImplementation,allgather_double)
{
    set_rand_seed();
    MpiWrapperImplementationTest_AllocateUtilities

    const size_t rank = example::get_rank(comm);
    const size_t size = example::get_size(comm);

    const double some_constant = 42.;
    const size_t expected_data_size_per_rank = 3u;

    std::vector<double> local_data = {some_constant, double(rank), double(some_constant * rank)};
    std::vector<double> global_data(expected_data_size_per_rank * size);

    example::all_gather(comm, local_data, global_data);

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

PSL_TEST(MpiWrapperImplementation,allreducesum_int)
{
    set_rand_seed();
    MpiWrapperImplementationTest_AllocateUtilities

    const size_t rank = example::get_rank(comm);
    const size_t size = example::get_size(comm);

    const int some_constant = 42;
    const size_t expected_data_size = 3u;

    std::vector<int> local_contribution = {some_constant, int(rank), int(some_constant * rank)};
    std::vector<int> global_result(expected_data_size);

    example::all_reduce_sum(comm, local_contribution, global_result);

    // check
    const size_t global_data_size = global_result.size();
    ASSERT_EQ(global_data_size, expected_data_size);
    EXPECT_EQ(global_result[0], int(some_constant*size));
    EXPECT_EQ(global_result[1], int(size*(size-1)/2));
    EXPECT_EQ(global_result[2], int(some_constant*size*(size-1)/2));
}

PSL_TEST(MpiWrapperImplementation,allreducesum_float)
{
    set_rand_seed();
    MpiWrapperImplementationTest_AllocateUtilities

    const size_t rank = example::get_rank(comm);
    const size_t size = example::get_size(comm);

    const float some_constant = 42.;
    const size_t expected_data_size = 3u;

    std::vector<float> local_contribution = {some_constant, float(rank), float(some_constant * rank)};
    std::vector<float> global_result(expected_data_size);

    example::all_reduce_sum(comm, local_contribution, global_result);

    // check
    const size_t global_data_size = global_result.size();
    ASSERT_EQ(global_data_size, expected_data_size);
    EXPECT_FLOAT_EQ(global_result[0], some_constant*size);
    EXPECT_FLOAT_EQ(global_result[1], size*(size-1)/2);
    EXPECT_FLOAT_EQ(global_result[2], some_constant*size*(size-1)/2);
}

PSL_TEST(MpiWrapperImplementation,allreducesum_double)
{
    set_rand_seed();
    MpiWrapperImplementationTest_AllocateUtilities

    const size_t rank = example::get_rank(comm);
    const size_t size = example::get_size(comm);

    const double some_constant = 47.;
    const size_t expected_data_size = 3u;

    std::vector<double> local_contribution = {some_constant, double(rank), double(some_constant * rank)};
    std::vector<double> global_result(expected_data_size);

    example::all_reduce_sum(comm, local_contribution, global_result);

    // check
    const size_t global_data_size = global_result.size();
    ASSERT_EQ(global_data_size, expected_data_size);
    EXPECT_FLOAT_EQ(global_result[0], some_constant*size);
    EXPECT_FLOAT_EQ(global_result[1], size*(size-1)/2);
    EXPECT_FLOAT_EQ(global_result[2], some_constant*size*(size-1)/2);
}

}

}
