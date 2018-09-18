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

#include <gtest/gtest.h>

#include "Plato_TimersTree.hpp"

#include <mpi.h>                    // for MPI_Comm_rank
#include <stddef.h>                 // for size_t
#include <vector>                   // for vector

namespace Plato
{
namespace TimersTreeTest
{

template<typename t>
void checkVectorIfRankZero(MPI_Comm& comm, const std::vector<t>& A, const std::vector<t>& B);

TEST(PlatoTimersTree, simple)
{
    // allocate problem
    const size_t num_keys = 2u;
    const size_t num_timers = num_keys + 1u;
    // define MPI
    MPI_Comm comm = MPI_COMM_WORLD;
    int rank;
    MPI_Comm_rank(comm, &rank);
    // define Tree
    TimersTree this_tree(comm, num_keys);

    // allocate for expected vectors
    std::vector<double> expected_times(num_timers, 0.0);
    std::vector<size_t> expected_entrances(num_timers, 0u);

    // stack: 2
    expected_entrances[2]++; // entrances: 0,0,1
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_entrances(), expected_entrances);

    this_tree.unit_testing_incrament(); // times: 0,0,1

    this_tree.begin_partition(1); // stack: 2, 1
    expected_entrances[1]++; // entrances: 0,1,1
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_entrances(), expected_entrances);

    this_tree.begin_partition(0); // stack: 2, 1, 0
    expected_entrances[0]++; // entrances: 1,1,1
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_entrances(), expected_entrances);

    this_tree.end_partition(); // stack: 2, 1
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_timers(), expected_times);

    this_tree.unit_testing_incrament(); // times: 0,1,1

    this_tree.end_partition(); // stack: 2
    expected_times[1]++; // accumulated: 0,1,0
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_timers(), expected_times);

    this_tree.unit_testing_incrament(); // times: 0,1,2

    this_tree.end_partition(); // stack: ~
    expected_times[2] += 2.0; // accumulated: 0,1,2
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_timers(), expected_times);

    this_tree.begin_partition(2); // stack: 2
    expected_entrances[2]++; // entrances: 1,1,2
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_entrances(), expected_entrances);

    EXPECT_EQ(this_tree.print_results(), (rank==0));
}

TEST(PlatoTimersTree, complex)
{
    // allocate problem
    const size_t num_keys = 3u;
    const size_t num_timers = num_keys + 1u;
    // define MPI
    MPI_Comm comm = MPI_COMM_WORLD;
    int rank;
    MPI_Comm_rank(comm, &rank);
    // define Tree
    TimersTree this_tree(comm, num_keys);

    // allocate for expected vectors
    std::vector<double> expected_times(num_timers, 0.0);
    std::vector<size_t> expected_entrances(num_timers, 0u);

    // stack: 3
    expected_entrances[3]++; // entrances: 0,0,0,1
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_entrances(), expected_entrances);

    this_tree.unit_testing_incrament(); // times: 0,0,0,1

    this_tree.begin_partition(0); // stack: 3, 0
    expected_entrances[0]++; // entrances: 1,0,0,1
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_entrances(), expected_entrances);

    this_tree.unit_testing_incrament(); // times: 1,0,0,1

    this_tree.begin_partition(2); // stack: 3, 0, 2
    expected_entrances[2]++; // entrances: 1,0,1,1
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_entrances(), expected_entrances);

    this_tree.unit_testing_incrament(); // times: 1,0,1,1

    this_tree.begin_partition(1); // stack: 3, 0, 2, 1
    expected_entrances[1]++; // entrances: 1,1,1,1
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_entrances(), expected_entrances);

    this_tree.unit_testing_incrament(); // times: 1,1,1,1

    this_tree.begin_partition(1); // stack: 3, 0, 2, 1, 1
    expected_entrances[1]++; // entrances: 1,2,1,1
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_entrances(), expected_entrances);

    this_tree.unit_testing_incrament(); // times: 1,2,1,1

    this_tree.end_partition(); // stack: 3, 0, 2, 1
    expected_times[1]++; // accumulated: 0,1,0,0
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_timers(), expected_times);

    this_tree.end_partition(); // stack: 3, 0, 2
    expected_times[1]++; // accumulated: 0,2,0,0
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_timers(), expected_times);

    this_tree.end_partition(); // stack: 3, 0
    expected_times[2]++; // accumulated: 0,2,1,0
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_timers(), expected_times);

    this_tree.end_partition(); // stack: 3
    expected_times[0]++; // accumulated: 1,2,1,0
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_timers(), expected_times);

    this_tree.unit_testing_incrament(); // times: 1,2,1,2
    this_tree.unit_testing_incrament(); // times: 1,2,1,3

    this_tree.begin_partition(1); // stack: 3, 1
    expected_entrances[1]++; // entrances: 1,3,1,1
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_entrances(), expected_entrances);

    this_tree.begin_partition(2); // stack: 3, 1, 2
    expected_entrances[2]++; // entrances: 1,3,2,1
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_entrances(), expected_entrances);

    this_tree.unit_testing_incrament(); // times: 1,2,2,3
    this_tree.unit_testing_incrament(); // times: 1,2,3,3
    this_tree.unit_testing_incrament(); // times: 1,2,4,3

    this_tree.end_partition(); // stack: 3, 1
    expected_times[2] += 3.0; // accumulated: 1,2,4,0
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_timers(), expected_times);

    this_tree.end_partition(); // stack: 3
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_timers(), expected_times);

    this_tree.end_partition(); // stack: ~
    expected_times[3] += 3.0; // accumulated: 1,2,4,3
    checkVectorIfRankZero(comm, this_tree.unit_testing_get_timers(), expected_times);
}

TEST(PlatoTimersTree, expectedGracefulFailureFromEmptyStack)
{
    // allocate problem
    const size_t num_keys = 4u;
    const size_t num_timers = num_keys + 1u;
    // define MPI
    MPI_Comm comm = MPI_COMM_WORLD;
    int rank;
    MPI_Comm_rank(comm, &rank);
    // define Tree
    TimersTree this_tree(comm, num_keys);

    // stack : 4
    EXPECT_EQ(this_tree.unit_testing_get_timers().size(), num_timers * (rank==0));
    EXPECT_EQ(this_tree.end_partition(), (rank==0)); // stack : ~

    EXPECT_EQ(this_tree.end_partition(), false); // stack : ~
    EXPECT_EQ(this_tree.end_partition(), false); // stack : ~
}

TEST(PlatoTimersTree, expectedGracefulFailureFromPrematureEnding)
{
    // allocate problem
    const size_t num_keys = 5u;
    const size_t num_timers = num_keys + 1u;
    // define MPI
    MPI_Comm comm = MPI_COMM_WORLD;
    int rank;
    MPI_Comm_rank(comm, &rank);
    // define Tree
    TimersTree this_tree(comm, num_keys);

    // stack : 5

    this_tree.begin_partition(0); // stack : 5, 0
    EXPECT_EQ(this_tree.unit_testing_get_timers().size(), num_timers * (rank==0));

    EXPECT_EQ(this_tree.print_results(), false); // can't print with premature
}

TEST(PlatoTimersTree, expectSuccessAllPartitionEnumMapped)
{
    // if this test fails, it is likely a partition in (timer_partition_t) is not mapped in
    //   (PlatoTimers::get_string_from_partition_enum)'s switch statement.

    // define MPI
    MPI_Comm comm = MPI_COMM_WORLD;
    int rank;
    MPI_Comm_rank(comm, &rank);

    // define Tree
    TimersTree this_tree(comm);

    // print timings
    EXPECT_EQ(this_tree.print_results(), (rank==0));
}

TEST(PlatoTimersTree, expectedGracefulFailureAllPartitionEnumMapped)
{
    // define MPI
    MPI_Comm comm = MPI_COMM_WORLD;
    int rank;
    MPI_Comm_rank(comm, &rank);

    // define Tree
    const int too_many_keys = int(Plato::timer_partition_t::timer_partition_t::TOTAL_NUM_KEYS) + 1;
    TimersTree this_tree(comm, too_many_keys);

    // print timings
    EXPECT_EQ(this_tree.print_results(), false);
}

TEST(PlatoTimersTree, expectedGracefulFailureBeginPartitionOutOfBounds)
{
    // allocate problem
    const size_t num_keys = 5u;
    const size_t num_timers = num_keys + 1u;
    // define MPI
    MPI_Comm comm = MPI_COMM_WORLD;
    int rank;
    MPI_Comm_rank(comm, &rank);
    // define Tree
    TimersTree this_tree(comm, num_keys);

    EXPECT_EQ(this_tree.unit_testing_get_timers().size(), num_timers * (rank==0));

    // out of bound partition
    EXPECT_EQ(this_tree.begin_partition(num_timers), false);
    EXPECT_EQ(this_tree.unit_testing_get_timers().size(), num_timers * (rank==0));
}

template<typename t>
void checkVectorIfRankZero(MPI_Comm& comm, const std::vector<t>& A, const std::vector<t>& B)
{
    int rank;
    MPI_Comm_rank(comm, &rank);
    if(rank == 0)
    {
        ASSERT_EQ(A.size(), B.size());
        const size_t common_size = A.size();

        for(size_t i = 0; i < common_size; i++)
        {
            EXPECT_FLOAT_EQ(A[i], B[i]);
        }
    }
}

}
}
