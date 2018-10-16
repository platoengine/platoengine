#include <gtest/gtest.h>
#include <Kokkos_Core.hpp>
#include <mpi.h>

int main(int argc, char **argv)
{
    MPI_Init(&argc, &argv);
    Kokkos::initialize(argc, argv);

    testing::InitGoogleTest(&argc, argv);
    int returnVal = RUN_ALL_TESTS();

    MPI_Finalize();
    return returnVal;
}
