#include <gtest/gtest.h>
#include <mpi.h>

int main(int argc, char **argv)
{
    MPI_Init(&argc, &argv);

    testing::InitGoogleTest(&argc, argv);
    int returnVal = RUN_ALL_TESTS();

    MPI_Finalize();
    return returnVal;
}
