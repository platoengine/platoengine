#include "PSL_UnitTestingHelper.hpp"

#include "PSL_AbstractAuthority.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_SparseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_Abstract_PositiveDefiniteLinearSolver.hpp"
#include "PSL_Random.hpp"

namespace PlatoSubproblemLibrary
{
namespace TestingAbstractAuthority
{

void testAuthority(AbstractAuthority* authority)
{
    authority->utilities->print("hello, world!\n");

    // has objects
    EXPECT_EQ(true, (authority->utilities != NULL));
    EXPECT_EQ(true, (authority->mpi_wrapper != NULL));
    EXPECT_EQ(true, (authority->sparse_builder != NULL));
    EXPECT_EQ(true, (authority->dense_builder != NULL));
    EXPECT_EQ(true, (authority->dense_vector_operations != NULL));
    EXPECT_EQ(true, (authority->dense_solver != NULL));
}
PSL_TEST(AbstractAuthority,basic)
{
    set_rand_seed();

    // no communicator specified
    AbstractAuthority authority0;
    testAuthority(&authority0);

    // communicator specified
    MPI_Comm comm = MPI_COMM_WORLD;
    AbstractAuthority authority1(&comm);
    testAuthority(&authority1);
}

}
}
