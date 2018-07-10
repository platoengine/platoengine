// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Abstract_MeshScaleAgent.hpp"
#include "PSL_ByOptimizedElementSide_MeshScaleAgent.hpp"
#include "PSL_Implementation_MeshModular.hpp"
#include "PSL_Interface_MeshModular.hpp"
#include "PSL_Abstract_OptimizationMesh.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Random.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <mpi.h>

namespace PlatoSubproblemLibrary
{

namespace TestingMeshScaleAgent
{
void test_mesh_scale(double expected_min, double expected_average, double expected_max, Abstract_MeshScaleAgent* agent);
}

PSL_TEST(MeshScaleAgent,test_mesh_scale_agents)
{
    set_rand_seed();
    AbstractAuthority authority;

    // build mesh
    const int xlen = 3;
    const int ylen = 4;
    const int zlen = 5;
    const double xdist = 3.0;
    const double ydist = 8.0;
    const double zdist = 15.0;
    example::ElementBlock modular_block;
    const size_t rank = authority.mpi_wrapper->get_rank();
    const size_t num_processors = authority.mpi_wrapper->get_size();
    modular_block.build_from_structured_grid(xlen, ylen, zlen, xdist, ydist, zdist, rank, num_processors);
    example::Interface_MeshModular modular_interface;
    modular_interface.set_mesh(&modular_block);

    // set expectations
    const double expected_min = std::min(std::min(xdist / (xlen - 1), ydist / (ylen - 1)), zdist / (zlen - 1));
    const double expected_average = (xdist / (xlen - 1) + ydist / (ylen - 1) + zdist / (zlen - 1)) / 3.0;
    const double expected_max = std::max(std::max(xdist / (xlen - 1), ydist / (ylen - 1)), zdist / (zlen - 1));

    // build agent
    ByOptimizedElementSide_MeshScaleAgent agent(&authority, &modular_interface);

    TestingMeshScaleAgent::test_mesh_scale(expected_min, expected_average, expected_max, &agent);
}

namespace TestingMeshScaleAgent
{

void test_mesh_scale(double expected_min, double expected_average, double expected_max, Abstract_MeshScaleAgent* agent)
{
    const double actual_min = agent->get_mesh_minimum_scale();
    EXPECT_FLOAT_EQ(actual_min, expected_min);
    const double actual_average = agent->get_mesh_average_scale();
    EXPECT_FLOAT_EQ(actual_average, expected_average);
    const double actual_max = agent->get_mesh_maximum_scale();
    EXPECT_FLOAT_EQ(actual_max, expected_max);
}

}

}
