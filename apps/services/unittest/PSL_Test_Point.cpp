// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Point.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Random.hpp"

namespace PlatoSubproblemLibrary
{
namespace TestingPoint
{

PSL_TEST(Point,allocation)
{
    set_rand_seed();
    // allocate
    Point p(0u, {0.0,uniform_rand_double(),1.0});

    // test
    EXPECT_EQ(p.dimension(), 3u);
    EXPECT_EQ(p(0), 0.0);
    EXPECT_EQ(p(2), 1.0);

    // mutate
    p.set(0u, {0.1,-0.1});

    // test
    EXPECT_EQ(p.dimension(), 2u);
    EXPECT_EQ(p(0), 0.1);
    EXPECT_EQ(p(1), -0.1);
}

}
}
