// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Triangle.hpp"
#include "PSL_Point.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Random.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace TestingPoint
{

PSL_TEST(Triangle,construct)
{
    set_rand_seed();

    // allocate
    Point p0(0u, {0.1,0.2,0.3});
    Point p1(1u, {0.4,0.5,-1.2});
    Point p2(2u, {0.4,-0.1,0.0});

    // make triangle
    const size_t tri_index = 7u;
    Triangle t(tri_index,&p0,&p1,&p2);

    // check data
    EXPECT_EQ(tri_index,t.get_index());
    EXPECT_EQ(t.get_vertex(0),&p0);
    EXPECT_EQ(t.get_vertex(1),&p1);
    EXPECT_EQ(t.get_vertex(2),&p2);

    // check distance
    EXPECT_NEAR(0.,t.get_vertex(0)->distance(&p0),1e-8);
    EXPECT_NEAR(0.,t.get_vertex(1)->distance(&p1),1e-8);
    EXPECT_NEAR(0.,t.get_vertex(2)->distance(&p2),1e-8);
    EXPECT_GT(t.get_vertex(0)->distance(&p2),0.5);
}

}
}
