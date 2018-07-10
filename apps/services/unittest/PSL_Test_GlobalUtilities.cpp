// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Random.hpp"

#include <iostream>
#include <vector>

namespace PlatoSubproblemLibrary
{
namespace GlobalUtilitiesTesting
{

PSL_TEST(GlobalUtilities, basic)
{
    set_rand_seed();
    example::Interface_BasicGlobalUtilities interface;

    // test print
    interface.print("a pleasant message\n");

    // test that an abort has not yet occurred
    EXPECT_EQ(interface.get_ignored_abort(), false);

    // disable abort
    interface.set_never_abort(true);
    interface.fatal_error("a fatal message\n");

    // test that an abort should have occurred
    EXPECT_EQ(interface.get_ignored_abort(), true);
    interface.set_never_abort(false);
}

PSL_TEST(GlobalUtilities, vectors)
{
    set_rand_seed();
    example::Interface_BasicGlobalUtilities interface;
    AbstractInterface::GlobalUtilities* abs_interface = &interface;

    // print some vectors
    const std::vector<double> v_double = {0.1, -3.0, 4.2};
    abs_interface->print(v_double, false);

    // print some vectors
    const std::vector<bool> v_bool = {true, false, true, false};
    abs_interface->print(v_bool, true);

    // print some vectors
    const std::vector<double> v_int = {-3, 1, 4, 2, -1, 0, 7};
    abs_interface->print(v_int, true);
}

}
}
