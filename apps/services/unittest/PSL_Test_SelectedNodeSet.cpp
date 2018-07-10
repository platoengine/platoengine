// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Implementation_MeshModular.hpp"
#include "PSL_Interface_MeshModular.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_NodeSet.hpp"
#include "PSL_Selector.hpp"
#include "PSL_Random.hpp"

#include <cstddef>
#include <algorithm>
#include <mpi.h>

namespace PlatoSubproblemLibrary
{

namespace SelectedNodeSetTest
{

PSL_TEST(SelectedNodeSet,boundsOfBlock)
{
    set_rand_seed();
    // build
    const int xlen = 3;
    const int ylen = 4;
    const int zlen = 5;
    const double xdist = 3.0;
    const double ydist = 8.0;
    const double zdist = 15.0;

    // build for modular
    example::ElementBlock modular_block;
    const int rank = 0;
    const int num_processors = 1;
    modular_block.build_from_structured_grid(xlen, ylen, zlen, xdist, ydist, zdist, rank, num_processors);
    example::Interface_MeshModular modular_interface;
    modular_interface.set_mesh(&modular_block);

    Selector selector;
    NodeSet ns;

    // single sides
    selector.set_selection_criteria_x(0.);
    selector.select(&modular_interface, ns);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*zlen));
    ns.clear();
    selector.set_selection_criteria_x(xdist);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*zlen));
    ns.clear();
    selector.set_selection_criteria_y(0.);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(xlen*zlen));
    ns.clear();
    selector.set_selection_criteria_y(ydist);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(xlen*zlen));
    ns.clear();
    selector.set_selection_criteria_z(0.);
    selector.select(&modular_interface, ns);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(xlen*ylen));
    ns.clear();
    selector.set_selection_criteria_z(zdist);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(xlen*ylen));
    ns.clear();

    // opposite pairs
    selector.set_selection_criteria_x(0.);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_x(xdist);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*zlen*2));
    ns.clear();
    selector.set_selection_criteria_y(0.);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_y(ydist);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(xlen*zlen*2));
    ns.clear();
    selector.set_selection_criteria_z(0.);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_z(zdist);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*xlen*2));
    ns.clear();

    // shared pairs
    selector.set_selection_criteria_x(0.);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_y(0.);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*zlen+xlen*zlen-zlen));
    ns.clear();
    selector.set_selection_criteria_x(0.);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_y(ydist);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*zlen+xlen*zlen-zlen));
    ns.clear();
    selector.set_selection_criteria_x(xdist);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_y(0.);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*zlen+xlen*zlen-zlen));
    ns.clear();
    selector.set_selection_criteria_x(xdist);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_y(ydist);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*zlen+xlen*zlen-zlen));
    ns.clear();

    selector.set_selection_criteria_z(0.);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_y(0.);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*xlen+xlen*zlen-xlen));
    ns.clear();
    selector.set_selection_criteria_z(0.);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_y(ydist);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*xlen+xlen*zlen-xlen));
    ns.clear();
    selector.set_selection_criteria_z(zdist);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_y(0.);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*xlen+xlen*zlen-xlen));
    ns.clear();
    selector.set_selection_criteria_z(zdist);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_y(ydist);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*xlen+xlen*zlen-xlen));
    ns.clear();

    selector.set_selection_criteria_z(0.);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_x(0.);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*xlen+ylen*zlen-ylen));
    ns.clear();
    selector.set_selection_criteria_z(0.);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_x(xdist);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*xlen+ylen*zlen-ylen));
    ns.clear();
    selector.set_selection_criteria_z(zdist);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_x(0.);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*xlen+ylen*zlen-ylen));
    ns.clear();
    selector.set_selection_criteria_z(zdist);
    selector.select(&modular_interface, ns);
    selector.set_selection_criteria_x(xdist);
    selector.select(&modular_interface, ns);
    EXPECT_EQ(ns.size(), size_t(ylen*xlen+ylen*zlen-ylen));
    ns.clear();
}

}

}

