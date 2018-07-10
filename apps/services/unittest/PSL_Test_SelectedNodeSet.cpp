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

