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

// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_AxisAlignedBoundingBox.hpp"
#include "PSL_Random.hpp"

namespace PlatoSubproblemLibrary
{
namespace AxisAlignedBoundBoxTest
{

PSL_TEST(AxisAlignedBoundBox,overlapAndContains)
{
    set_rand_seed();
    // small internal box
    AxisAlignedBoundingBox boxc(0.25, 0.75, 0.25, 0.75, 0.25, 0.75);
    // large external box
    AxisAlignedBoundingBox boxgB(-0.1, 1.1, -0.1, 1.1, -0.1, 1.1);
    // tight external box
    AxisAlignedBoundingBox boxgS(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

    const int num_boxes = 8;
    AxisAlignedBoundingBox boxes[num_boxes];
    boxes[0] = AxisAlignedBoundingBox(0, 0.45, 0, 0.45, 0, 0.45);
    boxes[1] = AxisAlignedBoundingBox(0.55, 1, 0, 0.45, 0, 0.45);
    boxes[2] = AxisAlignedBoundingBox(0, 0.45, 0.55, 1, 0, 0.45);
    boxes[3] = AxisAlignedBoundingBox(0.55, 1, 0.55, 1, 0, 0.45);
    boxes[4] = AxisAlignedBoundingBox(0, 0.45, 0, 0.45, 0.55, 1);
    boxes[5] = AxisAlignedBoundingBox(0.55, 1, 0, 0.45, 0.55, 1);
    boxes[6] = AxisAlignedBoundingBox(0, 0.45, 0.55, 1, 0.55, 1);
    boxes[7] = AxisAlignedBoundingBox(0.55, 1, 0.55, 1, 0.55, 1);

    for(int i = 0; i < num_boxes; i++)
    {
        for(int j = 0; j < num_boxes; j++)
        {
            if(i == j)
            {
                // check self overlap and contains
                EXPECT_EQ(boxes[i].overlap(boxes[j]), true);
                EXPECT_EQ(boxes[i].contains(boxes[j]), true);
            }
            else
            {
                // check non-overlap in array boxes
                EXPECT_EQ(boxes[i].overlap(boxes[j]), false);
                EXPECT_EQ(boxes[i].contains(boxes[j]), false);
            }
        }
    }

    for(int i = 0; i < num_boxes; i++)
    {
        // check array boxes contained in larger boxes
        EXPECT_EQ(boxgB.contains(boxes[i]), true);
        EXPECT_EQ(boxgS.contains(boxes[i]), true);

        // check larger boxes not contained in array boxes
        EXPECT_EQ(boxes[i].contains(boxgB), false);
        EXPECT_EQ(boxes[i].contains(boxgS), false);

        // check array boxes overlap with small internal box
        EXPECT_EQ(boxes[i].overlap(boxc), true);
        EXPECT_EQ(boxc.overlap(boxes[i]), true);

        // check array boxes not contained nor contains small internal box
        EXPECT_EQ(boxes[i].contains(boxc), false);
        EXPECT_EQ(boxc.contains(boxes[i]), false);
    }
}

PSL_TEST(AxisAlignedBoundBox,grow)
{
    set_rand_seed();
    // small internal box
    AxisAlignedBoundingBox small_box(0.4, 0.45, 0.2, 0.25, 0.1, 0.15);
    // big external box
    AxisAlignedBoundingBox big_box(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

    EXPECT_EQ(big_box.overlap(small_box), true);
    EXPECT_EQ(big_box.contains(small_box), true);

    // grow a little
    small_box.grow_in_each_axial_direction(0.09);
    EXPECT_EQ(big_box.overlap(small_box), true);
    EXPECT_EQ(big_box.contains(small_box), true);

    // grow too much
    small_box.grow_in_each_axial_direction(0.02);
    EXPECT_EQ(big_box.overlap(small_box), true);
    EXPECT_EQ(big_box.contains(small_box), false);

    // check final status
    EXPECT_FLOAT_EQ(small_box.get_x_min(), 0.4-0.11);
    EXPECT_FLOAT_EQ(small_box.get_x_max(), 0.45+0.11);
    EXPECT_FLOAT_EQ(small_box.get_y_min(), 0.2-0.11);
    EXPECT_FLOAT_EQ(small_box.get_y_max(), 0.25+0.11);
    EXPECT_FLOAT_EQ(small_box.get_z_min(), 0.1-0.11);
    EXPECT_FLOAT_EQ(small_box.get_z_max(), 0.15+0.11);
}

}
}
