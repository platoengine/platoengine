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

// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Point.hpp"
#include "PSL_AxisAlignedBoundingBox.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Abstract_OverlapSearcher.hpp"
#include "PSL_BoundingBoxBruteForce.hpp"
#include "PSL_BoundingBoxMortonHierarchy.hpp"
#include "PSL_Random.hpp"

#include <cmath>
#include <vector>

namespace PlatoSubproblemLibrary
{
namespace BoundingBoxSearchesTest
{

void get_random_boxes_in_unit_cube(std::vector<AxisAlignedBoundingBox>& test_boxes);
void rigorous_search_comparison(AbstractInterface::OverlapSearcher* searcher);
std::vector<AxisAlignedBoundingBox> GenerateCylinderBoxArrayForSearches(const Point base,
                                                                        const Point cyl_dir,
                                                                        const Point cyl_dir_perp1,
                                                                        const Point cyl_dir_perp2,
                                                                        const double z_quad_len,
                                                                        const double cyl_radius,
                                                                        const int z_num_layers,
                                                                        const int t_num_layers);
void cyliner_search(AbstractInterface::OverlapSearcher* searcher);

void get_random_boxes_in_unit_cube(std::vector<AxisAlignedBoundingBox>& test_boxes)
{
    const size_t num_boxes = test_boxes.size();
    for(size_t i = 0; i < num_boxes; i++)
    {
        const float x0 = uniform_rand_double();
        const float y0 = uniform_rand_double();
        const float z0 = uniform_rand_double();
        const float x1 = x0 + 0.1 * uniform_rand_double();
        const float y1 = y0 + 0.1 * uniform_rand_double();
        const float z1 = z0 + 0.1 * uniform_rand_double();
        test_boxes[i] = AxisAlignedBoundingBox(x0, x1, y0, y1, z0, z1, i);
    }
}

void rigorous_search_comparison(AbstractInterface::OverlapSearcher* searcher)
{
    // compare a search method to a locally rolled brute force search

    // create example sets of boxes for testing
    const size_t num_boxes = 100;
    std::vector<AxisAlignedBoundingBox> boxes_A(num_boxes);
    get_random_boxes_in_unit_cube(boxes_A);
    std::vector<AxisAlignedBoundingBox> boxes_B(num_boxes);
    get_random_boxes_in_unit_cube(boxes_B);

    std::vector<size_t> explicit_results(num_boxes);
    size_t num_explicit_results = 0;
    std::vector<size_t> other_results(num_boxes);
    size_t num_other_results = 0;

    // create other search method
    searcher->build(boxes_B);

    for(size_t i = 0; i < num_boxes; i++)
    {
        // compute overlaps by brute force
        num_explicit_results = 0;
        for(size_t j = 0; j < num_boxes; j++)
        {
            if(boxes_A[i].overlap(boxes_B[j]))
            {
                explicit_results[num_explicit_results++] = j;
            }
        }

        // compute overlaps by other search method
        num_other_results = 0u;
        searcher->get_overlaps(&boxes_A[i], other_results, num_other_results);

        std::sort(&other_results[0], &other_results[num_other_results]);

        ASSERT_EQ(num_other_results, num_explicit_results);
        for(size_t j = 0; j < num_explicit_results; j++)
        {
            EXPECT_EQ(other_results[j], explicit_results[j]);
        }
    }
}

PSL_TEST(BoundingBoxSearches,rigorousBoundingBoxMortonHierarchy)
{
    set_rand_seed();
    BoundingBoxMortonHierarchy searcher;
    rigorous_search_comparison(&searcher);
}

PSL_TEST(BoundingBoxSearches,rigorousBoundingBoxBruteForce)
{
    set_rand_seed();
    BoundingBoxBruteForce searcher;
    rigorous_search_comparison(&searcher);
}

std::vector<AxisAlignedBoundingBox> GenerateCylinderBoxArrayForSearches(const Point base,
                                                                        const Point cyl_dir,
                                                                        const Point cyl_dir_perp1,
                                                                        const Point cyl_dir_perp2,
                                                                        const double z_quad_len,
                                                                        const double cyl_radius,
                                                                        const int z_num_layers,
                                                                        const int t_num_layers)
{
    std::vector<AxisAlignedBoundingBox> returnBoxes;
    returnBoxes.reserve(z_num_layers * t_num_layers);
    int boxNum = 0;

    for(int zi = 0; zi < z_num_layers; zi++)
    {
        for(int ti = 0; ti < t_num_layers; ti++)
        {
            const Point p0 = base + cyl_dir * zi * z_quad_len + cyl_dir_perp1 * cyl_radius * cos(ti * M_PI * 2.0 / t_num_layers)
                             + cyl_dir_perp2 * cyl_radius * sin(ti * M_PI * 2.0 / t_num_layers);

            const Point p1 = base + cyl_dir * zi * z_quad_len
                             + cyl_dir_perp1 * cyl_radius * cos((ti + 1) * M_PI * 2.0 / t_num_layers)
                             + cyl_dir_perp2 * cyl_radius * sin((ti + 1) * M_PI * 2.0 / t_num_layers);

            const Point p2 = base + cyl_dir * (zi + 1) * z_quad_len
                             + cyl_dir_perp1 * cyl_radius * cos((ti + 1) * M_PI * 2.0 / t_num_layers)
                             + cyl_dir_perp2 * cyl_radius * sin((ti + 1) * M_PI * 2.0 / t_num_layers);

            const Point p3 = base + cyl_dir * (zi + 1) * z_quad_len
                             + cyl_dir_perp1 * cyl_radius * cos(ti * M_PI * 2.0 / t_num_layers)
                             + cyl_dir_perp2 * cyl_radius * sin(ti * M_PI * 2.0 / t_num_layers);

            const double x_min = std::min(std::min(p0(0), p1(0)), std::min(p2(0), p3(0)));
            const double x_max = std::max(std::max(p0(0), p1(0)), std::max(p2(0), p3(0)));
            const double y_min = std::min(std::min(p0(1), p1(1)), std::min(p2(1), p3(1)));
            const double y_max = std::max(std::max(p0(1), p1(1)), std::max(p2(1), p3(1)));
            const double z_min = std::min(std::min(p0(2), p1(2)), std::min(p2(2), p3(2)));
            const double z_max = std::max(std::max(p0(2), p1(2)), std::max(p2(2), p3(2)));

            returnBoxes.push_back(AxisAlignedBoundingBox(x_min, x_max, y_min, y_max, z_min, z_max, boxNum++));
        }
    }

    return returnBoxes;
}

void cyliner_search(AbstractInterface::OverlapSearcher* searcher)
{
    // for a single Bounding box overlap problem, check that overlaps reported were true overlaps found
    const size_t z_num_layers = 100;
    const size_t t_num_layers = 15;

    const Point base(0u, {0.0, 0.0, 0.0});
    const Point cyl_dir(0u, {0.0, 0.0, 1.0});
    const Point cyl_dir_perp1(0u, {1.0, 0.0, 0.0});
    const Point cyl_dir_perp2(0u, {0.0, 1.0, 0.0});
    const double z_len = 25.0;
    const double z_quad_len = z_len / z_num_layers;
    const double cyl_radius = 10.0;

    std::vector<AxisAlignedBoundingBox> cylBoxArray1 = GenerateCylinderBoxArrayForSearches(base,
                                                                                           cyl_dir,
                                                                                           cyl_dir_perp1,
                                                                                           cyl_dir_perp2,
                                                                                           z_quad_len,
                                                                                           cyl_radius,
                                                                                           z_num_layers,
                                                                                           t_num_layers);

    const Point base_2(0u, {0.5 * z_len, 0.0, 0.5 * z_len});
    const Point cyl_dir_2(0u, {-1.0, 0, 0});
    const Point cyl_dir_perp1_2(0u, {0.0, 1.0, 0.0});
    const Point cyl_dir_perp2_2(0u, {0.0, 0.0, 1.0});

    std::vector<AxisAlignedBoundingBox> cylBoxArray2 = GenerateCylinderBoxArrayForSearches(base_2,
                                                                                           cyl_dir_2,
                                                                                           cyl_dir_perp1_2,
                                                                                           cyl_dir_perp2_2,
                                                                                           z_quad_len,
                                                                                           cyl_radius,
                                                                                           z_num_layers,
                                                                                           t_num_layers);

    searcher->build(cylBoxArray1);

    std::vector<size_t> results(cylBoxArray1.size());
    size_t num_results = 0;

    const size_t size_cylBoxArray2 = cylBoxArray2.size();
    for(size_t j = 0; j < size_cylBoxArray2; j++)
    {
        num_results = 0;

        searcher->get_overlaps(&cylBoxArray2[j], results, num_results);

        EXPECT_EQ(size_t(cylBoxArray2[j].get_id()), j);

        for(size_t i = 0; i < num_results; i++)
        {
            EXPECT_EQ(size_t(cylBoxArray1[results[i]].get_id()), results[i]);

            EXPECT_EQ(true, cylBoxArray2[j].overlap(cylBoxArray1[results[i]]));
        }
    }
}

PSL_TEST(BoundingBoxSearches,cylinderBoundingBoxMortonHierarchy)
{
    set_rand_seed();
    BoundingBoxMortonHierarchy searcher;
    cyliner_search(&searcher);
}

PSL_TEST(BoundingBoxSearches,cylinderBoundingBoxBruteForce)
{
    set_rand_seed();
    BoundingBoxBruteForce searcher;
    cyliner_search(&searcher);
}

}

}
