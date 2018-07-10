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

#include "PSL_Point.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Abstract_FixedRadiusNearestNeighborsSearcher.hpp"
#include "PSL_RadixGridFixedRadiusNearestNeighbors.hpp"
#include "PSL_BruteForceFixedRadiusNearestNeighbors.hpp"
#include "PSL_SpatialSearcherFactory.hpp"
#include "PSL_Random.hpp"

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{

namespace FixedRadiusNearestNeighborsSearchesTest
{

void get_random_points_in_unit_cube(std::vector<PlatoSubproblemLibrary::Point>& test_points);
void rigorous_search_comparison(AbstractInterface::FixedRadiusNearestNeighborsSearcher* searcher);
void handle_zero_radius(spatial_searcher_t::spatial_searcher_t searcher_type);

PSL_TEST(FixedRadiusNearestNeighborsSearches,simpleRadixGrid)
{
    set_rand_seed();
    // possible answers
    std::vector<double> a0_data = {0., 0., 0.};
    std::vector<double> a1_data = {1., 0., 0.};
    std::vector<double> a2_data = {0., 1., 0.};
    std::vector<double> a3_data = {0., 0., 1.1};
    std::vector<double> a4_data = {0., 0., 1.3};
    std::vector<PlatoSubproblemLibrary::Point> answer_points = {PlatoSubproblemLibrary::Point(0u, a0_data),
                                                            PlatoSubproblemLibrary::Point(1u, a1_data),
                                                            PlatoSubproblemLibrary::Point(2u, a2_data),
                                                            PlatoSubproblemLibrary::Point(3u, a3_data),
                                                            PlatoSubproblemLibrary::Point(4u, a4_data)};

    // build point cloud
    PlatoSubproblemLibrary::PointCloud answer_point_cloud;
    answer_point_cloud.assign(answer_points);
    const size_t num_answer_points = answer_point_cloud.get_num_points();

    // build searcher
    const double radius = 1.;
    RadixGridFixedRadiusNearestNeighbors searcher;
    searcher.build(&answer_point_cloud, radius);

    // allocate results
    std::vector<size_t> results(num_answer_points);
    size_t num_results = 0;

    // query point
    std::vector<double> q0_data = {.5, .5, .5};
    PlatoSubproblemLibrary::Point query0(0u, q0_data);
    // get results
    num_results = 0;
    searcher.get_neighbors(&query0, results, num_results);
    // test expectations
    ASSERT_EQ(num_results, 4u);
    std::sort(&results[0], &results[num_results]);
    EXPECT_EQ(results[0], 0u);
    EXPECT_EQ(results[1], 1u);
    EXPECT_EQ(results[2], 2u);
    EXPECT_EQ(results[3], 3u);

    // query point
    std::vector<double> q1_data = {0., 0., 1.5};
    PlatoSubproblemLibrary::Point query1(0u, q1_data);
    // get results
    num_results = 0;
    searcher.get_neighbors(&query1, results, num_results);
    // test expectations
    ASSERT_EQ(num_results, 2u);
    std::sort(&results[0], &results[num_results]);
    EXPECT_EQ(results[0], 3u);
    EXPECT_EQ(results[1], 4u);

    // query point
    std::vector<double> q2_data = {-2., 3., -5.};
    PlatoSubproblemLibrary::Point query2(0u, q2_data);
    // get results
    num_results = 0;
    searcher.get_neighbors(&query2, results, num_results);
    // test expectations
    EXPECT_EQ(num_results, 0u);
}

void get_random_points_in_unit_cube(std::vector<PlatoSubproblemLibrary::Point>& test_points)
{
    const size_t num_points = test_points.size();
    for(size_t i = 0; i < num_points; i++)
    {
        std::vector<double> data = {uniform_rand_double(), uniform_rand_double(), uniform_rand_double()};
        test_points[i] = PlatoSubproblemLibrary::Point(i, data);
    }
}

void rigorous_search_comparison(AbstractInterface::FixedRadiusNearestNeighborsSearcher* searcher)
{
    // compare a search method to a locally rolled brute force search

    // create example sets of points for testing
    const size_t num_points = 100;
    const double radius = 0.02;

    std::vector<Point> points_A(num_points);
    get_random_points_in_unit_cube(points_A);
    PlatoSubproblemLibrary::PointCloud point_cloud_A;
    point_cloud_A.assign(points_A);

    std::vector<Point> points_B(num_points);
    get_random_points_in_unit_cube(points_B);
    PlatoSubproblemLibrary::PointCloud point_cloud_B;
    point_cloud_B.assign(points_B);

    std::vector<size_t> explicit_results(num_points);
    size_t num_explicit_results = 0;
    std::vector<size_t> other_results(num_points);
    size_t num_other_results = 0;

    // create other search method
    searcher->build(&point_cloud_B, radius);

    for(size_t i = 0; i < num_points; i++)
    {
        // compute overlaps by brute force
        num_explicit_results = 0;
        for(size_t j = 0; j < num_points; j++)
        {
            if(points_A[i].distance(&points_B[j]) <= radius)
            {
                explicit_results[num_explicit_results++] = j;
            }
        }

        // compute overlaps by other search method
        num_other_results = 0u;
        searcher->get_neighbors(&points_A[i], other_results, num_other_results);

        std::sort(&other_results[0], &other_results[num_other_results]);

        ASSERT_EQ(num_other_results, num_explicit_results);
        for(size_t j = 0; j < num_explicit_results; j++)
        {
            EXPECT_EQ(other_results[j], explicit_results[j]);
        }
    }
}

PSL_TEST(FixedRadiusNearestNeighborsSearches,rigorousRadixGrid)
{
    set_rand_seed();
    RadixGridFixedRadiusNearestNeighbors searcher;
    rigorous_search_comparison(&searcher);
}

PSL_TEST(FixedRadiusNearestNeighborsSearches,rigorousBruteForce)
{
    set_rand_seed();
    BruteForceFixedRadiusNearestNeighbors searcher;
    rigorous_search_comparison(&searcher);
}

void handle_zero_radius(spatial_searcher_t::spatial_searcher_t searcher_type)
{
    const size_t num_answer_points = 100u;
    const double radius = 0.;

    // answer points
    std::vector<Point> answer_points(num_answer_points);
    get_random_points_in_unit_cube(answer_points);
    std::vector<double> last_data = {1.5, 1.6, 1.7};
    answer_points.push_back(PlatoSubproblemLibrary::Point(num_answer_points, last_data));
    PlatoSubproblemLibrary::PointCloud point_cloud;
    point_cloud.assign(answer_points);

    AbstractInterface::FixedRadiusNearestNeighborsSearcher* searcher =
            build_fixed_radius_nearest_neighbors_searcher(searcher_type, NULL);

    // build searcher
    searcher->build(&point_cloud, radius);
    // allocate results
    std::vector<size_t> results(num_answer_points);
    size_t num_results = 0;

    // query last
    num_results = 0;
    searcher->get_neighbors(&answer_points.back(), results, num_results);
    // test expectations
    ASSERT_EQ(num_results, 1u);
    EXPECT_EQ(results[0], num_answer_points);

    // build outside point
    std::vector<double> outside_data = {2.1, 3.2, -5.7};
    PlatoSubproblemLibrary::Point outside_point(num_answer_points+1u, outside_data);
    // query outside
    num_results = 0;
    searcher->get_neighbors(&outside_point, results, num_results);
    // test expectations
    ASSERT_EQ(num_results, 0u);

    delete searcher;
}

PSL_TEST(FixedRadiusNearestNeighborsSearches,handleZeroRadius_recommended)
{
    set_rand_seed();
    handle_zero_radius(spatial_searcher_t::spatial_searcher_t::recommended);
}

PSL_TEST(FixedRadiusNearestNeighborsSearches,handleZeroRadius_boundingBoxBruteForce)
{
    set_rand_seed();
    handle_zero_radius(spatial_searcher_t::spatial_searcher_t::bounding_box_brute_force);
}

PSL_TEST(FixedRadiusNearestNeighborsSearches,handleZeroRadius_boundingBoxMortonHierarchy)
{
    set_rand_seed();
    handle_zero_radius(spatial_searcher_t::spatial_searcher_t::bounding_box_morton_hierarchy);
}

PSL_TEST(FixedRadiusNearestNeighborsSearches,handleZeroRadius_bruteForceFixedRadiusNearestNeighbors)
{
    set_rand_seed();
    handle_zero_radius(spatial_searcher_t::spatial_searcher_t::brute_force_fixed_radius_nearest_neighbors);
}

PSL_TEST(FixedRadiusNearestNeighborsSearches,handleZeroRadius_radixGridFixedRadiusNearestNeighbors)
{
    set_rand_seed();
    handle_zero_radius(spatial_searcher_t::spatial_searcher_t::radix_grid_fixed_radius_nearest_neighbors);
}

}

}
