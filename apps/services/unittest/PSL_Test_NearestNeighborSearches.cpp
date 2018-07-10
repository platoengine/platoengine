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
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Abstract_NearestNeighborSearcher.hpp"
#include "PSL_SpatialSearcherFactory.hpp"
#include "PSL_BruteForceNearestNeighbor.hpp"
#include "PSL_Random.hpp"

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{

namespace NearestNeighborSearchesTest
{

void simple_points(spatial_searcher_t::spatial_searcher_t searcher_type);
void get_random_points_in_unit_cube(std::vector<PlatoSubproblemLibrary::Point>& test_points);
void rigorous_search_comparison(AbstractInterface::NearestNeighborSearcher* searcher);

void simple_points(spatial_searcher_t::spatial_searcher_t searcher_type)
{
    // set answers
    std::vector<PlatoSubproblemLibrary::Point> answer_points(3u);
    answer_points[0u] = PlatoSubproblemLibrary::Point(0u, {0., 0.});
    answer_points[1u] = PlatoSubproblemLibrary::Point(1u, {1., 0.});
    answer_points[2u] = PlatoSubproblemLibrary::Point(2u, {0., 2.});
    PlatoSubproblemLibrary::PointCloud answer_cloud;
    answer_cloud.assign(answer_points);

    // build searcher
    AbstractInterface::NearestNeighborSearcher* searcher =
            build_nearest_neighbor_searcher(searcher_type, NULL);
    searcher->build(&answer_cloud);

    size_t neighbor = 0u;

    // test expectations

    // assuming uniqueness, you are closest to yourself
    const size_t num_answer_points = answer_points.size();
    for(size_t answer_index = 0u; answer_index < num_answer_points; answer_index++)
    {
        neighbor = searcher->get_neighbor(&answer_points[answer_index]);
        EXPECT_EQ(neighbor, answer_index);
    }

    // shifts near to perpendicular bisector
    PlatoSubproblemLibrary::Point query_a(3u, {0.49, 0.});
    neighbor = searcher->get_neighbor(&query_a);
    EXPECT_EQ(neighbor, 0u);
    PlatoSubproblemLibrary::Point query_b(4u, {0.51, 0.});
    neighbor = searcher->get_neighbor(&query_b);
    EXPECT_EQ(neighbor, 1u);
    PlatoSubproblemLibrary::Point query_c(5u, {0., 0.99});
    neighbor = searcher->get_neighbor(&query_c);
    EXPECT_EQ(neighbor, 0u);
    PlatoSubproblemLibrary::Point query_d(6u, {0., 1.01});
    neighbor = searcher->get_neighbor(&query_d);
    EXPECT_EQ(neighbor, 2u);

    // shifts near to the circle's center
    PlatoSubproblemLibrary::Point query_e(6u, {4./9., 8./9.});
    neighbor = searcher->get_neighbor(&query_e);
    EXPECT_EQ(neighbor, 0u);
    PlatoSubproblemLibrary::Point query_f(6u, {5./9., 8./9.});
    neighbor = searcher->get_neighbor(&query_f);
    EXPECT_EQ(neighbor, 1u);
    PlatoSubproblemLibrary::Point query_g(6u, {4./9., 10./9.});
    neighbor = searcher->get_neighbor(&query_g);
    EXPECT_EQ(neighbor, 2u);

    delete searcher;
}

PSL_TEST(NearestNeighborSearches,simplePoints_recommended)
{
    set_rand_seed();
    simple_points(spatial_searcher_t::spatial_searcher_t::recommended);
}

PSL_TEST(NearestNeighborSearches,simplePoints_bruteForceNearestNeighbor)
{
    set_rand_seed();
    simple_points(spatial_searcher_t::spatial_searcher_t::brute_force_nearest_neighbor);
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

void rigorous_search_comparison(AbstractInterface::NearestNeighborSearcher* searcher)
{
    // compare a search method to a locally rolled brute force search

    // create example sets of points for testing
    const size_t num_points = 100;

    std::vector<Point> points_A(num_points);
    get_random_points_in_unit_cube(points_A);
    PlatoSubproblemLibrary::PointCloud point_cloud_A;
    point_cloud_A.assign(points_A);

    std::vector<Point> points_B(num_points);
    get_random_points_in_unit_cube(points_B);
    PlatoSubproblemLibrary::PointCloud point_cloud_B;
    point_cloud_B.assign(points_B);

    size_t explicit_result = 0u;
    size_t other_result = 0u;

    // create other search method
    searcher->build(&point_cloud_B);

    for(size_t i = 0; i < num_points; i++)
    {
        Point* this_point = &points_A[i];

        // compute by brute force
        explicit_result = 0u;
        double explicit_min_distance = this_point->distance(&points_B[0u]);
        for(size_t j = 1u; j < num_points; j++)
        {
            double distance = this_point->distance(&points_B[j]);
            if(distance < explicit_min_distance)
            {
                explicit_result = j;
                explicit_min_distance = distance;
            }
        }

        // compute overlaps by other search method
        other_result = searcher->get_neighbor(&points_A[i]);

        EXPECT_EQ(explicit_result, other_result);
    }
}

PSL_TEST(NearestNeighborSearches,rigorousBruteForce)
{
    set_rand_seed();
    BruteForceNearestNeighbor searcher;
    rigorous_search_comparison(&searcher);
}

}

}
