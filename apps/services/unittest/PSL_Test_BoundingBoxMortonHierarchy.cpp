// PlatoSubproblemLibraryVersion(5): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_AxisAlignedBoundingBox.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_BoundingBoxMortonHierarchy.hpp"
#include "PSL_Random.hpp"

#include <cmath>

namespace PlatoSubproblemLibrary
{
namespace BoundingBoxMortonHierachyTest
{

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

PSL_TEST(BoundingBoxMortonHierachy,properties)
{
    set_rand_seed();
    // test properties of hierarchy (children contained in parent and number of true AxisAlignedBoundingBox in hierarchy consistent with input)
    const int num_boxes = 1000;
    std::vector<AxisAlignedBoundingBox> test_boxes(num_boxes);

    get_random_boxes_in_unit_cube(test_boxes);

    BoundingBoxMortonHierarchy ref_hierarchy;
    ref_hierarchy.build(test_boxes);

    EXPECT_EQ(true, ref_hierarchy.util_confirm_hierarchy_properties());
}

PSL_TEST(BoundingBoxMortonHierachy,utilitiesGetSizers)
{
    set_rand_seed();
    // test util_get_n_and_m versus various golds
    BoundingBoxMortonHierarchy hierarchy_for_utils;
    int n, m;

    hierarchy_for_utils.util_get_n_and_m(4, n, m);
    EXPECT_EQ(n, 2);
    EXPECT_EQ(m, 0);

    hierarchy_for_utils.util_get_n_and_m(15, n, m);
    EXPECT_EQ(n, 3);
    EXPECT_EQ(m, 7);

    hierarchy_for_utils.util_get_n_and_m((int) std::pow(2., 26) - 1, n, m);
    EXPECT_EQ(n, 25);
    EXPECT_EQ(m, (int)std::pow(2.,25)-1);

    hierarchy_for_utils.util_get_n_and_m((int) std::pow(2., 26), n, m);
    EXPECT_EQ(n, 26);
    EXPECT_EQ(m, 0);

    hierarchy_for_utils.util_get_n_and_m((int) std::pow(2., 26) + 1, n, m);
    EXPECT_EQ(n, 26);
    EXPECT_EQ(m, 1);
}

PSL_TEST(BoundingBoxMortonHierachy,utilitiesSmallestTwoPower)
{
    set_rand_seed();
    // test util_smallest_2_power and util_summed_smallest_2_power versus gold
    BoundingBoxMortonHierarchy hierarchy_for_utils;

    const int n = 4;
    const int smallest_two_power_expect        [16] = {0, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2,  0,  1,  0};
    const int summed_smallest_two_power_expect [16] = {0, 0, 1, 1, 3, 3, 4, 4, 7, 7, 8, 8, 10, 10, 11, 11};

    const int two_to_n = (int)std::pow(2.,n);
    std::vector<int> smallest_two_power_actual(two_to_n, -1);
    std::vector<int> summed_smallest_two_power_actual(two_to_n, -1);

    hierarchy_for_utils.util_smallest_two_power(n, smallest_two_power_actual);
    hierarchy_for_utils.util_summed_smallest_two_power(n,smallest_two_power_actual, summed_smallest_two_power_actual);

    for(int i = 0; i < two_to_n; i++)
    {
        EXPECT_EQ(smallest_two_power_actual[i], smallest_two_power_expect[i]);
        EXPECT_EQ(summed_smallest_two_power_actual[i], summed_smallest_two_power_expect[i]);
    }
}

PSL_TEST(BoundingBoxMortonHierachy,utilitiesIndexesToPlaceTwoNodes)
{
    set_rand_seed();
    // test util_indexes_to_place_two_nodes versus gold, util_is_two_nodes_here, and util_advances_because_of_two_nodes
    BoundingBoxMortonHierarchy hierarchy_for_utils;

    const int n = 3;
    const int two_to_n = 8; // std::pow(2,n);
    const int place_two_nodes_expect [7] = {0,4,2,6,1,5,3};
    const bool is_two_nodes_here_expect [8][8] = {{0,0,0,0,0,0,0,0},
                                                  {1,0,0,0,0,0,0,0},
                                                  {1,0,0,0,1,0,0,0},
                                                  {1,0,1,0,1,0,0,0},
                                                  {1,0,1,0,1,0,1,0},
                                                  {1,1,1,0,1,0,1,0},
                                                  {1,1,1,0,1,1,1,0},
                                                  {1,1,1,1,1,1,1,0}};
    const int advanced_indexes_because_of_two_nodes_expect [8][15] = {{0,1,2,3,4,5,6, 7, 8, 9, 10,11,12,13,14},
                                                                      {0,1,2,3,6,7,8, 9, 10,11,12,13,14,15,16},
                                                                      {0,1,2,3,6,7,8, 9, 10,11,12,15,16,17,18},
                                                                      {0,1,2,3,6,7,8, 11,12,13,14,17,18,19,20},
                                                                      {0,1,2,3,6,7,8, 11,12,13,14,17,18,19,22},
                                                                      {0,1,2,3,6,9,10,13,14,15,16,19,20,21,24},
                                                                      {0,1,2,3,6,9,10,13,14,15,16,19,22,23,26},
                                                                      {0,1,2,3,6,9,10,13,16,17,18,21,24,25,28}};

    std::vector<int> smallest_two_power (two_to_n, -1);
    hierarchy_for_utils.util_smallest_two_power(n,smallest_two_power);
    std::vector<int> summed_smallest_two_power (two_to_n, -1);
    hierarchy_for_utils.util_summed_smallest_two_power(n,smallest_two_power, summed_smallest_two_power);


    std::vector<int> place_two_nodes_actual(two_to_n - 1, -1);
    std::vector<bool> is_two_nodes_here_actual(two_to_n, false);
    std::vector<int> advanced_indexes_because_of_two_nodes_actual((int) std::pow(2., n + 1) - 1, -1);
    for (int m = 0; m < (int)std::pow(2.,n); m++ ) {
        hierarchy_for_utils.util_indexes_to_place_two_nodes(n, m, smallest_two_power, place_two_nodes_actual);
        for(int k = 0; k < m; k++)
        {
            EXPECT_EQ(place_two_nodes_actual[k], place_two_nodes_expect[k]);
        }

        hierarchy_for_utils.util_is_two_nodes_here(n, m, place_two_nodes_actual, is_two_nodes_here_actual);
        for(int k = 0; k < (int) std::pow(2., n) - 1; k++)
        {
            EXPECT_EQ(is_two_nodes_here_actual[k], is_two_nodes_here_expect[m][k]);
        }

        hierarchy_for_utils.util_advanced_indexes_because_of_two_nodes(n,
                                                                       summed_smallest_two_power,
                                                                       is_two_nodes_here_actual,
                                                                       advanced_indexes_because_of_two_nodes_actual);

        for(int k = 0; k < (int) std::pow(2., n + 1) - 1; k++)
        {
            EXPECT_EQ(advanced_indexes_because_of_two_nodes_actual[k], advanced_indexes_because_of_two_nodes_expect[m][k]);
        }
    }
}

PSL_TEST(BoundingBoxMortonHierachy,codeSortSimple)
{
    set_rand_seed();
    // test of util_morton_sort on three bounding boxes; the two boxes close together should be adjacent after the sort
    std::vector<AxisAlignedBoundingBox> test_boxes = {AxisAlignedBoundingBox(0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 0),
                                                             AxisAlignedBoundingBox(0.9, 1.0, 0.9, 1.0, 0.9, 1.0, 1),
                                                             AxisAlignedBoundingBox(0.2, 0.3, 0.2, 0.3, 0.2, 0.3, 2)};
    std::vector<int> sorted_box_access_array(3);

    BoundingBoxMortonHierarchy morton_sorter;
    morton_sorter.util_morton_sort_boxes(3, test_boxes, sorted_box_access_array);

    EXPECT_EQ((1 == test_boxes[sorted_box_access_array[0]].get_id()), false);
    EXPECT_EQ((1 == test_boxes[sorted_box_access_array[1]].get_id()), false);
    EXPECT_EQ(1, test_boxes[sorted_box_access_array[2]].get_id());
}

}
}
