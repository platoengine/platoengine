// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_BoundingBoxMortonHierarchy.hpp"

#include "PSL_AxisAlignedBoundingBox.hpp"

#include <algorithm> // for min, max
#include <iostream> // for cout
#include <utility> // for pair
#include <math.h>
#include <cmath>
#include <vector>
#include <stdio.h> // for NULL
#include <cstddef>
#include <cassert>

namespace PlatoSubproblemLibrary
{

BoundingBoxMortonHierarchy::BoundingBoxMortonHierarchy() :
        AbstractInterface::OverlapSearcher(),
        m_hierarchy(),
        m_num_boxes(0)
{
}

BoundingBoxMortonHierarchy::~BoundingBoxMortonHierarchy()
{
}

void BoundingBoxMortonHierarchy::build(const std::vector<AxisAlignedBoundingBox>& answer_boxes)
{
    const int num_boxes = answer_boxes.size();
    assert(num_boxes>0);

    // get array of sorted index, input_boxes remain in place
    std::vector<int> sorted_indexes(num_boxes);
    this->util_morton_sort_boxes(num_boxes, answer_boxes, sorted_indexes);

    m_num_boxes = num_boxes;
    m_hierarchy.assign(2 * m_num_boxes - 1, AxisAlignedBoundingBox());

    // num_boxes = 2^n + m , where n is as large as possible such that m is non-negative
    int n, m;
    this->util_get_n_and_m(m_num_boxes, n, m);

    std::vector<int> smallest_two_powers(pow(2, n));
    std::vector<int> summed_smallest_two_powers(pow(2, n));
    std::vector<int> place_two_nodes(m);
    std::vector<bool> is_two_nodes_here(pow(2, n), false);
    std::vector<int> index_advance_because_of_two_nodes(pow(2, n + 1) - 1);

    this->util_smallest_two_power(n, smallest_two_powers);
    this->util_summed_smallest_two_power(n, smallest_two_powers, summed_smallest_two_powers);
    this->util_indexes_to_place_two_nodes(n, m, smallest_two_powers, place_two_nodes);
    this->util_is_two_nodes_here(n, m, place_two_nodes, is_two_nodes_here);
    this->util_advanced_indexes_because_of_two_nodes(n,
                                                     summed_smallest_two_powers,
                                                     is_two_nodes_here,
                                                     index_advance_because_of_two_nodes);

    int base_step = 1;
    int this_step_increment = 1;
    int input_boxes_index = 0;

    // construct hierarchy
    for(int row_base = n; row_base >= 0; row_base--)
    {
        const int two_to_the_row_base = (int) pow(2, row_base);
        for(int in_row = 0; in_row < two_to_the_row_base; in_row++)
        {
            const int hierarchy_index_to_assign = row_base + in_row * base_step + summed_smallest_two_powers[in_row];
            const int advanced_hierachy_index_to_assign = index_advance_because_of_two_nodes[hierarchy_index_to_assign];

            // if processing leaf nodes
            if(row_base == n)
            {
                if(is_two_nodes_here[in_row])
                {
                    m_hierarchy[advanced_hierachy_index_to_assign + 1] = answer_boxes[sorted_indexes[input_boxes_index]];

                    m_hierarchy[advanced_hierachy_index_to_assign + 2] = answer_boxes[sorted_indexes[input_boxes_index + 1]];

                    m_hierarchy[advanced_hierachy_index_to_assign] =
                            AxisAlignedBoundingBox(answer_boxes[sorted_indexes[input_boxes_index]],
                                                   answer_boxes[sorted_indexes[input_boxes_index + 1]]);
                    m_hierarchy[advanced_hierachy_index_to_assign].set_id(-2);

                    input_boxes_index += 2;
                }
                else
                {
                    m_hierarchy[advanced_hierachy_index_to_assign] = answer_boxes[sorted_indexes[input_boxes_index]];

                    input_boxes_index++;
                }
            }
            else
            {
                const int advanced_right_child_index_to_assign = index_advance_because_of_two_nodes[hierarchy_index_to_assign
                                                                                                    + this_step_increment];
                m_hierarchy[advanced_hierachy_index_to_assign] =
                        AxisAlignedBoundingBox(m_hierarchy[advanced_hierachy_index_to_assign + 1],
                                               m_hierarchy[advanced_right_child_index_to_assign]);
                m_hierarchy[advanced_hierachy_index_to_assign].set_id(advanced_hierachy_index_to_assign
                                                                      - advanced_right_child_index_to_assign);

            }
        }
        this_step_increment *= 2;
        base_step += this_step_increment;
    }
}

void BoundingBoxMortonHierarchy::get_overlaps(AxisAlignedBoundingBox* query_box,
                                              std::vector<size_t>& neighbors_buffer,
                                              size_t& num_neighbors)
{
    assert(m_num_boxes>0);
    const size_t search_start = 0;
    this->internal_solve(query_box, neighbors_buffer, num_neighbors, search_start);
}

// public utilities

void BoundingBoxMortonHierarchy::util_get_n_and_m(const int N, int& n, int& m) const
{
    n = floor(log(N) / log(2));
    m = N - (int) pow(2, n);
}

// assuming smallest_two_powers is an array of int's at least size 2^n
void BoundingBoxMortonHierarchy::util_smallest_two_power(const int n, std::vector<int>& smallest_two_powers) const
{
    smallest_two_powers[0] = 0;
    const int two_to_the_n = (int) pow(2, n);

    int two_powers = 1;
    for(int placement = 0; placement < n; placement++)
    {
        for(int i = 0; two_powers * (1 + 2 * i) < two_to_the_n; i++)
        {
            smallest_two_powers[two_powers * (1 + 2 * i)] = placement;
        }
        two_powers *= 2;
    }
}

// assuming smallest_two_powers is an array of int's at least size 2^n
void BoundingBoxMortonHierarchy::util_summed_smallest_two_power(const int n,
                                                                const std::vector<int>& smallest_two_powers,
                                                                std::vector<int>& summed_smallest_two_powers) const
{
    summed_smallest_two_powers[0] = 0;
    const int two_to_the_n = (int) pow(2, n);
    for(int i = 1; i < two_to_the_n; i++)
    {
        summed_smallest_two_powers[i] = summed_smallest_two_powers[i - 1] + smallest_two_powers[i];
    }
}

// assuming place_two_nodes is array of int's of at least size m
void BoundingBoxMortonHierarchy::util_indexes_to_place_two_nodes(const int n,
                                                                 const int m,
                                                                 const std::vector<int>& smallest_two_powers,
                                                                 std::vector<int>& place_two_nodes) const
{
    if(m == 0)
        return;

    std::vector<int> tmp_for_cumsum(m / 2);

    for(int i = 0; i < m / 2; i++)
    {
        tmp_for_cumsum[i] = 3 * (int) pow(2, (n - 2) - smallest_two_powers[i + 1]);
    }

    const int spacer = (int) pow(2, n - 1);
    const int mod = (int) pow(2, n);

    place_two_nodes[0] = 0;
    for(int i = 1; i < m; i++)
    {
        if(i % 2 == 1)
        {
            place_two_nodes[i] = (place_two_nodes[i - 1] + spacer) % mod;
        }
        else
        {
            place_two_nodes[i] = (place_two_nodes[i - 1] + tmp_for_cumsum[(i / 2) - 1]) % mod;
        }
    }
}

// assuming advanced_indexes_because_of_two_nodes is an array of int's of at least size 2^(n+1) - 1
void BoundingBoxMortonHierarchy::util_advanced_indexes_because_of_two_nodes(const int n,
                                                                            const std::vector<int>& summed_smallest_two_power,
                                                                            const std::vector<bool>& is_two_nodes_here,
                                                                            std::vector<int>& advanced_indexes_because_of_two_nodes) const
{
    std::vector<int> tmp_for_cumsum((int) pow(2, n + 1) - 1, 0);

    const int tmp_specify_loop_size = pow(2, n);
    for(int in_row = 0; in_row < tmp_specify_loop_size; in_row++)
    {
        if(is_two_nodes_here[in_row])
        {
            const int hierarchy_index_to_assign = n + in_row + summed_smallest_two_power[in_row];
            tmp_for_cumsum[hierarchy_index_to_assign] = 2;
        }
    }

    const int cum_sum_loop_size = pow(2, n + 1) - 1;
    advanced_indexes_because_of_two_nodes[0] = 0;
    for(int i = 1; i < cum_sum_loop_size; i++)
    {
        advanced_indexes_because_of_two_nodes[i] = advanced_indexes_because_of_two_nodes[i - 1] + tmp_for_cumsum[i - 1];
        advanced_indexes_because_of_two_nodes[i - 1] += i - 1;
    }
    advanced_indexes_because_of_two_nodes[cum_sum_loop_size - 1] += cum_sum_loop_size - 1;
}

// assuming is_two_here is array of bool's of at least size 2^n
void BoundingBoxMortonHierarchy::util_is_two_nodes_here(const int n,
                                                        const int m,
                                                        const std::vector<int>& place_two_nodes,
                                                        std::vector<bool>& is_two_nodes_here) const
{
    for(int i = 0; i < m; i++)
    {
        is_two_nodes_here[place_two_nodes[i]] = true;
    }
}

void BoundingBoxMortonHierarchy::util_morton_sort_boxes(const int num_boxes,
                                                        const std::vector<AxisAlignedBoundingBox>& input_boxes,
                                                        std::vector<int>& sorted_indexes) const
{
    std::vector<std::pair<unsigned int, int> > morton_code_and_index(num_boxes);

    float x_min = input_boxes[0].get_x_min();
    float x_max = x_min;
    float y_min = input_boxes[0].get_y_min();
    float y_max = y_min;
    float z_min = input_boxes[0].get_z_min();
    float z_max = z_min;
    for(int i = 1; i < num_boxes; i++)
    {
        x_min = std::min(x_min, input_boxes[i].get_x_min());
        x_max = std::max(x_max, input_boxes[i].get_x_min());
        y_min = std::min(y_min, input_boxes[i].get_y_min());
        y_max = std::max(y_max, input_boxes[i].get_y_min());
        z_min = std::min(z_min, input_boxes[i].get_z_min());
        z_max = std::max(z_max, input_boxes[i].get_z_min());
    }

    // prevent division by zero
    const float epsilon = 0.0001;
    x_max = std::max(x_max, x_min + epsilon);
    y_max = std::max(y_max, y_min + epsilon);
    z_max = std::max(z_max, z_min + epsilon);

    for(int i = 0; i < num_boxes; i++)
    {
        const unsigned int morton = this->internal_morton3D((input_boxes[i].get_x_min() - x_min) / (x_max - x_min),
                                                            (input_boxes[i].get_y_min() - y_min) / (y_max - y_min),
                                                            (input_boxes[i].get_z_min() - z_min) / (z_max - z_min));
        morton_code_and_index[i] = std::make_pair(morton, i);
    }

    std::sort(&morton_code_and_index[0], &morton_code_and_index[num_boxes]);
    for(int i = 0; i < num_boxes; i++)
    {
        sorted_indexes[i] = morton_code_and_index[i].second;
    }
}

bool BoundingBoxMortonHierarchy::util_confirm_hierarchy_properties() const
{
    assert(m_num_boxes > 0);
    for(int i = 0; i < 2 * m_num_boxes - 1; i++)
    {
        if(m_hierarchy[i].get_id() < 0)
        {
            if(!m_hierarchy[i].contains(m_hierarchy[i + 1]) || !m_hierarchy[i].contains(m_hierarchy[i + 1]))
            {
                return false;
            }
            if(!m_hierarchy[i].contains(m_hierarchy[i - m_hierarchy[i].get_id()])
               || !m_hierarchy[i].contains(m_hierarchy[i - m_hierarchy[i].get_id()]))
            {
                return false;
            }
        }
    }
    const int num_boxes_found = this->internal_confirm_hierarchy_properties(0);
    return (num_boxes_found == m_num_boxes);
}

unsigned int BoundingBoxMortonHierarchy::internal_expand_bits(unsigned int v) const
{
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

unsigned int BoundingBoxMortonHierarchy::internal_morton3D(float x, float y, float z) const
{
    x = std::min(std::max(x * 1024.0f, 0.0f), 1023.0f);
    y = std::min(std::max(y * 1024.0f, 0.0f), 1023.0f);
    z = std::min(std::max(z * 1024.0f, 0.0f), 1023.0f);
    const unsigned int xx = this->internal_expand_bits((unsigned int) x);
    const unsigned int yy = this->internal_expand_bits((unsigned int) y);
    const unsigned int zz = this->internal_expand_bits((unsigned int) z);
    return xx * 4 + yy * 2 + zz;
}

void BoundingBoxMortonHierarchy::internal_solve(AxisAlignedBoundingBox* query_box,
                                                std::vector<size_t>& neighbors_buffer,
                                                size_t& num_neighbors,
                                                size_t search_start) const
{
    if(m_hierarchy[search_start].overlap(*query_box))
    {
        if(m_hierarchy[search_start].get_id() >= 0)
        {
            neighbors_buffer[num_neighbors++] = m_hierarchy[search_start].get_id();
        }
        else
        {
            this->internal_solve(query_box, neighbors_buffer, num_neighbors, search_start + 1);
            this->internal_solve(query_box, neighbors_buffer, num_neighbors, search_start - m_hierarchy[search_start].get_id());
        }
    }
}

int BoundingBoxMortonHierarchy::internal_confirm_hierarchy_properties(const int i) const
{
    // count the number of reachable leaf nodes of tree
    if(m_hierarchy[i].get_id() >= 0)
    {
        return 1;
    }
    else
    {
        return internal_confirm_hierarchy_properties(i + 1) + internal_confirm_hierarchy_properties(i - m_hierarchy[i].get_id());
    }
}

}
