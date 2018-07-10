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
#include "PSL_Implementation_MeshModular.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Point.hpp"
#include "PSL_SpatialSearcherFactory.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_Abstract_NearestNeighborSearcher.hpp"
#include "PSL_Random.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>
#include <cassert>
#include <cmath>
#include <iostream>
#include <map>

namespace PlatoSubproblemLibrary
{
namespace example
{

ElementBlock::ElementBlock() :
        m_elements(),
        m_nodes(),
        m_processor_then_vector_of_local_and_nonlocal_pairs(),
        m_global_ids(),
        m_global_num_nodes(-1),
        m_global_num_elements(-1),
        m_global_block(NULL)
{
}

ElementBlock::~ElementBlock()
{
    m_elements.clear();
    m_nodes.clear();
    m_processor_then_vector_of_local_and_nonlocal_pairs.clear();
    m_global_ids.clear();
    safe_free(m_global_block);
}

void ElementBlock::build_from_structured_grid(size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                              double x_dist, double y_dist, double z_dist,
                                              int rank, int num_processors)
{
    assert(x_dimen>1);
    assert(y_dimen>1);
    assert(z_dimen>1);

    // allocate
    m_nodes.resize(x_dimen * y_dimen * z_dimen);
    m_elements.resize((x_dimen - 1) * (y_dimen - 1) * (z_dimen - 1));
    m_processor_then_vector_of_local_and_nonlocal_pairs.resize(num_processors);
    const double x_spacing = x_dist / (x_dimen - 1);
    const double y_spacing = y_dist / (y_dimen - 1);
    const double z_spacing = z_dist / (z_dimen - 1);

    // parallel cube of processors
    const size_t parallel_cube_dimension = std::ceil(std::pow(num_processors, 1. / 3.));

    // get processor indexes
    size_t x_processor_index = 0;
    size_t y_processor_index = 0;
    size_t z_processor_index = 0;
    get_xyz_helper(rank,
                   parallel_cube_dimension, parallel_cube_dimension, parallel_cube_dimension,
                   x_processor_index, y_processor_index, z_processor_index);

    // advance spatial starts in each dimension
    const double x_start = x_processor_index * x_dist;
    const double y_start = y_processor_index * y_dist;
    const double z_start = z_processor_index * z_dist;

    // iterate through grid
    m_global_ids.resize(get_num_nodes());
    for(size_t x = 0; x < x_dimen; x++)
    {
        for(size_t y = 0; y < y_dimen; y++)
        {
            for(size_t z = 0; z < z_dimen; z++)
            {
                set_nodes_helper(x,y,z,
                                 x_dimen,y_dimen,z_dimen,
                                 x_start,y_start,z_start,
                                 x_spacing,y_spacing,z_spacing);

                set_elements_helper(x,y,z,
                                    x_dimen,y_dimen,z_dimen,
                                    x_spacing,y_spacing,z_spacing);

                // determine global ids
                const size_t node_index = get_index_helper(x, y, z, x_dimen, y_dimen, z_dimen);
                const size_t this_global_index = get_index_helper(x_processor_index * (x_dimen - 1) + x,
                                                                  y_processor_index * (y_dimen - 1) + y,
                                                                  z_processor_index * (z_dimen - 1) + z,
                                                                  (x_dimen - 1) * parallel_cube_dimension + 1,
                                                                  (y_dimen - 1) * parallel_cube_dimension + 1,
                                                                  (z_dimen - 1) * parallel_cube_dimension + 1);
                m_global_ids[node_index] = this_global_index;
                // NOTE: All global ids across all processors are an uninterrupted sequence of integers
                //   if and only if the number of processors equals the parallel cube dimension cubed.
                //   Otherwise there will be many global indexes say G, such that G will be less than a
                //   global index that exists on some processor, but no node on any processor will have
                //   global index G. TODO: if this matters, make the sequence always uninterrupted.
            }
        }
    }

    // determine shared nodes
    for(int change_processor_x = -1; change_processor_x < 2; change_processor_x++)
    {
        for(int change_processor_y = -1; change_processor_y < 2; change_processor_y++)
        {
            for(int change_processor_z = -1; change_processor_z < 2; change_processor_z++)
            {
                add_shared_nodes_helper(x_dimen,y_dimen,z_dimen,
                                        rank, num_processors, parallel_cube_dimension,
                                        change_processor_x, change_processor_y, change_processor_z,
                                        x_processor_index, y_processor_index, z_processor_index);
            }
        }
    }
}

void ElementBlock::build_random_subset_of_structured_grid(size_t approx_points_per_processor,
                                                          double x_dist, double y_dist, double z_dist,
                                                          AbstractAuthority* authority)
{
    const size_t mpi_size = authority->mpi_wrapper->get_size();

    // determine structured grid size
    size_t global_num_points = mpi_size * approx_points_per_processor;
    double approx_mesh_scale = pow(x_dist * y_dist * z_dist / double(global_num_points), 1. / 3.);
    size_t global_x_dimen = std::max(2., ceil(x_dist/approx_mesh_scale));
    size_t global_y_dimen = std::max(2., ceil(y_dist/approx_mesh_scale));
    size_t global_z_dimen = std::max(2., ceil(z_dist/approx_mesh_scale));

    m_global_block = new ElementBlock;
    m_global_block->build_from_structured_grid(global_x_dimen,global_y_dimen,global_z_dimen,
                                      x_dist, y_dist, z_dist,
                                      0, 1);
    m_global_num_nodes = m_global_block->get_num_nodes();
    m_global_num_elements = m_global_block->get_num_elements();

    std::vector<int> locally_owned_elements;
    determined_locally_owned_elements_for_random_subset(locally_owned_elements,
                                                        x_dist, y_dist, z_dist,
                                                        authority);

    // allocate elements
    const size_t local_num_elements = locally_owned_elements.size();
    m_elements.resize(local_num_elements);

    // mark locally aware nodes
    const size_t global_num_nodes = m_global_block->get_num_nodes();
    std::vector<bool> is_locally_aware(global_num_nodes, false);
    determine_locally_aware_node(is_locally_aware, locally_owned_elements);

    // count local to global id mapping
    size_t local_num_nodes = 0u;
    for(size_t global_node = 0u; global_node < global_num_nodes; global_node++)
    {
        local_num_nodes += (is_locally_aware[global_node]);
    }

    // allocate nodes
    m_global_ids.assign(local_num_nodes, 0u);
    std::map<size_t, size_t> global_to_local_map;
    m_nodes.resize(local_num_nodes);

    // fill nodes
    size_t local_node_counter = 0u;
    std::vector<Element*> empty_vec;
    for(size_t global_node = 0u; global_node < global_num_nodes; global_node++)
    {
        if(is_locally_aware[global_node])
        {
            // fill global ids
            m_global_ids[local_node_counter] = global_node;

            // fill map
            global_to_local_map[global_node] = local_node_counter;

            // fill nodes
            std::vector<double> coordinates = m_global_block->get_node(global_node)->get_coordinates();
            m_nodes[local_node_counter] = Node(this, local_node_counter, empty_vec, coordinates);

            local_node_counter++;
        }
    }

    // fill elements
    for(size_t local_elem = 0u; local_elem < local_num_elements; local_elem++)
    {
        const size_t global_elem_index = locally_owned_elements[local_elem];
        std::vector<Node*> global_adjacent_nodes = m_global_block->get_element(global_elem_index)->get_adjacent_nodes();
        const size_t num_adjacent_nodes = global_adjacent_nodes.size();
        std::vector<Node*> local_adjacent_nodes(num_adjacent_nodes, NULL);
        for(size_t adjacent_node = 0u; adjacent_node < num_adjacent_nodes; adjacent_node++)
        {
            const size_t global_node_id = global_adjacent_nodes[adjacent_node]->get_index();
            const size_t local_node_id = global_to_local_map[global_node_id];
            local_adjacent_nodes[adjacent_node] = &m_nodes[local_node_id];
        }

        m_elements[local_elem] = Element(this, local_elem, local_adjacent_nodes);
    }
}

void ElementBlock::random_perturb_local_nodal_locations(double max_perturb)
{
    // for each local node
    const size_t local_num_nodes = m_nodes.size();
    for(size_t n = 0u; n < local_num_nodes; n++)
    {
        Point* p = m_nodes[n].get_point();

        // for each co-ordinate location
        std::vector<double> pv;
        p->get_data(pv);
        const size_t pd = pv.size();
        for(size_t pdi = 0u; pdi < pd; pdi++)
        {

            // perturb that location
            double perturb = (2.0 * uniform_rand_double() - 1.0) * max_perturb;
            p->set(pdi, pv[pdi] + perturb);
        }
    }
}

size_t ElementBlock::get_num_nodes() const
{
    return m_nodes.size();
}

Node* ElementBlock::get_node(size_t index)
{
    return &m_nodes[index];
}

size_t ElementBlock::get_num_elements() const
{
    return m_elements.size();
}

Element* ElementBlock::get_element(size_t index)
{
    return &m_elements[index];
}

int ElementBlock::get_global_num_nodes() const
{
    return m_global_num_nodes;
}

int ElementBlock::get_global_num_elements() const
{
    return m_global_num_elements;
}

void ElementBlock::get_shared_node_data(std::vector<std::vector<std::pair<size_t, size_t> > >& data)
{
    const size_t size = m_processor_then_vector_of_local_and_nonlocal_pairs.size();
    data.resize(size);
    for(size_t k = 0u; k < size; k++)
    {
        data[k].assign(m_processor_then_vector_of_local_and_nonlocal_pairs[k].begin(),
                       m_processor_then_vector_of_local_and_nonlocal_pairs[k].end());
    }
}
void ElementBlock::get_global_ids(std::vector<size_t>& globals)
{
    globals.assign(m_global_ids.begin(), m_global_ids.end());
}

Point* ElementBlock::get_global_point(size_t index)
{
    if(m_global_block)
    {
        Node* global_node = m_global_block->get_node(index);
        Point* global_point = global_node->get_point();
        return global_point;
    }
    return NULL;
}

size_t ElementBlock::get_index_helper(size_t x_index, size_t y_index, size_t z_index, size_t x_dimen, size_t y_dimen, size_t z_dimen) const
{
    return x_index*(y_dimen*z_dimen)+y_index*(z_dimen)+z_index;
}

void ElementBlock::get_xyz_helper(size_t single_index,
                                  size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                  size_t& x_index, size_t& y_index, size_t& z_index) const
{
    z_index = single_index % z_dimen;
    y_index = ((single_index - z_index) / z_dimen) % y_dimen;
    x_index = (single_index - z_index - y_index * z_dimen) / (y_dimen * z_dimen);
}

void ElementBlock::set_nodes_helper(size_t x_index, size_t y_index, size_t z_index,
                                    size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                    double x_start, double y_start, double z_start,
                                    double x_spacing, double y_spacing, double z_spacing)
{
    const size_t node_index = get_index_helper(x_index, y_index, z_index, x_dimen, y_dimen, z_dimen);

    // iterate through neighboring elements
    std::vector<Element*> adjacent_elements;
    for(int xneigh = -1; xneigh < 1; xneigh++)
    {
        for(int yneigh = -1; yneigh < 1; yneigh++)
        {
            for(int zneigh = -1; zneigh < 1; zneigh++)
            {
                // cannot exceed bounds
                if((xneigh < 0 && x_index == 0) || (xneigh == 0 && x_index + 1 == x_dimen))
                {
                    continue;
                }
                if((yneigh < 0 && y_index == 0) || (yneigh == 0 && y_index + 1 == y_dimen))
                {
                    continue;
                }
                if((zneigh < 0 && z_index == 0) || (zneigh == 0 && z_index + 1 == z_dimen))
                {
                    continue;
                }

                const size_t element_neighbor = get_index_helper(x_index + xneigh,
                                                                 y_index + yneigh,
                                                                 z_index + zneigh,
                                                                 x_dimen - 1,
                                                                 y_dimen - 1,
                                                                 z_dimen - 1);
                adjacent_elements.push_back(&m_elements[element_neighbor]);
            }
        }
    }

    // establish locations
    std::vector<double> coordinates = {x_start + x_spacing * x_index,
                                       y_start + y_spacing * y_index,
                                       z_start + z_spacing * z_index};
    m_nodes[node_index] = Node(this, node_index, adjacent_elements, coordinates);
}

void ElementBlock::set_elements_helper(size_t x_index, size_t y_index, size_t z_index,
                                       size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                       double x_spacing, double y_spacing, double z_spacing)
{
    // if not on outside last, consider element
    if((x_index + 1 < x_dimen) && (y_index + 1 < y_dimen) && (z_index + 1 < z_dimen))
    {
        const size_t element_index = get_index_helper(x_index, y_index, z_index, x_dimen - 1, y_dimen - 1, z_dimen - 1);

        std::vector<Node*> adjacent_nodes;
        for(int xneigh = 0; xneigh < 2; xneigh++)
        {
            for(int yneigh = 0; yneigh < 2; yneigh++)
            {
                for(int zneigh = 0; zneigh < 2; zneigh++)
                {
                    const size_t adjacent_node = get_index_helper(x_index + xneigh,
                                                                  y_index + yneigh,
                                                                  z_index + zneigh,
                                                                  x_dimen,
                                                                  y_dimen,
                                                                  z_dimen);

                    adjacent_nodes.push_back(&m_nodes[adjacent_node]);
                }
            }
        }

        m_elements[element_index] = Element(this, element_index, adjacent_nodes);
    }
}

void ElementBlock::add_shared_nodes_helper(size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                           int rank, size_t num_processors,  size_t parallel_cube_dimension,
                                           int change_processor_x, int change_processor_y, int change_processor_z,
                                           size_t x_processor_index, size_t y_processor_index, size_t z_processor_index)
{
    // if no processor change, skip
    if(change_processor_x == 0 && change_processor_y == 0 && change_processor_z == 0)
    {
        return;
    }

    // if at processor cube boundary, skip
    if(   (change_processor_x < 0 && x_processor_index == 0u)
       || (change_processor_x > 0 && x_processor_index + 1u == parallel_cube_dimension)
       || (change_processor_y < 0 && y_processor_index == 0u)
       || (change_processor_y > 0 && y_processor_index + 1u == parallel_cube_dimension)
       || (change_processor_z < 0 && z_processor_index == 0u)
       || (change_processor_z > 0 && z_processor_index + 1u == parallel_cube_dimension))
    {
        return;
    }

    // determine other processor index
    size_t other_processor = get_index_helper(x_processor_index + change_processor_x,
                                              y_processor_index + change_processor_y,
                                              z_processor_index + change_processor_z,
                                              parallel_cube_dimension,
                                              parallel_cube_dimension,
                                              parallel_cube_dimension);

    // if not a processor, skip
    if(other_processor >= num_processors)
    {
        return;
    }

    //
    // face sharing
    //
    if(change_processor_x == 1 && change_processor_y == 0  && change_processor_z == 0)
    {
        for(size_t y = 1u; y + 1u < y_dimen; y++)
        {
            for(size_t z = 1u; z + 1u < z_dimen; z++)
            {
                size_t local_node = get_index_helper(x_dimen - 1, y, z, x_dimen, y_dimen, z_dimen);
                size_t nonlocal_node = get_index_helper(0u, y, z, x_dimen, y_dimen, z_dimen);
                m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                              nonlocal_node));
            }
        }
    }
    if(change_processor_x == -1 && change_processor_y == 0  && change_processor_z == 0)
    {
        for(size_t y = 1u; y + 1u < y_dimen; y++)
        {
            for(size_t z = 1u; z + 1u < z_dimen; z++)
            {
                size_t local_node = get_index_helper(0u, y, z, x_dimen, y_dimen, z_dimen);
                size_t nonlocal_node = get_index_helper(x_dimen - 1, y, z, x_dimen, y_dimen, z_dimen);
                m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                              nonlocal_node));
            }
        }
    }
    if(change_processor_x == 0 && change_processor_y == 1 && change_processor_z == 0)
    {
        for(size_t x = 1u; x + 1u < x_dimen; x++)
        {
            for(size_t z = 1u; z + 1u < z_dimen; z++)
            {
                size_t local_node = get_index_helper(x, y_dimen - 1, z, x_dimen, y_dimen, z_dimen);
                size_t nonlocal_node = get_index_helper(x, 0u, z, x_dimen, y_dimen, z_dimen);
                m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                              nonlocal_node));
            }
        }
    }
    if(change_processor_x == 0 && change_processor_y == -1 && change_processor_z == 0)
    {
        for(size_t x = 1u; x + 1u < x_dimen; x++)
        {
            for(size_t z = 1u; z + 1u < z_dimen; z++)
            {
                size_t local_node = get_index_helper(x, 0u, z, x_dimen, y_dimen, z_dimen);
                size_t nonlocal_node = get_index_helper(x, y_dimen - 1, z, x_dimen, y_dimen, z_dimen);
                m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                              nonlocal_node));
            }
        }
    }
    if(change_processor_x == 0 && change_processor_y == 0 && change_processor_z == 1)
    {
        for(size_t x = 1u; x + 1u < x_dimen; x++)
        {
            for(size_t y = 1u; y + 1u < y_dimen; y++)
            {
                size_t local_node = get_index_helper(x, y, z_dimen - 1, x_dimen, y_dimen, z_dimen);
                size_t nonlocal_node = get_index_helper(x, y, 0u, x_dimen, y_dimen, z_dimen);
                m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                              nonlocal_node));
            }
        }
    }
    if(change_processor_x == 0 && change_processor_y == 0 && change_processor_z == -1)
    {
        for(size_t x = 1u; x + 1u < x_dimen; x++)
        {
            for(size_t y = 1u; y + 1u < y_dimen; y++)
            {
                size_t local_node = get_index_helper(x, y, 0u, x_dimen, y_dimen, z_dimen);
                size_t nonlocal_node = get_index_helper(x, y, z_dimen - 1, x_dimen, y_dimen, z_dimen);
                m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                              nonlocal_node));
            }
        }
    }

    //
    // edge sharing
    //
    if(change_processor_x >= 0 && change_processor_y >= 0 && change_processor_z == 0)
    {
        for(size_t z = 1u; z + 1u < z_dimen; z++)
        {
            size_t local_node = get_index_helper(x_dimen - 1, y_dimen - 1, z, x_dimen, y_dimen, z_dimen);
            const size_t nonlocal_x = (change_processor_x == 0) * (x_dimen - 1);
            const size_t nonlocal_y = (change_processor_y == 0) * (y_dimen - 1);
            size_t nonlocal_node = get_index_helper(nonlocal_x, nonlocal_y, z, x_dimen, y_dimen, z_dimen);
            m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                          nonlocal_node));
        }
    }
    if(change_processor_x <= 0 && change_processor_y >= 0 && change_processor_z == 0)
    {
        for(size_t z = 1u; z + 1u < z_dimen; z++)
        {
            size_t local_node = get_index_helper(0u, y_dimen - 1, z, x_dimen, y_dimen, z_dimen);
            const size_t nonlocal_x = (change_processor_x == -1) * (x_dimen - 1);
            const size_t nonlocal_y = (change_processor_y == 0) * (y_dimen - 1);
            size_t nonlocal_node = get_index_helper(nonlocal_x, nonlocal_y, z, x_dimen, y_dimen, z_dimen);
            m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                          nonlocal_node));
        }
    }
    if(change_processor_x >= 0 && change_processor_y <= 0 && change_processor_z == 0)
    {
        for(size_t z = 1u; z + 1u < z_dimen; z++)
        {
            size_t local_node = get_index_helper(x_dimen - 1, 0u, z, x_dimen, y_dimen, z_dimen);
            const size_t nonlocal_x = (change_processor_x == 0) * (x_dimen - 1);
            const size_t nonlocal_y = (change_processor_y == -1) * (y_dimen - 1);
            size_t nonlocal_node = get_index_helper(nonlocal_x, nonlocal_y, z, x_dimen, y_dimen, z_dimen);
            m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                          nonlocal_node));
        }
    }
    if(change_processor_x <= 0 && change_processor_y <= 0 && change_processor_z == 0)
    {
        for(size_t z = 1u; z + 1u < z_dimen; z++)
        {
            size_t local_node = get_index_helper(0u, 0u, z, x_dimen, y_dimen, z_dimen);
            const size_t nonlocal_x = (change_processor_x == -1) * (x_dimen - 1);
            const size_t nonlocal_y = (change_processor_y == -1) * (y_dimen - 1);
            size_t nonlocal_node = get_index_helper(nonlocal_x, nonlocal_y, z, x_dimen, y_dimen, z_dimen);
            m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                          nonlocal_node));
        }
    }
    if(change_processor_x >= 0 && change_processor_y == 0 && change_processor_z >= 0)
    {
        for(size_t y = 1u; y + 1u < y_dimen; y++)
        {
            size_t local_node = get_index_helper(x_dimen - 1, y, z_dimen - 1, x_dimen, y_dimen, z_dimen);
            const size_t nonlocal_x = (change_processor_x == 0) * (x_dimen - 1);
            const size_t nonlocal_z = (change_processor_z == 0) * (z_dimen - 1);
            size_t nonlocal_node = get_index_helper(nonlocal_x, y, nonlocal_z, x_dimen, y_dimen, z_dimen);
            m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                          nonlocal_node));
        }
    }
    if(change_processor_x <= 0 && change_processor_y == 0 && change_processor_z >= 0)
    {
        for(size_t y = 1u; y + 1u < y_dimen; y++)
        {
            size_t local_node = get_index_helper(0u, y, z_dimen - 1, x_dimen, y_dimen, z_dimen);
            const size_t nonlocal_x = (change_processor_x == -1) * (x_dimen - 1);
            const size_t nonlocal_z = (change_processor_z == 0) * (z_dimen - 1);
            size_t nonlocal_node = get_index_helper(nonlocal_x, y, nonlocal_z, x_dimen, y_dimen, z_dimen);
            m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                          nonlocal_node));
        }
    }
    if(change_processor_x >= 0 && change_processor_y == 0 && change_processor_z <= 0)
    {
        for(size_t y = 1u; y + 1u < y_dimen; y++)
        {
            size_t local_node = get_index_helper(x_dimen - 1, y, 0u, x_dimen, y_dimen, z_dimen);
            const size_t nonlocal_x = (change_processor_x == 0) * (x_dimen - 1);
            const size_t nonlocal_z = (change_processor_z == -1) * (z_dimen - 1);
            size_t nonlocal_node = get_index_helper(nonlocal_x, y, nonlocal_z, x_dimen, y_dimen, z_dimen);
            m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                          nonlocal_node));
        }
    }
    if(change_processor_x <= 0 && change_processor_y == 0 && change_processor_z <= 0)
    {
        for(size_t y = 1u; y + 1u < y_dimen; y++)
        {
            size_t local_node = get_index_helper(0u, y, 0u, x_dimen, y_dimen, z_dimen);
            const size_t nonlocal_x = (change_processor_x == -1) * (x_dimen - 1);
            const size_t nonlocal_z = (change_processor_z == -1) * (z_dimen - 1);
            size_t nonlocal_node = get_index_helper(nonlocal_x, y, nonlocal_z, x_dimen, y_dimen, z_dimen);
            m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                          nonlocal_node));
        }
    }
    if(change_processor_x == 0 && change_processor_y >= 0 && change_processor_z >= 0)
    {
        for(size_t x = 1u; x + 1u < x_dimen; x++)
        {
            size_t local_node = get_index_helper(x, y_dimen - 1, z_dimen - 1, x_dimen, y_dimen, z_dimen);
            const size_t nonlocal_y = (change_processor_y == 0) * (y_dimen - 1);
            const size_t nonlocal_z = (change_processor_z == 0) * (z_dimen - 1);
            size_t nonlocal_node = get_index_helper(x, nonlocal_y, nonlocal_z, x_dimen, y_dimen, z_dimen);
            m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                          nonlocal_node));
        }
    }
    if(change_processor_x == 0 && change_processor_y <= 0 && change_processor_z >= 0)
    {
        for(size_t x = 1u; x + 1u < x_dimen; x++)
        {
            size_t local_node = get_index_helper(x, 0u, z_dimen - 1, x_dimen, y_dimen, z_dimen);
            const size_t nonlocal_y = (change_processor_y == -1) * (y_dimen - 1);
            const size_t nonlocal_z = (change_processor_z == 0) * (z_dimen - 1);
            size_t nonlocal_node = get_index_helper(x, nonlocal_y, nonlocal_z, x_dimen, y_dimen, z_dimen);
            m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                          nonlocal_node));
        }
    }
    if(change_processor_x == 0 && change_processor_y >= 0 && change_processor_z <= 0)
    {
        for(size_t x = 1u; x + 1u < x_dimen; x++)
        {
            size_t local_node = get_index_helper(x, y_dimen - 1, 0u, x_dimen, y_dimen, z_dimen);
            const size_t nonlocal_y = (change_processor_y == 0) * (y_dimen - 1);
            const size_t nonlocal_z = (change_processor_z == -1) * (z_dimen - 1);
            size_t nonlocal_node = get_index_helper(x, nonlocal_y, nonlocal_z, x_dimen, y_dimen, z_dimen);
            m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                          nonlocal_node));
        }
    }
    if(change_processor_x == 0 && change_processor_y <= 0 && change_processor_z <= 0)
    {
        for(size_t x = 1u; x + 1u < x_dimen; x++)
        {
            size_t local_node = get_index_helper(x, 0u, 0u, x_dimen, y_dimen, z_dimen);
            const size_t nonlocal_y = (change_processor_y == -1) * (y_dimen - 1);
            const size_t nonlocal_z = (change_processor_z == -1) * (z_dimen - 1);
            size_t nonlocal_node = get_index_helper(x, nonlocal_y, nonlocal_z, x_dimen, y_dimen, z_dimen);
            m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node,
                                                                                                          nonlocal_node));
        }
    }

    //
    // vertex sharing
    //
    if(change_processor_x >= 0 && change_processor_y >= 0 && change_processor_z >= 0)
    {
        size_t local_node = get_index_helper(x_dimen - 1, y_dimen - 1, z_dimen - 1, x_dimen, y_dimen, z_dimen);
        const size_t nonlocal_x = (change_processor_x == 0) * (x_dimen - 1);
        const size_t nonlocal_y = (change_processor_y == 0) * (y_dimen - 1);
        const size_t nonlocal_z = (change_processor_z == 0) * (z_dimen - 1);
        size_t nonlocal_node = get_index_helper(nonlocal_x, nonlocal_y, nonlocal_z, x_dimen, y_dimen, z_dimen);
        m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node, nonlocal_node));
    }
    if(change_processor_x <= 0 && change_processor_y >= 0 && change_processor_z >= 0)
    {
        size_t local_node = get_index_helper(0u, y_dimen - 1, z_dimen - 1, x_dimen, y_dimen, z_dimen);
        const size_t nonlocal_x = (change_processor_x == -1) * (x_dimen - 1);
        const size_t nonlocal_y = (change_processor_y == 0) * (y_dimen - 1);
        const size_t nonlocal_z = (change_processor_z == 0) * (z_dimen - 1);
        size_t nonlocal_node = get_index_helper(nonlocal_x, nonlocal_y, nonlocal_z, x_dimen, y_dimen, z_dimen);
        m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node, nonlocal_node));
    }
    if(change_processor_x >= 0 && change_processor_y <= 0 && change_processor_z >= 0)
    {
        size_t local_node = get_index_helper(x_dimen - 1, 0u, z_dimen - 1, x_dimen, y_dimen, z_dimen);
        const size_t nonlocal_x = (change_processor_x == 0) * (x_dimen - 1);
        const size_t nonlocal_y = (change_processor_y == -1) * (y_dimen - 1);
        const size_t nonlocal_z = (change_processor_z == 0) * (z_dimen - 1);
        size_t nonlocal_node = get_index_helper(nonlocal_x, nonlocal_y, nonlocal_z, x_dimen, y_dimen, z_dimen);
        m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node, nonlocal_node));
    }
    if(change_processor_x >= 0 && change_processor_y >= 0 && change_processor_z <= 0)
    {
        size_t local_node = get_index_helper(x_dimen - 1, y_dimen - 1, 0u, x_dimen, y_dimen, z_dimen);
        const size_t nonlocal_x = (change_processor_x == 0) * (x_dimen - 1);
        const size_t nonlocal_y = (change_processor_y == 0) * (y_dimen - 1);
        const size_t nonlocal_z = (change_processor_z == -1) * (z_dimen - 1);
        size_t nonlocal_node = get_index_helper(nonlocal_x, nonlocal_y, nonlocal_z, x_dimen, y_dimen, z_dimen);
        m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node, nonlocal_node));
    }
    if(change_processor_x <= 0 && change_processor_y <= 0 && change_processor_z >= 0)
    {
        size_t local_node = get_index_helper(0u, 0u, z_dimen - 1, x_dimen, y_dimen, z_dimen);
        const size_t nonlocal_x = (change_processor_x == -1) * (x_dimen - 1);
        const size_t nonlocal_y = (change_processor_y == -1) * (y_dimen - 1);
        const size_t nonlocal_z = (change_processor_z == 0) * (z_dimen - 1);
        size_t nonlocal_node = get_index_helper(nonlocal_x, nonlocal_y, nonlocal_z, x_dimen, y_dimen, z_dimen);
        m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node, nonlocal_node));
    }
    if(change_processor_x <= 0 && change_processor_y >= 0 && change_processor_z <= 0)
    {
        size_t local_node = get_index_helper(0u, y_dimen - 1, 0u, x_dimen, y_dimen, z_dimen);
        const size_t nonlocal_x = (change_processor_x == -1) * (x_dimen - 1);
        const size_t nonlocal_y = (change_processor_y == 0) * (y_dimen - 1);
        const size_t nonlocal_z = (change_processor_z == -1) * (z_dimen - 1);
        size_t nonlocal_node = get_index_helper(nonlocal_x, nonlocal_y, nonlocal_z, x_dimen, y_dimen, z_dimen);
        m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node, nonlocal_node));
    }
    if(change_processor_x >= 0 && change_processor_y <= 0 && change_processor_z <= 0)
    {
        size_t local_node = get_index_helper(x_dimen - 1, 0u, 0u, x_dimen, y_dimen, z_dimen);
        const size_t nonlocal_x = (change_processor_x == 0) * (x_dimen - 1);
        const size_t nonlocal_y = (change_processor_y == -1) * (y_dimen - 1);
        const size_t nonlocal_z = (change_processor_z == -1) * (z_dimen - 1);
        size_t nonlocal_node = get_index_helper(nonlocal_x, nonlocal_y, nonlocal_z, x_dimen, y_dimen, z_dimen);
        m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node, nonlocal_node));
    }
    if(change_processor_x <= 0 && change_processor_y <= 0 && change_processor_z <= 0)
    {
        size_t local_node = get_index_helper(0u, 0u, 0u, x_dimen, y_dimen, z_dimen);
        const size_t nonlocal_x = (change_processor_x == -1) * (x_dimen - 1);
        const size_t nonlocal_y = (change_processor_y == -1) * (y_dimen - 1);
        const size_t nonlocal_z = (change_processor_z == -1) * (z_dimen - 1);
        size_t nonlocal_node = get_index_helper(nonlocal_x, nonlocal_y, nonlocal_z, x_dimen, y_dimen, z_dimen);
        m_processor_then_vector_of_local_and_nonlocal_pairs[other_processor].push_back(std::make_pair(local_node, nonlocal_node));
    }
}

void ElementBlock::determined_locally_owned_elements_for_random_subset(std::vector<int>& locally_owned_elements,
                                                                       double x_dist, double y_dist, double z_dist,
                                                                       AbstractAuthority* authority)
{
    const size_t mpi_rank = authority->mpi_wrapper->get_rank();
    const size_t mpi_size = authority->mpi_wrapper->get_size();

    if(mpi_rank == 0u)
    {
        // decide element ownership

        std::vector<Point> rank_centers(mpi_size);
        std::vector<bool> should_refresh_centers(mpi_size, true);
        std::vector<std::vector<int> > owned_elements_per_rank;
        AbstractInterface::NearestNeighborSearcher* searcher = NULL;

        // try these divisions
        const size_t max_division_tries = 100u;
        size_t division_try = 1u;
        bool atleast_one_refresh = true;
        while(division_try <= max_division_tries && atleast_one_refresh)
        {
            // refresh ranks if needed
            for(size_t rank = 0u; rank < mpi_size; rank++)
            {
                if(should_refresh_centers[rank])
                {
                    std::vector<double> rand_vec = {x_dist * uniform_rand_double(), y_dist * uniform_rand_double(), z_dist * uniform_rand_double()};
                    rank_centers[rank] = Point(rank, rand_vec);
                }
            }

            // build point cloud
            PointCloud rank_centers_cloud;
            rank_centers_cloud.assign(rank_centers);

            // build searcher
            searcher = build_nearest_neighbor_searcher(spatial_searcher_t::recommended, authority);
            searcher->build(&rank_centers_cloud);

            // assign global elements to ranks
            owned_elements_per_rank.assign(mpi_size, std::vector<int>(0));
            const size_t num_global_elements = m_global_block->get_num_elements();
            for(size_t elem_index = 0u; elem_index < num_global_elements; elem_index++)
            {
                Point center_point = m_global_block->get_element(elem_index)->get_center();
                size_t neighbor = searcher->get_neighbor(&center_point);
                owned_elements_per_rank[neighbor].push_back(elem_index);
            }

            // set refresh
            atleast_one_refresh = false;
            for(size_t rank = 0u; rank < mpi_size; rank++)
            {
                should_refresh_centers[rank] = (owned_elements_per_rank[rank].size() == 0u);
                atleast_one_refresh |= should_refresh_centers[rank];
            }

            // clean up searcher
            delete searcher;
            searcher = NULL;

            division_try++;
        }

        if(division_try == max_division_tries)
        {
            // could not divide
            authority->utilities->fatal_error("PSL_Implementation_MeshModular could not divide for build_random_subset_of_structured_grid. Aborting.\n");
        }

        locally_owned_elements.assign(owned_elements_per_rank[0].begin(), owned_elements_per_rank[0].end());
        for(size_t rank = 1u; rank < mpi_size; rank++)
        {
            authority->mpi_wrapper->send(rank, int(owned_elements_per_rank[rank].size()));
            authority->mpi_wrapper->send(rank, owned_elements_per_rank[rank]);
        }
    }
    else
    {
        // receive element ownership
        int num_recv = -1;
        authority->mpi_wrapper->receive(0u, num_recv);
        locally_owned_elements.resize(num_recv);
        authority->mpi_wrapper->receive(0u, locally_owned_elements);
    }
}

void ElementBlock::determine_locally_aware_node(std::vector<bool>& is_locally_aware,
                                                const std::vector<int>& locally_owned_elements)
{
    const size_t local_num_elements = locally_owned_elements.size();
    for(size_t local_elem = 0u; local_elem < local_num_elements; local_elem++)
    {
        const size_t global_elem_index = locally_owned_elements[local_elem];
        std::vector<Node*> aware_nodes = m_global_block->get_element(global_elem_index)->get_adjacent_nodes();
        const size_t num_aware_nodes = aware_nodes.size();
        for(size_t aware_node = 0u; aware_node < num_aware_nodes; aware_node++)
        {
            const size_t node_index = aware_nodes[aware_node]->get_index();
            is_locally_aware[node_index] = true;
        }
    }
}

Element::Element(ElementBlock* block, size_t index, const std::vector<Node*>& nodes) :
        m_block(block),
        m_index(index),
        m_nodes(nodes.begin(), nodes.end())
{
}

Element::Element() :
        m_block(NULL),
        m_index(0u),
        m_nodes()
{
}

Element::~Element()
{
    m_block = NULL;
    m_nodes.clear();
}

ElementBlock* Element::get_block()
{
    return m_block;
}

size_t Element::get_index()
{
    return m_index;
}

std::vector<Node*> Element::get_adjacent_nodes()
{
    return m_nodes;
}

Point Element::get_center()
{
    std::vector<double> origin(3, 0.);
    Point center(m_index, origin);

    const size_t num_adjacent_nodes = m_nodes.size();
    for(size_t adj_node = 0u; adj_node < num_adjacent_nodes; adj_node++)
    {
        center = center + (*m_nodes[adj_node]->get_point()) * (1. / double(num_adjacent_nodes));
    }

    return center;
}

Node::Node(ElementBlock* block, size_t index, const std::vector<Element*>& elements, const std::vector<double>& coordinates) :
        m_block(block),
        m_elements(elements.begin(), elements.end()),
        m_point(new Point(index, coordinates))
{
}

Node::Node() :
        m_block(NULL),
        m_elements(),
        m_point(NULL)
{
}

Node::~Node()
{
    m_block = NULL;
    m_elements.clear();
    if(m_point)
    {
        delete m_point;
        m_point = NULL;
    }
}

ElementBlock* Node::get_block() const
{
    return m_block;
}

size_t Node::get_index() const
{
    return m_point->get_index();
}

std::vector<Element*> Node::get_adjacent_element() const
{
    return m_elements;
}

std::vector<double> Node::get_coordinates() const
{
    std::vector<double> coordinates;
    m_point->get_data(coordinates);
    return coordinates;
}

Point* Node::get_point() const
{
    return m_point;
}

Node& Node::operator=(const Node& source)
{
    m_block = source.get_block();
    m_elements = source.get_adjacent_element();
    m_point = new Point(source.get_index(), source.get_coordinates());

    return *this;
}

}
}

