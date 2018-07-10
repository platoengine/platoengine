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
#include "PSL_Implementation_MeshMonolithic.hpp"

#include <vector>
#include <cstddef>
#include <cassert>
#include <iostream>

namespace PlatoSubproblemLibrary
{
namespace example
{

MeshMonolithic::MeshMonolithic() :
        m_node_index_to_adjacent_elements(),
        m_element_index_to_adjacent_nodes(),
        m_node_locations()
{
}

MeshMonolithic::~MeshMonolithic()
{
    m_node_index_to_adjacent_elements.clear();
    m_element_index_to_adjacent_nodes.clear();
    m_node_locations.clear();
}

void MeshMonolithic::build_from_structured_grid(size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                                double x_dist, double y_dist, double z_dist)
{
    assert(x_dimen>1);
    assert(y_dimen>1);
    assert(z_dimen>1);

    // allocate
    m_node_index_to_adjacent_elements.resize(x_dimen * y_dimen * z_dimen, std::vector<size_t>(8u, 0u));
    m_element_index_to_adjacent_nodes.resize((x_dimen - 1) * (y_dimen - 1) * (z_dimen - 1), std::vector<size_t>(8u, 0u));
    m_node_locations.resize(x_dimen * y_dimen * z_dimen, std::vector<double>(3u, -1.0));
    const double x_spacing = x_dist / (x_dimen - 1);
    const double y_spacing = y_dist / (y_dimen - 1);
    const double z_spacing = z_dist / (z_dimen - 1);

    // iterate through grid
    for(size_t x = 0; x < x_dimen; x++)
    {
        for(size_t y = 0; y < y_dimen; y++)
        {
            for(size_t z = 0; z < z_dimen; z++)
            {
                set_node_locations_helper(x,y,z,x_dimen,y_dimen,z_dimen,x_spacing,y_spacing,z_spacing);

                set_node_index_to_adjacent_elements_helper(x,y,z,x_dimen,y_dimen,z_dimen,x_spacing,y_spacing,z_spacing);

                set_element_index_to_adjacent_nodes_helper(x,y,z,x_dimen,y_dimen,z_dimen,x_spacing,y_spacing,z_spacing);
            }
        }
    }
}

size_t MeshMonolithic::get_num_nodes() const
{
    return m_node_index_to_adjacent_elements.size();
}

size_t MeshMonolithic::get_num_elements() const
{
    return m_element_index_to_adjacent_nodes.size();
}

std::vector<size_t> MeshMonolithic::get_adjacent_nodes(size_t element_index) const
{
    return m_element_index_to_adjacent_nodes[element_index];
}

std::vector<size_t> MeshMonolithic::get_adjacent_elements(size_t node_index) const
{
    return m_node_index_to_adjacent_elements[node_index];
}

std::vector<double> MeshMonolithic::get_node_location(size_t node_index) const
{
    return m_node_locations[node_index];
}

size_t MeshMonolithic::get_index_helper(size_t x_index, size_t y_index, size_t z_index, size_t x_dimen, size_t y_dimen, size_t z_dimen) const
{
    return x_index*(y_dimen*z_dimen)+y_index*(z_dimen)+z_index;
}

void MeshMonolithic::set_node_index_to_adjacent_elements_helper(size_t x_index, size_t y_index, size_t z_index,
                                                                size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                                                double x_spacing, double y_spacing, double z_spacing)
{
    const size_t node_index = get_index_helper(x_index, y_index, z_index, x_dimen, y_dimen, z_dimen);

    // establish number of missing neighboring elements
    int missing_neighbors = 0;
    const int num_extreme = (x_index == 0 || x_index + 1 == x_dimen) + (y_index == 0 || y_index + 1 == y_dimen)
                            + (z_index == 0 || z_index + 1 == z_dimen);
    switch(num_extreme)
    {
        case 1:
            missing_neighbors = 4;
            break;
        case 2:
            missing_neighbors = 6;
            break;
        case 3:
            missing_neighbors = 7;
            break;
        default:
            break;
    }
    if(missing_neighbors > 0)
    {
        m_node_index_to_adjacent_elements[node_index].resize(8 - missing_neighbors);
    }

    // iterate through neighboring elements
    int neighbor_index = 0;
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
                m_node_index_to_adjacent_elements[node_index][neighbor_index++] = element_neighbor;
            }
        }
    }
}

void MeshMonolithic::set_element_index_to_adjacent_nodes_helper(size_t x_index, size_t y_index, size_t z_index,
                                                                size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                                                double x_spacing, double y_spacing, double z_spacing)
{
    // if not on outside last, consider element
    if((x_index + 1 < x_dimen) && (y_index + 1 < y_dimen) && (z_index + 1 < z_dimen))
    {
        const size_t element_index = get_index_helper(x_index, y_index, z_index, x_dimen - 1, y_dimen - 1, z_dimen - 1);

        size_t neighbor_counter = 0u;
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

                    m_element_index_to_adjacent_nodes[element_index][neighbor_counter++] = adjacent_node;
                }
            }
        }
    }
}

void MeshMonolithic::set_node_locations_helper(size_t x_index, size_t y_index, size_t z_index,
                                               size_t x_dimen, size_t y_dimen, size_t z_dimen,
                                               double x_spacing, double y_spacing, double z_spacing)
{
    const size_t node_index = get_index_helper(x_index, y_index, z_index, x_dimen, y_dimen, z_dimen);

    // establish locations
    m_node_locations[node_index] = {x_spacing * x_index, y_spacing * y_index, z_spacing * z_index};
}

}
}
