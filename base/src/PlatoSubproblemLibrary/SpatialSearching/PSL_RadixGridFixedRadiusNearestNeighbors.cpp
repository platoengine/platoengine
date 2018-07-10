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
#include "PSL_RadixGridFixedRadiusNearestNeighbors.hpp"

#include "PSL_Point.hpp"
#include "PSL_PointCloud.hpp"

#include <cstddef>
#include <map>
#include <vector>
#include <limits>

namespace PlatoSubproblemLibrary
{

RadixGridFixedRadiusNearestNeighbors::RadixGridFixedRadiusNearestNeighbors() :
        AbstractInterface::FixedRadiusNearestNeighborsSearcher(),
        m_answer_points(),
        m_radius(-1.)
{
}

RadixGridFixedRadiusNearestNeighbors::~RadixGridFixedRadiusNearestNeighbors()
{
    m_answer_points = NULL;
    m_radix_grid.clear();
}

// save answer points
void RadixGridFixedRadiusNearestNeighbors::build(PlatoSubproblemLibrary::PointCloud* answer_points, double radius)
{
    m_answer_points = answer_points;
    m_radius = radius;

    double min_x = 0.;
    double max_x = 0.;
    double min_y = 0.;
    double max_y = 0.;
    double min_z = 0.;
    double max_z = 0.;

    const size_t num_points = m_answer_points->get_num_points();
    if(num_points > 0u)
    {
        min_x = (*m_answer_points->get_point(0u))(0);
        max_x = min_x;
        min_y = (*m_answer_points->get_point(0u))(1);
        max_y = min_y;
        min_z = (*m_answer_points->get_point(0u))(2);
        max_z = min_z;
    }
    for(size_t point_index = 1u; point_index < num_points; point_index++)
    {
        PlatoSubproblemLibrary::Point* this_point = m_answer_points->get_point(point_index);

        const double x = (*this_point)(0);
        const double y = (*this_point)(1);
        const double z = (*this_point)(2);

        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);
    }

    m_min_x_domain = min_x - m_radius;
    m_max_x_domain = max_x + m_radius;
    m_min_y_domain = min_y - m_radius;
    m_max_y_domain = max_y + m_radius;
    m_min_z_domain = min_z - m_radius;
    m_max_z_domain = max_z + m_radius;

    m_radix_step_x = m_radius;
    m_radix_step_y = m_radius;
    m_radix_step_z = m_radius;

    if(double(m_radix_step_x * std::numeric_limits<size_t>::max()) < m_max_x_domain - m_min_x_domain)
    {
        m_radix_step_x = (m_max_x_domain - m_min_x_domain) / (std::numeric_limits<size_t>::max() - 1024u);
    }
    if(double(m_radix_step_y * std::numeric_limits<size_t>::max()) < m_max_y_domain - m_min_y_domain)
    {
        m_radix_step_y = (m_max_y_domain - m_min_y_domain) / (std::numeric_limits<size_t>::max() - 1024u);
    }
    if(double(m_radix_step_z * std::numeric_limits<size_t>::max()) < m_max_z_domain - m_min_z_domain)
    {
        m_radix_step_z = (m_max_z_domain - m_min_z_domain) / (std::numeric_limits<size_t>::max() - 1024u);
    }

    for(size_t point_index = 0u; point_index < num_points; point_index++)
    {
        PlatoSubproblemLibrary::Point* this_point = m_answer_points->get_point(point_index);

        const double x = (*this_point)(0);
        const double y = (*this_point)(1);
        const double z = (*this_point)(2);

        const size_t radix_x = size_t((x - m_min_x_domain) / m_radix_step_x);
        const size_t radix_y = size_t((y - m_min_y_domain) / m_radix_step_y);
        const size_t radix_z = size_t((z - m_min_z_domain) / m_radix_step_z);

        m_radix_grid[std::make_pair(std::make_pair(radix_x, radix_y), radix_z)].push_back(this_point);
    }
}

// find neighbors of query point within radius
void RadixGridFixedRadiusNearestNeighbors::get_neighbors(PlatoSubproblemLibrary::Point* query_point,
                                                          std::vector<size_t>& neighbors_buffer,
                                                          size_t& num_neighbors)
{
    const double x = (*query_point)(0);
    const double y = (*query_point)(1);
    const double z = (*query_point)(2);

    if((x < m_min_x_domain) || (y < m_min_y_domain) || (z < m_min_z_domain) || (m_max_x_domain < x) || (m_max_y_domain < y)
       || (m_max_z_domain < z))
    {
        return;
    }

    const size_t radix_x = size_t((x - m_min_x_domain) / m_radix_step_x);
    const size_t radix_y = size_t((y - m_min_y_domain) / m_radix_step_y);
    const size_t radix_z = size_t((z - m_min_z_domain) / m_radix_step_z);

    const size_t radix_x_begin = (radix_x == 0u ? 0u : radix_x - 1);
    const size_t radix_y_begin = (radix_y == 0u ? 0u : radix_y - 1);
    const size_t radix_z_begin = (radix_z == 0u ? 0u : radix_z - 1);

    std::map<std::pair<std::pair<size_t, size_t>, size_t>, std::vector<PlatoSubproblemLibrary::Point*> >::iterator map_iter;
    for(size_t answer_radix_x = radix_x_begin; answer_radix_x <= radix_x + 1u; answer_radix_x++)
    {
        for(size_t answer_radix_y = radix_y_begin; answer_radix_y <= radix_y + 1u; answer_radix_y++)
        {
            for(size_t answer_radix_z = radix_z_begin; answer_radix_z <= radix_z + 1u; answer_radix_z++)
            {
                map_iter = m_radix_grid.find(std::make_pair(std::make_pair(answer_radix_x, answer_radix_y), answer_radix_z));
                if(map_iter != m_radix_grid.end())
                {
                    const size_t bin_size = map_iter->second.size();
                    for(size_t bin_index = 0u; bin_index < bin_size; bin_index++)
                    {
                        PlatoSubproblemLibrary::Point* pt = map_iter->second[bin_index];
                        if(query_point->distance(pt) <= m_radius)
                        {
                            neighbors_buffer[num_neighbors++] = pt->get_index();
                        }
                    }
                }
            }
        }
    }
}

}
