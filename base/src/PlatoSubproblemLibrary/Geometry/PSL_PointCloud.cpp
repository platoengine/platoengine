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
#include "PSL_PointCloud.hpp"

#include "PSL_Point.hpp"
#include "PSL_AxisAlignedBoundingBox.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

PointCloud::PointCloud()
    : m_points() {

}

PointCloud::PointCloud(PointCloud* other, const std::vector<size_t>& indexes_to_transfer)
    : m_points(indexes_to_transfer.size())
{
    const size_t num_indexes_to_transfer = indexes_to_transfer.size();
    for(size_t i = 0u; i < num_indexes_to_transfer; i++)
    {
        m_points[i] = *other->get_point(indexes_to_transfer[i]);
    }
}

PointCloud::~PointCloud() {

}

void PointCloud::resize(size_t num_points)
{
    m_points.resize(num_points);
}

void PointCloud::assign(std::vector<Point>& points)
{
    m_points.assign(points.begin(), points.end());
}

void PointCloud::assign(size_t index, const Point& point)
{
    m_points[index] = point;
}

void PointCloud::push_back(const Point& point)
{
    m_points.push_back(point);
}

size_t PointCloud::get_num_points() const
{
    return m_points.size();
}

Point* PointCloud::get_point(size_t index)
{
    return &m_points[index];
}

AxisAlignedBoundingBox PointCloud::get_bound()
{
    AxisAlignedBoundingBox result(0., 0., 0., 0u);

    // if no points, return origin
    if(m_points.size() == 0u)
    {
        return result;
    }

    // build result to contain each point
    result.set(&m_points[0]);
    const size_t num_points = m_points.size();
    for(size_t index = 1u; index < num_points; index++)
    {
        result.grow_to_include(&m_points[index]);
    }

    return result;
}

}
