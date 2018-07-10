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
#include "PSL_Abstract_OverlapSearcher.hpp"

#include "PSL_Abstract_FixedRadiusNearestNeighborsSearcher.hpp"
#include "PSL_Point.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_AxisAlignedBoundingBox.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{

OverlapSearcher::OverlapSearcher() :
        FixedRadiusNearestNeighborsSearcher(),
        m_radius(-1)
{
}

OverlapSearcher::~OverlapSearcher()
{
}

// build searcher
void OverlapSearcher::build(PlatoSubproblemLibrary::PointCloud* answer_points, double radius)
{
    m_radius = radius;

    // build boxes
    const size_t num_points = answer_points->get_num_points();
    std::vector<AxisAlignedBoundingBox> answer_boxes(num_points);
    for(size_t index = 0u; index < num_points; index++)
    {
        answer_boxes[index].set(answer_points->get_point(index));
    }

    // build searcher from boxes
    this->build(answer_boxes);
}

// find neighbors within radius
void OverlapSearcher::get_neighbors(PlatoSubproblemLibrary::Point* query_point, std::vector<size_t>& neighbors_buffer, size_t& num_neighbors)
{
    // build query_box
    AxisAlignedBoundingBox query_box;
    query_box.set(query_point);
    query_box.grow_in_each_axial_direction(m_radius);

    // query
    this->get_overlaps(&query_box, neighbors_buffer, num_neighbors);
}

}

}
