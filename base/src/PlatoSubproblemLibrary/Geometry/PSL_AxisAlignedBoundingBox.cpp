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
#include "PSL_AxisAlignedBoundingBox.hpp"

#include "PSL_Point.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

namespace PlatoSubproblemLibrary
{

AxisAlignedBoundingBox::AxisAlignedBoundingBox() :
                    m_x_min(0.0), m_x_max(0.0),
                    m_y_min(0.0), m_y_max(0.0),
                    m_z_min(0.0), m_z_max(0.0),
                    m_id(-1)
{
}

AxisAlignedBoundingBox::AxisAlignedBoundingBox(float _m_x_min, float _m_x_max, float _m_y_min, float _m_y_max, float _m_z_min, float _m_z_max, int _m_id) :
                    m_x_min(_m_x_min), m_x_max(_m_x_max),
                    m_y_min(_m_y_min), m_y_max(_m_y_max),
                    m_z_min(_m_z_min), m_z_max(_m_z_max),
                    m_id(_m_id)
{
}

AxisAlignedBoundingBox::AxisAlignedBoundingBox(float _x, float _y, float _z, int _id) :
                    m_x_min(_x), m_x_max(_x),
                    m_y_min(_y), m_y_max(_y),
                    m_z_min(_z), m_z_max(_z),
                    m_id(_id)
{
}

AxisAlignedBoundingBox::AxisAlignedBoundingBox(const AxisAlignedBoundingBox box1, const AxisAlignedBoundingBox box2) :
        m_x_min(std::min(box1.get_x_min(), box2.get_x_min())),
        m_x_max(std::max(box1.get_x_max(), box2.get_x_max())),
        m_y_min(std::min(box1.get_y_min(), box2.get_y_min())),
        m_y_max(std::max(box1.get_y_max(), box2.get_y_max())),
        m_z_min(std::min(box1.get_z_min(), box2.get_z_min())),
        m_z_max(std::max(box1.get_z_max(), box2.get_z_max())),
        m_id(-1)
{
}

bool AxisAlignedBoundingBox::overlap(const AxisAlignedBoundingBox& other) const
{
    return !(   m_x_max < other.get_x_min()
             || other.get_x_max() < m_x_min
             || m_y_max < other.get_y_min()
             || other.get_y_max() < m_y_min
             || m_z_max < other.get_z_min()
             || other.get_z_max() < m_z_min);
}

bool AxisAlignedBoundingBox::overlap_within_tolerance(const AxisAlignedBoundingBox& other, const float tolerance) const
{
    return !(   m_x_max < other.get_x_min() - tolerance
             || other.get_x_max() < m_x_min - tolerance
             || m_y_max < other.get_y_min() - tolerance
             || other.get_y_max() < m_y_min - tolerance
             || m_z_max < other.get_z_min() - tolerance
             || other.get_z_max() < m_z_min - tolerance);
}

bool AxisAlignedBoundingBox::overlap(const Point& other) const
{
    return !(   m_x_max < other(0u)
             || other(0u) < m_x_min
             || m_y_max < other(1u)
             || other(1u) < m_y_min
             || m_z_max < other(2u)
             || other(2u) < m_z_min);
}

bool AxisAlignedBoundingBox::overlap_within_tolerance(const Point& other, const float tolerance) const
{
    return !(   m_x_max < other(0u) - tolerance
             || other(0u) < m_x_min - tolerance
             || m_y_max < other(1u) - tolerance
             || other(1u) < m_y_min - tolerance
             || m_z_max < other(2u) - tolerance
             || other(2u) < m_z_min - tolerance);
}

bool AxisAlignedBoundingBox::contains(const AxisAlignedBoundingBox small) const
{
    return !(m_x_min > small.get_x_min()
             || small.get_x_max() > m_x_max
             || m_y_min > small.get_y_min()
             || small.get_y_max() > m_y_max
             || m_z_min > small.get_z_min()
             || small.get_z_max() > m_z_max);
}

void AxisAlignedBoundingBox::set(Point* other)
{
    m_id = other->get_index();
    m_x_min = (*other)(0);
    m_x_max = (*other)(0);
    m_y_min = (*other)(1);
    m_y_max = (*other)(1);
    m_z_min = (*other)(2);
    m_z_max = (*other)(2);
}

void AxisAlignedBoundingBox::grow_in_each_axial_direction(float growth)
{
    m_x_min -= growth;
    m_y_min -= growth;
    m_z_min -= growth;
    m_x_max += growth;
    m_y_max += growth;
    m_z_max += growth;
}

void AxisAlignedBoundingBox::grow_to_include(Point* other)
{
    m_x_min = std::min(float((*other)(0)), m_x_min);
    m_x_max = std::max(float((*other)(0)), m_x_max);
    m_y_min = std::min(float((*other)(1)), m_y_min);
    m_y_max = std::max(float((*other)(1)), m_y_max);
    m_z_min = std::min(float((*other)(2)), m_z_min);
    m_z_max = std::max(float((*other)(2)), m_z_max);
}

void AxisAlignedBoundingBox::print(const bool should_end_line) const
{
    std::cout << m_x_min << "," << m_x_max << "," << m_y_min << "," << m_y_max << "," << m_z_min << "," << m_z_max;
    if(should_end_line) std::cout << std::endl;
}

AxisAlignedBoundingBox::AxisAlignedBoundingBox(const AxisAlignedBoundingBox& source)
{
    m_x_min = source.get_x_min();
    m_x_max = source.get_x_max();
    m_y_min = source.get_y_min();
    m_y_max = source.get_y_max();
    m_z_min = source.get_z_min();
    m_z_max = source.get_z_max();
    m_id = source.get_id();
}
AxisAlignedBoundingBox& AxisAlignedBoundingBox::operator=(const AxisAlignedBoundingBox& source)
{
    if(this != &source)
    {
        m_x_min = source.get_x_min();
        m_x_max = source.get_x_max();
        m_y_min = source.get_y_min();
        m_y_max = source.get_y_max();
        m_z_min = source.get_z_min();
        m_z_max = source.get_z_max();
        m_id = source.get_id();
    }
    return *this;
}

}
