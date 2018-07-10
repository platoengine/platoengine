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
#pragma once

namespace PlatoSubproblemLibrary
{
class Point;

class AxisAlignedBoundingBox
{
public:
    AxisAlignedBoundingBox();
    AxisAlignedBoundingBox(float _m_x_min, float _m_x_max, float _m_y_min, float _m_y_max, float _m_z_min, float _m_z_max, int _m_id = -1);
    AxisAlignedBoundingBox(float _x, float _y, float _z, int _id = -1);

    AxisAlignedBoundingBox(const AxisAlignedBoundingBox box1, const AxisAlignedBoundingBox box2);

    bool overlap(const AxisAlignedBoundingBox& other) const;
    bool overlap_within_tolerance(const AxisAlignedBoundingBox& other, const float tolerance) const;

    bool overlap(const Point& other) const;
    bool overlap_within_tolerance(const Point& other, const float tolerance) const;

    bool contains(const AxisAlignedBoundingBox small) const;

    void set(Point* other);
    void grow_in_each_axial_direction(float growth);
    void grow_to_include(Point* other);

    float get_x_min() const { return m_x_min; }
    float get_x_max() const { return m_x_max; }
    float get_y_min() const { return m_y_min; }
    float get_y_max() const { return m_y_max; }
    float get_z_min() const { return m_z_min; }
    float get_z_max() const { return m_z_max; }

    void set_x_min(float _x) { m_x_min = _x; }
    void set_x_max(float _x) { m_x_max = _x; }
    void set_y_min(float _y) { m_y_min = _y; }
    void set_y_max(float _y) { m_y_max = _y; }
    void set_z_min(float _z) { m_z_min = _z; }
    void set_z_max(float _z) { m_z_max = _z; }

    int get_id() const { return m_id; }

    void set_id(int _id) { m_id = _id; }

    float get_x_center() const { return (m_x_min + m_x_max) * 0.5; }
    float get_y_center() const { return (m_y_min + m_y_max) * 0.5; }
    float get_z_center() const { return (m_z_min + m_z_max) * 0.5; }

    float get_x_length() const { return m_x_max - m_x_min; }
    float get_y_length() const { return m_y_max - m_y_min; }
    float get_z_length() const { return m_z_max - m_z_min; }

    void print(const bool should_end_line) const;

    AxisAlignedBoundingBox(const AxisAlignedBoundingBox& source);
    AxisAlignedBoundingBox& operator=(const AxisAlignedBoundingBox& source);

private:
    float m_x_min, m_x_max, m_y_min, m_y_max, m_z_min, m_z_max;
    int m_id;

};

}
