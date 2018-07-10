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

// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_SymmetryPlaneAgent.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
enum symPlane_side_t
{
    positive, negative, both, no_information
};

namespace AbstractInterface
{
class PointCloud;
}
class AbstractAuthority;
class ParameterData;
class Point;

class ByNarrowClone_SymmetryPlaneAgent : public Abstract_SymmetryPlaneAgent
{
public:
    ByNarrowClone_SymmetryPlaneAgent(AbstractAuthority* authority);
    virtual ~ByNarrowClone_SymmetryPlaneAgent();

    virtual PointCloud* build_kernel_points(double support_distance,
                                            double point_resolution_tolerance,
                                            ParameterData* input_data,
                                            AbstractInterface::PointCloud* all_points,
                                            const std::vector<size_t>& indexes_of_local_points);
    virtual std::vector<double> expand_with_symmetry_points(const std::vector<double>& contracted_data_vector);
    virtual std::vector<double> contract_by_symmetry_points(const std::vector<double>& expanded_data_vector);

protected:
    void read_input_data(ParameterData* input_data);

    void determine_num_reflection_points_and_local_plane_side(const std::vector<Point>& kernel_point_vector,
                                                              symPlane_side_t& local_plane_side,
                                                              size_t& num_symmetric_points);
    void determine_global_plane_side(const std::vector<int>& all_symPlane_sides, symPlane_side_t& global_plane_side);
    void check_allowable_global_plane_side(symPlane_side_t global_plane_side);
    double get_symmetry_axial_value(const Point& point);
    bool is_reflection_point(double axial_value);
    void reflect_point(Point& to_reflect);
    double is_on_symmetry_plane(double this_point_axial_value);

    size_t m_num_contracted_points;
    std::vector<size_t> m_indexes_to_clone;

    // for build
    bool m_sym_plane_x;
    bool m_sym_plane_y;
    bool m_sym_plane_z;
    double m_symmetry_plane_axial_offset;
    double m_symmetric_buffer_distance;
    double m_point_resolution_tolerance;

};

}
