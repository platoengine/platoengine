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
#pragma once

#include "PSL_Filter.hpp"
#include "PSL_ParameterDataEnums.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class PointCloud;
class ParallelVector;
class ParallelExchanger;
}
class ParameterData;
class AbstractAuthority;
class KernelFilter;
class PointCloud;

class OverhangFilter : public Filter
{
public:
    OverhangFilter(AbstractAuthority* authority,
                   ParameterData* data,
                   AbstractInterface::PointCloud* points,
                   AbstractInterface::ParallelExchanger* exchanger);
    virtual ~OverhangFilter();

    void set_authority(AbstractAuthority* authority);
    void set_input_data(ParameterData* data);
    void set_points(AbstractInterface::PointCloud* points);
    void set_parallel_exchanger(AbstractInterface::ParallelExchanger* exchanger);
    void announce_radius();

    // Filter operations
    virtual void build();
    virtual void apply(AbstractInterface::ParallelVector* field);
    virtual void apply(AbstractInterface::ParallelVector* base_field, AbstractInterface::ParallelVector* gradient);
    virtual void advance_continuation();

private:

    void compute_overhang_bias(PointCloud* kernel_points);
    void internal_apply(const std::vector<double>& input,
                        std::vector<double>& inner_preactivation,
                        std::vector<double>& inner_postactivation,
                        std::vector<double>& outer_preactivation);

    bool m_built;
    bool m_announce_radius;
    AbstractAuthority* m_authority;
    ParameterData* m_input_data;
    AbstractInterface::PointCloud* m_original_points;
    AbstractInterface::ParallelExchanger* m_parallel_exchanger;
    KernelFilter* m_smoothing_kernel;
    KernelFilter* m_overhang_kernel;
    double m_max_overhang_angle;
    double m_overhang_projection_angle_fraction;
    double m_overhang_projection_radius_fraction;
    double m_build_direction_x;
    double m_build_direction_y;
    double m_build_direction_z;
    double m_current_heaviside_parameter;
    double m_heaviside_parameter_continuation_scale;
    double m_max_heaviside_parameter;
    double m_overhang_fractional_threshold;
    std::vector<double> m_overhang_bias;

    void check_input_data();
};

}
