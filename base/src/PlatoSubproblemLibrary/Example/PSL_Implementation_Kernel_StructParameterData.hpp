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

#include "PSL_ParameterDataEnums.hpp"

namespace PlatoSubproblemLibrary
{
namespace example
{

#define PSL_KERNEL_STRUCTPARAMETERDATA(_type, _name) \
    bool has_##_name; _type _name;

struct Kernel_StructParameterData
{
    PSL_KERNEL_STRUCTPARAMETERDATA(double, scale)
    PSL_KERNEL_STRUCTPARAMETERDATA(double, absolute)
    PSL_KERNEL_STRUCTPARAMETERDATA(int, iterations)
    PSL_KERNEL_STRUCTPARAMETERDATA(double, symPlane_X)
    PSL_KERNEL_STRUCTPARAMETERDATA(double, symPlane_Y)
    PSL_KERNEL_STRUCTPARAMETERDATA(double, symPlane_Z)
    PSL_KERNEL_STRUCTPARAMETERDATA(double, penalty)
    PSL_KERNEL_STRUCTPARAMETERDATA(double, node_resolution_tolerance)
    PSL_KERNEL_STRUCTPARAMETERDATA(spatial_searcher_t::spatial_searcher_t, spatial_searcher)
    PSL_KERNEL_STRUCTPARAMETERDATA(normalization_t::normalization_t, normalization)
    PSL_KERNEL_STRUCTPARAMETERDATA(reproduction_level_t::reproduction_level_t, reproduction)
    PSL_KERNEL_STRUCTPARAMETERDATA(matrix_assembly_agent_t::matrix_assembly_agent_t, matrix_assembly_agent)
    PSL_KERNEL_STRUCTPARAMETERDATA(symmetry_plane_agent_t::symmetry_plane_agent_t, symmetry_plane_agent)
    PSL_KERNEL_STRUCTPARAMETERDATA(mesh_scale_agent_t::mesh_scale_agent_t, mesh_scale_agent)
    PSL_KERNEL_STRUCTPARAMETERDATA(point_ghosting_agent_t::point_ghosting_agent_t, point_ghosting_agent)
    PSL_KERNEL_STRUCTPARAMETERDATA(bounded_support_function_t::bounded_support_function_t, bounded_support_function)
};

}
}
