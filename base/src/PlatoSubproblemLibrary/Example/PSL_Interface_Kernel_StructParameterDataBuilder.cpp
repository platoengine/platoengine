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
#include "PSL_Interface_Kernel_StructParameterDataBuilder.hpp"

#include "PSL_Implementation_Kernel_StructParameterData.hpp"
#include "PSL_Abstract_ParameterDataBuilder.hpp"
#include "PSL_ParameterData.hpp"

#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_Kernel_StructParameterDataBuilder::Interface_Kernel_StructParameterDataBuilder() :
        AbstractInterface::ParameterDataBuilder(),
        m_data(NULL)
{
}

Interface_Kernel_StructParameterDataBuilder::~Interface_Kernel_StructParameterDataBuilder()
{
    m_data = NULL;
}

void Interface_Kernel_StructParameterDataBuilder::set_data(example::Kernel_StructParameterData* data)
{
    m_data = data;
}

#define PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(_input_data, _output_data, _field) \
    if(_input_data->has_##_field) { \
        _output_data->set_##_field(_input_data->_field); \
    }

ParameterData* Interface_Kernel_StructParameterDataBuilder::build()
{
    if (m_data == NULL) {
        return NULL;
    }

    // allocate
    ParameterData* result = new ParameterData;

    // fill
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, scale)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, absolute)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, iterations)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, symPlane_X)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, symPlane_Y)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, symPlane_Z)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, penalty)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, node_resolution_tolerance)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, spatial_searcher)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, normalization)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, reproduction)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, matrix_assembly_agent)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, symmetry_plane_agent)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, mesh_scale_agent)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, point_ghosting_agent)
    PSL_STRUCT_KERNEL_TO_INPUT_DATA_BUILDER(m_data, result, bounded_support_function)

    return result;
}

}
}
