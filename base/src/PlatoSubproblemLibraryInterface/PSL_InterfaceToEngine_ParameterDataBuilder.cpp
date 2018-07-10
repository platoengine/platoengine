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

#include "PSL_InterfaceToEngine_ParameterDataBuilder.hpp"

#include "PSL_ParameterDataEnums.hpp"
#include "PSL_ParameterData.hpp"
#include "Plato_InputData.hpp"
#include "Plato_Parser.hpp"

namespace Plato
{

InterfaceToEngine_ParameterDataBuilder::InterfaceToEngine_ParameterDataBuilder(InputData aInputData) :
        PlatoSubproblemLibrary::AbstractInterface::ParameterDataBuilder(),
        m_inputData(aInputData)
{
}

InterfaceToEngine_ParameterDataBuilder::~InterfaceToEngine_ParameterDataBuilder()
{
}

PlatoSubproblemLibrary::ParameterData* InterfaceToEngine_ParameterDataBuilder::build()
{
    // allocate result
    PlatoSubproblemLibrary::ParameterData* result = new PlatoSubproblemLibrary::ParameterData;

    // set scale and/or absolute
    double absolute=-1.0, scale=-1.0;
    if( m_inputData.size<Plato::InputData>("Filter") )
    {
        auto tFilterNode = m_inputData.get<Plato::InputData>("Filter");
        if(tFilterNode.size<std::string>("Absolute") > 0)
            absolute = Plato::Get::Double(tFilterNode, "Absolute");
        if(tFilterNode.size<std::string>("Scale") > 0)
            scale = Plato::Get::Double(tFilterNode, "Scale");
    }
    const bool meaningful_absolute = (absolute >= 0.);
    const bool meaningful_scale = (scale >= 0.);
    if(meaningful_absolute)
    {
        result->set_absolute(absolute);
    }
    if(meaningful_scale)
    {
        result->set_scale(scale);
    }
    if(!meaningful_absolute && !meaningful_scale)
    {
        const double default_scale = 3.5;
        result->set_scale(default_scale);
    }

    result->set_iterations(1);
    result->set_penalty(1.0);

    // defaults
    result->set_spatial_searcher(PlatoSubproblemLibrary::spatial_searcher_t::spatial_searcher_t::recommended);
    result->set_normalization(PlatoSubproblemLibrary::normalization_t::normalization_t::classical_row_normalization);
    result->set_reproduction(PlatoSubproblemLibrary::reproduction_level_t::reproduction_level_t::reproduce_constant);
    result->set_symmetry_plane_agent(PlatoSubproblemLibrary::symmetry_plane_agent_t::by_narrow_clone);
    result->set_matrix_assembly_agent(PlatoSubproblemLibrary::matrix_assembly_agent_t::by_row);
    result->set_mesh_scale_agent(PlatoSubproblemLibrary::mesh_scale_agent_t::by_average_optimized_element_side);
    result->set_matrix_normalization_agent(PlatoSubproblemLibrary::matrix_normalization_agent_t::default_agent);
    result->set_point_ghosting_agent(PlatoSubproblemLibrary::point_ghosting_agent_t::by_narrow_share);
    result->set_bounded_support_function(PlatoSubproblemLibrary::bounded_support_function_t::polynomial_tent_function);

    return result;
}

}

