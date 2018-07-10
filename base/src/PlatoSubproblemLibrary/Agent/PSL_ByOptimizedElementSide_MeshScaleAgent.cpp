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

// PlatoSubproblemLibraryVersion(6): a stand-alone library for the kernel filter for plato.
#include "PSL_ByOptimizedElementSide_MeshScaleAgent.hpp"

#include "PSL_Abstract_MeshScaleAgent.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_PointCloud.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_AxisAlignedBoundingBox.hpp"
#include "PSL_Abstract_OptimizationMesh.hpp"
#include "PSL_Point.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_Mesh.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <cassert>
#include <vector>

namespace PlatoSubproblemLibrary
{

ByOptimizedElementSide_MeshScaleAgent::ByOptimizedElementSide_MeshScaleAgent(AbstractAuthority* authority,
                                                                             AbstractInterface::PointCloud* points) :
        Abstract_MeshScaleAgent(mesh_scale_agent_t::by_average_optimized_element_side, authority, points),
        m_calculated_mesh_scales(false),
        m_mesh_minimum_scale(-1.),
        m_mesh_average_scale(-1.),
        m_mesh_maximum_scale(-1.)
{
}

ByOptimizedElementSide_MeshScaleAgent::~ByOptimizedElementSide_MeshScaleAgent()
{
}

double ByOptimizedElementSide_MeshScaleAgent::get_mesh_minimum_scale()
{
    if(!m_calculated_mesh_scales)
    {
        this->calculate_mesh_scales();
    }

    return m_mesh_minimum_scale;
}

double ByOptimizedElementSide_MeshScaleAgent::get_mesh_average_scale()
{
    if(!m_calculated_mesh_scales)
    {
        this->calculate_mesh_scales();
    }

    return m_mesh_average_scale;
}

double ByOptimizedElementSide_MeshScaleAgent::get_mesh_maximum_scale()
{
    if(!m_calculated_mesh_scales)
    {
        this->calculate_mesh_scales();
    }

    return m_mesh_maximum_scale;
}

void ByOptimizedElementSide_MeshScaleAgent::calculate_mesh_scales()
{
    AbstractInterface::Mesh* mesh = dynamic_cast<AbstractInterface::Mesh*>(m_points);
    if(!mesh)
    {
        m_authority->utilities->fatal_error("ByOptimizedElementSide_MeshScaleAgent: failed to dynamic cast pointer. Aborting.\n\n");
    }

    std::vector<bool> block_is_optimizable = get_is_optimizable();

    // count  elements
    const size_t num_blocks = mesh->get_num_blocks();
    std::vector<size_t> elems_per_block(1u+num_blocks);
    int local_num_optimization_elems = 0;
    for(size_t block_index = 0u; block_index < num_blocks; block_index++)
    {
        const bool is_optimizable = block_is_optimizable[block_index];
        const size_t num_elem = mesh->get_num_elements(block_index);

        // add to counter
        elems_per_block[1u+block_index] += num_elem;
        local_num_optimization_elems += num_elem * is_optimizable;
    }
    std::vector<size_t> start_of_element_insert(1u+num_blocks);

    // allocate elements
    cumulative_sum(elems_per_block, start_of_element_insert);
    std::vector<AxisAlignedBoundingBox> local_element_boxes(start_of_element_insert.back());
    size_t num_local_element_boxes = local_element_boxes.size();

    // fill elements
    std::vector<bool> elem_is_optimizable(num_local_element_boxes);
    for(size_t block_index = 0u; block_index < num_blocks; block_index++)
    {
        const bool is_optimizable = block_is_optimizable[block_index];
        std::fill(elem_is_optimizable.begin() + start_of_element_insert[block_index],
                  elem_is_optimizable.begin() + start_of_element_insert[block_index + 1u],
                  is_optimizable);
        const size_t num_elem = mesh->get_num_elements(block_index);
        for(size_t elem_index = 0u; elem_index < num_elem; elem_index++)
        {
            std::vector<size_t> elem_nodes = mesh->get_nodes_from_element(block_index, elem_index);
            const size_t num_elem_nodes = elem_nodes.size();
            if(num_elem_nodes == 0u)
            {
                m_authority->utilities->fatal_error("ByOptimizedElementSide_MeshScaleAgent: unable to handle element with no nodes. Aborting.\n\n");
            }
            Point first_node = mesh->get_point(elem_nodes[0u]);
            local_element_boxes[start_of_element_insert[block_index]].set(&first_node);
            for(size_t elem_node_index = 1u; elem_node_index < num_elem_nodes; elem_node_index++)
            {
                Point later_node = mesh->get_point(elem_nodes[elem_node_index]);
                local_element_boxes[start_of_element_insert[block_index]].grow_to_include(&later_node);
            }
            start_of_element_insert[block_index]++;
        }
    }

    // aggregate locally
    double local_min_mesh_length_scale = 1e10;
    double local_sum_mesh_length_scale = 0.;
    double local_max_mesh_length_scale = 0.;
    for(size_t local_element_box_index = 0; local_element_box_index < num_local_element_boxes; local_element_box_index++)
    {
        const double this_x_length = local_element_boxes[local_element_box_index].get_x_length();
        const double this_y_length = local_element_boxes[local_element_box_index].get_y_length();
        const double this_z_length = local_element_boxes[local_element_box_index].get_z_length();

        local_min_mesh_length_scale = std::min(std::min(this_x_length, this_y_length),
                                               std::min(this_z_length, local_min_mesh_length_scale));
        local_max_mesh_length_scale = std::max(std::max(this_x_length, this_y_length),
                                               std::max(this_z_length, local_max_mesh_length_scale));
        if(elem_is_optimizable[local_element_box_index])
        {
            local_sum_mesh_length_scale += this_x_length + this_y_length + this_z_length;
        }
    }

    // aggregate globally
    double global_min_length_scale = 0.;
    m_authority->mpi_wrapper->all_reduce_min(local_min_mesh_length_scale, global_min_length_scale);
    double global_sum_length_scale = 0.;
    m_authority->mpi_wrapper->all_reduce_sum(local_sum_mesh_length_scale, global_sum_length_scale);
    int global_num_optimization_elems = 0;
    m_authority->mpi_wrapper->all_reduce_sum(local_num_optimization_elems, global_num_optimization_elems);
    double global_max_length_scale = 0.;
    m_authority->mpi_wrapper->all_reduce_max(local_max_mesh_length_scale, global_max_length_scale);

    // average
    const double num_coords = 3.0;
    const double global_average_mesh_length_scale = global_sum_length_scale / (num_coords * double(global_num_optimization_elems));

    // set results
    m_mesh_minimum_scale = global_min_length_scale;
    m_mesh_average_scale = global_average_mesh_length_scale;
    m_mesh_maximum_scale = global_max_length_scale;
    m_calculated_mesh_scales = true;
}

std::vector<bool> ByOptimizedElementSide_MeshScaleAgent::get_is_optimizable()
{
    AbstractInterface::Mesh* mesh = dynamic_cast<AbstractInterface::Mesh*>(m_points);
    if(!mesh)
    {
        m_authority->utilities->fatal_error("ByOptimizedElementSide_MeshScaleAgent: failed to dynamic cast pointer. Aborting.\n\n");
    }

    // assume optimizable
    const size_t num_blocks = mesh->get_num_blocks();
    std::vector<bool> optimizable(num_blocks, true);

    // if possible, determine if blocks actually optimizable
    AbstractInterface::OptimizationMesh* optimization_mesh = dynamic_cast<AbstractInterface::OptimizationMesh*>(m_points);
    if(optimization_mesh)
    {
        for(size_t block = 0u; block < num_blocks; block++) {
            optimizable[block] = optimization_mesh->is_block_optimizable(block);
        }
    }

    return optimizable;
}

}
