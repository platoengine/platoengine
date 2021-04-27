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
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Implementation_MeshModular.hpp"
#include "PSL_Interface_MeshModular.hpp"
#include "PSL_Abstract_OptimizationMesh.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_KernelFilter.hpp"
#include "PSL_Interface_PointCloud.hpp"
#include "PSL_BruteForceFixedRadiusNearestNeighbors.hpp"
#include "PSL_BoundingBoxMortonHierarchy.hpp"
#include "PSL_Interface_ConjugateGradient.hpp"
#include "PSL_Interface_SparseMatrixBuilder.hpp"
#include "PSL_Abstract_SparseMatrixBuilder.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_Interface_ParallelVector.hpp"
#include "PSL_Interface_SparseMatrixBuilder.hpp"
#include "PSL_Abstract_SparseMatrixBuilder.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Interface_BasicDenseVectorOperations.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_Point.hpp"
#include "PSL_Interface_ParallelExchanger_localAndNonlocal.hpp"
#include "PSL_Abstract_ParallelExchanger.hpp"
#include "PSL_Interface_ParallelExchanger_global.hpp"
#include "PSL_Random.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_KernelThenHeavisideFilter.hpp"
#include "PSL_GradientCheck.hpp"

#include <mpi.h>
#include <string>
#include <cstddef>
#include <sstream>
#include <cmath>

namespace PlatoSubproblemLibrary
{
namespace TestingKernelThenHeavisideFilter
{

PSL_TEST(KernelThenHeavisideFilter,applySmall)
{
    set_rand_seed();
    AbstractAuthority authority;

    const size_t mpi_size = authority.mpi_wrapper->get_size();
    if(mpi_size > 1u)
    {
        return;
    }

    // build mesh
    const size_t xlen = 3;
    const size_t ylen = 3;
    const size_t zlen = 3;
    const double xdist = 1.;
    const double ydist = 1.;
    const double zdist = 1.;
    example::ElementBlock modular_block;
    const size_t rank = authority.mpi_wrapper->get_rank();
    modular_block.build_from_structured_grid(xlen, ylen, zlen, xdist, ydist, zdist, rank, mpi_size);
    example::Interface_MeshModular modular_interface;
    modular_interface.set_mesh(&modular_block);
    const size_t num_points = modular_interface.get_num_points();
    EXPECT_EQ(num_points, 27u);

    // set input data
    ParameterData input_data;
    input_data.set_scale(1.8);
    input_data.set_iterations(1);
    input_data.set_penalty(1.);
    input_data.set_spatial_searcher(spatial_searcher_t::recommended);
    input_data.set_normalization(normalization_t::classical_row_normalization);
    input_data.set_reproduction(reproduction_level_t::reproduce_constant);
    input_data.set_matrix_assembly_agent(matrix_assembly_agent_t::by_row);
    input_data.set_symmetry_plane_agent(symmetry_plane_agent_t::by_narrow_clone);
    input_data.set_mesh_scale_agent(mesh_scale_agent_t::by_average_optimized_element_side);
    input_data.set_matrix_normalization_agent(matrix_normalization_agent_t::default_agent);
    input_data.set_point_ghosting_agent(point_ghosting_agent_t::by_narrow_share);
    input_data.set_bounded_support_function(bounded_support_function_t::polynomial_tent_function);
    input_data.set_min_heaviside_parameter(5.0);
    input_data.set_heaviside_continuation_scale(2.0);
    input_data.set_max_heaviside_parameter(50.0);

    // build control
    std::vector<double> control_data(num_points);
    uniform_rand_double(0.0, 1.0, control_data);
    example::Interface_ParallelVector control0(control_data);
    example::Interface_ParallelVector control1(control_data);
    example::Interface_ParallelVector control2(control_data);

    // build exchanger
    example::Interface_ParallelExchanger_localAndNonlocal parallel_exchanger(&authority);
    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data;
    modular_block.get_shared_node_data(shared_node_data);
    parallel_exchanger.put_shared_pairs(shared_node_data);
    parallel_exchanger.put_num_local_locations(num_points);
    parallel_exchanger.build();

    // build filter
    KernelThenHeavisideFilter filter(&authority, &input_data, &modular_interface, &parallel_exchanger);
    filter.build();

    // apply filter
    filter.apply(&control0);
    filter.advance_continuation();
    filter.apply(&control1);
    filter.advance_continuation();
    filter.apply(&control2);

    // compute separations
    double separations0 = 0.;
    double separations1 = 0.;
    double separations2 = 0.;
    for(size_t i = 0u; i < num_points; i++)
    {
        separations0 += fabs(control0.get_value(i) - 0.5);
        separations1 += fabs(control1.get_value(i) - 0.5);
        separations2 += fabs(control2.get_value(i) - 0.5);
    }

    // separation should be greatest with most continuation
    EXPECT_GT(separations1, separations0);
    EXPECT_GT(separations2, separations1);
}

class KTHFGradientCheck : public GradientCheck
{
public:
    KTHFGradientCheck(AbstractAuthority* authority, bool expect_accurate) :
            GradientCheck(1e-2),
            m_authority(authority),
            m_expect_accurate(expect_accurate),
            m_center(),
            m_step(),
            m_local_objective_scale(),
            m_parallel_objective_scale(),
            m_filter(NULL),
            m_exchanger(NULL),
            m_modular_block(),
            m_modular_interface(),
            m_input_data()
    {
    }
    virtual ~KTHFGradientCheck()
    {
        safe_free(m_filter);
        safe_free(m_exchanger);
    }

    virtual void initialize()
    {
        const size_t mpi_rank = m_authority->mpi_wrapper->get_rank();
        const size_t mpi_size = m_authority->mpi_wrapper->get_size();

        // build mesh
        const size_t xlen = 3;
        const size_t ylen = 5;
        const size_t zlen = 7;
        const double xdist = 1.;
        const double ydist = 1.;
        const double zdist = 1.;
        m_modular_block.build_from_structured_grid(xlen, ylen, zlen, xdist, ydist, zdist, mpi_rank, mpi_size);
        m_modular_interface.set_mesh(&m_modular_block);

        // input data
        m_input_data.set_absolute(2.5);
        m_input_data.set_iterations(1);
        m_input_data.set_penalty(1.);
        m_input_data.set_spatial_searcher(spatial_searcher_t::recommended);
        m_input_data.set_normalization(normalization_t::classical_row_normalization);
        m_input_data.set_reproduction(reproduction_level_t::reproduce_constant);
        m_input_data.set_matrix_assembly_agent(matrix_assembly_agent_t::by_row);
        m_input_data.set_symmetry_plane_agent(symmetry_plane_agent_t::by_narrow_clone);
        m_input_data.set_mesh_scale_agent(mesh_scale_agent_t::by_average_optimized_element_side);
        m_input_data.set_matrix_normalization_agent(matrix_normalization_agent_t::default_agent);
        m_input_data.set_point_ghosting_agent(point_ghosting_agent_t::by_narrow_share);
        m_input_data.set_bounded_support_function(bounded_support_function_t::polynomial_tent_function);
        m_input_data.set_min_heaviside_parameter(4.0);
        m_input_data.set_heaviside_continuation_scale(2.5);
        m_input_data.set_max_heaviside_parameter(50.0);

        // parallel exchanger globals
        m_exchanger = new example::Interface_ParallelExchanger_global(m_authority);
        std::vector<size_t> global_ids;
        m_modular_block.get_global_ids(global_ids);
        m_exchanger->put_globals(global_ids);
        m_exchanger->build();
        const size_t local_num_nodes = m_modular_block.get_num_nodes();

        // build center
        std::vector<double> localField(local_num_nodes);
        uniform_rand_double(0., 1., localField);
        example::Interface_ParallelVector parallelField(localField);
        m_exchanger->get_expansion_to_parallel_vector(m_exchanger->get_contraction_to_local_indexes(&parallelField), &parallelField);
        parallelField.get_values(m_center);

        // build step
        uniform_rand_double(-0.01, 0.01, localField);
        parallelField.set_values(localField);
        m_exchanger->get_expansion_to_parallel_vector(m_exchanger->get_contraction_to_local_indexes(&parallelField), &parallelField);
        parallelField.get_values(m_step);

        // build objective_scale
        normal_rand_double(0., 1., localField);
        parallelField.set_values(localField);
        m_local_objective_scale = m_exchanger->get_contraction_to_local_indexes(&parallelField);
        m_exchanger->get_expansion_to_parallel_vector(m_local_objective_scale, &parallelField);
        parallelField.get_values(m_parallel_objective_scale);

        // build filter
        m_filter = new KernelThenHeavisideFilter(m_authority, &m_input_data, &m_modular_interface, m_exchanger);
        m_filter->build();
    }
    virtual double gradient_dot_step()
    {
        // build control
        example::Interface_ParallelVector parallelControl(m_center);

        // build gradient
        example::Interface_ParallelVector parallelGradient(m_parallel_objective_scale);
        m_exchanger->get_expansion_to_parallel_vector(m_exchanger->get_contraction_to_local_indexes(&parallelGradient),
                                                      &parallelGradient);

        if(!m_expect_accurate)
        {
            m_filter->advance_continuation();
        }

        // compute gradient
        m_filter->apply(&parallelControl, &parallelGradient);

        // dot with step
        std::vector<double> local_gradient = m_exchanger->get_contraction_to_local_indexes(&parallelGradient);
        example::Interface_ParallelVector parallelStep(m_step);
        std::vector<double> local_step = m_exchanger->get_contraction_to_local_indexes(&parallelStep);
        double local_gradDotStep = m_authority->dense_vector_operations->dot(local_gradient, local_step);

        // get global contribution
        double global_gradDotStep = 0.;
        m_authority->mpi_wrapper->all_reduce_sum(local_gradDotStep, global_gradDotStep);
        return global_gradDotStep;
    }
    virtual double objective_of_minus_step()
    {
        // build control
        std::vector<double> center_plus_step = m_center;
        m_authority->dense_vector_operations->axpy(-1., m_step, center_plus_step);

        if(!m_expect_accurate)
        {
            m_filter->advance_continuation();
        }

        return evaluate_objective(center_plus_step);
    }
    virtual double objective_of_plus_step()
    {
        // build control
        std::vector<double> center_plus_step = m_center;
        m_authority->dense_vector_operations->axpy(1., m_step, center_plus_step);

        if(!m_expect_accurate)
        {
            m_filter->advance_continuation();
        }

        return evaluate_objective(center_plus_step);
    }

protected:

    double evaluate_objective(const std::vector<double>& control)
    {
        // apply on filter
        example::Interface_ParallelVector parallelField(control);
        m_filter->apply(&parallelField);

        // evaluate objective
        std::vector<double> localField = m_exchanger->get_contraction_to_local_indexes(&parallelField);
        double local_objective = m_authority->dense_vector_operations->dot(localField, m_local_objective_scale);

        // get global contribution
        double global_objective = 0.;
        m_authority->mpi_wrapper->all_reduce_sum(local_objective, global_objective);
        return global_objective;
    }

    AbstractAuthority* m_authority;
    bool m_expect_accurate;
    std::vector<double> m_center;
    std::vector<double> m_step;
    std::vector<double> m_local_objective_scale;
    std::vector<double> m_parallel_objective_scale;
    Filter* m_filter;
    example::Interface_ParallelExchanger_global* m_exchanger;
    example::ElementBlock m_modular_block;
    example::Interface_MeshModular m_modular_interface;
    ParameterData m_input_data;

};

PSL_TEST(KernelThenHeavisideFilter, gradientCheck)
{
    set_rand_seed();
    AbstractAuthority authority;

    // expect to pass
    KTHFGradientCheck checker0(&authority, true);
    EXPECT_EQ(checker0.check_pass(), true);

    // expect to fail, because used continuations mid calculation
    KTHFGradientCheck checker1(&authority, false);
    EXPECT_EQ(checker1.check_pass(), false);
}

}
}
