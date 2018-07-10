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

#include <mpi.h>
#include <string>
#include <cstddef>
#include <sstream>

namespace PlatoSubproblemLibrary
{
namespace TestingKernelFilter
{

void build_brick_of_points(example::Interface_PointCloud* points,
                           const size_t num_vert_points,
                           const size_t num_horiz_points,
                           const double horiz_step,
                           const double vert_step,
                           const double x_base,
                           const double y_base,
                           const double z);
void kernel_filter_test_two_methods(AbstractAuthority* authority,
                                    KernelFilter* kernel_one,
                                    KernelFilter* kernel_two,
                                    size_t multiplier);

PSL_TEST(KernelFilter,small)
{
    set_rand_seed();
    AbstractAuthority authority;

    // only serial
    const size_t mpi_size = authority.mpi_wrapper->get_size();
    if(mpi_size > 1u)
    {
        return;
    }

    // build mesh
    const size_t xlen = 2;
    const size_t ylen = 2;
    const size_t zlen = 2;
    const double xdist = 1.;
    const double ydist = 1.;
    const double zdist = 1.;
    example::ElementBlock modular_block;
    const size_t rank = authority.mpi_wrapper->get_rank();
    modular_block.build_from_structured_grid(xlen, ylen, zlen, xdist, ydist, zdist, rank, mpi_size);
    example::Interface_MeshModular modular_interface;
    modular_interface.set_mesh(&modular_block);
    const size_t num_points = modular_interface.get_num_points();
    EXPECT_EQ(num_points, 8u);

    // set input data
    ParameterData input_data;
    input_data.set_scale(1.7);
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

    // build controls
    const double non_zero_field_value = 0.95;
    std::vector<double> control_data(num_points, 0.);
    control_data[0] = non_zero_field_value;
    example::Interface_ParallelVector control(control_data);
    example::Interface_ParallelVector control_sym(control_data);
    example::Interface_ParallelVector control_grad(control_data);

    // build exchanger
    example::Interface_ParallelExchanger_localAndNonlocal parallel_exchanger(&authority);
    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data;
    modular_block.get_shared_node_data(shared_node_data);
    parallel_exchanger.put_shared_pairs(shared_node_data);
    parallel_exchanger.put_num_local_locations(num_points);
    parallel_exchanger.build();

    // apply filter without symmetry
    KernelFilter kernel_filter(&authority,
                               &input_data,
                               &modular_interface,
                               &parallel_exchanger);
    kernel_filter.build();
    kernel_filter.apply(&control);

    // apply filter
    kernel_filter.apply(&control, &control_grad);

    // apply filter with symmetry
    input_data.set_symPlane_Z(0.);
    KernelFilter kernel_filter_sym(&authority,
                                   &input_data,
                                   &modular_interface,
                                   &parallel_exchanger);
    kernel_filter_sym.build();
    kernel_filter_sym.apply(&control_sym);

    const double source = 0.365013709656965;
    const double edgeShare = 0.150299762799927;
    const double faceShare = 0.0613623339810846;
    const double unscaledGold_data[] = {source, edgeShare, edgeShare, faceShare, edgeShare, faceShare, faceShare, 0.0};

    const double source_sym = 0.286729540169508;
    const double edgeShare_sym = 0.118065104775680;
    const double faceShare_sym = 0.0482020081455546;
    const double unscaledGold_data_sym[] = {source_sym, edgeShare, edgeShare_sym, faceShare, edgeShare_sym, faceShare,
                                            faceShare_sym, 0.0};

    for(size_t i = 0u; i < num_points; i++)
    {
        EXPECT_FLOAT_EQ(non_zero_field_value*unscaledGold_data[i], control.get_value(i));
        EXPECT_FLOAT_EQ(non_zero_field_value*unscaledGold_data_sym[i], control_sym.get_value(i));
        EXPECT_FLOAT_EQ(non_zero_field_value*unscaledGold_data[i], control_grad.get_value(i));
    }
}

PSL_TEST(KernelFilter,medium)
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
    input_data.set_scale(1.9);
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

    // build controls
    const double nonZero_fieldValue = 0.95;
    std::vector<double> control_data(num_points, 0.);
    control_data[0] = nonZero_fieldValue;
    example::Interface_ParallelVector control(control_data);
    example::Interface_ParallelVector gradient(control_data);
    example::Interface_ParallelVector control_correctionFunction_reproduceLinear(control_data);
    example::Interface_ParallelVector gradient_correctionFunction_reproduceLinear(control_data);
    example::Interface_ParallelVector control_minimalChange_reproduceLinear(control_data);
    example::Interface_ParallelVector gradient_minimalChange_reproduceLinear(control_data);

    // build exchanger
    example::Interface_ParallelExchanger_localAndNonlocal parallel_exchanger(&authority);
    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data;
    modular_block.get_shared_node_data(shared_node_data);
    parallel_exchanger.put_shared_pairs(shared_node_data);
    parallel_exchanger.put_num_local_locations(num_points);
    parallel_exchanger.build();

    // build filter
    KernelFilter kernel_filter(&authority,
                               &input_data,
                               &modular_interface,
                               &parallel_exchanger);
    kernel_filter.build();

    // apply filter
    kernel_filter.apply(&control);
    kernel_filter.apply(&control, &gradient);

    // filter with normalization by correction functions for reproducing linear
    KernelFilter kernel_correctionFunction_reproduceLinear(&authority,
                                                           &input_data,
                                                           &modular_interface,
                                                           &parallel_exchanger);
    input_data.set_normalization(normalization_t::correction_function_reproducing_conditions);
    input_data.set_reproduction(reproduction_level_t::reproduce_linear);
    kernel_correctionFunction_reproduceLinear.build();
    kernel_correctionFunction_reproduceLinear.apply(&control_correctionFunction_reproduceLinear);
    kernel_correctionFunction_reproduceLinear.apply(&control, &gradient_correctionFunction_reproduceLinear);

    // filter with normalization by minimal change to reproducing linear
    KernelFilter kernel_minimalChange_reproduceLinear(&authority,
                                                      &input_data,
                                                      &modular_interface,
                                                      &parallel_exchanger);
    input_data.set_normalization(normalization_t::minimal_change_to_reproducing_conditions);
    input_data.set_reproduction(reproduction_level_t::reproduce_linear);
    kernel_minimalChange_reproduceLinear.build();
    kernel_minimalChange_reproduceLinear.apply(&control_minimalChange_reproduceLinear);
    kernel_minimalChange_reproduceLinear.apply(&control, &gradient_minimalChange_reproduceLinear);

    // these gold values are based on external calculations
    const double source_control = 0.305205757815665;
    const double edgeShare_control = 0.108895154238312;
    const double faceShare_control = 0.044331312691105;
    const double elemShare_control = 0.011604286265727;
    const double unscaledGold_control[] = {0., source_control, edgeShare_control, faceShare_control, elemShare_control};

    const double source_gradient = 0.305205757815665;
    const double edgeShare_gradient = 0.144571148438999;
    const double faceShare_gradient = 0.0780341146486792;
    const double elemShare_gradient = 0.0269784529212995;
    const double unscaledGold_gradient[] = {0., source_gradient, edgeShare_gradient, faceShare_gradient, elemShare_gradient};

    const double source_control_correctionFunction_reproduceLinear = 0.760942878305823;
    const double edgeShare_control_correctionFunction_reproduceLinear = 0.217521825902515;
    const double faceShare_control_correctionFunction_reproduceLinear = 0.065266280402177;
    const double elemShare_control_correctionFunction_reproduceLinear = 0.011604286265727;
    const double unscaledGold_control_correctionFunction_reproduceLinear[] = {
            0., source_control_correctionFunction_reproduceLinear, edgeShare_control_correctionFunction_reproduceLinear,
            faceShare_control_correctionFunction_reproduceLinear, elemShare_control_correctionFunction_reproduceLinear};

    const double source_gradient_correctionFunction_reproduceLinear = 0.760942878305823;
    const double edgeShare_gradient_correctionFunction_reproduceLinear = 0.140805062599378;
    const double faceShare_gradient_correctionFunction_reproduceLinear = -0.042553003504579;
    const double elemShare_gradient_correctionFunction_reproduceLinear = -0.055699055590220;
    const double unscaledGold_gradient_correctionFunction_reproduceLinear[] = {
            0., source_gradient_correctionFunction_reproduceLinear, edgeShare_gradient_correctionFunction_reproduceLinear,
            faceShare_gradient_correctionFunction_reproduceLinear, elemShare_gradient_correctionFunction_reproduceLinear};

    const double source_control_minimalChange_reproduceLinear = 0.550919130808908;
    const double edgeShare_control_minimalChange_reproduceLinear = 0.217518381052846;
    const double faceShare_control_minimalChange_reproduceLinear = 0.079971577604606;
    const double elemShare_control_minimalChange_reproduceLinear = 0.011604286265727;
    const double unscaledGold_control_minimalChange_reproduceLinear[] = {0., source_control_minimalChange_reproduceLinear,
                                                                         edgeShare_control_minimalChange_reproduceLinear,
                                                                         faceShare_control_minimalChange_reproduceLinear,
                                                                         elemShare_control_minimalChange_reproduceLinear};

    const double source_gradient_minimalChange_reproduceLinear = 0.550919130808908;
    const double edgeShare_gradient_minimalChange_reproduceLinear = 0.226475606103413;
    const double faceShare_gradient_minimalChange_reproduceLinear = -0.003870343015735;
    const double elemShare_gradient_minimalChange_reproduceLinear = -0.218734920071944;
    const double unscaledGold_gradient_minimalChange_reproduceLinear[] = {0., source_gradient_minimalChange_reproduceLinear,
                                                                          edgeShare_gradient_minimalChange_reproduceLinear,
                                                                          faceShare_gradient_minimalChange_reproduceLinear,
                                                                          elemShare_gradient_minimalChange_reproduceLinear};

    const size_t nodeToGoldIndex[] = {1, 2, 0, 2, 3, 0, 0, 0, 0, 2, 3, 0, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    for(size_t i = 0; i < num_points; i++)
    {
        const size_t gold = nodeToGoldIndex[i];
        EXPECT_FLOAT_EQ(nonZero_fieldValue*unscaledGold_control[gold], control.get_value(i));
        EXPECT_FLOAT_EQ(nonZero_fieldValue*unscaledGold_gradient[gold], gradient.get_value(i));
        EXPECT_FLOAT_EQ(nonZero_fieldValue*unscaledGold_control_correctionFunction_reproduceLinear[gold],
                        control_correctionFunction_reproduceLinear.get_value(i));
        EXPECT_FLOAT_EQ(nonZero_fieldValue*unscaledGold_gradient_correctionFunction_reproduceLinear[gold],
                        gradient_correctionFunction_reproduceLinear.get_value(i));
        EXPECT_FLOAT_EQ(nonZero_fieldValue*unscaledGold_control_minimalChange_reproduceLinear[gold],
                        control_minimalChange_reproduceLinear.get_value(i));
        EXPECT_FLOAT_EQ(nonZero_fieldValue*unscaledGold_gradient_minimalChange_reproduceLinear[gold],
                        gradient_minimalChange_reproduceLinear.get_value(i));
    }
}

void build_brick_of_points(example::Interface_PointCloud* points,
                           const size_t num_vert_points,
                           const size_t num_horiz_points,
                           const double horiz_step,
                           const double vert_step,
                           const double x_base,
                           const double y_base,
                           const double z)
{
    points->set_num_points(num_vert_points * num_horiz_points);

    size_t point_index = 0;
    for(size_t vert_index = 0; vert_index < num_vert_points; vert_index++)
    {
        for(size_t horiz_index = 0; horiz_index < num_horiz_points; horiz_index++)
        {
            const double x = x_base + horiz_index * horiz_step;
            const double y = y_base + vert_index * vert_step;
            std::vector<double> point_data({x, y, z});

            points->set_point_data(point_index, point_data);
            point_index++;
        }
    }
}

PSL_TEST(KernelFilter,elementParallelVsSerialConsistencyCheck)
{
    set_rand_seed();
    AbstractAuthority authority;

    // check for consistency for "parallel" and "serial" runs of the "same" problem

    // In truth: both runs are parallel but the problem is different.

    // For the parallel run, each processor is directly adjacent to its sequential neighbors;
    // the processors are stacked in the y coordinate, and the processors only know locally about
    // their portion of the global domain.

    // For the serial run, each processor is sufficiently spaced from its sequential neighbors
    // such that their kernel filters will not interact in parallel; the processors are stacked in
    // the z coordinate, and the processor has a copy of the global domain shifted to its rank-
    // appropriate z value.

    const size_t mpi_rank = authority.mpi_wrapper->get_rank();
    const size_t mpi_size = authority.mpi_wrapper->get_size();

    // specify problem
    const size_t num_horiz_points = 3u;

    const size_t local_num_vert_points = 3u;
    const size_t global_num_vert_points = local_num_vert_points * mpi_size;

    const size_t num_local_points = num_horiz_points * local_num_vert_points;
    const size_t num_global_points = num_horiz_points * global_num_vert_points;

    const double horiz_step = 0.5;
    const double vert_step = 1.0;
    const double local_zmin = 0.;
    const double filter_radius = 3.1;

    // set input data
    ParameterData input_data;
    input_data.set_absolute(filter_radius);
    input_data.set_iterations(1);
    input_data.set_penalty(1.);
    input_data.set_node_resolution_tolerance(1e-6);
    input_data.set_spatial_searcher(spatial_searcher_t::recommended);
    input_data.set_normalization(normalization_t::classical_row_normalization);
    input_data.set_reproduction(reproduction_level_t::reproduce_constant);
    input_data.set_matrix_assembly_agent(matrix_assembly_agent_t::by_row);
    input_data.set_symmetry_plane_agent(symmetry_plane_agent_t::by_narrow_clone);
    input_data.set_mesh_scale_agent(mesh_scale_agent_t::by_average_optimized_element_side);
    input_data.set_matrix_normalization_agent(matrix_normalization_agent_t::default_agent);
    input_data.set_point_ghosting_agent(point_ghosting_agent_t::by_narrow_share);
    input_data.set_bounded_support_function(bounded_support_function_t::polynomial_tent_function);

    // creating local bounding points of a grid, rank-offset in y
    example::Interface_PointCloud local_points;
    const double local_y_base = mpi_rank * local_num_vert_points * vert_step;
    build_brick_of_points(&local_points, local_num_vert_points, num_horiz_points,
                          horiz_step, vert_step, 0.0, local_y_base, local_zmin);

    // create global bounding points of a grid, rank-offset in z
    example::Interface_PointCloud global_points;
    const double local_z_base = mpi_rank * filter_radius * 1.01 + local_zmin;
    build_brick_of_points(&global_points, global_num_vert_points, num_horiz_points,
                          horiz_step, vert_step, 0.0, 0.0, local_z_base);

    // field values
    std::vector<double> global_data_vector(num_global_points, -1.0);
    std::vector<double> local_data_vector(num_local_points, -1.0);
    const size_t local_element_index_offset_begin = mpi_rank * num_local_points;
    const size_t local_element_index_offset_nonInclusive_end = (mpi_rank + 1) * num_local_points;

    if(mpi_rank == 0u)
    {
        const size_t seed = time(NULL);
        std::string seed_message = std::string("time-based random seed: ") + std::to_string(seed) + std::string("\n");
        authority.utilities->print(seed_message);
        srand(seed);

        // create global field
        for(size_t global_field_index = 0; global_field_index < num_global_points; global_field_index++)
        {
            global_data_vector[global_field_index] = 0.1 + 0.8 * uniform_rand_double();
        }

        // broadcast send global field
        authority.mpi_wrapper->broadcast(0u, global_data_vector);
    }
    else
    {
        // receive broadcasted global field
        authority.mpi_wrapper->broadcast(0u, global_data_vector);
    }

    // get local field from global field
    std::copy(&global_data_vector[local_element_index_offset_begin],
              &global_data_vector[local_element_index_offset_nonInclusive_end],
              local_data_vector.data());

    // create field copies
    example::Interface_ParallelVector localElement_fieldValues(local_data_vector);
    example::Interface_ParallelVector globalElement_fieldValues(global_data_vector);
    example::Interface_ParallelVector localElement_gradientValues(local_data_vector);
    example::Interface_ParallelVector globalElement_gradientValues(global_data_vector);
    example::Interface_ParallelVector localElement_correctionFunction_Linear_fieldValues(local_data_vector);
    example::Interface_ParallelVector localElement_correctionFunction_Linear_gradientValues(local_data_vector);
    example::Interface_ParallelVector globalElement_correctionFunction_Linear_fieldValues(global_data_vector);
    example::Interface_ParallelVector globalElement_correctionFunction_Linear_gradientValues(global_data_vector);
    example::Interface_ParallelVector localElement_correctionFunction_Quadratic_fieldValues(local_data_vector);
    example::Interface_ParallelVector localElement_correctionFunction_Quadratic_gradientValues(local_data_vector);
    example::Interface_ParallelVector globalElement_correctionFunction_Quadratic_fieldValues(global_data_vector);
    example::Interface_ParallelVector globalElement_correctionFunction_Quadratic_gradientValues(global_data_vector);
    example::Interface_ParallelVector localElement_minimalChange_Linear_fieldValues(local_data_vector);
    example::Interface_ParallelVector localElement_minimalChange_Linear_gradientValues(local_data_vector);
    example::Interface_ParallelVector globalElement_minimalChange_Linear_fieldValues(global_data_vector);
    example::Interface_ParallelVector globalElement_minimalChange_Linear_gradientValues(global_data_vector);
    example::Interface_ParallelVector localElement_minimalChange_Quadratic_fieldValues(local_data_vector);
    example::Interface_ParallelVector localElement_minimalChange_Quadratic_gradientValues(local_data_vector);
    example::Interface_ParallelVector globalElement_minimalChange_Quadratic_fieldValues(global_data_vector);
    example::Interface_ParallelVector globalElement_minimalChange_Quadratic_gradientValues(global_data_vector);

    // build exchangers
    example::Interface_ParallelExchanger_localAndNonlocal parallelExchanger_parallelRun(&authority);
    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data(mpi_size);
    parallelExchanger_parallelRun.put_shared_pairs(shared_node_data);
    parallelExchanger_parallelRun.put_num_local_locations(num_local_points);
    parallelExchanger_parallelRun.build();
    example::Interface_ParallelExchanger_localAndNonlocal parallelExchanger_serialRun(&authority);
    parallelExchanger_serialRun.put_shared_pairs(shared_node_data);
    parallelExchanger_serialRun.put_num_local_locations(num_global_points);
    parallelExchanger_serialRun.build();

    // do filters

    KernelFilter kernel_parallelRun(&authority,
                                    &input_data,
                                    &local_points,
                                    &parallelExchanger_parallelRun);
    kernel_parallelRun.build();
    kernel_parallelRun.apply(&localElement_fieldValues);
    kernel_parallelRun.apply(&localElement_fieldValues, &localElement_gradientValues);

    KernelFilter kernel_serialRun(&authority,
                                  &input_data,
                                  &global_points,
                                  &parallelExchanger_serialRun);
    kernel_serialRun.build();
    kernel_serialRun.apply(&globalElement_fieldValues);
    kernel_serialRun.apply(&globalElement_fieldValues, &globalElement_gradientValues);

    KernelFilter kernel_parallelRun_correctionFunction_Linear(&authority,
                                                              &input_data,
                                                              &local_points,
                                                              &parallelExchanger_parallelRun);
    input_data.set_normalization(normalization_t::correction_function_reproducing_conditions);
    input_data.set_reproduction(reproduction_level_t::reproduce_linear);
    kernel_parallelRun_correctionFunction_Linear.build();
    kernel_parallelRun_correctionFunction_Linear.apply(&localElement_correctionFunction_Linear_fieldValues);
    kernel_parallelRun_correctionFunction_Linear.apply(&localElement_fieldValues,
                                                       &localElement_correctionFunction_Linear_gradientValues);

    KernelFilter kernel_serialRun_correctionFunction_Linear(&authority,
                                                            &input_data,
                                                            &global_points,
                                                            &parallelExchanger_serialRun);
    kernel_serialRun_correctionFunction_Linear.build();
    kernel_serialRun_correctionFunction_Linear.apply(&globalElement_correctionFunction_Linear_fieldValues);
    kernel_serialRun_correctionFunction_Linear.apply(&globalElement_fieldValues,
                                                     &globalElement_correctionFunction_Linear_gradientValues);

    KernelFilter kernel_parallelRun_correctionFunction_Quadratic(&authority,
                                                                 &input_data,
                                                                 &local_points,
                                                                 &parallelExchanger_parallelRun);
    input_data.set_normalization(normalization_t::correction_function_reproducing_conditions);
    input_data.set_reproduction(reproduction_level_t::reproduce_quadratic);
    kernel_parallelRun_correctionFunction_Quadratic.build();
    kernel_parallelRun_correctionFunction_Quadratic.apply(&localElement_correctionFunction_Quadratic_fieldValues);
    kernel_parallelRun_correctionFunction_Quadratic.apply(&localElement_fieldValues,
                                                          &localElement_correctionFunction_Quadratic_gradientValues);

    KernelFilter kernel_serialRun_correctionFunction_Quadratic(&authority,
                                                               &input_data,
                                                               &global_points,
                                                               &parallelExchanger_serialRun);
    kernel_serialRun_correctionFunction_Quadratic.build();
    kernel_serialRun_correctionFunction_Quadratic.apply(&globalElement_correctionFunction_Quadratic_fieldValues);
    kernel_serialRun_correctionFunction_Quadratic.apply(&globalElement_fieldValues,
                                                        &globalElement_correctionFunction_Quadratic_gradientValues);

    KernelFilter kernel_parallelRun_minimalChange_Linear(&authority,
                                                         &input_data,
                                                         &local_points,
                                                         &parallelExchanger_parallelRun);
    input_data.set_normalization(normalization_t::minimal_change_to_reproducing_conditions);
    input_data.set_reproduction(reproduction_level_t::reproduce_linear);
    kernel_parallelRun_minimalChange_Linear.build();
    kernel_parallelRun_minimalChange_Linear.apply(&localElement_minimalChange_Linear_fieldValues);
    kernel_parallelRun_minimalChange_Linear.apply(&localElement_fieldValues, &localElement_minimalChange_Linear_gradientValues);

    KernelFilter kernel_serialRun_minimalChange_Linear(&authority,
                                                       &input_data,
                                                       &global_points,
                                                       &parallelExchanger_serialRun);
    kernel_serialRun_minimalChange_Linear.build();
    kernel_serialRun_minimalChange_Linear.apply(&globalElement_minimalChange_Linear_fieldValues);
    kernel_serialRun_minimalChange_Linear.apply(&globalElement_fieldValues, &globalElement_minimalChange_Linear_gradientValues);

    KernelFilter kernel_parallelRun_minimalChange_Quadratic(&authority,
                                                            &input_data,
                                                            &local_points,
                                                            &parallelExchanger_parallelRun);
    input_data.set_normalization(normalization_t::minimal_change_to_reproducing_conditions);
    input_data.set_reproduction(reproduction_level_t::reproduce_quadratic);
    kernel_parallelRun_minimalChange_Quadratic.build();
    kernel_parallelRun_minimalChange_Quadratic.apply(&localElement_minimalChange_Quadratic_fieldValues);
    kernel_parallelRun_minimalChange_Quadratic.apply(&localElement_fieldValues,
                                                     &localElement_minimalChange_Quadratic_gradientValues);

    KernelFilter kernel_serialRun_minimalChange_Quadratic(&authority,
                                                          &input_data,
                                                          &global_points,
                                                          &parallelExchanger_serialRun);
    input_data.set_normalization(normalization_t::minimal_change_to_reproducing_conditions);
    input_data.set_reproduction(reproduction_level_t::reproduce_quadratic);
    kernel_serialRun_minimalChange_Quadratic.build();
    kernel_serialRun_minimalChange_Quadratic.apply(&globalElement_minimalChange_Quadratic_fieldValues);
    kernel_serialRun_minimalChange_Quadratic.apply(&globalElement_fieldValues,
                                                   &globalElement_minimalChange_Quadratic_gradientValues);

    const double tol = 1e-5;
    for(size_t local_index_check = 0; local_index_check < num_local_points; local_index_check++)
    {
        EXPECT_NEAR(localElement_fieldValues.get_value(local_index_check),
                    globalElement_fieldValues.get_value(local_element_index_offset_begin+local_index_check),
                    tol);
        EXPECT_NEAR(localElement_gradientValues.get_value(local_index_check),
                    globalElement_gradientValues.get_value(local_element_index_offset_begin+local_index_check),
                    tol);

        EXPECT_NEAR(localElement_correctionFunction_Linear_fieldValues.get_value(local_index_check),
                    globalElement_correctionFunction_Linear_fieldValues.get_value(local_element_index_offset_begin+local_index_check),
                    tol);
        EXPECT_NEAR(localElement_correctionFunction_Linear_gradientValues.get_value(local_index_check),
                    globalElement_correctionFunction_Linear_gradientValues.get_value(local_element_index_offset_begin+local_index_check),
                    tol);
        EXPECT_NEAR(localElement_correctionFunction_Quadratic_fieldValues.get_value(local_index_check),
                    globalElement_correctionFunction_Quadratic_fieldValues.get_value(local_element_index_offset_begin+local_index_check),
                    tol);
        EXPECT_NEAR(localElement_correctionFunction_Quadratic_gradientValues.get_value(local_index_check),
                    globalElement_correctionFunction_Quadratic_gradientValues.get_value(local_element_index_offset_begin+local_index_check),
                    tol);

        EXPECT_NEAR(localElement_minimalChange_Linear_fieldValues.get_value(local_index_check),
                    globalElement_minimalChange_Linear_fieldValues.get_value(local_element_index_offset_begin+local_index_check),
                    tol);
        EXPECT_NEAR(localElement_minimalChange_Linear_gradientValues.get_value(local_index_check),
                    globalElement_minimalChange_Linear_gradientValues.get_value(local_element_index_offset_begin+local_index_check),
                    tol);
        EXPECT_NEAR(localElement_minimalChange_Quadratic_fieldValues.get_value(local_index_check),
                    globalElement_minimalChange_Quadratic_fieldValues.get_value(local_element_index_offset_begin+local_index_check),
                    tol);
        EXPECT_NEAR(localElement_minimalChange_Quadratic_gradientValues.get_value(local_index_check),
                    globalElement_minimalChange_Quadratic_gradientValues.get_value(local_element_index_offset_begin+local_index_check),
                    tol);
    }
}

void kernel_filter_test_two_methods(AbstractAuthority* authority,
                                    KernelFilter* kernel_one,
                                    KernelFilter* kernel_two,
                                    size_t multiplier)
{
    // test two methods for consistency
    const size_t mpi_rank = authority->mpi_wrapper->get_rank();
    const size_t mpi_size = authority->mpi_wrapper->get_size();

    const size_t num_horiz_points = 3u * multiplier;
    const size_t num_vert_points = 3u * multiplier;

    const size_t num_local_points = num_horiz_points * num_vert_points;

    const double horiz_step = 1.0;
    const double vert_step = 1.0;

    const double zmin = 0.0;

    // proc's are stacked vertically in x
    const double local_x_base = mpi_rank * num_horiz_points * horiz_step;
    example::Interface_PointCloud local_points;
    build_brick_of_points(&local_points, num_vert_points, num_horiz_points, horiz_step, vert_step, local_x_base, 0., zmin);

    // fill the element field arbitrarily, but not randomly
    std::vector<double> element_field(num_local_points, 0.0);
    for(size_t local_point = 0; local_point < num_local_points; local_point++)
    {
        element_field[local_point] = 1.0 / (1.0 + ((mpi_rank + local_point) % 13));
    }

    // point clouds
    kernel_one->set_points(&local_points);
    kernel_two->set_points(&local_points);

    // parallel exchanger
    example::Interface_ParallelExchanger_localAndNonlocal exchanger(authority);
    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data(mpi_size);
    exchanger.put_shared_pairs(shared_node_data);
    exchanger.put_num_local_locations(num_local_points);
    exchanger.build();
    kernel_one->set_parallel_exchanger(&exchanger);
    kernel_two->set_parallel_exchanger(&exchanger);

    // apply first kernel
    example::Interface_ParallelVector field_one(element_field);
    double kernel_one_time = -getTimeInSeconds();
    kernel_one->build();
    kernel_one_time += getTimeInSeconds();
    kernel_one->apply(&field_one);

    // apply second kernel
    example::Interface_ParallelVector field_two(element_field);
    double kernel_two_time = -getTimeInSeconds();
    kernel_two->build();
    kernel_two_time += getTimeInSeconds();
    kernel_two->apply(&field_two);

    // expect allocation methods to have identical results
    for(size_t local_point = 0; local_point < num_local_points; local_point++)
    {
        EXPECT_FLOAT_EQ(field_one.get_value(local_point), field_two.get_value(local_point));
    }

    // print
    if(0u == authority->mpi_wrapper->get_rank())
    {
        std::stringstream stream;
        stream << "k1:" << kernel_one_time << ",k2:" << kernel_two_time << std::endl;
        authority->utilities->print(stream.str());
    }
}

PSL_TEST(KernelFilter,searchMethodsBruteForceToMorton)
{
    set_rand_seed();
    AbstractAuthority authority;

    ParameterData inputData_bruteForce;
    inputData_bruteForce.set_absolute(3.5);
    inputData_bruteForce.set_iterations(1);
    inputData_bruteForce.set_penalty(1.);
    inputData_bruteForce.set_node_resolution_tolerance(1e-6);
    inputData_bruteForce.set_spatial_searcher(spatial_searcher_t::brute_force_fixed_radius_nearest_neighbors);
    inputData_bruteForce.set_normalization(normalization_t::classical_row_normalization);
    inputData_bruteForce.set_reproduction(reproduction_level_t::reproduce_constant);
    inputData_bruteForce.set_matrix_assembly_agent(matrix_assembly_agent_t::by_row);
    inputData_bruteForce.set_symmetry_plane_agent(symmetry_plane_agent_t::by_narrow_clone);
    inputData_bruteForce.set_mesh_scale_agent(mesh_scale_agent_t::by_average_optimized_element_side);
    inputData_bruteForce.set_matrix_normalization_agent(matrix_normalization_agent_t::default_agent);
    inputData_bruteForce.set_point_ghosting_agent(point_ghosting_agent_t::by_narrow_share);
    inputData_bruteForce.set_bounded_support_function(bounded_support_function_t::polynomial_tent_function);

    // brute force
    KernelFilter kernel_bruteForce(&authority,
                                   &inputData_bruteForce,
                                   NULL,
                                   NULL);

    // Morton search
    ParameterData inputData_Morton = inputData_bruteForce;
    inputData_Morton.set_spatial_searcher(spatial_searcher_t::bounding_box_morton_hierarchy);
    KernelFilter kernel_MortonSearch(&authority,
                                     &inputData_Morton,
                                     NULL,
                                     NULL);

    // compare
    kernel_filter_test_two_methods(&authority, &kernel_bruteForce, &kernel_MortonSearch, 1u);
}

PSL_TEST(KernelFilter,searchMethodsMortonToRadixGrid)
{
    set_rand_seed();
    AbstractAuthority authority;

    ParameterData inputData_Morton;
    inputData_Morton.set_absolute(3.5);
    inputData_Morton.set_iterations(1);
    inputData_Morton.set_penalty(1.);
    inputData_Morton.set_node_resolution_tolerance(1e-6);
    inputData_Morton.set_spatial_searcher(spatial_searcher_t::bounding_box_morton_hierarchy);
    inputData_Morton.set_normalization(normalization_t::classical_row_normalization);
    inputData_Morton.set_reproduction(reproduction_level_t::reproduce_constant);
    inputData_Morton.set_matrix_assembly_agent(matrix_assembly_agent_t::by_row);
    inputData_Morton.set_symmetry_plane_agent(symmetry_plane_agent_t::by_narrow_clone);
    inputData_Morton.set_mesh_scale_agent(mesh_scale_agent_t::by_average_optimized_element_side);
    inputData_Morton.set_matrix_normalization_agent(matrix_normalization_agent_t::default_agent);
    inputData_Morton.set_point_ghosting_agent(point_ghosting_agent_t::by_narrow_share);
    inputData_Morton.set_bounded_support_function(bounded_support_function_t::polynomial_tent_function);

    // brute force
    KernelFilter kernel_Morton(&authority,
                               &inputData_Morton,
                               NULL,
                               NULL);

    // Morton search
    ParameterData inputData_RadixGrid = inputData_Morton;
    inputData_RadixGrid.set_spatial_searcher(spatial_searcher_t::radix_grid_fixed_radius_nearest_neighbors);
    KernelFilter kernel_RadixGrid(&authority,
                                  &inputData_RadixGrid,
                                  NULL,
                                  NULL);

    // compare
    kernel_filter_test_two_methods(&authority, &kernel_Morton, &kernel_RadixGrid, 5u);//500u);
}

PSL_TEST(KernelFilter,reproduceConstant)
{
    set_rand_seed();
    AbstractAuthority authority;

    // kernel filter should reproduce a constant field
    size_t mpi_rank = authority.mpi_wrapper->get_rank();
    size_t mpi_size = authority.mpi_wrapper->get_size();

    const size_t num_horiz_points = 10;
    const size_t num_vert_points = 10;
    const size_t num_local_points = num_horiz_points * num_vert_points;
    const double horiz_step = 1.;
    const double vert_step = 1.;
    const double zmin = 0.;
    const double filter_radius = vert_step * 3.5;

    // input data
    ParameterData input_data;
    input_data.set_absolute(filter_radius);
    input_data.set_iterations(1);
    input_data.set_penalty(1.);
    input_data.set_node_resolution_tolerance(1e-6);
    input_data.set_spatial_searcher(spatial_searcher_t::recommended);
    input_data.set_normalization(normalization_t::classical_row_normalization);
    input_data.set_reproduction(reproduction_level_t::reproduce_constant);
    input_data.set_matrix_assembly_agent(matrix_assembly_agent_t::by_row);
    input_data.set_symmetry_plane_agent(symmetry_plane_agent_t::by_narrow_clone);
    input_data.set_mesh_scale_agent(mesh_scale_agent_t::by_average_optimized_element_side);
    input_data.set_matrix_normalization_agent(matrix_normalization_agent_t::default_agent);
    input_data.set_point_ghosting_agent(point_ghosting_agent_t::by_narrow_share);
    input_data.set_bounded_support_function(bounded_support_function_t::polynomial_tent_function);

    // parallel exchanger
    example::Interface_ParallelExchanger_localAndNonlocal exchanger(&authority);
    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data(mpi_size);
    exchanger.put_shared_pairs(shared_node_data);
    exchanger.put_num_local_locations(num_local_points);
    exchanger.build();

    // proc's are stacked vertically in x
    const double local_x_base = mpi_rank * num_horiz_points * horiz_step;
    example::Interface_PointCloud local_points;
    build_brick_of_points(&local_points, num_vert_points, num_horiz_points, horiz_step, vert_step, local_x_base, 0., zmin);

    const double constant_value = 0.31415;
    std::vector<double> element_field(num_local_points, constant_value);

    KernelFilter kernel_filter(&authority,
                               &input_data,
                               &local_points,
                               &exchanger);
    kernel_filter.build();

    example::Interface_ParallelVector field(element_field);
    kernel_filter.apply(&field);

    for(size_t local_point = 0; local_point < num_local_points; local_point++)
    {
        EXPECT_FLOAT_EQ(constant_value, field.get_value(local_point));
    }
}

PSL_TEST(KernelFilter,reproduceLinear)
{
    set_rand_seed();
    AbstractAuthority authority;

    // kernel filter should reproduce a spatially linear field
    size_t mpi_rank = authority.mpi_wrapper->get_rank();
    size_t mpi_size = authority.mpi_wrapper->get_size();

    const size_t num_horiz_points = 10;
    const size_t num_vert_points = 10;
    const size_t num_local_points = num_horiz_points * num_vert_points;
    const double horiz_step = 1.;
    const double vert_step = 1.;
    const double zmin = 0.;
    const double filter_radius = vert_step * 3.5;

    // input data
    ParameterData input_data;
    input_data.set_absolute(filter_radius);
    input_data.set_iterations(1);
    input_data.set_penalty(1.);
    input_data.set_node_resolution_tolerance(1e-6);
    input_data.set_spatial_searcher(spatial_searcher_t::recommended);
    input_data.set_normalization(normalization_t::classical_row_normalization);
    input_data.set_reproduction(reproduction_level_t::reproduce_constant);
    input_data.set_matrix_assembly_agent(matrix_assembly_agent_t::by_row);
    input_data.set_symmetry_plane_agent(symmetry_plane_agent_t::by_narrow_clone);
    input_data.set_mesh_scale_agent(mesh_scale_agent_t::by_average_optimized_element_side);
    input_data.set_matrix_normalization_agent(matrix_normalization_agent_t::default_agent);
    input_data.set_point_ghosting_agent(point_ghosting_agent_t::by_narrow_share);
    input_data.set_bounded_support_function(bounded_support_function_t::polynomial_tent_function);

    // parallel exchanger
    example::Interface_ParallelExchanger_localAndNonlocal exchanger(&authority);
    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data(mpi_size);
    exchanger.put_shared_pairs(shared_node_data);
    exchanger.put_num_local_locations(num_local_points);
    exchanger.build();

    // proc's are stacked vertically in x
    const double local_x_base = mpi_rank * num_horiz_points * horiz_step;
    example::Interface_PointCloud local_points;
    build_brick_of_points(&local_points, num_vert_points, num_horiz_points, horiz_step, vert_step, local_x_base, 0., zmin);

    // build fields, these numbers are arbitrary
    const double constant_value = 0.31415;
    const double x_slope = 0.2175;
    const double y_slope = -0.4327;
    const double z_slope = -0.2022;
    std::vector<double> elementField_gold(num_local_points);
    for(size_t local_point = 0; local_point < num_local_points; local_point++)
    {
        Point this_point = local_points.get_point(local_point);
        const double x = this_point(0);
        const double y = this_point(1);
        const double z = this_point(2);
        elementField_gold[local_point] = constant_value + x_slope * x + y_slope * y + z_slope * z;
    }
    example::Interface_ParallelVector elementField_correctionFunction(elementField_gold);
    example::Interface_ParallelVector elementField_minimalChange(elementField_gold);

    // build filter
    KernelFilter kernelFilter_correctionFunction(&authority,
                                                 &input_data,
                                                 &local_points,
                                                 &exchanger);
    input_data.set_reproduction(reproduction_level_t::reproduce_linear);
    input_data.set_normalization(normalization_t::correction_function_reproducing_conditions);
    kernelFilter_correctionFunction.build();
    kernelFilter_correctionFunction.apply(&elementField_correctionFunction);

    // build filter
    KernelFilter kernelFilter_minimalChange(&authority,
                                            &input_data,
                                            &local_points,
                                            &exchanger);
    input_data.set_normalization(normalization_t::minimal_change_to_reproducing_conditions);
    kernelFilter_minimalChange.build();
    kernelFilter_minimalChange.apply(&elementField_minimalChange);

    // check
    for(size_t local_point = 0; local_point < num_local_points; local_point++)
    {
        EXPECT_FLOAT_EQ(elementField_correctionFunction.get_value(local_point), elementField_gold[local_point]);
        EXPECT_FLOAT_EQ(elementField_minimalChange.get_value(local_point), elementField_gold[local_point]);
    }
}

PSL_TEST(KernelFilter,reproduceQuadratic)
{
    set_rand_seed();
    AbstractAuthority authority;

    // kernel filter should reproduce a spatially quadratic field
    size_t mpi_rank = authority.mpi_wrapper->get_rank();
    size_t mpi_size = authority.mpi_wrapper->get_size();

    const size_t num_horiz_points = 10;
    const size_t num_vert_points = 10;
    const size_t num_local_points = num_horiz_points * num_vert_points;
    const double horiz_step = 1.;
    const double vert_step = 1.;
    const double zmin = 0.;
    const double filter_radius = vert_step * 3.5;

    // input data
    ParameterData input_data;
    input_data.set_absolute(filter_radius);
    input_data.set_iterations(1);
    input_data.set_penalty(1.);
    input_data.set_node_resolution_tolerance(1e-6);
    input_data.set_spatial_searcher(spatial_searcher_t::recommended);
    input_data.set_normalization(normalization_t::classical_row_normalization);
    input_data.set_reproduction(reproduction_level_t::reproduce_constant);
    input_data.set_matrix_assembly_agent(matrix_assembly_agent_t::by_row);
    input_data.set_symmetry_plane_agent(symmetry_plane_agent_t::by_narrow_clone);
    input_data.set_mesh_scale_agent(mesh_scale_agent_t::by_average_optimized_element_side);
    input_data.set_matrix_normalization_agent(matrix_normalization_agent_t::default_agent);
    input_data.set_point_ghosting_agent(point_ghosting_agent_t::by_narrow_share);
    input_data.set_bounded_support_function(bounded_support_function_t::polynomial_tent_function);

    // parallel exchanger
    example::Interface_ParallelExchanger_localAndNonlocal exchanger(&authority);
    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data(mpi_size);
    exchanger.put_shared_pairs(shared_node_data);
    exchanger.put_num_local_locations(num_local_points);
    exchanger.build();

    // proc's are stacked vertically in x
    const double local_x_base = mpi_rank * num_horiz_points * horiz_step;
    example::Interface_PointCloud local_points;
    build_brick_of_points(&local_points, num_vert_points, num_horiz_points, horiz_step, vert_step, local_x_base, 0., zmin);

    // these numbers are arbitrary
    const double constant_value = 0.3142;
    const double x_slope = 0.2175;
    const double y_slope = -0.4327;
    const double z_slope = -0.2022;
    const double xx_slope = 0.4175;
    const double yy_slope = 0.2436;
    const double zz_slope = 0.1235;
    const double xy_slope = -0.1137;
    const double xz_slope = 0.3711;
    const double yz_slope = 0.3117;
    std::vector<double> elementField_gold(num_local_points);
    for(size_t local_point = 0; local_point < num_local_points; local_point++)
    {
        Point this_point = local_points.get_point(local_point);
        const double x = this_point(0);
        const double y = this_point(1);
        const double z = this_point(2);
        elementField_gold[local_point] = constant_value
                                         + x_slope * x + y_slope * y + z_slope * z
                                         + xx_slope * x * x
                                         + yy_slope * y * y
                                         + zz_slope * z * z
                                         + xy_slope * x * y
                                         + xz_slope * x * z
                                         + yz_slope * y * z;
    }
    example::Interface_ParallelVector elementField_correctionFunction(elementField_gold);
    example::Interface_ParallelVector elementField_minimalChange(elementField_gold);

    // build filter
    KernelFilter kernelFilter_correctionFunction(&authority,
                                                 &input_data,
                                                 &local_points,
                                                 &exchanger);
    input_data.set_reproduction(reproduction_level_t::reproduce_quadratic);
    input_data.set_normalization(normalization_t::correction_function_reproducing_conditions);
    kernelFilter_correctionFunction.build();
    kernelFilter_correctionFunction.apply(&elementField_correctionFunction);

    // build filter
    KernelFilter kernelFilter_minimalChange(&authority,
                                            &input_data,
                                            &local_points,
                                            &exchanger);
    input_data.set_normalization(normalization_t::minimal_change_to_reproducing_conditions);
    kernelFilter_minimalChange.build();
    kernelFilter_minimalChange.apply(&elementField_minimalChange);

    for(size_t local_point = 0; local_point < num_local_points; local_point++)
    {
        EXPECT_FLOAT_EQ(elementField_correctionFunction.get_value(local_point), elementField_gold[local_point]);
        EXPECT_FLOAT_EQ(elementField_minimalChange.get_value(local_point), elementField_gold[local_point]);
    }
}

PSL_TEST(KernelFilter,consistentExchangers)
{
    set_rand_seed();
    AbstractAuthority authority;

    const size_t mpi_rank = authority.mpi_wrapper->get_rank();
    const size_t mpi_size = authority.mpi_wrapper->get_size();

    // build mesh
    const size_t xlen = 3;
    const size_t ylen = 5;
    const size_t zlen = 7;
    const double xdist = 1.;
    const double ydist = 1.;
    const double zdist = 1.;
    example::ElementBlock modular_block;
    modular_block.build_from_structured_grid(xlen, ylen, zlen, xdist, ydist, zdist, mpi_rank, mpi_size);
    example::Interface_MeshModular modular_interface;
    modular_interface.set_mesh(&modular_block);

    // input data
    ParameterData input_data;
    input_data.set_absolute(3.5);
    input_data.set_iterations(1);
    input_data.set_penalty(1.);
    input_data.set_node_resolution_tolerance(1e-6);
    input_data.set_spatial_searcher(spatial_searcher_t::recommended);
    input_data.set_normalization(normalization_t::classical_row_normalization);
    input_data.set_reproduction(reproduction_level_t::reproduce_constant);
    input_data.set_matrix_assembly_agent(matrix_assembly_agent_t::by_row);
    input_data.set_symmetry_plane_agent(symmetry_plane_agent_t::by_narrow_clone);
    input_data.set_mesh_scale_agent(mesh_scale_agent_t::by_average_optimized_element_side);
    input_data.set_matrix_normalization_agent(matrix_normalization_agent_t::default_agent);
    input_data.set_point_ghosting_agent(point_ghosting_agent_t::by_narrow_share);
    input_data.set_bounded_support_function(bounded_support_function_t::polynomial_tent_function);

    // parallel exchanger localAndNonlocal
    example::Interface_ParallelExchanger_localAndNonlocal exchanger_localAndNonlocal(&authority);
    std::vector<std::vector<std::pair<size_t, size_t> > > shared_node_data;
    modular_block.get_shared_node_data(shared_node_data);
    exchanger_localAndNonlocal.put_shared_pairs(shared_node_data);
    const size_t local_num_nodes = modular_block.get_num_nodes();
    exchanger_localAndNonlocal.put_num_local_locations(local_num_nodes);
    exchanger_localAndNonlocal.build();

    // parallel exchanger globals
    example::Interface_ParallelExchanger_global exchanger_global(&authority);
    std::vector<size_t> global_ids;
    modular_block.get_global_ids(global_ids);
    exchanger_global.put_globals(global_ids);
    exchanger_global.build();

    // arbitrary parallel consistent field
    std::vector<double> field(local_num_nodes);
    for(size_t local_point = 0; local_point < local_num_nodes; local_point++)
    {
        field[local_point] = 2.4 / (0.13 + 0.41 * (global_ids[local_point] % 5u));
    }
    example::Interface_ParallelVector field_localAndNonlocal(field);
    example::Interface_ParallelVector field_global(field);
    example::Interface_ParallelVector field_validate(field);

    // build filter
    KernelFilter kernelFilter_searcher_localAndNonlocal(&authority,
                                                        &input_data,
                                                        &modular_interface,
                                                        &exchanger_localAndNonlocal);
    kernelFilter_searcher_localAndNonlocal.build();
    EXPECT_EQ(kernelFilter_searcher_localAndNonlocal.is_valid(&field_validate), true);
    kernelFilter_searcher_localAndNonlocal.apply(&field_localAndNonlocal);

    // build filter
    KernelFilter kernelFilter_global(&authority,
                                     &input_data,
                                     &modular_interface,
                                     &exchanger_global);
    kernelFilter_global.build();
    EXPECT_EQ(kernelFilter_global.is_valid(&field_validate), true);
    kernelFilter_global.apply(&field_global);

    for(size_t local_point = 0; local_point < local_num_nodes; local_point++)
    {
        EXPECT_FLOAT_EQ(field_localAndNonlocal.get_value(local_point), field_global.get_value(local_point));
    }

    // check parallel errors
    const double parallel_error_tolerance = 1e-10;
    const double lanl_max_error = exchanger_localAndNonlocal.get_maximum_absolute_parallel_error(&field_localAndNonlocal);
    EXPECT_GT(parallel_error_tolerance, lanl_max_error);
    const double global_max_error = exchanger_global.get_maximum_absolute_parallel_error(&field_global);
    EXPECT_GT(parallel_error_tolerance, global_max_error);
}

}

}
