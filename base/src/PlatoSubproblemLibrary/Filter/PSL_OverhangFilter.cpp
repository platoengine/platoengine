// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_OverhangFilter.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_ParallelExchanger.hpp"
#include "PSL_Abstract_ParallelVector.hpp"
#include "PSL_Abstract_PointCloud.hpp"
#include "PSL_Abstract_MatrixAssemblyAgent.hpp"
#include "PSL_ByRow_MatrixAssemblyAgent.hpp"
#include "PSL_Abstract_SymmetryPlaneAgent.hpp"
#include "PSL_ByNarrowClone_SymmetryPlaneAgent.hpp"
#include "PSL_Abstract_MeshScaleAgent.hpp"
#include "PSL_ByOptimizedElementSide_MeshScaleAgent.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_Abstract_MatrixNormalizationAgent.hpp"
#include "PSL_Default_MatrixNormalizationAgent.hpp"
#include "PSL_Abstract_SparseMatrixBuilder.hpp"
#include "PSL_Abstract_FixedRadiusNearestNeighborsSearcher.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_SparseMatrix.hpp"
#include "PSL_Abstract_PositiveDefiniteLinearSolver.hpp"
#include "PSL_Abstract_PointGhostingAgent.hpp"
#include "PSL_ByNarrowShare_PointGhostingAgent.hpp"
#include "PSL_Abstract_BoundedSupportFunction.hpp"
#include "PSL_BoundedSupportFunctionFactory.hpp"
#include "PSL_Point.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_KernelFilter.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"

#include <cassert>
#include <vector>
#include <cstddef>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <math.h>

namespace PlatoSubproblemLibrary
{

OverhangFilter::OverhangFilter(AbstractAuthority* authority,
                               ParameterData* data,
                               AbstractInterface::PointCloud* points,
                               AbstractInterface::ParallelExchanger* exchanger) :
        Filter(),
        m_built(false),
        m_announce_radius(false),
        m_authority(authority),
        m_input_data(data),
        m_original_points(points),
        m_parallel_exchanger(exchanger),
        m_smoothing_kernel(NULL),
        m_overhang_kernel(NULL),
        m_max_overhang_angle(-1.),
        m_overhang_projection_angle_fraction(-1.),
        m_overhang_projection_radius_fraction(-1.),
        m_build_direction_x(-1.),
        m_build_direction_y(-1.),
        m_build_direction_z(-1.),
        m_current_heaviside_parameter(-1.),
        m_heaviside_parameter_continuation_scale(-1.),
        m_max_heaviside_parameter(-1.),
        m_overhang_fractional_threshold(-1.)
{
}

OverhangFilter::~OverhangFilter()
{
    m_authority = NULL;
    m_input_data = NULL;
    m_original_points = NULL;
    m_parallel_exchanger = NULL;
    safe_free(m_smoothing_kernel);
    safe_free(m_overhang_kernel);
}

void OverhangFilter::set_authority(AbstractAuthority* authority)
{
    m_authority = authority;
}
void OverhangFilter::set_input_data(ParameterData* data)
{
    m_input_data = data;
}
void OverhangFilter::set_points(AbstractInterface::PointCloud* points)
{
    m_original_points = points;
}
void OverhangFilter::set_parallel_exchanger(AbstractInterface::ParallelExchanger* exchanger)
{
    m_parallel_exchanger = exchanger;
}

void OverhangFilter::announce_radius()
{
    m_announce_radius = true;
    if(m_smoothing_kernel)
    {
        m_smoothing_kernel->announce_radius();
    }
    if(m_overhang_kernel)
    {
        m_overhang_kernel->announce_radius();
    }
}

void OverhangFilter::build()
{
    assert(m_authority);
    assert(m_authority->utilities);
    if(m_built)
    {
        m_authority->utilities->print("OverhangFilter attempted to be built multiple times. Warning.\n\n");
        return;
    }
    m_built = true;

    // check parameters
    check_input_data();

    // transfer parameters
    m_max_overhang_angle = m_input_data->get_max_overhang_angle();
    m_overhang_projection_angle_fraction = m_input_data->get_overhang_projection_angle_fraction();
    m_overhang_projection_radius_fraction = m_input_data->get_overhang_projection_radius_fraction();
    m_build_direction_x = m_input_data->get_build_direction_x();
    m_build_direction_y = m_input_data->get_build_direction_y();
    m_build_direction_z = m_input_data->get_build_direction_z();
    m_current_heaviside_parameter = m_input_data->get_min_heaviside_parameter();
    m_heaviside_parameter_continuation_scale = m_input_data->get_heaviside_continuation_scale();
    m_max_heaviside_parameter = m_input_data->get_max_heaviside_parameter();

    // normalize build direction
    const double build_direction_norm = std::sqrt(m_build_direction_x * m_build_direction_x
                                                  + m_build_direction_y * m_build_direction_y
                                                  + m_build_direction_z * m_build_direction_z);
    if(build_direction_norm < 1e-10)
    {
        m_authority->utilities->fatal_error("OverhangFilter: build direction is zero vector. Aborting.\n\n");
    }
    m_build_direction_x /= build_direction_norm;
    m_build_direction_y /= build_direction_norm;
    m_build_direction_z /= build_direction_norm;

    // overhang fraction threshold
    m_overhang_fractional_threshold = fractional_spherical_sector(m_overhang_projection_radius_fraction,
                                                                  m_overhang_projection_angle_fraction,
                                                                  m_max_overhang_angle);

    // build smoothing kernel
    safe_free(m_smoothing_kernel);
    m_input_data->set_bounded_support_function(bounded_support_function_t::bounded_support_function_t::polynomial_tent_function);
    m_smoothing_kernel = new KernelFilter(m_authority, m_input_data, m_original_points, m_parallel_exchanger);
    m_smoothing_kernel->enable_maintain_kernel_points();
    m_smoothing_kernel->build();
    if(m_announce_radius)
    {
        m_smoothing_kernel->announce_radius();
    }

    // get point cloud
    PointCloud* kernel_points = m_smoothing_kernel->internal_transfer_kernel_points();

    // build overhang kernel
    safe_free(m_overhang_kernel);
    m_input_data->set_bounded_support_function(bounded_support_function_t::bounded_support_function_t::overhang_inclusion_function);
    m_overhang_kernel = new KernelFilter(m_authority, m_input_data, m_original_points, m_parallel_exchanger);
    m_overhang_kernel->build();
    if(m_announce_radius)
    {
        m_overhang_kernel->announce_radius();
    }

    compute_overhang_bias(kernel_points);

    // clean up
    safe_free(kernel_points);
}

void OverhangFilter::apply(AbstractInterface::ParallelVector* field)
{
    if(!m_built)
    {
        m_authority->utilities->fatal_error("OverhangFilter applied before built. Aborting.\n\n");
        return;
    }

    // get local
    std::vector<double> input_field = m_smoothing_kernel->internal_get_field_at_kernel_points(field);

    // do partial computation
    std::vector<double> inner_preactivation;
    std::vector<double> inner_postactivation;
    std::vector<double> outer_preactivation;
    internal_apply(input_field, inner_preactivation, inner_postactivation, outer_preactivation);

    // do outer activation
    const size_t dimension = outer_preactivation.size();
    for(size_t i = 0u; i < dimension; i++)
    {
        outer_preactivation[i] = heaviside_apply(m_current_heaviside_parameter, .5, outer_preactivation[i]);
    }

    // get parallel
    m_smoothing_kernel->internal_set_field_at_kernel_points(field, outer_preactivation);
}
void OverhangFilter::apply(AbstractInterface::ParallelVector* base_field, AbstractInterface::ParallelVector* gradient)
{
    if(!m_built)
    {
        m_authority->utilities->fatal_error("OverhangFilter applied before built. Aborting.\n\n");
        return;
    }

    // get local
    std::vector<double> input_field = m_smoothing_kernel->internal_get_field_at_kernel_points(base_field);
    std::vector<double> input_gradient = m_smoothing_kernel->internal_get_field_at_kernel_points(gradient);

    // do partial computation
    std::vector<double> inner_preactivation;
    std::vector<double> inner_postactivation;
    std::vector<double> outer_preactivation;
    internal_apply(input_field, inner_preactivation, inner_postactivation, outer_preactivation);

    // working vector
    const size_t dimension = input_field.size();
    std::vector<double> working(dimension);

    // y0 = sigma_outer'(x4)
    for(size_t i = 0u; i < dimension; i++)
    {
        outer_preactivation[i] = heaviside_gradient(m_current_heaviside_parameter, .5, outer_preactivation[i]);
    }

    // y1 = in_grad .* y0
    m_authority->dense_vector_operations->multiply(input_gradient, outer_preactivation, working);

    // y2 = H' * y1
    std::vector<double> y2 = m_smoothing_kernel->internal_parallel_matvec_apply(working, true);

    // y3 = x2 .* y2
    std::vector<double> output_gradient;
    m_authority->dense_vector_operations->multiply(inner_postactivation, y2, output_gradient);

    // y4 = sigma_inner'(x1)
    for(size_t i = 0u; i < dimension; i++)
    {
        working[i] = heaviside_gradient(m_current_heaviside_parameter, m_overhang_fractional_threshold, inner_preactivation[i]);
    }

    // y5 = in .* y4 .* y2
    m_authority->dense_vector_operations->multiply(input_field, working);
    m_authority->dense_vector_operations->multiply(y2, working);

    // y6 =  S' * y5
    working = m_overhang_kernel->internal_parallel_matvec_apply(working, true);

    // out_grad = y3 + y6
    m_authority->dense_vector_operations->axpy(1., working, output_gradient);

    // get parallel
    m_smoothing_kernel->internal_set_field_at_kernel_points(gradient, output_gradient);
}
void OverhangFilter::advance_continuation()
{
    m_current_heaviside_parameter = std::min(m_current_heaviside_parameter * m_heaviside_parameter_continuation_scale,
                                             m_max_heaviside_parameter);
}

void OverhangFilter::check_input_data()
{
    if(!m_input_data->didUserInput_min_heaviside_parameter())
    {
        m_authority->utilities->fatal_error("OverhangFilter: min_heaviside_parameter not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_heaviside_continuation_scale())
    {
        m_authority->utilities->fatal_error("OverhangFilter: heaviside_continuation_scale not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_max_heaviside_parameter())
    {
        m_authority->utilities->fatal_error("OverhangFilter: max_heaviside_parameter not set in input. Aborting.\n\n");
    }
}

void OverhangFilter::compute_overhang_bias(PointCloud* kernel_points)
{
    // build direction
    std::vector<double> build_direction = {m_build_direction_x, m_build_direction_y, m_build_direction_z};

    // find local smallest inner product with build direction
    const size_t local_num_points = kernel_points->get_num_points();
    double local_smallest_inner_product = std::numeric_limits<double>::max();
    for(size_t p = 0u; p < local_num_points; p++)
    {
        // get point data
        std::vector<double> this_point;
        kernel_points->get_point(p)->get_data(this_point);
        double this_inp = m_authority->dense_vector_operations->dot(build_direction, this_point);
        local_smallest_inner_product = std::min(local_smallest_inner_product, this_inp);
    }

    // find global
    double global_smallest_inner_product = std::numeric_limits<double>::max();
    m_authority->mpi_wrapper->all_reduce_min(local_smallest_inner_product, global_smallest_inner_product);

    // compute bias
    const double point_tolerance = m_input_data->get_node_resolution_tolerance();
    m_overhang_bias.assign(local_num_points, 0.);
    for(size_t p = 0u; p < local_num_points; p++)
    {
        // get point data
        std::vector<double> this_point;
        kernel_points->get_point(p)->get_data(this_point);
        double this_inp = m_authority->dense_vector_operations->dot(build_direction, this_point);

        // if global minimum, then on build plate, and thus bias on
        if(std::fabs(this_inp - global_smallest_inner_product) < point_tolerance)
        {
            m_overhang_bias[p] = 1.;
        }
    }
}

void OverhangFilter::internal_apply(const std::vector<double>& input,
                                    std::vector<double>& inner_preactivation,
                                    std::vector<double>& inner_postactivation,
                                    std::vector<double>& outer_preactivation)
{
    // x0 = S * in
    std::vector<double> working = m_overhang_kernel->internal_parallel_matvec_apply(input, false);

    // x1 = x0 + s
    inner_preactivation = m_overhang_bias;
    m_authority->dense_vector_operations->axpy(1., working, inner_preactivation);

    // x2 = sigma_inner(x1)
    const size_t dimension = input.size();
    inner_postactivation.resize(dimension);
    for(size_t i = 0u; i < dimension; i++)
    {
        inner_postactivation[i] = heaviside_apply(m_current_heaviside_parameter,
                                                  m_overhang_fractional_threshold,
                                                  inner_preactivation[i]);
    }

    // x3 = in .* x2
    m_authority->dense_vector_operations->multiply(input, inner_postactivation, working);

    // x4 = H * x3
    outer_preactivation = m_smoothing_kernel->internal_parallel_matvec_apply(working, false);
}

}
