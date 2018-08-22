// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_KernelFilter.hpp"

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

#include <cassert>
#include <vector>
#include <cstddef>
#include <sstream>
#include <cmath>

namespace PlatoSubproblemLibrary
{

KernelFilter::KernelFilter(AbstractAuthority* authority,
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
        m_bounded_support_function(),
        m_mesh_scale_agent(),
        m_symmetry_plane_agent(),
        m_matrix_assembly_agent(),
        m_matrix_normalization_agent(),
        m_point_ghosting_agent(),
        m_local_kernel_matrix(),
        m_parallel_block_row_kernel_matrices(),
        m_parallel_block_column_kernel_matrices(),
        m_maintain_kernel_points(false),
        m_kernel_points()
{
}

KernelFilter::~KernelFilter()
{
    m_authority = NULL;
    m_input_data = NULL;
    m_original_points = NULL;
    m_parallel_exchanger = NULL;

    safe_free(m_bounded_support_function);
    safe_free(m_mesh_scale_agent);
    safe_free(m_symmetry_plane_agent);
    safe_free(m_matrix_assembly_agent);
    safe_free(m_matrix_normalization_agent);
    safe_free(m_point_ghosting_agent);
    safe_free(m_local_kernel_matrix);
    const size_t num_row_matrices = m_parallel_block_row_kernel_matrices.size();
    for(size_t index = 0u; index < num_row_matrices; index++)
    {
        safe_free(m_parallel_block_row_kernel_matrices[index]);
    }
    m_parallel_block_row_kernel_matrices.clear();
    const size_t num_column_matrices = m_parallel_block_column_kernel_matrices.size();
    for(size_t index = 0u; index < num_column_matrices; index++)
    {
        safe_free(m_parallel_block_column_kernel_matrices[index]);
    }
    m_parallel_block_column_kernel_matrices.clear();
    safe_free(m_kernel_points);
}

void KernelFilter::set_authority(AbstractAuthority* authority)
{
    m_authority = authority;
}
void KernelFilter::set_input_data(ParameterData* data)
{
    m_input_data = data;
}
void KernelFilter::set_points(AbstractInterface::PointCloud* points)
{
    m_original_points = points;
}
void KernelFilter::set_parallel_exchanger(AbstractInterface::ParallelExchanger* exchanger)
{
    m_parallel_exchanger = exchanger;
}

void KernelFilter::announce_radius()
{
    m_announce_radius = true;
}
void KernelFilter::enable_maintain_kernel_points()
{
    m_maintain_kernel_points = true;
}

void KernelFilter::build()
{
    assert(m_authority);
    assert(m_authority->utilities);
    if(m_built)
    {
        m_authority->utilities->print("KernelFilter attempted to be built multiple times. Warning.\n\n");
        return;
    }
    m_built = true;

    // check functionalities
    check_required_functionalities();

    // check parameters
    check_input_data();

    // build all agents
    build_agents();

    // determine radial function
    determine_function();

    // separate indexes with parallel agent
    assert(m_parallel_exchanger);
    std::vector<size_t> indexes_of_local_points = m_parallel_exchanger->get_local_contracted_indexes();

    // clone indexes with symmetry plane agent
    double point_resolution_tolerance = 0.;
    if(m_input_data->didUserInput_node_resolution_tolerance())
    {
        point_resolution_tolerance = m_input_data->get_node_resolution_tolerance();
    }
    else
    {
        point_resolution_tolerance = m_mesh_scale_agent->get_mesh_minimum_scale() / 10.0;
        m_input_data->set_node_resolution_tolerance(point_resolution_tolerance);
    }

    m_kernel_points = m_symmetry_plane_agent->build_kernel_points(m_bounded_support_function->get_support(),
                                                                  point_resolution_tolerance,
                                                                  m_input_data,
                                                                  m_original_points,
                                                                  indexes_of_local_points);

    // build ghosted kernel points
    std::vector<PointCloud*> nonlocal_kernel_points;
    std::vector<size_t> processor_neighbors_below;
    std::vector<size_t> processor_neighbors_above;
    m_point_ghosting_agent->share(m_bounded_support_function->get_support(),
                                  m_kernel_points,
                                  nonlocal_kernel_points,
                                  processor_neighbors_below,
                                  processor_neighbors_above);

    // assemble kernel matrix with matrix assembly agent
    m_matrix_assembly_agent->build(m_bounded_support_function,
                                   m_kernel_points,
                                   nonlocal_kernel_points,
                                   processor_neighbors_below,
                                   processor_neighbors_above,
                                   &m_local_kernel_matrix,
                                   m_parallel_block_row_kernel_matrices,
                                   m_parallel_block_column_kernel_matrices);

    // normalize kernel matrix with matrix normalization agent
    m_matrix_normalization_agent->normalize(m_kernel_points,
                                            nonlocal_kernel_points,
                                            m_local_kernel_matrix,
                                            m_parallel_block_row_kernel_matrices,
                                            m_parallel_block_column_kernel_matrices,
                                            processor_neighbors_below,
                                            processor_neighbors_above);

    // clean up
    if(!m_maintain_kernel_points)
    {
        safe_free(m_kernel_points);
    }
    safe_free(nonlocal_kernel_points);
    nonlocal_kernel_points.clear();
}

void KernelFilter::apply(AbstractInterface::ParallelVector* field)
{
    internal_apply(field, false);
}

void KernelFilter::apply(AbstractInterface::ParallelVector* base_field, AbstractInterface::ParallelVector* gradient)
{
    internal_apply(gradient, true);
}

bool KernelFilter::is_valid(AbstractInterface::ParallelVector* field)
{
    bool valid = true;

    // check that shared nodes are at the same location
    const double data_tol = 1e-6;
    const size_t local_num_points = m_original_points->get_num_points();
    std::vector<size_t> contracted_indexes = m_parallel_exchanger->get_local_contracted_indexes();
    const size_t contracted_num_nodes = contracted_indexes.size();
    std::vector<bool> transfer_all(local_num_points, true);
    PlatoSubproblemLibrary::PointCloud* points = m_original_points->build_point_cloud(transfer_all);
    for(size_t d = 0; d < 3u; d++)
    {
        std::vector<double> initial_contracted_data(contracted_num_nodes, 0.);
        for(size_t cn = 0u; cn < contracted_num_nodes; cn++)
        {
            initial_contracted_data[cn] = (*points->get_point(contracted_indexes[cn]))(d);
        }
        m_parallel_exchanger->get_expansion_to_parallel_vector(initial_contracted_data, field);
        for(size_t l = 0u; l < local_num_points; l++)
        {
            const double from_parallel_vector = field->get_value(l);
            const double from_local_information = (*points->get_point(l))(d);
            if(fabs(from_parallel_vector - from_local_information) > data_tol)
            {
                valid = false;
            }
        }
        std::vector<double> final_contracted_data = m_parallel_exchanger->get_contraction_to_local_indexes(field);
        for(size_t cn = 0u; cn < contracted_num_nodes; cn++)
        {
            const double initial_contracted = initial_contracted_data[cn];
            const double final_contracted = final_contracted_data[cn];
            if(fabs(initial_contracted - final_contracted) > data_tol)
            {
                valid = false;
            }
        }
    }

    delete points;
    return valid;
}

PointCloud* KernelFilter::internal_transfer_kernel_points()
{
    PointCloud* result = m_kernel_points;
    m_kernel_points = NULL;
    return result;
}

std::vector<double> KernelFilter::internal_get_field_at_kernel_points(AbstractInterface::ParallelVector* parallel_field)
{
    if(!m_built)
    {
        m_authority->utilities->fatal_error("KernelFilter applied before being built. Aborting.\n\n");
    }

    // contract to local indexes with parallel agent
    std::vector<double> field_at_local_points = m_parallel_exchanger->get_contraction_to_local_indexes(parallel_field);
    // clone field values with symmetry plane agent
    return m_symmetry_plane_agent->expand_with_symmetry_points(field_at_local_points);
}

void KernelFilter::internal_set_field_at_kernel_points(AbstractInterface::ParallelVector* parallel_field,
                                                       const std::vector<double>& field_at_kernel_points)
{
    if(!m_built)
    {
        m_authority->utilities->fatal_error("KernelFilter applied before being built. Aborting.\n\n");
    }

    // forget clones with symmetry plane agent
    std::vector<double> output_field_at_local_points =
            m_symmetry_plane_agent->contract_by_symmetry_points(field_at_kernel_points);
    // send and receive with parallel agent
    m_parallel_exchanger->get_expansion_to_parallel_vector(output_field_at_local_points, parallel_field);
}

void KernelFilter::internal_apply(AbstractInterface::ParallelVector* parallel_field, bool transpose)
{
    if(!m_built)
    {
        m_authority->utilities->fatal_error("KernelFilter applied before being built. Aborting.\n\n");
    }

    std::vector<double> field_at_kernel_points = internal_get_field_at_kernel_points(parallel_field);

    // matrix-vector product
    std::vector<double> output_field_at_kernel_points = internal_parallel_matvec_apply(field_at_kernel_points, transpose);

    internal_set_field_at_kernel_points(parallel_field, output_field_at_kernel_points);
}

std::vector<double> KernelFilter::internal_parallel_matvec_apply(const std::vector<double>& input, const bool transpose)
{
    const int num_iterations = m_input_data->get_iterations();
    std::vector<double> output(input);

    for(int iteration = 0; iteration < num_iterations; iteration++)
    {
        if(transpose)
        {
            parallel_matvec_apply_transpose(output);
        }
        else
        {
            parallel_matvec_apply_noTranspose(output);
        }
    }

    return output;
}

void KernelFilter::parallel_matvec_apply_transpose(std::vector<double>& field)
{
    const size_t field_size = field.size();
    const size_t mpi_rank = m_authority->mpi_wrapper->get_rank();
    const size_t mpi_size = m_authority->mpi_wrapper->get_size();

    // separate input and output
    std::vector<double> input(field);
    field.assign(field_size, 0.);

    // local matrix vector product
    m_local_kernel_matrix->matVec(input, field, true);

    // parallel matrix vector product
    for(size_t proc = 0; proc < mpi_size; proc++)
    {
        AbstractInterface::SparseMatrix* blockRowMatrix = m_parallel_block_row_kernel_matrices[proc];
        if(blockRowMatrix)
        {
            std::vector<double> data_to_send;
            blockRowMatrix->matVecToReduced(input, data_to_send, true);

            std::vector<size_t> reducedVector;
            blockRowMatrix->getNonZeroSortedRows(reducedVector);
            const size_t num_reduced = reducedVector.size();
            std::vector<double> data_to_recv(num_reduced);

            if(proc < mpi_rank)
            {
                m_authority->mpi_wrapper->send(proc, data_to_send);
                m_authority->mpi_wrapper->receive(proc, data_to_recv);
            }
            else
            {
                m_authority->mpi_wrapper->receive(proc, data_to_recv);
                m_authority->mpi_wrapper->send(proc, data_to_send);
            }

            for(size_t reduced_index = 0; reduced_index < num_reduced; reduced_index++)
            {
                const size_t local_id = reducedVector[reduced_index];
                field[local_id] += data_to_recv[reduced_index];
            }
        }
    }
}

void KernelFilter::parallel_matvec_apply_noTranspose(std::vector<double>& field)
{
    const size_t field_size = field.size();
    const size_t mpi_rank = m_authority->mpi_wrapper->get_rank();
    const size_t mpi_size = m_authority->mpi_wrapper->get_size();

    // separate input and output
    std::vector<double> input(field);
    field.assign(field_size, 0.);

    // local matrix vector product
    m_local_kernel_matrix->matVec(input, field, false);

    // parallel matrix vector product
    for(size_t proc = 0; proc < mpi_size; proc++)
    {
        AbstractInterface::SparseMatrix* blockColumnMatrix = m_parallel_block_column_kernel_matrices[proc];
        if(blockColumnMatrix)
        {
            std::vector<double> data_to_send;
            blockColumnMatrix->matVecToReduced(input, data_to_send, false);

            std::vector<size_t> reducedVector;
            blockColumnMatrix->getNonZeroSortedColumns(reducedVector);
            const size_t num_reduced = reducedVector.size();
            std::vector<double> data_to_recv(num_reduced);

            if(proc < mpi_rank)
            {
                m_authority->mpi_wrapper->send(proc, data_to_send);
                m_authority->mpi_wrapper->receive(proc, data_to_recv);
            }
            else
            {
                m_authority->mpi_wrapper->receive(proc, data_to_recv);
                m_authority->mpi_wrapper->send(proc, data_to_send);
            }

            for(size_t reduced_index = 0; reduced_index < num_reduced; reduced_index++)
            {
                const size_t local_id = reducedVector[reduced_index];
                field[local_id] += data_to_recv[reduced_index];
            }
        }
    }
}

void KernelFilter::check_required_functionalities()
{
    if(!m_authority->mpi_wrapper)
    {
        m_authority->utilities->fatal_error("KernelFilter attempted to be built without mpi wrapper specified. Aborting.\n\n");
    }
    if(!m_input_data)
    {
        m_authority->utilities->fatal_error("KernelFilter attempted to be built without input data specified. Aborting.\n\n");
    }
    if(!m_original_points)
    {
        m_authority->utilities->fatal_error("KernelFilter attempted to be built without points specified. Aborting.\n\n");
    }
    if(!m_parallel_exchanger)
    {
        m_authority->utilities->fatal_error("KernelFilter attempted to be built without parallel exchanger specified. Aborting.\n\n");
    }
    if(!m_authority->sparse_builder)
    {
        m_authority->utilities->fatal_error("KernelFilter attempted to be built without sparse matrix builder specified. Aborting.\n\n");
    }
    // m_dense_matrix_builder not strictly required
    if(!m_authority->dense_vector_operations)
    {
        m_authority->utilities->fatal_error("KernelFilter attempted to be built without dense vector operations specified. Aborting.\n\n");
    }
    // m_dense_solver not strictly required
}


void KernelFilter::check_input_data()
{
    if(!m_input_data->didUserInput_absolute() && !m_input_data->didUserInput_scale())
    {
        m_authority->utilities->fatal_error("KernelFilter: neither absolute nor scale parameters set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_iterations())
    {
        m_authority->utilities->fatal_error("KernelFilter: iterations not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_penalty())
    {
        m_authority->utilities->fatal_error("KernelFilter: penalty not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_spatial_searcher())
    {
        m_authority->utilities->fatal_error("KernelFilter: spatial searcher not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_normalization())
    {
        m_authority->utilities->fatal_error("KernelFilter: normalization not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_reproduction())
    {
        m_authority->utilities->fatal_error("KernelFilter: reproduction level not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_mesh_scale_agent())
    {
        m_authority->utilities->fatal_error("KernelFilter: mesh scale agent not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_symmetry_plane_agent())
    {
        m_authority->utilities->fatal_error("KernelFilter: symmetry plane agent not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_matrix_assembly_agent())
    {
        m_authority->utilities->fatal_error("KernelFilter: matrix assembly agent not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_matrix_normalization_agent())
    {
        m_authority->utilities->fatal_error("KernelFilter: matrix normalization agent not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_point_ghosting_agent())
    {
        m_authority->utilities->fatal_error("KernelFilter: point ghosting agent not set in input. Aborting.\n\n");
    }
    if(!m_input_data->didUserInput_bounded_support_function())
    {
        m_authority->utilities->fatal_error("KernelFilter: bounded support function not set in input. Aborting.\n\n");
    }
}

void KernelFilter::determine_function()
{
    double radius = -1.0;

    // determine radius
    if(m_input_data->didUserInput_absolute())
    {
        const double absolute = m_input_data->get_absolute();
        radius = absolute;
        if(m_announce_radius && (m_authority->mpi_wrapper->get_rank() == 0u))
        {
            std::stringstream stream;
            stream << "Kernel Filter: detected absolute radius was input, using radius = " << absolute << std::endl;
            m_authority->utilities->print(stream.str());
        }
    }
    else if(m_input_data->didUserInput_scale())
    {
        const double mesh_scale = m_mesh_scale_agent->get_mesh_average_scale();
        const double input_scale = m_input_data->get_scale();
        radius = mesh_scale * input_scale;
        if(m_announce_radius && (m_authority->mpi_wrapper->get_rank() == 0u))
        {
            std::stringstream stream;
            stream << "Kernel Filter: inferring radius from element length scales" << std::endl;
            stream << " determined average scale (" << mesh_scale << ") and input scale (" << input_scale << ")" << std::endl;
            stream << " using radius = " << radius << " = " << mesh_scale << " * " << input_scale << std::endl;
            m_authority->utilities->print(stream.str());
        }
    }
    else
    {
        m_authority->utilities->fatal_error("KernelFilter: neither absolute nor scale parameters set as input. Aborting.\n\n");
    }

    // build
    m_bounded_support_function = build_bounded_support_function(m_input_data->get_bounded_support_function(), m_authority);
    m_bounded_support_function->build(radius, m_input_data);
}

void KernelFilter::build_agents()
{
    // mesh scale agent
    build_mesh_scale_agent(m_input_data->get_mesh_scale_agent());

    // symmetry plane agent
    build_symmetry_plane_agent(m_input_data->get_symmetry_plane_agent());

    // matrix assembly agent
    build_matrix_assembly_agent(m_input_data->get_matrix_assembly_agent());

    // matrix normalization agent
    build_matrix_normalization_agent(m_input_data->get_matrix_normalization_agent());

    // point ghosting agent
    build_point_ghosting_agent(m_input_data->get_point_ghosting_agent());
}

void KernelFilter::build_mesh_scale_agent(mesh_scale_agent_t::mesh_scale_agent_t agent)
{
    // if already allocated, abort
    if(m_mesh_scale_agent)
    {
        m_authority->utilities->fatal_error("KernelFilter: attempted to build mesh scale agent multiple times. Aborting.\n\n");
    }

    // otherwise, build
    switch(agent)
    {
        case mesh_scale_agent_t::by_average_optimized_element_side:
        {
            m_mesh_scale_agent = new ByOptimizedElementSide_MeshScaleAgent(m_authority, m_original_points);
            break;
        }
        case mesh_scale_agent_t::unset_mesh_scale_agent:
        default:
        {
            m_authority->utilities->fatal_error("Unable to build kernel mesh scale agent in KernelFilter. Aborting.\n\n");
            break;
        }
    }
}

void KernelFilter::build_symmetry_plane_agent(symmetry_plane_agent_t::symmetry_plane_agent_t agent)
{
    // if already allocated, abort
    if(m_symmetry_plane_agent)
    {
        m_authority->utilities->fatal_error("KernelFilter: attempted to build symmetry plane agent multiple times. Aborting.\n\n");
    }

    // otherwise, build
    switch(agent)
    {
        case symmetry_plane_agent_t::by_narrow_clone:
        {
            m_symmetry_plane_agent = new ByNarrowClone_SymmetryPlaneAgent(m_authority);
            break;
        }
        case symmetry_plane_agent_t::unset_symmetry_plane_agent:
        default:
        {
            m_authority->utilities->fatal_error("Unable to build kernel symmetry plane assembly agent in KernelFilter. Aborting.\n\n");
            break;
        }
    }
}

void KernelFilter::build_matrix_assembly_agent(matrix_assembly_agent_t::matrix_assembly_agent_t agent)
{
    // if already allocated, abort
    if(m_matrix_assembly_agent)
    {
        m_authority->utilities->fatal_error("KernelFilter: attempted to build matrix assembly agent multiple times. Aborting.\n\n");
    }

    // otherwise, build
    switch(agent)
    {
        case matrix_assembly_agent_t::by_row:
        {
            m_matrix_assembly_agent = new ByRow_MatrixAssemblyAgent(m_authority,
                                                                    m_input_data);
            break;
        }
        case matrix_assembly_agent_t::unset_matrix_assembly_agent:
        default:
        {
            m_authority->utilities->fatal_error("Unable to build kernel matrix assembly agent in KernelFilter. Aborting.\n\n");
            break;
        }
    }
}

void KernelFilter::build_matrix_normalization_agent(matrix_normalization_agent_t::matrix_normalization_agent_t agent)
{
    // if already allocated, abort
    if(m_matrix_normalization_agent)
    {
        m_authority->utilities->fatal_error("KernelFilter: attempted to build matrix normalization agent multiple times. Aborting.\n\n");
    }

    // otherwise, build
    switch(agent)
    {
        case matrix_normalization_agent_t::default_agent:
        {
            m_matrix_normalization_agent = new Default_MatrixNormalizationAgent(m_authority,
                                                                                m_input_data);
            break;
        }
        case matrix_normalization_agent_t::unset_matrix_normalization_agent:
        default:
        {
            m_authority->utilities->fatal_error("Unable to build kernel matrix normalization agent in KernelFilter. Aborting.\n\n");
            break;
        }
    }
}

void KernelFilter::build_point_ghosting_agent(point_ghosting_agent_t::point_ghosting_agent_t agent)
{
    // if already allocated, abort
    if(m_point_ghosting_agent)
    {
        m_authority->utilities->fatal_error("KernelFilter: attempted to build point ghosting agent multiple times. Aborting.\n\n");
    }

    // otherwise, build
    switch(agent)
    {
        case point_ghosting_agent_t::by_narrow_share:
        {
            m_point_ghosting_agent = new ByNarrowShare_PointGhostingAgent(m_authority);
            break;
        }
        case point_ghosting_agent_t::unset_point_ghosting_agent:
        default:
        {
            m_authority->utilities->fatal_error("Unable to build point ghosting agent in KernelFilter. Aborting.\n\n");
            break;
        }
    }
}

}
