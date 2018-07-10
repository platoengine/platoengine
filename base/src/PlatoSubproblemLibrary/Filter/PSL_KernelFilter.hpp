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
class GlobalUtilities;
class MpiWrapper;
class SparseMatrix;
class SparseMatrixBuilder;
class FixedRadiusNearestNeighborsSearcher;
class DenseMatrixBuilder;
class DenseVectorOperations;
class PositiveDefiniteLinearSolver;
}
class ParameterData;
class Abstract_MeshScaleAgent;
class Abstract_SymmetryPlaneAgent;
class Abstract_MatrixAssemblyAgent;
class Abstract_MatrixNormalizationAgent;
class Abstract_PointGhostingAgent;
class PointCloud;
class Abstract_BoundedSupportFunction;
class AbstractAuthority;

class KernelFilter : public Filter
{
public:
    KernelFilter(AbstractAuthority* authority,
                 ParameterData* data,
                 AbstractInterface::PointCloud* points,
                 AbstractInterface::ParallelExchanger* exchanger);
    virtual ~KernelFilter();

    // set required functionalities
    void set_authority(AbstractAuthority* authority);
    void set_input_data(ParameterData* data);
    void set_points(AbstractInterface::PointCloud* points);
    void set_parallel_exchanger(AbstractInterface::ParallelExchanger* exchanger);

    void announce_radius();
    void enable_maintain_kernel_points();

    // Filter operations
    virtual void build();
    virtual void apply(AbstractInterface::ParallelVector* field);
    virtual void apply(AbstractInterface::ParallelVector* base_field, AbstractInterface::ParallelVector* gradient);
    bool is_valid(AbstractInterface::ParallelVector* field);

    // to be used as utilities, use cautiously
    PointCloud* internal_transfer_kernel_points();
    std::vector<double> internal_get_field_at_kernel_points(AbstractInterface::ParallelVector* parallel_field);
    void internal_set_field_at_kernel_points(AbstractInterface::ParallelVector* parallel_field,
                                             const std::vector<double>& field_at_kernel_points);
    std::vector<double> internal_parallel_matvec_apply(const std::vector<double>& input, const bool transpose);

private:

    void internal_apply(AbstractInterface::ParallelVector* field, bool transpose);
    void parallel_matvec_apply_transpose(std::vector<double>& field);
    void parallel_matvec_apply_noTranspose(std::vector<double>& field);

    bool m_built;
    bool m_announce_radius;

    // required functionalities
    void check_required_functionalities();
    AbstractAuthority* m_authority;
    ParameterData* m_input_data;
    AbstractInterface::PointCloud* m_original_points;
    AbstractInterface::ParallelExchanger* m_parallel_exchanger;

    void check_input_data();

    // radial function of finite support
    void determine_function();
    Abstract_BoundedSupportFunction* m_bounded_support_function;

    // internal agents
    void build_agents();
    void build_mesh_scale_agent(mesh_scale_agent_t::mesh_scale_agent_t agent);
    void build_symmetry_plane_agent(symmetry_plane_agent_t::symmetry_plane_agent_t agent);
    void build_matrix_assembly_agent(matrix_assembly_agent_t::matrix_assembly_agent_t agent);
    void build_matrix_normalization_agent(matrix_normalization_agent_t::matrix_normalization_agent_t agent);
    void build_point_ghosting_agent(point_ghosting_agent_t::point_ghosting_agent_t agent);
    Abstract_MeshScaleAgent* m_mesh_scale_agent;
    Abstract_SymmetryPlaneAgent* m_symmetry_plane_agent;
    Abstract_MatrixAssemblyAgent* m_matrix_assembly_agent;
    Abstract_MatrixNormalizationAgent* m_matrix_normalization_agent;
    Abstract_PointGhostingAgent* m_point_ghosting_agent;

    // kernel matrix
    AbstractInterface::SparseMatrix* m_local_kernel_matrix;
    std::vector<AbstractInterface::SparseMatrix*> m_parallel_block_row_kernel_matrices;
    std::vector<AbstractInterface::SparseMatrix*> m_parallel_block_column_kernel_matrices;

    // kernel points for transfer
    bool m_maintain_kernel_points;
    PointCloud* m_kernel_points;
};

}
