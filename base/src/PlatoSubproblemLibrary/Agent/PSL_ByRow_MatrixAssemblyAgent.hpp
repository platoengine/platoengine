// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_MatrixAssemblyAgent.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class FixedRadiusNearestNeighborsSearcher;
class SparseMatrix;
}
class AbstractAuthority;
class ParameterData;
class PointCloud;
class AxisAlignedBoundingBox;
class Point;
class Abstract_BoundedSupportFunction;

class ByRow_MatrixAssemblyAgent : public Abstract_MatrixAssemblyAgent
{
public:
    ByRow_MatrixAssemblyAgent(AbstractAuthority* authority,
                              ParameterData* input_data);
    virtual ~ByRow_MatrixAssemblyAgent();

    virtual void build(Abstract_BoundedSupportFunction* bounded_support_function,
                       PointCloud* local_kernel_points,
                       std::vector<PointCloud*>& nonlocal_kernel_points,
                       const std::vector<size_t>& processor_neighbors_below,
                       const std::vector<size_t>& processor_neighbors_above,
                       AbstractInterface::SparseMatrix** local_kernel_matrix,
                       std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                       std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices);

protected:

    void build_local(Abstract_BoundedSupportFunction* bounded_support_function,
                     PointCloud* kernel_points,
                     AbstractInterface::SparseMatrix** local_kernel_matrix);
    void compute_nonlocal_matrix_for_above_processor(Abstract_BoundedSupportFunction* bounded_support_function,
                                                     const std::vector<size_t>& processor_neighbors_above,
                                                     std::vector<PointCloud*>& nonlocal_kernel_points,
                                                     PointCloud* local_kernel_points,
                                                     std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                                                     const std::vector<int>& num_points_per_processor);
    void send_block_row_to_above_processor(const std::vector<size_t>& processor_neighbors_above,
                                           std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices);
    void recv_block_row_from_below_processor(const std::vector<size_t>& processor_neighbors_below,
                                             std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices);

    ParameterData* m_input_data;
    AbstractInterface::FixedRadiusNearestNeighborsSearcher* m_searcher;
    double m_support_distance;

};

}
