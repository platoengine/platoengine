// PlatoSubproblemLibraryVersion(7): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_ParameterDataEnums.hpp"
#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class SparseMatrix;
}
class AbstractAuthority;
class ParameterData;
class PointCloud;
class Abstract_BoundedSupportFunction;

class Abstract_MatrixAssemblyAgent
{
public:
    Abstract_MatrixAssemblyAgent(matrix_assembly_agent_t::matrix_assembly_agent_t type,
                                 AbstractAuthority* authority);
    virtual ~Abstract_MatrixAssemblyAgent();

    virtual void build(Abstract_BoundedSupportFunction* bounded_support_function,
                       PointCloud* local_kernel_points,
                       std::vector<PointCloud*>& nonlocal_kernel_points,
                       const std::vector<size_t>& processor_neighbors_below,
                       const std::vector<size_t>& processor_neighbors_above,
                       AbstractInterface::SparseMatrix** local_kernel_matrix,
                       std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                       std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices) = 0;

protected:
    matrix_assembly_agent_t::matrix_assembly_agent_t m_type;
    AbstractAuthority* m_authority;

};

}
