// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
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

class Abstract_MatrixNormalizationAgent
{
public:
    Abstract_MatrixNormalizationAgent(matrix_normalization_agent_t::matrix_normalization_agent_t type,
                                      AbstractAuthority* authority,
                                      ParameterData* data);
    virtual ~Abstract_MatrixNormalizationAgent();

    virtual void normalize(PointCloud* kernel_points,
                           std::vector<PointCloud*>& nonlocal_kernel_points,
                           AbstractInterface::SparseMatrix* local_kernel_matrix,
                           std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                           std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices,
                           std::vector<size_t>& processor_neighbors_below,
                           std::vector<size_t>& processor_neighbors_above) = 0;

protected:
    matrix_normalization_agent_t::matrix_normalization_agent_t m_type;
    AbstractAuthority* m_authority;
    ParameterData* m_input_data;

};

}
