// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

#include "PSL_Abstract_MatrixNormalizationAgent.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class SparseMatrix;
class DenseMatrix;
}
class AbstractAuthority;
class ParameterData;
class PointCloud;

class Default_MatrixNormalizationAgent : public Abstract_MatrixNormalizationAgent
{
public:
    Default_MatrixNormalizationAgent(AbstractAuthority* authority, ParameterData* data);
    virtual ~Default_MatrixNormalizationAgent();

    virtual void normalize(PointCloud* kernel_points,
                           std::vector<PointCloud*>& nonlocal_kernel_points,
                           AbstractInterface::SparseMatrix* local_kernel_matrix,
                           std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                           std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices,
                           std::vector<size_t>& processor_neighbors_below,
                           std::vector<size_t>& processor_neighbors_above);

protected:

    void normalize_block_row(PointCloud* kernel_points,
                             std::vector<PointCloud*>& nonlocal_kernel_points,
                             AbstractInterface::SparseMatrix* local_kernel_matrix,
                             std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                             std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices);
    void normalizeBlockRow_bySolveClassicalRowNormalization(AbstractInterface::SparseMatrix* local_kernel_matrix,
                                                            std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                                                            std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices);
    void normalizeBlockRow_bySetupReproducingConditions(PointCloud* kernel_points,
                                                        std::vector<PointCloud*>& nonlocal_kernel_points,
                                                        AbstractInterface::SparseMatrix* local_kernel_matrix,
                                                        std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                                                        std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices);

    void normalizeBlockRow_byAssembleReproducingConditionsMatrix(std::vector<double>& Constraints_x,
                                                               std::vector<double>& Constraints_y,
                                                               std::vector<double>& Constraints_z,
                                                               std::vector<double>& initial_weights,
                                                               std::vector<double>& final_weights);
    void normalizeBlockRow_byAssembleReproducingConditionsMatrix_forLinearReproduction(std::vector<double>& Constraints_x,
                                                                                     std::vector<double>& Constraints_y,
                                                                                     std::vector<double>& Constraints_z,
                                                                                     std::vector<double>& initial_weights,
                                                                                     std::vector<double>& final_weights);
    void normalizeBlockRow_byAssembleReproducingConditionsMatrix_forQuadraticReproduction(std::vector<double>& Constraints_x,
                                                                                        std::vector<double>& Constraints_y,
                                                                                        std::vector<double>& Constraints_z,
                                                                                        std::vector<double>& initial_weights,
                                                                                        std::vector<double>& final_weights);
    void normalizeBlockRow_bySolveForReproducingConditions(AbstractInterface::DenseMatrix* constraints,
                                                           std::vector<double>& initial_weights,
                                                           std::vector<double>& final_weights);
    void normalizeBlockRow_bySolveConstantReproducingConditions(std::vector<double>& initial_weights,
                                                                std::vector<double>& final_weights);
    void normalizeBlockRow_bySolveForCorrectionFunction(AbstractInterface::DenseMatrix* constraints,
                                                        std::vector<double>& initial_weights,
                                                        std::vector<double>& final_weights);
    void normalizeBlockRow_bySolveForMinimalChange(AbstractInterface::DenseMatrix* constraints,
                                                   std::vector<double>& initial_weights,
                                                   std::vector<double>& final_weights);
    size_t normalizeBlockRow_bySolveForReproducingConditions_decreaseNumConstraints(size_t num_constraints);

    void send_below_neighbors_of_block_row(std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                                           std::vector<size_t>& processor_neighbors_below);
    void recv_above_neighbors_of_block_column(std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices,
                                              std::vector<size_t>& processor_neighbors_above);
    void recv_below_neighbors_of_block_column(std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices,
                                              std::vector<size_t>& processor_neighbors_below);
    void send_above_of_block_row(std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                                 std::vector<size_t>& processor_neighbors_above);

};

}
