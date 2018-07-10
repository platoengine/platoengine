// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Default_MatrixNormalizationAgent.hpp"

#include "PSL_Abstract_MatrixNormalizationAgent.hpp"
#include "PSL_ParameterDataEnums.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_SparseMatrix.hpp"
#include "PSL_PointCloud.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_Abstract_SparseMatrixBuilder.hpp"
#include "PSL_Point.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Abstract_PositiveDefiniteLinearSolver.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>
#include <cassert>
#include <algorithm>

namespace PlatoSubproblemLibrary
{

Default_MatrixNormalizationAgent::Default_MatrixNormalizationAgent(AbstractAuthority* authority,
                                                                   ParameterData* data) :
        Abstract_MatrixNormalizationAgent(matrix_normalization_agent_t::default_agent, authority, data)
{
}

Default_MatrixNormalizationAgent::~Default_MatrixNormalizationAgent()
{
}

void Default_MatrixNormalizationAgent::normalize(PointCloud* kernel_points,
                                                 std::vector<PointCloud*>& nonlocal_kernel_points,
                                                 AbstractInterface::SparseMatrix* local_kernel_matrix,
                                                 std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                                                 std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices,
                                                 std::vector<size_t>& processor_neighbors_below,
                                                 std::vector<size_t>& processor_neighbors_above)
{
    normalize_block_row(kernel_points,
                        nonlocal_kernel_points,
                        local_kernel_matrix,
                        parallel_block_row_kernel_matrices,
                        parallel_block_column_kernel_matrices);

    send_below_neighbors_of_block_row(parallel_block_row_kernel_matrices, processor_neighbors_below);

    recv_above_neighbors_of_block_column(parallel_block_column_kernel_matrices, processor_neighbors_above);

    recv_below_neighbors_of_block_column(parallel_block_column_kernel_matrices, processor_neighbors_below);

    send_above_of_block_row(parallel_block_row_kernel_matrices, processor_neighbors_above);
}

void Default_MatrixNormalizationAgent::normalize_block_row(PointCloud* kernel_points,
                                                           std::vector<PointCloud*>& nonlocal_kernel_points,
                                                           AbstractInterface::SparseMatrix* local_kernel_matrix,
                                                           std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                                                           std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices)
{
    if(m_input_data->get_normalization() == normalization_t::classical_row_normalization)
    {
        normalizeBlockRow_bySolveClassicalRowNormalization(local_kernel_matrix,
                                                           parallel_block_row_kernel_matrices,
                                                           parallel_block_column_kernel_matrices);
    }
    else if(m_input_data->get_normalization() == normalization_t::minimal_change_to_reproducing_conditions
            || m_input_data->get_normalization() == normalization_t::correction_function_reproducing_conditions)
    {
        normalizeBlockRow_bySetupReproducingConditions(kernel_points,
                                                       nonlocal_kernel_points,
                                                       local_kernel_matrix,
                                                       parallel_block_row_kernel_matrices,
                                                       parallel_block_column_kernel_matrices);
    }
    else
    {
        m_authority->utilities->fatal_error("MatrixNormalizationAgent: could not identify normalization type. Aborting.\n\n");
    }
}

void Default_MatrixNormalizationAgent::normalizeBlockRow_bySolveClassicalRowNormalization(AbstractInterface::SparseMatrix* local_kernel_matrix,
                                                                                          std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                                                                                          std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices)
{
    // get local contribution
    const size_t num_rows = local_kernel_matrix->getNumColumns();
    std::vector<double> ones_local(num_rows, 1.);
    std::vector<double> row_sums;
    local_kernel_matrix->matVec(ones_local, row_sums, false);

    // get parallel contribution
    const size_t num_blocks = parallel_block_row_kernel_matrices.size();
    for(size_t block_index = 0u; block_index < num_blocks; block_index++)
    {
        AbstractInterface::SparseMatrix* block_matrix = parallel_block_row_kernel_matrices[block_index];
        if(!block_matrix)
        {
            continue;
        }

        const size_t num_columns = block_matrix->getNumColumns();
        std::vector<double> ones(num_columns, 1.);
        std::vector<double> row_sum_contribution;
        block_matrix->matVec(ones, row_sum_contribution, false);

        m_authority->dense_vector_operations->axpy(1., row_sum_contribution, row_sums);
    }

    // compute 1/sums
    for(size_t row = 0; row < num_rows; row++)
    {
        row_sums[row] = 1. / row_sums[row];
    }

    // local normalization
    local_kernel_matrix->rowNormalize(row_sums);

    // parallel normalization
    for(size_t block_index = 0u; block_index < num_blocks; block_index++)
    {
        AbstractInterface::SparseMatrix* block_matrix = parallel_block_row_kernel_matrices[block_index];
        if(!block_matrix)
        {
            continue;
        }

        block_matrix->rowNormalize(row_sums);
    }
}

void Default_MatrixNormalizationAgent::normalizeBlockRow_bySetupReproducingConditions(PointCloud* kernel_points,
                                                                                      std::vector<PointCloud*>& nonlocal_kernel_points,
                                                                                      AbstractInterface::SparseMatrix* local_kernel_matrix,
                                                                                      std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                                                                                      std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices)
{
    const size_t mpi_size = m_authority->mpi_wrapper->get_size();

    // build index mapping of nonLocal columns of the blow row smoothing matrix to indexes in the nonlocal_boxes
    std::vector<std::map<int, int> > nonLocalColumnToIndex(mpi_size);
    for(size_t rank_ = 0; rank_ < mpi_size; rank_++)
    {
        if(!parallel_block_row_kernel_matrices[rank_])
        {
            continue;
        }

        const size_t num_nonLocal_boxes = nonlocal_kernel_points[rank_]->get_num_points();
        for(size_t index = 0; index < num_nonLocal_boxes; index++)
        {
            nonLocalColumnToIndex[rank_][nonlocal_kernel_points[rank_]->get_point(index)->get_index()] = index;
        }
    }

    const size_t num_rows = local_kernel_matrix->getNumRows();
    for(size_t row = 0; row < num_rows; row++)
    {
        // get this row's center
        Point* this_row = kernel_points->get_point(row);
        const double row_x = (*this_row)(0);
        const double row_y = (*this_row)(1);
        const double row_z = (*this_row)(2);

        // get data
        std::vector<double> initial_weights;
        std::vector<double> Constraints_x;
        std::vector<double> Constraints_y;
        std::vector<double> Constraints_z;
        std::vector<double> tmp_initial_weights;
        std::vector<size_t> tmp_row_columns;

        // get local
        local_kernel_matrix->getRow(row, tmp_initial_weights, tmp_row_columns);
        initial_weights.insert(initial_weights.end(), tmp_initial_weights.begin(), tmp_initial_weights.end());
        const size_t num_local_tmp_row_columns = tmp_row_columns.size();
        for(size_t nz = 0u; nz < num_local_tmp_row_columns; nz++)
        {
            Point* this_column = kernel_points->get_point(tmp_row_columns[nz]);
            Constraints_x.push_back((*this_column)(0u) - row_x);
            Constraints_y.push_back((*this_column)(1u) - row_y);
            Constraints_z.push_back((*this_column)(2u) - row_z);
        }

        // add nonlocal contributions
        for(size_t rank_ = 0; rank_ < mpi_size; rank_++)
        {
            AbstractInterface::SparseMatrix* sparse_matrix = parallel_block_row_kernel_matrices[rank_];
            if(!sparse_matrix)
            {
                continue;
            }

            sparse_matrix->getRow(row, tmp_initial_weights, tmp_row_columns);
            initial_weights.insert(initial_weights.end(), tmp_initial_weights.begin(), tmp_initial_weights.end());
            const size_t num_nonlocal_tmp_row_columns = tmp_row_columns.size();
            for(size_t nz = 0u; nz < num_nonlocal_tmp_row_columns; nz++)
            {
                Point* this_column = nonlocal_kernel_points[rank_]->get_point(nonLocalColumnToIndex[rank_][tmp_row_columns[nz]]);
                Constraints_x.push_back((*this_column)(0u) - row_x);
                Constraints_y.push_back((*this_column)(1u) - row_y);
                Constraints_z.push_back((*this_column)(2u) - row_z);
            }
        }

        std::vector<double> final_weights(initial_weights.size(), 0.);

        normalizeBlockRow_byAssembleReproducingConditionsMatrix(Constraints_x,
                                                                Constraints_y,
                                                                Constraints_z,
                                                                initial_weights,
                                                                final_weights);

        // map local weights back
        size_t index_counter = 0u;
        local_kernel_matrix->getRow(row, tmp_initial_weights, tmp_row_columns);
        size_t tmp_num_weights = tmp_initial_weights.size();
        std::vector<double> tmp_local_final_weights(&final_weights[index_counter], &final_weights[index_counter + tmp_num_weights]);
        local_kernel_matrix->setRow(row, tmp_local_final_weights);
        index_counter += tmp_num_weights;

        // map parallel weights back
        for(size_t rank_ = 0; rank_ < mpi_size; rank_++)
        {
            AbstractInterface::SparseMatrix* sparse_matrix = parallel_block_row_kernel_matrices[rank_];
            if(!sparse_matrix)
            {
                continue;
            }

            sparse_matrix->getRow(row, tmp_initial_weights, tmp_row_columns);
            tmp_num_weights = tmp_initial_weights.size();
            std::vector<double> tmp_nonlocal_final_weights(&final_weights[index_counter], &final_weights[index_counter + tmp_num_weights]);
            sparse_matrix->setRow(row, tmp_nonlocal_final_weights);
            index_counter += tmp_num_weights;
        }
    }
}

void Default_MatrixNormalizationAgent::normalizeBlockRow_byAssembleReproducingConditionsMatrix(std::vector<double>& Constraints_x,
                                                                                               std::vector<double>& Constraints_y,
                                                                                               std::vector<double>& Constraints_z,
                                                                                               std::vector<double>& initial_weights,
                                                                                               std::vector<double>& final_weights)
{
    const size_t num_weights = initial_weights.size();
    const reproduction_level_t::reproduction_level_t input_reproduction_level = m_input_data->get_reproduction();

    if(input_reproduction_level == reproduction_level_t::reproduce_constant || num_weights < 4)
    {
        normalizeBlockRow_bySolveConstantReproducingConditions(initial_weights, final_weights);
    }
    else if(input_reproduction_level == reproduction_level_t::reproduce_linear || num_weights < 10)
    {
        normalizeBlockRow_byAssembleReproducingConditionsMatrix_forLinearReproduction(Constraints_x,
                                                                                      Constraints_y,
                                                                                      Constraints_z,
                                                                                      initial_weights,
                                                                                      final_weights);
    }
    else if(input_reproduction_level == reproduction_level_t::reproduce_quadratic)
    {
        normalizeBlockRow_byAssembleReproducingConditionsMatrix_forQuadraticReproduction(Constraints_x,
                                                                                         Constraints_y,
                                                                                         Constraints_z,
                                                                                         initial_weights,
                                                                                         final_weights);
    }
    else
    {
        m_authority->utilities->fatal_error("Default_MatrixNormalizationAgent: Unable to match reproduction level. Aborting.\n\n");
    }
}

void Default_MatrixNormalizationAgent::normalizeBlockRow_byAssembleReproducingConditionsMatrix_forLinearReproduction(std::vector<double>& Constraints_x,
                                                                                                                     std::vector<double>& Constraints_y,
                                                                                                                     std::vector<double>& Constraints_z,
                                                                                                                     std::vector<double>& initial_weights,
                                                                                                                     std::vector<double>& final_weights)
{
    const size_t num_weights = initial_weights.size();

    // allocate constraint matrix C
    const size_t num_constraints = 4u;
    std::vector<double> constraints_values(num_weights * num_constraints, 1);
    std::copy(Constraints_x.begin(), Constraints_x.end(), &constraints_values[num_weights]);
    std::copy(Constraints_y.begin(), Constraints_y.end(), &constraints_values[2 * num_weights]);
    std::copy(Constraints_z.begin(), Constraints_z.end(), &constraints_values[3 * num_weights]);
    AbstractInterface::DenseMatrix* constraints = m_authority->dense_builder->build_by_row_major(num_constraints,
                                                                                      num_weights,
                                                                                      constraints_values);

    // solve
    normalizeBlockRow_bySolveForReproducingConditions(constraints, initial_weights, final_weights);

    // clean up
    safe_free(constraints);
}

void Default_MatrixNormalizationAgent::normalizeBlockRow_byAssembleReproducingConditionsMatrix_forQuadraticReproduction(std::vector<double>& Constraints_x,
                                                                                                                        std::vector<double>& Constraints_y,
                                                                                                                        std::vector<double>& Constraints_z,
                                                                                                                        std::vector<double>& initial_weights,
                                                                                                                        std::vector<double>& final_weights)
{
    const size_t num_weights = initial_weights.size();

    // allocate constraint matrix C
    const size_t num_constraints = 10u;
    std::vector<double> constraints_values(num_weights * num_constraints, 1);
    std::copy(Constraints_x.begin(), Constraints_x.end(), &constraints_values[num_weights]);
    std::copy(Constraints_y.begin(), Constraints_y.end(), &constraints_values[2 * num_weights]);
    std::copy(Constraints_z.begin(), Constraints_z.end(), &constraints_values[3 * num_weights]);
    for(size_t point_index = 0; point_index < num_weights; point_index++)
    {
        constraints_values[4 * num_weights + point_index] = Constraints_x[point_index] * Constraints_x[point_index];
        constraints_values[5 * num_weights + point_index] = Constraints_x[point_index] * Constraints_y[point_index];
        constraints_values[6 * num_weights + point_index] = Constraints_x[point_index] * Constraints_z[point_index];
        constraints_values[7 * num_weights + point_index] = Constraints_y[point_index] * Constraints_y[point_index];
        constraints_values[8 * num_weights + point_index] = Constraints_y[point_index] * Constraints_z[point_index];
        constraints_values[9 * num_weights + point_index] = Constraints_z[point_index] * Constraints_z[point_index];
    }
    AbstractInterface::DenseMatrix* constraints = m_authority->dense_builder->build_by_row_major(num_constraints,
                                                                                      num_weights,
                                                                                      constraints_values);

    // solve
    normalizeBlockRow_bySolveForReproducingConditions(constraints, initial_weights, final_weights);

    // clean up
    safe_free(constraints);
}

void Default_MatrixNormalizationAgent::normalizeBlockRow_bySolveForReproducingConditions(AbstractInterface::DenseMatrix* constraints,
                                                                                         std::vector<double>& initial_weights,
                                                                                         std::vector<double>& final_weights)
{
    const normalization_t::normalization_t normalization_strategy = m_input_data->get_normalization();

    if(normalization_strategy == normalization_t::classical_row_normalization)
    {
        normalizeBlockRow_bySolveConstantReproducingConditions(initial_weights, final_weights);
    }
    else if(normalization_strategy == normalization_t::correction_function_reproducing_conditions)
    {
        normalizeBlockRow_bySolveForCorrectionFunction(constraints, initial_weights, final_weights);
    }
    else if(normalization_strategy == normalization_t::minimal_change_to_reproducing_conditions)
    {
        normalizeBlockRow_bySolveForMinimalChange(constraints, initial_weights, final_weights);
    }
    else
    {
        m_authority->utilities->fatal_error("Default_MatrixNormalizationAgent: Unable to match normalization strategy. Aborting.\n\n");
    }
}

void Default_MatrixNormalizationAgent::normalizeBlockRow_bySolveConstantReproducingConditions(std::vector<double>& initial_weights,
                                                                                              std::vector<double>& final_weights)
{
    const size_t num_points = initial_weights.size();

    // normalize by row sum
    std::vector<double> ones(num_points, 1.);
    double normalization_factor = 1. / m_authority->dense_vector_operations->dot(ones, initial_weights);
    m_authority->dense_vector_operations->axpy(normalization_factor, initial_weights, final_weights);
}

void Default_MatrixNormalizationAgent::normalizeBlockRow_bySolveForCorrectionFunction(AbstractInterface::DenseMatrix* constraints,
                                                                                      std::vector<double>& initial_weights,
                                                                                      std::vector<double>& final_weights)
{
    const size_t num_constraints = constraints->get_num_rows();
    const size_t num_points = initial_weights.size();

    // fill weight scaled constraints C*diag(wi)
    AbstractInterface::DenseMatrix* diagonal_weights = m_authority->dense_builder->build_diagonal(initial_weights);
    AbstractInterface::DenseMatrix* C_w = m_authority->dense_builder->build_by_fill(num_constraints, num_points, 0.);
    C_w->matrix_matrix_product(1., constraints, false, diagonal_weights, false);
    safe_free(diagonal_weights);

    // fill correction function constants matrix
    AbstractInterface::DenseMatrix* C_w_Ct = m_authority->dense_builder->build_by_fill(num_constraints, num_constraints, 0.);
    C_w_Ct->matrix_matrix_product(1., constraints, false, C_w, true);

    // solve for correction function constants
    std::vector<double> target(num_constraints, 0.);
    target[0] = 1.;
    std::vector<double> correction_function_constants;
    bool success = m_authority->dense_solver->solve(C_w_Ct, target, correction_function_constants);
    safe_free(C_w_Ct);

    if(!success)
    {
        safe_free(C_w);

        const size_t reduced_num_constraints =
                normalizeBlockRow_bySolveForReproducingConditions_decreaseNumConstraints(num_constraints);

        if(reduced_num_constraints == 1u)
        {
            normalizeBlockRow_bySolveConstantReproducingConditions(initial_weights, final_weights);
        }
        else
        {
            AbstractInterface::DenseMatrix* reduced_constraints = m_authority->dense_builder->build_submatrix(0u, reduced_num_constraints,
                                                                                                   0u, num_points,
                                                                                                   constraints);
            normalizeBlockRow_bySolveForCorrectionFunction(reduced_constraints, initial_weights, final_weights);
            safe_free(reduced_constraints);
        }
    }

    // solve for final weights
    C_w->matvec(correction_function_constants, final_weights, true);
    safe_free(C_w);
}

void Default_MatrixNormalizationAgent::normalizeBlockRow_bySolveForMinimalChange(AbstractInterface::DenseMatrix* constraints,
                                                                                 std::vector<double>& initial_weights,
                                                                                 std::vector<double>& final_weights)
{
    const size_t num_constraints = constraints->get_num_rows();
    const size_t num_points = initial_weights.size();

    // compute normalized initial weights
    std::vector<double> normalized_initial_weights(num_points, 0.);
    normalizeBlockRow_bySolveConstantReproducingConditions(initial_weights, normalized_initial_weights);

    // compute target correction for constraints
    std::vector<double> target_correction(num_constraints, 0.);
    constraints->matvec(normalized_initial_weights, target_correction, false);

    // compute shifted target
    std::vector<double> shifted_target(num_constraints, 0.);
    shifted_target[0] = 1.;
    m_authority->dense_vector_operations->axpy(-1., target_correction, shifted_target);

    // fill correction functions matrix
    AbstractInterface::DenseMatrix* correction_function_constants_matrix = m_authority->dense_builder->build_by_fill(num_constraints,
                                                                                                          num_constraints,
                                                                                                          0.);
    correction_function_constants_matrix->matrix_matrix_product(1., constraints, false, constraints, true);

    // solve for minimal change constants
    std::vector<double> minimal_change_constants;
    bool success = m_authority->dense_solver->solve(correction_function_constants_matrix, shifted_target, minimal_change_constants);
    safe_free(correction_function_constants_matrix);

    if(!success)
    {
        const size_t reduced_num_constraints =
                normalizeBlockRow_bySolveForReproducingConditions_decreaseNumConstraints(num_constraints);

        if(reduced_num_constraints == 1u)
        {
            normalizeBlockRow_bySolveConstantReproducingConditions(initial_weights, final_weights);
        }
        else
        {
            AbstractInterface::DenseMatrix* reduced_constraints = m_authority->dense_builder->build_submatrix(0u, reduced_num_constraints,
                                                                                                   0u, num_points,
                                                                                                   constraints);
            normalizeBlockRow_bySolveForMinimalChange(constraints, initial_weights, final_weights);
            safe_free(reduced_constraints);
        }
    }

    // compute change in weights
    constraints->matvec(minimal_change_constants, final_weights, true);

    // compute final weights
    m_authority->dense_vector_operations->axpy(1., normalized_initial_weights, final_weights);
}

size_t Default_MatrixNormalizationAgent::normalizeBlockRow_bySolveForReproducingConditions_decreaseNumConstraints(size_t num_constraints)
{
    if(num_constraints == 1u)
    {
        m_authority->utilities->fatal_error("Default_MatrixNormalizationAgent: cannot reduce below one constraint. Aborting.\n\n");
    }
    else if(num_constraints == 4u)
    {
        return 1u;
    }
    else if(num_constraints == 10u)
    {
        return 4u;
    }

    m_authority->utilities->fatal_error("Default_MatrixNormalizationAgent: requested reduce on irregular number of constraints. Aborting.\n\n");
    return 0u;
}

void Default_MatrixNormalizationAgent::send_below_neighbors_of_block_row(std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                                                                         std::vector<size_t>& processor_neighbors_below)
{
    // send row block matrix to lower procs that neighbor
    const size_t num_neighbors_below = processor_neighbors_below.size();
    for(size_t below_index = 0; below_index < num_neighbors_below; below_index++)
    {
        const size_t lower_proc_id = processor_neighbors_below[below_index];
        m_authority->sparse_builder->send_matrix(lower_proc_id, parallel_block_row_kernel_matrices[lower_proc_id]);
    }
}

void Default_MatrixNormalizationAgent::recv_above_neighbors_of_block_column(std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices,
                                                                            std::vector<size_t>& processor_neighbors_above)
{
    // recv column block matrix from upper procs that neighbor
    const size_t num_neighbors_above = processor_neighbors_above.size();
    for(size_t above_index = 0; above_index < num_neighbors_above; above_index++)
    {
        const size_t upper_proc_id = processor_neighbors_above[above_index];
        parallel_block_column_kernel_matrices[upper_proc_id] = m_authority->sparse_builder->receive_matrix(upper_proc_id);
    }
}

void Default_MatrixNormalizationAgent::recv_below_neighbors_of_block_column(std::vector<AbstractInterface::SparseMatrix*>& parallel_block_column_kernel_matrices,
                                                                            std::vector<size_t>& processor_neighbors_below)
{
    // recv column block matrix from lower procs that neighbor
    const size_t num_neighbors_below = processor_neighbors_below.size();
    for(size_t below_index = 0; below_index < num_neighbors_below; below_index++)
    {
        const size_t lower_proc_id = processor_neighbors_below[below_index];
        parallel_block_column_kernel_matrices[lower_proc_id] = m_authority->sparse_builder->receive_matrix(lower_proc_id);
    }
}

void Default_MatrixNormalizationAgent::send_above_of_block_row(std::vector<AbstractInterface::SparseMatrix*>& parallel_block_row_kernel_matrices,
                                                               std::vector<size_t>& processor_neighbors_above)
{
    // send row block matrix to upper procs that neighbor
    const size_t num_neighbors_above = processor_neighbors_above.size();
    for(size_t above_index = 0; above_index < num_neighbors_above; above_index++)
    {
        const size_t upper_proc_id = processor_neighbors_above[above_index];
        m_authority->sparse_builder->send_matrix(upper_proc_id, parallel_block_row_kernel_matrices[upper_proc_id]);
    }
}

}
