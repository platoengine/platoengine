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

// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#pragma once

/* Able to perform a variety of normalization strategies on an assembled linear system.
 * Typical form is classical row normalization. Other strategies exist, but have not been found to be useful.
 */

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
