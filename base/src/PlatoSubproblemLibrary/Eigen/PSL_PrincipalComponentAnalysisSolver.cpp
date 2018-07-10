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

#include "PSL_PrincipalComponentAnalysisSolver.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_JacobiEigenpairsSolver.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cassert>
#include <cmath>
#include <math.h>
#include <utility>
#include <algorithm>

namespace PlatoSubproblemLibrary
{

PrincipalComponentAnalysisSolver::PrincipalComponentAnalysisSolver(AbstractAuthority* authority) :
        m_authority(authority),
        m_upper_variation_fraction(.95)
{
}
PrincipalComponentAnalysisSolver::~PrincipalComponentAnalysisSolver()
{
}

void PrincipalComponentAnalysisSolver::set_upper_variation_fraction(double fraction)
{
    m_upper_variation_fraction = fraction;
}

void PrincipalComponentAnalysisSolver::solve(AbstractInterface::DenseMatrix* input,
                                             AbstractInterface::DenseMatrix*& compressed_basis,
                                             std::vector<double>& input_column_means,
                                             std::vector<double>& basis_column_stds)
{
    // get sizes
    const int input_rows = input->get_num_rows();
    const int input_columns = input->get_num_columns();
    assert(0 < input_rows);
    assert(0 < input_columns);

    // get column means
    input_column_means.resize(input_columns);
    std::vector<double> working_column;
    for(int c = 0; c < input_columns; c++)
    {
        input->get_column(c, working_column);
        input_column_means[c] = sum(working_column) / double(input_rows);
    }

    // build covariance matrix
    AbstractInterface::DenseMatrix* covariance = m_authority->dense_builder->build_by_fill(input_columns, input_columns, 0.);
    std::vector<double> ones_column(input_rows, 1.);
    for(int r = 0; r < input_columns; r++)
    {
        // column r minus mean
        input->get_column(r, working_column);
        m_authority->dense_vector_operations->axpy(-1. * input_column_means[r], ones_column, working_column);

        std::vector<double> c_column;
        for(int c = r; c < input_columns; c++)
        {
            // column c minus mean
            input->get_column(c, c_column);
            m_authority->dense_vector_operations->axpy(-1. * input_column_means[c], ones_column, c_column);

            // get this covariance value
            const double this_value = m_authority->dense_vector_operations->dot(working_column, c_column) / double(input_rows - 1);

            // symmetric matrix
            covariance->set_value(r, c, this_value);
            covariance->set_value(c, r, this_value);
        }
    }

    // build eigen solver
    JacobiEigenpairsSolver eigen(m_authority);
    eigen.set_tolerance(1e-10);

    // construct eigenvalues and eigenvectors
    std::vector<double> eigenvalues;
    AbstractInterface::DenseMatrix* eigenvectors = NULL;
    eigen.solve(covariance, eigenvalues, eigenvectors);
    safe_free(covariance);

    // get descending eigenvalue ordering
    std::vector<std::pair<double, int> > descending_eigenvalue_column(input_columns);
    for(int c = 0; c < input_columns; c++)
    {
        descending_eigenvalue_column[c] = std::make_pair(eigenvalues[c], c);
    }
    sort_descending(descending_eigenvalue_column);

    // compute cumulative sum
    std::vector<double> cumulative_descend_eigenvalues(input_columns);
    for(int c = 0; c < input_columns; c++)
    {
        cumulative_descend_eigenvalues[c] = descending_eigenvalue_column[c].first;
    }
    cumulative_sum(cumulative_descend_eigenvalues);

    // Determine eigenvectors needed
    const double sum_threshold = m_upper_variation_fraction * sum(eigenvalues);
    int num_eigen_selected = input_columns;
    for(int c = 0; c < input_columns; c++)
    {
        if(sum_threshold <= cumulative_descend_eigenvalues[c])
        {
            num_eigen_selected = c + 1;
            break;
        }
    }

    // scale in standardized basis
    basis_column_stds.resize(num_eigen_selected);
    for(int e = 0; e < num_eigen_selected; e++)
    {
        basis_column_stds[e] = std::sqrt(descending_eigenvalue_column[e].first);
    }

    // compute basis
    safe_free(compressed_basis);
    compressed_basis = m_authority->dense_builder->build_by_fill(input_columns, num_eigen_selected, 0.);
    for(int r = 0; r < input_columns; r++)
    {
        for(int e = 0; e < num_eigen_selected; e++)
        {
            const int eigen_column = descending_eigenvalue_column[e].second;
            const double basis_value = eigenvectors->get_value(r, eigen_column) / basis_column_stds[e];
            compressed_basis->set_value(r, e, basis_value);
        }
    }

    // clean up
    safe_free(eigenvectors);
}

void PrincipalComponentAnalysisSolver::get_compressed_row(std::vector<double>& this_row,
                                                          AbstractInterface::DenseMatrix* compressed_basis,
                                                          const std::vector<double>& input_column_means,
                                                          const std::vector<double>& basis_column_stds)
{
    // subtract means
    std::vector<double> meanless_input = this_row;
    m_authority->dense_vector_operations->axpy(-1., input_column_means, meanless_input);

    // compress
    compressed_basis->matvec(meanless_input, this_row, true);
}
void PrincipalComponentAnalysisSolver::get_decompressed_row(std::vector<double>& this_row,
                                                            AbstractInterface::DenseMatrix* compressed_basis,
                                                            const std::vector<double>& input_column_means,
                                                            const std::vector<double>& basis_column_stds)
{
    get_compressed_row(this_row, compressed_basis, input_column_means, basis_column_stds);

    // scale compressed
    std::vector<double> scaled_compressed;
    m_authority->dense_vector_operations->multiply(this_row, basis_column_stds, scaled_compressed);
    std::vector<double> scaled_compressed_squared;
    m_authority->dense_vector_operations->multiply(scaled_compressed, basis_column_stds, scaled_compressed_squared);

    // decompress
    compressed_basis->matvec(scaled_compressed_squared, this_row, false);

    // add back in means
    m_authority->dense_vector_operations->axpy(1., input_column_means, this_row);
}

}
