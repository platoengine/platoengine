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

#include "PSL_PreprocessorPCA.hpp"

#include "PSL_Preprocessor.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_DataSequence.hpp"
#include "PSL_PrincipalComponentAnalysisSolver.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"

#include <vector>
#include <cstddef>
#include <cassert>

namespace PlatoSubproblemLibrary
{

PreprocessorPCA::PreprocessorPCA(AbstractAuthority* authority) :
        Preprocessor(authority),
        m_input_column_means(NULL),
        m_basis_column_stds(NULL),
        m_compressed_basis(NULL)
{
    set_preprocessor_type(preprocessor_t::preprocessor_t::PCA_preprocessor);
}
PreprocessorPCA::~PreprocessorPCA()
{
    m_input_column_means = NULL;
    m_basis_column_stds = NULL;
    m_compressed_basis = NULL;
}

void PreprocessorPCA::unpack_data()
{
    // unpack double vectors
    m_input_column_means = m_data_sequence->get_double_vector(m_data_sequence->dequeue_double_vector());
    m_basis_column_stds = m_data_sequence->get_double_vector(m_data_sequence->dequeue_double_vector());
    m_compressed_basis = m_data_sequence->get_matrix_ptr(m_data_sequence->dequeue_matrix());
}

void PreprocessorPCA::allocate_data()
{
    m_data_sequence->enqueue_double_vector(2);
    m_data_sequence->enqueue_matrix();
}

void PreprocessorPCA::preprocess(std::vector<double>& input_all)
{
    // build for processing
    PrincipalComponentAnalysisSolver processor(m_authority);

    // process
    processor.get_compressed_row(input_all, *m_compressed_basis, *m_input_column_means, *m_basis_column_stds);
}

void PreprocessorPCA::internal_initialize(ParameterData* parameters,
                                          ClassificationArchive* archive,
                                          AbstractInterface::DenseMatrix* input,
                                          const int& considered_columns,
                                          int& processed_length)
{
    const int num_rows = input->get_num_rows();

    // get input-only sub-matrix
    AbstractInterface::DenseMatrix* considered_input = m_authority->dense_builder->build_submatrix(0, num_rows,
                                                                                                   0, considered_columns,
                                                                                                   input);

    // build for PCA
    PrincipalComponentAnalysisSolver PCA_solver(m_authority);
    assert(parameters->didUserInput_PCA_upper_variation_fraction());
    assert(0 < parameters->get_PCA_upper_variation_fraction());
    assert(parameters->get_PCA_upper_variation_fraction() <= 1);
    PCA_solver.set_upper_variation_fraction(parameters->get_PCA_upper_variation_fraction());

    // solve PCA
    PCA_solver.solve(considered_input, *m_compressed_basis, *m_input_column_means, *m_basis_column_stds);

    // set processed length
    processed_length = m_basis_column_stds->size();
    assert(0 < processed_length);
    assert(processed_length <= considered_columns);

    // clean up
    safe_free(considered_input);
}

}
