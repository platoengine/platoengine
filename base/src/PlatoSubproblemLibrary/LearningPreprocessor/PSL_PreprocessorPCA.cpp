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
