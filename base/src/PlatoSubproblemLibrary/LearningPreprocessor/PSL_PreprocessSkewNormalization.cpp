#include "PSL_PreprocessSkewNormalization.hpp"

#include "PSL_Preprocessor.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_FreeStandardization.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_DataSequence.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>
#include <cstddef>
#include <cassert>

namespace PlatoSubproblemLibrary
{

SkewNormalization::SkewNormalization(AbstractAuthority* authority) :
        Preprocessor(authority),
        m_alphas(NULL),
        m_medians(NULL),
        m_stds(NULL)
{
    set_preprocessor_type(preprocessor_t::preprocessor_t::skew_normalization_preprocessor);
}
SkewNormalization::~SkewNormalization()
{
    m_alphas = NULL;
    m_medians = NULL;
    m_stds = NULL;
}

void SkewNormalization::unpack_data()
{
    // unpack double vectors
    m_alphas = m_data_sequence->get_double_vector(m_data_sequence->dequeue_double_vector());
    m_medians = m_data_sequence->get_double_vector(m_data_sequence->dequeue_double_vector());
    m_stds = m_data_sequence->get_double_vector(m_data_sequence->dequeue_double_vector());
}

void SkewNormalization::allocate_data()
{
    m_data_sequence->enqueue_double_vector(3);
}

void SkewNormalization::preprocess(std::vector<double>& input_all)
{
    const int indexes_to_process = input_all.size();

    assert(m_alphas->size() == m_medians->size());
    assert(m_medians->size() == m_stds->size());
    assert(indexes_to_process <= int(m_alphas->size()));

    // process
    for(int i = 0; i < indexes_to_process; i++)
    {
        skew_normalize(input_all[i], (*m_alphas)[i], (*m_medians)[i], (*m_stds)[i]);
    }
}

void SkewNormalization::internal_initialize(ParameterData* parameters,
                                            ClassificationArchive* archive,
                                            AbstractInterface::DenseMatrix* input,
                                            const int& considered_columns,
                                            int& processed_length)
{
    processed_length = considered_columns;

    // allocate
    m_alphas->resize(considered_columns);
    m_medians->resize(considered_columns);
    m_stds->resize(considered_columns);

    // fill
    for(int c = 0; c < considered_columns; c++)
    {
        // get column
        std::vector<double> column;
        input->get_column(c, column);

        // get statistics
        get_skew_normalize(column, (*m_alphas)[c], (*m_medians)[c], (*m_stds)[c]);
    }
}

}
