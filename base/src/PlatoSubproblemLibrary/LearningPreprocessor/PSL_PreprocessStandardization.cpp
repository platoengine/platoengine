#include "PSL_PreprocessStandardization.hpp"

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

Standardization::Standardization(AbstractAuthority* authority) :
        Preprocessor(authority),
        m_means(NULL),
        m_stds(NULL)
{
    set_preprocessor_type(preprocessor_t::preprocessor_t::standardization_preprocessor);
}
Standardization::~Standardization()
{
    m_means = NULL;
    m_stds = NULL;
}

void Standardization::unpack_data()
{
    // unpack double vectors
    m_means = m_data_sequence->get_double_vector(m_data_sequence->dequeue_double_vector());
    m_stds = m_data_sequence->get_double_vector(m_data_sequence->dequeue_double_vector());
}

void Standardization::allocate_data()
{
    m_data_sequence->enqueue_double_vector(2);
}

void Standardization::preprocess(std::vector<double>& input_all)
{
    const int indexes_to_process = input_all.size();

    assert(m_means->size() == m_stds->size());
    assert(indexes_to_process <= int(m_means->size()));

    // process
    for(int i = 0; i < indexes_to_process; i++)
    {
        standardize(input_all[i], (*m_means)[i], (*m_stds)[i]);
    }
}

void Standardization::internal_initialize(ParameterData* parameters,
                                          ClassificationArchive* archive,
                                          AbstractInterface::DenseMatrix* input,
                                          const int& considered_columns,
                                          int& processed_length)
{
    processed_length = considered_columns;

    // allocate
    m_means->resize(considered_columns);
    m_stds->resize(considered_columns);

    // fill
    for(int c = 0; c < considered_columns; c++)
    {
        // get column
        std::vector<double> column;
        input->get_column(c, column);

        // get statistics
        get_mean_and_std(column, (*m_means)[c], (*m_stds)[c]);
    }
}

}
