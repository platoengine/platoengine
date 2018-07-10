#include "PSL_PreprocessBipolarNormalization.hpp"

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

BipolarNormalization::BipolarNormalization(AbstractAuthority* authority) :
        Preprocessor(authority),
        m_mins(NULL),
        m_maxs(NULL)
{
    set_preprocessor_type(preprocessor_t::preprocessor_t::bipolar_normalization_preprocessor);
}
BipolarNormalization::~BipolarNormalization()
{
    m_mins = NULL;
    m_maxs = NULL;
}

void BipolarNormalization::unpack_data()
{
    // unpack double vectors
    m_mins = m_data_sequence->get_double_vector(m_data_sequence->dequeue_double_vector());
    m_maxs = m_data_sequence->get_double_vector(m_data_sequence->dequeue_double_vector());
}

void BipolarNormalization::allocate_data()
{
    m_data_sequence->enqueue_double_vector(2);
}

void BipolarNormalization::preprocess(std::vector<double>& input_all)
{
    const int indexes_to_process = input_all.size();

    assert(m_mins->size() == m_maxs->size());
    assert(indexes_to_process <= int(m_mins->size()));

    // process
    for(int i = 0; i < indexes_to_process; i++)
    {
        bipolar_normalize(input_all[i], (*m_mins)[i], (*m_maxs)[i]);
    }
}

void BipolarNormalization::internal_initialize(ParameterData* parameters,
                                     ClassificationArchive* archive,
                                     AbstractInterface::DenseMatrix* input,
                                     const int& considered_columns,
                                     int& processed_length)
{
    processed_length = considered_columns;

    // allocate
    m_mins->resize(considered_columns);
    m_maxs->resize(considered_columns);

    // fill
    for(int c = 0; c < considered_columns; c++)
    {
        // get column
        std::vector<double> column;
        input->get_column(c, column);

        // get statistics
        get_bipolar_normalize(column, (*m_mins)[c], (*m_maxs)[c]);
    }
}

}
