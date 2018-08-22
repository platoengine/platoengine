#include "PSL_Preprocessor.hpp"

#include "PSL_DataFlow.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{


Preprocessor::Preprocessor(AbstractAuthority* authority) :
        DataFlow(authority),
        m_preprocessor_type(preprocessor_t::preprocessor_t::no_preprocessor)
{
}
Preprocessor::~Preprocessor()
{
}

void Preprocessor::set_preprocessor_type(const preprocessor_t::preprocessor_t& type)
{
    m_preprocessor_type = type;
}
preprocessor_t::preprocessor_t Preprocessor::get_preprocessor_type()
{
    return m_preprocessor_type;
}

void Preprocessor::unpack_data()
{
}
void Preprocessor::allocate_data()
{
}

void Preprocessor::begin_initialize(ParameterData* parameters, ClassificationArchive* archive, int& processed_length)
{
    // get enum sizes
    std::vector<int> input_enum_sizes;
    int output_enum_size = -1;
    archive->get_enum_size(input_enum_sizes, output_enum_size);

    // get training
    AbstractInterface::DenseMatrix* training_matrix = archive->get_all_rows_onehot_encoded();
    const int considered_columns = int(training_matrix->get_num_columns()) - output_enum_size;

    // initialize
    internal_initialize(parameters, archive, training_matrix, considered_columns, processed_length);

    // clean up
    safe_free(training_matrix);
}
void Preprocessor::preprocess(std::vector<double>& input_all)
{
}
void Preprocessor::internal_initialize(ParameterData* parameters,
                                       ClassificationArchive* archive,
                                       AbstractInterface::DenseMatrix* input,
                                       const int& considered_columns,
                                       int& processed_length)
{
    processed_length = considered_columns;
}

}

