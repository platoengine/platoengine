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

#include "PSL_ClassificationAssessor.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Classifier.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_VectorCoding.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <vector>

namespace PlatoSubproblemLibrary
{

ClassificationAssessor::ClassificationAssessor(AbstractAuthority* authority) :
        m_authority(authority),
        m_parameters(NULL),
        m_output_enum_size(-1),
        m_confusion_matrix(NULL)
{
}

ClassificationAssessor::~ClassificationAssessor()
{
    m_authority = NULL;
    m_parameters = NULL;
    safe_free(m_confusion_matrix);
}

void ClassificationAssessor::initialize(ParameterData* parameters, int output_enum_size)
{
    m_parameters = parameters;
    m_output_enum_size = output_enum_size;
    m_confusion_matrix = m_authority->dense_builder->build_by_fill(output_enum_size, output_enum_size, 0.);
}

void ClassificationAssessor::clear()
{
    m_confusion_matrix->scale(0.);
}

void ClassificationAssessor::assess(Classifier* classifier, ClassificationArchive* archive, const std::vector<int>& rows)
{
    internal_assess(classifier, archive, false, rows);
}

void ClassificationAssessor::assess(Classifier* classifier, ClassificationArchive* archive)
{
    internal_assess(classifier, archive, true, std::vector<int>());
}

void ClassificationAssessor::add_to_assessment(const int& actual_output, const int& predicted_output)
{
    // introduce this assessment
    const double current_value = m_confusion_matrix->get_value(size_t(actual_output), size_t(predicted_output));
    m_confusion_matrix->set_value(size_t(actual_output), size_t(predicted_output), current_value + 1.);
}

AbstractInterface::DenseMatrix* ClassificationAssessor::get_confusion_matrix()
{
    return m_confusion_matrix;
}

double ClassificationAssessor::get_classification_accuracy()
{
    double accuracy_numerator = 0.;
    double accuracy_denominator = 0.;

    // print confusion matrix if verbose
    if(m_parameters->didUserInput_verbose())
    {
        if(m_parameters->get_verbose())
        {
            AbstractInterface::GlobalUtilities* utilities = m_authority->utilities;
            utilities->print("confusion matrix:\n");
            for(int row = 0; row < m_output_enum_size; row++)
            {
                std::vector<double> this_row;
                m_confusion_matrix->get_row(row, this_row);
                utilities->print(this_row, true);
            }
        }
    }

    // count diagonal and off-diagonal elements
    for(int row = 0; row < m_output_enum_size; row++)
    {
        for(int column = 0; column < m_output_enum_size; column++)
        {
            const double matrix_value = m_confusion_matrix->get_value(row, column);
            accuracy_denominator += matrix_value;
            if(row == column)
            {
                accuracy_numerator += matrix_value;
            }
        }
    }

    // avoid dividing by zero
    if(accuracy_denominator == 0.)
    {
        return 0.;
    }
    return accuracy_numerator / accuracy_denominator;
}

void ClassificationAssessor::internal_assess(Classifier* classifier,
                                             ClassificationArchive* archive,
                                             const bool& all_rows,
                                             const std::vector<int>& rows)
{
    // get sizes
    std::vector<int> input_enum_sizes;
    int output_enum_size = -1;
    archive->get_enum_size(input_enum_sizes, output_enum_size);

    // get data set
    AbstractInterface::DenseMatrix* dataset = archive->get_all_rows_onehot_encoded();

    // determine loop length
    const int num_rows = (all_rows ? dataset->get_num_rows() : rows.size());

    // for each abstract row
    for(int abstract_row = 0; abstract_row < num_rows; abstract_row++)
    {
        // get row
        const int acutal_row = (all_rows ? abstract_row : rows[abstract_row]);
        std::vector<double> this_row;
        dataset->get_row(acutal_row, this_row);

        // split encoding
        std::vector<double> this_row_input_scalar;
        std::vector<int> this_row_input_enum;
        int this_row_actual_output_enum = -1;
        onehot_decode(this_row,
                      input_enum_sizes,
                      m_output_enum_size,
                      this_row_input_scalar,
                      this_row_input_enum,
                      this_row_actual_output_enum);

        // classify
        const int this_row_computed_output_enum = classifier->classify(this_row_input_scalar, this_row_input_enum);

        // assess
        add_to_assessment(this_row_actual_output_enum, this_row_computed_output_enum);
    }

    delete dataset;
}

}

