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

