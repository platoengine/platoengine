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
