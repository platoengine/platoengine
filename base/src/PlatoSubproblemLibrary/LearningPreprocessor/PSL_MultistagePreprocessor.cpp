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

#include "PSL_MultistagePreprocessor.hpp"

#include "PSL_Preprocessor.hpp"
#include "PSL_ClassificationArchive.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_ParameterData.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_DataSequence.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_PreprocessorFactory.hpp"

#include <vector>
#include <cstddef>
#include <cassert>

namespace PlatoSubproblemLibrary
{

MultistagePreprocessor::MultistagePreprocessor(AbstractAuthority* authority,
                                               const std::vector<preprocessor_t::preprocessor_t>& preprocess_stages,
                                               DataSequence* data_sequence) :
        Preprocessor(authority),
        m_staged_preprocessors()
{
    // transfer data sequence
    if(data_sequence != NULL)
    {
        set_data_sequence(data_sequence, false);
    }

    // for each stage
    const size_t num_stages = preprocess_stages.size();
    m_staged_preprocessors.resize(num_stages);
    for(size_t s = 0u; s < num_stages; s++)
    {
        // build each preprocessor
        m_staged_preprocessors[s] = build_preprocessor(preprocess_stages[s], m_authority, m_data_sequence);
    }
}
MultistagePreprocessor::~MultistagePreprocessor()
{
    safe_free(m_staged_preprocessors);
}

void MultistagePreprocessor::unpack_data()
{
    // unpack each stage
    const size_t num_stages = m_staged_preprocessors.size();
    for(size_t s = 0u; s < num_stages; s++)
    {
        m_staged_preprocessors[s]->unpack_data();
    }
}

void MultistagePreprocessor::allocate_data()
{
    // allocate each stage
    const size_t num_stages = m_staged_preprocessors.size();
    for(size_t s = 0u; s < num_stages; s++)
    {
        m_staged_preprocessors[s]->allocate_data();
    }
}

void MultistagePreprocessor::preprocess(std::vector<double>& input_all)
{
    // process each stage
    const size_t num_stages = m_staged_preprocessors.size();
    for(size_t s = 0u; s < num_stages; s++)
    {
        m_staged_preprocessors[s]->preprocess(input_all);
    }
}

void MultistagePreprocessor::internal_initialize(ParameterData* parameters,
                                                 ClassificationArchive* archive,
                                                 AbstractInterface::DenseMatrix* input,
                                                 const int& considered_columns,
                                                 int& processed_length)
{
    AbstractInterface::DenseMatrix* previous_stage_output = input;
    int previous_considered_columns = considered_columns;
    int current_processed_length = -1;

    // process each stage
    const size_t num_stages = m_staged_preprocessors.size();
    for(size_t s = 0u; s < num_stages; s++)
    {
        // initialize
        m_staged_preprocessors[s]->internal_initialize(parameters,
                                                       archive,
                                                       previous_stage_output,
                                                       previous_considered_columns,
                                                       current_processed_length);

        const bool last_stage = s + 1u == num_stages;
        if(last_stage)
        {
            // free previous if not input
            if(s != 0u)
            {
                safe_free(previous_stage_output);
            }
        }
        else
        {
            // if not last stage, process for next stage

            // process from previous stage output
            const int num_rows = previous_stage_output->get_num_rows();
            AbstractInterface::DenseMatrix* this_stage_output =
                    m_authority->dense_builder->build_by_fill(num_rows, current_processed_length, 0.);

            // for each row
            for(int r = 0; r < num_rows; r++)
            {
                // get considered row
                std::vector<double> this_row;
                previous_stage_output->get_row(r, this_row);
                this_row.resize(previous_considered_columns);

                // process
                m_staged_preprocessors[s]->preprocess(this_row);
                assert(int(this_row.size()) == current_processed_length);

                // transfer
                for(int c = 0; c < current_processed_length; c++)
                {
                    this_stage_output->set_value(r, c, this_row[c]);
                }
            }

            // free previous if not input
            if(s != 0u)
            {
                safe_free(previous_stage_output);
            }

            // advance
            previous_stage_output = this_stage_output;
            previous_considered_columns = current_processed_length;
        }
    }

    // finalize
    processed_length = current_processed_length;
}

}
