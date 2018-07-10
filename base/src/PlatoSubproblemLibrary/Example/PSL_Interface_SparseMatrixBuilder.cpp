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

// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_Interface_SparseMatrixBuilder.hpp"

#include <cstddef>
#include <string>
#include <vector>
#include <cassert>

#include "PSL_Abstract_SparseMatrix.hpp"
#include "PSL_Implementation_CompressedRowSparseMatrix.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Abstract_SparseMatrixBuilder.hpp"

namespace PlatoSubproblemLibrary
{
namespace example
{

SparseMatrixBuilder::SparseMatrixBuilder(AbstractInterface::MpiWrapper* mpi_wrapper) :
        AbstractInterface::SparseMatrixBuilder(mpi_wrapper),
        m_pass(),
        m_num_rows(),
        m_num_columns(),
        m_temporary_for_integer_row_bounds(),
        m_integer_row_bounds(),
        m_columns(),
        m_data()
{
    assert(m_utilities);
    reset();
}

SparseMatrixBuilder::~SparseMatrixBuilder()
{
    m_mpi_wrapper = NULL;
}

size_t SparseMatrixBuilder::get_number_of_passes_over_all_nonzero_entries()
{
    return 2u;
}

void SparseMatrixBuilder::begin_build(size_t num_rows, size_t num_columns)
{
    if(m_num_rows != 0u || m_num_columns != 0u)
    {
        m_utilities->fatal_error("SparseMatrixBuilder: Invalid call to begin_build. Aborting.\n\n");
    }

    // allocate
    m_pass = 0u;
    m_num_rows = num_rows;
    m_num_columns = num_columns;
    m_temporary_for_integer_row_bounds.assign(m_num_rows + 1, 0);
    m_integer_row_bounds.assign(m_num_rows + 1, 0);
    m_columns.clear();
    m_data.clear();
}

bool SparseMatrixBuilder::needs_value_this_pass()
{
    switch(m_pass)
    {
        case 0u:
        {
            // do not need values on first pass
            return false;
            break;
        }
        case 1u:
        {
            // need values on second pass
            return true;
            break;
        }
        default:
        {
            break;
        }
    }

    m_utilities->fatal_error("SparseMatrixBuilder: Invalid call to needs_value_this_pass. Aborting.\n\n");
    return false;
}

void SparseMatrixBuilder::specify_nonzero(size_t row, size_t column)
{
    switch(m_pass)
    {
        case 0u:
        {
            // register room for nonzero
            m_temporary_for_integer_row_bounds[1+row]++;
            break;
        }
        case 1u:
        {
            m_utilities->fatal_error("SparseMatrixBuilder: Invalid call to specify_nonzero. Needed a value. Aborting.\n\n");
            break;
        }
        default:
        {
            m_utilities->fatal_error("SparseMatrixBuilder: Invalid call to specify_nonzero. Unexpected pass. Aborting.\n\n");
            break;
        }
    }
}

void SparseMatrixBuilder::specify_nonzero(size_t row, size_t column, double value)
{
    switch(m_pass)
    {
        case 0u:
        {
            // register room for nonzero
            m_temporary_for_integer_row_bounds[1 + row]++;
            break;
        }
        case 1u:
        {
            m_columns[m_temporary_for_integer_row_bounds[row]] = column;
            m_data[m_temporary_for_integer_row_bounds[row]] = value;
            m_temporary_for_integer_row_bounds[row]++;
            break;
        }
        default:
        {
            m_utilities->fatal_error("SparseMatrixBuilder: Invalid call to specify_nonzero. Unexpected pass. Aborting.\n\n");
            break;
        }
    }
}

void SparseMatrixBuilder::advance_pass()
{
    switch(m_pass)
    {
        case 0u:
        {
            // allocate for next pass
            cumulative_sum(m_temporary_for_integer_row_bounds, m_integer_row_bounds);
            m_temporary_for_integer_row_bounds.assign(m_integer_row_bounds.begin(), m_integer_row_bounds.end());
            const size_t num_nonzero = m_integer_row_bounds.back();
            m_columns.resize(num_nonzero);
            m_data.resize(num_nonzero);
            break;
        }
        case 1u:
        {
            break;
        }
        default:
        {
            m_utilities->fatal_error("SparseMatrixBuilder: Invalid call to advance_pass. Aborting.\n\n");
            break;
        }
    }

    m_pass++;
}

AbstractInterface::SparseMatrix* SparseMatrixBuilder::end_build()
{
    if(m_pass != 2u)
    {
        m_utilities->fatal_error("SparseMatrixBuilder: Invalid call to end_build. Unexpected pass. Aborting.\n\n");
    }
    for(size_t row = 0; row < m_num_rows; row++)
    {
        if(m_temporary_for_integer_row_bounds[row] != m_integer_row_bounds[row + 1])
        {
            m_utilities->fatal_error("SparseMatrixBuilder: Invalid call to end_build. Expected row bounds not met. Aborting.\n\n");
        }
    }

    // build
    CompressedRowSparseMatrix* result = new CompressedRowSparseMatrix(m_num_rows,
                                                                      m_num_columns,
                                                                      m_integer_row_bounds,
                                                                      m_columns,
                                                                      m_data);

    // reset memory for another usage
    this->reset();

    return result;
}

void SparseMatrixBuilder::send_matrix(size_t send_rank, AbstractInterface::SparseMatrix* matrix)
{
    if(!matrix)
    {
        sendCompressedRowSparseMatrix(m_mpi_wrapper, send_rank, NULL);
        return;
    }

    CompressedRowSparseMatrix* matrix_casted = dynamic_cast<CompressedRowSparseMatrix*>(matrix);
    if(!matrix_casted)
    {
        m_utilities->fatal_error("SparseMatrixBuilder: failed to dynamic cast pointer in send_matrix. Aborting.\n\n");
    }

    sendCompressedRowSparseMatrix(m_mpi_wrapper, send_rank, matrix_casted);
}

AbstractInterface::SparseMatrix* SparseMatrixBuilder::receive_matrix(size_t recv_rank)
{
    return receiveCompressedRowSparseMatrix(m_mpi_wrapper, recv_rank);
}

AbstractInterface::SparseMatrix* SparseMatrixBuilder::transpose(AbstractInterface::SparseMatrix* input)
{
    if(!input)
    {
        return NULL;
    }

    CompressedRowSparseMatrix* input_casted = dynamic_cast<CompressedRowSparseMatrix*>(input);
    if(!input_casted)
    {
        m_utilities->fatal_error("SparseMatrixBuilder: failed to dynamic cast pointer in transpose. Aborting.\n\n");
    }

    return transposeCompressedRowSparseMatrix(input_casted);
}

void SparseMatrixBuilder::reset()
{
    m_pass = 0u;
    m_num_rows = 0u;
    m_num_columns = 0u;
    m_temporary_for_integer_row_bounds.clear();
    m_integer_row_bounds.clear();
    m_columns.clear();
    m_data.clear();
}

}
}
