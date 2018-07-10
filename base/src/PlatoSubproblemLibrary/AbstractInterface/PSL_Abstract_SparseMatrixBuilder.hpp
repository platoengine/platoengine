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
#pragma once

#include <cstddef>

namespace PlatoSubproblemLibrary
{

namespace AbstractInterface
{
class SparseMatrix;
class MpiWrapper;
class GlobalUtilities;

class SparseMatrixBuilder
{
public:
    SparseMatrixBuilder(MpiWrapper* mpi_wrapper);
    virtual ~SparseMatrixBuilder();

    virtual MpiWrapper* get_mpi_wrapper();
    virtual GlobalUtilities* get_utilities();

    virtual size_t get_number_of_passes_over_all_nonzero_entries() = 0;
    virtual void begin_build(size_t num_rows, size_t num_columns) = 0;

    virtual bool needs_value_this_pass() = 0;
    virtual void specify_nonzero(size_t row, size_t column) = 0;
    virtual void specify_nonzero(size_t row, size_t column, double value) = 0;
    virtual void advance_pass() = 0;

    virtual SparseMatrix* end_build() = 0;

    virtual void send_matrix(size_t rank, SparseMatrix* matrix) = 0;
    virtual SparseMatrix* receive_matrix(size_t rank) = 0;

    virtual SparseMatrix* transpose(SparseMatrix* input) = 0;
protected:
    MpiWrapper* m_mpi_wrapper;
    GlobalUtilities* m_utilities;

};

}

}
