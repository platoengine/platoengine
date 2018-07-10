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
#include "PSL_Interface_DenseMatrixBuilder.hpp"

#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Implementation_DenseMatrix.hpp"
#include "PSL_Interface_DenseMatrix.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_DenseMatrixBuilder::Interface_DenseMatrixBuilder(AbstractInterface::GlobalUtilities* utilities) :
        AbstractInterface::DenseMatrixBuilder(utilities)
{
}

Interface_DenseMatrixBuilder::~Interface_DenseMatrixBuilder()
{
}

AbstractInterface::DenseMatrix* Interface_DenseMatrixBuilder::build_by_row_major(size_t num_rows,
                                                                                 size_t num_columns,
                                                                                 const std::vector<double>& in)
{
    // build
    double** data = dense_matrix::create(num_rows, num_columns);
    dense_matrix::fill_by_row_major(in.data(), data, num_rows, num_columns);

    // transfer
    Interface_DenseMatrix* result = new Interface_DenseMatrix(m_utilities, this);
    result->receive(data, num_rows, num_columns);
    return result;
}

}
}
