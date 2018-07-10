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

#include "PSL_AbstractAuthority.hpp"

#include "PSL_FreeHelpers.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_MpiWrapper.hpp"
#include "PSL_Abstract_SparseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_Abstract_PositiveDefiniteLinearSolver.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_MpiWrapper.hpp"
#include "PSL_Interface_SparseMatrixBuilder.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_Interface_BasicDenseVectorOperations.hpp"
#include "PSL_Interface_ConjugateGradient.hpp"

#include <mpi.h>

namespace PlatoSubproblemLibrary
{

AbstractAuthority::AbstractAuthority(MPI_Comm* comm) :
        utilities(NULL),
        mpi_wrapper(NULL),
        sparse_builder(NULL),
        dense_builder(NULL),
        dense_vector_operations(NULL),
        dense_solver(NULL),
        m_comm()
{
    if(comm == NULL)
    {
        m_comm = MPI_COMM_WORLD;
        comm = &m_comm;
    }
    allocate(comm);
}
AbstractAuthority::~AbstractAuthority()
{
    safe_free(utilities);
    safe_free(mpi_wrapper);
    safe_free(sparse_builder);
    safe_free(dense_builder);
    safe_free(dense_vector_operations);
    safe_free(dense_solver);
}
void AbstractAuthority::allocate(MPI_Comm* comm)
{
    utilities = new example::Interface_BasicGlobalUtilities;
    mpi_wrapper = new example::Interface_MpiWrapper(utilities, comm);
    sparse_builder = new example::SparseMatrixBuilder(mpi_wrapper);
    dense_builder = new example::Interface_DenseMatrixBuilder(utilities);
    dense_vector_operations = new example::Interface_BasicDenseVectorOperations;
    dense_solver = new example::Interface_CojugateGradient(utilities, dense_vector_operations);
}

}
