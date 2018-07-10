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
#include "PSL_Interface_ConjugateGradient.hpp"

#include "PSL_Abstract_PositiveDefiniteLinearSolver.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"

namespace PlatoSubproblemLibrary
{
namespace example
{

Interface_CojugateGradient::Interface_CojugateGradient(AbstractInterface::GlobalUtilities* utilities,
                                                       AbstractInterface::DenseVectorOperations* operations) :
        AbstractInterface::PositiveDefiniteLinearSolver(),
        m_utilities(utilities),
        m_operations(operations),
        m_tolerance(1e-8),
        m_verbosity(false)
{
}

Interface_CojugateGradient::~Interface_CojugateGradient()
{
}

void Interface_CojugateGradient::setTolerance(double tolerance_)
{
    m_tolerance = tolerance_;
}

void Interface_CojugateGradient::setVerbosity(bool verbose_)
{
    m_verbosity = verbose_;
}

// true if success
bool Interface_CojugateGradient::solve(AbstractInterface::DenseMatrix* matrix, const std::vector<double>& rhs, std::vector<double>& sol)
{
    // initialize and allocate
    const size_t n = rhs.size();
    sol.assign(n, 0.);

    std::vector<double> r(rhs);
    std::vector<double> p(r);
    std::vector<double> Ap(n);

    double r_dot = m_operations->dot(r, r);
    double r_dot_next = -1.;

    // iteratively solve
    const size_t max_repetition = 10 * n;
    for(size_t repetition = 0u; repetition < max_repetition; repetition++)
    {
        matrix->matvec(p, Ap, false);

        const double alpha = r_dot / m_operations->dot(p, Ap);

        m_operations->axpy(alpha, p, sol);
        m_operations->axpy(-alpha, Ap, r);

        r_dot_next = m_operations->dot(r, r);
        if(r_dot_next < m_tolerance)
        {
            return true;
        }

        m_operations->scale(r_dot_next / r_dot, p);
        m_operations->axpy(1., r, p);

        r_dot = r_dot_next;
    }

    if(m_verbosity)
    {
        m_utilities->print("PlatoSubproblemLibrary: warning conjugate gradient solver did not converge.\n");
    }
    return false;
}

}
}
