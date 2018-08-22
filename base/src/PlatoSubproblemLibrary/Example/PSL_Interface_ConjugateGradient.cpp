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
