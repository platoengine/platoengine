#include "PSL_JacobiEigenpairsSolver.hpp"

#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Abstract_DenseMatrixBuilder.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_AbstractAuthority.hpp"

#include <cassert>
#include <vector>
#include <cstddef>
#include <cmath>
#include <math.h>
#include <algorithm>

namespace PlatoSubproblemLibrary
{

JacobiEigenpairsSolver::JacobiEigenpairsSolver(AbstractAuthority* authority) :
        m_authority(authority),
        m_tolerance(1e-6)
{
}
JacobiEigenpairsSolver::~JacobiEigenpairsSolver()
{
}

void JacobiEigenpairsSolver::set_tolerance(double tol)
{
    m_tolerance = tol;
}

void JacobiEigenpairsSolver::solve(AbstractInterface::DenseMatrix* input,
                                   std::vector<double>& eigenvalues,
                                   AbstractInterface::DenseMatrix*& eigenvectors)
{
    const int dimension = input->get_num_rows();
    assert(dimension == int(input->get_num_columns()));

    // identity fill
    safe_free(eigenvectors);
    std::vector<double> ones(dimension, 1.);
    eigenvectors = m_authority->dense_builder->build_diagonal(ones);

    int max_p = -1;
    int max_q = -1;
    double max_off_diagonal = -1.;
    get_absmax_upper(input, max_p, max_q, max_off_diagonal);

    // until max is below tolerance
    while(m_tolerance < fabs(max_off_diagonal))
    {
        // reference locations
        const double dpp = input->get_value(max_p, max_p);
        const double dqq = input->get_value(max_q, max_q);
        const double dpq = input->get_value(max_p, max_q);

        // compute rotation
        const double theta = (dqq - dpp) / (2. * dpq);
        const double theta_sign = (theta >= 0. ? 1. : -1.);
        const double tan_value = theta_sign / (fabs(theta) + std::sqrt(theta * theta + 1));
        const double cos_value = 1. / std::sqrt(tan_value * tan_value + 1.);
        const double sin_value = cos_value * tan_value;
        const double cos_squared = cos_value * cos_value;
        const double sin_squared = sin_value * sin_value;

        // perform rotation on D
        // modify pp,pq,qq
        input->set_value(max_p, max_p, dpp * cos_squared + dqq * sin_squared - 2. * cos_value * sin_value * dpq);
        input->set_value(max_p, max_q, 0.);
        input->set_value(max_q, max_q, dpp * sin_squared + dqq * cos_squared + 2. * cos_value * sin_value * dpq);

        // modify remainder of row/column for p and q
        for(int j = 0; j < max_p; j++)
        {
            // j p q

            // store original values
            const double djp = input->get_value(j, max_p);
            const double djq = input->get_value(j, max_q);

            // update
            input->set_value(j, max_p, cos_value * djp - sin_value * djq);
            input->set_value(j, max_q, cos_value * djq + sin_value * djp);
        }
        for(int j = max_p + 1; j < max_q; j++)
        {
            // p j q

            // store original values
            const double djp = input->get_value(max_p, j);
            const double djq = input->get_value(j, max_q);

            // update
            input->set_value(max_p, j, cos_value * djp - sin_value * djq);
            input->set_value(j, max_q, cos_value * djq + sin_value * djp);
        }
        for(int j = max_q + 1; j < dimension; j++)
        {
            // p q j

            // store original values
            const double djp = input->get_value(max_p, j);
            const double djq = input->get_value(max_q, j);

            // update
            input->set_value(max_p, j, cos_value * djp - sin_value * djq);
            input->set_value(max_q, j, cos_value * djq + sin_value * djp);
        }

        // perform rotation on V
        for(int r = 0; r < dimension; r++)
        {
            const double erp = eigenvectors->get_value(r, max_p);
            const double erq = eigenvectors->get_value(r, max_q);
            eigenvectors->set_value(r, max_p, cos_value * erp - sin_value * erq);
            eigenvectors->set_value(r, max_q, cos_value * erq + sin_value * erp);
        }

        // determine new max
        get_absmax_upper(input, max_p, max_q, max_off_diagonal);
    }

    post_process(input, eigenvalues, eigenvectors);
}

void JacobiEigenpairsSolver::post_process(AbstractInterface::DenseMatrix* input,
                                          std::vector<double>& eigenvalues,
                                          AbstractInterface::DenseMatrix*& eigenvectors)
{
    const int dimension = input->get_num_rows();

    // get from diagonal
    input->get_diagonal(eigenvalues);

    // get ordering
    std::vector<std::pair<double, int> > values_and_indexes(dimension);
    for(int k = 0; k < dimension; k++)
    {
        values_and_indexes[k] = std::make_pair(eigenvalues[k], k);
    }
    std::sort(values_and_indexes.begin(), values_and_indexes.end());
    std::vector<int> column_reindexing(dimension);
    for(int k = 0; k < dimension; k++)
    {
        eigenvalues[k] = values_and_indexes[k].first;
        column_reindexing[k] = values_and_indexes[k].second;
    }

    // permute eigenvectors
    eigenvectors->permute_columns(column_reindexing);

    // enforce first non-zero entry positive
    for(int c = 0; c < dimension; c++)
    {
        for(int r = 0; r < dimension; r++)
        {
            const double entry = eigenvectors->get_value(r, c);
            const bool is_zero = (fabs(entry) < m_tolerance / 100.);
            const bool is_negative = (entry < 0.);

            if(!is_zero && is_negative)
            {
                // if non-zero and negative, scale by -1
                eigenvectors->scale_column(c, -1.);
                break;
            }
            else if(!is_zero)
            {
                // if non-zero, scale by 1
                break;
            }
        }
    }
}

void JacobiEigenpairsSolver::get_absmax_upper(AbstractInterface::DenseMatrix* input,
                                              int& max_p,
                                              int& max_q,
                                              double& max_off_diagonal)
{
    double absmax_off_diagonal = 0.;
    max_off_diagonal = 0.;
    const int dimension = input->get_num_rows();
    for(int r = 0; r < dimension; r++)
    {
        for(int c = r + 1; c < dimension; c++)
        {
            const double this_value = input->get_value(r, c);
            const double this_abs = fabs(this_value);
            if(absmax_off_diagonal < this_abs)
            {
                absmax_off_diagonal = this_abs;
                max_off_diagonal = this_value;
                max_p = r;
                max_q = c;
            }
        }
    }
}

}
