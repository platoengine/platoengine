#pragma once

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class DenseMatrix;
}
class AbstractAuthority;

class JacobiEigenpairsSolver
{
public:
    JacobiEigenpairsSolver(AbstractAuthority* authority);
    ~JacobiEigenpairsSolver();

    void set_tolerance(double tol);

    // warning: mutates input
    void solve(AbstractInterface::DenseMatrix* input,
               std::vector<double>& eigenvalues,
               AbstractInterface::DenseMatrix*& eigenvectors);
private:
    void post_process(AbstractInterface::DenseMatrix* input,
                      std::vector<double>& eigenvalues,
                      AbstractInterface::DenseMatrix*& eigenvectors);
    void get_absmax_upper(AbstractInterface::DenseMatrix* input, int& max_p, int& max_q, double& max_off_diagonal);

    AbstractAuthority* m_authority;
    double m_tolerance;
};

}
