#pragma once

#include <vector>

namespace PlatoSubproblemLibrary
{
namespace AbstractInterface
{
class DenseMatrix;
}
class AbstractAuthority;

class PrincipalComponentAnalysisSolver
{
public:
    PrincipalComponentAnalysisSolver(AbstractAuthority* authority);
    ~PrincipalComponentAnalysisSolver();

    void set_upper_variation_fraction(double fraction);

    void solve(AbstractInterface::DenseMatrix* input,
               AbstractInterface::DenseMatrix*& compressed_basis,
               std::vector<double>& input_column_means,
               std::vector<double>& basis_column_stds);
    void get_compressed_row(std::vector<double>& this_row,
                            AbstractInterface::DenseMatrix* compressed_basis,
                            const std::vector<double>& input_column_means,
                            const std::vector<double>& basis_column_stds);
    void get_decompressed_row(std::vector<double>& this_row,
                              AbstractInterface::DenseMatrix* compressed_basis,
                              const std::vector<double>& input_column_means,
                              const std::vector<double>& basis_column_stds);

private:

    AbstractAuthority* m_authority;
    double m_upper_variation_fraction;
};

}
