#pragma once

/* Solves for Principal Component Analysis (PCA) on a posed set of data.
 *
 * Returns compressed basis and shifts for mean and standard deviation.
 * In the compressed space, only a fraction of the variation is retained.
 * Once compressed space is solved for, decompressed rows can be compressed,
 * and then, decompressed. If the variation fraction is not 1, this will be lossy.
 */

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
