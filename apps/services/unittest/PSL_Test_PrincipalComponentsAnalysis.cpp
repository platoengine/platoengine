#include "PSL_UnitTestingHelper.hpp"

#include "PSL_FreeHelpers.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_Interface_BasicDenseVectorOperations.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_PrincipalComponentAnalysisSolver.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Random.hpp"

#include <vector>
#include <cstddef>
#include <cmath>
#include <math.h>
#include <cassert>

namespace PlatoSubproblemLibrary
{
namespace TestingPrincipalComponentsAnalysis
{

AbstractInterface::DenseMatrix* build_mostlyIndependentInput(AbstractInterface::DenseMatrixBuilder* builder, const int& num_rows)
{
    const int num_columns = 5;
    AbstractInterface::DenseMatrix* input = builder->build_by_fill(num_rows, num_columns, 0.);
    std::vector<int> column_order;
    random_permutation(num_columns, column_order);
    for(int r = 0; r < num_rows; r++)
    {
        const double val0 = uniform_rand_double();
        const double val1 = double(r) / double(num_rows - 1);
        const double val2 = pow(double(2 * r) / double(num_rows - 1) - 1., 2.);
        const double val3 = .7 * uniform_rand_double() + .15 * val0 + .1 * val1 + .05 * val2;
        const double val4 = .7 * uniform_rand_double() + .05 * val0 + .1 * val1 + .15 * val2;
        input->set_value(r, column_order[0], val0);
        input->set_value(r, column_order[1], val1);
        input->set_value(r, column_order[2], val2);
        input->set_value(r, column_order[3], val3);
        input->set_value(r, column_order[4], val4);
    }
    return input;
}
void test_mostlyIndependentInput(const int& num_rows,
                                 const double& upper_variation_fraction,
                                 int& compressed_dimension,
                                 double& decompressed_mean_error)
{
    // allocate
    AbstractAuthority authority;

    // get input
    AbstractInterface::DenseMatrix* input = build_mostlyIndependentInput(authority.dense_builder, num_rows);

    // build PCA solver
    PrincipalComponentAnalysisSolver solver(&authority);
    solver.set_upper_variation_fraction(upper_variation_fraction);

    // solve
    AbstractInterface::DenseMatrix* compressed_basis = NULL;
    std::vector<double> input_column_means;
    std::vector<double> basis_column_stds;
    solver.solve(input, compressed_basis, input_column_means, basis_column_stds);

    // count compressed
    compressed_dimension = basis_column_stds.size();

    // for each row, add error contribution to mean
    decompressed_mean_error = 0.;
    for(int r = 0; r < num_rows; r++)
    {
        std::vector<double> this_row;
        input->get_row(r, this_row);

        // get in decompressed
        std::vector<double> decompressed_row = this_row;
        solver.get_decompressed_row(decompressed_row, compressed_basis, input_column_means, basis_column_stds);

        // compute error
        const double this_delta = authority.dense_vector_operations->delta_squared(this_row, decompressed_row);

        decompressed_mean_error += this_delta / double(num_rows);
    }

    // clean up
    safe_free(compressed_basis);
    safe_free(input);
}
PSL_TEST(PrincipalComponentsAnalysis,mostlyIndependentInput)
{
    set_rand_seed();
    // pose problems
    const int num_rows = 150;
    std::vector<double> upper_variation_fractions = {.95, .85, .7, .5};
    std::vector<double> expected_compressed_dimension = {5, 4, 3, 2};

    // store previous
    double previous_decompressed_mean_error = -1;

    // for each simulation
    assert(upper_variation_fractions.size() == expected_compressed_dimension.size());
    const int num_simulations = upper_variation_fractions.size();
    for(int sim = 0; sim < num_simulations; sim++)
    {
        // do simulation
        int computed_compressed_dimension = -1;
        double computed_decompressed_mean_error = 0.;
        test_mostlyIndependentInput(num_rows,
                                    upper_variation_fractions[sim],
                                    computed_compressed_dimension,
                                    computed_decompressed_mean_error);

        // confirm expectations
        EXPECT_EQ(computed_compressed_dimension, expected_compressed_dimension[sim]);
        EXPECT_GT(computed_decompressed_mean_error, previous_decompressed_mean_error);
        previous_decompressed_mean_error = computed_decompressed_mean_error;
    }
}

void build_majorAndMinorColumns_input(AbstractInterface::DenseMatrix* input, const int& num_major_columns)
{
    // get sizes
    const int num_rows = input->get_num_rows();
    const int num_columns = input->get_num_columns();
    assert(num_major_columns <= num_columns);

    std::vector<int> column_order;
    random_permutation(num_columns, column_order);
    for(int r = 0; r < num_rows; r++)
    {
        // major column
        for(int c = 0; c < num_major_columns; c++)
        {
            const double this_value = uniform_rand_double(-1., 1.) * 100.;
            input->set_value(r, column_order[c], this_value);
        }
        // minor column
        for(int c = num_major_columns; c < num_columns; c++)
        {
            const double this_value = uniform_rand_double(-1., 1.) * .01;
            input->set_value(r, column_order[c], this_value);
        }
    }
}

void test_PrincipalComponentsAnalysis_predictCounts(const int& num_rows,
                                                    const int& num_major_columns,
                                                    const int& num_columns,
                                                    const double& upper_variation_fraction)
{
    // allocate
    AbstractAuthority authority;

    // build input
    AbstractInterface::DenseMatrix* input = authority.dense_builder->build_by_fill(num_rows, num_columns, 0.);
    build_majorAndMinorColumns_input(input, num_major_columns);

    // build PCA solver
    PrincipalComponentAnalysisSolver solver(&authority);
    solver.set_upper_variation_fraction(upper_variation_fraction);

    // solve
    AbstractInterface::DenseMatrix* compressed_basis = NULL;
    std::vector<double> input_column_means;
    std::vector<double> basis_column_stds;
    solver.solve(input, compressed_basis, input_column_means, basis_column_stds);

    // expect only retained major
    EXPECT_EQ(int(basis_column_stds.size()), num_major_columns);

    // expect correct sizes
    for(int r = 0; r < num_rows; r++)
    {
        std::vector<double> this_row;
        input->get_row(r, this_row);

        // get in compressed
        std::vector<double> compressed_row = this_row;
        solver.get_compressed_row(compressed_row, compressed_basis, input_column_means, basis_column_stds);
        EXPECT_EQ(int(compressed_row.size()), num_major_columns);

        // should be non-zero
        const double this_dot = authority.dense_vector_operations->dot(compressed_row, compressed_row);
        EXPECT_GT(this_dot, 1e-5);

        // get in decompressed
        std::vector<double> decompressed_row = this_row;
        solver.get_decompressed_row(decompressed_row, compressed_basis, input_column_means, basis_column_stds);
        EXPECT_EQ(int(decompressed_row.size()), num_columns);

        // difference should be close, but not zero
        const double this_delta = authority.dense_vector_operations->delta_squared(this_row, decompressed_row);
        EXPECT_NEAR(0., this_delta, 1e-3);
        EXPECT_GT(this_delta, 0.);
    }

    // clean up
    safe_free(compressed_basis);
    safe_free(input);
}
PSL_TEST(PrincipalComponentsAnalysis,predictCounts2Of6)
{
    set_rand_seed();
    test_PrincipalComponentsAnalysis_predictCounts(150, 2, 6, .92);
    test_PrincipalComponentsAnalysis_predictCounts(150, 2, 6, .99);
}
PSL_TEST(PrincipalComponentsAnalysis,predictCounts5Of6)
{
    set_rand_seed();
    test_PrincipalComponentsAnalysis_predictCounts(150, 5, 6, .93);
    test_PrincipalComponentsAnalysis_predictCounts(150, 5, 6, .98);
}
PSL_TEST(PrincipalComponentsAnalysis,predictCounts3Of7)
{
    set_rand_seed();
    test_PrincipalComponentsAnalysis_predictCounts(135, 3, 7, .94);
    test_PrincipalComponentsAnalysis_predictCounts(135, 3, 7, .97);
}
PSL_TEST(PrincipalComponentsAnalysis,predictCounts4Of7)
{
    set_rand_seed();
    test_PrincipalComponentsAnalysis_predictCounts(135, 4, 7, .93);
    test_PrincipalComponentsAnalysis_predictCounts(135, 4, 7, .98);
}

}
}
