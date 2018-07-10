// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Abstract_PositiveDefiniteLinearSolver.hpp"
#include "PSL_Abstract_GlobalUtilities.hpp"
#include "PSL_Interface_ConjugateGradient.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_DenseMatrix.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_Interface_BasicDenseVectorOperations.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Random.hpp"

#include <iostream>
#include <vector>

namespace PlatoSubproblemLibrary
{

#define ConjugateGradientTest_AllocateUtilities \
    example::Interface_BasicGlobalUtilities basic_utilities; \
    example::Interface_DenseMatrixBuilder e_builder(&basic_utilities); \
    example::Interface_BasicDenseVectorOperations basic_operations; \
    example::Interface_CojugateGradient cg_solver(&basic_utilities, &basic_operations); \
    AbstractInterface::DenseMatrixBuilder* builder = &e_builder; \
    AbstractInterface::PositiveDefiniteLinearSolver* solver = &cg_solver;

namespace ConjugateGradientTest
{

PSL_TEST(ConjugateGradient,twoByTwo)
{
    set_rand_seed();
    // test solve(...) success for a simple 2 by 2 matrix
    ConjugateGradientTest_AllocateUtilities

    // allocate problem
    const size_t nrows = 2;
    const size_t ncols = 2;
    std::vector<double> matrix_values = {4., 1., 1., 3.};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(nrows, ncols, matrix_values);
    std::vector<double> rhs = {1., 2.};

    // solve
    std::vector<double> x;
    const bool converged = solver->solve(matrix, rhs, x);
    EXPECT_EQ(converged, true);
    ASSERT_EQ(x.size(), ncols);
    EXPECT_FLOAT_EQ(x[0], 1./11.);
    EXPECT_FLOAT_EQ(x[1], 7./11.);

    // clean up
    safe_free(matrix);
}

PSL_TEST(ConjugateGradient,threeByThree)
{
    set_rand_seed();
    // test solve(...) success for a simple 3 by 3 matrix
    ConjugateGradientTest_AllocateUtilities

    // allocate problem
    const size_t nrows = 3;
    const size_t ncols = 3;
    std::vector<double> matrix_values = {10., 2., 5., 2., 6., 8., 5., 8., 12.};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(nrows, ncols, matrix_values);
    std::vector<double> rhs = {21., -14., -13.};

    // solve
    std::vector<double> x;
    const bool converged = solver->solve(matrix, rhs, x);
    EXPECT_EQ(converged, true);
    ASSERT_EQ(x.size(), ncols);
    EXPECT_FLOAT_EQ(x[0], 3.);
    EXPECT_FLOAT_EQ(x[1], -2.);
    EXPECT_FLOAT_EQ(x[2], -1.);

    // clean up
    safe_free(matrix);
}

PSL_TEST(ConjugateGradient,singular)
{
    set_rand_seed();
    // test solve(...) failure by singular
    ConjugateGradientTest_AllocateUtilities

    // allocate problem
    const size_t nrows = 3;
    const size_t ncols = 3;
    std::vector<double> matrix_values = {0., 1., 6., 0., 5., 7., 0., 9., 2.};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(nrows, ncols, matrix_values);
    std::vector<double> rhs = {1.,2.,3.};

    // solve
    std::vector<double> x;
    const bool converged = solver->solve(matrix, rhs, x);
    EXPECT_EQ(converged, false);

    // clean up
    safe_free(matrix);
}

PSL_TEST(ConjugateGradient,nonSymmetric)
{
    set_rand_seed();
    // test solve(...) failure by not symmetric
    ConjugateGradientTest_AllocateUtilities

    // allocate problem
    const size_t nrows = 3;
    const size_t ncols = 3;
    std::vector<double> matrix_values = {8., 1., 6., 3., 5., 7., 4., 9., 2.};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(nrows, ncols, matrix_values);
    std::vector<double> rhs = {1.,2.,3.};

    // solve
    std::vector<double> x;
    const bool converged = solver->solve(matrix, rhs, x);
    EXPECT_EQ(converged, false);

    // clean up
    safe_free(matrix);
}

PSL_TEST(ConjugateGradient,nonPositiveDefinite)
{
    set_rand_seed();
    // test solve(...) failure by not positive definite
    ConjugateGradientTest_AllocateUtilities

    // allocate problem
    const size_t nrows = 3;
    const size_t ncols = 3;
    std::vector<double> matrix_values = {0., 1., 6., 3., -5., 7., 4., 9., 0.};
    AbstractInterface::DenseMatrix* matrix = builder->build_by_row_major(nrows, ncols, matrix_values);
    std::vector<double> rhs = {1.,2.,3.};

    // solve
    std::vector<double> x;
    const bool converged = solver->solve(matrix, rhs, x);
    EXPECT_EQ(converged, false);

    // clean up
    safe_free(matrix);
}

}

}
