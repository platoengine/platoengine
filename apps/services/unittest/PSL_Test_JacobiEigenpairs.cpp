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

#include "PSL_UnitTestingHelper.hpp"

#include "PSL_FreeHelpers.hpp"
#include "PSL_Interface_BasicGlobalUtilities.hpp"
#include "PSL_Interface_DenseMatrixBuilder.hpp"
#include "PSL_Interface_BasicDenseVectorOperations.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_JacobiEigenpairsSolver.hpp"
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Random.hpp"

#include <vector>
#include <cstddef>

namespace PlatoSubproblemLibrary
{
namespace TestingJacobiEigenpairs
{

void test_JacobiEigenpairs_solveExample(const size_t& dimension,
                                        const std::vector<double>& input_rowMajor,
                                        const std::vector<double>& expected_eigenvectors_rowMajor,
                                        const std::vector<double>& expected_eigenvalues)
{
    // allocate
    AbstractAuthority authority;

    // set problem
    AbstractInterface::DenseMatrix* input = authority.dense_builder->build_by_row_major(dimension, dimension, input_rowMajor);

    // build solver
    JacobiEigenpairsSolver solver(&authority);
    solver.set_tolerance(1e-7);

    // solve problem
    std::vector<double> computed_eigenvalues;
    AbstractInterface::DenseMatrix* computed_eigenvectors = NULL;
    solver.solve(input, computed_eigenvalues, computed_eigenvectors);

    // set expected
    AbstractInterface::DenseMatrix* expected_eigenvectors =
            authority.dense_builder->build_by_row_major(dimension, dimension, expected_eigenvectors_rowMajor);

    // compare
    const double expected_tolerance = 1e-5;
    expect_near_vectors(expected_eigenvalues, computed_eigenvalues, expected_tolerance);
    expect_near_matrix(expected_eigenvectors, computed_eigenvectors, expected_tolerance);

    // clean up
    safe_free(input);
    safe_free(computed_eigenvectors);
    safe_free(expected_eigenvectors);
}

PSL_TEST(JacobiEigenpairs,solve5by5)
{
    set_rand_seed();
    // set problem
    const size_t dimension = 5u;
    std::vector<double> input_matrix = {2., -1., 2., -2., 0.,
                                         -1., 5., 8., 2., 1.,
                                         2., 8., 2., 2., -4.,
                                         -2., 2., 2., 10., -9.,
                                         0., 1., -4., -9., -5};
    std::vector<double> expected_eigenvectors = {.028512335426032, .383396811227453, .904187440572832, .170937367723134,
                                                 .073616713424412, .285210196726090, .489557842264150, -.311555281466014,
                                                 .664068387815193, -.375412767039558, -.379348043775042, -.630258629380463,
                                                 .216140817541520, .504551049070395, -.396964113806218, -.337272932879236,
                                                 .300233401132535, .035688972737555, -.483269353217089, -.749188830040534,
                                                 -.812517212644204, .354928118291776, -.193359310917280, .204138589550132,
                                                 .367125895492037};
    std::vector<double> expected_eigenvalues = {-10.954407724372347, -4.130833306557304, 2.743716122323828, 9.672818616389282,
                                                16.668706292216541};

    // test problem
    test_JacobiEigenpairs_solveExample(dimension, input_matrix, expected_eigenvectors, expected_eigenvalues);
}

PSL_TEST(JacobiEigenpairs,solve4by4)
{
    set_rand_seed();
    // set problem
    const size_t dimension = 4u;
    std::vector<double> input_matrix = {-2., 5., -2., -4.,
                                        5., 8., -1., -6.,
                                       -2., -1., -2., -1.,
                                       -4., -6., -1., -4.};
    std::vector<double> expected_eigenvectors = {.490942759917225, .702527917543915, .339651815760481, .387384015683244,
                                                 .163332075182686, -.497434823202030, -.161338950825583, .836570961024218,
                                                 .317748299412537, .261717086995978, -.907690743748691, -.081472070721994,
                                                 .794565200547958, -.436482861188976, .186289829862119, -.378741274825456};
    std::vector<double> expected_eigenvalues = {-8.104776476128785, -3.800178990726095, -1.224125069310378, 13.129080536165272};

    // test problem
    test_JacobiEigenpairs_solveExample(dimension, input_matrix, expected_eigenvectors, expected_eigenvalues);
}

PSL_TEST(JacobiEigenpairs,solve3by3)
{
    set_rand_seed();
    // set problem
    const size_t dimension = 3u;
    std::vector<double> input_matrix = {5., -1., -2.,
                                        -1., -8., -2.,
                                        -2., -2., 2.};
    std::vector<double> expected_eigenvectors = {.102208868896495, .435377246900776, .894427190999916,
                                                 .973533119926844, -.228545978755928, 0.,
                                                 .204417737792991, .870754493801552, -.447213595499958};
    std::vector<double> expected_eigenvalues = {-8.524937810560449, 1.524937810560445, 6.};

    // test problem
    test_JacobiEigenpairs_solveExample(dimension, input_matrix, expected_eigenvectors, expected_eigenvalues);
}

PSL_TEST(JacobiEigenpairs,solve2by2)
{
    set_rand_seed();
    // set problem
    const size_t dimension = 2u;
    std::vector<double> input_matrix = {-9., 3., 3., -5.};
    std::vector<double> expected_eigenvectors = {.881674598767944, .471857925532024, -.471857925532024, .881674598767944};
    std::vector<double> expected_eigenvalues = {-10.605551275463984, -3.394448724536010};

    // test problem
    test_JacobiEigenpairs_solveExample(dimension, input_matrix, expected_eigenvectors, expected_eigenvalues);
}

}
}
