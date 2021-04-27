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
#include "PSL_AbstractAuthority.hpp"
#include "PSL_Abstract_DenseVectorOperations.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Random.hpp"
#include "PSL_Abstract_DenseMatrix.hpp"
#include "PSL_Point.hpp"
#include "PSL_OverhangInclusionFunction.hpp"
#include "PSL_ParameterData.hpp"

#include <vector>
#include <cstddef>
#include <iostream>
#include <cmath>
#include <math.h>

namespace PlatoSubproblemLibrary
{
namespace TestingFreeHelpers
{

PSL_TEST(FreeHelpers, fractionalSphericalSector)
{
    set_rand_seed();

    const double a0 = 45.;
    double computed = fractional_spherical_sector(0., 0., a0);
    EXPECT_FLOAT_EQ(computed, 1.);
    computed = fractional_spherical_sector(0., .5, a0);
    EXPECT_NEAR(computed, .656, .001);
    computed = fractional_spherical_sector(.5, 0., a0);
    EXPECT_FLOAT_EQ(computed, .875);
    computed = fractional_spherical_sector(.5, .5, a0);
    EXPECT_NEAR(computed, .574, .001);
    computed = fractional_spherical_sector(.1, .1, a0);
    EXPECT_NEAR(computed, .984, .001);
    computed = fractional_spherical_sector(.9, .9, a0);
    EXPECT_NEAR(computed, .032, .001);

    const double a1 = 26.57;
    computed = fractional_spherical_sector(0., 0., a1);
    EXPECT_FLOAT_EQ(computed, 1.);
    computed = fractional_spherical_sector(0., .5, a1);
    EXPECT_NEAR(computed, .690, .001);
    computed = fractional_spherical_sector(.5, 0., a1);
    EXPECT_FLOAT_EQ(computed, .875);
    computed = fractional_spherical_sector(.5, .5, a1);
    EXPECT_NEAR(computed, .604, .001);
    computed = fractional_spherical_sector(.1, .1, a1);
    EXPECT_NEAR(computed, .302, .001);
    computed = fractional_spherical_sector(.9, .9, a1);
    EXPECT_NEAR(computed, .256, .001);

    const double a2 = 63.44;
    computed = fractional_spherical_sector(0., 0., a2);
    EXPECT_FLOAT_EQ(computed, 1.);
    computed = fractional_spherical_sector(0., .5, a2);
    EXPECT_NEAR(computed, .495, .001);
    computed = fractional_spherical_sector(.5, 0., a2);
    EXPECT_FLOAT_EQ(computed, .875);
    computed = fractional_spherical_sector(.5, .5, a2);
    EXPECT_NEAR(computed, .433, .001);
    computed = fractional_spherical_sector(.1, .1, a2);
    EXPECT_NEAR(computed, .794, .001);
    computed = fractional_spherical_sector(.9, .9, a2);
    EXPECT_NEAR(computed, .058, .001);
}

PSL_TEST(FreeHelpers, computeOverhang)
{
    set_rand_seed();

    // establish points
    std::vector<double> center_data(3);
    normal_rand_double(0., 1., center_data);
    Point center(0u, center_data);
    Point build_direction(0u, {0.,0.,1.0});
    Point other(0u, {0.,0.,0.});
    std::vector<double> other_data;

    // set parameter data
    ParameterData param_data;
    const double bds = 1.29;
    param_data.set_build_direction_x(build_direction(0) * bds);
    param_data.set_build_direction_y(build_direction(1) * bds);
    param_data.set_build_direction_z(build_direction(2) * bds);

    // set overhang functions
    OverhangInclusionFunction func_angle44_radius1;
    param_data.set_absolute(1.);
    param_data.set_max_overhang_angle(44.);
    func_angle44_radius1.build(param_data.get_absolute(), &param_data);

    // set overhang functions
    OverhangInclusionFunction func_angle51_radius2;
    param_data.set_absolute(2.);
    param_data.set_max_overhang_angle(51.);
    func_angle51_radius2.build(param_data.get_absolute(), &param_data);

    // 0 -- vertical (always allowable)
    other_data = center_data;
    other_data[2] -= 0.5;
    other.set(0u, other_data);
    EXPECT_FLOAT_EQ(0., compute_overhang_angle(&center, &other, &build_direction));
    EXPECT_FLOAT_EQ(1., func_angle44_radius1.evaluate(&center, &other));
    EXPECT_FLOAT_EQ(1., func_angle51_radius2.evaluate(&center, &other));

    // 45
    other_data = center_data;
    other_data[1] -= 0.9;
    other_data[2] -= 0.9;
    other.set(0u, other_data);
    EXPECT_FLOAT_EQ(45., compute_overhang_angle(&center, &other, &build_direction));
    EXPECT_GE(0., func_angle44_radius1.evaluate(&center, &other));
    EXPECT_FLOAT_EQ(1., func_angle51_radius2.evaluate(&center, &other));

    // 90 -- horizontal (always denied)
    other_data = center_data;
    other_data[0] += 0.5;
    other_data[1] -= 1.5;
    other.set(0u, other_data);
    EXPECT_FLOAT_EQ(90., compute_overhang_angle(&center, &other, &build_direction));
    EXPECT_GE(0., func_angle44_radius1.evaluate(&center, &other));
    EXPECT_GE(0., func_angle51_radius2.evaluate(&center, &other));

    // 135
    other_data = center_data;
    other_data[0] -= 1.3;
    other_data[2] += 1.3;
    other.set(0u, other_data);
    EXPECT_FLOAT_EQ(135., compute_overhang_angle(&center, &other, &build_direction));
    EXPECT_GE(0., func_angle44_radius1.evaluate(&center, &other));
    EXPECT_GE(0., func_angle51_radius2.evaluate(&center, &other));

    // 180 -- other above (always denied)
    other_data = center_data;
    other_data[2] += 1.4;
    other.set(0u, other_data);
    EXPECT_FLOAT_EQ(180., compute_overhang_angle(&center, &other, &build_direction));
    EXPECT_GE(0., func_angle44_radius1.evaluate(&center, &other));
    EXPECT_GE(0., func_angle51_radius2.evaluate(&center, &other));

    // undefined (180)
    other_data = center_data;
    other.set(0u, other_data);
    EXPECT_FLOAT_EQ(180., compute_overhang_angle(&center, &other, &build_direction));
    EXPECT_GE(0., func_angle44_radius1.evaluate(&center, &other));
    EXPECT_GE(0., func_angle51_radius2.evaluate(&center, &other));
}

PSL_TEST(FreeHelpers, checkHeavisideDerivative_threshold)
{
    set_rand_seed();
    // test that heaviside derivative agrees with finite difference approximation

    double heaviside_beta = 6.2;
    double threshold = 0.5;
    double control_base = 0.425;
    double control_step = 1e-3;

    // evaluate finite difference
    double plus_step = heaviside_apply(heaviside_beta, threshold, control_base + control_step);
    double minus_step = heaviside_apply(heaviside_beta, threshold, control_base - control_step);
    double finite_diff_approx = (plus_step - minus_step) / (2. * control_step);

    // evaluate derivative
    double analytical_derivative = heaviside_gradient(heaviside_beta, threshold, control_base);

    // expect small error
    double tolerance = 1e-3;
    EXPECT_NEAR(finite_diff_approx, analytical_derivative, tolerance);
}
PSL_TEST(FreeHelpers, checkHeavisideDerivative_noThreshold)
{
    set_rand_seed();
    // test that heaviside derivative agrees with finite difference approximation

    double heaviside_beta = 6.2;
    double control_base = 0.425;
    double control_step = 1e-3;

    // evaluate finite difference
    double plus_step = heaviside_apply(heaviside_beta, control_base + control_step);
    double minus_step = heaviside_apply(heaviside_beta, control_base - control_step);
    double finite_diff_approx = (plus_step - minus_step) / (2. * control_step);

    // evaluate derivative
    double analytical_derivative = heaviside_gradient(heaviside_beta, control_base);

    // expect small error
    double tolerance = 1e-3;
    EXPECT_NEAR(finite_diff_approx, analytical_derivative, tolerance);
}
PSL_TEST(FreeHelpers, checkTANHDerivative_finite_difference)
{
    // test that tanh derivative agrees with finite difference approximation

    double heaviside_beta = 6.2;
    double control_base = 0.425;
    double control_step = 1e-3;

    // evaluate finite difference
    double plus_step = tanh_apply(heaviside_beta, control_base + control_step);
    double minus_step = tanh_apply(heaviside_beta, control_base - control_step);
    double finite_diff_approx = (plus_step - minus_step) / (2. * control_step);

    // evaluate derivative
    double analytical_derivative = tanh_gradient(heaviside_beta, control_base);

    // expect small error
    double tolerance = 1e-3;
    EXPECT_NEAR(finite_diff_approx, analytical_derivative, tolerance);
}
PSL_TEST(FreeHelpers, checkTANHEval)
{
    // test that tanh derivative agrees with finite difference approximation

    double beta = 6.2;

    const int num_comps = 5;
    double input[num_comps] = {0.0,0.25,0.5,0.75,1.0};

    double expected_output[num_comps] = {0.0,0.0412490195125303,0.5,0.9587509804874696,1.0};
    double output[num_comps] = {0.0};

    for(int i = 0; i < num_comps; i++)
      output[i] = tanh_apply(beta,input[i]);

    for(int i = 0; i < num_comps; i++)
      EXPECT_DOUBLE_EQ(output[i], expected_output[i]);
}
PSL_TEST(FreeHelpers, checkTANHDerivative)
{
    // test that tanh derivative agrees with finite difference approximation

    double beta = 6.2;

    const int num_comps = 5;
    double input[num_comps] = {0.0,0.25,0.5,0.75,1.0};

    double expected_output[num_comps] = {0.025165043534531462,0.51356812193240786,3.112608057122421,0.51356812193240786,0.025165043534531462};
    double output[num_comps] = {0.0};

    for(int i = 0; i < num_comps; i++)
      output[i] = tanh_gradient(beta,input[i]);

    for(int i = 0; i < num_comps; i++)
      EXPECT_DOUBLE_EQ(output[i], expected_output[i]);
}
PSL_TEST(FreeHelpers,fillWithIndex)
{
    set_rand_seed();
    std::vector<int> computed0(4);
    fill_with_index(computed0);
    std::vector<int> expected0 = {0, 1, 2, 3};
    expect_equal_vectors(computed0, expected0);

    std::vector<size_t> computed1(3u);
    fill_with_index(computed1);
    std::vector<size_t> expected1 = {0u, 1u, 2u};
    expect_equal_vectors(computed1, expected1);
}

void check_orthogonal_vectors(const std::vector<double>& input, const std::vector<double>& output)
{
    set_rand_seed();
    AbstractAuthority authority;
    const double tol = 1e-5;

    // check orthogonal
    ASSERT_EQ(input.size(), output.size());
    double inp_result = authority.dense_vector_operations->dot(input, output);
    EXPECT_NEAR(inp_result, 0., tol);

    // check not trivial solution
    double self_inp_result = authority.dense_vector_operations->dot(output, output);
    EXPECT_GT(self_inp_result, tol);
}

PSL_TEST(FreeHelpers,randomOrthogonal)
{
    set_rand_seed();

    for(int repeat = 0; repeat < 5; repeat++)
    {
        // a point
        const int dimension = rand_int(2, 15);
        std::vector<double> vec(dimension);
        uniform_rand_double(-1., 2., vec);

        // make orthogonal vector
        std::vector<double> ortho_vec;
        random_orthogonal_vector(vec, ortho_vec);

        check_orthogonal_vectors(vec, ortho_vec);
    }
}

PSL_TEST(FreeHelpers,randomOrthogonalWithZeros)
{
    set_rand_seed();
    for(int repeat = 0; repeat < 5; repeat++)
    {
        // a point
        std::vector<double> vec = {1., 0., -2., 0.};

        // make orthogonal vector
        std::vector<double> ortho_vec;
        random_orthogonal_vector(vec, ortho_vec);

        check_orthogonal_vectors(vec, ortho_vec);
    }
}

PSL_TEST(FreeHelpers,solveHyperplaneSeparators)
{
    set_rand_seed();
    AbstractAuthority authority;

    for(int repeat = 0; repeat < 5; repeat++)
    {
        // two random points
        const int dimension = rand_int(2, 15);
        std::vector<double> p1(dimension);
        uniform_rand_double(-1., 2., p1);
        std::vector<double> p2(dimension);
        uniform_rand_double(3., 5., p2);
        const double p1_to_p2 = std::sqrt(authority.dense_vector_operations->delta_squared(p1, p2));

        // solve for random
        const int num_separators = rand_int(2, 8);
        std::vector<std::vector<double> > hyperplanes(num_separators);
        solve_for_hyperplane_separators(p1, p2, hyperplanes);

        // for each separator
        for(int sep = 0; sep < num_separators; sep++)
        {
            // expect different sides of hyperplane
            const bool sign_p1 = get_hyperplane_sign(p1, hyperplanes[sep]);
            const bool sign_p2 = get_hyperplane_sign(p2, hyperplanes[sep]);
            EXPECT_NE(sign_p1, sign_p2);

            // expect distances to sum
            const double p1_to_plane = point_to_plane_distance(p1, hyperplanes[sep]);
            const double p2_to_plane = point_to_plane_distance(p2, hyperplanes[sep]);
            EXPECT_FLOAT_EQ(p1_to_p2, p1_to_plane + p2_to_plane);

            // for each other separator
            for(int other_sep = sep + 1; other_sep < num_separators; other_sep++)
            {
                // expect same plane normal
                std::vector<double> sep_plane_normal = hyperplanes[sep];
                sep_plane_normal.resize(dimension);
                std::vector<double> other_sep_plane_normal = hyperplanes[other_sep];
                other_sep_plane_normal.resize(dimension);
                expect_equal_float_vectors(sep_plane_normal, other_sep_plane_normal);

                // expect different plane offsets
                EXPECT_NE(hyperplanes[sep].back(), hyperplanes[other_sep].back());
            }
        }
    }
}

PSL_TEST(FreeHelpers,solveBisector)
{
    set_rand_seed();
    AbstractAuthority authority;

    for(int repeat = 0; repeat < 5; repeat++)
    {
        // two random points
        const int dimension = rand_int(2, 15);
        std::vector<double> p1(dimension);
        uniform_rand_double(-1., 2., p1);
        std::vector<double> p2(dimension);
        uniform_rand_double(3., 5., p2);

        // solve for random
        std::vector<double> hyperplane;
        solve_for_bisector(p1, p2, hyperplane);

        // expect same distance
        const double p1_to_plane = point_to_plane_distance(p1, hyperplane);
        const double p2_to_plane = point_to_plane_distance(p2, hyperplane);
        const double p1_to_p2 = std::sqrt(authority.dense_vector_operations->delta_squared(p1, p2));
        EXPECT_FLOAT_EQ(p1_to_plane, p2_to_plane);
        EXPECT_FLOAT_EQ(p1_to_plane, p1_to_p2 * .5);
    }
}

PSL_TEST(FreeHelpers,pointToPlaneDistance)
{
    set_rand_seed();
    const std::vector<double> hyperplane = {-4., 3., -3.};

    // test versus expected distance
    const std::vector<double> point0 = {-4., 4.};
    EXPECT_FLOAT_EQ(5., point_to_plane_distance(point0, hyperplane));
    EXPECT_EQ(get_hyperplane_sign(point0, hyperplane), true);

    const std::vector<double> point1 = {7., 2.};
    EXPECT_FLOAT_EQ(5., point_to_plane_distance(point1, hyperplane));
    EXPECT_EQ(get_hyperplane_sign(point1, hyperplane), false);

    const std::vector<double> point2 = {5., -9.};
    EXPECT_FLOAT_EQ(10., point_to_plane_distance(point2, hyperplane));
    EXPECT_EQ(get_hyperplane_sign(point2, hyperplane), false);

    const std::vector<double> point3 = {3., 5.};
    EXPECT_FLOAT_EQ(0., point_to_plane_distance(point3, hyperplane));
    EXPECT_EQ(get_hyperplane_sign(point3, hyperplane), true);
}

PSL_TEST(FreeHelpers,simpleBinaryBoolVector)
{
    set_rand_seed();
    size_t expected = 0;
    size_t computed = 0;

    std::vector<bool> binary_vec(4, false);
    for(int a = 0; a < 2; a++)
    {
        binary_vec[3] = a;
        for(int b = 0; b < 2; b++)
        {
            binary_vec[2] = b;
            for(int c = 0; c < 2; c++)
            {
                binary_vec[1] = c;
                for(int d = 0; d < 2; d++)
                {
                    binary_vec[0] = d;

                    // convert
                    binary_bool_vector_to_integer(binary_vec, computed);

                    // compare to expectations
                    EXPECT_EQ(expected, computed);

                    // advance expectations
                    expected++;
                }
            }
        }
    }
}

PSL_TEST(FreeHelpers,longBinaryBoolVector)
{
    int computed = -1;

    std::vector<bool> example0 = {false, false, false, false, false, false, true};
    binary_bool_vector_to_integer(example0, computed);
    EXPECT_EQ(computed, 64);

    std::vector<bool> example1 = {true, false, true, false, false, false, false, true};
    binary_bool_vector_to_integer(example1, computed);
    EXPECT_EQ(computed, 1+4+128);

    std::vector<bool> example2 = {true, false, false, false, true, false, false, false, true};
    binary_bool_vector_to_integer(example2, computed);
    EXPECT_EQ(computed, 1+16+256);

    std::vector<bool> example3 = {false, false, false, false, false, true, false, false, false, false, true};
    binary_bool_vector_to_integer(example3, computed);
    EXPECT_EQ(computed, 32+1024);
}

PSL_TEST(FreeHelpers,isPrime)
{
    EXPECT_EQ(is_prime(1), false);
    EXPECT_EQ(is_prime(2), true);
    EXPECT_EQ(is_prime(3), true);
    EXPECT_EQ(is_prime(4), false);
    EXPECT_EQ(is_prime(5), true);
    EXPECT_EQ(is_prime(6), false);
    EXPECT_EQ(is_prime(7), true);
    EXPECT_EQ(is_prime(8), false);
    EXPECT_EQ(is_prime(9), false);
    EXPECT_EQ(is_prime(10), false);
    EXPECT_EQ(is_prime(11), true);
    EXPECT_EQ(is_prime(12), false);
    EXPECT_EQ(is_prime(13), true);
    EXPECT_EQ(is_prime(14), false);
    EXPECT_EQ(is_prime(15), false);
    EXPECT_EQ(is_prime(16), false);
    EXPECT_EQ(is_prime(17), true);
    EXPECT_EQ(is_prime(18), false);
    EXPECT_EQ(is_prime(19), true);
    EXPECT_EQ(is_prime(20), false);
    EXPECT_EQ(is_prime(21), false);
    EXPECT_EQ(is_prime(22), false);
    EXPECT_EQ(is_prime(23), true);
    EXPECT_EQ(is_prime(24), false);
    EXPECT_EQ(is_prime(25), false);

    EXPECT_EQ(is_prime(360), false);
    EXPECT_EQ(is_prime(361), false);
    EXPECT_EQ(is_prime(362), false);

    EXPECT_EQ(is_prime(441), false);
    EXPECT_EQ(is_prime(442), false);
    EXPECT_EQ(is_prime(443), true);
    EXPECT_EQ(is_prime(444), false);
    EXPECT_EQ(is_prime(445), false);
    EXPECT_EQ(is_prime(446), false);
    EXPECT_EQ(is_prime(447), false);
    EXPECT_EQ(is_prime(448), false);
    EXPECT_EQ(is_prime(449), true);

    EXPECT_EQ(is_prime(971), true);
    EXPECT_EQ(is_prime(972), false);
    EXPECT_EQ(is_prime(973), false);
    EXPECT_EQ(is_prime(974), false);
    EXPECT_EQ(is_prime(975), false);
    EXPECT_EQ(is_prime(976), false);
    EXPECT_EQ(is_prime(977), true);
    EXPECT_EQ(is_prime(978), false);
    EXPECT_EQ(is_prime(979), false);

    EXPECT_EQ(is_prime(8189), false);
    EXPECT_EQ(is_prime(8191), true);
    EXPECT_EQ(is_prime(8193), false);

    EXPECT_EQ(is_prime(25800), false);
    EXPECT_EQ(is_prime(25801), true);
    EXPECT_EQ(is_prime(25802), false);
}

PSL_TEST(FreeHelpers, transferVectorTypes)
{
    const std::vector<double> original = {1., 0.1, 2.3, -5., 4.2};

    // transfer
    std::vector<float> computed_f;
    transfer_vector_types(computed_f, original);
    std::vector<double> computed_d;
    transfer_vector_types(computed_d, computed_f);

    // confirm expectations
    expect_equal_float_vectors(original, computed_f);
    expect_equal_float_vectors(original, computed_d);
}

PSL_TEST(FreeHelpers, testAxpy)
{
    // pose problem
    const std::vector<double> x0 = {1., 0., 2., -5., 4.};
    std::vector<double> y0 = {5., 7., 8., -2., -3.};
    const double alpha0 = 1.2;
    axpy(alpha0, x0, y0);

    // compare to expectations
    const std::vector<double> expected0 = {6.2, 7., 10.4, -8., 1.8};
    expect_equal_float_vectors(y0, expected0);

    // pose problem
    const std::vector<double> x1 = {1., 2., 0., -5., 4.};
    std::vector<int> y1 = {5, 7, 8, -2, -3};
    const size_t alpha1 = 2u;
    axpy(alpha1, x1, y1);

    // compare to expectations
    const std::vector<int> expected1 = {7, 11, 8, -12, 5};
    expect_equal_vectors(y1, expected1);
}

PSL_TEST(FreeHelpers, subsetsOfSets)
{
    // define sets
    std::set<int> s1 = {1, 3, 2};
    std::set<int> s2 = {2, 1, 2};

    // s2 is a proper subset of s1
    EXPECT_EQ((s1 == s2), false);
    EXPECT_EQ(is_subset(s1,s2), false);
    EXPECT_EQ(is_subset(s2,s1), true);

    // equal sets
    s2.insert(3);
    EXPECT_EQ((s1 == s2), true);
    EXPECT_EQ(is_subset(s1,s2), true);
    EXPECT_EQ(is_subset(s2,s1), true);

    // s1 is a proper subset of s2
    s2.insert(7);
    EXPECT_EQ((s1 == s2), false);
    EXPECT_EQ(is_subset(s1,s2), true);
    EXPECT_EQ(is_subset(s2,s1), false);

    // nonempty symmetric difference
    s1.insert(11);
    EXPECT_EQ((s1 == s2), false);
    EXPECT_EQ(is_subset(s1,s2), false);
    EXPECT_EQ(is_subset(s2,s1), false);
}

PSL_TEST(FreeHelpers, factorials)
{
    // for ints
    const int five = 5;
    const int two = 2;
    const int zero = 0;
    EXPECT_EQ(120, factorial(five));
    EXPECT_EQ(2, factorial(two));
    EXPECT_EQ(1, factorial(zero));
    EXPECT_EQ(1, partial_factorial(zero, zero));
    EXPECT_EQ(2, partial_factorial(zero, two));
    EXPECT_EQ(120, partial_factorial(zero, five));
    EXPECT_EQ(1, partial_factorial(two, zero));
    EXPECT_EQ(2, partial_factorial(two, two));
    EXPECT_EQ(120, partial_factorial(two, five));
    EXPECT_EQ(1, partial_factorial(five, zero));
    EXPECT_EQ(1, partial_factorial(five, two));
    EXPECT_EQ(5, partial_factorial(five, five));

    // for size_t
    const size_t one = 1u;
    const size_t three = 3u;
    const size_t four = 4u;
    EXPECT_EQ(1u, factorial(one));
    EXPECT_EQ(6u, factorial(three));
    EXPECT_EQ(24u, factorial(four));
    EXPECT_EQ(1u, partial_factorial(one, one));
    EXPECT_EQ(6u, partial_factorial(one, three));
    EXPECT_EQ(24u, partial_factorial(one, four));
    EXPECT_EQ(1u, partial_factorial(three, one));
    EXPECT_EQ(3u, partial_factorial(three, three));
    EXPECT_EQ(12u, partial_factorial(three, four));
    EXPECT_EQ(1u, partial_factorial(four, one));
    EXPECT_EQ(1u, partial_factorial(four, three));
    EXPECT_EQ(4u, partial_factorial(four, four));
}

PSL_TEST(FreeHelpers, chooses)
{
    // for ints
    const std::vector<int> ints = {5, 7, 2, 0};
    const std::vector<int> int_expects = {1, 0, 10, 1, 1, 21, 1, 1, 1, 1};
    int counter = 0;
    const int num_ints = ints.size();
    for(int i = 0; i < num_ints; i++)
    {
        for(int j = i; j < num_ints; j++)
        {
            const int expect_value = int_expects[counter++];
            EXPECT_EQ(expect_value, choose(ints[i], ints[j]));
        }
    }

    // for size_t
    const std::vector<size_t> size_ts = {3u, 6u, 4u, 1u};
    const std::vector<size_t> size_t_expects = {1u, 0u, 0u, 3u, 1u, 15u, 6u, 1u, 4u, 1u};
    counter = 0;
    const size_t num_size_ts = size_ts.size();
    for(size_t i = 0; i < num_size_ts; i++)
    {
        for(size_t j = i; j < num_size_ts; j++)
        {
            const size_t expect_value = size_t_expects[counter++];
            EXPECT_EQ(expect_value, choose(size_ts[i], size_ts[j]));
        }
    }
}

PSL_TEST(FreeHelpers, multichooses)
{
    // for ints
    const std::vector<int> ints = {5, 7, 2, 0};
    const std::vector<int> int_expects = {126, 330, 15, 1, 1716, 28, 1, 3, 1, 1};
    int counter = 0;
    const int num_ints = ints.size();
    for(int i = 0; i < num_ints; i++)
    {
        for(int j = i; j < num_ints; j++)
        {
            const int expect_value = int_expects[counter++];
            EXPECT_EQ(expect_value, multichoose(ints[i], ints[j]));
        }
    }

    // for size_t
    const std::vector<size_t> size_ts = {3u, 6u, 4u, 1u};
    const std::vector<size_t> size_t_expects = {10u, 28u, 15u, 3u, 462u, 126u, 6u, 35u, 4u, 1u};
    counter = 0;
    const size_t num_size_ts = size_ts.size();
    for(size_t i = 0; i < num_size_ts; i++)
    {
        for(size_t j = i; j < num_size_ts; j++)
        {
            const size_t expect_value = size_t_expects[counter++];
            EXPECT_EQ(expect_value, multichoose(size_ts[i], size_ts[j]));
        }
    }
}

PSL_TEST(FreeHelpers, sortDescending)
{
    // pose problem
    std::vector<int> computed0 = {0, 1, 3, 2, -2};
    sort_descending(computed0);

    // compare to expectations
    const std::vector<int> gold0 = {3, 2, 1, 0, -2};
    expect_equal_vectors(computed0, gold0);

    // pose problem
    std::vector<double> computed1 = {2.1, -2.15, 2.05, 2.2};
    sort_descending(computed1);

    // compare to expectations
    const std::vector<double> gold1 = {2.2, 2.1, 2.05, -2.15};
    expect_equal_float_vectors(computed1, gold1);
}

PSL_TEST(FreeHelpers, cumulativeSumInt)
{
    // pose problem
    const std::vector<int> input0 = {0, 1, 3, 2, 0};
    std::vector<int> computed0(input0.size());
    cumulative_sum(input0, computed0);
    std::vector<int> alternate0 = input0;
    cumulative_sum(alternate0);

    // compare to expectations
    const std::vector<int> gold0 = {0, 1, 4, 6, 6};
    expect_equal_vectors(computed0, gold0);
    expect_equal_vectors(alternate0, gold0);

    // pose problem
    const std::vector<int> input1 = {2, -5, 4, 1};
    std::vector<int> computed1(input1.size());
    cumulative_sum(input1, computed1);
    std::vector<int> alternate1 = input1;
    cumulative_sum(alternate1);

    // compare to expectations
    const std::vector<int> gold1 = {2, -3, 1, 2};
    expect_equal_vectors(computed1, gold1);
    expect_equal_vectors(alternate1, gold1);
}

PSL_TEST(FreeHelpers, cumulativeSumDouble)
{
    // pose problem
    const std::vector<double> input0 = {0., 1.1, 2.9, 2.4, 0.};
    std::vector<double> computed0(input0.size());
    cumulative_sum(input0, computed0);
    std::vector<double> alternate0 = input0;
    cumulative_sum(alternate0);

    // compare to expectations
    const std::vector<double> gold0 = {0., 1.1, 4., 6.4, 6.4};
    expect_equal_float_vectors(computed0, gold0);
    expect_equal_float_vectors(alternate0, gold0);

    // pose problem
    const std::vector<double> input1 = {-3.1, 2.1, 5.7, 3.1};
    std::vector<double> computed1(input1.size());
    cumulative_sum(input1, computed1);
    std::vector<double> alternate1 = input1;
    cumulative_sum(alternate1);

    // compare to expectations
    const std::vector<double> gold1 = {-3.1, -1., 4.7, 7.8};
    expect_equal_float_vectors(computed1, gold1);
    expect_equal_float_vectors(alternate1, gold1);
}

PSL_TEST(FreeHelpers, sumInt)
{
    // pose problem
    const std::vector<int> input0 = {0, 1, 3, 2, 0};
    const int computed0 = sum(input0);

    // compare to expectations
    EXPECT_EQ(6, computed0);

    // pose problem
    const std::vector<int> input1 = {2, -5, 4, 1};
    const int computed1 = sum(input1);

    // compare to expectations
    EXPECT_EQ(2, computed1);
}
PSL_TEST(FreeHelpers, sumDouble)
{
    // pose problem
    const std::vector<double> input0 = {0., 1.1, 2.9, 2.4, 0.};
    const double computed0 = sum(input0);

    // compare to expectations
    EXPECT_FLOAT_EQ(6.4, computed0);

    // pose problem
    const std::vector<double> input1 = {-3.1, 2.1, 5.7, 3.1};
    const double computed1 = sum(input1);

    // compare to expectations
    EXPECT_FLOAT_EQ(7.8, computed1);
}

PSL_TEST(FreeHelpers, meanDouble)
{
    // pose problem
    const std::vector<double> input0 = {52., 1.1, 2.4, -52., 2.9};
    const double computed0 = mean(input0);

    // compare to expectations
    EXPECT_FLOAT_EQ(1.28, computed0);

    // pose problem
    const std::vector<double> input1 = {0., -3.1, 5.2, 5.7};
    const double computed1 = mean(input1);

    // compare to expectations
    EXPECT_FLOAT_EQ(1.95, computed1);
}

PSL_TEST(FreeHelpers, productInt)
{
    // pose problem
    const std::vector<int> input0 = {4, 1, 3, 2, 0};
    const int computed0 = product(input0);

    // compare to expectations
    EXPECT_EQ(0, computed0);

    // pose problem
    const std::vector<int> input1 = {2, -5, 4, 1};
    const int computed1 = product(input1);

    // compare to expectations
    EXPECT_EQ(-40, computed1);
}

PSL_TEST(FreeHelpers, productDouble)
{
    // pose problem
    const std::vector<double> input0 = {0., 1.1, 2.9, 2.4, 4.1};
    const double computed0 = product(input0);

    // compare to expectations
    EXPECT_FLOAT_EQ(0., computed0);

    // pose problem
    const std::vector<double> input1 = {-3.1, 2.1, 5.7, -7.};
    const double computed1 = product(input1);

    // compare to expectations
    EXPECT_FLOAT_EQ(259.749, computed1);
}

PSL_TEST(FreeHelpers, logProductInt)
{
    // pose problem
    const std::vector<int> input0 = {4, 1, 3, 2, 7};
    const double computed0 = log_product(input0);

    // compare to expectations
    EXPECT_FLOAT_EQ(computed0, 5.123963979403259);

    // pose problem
    const std::vector<int> input1 = {2, 50, 4};
    const double computed1 = log_product(input1);

    // compare to expectations
    EXPECT_FLOAT_EQ(computed1, 5.991464547107982);
}

PSL_TEST(FreeHelpers, logProductDouble)
{
    // pose problem
    const std::vector<double> input0 = {171.254, 1.1, 2.9, 2.4, 4.1};
    const double computed0 = log_product(input0);

    // compare to expectations
    EXPECT_FLOAT_EQ(computed0, 8.589624462394697);

    // pose problem
    const std::vector<double> input1 = {3.1, 2.1, 5.7};
    const double computed1 = log_product(input1);

    // compare to expectations
    EXPECT_FLOAT_EQ(computed1, 3.613805631060982);
}

PSL_TEST(FreeHelpers, countBool)
{
    // pose problem
    const std::vector<bool> input0 = {true, false, true, false, true, true};
    const int computed0 = count(input0);

    // compare to expectations
    EXPECT_EQ(4, computed0);

    // pose problem
    const std::vector<bool> input1 = {false, false};
    const int computed1 = count(input1);

    // compare to expectations
    EXPECT_EQ(0, computed1);
}

PSL_TEST(FreeHelpers, countBoolVec)
{
    // pose problem
    const std::vector<std::vector<bool> > input0 = { {true, false, false}, {}, {false, true}, {true}};
    const int computed0 = count(input0);

    // compare to expectations
    EXPECT_EQ(3, computed0);

    // pose problem
    const std::vector<std::vector<bool> > input1 = { {false, false}, {false, false, false}};
    const int computed1 = count(input1);

    // compare to expectations
    EXPECT_EQ(0, computed1);
}

PSL_TEST(FreeHelpers, countPtr)
{
    int one = 1;
    int two = 2;

    // pose problem
    const std::vector<int*> input0 = {&one, NULL, &two};
    const int computed0 = count(input0);

    // compare to expectations
    EXPECT_EQ(2, computed0);

    // pose problem
    const std::vector<int*> input1 = {NULL, &one, &two, &one};
    const int computed1 = count(input1);

    // compare to expectations
    EXPECT_EQ(3, computed1);
}

PSL_TEST(FreeHelpers, maxInt)
{
    // pose problem
    const std::vector<int> input0 = {3, 1, -1, 2, 0};
    const int computed0 = max(input0);

    // compare to expectations
    EXPECT_EQ(3, computed0);

    // pose problem
    const std::vector<int> input1 = {2, -5, 4, 1};
    const int computed1 = max(input1);

    // compare to expectations
    EXPECT_EQ(4, computed1);

    // pose problem
    const std::vector<int> input2 = {-5, -10, -7, -4, -5, -11, -2};
    const int computed2 = max(input2);

    // compare to expectations
    EXPECT_EQ(-2, computed2);
}

PSL_TEST(FreeHelpers, maxDouble)
{
    // pose problem
    const std::vector<double> input0 = {2.9, 1.1, -3.1, 2.4, 0.};
    const double computed0 = max(input0);

    // compare to expectations
    EXPECT_FLOAT_EQ(2.9, computed0);

    // pose problem
    const std::vector<double> input1 = {-3.1, 2.1, 5.7, 3.1};
    const double computed1 = max(input1);

    // compare to expectations
    EXPECT_FLOAT_EQ(5.7, computed1);

    // pose problem
    const std::vector<double> input2 = {-5.4, -10.1, -7.8, -4.3, -11.9, -2.1};
    const double computed2 = max(input2);

    // compare to expectations
    EXPECT_FLOAT_EQ(-2.1, computed2);
}

PSL_TEST(FreeHelpers, minInt)
{
    // pose problem
    const std::vector<int> input0 = {3, 4, 5, 2, 1};
    const int computed0 = min(input0);

    // compare to expectations
    EXPECT_EQ(1, computed0);

    // pose problem
    const std::vector<int> input1 = {2, -5, 4, 1};
    const int computed1 = min(input1);

    // compare to expectations
    EXPECT_EQ(-5, computed1);

    // pose problem
    const std::vector<int> input2 = {-11, -10, -7, -4, -5, -3, -2};
    const int computed2 = min(input2);

    // compare to expectations
    EXPECT_EQ(-11, computed2);
}

PSL_TEST(FreeHelpers, minDouble)
{
    // pose problem
    const std::vector<double> input0 = {2.9, 1.1, 3.1, 2.4, 0.5};
    const double computed0 = min(input0);

    // compare to expectations
    EXPECT_FLOAT_EQ(0.5, computed0);

    // pose problem
    const std::vector<double> input1 = {-3.1, 2.1, 5.7, 3.1};
    const double computed1 = min(input1);

    // compare to expectations
    EXPECT_FLOAT_EQ(-3.1, computed1);

    // pose problem
    const std::vector<double> input2 = {-5.4, -10.1, -7.8, -4.3, -11.9, -2.1};
    const double computed2 = min(input2);

    // compare to expectations
    EXPECT_FLOAT_EQ(-11.9, computed2);
}

PSL_TEST(FreeHelpers, invertVectorToMap)
{
    // pose problem
    const std::vector<int> input0 = {1, -5};
    std::map<int, int> computed0;
    invert_vector_to_map(input0, computed0);

    // compare to expectations
    ASSERT_EQ(computed0.size(), 2u);
    EXPECT_EQ(computed0[1], 0);
    EXPECT_EQ(computed0[-5], 1);
    const bool no3 = (computed0.find(3) == computed0.end());
    EXPECT_EQ(no3, true);

    // pose problem
    const std::vector<size_t> input1 = {5u, 1u, 4u};
    std::map<size_t, int> computed1;
    invert_vector_to_map(input1, computed1);

    // compare to expectations
    ASSERT_EQ(computed1.size(), 3u);
    EXPECT_EQ(computed1[4u], 2);
    EXPECT_EQ(computed1[1u], 1);
    EXPECT_EQ(computed1[5u], 0);
    const bool no2 = (computed1.find(2u) == computed1.end());
    EXPECT_EQ(no2, true);
}

PSL_TEST(FreeHelpers, safeFree)
{
    // allocate null pointer
    double* _null_ptr_ = NULL;
    const bool is_null0 = (_null_ptr_ == NULL);

    // free memory
    safe_free(_null_ptr_);
    const bool is_null1 = (_null_ptr_ == NULL);

    // allocate memory
    _null_ptr_ = new double;
    const bool is_null2 = (_null_ptr_ == NULL);

    // free memory
    safe_free(_null_ptr_);
    const bool is_null3 = (_null_ptr_ == NULL);

    // free memory
    safe_free(_null_ptr_);
    const bool is_null4 = (_null_ptr_ == NULL);

    // compare to expectations
    EXPECT_EQ(is_null0, true);
    EXPECT_EQ(is_null1, true);
    EXPECT_EQ(is_null2, false);
    EXPECT_EQ(is_null3, true);
    EXPECT_EQ(is_null4, true);
}

PSL_TEST(FreeHelpers, filenamePrefix)
{
    // example without prefix
    const std::string input0 = "example.txt";
    EXPECT_EQ(input0, remove_a_filename_prefix(input0));
    EXPECT_EQ(input0, remove_all_filename_prefix(input0));

    // example with a prefix
    const std::string input1 = "../foo.psl";
    const std::string actual_a_output1 = remove_a_filename_prefix(input1);
    const std::string actual_all_output1 = remove_all_filename_prefix(input1);
    const std::string expected_output1 = "foo.psl";
    EXPECT_EQ(expected_output1, actual_a_output1);
    EXPECT_EQ(expected_output1, actual_all_output1);

    // example with two prefix
    const std::string input2 = "folder0/bar/sigma.exe";
    const std::string actual_a_output2 = remove_a_filename_prefix(input2);
    const std::string actual_all_output2 = remove_all_filename_prefix(input2);
    const std::string expected_a_output2 = "bar/sigma.exe";
    const std::string expected_all_output2 = "sigma.exe";
    EXPECT_EQ(expected_a_output2, actual_a_output2);
    EXPECT_EQ(expected_all_output2, actual_all_output2);

    // example with four prefix
    const std::string input3 = "../bar/a/folder1/longFileName.cpp";
    const std::string actual_a_output3 = remove_a_filename_prefix(input3);
    const std::string actual_all_output3 = remove_all_filename_prefix(input3);
    const std::string expected_a_output3 = "bar/a/folder1/longFileName.cpp";
    const std::string expected_all_output3 = "longFileName.cpp";
    EXPECT_EQ(expected_a_output3, actual_a_output3);
    EXPECT_EQ(expected_all_output3, actual_all_output3);
}

PSL_TEST(FreeHelpers, determinant3X3)
{
    Vector tRow1({1.0,0.0,0.0});
    Vector tRow2({0.0,1.0,0.0});
    Vector tRow3({0.0,0.0,1.0});
    double tDeterminant = determinant3X3(tRow1,tRow2,tRow3);
    EXPECT_DOUBLE_EQ(tDeterminant,1.0);

    tRow1 = Vector({1.0,2.0,0.0});
    tRow2 = Vector({6.3,1.0,-8.4});
    tRow3 = Vector({9.21,0.0,1.0});
    tDeterminant = determinant3X3(tRow1,tRow2,tRow3);
    EXPECT_DOUBLE_EQ(tDeterminant,-166.328);

    tRow1 = Vector({1.0,2.0,0.0});
    tRow2 = Vector({2.0,4.0,0.0});
    tRow3 = Vector({9.21,0.0,1.0});
    tDeterminant = determinant3X3(tRow1,tRow2,tRow3);
    EXPECT_DOUBLE_EQ(tDeterminant,0.0);

    Vector tColumn1({3.9, 4.6, -2.8});
    Vector tColumn2({1.2, 3.6, -1.7});
    Vector tColumn3({5.4, -7.6, -2.8});
    tDeterminant = determinant3X3(tColumn1,tColumn2,tColumn3);
    EXPECT_DOUBLE_EQ(tDeterminant,-36.504);
}

PSL_TEST(FreeHelpers, linearInterpolation)
{
    double a = 2.0;
    double b = 5.0;

    EXPECT_DOUBLE_EQ(linearInterpolation(a,b,0.0),2.0);
    EXPECT_DOUBLE_EQ(linearInterpolation(a,b,1.0),5.0);
    EXPECT_DOUBLE_EQ(linearInterpolation(a,b,0.5),3.5);
    EXPECT_DOUBLE_EQ(linearInterpolation(a,b,2.0/3.0),4.0);
}

}
}
