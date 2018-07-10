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

#include "PSL_FreeStandardization.hpp"
#include "PSL_Random.hpp"

#include <vector>
#include <math.h>
#include <cmath>

namespace PlatoSubproblemLibrary
{
namespace TestingStandardization
{

void confirm_solve_bipolar_normalize(const std::vector<double>& input,
                                     const std::vector<double>& expected_output,
                                     const double& expected_min,
                                     const double& expected_max)
{
    // transform
    double computed_min = -5.;
    double computed_max = -5.;
    get_bipolar_normalize(input, computed_min, computed_max);

    // transform
    std::vector<double> computed_output = input;
    bipolar_normalize(computed_output, computed_min, computed_max);
    expect_equal_float_vectors(computed_output, expected_output);
}
PSL_TEST(Standardization, solveBipolarNormalize)
{
    set_rand_seed();
    std::vector<double> input0 = {1., 2., 0., 3., 4.};
    std::vector<double> output0 = {-.5, 0., -1., .5, 1.};
    confirm_solve_bipolar_normalize(input0, output0, 0., 4.);

    std::vector<double> input1 = {0., 1., 1., 0.};
    std::vector<double> output1 = {-1., 1., 1., -1.};
    confirm_solve_bipolar_normalize(input1, output1, 0., 1.);

    std::vector<double> input2 = {.5, .5, .5};
    std::vector<double> output2 = {0., 0., 0.};
    confirm_solve_bipolar_normalize(input2, output2, -.5, 1.5);
}

PSL_TEST(Standardization, getMedianIndexes)
{
    set_rand_seed();
    int index0 = -1;
    int index1 = -1;

    // 4:0,1,2,3
    get_median_indexes(4,index0,index1);
    EXPECT_EQ(index0,1);
    EXPECT_EQ(index1,2);

    // 5:0,1,2,3,4
    get_median_indexes(5, index0, index1);
    EXPECT_EQ(index0, 2);
    EXPECT_EQ(index1, 2);
}

void confirm_solve_nonparametric_skew(const std::vector<double>& x, const double& expected_alpha, const double& expected_median)
{
    double computed_alpha = -5.;
    double computed_median = -5.;

    // compare
    solve_nonparametric_skew(x, computed_alpha, computed_median);
    EXPECT_FLOAT_EQ(computed_alpha, expected_alpha);
    EXPECT_FLOAT_EQ(computed_median, expected_median);

    // compute normalized transform
    computed_alpha = -5.;
    computed_median = -5.;
    double computed_std = -5.;
    get_skew_normalize(x, computed_alpha, computed_median, computed_std);
    EXPECT_FLOAT_EQ(computed_alpha, expected_alpha);
    EXPECT_FLOAT_EQ(computed_median, expected_median);

    // transform
    std::vector<double> transformed_data = x;
    skew_normalize(transformed_data, computed_alpha, computed_median, computed_std);

    // confirm skew, mean, std
    double final_skew = -1.;
    double final_mean = -1.;
    double final_std = -1.;
    get_nonparametric_skew(transformed_data, final_skew, final_mean, final_std);
    EXPECT_NEAR(final_skew, 0., .05);
    EXPECT_NEAR(final_mean, 0., 1e-6);
    EXPECT_NEAR(final_std, 1., 1e-6);

    // confirm median
    double final_median = -1.;
    std::vector<double> sorted_transform = transformed_data;
    get_median(sorted_transform, final_median);
    EXPECT_NEAR(final_median, 0., .05);
}
void confirm_solve_bad_nonparametric_skew(const std::vector<double>& input,
                                          const std::vector<double>& expected_output,
                                          const double& expected_alpha,
                                          const double& expected_median,
                                          const double& expected_std)
{
    // compute transform
    double computed_alpha = -5.;
    double computed_median = -5.;
    double computed_std = -5.;
    get_skew_normalize(input, computed_alpha, computed_median, computed_std);

    // compare to expectations
    EXPECT_FLOAT_EQ(computed_alpha, expected_alpha);
    EXPECT_FLOAT_EQ(computed_median, expected_median);
    EXPECT_FLOAT_EQ(computed_std, expected_std);

    // transform
    std::vector<double> transformed_data = input;
    skew_normalize(transformed_data, computed_alpha, computed_median, computed_std);
    expect_equal_float_vectors(transformed_data, expected_output);
}
PSL_TEST(Standardization, solveSkew)
{
    set_rand_seed();
    // good example
    std::vector<double> data0 = {.1, 2.2, 8.6, 5.2, 9.2, 3.5, .6, 8.8, 4.4, .8};
    confirm_solve_nonparametric_skew(data0, .41, 3.95);
    std::vector<double> data1 = {5.4, 1.1, 3.6, 5.6, 3., 4., 2.5};
    confirm_solve_nonparametric_skew(data1, .5, 3.6);
    std::vector<double> data2 = {-2.2, 4., -2.1, 4.1, 3.7, -1.8, -2.4, 4.3};
    confirm_solve_nonparametric_skew(data2, .5, .95);

    // bad examples
    std::vector<double> bad_data0 = {-3., -3., -3.};
    std::vector<double> expected_transform_bad_data0(3u, 0.);
    confirm_solve_bad_nonparametric_skew(bad_data0, expected_transform_bad_data0, -1., -3., 1.);
    std::vector<double> bad_data1 = {0., 0., 1., 0.};
    std::vector<double> expected_transform_bad_data1 = {0., 0., 2., 0.};
    confirm_solve_bad_nonparametric_skew(bad_data1, expected_transform_bad_data1, -1., 0., .5);
}

void confirm_get_nonparametric_skew(const std::vector<double>& x,
                                    const double& expected_skew,
                                    const double& expected_mean,
                                    const double& expected_std)
{
    double computed_skew = -5.;
    double computed_mean = -5.;
    double computed_std = -5.;

    // compare
    get_nonparametric_skew(x, computed_skew, computed_mean, computed_std);
    EXPECT_FLOAT_EQ(computed_skew, expected_skew);
    EXPECT_FLOAT_EQ(computed_mean, expected_mean);
    EXPECT_FLOAT_EQ(computed_std, expected_std);

    // compare
    computed_skew = -5.;
    get_nonparametric_skew(x, computed_skew);
    EXPECT_FLOAT_EQ(computed_skew, expected_skew);
}
PSL_TEST(Standardization, getSkew)
{
    set_rand_seed();
    std::vector<double> data0 = {.3, 8.1, 3.5, 2.7, 5.9};
    confirm_get_nonparametric_skew(data0, .2, 4.1, 3.);

    std::vector<double> data1 = {3.3, 0., 2.1, 2.8, 4.8, -.9, 3.3};
    confirm_get_nonparametric_skew(data1, -.3, 2.2, 2.);

    std::vector<double> data2 = {-.5, -2.1, 3.2, 2.5, 4., 4.3, 2.2, 1.5, -2.6, 4.5};
    confirm_get_nonparametric_skew(data2, -.25, 1.7, 2.6);
}

PSL_TEST(Standardization, simple)
{
    set_rand_seed();
    // pose problem
    std::vector<double> simple_data = {5., 7., 9.8, 3.3, 8.4};
    const std::vector<double> original_simple_data = simple_data;
    double calculated_mean = -1.;
    double calculated_std = -1.;

    // compute mean and std
    get_mean_and_std(simple_data, calculated_mean, calculated_std);
    // compare to expectation
    const double expected_mean = 6.7;
    EXPECT_FLOAT_EQ(calculated_mean, expected_mean);
    const double expected_std = 2.6;
    EXPECT_FLOAT_EQ(calculated_std, expected_std);

    // trail zero
    const double expected_trial_zero = 0.;
    double computed_trial_zero = expected_mean + expected_trial_zero * expected_std;
    standardize(computed_trial_zero, expected_mean, expected_std);
    // compare to expectation
    EXPECT_FLOAT_EQ(expected_trial_zero, computed_trial_zero);

    // trail one
    const double expected_trial_one = .5;
    double computed_trial_one = expected_mean + expected_trial_one * expected_std;
    standardize(computed_trial_one, expected_mean, expected_std);
    // compare to expectation
    EXPECT_FLOAT_EQ(expected_trial_one, computed_trial_one);

    // ensure no mutation
    expect_equal_float_vectors(original_simple_data, simple_data);
}

PSL_TEST(Standardization, lowestFraction)
{
    set_rand_seed();
    // pose problem
    const double fraction = 5. / 7.;
    std::vector<double> simple_data = {11.9, 5., 7., 10.2, 9.8, 3.3, 8.4};
    const std::vector<double> original_simple_data = simple_data;
    double calculated_mean = -1.;
    double calculated_std = -1.;

    // compute mean and std
    get_mean_and_std_of_lowest_fraction(fraction, simple_data, calculated_mean, calculated_std);
    // compare to expectation
    const double expected_mean = 6.7;
    EXPECT_FLOAT_EQ(calculated_mean, expected_mean);
    const double expected_std = 2.6;
    EXPECT_FLOAT_EQ(calculated_std, expected_std);

    // ensure no mutation
    expect_equal_float_vectors(original_simple_data, simple_data);
}

PSL_TEST(Standardization, transform)
{
    set_rand_seed();
    std::vector<double> data = {9., 4.8, 4.4, 7.6, 5.2};
    double calculated_mean = -1.;
    double calculated_std = -1.;

    // compute mean and std
    get_mean_and_std(data, calculated_mean, calculated_std);
    // compare to expectation
    const double expected_mean = 6.2;
    EXPECT_FLOAT_EQ(calculated_mean, expected_mean);
    const double expected_std = 2.;
    EXPECT_FLOAT_EQ(calculated_std, expected_std);

    // transform
    standardize(data, expected_mean, expected_std);
    // compare to expectation
    const std::vector<double> expected_data = {1.4, -.7, -.9, .7, -.5};
    expect_equal_float_vectors(data, expected_data);
}

PSL_TEST(Standardization, zeroStd)
{
    set_rand_seed();
    std::vector<double> data = {3., 3.};
    double calculated_mean = -1.;
    double calculated_std = -1.;

    // compute mean and std
    get_mean_and_std(data, calculated_mean, calculated_std);
    // compare to expectation
    const double expected_mean = 3.;
    EXPECT_FLOAT_EQ(calculated_mean, expected_mean);
    const double expected_std = 1.;
    EXPECT_FLOAT_EQ(calculated_std, expected_std);

    // transform
    standardize(data, expected_mean, expected_std);
    // compare to expectation
    const std::vector<double> expected_data(data.size(), 0.);
    expect_equal_float_vectors(data, expected_data);

    // trail zero
    const double expected_trial_zero = 0.;
    double computed_trial_zero = expected_mean + expected_trial_zero * expected_std;
    standardize(computed_trial_zero, expected_mean, expected_std);
    // compare to expectation
    EXPECT_FLOAT_EQ(expected_trial_zero, computed_trial_zero);

    // trail one
    const double expected_trial_one = -.4;
    double computed_trial_one = expected_mean + expected_trial_one * expected_std;
    standardize(computed_trial_one, expected_mean, expected_std);
    // compare to expectation
    EXPECT_FLOAT_EQ(expected_trial_one, computed_trial_one);
}

}
}
