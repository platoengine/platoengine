#include "PSL_FreeStandardization.hpp"

#include "PSL_FreeHelpers.hpp"

#include <vector>
#include <cstddef>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <cassert>

namespace PlatoSubproblemLibrary
{

void get_mean_and_std(const std::vector<double>& data, double& mean, double& std)
{
    const size_t length = data.size();

    // calculate mean
    mean = 0;
    for(size_t i = 0u; i < length; i++)
    {
        mean += data[i] / double(length);
    }

    // calculate sample standard deviation
    std = 0;
    for(size_t i = 0u; i < length; i++)
    {
        std += (data[i] - mean) * (data[i] - mean) / (double(length) - 1.);
    }
    std = pow(std, 0.5);

    // avoid later division by zero
    if(std == 0.)
    {
        std = 1.;
    }
}

void get_mean_and_std_of_lowest_fraction(const double& fraction, const std::vector<double>& data, double& mean, double& std)
{
    // determine lengths
    const int length = data.size();
    const int fraction_length = ceil(double(length) * fraction);

    // compute lowest fraction
    std::vector<double> cloned_data = data;
    std::sort(cloned_data.begin(), cloned_data.end());
    cloned_data.resize(fraction_length);

    // compute mean and std
    get_mean_and_std(cloned_data, mean, std);
}

void get_median(std::vector<double>& data_to_sort, double& median)
{
    std::sort(data_to_sort.begin(), data_to_sort.end());
    const size_t length = data_to_sort.size();
    if(length > 0u)
    {
        if(length % 2u == 0u)
        {
            median = .5 * (data_to_sort[(length / 2u) - 1u] + data_to_sort[length / 2u]);
        }
        else
        {
            median = data_to_sort[(length - 1u) / 2u];
        }
    }
}
void get_median_indexes(const int& length, int& index0, int& index1)
{
    if(length % 2 == 0)
    {
        index0 = (length / 2) - 1;
        index1 = index0 + 1;
    }
    else
    {
        index0 = (length - 1) / 2;
        index1 = index0;
    }
}

void get_nonparametric_skew(const std::vector<double>& data, double& skew, double& mean, double& std)
{
    // get mean and standard deviation
    get_mean_and_std(data, mean, std);

    // get median
    std::vector<double> cloned_data = data;
    double median = 0.;
    get_median(cloned_data, median);

    // get skew
    skew = (mean - median) / std;
}

void get_nonparametric_skew(const std::vector<double>& data, double& skew)
{
    double mean = -1;
    double std = -1;
    get_nonparametric_skew(data, skew, mean, std);
}

void solve_nonparametric_skew(const std::vector<double>& data, double& alpha, double& median)
{
    // get median
    std::vector<double> sorted_data = data;
    get_median(sorted_data, median);

    const size_t length = data.size();
    assert(0u < length);
    size_t lower_half_exclusive_upper = 0u;
    size_t upper_half_begin = 0u;
    if(length % 2u == 0u)
    {
        lower_half_exclusive_upper = (length / 2u);
        upper_half_begin = lower_half_exclusive_upper;
    }
    else
    {
        lower_half_exclusive_upper = (length - 1u) / 2u;
        upper_half_begin = lower_half_exclusive_upper + 1u;
    }

    // compute sum terms
    double lower_linear = 0.;
    double lower_log = 0.;
    for(size_t low = 0u; low < lower_half_exclusive_upper; low++)
    {
        lower_linear += sorted_data[low] - median;
        lower_log += log(1. + median - sorted_data[low]);
    }
    double upper_linear = 0.;
    double upper_log = 0.;
    for(size_t upper = upper_half_begin; upper < length; upper++)
    {
        upper_linear += sorted_data[upper] - median;
        upper_log += log(1. + sorted_data[upper] - median);
    }

    // solve quadratic
    double quad_a = -lower_linear - lower_log - upper_linear + upper_log;
    double quad_b = lower_linear + 2. * lower_log + upper_linear;
    double quad_c = -lower_log;
    if(fabs(quad_a) < 1e-10)
    {
        if(fabs(quad_b) < 1e-10)
        {
            // no transform
            alpha = -1.;
        }
        else
        {
            // linear
            alpha = -quad_c / quad_b;
            assert(0. <= alpha);
            assert(alpha <= 1.);
        }
    }
    else
    {
        double alpha_left = -quad_b / (2. * quad_a);
        double alpha_right = sqrt(quad_b * quad_b - 4 * quad_a * quad_c) / (2. * quad_a);

        // alpha is one root in [0,1] range
        double alpha_plus = alpha_left + alpha_right;
        double alpha_minus = alpha_left - alpha_right;
        if(fabs(alpha_plus - .5) < fabs(alpha_minus - .5))
        {
            alpha = alpha_plus;
        }
        else
        {
            alpha = alpha_minus;
        }

        // if this assert fails, data's distribution should be avoided
        assert(0. <= alpha);
        assert(alpha <= 1.);
    }

    // don't allow alpha to be zero or one
    if((fabs(alpha) < 1e-10) || (fabs(alpha - 1.) < 1e-10))
    {
        alpha = -1.;
    }
}

void get_skew_normalize(const std::vector<double>& input, double& alpha, double& median, double& std)
{
    // get alpha, median
    solve_nonparametric_skew(input, alpha, median);

    // fill output
    std::vector<double> output = input;
    skew_normalize(output, alpha, median, 1.);

    // get standard deviation
    double mean = -1.;
    get_mean_and_std(output, mean, std);
}
void skew_normalize(std::vector<double>& data, const double& alpha, const double& median, const double& std)
{
    const size_t length = data.size();
    for(size_t i = 0; i < length; i++)
    {
        skew_normalize(data[i], alpha, median, std);
    }
}
void skew_normalize(double& data, const double& alpha, const double& median, const double& std)
{
    if(alpha < 0.)
    {
        data = (data - median) / std;
    }
    else
    {
        if(data < median)
        {
            data = ((1. - alpha) * (alpha * (data - median) + (alpha - 1.) * log(1. + median - data))) / std;
        }
        else
        {
            data = (alpha * ((1. - alpha) * (data - median) + alpha * log(1. + data - median))) / std;
        }
    }
}

void standardize(std::vector<double>& data, const double& mean, const double& std)
{
    const size_t length = data.size();
    for(size_t i = 0u; i < length; i++)
    {
        standardize(data[i], mean, std);
    }
}

void standardize(double& data, const double& mean, const double& std)
{
    data = (data - mean) / std;
}

void get_bipolar_normalize(const std::vector<double>& input, double& min, double& max)
{
    max = PlatoSubproblemLibrary::max(input);
    min = PlatoSubproblemLibrary::min(input);

    // separate max and min
    if(max - min < 1e-10)
    {
        max++;
        min--;
    }
}
void bipolar_normalize(std::vector<double>& data, const double& min, const double& max)
{
    const size_t length = data.size();
    for(size_t i = 0u; i < length; i++)
    {
        bipolar_normalize(data[i], min, max);
    }
}
void bipolar_normalize(double& data, const double& min, const double& max)
{
    data = (2. * data - max - min) / (max - min);
}

}
