// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_FreeHelpers.hpp"

#include "PSL_Random.hpp"
#include "PSL_Point.hpp"

#include <sys/time.h>
#include <cstdlib>
#include <ctime>
#include <cstddef>
#include <string>
#include <cmath>
#include <cassert>
#include <iostream>
#include <stdexcept>

namespace PlatoSubproblemLibrary
{

double fractional_spherical_sector(const double& radius_fraction,
                                   const double& angle_fraction,
                                   const double& outside_angle)
{
    const double deg_to_rad = 180.0 / M_PI;
    const double one_minus_cosine = 1. - cos(deg_to_rad * outside_angle);
    const double one_minus_scaled_cosine = 1. - cos(deg_to_rad * outside_angle * angle_fraction);
    const double one_minus_radius_cubed = 1. - radius_fraction * radius_fraction * radius_fraction;

    const double fraction = (one_minus_radius_cubed * one_minus_cosine - one_minus_radius_cubed * one_minus_scaled_cosine)
                            / one_minus_cosine;
    return fraction;
}

double compute_overhang_angle(Point* center, Point* other, Point* normalized_build_direction)
{
    // NOTE: build direction must be normalized!

    // get size
    const size_t length = center->dimension();
    assert(other->dimension() == length);
    assert(normalized_build_direction->dimension() == length);

    double numerator = 0.;
    double query_norm = 0.;
    for(size_t i = 0u; i < length; i++)
    {
        const double build_value = (*normalized_build_direction)(i);
        const double query_value = (*center)(i) - (*other)(i);

        numerator += query_value * build_value;
        query_norm += query_value * query_value;
    }
    if(query_norm < 1e-10) {
        return 180.;
    }
    const double cos_of_angle = numerator / sqrt(query_norm);
    const double angle = acos(cos_of_angle) * 180.0 / M_PI;
    return angle;
}

double heaviside_apply(const double& beta, const double& input)
{
    const double exp_term = -std::exp(-beta * input);
    const double linear_term = input * std::exp(-beta);
    return 1. + exp_term + linear_term;
}
double heaviside_gradient(const double& beta, const double& input)
{
    const double exp_term = beta * std::exp(-beta * input);
    const double linear_term = std::exp(-beta);
    return exp_term + linear_term;
}
double heaviside_apply(const double& beta, const double& threshold, const double& input)
{
    const double exp_quantity = exp(-2.0 * beta * (input - threshold));
    return 1. / (1. + exp_quantity);
}
double heaviside_gradient(const double& beta, const double& threshold, const double& input)
{
    const double exp_quantity = exp(-2.0 * beta * (input - threshold));
    return (2.0 * beta * exp_quantity) / (pow(1. + exp_quantity, 2.0));
}

double tanh_apply(const double& aBeta, const double& aInput)
{
  double tEta = 0.5;
  double tNumerator = std::tanh(aBeta*tEta) + tanh(aBeta*(aInput - tEta));
  double tDenominator = std::tanh(aBeta*tEta) + tanh(aBeta*(1.0-tEta));
  return tNumerator/tDenominator;
}

double tanh_gradient(const double& aBeta, const double& aInput)
{
  double tEta = 0.5;
  double tNumerator = aBeta*(1-std::tanh(aBeta*(aInput - tEta))*std::tanh(aBeta*(aInput - tEta)));
  double tDenominator = std::tanh(aBeta*tEta) + tanh(aBeta*(1.0-tEta));
  return tNumerator/tDenominator;
}

void random_orthogonal_vector(const std::vector<double>& input, std::vector<double>& output)
{
    const size_t dimension = input.size();

    // fill with random
    output.resize(dimension);
    uniform_rand_double(0., 1., output);

    // choose index to solve for
    std::vector<int> index_permutation;
    random_permutation(dimension, index_permutation);
    for(size_t abstract_index = 0u; abstract_index < dimension; abstract_index++)
    {
        const int solve_index = index_permutation[abstract_index];

        // prepare to divide later
        if(fabs(input[solve_index]) < 1e-10)
        {
            continue;
        }

        // solve for output
        double inp = 0.;
        for(size_t i = 0u; i < dimension; i++)
        {
            inp += input[i] * output[i];
        }
        inp -= input[solve_index] * output[solve_index];
        output[solve_index] = -inp / input[solve_index];

        // completed
        break;
    }
}

void solve_for_hyperplane_separators(const std::vector<double>& point_0,
                                            const std::vector<double>& point_1,
                                            std::vector<std::vector<double> >& hyperplanes_slope_and_offset)
{
    // the hyperplane is w'*x+c=0
    const size_t dimension = point_0.size();
    assert(dimension == point_1.size());
    const size_t num_hyperplanes = hyperplanes_slope_and_offset.size();
    assert(0 < num_hyperplanes);

    // compute point difference
    std::vector<double> working_difference = point_1;
    axpy(-1., point_0, working_difference);
    working_difference.push_back(0.);

    // for each hyperplane
    for(size_t p = 0u; p < num_hyperplanes; p++)
    {
        const double fraction_0 = double(p + 1) / double(num_hyperplanes + 1);
        const double fraction_1 = double(num_hyperplanes - p) / double(num_hyperplanes + 1);

        // choose slope randomly
        hyperplanes_slope_and_offset[p] = working_difference;

        // choose offset
        hyperplanes_slope_and_offset[p][dimension] = 0.;
        for(size_t i = 0u; i < dimension; i++)
        {
            hyperplanes_slope_and_offset[p][dimension] += hyperplanes_slope_and_offset[p][i]
                                                          * (fraction_0 * point_0[i] + fraction_1 * point_1[i]);
        }
        hyperplanes_slope_and_offset[p][dimension] *= -1.;
    }
}

void solve_for_bisector(const std::vector<double>& point_0,
                               const std::vector<double>& point_1,
                               std::vector<double>& hyperplane_slope_and_offset)
{
    // solves for w'*x+c=0 where w is hyperplane_slope, x is any point
    //  on the hyperplane, and c is the hyperplane_offset
    const size_t dimension = point_0.size();
    assert(dimension == point_1.size());

    // compute point difference
    hyperplane_slope_and_offset = point_1;
    axpy(-1., point_0, hyperplane_slope_and_offset);
    hyperplane_slope_and_offset.push_back(0.);

    // choose offset
    hyperplane_slope_and_offset[dimension] = 0.;
    for(size_t i = 0u; i < dimension; i++)
    {
        hyperplane_slope_and_offset[dimension] += hyperplane_slope_and_offset[i] * (point_0[i] + point_1[i]);
    }
    hyperplane_slope_and_offset[dimension] *= -.5;
}

double point_to_plane_distance(const std::vector<double>& point, const std::vector<double>& hyperplane_slope_and_offset)
{
    // the hyperplane is w'*x+c=0
    const size_t dimension = point.size();
    assert(hyperplane_slope_and_offset.size() == dimension+1u);

    // result =  |w'*p+c|/||w||
    double numerator = hyperplane_slope_and_offset[dimension];
    double denominator = 0.;
    for(size_t i = 0; i < dimension; i++)
    {
        numerator += hyperplane_slope_and_offset[i] * point[i];
        denominator += hyperplane_slope_and_offset[i] * hyperplane_slope_and_offset[i];
    }

    // finalize calculation
    return std::fabs(numerator) / std::sqrt(denominator);
}

bool get_hyperplane_sign(const std::vector<double>& point,
                         const std::vector<double>& hyperplane_slope_and_offset)
{
    // the hyperplane is w'*x+c=0
    const size_t dimension = point.size();
    assert(hyperplane_slope_and_offset.size() == dimension+1u);

    double value = hyperplane_slope_and_offset[dimension];
    for(size_t i = 0; i < dimension; i++)
    {
        value += hyperplane_slope_and_offset[i] * point[i];
    }
    return (0. <= value);
}

bool is_prime(const int& number)
{
    // 1 and non-positives are composite
    if(number <= 1)
    {
        return false;
    }

    // for each candidate divisor
    const int sqrt_n = floor(sqrt(double(number)));
    for(int d = 2; d <= sqrt_n; d++)
    {
        if(number % d == 0)
        {
            return false;
        }
    }
    return true;
}

double getTimeInSeconds()
{
   timeval now;
   gettimeofday(&now, NULL);
   return double(now.tv_sec) + double(now.tv_usec)/1.0e6;
}

std::string remove_a_filename_prefix(const std::string& full_filename)
{
    size_t loc = full_filename.find_first_of('/');
    if(loc == std::string::npos)
    {
        return full_filename;
    }
    if(full_filename[loc] == '/')
    {
        return full_filename.substr(loc + 1u);
    }
    return full_filename;
}

std::string remove_all_filename_prefix(const std::string& full_filename)
{
    size_t loc = full_filename.find_last_of('/');
    if(loc == std::string::npos)
    {
        return full_filename;
    }
    if(full_filename[loc] == '/')
    {
        return full_filename.substr(loc + 1u);
    }
    return full_filename;
}

double determinant3X3(const Vector& aRow1,
                      const Vector& aRow2,
                      const Vector& aRow3)
{
    // rows may actually be columns instead because the
    // determinant of a matrix is equal to the determinant
    // of its transpose
    
    double tTerm1 = aRow1(0)*(aRow2(1)*aRow3(2) - aRow2(2)*aRow3(1));
    double tTerm2 = aRow1(1)*(aRow2(0)*aRow3(2) - aRow2(2)*aRow3(0));
    double tTerm3 = aRow1(2)*(aRow2(0)*aRow3(1) - aRow2(1)*aRow3(0));

    return tTerm1 - tTerm2 + tTerm3;
}

double linearInterpolation(const double& a, const double& b, const double &t)
{
    if(t < 0.0 || t > 1.0)
        throw(std::domain_error("FreeHelpers::interpolate1D: t must be between 0 and 1"));

    return (1-t)*a + t*b;
}

}
