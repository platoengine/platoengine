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

// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_FreeHelpers.hpp"

#include "PSL_Random.hpp"
#include "PSL_Point.hpp"

#include <stdlib.h>
#include <sys/time.h>
#include <cstdlib>
#include <ctime>
#include <cstddef>
#include <string>
#include <cmath>
#include <math.h>
#include <cassert>
#include <iostream>

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

void heaviside_apply(const double& beta,
                     const std::vector<double>& threshold,
                     const std::vector<double>& input,
                     std::vector<double>& output)
{
    const size_t dimension = input.size();
    assert(threshold.size() == dimension);

    // do compute
    output.resize(dimension);
    for(size_t i = 0u; i < dimension; i++)
    {
        output[i] = heaviside_apply(beta, threshold[i], input[i]);
    }
}
void heaviside_gradient(const double& beta,
                        const std::vector<double>& threshold,
                        const std::vector<double>& input,
                        std::vector<double>& output_gradient)
{
    const size_t dimension = input.size();
    assert(threshold.size() == dimension);

    // do compute
    output_gradient.resize(dimension);
    for(size_t i = 0u; i < dimension; i++)
    {
        output_gradient[i] = heaviside_gradient(beta, threshold[i], input[i]);
    }
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

}
