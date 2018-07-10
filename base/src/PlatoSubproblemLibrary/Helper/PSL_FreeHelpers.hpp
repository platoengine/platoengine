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

#pragma once

#include <vector>
#include <cstddef>
#include <map>
#include <algorithm>
#include <cassert>
#include <functional>
#include <cmath>
#include <math.h>
#include <string>
#include <set>

namespace PlatoSubproblemLibrary
{
class Point;

double fractional_spherical_sector(const double& radius_fraction,
                                   const double& angle_fraction,
                                   const double& outside_angle);

double compute_overhang_angle(Point* center, Point* other, Point* normalized_build_direction);

void heaviside_apply(const double& beta,
                     const std::vector<double>& threshold,
                     const std::vector<double>& input,
                     std::vector<double>& output);
void heaviside_gradient(const double& beta,
                        const std::vector<double>& threshold,
                        const std::vector<double>& input,
                        std::vector<double>& output_gradient);
double heaviside_apply(const double& beta, const double& threshold, const double& input);
double heaviside_gradient(const double& beta, const double& threshold, const double& input);

template<typename t>
void fill_with_index(std::vector<t>& input)
{
    const size_t length = input.size();
    for(size_t i = 0; i < length; i++)
    {
        input[i] = i;
    }
}

void random_orthogonal_vector(const std::vector<double>& input, std::vector<double>& output);

void solve_for_hyperplane_separators(const std::vector<double>& point_0,
                                     const std::vector<double>& point_1,
                                     std::vector<std::vector<double> >& hyperplanes_slope_and_offset);

void solve_for_bisector(const std::vector<double>& point_0,
                        const std::vector<double>& point_1,
                        std::vector<double>& hyperplane_slope_and_offset);

double point_to_plane_distance(const std::vector<double>& point, const std::vector<double>& hyperplane_slope_and_offset);

bool get_hyperplane_sign(const std::vector<double>& point,
                         const std::vector<double>& hyperplane_slope_and_offset);

template<typename t>
void binary_bool_vector_to_integer(const std::vector<bool>& input, t& output)
{
    output = 0;
    const size_t length = input.size();
    t pow_of_2 = 1;
    for(size_t i = 0; i < length; i++)
    {
        output += pow_of_2 * input[i];
        pow_of_2 *= 2;
    }
}

bool is_prime(const int& number);

template<typename T1, typename T2>
void transfer_vector_types(std::vector<T1>& destination, const std::vector<T2>& source)
{
    const size_t length = source.size();
    destination.resize(length);
    for(size_t i = 0; i < length; i++)
    {
        destination[i] = T1(source[i]);
    }
}

template<typename T1, typename T2, typename T3>
void axpy(const T1& alpha, const std::vector<T2>& x, std::vector<T3>& y)
{
    const size_t y_size = y.size();
    for(size_t i = 0u; i < y_size; i++)
    {
        y[i] += T3(alpha) * T3(x[i]);
    }
}

template<typename T>
bool is_subset(const std::set<T>& left, const std::set<T>& right)
{
    // true if and only if (left is subset of right).
    return std::includes(right.begin(), right.end(), left.begin(), left.end());
}

template<typename T>
T partial_factorial(const T& inclusive_lower, const T& inclusive_upper)
{
    T result = T(1);
    for(T i = std::max(inclusive_lower, T(1)); i <= inclusive_upper; i++)
    {
        result *= i;
    }
    return result;
}
template<typename T>
T factorial(const T& in_)
{
    return partial_factorial(T(0), in_);
}

template<typename T>
T choose(const T& n_, const T& k_)
{
    // unordered samples without replacement
    // a.k.a. binomial coefficient, n choose k, etc.

    // if nothing to choose
    if(k_ > n_)
    {
        return T(0);
    }

    // something to choose
    const T nmk = (n_ >= k_ ? n_ - k_ : 0);
    if(nmk > k_)
    {
        return partial_factorial(nmk + T(1), n_) / factorial(k_);
    }
    return partial_factorial(k_ + T(1), n_) / factorial(nmk);
}

template<typename T>
T multichoose(const T& n_, const T& k_)
{
    // unordered samples with replacement
    // note: significantly less common

    const T nkm1 = (n_ + k_ >= 1 ? n_ + k_ - 1 : 0);
    return choose(nkm1, k_);
}

template<typename T>
void sort_descending(std::vector<T>& v_)
{
    std::sort(v_.begin(), v_.end(), std::greater<T>());
}

template<typename T>
void safe_free(T*& ptr)
{
    if(ptr)
    {
        delete ptr;
        ptr = NULL;
    }
}

template<typename T>
void safe_free(std::vector<T*>& vec_ptr)
{
    const size_t length = vec_ptr.size();
    for(size_t i = 0; i < length; i++)
    {
        safe_free(vec_ptr[i]);
    }
}

template<typename T>
void cumulative_sum(const std::vector<T>& in_, std::vector<T>& out_)
{
    T previous = in_[0];
    out_[0] = previous;
    const size_t length = in_.size();
    assert(length > 0u);
    for(size_t i = 1u; i < length; i++)
    {
        out_[i] = previous + in_[i];
        previous = out_[i];
    }
}
template<typename T>
void cumulative_sum(std::vector<T>& v_)
{
    T previous = v_[0];
    const size_t length = v_.size();
    assert(length > 0u);
    for(size_t i = 1u; i < length; i++)
    {
        v_[i] += previous;
        previous = v_[i];
    }
}

template<typename T>
T sum(const std::vector<T>& in_)
{
    T result = T(0.);
    const size_t length = in_.size();
    for(size_t i = 0u; i < length; i++)
    {
        result += in_[i];
    }
    return result;
}

template<typename T>
T mean(const std::vector<T>& in_)
{
    T result = T(0.);
    const size_t length = in_.size();
    if(length != 0u)
    {
        for(size_t i = 0u; i < length; i++)
        {
            result += in_[i];
        }
        result /= T(length);
    }
    return result;
}

template<typename T>
double log_product(const std::vector<T>& in_)
{
    double result = 0.;
    const size_t length = in_.size();
    for(size_t i = 0u; i < length; i++)
    {
        result += log(double(in_[i]));
    }
    return result;
}

template<typename T>
T product(const std::vector<T>& in_)
{
    T result = T(1.);
    const size_t length = in_.size();
    for(size_t i = 0u; i < length; i++)
    {
        result *= in_[i];
    }
    return result;
}

template<typename T>
int count(const std::vector<T>& in_)
{
    int result = 0;
    const size_t length = in_.size();
    for(size_t i = 0u; i < length; i++)
    {
        if(in_[i])
        {
            result++;
        }
    }
    return result;
}

template<typename T>
int count(const std::vector<std::vector<T> >& in_)
{
    int result = 0;
    const size_t length = in_.size();
    for(size_t i = 0u; i < length; i++)
    {
        result += count(in_[i]);
    }
    return result;
}

template<typename T>
T max(const std::vector<T>& in_)
{
    T result = in_[0];
    const size_t length = in_.size();
    assert(length > 0u);
    for(size_t i = 1u; i < length; i++)
    {
        result = std::max(result, in_[i]);
    }
    return result;
}

template<typename T>
T min(const std::vector<T>& in_)
{
    T result = in_[0];
    const size_t length = in_.size();
    assert(length > 0u);
    for(size_t i = 1u; i < length; i++)
    {
        result = std::min(result, in_[i]);
    }
    return result;
}

template<typename T>
void invert_vector_to_map(const std::vector<T>& in_, std::map<T,int>& out_)
{
    const int num_in = in_.size();
    for(int in_index = 0; in_index < num_in; in_index++)
    {
        out_[in_[in_index]] = in_index;
    }
}

double getTimeInSeconds();

std::string remove_a_filename_prefix(const std::string& full_filename);
std::string remove_all_filename_prefix(const std::string& full_filename);

}
