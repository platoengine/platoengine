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
#include "PSL_Point.hpp"

#include <cstddef>
#include <vector>
#include <cmath>
#include <cassert>

namespace PlatoSubproblemLibrary
{

Point::Point() :
        m_index(),
        m_data()
{
}

Point::~Point()
{
}

// fill data
Point::Point(size_t index, const std::vector<double>& data) :
        m_index(index),
        m_data(data)
{
}

// fill data
void Point::set(size_t index, const std::vector<double>& data)
{
    m_index = index;
    m_data.assign(data.begin(),data.end());
}

void Point::set_index(size_t index)
{
    m_index = index;
}
size_t Point::get_index() const
{
    return m_index;
}

// get dimension
size_t Point::dimension() const
{
    return m_data.size();
}

// get a value
double Point::operator()(size_t index) const
{
    return m_data[index];
}

// get all values
void Point::get_data(std::vector<double>& data) const
{
    data.assign(m_data.begin(), m_data.end());
}

void Point::set(size_t index, double value)
{
    m_data[index] = value;
}

// get Euclidean distance
double Point::distance(Point* other) const
{
    double result = 0;
    const size_t dimension = this->dimension();
    assert(dimension == other->dimension());
    for(size_t i = 0u; i < dimension; i++)
    {
        result += ((*other)(i) - (*this)(i)) * ((*other)(i) - (*this)(i));
    }
    return std::sqrt(result);
}

Point Point::operator +(const Point& other) const
{
    // get data
    std::vector<double> other_data;
    other.get_data(other_data);
    std::vector<double> this_data;
    this->get_data(this_data);

    // add data
    const size_t this_dimension = this->dimension();
    assert(this_dimension == other.dimension());
    std::vector<double> combined_data(this_dimension, 0.);
    for(size_t i = 0u; i < this_dimension; i++)
    {
        combined_data[i] = other_data[i] + this_data[i];
    }

    return Point(0u, combined_data);
}

Point Point::operator -(const Point& other) const
{
    // get data
    std::vector<double> other_data;
    other.get_data(other_data);
    std::vector<double> this_data;
    this->get_data(this_data);

    // add data
    const size_t this_dimension = this->dimension();
    assert(this_dimension == other.dimension());
    std::vector<double> combined_data(this_dimension, 0.);
    for(size_t i = 0u; i < this_dimension; i++)
    {
        combined_data[i] = this_data[i] - other_data[i];
    }

    return Point(0u, combined_data);
}

Point& Point::operator =(const Point& other)
{
    std::vector<double> data;
    other.get_data(data);
    this->set(other.get_index(), data);

    return *this;
}

Point operator *(const double scalar, const Point& P)
{
    // get data
    std::vector<double> data;
    P.get_data(data);

    // scale data
    const size_t dimension = P.dimension();
    for(size_t i = 0u; i < dimension; i++)
    {
        data[i] *= scalar;
    }

    return Point(P.get_index(), data);
}

Point operator *(const Point& P, const double scalar)
{
    return scalar * P;
}

}
