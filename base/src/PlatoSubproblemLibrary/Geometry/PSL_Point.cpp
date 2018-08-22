// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_Point.hpp"

#include <cstddef>
#include <vector>
#include <cmath>
#include <cassert>
#include <cstring>

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

// fill data
void Point::set(size_t index, char* data)
{
    m_index = index;

    // currently assuming dimension
    const size_t dimension = 3u;
    m_data.resize(dimension);
    for(size_t di = 0u; di < dimension; di++)
    {
        char float_array[4] = {data[0 + 4u * di], data[1u + 4u * di], data[2u + 4u * di], data[3u + 4u * di]};
        float working_float = 0.;
        memcpy(&working_float, float_array, sizeof(float));
        m_data[di] = working_float;
    }
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
