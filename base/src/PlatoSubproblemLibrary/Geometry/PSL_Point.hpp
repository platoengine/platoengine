// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{

class Point {
public:
    Point();
    ~Point();

    // fill data
    Point(size_t index, const std::vector<double>& data);
    void set(size_t index, const std::vector<double>& data);

    void set_index(size_t index);
    size_t get_index() const;

    // get dimension of data
    size_t dimension() const;

    // get a value
    double operator()(size_t index) const;

    // get all values
    void get_data(std::vector<double>& data) const;

    void set(size_t index, double value);

    // get Euclidean distance
    double distance(Point* other) const;

    Point operator +(const Point& other) const;

    Point operator -(const Point& other) const;

    Point& operator =(const Point& other);

private:
    size_t m_index;
    std::vector<double> m_data;
};

Point operator *(const double scalar, const Point& P);
Point operator *(const Point& P, const double scalar);

}
