// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

/* A Vector in three dimensions.
 *
 */
#include "PSL_Point.hpp"

#include <cstddef>
#include <vector>

namespace PlatoSubproblemLibrary
{

class Vector {
public:

    Vector();
    Vector(const std::vector<double>& data);
    Vector(const Point& aPoint);

    ~Vector();

    void set(const std::vector<double>& data);
    void set(size_t index, double value);

    double operator()(size_t index) const;

    double euclideanNorm() const;
    void normalize();

    Vector operator +(const Vector& aVec) const;

    Vector operator -(const Vector& aVec) const;

    Vector& operator =(const Vector& aVec);

private:
    std::vector<double> m_data;
};

Vector operator *(const double aScalar, const Vector& aVec);
Vector operator *(const Vector& aVec, const double aScalar);

Vector cross_product(const Vector& aVec1, const Vector& aVec2);
double dot_product(const Vector& aVec1, const Vector& aVec2);
double angle_between(const Vector& aVec1, const Vector& aVec2);

}
