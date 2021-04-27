// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#pragma once

/* A Vector in three dimensions.
 *
 */
#include "PSL_Point.hpp"

#include <cstddef>
#include <vector>
#include <iostream>

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

    void add(const Vector& aInputVector);
    void subtract(const Vector& aInputVector);
    void multiply(const double scalar);

    Vector& operator =(const Vector& aVec);
    bool operator ==(const Vector& aVec) const;

    friend std::ostream & operator<< (std::ostream &out, const Vector &c);

    double X() const {return m_data[0];}
    double Y() const {return m_data[1];}
    double Z() const {return m_data[2];}

private:
    double m_data[3];
};

Vector cross_product(const Vector& aVec1, const Vector& aVec2);
double dot_product(const Vector& aVec1, const Vector& aVec2);
double angle_between(const Vector& aVec1, const Vector& aVec2);
void computeBoundingBox(const std::vector<Vector>& aPoints,
                        Vector& aMinCoords,
                        Vector& aMaxCoords);

}
