// PlatoSubproblemLibraryVersion(8): a stand-alone library for the kernel filter for plato.
#include "PSL_Vector.hpp"

#include <cstddef>
#include <vector>
#include <cmath>
#include <cassert>
#include <cstring>
#include <stdexcept>

#define EPSILON 1e-12 

namespace PlatoSubproblemLibrary
{

Vector::Vector() :
        m_data()
{
}

Vector::Vector(const std::vector<double>& data)
{
    if(data.size() != 3)
        throw(std::length_error("PSL vectors must be of length 3"));

    m_data[0] = data[0];
    m_data[1] = data[1];
    m_data[2] = data[2];
}

Vector::Vector(const Point& aPoint)
{
    if(aPoint.dimension() != 3)
        throw(std::length_error("PSL vectors must be of length 3"));

    for(int i = 0; i < 3; ++i)
        m_data[i] = aPoint(i);
}

Vector::~Vector()
{
}

void Vector::set(const std::vector<double>& data)
{
     if(data.size() != 3)
         throw(std::length_error("PSL vectors must be of length 3"));

    m_data[0] = data[0];
    m_data[1] = data[1];
    m_data[2] = data[2];
}

void Vector::set(size_t index, double value)
{
    if(index > 2)
        throw(std::length_error("Index must be between 0 and 2"));
    m_data[index] = value;
}

double Vector::operator()(size_t index) const
{
    if(index > 2)
        throw(std::length_error("Index must be between 0 and 2"));
    return m_data[index];
}


double Vector::euclideanNorm() const
{
    double result = 0;
    for(size_t i = 0u; i < 3; ++i)
    {
        result += (*this)(i)*(*this)(i);
    }
    return std::sqrt(result);
}

void Vector::normalize()
{
    if(this->euclideanNorm() < EPSILON)
        throw(std::overflow_error("The zero vector cannot be normalized"));

    double tNorm = 0;
    for(size_t i = 0u; i < 3; ++i)
    {
        tNorm += (*this)(i)*(*this)(i);
    }

    tNorm = std::sqrt(tNorm);

    for(size_t i = 0; i < 3; ++i)
    {
        m_data[i] = m_data[i]/tNorm;
    }
}

void Vector::add(const Vector& aInputVector)
{
    // add data
    for(size_t i = 0u; i < 3; i++)
    {
        m_data[i] += aInputVector(i);
    }
}

void Vector::subtract(const Vector& aInputVector)
{
    // add data
    for(size_t i = 0u; i < 3; i++)
    {
        m_data[i] -= aInputVector(i);
    }
}

// Vector& Vector::operator =(const Vector& aInputVector)
// {
//     // add data
//     for(size_t i = 0u; i < 3; i++)
//     {
//         this->set(i,aInputVector(i));
//     }

//     return *this;
// }

Vector& Vector::operator =(const Vector& aInputVector)
{
    // add data
    for(size_t i = 0u; i < 3; i++)
    {
        this->set(i,aInputVector(i));
    }

    return *this;
}

bool Vector::operator ==(const Vector& aVec) const
{
    if(aVec(0) == (*this)(0) && aVec(1) == (*this)(1) && aVec(2) == (*this)(2))
        return true;
    else
        return false;
}

void Vector::multiply(const double scalar)
{
    for(size_t i = 0u; i < 3; i++)
    {
        m_data[i] *= scalar;
    }
}

Vector cross_product(const Vector& aVec1, const Vector& aVec2)
{
    std::vector<double> tCrossProduct(3);

    tCrossProduct[0] = aVec1(1)*aVec2(2) - aVec1(2)*aVec2(1);
    tCrossProduct[1] = aVec1(2)*aVec2(0) - aVec1(0)*aVec2(2);
    tCrossProduct[2] = aVec1(0)*aVec2(1) - aVec1(1)*aVec2(0);

    return Vector(tCrossProduct);
}

double dot_product(const Vector& aVec1, const Vector& aVec2)
{
    double tDotProduct = 0;

    for(size_t i = 0; i < 3; ++i)
    {
        tDotProduct += aVec1(i)*aVec2(i);
    }

    return tDotProduct;
}

double angle_between(const Vector& aVec1, const Vector& aVec2)
{
    Vector tVec1Normalized = aVec1;
    Vector tVec2Normalized = aVec2;

    tVec1Normalized.normalize();
    tVec2Normalized.normalize();
     
    double tCosineOfAngle = dot_product(tVec1Normalized,tVec2Normalized);

    return std::acos(tCosineOfAngle);
}

std::ostream & operator<< (std::ostream &out, const Vector &aVec)
{
    out << "{" << aVec.X() << "," << aVec.Y() << "," << aVec.Z() << "}";
    
    return out;
}

void computeBoundingBox(const std::vector<Vector>& aPoints,
                       Vector& aMinCoords,
                       Vector& aMaxCoords)
{
    aMinCoords = Vector(aPoints[0]);
    aMaxCoords = Vector(aPoints[0]);

    for(auto tPoint : aPoints)
    {
        for(int i = 0; i < 3; ++i)
        {
            if(tPoint(i) > aMaxCoords(i))
                aMaxCoords.set(i,tPoint(i));
            else if(tPoint(i) < aMinCoords(i))
                aMinCoords.set(i,tPoint(i));
        }
    }
}

}
