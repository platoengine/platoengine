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
    m_data.resize(3);
}

Vector::Vector(const std::vector<double>& data)
{
    if(data.size() != 3)
        throw(std::length_error("PSL vectors must be of length 3"));

    m_data = data;
}

Vector::Vector(const Point& aPoint)
{
    if(aPoint.dimension() != 3)
        throw(std::length_error("PSL vectors must be of length 3"));

    m_data.resize(3);

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

     m_data = data;
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

Vector Vector::operator +(const Vector& aInputVector) const
{
    // add data
    std::vector<double> combined_data(3);
    for(size_t i = 0u; i < 3; i++)
    {
        combined_data[i] = (*this)(i) + aInputVector(i);
    }

    return Vector(combined_data);
}

Vector Vector::operator -(const Vector& aInputVector) const
{
    // add data
    std::vector<double> combined_data(3);
    for(size_t i = 0u; i < 3; i++)
    {
        combined_data[i] = (*this)(i) - aInputVector(i);
    }

    return Vector(combined_data);
}

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

Vector operator *(const double scalar, const Vector& aInputVector)
{
    std::vector<double> tVec(3);

    for(size_t i = 0u; i < 3; i++)
    {
        tVec[i] = scalar*aInputVector(i);
    }

    return Vector(tVec);
}

Vector operator *(const Vector& aVec, const double scalar)
{
    return scalar * aVec;
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

}