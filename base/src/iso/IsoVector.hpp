/*
 * IsoVector.hpp
 *
 *  Created on: Oct 24, 2017
 *      Author: bwclark
 */

#ifndef ISOVECTOR_HPP_
#define ISOVECTOR_HPP_

#include <cmath>

class IsoVector
{
private:
    double xVal;  //- x component of vector.
    double yVal;  //- y component of vector.
    double zVal;  //- z component of vector.

public:
    IsoVector();
    virtual ~IsoVector();
    double normalize();
    explicit IsoVector(const double x, const double y, const double z);
    friend IsoVector operator-(const IsoVector &v1,
                               const IsoVector &v2);
    double length() const;
    friend double operator%(const IsoVector &v1, const IsoVector &v2);
    friend IsoVector operator*(const IsoVector &v1, const double sclr);
    friend IsoVector operator*(const double sclr, const IsoVector &v1);
    IsoVector& operator*=(const double scalar);
    friend IsoVector operator+(const IsoVector &v1,
                                 const IsoVector &v2);
    double x() const; //- Return x component of vector
    double y() const; //- Return y component of vector
    double z() const; //- Return z component of vector
    double distance_between_squared(const IsoVector& test_vector) const;
    friend IsoVector operator*(const IsoVector &v1,
                                 const IsoVector &v2);
    IsoVector& operator*=(const IsoVector &vec);


};
inline IsoVector operator*(const IsoVector &vector1,
                      const IsoVector &vector2)
{
  return IsoVector(vector1) *= vector2;
}
inline IsoVector& IsoVector::operator*=(const IsoVector &v)
{
  double xcross, ycross, zcross;
  xcross = yVal * v.zVal - zVal * v.yVal;
  ycross = zVal * v.xVal - xVal * v.zVal;
  zcross = xVal * v.yVal - yVal * v.xVal;
  xVal = xcross;
  yVal = ycross;
  zVal = zcross;
  return *this;
}

inline double IsoVector::x() const
{ return xVal; }
inline double IsoVector::y() const
{ return yVal; }
inline double IsoVector::z() const
{ return zVal; }

inline IsoVector operator-(const IsoVector &vector1,
                      const IsoVector &vector2)
{
  double xv = vector1.xVal - vector2.xVal;
  double yv = vector1.yVal - vector2.yVal;
  double zv = vector1.zVal - vector2.zVal;
  return IsoVector(xv,yv,zv);
}

inline IsoVector::IsoVector(const double xIn,
                                const double yIn,
                                const double zIn)
    : xVal(xIn), yVal(yIn), zVal(zIn)
{}

inline double IsoVector::normalize()
{
  double mag = length();
  if (mag != 0)
  {
    xVal = xVal / mag;
    yVal = yVal / mag;
    zVal = zVal / mag;
  }
  return mag;
}
inline double IsoVector::length() const
{
  return( sqrt(xVal*xVal + yVal*yVal + zVal*zVal) );
}
inline double operator%(const IsoVector &vector1,
                        const IsoVector &vector2)
{
  return( vector1.xVal * vector2.xVal +
          vector1.yVal * vector2.yVal +
          vector1.zVal * vector2.zVal );
}
inline IsoVector operator*(const IsoVector &vector1,
                      const double scalar)
{
  return IsoVector(vector1) *= scalar;
}
inline IsoVector operator*(const double scalar,
                             const IsoVector &vector1)
{
  return IsoVector(vector1) *= scalar;
}
inline IsoVector& IsoVector::operator*=(const double scalar)
{
  xVal *= scalar;
  yVal *= scalar;
  zVal *= scalar;
  return *this;
}
inline IsoVector operator+(const IsoVector &vector1,
                      const IsoVector &vector2)
{
  double xv = vector1.xVal + vector2.xVal;
  double yv = vector1.yVal + vector2.yVal;
  double zv = vector1.zVal + vector2.zVal;
  return IsoVector(xv,yv,zv);
}


#endif /* ISOVECTOR_HPP_ */
