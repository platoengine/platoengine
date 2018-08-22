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
