/*
 * IsoVector.cpp
 *
 *  Created on: Oct 24, 2017
 *      Author: bwclark
 */

#include "IsoVector.hpp"

IsoVector::IsoVector()
{
    // TODO Auto-generated constructor stub

}

IsoVector::~IsoVector()
{
    // TODO Auto-generated destructor stub
}

double IsoVector::distance_between_squared(const IsoVector& test_vector) const
{
  double _x = xVal - test_vector.xVal;
  double _y = yVal - test_vector.yVal;
  double _z = zVal - test_vector.zVal;

  return(_x*_x + _y*_y + _z*_z);
}
