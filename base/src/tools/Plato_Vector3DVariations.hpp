#pragma once

#include <string>

namespace Plato
{

namespace axis3D
{

enum axis3D
{
    x, y, z
};
// enum axis3D

}
// namespace axis3D

/******************************************************************************//**
 * @brief Return gradient free objective function stage name
 * @param [in] aOptimizerNode data structure with optimization related input options
 * @return gradient free objective function stage name
 **********************************************************************************/
struct Vector3D
{
    double mX; /*!< x-direction value */
    double mY; /*!< y-direction value */
    double mZ; /*!< z-direction value */

    /******************************************************************************//**
     * @brief Default constructor
     * @param [in] aX x-direction value
     * @param [in] aY y-direction value
     * @param [in] aZ z-direction value
    **********************************************************************************/
    Vector3D(double aX = 0., double aY = 0., double aZ = 0.) :
            mX(aX),
            mY(aY),
            mZ(aZ)
    {
    }
};
// struct Vector3D

/******************************************************************************//**
 * @brief Return rotation axis
 * @param [in] aStringAxis rotation axis
 * @param [out] aEnumAxis rotation axis
**********************************************************************************/
void axis3D_stringToEnum(const std::string & aStringAxis, Plato::axis3D::axis3D & aEnumAxis);

/******************************************************************************//**
 * @brief Return rotation axis
 * @param [in] aAxisName rotation axis
 * @param [in] aAngleInDegrees angle of rotation in degrees
 * @param [out] aVectorToVary vector to rotate
**********************************************************************************/
void rotate_vector_by_axis(Plato::Vector3D & aVectorToVary,
                           const axis3D::axis3D & aAxisName,
                           const double & aAngleInDegrees);

}
// namespace Plato
