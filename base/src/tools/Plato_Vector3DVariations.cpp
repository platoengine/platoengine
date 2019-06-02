#include "Plato_Vector3DVariations.hpp"

#include <cmath>
#include <stdlib.h>
#include <cstdlib>

namespace Plato
{

void axis3D_stringToEnum(const std::string& aStringAxis, axis3D::axis3D& aEnumAxis)
{
    if(aStringAxis == "x")
    {
        aEnumAxis = axis3D::axis3D::x;
    }
    else if(aStringAxis == "y")
    {
        aEnumAxis = axis3D::axis3D::y;
    }
    else if(aStringAxis == "z")
    {
        aEnumAxis = axis3D::axis3D::z;
    }
    else
    {
        std::abort();
    }
}

void rotate_vector_by_axis(Vector3D& aVectorToVary, const axis3D::axis3D& aAxisName, const double& aAngleInDegrees)
{
    // copy input
    double vx = aVectorToVary.mX;
    double vy = aVectorToVary.mY;
    double vz = aVectorToVary.mZ;
    // compute cosine/sine
    double cos_angle = cos(aAngleInDegrees * M_PI / 180.0);
    double sin_angle = sin(aAngleInDegrees * M_PI / 180.0);

    // compute output coordinate locations after rotations
    switch(aAxisName)
    {
        case axis3D::axis3D::x:
        {
            aVectorToVary.mX = vx;
            aVectorToVary.mY = vy * cos_angle - vz * sin_angle;
            aVectorToVary.mZ = vy * sin_angle + vz * cos_angle;
            break;
        }
        case axis3D::axis3D::y:
        {
            aVectorToVary.mX = vx * cos_angle + vz * sin_angle;
            aVectorToVary.mY = vy;
            aVectorToVary.mZ = vz * cos_angle - vx * sin_angle;
            break;
        }
        case axis3D::axis3D::z:
        {
            aVectorToVary.mX = vx * cos_angle - vy * sin_angle;
            aVectorToVary.mY = vx * sin_angle + vy * cos_angle;
            aVectorToVary.mZ = vz;
            break;
        }
        default:
        {
            std::abort();
            break;
        }
    }
}

}
