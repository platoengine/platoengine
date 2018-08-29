#include "Plato_Vector3DVariations.hpp"

#include <cmath>
#include <stdlib.h>
#include <cstdlib>

namespace Plato
{

void axis3D_stringToEnum(const std::string& string_axis, axis3D::axis3D& enum_axis)
{
    if(string_axis == "x")
    {
        enum_axis = axis3D::axis3D::x;
    }
    else if(string_axis == "y")
    {
        enum_axis = axis3D::axis3D::y;
    }
    else if(string_axis == "z")
    {
        enum_axis = axis3D::axis3D::z;
    }
    else
    {
        std::abort();
    }
}

Vector3D::Vector3D(double a_x, double a_y, double a_z) :
        x(a_x),
        y(a_y),
        z(a_z)
{
}

void rotate_vector_by_axis(Vector3D& vector_to_vary, const axis3D::axis3D& axis_name, const double& angle_in_degrees)
{
    // copy input
    double vx = vector_to_vary.x;
    double vy = vector_to_vary.y;
    double vz = vector_to_vary.z;
    // compute cosine/sine
    double cos_angle = cos(angle_in_degrees * M_PI / 180.0);
    double sin_angle = sin(angle_in_degrees * M_PI / 180.0);

    // compute output coordinate locations after rotations
    switch(axis_name)
    {
        case axis3D::axis3D::x:
        {
            vector_to_vary.x = vx;
            vector_to_vary.y = vy * cos_angle - vz * sin_angle;
            vector_to_vary.z = vy * sin_angle + vz * cos_angle;
            break;
        }
        case axis3D::axis3D::y:
        {
            vector_to_vary.x = vx * cos_angle + vz * sin_angle;
            vector_to_vary.y = vy;
            vector_to_vary.z = vz * cos_angle - vx * sin_angle;
            break;
        }
        case axis3D::axis3D::z:
        {
            vector_to_vary.x = vx * cos_angle - vy * sin_angle;
            vector_to_vary.y = vx * sin_angle + vy * cos_angle;
            vector_to_vary.z = vz;
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
