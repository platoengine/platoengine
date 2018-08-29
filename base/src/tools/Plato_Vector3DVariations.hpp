#pragma once

namespace Plato
{

namespace axis3D
{
enum axis3D
{
    x, y, z
};
}

struct Vector3D
{
    double x;
    double y;
    double z;

    // default constructor
    Vector3D(double a_x = 0., double a_y = 0., double a_z = 0.);
};

void rotate_vector_by_axis(Vector3D& vector_to_vary,
                           const axis3D::axis3D& axis_name,
                           const double& angle_in_degrees);

}
