/*
 * Plato_Test_Vector3DVariations.cpp
 *
 *  Created on: Aug 27, 2018
 */

#include "gtest/gtest.h"

#include "Plato_Vector3DVariations.hpp"

#include <string>

namespace Plato
{

TEST(Vector3DVariations,strToEnum)
{
    axis3D::axis3D axis = axis3D::axis3D::x;

    // check mappings
    axis3D_stringToEnum("y", axis);
    EXPECT_EQ(axis, axis3D::axis3D::y);
    axis3D_stringToEnum("z", axis);
    EXPECT_EQ(axis, axis3D::axis3D::z);
    axis3D_stringToEnum("x", axis);
    EXPECT_EQ(axis, axis3D::axis3D::x);
}

TEST(Vector3DVariations,vary)
{
    // pose problem
    Vector3D original_vec = {1., 2., 3.};
    axis3D::axis3D axis_name = axis3D::axis3D::x;
    double angle_to_vary = 5.;
    Vector3D rotated_vec;
    const double tol = 1e-5;

    // X example
    rotated_vec = original_vec;
    rotate_vector_by_axis(rotated_vec, axis_name, angle_to_vary);
    EXPECT_NEAR(rotated_vec.mX, 1., tol);
    EXPECT_NEAR(rotated_vec.mY, 1.730922168, tol);
    EXPECT_NEAR(rotated_vec.mZ, 3.162895580, tol);
    // example
    angle_to_vary = -5.;
    rotated_vec = original_vec;
    rotate_vector_by_axis(rotated_vec, axis_name, angle_to_vary);
    EXPECT_NEAR(rotated_vec.mX, 1., tol);
    EXPECT_NEAR(rotated_vec.mY, 2.253856624, tol);
    EXPECT_NEAR(rotated_vec.mZ, 2.814272608, tol);

    // Y example
    axis_name = axis3D::axis3D::y;
    original_vec = {3., -1., 2.};
    rotated_vec = original_vec;
    angle_to_vary = 10.;
    rotate_vector_by_axis(rotated_vec, axis_name, angle_to_vary);
    EXPECT_NEAR(rotated_vec.mX, 3.301719614, tol);
    EXPECT_NEAR(rotated_vec.mY, -1., tol);
    EXPECT_NEAR(rotated_vec.mZ, 1.448670973, tol);
    // example
    rotated_vec = original_vec;
    angle_to_vary = -10.;
    rotate_vector_by_axis(rotated_vec, axis_name, angle_to_vary);
    EXPECT_NEAR(rotated_vec.mX, 2.607126904, tol);
    EXPECT_NEAR(rotated_vec.mY, -1., tol);
    EXPECT_NEAR(rotated_vec.mZ, 2.490560039, tol);

    // Z example
    axis_name = axis3D::axis3D::z;
    original_vec = {-4., 2., 1.};
    rotated_vec = original_vec;
    angle_to_vary = 8.;
    rotate_vector_by_axis(rotated_vec, axis_name, angle_to_vary);
    EXPECT_NEAR(rotated_vec.mX, -4.239418477, tol);
    EXPECT_NEAR(rotated_vec.mY, 1.423843733, tol);
    EXPECT_NEAR(rotated_vec.mZ, 1., tol);
    // example
    rotated_vec = original_vec;
    angle_to_vary = -8.;
    rotate_vector_by_axis(rotated_vec, axis_name, angle_to_vary);
    EXPECT_NEAR(rotated_vec.mX, -3.682726073, tol);
    EXPECT_NEAR(rotated_vec.mY, 2.537228541, tol);
    EXPECT_NEAR(rotated_vec.mZ, 1., tol);
}

}
