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

// PlatoSubproblemLibraryVersion(3): a stand-alone library for the kernel filter for plato.
#include "PSL_UnitTestingHelper.hpp"

#include "PSL_Vector.hpp"
#include "PSL_Point.hpp"
#include "PSL_FreeHelpers.hpp"
#include "PSL_Random.hpp"

#include <vector>
#include <cmath>

namespace PlatoSubproblemLibrary
{
namespace TestingPoint
{

PSL_TEST(Vector,allocation)
{
    EXPECT_NO_THROW(Vector tVec);

    Vector tVec;
    EXPECT_EQ(tVec(0), 0);
    EXPECT_EQ(tVec(1), 0);
    EXPECT_EQ(tVec(2), 0);

    std::vector<double> tTemp = {0.0,-1,1.0};
    EXPECT_NO_THROW(Vector tVec2(tTemp));

    Vector tVec3(tTemp);
    EXPECT_EQ(tVec3(0), 0);
    EXPECT_EQ(tVec3(1), -1);
    EXPECT_EQ(tVec3(2), 1);

    Point p(0, tTemp);
    EXPECT_NO_THROW(Vector(p));

    Vector tVec4(tTemp);
    EXPECT_EQ(tVec4(0), 0);
    EXPECT_EQ(tVec4(1), -1);
    EXPECT_EQ(tVec4(2), 1);

    std::vector<double> tTemp2 = {0.0, 1.0, 3.0, 2.248};
    EXPECT_THROW(Vector tVec5(tTemp2),std::length_error);
    Point p2(0, tTemp2);
    EXPECT_THROW(Vector tVec6(p2),std::length_error);
}

PSL_TEST(Vector,set_and_get)
{
    Vector tVec;
    tVec.set(0, 3.0);
    tVec.set(1, 3.5);
    tVec.set(2, 3.7);
    EXPECT_EQ(tVec(0), 3.0);
    EXPECT_EQ(tVec(1), 3.5);
    EXPECT_EQ(tVec(2), 3.7);
    EXPECT_EQ(tVec.X(), 3.0);
    EXPECT_EQ(tVec.Y(), 3.5);
    EXPECT_EQ(tVec.Z(), 3.7);

    std::vector<double> tTemp = {4.1,4.2,4.3};
    tVec.set(tTemp);
    EXPECT_EQ(tVec(0), 4.1);
    EXPECT_EQ(tVec(1), 4.2);
    EXPECT_EQ(tVec(2), 4.3);
    EXPECT_EQ(tVec.X(), 4.1);
    EXPECT_EQ(tVec.Y(), 4.2);
    EXPECT_EQ(tVec.Z(), 4.3);

    std::vector<double> tTemp2 = {0,0};
    EXPECT_THROW(tVec.set(tTemp2),std::length_error);

    EXPECT_THROW(tVec.set((size_t) -1, 10),std::length_error);
    EXPECT_THROW(tVec.set(3, 10),std::length_error);

    EXPECT_THROW(tVec((size_t) -1),std::length_error);
    EXPECT_THROW(tVec(3),std::length_error);
}

PSL_TEST(Vector,euclideanNorm)
{
    Vector tVec;
    EXPECT_EQ(tVec.euclideanNorm(),0);

    tVec.set(std::vector<double>({1,0,0}));
    EXPECT_EQ(tVec.euclideanNorm(),1);

    tVec.set(std::vector<double>({1,1,1}));
    EXPECT_EQ(tVec.euclideanNorm(),std::sqrt(3));

    tVec.set(std::vector<double>({1,2,5}));
    EXPECT_EQ(tVec.euclideanNorm(),std::sqrt(30));
}

PSL_TEST(Vector,normalize)
{
    Vector tVec;
    EXPECT_THROW(tVec.normalize(),std::overflow_error);

    tVec.set(std::vector<double>({1.0,0,0}));
    tVec.normalize();
    EXPECT_EQ(tVec(0), 1.0);
    EXPECT_EQ(tVec(1), 0);
    EXPECT_EQ(tVec(2), 0);

    tVec.set(std::vector<double>({1.0,1.0,1.0}));
    tVec.normalize();
    EXPECT_EQ(tVec(0), 1.0/std::sqrt(3));
    EXPECT_EQ(tVec(1), 1.0/std::sqrt(3));
    EXPECT_EQ(tVec(2), 1.0/std::sqrt(3));
    EXPECT_DOUBLE_EQ(tVec.euclideanNorm(),1);

    tVec.set(std::vector<double>({1000,-42.36,.02586}));
    tVec.normalize();
    EXPECT_DOUBLE_EQ(tVec.euclideanNorm(),1);
}

PSL_TEST(Vector,add)
{
    Vector tVec1({1.0,0,2.3});
    Vector tVec2({2.3,1.0,1.6});

    tVec1.add(tVec2);
    EXPECT_DOUBLE_EQ(tVec1(0),3.3);
    EXPECT_DOUBLE_EQ(tVec1(1),1.0);
    EXPECT_DOUBLE_EQ(tVec1(2),3.9);
}

PSL_TEST(Vector,subtract)
{
    Vector tVec1({1.0,0,2.3});
    Vector tVec2({2.3,1.0,1.6});

    tVec1.subtract(tVec2);
    EXPECT_DOUBLE_EQ(tVec1(0),-1.3);
    EXPECT_DOUBLE_EQ(tVec1(1),-1.0);
    EXPECT_DOUBLE_EQ(tVec1(2),0.7);
}

PSL_TEST(Vector,operatorEquals)
{
    Vector tVec1({2.3,1.0,1.6});

    Vector tVec2 = tVec1;
    EXPECT_EQ(tVec2(0),2.3);
    EXPECT_EQ(tVec2(1),1.0);
    EXPECT_EQ(tVec2(2),1.6);
}

PSL_TEST(Vector,multiply)
{
    double tScalar = 2.3;
    Vector tVec2({0,1.0,1.6});

    tVec2.multiply(tScalar);
    EXPECT_DOUBLE_EQ(tVec2(0),0.0);
    EXPECT_DOUBLE_EQ(tVec2(1),2.3);
    EXPECT_DOUBLE_EQ(tVec2(2),3.68);
}

PSL_TEST(Vector,crossProduct)
{
    Vector tVec1({1.0,0.0,0.0});
    Vector tVec2({0.0,1.0,0.0});

    Vector tVec3 = cross_product(tVec1,tVec2);
    EXPECT_EQ(tVec3(0),0.0);
    EXPECT_EQ(tVec3(1),0.0);
    EXPECT_EQ(tVec3(2),1.0);

    Vector tVec4 = cross_product(tVec2,tVec1);
    EXPECT_EQ(tVec4(0),0.0);
    EXPECT_EQ(tVec4(1),0.0);
    EXPECT_EQ(tVec4(2),-1.0);

    Vector tVec5({3.6,2.8,-1.4});
    Vector tVec6({100,-11.2,3.5});

    Vector tVec7 = cross_product(tVec5,tVec6);
    EXPECT_DOUBLE_EQ(tVec7(0),-5.88);
    EXPECT_DOUBLE_EQ(tVec7(1),-152.6);
    EXPECT_DOUBLE_EQ(tVec7(2),-320.32);

    Vector tVec8 = cross_product(tVec6,tVec6);
    EXPECT_NEAR(tVec8(0),0.0,1e-13);
    EXPECT_NEAR(tVec8(1),0.0,1e-13);
    EXPECT_NEAR(tVec8(2),0.0,1e-13);
    // EXPECT_DOUBLE_EQ(tVec8(1),0.0);
    // EXPECT_DOUBLE_EQ(tVec8(2),0.0);
}

PSL_TEST(Vector,dotProduct)
{
    Vector tVec1({1.0,0.0,0.0});
    Vector tVec2({0.0,1.0,0.0});

    double tResult = dot_product(tVec1,tVec2);
    EXPECT_EQ(tResult,0.0);

    tVec2.set({1.0,0.0,0.0});

    tResult = dot_product(tVec1,tVec2);
    EXPECT_EQ(tResult,1.0);

    tVec1.set({3.6,2.8,-1.4});
    tVec2.set({100,-11.2,3.5});
    tResult = dot_product(tVec1,tVec2);
    EXPECT_DOUBLE_EQ(tResult,323.74);
}

PSL_TEST(Vector,angleBetween)
{
    Vector tVec1({1.0,0.0,0.0});
    Vector tVec2({0.0,1.0,0.0});

    double tResult = angle_between(tVec1,tVec2);
    EXPECT_DOUBLE_EQ(tResult,M_PI/2);

    tResult = angle_between(tVec1,tVec1);
    EXPECT_EQ(tResult,0.0);

    tVec2.set({-1.0,0.0,0.0});
    tResult = angle_between(tVec1,tVec2);
    EXPECT_DOUBLE_EQ(tResult,M_PI);

    tVec1.set({3.6,2.8,-1.4});
    tVec2.set({100,-11.2,3.5});
    tResult = angle_between(tVec1,tVec2);
    EXPECT_DOUBLE_EQ(tResult,0.83122639765689199);
}

PSL_TEST(Vector, operatorEqualsEquals)
{
    Vector tVec1({1.0,0.0,0.0});
    Vector tVec2({0.0,1.0,0.0});
    Vector tVec3({1.0,0.0,0.0});

    EXPECT_EQ(tVec1 == tVec1, true);
    EXPECT_EQ(tVec1 == tVec3, true);
    EXPECT_EQ(tVec1 == tVec2, false);
}

PSL_TEST(Vector, getBoundingBox)
{
    Vector tVec1({1.0,2.0,3.0});
    Vector tVec2({-1.0,3.0,-7.0});
    Vector tVec3({1.0,0.0,5.0});

    std::vector<Vector> tVectors;
    tVectors.push_back(tVec1);
    tVectors.push_back(tVec2);
    tVectors.push_back(tVec3);

    Vector tMinCoords;
    Vector tMaxCoords;

    computeBoundingBox(tVectors,tMinCoords,tMaxCoords);

    EXPECT_EQ(tMinCoords(0),-1.0);
    EXPECT_EQ(tMinCoords(1),0.0);
    EXPECT_EQ(tMinCoords(2),-7.0);
    EXPECT_EQ(tMaxCoords(0),1.0);
    EXPECT_EQ(tMaxCoords(1),3.0);
    EXPECT_EQ(tMaxCoords(2),5.0);
}

// PSL_TEST(Vector, performance)
// {
//     // add 1000000000 vectors
    
//     // Vector tVec;
//     double tVec[3] = {0.0,0.0,0.0};

//     for(int i = 0; i < 1000000000; ++i)
//     {
//         double* tTemp = new double[3];
//         tTemp[0] = 1.0;
//         tTemp[1] = 0.0;
//         tTemp[2] = 0.0;
//         tVec[0] += tTemp[0];
//         tVec[1] = tTemp[1];
//         tVec[2] = tTemp[2];
//         delete[] tTemp;
//     }

// //     EXPECT_EQ(tVec(0),1000000000.0);
// //     EXPECT_EQ(tVec(1),0.0);
// //     EXPECT_EQ(tVec(2),0.0);

//     EXPECT_EQ(tVec[0],1000000000.0);
//     EXPECT_EQ(tVec[1],0.0);
//     EXPECT_EQ(tVec[2],0.0);
// }

}
}
