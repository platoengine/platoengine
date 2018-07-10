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

#include "math_library.hpp"
#include <math.h>

using namespace MLib;

/******************************************************************************/
Real Trace( const SymTensor& t )
/******************************************************************************/
{
  return t(SYM::XX)+t(SYM::YY)+t(SYM::ZZ);
}

/******************************************************************************/
SymTensor Deviator( const SymTensor& t )
/******************************************************************************/
{
  SymTensor newst(t);
  Real mean = Trace(t)/3.0;
  newst.data_[SYM::XX] -= mean;
  newst.data_[SYM::YY] -= mean;
  newst.data_[SYM::ZZ] -= mean;
  return newst;
}

/******************************************************************************/
SymTensor::SymTensor(Real xx, Real yy, Real zz, Real yz, Real xz, Real xy)
/******************************************************************************/
{
  data_[SYM::XX] = xx;
  data_[SYM::YY] = yy;
  data_[SYM::ZZ] = zz;
  data_[SYM::YZ] = yz;
  data_[SYM::XZ] = xz;
  data_[SYM::XY] = xy;
}

/******************************************************************************/
SymTensor::SymTensor(Real* initdata)
/******************************************************************************/
{
  data_[SYM::XX] = initdata[SYM::XX];
  data_[SYM::YY] = initdata[SYM::YY];
  data_[SYM::ZZ] = initdata[SYM::ZZ];
  data_[SYM::YZ] = initdata[SYM::YZ];
  data_[SYM::XZ] = initdata[SYM::XZ];
  data_[SYM::XY] = initdata[SYM::XY];
}

/******************************************************************************/
SymTensor::SymTensor()
/******************************************************************************/
{
  data_[SYM::XX] = 0.0;
  data_[SYM::YY] = 0.0;
  data_[SYM::ZZ] = 0.0;
  data_[SYM::YZ] = 0.0;
  data_[SYM::XZ] = 0.0;
  data_[SYM::XY] = 0.0;
}

/******************************************************************************/
Real& Tensor::operator[]( int index )
/******************************************************************************/
{
  return data_[index];
}

/******************************************************************************/
Real& SymTensor::operator[]( int index )
/******************************************************************************/
{
  return data_[index];
}

/******************************************************************************/
Real Tensor::operator()( int index ) const
/******************************************************************************/
{
  return data_[index];
}
/******************************************************************************/
Real SymTensor::operator()( int index ) const
/******************************************************************************/
{
  return data_[index];
}
/******************************************************************************/
SymTensor& SymTensor::operator-=( const SymTensor& T )
/******************************************************************************/
{
  data_[SYM::XX] -= T(SYM::XX);
  data_[SYM::YY] -= T(SYM::YY);
  data_[SYM::ZZ] -= T(SYM::ZZ);
  data_[SYM::YZ] -= T(SYM::YZ);
  data_[SYM::XZ] -= T(SYM::XZ);
  data_[SYM::XY] -= T(SYM::XY);
  return *this;
}
/******************************************************************************/
SymTensor& SymTensor::operator+=( const SymTensor& T )
/******************************************************************************/
{
  data_[SYM::XX] += T(SYM::XX);
  data_[SYM::YY] += T(SYM::YY);
  data_[SYM::ZZ] += T(SYM::ZZ);
  data_[SYM::YZ] += T(SYM::YZ);
  data_[SYM::XZ] += T(SYM::XZ);
  data_[SYM::XY] += T(SYM::XY);
  return *this;
}
/******************************************************************************/
SymTensor& SymTensor::operator/=( Real f )
/******************************************************************************/
{
  data_[SYM::XX] /= f;
  data_[SYM::YY] /= f;
  data_[SYM::ZZ] /= f;
  data_[SYM::YZ] /= f;
  data_[SYM::XZ] /= f;
  data_[SYM::XY] /= f;
  return *this;
}
/******************************************************************************/
void Tensor::operator=( const Real& T )
/******************************************************************************/
{
  data_[TENSOR::XX] = T;
  data_[TENSOR::YY] = T;
  data_[TENSOR::ZZ] = T;
  data_[TENSOR::YZ] = T;
  data_[TENSOR::XZ] = T;
  data_[TENSOR::XY] = T;
  data_[TENSOR::ZY] = T;
  data_[TENSOR::ZX] = T;
  data_[TENSOR::YX] = T;
}
/******************************************************************************/
void Tensor::operator=( const Tensor& T )
/******************************************************************************/
{
  data_[TENSOR::XX] = T(TENSOR::XX);
  data_[TENSOR::YY] = T(TENSOR::YY);
  data_[TENSOR::ZZ] = T(TENSOR::ZZ);
  data_[TENSOR::YZ] = T(TENSOR::YZ);
  data_[TENSOR::XZ] = T(TENSOR::XZ);
  data_[TENSOR::XY] = T(TENSOR::XY);
  data_[TENSOR::ZY] = T(TENSOR::ZY);
  data_[TENSOR::ZX] = T(TENSOR::ZX);
  data_[TENSOR::YX] = T(TENSOR::YX);
}
/******************************************************************************/
void Tensor::operator=( const SymTensor& T )
/******************************************************************************/
{
  data_[TENSOR::XX] = T(SYM::XX);
  data_[TENSOR::YY] = T(SYM::YY);
  data_[TENSOR::ZZ] = T(SYM::ZZ);
  data_[TENSOR::YZ] = T(SYM::YZ);
  data_[TENSOR::XZ] = T(SYM::XZ);
  data_[TENSOR::XY] = T(SYM::XY);
  data_[TENSOR::ZY] = T(SYM::YZ);
  data_[TENSOR::ZX] = T(SYM::XZ);
  data_[TENSOR::YX] = T(SYM::XY);
}

/******************************************************************************/
void SymTensor::operator=( const Real& T )
/******************************************************************************/
{
  data_[SYM::XX] = T;
  data_[SYM::YY] = T;
  data_[SYM::ZZ] = T;
  data_[SYM::YZ] = T;
  data_[SYM::XZ] = T;
  data_[SYM::XY] = T;
}

/******************************************************************************/
void SymTensor::operator=( const SymTensor& T )
/******************************************************************************/
{
  data_[SYM::XX] = T(SYM::XX);
  data_[SYM::YY] = T(SYM::YY);
  data_[SYM::ZZ] = T(SYM::ZZ);
  data_[SYM::YZ] = T(SYM::YZ);
  data_[SYM::XZ] = T(SYM::XZ);
  data_[SYM::XY] = T(SYM::XY);
}

/******************************************************************************/
void SymTensor::operator=( const Vector& v )
/******************************************************************************/
{
  data_[SYM::XX] = v(VEC::X);
  data_[SYM::YY] = 0.0;
  data_[SYM::ZZ] = 0.0;
  data_[SYM::YZ] = 0.0;
  data_[SYM::XZ] = v(VEC::Z);
  data_[SYM::XY] = v(VEC::Y);
}
/******************************************************************************/
SymTensor operator/( Real fac, const SymTensor& st )
/******************************************************************************/
{
  SymTensor newst(st);
  newst[SYM::XX]/=fac;
  newst[SYM::YY]/=fac;
  newst[SYM::ZZ]/=fac;
  newst[SYM::YZ]/=fac;
  newst[SYM::XZ]/=fac;
  newst[SYM::XY]/=fac;
  return newst;
}
/******************************************************************************/
SymTensor operator/( const SymTensor& st, Real fac )
/******************************************************************************/
{
  SymTensor newst(st);
  newst[SYM::XX]/=fac;
  newst[SYM::YY]/=fac;
  newst[SYM::ZZ]/=fac;
  newst[SYM::YZ]/=fac;
  newst[SYM::XZ]/=fac;
  newst[SYM::XY]/=fac;
  return newst;
}
/******************************************************************************/
SymTensor operator*( Real fac, const SymTensor& st )
/******************************************************************************/
{
  SymTensor newst(st);
  newst[SYM::XX]*=fac;
  newst[SYM::YY]*=fac;
  newst[SYM::ZZ]*=fac;
  newst[SYM::YZ]*=fac;
  newst[SYM::XZ]*=fac;
  newst[SYM::XY]*=fac;
  return newst;
}
/******************************************************************************/
SymTensor operator*( const SymTensor& st, Real fac )
/******************************************************************************/
{
  SymTensor newst(st);
  newst[SYM::XX]*=fac;
  newst[SYM::YY]*=fac;
  newst[SYM::ZZ]*=fac;
  newst[SYM::YZ]*=fac;
  newst[SYM::XZ]*=fac;
  newst[SYM::XY]*=fac;
  return newst;
}

/******************************************************************************/
Tensor operator*( const Tensor& a, const Tensor& b )
/******************************************************************************/
{
  Tensor ab;
  using namespace TENSOR;
  
  ab.data_[XX] = a.data_[XX]*b.data_[XX]+a.data_[XY]*b.data_[YX]+a.data_[XZ]*b.data_[ZX];
  ab.data_[XY] = a.data_[XX]*b.data_[XY]+a.data_[XY]*b.data_[YY]+a.data_[XZ]*b.data_[ZY];
  ab.data_[XZ] = a.data_[XX]*b.data_[XZ]+a.data_[XY]*b.data_[YZ]+a.data_[XZ]*b.data_[ZZ];
  ab.data_[YX] = a.data_[YX]*b.data_[XX]+a.data_[YY]*b.data_[YX]+a.data_[YZ]*b.data_[ZX];
  ab.data_[YY] = a.data_[YX]*b.data_[XY]+a.data_[YY]*b.data_[YY]+a.data_[YZ]*b.data_[ZY];
  ab.data_[YZ] = a.data_[YX]*b.data_[XZ]+a.data_[YY]*b.data_[YZ]+a.data_[YZ]*b.data_[ZZ];
  ab.data_[ZX] = a.data_[ZX]*b.data_[XX]+a.data_[ZY]*b.data_[YX]+a.data_[ZZ]*b.data_[ZX];
  ab.data_[ZY] = a.data_[ZX]*b.data_[XY]+a.data_[ZY]*b.data_[YY]+a.data_[ZZ]*b.data_[ZY];
  ab.data_[ZZ] = a.data_[ZX]*b.data_[XZ]+a.data_[ZY]*b.data_[YZ]+a.data_[ZZ]*b.data_[ZZ];
  return ab;
}

/******************************************************************************/
SymTensor operator*( const Intrepid::FieldContainer<double>& C, const SymTensor& s )
/******************************************************************************/
{
  SymTensor S;
  for(int i=0; i<6; i++){
    for(int j=0; j<3; j++)
      S.data_[i] += C(i,j)*s.data_[j];
    for(int j=3; j<6; j++)
      S.data_[i] += 2.0*C(i,j)*s.data_[j];
  }
  return S;
}

/******************************************************************************/
SymTensor operator*( const Intrepid::FieldContainer<double>& C, const Real& s )
/******************************************************************************/
{
  SymTensor S;
  for(int i=0; i<6; i++)
      S.data_[i] = C(i)*s;
  return S;
}

/******************************************************************************/
Real& Vector::operator[]( int index )
/******************************************************************************/
{
  return data_[index];
}

/******************************************************************************/
Real Vector::operator()( int index ) const
/******************************************************************************/
{
  return data_[index];
}

/******************************************************************************/
void Vector::operator=( const Real& v )
/******************************************************************************/
{
  data_[VEC::X] = v;
  data_[VEC::Y] = v;
  data_[VEC::Z] = v;
}

/******************************************************************************/
void Vector::operator=( const Vector& v )
/******************************************************************************/
{
  data_[VEC::X] = v(VEC::X);
  data_[VEC::Y] = v(VEC::Y);
  data_[VEC::Z] = v(VEC::Z);
}


/******************************************************************************/
Vector::Vector(Real x, Real y, Real z)
/******************************************************************************/
{
  data_[VEC::X] = x;
  data_[VEC::Y] = y;
  data_[VEC::Z] = z;
}

/******************************************************************************/
Vector::Vector(Real* initdata)
/******************************************************************************/
{
  data_[VEC::X] = initdata[VEC::X];
  data_[VEC::Y] = initdata[VEC::Y];
  data_[VEC::Z] = initdata[VEC::Z];
}

/******************************************************************************/
Vector::Vector()
/******************************************************************************/
{
  data_[VEC::X] = 0.0;
  data_[VEC::Y] = 0.0;
  data_[VEC::Z] = 0.0;
}

/******************************************************************************/
Vector operator/( const Vector& vec, Real f )
/******************************************************************************/
{
  Vector newvec(vec);
  newvec[VEC::X]/=f;
  newvec[VEC::Y]/=f;
  newvec[VEC::Z]/=f;
  return newvec;
}

/******************************************************************************/
Real operator*(const Vector& v1, const Vector& v2)
/******************************************************************************/
{
  Real val=0.0;
  val += v1.data_[0]*v2.data_[0];
  val += v1.data_[1]*v2.data_[1];
  val += v1.data_[2]*v2.data_[2];
  return val;
}

/******************************************************************************/
Vector operator*( Real fac, const Vector& vec )
/******************************************************************************/
{
  Vector newvec(vec);
  newvec.data_[VEC::X]*=fac;
  newvec.data_[VEC::Y]*=fac;
  newvec.data_[VEC::Z]*=fac;
  return newvec;
}

/******************************************************************************/
Vector operator-( const Vector& v1, const Vector& v2 )
/******************************************************************************/
{
  Vector newvec;
  newvec.data_[VEC::X] = v1(VEC::X) - v2(VEC::X);
  newvec.data_[VEC::Y] = v1(VEC::Y) - v2(VEC::Y);
  newvec.data_[VEC::Z] = v1(VEC::Z) - v2(VEC::Z);
  return newvec;
}

/******************************************************************************/
SymTensor operator+( const SymTensor& t1, const SymTensor& t2 )
/******************************************************************************/
{
  SymTensor newtensor;
  newtensor.data_[SYM::XX] = t1(SYM::XX) + t2(SYM::XX);
  newtensor.data_[SYM::YY] = t1(SYM::YY) + t2(SYM::YY);
  newtensor.data_[SYM::ZZ] = t1(SYM::ZZ) + t2(SYM::ZZ);
  newtensor.data_[SYM::YZ] = t1(SYM::YZ) + t2(SYM::YZ);
  newtensor.data_[SYM::XZ] = t1(SYM::XZ) + t2(SYM::XZ);
  newtensor.data_[SYM::XY] = t1(SYM::XY) + t2(SYM::XY);
  return newtensor;
}



/******************************************************************************/
SymTensor operator-( const SymTensor& t1, const SymTensor& t2 )
/******************************************************************************/
{
  SymTensor newt;
  newt.data_[SYM::XX] = t1(SYM::XX) - t2(SYM::XX);
  newt.data_[SYM::YY] = t1(SYM::YY) - t2(SYM::YY);
  newt.data_[SYM::ZZ] = t1(SYM::ZZ) - t2(SYM::ZZ);
  newt.data_[SYM::YZ] = t1(SYM::YZ) - t2(SYM::YZ);
  newt.data_[SYM::XZ] = t1(SYM::XZ) - t2(SYM::XZ);
  newt.data_[SYM::XY] = t1(SYM::XY) - t2(SYM::XY);
  return newt;
}

/******************************************************************************/
Vector operator+( const Vector& v1, const Vector& v2 )
/******************************************************************************/
{
  Vector newvec;
  newvec.data_[VEC::X] = v1(VEC::X) + v2(VEC::X);
  newvec.data_[VEC::Y] = v1(VEC::Y) + v2(VEC::Y);
  newvec.data_[VEC::Z] = v1(VEC::Z) + v2(VEC::Z);
  return newvec;
}

/******************************************************************************/
Vector& Vector::operator+=( const Vector& vec )
/******************************************************************************/
{
  data_[VEC::X] += vec(VEC::X);
  data_[VEC::Y] += vec(VEC::Y);
  data_[VEC::Z] += vec(VEC::Z);
  return *this;
}

/******************************************************************************/
Vector& Vector::operator*=( Real f )
/******************************************************************************/
{
  data_[VEC::X] *= f;
  data_[VEC::Y] *= f;
  data_[VEC::Z] *= f;
  return *this;
}

/******************************************************************************/
Vector& Vector::operator/=( Real f )
/******************************************************************************/
{
  data_[VEC::X] /= f;
  data_[VEC::Y] /= f;
  data_[VEC::Z] /= f;
  return *this;
}
/******************************************************************************/
Real operator*( const SymTensor& T1, const SymTensor& T2 )
/******************************************************************************/
{
  Real scalar_product = 0.0;
  const Real* t1 = T1.getData();
  const Real* t2 = T2.getData();
  for(int i=0; i<SYM::NUM_TERMS; i++)
    scalar_product += t1[i]*t2[i];
  return scalar_product;
}
/******************************************************************************/
std::ostream& operator << (std::ostream& fout, const Vector& tmp )
/******************************************************************************/
{
  fout << tmp.data_[VEC::X] << " " 
       << tmp.data_[VEC::Y] << " " 
       << tmp.data_[VEC::Z];

  return fout;
}
/******************************************************************************/
std::ostream& operator << (std::ostream& fout, const SymTensor& tmp )
/******************************************************************************/
{
  fout << tmp.data_[SYM::XX] << " " 
       << tmp.data_[SYM::YY] << " " 
       << tmp.data_[SYM::ZZ] << " "
       << tmp.data_[SYM::YZ] << " " 
       << tmp.data_[SYM::XZ] << " " 
       << tmp.data_[SYM::XY];

  return fout;
}

/******************************************************************************/
void ChangeStrainFromBasis(SymTensor* ct, const Tensor* B, int num_data)
/******************************************************************************/
{
  //  R.ct.Transpose[R]
  //  Note:
  //  1.  the basis is assumed to be { x_basis_vec | y_basis_vec | z_basis_vec }
  //  2.  this is computed for a strain tensor, so the shear terms are divided
  //      by 2 when converting to the tensor form for transformation, and 
  //      multiplied by 2 when converting back to voigt form.

  const Real* R = B->getData();

  for(int idat=0; idat<num_data; idat++){
    SymTensor& c = ct[idat];
    const Real* C = c.getData();
    Real cxx = C[SYM::XX]*R[TENSOR::XX]*R[TENSOR::XX]
             + C[SYM::XY]*R[TENSOR::XX]*R[TENSOR::XY]
             + C[SYM::YY]*R[TENSOR::XY]*R[TENSOR::XY]
             + C[SYM::XZ]*R[TENSOR::XX]*R[TENSOR::XZ]
             + C[SYM::YZ]*R[TENSOR::XY]*R[TENSOR::XZ]
             + C[SYM::ZZ]*R[TENSOR::XZ]*R[TENSOR::XZ];
    Real cyy = C[SYM::XX]*R[TENSOR::YX]*R[TENSOR::YX]
             + C[SYM::XY]*R[TENSOR::YX]*R[TENSOR::YY]
             + C[SYM::YY]*R[TENSOR::YY]*R[TENSOR::YY]
             + C[SYM::XZ]*R[TENSOR::YX]*R[TENSOR::YZ]
             + C[SYM::YZ]*R[TENSOR::YY]*R[TENSOR::YZ]
             + C[SYM::ZZ]*R[TENSOR::YZ]*R[TENSOR::YZ];
    Real czz = C[SYM::XX]*R[TENSOR::ZX]*R[TENSOR::ZX]
             + C[SYM::XY]*R[TENSOR::ZX]*R[TENSOR::ZY]
             + C[SYM::YY]*R[TENSOR::ZY]*R[TENSOR::ZY]
             + C[SYM::XZ]*R[TENSOR::ZX]*R[TENSOR::ZZ]
             + C[SYM::YZ]*R[TENSOR::ZY]*R[TENSOR::ZZ]
             + C[SYM::ZZ]*R[TENSOR::ZZ]*R[TENSOR::ZZ];
    Real cyz = 2.0*C[SYM::XX]*R[TENSOR::YX]*R[TENSOR::ZX]
             + 2.0*C[SYM::YY]*R[TENSOR::YY]*R[TENSOR::ZY]
             + 2.0*C[SYM::ZZ]*R[TENSOR::YZ]*R[TENSOR::ZZ]
             + C[SYM::YZ]*(R[TENSOR::ZY]*R[TENSOR::YZ]+R[TENSOR::YY]*R[TENSOR::ZZ])
             + C[SYM::XZ]*(R[TENSOR::ZX]*R[TENSOR::YZ]+R[TENSOR::YX]*R[TENSOR::ZZ])
             + C[SYM::XY]*(R[TENSOR::ZX]*R[TENSOR::YY]+R[TENSOR::YX]*R[TENSOR::ZY]);
    Real cxz = 2.0*C[SYM::XX]*R[TENSOR::XX]*R[TENSOR::ZX]
             + 2.0*C[SYM::YY]*R[TENSOR::XY]*R[TENSOR::ZY]
             + 2.0*C[SYM::ZZ]*R[TENSOR::XZ]*R[TENSOR::ZZ]
             + C[SYM::YZ]*(R[TENSOR::ZY]*R[TENSOR::XZ]+R[TENSOR::XY]*R[TENSOR::ZZ])
             + C[SYM::XZ]*(R[TENSOR::ZX]*R[TENSOR::XZ]+R[TENSOR::XX]*R[TENSOR::ZZ])
             + C[SYM::XY]*(R[TENSOR::ZX]*R[TENSOR::XY]+R[TENSOR::XX]*R[TENSOR::ZY]);
    Real cxy = 2.0*C[SYM::XX]*R[TENSOR::XX]*R[TENSOR::YX]
             + 2.0*C[SYM::YY]*R[TENSOR::XY]*R[TENSOR::YY]
             + 2.0*C[SYM::ZZ]*R[TENSOR::XZ]*R[TENSOR::YZ]
             + C[SYM::YZ]*(R[TENSOR::YY]*R[TENSOR::XZ]+R[TENSOR::XY]*R[TENSOR::YZ])
             + C[SYM::XZ]*(R[TENSOR::YX]*R[TENSOR::XZ]+R[TENSOR::XX]*R[TENSOR::YZ])
             + C[SYM::XY]*(R[TENSOR::YX]*R[TENSOR::XY]+R[TENSOR::XX]*R[TENSOR::YY]);
    c[SYM::XX] = cxx;
    c[SYM::YY] = cyy;
    c[SYM::ZZ] = czz;
    c[SYM::YZ] = cyz;
    c[SYM::XZ] = cxz;
    c[SYM::XY] = cxy;
  }
 
}

/******************************************************************************/
void ChangeStrainToBasis(SymTensor* ct, const Tensor* B, int num_data)
/******************************************************************************/
{
  //  Transpose[R].ct.R
  //  Note:
  //  1.  the basis is assumed to be { x_basis_vec | y_basis_vec | z_basis_vec }
  //  2.  this is computed for a strain tensor, so the shear terms are divided
  //      by 2 when converting to the tensor form for transformation, and 
  //      multiplied by 2 when converting back to voigt form.

  const Real* R = B->getData();

  for(int idat=0; idat<num_data; idat++){
    SymTensor& c = ct[idat];
    const Real* C = c.getData();
    Real cxx = C[SYM::XX]*R[TENSOR::XX]*R[TENSOR::XX]
             + C[SYM::XY]*R[TENSOR::XX]*R[TENSOR::YX]
             + C[SYM::YY]*R[TENSOR::YX]*R[TENSOR::YX]
             + C[SYM::XZ]*R[TENSOR::XX]*R[TENSOR::ZX]
             + C[SYM::YZ]*R[TENSOR::YX]*R[TENSOR::ZX]
             + C[SYM::ZZ]*R[TENSOR::ZX]*R[TENSOR::ZX];
    Real cyy = C[SYM::XX]*R[TENSOR::XY]*R[TENSOR::XY]
             + C[SYM::XY]*R[TENSOR::XY]*R[TENSOR::YY]
             + C[SYM::YY]*R[TENSOR::YY]*R[TENSOR::YY]
             + C[SYM::XZ]*R[TENSOR::XY]*R[TENSOR::ZY]
             + C[SYM::YZ]*R[TENSOR::YY]*R[TENSOR::ZY]
             + C[SYM::ZZ]*R[TENSOR::ZY]*R[TENSOR::ZY];
    Real czz = C[SYM::XX]*R[TENSOR::XZ]*R[TENSOR::XZ]
             + C[SYM::XY]*R[TENSOR::XZ]*R[TENSOR::YZ]
             + C[SYM::YY]*R[TENSOR::YZ]*R[TENSOR::YZ]
             + C[SYM::XZ]*R[TENSOR::XZ]*R[TENSOR::ZZ]
             + C[SYM::YZ]*R[TENSOR::YZ]*R[TENSOR::ZZ]
             + C[SYM::ZZ]*R[TENSOR::ZZ]*R[TENSOR::ZZ];
    Real cyz = 2.0*C[SYM::XX]*R[TENSOR::XY]*R[TENSOR::XZ]
             + 2.0*C[SYM::YY]*R[TENSOR::YY]*R[TENSOR::YZ]
             + 2.0*C[SYM::ZZ]*R[TENSOR::ZY]*R[TENSOR::ZZ]
             + C[SYM::YZ]*(R[TENSOR::YZ]*R[TENSOR::ZY]+R[TENSOR::YY]*R[TENSOR::ZZ])
             + C[SYM::XZ]*(R[TENSOR::XZ]*R[TENSOR::ZY]+R[TENSOR::XY]*R[TENSOR::ZZ])
             + C[SYM::XY]*(R[TENSOR::XZ]*R[TENSOR::YY]+R[TENSOR::XY]*R[TENSOR::YZ]);
    Real cxz = 2.0*C[SYM::XX]*R[TENSOR::XX]*R[TENSOR::XZ]
             + 2.0*C[SYM::YY]*R[TENSOR::YX]*R[TENSOR::YZ]
             + 2.0*C[SYM::ZZ]*R[TENSOR::ZX]*R[TENSOR::ZZ]
             + C[SYM::YZ]*(R[TENSOR::YZ]*R[TENSOR::ZX]+R[TENSOR::YX]*R[TENSOR::ZZ])
             + C[SYM::XZ]*(R[TENSOR::XZ]*R[TENSOR::ZX]+R[TENSOR::XX]*R[TENSOR::ZZ])
             + C[SYM::XY]*(R[TENSOR::XZ]*R[TENSOR::YX]+R[TENSOR::XX]*R[TENSOR::YZ]);
    Real cxy = 2.0*C[SYM::XX]*R[TENSOR::XX]*R[TENSOR::XY]
             + 2.0*C[SYM::YY]*R[TENSOR::YX]*R[TENSOR::YY]
             + 2.0*C[SYM::ZZ]*R[TENSOR::ZX]*R[TENSOR::ZY]
             + C[SYM::YZ]*(R[TENSOR::YY]*R[TENSOR::ZX]+R[TENSOR::YX]*R[TENSOR::ZY])
             + C[SYM::XZ]*(R[TENSOR::XY]*R[TENSOR::ZX]+R[TENSOR::XX]*R[TENSOR::ZY])
             + C[SYM::XY]*(R[TENSOR::XY]*R[TENSOR::YX]+R[TENSOR::XX]*R[TENSOR::YY]);
    c[SYM::XX] = cxx;
    c[SYM::YY] = cyy;
    c[SYM::ZZ] = czz;
    c[SYM::YZ] = cyz;
    c[SYM::XZ] = cxz;
    c[SYM::XY] = cxy;
  }
 
}
/******************************************************************************/
void ChangeFromBasis(Vector* v, const Tensor* B, int num_data)
/******************************************************************************/
{
  // This function transforms an array of vectors from the given basis, B,
  // into the global basis.  That is, we have vectors v' given wrt the basis,
  // B = { x_basis_vec | y_basis_vec | z_basis_vec }, and we want vectors v:
  //  
  //   v = Transpose[B].v'
  //
  // B is assumed to be orthonormal (i.e., Transpose[B].B = I).

  const Real* R = B->getData();

  for(int i=0; i<num_data; i++){
    Vector& V = v[i];
    Real bx = R[TENSOR::XX]*V[VEC::X] + R[TENSOR::YX]*V[VEC::Y] + R[TENSOR::ZX]*V[VEC::Z];
    Real by = R[TENSOR::XY]*V[VEC::X] + R[TENSOR::YY]*V[VEC::Y] + R[TENSOR::ZY]*V[VEC::Z];
    Real bz = R[TENSOR::XZ]*V[VEC::X] + R[TENSOR::YZ]*V[VEC::Y] + R[TENSOR::ZZ]*V[VEC::Z];
    V[VEC::X] = bx;
    V[VEC::Y] = by;
    V[VEC::Z] = bz;
  }
}




/******************************************************************************/
void ChangeToBasis(SymTensor* st, const Tensor* B, int num_data)
/******************************************************************************/
{

  //  Transpose[R].ct.R


  const Real* R = B->getData();

  for(int i=0; i<num_data; i++){
    SymTensor& s = st[i];
    Real bxx = R[TENSOR::XX]*R[TENSOR::XX]*s[SYM::XX]
             + R[TENSOR::YX]*R[TENSOR::YX]*s[SYM::YY]
             + R[TENSOR::ZX]*R[TENSOR::ZX]*s[SYM::ZZ]
             + 2.0*R[TENSOR::XX]*R[TENSOR::YX]*s[SYM::XY]
             + 2.0*R[TENSOR::XX]*R[TENSOR::ZX]*s[SYM::XZ]
             + 2.0*R[TENSOR::YX]*R[TENSOR::ZX]*s[SYM::YZ];
    Real byy = R[TENSOR::XY]*R[TENSOR::XY]*s[SYM::XX]
             + R[TENSOR::YY]*R[TENSOR::YY]*s[SYM::YY]
             + R[TENSOR::ZY]*R[TENSOR::ZY]*s[SYM::ZZ]
             + 2.0*R[TENSOR::XY]*R[TENSOR::YY]*s[SYM::XY]
             + 2.0*R[TENSOR::XY]*R[TENSOR::ZY]*s[SYM::XZ]
             + 2.0*R[TENSOR::YY]*R[TENSOR::ZY]*s[SYM::YZ];
    Real bzz = R[TENSOR::XZ]*R[TENSOR::XZ]*s[SYM::XX]
             + R[TENSOR::YZ]*R[TENSOR::YZ]*s[SYM::YY]
             + R[TENSOR::ZZ]*R[TENSOR::ZZ]*s[SYM::ZZ]
             + 2.0*R[TENSOR::XZ]*R[TENSOR::YZ]*s[SYM::XY]
             + 2.0*R[TENSOR::XZ]*R[TENSOR::ZZ]*s[SYM::XZ]
             + 2.0*R[TENSOR::YZ]*R[TENSOR::ZZ]*s[SYM::YZ];
    Real byz = R[TENSOR::XY]*R[TENSOR::XZ]*s[SYM::XX]
             + R[TENSOR::YY]*R[TENSOR::YZ]*s[SYM::YY]
             + R[TENSOR::ZY]*R[TENSOR::ZZ]*s[SYM::ZZ]
             + (R[TENSOR::XY]*R[TENSOR::YZ]+R[TENSOR::XZ]*R[TENSOR::YY])*s[SYM::XY]
             + (R[TENSOR::XZ]*R[TENSOR::ZY]+R[TENSOR::XY]*R[TENSOR::ZZ])*s[SYM::XZ]
             + (R[TENSOR::YZ]*R[TENSOR::ZY]+R[TENSOR::YY]*R[TENSOR::ZZ])*s[SYM::YZ];
    Real bxz = R[TENSOR::XX]*R[TENSOR::XZ]*s[SYM::XX]
             + R[TENSOR::YX]*R[TENSOR::YZ]*s[SYM::YY]
             + R[TENSOR::ZX]*R[TENSOR::ZZ]*s[SYM::ZZ]
             + (R[TENSOR::XZ]*R[TENSOR::YX]+R[TENSOR::XX]*R[TENSOR::YZ])*s[SYM::XY]
             + (R[TENSOR::XZ]*R[TENSOR::ZX]+R[TENSOR::XX]*R[TENSOR::ZZ])*s[SYM::XZ]
             + (R[TENSOR::YZ]*R[TENSOR::ZX]+R[TENSOR::YX]*R[TENSOR::ZZ])*s[SYM::YZ];
    Real bxy = R[TENSOR::XX]*R[TENSOR::XY]*s[SYM::XX]
             + R[TENSOR::YX]*R[TENSOR::YY]*s[SYM::YY]
             + R[TENSOR::ZX]*R[TENSOR::ZY]*s[SYM::ZZ]
             + (R[TENSOR::XY]*R[TENSOR::YX]+R[TENSOR::XX]*R[TENSOR::YY])*s[SYM::XY]
             + (R[TENSOR::XY]*R[TENSOR::ZX]+R[TENSOR::XX]*R[TENSOR::ZY])*s[SYM::XZ]
             + (R[TENSOR::YY]*R[TENSOR::ZX]+R[TENSOR::YX]*R[TENSOR::ZY])*s[SYM::YZ];
    s[SYM::XX] = bxx;
    s[SYM::YY] = byy;
    s[SYM::ZZ] = bzz;
    s[SYM::YZ] = byz;
    s[SYM::XZ] = bxz;
    s[SYM::XY] = bxy;
  }
}
/******************************************************************************/
void ChangeFromBasis(SymTensor* st, const Tensor* B, int num_data)
/******************************************************************************/
{

  //  R.st.Transpose[R]


  const Real* R = B->getData();

  for(int i=0; i<num_data; i++){
    SymTensor& s = st[i];
    Real bxx = R[TENSOR::XX]*R[TENSOR::XX]*s[SYM::XX]
             + R[TENSOR::XY]*R[TENSOR::XY]*s[SYM::YY]
             + R[TENSOR::XZ]*R[TENSOR::XZ]*s[SYM::ZZ]
             + 2.0*R[TENSOR::XX]*R[TENSOR::XY]*s[SYM::XY]
             + 2.0*R[TENSOR::XX]*R[TENSOR::XZ]*s[SYM::XZ]
             + 2.0*R[TENSOR::XY]*R[TENSOR::XZ]*s[SYM::YZ];
    Real byy = R[TENSOR::YX]*R[TENSOR::YX]*s[SYM::XX]
             + R[TENSOR::YY]*R[TENSOR::YY]*s[SYM::YY]
             + R[TENSOR::YZ]*R[TENSOR::YZ]*s[SYM::ZZ]
             + 2.0*R[TENSOR::YX]*R[TENSOR::YY]*s[SYM::XY]
             + 2.0*R[TENSOR::YX]*R[TENSOR::YZ]*s[SYM::XZ]
             + 2.0*R[TENSOR::YY]*R[TENSOR::YZ]*s[SYM::YZ];
    Real bzz = R[TENSOR::ZX]*R[TENSOR::ZX]*s[SYM::XX]
             + R[TENSOR::ZY]*R[TENSOR::ZY]*s[SYM::YY]
             + R[TENSOR::ZZ]*R[TENSOR::ZZ]*s[SYM::ZZ]
             + 2.0*R[TENSOR::ZX]*R[TENSOR::ZY]*s[SYM::XY]
             + 2.0*R[TENSOR::ZX]*R[TENSOR::ZZ]*s[SYM::XZ]
             + 2.0*R[TENSOR::ZY]*R[TENSOR::ZZ]*s[SYM::YZ];
    Real byz = R[TENSOR::YX]*R[TENSOR::ZX]*s[SYM::XX]
             + R[TENSOR::YY]*R[TENSOR::ZY]*s[SYM::YY]
             + R[TENSOR::YZ]*R[TENSOR::ZZ]*s[SYM::ZZ]
             + (R[TENSOR::YX]*R[TENSOR::ZY]+R[TENSOR::ZX]*R[TENSOR::YY])*s[SYM::XY]
             + (R[TENSOR::ZX]*R[TENSOR::YZ]+R[TENSOR::YX]*R[TENSOR::ZZ])*s[SYM::XZ]
             + (R[TENSOR::ZY]*R[TENSOR::YZ]+R[TENSOR::YY]*R[TENSOR::ZZ])*s[SYM::YZ];
    Real bxz = R[TENSOR::XX]*R[TENSOR::ZX]*s[SYM::XX]
             + R[TENSOR::XY]*R[TENSOR::ZY]*s[SYM::YY]
             + R[TENSOR::XZ]*R[TENSOR::ZZ]*s[SYM::ZZ]
             + (R[TENSOR::ZX]*R[TENSOR::XY]+R[TENSOR::XX]*R[TENSOR::ZY])*s[SYM::XY]
             + (R[TENSOR::ZX]*R[TENSOR::XZ]+R[TENSOR::XX]*R[TENSOR::ZZ])*s[SYM::XZ]
             + (R[TENSOR::ZY]*R[TENSOR::XZ]+R[TENSOR::XY]*R[TENSOR::ZZ])*s[SYM::YZ];
    Real bxy = R[TENSOR::XX]*R[TENSOR::YX]*s[SYM::XX]
             + R[TENSOR::XY]*R[TENSOR::YY]*s[SYM::YY]
             + R[TENSOR::XZ]*R[TENSOR::YZ]*s[SYM::ZZ]
             + (R[TENSOR::YX]*R[TENSOR::XY]+R[TENSOR::XX]*R[TENSOR::YY])*s[SYM::XY]
             + (R[TENSOR::YX]*R[TENSOR::XZ]+R[TENSOR::XX]*R[TENSOR::YZ])*s[SYM::XZ]
             + (R[TENSOR::YY]*R[TENSOR::XZ]+R[TENSOR::XY]*R[TENSOR::YZ])*s[SYM::YZ];
    s[SYM::XX] = bxx;
    s[SYM::YY] = byy;
    s[SYM::ZZ] = bzz;
    s[SYM::YZ] = byz;
    s[SYM::XZ] = bxz;
    s[SYM::XY] = bxy;
  }
}

/******************************************************************************/
Tensor::Tensor(Real xx, Real yy, Real zz, 
               Real yz, Real xz, Real xy,
               Real zy, Real zx, Real yx)
/******************************************************************************/
{
  data_[TENSOR::XX] = xx;
  data_[TENSOR::YY] = yy;
  data_[TENSOR::ZZ] = zz;
  data_[TENSOR::YZ] = yz;
  data_[TENSOR::XZ] = xz;
  data_[TENSOR::XY] = xy;
  data_[TENSOR::ZY] = zy;
  data_[TENSOR::ZX] = zx;
  data_[TENSOR::YX] = yx;
}


/******************************************************************************/
Tensor::Tensor(Real* initdata)
/******************************************************************************/
{
  data_[TENSOR::XX] = initdata[TENSOR::XX];
  data_[TENSOR::YY] = initdata[TENSOR::YY];
  data_[TENSOR::ZZ] = initdata[TENSOR::ZZ];
  data_[TENSOR::YZ] = initdata[TENSOR::YZ];
  data_[TENSOR::XZ] = initdata[TENSOR::XZ];
  data_[TENSOR::XY] = initdata[TENSOR::XY];
  data_[TENSOR::ZY] = initdata[TENSOR::ZY];
  data_[TENSOR::ZX] = initdata[TENSOR::ZX];
  data_[TENSOR::YX] = initdata[TENSOR::YX];
}

/******************************************************************************/
Tensor::Tensor()
/******************************************************************************/
{
  data_[TENSOR::XX] = 0.0;
  data_[TENSOR::YY] = 0.0;
  data_[TENSOR::ZZ] = 0.0;
  data_[TENSOR::YZ] = 0.0;
  data_[TENSOR::XZ] = 0.0;
  data_[TENSOR::XY] = 0.0;
  data_[TENSOR::ZY] = 0.0;
  data_[TENSOR::ZX] = 0.0;
  data_[TENSOR::YX] = 0.0;
}

/******************************************************************************/
bool Tensor::isOrthonormal() const
/******************************************************************************/
{
  Real tol = 1e-5;
  if(  
   fabs(data_[TENSOR::XX]*data_[TENSOR::XY]+
        data_[TENSOR::YX]*data_[TENSOR::YY]+
        data_[TENSOR::ZX]*data_[TENSOR::ZY]) > tol 
  ) return false;

  if(  
   fabs(data_[TENSOR::XY]*data_[TENSOR::XZ]+
        data_[TENSOR::YY]*data_[TENSOR::YZ]+
        data_[TENSOR::ZY]*data_[TENSOR::ZZ]) > tol 
  ) return false;
  
  if(  
   fabs(data_[TENSOR::XZ]*data_[TENSOR::XX]+
        data_[TENSOR::YZ]*data_[TENSOR::YX]+
        data_[TENSOR::ZZ]*data_[TENSOR::ZX]) > tol 
  ) return false;

  return true;
}


/******************************************************************************/
std::ostream& operator << (std::ostream& fout, const Tensor& tmp)
/******************************************************************************/
{

  fout << "(xx, xy, xz): " << tmp.data_[TENSOR::XX] 
       << tmp.data_[TENSOR::XY] << tmp.data_[TENSOR::XZ] << std::endl;
  fout << "(yx, yy, yz): " << tmp.data_[TENSOR::YX] 
       << tmp.data_[TENSOR::YY] << tmp.data_[TENSOR::YZ] << std::endl;
  fout << "(zx, zy, zz): " << tmp.data_[TENSOR::ZX] 
       << tmp.data_[TENSOR::ZY] << tmp.data_[TENSOR::ZZ] << std::endl;
  
  return fout;
}

/******************************************************************************/
Real Norm(const Vector& v)
/******************************************************************************/
{
  return sqrt( v.data_[VEC::X]*v.data_[VEC::X]
              +v.data_[VEC::Y]*v.data_[VEC::Y]
              +v.data_[VEC::Z]*v.data_[VEC::Z] );
}

/******************************************************************************/
void ChangeBasis_FourthRankVoigt( Intrepid::FieldContainer<double>& C, Tensor& s )
/******************************************************************************/
{
  Intrepid::FieldContainer<double> Ct(3,3,3,3);

  // change to indexed tensors
  int m[3][3] = {{SYM::XX,SYM::XY,SYM::XZ}, 
                 {SYM::XY,SYM::YY,SYM::YZ}, 
                 {SYM::XZ,SYM::YZ,SYM::ZZ}};
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      for(int k=0;k<3;k++)
        for(int l=0;l<3;l++)
          Ct(i,j,k,l) = C(m[i][j],m[k][l]);

  Intrepid::FieldContainer<double> R(3,3);
  int n[3][3] = {{TENSOR::XX, TENSOR::XY, TENSOR::XZ}, 
                 {TENSOR::YX, TENSOR::YY, TENSOR::YZ}, 
                 {TENSOR::ZX, TENSOR::ZY, TENSOR::ZZ}};
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      R(i,j) = s(n[i][j]);
  
  // change basis
  int M[6] = {0,1,2,1,0,0};
  int N[6] = {0,1,2,2,2,1};

  for(int w=0;w<6;w++)
    for(int u=0;u<6;u++){
      C(w,u) = 0.0;
      for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
          for(int k=0;k<3;k++)
            for(int l=0;l<3;l++)
              C(w,u) += Ct(i,j,k,l)*R(M[w],i)*R(N[w],j)*R(M[u],k)*R(N[u],l);
    }
}
 

