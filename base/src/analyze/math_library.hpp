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

#ifndef MATH_LIBRARY
#define MATH_LIBRARY

#include <iostream>
#include "types.hpp"

#include "Intrepid_FieldContainer.hpp"

namespace MLib {
  namespace VEC { enum {X=0, Y, Z, NUM_TERMS}; }
  namespace SYM { enum {XX=0, YY, ZZ, YZ, XZ, XY, NUM_TERMS}; }
  namespace TENSOR { enum {XX=0, XY, XZ, YX, YY, YZ, ZX, ZY, ZZ, NUM_TERMS}; }
}

using namespace MLib;

class SymTensor;


/******************************************************************************/
class Vector
/******************************************************************************/
{
  public:
    Vector(Real x, Real y, Real z);
    Vector(Real*);
    Vector();

    const Real* getData() const { return data_; }
    void extractData( Real*& d ) { d = data_; }
    void copyData( Real* d ) { for(int i=0; i<3; i++) d[i] = data_[i]; }

    void zero(){data_[0]=0.0; data_[1]=0.0; data_[2]=0.0;}

    void operator=(const Vector& v);
    void operator=(const Real& v);
    Real& operator[]( int index );
    Real operator()( int index ) const;

    Vector& operator+=(const Vector&);
    Vector& operator*=(Real);
    Vector& operator/=(Real);

    friend Vector operator*(Real f, const Vector& v);
    friend Real operator*(const Vector& v1, const Vector& v2);
    friend Vector operator+( const Vector& v1, const Vector& v2 );
    friend Vector operator-( const Vector& v1, const Vector& v2 );
    friend std::ostream& operator << (std::ostream& fout, const Vector& tmp );
    friend Real Norm(const Vector& v);
 
  private:
    Real data_[VEC::NUM_TERMS];
};

/******************************************************************************/
class SymTensor
/******************************************************************************/
{
  public:
    SymTensor(Real xx, Real yy, Real zz, Real yz, Real xz, Real xy);
    SymTensor(Real*);
    SymTensor();

    const Real* getData() const { return data_; }
    void extractData( Real*& d ) { d = data_; }
    void copyData( Real* d ) { for(int i=0; i<6; i++) d[i] = data_[i]; }

    void operator=(const SymTensor& v);
    void operator=(const Vector& v);
    void operator=(const Real& v);
    Real& operator[]( int index );
    Real operator()( int index ) const;

    SymTensor& operator+=(const SymTensor& T);
    SymTensor& operator-=(const SymTensor& T);
    SymTensor& operator/=(Real);

    friend SymTensor operator-( const SymTensor& st, const SymTensor& ct );
    friend SymTensor operator+( const SymTensor& st, const SymTensor& ct );
    friend SymTensor operator*( const SymTensor& st, Real r );
    friend SymTensor operator/( const SymTensor& st, Real r );
    friend SymTensor operator*( Real r, const SymTensor& st );
    friend SymTensor operator/( Real r, const SymTensor& st );
    friend Real Trace( const SymTensor& t );
    friend SymTensor Deviator( const SymTensor& t );
    friend std::ostream& operator << (std::ostream& fout, const SymTensor& tmp );

    friend SymTensor operator*( const Intrepid::FieldContainer<double>& C, const SymTensor& s );
    friend SymTensor operator*( const Intrepid::FieldContainer<double>& C, const Real& s );

  private:
    Real data_[SYM::NUM_TERMS];
};

/******************************************************************************/
class Tensor
/******************************************************************************/
{
  public:
    Tensor(Real xx, Real yy, Real zz, 
           Real yz, Real xz, Real xy,
           Real zy, Real zx, Real yx);
    Tensor(Real*);
    Tensor();

    const Real* getData() const { return data_; }
    void extractData( Real*& d ) { d = data_; }
    void copyData( Real* d ) { for(int i=0; i<9; i++) d[i] = data_[i]; }
    bool isOrthonormal() const;

    void operator=(const SymTensor& v);
    void operator=(const Tensor& v);
    void operator=(const Real& v);
    Real& operator[]( int index );
    Real operator()( int index ) const;


    friend std::ostream& operator << (std::ostream& fout, const Tensor& tmp);
    friend Tensor operator*( const Tensor& a, const Tensor& b );

    
  private:
    Real data_[TENSOR::NUM_TERMS];
};


Tensor operator*( const Tensor& a, const Tensor& b );

Vector operator*( Real f, const Vector& v );
Real operator*(const Vector& v1, const Vector& v2);
Vector operator/( const Vector& v, Real f );
Vector operator+( const Vector& v1, const Vector& v2 );
Vector operator-( const Vector& v1, const Vector& v2 );

SymTensor operator*( const SymTensor& st, Real r );
SymTensor operator*( Real r, const SymTensor& st );

SymTensor operator+( const SymTensor& t1, const SymTensor& t2 );

SymTensor operator*( const Intrepid::FieldContainer<double>& C, const SymTensor& s );

SymTensor operator*( const Intrepid::FieldContainer<double>& C, const Real& s );

Real operator*( const SymTensor& t1, const SymTensor& t2 );

std::ostream& operator << (std::ostream& fout, const Vector& tmp );
std::ostream& operator << (std::ostream& fout, const SymTensor& tmp );

Real Trace( const SymTensor& t );
SymTensor Deviator( const SymTensor& t );

void ChangeFromBasis(Vector* v, const Tensor* R, int num_data);

void ChangeToBasis(SymTensor* st, const Tensor* R, int num_data);
void ChangeStrainToBasis(SymTensor* st, const Tensor* R, int num_data);

void ChangeFromBasis(SymTensor* st, const Tensor* R, int num_data);
void ChangeStrainFromBasis(SymTensor* st, const Tensor* R, int num_data);

Real Norm(const Vector& v);

void ChangeBasis_FourthRankVoigt( Intrepid::FieldContainer<double>& C, Tensor& s );

#endif
