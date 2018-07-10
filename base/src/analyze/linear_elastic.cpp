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

#include "linear_elastic.hpp"
#include "exception_handling.hpp"
#include "data_container.hpp"
#include "math_library.hpp"
#include <stdlib.h>
#include <math.h>

MaterialModel* NewIsotropicElastic3D(pugi::xml_node& node)
{ return new IsotropicElastic3D(node); }
MaterialModel* NewIsotropicElastic2D(pugi::xml_node& node)
{ return new IsotropicElastic2D(node); }
/*****************************************************************************/
IsotropicElastic::IsotropicElastic(pugi::xml_node& node) : MaterialModel( node )
/*****************************************************************************/
{
  param = new double[NUMPARAM];
  param[POISSONS_RATIO] = Plato::Parse::getDouble(node,"poissons_ratio");
  param[YOUNGS_MODULUS] = Plato::Parse::getDouble(node,"youngs_modulus");
  param[C11] = Plato::Parse::getDouble(node,"C11");
  param[C22] = Plato::Parse::getDouble(node,"C22");
  param[C33] = Plato::Parse::getDouble(node,"C33");
  param[C23] = Plato::Parse::getDouble(node,"C23");
  param[C13] = Plato::Parse::getDouble(node,"C13");
  param[C12] = Plato::Parse::getDouble(node,"C12");
  param[C44] = Plato::Parse::getDouble(node,"C44");
  param[C55] = Plato::Parse::getDouble(node,"C55");
  param[C66] = Plato::Parse::getDouble(node,"C66");
}
/*****************************************************************************/
IsotropicElastic3D::IsotropicElastic3D(pugi::xml_node& node) : IsotropicElastic(node)
/*****************************************************************************/
{

  C = new Intrepid::FieldContainer<double>(6,6);
  Intrepid::FieldContainer<double>& c = *C;
  for( int i=0; i<6; i++)
    for( int j=0; j<6; j++) 
      c(i,j) = 0.0;

  if( param[POISSONS_RATIO] && param[YOUNGS_MODULUS] ){
    double vl = param[POISSONS_RATIO];
    double cl = param[YOUNGS_MODULUS]/((1.0+vl)*(1.0-2.0*vl));
    c(0,0)=cl*(1-vl); c(0,1)=cl*vl;     c(0,2)=cl*vl;
    c(1,0)=cl*vl;     c(1,1)=cl*(1-vl); c(1,2)=cl*vl;
    c(2,0)=cl*vl;     c(2,1)=cl*vl;     c(2,2)=cl*(1-vl);
    c(3,3)=0.5*cl*(1.0-2.0*vl);
    c(4,4)=0.5*cl*(1.0-2.0*vl);
    c(5,5)=0.5*cl*(1.0-2.0*vl);
  } else if( param[C11] && param[C12] && param[C44]){
    c(0,0) = param[C11];
    c(1,1) = param[C22] ? param[C22] : param[C11];
    c(2,2) = param[C33] ? param[C33] : param[C11];
    c(0,1) = param[C12];
    c(0,2) = param[C13] ? param[C13] : param[C12];
    c(1,2) = param[C23] ? param[C23] : param[C12];
    c(3,3) = param[C44];
    c(4,4) = param[C55] ? param[C55] : param[C44];
    c(5,5) = param[C66] ? param[C66] : param[C44];

    c(1,0) = c(0,1);
    c(2,0) = c(0,2);
    c(2,1) = c(1,2);

  } else {
    throw ParsingException( "check linear elastic constants." );
  }

}
IsotropicElastic2D::IsotropicElastic2D(pugi::xml_node& node) : IsotropicElastic(node) { }


/*****************************************************************************/
bool IsotropicElastic2D::SetUp(DataContainer* dc, Tensor& R)
/*****************************************************************************/
{ return true; }

/*****************************************************************************/
bool IsotropicElastic3D::SetUp(DataContainer* dc, Tensor& R)
/*****************************************************************************/
{
  bool plottable = true;
  string name;
  int twoState=2;
  name = "cauchy stress";
  STRESS = dc->registerVariable( SymTensorType, name, 
                                 MATPNT, plottable, twoState );
  name = "strain increment";
  STRAIN_INCREMENT = dc->registerVariable( SymTensorType, name, 
                                           MATPNT, plottable );
  name = "total strain";
  TOTAL_STRAIN = dc->registerVariable( SymTensorType, name, 
                                       MATPNT, plottable, twoState );

  // this is essential.  This tells the world what the dependent variable (Findex)
  // and independent variable (Xindex) are.  
  Findex = STRESS;
  Xindex = STRAIN_INCREMENT;

  // Tensor R is the crystal basis.  Transform the stiffness into that basis.
  ChangeBasis_FourthRankVoigt( *C, R );

  return true;
}

/*****************************************************************************/
IsotropicElastic::~IsotropicElastic() 
/*****************************************************************************/
{ delete [] param; }

/*****************************************************************************/
IsotropicElastic3D::~IsotropicElastic3D() { }
/*****************************************************************************/

/*****************************************************************************/
IsotropicElastic2D::~IsotropicElastic2D() { }
/*****************************************************************************/

/*****************************************************************************/
/*!  This function initializes the local data members.  It also initializes 
    the material state.  If the initial condition is given as a strain, then
    update the stress.  */
bool IsotropicElastic3D::Initialize(int dataIndex, DataContainer* dc)
/*****************************************************************************/
{ return true; }

/*****************************************************************************/
/*!  This function initializes the local data members.  It also initializes 
    the material state.  If the initial condition is given as a strain, then
    update the stress.  */
bool IsotropicElastic2D::Initialize(int dataIndex, DataContainer* dc)
/*****************************************************************************/
{ return true; }

/*****************************************************************************/
bool IsotropicElastic2D::UpdateMaterialState( int dataIndex,
                                            DataContainer* dc )
/*****************************************************************************/
{ return true; }

/*****************************************************************************/
bool IsotropicElastic3D::UpdateMaterialState( int dataIndex,
                                            DataContainer* dc )
/*****************************************************************************/
{

  SymTensor* strain_inc_ptr;
  dc->getVariable( STRAIN_INCREMENT, strain_inc_ptr, dataIndex );
  SymTensor& strain_inc = *strain_inc_ptr;
  
  SymTensor* stress_ptr;
  dc->getVariable( STRESS, stress_ptr, dataIndex, UPDATED );
  SymTensor& stress = *stress_ptr;
  
//  SymTensor *old_strain_ptr, *new_strain_ptr;
//  dc->getVariable( TOTAL_STRAIN, old_strain_ptr, dataIndex, CURRENT );
//  dc->getVariable( TOTAL_STRAIN, new_strain_ptr, dataIndex, UPDATED );
//  SymTensor& old_strain = *old_strain_ptr;
//  SymTensor& new_strain = *new_strain_ptr;

//  new_strain = old_strain + strain_inc;
//  stress = (*C)*new_strain;

  stress = (*C)*strain_inc;
  
  return true;
}


/*****************************************************************************/
bool IsotropicElastic3D::Tangent(int dataIndex, DataContainer* dc, 
                                 Intrepid::FieldContainer<double>*& s)
/*****************************************************************************/
{
  s = C;
  return true;
}
