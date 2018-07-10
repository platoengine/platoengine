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

#ifndef BCS
#define BCS

#include "Plato_Parser.hpp"

#include <unordered_map>
#include <limits>
#include <vector>

const Real INF = std::numeric_limits<Real>::infinity();

#include <functional>
#include <iostream>
#include <random>

/******************************************************************************/
template <typename Type>
class BoundaryCondition {
  public:
    BoundaryCondition(pugi::xml_node& bc_spec);
    virtual ~BoundaryCondition(){}

    virtual Type Value(Real time = 0.0) = 0;
    virtual void Print(std::ostream& fout) const {}

    void findNodeSet( const vector<DMNodeSet>& NodeSet );
    const DMNodeSet& getNodeSet(){ return nodeset; }
    int getDofIndex(){ return dofIndex; }
  
  protected:
    DMNodeSet nodeset;
    int nodeset_id;
    Real scale;
    int dofIndex;
};    
/******************************************************************************/

/******************************************************************************/
template <typename Type>
class ConstantValueBC : public BoundaryCondition<Type> {

  public:
    ConstantValueBC(pugi::xml_node& bc_spec);
    virtual Type Value(Real time = 0.0);
    void setValue(Type value){ constantValue = value; }
    virtual void Print(std::ostream& fout) const {
      fout << setw(30) << left << "  Constant value: "
           << setw(30) << right << constantValue << endl;
    }

  private:
    Type constantValue;
};
/******************************************************************************/

/******************************************************************************/
template <typename Type>
void BoundaryCondition<Type>::findNodeSet( const vector<DMNodeSet>& nodeSets )
/******************************************************************************/
{

  int nns = nodeSets.size();
  for(int ins=0; ins<nns; ins++){
    if(nodeSets[ins].id == nodeset_id){
      nodeset = nodeSets[ins];
    }
  }
}

/******************************************************************************/
template <typename Type>
BoundaryCondition<Type>::BoundaryCondition(pugi::xml_node& bc_spec)
/******************************************************************************/
{   
  nodeset_id = Plato::Parse::getInt(bc_spec, "nodeset");
  scale = Plato::Parse::getDouble(bc_spec, "scale");
  string dir = Plato::Parse::getString(bc_spec, "direction");
  if( dir == "x" ) dofIndex = 0;
  else if( dir == "y" ) dofIndex = 1;
  else if( dir == "z" ) dofIndex = 2;
}

 
/******************************************************************************/
template <typename Type>
ConstantValueBC<Type>::ConstantValueBC(pugi::xml_node& bc_spec) 
: BoundaryCondition<Type>::BoundaryCondition(bc_spec){}
/******************************************************************************/

/******************************************************************************/
template <typename Type>
Type ConstantValueBC<Type>::Value(Real time)
/******************************************************************************/
{
  return BoundaryCondition<Type>::scale*constantValue;
}



#endif
