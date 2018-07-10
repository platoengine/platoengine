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
