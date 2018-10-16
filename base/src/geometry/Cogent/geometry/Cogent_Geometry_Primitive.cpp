/*
 * Cogent_Geometry_Primitive.cpp
 *
 *  Created on: Nov 23, 2016
 *      Author: jrobbin
 */

#include "core/Cogent_Utilities.hpp"
#include "Cogent_Geometry_Primitive.hpp"

namespace Cogent {
namespace Geometry {

/******************************************************************************/
Primitive::
Primitive(const Teuchos::ParameterList& params, 
          const std::vector<std::string>& shapeParameterNames, 
          Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem,
          int numDim, int numLevelsets, int numLocalParams) : 
  m_parameterNames(shapeParameterNames),
  m_coordinateSystem(coordinateSystem),
  c_numDim(numDim), c_numLevelsets(numLevelsets),
  c_numGlobalParameters(m_parameterNames.size()),
  c_numLocalParameters(numLocalParams)
/******************************************************************************/
{
  if(params.isType<int>("Number of Aliases")){
    int nAliases = params.get<int>("Number of Aliases");
    m_boundaryAliases.resize(nAliases);
    for(int iAlias=0; iAlias<nAliases; ++iAlias){
      const Teuchos::ParameterList& aliasSpec = params.sublist(Cogent::strint("Alias",iAlias));
      m_boundaryAliases[iAlias].aliasName = aliasSpec.get<std::string>("Alias Name");
      m_boundaryAliases[iAlias].boundaryName = aliasSpec.get<std::string>("Surface Name");
    }
  }
}

Primitive::~Primitive(){}

/******************************************************************************/
void Primitive::update(const std::vector<RealType>& P)
/******************************************************************************/
{

  if(!(m_coordinateSystem.is_null()))  m_coordinateSystem->update(P);

  int numParams = m_localParameters.size();
  for(int iParam=0; iParam<numParams; ++iParam){
    auto& localParameter = m_localParameters[iParam];
    localParameter->update(P);
  }
}

/******************************************************************************/
bool Primitive::nodeIsInside(const FContainer<RealType>& coordCon, int nodeIndex) const
/******************************************************************************/
{
  Positive<RealType> positive;

  std::vector<RealType> X(c_numDim);
  for(int i=0; i<c_numDim; ++i) 
    X[i] = coordCon(nodeIndex,i);

  std::vector<RealType> vals(c_numLevelsets); compute(X, vals);
  
  bool inside = true;
  for(int i=0; i<c_numLevelsets; ++i)
    inside = inside && positive.compare(vals[i],0.0);
  
  return inside;
  
}

/******************************************************************************/
void Primitive::intersected(
  const FContainer<RealType>& coordCon, bool& inside,
  std::vector<int>& lsIndices) const
/******************************************************************************/
{
  lsIndices.clear();

  uint nNodes = coordCon.dimension(0);
  inside = nodeIsInside(coordCon,0);
  bool intersected = false;
  for(uint iNode=1; iNode<nNodes; ++iNode){
    if(inside != nodeIsInside(coordCon,iNode)){
      intersected = true;
      inside = true;
      break;
    }
  }
  
  if(!intersected) return;

  std::vector<RealType> X(c_numDim);
  std::vector<std::vector<RealType>> vals(nNodes,std::vector<RealType>(c_numLevelsets));
  for(uint iNode=0; iNode<nNodes; ++iNode){
    for(int iDim=0; iDim<c_numDim; ++iDim) 
      X[iDim] = coordCon(iNode,iDim);
    compute(X,vals[iNode]);
  }
  for(int iLevelset=0; iLevelset<c_numLevelsets; ++iLevelset){
    for(uint iNode=0; iNode<nNodes; ++iNode)
      for(uint jNode=iNode+1; jNode<nNodes; ++jNode)
        if(vals[iNode][iLevelset]*vals[jNode][iLevelset] < 0.0){
          lsIndices.push_back(iLevelset);
          goto acceptableUseofGoto;
        }
    acceptableUseofGoto: ;
  }
}

/******************************************************************************/
void Primitive::compress(
       const std::vector<Teuchos::RCP<Cogent::Geometry::Primitive>> primitives, 
       int fromIndex, int toIndex)
/******************************************************************************/
{

  for( const auto& primitive : primitives ){
   
    if(m_coordinateSystem != primitive->getCoordinateSystem()) continue;

    const std::vector<Teuchos::RCP<Cogent::Geometry::PrimitiveSurface>>
      modelBoundaries = primitive->getBoundaries();

    for(auto& localBoundary : m_boundaries){
      for(const auto& modelBoundary : modelBoundaries){
        if(localBoundary->getSurfaceType() == modelBoundary->getSurfaceType()){
          if(localBoundary->isEqual(modelBoundary)){
            localBoundary = modelBoundary;
            break;
          }
        }
      }
    }

  }

}


/******************************************************************************/
void Primitive::compute(std::vector<RealType> X, std::vector<RealType>& vals) const
/******************************************************************************/
{
  if(!(m_coordinateSystem.is_null()))  X = m_coordinateSystem->localFromGlobal(X);
  for(int i=0; i<c_numLevelsets; ++i) 
    m_boundaries[i]->compute(&X[0],vals[i]);
}

/******************************************************************************/
RealType Primitive::compute(std::vector<RealType> X, int localLSIndex) const
/******************************************************************************/
{
  RealType retVal=0.0;

  if(!(m_coordinateSystem.is_null())) X = m_coordinateSystem->localFromGlobal(X);
  m_boundaries[localLSIndex]->compute(&X[0],retVal);

  return retVal;
}

/******************************************************************************/
void Primitive::compute(
  const FContainer<RealType>& coordCon, 
  FContainer<RealType> topoVals, 
  int localLSIndex,
  int globalLSIndex) const
/******************************************************************************/
{
  std::vector<RealType> X(c_numDim);
  std::vector<RealType> vals(c_numLevelsets);
  uint nNodes = coordCon.dimension(0);
  for(uint iNode=0; iNode<nNodes; iNode++){
    for(int i=0; i<c_numDim; ++i) X[i] = coordCon(iNode,i);
    topoVals(iNode,globalLSIndex) = compute(X,localLSIndex);
  }
}

/** public ********************************************************************/
void Primitive::computeDeriv(
  const FContainer<RealType>& coordCon, 
  Cogent::LocalMatrix<RealType>& dTdG,
  int localLSIndex) const
/******************************************************************************/
{
  uint nNodes = coordCon.dimension(0);
  std::vector<DFadType> X(c_numDim);
  for(uint iNode=0; iNode<nNodes; iNode++){
    for(int i=0; i<c_numDim; ++i) X[i] = coordCon(iNode,i);
    if(!(m_coordinateSystem.is_null()))  X = m_coordinateSystem->localFromGlobal(X);
    m_boundaries[localLSIndex]->computeDeriv(&X[0],dTdG(iNode));
  }
}

} /* namespace Geometry */
} /* namespace Cogent */
