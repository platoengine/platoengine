/*
 * Cogent_ParameterizedModel.cpp
 *
 *  Created on: Dec 6, 2016
 *      Author: jrobbin
 */

#include <cassert>

#include "Cogent_ParameterizedModel.hpp"
#include "Cogent_Utilities.hpp"
#include "Cogent_CoordinateSystem.hpp"
#include "geometry/Cogent_PrimitiveFactory.hpp"


namespace Cogent {

/******************************************************************************/
ParameterizedModel::ParameterizedModel(const Teuchos::ParameterList& geomSpec) : 
Model(geomSpec) {
/******************************************************************************/

  int nSubs = 0;
  if( geomSpec.isType<int>("Number of Subdomains")){
    nSubs = geomSpec.get<int>("Number of Subdomains");
  } else {
    TEUCHOS_TEST_FOR_EXCEPTION(true, std::runtime_error, 
      std::endl << "Cogent_ParameterizedModel: 'Number of Subdomains' must be provided." << std::endl);
  }
  m_subDomains.resize(nSubs);

  Teuchos::Array<std::string> shapeParameters = geomSpec.get<Teuchos::Array<std::string>>("Shape Parameters");
  m_parameterNames = shapeParameters.toVector();

  // create model coordinate system if given:
  Teuchos::RCP<Cogent::CoordinateSystem> modelCoordSystem;
  if( geomSpec.isSublist("Local Coordinate System") ){
    const Teuchos::ParameterList& coordSysSpec = geomSpec.sublist("Local Coordinate System");
    modelCoordSystem = Teuchos::rcp(new Cogent::CoordinateSystem(coordSysSpec, m_parameterNames));
  } else {
    modelCoordSystem = Teuchos::null;
  }

  Cogent::Geometry::PrimitiveFactory pFactory;
  for(int iSub=0; iSub<nSubs; ++iSub){
    const Teuchos::ParameterList& subdomainSpec = geomSpec.sublist(Cogent::strint("Subdomain",iSub));
    // create primitive coordinate system if given:
    Teuchos::RCP<Cogent::CoordinateSystem> coordinateSystem;
    if( subdomainSpec.isSublist("Local Coordinate System") ){
      const Teuchos::ParameterList& coordSysSpec = subdomainSpec.sublist("Local Coordinate System");
      coordinateSystem = Teuchos::rcp(new Cogent::CoordinateSystem(coordSysSpec, 
                                                                   m_parameterNames, 
                                                                   modelCoordSystem));
    } else {
      coordinateSystem = modelCoordSystem;
    }
    const Teuchos::ParameterList& primitiveSpec = subdomainSpec.sublist("Primitive");
    m_subDomains[iSub].m_primitive = pFactory.create(primitiveSpec, 
                                                     m_parameterNames, 
                                                     coordinateSystem);

    std::string opStr = subdomainSpec.get<std::string>("Operation");
    if(opStr == "Add")
      m_subDomains[iSub].m_operation = Operation::Add;
    if(opStr == "Subtract")
      m_subDomains[iSub].m_operation = Operation::Subtract;
  }

  if( geomSpec.get<std::string>("Geometry Type") == "Boundary"){
    std::string surfaceName;
    if(geomSpec.isType<std::string>("Boundary Name"))
      surfaceName = geomSpec.get<std::string>("Boundary Name");
    else
      TEUCHOS_TEST_FOR_EXCEPTION(true, std::runtime_error, 
        std::endl << "'Boundary Name' missing" << std::endl);

    bool foundAlias = false;
    for(int iSub=0; iSub<nSubs; ++iSub){
      const auto& aliases = m_subDomains[iSub].m_primitive->getBoundaryAliases();
      for(const auto& alias : aliases ){
        if(surfaceName == alias.aliasName){
          foundAlias = true;
          m_surfacePair.first = iSub;
          m_surfacePair.second = alias.localIndex;
        }
      }
    }
    TEUCHOS_TEST_FOR_EXCEPTION(!foundAlias, std::runtime_error, 
      std::endl << "Boundary with alias '" << surfaceName << "' not found." << std::endl);
  }

  // compress out duplicate geometry
  //
  std::vector<Teuchos::RCP<Cogent::Geometry::Primitive>> primitives(nSubs);
  for(int iSub=0; iSub<nSubs; ++iSub)
    primitives[iSub] = m_subDomains[iSub].m_primitive;
  for(int iSub=1; iSub<nSubs; ++iSub){
    primitives[iSub]->compress(primitives,0,iSub);
  }

  for(auto sub : m_subDomains ) m_numLevelsets += sub.m_primitive->getNumLevelsets();

  // if parameter values are provided ...
  //
  if(geomSpec.isType<Teuchos::Array<double>>("Shape Parameter Values")){
    Teuchos::Array<double> shapeParameterValues = geomSpec.get<Teuchos::Array<double>>("Shape Parameter Values");

    std::vector<double> P = shapeParameterValues.toVector();
    
    for(auto& sub : m_subDomains){
      sub.m_primitive->update(P);
    }
  }

}


ParameterizedModel::~ParameterizedModel() {
}

/******************************************************************************/
void ParameterizedModel::setElementData(const Cogent::FContainer<RealType>& coordCon)
/******************************************************************************/
{
  m_coordCon = coordCon;
  createMaps();
}

/******************************************************************************/
void ParameterizedModel::setModelData(const Cogent::FContainer<RealType>& geomData)
/******************************************************************************/
{
  m_geomData = geomData;
  
  int nParams = m_geomData.dimension(0);
  std::vector<RealType> P(nParams);
  for(int i=0; i<nParams; ++i) P[i] = m_geomData(i);

  for(auto& sub : m_subDomains){
    sub.m_primitive->update(P);
  }
}

/******************************************************************************/
bool ParameterizedModel::isMaterial(const std::vector<Sense>& sense)
/******************************************************************************/
{
// This function checks the senses to see if it is material.
//
  bool isMat = false;

  for(auto& activeDomain : m_SubdomainMap ){
    std::vector<int>& indices = activeDomain.first;
    bool inside = true;
    for(int lsIndex : indices ){
      inside = inside && (sense[lsIndex] == Sense::Positive);
    }
    Operation op = activeDomain.second;
    if(op == Operation::Add)
      isMat = isMat || inside;
    if(op == Operation::Subtract)
      isMat = isMat && !inside;
  }
  return isMat;
}

/******************************************************************************/
bool ParameterizedModel::isEmptyElement()
/******************************************************************************/
{
// This function checks each node to see if it is material.  As soon as it
// finds a node with material it returns false.  If none are found, it returns 
// true.  Boundary and body blocks have subDomains, so this function works
// for both.

  int nNodes = m_coordCon.dimension(0);
  for(int i=0; i<nNodes; i++){
    bool isMat = false;
    for(const auto& geom : m_subDomains ){
      bool inside = geom.m_primitive->nodeIsInside(m_coordCon,i);
      if(geom.m_operation == Operation::Add)
        isMat = isMat || inside;
      if(geom.m_operation == Operation::Subtract)
        isMat = isMat && !inside;
    }
    if( isMat ) return false;
  }
  return true;
}

/******************************************************************************/
bool ParameterizedModel::isPartialElement()
/******************************************************************************/
{

  bool firstMat = false;
  for(auto geom : m_subDomains ){
    bool inside = geom.m_primitive->nodeIsInside(m_coordCon,0);
    if(geom.m_operation == Operation::Add)
      firstMat = firstMat || inside;
    if(geom.m_operation == Operation::Subtract)
      firstMat = firstMat && !inside;
  }

  int nNodes = m_coordCon.dimension(0);
  for(int i=1; i<nNodes; i++){
    bool isMat = false;
    for(auto geom : m_subDomains ){
      bool inside = geom.m_primitive->nodeIsInside(m_coordCon,i);
      if(geom.m_operation == Operation::Add)
        isMat = isMat || inside;
      if(geom.m_operation == Operation::Subtract)
        isMat = isMat && !inside;
    }
    if( isMat != firstMat ) return true;
  }
  return false;
}


/******************************************************************************/
void ParameterizedModel::mapToGeometryData(
  const FContainer<RealType>& dMdT,
        FContainer<RealType>& dMdG )
/******************************************************************************/
{
  int numNodes = m_coordCon.dimension(0);
  int numParams = m_parameterNames.size();

  LocalMatrix<RealType> dTdG(numNodes,numParams);

  int numMeas = dMdT.dimension(0);
  dMdG = Cogent::FContainer<RealType>("dMdG",numMeas,numParams);

  int nActiveLS = m_LevelsetMap.size();
  for(int iLS=0; iLS<nActiveLS; ++iLS){
    auto& map = m_LevelsetMap[iLS];
    for(auto& pair : map){
      int subIndex = pair.first;
      int lsIndex = pair.second;
      m_subDomains[subIndex].m_primitive->computeDeriv(m_coordCon,dTdG,lsIndex);
    }
    // TODO: dMdT can be 1D or 2D array
    for(int iParam=0; iParam<numParams; ++iParam){
      for(int iMeas=0; iMeas<numMeas; ++iMeas){
        for(int iNode=0; iNode<numNodes; ++iNode){
          dMdG(iMeas,iParam) += dMdT(iMeas,iNode,iLS)*dTdG(iNode,iParam);
        }
      }
    }
  }
}

/******************************************************************************/
void ParameterizedModel::createMaps()
/******************************************************************************/
{

  m_LevelsetMap.clear();
  m_SubdomainMap.clear();

  bool inside;
  std::vector<int> lsIndices;
  int numSubdomains = m_subDomains.size();
  for(int iSubdomain=0; iSubdomain<numSubdomains; ++iSubdomain){
    const auto& subdomain = m_subDomains[iSubdomain];
    const auto& primitive = *(subdomain.m_primitive);
    primitive.intersected(m_coordCon, inside, lsIndices);
    if(inside){
      int numLocalIndices = lsIndices.size();
      std::vector<int> globalIndices;
      for(int iLS=0; iLS<numLocalIndices; ++iLS){
        int iLocalLS = lsIndices[iLS];
        bool unique = true;
        int matchIndex = -1;
        int numActiveLS = m_LevelsetMap.size();
        for(int iActiveLS=0; iActiveLS<numActiveLS; ++iActiveLS){
          const auto& map = m_LevelsetMap[iActiveLS];
          const auto& pair = *(map.begin());
          const auto& activePrimitive = *(m_subDomains[pair.first].m_primitive);
          if(primitive.getBoundaries()[iLocalLS] == activePrimitive.getBoundaries()[pair.second]){
            unique = false;
            matchIndex = iActiveLS;
          }
        }
        if( unique ){
          globalIndices.push_back( m_LevelsetMap.size() );
          std::vector<std::pair<int,int>> newVec;
          newVec.push_back(std::pair<int,int>(iSubdomain,iLocalLS));
          m_LevelsetMap.push_back( newVec );
        } else {
          globalIndices.push_back( matchIndex );
          m_LevelsetMap[matchIndex].push_back( std::pair<int,int>(iSubdomain,iLocalLS) );
        }
      }
      m_SubdomainMap.push_back( std::pair<std::vector<int>,Operation>(globalIndices,subdomain.m_operation) );
    }
  }

  // set m_surfaceIndex
  m_surfaceIsPresent = false;
  int numLists = m_LevelsetMap.size();
  for( int iList=0; iList<numLists; ++iList){
    for( const auto& surf : m_LevelsetMap[iList] ){
      if( surf == m_surfacePair ){
        m_surfaceIndex = iList;
        m_surfaceIsPresent = true;
        return;
      }
    }
  }

}

/******************************************************************************/
void ParameterizedModel::getSignedDistance(
        FContainer<RealType>& topoData )
/******************************************************************************/
{

  
  // size topoData
  uint nNodes = m_coordCon.dimension(0);
  uint nActive = m_LevelsetMap.size();
  topoData = Cogent::FContainer<RealType>("topoData",nNodes, nActive);

  // for each active subdomain
  //
  int offset = 0;
  for( const auto& levelsets : m_LevelsetMap )
    for( const auto& levelset : levelsets ){
      const auto& subdomain = m_subDomains[levelset.first];

      // TODO (fix this!) execution gets in here more than nActive times.  It should only 
      // come through here once per list in the m_LevelsetMap, not for each item in each list
      subdomain.m_primitive->compute(m_coordCon, topoData, levelset.second, offset);
      offset++;
    }
}


} /* namespace Cogent */
