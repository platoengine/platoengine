/*
 * Cogent_NonParameterizedModel.cpp
 *
 *  Created on: Dec 6, 2016
 *      Author: jrobbin
 */

#include "Cogent_NonParameterizedModel.hpp"
#include "Cogent_Utilities.hpp"

namespace Cogent {

//******************************************************************************//
NonParameterizedModel::NonParameterizedModel(const Teuchos::ParameterList& geomSpec) :
Model(geomSpec) {
//******************************************************************************//

  // parse level set field names
  if(geomSpec.isType<Teuchos::Array<std::string> >("Level Set Names"))
    m_fieldNames = geomSpec.get<Teuchos::Array<std::string> >("Level Set Names");
  else
    TEUCHOS_TEST_FOR_EXCEPTION(true, std::runtime_error, 
      std::endl << "Cogent_Model: Array(string) of 'Level Set Names' must be provided." << std::endl);

  if(geomSpec.isType<double>("Interface Value"))
    m_interfaceValue = geomSpec.get<double>("Interface Value");
  else
    m_interfaceValue = 0.0;
 
  m_surfaceIsPresent = false;
  if(geomSpec.get<std::string>("Geometry Type") == "Boundary"){
    if(geomSpec.isType<int>("Surface Level Set Index")){
      m_surfaceIndex = geomSpec.get<int>("Surface Level Set Index");
      m_surfaceIsPresent = true;
    } else
      TEUCHOS_TEST_FOR_EXCEPTION(true, std::runtime_error, 
        std::endl << "'Surface Level Set Index' missing" << std::endl);
  }

  // parse sub domains
  int nsubs = geomSpec.get<int>("Number of Subdomains");
  for(int isub=0; isub<nsubs; isub++){
    const Teuchos::ParameterList& sparams = geomSpec.sublist(Cogent::strint("Subdomain",isub));
    SubDomain newSub;
    std::string opStr = sparams.get<std::string>("Operation");
    if(opStr == "Add")
      newSub.operation = Operation::Add;
    if(opStr == "Subtract")
      newSub.operation = Operation::Subtract;
    Teuchos::Array<int> indices = sparams.get<Teuchos::Array<int> >("Level Set Indices");
    newSub.indices.resize(indices.size());
    for(int i=0; i<indices.size(); i++)
      newSub.indices[i] = indices[i];
    m_subDomains.push_back(newSub);
  }
}

NonParameterizedModel::~NonParameterizedModel() {
}


//******************************************************************************//
bool NonParameterizedModel::isPartialElement()
//******************************************************************************//
{
// This function checks that all nodes 1 to N have the same material state (either
// material or no-material) as node 0.
//
// output: true if partially filled
//         false if empty or full

  Positive<RealType> positive;

  int nNodes = m_geomData.dimension(0);

  // check first node for material
  bool firstMat = false;
  int nSubDomains = m_subDomains.size();
  for(int isub=0; isub<nSubDomains; isub++){
    SubDomain& sub = m_subDomains[isub];
    int nls = sub.indices.size();
    bool inside = true;
    for(int ils=0; ils<nls; ils++){
      int j = sub.indices[ils];
      inside = inside && (positive.compare(m_geomData(0,j),m_interfaceValue));
    }
    if(sub.operation == Operation::Add)
      firstMat = (firstMat || inside);
    if(sub.operation == Operation::Subtract)
      firstMat = (firstMat && !inside);
  }

  for(int i=1; i<nNodes; i++){
    bool isMat = false;
    for(int isub=0; isub<nSubDomains; isub++){
      SubDomain& sub = m_subDomains[isub];
      int nls = sub.indices.size();
      bool inside = true;
      for(int ils=0; ils<nls; ils++){
        int j = sub.indices[ils];
        inside = inside && (positive.compare(m_geomData(i,j),m_interfaceValue));
      }
      if(sub.operation == Operation::Add)
        isMat = isMat || inside;
      if(sub.operation == Operation::Subtract)
        isMat = isMat && !inside;
    }
    if( isMat != firstMat ) return true;
  }
  return false;
}


//******************************************************************************//
bool NonParameterizedModel::isEmptyElement()
//******************************************************************************//
{
// This function checks each node to see if it is material.  As soon as it
// finds a node with material it returns false.  If none are found, it returns 
// true.  Boundary and body blocks have subDomains, so this function works
// for both.

  Positive<RealType> positive;

  int nNodes = m_geomData.dimension(0);

  int nSubDomains = m_subDomains.size();

  for(int i=0; i<nNodes; i++){
    bool isMat = false;
    for(int isub=0; isub<nSubDomains; isub++){
      SubDomain& sub = m_subDomains[isub];
      int nls = sub.indices.size();
      bool inside = true;
      for(int ils=0; ils<nls; ils++){
        int j = sub.indices[ils];
        inside = inside && (positive.compare(m_geomData(i,j),m_interfaceValue));
      }
      if(sub.operation == Operation::Add)
        isMat = isMat || inside;
      if(sub.operation == Operation::Subtract)
        isMat = isMat && !inside;
    }
    if( isMat ) return false;
  }
  return true;
}

//******************************************************************************//
void NonParameterizedModel::
getSignedDistance(
        FContainer<RealType>& topoData)
//******************************************************************************//
{
  topoData = m_geomData;
}
 
//******************************************************************************//
void NonParameterizedModel::
mapToGeometryData(
  const FContainer<RealType>& dMdT,
        FContainer<RealType>& dMdG)
//******************************************************************************//
{
  dMdG = dMdT;
}
 

//******************************************************************************//
bool NonParameterizedModel::isMaterial(const std::vector<Sense>& sense)
//******************************************************************************//
{
// This function checks the senses to see if it is material.
//
  bool isMat = false;
  int nSubDomains = m_subDomains.size();
  for(int isub=0; isub<nSubDomains; isub++){
    const SubDomain& sub = m_subDomains[isub];
    int nls = sub.indices.size();
    bool inside = true;
    for(int ils=0; ils<nls; ils++)
      inside = inside && (sense[sub.indices[ils]] == Sense::Positive);
    if(sub.operation == Operation::Add)
      isMat = isMat || inside;
    if(sub.operation == Operation::Subtract)
      isMat = isMat && !inside;
  }
  return isMat;
}


} /* namespace Cogent */
