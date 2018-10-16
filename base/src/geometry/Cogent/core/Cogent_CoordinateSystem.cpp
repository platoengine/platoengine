/*
 * Cogent_CoordinateSystem.cpp
 *
 *  Created on: July 11, 2017
 *      Author: jrobbin
 */

#include "Cogent_CoordinateSystem.hpp"

namespace Cogent {

CoordinateSystem::
CoordinateSystem(
  const Teuchos::ParameterList& params,
  const std::vector<std::string>& parameterNames) :
  m_modelCoordinateSystem( nullptr ), c_numDim(3)
{

  parse(parameterNames, params);

}

CoordinateSystem::
CoordinateSystem(
  const Teuchos::ParameterList& params,
  const std::vector<std::string>& parameterNames, 
  const Teuchos::RCP<CoordinateSystem> coordSys) :
  m_modelCoordinateSystem( coordSys ), c_numDim(3)
{

  parse(parameterNames, params);

}


void
CoordinateSystem::
parse(const std::vector<std::string>& parameterNames, 
      const Teuchos::ParameterList& params)
{
  // TODO: add error catching

  // parse origin
  std::string strXpos = params.sublist("Origin").get<std::string>("X");
  m_P.push_back(Teuchos::rcp(new Cogent::ParameterFunction(parameterNames, strXpos)));
  std::string strYpos = params.sublist("Origin").get<std::string>("Y");
  m_P.push_back(Teuchos::rcp(new Cogent::ParameterFunction(parameterNames, strYpos)));
  std::string strZpos = params.sublist("Origin").get<std::string>("Z");
  m_P.push_back(Teuchos::rcp(new Cogent::ParameterFunction(parameterNames, strZpos)));

  // parse basis
  std::vector<std::string> vectorNames = {"X Axis", "Y Axis", "Z Axis"};
  m_Basis.resize(c_numDim);
  for(int j=0; j<c_numDim; j++){
    //m_Basis[j].resize(c_numDim);
    Teuchos::Array<std::string> 
      strXvec = params.sublist("Basis").get<Teuchos::Array<std::string>>(vectorNames[j]);
    for(int i=0; i<c_numDim; i++){
//      m_Basis[i][j] = Teuchos::rcp(new Cogent::ParameterFunction(parameterNames, strXvec[i]));
      m_Basis[i].push_back(Teuchos::rcp(new Cogent::ParameterFunction(parameterNames, strXvec[i])));
    }
  }
}

void 
CoordinateSystem::
update(const std::vector<RealType>& P)
{
  if( !(m_modelCoordinateSystem.is_null()) )
    m_modelCoordinateSystem->update(P);

  for( auto& p : m_P ) p->update(P);
  for( auto& vec : m_Basis )
    for( auto& b : vec ) b->update(P);
}

bool 
CoordinateSystem::
operator==(const CoordinateSystem& rhs)
{
  return 
    ( m_stringP == rhs.m_stringP ) &&
    ( m_stringBasis == rhs.m_stringBasis ) &&
    ( m_modelCoordinateSystem == rhs.m_modelCoordinateSystem );
}

} /* namespace Cogent */
