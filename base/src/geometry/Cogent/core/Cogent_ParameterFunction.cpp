/*
 * Cogent_ParameterFunction.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: jrobbin
 */

#include "Cogent_ParameterFunction.hpp"

namespace Cogent {

/******************************************************************************/
ParameterFunction::
ParameterFunction(
  std::vector<std::string> a_xVarNames,
  std::string a_strFunc, RealType epsilon) :
  m_xVarNames(a_xVarNames), c_epsilon(epsilon), 
  c_numVars(a_xVarNames.size()),
  m_currentDeriv_RealType(c_numVars,0.0)
/******************************************************************************/
{
  // add independent variables
  for(std::string varName : a_xVarNames)
    m_function.addVar("double",varName);

  // add and initialize result variable
  m_function.addVar("double","result");
  m_function.varValueFill(c_numVars, 0.0);

  // create function expression
  m_strFunc = "result="+a_strFunc+";";
  m_function.addBody(m_strFunc);

  // create DFadTypes
  m_currentValue_DFadType = DFadType(c_numVars, 0, 0.0);

}

ParameterFunction::~ParameterFunction() {
    // TODO Auto-generated destructor stub
}

/******************************************************************************/
RealType ParameterFunction::
getValue(std::vector<RealType> P)
/******************************************************************************/
{
    // set values of independent variables
    for(uint i=0; i<P.size(); i++)
        m_function.varValueFill(i, P[i]);

    int success = m_function.execute();
    if(!success)
      std::cout << m_function.getErrors() << std::endl;

    return m_function.getValueOfVar("result");
}

/******************************************************************************/
void ParameterFunction::update(std::vector<RealType> P)
/******************************************************************************/
{
  m_currentValue_RealType = getValue(P);

  m_currentValue_DFadType.val() = m_currentValue_RealType;
  getDeriv(P,&(m_currentDeriv_RealType[0]));
  for(int iDeriv=0; iDeriv<c_numVars; iDeriv++){
    m_currentValue_DFadType.fastAccessDx(iDeriv) = m_currentDeriv_RealType[iDeriv];
  }
}


/******************************************************************************/
void ParameterFunction::
getDeriv(std::vector<RealType> P, RealType* dvdp)
/******************************************************************************/
{
  // set values of independent variables
  for(uint i=0; i<P.size(); i++)
    m_function.varValueFill(i, P[i]);

  int success = m_function.execute();
  if(!success)
    std::cout << m_function.getErrors() << std::endl;

  RealType f0 = m_function.getValueOfVar("result");
  for(uint i=0; i<P.size(); i++)
    dvdp[i] = f0;

  for(uint i=0; i<P.size(); i++){
    m_function.varValueFill(i, P[i]+c_epsilon);
    int success = m_function.execute();
    if(!success) std::cout << m_function.getErrors() << std::endl;
    RealType f0_plus_eps = m_function.getValueOfVar("result");

    m_function.varValueFill(i, P[i]-c_epsilon);
    success = m_function.execute();
    if(!success) std::cout << m_function.getErrors() << std::endl;
    RealType f0_minus_eps = m_function.getValueOfVar("result");

    dvdp[i] = (f0_plus_eps - f0_minus_eps)/(2.0*c_epsilon);

    m_function.varValueFill(i, P[i]);
  }
}

/******************************************************************************/
std::ostream& operator<<(std::ostream& out, const Cogent::ParameterFunction& pf)
/******************************************************************************/
{
  out << "ParameterFunction: "<< std::endl;
  out << "|  expression: " << pf.m_strFunc << std::endl;
  for( std::string varName : pf.m_xVarNames )
    out << "|    variable: " << varName << std::endl;
  return out;
}

template <>
RealType 
ParameterFunction::getValue<RealType>(){
  return m_currentValue_RealType;
}

template <>
DFadType 
ParameterFunction::getValue<DFadType>(){
  return m_currentValue_DFadType;
}


} /* namespace Cogent */
